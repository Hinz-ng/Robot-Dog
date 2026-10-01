#include <Arduino.h>
#include <SimpleFOC.h>
#include "fleet_config.h"   // FLEET: pole pairs, encoder, dq convention, driver config
#include "joint_cal.h"      // PER-UNIT: this assembly only, picked by -D JOINT_ID

// ============================================================================
// ACTUATOR BASELINE + FOC CURRENT MODE + MT6816 4-WIRE SPI (bit-banged)
// Board: B-G431B-ESC1 clone (EG2124A). SimpleFOC 2.3.1, platform ststm32@17.6.0.
// THIS IS THE SPI MIGRATION BUILD -- flash it on the SPARE board/motor only.
// The ABZ build (TIM4) is preserved on the original assembly for fault debug.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS
//   ABZ lost ~60 counts (1.5% of a revolution) under 8-11 A, which destroyed
//   commutation progressively and required a fresh 'f' to recover. An incremental
//   counter has no mechanism to detect or correct that -- it "drifts confidently".
//   The design point is 30 A x 12 joints, 3x harsher than what broke it.
//   SPI returns the ABSOLUTE 14-bit angle every read: a corrupted sample costs
//   ONE cycle and then self-corrects, and the frame carries a parity bit so the
//   corruption is DETECTABLE rather than silent.
//
// ---------------------------------------------------------------------------
// MT6816 FACTS (datasheet Rev 2.1 2022.12, section 8 + 8.6) -- all verified:
//   * ABZ / UVW / SPI MULTIPLEX THE SAME CHIP PINS. Simultaneous use is
//     IMPOSSIBLE. HVPP selects: HVPP=GND -> ABZ/UVW, HVPP=VDD -> SPI.
//        chip pin 5  A/U (ABZ)  ==  MOSI (4-wire SPI)
//        chip pin 6  B/V (ABZ)  ==  MISO (4-wire SPI)
//        chip pin 7  Z/W (ABZ)  ==  SCK  (4-wire SPI)
//        chip pin 1  CSN        (SPI only; has an INTERNAL PULL-UP)
//        chip pin 2  HVPP       (mode select; internal 150k pulldown)
//     => the existing A/B/Z wiring maps 1:1 onto MOSI/MISO/SCK. Only CSN and
//        HVPP are new conductors.
//   * SPI mode 3 (CPOL=1, CPHA=1). SCK idles HIGH. Transfer starts on the CSN
//     falling edge, ends on the rising edge. Data changes on the SCK falling
//     edge and is sampled on the rising edge.
//   * 16-bit frame: bit0 = R/W (1=read), bits1-7 = A6..A0, bits8-15 = data.
//     So the first byte out is (0x80 | addr); the data comes back in the LOW byte.
//   * Angle registers:
//        0x03 = Angle<13:6>
//        0x04 = Angle<5:0> | No_Mag_Warning(bit1) | PC(bit0)
//        0x05 = bit3 Over_Speed
//     PC is EVEN parity over 0x03[7:0] + 0x04[7:1], so the 16-bit word
//     (reg03<<8 | reg04) always has EVEN parity. That is the integrity check.
//   * Timing: TSCK min 64 ns (15.6 MHz max), TSCKL/TSCKH min 30 ns, TL min
//     100 ns (CSN fall -> first SCK fall), TDV max 15 ns.
//   * TPwrUp 16 ms after VDD. Angle propagation delay 1 us typ / 3 us max.
//   * 3-wire SPI exists but SPI_Mode is an OTP register (factory default =
//     4-wire) and changing it needs 7.0-7.2 V on HVPP. Not attempted.
//
// ---------------------------------------------------------------------------
// WIRING (XJX-135 JP2 header -> board). ALL FOUR SPI PINS MUST BE ON GPIOB:
//   the fast path writes GPIOB->BSRR / reads GPIOB->IDR directly.
//     CSN  -> PB5     (new wire)
//     MOSI -> PB6     (was ABZ 'A'  -- same chip pin, no rewiring)
//     MISO -> PB7     (was ABZ 'B'  -- same chip pin, no rewiring)
//     SCK  -> PB8     (was ABZ 'Z'  -- same chip pin; see BOOT0 note)
//     HVPP -> VDD/3V3 (new wire -- SELECTS SPI MODE. Without this you get ABZ.)
//     VDD  -> 3V3,  GND -> GND
//
//   *** PB8 IS BOOT0. *** Fit a 10k pulldown from PB8 to GND. At MCU reset PB8
//   is high-Z and the MT6816's SCK pin is an input, so nothing drives it -- a
//   floating BOOT0 can boot the system bootloader instead of this firmware.
//   SCK was chosen for PB8 deliberately: CSN (internal pull-UP) and MISO (an
//   output) would both risk holding BOOT0 high at reset. Do not swap them.
//
//   The telemetry UART stays on PB4/PB3, UNTOUCHED. Bit-banging needs no
//   peripheral pin map, so nothing has to move and the serial monitor survives.
//
// ---------------------------------------------------------------------------
// WHAT THIS BUYS BEYOND THE BUG FIX
//   * ZEA becomes a PERSISTENT constant (absolute angle within one mech rev).
//     Measure it once, put it in ZEA_STORED, and 'f' stops twitching the rotor
//     on every power-up. For a 12-DOF robot that is close to a requirement.
//   * Alignment noise (measured 9.2 counts = 5.68 deg elec, belt-on) leaves the
//     error budget once ZEA is stored instead of re-drawn per session.
//   * No_Mag_Warning detects the failure mode in which a weak field makes the
//     angle engine emit garbage -- previously an undetectable blind spot.
//
// ---------------------------------------------------------------------------
// COMMANDS: g=go x/s=stop +/-=target | o=openloop t=torque(V) c=torque(I)
//           v=velocity | f=initFOC  F=force fresh alignment | ?=help
//           e=encoder self-test (SPI health, no motor current)
//   logger: l=fast capture  L=slow capture  k=kick-step  j=zero-step
//           d=dump CSV      a=stats (mean of last capture)
//   autocalib: Y=menu/status  1..6=phases  7=report  0=reset  V=verify stored ZEA
//   manual:    N=M2 current-sense ladder (needs a meter)  B/b=M4 breakaway +/-
//           q=toggle telemetry interval 300 <-> 3000 ms
// Boots DISABLED. 20 s auto-stop. 150 rad/s overspeed cutoff. Torque modes arm at 0.
//
// ---------------------------------------------------------------------------
// CHANGELOG vs the ABZ build (open_test.cpp, md5 71a41c24...). Every change:
//   1. TIM4Encoder REPLACED by MT6816SPI (bit-banged 4-wire, mode 3). TIM4 is no
//      longer used at all. Resolution 4096 -> 16384 counts/rev.
//   2. Parity checked on EVERY read. Failed reads reuse the last good angle and
//      increment spi_err; No_Mag_Warning and Over_Speed are surfaced.
//   3. Optional jump-plausibility reject (SPI_JUMP_GUARD) -- DEFAULT OFF so
//      bring-up debugs one thing at a time. Turn on after basic operation.
//   4. New 'e' command: encoder self-test -- N reads, reports parity error rate,
//      per-read time, angle span. Runs with the motor disabled, zero current.
//   5. ZEA_STORED / DIR_STORED: if set, 'f' skips alignment entirely. 'F' forces
//      a fresh alignment regardless. Boot banner says which path was used.
//   6. sensor_direction is NO LONGER hardcoded to CCW -- the SPI angle convention
//      is not the TIM4 count convention, so it MUST be re-derived on this board.
//      DIR_STORED = 0 means "let initFOC detect it". Record the result, then pin it.
//   7. Telemetry: cnt is now the 14-bit SPI raw; added nmg / ovs / perr / spi_us.
//   8. Log field cnt now holds the 14-bit raw angle (still uint16, wraps at
//      16383 = one mechanical revolution -- UNWRAP before differentiating).
//   9. Header comment "cannot lose counts" DELETED -- falsified by bench evidence.
// UNCHANGED: the safety path, sense-mismatch guard, overspeed / auto-stop,
//   mode transitions, every gain, the Vbus block, the logger, all print formats
//   except the additions in item 7.
// ============================================================================

// ---------------------------------------------------------------------------
// BUILD GUARD -- WRONG-HARDWARE BINARY
// ---------------------------------------------------------------------------
// platformio.ini's [env:A1] defines ENCODER_ABZ because A1 is the ORIGINAL
// quadrature assembly. This source is the MT6816 4-wire SPI build. Without this
// guard the pairing produced a binary that flashes, boots, prints an entirely
// plausible JOINT/CFG banner, and then reads garbage angles -- the exact class
// of silent wrong-hardware failure the failure catalogue exists to prevent. A
// comment was not enough; make it fail at BUILD time instead of on the bench.
#ifdef ENCODER_ABZ
  #error "This source is the MT6816 4-wire SPI build; [env:A1] is ABZ/TIM4 hardware. \
It would boot, print a valid-looking banner and read garbage. Restore the ABZ \
sources on a branch and build there -- do not build this environment."
#endif

HardwareSerial SerialUART(PB4, PB3);

#include "mt6816.h"   // MT6816 SPI encoder -- extracted verbatim, see that file

// ---------------------------------------------------------------------------


// ---- safety / tuning constants ----
// THIS SKETCH'S bench envelope. Not fleet (fleet_config.h) and not per-unit
// (joint_cal.h) -- a test harness owns these and the robot will not inherit them.
//
// Uq rails -> real current = VOLT_LIMIT / CAL.R_eff. At 2.0 V into 0.221 ohm that
// is ~9.0 A / 18 W for the ~1 s it takes to react: thermally trivial. The
// debounced sense guard is the backstop. (The old comment said R_eff = 0.218,
// which is A1's number, not this joint's -- R_eff is per-unit, see joint_cal.h.)
const float VOLT_LIMIT      = 2.0f;
// The SVPWM modulation reference handed to driver.voltage_limit -- see the long
// note at the assignment in setup(). It sets the HARD phase-voltage ceiling at
// rail/sqrt(3) = 3.46 V, which is 28% of a 12.46 V bus.
//
// *** THIS IS A SPEED CEILING YOU WILL HIT. *** Required Uq at 270 rad/s is
// Ke*w + R*Iq = 4.79 V unloaded and ~7 V at 10 A, against 3.46 V available.
// The rig tops out near 190 rad/s unloaded and well below that under load.
// Nothing measured so far was clipped -- the highest Uq ever commanded is 2.60
// (AC_W_VLIMIT) and the highest reached is 2.00 -- so every constant in the
// J01 row stands. But this must be raised before any high-speed work.
// See README section 8.3 for the promotion condition and what a change costs.
const float DRIVER_VOLT_LIMIT = 6.0f;
const float VEL_MAX         = 20.0f;
const float OVERSPEED_RADS  = 150.0f;   // torque mode has NO built-in speed limit
const float VEL_STEP        = 1.0f;
const float TORQUE_STEP     = 0.01f;
// TORQUE(V) target ceiling. Raised to 2.6 for the angle-lag sweep (130 rad/s
// needs Uq = 2.56 V); 3.5 was rejected because Uq = 3.5 settles at 183 rad/s,
// past the 150 rad/s overspeed guard. current_limit does not bind in voltage
// mode (section 12).
//
// READ THIS BEFORE USING IT: TORQUE_MAX only sets how far '+' can wind `target`.
// What is actually DELIVERED is clamped by motor.voltage_limit = VOLT_LIMIT in
// BLDCMotor::move() -- voltage.q = constrain(target, -voltage_limit, +voltage_limit).
// With VOLT_LIMIT = 2.0 every target above 2.0 V delivers exactly 2.0 V, so the
// top 0.6 V of this range is currently UNREACHABLE. That is deliberate today:
// the angle-lag sweep this headroom existed for is CLOSED (see fleet_config.h,
// T_DELAY_PER_LOOP), and AUTOCALIB phase 5 raises voltage_limit itself for the
// one sweep that still needs 2.6 and restores it afterwards. If a future test
// needs > 2.0 V delivered, raise VOLT_LIMIT -- raising this alone does nothing.
const float TORQUE_MAX      = 2.6f;
const unsigned long AUTO_STOP_MS = 20000;
const unsigned long OVERSPEED_GRACE_MS = 300;   // ignore overspeed right after arming

const float CURR_STEP   = 0.1f;
// TORQUE(I) TARGET CLAMP. Raised 2.0 -> 3.0 on 2026-09-17 for the swing ladder
// (section 22.3): swing(I) = [slack + lost motion] + 2*F(I)/k needs three well
// separated currents to separate the intercept from the slope, and 1/2/3 A is
// the widest span the plant allows.
//
// *** THIS IS THE LIMIT THAT ACTUALLY BINDS IN TORQUE(I), NOT CURR_LIMIT. ***
// Verified in the library, not assumed: BLDCMotor::move() case
// MotionControlType::torque with foc_current is a bare `current_sp = target;`
// with NO constrain against current_limit. So CURR_LIMIT/motor.current_limit is
// inert here and binds only in VELOCITY mode, via PID_velocity.limit.
// CURR_LIMIT IS DELIBERATELY LEFT AT 2.0 -- raising it would widen the velocity
// envelope, which no test asked for.
//
// WHAT 3.0 A COSTS, checked before raising it:
//   thermal   P_cu = 1.5*R*I^2 = 1.5*0.2168*9 = 2.9 W stalled, no airflow. The
//             ladder dwells ~10 s at 3 A, so ~30 J. Section 16's concern starts
//             at 4.5 A / 6.6 W sustained. Fine for a 15-minute test, NOT for a
//             standing hold -- do not leave a 3 A target parked.
//   headroom  Uq needed at stall = 3.0*0.2168 + U0 = 0.67 V against
//             VOLT_LIMIT 2.0 and Uq_ceil 3.46. Not clipped.
//   belt      3 A = 0.081 N.m at the pinion = 21 N of belt force with 3.1 teeth
//             in mesh and no idlers. THIS IS THE REAL RISK -- watch for skip.
//             acSwingLadder() detects it from the return leg (ENC_CNT_PER_TOOTH).
//   guards    AC_IMAX_ABORT is 6.0 A (reported); untouched and still 2x above.
const float CURR_MAX    = 3.0f;
const float CURR_LIMIT  = 2.0f;   // VELOCITY mode only -- see above. Unchanged.

// DEAD ZONE, PWM FREQUENCY: moved to fleet_config.h. Both are properties of the
// EG2124A/B-G431B-ESC1 board family, identical on all twelve joints, and both are
// echoed in the CFG banner because a measurement is only comparable to others
// taken under the same values.

const float CURQ_P = 0.1f,  CURQ_I = 335.0f;
const float CURD_P = 0.1f,  CURD_I = 335.0f;
// MEASURED: with CUR_TF=0.005 (32 Hz corner) the current loop achieved only 80 Hz
// bandwidth (tau 2.0 ms) with 83% overshoot -- the filter was the dominant lag inside
// a loop measured at 412 Hz. Filter corner must sit 5-10x ABOVE loop bandwidth.
const float CUR_TF = 0.00025f;    // validated value; the filter-corner bisection is closed.

const float VEL_P  = 0.2f;   // A/(rad/s) -- velocity closes on current; sweep fresh
const float VEL_I  = 1.5f;   // 0 for the P_crit sweep; restore after
const float VEL_D  = 0.0f;
const float VEL_TF = 0.02f;

// ---------------------------------------------------------------------------
// PER-JOINT CALIBRATION -- see joint_cal.h
// ---------------------------------------------------------------------------
// These used to be literals edited by hand before each flash. They now come from
// joint_cal.h, selected at COMPILE TIME by -D JOINT_ID=n from platformio.ini, so
// flashing the wrong joint's constants requires typing the wrong ENVIRONMENT
// rather than mistyping a number -- and the boot banner prints which one it is.
//
// With an ABSOLUTE sensor, ZEA is a fixed property of THIS motor + THIS magnet
// mount, not a per-session measurement. That is the payoff: 'f' stops twitching
// the rotor, and the ~3.4 deg elec of alignment scatter leaves the error budget
// instead of being redrawn on every power-up.
//
// THE PRICE: nothing re-derives ZEA any more, so a slipped magnet or a
// wrong-joint flash is SILENT. Press 'V' after every flash -- one forced
// alignment, compared against the stored value. That check is not optional.
//
// An unfilled row (zea < 0, dir == 0) makes runInitFOC() fall back to a full
// alignment, so an uncalibrated joint degrades to the old behaviour rather than
// commutating on garbage.
const float ZEA_STORED = CAL.zea;
const int   DIR_STORED = CAL.dir;


// ---------------------------------------------------------------------------
// BUS VOLTAGE SENSING
// ---------------------------------------------------------------------------
// WHY: driver.voltage_power_supply is the DIVISOR SimpleFOC uses to convert a
// requested voltage into a PWM duty cycle. It was a hardcoded 11.4 f -- a belief,
// not a measurement. Bench pack measured 11.30 V (2026-__-__): 0.9% error, inside
// the R_eff scatter, so the master table stands. On the robot at ~230 A the pack
// sags 3-7 V and a hardcoded divisor causes:
//   (a) delivered voltage != commanded voltage,
//   (b) PID_current limits sitting ABOVE the achievable modulation ceiling, so
//       the integrator winds up against a ceiling that does not physically exist,
//   (c) every logged Uq becoming a REQUEST rather than a DELIVERY, which silently
//       breaks the Uq = R*Iq + U0 + Ke*w cross-check. Same failure family as the
//       stale motor.current in the voltage branch: a value nobody updates that
//       reads exactly like data.
//
// PIN: PA0. CONFIRMED on the bench, not inferred -- 11.30 V -> 1351 counts,
// 22.50 V -> 2694 counts. Voltage ratio 1.991, count ratio 1.994 (0.15% apart).
// PA1 / PB12 were flat; PB14 moved the wrong way (it is the board thermistor).
//
// SCALE: measured, not from a datasheet -- the divider ratio is undocumented on
// this clone. Pure proportional fit; the 33 mV offset from a 2-point line fit is
// smaller than the +-0.05 V rounding in the 22.5 V meter reading, so it is not
// resolvable and is discarded. Back-predicts both points to within 0.07%.
// THAT 2-POINT FIT GAVE 0.008358 AND WAS 1.1% LOW -- not because the fit was
// bad, but because the METER was: the DT9205A used for it has a measured ~1.11%
// DCV gain error, and 0.008358 x 1.0111 lands 0.03% from the 2026-08-18 M1
// re-measurement against a UT89X. The fit residual was never the problem.
//   full scale  = 4095 * 0.008448 = 34.60 V bus   (B-SPI-01 / J01, M1 2026-08-18)
//   resolution  = 8.45 mV / count
// Both figures are PER BOARD. B-ABZ-01 / J02 measures 0.008516 -> 34.87 V full
// scale, 8.52 mV/count: 0.80% apart, measured, not estimated.
// LEFT AT 0.008516 AFTER 2026-08-20, DELIBERATELY. A later session read the
// banner at 12.29 against a UT89X 12.27 -- 0.02 V, 0.16%, against that meter's
// own +-(0.5%+2) = +-0.064 V at 12.3 V. The disagreement is a QUARTER of the
// resolving power of the instrument being used to judge it. Chasing it would
// move R_eff, U0, Ke and L by less than their own uncertainties on evidence the
// meter cannot supply. It is folded into M2's error budget instead.
//   6S at 25.2 V = 3015 counts = 74% of range -> NO PB10 / 48V_EN change needed.
// Re-calibrate if PB10 (48V_EN) is ever driven: it switches the divider range.
//
// THE SCALE ITSELF IS PER-BOARD AND COMES FROM joint_cal.h. It used to be a
// literal here, which made it look like a fleet constant. It is not: two boards
// have now been MEASURED 0.80% apart (0.008448 / 0.008516, M1 2026-08-18), and
// R_eff, U0 and Ke all scale linearly with it -- so that 0.80% lands straight on
// every torque command. A literal here would also have silently overridden
// whatever a future row said. M1 (multimeter, two bus voltages) is mandatory per
// board, and it is the FIRST thing run on a new board, before AUTOCALIB.
//
// vbus_scale = 0 in the row disables this entire feature. Behaviour then is
// byte-identical to a hardcoded divisor, which keeps rollback a one-field edit.
const uint32_t PIN_VBUS   = PA0;
const float VBUS_SCALE    = CAL.vbus_scale;   // V per ADC count -- PER BOARD, M1
// SEED-ONLY. Arduino analogRead() returns 0 on any pin of this ADC once
// currentSense.init() has run -- confirmed 2026-__-__ by vraw=0 from an
// isolated call in the print block, while the pre-init setup read works every
// time. Not a contention-between-two-calls issue: ONE call fails.
//
// MECHANISM CORRECTED 2026-08-21 by the v1 'p' probe dump on J02. This block
// previously said the current sense owned the INJECTED group, and that regular
// and injected coexist by design (injected preempts, regular resumes). BOTH ARE
// RETRACTED -- the dump shows JSQR = 0 and JADSTART = 0 on both instances, so
// there is no injected group at all, and ADSTART = 1 on both, so the sense runs
// on the REGULAR group. HAL_ADC_Start therefore returns BUSY because the
// REGULAR group is already started. Same symptom, different owner.
//
// *** AFTER currentSense.init(), analogRead() IS FORBIDDEN ON THIS BOARD. ***
// It fails safe TODAY (returns 0) only because ADSTART is already 1. It fails
// by contending for the very sequence the current sense owns: a core or HAL
// version that stops the ADC first would let it through, rewrite SQR1/SMPR and
// destroy current sensing with no error and no symptom beyond garbage |I|/Iq.
//
// LIVE TRACKING NEEDS NO ADC CONFIGURATION -- CONFIRMED 2026-08-30, and that is
// the whole of what the detour bought. PA0 is rank 5 of the regular sequence and
// already lands in the DMA buffer every PWM period, so a live reading is a RAM
// read (see 'p', and HARDWARE.md section 3 for the map).
//
// WHAT STILL BLOCKS IT is the number, not the mechanism: that buffer sits ~60
// counts (0.51 V) below the seed because SimpleFOC's init never calibrates the
// converter (CALFACT = 0 on both instances, every boot). The seed path is the
// correct one -- validated against a UT89X at 12.24 / 12.25 / 22.73 V to better
// than 0.16%. Calibrating the ADC earlier was tried and ABANDONED: at the only
// point in boot where the ADC is idle there is no kernel clock at all
// (RCC->CCIPR ADC12SEL = 0), so ADCAL cannot run, and arming it there leaves a
// PENDING calibration that fires later inside currentSense.init(). See
// CHANGELOG section 0. Do not re-attempt without reading that entry first.
// Deferred -- the bench has no sag to track. See README 8.3.
const bool  VBUS_LIVE     = false;
const float VBUS_TF       = 0.020f;     // 20 ms. Noise here becomes motor current.
const float VBUS_MIN      = 8.0f;       // PLAUSIBILITY window only -- NOT a
const float VBUS_MAX      = 30.0f;      //   low-voltage cutoff. See note below.
const float VBUS_FALLBACK = 11.30f;     // measured bench pack, used if read fails
#include "actuator_hw.h"   // motor/driver/currentSense/encoder + Vbus state
// NOTE ON VBUS_MAX: the ADC saturates at 34.2 V, so a genuine overvoltage above
// that would read as exactly 34.2 and be REJECTED by this window -- the filter
// would then hold its last good value rather than reporting the fault. That is
// the correct behaviour for a divisor, but it means this window is not, and must
// not be mistaken for, overvoltage protection.

// ===========================================================================
// VBUS ADC PROBE v3 -- 'p'.  PURE READ. No peripheral register is written.
//
// PREMISE CORRECTION (2026-08-21, from the v1 dump on J02):
//   This board does NOT use the ADC injected group. JSQR = 0 and JADSTART = 0
//   on BOTH instances; ADSTART = 1 on both. SimpleFOC's b_g431 path runs the
//   current sense on the REGULAR group with circular DMA. The "regular and
//   injected coexist" note in the SEED-ONLY block above is RETRACTED, and so is
//   the claim that analogRead() fails on an injected-BUSY peripheral -- it
//   fails because the REGULAR group is permanently started.
//
// WHAT THIS LOOKS FOR: ADC1 SQR1 shows a 5-conversion sequence (ranks 12,3,11,5)
// and SMPR1 sets 47.5 cycles on channel 1 -- which is ADC1_IN1 = PA0 and is NOT
// in ranks 1-4. If PA0 is rank 5, live Vbus needs no ADC configuration at all.
//
// v1's adcBringUp() and adcReadCh() are DELETED, not commented out: their
// premise is dead, and adcReadCh's read-modify-write on CR was the one real
// hazard in v1. Nothing here writes a peripheral register.
// ===========================================================================
// RM0440 ADC_SMPR sample times, in TENTHS of an ADC clock cycle so the .5s stay
// exact in integer arithmetic. Index is the raw 3-bit SMP field.
static const uint16_t SMP_CYC_X10[8] = { 25, 65, 125, 245, 475, 925, 2475, 6405 };
// Successive-approximation time by ADC_CFGR.RES (12/10/8/6-bit), same units.
static const uint16_t RES_CYC_X10[4] = { 125, 105, 85, 65 };

static uint8_t adcGetSmp(ADC_TypeDef* a, uint8_t ch) {
  return (ch <= 9) ? ((a->SMPR1 >> (3u*ch)) & 0x7u)
                   : ((a->SMPR2 >> (3u*(ch-10u))) & 0x7u);
}

// Kept from v1 and still CALLED: this is the only thing that prints CFGR, SMPR
// and JSQR, and two of v2's failure branches ask for exactly those. The
// JADSTART / JSQR fields now read as evidence that the injected group is empty
// rather than as a description of how the sense works.
static void adcDump(const __FlashStringHelper* nm, ADC_TypeDef* a) {
  SerialUART.print(nm);
  SerialUART.print(F(" ADEN="));    SerialUART.print((a->CR & ADC_CR_ADEN)     ? 1:0);
  SerialUART.print(F(" ADSTART=")); SerialUART.print((a->CR & ADC_CR_ADSTART)  ? 1:0);
  SerialUART.print(F(" JADSTART="));SerialUART.print((a->CR & ADC_CR_JADSTART) ? 1:0);
  SerialUART.print(F(" DEEPPWD=")); SerialUART.print((a->CR & ADC_CR_DEEPPWD)  ? 1:0);
  SerialUART.print(F(" CFGR=0x"));  SerialUART.print(a->CFGR, HEX);
  SerialUART.print(F(" SQR1=0x"));  SerialUART.print(a->SQR1, HEX);
  SerialUART.print(F(" JSQR=0x"));  SerialUART.print(a->JSQR, HEX);
  SerialUART.print(F(" SMPR=0x"));  SerialUART.print(a->SMPR1, HEX);
  SerialUART.print('/');            SerialUART.print(a->SMPR2, HEX);
  uint8_t jl = (uint8_t)((a->JSQR & 0x3u) + 1u);
  SerialUART.print(F("  JL=")); SerialUART.print(jl); SerialUART.print(F(" JSQ="));
  for (uint8_t k = 0; k < jl; k++) {
    uint8_t sh = (uint8_t)(ADC_JSQR_JSQ1_Pos + 6u*k);
    uint8_t ch = (uint8_t)((a->JSQR >> sh) & 0x1Fu);
    SerialUART.print(ch); SerialUART.print(k+1 < jl ? ',' : ' ');
    SerialUART.print(F("(smp")); SerialUART.print(adcGetSmp(a, ch)); SerialUART.print(')');
  }
  SerialUART.println();
}

static uint8_t adcSeqCh(ADC_TypeDef* a, uint8_t rank) {          // rank 1..9
  if (rank <= 4) return (uint8_t)((a->SQR1 >> (6u + 6u*(rank-1u))) & 0x1Fu);
  return             (uint8_t)((a->SQR2 >> (6u*(rank-5u)))        & 0x1Fu);
}

// Whole-sequence conversion time in tenths of an ADC cycle, summed from the
// registers rather than written down: sum over ranks of (sample + SAR time).
// Derived, not hardcoded, because §3's hazard is precisely that this total
// changes silently when someone edits SQR1 or SMPR.
static uint32_t adcSeqCycX10(ADC_TypeDef* a) {
  const uint8_t L   = (uint8_t)((a->SQR1 & 0xFu) + 1u);
  const uint8_t res = (uint8_t)((a->CFGR & ADC_CFGR_RES_Msk) >> ADC_CFGR_RES_Pos);
  uint32_t tot = 0;
  for (uint8_t r = 1; r <= L && r <= 9; r++)
    tot += (uint32_t)SMP_CYC_X10[adcGetSmp(a, adcSeqCh(a, r))] + RES_CYC_X10[res];
  return tot;
}

static bool addrIsRam(uint32_t p) {
  return (p >= 0x20000000u && p < 0x20008000u)      // SRAM1/2
      || (p >= 0x10000000u && p < 0x10003000u);     // CCM SRAM
}

static void adcSeqDump(const __FlashStringHelper* nm, ADC_TypeDef* a) {
  uint8_t L = (uint8_t)((a->SQR1 & 0xFu) + 1u);
  SerialUART.print(nm);
  SerialUART.print(F(" ADSTART=")); SerialUART.print((a->CR & ADC_CR_ADSTART) ? 1 : 0);
  SerialUART.print(F(" SQR2=0x")); SerialUART.print(a->SQR2, HEX);
  SerialUART.print(F(" L=")); SerialUART.print(L);
  SerialUART.print(F(" seq="));
  for (uint8_t r = 1; r <= L && r <= 9; r++) {
    uint8_t ch = adcSeqCh(a, r);
    SerialUART.print(ch);
    SerialUART.print(F("(smp")); SerialUART.print(adcGetSmp(a, ch)); SerialUART.print(')');
    if (r < L) SerialUART.print(',');
  }
  SerialUART.print(F("  DR=")); SerialUART.println(a->DR);
}

// Walk every DMA channel; a channel whose CPAR points at this ADC's DR is the
// one filling the sequence buffer. CMAR is then the buffer address.
// G431 has DMA1_Channel1..6 and DMA2_Channel1..6 only -- channels 7/8 exist on
// larger G4 parts, so the #ifdefs are what keep this source portable across the
// family rather than dead branches.
static DMA_Channel_TypeDef* const PROBE_DMA[] = {
  DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4,
  DMA1_Channel5, DMA1_Channel6,
#ifdef DMA1_Channel7
  DMA1_Channel7,
#endif
#ifdef DMA1_Channel8
  DMA1_Channel8,
#endif
  DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4,
  DMA2_Channel5, DMA2_Channel6,
#ifdef DMA2_Channel7
  DMA2_Channel7,
#endif
#ifdef DMA2_Channel8
  DMA2_Channel8,
#endif
};

// Index into PROBE_DMA of the channel feeding this ADC, or -1. Split out of
// probeAdcDma so setup()'s seed-vs-DMA comparison can find the same buffer
// without duplicating the walk -- one definition of "where the buffer is".
static int adcDmaIdx(ADC_TypeDef* a) {
  const uint32_t dr = (uint32_t)&a->DR;
  for (uint8_t i = 0; i < (uint8_t)(sizeof(PROBE_DMA)/sizeof(PROBE_DMA[0])); i++)
    if (PROBE_DMA[i]->CPAR == dr) return (int)i;
  return -1;
}

// The live conversion buffer, or nullptr if there is no DMA channel or CMAR is
// not RAM. Never dereference without checking -- a wild CMAR would fault.
static volatile const uint16_t* adcDmaBuf(ADC_TypeDef* a) {
  const int i = adcDmaIdx(a);
  if (i < 0) return nullptr;
  const uint32_t cmar = PROBE_DMA[i]->CMAR;
  return addrIsRam(cmar) ? (volatile const uint16_t*)cmar : nullptr;
}

// 0-based buffer slot holding this channel, or -1. Looked up through the
// sequence registers rather than written down: HARDWARE.md section 3's hazard
// is exactly that these slot numbers move when someone edits SQR1.
static int adcSlotOfCh(ADC_TypeDef* a, uint8_t ch) {
  const uint8_t L = (uint8_t)((a->SQR1 & 0xFu) + 1u);
  for (uint8_t r = 1; r <= L && r <= 9; r++)
    if (adcSeqCh(a, r) == ch) return (int)(r - 1u);
  return -1;
}

// ---------------------------------------------------------------------------
// Vdma -- the bus voltage as the DMA path sees it. RAW: one sample, no filter,
// no plausibility window, NOT driver.voltage_power_supply. It exists to be
// compared against Vb (the calibrated seed) while a run is putting real current
// through the board, which is the ONLY thing that tests H7: a reference shift
// that scales with return current. Flat Vdma-Vb from 0.2 A to 3.0 A bounds H7
// at bench scale; growth is disqualifying and is a board-layout finding.
//
// NAN, not 0, when the buffer is unreachable -- a zero would read as a real
// measurement of a collapsed bus, and Print emits "nan" which parses as NaN
// offline. KEPT after the 2026-08-30 cleanup: CONSTANTS.md section 8.3 names
// Vdma-vs-terminal-meter as the promotion condition for live Vbus, and this
// field plus the M2 column are what make that test free. Nothing reads it.
// ---------------------------------------------------------------------------

static float vbusDmaRaw() {
  volatile const uint16_t* b = adcDmaBuf(ADC1);
  const int s = adcSlotOfCh(ADC1, 1);            // ADC1_IN1 = PA0 = VBUS_ADC
  return (b && s >= 0) ? (float)b[s] * VBUS_SCALE : (float)NAN;
}
static void printVdma() {
  SerialUART.print(F(" Vdma="));
  const float v = vbusDmaRaw();
  if (isnan(v)) SerialUART.print(F("--"));
  else          SerialUART.print(v, 3);
}

static void probeAdcDma(const __FlashStringHelper* nm, ADC_TypeDef* a, uint16_t seed_cnt) {
  const uint8_t  L  = (uint8_t)((a->SQR1 & 0xFu) + 1u);
  const int      ix = adcDmaIdx(a);
  if (ix >= 0) {
    DMA_Channel_TypeDef* d = PROBE_DMA[ix];
    const uint8_t i = (uint8_t)ix;
    SerialUART.print(nm); SerialUART.print(F(" <- DMA idx ")); SerialUART.print(i);
    SerialUART.print(F("  CCR=0x")); SerialUART.print(d->CCR, HEX);
    SerialUART.print(F(" EN=")); SerialUART.print(d->CCR & 1u);
    SerialUART.print(F(" CIRC=")); SerialUART.print((d->CCR >> 5) & 1u);
    SerialUART.print(F(" MSIZE=")); SerialUART.print((d->CCR >> 10) & 3u);
    SerialUART.print(F(" CNDTR=")); SerialUART.print(d->CNDTR);
    SerialUART.print(F(" CMAR=0x")); SerialUART.println(d->CMAR, HEX);
    if (!addrIsRam(d->CMAR)) { SerialUART.println(F("  !! CMAR not in RAM -- not reading")); return; }

    volatile const uint16_t* buf = (volatile const uint16_t*)d->CMAR;
    // Three dumps 5 ms apart. The current-sense slots MUST jitter; a frozen
    // buffer means the DMA is not running and nothing below is usable.
    for (uint8_t pass = 0; pass < 3; pass++) {
      SerialUART.print(F("  buf["));
      for (uint8_t k = 0; k < L; k++) { SerialUART.print(buf[k]); if (k+1 < L) SerialUART.print(' '); }
      SerialUART.println(']');
      delay(5);
    }
    // Where VBUS actually is, from the sequence registers. AUTHORITATIVE -- the
    // nearest-seed line below is only a heuristic, and it picked the WRONG slot
    // once the offset's sign flipped (2026-08-21: ch1 read +51 while ch5 read
    // -44, so "nearest" chose ch5). Read this line, not that one.
    if (a == ADC1) {
      const int s1 = adcSlotOfCh(ADC1, 1);
      SerialUART.print(F("  ch1 (PA0/VBUS) is slot ")); SerialUART.print(s1);
      SerialUART.print(F(" per SQR -> "));
      if (s1 >= 0) SerialUART.println(buf[s1] * VBUS_SCALE, 4);
      else         SerialUART.println(F("NOT IN SEQUENCE"));
    }
    // ADC2's sequence is one phase-current channel. Matching it against a BUS
    // seed is meaningless and prints an alarming "|err|=1122 counts = 9.55 V".
    // Suppressed rather than explained away.
    if (a != ADC1) { SerialUART.println(F("  (no bus channel on this instance)")); return; }
    // Flag the slot nearest the seed, at both possible alignments. HEURISTIC.
    int bestK = -1; int32_t bestE = 0x7FFFFFFF; bool left = false;
    for (uint8_t k = 0; k < L; k++) {
      int32_t r = (int32_t)buf[k] - (int32_t)seed_cnt;              if (r < 0) r = -r;
      int32_t s = (int32_t)(buf[k] >> 4) - (int32_t)seed_cnt;       if (s < 0) s = -s;
      if (r < bestE) { bestE = r; bestK = k; left = false; }
      if (s < bestE) { bestE = s; bestK = k; left = true;  }
    }
    SerialUART.print(F("  [heuristic] nearest seed: slot ")); SerialUART.print(bestK);
    SerialUART.print(left ? F(" (>>4, LEFT-ALIGNED)") : F(" (right-aligned)"));
    SerialUART.print(F("  |err|=")); SerialUART.print(bestE);
    SerialUART.print(F(" counts = ")); SerialUART.print(bestE * VBUS_SCALE, 4);
    SerialUART.println(F(" V"));
    SerialUART.print(F("  rank ")); SerialUART.print(bestK + 1);
    SerialUART.print(F(" is channel ")); SerialUART.println(adcSeqCh(a, (uint8_t)(bestK + 1)));
    return;
  }
  SerialUART.print(nm); SerialUART.println(F(": no DMA channel targets its DR"));
}

// ===========================================================================
// PROBE v3 REGISTER BLOCK -- hunting the 62-count DMA-vs-seed subtraction.
//
// Measured 2026-08-21, two bus voltages: buf[4] sits 62.3 +/- 3.0 counts BELOW
// the pre-init analogRead() seed, gain 1.006 (unity within meter resolution),
// thermally inert. H1 (charge sharing from rank 4) is dead -- PB14 swung 60.7
// counts at a fixed bus while PA0 moved 1.0, a coupling of 0.017 against the
// 0.474 required. What is left is a FIXED SUBTRACTION, and three mechanisms can
// produce one:
//   H6  ADC1->OFRn programmed on channel 1. The register's literal function is
//       DR = raw - OFFSET. Exact signature match. <- adcOffsetDump()
//   H5  ADC-wide offset from a bad CALFACT (calibrated with the opamps live).
//       62 LSB is ~12x the datasheet offset error, so this is a stretch.
//   H4  PA0's divider node is physically loaded after init. The seed is the ONLY
//       reading taken pre-driver.init(), and the seed block's own comment
//       ("detach digital buffer, unload divider") predicts exactly this.
//       <- gpioPinDump(): if PA0's MODER left 0b11, H4 moves to a physical probe
// All pure reads. Nothing here writes a peripheral register.
// ===========================================================================
static void adcOffsetDump(const __FlashStringHelper* nm, ADC_TypeDef* a) {
  const __IO uint32_t* ofr[4] = { &a->OFR1, &a->OFR2, &a->OFR3, &a->OFR4 };
  SerialUART.print(nm); SerialUART.print(F(" OFR"));
  bool any = false;
  for (uint8_t i = 0; i < 4; i++) {
    const uint32_t v = *ofr[i];
    SerialUART.print(' '); SerialUART.print(i + 1); SerialUART.print('=');
    SerialUART.print(v, HEX);
    if (v & ADC_OFR1_OFFSET1_EN_Msk) {
      any = true;
      SerialUART.print(F("[EN ch"));
      SerialUART.print((v & ADC_OFR1_OFFSET1_CH_Msk) >> ADC_OFR1_OFFSET1_CH_Pos);
      SerialUART.print(F(" off="));
      SerialUART.print(v & ADC_OFR1_OFFSET1_Msk);
      SerialUART.print((v & ADC_OFR1_OFFSETPOS_Msk) ? F(" POS") : F(" NEG"));
      SerialUART.print((v & ADC_OFR1_SATEN_Msk)     ? F(" SAT]") : F("]"));
    }
  }
  SerialUART.println(any ? F("   <<< AN OFFSET IS ENABLED -- H6")
                         : F("   (none enabled -- H6 dead on this instance)"));

  const uint32_t cf = a->CALFACT, c2 = a->CFGR2;
  SerialUART.print(nm);
  SerialUART.print(F(" CALFACT=0x")); SerialUART.print(cf, HEX);
  SerialUART.print(F(" S="));  SerialUART.print((cf & ADC_CALFACT_CALFACT_S_Msk) >> ADC_CALFACT_CALFACT_S_Pos);
  SerialUART.print(F(" D="));  SerialUART.print((cf & ADC_CALFACT_CALFACT_D_Msk) >> ADC_CALFACT_CALFACT_D_Pos);
  SerialUART.print(F("  DIFSEL=0x")); SerialUART.print(a->DIFSEL, HEX);
  SerialUART.print(F("  CFGR2=0x")); SerialUART.print(c2, HEX);
  // Oversampling would divide the result and read as a gain, not an offset --
  // ruled in or out here rather than assumed.
  SerialUART.print(F(" ROVSE=")); SerialUART.print((c2 & ADC_CFGR2_ROVSE_Msk) ? 1 : 0);
  SerialUART.print(F(" JOVSE=")); SerialUART.print((c2 & ADC_CFGR2_JOVSE_Msk) ? 1 : 0);
  SerialUART.print(F(" OVSR="));  SerialUART.print((c2 & ADC_CFGR2_OVSR_Msk) >> ADC_CFGR2_OVSR_Pos);
  SerialUART.print(F(" OVSS="));  SerialUART.print((c2 & ADC_CFGR2_OVSS_Msk) >> ADC_CFGR2_OVSS_Pos);
  SerialUART.print(F(" GCOMP=")); SerialUART.println((c2 & ADC_CFGR2_GCOMP_Msk) ? 1 : 0);
}

// MODER 11 = analog, which is what the pre-init pinMode(INPUT_ANALOG) sets and
// what H4 says something later undoes. PUPDR must be 00: a pull on a divider
// node IS the loading mechanism, and it would be a fixed, bus-independent shift.
static void gpioPinDump(const __FlashStringHelper* nm, GPIO_TypeDef* g, uint8_t pin) {
  const uint8_t m = (uint8_t)((g->MODER >> (2u*pin)) & 0x3u);
  const uint8_t p = (uint8_t)((g->PUPDR >> (2u*pin)) & 0x3u);
  SerialUART.print(' '); SerialUART.print(nm); SerialUART.print(F(":MODER="));
  SerialUART.print(m);
  SerialUART.print(m == 3 ? F("(analog)") : m == 0 ? F("(IN!)")
                 : m == 1 ? F("(OUT!)")   : F("(AF!)"));
  SerialUART.print(F(" PUPDR=")); SerialUART.print(p);
  SerialUART.print(p == 0 ? F("(none)") : p == 1 ? F("(PULLUP!)") : F("(PULLDN!)"));
}

// ADC KERNEL CLOCK, as a register fact rather than arithmetic. Three registers
// decide it and no two of them live in the same peripheral:
//   RCC->CCIPR.ADC12SEL  picks the asynchronous SOURCE (none / PLL"P" / SYSCLK)
//   ADC_CCR.CKMODE       picks async-with-PRESC (00) vs HCLK/1,2,4 (01/10/11)
//   ADC_CCR.PRESC        divides, and ONLY applies when CKMODE == 00
// Printing the decode matters because the whole SMP time budget hangs off it.
// The sequence length is summed from the registers by adcSeqCycX10(), so the
// microseconds printed here are a register fact too -- and whether the answer is
// ~20 us or ~1 us decides whether raising SMP on channel 1 is even arguable.
// The source frequency comes from the HAL's own RCC walk, not from a constant.
static void adcClockDump() {
  static const uint16_t PRESC_DIV[16] =                 // RM0440 ADC_CCR.PRESC
    { 1,2,4,6,8,10,12,16,32,64,128,256,0,0,0,0 };       // 12..15 reserved -> 0
  const uint32_t ccipr  = RCC->CCIPR;
  const uint32_t ccr    = ADC12_COMMON->CCR;
  const uint8_t  sel    = (uint8_t)((ccipr & RCC_CCIPR_ADC12SEL_Msk) >> RCC_CCIPR_ADC12SEL_Pos);
  const uint8_t  ckmode = (uint8_t)((ccr & ADC_CCR_CKMODE_Msk) >> ADC_CCR_CKMODE_Pos);
  const uint8_t  presc  = (uint8_t)((ccr & ADC_CCR_PRESC_Msk)  >> ADC_CCR_PRESC_Pos);
  const uint32_t fsrc   = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC12);

  SerialUART.print(F("CLK CCIPR=0x")); SerialUART.print(ccipr, HEX);
  SerialUART.print(F(" ADC12SEL=")); SerialUART.print(sel);
  SerialUART.print(sel == 0 ? F("(NONE!)") : sel == 1 ? F("(PLL'P')")
                 : sel == 2 ? F("(SYSCLK)") : F("(reserved)"));
  SerialUART.print(F(" src=")); SerialUART.print(fsrc); SerialUART.print(F(" Hz"));
  SerialUART.print(F(" | CCR=0x")); SerialUART.print(ccr, HEX);
  SerialUART.print(F(" CKMODE=")); SerialUART.print(ckmode);
  SerialUART.print(ckmode == 0 ? F("(async)") : F("(HCLK)"));
  SerialUART.print(F(" PRESC=")); SerialUART.print(presc);
  SerialUART.print(F("(/")); SerialUART.print(PRESC_DIV[presc]); SerialUART.print(')');

  // CKMODE != 0 takes the clock from HCLK and IGNORES PRESC entirely.
  uint32_t f_adc = 0;
  if (ckmode == 0) { if (PRESC_DIV[presc]) f_adc = fsrc / PRESC_DIV[presc]; }
  else             { f_adc = SystemCoreClock >> (ckmode - 1u); }
  SerialUART.print(F(" -> f_adc=")); SerialUART.print(f_adc); SerialUART.print(F(" Hz"));
  if (f_adc) {
    const uint32_t cx10 = adcSeqCycX10(ADC1);
    SerialUART.print(F("  1cyc=")); SerialUART.print(1e9f / (float)f_adc, 1);
    SerialUART.print(F(" ns  ADC1 seq=")); SerialUART.print(cx10 / 10.0f, 1);
    SerialUART.print(F(" cyc = "));
    SerialUART.print((float)cx10 * 1e5f / (float)f_adc, 2); SerialUART.print(F(" us"));
  } else {
    SerialUART.print(F("  !! cannot resolve -- reserved field or no clock"));
  }
  SerialUART.println();
}

static void vbusProbe() {
  // Nothing below writes a peripheral register, so this gate is no longer a
  // safety requirement -- it is kept to preserve the "probes run disarmed"
  // habit. motor.enabled is SimpleFOC's own int8_t and is a strict superset of
  // this harness's `running` flag, which is declared further down the file.
  if (motor.enabled) { SerialUART.println(F("p: disable the motor first")); return; }

  const uint16_t seed_cnt = (VBUS_SCALE > 0.0f)
                          ? (uint16_t)(vbus_filt / VBUS_SCALE + 0.5f) : 0;
  SerialUART.println(F("\n---- VBUS ADC PROBE v3 (pure read) ----"));
  SerialUART.print(F("seed Vb=")); SerialUART.print(vbus_filt, 4);
  SerialUART.print(F(" V  scale=")); SerialUART.print(VBUS_SCALE, 6);
  SerialUART.print(F(" -> seed_cnt=")); SerialUART.println(seed_cnt);
  adcClockDump();                      // kernel clock + sequence time, from regs
  adcDump(F("ADC1"), ADC1);            // CFGR / SMPR / JSQR -- the v1 evidence
  adcDump(F("ADC2"), ADC2);            //   lines, so the two runs stay comparable
  adcSeqDump(F("ADC1"), ADC1);
  adcSeqDump(F("ADC2"), ADC2);
  adcOffsetDump(F("ADC1"), ADC1);      // H6 / H5 -- offset registers, CALFACT
  adcOffsetDump(F("ADC2"), ADC2);
  SerialUART.print(F("GPIO"));         // H4 -- is PA0 still an analog input?
  gpioPinDump(F("PA0"),  GPIOA, 0);
  gpioPinDump(F("PB12"), GPIOB, 12);
  gpioPinDump(F("PB14"), GPIOB, 14);
  SerialUART.println();
  probeAdcDma(F("ADC1"), ADC1, seed_cnt);
  probeAdcDma(F("ADC2"), ADC2, seed_cnt);
  SerialUART.println(F("---- end probe v3 ----\n"));
}



enum Mode { MODE_OPENLOOP, MODE_TORQUE, MODE_TORQUE_CURRENT, MODE_VELOCITY };
Mode mode = MODE_OPENLOOP;
bool foc_ready = false;
bool running = false;
float target = 2.0f;

bool driver_ok=false, cs_ok=false, cs_linked=false;
unsigned long run_started=0, last_blink=0, last_print=0;
bool led_state=false;

volatile uint32_t g_loops = 0;
uint32_t lps = 0;
uint32_t pr_us = 0;
uint16_t print_ms = 300;   // 'q' toggles 300 <-> 3000 for the loop-rate test

// ---------------------------------------------------------------------------
// BURST LOGGER -- captures every loop iteration into RAM, dumps afterwards.
// 300 ms serial telemetry aliases everything above ~1.7 Hz; this does not.
//   l = fast capture  (decim 1  -> ~65 ms  @15.5kHz, for current-loop steps)
//   L = slow capture  (decim 8  -> ~520 ms,           for judder / resonance)
//   k = kick: step target BASE -> STEP and fast-capture it. TORQUE(V) or (I).
//   j = kick from ZERO -- deliberately includes the dead-zone traverse.
//   d = dump last capture as CSV
//   a = mean of last capture (one line per sweep point; no CSV needed)
// ---------------------------------------------------------------------------
#define LOG_N 1000
struct LogSample {
  uint16_t dt_us;         // us since the PREVIOUS sample. logDump() used to
                          // reconstruct t = k * dt_mean, which ASSUMES uniform
                          // sampling. The sense guard's extra ADC read (every 30
                          // loops), the LED blink and handleSerial all jitter the
                          // loop; an inertia fit against an assumed-uniform clock
                          // is biased by exactly that jitter. Also the decisive
                          // datum for the 57 kHz vs 15.5 kHz lps discrepancy.
                          // Saturates at 65.5 ms; decim 8 @15.5 kHz is ~516 us.
  int16_t  vel_x50;       // rad/s * 50 -- FILTERED by LPF_velocity (Tf=20 ms).
                          // Do NOT fit inertia from this column; use cnt.
  int16_t  iq_x1000;      // A * 1000
  int16_t  id_x1000;      // A * 1000
  int16_t  uq_x1000;      // V * 1000
  int16_t  ud_x1000;      // V * 1000 -- d-axis PI output. In current mode this IS
                          // the cross-coupling term (w_e*L*Iq) plus any angle-error
                          // contribution: the direct measurement of angle lag.
  int16_t  sp_x1000;      // current SETPOINT * 1000
  int16_t  raw_x100;      // RAW unsynchronised |I| * 100 -- sees PWM-rate ripple that
                          // the synchronously sampled dq path is blind to
  uint16_t cnt;           // raw 14-bit MT6816 angle (ground truth for velocity).
                          // uint16, wraps at 16383 = one motor revolution: UNWRAP
                          // in post-processing before differentiating.
                          // 1 count = 0.02197 deg mech = 0.1538 deg elec
                          // (was 0.0879 / 0.6152 on the 4096-count ABZ path).
};
LogSample logbuf[LOG_N];
volatile uint16_t log_i = 0;
volatile bool     log_active = false;
bool     log_ready = false;
bool     log_announced = false;
uint8_t  log_decim = 1;
uint8_t  log_skip  = 0;
uint32_t log_t0 = 0, log_t1 = 0;
uint32_t log_t_prev = 0;        // timestamp of the previous stored sample
// Conditions the capture was taken under. Gate D was invalidated by a capture
// taken with the motor disarmed -- the STATS line showed a stale Uq=2.0 and
// vel=2.0 and looked entirely healthy. Record the state so a capture can never
// be separated from the conditions that produced it.
Mode     log_mode = MODE_OPENLOOP;
bool     log_running = false;
// Step between two NONZERO currents: stepping from 0 puts the 532 us dead-zone
// traverse in the measurement and hides the true electrical bandwidth.
const float KICK_BASE = 0.5f;   // pre-step hold current
const float KICK_A    = 1.5f;   // post-step current
// TORQUE(V) step pair, for the inertia (J) and electrical-time-constant (L)
// measurements. A Uq step is a KNOWN excitation even when the current loop is
// untuned, so those tests do not depend on the open CUR_TF bisection. Iq is
// logged every sample, so torque is reconstructed rather than assumed constant.
// 0.20 V clears the dead-zone offset and keeps the rotor moving (stiction out of
// the picture); 0.80 V gives dIq = 0.6/0.218 = 2.75 A of step at ~4 W.
const float KICKV_BASE = 0.20f;   // V -- pre-step hold voltage
const float KICKV_A    = 0.80f;   // V -- post-step voltage
uint32_t kick_at = 0;           // scheduled kick (two-stage: zero -> settle -> step)
bool kick_zero = false;
// DIRECTION OF THE KICK. Added 2026-09-05: every ring capture in the belt
// stiffness campaign stepped POSITIVE, because 'k' overwrote target with
// +KICKV_BASE regardless of which way the shaft had been jogged first. Four
// captures were taken believing two of them were negative-going. A drive with
// dead-time asymmetry (+0.2093 / -0.2239 ohm, 7% apart) and one-sided tooth
// engagement has no reason to be symmetric, so this was a real blind spot and
// not a cosmetic one. 'k' steps positive, 'K' steps negative; the sign is
// applied to BOTH the hold and the step so the whole excitation mirrors.
float kick_sign = +1.0f;

// Capture serial number and dump counter. Added 2026-09-05 after a ring-test
// session archived FOUR dumps of which only THREE were unique: a 'k' press was
// refused (wrong mode), no capture was armed, and the following 'd' re-dumped
// the previous buffer byte-for-byte. Nothing in the CSV said so. The sequence
// number is printed in the dump HEADER precisely so it survives into the
// archived file -- two rows with the same cap= is unmistakable offline, which a
// "check the console for CAPTURE start" habit is not.
uint16_t log_seq   = 0;         // increments per capture armed
uint8_t  log_dumps = 0;         // times THIS capture has been dumped

void logStart(uint8_t decim) {
  log_decim = decim; log_skip = 0; log_i = 0;
  log_ready = false; log_announced = false;
  log_mode = mode; log_running = running;
  log_seq++; log_dumps = 0;
  log_t0 = micros(); log_t_prev = log_t0; log_active = true;
  SerialUART.print(F("CAPTURE start decim=")); SerialUART.print(decim);
  SerialUART.print(F(" cap=")); SerialUART.println(log_seq);
}

void logDump() {
  if (!log_ready) { SerialUART.println(F("no capture in buffer")); return; }
  float dt_us = (float)(log_t1 - log_t0) / (float)(log_i > 1 ? (log_i - 1) : 1);
  log_dumps++;
  if (log_dumps > 1) {
    SerialUART.print(F("!! RE-DUMP of capture ")); SerialUART.print(log_seq);
    SerialUART.print(F(" (dump #")); SerialUART.print(log_dumps);
    SerialUART.println(F(") -- NO new capture since the last dump. Identical data."));
  }
  SerialUART.println(F("# BURST DUMP"));
  SerialUART.print(F("# cap=")); SerialUART.print(log_seq);
  SerialUART.print(F(" dump=")); SerialUART.print(log_dumps);
  SerialUART.print(F(" samples=")); SerialUART.print(log_i);
  SerialUART.print(F(" decim=")); SerialUART.print(log_decim);
  SerialUART.print(F(" dt_us=")); SerialUART.print(dt_us, 2);
  SerialUART.print(F(" fs_Hz=")); SerialUART.println(1e6f / dt_us, 1);
  SerialUART.print(F("# mode=")); SerialUART.print(log_mode == MODE_OPENLOOP ? "OL"
  : log_mode == MODE_TORQUE ? "TV"
  : log_mode == MODE_TORQUE_CURRENT ? "TI" : "VEL");
  SerialUART.print(F(" run=")); SerialUART.println(log_running ? 1 : 0);
  // Sampling jitter. If max/min spread more than ~20%, every downstream fit must
  // use the per-sample t_us column, not the mean.
  uint16_t dt_min = 0xFFFF, dt_max = 0;
  for (uint16_t k = 1; k < log_i; k++) {      // k=0 spans logStart -> first sample
    if (logbuf[k].dt_us < dt_min) dt_min = logbuf[k].dt_us;
    if (logbuf[k].dt_us > dt_max) dt_max = logbuf[k].dt_us;
  }
  SerialUART.print(F("# dt_us min=")); SerialUART.print(dt_min);
  SerialUART.print(F(" max="));        SerialUART.println(dt_max);
  SerialUART.println(F("i,t_us,vel,Iq,Id,Uq,Ud,sp,rawI,cnt"));
  uint32_t t_acc = 0;
  for (uint16_t k = 0; k < log_i; k++) {
    t_acc += logbuf[k].dt_us;               // true elapsed, not k * dt_mean
    SerialUART.print(k); SerialUART.print(',');
    SerialUART.print(t_acc); SerialUART.print(',');
    SerialUART.print(logbuf[k].vel_x50 / 50.0f, 2); SerialUART.print(',');
    SerialUART.print(logbuf[k].iq_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].id_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].uq_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].ud_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].sp_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].raw_x100 / 100.0f, 2); SerialUART.print(',');
    SerialUART.println(logbuf[k].cnt);
  }
  SerialUART.println(F("# END"));
}

// Mean of the capture buffer. The R_eff and Ke sweeps are ~10 steady-state points
// each; reading every point off a 1000-row CSV is the dominant time cost of the
// campaign. Skips the first 25% so an arming transient never averages into a
// steady-state point. For k/j step captures, this reports the settled value, not
// the transient. Echoes dead_zone so a measurement can never get separated
// from the condition it was taken under.
void logStats() {
  if (log_active)  { SerialUART.println(F("capture still running -- wait")); return; }
  if (!log_ready)  { SerialUART.println(F("no capture in buffer")); return; }
  uint16_t n = log_i;
  if (n < 8)       { SerialUART.println(F("capture too short")); return; }
  uint16_t k0 = n / 4, m = n - k0;
  float sIq=0, sId=0, sUq=0, sUd=0, sVel=0, sRaw=0, iq_lo=1e6f, iq_hi=-1e6f;
  for (uint16_t k = k0; k < n; k++) {
    float iq = logbuf[k].iq_x1000 / 1000.0f;
    sIq  += iq;
    sId  += logbuf[k].id_x1000 / 1000.0f;
    sUq  += logbuf[k].uq_x1000 / 1000.0f;
    sUd  += logbuf[k].ud_x1000 / 1000.0f;
    sVel += logbuf[k].vel_x50 / 50.0f;
    sRaw += logbuf[k].raw_x100 / 100.0f;
    if (iq < iq_lo) iq_lo = iq;
    if (iq > iq_hi) iq_hi = iq;
  }
  float mIq = sIq / m, mRaw = sRaw / m;
  SerialUART.print(F("STATS n="));  SerialUART.print(m);
  SerialUART.print(F(" m="));       SerialUART.print(log_mode == MODE_OPENLOOP ? "OL"
  : log_mode == MODE_TORQUE ? "TV"
  : log_mode == MODE_TORQUE_CURRENT ? "TI" : "VEL");
  SerialUART.print(F(" run="));     SerialUART.print(log_running ? 1 : 0);
  SerialUART.print(F(" dz="));      SerialUART.print(driver.dead_zone, 4);
  SerialUART.print(F(" Uq="));      SerialUART.print(sUq / m, 4);
  SerialUART.print(F(" Ud="));      SerialUART.print(sUd / m, 4);
  SerialUART.print(F(" Iq="));      SerialUART.print(mIq, 4);
  SerialUART.print(F(" Id="));      SerialUART.print(sId / m, 4);
  SerialUART.print(F(" vel="));     SerialUART.print(sVel / m, 3);
  SerialUART.print(F(" |I|="));     SerialUART.print(mRaw, 3);
  SerialUART.print(F(" Iq_pp="));   SerialUART.print(iq_hi - iq_lo, 3);
  // Angle + calibration integrity: |I| / |Iq| must sit near sqrt(3/2) = 1.225.
  SerialUART.print(F(" ratio="));
  SerialUART.println(fabsf(mIq) > 0.05f ? mRaw / fabsf(mIq) : 0.0f, 3);
}

const char* modeName() {
  switch (mode) {
    case MODE_OPENLOOP:       return "OPENLOOP";
    case MODE_TORQUE:         return "TORQUE(V)";
    case MODE_TORQUE_CURRENT: return "TORQUE(I)";
    case MODE_VELOCITY:       return "VELOCITY";
  }
  return "?";
}

bool needsFOC(Mode m)   { return (m == MODE_TORQUE || m == MODE_TORQUE_CURRENT || m == MODE_VELOCITY); }
bool isTorqueMode(Mode m){ return (m == MODE_TORQUE || m == MODE_TORQUE_CURRENT); }

// ---------------------------------------------------------------------------
// ENCODER SELF-TEST -- the acceptance gate for the SPI link.
// Motor DISABLED, zero current, zero risk. Answers three questions that must all
// pass before any current flows:
//   1. Does the link work at all?          (parity error rate)
//   2. How long does a read actually take? (loop-rate budget)
//   3. Is the magnet strong enough?        (No_Mag_Warning -- ABZ could not tell)
// Run it stationary AND while hand-spinning: a link that passes at rest and
// fails while moving is a signal-integrity problem, not a wiring problem.
// ---------------------------------------------------------------------------
void encoderSelfTest() {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  // N RAISED 2000 -> 20000 on 2026-09-20, and the verdict now includes span.
  // 'e' was reporting "ENC PASS: link clean" on an assembly that 'E' showed
  // failing in bursts, and both halves of that were its own fault:
  //   SAMPLE SIZE. 2000 reads is ~13 ms. The observed fault is BURSTY -- clean
  //   for tens of seconds, then thousands of errors inside one 250 ms window.
  //   A 13 ms probe usually lands in the quiet. At the 2026-09-20 "clean" run's
  //   own rate (2 errors in 1,441,635 reads) 'e' expects 0.003 errors per press,
  //   so it reads PASS about 99.7% of the time REGARDLESS of the link.
  //   VERDICT. span was printed but was NOT in the pass condition, so a run with
  //   a visibly corrupted angle could still print "link clean".
  // 20000 reads is ~133 ms -- still instant to a human, 10x the detection floor.
  // It is still a SNAPSHOT: for a bursty fault use 'E' and let it soak.
  const uint16_t N = 20000;
  uint32_t err0 = encoder.spi_err, ok0 = encoder.spi_ok;
  uint16_t lo = 0xFFFF, hi = 0;
  uint8_t  nmg = 0;
  uint32_t t0 = micros();
  for (uint16_t k = 0; k < N; k++) {
    if (encoder.readAngleRaw()) {
      if (encoder.raw < lo) lo = encoder.raw;
      if (encoder.raw > hi) hi = encoder.raw;
      nmg |= encoder.no_mag;
    }
  }
  uint32_t dt = micros() - t0;
  uint32_t errs = encoder.spi_err - err0, oks = encoder.spi_ok - ok0;
  encoder.readOverSpeed();
  SerialUART.print(F("ENC n="));        SerialUART.print(N);
  SerialUART.print(F(" ok="));          SerialUART.print(oks);
  SerialUART.print(F(" parity_err="));  SerialUART.print(errs);
  SerialUART.print(F(" ("));            SerialUART.print(100.0f * errs / N, 3);
  SerialUART.print(F("%) us_per_read="));SerialUART.print((float)dt / N, 2);
  SerialUART.print(F(" raw="));         SerialUART.print(encoder.raw);
  SerialUART.print(F(" span="));        SerialUART.print(hi - lo);
  SerialUART.print(F(" no_mag="));      SerialUART.print(nmg);
  SerialUART.print(F(" over_speed="));  SerialUART.println(encoder.over_speed);
  // span is in the verdict now. On a STATIONARY shaft raw must not move, and a
  // nonzero span is a corrupted frame that PASSED parity -- the failure parity
  // structurally cannot see. Reporting it and then ignoring it was worse than
  // not measuring it, because it made a bad link print the word "clean".
  const uint16_t span = (hi > lo) ? (uint16_t)(hi - lo) : 0;
  if (errs == 0 && oks == N && span == 0) {
    SerialUART.println(F("ENC PASS: link clean (SNAPSHOT -- blind to a bursty fault, use E to soak)"));
  } else {
    SerialUART.println(F("ENC FAIL: check wiring / slow SPI_HALF_NOPS down"));
    if (span) SerialUART.println(F("  span != 0 on a still shaft = corruption that PASSED parity"));
  }
  if (nmg) SerialUART.println(F("!! No_Mag_Warning -- magnet too weak or too far. Angle is GARBAGE."));
}

// ---------------------------------------------------------------------------
// 'E' -- CONTINUOUS HARNESS MONITOR. The instrument for the wiggle test.
// ---------------------------------------------------------------------------
// 'e' is one-shot: 2000 reads, ~13 ms, print, done. Useless for flexing a
// conductor, because you cannot press a key and wiggle a wire at the same time
// and the answer arrives after you have stopped. This repeats until a key is
// pressed, so both hands are free and the console is a live readout.
//
// TWO DETECTORS, and the second one is the point:
//   perr   PARITY. One bit over a 16-bit word, so it catches an ODD number of
//          flipped bits and MISSES AN EVEN NUMBER -- roughly half of all
//          multi-bit corruptions sail through it looking perfect.
//   span   STATIONARY-SHAFT ANGLE SPAN. With the shaft held still, `raw` must
//          not move. Any span at all is a corrupted frame that PASSED parity.
//          This is the detector parity cannot be, and nothing in the firmware
//          was watching it -- SPI_JUMP_GUARD is false (mt6816.h), so spi_jump
//          is never incremented and `jrej` never prints.
//
// ⚠ no_mag is only written on a SUCCESSFUL read (mt6816.h, after the parity
// check). During an error burst it holds its last good value, so nmg = 0 across
// a fault does NOT exclude a field problem. Stated because that inference has
// already been drawn once from the session logs.
// ---------------------------------------------------------------------------
void encoderMonitor() {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  SerialUART.println(F("ENC MONITOR -- shaft STATIONARY. Flex ONE conductor at a time,"));
  SerialUART.println(F("  near the connector then at the breakout: PB5 CSN, PB6 MOSI, PB7 MISO,"));
  SerialUART.println(F("  PB8 SCK, then VCC and GND. Any key stops."));
  const uint16_t WIN_MS = 250;
  const uint32_t e0_all = encoder.spi_err;
  uint32_t n_all = 0, win = 0, worst_err = 0;
  uint16_t worst_span = 0;
  while (!SerialUART.available()) {
    const uint32_t e0 = encoder.spi_err;
    uint32_t n = 0;
    uint16_t lo = 0xFFFF, hi = 0;
    uint8_t  nmg = 0;
    const uint32_t t_end = millis() + WIN_MS;
    while (millis() < t_end) {
      if (encoder.readAngleRaw()) {
        if (encoder.raw < lo) lo = encoder.raw;
        if (encoder.raw > hi) hi = encoder.raw;
        nmg |= encoder.no_mag;
      }
      n++;
    }
    const uint32_t errs = encoder.spi_err - e0;
    // raw is only written on a SUCCESSFUL read, so a corrupt-but-parity-passing
    // frame shows up as one outlier inside the window and the next good read
    // pulls it back -- which is exactly what this span sees.
    const uint16_t span = (uint16_t)((hi > lo) ? (hi - lo) : 0);
    n_all += n; win++;
    if (errs > worst_err)  worst_err  = errs;
    if (span > worst_span) worst_span = span;
    SerialUART.print(F("ENC ")); SerialUART.print(win);
    SerialUART.print(F("  n="));      SerialUART.print(n);
    SerialUART.print(F(" perr="));    SerialUART.print(errs);
    SerialUART.print(F(" span="));    SerialUART.print(span);
    SerialUART.print(F(" nmg="));     SerialUART.print(nmg);
    // Loud, because you are looking at the wire and not at the screen.
    if (errs || span) SerialUART.println(F("   <<<<<< HIT"));
    else              SerialUART.println();
  }
  while (SerialUART.available()) (void)SerialUART.read();
  SerialUART.print(F("ENC MONITOR end: windows=")); SerialUART.print(win);
  SerialUART.print(F(" reads="));       SerialUART.print(n_all);
  SerialUART.print(F(" perr_total="));  SerialUART.print(encoder.spi_err - e0_all);
  SerialUART.print(F(" worst_window=")); SerialUART.print(worst_err);
  SerialUART.print(F(" worst_span="));  SerialUART.println(worst_span);
  SerialUART.println(F("  span > 0 on a stationary shaft = corruption that PASSED parity."));
}

void printHelp() {
  SerialUART.println(F("--- g:go  x/s:stop  +/-:target | modes: o=open t=torque(V) c=torque(I) v=vel ---"));
  SerialUART.println(F("--- f:initFOC(stored)  F:force align  e:enc self-test  E:enc MONITOR (wiggle)  q:print int ---"));
  SerialUART.println(F("--- logger: l=fast L=slow  k=kick(+) K=kick(-)  j=zero-kick d=dump a=stats ---"));
  SerialUART.println(F("--- V:verify stored ZEA | Y:autocalib menu  1..6:phases  7:report  0:reset ---"));
  SerialUART.println(F("--- p: VBUS/ADC register dump (read-only, motor disabled) ---"));
  SerialUART.println(F("--- manual: N=M2 bus-power ladder  B/b=M4 breakaway ramp +/- ---"));
  SerialUART.println(F("--- w: SWING LADDER 0.6/1.0/1.4/1.6 A -- OUTPUT LOCKED, ~80 s, stops on tooth skip ---"));
  SerialUART.println(F("--- '-' then '5' (within 0.8s): phase 5 runs REVERSE first, not forward ---"));
}

void startMotor() {
  if (!driver_ok) { SerialUART.println(F("refused: driver init failed")); return; }
  if (needsFOC(mode) && !foc_ready) { SerialUART.println(F("refused: run initFOC (f) first")); return; }
  if (mode == MODE_TORQUE_CURRENT && !cs_linked) { SerialUART.println(F("refused: current sense not linked")); return; }
  if (isTorqueMode(mode) && target != 0.0f) {
    target = 0.0f;
    SerialUART.println(F("torque mode: target reset to 0 on arm (ramp with +)"));
  }
  // clear stale controller state so a previous run cannot leak into this one
  motor.PID_velocity.reset();
  motor.PID_current_q.reset();
  motor.PID_current_d.reset();
  encoder.update();                 // refresh sensor before the loop engages
  motor.enable();
  running = true; run_started = millis();
  SerialUART.print(F("RUNNING ")); SerialUART.print(modeName());
  SerialUART.print(F(" target=")); SerialUART.println(target);
}

#include "safety.h"   // stopMotor() -- the single disable path, extracted verbatim

void setMode(Mode m) {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  if (needsFOC(m) && !foc_ready) { SerialUART.println(F("run initFOC (f) first")); return; }
  if (m == MODE_TORQUE_CURRENT && !cs_linked) { SerialUART.println(F("current sense not linked")); return; }
  mode = m;
  switch (mode) {
    case MODE_OPENLOOP:
      motor.controller = MotionControlType::velocity_openloop; target = 2.0f; break;
    case MODE_TORQUE:  // legacy voltage-torque, kept for A/B
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::torque;
      target = 0.0f; break;
    case MODE_TORQUE_CURRENT:
      motor.torque_controller = TorqueControlType::foc_current;
      motor.controller = MotionControlType::torque; target = 0.0f; break;
    case MODE_VELOCITY:
      // velocity closes on the CURRENT loop: current_limit binds here (stall = 2 A,
      // not 7-9 A) and Iq/Id telemetry is live. Velocity PID output is in AMPS.
      motor.torque_controller = TorqueControlType::foc_current;
      motor.controller = MotionControlType::velocity;
      motor.PID_velocity.limit = CURR_LIMIT;
      target = 2.0f; break;
  }
  SerialUART.print(F("mode=")); SerialUART.print(modeName());
  SerialUART.print(F(" target=")); SerialUART.println(target);
}


void adjustTarget(float dir) {
  if (mode == MODE_TORQUE)              target = constrain(target + dir*TORQUE_STEP, -TORQUE_MAX, TORQUE_MAX);
  else if (mode == MODE_TORQUE_CURRENT) target = constrain(target + dir*CURR_STEP,  -CURR_MAX,  CURR_MAX);
  else                                  target = constrain(target + dir*VEL_STEP,   -VEL_MAX,   VEL_MAX);
  SerialUART.print(F("target=")); SerialUART.println(target);
}

#include "autocalib.h"          // immediately above void handleSerial()

// Chord tracking for '-' then '5' (swapped-order phase 5, see ac_p5_swap_next
// in autocalib.h). Deliberately narrow: only a '5' arriving within
// AC_CHORD_WINDOW_MS of a '-'/'_' arms the swap, so a '-' typed minutes
// earlier for ordinary target jogging can never silently swap a later,
// unrelated phase-5 run -- the exact stale-flag mislabeling class the belt
// banner bug already cost this project twice.
static char     ac_last_key    = 0;
static uint32_t ac_last_key_ms = 0;
static const uint32_t AC_CHORD_WINDOW_MS = 800;

void handleSerial() {
  while (SerialUART.available()) {
    char c = (char)SerialUART.read();
    if (c == '5' && (ac_last_key == '-' || ac_last_key == '_') &&
        (millis() - ac_last_key_ms < AC_CHORD_WINDOW_MS)) {
      ac_p5_swap_next = true;
    }
    switch (c) {
      case 'g': case 'G': startMotor(); break;
      case 'x': case 'X': case 's': case 'S': stopMotor("user"); break;
      case '+': case '=': adjustTarget(+1); break;
      case '-': case '_': adjustTarget(-1); break;   // also arms swap if '5' follows -- see chord above
      case 'o': case 'O': setMode(MODE_OPENLOOP); break;
      case 't': case 'T': setMode(MODE_TORQUE); break;
      case 'c': case 'C': setMode(MODE_TORQUE_CURRENT); break;
      case 'v': setMode(MODE_VELOCITY); break;   // 'V' is NO LONGER velocity:
      case 'V': acVerifyZea();          break;   //   it verifies the stored ZEA
      case 'f': runInitFOC(false, running, foc_ready, target, SerialUART); break;   // uses STORED ZEA when available
      case 'F': runInitFOC(true,  running, foc_ready, target, SerialUART);  break;   // force a fresh alignment
      case 'e': encoderSelfTest(); break;              // one-shot, 2000 reads
      case 'E': encoderMonitor();  break;              // continuous -- harness wiggle test
      case 'p': vbusProbe(); break;    // VBUS ADC probe -- read-only, motor disabled
      case 'l': logStart(1); break;                     // fast capture (~65 ms)
      case 'L': logStart(8); break;                     // slow capture (~520 ms)
      case 'd': case 'D': logDump(); break;
      case 'a': case 'A': logStats(); break;            // mean of last capture
      case 'Y': case 'y': acStatus();   break;         // status / menu
      // Manual-assist, NOT part of the 1..7 chain. N / B / b chosen because
      // 'F' is force-align and 'G' is go -- binding either would have shadowed
      // an existing command silently.
      case 'N': case 'n': acM2Assist(); break;         // M2 bus-power ladder
      case 'w': case 'W': acSwingLadder(); break;   // swing ladder, currents AC_SW_I, output LOCKED
      case 'B': acM4Breakaway(+1.0f);   break;         // M4 breakaway, forward
      case 'b': acM4Breakaway(-1.0f);   break;         // M4 breakaway, reverse
      case '0': acPhase(0); break;                     // reset results
      case '1': acPhase(1); break;                     // LINK
      case '2': acPhase(2); break;                     // ALIGN
      case '3': acPhase(3); break;                     // R/U0
      case '4': acPhase(4); break;                     // L
      case '5': acPhase(5); break;                     // FREE-SPIN ('-' then '5': reverse first)
      case '6': acPhase(6); break;                     // T/INL
      case '7': acPhase(7); break;                     // REPORT
      case 'q': case 'Q':
        print_ms = (print_ms == 300) ? 3000 : 300;
        SerialUART.print(F("print_ms=")); SerialUART.println(print_ms); break;
      case 'k': case 'K':                               // step + capture. K = NEGATIVE
        if (!running || !isTorqueMode(mode)) {
          SerialUART.println(F("k: need TORQUE(V) or TORQUE(I) running"));
        } else {
          bool volts = (mode == MODE_TORQUE);
          kick_sign = (c == 'K') ? -1.0f : +1.0f;
          target = kick_sign * (volts ? KICKV_BASE : KICK_BASE);  // out of the dead zone
          kick_at = millis() + 300;                     // settle, then step (in loop)
          SerialUART.print(F("kick armed ")); SerialUART.print(kick_sign > 0 ? '+' : '-');
          SerialUART.print(F(": hold ")); SerialUART.print(target, 3);
          SerialUART.print(volts ? F(" V") : F(" A"));
          SerialUART.print(F(" 300ms, then step to "));
          SerialUART.println(kick_sign * (volts ? KICKV_A : KICK_A), 3);
        }
        break;
      case 'j': case 'J':                               // zero-based step: measures dead-zone traverse
        if (!running || !isTorqueMode(mode)) {
          SerialUART.println(F("j: need TORQUE(V) or TORQUE(I) running"));
        } else {
          target = 0.0f;
          kick_at = millis() + 300;
          kick_zero = true;
          kick_sign = +1.0f;          // j is positive-only; do not inherit a prior K
          SerialUART.println(F("kick armed from ZERO (dead-zone traverse)"));
        }
        break;
      case '?': printHelp(); break;
      default: break;
    }
    ac_last_key = c; ac_last_key_ms = millis();
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // PC6 (STATUS)
  SerialUART.begin(921600);
  _delay(2000);
  SimpleFOCDebug::enable(&SerialUART);
  SerialUART.println(F("=== actuator + current mode + MT6816 SPI (bit-banged) ==="));
  printJointCal(SerialUART);

  // ---- HARDWARE BRING-UP, PHASE 1 -- the order lives in actuator_hw.h ----
  // The bench envelope is passed IN rather than read out of this file, so the
  // header carries no harness policy and Tier-0 can supply its own.
  const ActuatorHwCfg hw_cfg = {
    DRIVER_VOLT_LIMIT, DEAD_ZONE, (long)PWM_FREQ_HZ,
    PIN_VBUS, VBUS_SCALE, VBUS_MIN, VBUS_MAX, VBUS_FALLBACK
  };
  driver_ok = actuatorInitHw(hw_cfg, SerialUART);

  // ---- MOTOR POLICY -- THE HARNESS'S OWN. Tier-0 replaces this block. ----
  // It sits BETWEEN the two init phases because motor.init() consumes
  // voltage_limit; see the init-order note in actuator_hw.h. Do not move it.
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit  = VOLT_LIMIT;
  motor.velocity_limit = VEL_MAX;
  motor.current_limit  = CURR_LIMIT;

  // 2.3.1 defaults foc_modulation to SinePWM; this sketch inherited that silently.
  // SVPWM raises the linear ceiling from V_bus/2 to V_bus/sqrt(3) (+15.5% of usable
  // voltage). Only the CEILING changes: at bench modulation depth (2 V of 11.4 V)
  // the fundamental phase voltage for a given Uq is identical, so the phase
  // currents should NOT change. The zero-sequence component SVPWM adds is
  // common-mode; with no neutral connection the phase currents are unaffected, so
  // the |I|/Iq = 1.225 integrity check still holds. A/B this against the ammeter
  // baseline before trusting anything downstream of it.
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.PID_velocity.P = VEL_P;
  motor.PID_velocity.I = VEL_I;
  motor.PID_velocity.D = VEL_D;
  motor.LPF_velocity.Tf = VEL_TF;

  motor.PID_current_q.P = CURQ_P;  motor.PID_current_q.I = CURQ_I;  motor.PID_current_q.D = 0.0f;
  motor.PID_current_d.P = CURD_P;  motor.PID_current_d.I = CURD_I;  motor.PID_current_d.D = 0.0f;
  motor.LPF_current_q.Tf = CUR_TF;
  motor.LPF_current_d.Tf = CUR_TF;
  motor.PID_current_q.limit = VOLT_LIMIT;
  motor.PID_current_d.limit = VOLT_LIMIT;

  motor.voltage_sensor_align = 1.0f;

  // ---- HARDWARE BRING-UP, PHASE 2 ----
  cs_ok = actuatorInitMotor(SerialUART, cs_linked);

  printCfgBanner(SerialUART, DRIVER_VOLT_LIMIT, CURR_MAX);

  // DIRECTION IS NOT PRESET ON THIS BOARD. The ABZ build hardcoded CCW because
  // the TIM4 count convention had been confirmed; the MT6816 SPI angle runs on
  // its own convention (ROT_DIR register, CCW-increasing by default) and the
  // magnet mount orientation may differ on this assembly. Let initFOC detect it
  // once, then pin DIR_STORED. Copying CCW across untested would silently
  // invert the torque sign.
  motor.sensor_direction = (DIR_STORED > 0) ? Direction::CW
                         : (DIR_STORED < 0) ? Direction::CCW
                                            : Direction::UNKNOWN;
  foc_ready = false;
  SerialUART.print(F("ALIGN src="));
  SerialUART.println((ZEA_STORED >= 0.0f && DIR_STORED != 0) ? F("STORED") : F("measure with f"));

  SerialUART.println(F("Motor DISABLED. Run 'e' (encoder self-test) BEFORE 'f'."));
  printHelp();
}

void loop() {
  handleSerial();

  // ---- BUS VOLTAGE: 1 kHz sample -> filter -> republish derived limits ----
  if (VBUS_SCALE > 0.0f && VBUS_LIVE) {
    static uint32_t vbus_last_us = 0;
    uint32_t now_us = micros();
    // Cast handles micros() wraparound at ~71 min. Rate-limited to 1 kHz because
    // analogRead() blocks for a few us; once per 12.5 loops is 0.5% overhead,
    // once per loop would be ~6%. The cost lands in dt_us, which is logged.
    if ((uint32_t)(now_us - vbus_last_us) >= 1000) {
      vbus_last_us = now_us;
      (void)analogRead(PIN_VBUS);
      float v = (float)analogRead(PIN_VBUS) * VBUS_SCALE;
      if (v > VBUS_MIN && v < VBUS_MAX) {
        // One-pole low-pass, 1 ms sample period, 20 ms time constant -> each new
        // reading contributes 4.8%. HEAVILY filtered on purpose: this value is the
        // divisor in every duty-cycle calculation, so ADC noise here is injected
        // straight into the motor current and from there into the current loop.
        const float a = 0.001f / (VBUS_TF + 0.001f);
        vbus_filt += a * (v - vbus_filt);
        vbus_valid = true;
        driver.voltage_power_supply = vbus_filt;
        // SVPWM can only synthesise rail/sqrt(3) = 0.57735*rail. A PI output
        // limit ABOVE that is not a limit: the integrator winds up against a
        // ceiling that does not exist, then dumps the windup when the bus
        // recovers.
        // THE RAIL IS NOT THE BUS. setPhaseVoltage() normalises against
        // driver.voltage_limit and setPwm() clamps each phase to it, so the
        // binding rail is min(driver.voltage_limit, V_bus). This used to read
        // vbus_filt * 0.57735, which at DRIVER_VOLT_LIMIT = 6.0 on a 12.5 V bus
        // claims 7.19 V against a real 3.46 V -- a 2x-optimistic "limit", i.e.
        // exactly the failure this block exists to prevent. Dormant while
        // VBUS_LIVE = false; fixed now rather than the day it is switched on.
        float rail    = (DRIVER_VOLT_LIMIT < vbus_filt) ? DRIVER_VOLT_LIMIT : vbus_filt;
        float ceiling = rail * 0.57735f;
        float lim = (VOLT_LIMIT < ceiling) ? VOLT_LIMIT : ceiling;
        motor.voltage_limit       = lim;
        motor.PID_current_q.limit = lim;
        motor.PID_current_d.limit = lim;
      } else {
        vbus_valid = false;              // implausible: hold last good value
      }
    }
  }

  unsigned long period = (driver_ok && cs_ok) ? 500 : 80;
  if (millis() - last_blink > period) { led_state=!led_state; digitalWrite(LED_BUILTIN, led_state); last_blink=millis(); }

  motor.loopFOC();
  g_loops++;
  // Avoid a second encoder read here. The extra update was measured to be
  // redundant in the disarmed/open-loop paths and did not change the frozen
  // velocity behavior, so it only costs additional loop time.

  // 2.3.1's loopFOC() refreshes motor.current ONLY in the dc_current and
  // foc_current branches; the voltage branch returns without touching it. In
  // TORQUE(V), Iq/Id were therefore STALE -- left behind by initFOC or the last
  // TORQUE(I) run -- and every Uq-vs-Iq measurement would have read a frozen
  // number that looks exactly like data. (|I| from getPhaseCurrents() was always
  // live, which is why this never showed up before.)
  // Unfiltered on purpose: LPF_current (Tf = 500 us) is ~2x the electrical time
  // constant and would dominate any current rise-time fit.
  if (cs_linked && mode == MODE_TORQUE) {
    motor.current = currentSense.getFOCCurrents(motor.electrical_angle);
  } else if (mode == MODE_OPENLOOP) {
    // Deliberately zeroed rather than left stale: velocityOpenloop() never writes
    // electrical_angle, so a dq transform here would be meaningless. Read |I|.
    motor.current.q = 0.0f; motor.current.d = 0.0f;
  }

  // SENSE-MISMATCH GUARD: raw (unsynchronised) |I| must stay near AMP_INV_MAG*|Iq|.
  // A large divergence means the dq feedback has collapsed and the loop is winding
  // to the voltage rail -- the 8-12 A runaway. Checked every ~2 ms.
  // The raw read is ONE unsynchronised instant of a PWM-rippling current, so it spikes
  // several x the average during legitimate transients (e.g. breaking free of stiction).
  // A real runaway is sustained -> require GUARD_HITS consecutive violations.
  static uint16_t guard_div = 0;
  static uint8_t  guard_hits = 0;
  const uint8_t   GUARD_HITS = 8;          // ~8 x 2 ms = 16 ms of sustained mismatch
  if (running && mode == MODE_TORQUE_CURRENT && ++guard_div >= 30) {
    guard_div = 0;
    PhaseCurrent_s gc = currentSense.getPhaseCurrents();
    float raw = sqrtf(gc.a*gc.a + gc.b*gc.b + gc.c*gc.c);
    float expect = AMP_INV_MAG * fabsf(motor.current.q) + 0.6f;
    if (raw > expect * 3.0f && raw > 3.0f && fabsf(motor.voltage.q) < VOLT_LIMIT * 0.9f) {
      if (++guard_hits >= GUARD_HITS) {
        guard_hits = 0;
        stopMotor("SENSE MISMATCH (sustained raw |I| >> dq Iq)");
      }
    } else {
      guard_hits = 0;                      // any good sample clears the count
    }
  }

  if (running) {
    unsigned long run_ms = millis() - run_started;
    // WHO WRITES shaft_velocity: motor.move() only. In the closed-loop modes it
    // is the MEASURED speed and this guard is real. In OPENLOOP,
    // velocityOpenloop() overwrites it with the COMMANDED value, which `target`
    // already caps at VEL_MAX = 20 -- so this guard cannot trip in openloop and
    // is not protection there. acService() carries the same caveat.
    if (run_ms > OVERSPEED_GRACE_MS && fabsf(motor.shaft_velocity) > OVERSPEED_RADS)
                                                          stopMotor("OVERSPEED");
    else if (run_ms > AUTO_STOP_MS)                        stopMotor("auto 20s");
    else                                                   motor.move(target);
  }

  // scheduled kick: base settled -> start capture, then step (capture sees the edge)
  if (kick_at && millis() >= kick_at) {
    kick_at = 0;
    if (running && isTorqueMode(mode)) {
      logStart(1);
      target = kick_sign * ((mode == MODE_TORQUE) ? KICKV_A
                                                  : (kick_zero ? 1.0f : KICK_A));
      kick_zero = false;
    }
  }

  // burst capture: one sample per (decim) loop iterations, no serial in the path
  if (log_active) {
    if (++log_skip >= log_decim) {
      log_skip = 0;
      uint32_t t_now = micros();
      LogSample &e = logbuf[log_i];
      e.dt_us    = (uint16_t)(t_now - log_t_prev);
      log_t_prev = t_now;
      e.vel_x50  = (int16_t)(motor.shaft_velocity * 50.0f);
      e.iq_x1000 = (int16_t)(motor.current.q * 1000.0f);
      e.id_x1000 = (int16_t)(motor.current.d * 1000.0f);
      e.uq_x1000 = (int16_t)(motor.voltage.q * 1000.0f);
      e.ud_x1000 = (int16_t)(motor.voltage.d * 1000.0f);
      e.sp_x1000 = (int16_t)(motor.current_sp * 1000.0f);
      PhaseCurrent_s lc = currentSense.getPhaseCurrents();
      e.raw_x100 = (int16_t)(sqrtf(lc.a*lc.a + lc.b*lc.b + lc.c*lc.c) * 100.0f);
      e.cnt      = encoder.raw;
      if (++log_i >= LOG_N) {
        log_active = false; log_ready = true; log_t1 = micros();
      }
    }
  }

  if (!log_active && millis() - last_print > print_ms) {
    uint32_t pr_start = micros();
    unsigned long now = millis();
    unsigned long dt  = now - last_print;
    lps = (dt > 0) ? (uint32_t)((g_loops * 1000UL) / dt) : 0;
    g_loops = 0;

    SerialUART.print(F("m=")); SerialUART.print(modeName());
    SerialUART.print(F(" run=")); SerialUART.print(running?1:0);
    SerialUART.print(F(" tgt=")); SerialUART.print(target, 2);
    SerialUART.print(F(" cnt=")); SerialUART.print(encoder.rawCount());
    SerialUART.print(F(" vel=")); SerialUART.print(motor.shaft_velocity, 2);

    // Print dq in every mode: Id is the desync detector, not a current-mode luxury.
    // In OPENLOOP these are forced to 0 above -- a deliberate zero, not a stale read.
    SerialUART.print(F(" Iq=")); SerialUART.print(motor.current.q, 2);
    SerialUART.print(F(" Id=")); SerialUART.print(motor.current.d, 2);
    PhaseCurrent_s c = currentSense.getPhaseCurrents();
    SerialUART.print(F(" |I|=")); SerialUART.print(sqrtf(c.a*c.a + c.b*c.b + c.c*c.c), 2);

    SerialUART.print(F(" Uq=")); SerialUART.print(motor.voltage.q, 3);
    SerialUART.print(F(" Ud=")); SerialUART.print(motor.voltage.d, 3);
    SerialUART.print(F(" Vb=")); SerialUART.print(vbus_filt, 2);
    // 'seed' not 'vok': with VBUS_LIVE=false, Vb is the boot measurement and is
    // NOT tracking. Printed so no capture can be read as if it were live.
    SerialUART.print(F(" Vb_src=")); SerialUART.print(VBUS_LIVE ? F("live") : F("seed"));
    // DIAGNOSTIC -- the DMA path's raw view of the same pin, so every routine
    // that already runs (phase 3, M2, free spin, burst captures) carries the
    // H7 load test for free. Vdma-Vb flat across current bounds H7; growth
    // disqualifies live Vbus. Delete with the rest of the DMA-offset work.
    printVdma();
    // SPI link health. perr is CUMULATIVE since boot: any nonzero value means
    // frames are being corrupted and the angle was stale for that many cycles.
    // nmg=1 means the magnet field is below the AMR saturation threshold and the
    // angle is meaningless -- the failure ABZ could never report.
    encoder.readOverSpeed();                 // 0x05: one extra frame per print only
    SerialUART.print(F(" perr=")); SerialUART.print(encoder.spi_err);
    SerialUART.print(F(" nmg="));  SerialUART.print(encoder.no_mag);
    SerialUART.print(F(" ovs="));  SerialUART.print(encoder.over_speed);
    if (SPI_JUMP_GUARD) { SerialUART.print(F(" jrej=")); SerialUART.print(encoder.spi_jump); }
    SerialUART.print(F(" lps=")); SerialUART.print(lps);
    SerialUART.print(F(" pr_us=")); SerialUART.println(pr_us);   // cost of the PREVIOUS print block
    if (log_ready && !log_announced) {
      SerialUART.println(F("CAPTURE done -- press d to dump")); log_announced = true;
    }
    pr_us = micros() - pr_start;      // how long this telemetry block actually took
    last_print = millis();            // start the next window AFTER printing
    g_loops = 0;                      // so lps measures control-loop rate only
  }
}