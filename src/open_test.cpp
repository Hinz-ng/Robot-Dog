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

// ---------------------------------------------------------------------------
// MT6816 absolute encoder, 4-wire SPI, BIT-BANGED
// ---------------------------------------------------------------------------
// Bit-banged on purpose. Arduino-API hardware SPI hangs on this clone: the
// vendor-pruned PeripheralPins_B_G431B_ESC1.c lookup miss lands in
// Error_Handler(), an infinite loop -- the same trap that killed STM32HWEncoder.
// Bit-banging consults no pin map at all, and MODE3 bit-bang was already proven
// working on this hardware. It also leaves the telemetry UART on PB4/PB3.
//
// ALL FOUR PINS MUST BE ON GPIOB (the fast path touches GPIOB->BSRR / ->IDR).
// Remapping is a four-line edit here; keep SCK on PB8 unless you re-read the
// BOOT0 note in the header.
const uint8_t SPI_CSN_BIT  = 5;    // PB5
const uint8_t SPI_MOSI_BIT = 6;    // PB6  (chip pin 5, was ABZ 'A')
const uint8_t SPI_MISO_BIT = 7;    // PB7  (chip pin 6, was ABZ 'B')
const uint8_t SPI_SCK_BIT  = 8;    // PB8  (chip pin 7, was ABZ 'Z') -- BOOT0

// Half-period padding. 170 MHz -> 5.88 ns/cycle. Datasheet minima: TSCK 64 ns,
// TSCKL/TSCKH 30 ns each. Measured frame times (2 frames per angle read):
//    8 NOPs -> SCK 7.7 MHz, 2.1 us/frame     20 NOPs -> 3.7 MHz, 4.3 us/frame
//   30 NOPs -> SCK 2.6 MHz, 6.2 us/frame     40 NOPs -> 2.0 MHz, 8.1 us/frame
// START CONSERVATIVE. Dupont wire next to a 25 kHz inverter switching 10 A is a
// worse signal-integrity problem than 15 kHz quadrature was; slow is free here
// because 2 frames at 30 NOPs cost 12.4 us of a 74 us loop. Only speed up if
// the loop rate actually hurts, and re-run 'e' after every change.
const uint8_t SPI_HALF_NOPS = 1; // 6.65 us/read measured.

static inline void spiHalf() {
  for (uint8_t i = 0; i < SPI_HALF_NOPS; i++) __asm__ volatile ("nop");
}

// Reject an angle step larger than this (radians, mechanical) as corruption.
// DEFAULT OFF for bring-up: one variable at a time. At 150 rad/s and a 74 us
// loop the true step is 0.011 rad, but a 1 ms serial block makes it 0.15 rad,
// so the threshold must clear that with margin. Enable only AFTER the parity
// error rate from 'e' is known to be zero.
const bool  SPI_JUMP_GUARD  = false;
const float SPI_MAX_JUMP    = 1.0f;    // rad mechanical between consecutive reads

class MT6816SPI : public Sensor {
public:
  void init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // CSN / MOSI / SCK -> push-pull outputs, very high speed.
    // MISO -> input. No pull: the MT6816 drives it push-pull in SPI mode.
    const uint8_t outs[3] = { SPI_CSN_BIT, SPI_MOSI_BIT, SPI_SCK_BIT };
    for (uint8_t k = 0; k < 3; k++) {
      uint8_t p = outs[k];
      GPIOB->MODER   &= ~(3UL << (p * 2));
      GPIOB->MODER   |=  (1UL << (p * 2));    // 01 = general purpose output
      GPIOB->OTYPER  &= ~(1UL << p);          // push-pull
      GPIOB->OSPEEDR |=  (3UL << (p * 2));    // very high speed
      GPIOB->PUPDR   &= ~(3UL << (p * 2));    // no pull
    }
    GPIOB->MODER &= ~(3UL << (SPI_MISO_BIT * 2));   // 00 = input
    GPIOB->PUPDR &= ~(3UL << (SPI_MISO_BIT * 2));

    // Idle state: CSN high (deselected), SCK HIGH (mode 3 requires CPOL=1).
    GPIOB->BSRR = (1UL << SPI_CSN_BIT) | (1UL << SPI_SCK_BIT);
    GPIOB->BSRR = (1UL << (SPI_MOSI_BIT + 16));

    // TPwrUp is 16 ms from VDD. setup() has already burned 2 s on the serial
    // delay, so the chip is long ready -- but be explicit rather than lucky.
    delay(20);

    last_ok_rad = 0.0f;
    (void)readAngleRaw();          // prime last_ok_rad and the error counters
    last_ok_rad = raw * ENC_RAD_PER_COUNT;

    this->Sensor::init();
  }

  // ---- one 16-bit mode-3 transfer -------------------------------------------
  // CPOL=1 CPHA=1: SCK idles high; data changes on the falling edge and is
  // sampled on the rising edge (datasheet 8.6.2 / figure 17).
  uint16_t xfer16(uint16_t out) {
    uint16_t in = 0;
    GPIOB->BSRR = (1UL << (SPI_CSN_BIT + 16));      // CSN low -> start
    spiHalf();                                      // TL: CSN fall -> first SCK fall
    for (int8_t i = 15; i >= 0; i--) {
      GPIOB->BSRR = (1UL << (SPI_SCK_BIT + 16));    // SCK falling edge
      if (out & (1UL << i)) GPIOB->BSRR = (1UL << SPI_MOSI_BIT);
      else                  GPIOB->BSRR = (1UL << (SPI_MOSI_BIT + 16));
      spiHalf();
      GPIOB->BSRR = (1UL << SPI_SCK_BIT);           // SCK rising edge -> sample
      spiHalf();
      in <<= 1;
      if (GPIOB->IDR & (1UL << SPI_MISO_BIT)) in |= 1;
    }
    spiHalf();                                      // TH: last SCK rise -> CSN rise
    GPIOB->BSRR = (1UL << SPI_CSN_BIT);             // CSN high -> stop
    return in;
  }

  // ---- read 0x03 + 0x04, check parity, extract the 14-bit angle --------------
  // Returns true if parity passed. On failure `raw` is left at its previous value.
  bool readAngleRaw() {
    uint8_t d03 = (uint8_t)(xfer16(0x8300) & 0xFF);   // 0x80 = read, addr 0x03
    uint8_t d04 = (uint8_t)(xfer16(0x8400) & 0xFF);
    uint16_t word = ((uint16_t)d03 << 8) | d04;
    // PC (0x04[0]) is EVEN parity over 0x03[7:0] + 0x04[7:1], so the whole
    // 16-bit word must have even parity. This is the corruption detector that
    // ABZ structurally could not provide.
    uint16_t p = word;
    p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    if (p & 1) { spi_err++; return false; }          // ODD -> corrupted frame
    raw     = (uint16_t)(((uint16_t)d03 << 6) | (d04 >> 2));   // 14 bits
    no_mag  = (d04 >> 1) & 1;
    spi_ok++;
    return true;
  }

  // SimpleFOC calls this every loopFOC(); must return 0..2PI.
  float getSensorAngle() override {
    if (!readAngleRaw()) return last_ok_rad;         // stale for ONE cycle, then recovers
    float a = raw * ENC_RAD_PER_COUNT;
    if (SPI_JUMP_GUARD) {
      float d = a - last_ok_rad;
      while (d >  _PI) d -= _2PI;                    // wrap to +-PI
      while (d < -_PI) d += _2PI;
      if (fabsf(d) > SPI_MAX_JUMP) { spi_jump++; return last_ok_rad; }
    }
    last_ok_rad = a;
    return a;
  }

  // Over_Speed lives in 0x05 and is not needed at loop rate. Poll it from the
  // telemetry block instead of paying for a third frame every cycle.
  bool readOverSpeed() {
    uint8_t d05 = (uint8_t)(xfer16(0x8500) & 0xFF);
    over_speed = (d05 >> 3) & 1;
    return over_speed;
  }

  // diagnostics
  int32_t  rawCount()   { return (int32_t)raw; }     // 0..ENC_CPR-1, one mech rev
  uint16_t raw        = 0;
  uint8_t  no_mag     = 0;
  uint8_t  over_speed = 0;
  uint32_t spi_ok     = 0;
  uint32_t spi_err    = 0;
  uint32_t spi_jump   = 0;

private:
  float last_ok_rad = 0.0f;
};

// ---------------------------------------------------------------------------

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);
// Clone sense chain is gain-compensated -> genuine constants. Do not change.
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MT6816SPI encoder = MT6816SPI();           // ENC_BITS-bit absolute, ENC_CPR counts/rev

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
const float CURR_MAX    = 2.0f;
const float CURR_LIMIT  = 2.0f;

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
// currentSense.init() has armed the injected-conversion group -- confirmed
// 2026-__-__ by vraw=0 from an isolated call in the print block, while the
// pre-init setup read works every time. Not a contention-between-two-calls
// issue: ONE call fails. HAL_ADC_Start returns BUSY because the peripheral is
// never in the READY state again.
// Live tracking requires a register-level REGULAR-group conversion on PA0.
// Regular and injected groups coexist by design: injected preempts, regular
// resumes. No pausing, no blind interval in the current loop.
// Deferred -- the bench has no sag to track. See README 8.3.
const bool  VBUS_LIVE     = false;
const float VBUS_TF       = 0.020f;     // 20 ms. Noise here becomes motor current.
const float VBUS_MIN      = 8.0f;       // PLAUSIBILITY window only -- NOT a
const float VBUS_MAX      = 30.0f;      //   low-voltage cutoff. See note below.
const float VBUS_FALLBACK = 11.30f;     // measured bench pack, used if read fails
float vbus_filt           = VBUS_FALLBACK;
bool  vbus_valid          = false;
// NOTE ON VBUS_MAX: the ADC saturates at 34.2 V, so a genuine overvoltage above
// that would read as exactly 34.2 and be REJECTED by this window -- the filter
// would then hold its last good value rather than reporting the fault. That is
// the correct behaviour for a divisor, but it means this window is not, and must
// not be mistaken for, overvoltage protection.

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

void logStart(uint8_t decim) {
  log_decim = decim; log_skip = 0; log_i = 0;
  log_ready = false; log_announced = false;
  log_mode = mode; log_running = running;
  log_t0 = micros(); log_t_prev = log_t0; log_active = true;
  SerialUART.print(F("CAPTURE start decim=")); SerialUART.println(decim);
}

void logDump() {
  if (!log_ready) { SerialUART.println(F("no capture in buffer")); return; }
  float dt_us = (float)(log_t1 - log_t0) / (float)(log_i > 1 ? (log_i - 1) : 1);
  SerialUART.println(F("# BURST DUMP"));
  SerialUART.print(F("# samples=")); SerialUART.print(log_i);
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
  const uint16_t N = 2000;
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
  if (errs == 0 && oks == N) SerialUART.println(F("ENC PASS: link clean"));
  else                       SerialUART.println(F("ENC FAIL: check wiring / slow SPI_HALF_NOPS down"));
  if (nmg) SerialUART.println(F("!! No_Mag_Warning -- magnet too weak or too far. Angle is GARBAGE."));
}

void printHelp() {
  SerialUART.println(F("--- g:go  x/s:stop  +/-:target | modes: o=open t=torque(V) c=torque(I) v=vel ---"));
  SerialUART.println(F("--- f:initFOC(stored)  F:force align  e:encoder self-test  q:print interval ---"));
  SerialUART.println(F("--- logger: l=fast L=slow k=kick j=zero-kick d=dump a=stats ---"));
  SerialUART.println(F("--- V:verify stored ZEA | Y:autocalib menu  1..6:phases  7:report  0:reset ---"));
  SerialUART.println(F("--- manual: N=M2 bus-power ladder  B/b=M4 breakaway ramp +/- ---"));
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

void stopMotor(const char* reason) {
  motor.disable(); running = false;
  SerialUART.print(F("STOPPED (")); SerialUART.print(reason); SerialUART.println(F(")"));
}

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

void runInitFOC(bool force_align) {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  bool use_stored = (!force_align && ZEA_STORED >= 0.0f && DIR_STORED != 0);
  if (use_stored) {
    // Absolute sensor: ZEA is a constant, not a per-session measurement.
    motor.zero_electric_angle = ZEA_STORED;
    motor.sensor_direction    = (DIR_STORED > 0) ? Direction::CW : Direction::CCW;
    SerialUART.println(F("initFOC: STORED ZEA -- no alignment, no twitch."));
  } else {
    motor.zero_electric_angle = NOT_SET;
    // Direction must be DETECTED on this board: the SPI angle convention is not
    // the TIM4 count convention. Pin it only once DIR_STORED has been measured.
    motor.sensor_direction = (DIR_STORED > 0) ? Direction::CW
                           : (DIR_STORED < 0) ? Direction::CCW
                                              : Direction::UNKNOWN;
    SerialUART.println(F("initFOC: aligning (expect a small twitch)."));
  }
  motor.enable();
  int ok = motor.initFOC();
  motor.disable();
  target = 0.0f;
  if (ok) {
    foc_ready = true;
    SerialUART.println(F("initFOC SUCCESS"));
    SerialUART.print(F("zero_electric_angle=")); SerialUART.println(motor.zero_electric_angle, 4);
    SerialUART.print(F("sensor_direction=")); SerialUART.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
  } else {
    foc_ready = false;
    SerialUART.println(F("initFOC FAILED"));
  }
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
      case 'f': runInitFOC(false); break;   // uses STORED ZEA when available
      case 'F': runInitFOC(true);  break;   // force a fresh alignment
      case 'e': case 'E': encoderSelfTest(); break;
      case 'l': logStart(1); break;                     // fast capture (~65 ms)
      case 'L': logStart(8); break;                     // slow capture (~520 ms)
      case 'd': case 'D': logDump(); break;
      case 'a': case 'A': logStats(); break;            // mean of last capture
      case 'Y': case 'y': acStatus();   break;         // status / menu
      // Manual-assist, NOT part of the 1..7 chain. N / B / b chosen because
      // 'F' is force-align and 'G' is go -- binding either would have shadowed
      // an existing command silently.
      case 'N': case 'n': acM2Assist(); break;         // M2 bus-power ladder
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
      case 'k': case 'K':                               // step + capture
        if (!running || !isTorqueMode(mode)) {
          SerialUART.println(F("k: need TORQUE(V) or TORQUE(I) running"));
        } else {
          bool volts = (mode == MODE_TORQUE);
          target = volts ? KICKV_BASE : KICK_BASE;      // pre-load OUT of the dead zone
          kick_at = millis() + 300;                     // settle, then step (in loop)
          SerialUART.print(F("kick armed: hold ")); SerialUART.print(target, 3);
          SerialUART.print(volts ? F(" V") : F(" A"));
          SerialUART.print(F(" 300ms, then step to "));
          SerialUART.println(volts ? KICKV_A : KICK_A, 3);
        }
        break;
      case 'j': case 'J':                               // zero-based step: measures dead-zone traverse
        if (!running || !isTorqueMode(mode)) {
          SerialUART.println(F("j: need TORQUE(V) or TORQUE(I) running"));
        } else {
          target = 0.0f;
          kick_at = millis() + 300;
          kick_zero = true;
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

  // ---- BUS VOLTAGE: seed BEFORE driver.init() and BEFORE currentSense.init().
  // Ordering is deliberate: currentSense.init() reconfigures the ADC, so any
  // Arduino-API analogRead() must either happen before it or be verified against
  // it afterwards (see the |I|/Iq gate in the verification procedure).
  analogReadResolution(12);                  // default is 10-bit; 2 bits for free
  pinMode(PIN_VBUS, INPUT_ANALOG);           // detach digital buffer, unload divider
  // THE SEED IS ALREADY AVERAGED, AND THAT MATTERS FOR HOW ITS SCATTER IS READ.
  // 64 samples with the first conversion discarded. So when 5 back-to-back power
  // cycles on J02 (2026-08-20) gave four boots at 12.29 V and ONE at 12.34 --
  // 5.9 counts, 0.05 V -- that outlier CANNOT be per-sample ADC noise: white
  // noise is suppressed 8x by this mean, and 5.9 counts of it would need a
  // per-sample sd of ~47 counts, which nothing here shows. The "unaveraged seed"
  // explanation offered for it is therefore WRONG, and README section 24.14 has
  // been corrected. Whatever moves it -- pack recovery between power cycles is
  // the leading candidate, since the pack is unplugged each time -- is a real
  // per-boot offset common to all 64 samples, and averaging harder cannot touch
  // it.
  // WHY IT IS NOT COSMETIC: this value IS driver.voltage_power_supply, which is
  // the divisor velocityOpenloop() uses, so it lands 1:1 on M2's g. A boot at
  // 12.34 instead of 12.19 would have biased J02's g by +1.23% -- larger than
  // J01's entire error budget, with NO symptom in the data. The mitigation is
  // procedural and it is in CALIBRATION.md's M2 row: compare the banner against
  // the meter at session start, and REBOOT if they differ by more than 0.03 V.
  {
    uint32_t acc = 0;
    (void)analogRead(PIN_VBUS);   // discard: first conversion carries residue
    for (int k = 0; k < 64; k++) acc += analogRead(PIN_VBUS);   // average 64
    float v = (acc / 64.0f) * VBUS_SCALE;
    if (VBUS_SCALE > 0.0f && v > VBUS_MIN && v < VBUS_MAX) {
      vbus_filt = v;  vbus_valid = true;
    } else {
      vbus_filt = VBUS_FALLBACK;  vbus_valid = false;
      SerialUART.println(F("!! VBUS read implausible -- using fallback"));
    }
  }
  driver.voltage_power_supply = vbus_filt;
  // driver.voltage_limit is NOT a safety limit -- it is the SVPWM MODULATION
  // REFERENCE. setPhaseVoltage() normalises Ud/Uq against it and (with the
  // library default modulation_centered = 1) centres the modulation at
  // driver.voltage_limit/2. So this one number sets BOTH the achievable phase
  // voltage, rail/sqrt(3) = 3.46 V, AND the common-mode duty centre,
  // 6.0/12.46 = 24% rather than 50%.
  //
  // WHAT RAISING IT WOULD AND WOULD NOT INVALIDATE (an earlier note here said
  // "it invalidates R_eff and U0" -- the R_eff half of that was WRONG):
  //   R_eff, Ke  IMMUNE. Ua = Ta*driver_vl while Ta ~ Uout = Uq/driver_vl, so
  //              the factor cancels: the DIFFERENTIAL phase voltage depends on
  //              commanded Uq alone. The star point floats, so only the
  //              differential drives current. Both were fit against commanded
  //              Uq, so both survive unchanged.
  //   U0         AFFECTED. It is the dead-time / body-diode intercept, and
  //              moving the duty centre 24% -> 50% changes the regime it was
  //              measured in. Re-run phase 3 (which re-checks R_eff too, and
  //              so tests the cancellation argument above rather than assuming
  //              it).
  //   LOW-SIDE   AFFECTED, and this is the one to watch. LowsideCurrentSense
  //   SENSING    samples while the low-side FETs conduct. At a 24% centre the
  //              low side is on ~76% of the time -- a comfortable window. At a
  //              50% centre with high modulation that window shrinks, which is
  //              a known failure mode on this board family. Confirm phase 1 and
  //              the phase-5 |I| ratio after any change.
  driver.voltage_limit = DRIVER_VOLT_LIMIT;
  driver.dead_zone     = DEAD_ZONE;
  // Assigned explicitly so the CFG banner prints a NUMBER. Left unset it stays
  // at NOT_SET and the banner printed -12345 -- a sentinel that reads like data.
  // The STM32 HAL substitutes exactly 25000 when unset, so this changes nothing
  // but the banner. "A library default is a decision nobody made."
  driver.pwm_frequency = (long)PWM_FREQ_HZ;
  driver_ok = driver.init();
  SerialUART.println(driver_ok ? F("driver OK") : F("driver FAILED"));

  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);

  SerialUART.println(F("initialising MT6816 SPI encoder..."));
  encoder.init();                            // bit-banged: consults no pin map
  motor.linkSensor(&encoder);
  SerialUART.print(F("MT6816 raw=")); SerialUART.print(encoder.rawCount());
  SerialUART.print(F(" / ")); SerialUART.print(ENC_CPR);
  SerialUART.print(F("  parity_err=")); SerialUART.print(encoder.spi_err);
  SerialUART.print(F(" no_mag=")); SerialUART.println(encoder.no_mag);
  if (encoder.spi_err) SerialUART.println(F("!! SPI parity errors at boot -- check CSN and HVPP->VDD"));
  if (encoder.no_mag)  SerialUART.println(F("!! No_Mag_Warning at boot -- magnet weak/far. Fix before running."));

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
  motor.init();

  cs_ok = currentSense.init();
  SerialUART.println(cs_ok ? F("currentSense OK") : F("currentSense FAILED"));
  if (cs_ok) {
    motor.linkCurrentSense(&currentSense);
    cs_linked = true;
    SerialUART.println(F("currentSense LINKED"));
  }

  motor.disable();

  // "Note which config is actually flashed." A measurement is only comparable to
  // others taken under the SAME four values. On STM32 6-PWM, dead_zone is
  // converted to a timer dead-time register value at driver.init() and quantised,
  // so this echoes what was REQUESTED -- the measured U0 intercept is what the
  // hardware actually does.
  SerialUART.print(F("CFG modulation="));
  SerialUART.print(motor.foc_modulation == FOCModulationType::SpaceVectorPWM ? F("SVPWM") : F("SinePWM"));
  SerialUART.print(F(" dead_zone=")); SerialUART.print(driver.dead_zone, 4);
  SerialUART.print(F(" pwm_Hz="));    SerialUART.print(driver.pwm_frequency);
  SerialUART.print(F(" Vbus="));      SerialUART.print(driver.voltage_power_supply, 2);
  SerialUART.print(F(" vok="));       SerialUART.print(vbus_valid ? 1 : 0);
  // The limits that actually bind, echoed because "a limit that does not bind is
  // not protection" and "a clamp in the wrong units is not a clamp". Uq_max is
  // what motor.move() constrains voltage.q to; Uq_ceil is what the modulator can
  // physically synthesise. Uq_max > Uq_ceil would be a limit that does not exist.
  SerialUART.print(F(" Uq_max="));    SerialUART.print(motor.voltage_limit, 2);
  SerialUART.print(F(" Uq_ceil="));   SerialUART.print(
      ((DRIVER_VOLT_LIMIT < driver.voltage_power_supply)
         ? DRIVER_VOLT_LIMIT : driver.voltage_power_supply) * 0.57735f, 2);
  SerialUART.print(F(" Ilim="));      SerialUART.print(motor.current_limit, 2);
  // Echo every load-bearing default: a library default is a decision nobody made.
  SerialUART.print(F(" v_align="));   SerialUART.print(motor.voltage_sensor_align, 2);
  // R_eff is 0.0f on every UNBUILT row (= NOT MEASURED, see joint_cal.h), so the
  // alignment current is genuinely unknown there. Print "?" rather than an inf.
  SerialUART.print(F(" (I_align="));
  if (CAL.R_eff > 0.0f) SerialUART.print(motor.voltage_sensor_align / CAL.R_eff, 1);
  else                  SerialUART.print(F("? R_eff NOT MEASURED"));
  SerialUART.print(F("A) spi_nops=")); SerialUART.print(SPI_HALF_NOPS);
  SerialUART.print(F(" jump_guard=")); SerialUART.println(SPI_JUMP_GUARD ? 1 : 0);

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
      target = (mode == MODE_TORQUE) ? KICKV_A
                                     : (kick_zero ? 1.0f : KICK_A);
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