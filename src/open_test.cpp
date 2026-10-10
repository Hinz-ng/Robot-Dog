#include <Arduino.h>
#include <SimpleFOC.h>
#include "fleet_config.h"   // FLEET: pole pairs, encoder, dq convention, driver config
#include "joint_cal.h"      // PER-UNIT: this assembly only, picked by -D JOINT_ID

// ============================================================================
// open_test.cpp -- BENCH HARNESS: FOC current mode + MT6816 4-wire SPI.
// Board: B-G431B-ESC1 clone (EG2124A). SimpleFOC 2.3.1, ststm32@17.6.0.
// Not shipped to the robot (Tier 0 is src/tier0/). Keys and session workflow:
// docs/FIRMWARE.md §11. Boots DISABLED; 20 s auto-stop; 150 rad/s overspeed
// cutoff; torque modes arm at 0.
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
// WIRING (XJX-135 JP2 header -> board). All four SPI pins must be on GPIOB:
// the fast path writes GPIOB->BSRR and reads GPIOB->IDR directly.
//     CSN  -> PB5     MOSI -> PB6     MISO -> PB7     SCK -> PB8 (= BOOT0)
//     HVPP -> VDD/3V3 (selects SPI; low gives ABZ)     VDD -> 3V3, GND -> GND
//   SCK sits on PB8 because CSN (internal pull-up) and MISO (an output) could
//   hold BOOT0 high at reset. Keep a 10k pulldown PB8 -> GND.
//   The telemetry UART stays on PB4/PB3: bit-banging needs no pin map.
// ============================================================================

// BUILD GUARD: [env:A1] defines ENCODER_ABZ. This source is the SPI build; on
// ABZ hardware it would flash, print a plausible banner and read garbage.
#ifdef ENCODER_ABZ
  #error "This source is the MT6816 4-wire SPI build; [env:A1] is ABZ/TIM4 hardware. \
It would boot, print a valid-looking banner and read garbage. Restore the ABZ \
sources on a branch and build there -- do not build this environment."
#endif

HardwareSerial SerialUART(PB4, PB3);

#include "mt6816.h"   // MT6816 SPI encoder

// ---------------------------------------------------------------------------


// ---- safety / tuning constants ----
// THIS SKETCH'S bench envelope. Not fleet (fleet_config.h) and not per-unit
// (joint_cal.h) -- a test harness owns these and the robot will not inherit them.
//
// Uq rails -> real current = VOLT_LIMIT / CAL.R_eff: ~9 A at 2.0 V for the ~1 s
// it takes to react. The debounced sense guard is the backstop.
const float VOLT_LIMIT      = 2.0f;
// SVPWM modulation reference for driver.voltage_limit (see actuator_hw.h):
// phase-voltage ceiling 6.0/sqrt(3) = 3.46 V. A SPEED CEILING: ~190 rad/s
// unloaded. Raise before any high-speed work (CONSTANTS §8.3).
const float DRIVER_VOLT_LIMIT = 6.0f;
const float VEL_MAX         = 20.0f;
const float OVERSPEED_RADS  = 150.0f;   // torque mode has NO built-in speed limit
const float VEL_STEP        = 1.0f;
const float TORQUE_STEP_V     = 0.01f;
// TORQUE(V) target ceiling. It only sets how far '+' winds `target`: delivered
// Uq is clamped to VOLT_LIMIT in move(), so the top 0.6 V is unreachable. To
// deliver more than 2.0 V raise VOLT_LIMIT, not this (phase 5 raises
// voltage_limit itself). current_limit does not bind in voltage mode.
const float TORQUE_MAX_V      = 2.6f;
const unsigned long AUTO_STOP_MS = 20000;
const unsigned long OVERSPEED_GRACE_MS = 300;   // ignore overspeed right after arming

const float CURR_STEP_A_rep   = 0.1f;
// TORQUE(I) TARGET CLAMP -- the limit that actually binds in TORQUE(I):
// BLDCMotor::move() torque/foc_current is a bare `current_sp = target;`, so
// CURR_LIMIT_A_rep (motor.current_limit) binds only in VELOCITY mode.
// 3.0 A: 2.9 W stalled -- fine for a short test, never a standing hold; Uq at
// stall 0.67 V. Belt force at 3 A is 21 N: watch for tooth skip.
const float CURR_MAX_A_rep    = 3.0f;
const float CURR_LIMIT_A_rep  = 2.0f;   // VELOCITY mode only -- see above. Unchanged.


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
// PER-JOINT CALIBRATION -- joint_cal.h, selected by -D JOINT_ID
// ---------------------------------------------------------------------------
// ZEA is a stored constant on the absolute encoder, so a slipped magnet or a
// wrong-joint flash is silent: press 'V' after every flash. An unfilled row
// (zea < 0, dir == 0) falls back to a full alignment.
const float ZEA_STORED = CAL.zea;
const int   DIR_STORED = CAL.dir;


// ---------------------------------------------------------------------------
// BUS VOLTAGE SENSING
// ---------------------------------------------------------------------------
// driver.voltage_power_supply is the divisor SimpleFOC uses to turn a voltage
// request into a duty cycle; a wrong value makes delivered != commanded Uq and
// breaks the Uq = R*Iq + U0 + Ke*w cross-check.
// PIN PA0 (VBUS_ADC, ADC1 rank 5). The scale is PER BOARD, from joint_cal.h (M1,
// run before AUTOCALIB). Full scale ~34.6 V, so 6S needs no PB10 / 48V_EN
// change -- and driving PB10 moves the scale ~1.9x (HARDWARE §3).
// vbus_scale = 0 in the row disables the feature (same as a fixed divisor).
const uint32_t PIN_VBUS   = PA0;
const float VBUS_SCALE    = CAL.vbus_scale;   // V per ADC count -- PER BOARD, M1
// SEED-ONLY. After currentSense.init(), analogRead() is FORBIDDEN: the current
// sense owns the ADC regular group (ADSTART = 1), and a HAL that stopped the ADC
// first would rewrite SQR1/SMPR and silently corrupt current sensing. A live
// reading would be a RAM read of the DMA buffer, but that converter is
// uncalibrated (CALFACT = 0, ~60 counts low) and cannot be calibrated before
// init (no ADC kernel clock there). The seed matches a UT89X to < 0.16% at
// 12-23 V. VBUS_LIVE stays false; promotion condition in CONSTANTS §8.3.
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
// VBUS ADC PROBE -- 'p'.  PURE READ: no peripheral register is written.
// Dumps the ADC kernel clock, both instances' sequence, offset and CALFACT
// registers, PA0/PB12/PB14 GPIO state and the live DMA buffer -- the instrument
// behind HARDWARE §3's sequence map.
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

// CFGR, SMPR and JSQR. The injected group (JSQR) is empty on this board.
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

// Index into PROBE_DMA of the channel feeding this ADC, or -1. Shared by the
// probe and vbusDmaRaw() -- one definition of "where the buffer is".
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
// no plausibility window, NOT driver.voltage_power_supply. Diagnostic only;
// kept because the live-Vbus acceptance test is Vdma vs a terminal meter
// (CONSTANTS §8.3). NAN (printed "--") when the buffer is unreachable.
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
// OFFSET REGISTERS, CALFACT, GPIO STATE -- pure reads. Measured 2026-08-21/30:
// OFR all 0, CALFACT 0 on both instances, PA0 analog with no pull. The ~60-count
// DMA offset is the uncalibrated converter (HARDWARE §3).
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
  SerialUART.println(any ? F("   <<< AN OFFSET IS ENABLED")
                         : F("   (none enabled)"));

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

// MODER 11 = analog (set before init); PUPDR must be 00 -- a pull on a divider
// node would load it.
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
  SerialUART.println(F("\n---- VBUS ADC PROBE (read-only) ----"));
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
  SerialUART.println(F("---- end probe ----\n"));
}



// MODE_MIT (B12a): FOC current control with the target computed every loop by
// the MIT law (mitService). Every guard that applies to TORQUE(I) applies to it.
enum Mode { MODE_OPENLOOP, MODE_TORQUE, MODE_TORQUE_CURRENT, MODE_VELOCITY, MODE_MIT };
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
  uint16_t dt_us;         // us since the PREVIOUS sample. Fit against this, not
                          // k * dt_mean: the sense-guard read, LED blink and
                          // handleSerial all jitter the loop. Saturates at 65.5 ms.
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
                          // Wraps at 16383 = one motor rev: UNWRAP before
                          // differentiating. 1 count = 0.02197 deg mech.
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
// Conditions the capture was taken under, recorded with it: a capture taken
// disarmed once showed stale Uq and vel and looked entirely healthy.
Mode     log_mode = MODE_OPENLOOP;
bool     log_running = false;
// Step between two NONZERO currents: stepping from 0 puts the 532 us dead-zone
// traverse in the measurement and hides the true electrical bandwidth.
const float KICK_BASE_A_rep = 0.5f;   // pre-step hold current
const float KICK_A_rep    = 1.5f;   // post-step current
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
// 'k' steps positive, 'K' negative; the sign applies to the hold AND the step.
float kick_sign = +1.0f;

// Capture serial number (printed in the dump header) and dump count, so a
// re-dump of an old buffer is recognisable offline.
uint16_t log_seq   = 0;         // increments per capture armed
uint8_t  log_dumps = 0;         // times THIS capture has been dumped

// ===========================================================================
// MIT MODE (B12a, 2026-10-01) -- the Tier-0 contract law on the bench.
// ===========================================================================
// The law, the estimator and command validation live in mit_law.h (pure, for
// Tier-0 reuse); the contract structs in mit_types.h; the torque boundary and
// clamp chain in joint_cal.h. Everything HERE is harness: limits, UI, capture.
// Entered with 'm' <Enter>; all MIT commands are typed lines -- see mitHelp().
// ===========================================================================
#include "mit_law.h"

// ---- C2: THE BENCH ENVELOPE -- harness policy. Raise only through B10.
// INNER clamp = the DEMONSTRATED envelope, D3's 1.6 A_rep (swing ladder,
// BELT_DRIVE §22.4.9). Compile-time: the clamp a wrong conversion cannot pass.
// Set any future envelope >= 6% below the demonstrated-safe current (measured
// Iq overshoots the clamped command by up to 6%, B12a a5).
const float MIT_ENV_A_rep = 1.6f;
// OUTER clamp, contract units. 0.39 sits just under the inner clamp's 0.403
// N.m (J01), so the OUTER binds first in normal use and an INNER hit is a
// finding (conversion vs envelope), not a saturation. Settable to at most
// MIT_TAU_MAX_CEIL_Nm -- a5 run 2 raises it ON PURPOSE to make the inner clamp
// bind -- and RESET TO THIS DEFAULT ON EVERY ARM, so a raised value cannot
// outlive the test that needed it.
const float MIT_TAU_MAX_Nm      = 0.39f;
const float MIT_TAU_MAX_CEIL_Nm = 1.0f;
// Command sanity ranges, output side: kp, kd, |p_des|, |v_des|, |tau_ff|.
// Generous on purpose -- they catch typos and NaN, not physics; the clamp
// chain is what bounds torque. |v_des| 5 rad/s out = 45 rad/s at the motor,
// well inside the ~81-91 rad/s Uq ceiling at VOLT_LIMIT 2.0.
const MitRanges MIT_RANGES = { 200.0f, 2.0f, 6.2832f, 5.0f, 1.0f };
// Velocity filter Tf_mit: 1.0 ms, chosen by B12a a0 (BELT_DRIVE §22.7.2).
// Bounds: lateness <= 15 deg at 25 Hz -> Tf <= 1.7 ms; noise x kd <= 7 mN.m.
const float MIT_TF_DEFAULT_S = 0.001f;
const float MIT_TF_MIN_S     = 0.00005f;
const float MIT_TF_MAX_S     = 0.050f;
// Timed switch: samples logged BEFORE command B takes effect, so every step
// capture carries its own pre-step baseline (~4 ms at decim 1).
const uint16_t MIT_SW_PRETRIG = 50;

MitEstimator mit_est   = {};
MitCmd   mit_cmd       = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // LIVE command ("A")
// STAGED CHANGES ("B"): field changes applied to the live A when `m go` fires
// (B = A + changes). Each `m b` line replaces the previous changes; `m b` alone
// clears them.
enum : uint8_t { MIT_F_PD = 1, MIT_F_VD = 2, MIT_F_KP = 4, MIT_F_KD = 8, MIT_F_FF = 16 };
MitCmd   mit_stage      = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // values of the staged fields
uint8_t  mit_stage_mask = 0;                               // which fields are staged
MitCmd   mit_cmd_b     = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // RESOLVED B, fixed at `m go`
float    mit_tau_max_Nm = MIT_TAU_MAX_Nm;
float    mit_tf_s       = MIT_TF_DEFAULT_S;
float    mit_tau_law_Nm = 0.0f;          // last law output, BEFORE any clamp
IqClamp  mit_clamp      = IQ_CLAMP_NONE; // last clamp code
uint32_t mit_n_tau = 0, mit_n_iq = 0, mit_n_rej = 0;   // loops each clamp bound since arm
bool     mit_sw_armed       = false;     // B waits for MIT_SW_PRETRIG samples
bool     mit_revert_pending = false;
// TIMING ON THE DWT CYCLE COUNTER (enabled by the core at boot, hw_config.c).
// One register read per loop instead of micros()' call chain, and cycle
// resolution for dt. 32-bit wrap at 170 MHz is 25 s; every use is an unsigned
// difference over < 6 s, so the wrap is harmless.
uint32_t mit_sw_revert_cyc  = 0;         // 0 = no auto-revert
uint32_t mit_sw_cyc         = 0;
uint32_t mit_cyc_prev       = 0;         // CYCCNT at the previous mitService()
float    mit_s_per_cyc      = 0.0f;      // 1/SystemCoreClock, set at mode entry
uint32_t mit_svc_cyc_max    = 0;         // longest mitService() since the last status line
// What a capture was taken under, printed in its dump header (C6).
MitCmd   log_mit_a = {}, log_mit_b = {};
int16_t  log_mit_sw_i = -1, log_mit_rv_i = -1;
int32_t  log_mit_cnt0 = 0;               // session counts at sample 0
int8_t   log_mit_dir  = 0;
float    log_mit_tf_s = 0.0f, log_mit_tau_max = 0.0f;

static inline int8_t mitDir() { return (int8_t)motor.sensor_direction; }
static inline int16_t sat16(float x) {
  return (x > 32767.0f) ? 32767 : (x < -32768.0f) ? -32768 : (int16_t)x;
}

// Arming in MIT mode. "Torque modes arm at zero": p is re-zeroed HERE, and
// p_des, v_des and tau_ff are cleared, so the law's output at arm is zero for a
// joint at rest even though kp/kd are KEPT (convenient for repeated steps --
// zero error times any kp is zero). tau_max goes back to its default.
// The STAGED changes are KEPT: they are only ever applied by an explicit `m go`,
// and a staged p_des is relative to the new zero. Clearing them (as the first
// build did) forced every B to be typed AFTER `g`, inside the 20 s auto-stop.
void mitOnArm() {
  mitEstZero(mit_est);
  mit_cmd.p_des_rad = 0.0f; mit_cmd.v_des_rads = 0.0f; mit_cmd.tau_ff_Nm = 0.0f;
  mit_tau_max_Nm = MIT_TAU_MAX_Nm;
  mit_n_tau = mit_n_iq = mit_n_rej = 0;
  mit_sw_armed = false; mit_revert_pending = false;
  mit_clamp = IQ_CLAMP_NONE; mit_tau_law_Nm = 0.0f;
}

// Once per loop in MIT mode, after loopFOC() and BEFORE motor.move(target).
// Disarmed it only tracks p and v, so the filter is converged at arm. Its cost
// is measured: svc_us= is the MAX per status window (includes interrupt
// preemption); the mean cost is the lps difference vs TORQUE(I) (§22.7.7).
static void mitServiceBody(uint32_t c0) {
  const int8_t dir = mitDir();
  const float  dt  = (float)(uint32_t)(c0 - mit_cyc_prev) * mit_s_per_cyc;
  mit_cyc_prev = c0;
  mitEstUpdate(mit_est, encoder.raw, dir, dt, mit_tf_s);
  if (!running) return;
  // Timed switch (C5): B takes effect once the pre-trigger is in the buffer.
  if (mit_sw_armed && log_active && log_i >= MIT_SW_PRETRIG) {
    mit_cmd = mit_cmd_b; mit_sw_armed = false;
    log_mit_sw_i = (int16_t)log_i; mit_sw_cyc = c0;
    mit_revert_pending = (mit_sw_revert_cyc > 0);
  }
  // Auto-revert to a ZERO-TORQUE command: kp, kd, v_des, tau_ff -> 0.
  if (mit_revert_pending && (uint32_t)(c0 - mit_sw_cyc) >= mit_sw_revert_cyc) {
    mit_cmd.kp_Nm_per_rad = 0.0f; mit_cmd.kd_Nms_per_rad = 0.0f;
    mit_cmd.v_des_rads = 0.0f;    mit_cmd.tau_ff_Nm = 0.0f;
    mit_revert_pending = false;
    if (log_active) log_mit_rv_i = (int16_t)log_i;
  }
  // tau is NOT computed here: the law does not use it, and removing it (with the
  // micros() -> DWT change) is what took the MIT deficit from -8.2% to -4.7%
  // (2026-10-02). The status line and the dump compute the measured
  // tau themselves (irepToTorqueOut(Iq)); Tier 0 computes it per CAN frame.
  const MitState s = { mitEstP(mit_est, dir), mit_est.v_out_rads, 0.0f };
  mit_tau_law_Nm = mitLawTau(mit_cmd, s);
  const IqCmd r = tauOutCmdToIq(mit_tau_law_Nm, mit_tau_max_Nm, MIT_ENV_A_rep);
  mit_clamp = r.clamp;
  if      (r.clamp == IQ_CLAMP_TAU)    mit_n_tau++;
  else if (r.clamp == IQ_CLAMP_IQ)     mit_n_iq++;
  else if (r.clamp == IQ_CLAMP_REJECT) mit_n_rej++;
  target = r.iq_A_rep;
}

void mitService() {
  const uint32_t c0 = DWT->CYCCNT;
  mitServiceBody(c0);
  const uint32_t dc = DWT->CYCCNT - c0;
  if (dc > mit_svc_cyc_max) mit_svc_cyc_max = dc;
}

void logStart(uint8_t decim) {
  log_decim = decim; log_skip = 0; log_i = 0;
  log_ready = false; log_announced = false;
  log_mode = mode; log_running = running;
  if (mode == MODE_MIT) {                // the command a capture was taken under
    log_mit_a = mit_cmd;
    log_mit_b = mit_sw_armed ? mit_cmd_b : mit_cmd;
    log_mit_sw_i = -1; log_mit_rv_i = -1;
    log_mit_dir = mitDir(); log_mit_tf_s = mit_tf_s; log_mit_tau_max = mit_tau_max_Nm;
  }
  log_seq++; log_dumps = 0;
  log_t0 = micros(); log_t_prev = log_t0; log_active = true;
  SerialUART.print(F("CAPTURE start decim=")); SerialUART.print(decim);
  SerialUART.print(F(" cap=")); SerialUART.println(log_seq);
}

const char* modeTag(Mode m) {
  switch (m) {
    case MODE_OPENLOOP:       return "OL";
    case MODE_TORQUE:         return "TV";
    case MODE_TORQUE_CURRENT: return "TI";
    case MODE_VELOCITY:       return "VEL";
    case MODE_MIT:            return "MIT";
  }
  return "?";
}

static void printMitCmd(const MitCmd& c) {
  SerialUART.print(F("pd="));  SerialUART.print(c.p_des_rad, 5);
  SerialUART.print(F(" vd=")); SerialUART.print(c.v_des_rads, 4);
  SerialUART.print(F(" kp=")); SerialUART.print(c.kp_Nm_per_rad, 3);
  SerialUART.print(F(" kd=")); SerialUART.print(c.kd_Nms_per_rad, 4);
  SerialUART.print(F(" ff=")); SerialUART.print(c.tau_ff_Nm, 4);
}

// A with the staged fields overwritten -- what B will be if `m go` fires now.
static MitCmd mitApply(MitCmd a, const MitCmd& s, uint8_t m) {
  if (m & MIT_F_PD) a.p_des_rad      = s.p_des_rad;
  if (m & MIT_F_VD) a.v_des_rads     = s.v_des_rads;
  if (m & MIT_F_KP) a.kp_Nm_per_rad  = s.kp_Nm_per_rad;
  if (m & MIT_F_KD) a.kd_Nms_per_rad = s.kd_Nms_per_rad;
  if (m & MIT_F_FF) a.tau_ff_Nm      = s.tau_ff_Nm;
  return a;
}

static void printMitStage() {
  if (!mit_stage_mask) { SerialUART.print(F("A (no changes)")); return; }
  SerialUART.print(F("A +"));
  if (mit_stage_mask & MIT_F_PD) { SerialUART.print(F(" pd=")); SerialUART.print(mit_stage.p_des_rad, 5); }
  if (mit_stage_mask & MIT_F_VD) { SerialUART.print(F(" vd=")); SerialUART.print(mit_stage.v_des_rads, 4); }
  if (mit_stage_mask & MIT_F_KP) { SerialUART.print(F(" kp=")); SerialUART.print(mit_stage.kp_Nm_per_rad, 3); }
  if (mit_stage_mask & MIT_F_KD) { SerialUART.print(F(" kd=")); SerialUART.print(mit_stage.kd_Nms_per_rad, 4); }
  if (mit_stage_mask & MIT_F_FF) { SerialUART.print(F(" ff=")); SerialUART.print(mit_stage.tau_ff_Nm, 4); }
}

// A target the law multiplies by a zero gain does nothing, silently. a3 part 2
// first ran vd 2 with kd 0 and produced exactly zero torque (2026-10-02).
static void mitWarnInert(const MitCmd& c) {
  if (c.v_des_rads != 0.0f && c.kd_Nms_per_rad == 0.0f)
    SerialUART.println(F("  !! vd does nothing while kd = 0  (tau = kd*(vd - v))"));
  if (c.p_des_rad != 0.0f && c.kp_Nm_per_rad == 0.0f)
    SerialUART.println(F("  !! pd does nothing while kp = 0  (tau = kp*(pd - p))"));
}

// MIT capture body (C6). The LogSample struct is NOT enlarged (the buffer is
// already 18 KB, 9 x 2 B x 1000). In MIT captures three
// slots carry MIT data:
//   vel_x50   -> v_mit x 1000   (OUTPUT rad/s, the law's own filtered speed)
//   ud_x1000  -> tau_law x 1e4  (N.m, BEFORE clamps; saturates at +-3.2767)
//   cnt b14-15 -> clamp code    (0 none, 1 outer, 2 inner, 3 reject)
// Everything else is reconstructed HERE, so it costs no RAM per sample:
//   p        from cnt, unwrapped, anchored to the session count at sample 0
//   p_des    A before the switch sample, B from it
//   tau_cmd  irepToTorqueOut(sp)  -- the torque actually commanded (post-clamp)
//   tau      irepToTorqueOut(Iq)  -- MEASURED, the feedback half of the boundary
// For a3's tau-vs-v check, plot `tau` (measured) against a v DIFFERENTIATED
// OFFLINE from cnt -- tau_law vs v is true by construction and proves nothing.
static void mitDumpBody() {
  SerialUART.print(F("# MIT A: ")); printMitCmd(log_mit_a);
  SerialUART.print(F("  |  B: "));   printMitCmd(log_mit_b); SerialUART.println();
  SerialUART.print(F("# MIT sw_i=")); SerialUART.print(log_mit_sw_i);
  SerialUART.print(F(" revert_i=")); SerialUART.print(log_mit_rv_i);
  SerialUART.print(F(" Tf_ms="));    SerialUART.print(log_mit_tf_s * 1000.0f, 3);
  SerialUART.print(F(" tau_max="));  SerialUART.print(log_mit_tau_max, 3);
  SerialUART.print(F(" env_A_rep=")); SerialUART.print(MIT_ENV_A_rep, 2);
  SerialUART.print(F(" Nm_per_A_rep=")); SerialUART.print(irepToTorqueOut(1.0f), 5);
  SerialUART.print(F(" dir="));      SerialUART.println(log_mit_dir);
  SerialUART.println(F("i,t_us,p,p_des,v,tau_law,tau_cmd,tau,sp,Iq,Id,Uq,rawI,cnt,clamp"));
  uint32_t t_acc = 0;
  int32_t  c_acc = log_mit_cnt0;
  uint16_t raw_prev = logbuf[0].cnt & 0x3FFF;
  for (uint16_t k = 0; k < log_i; k++) {
    const uint16_t raw = logbuf[k].cnt & 0x3FFF;
    int32_t d = (int32_t)raw - (int32_t)raw_prev;
    if      (d >  (int32_t)(ENC_CPR / 2)) d -= (int32_t)ENC_CPR;
    else if (d < -(int32_t)(ENC_CPR / 2)) d += (int32_t)ENC_CPR;
    c_acc += d; raw_prev = raw;
    t_acc += logbuf[k].dt_us;
    const float p  = (float)(log_mit_dir * c_acc) * ENC_RAD_PER_COUNT / GEAR_RATIO;
    const float pd = (log_mit_sw_i >= 0 && k >= (uint16_t)log_mit_sw_i)
                     ? log_mit_b.p_des_rad : log_mit_a.p_des_rad;
    const float sp = logbuf[k].sp_x1000 / 1000.0f, iq = logbuf[k].iq_x1000 / 1000.0f;
    SerialUART.print(k);                                  SerialUART.print(',');
    SerialUART.print(t_acc);                              SerialUART.print(',');
    SerialUART.print(p, 6);                               SerialUART.print(',');
    SerialUART.print(pd, 6);                              SerialUART.print(',');
    SerialUART.print(logbuf[k].vel_x50 / 1000.0f, 3);     SerialUART.print(',');
    SerialUART.print(logbuf[k].ud_x1000 / 10000.0f, 4);   SerialUART.print(',');
    SerialUART.print(irepToTorqueOut(sp), 4);             SerialUART.print(',');
    SerialUART.print(irepToTorqueOut(iq), 4);             SerialUART.print(',');
    SerialUART.print(sp, 3);                              SerialUART.print(',');
    SerialUART.print(iq, 3);                              SerialUART.print(',');
    SerialUART.print(logbuf[k].id_x1000 / 1000.0f, 3);    SerialUART.print(',');
    SerialUART.print(logbuf[k].uq_x1000 / 1000.0f, 3);    SerialUART.print(',');
    SerialUART.print(logbuf[k].raw_x100 / 100.0f, 2);     SerialUART.print(',');
    SerialUART.print(raw);                                SerialUART.print(',');
    SerialUART.println(logbuf[k].cnt >> 14);
  }
  SerialUART.println(F("# END"));
}

void logDump() {
  if (!log_ready) { SerialUART.println(F("no capture in buffer")); return; }
  float dt_us = (float)(log_t1 - log_t0) / (float)(log_i > 1 ? (log_i - 1) : 1);
  log_dumps++;
  if (log_dumps > 1) {
    SerialUART.print(F("!! RE-DUMP of capture ")); SerialUART.print(log_seq);
    SerialUART.print(F(" (dump #")); SerialUART.print(log_dumps);
    SerialUART.println(F(") -- no new capture since the last dump."));
  }
  SerialUART.println(F("# BURST DUMP"));
  SerialUART.print(F("# cap=")); SerialUART.print(log_seq);
  SerialUART.print(F(" dump=")); SerialUART.print(log_dumps);
  SerialUART.print(F(" samples=")); SerialUART.print(log_i);
  SerialUART.print(F(" decim=")); SerialUART.print(log_decim);
  SerialUART.print(F(" dt_us=")); SerialUART.print(dt_us, 2);
  SerialUART.print(F(" fs_Hz=")); SerialUART.println(1e6f / dt_us, 1);
  SerialUART.print(F("# mode=")); SerialUART.print(modeTag(log_mode));
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
  if (log_mode == MODE_MIT) { mitDumpBody(); return; }
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
  // MIT captures reuse two slots (see mitDumpBody): vel holds v_out x1000 and
  // Ud holds tau_law x1e4. Scale them back here so 'a' never mis-reads them.
  const bool  mit    = (log_mode == MODE_MIT);
  const float vel_sc = mit ? 1000.0f  : 50.0f;
  const float ud_sc  = mit ? 10000.0f : 1000.0f;
  float sIq=0, sId=0, sUq=0, sUd=0, sVel=0, sRaw=0, iq_lo=1e6f, iq_hi=-1e6f;
  for (uint16_t k = k0; k < n; k++) {
    float iq = logbuf[k].iq_x1000 / 1000.0f;
    sIq  += iq;
    sId  += logbuf[k].id_x1000 / 1000.0f;
    sUq  += logbuf[k].uq_x1000 / 1000.0f;
    sUd  += logbuf[k].ud_x1000 / ud_sc;
    sVel += logbuf[k].vel_x50 / vel_sc;
    sRaw += logbuf[k].raw_x100 / 100.0f;
    if (iq < iq_lo) iq_lo = iq;
    if (iq > iq_hi) iq_hi = iq;
  }
  float mIq = sIq / m, mRaw = sRaw / m;
  SerialUART.print(F("STATS n="));  SerialUART.print(m);
  SerialUART.print(F(" m="));       SerialUART.print(modeTag(log_mode));
  SerialUART.print(F(" run="));     SerialUART.print(log_running ? 1 : 0);
  SerialUART.print(F(" dz="));      SerialUART.print(driver.dead_zone, 4);
  SerialUART.print(F(" Uq="));      SerialUART.print(sUq / m, 4);
  SerialUART.print(mit ? F(" tau_law=") : F(" Ud="));  SerialUART.print(sUd / m, 4);
  SerialUART.print(F(" Iq="));      SerialUART.print(mIq, 4);
  SerialUART.print(F(" Id="));      SerialUART.print(sId / m, 4);
  SerialUART.print(mit ? F(" v_out=") : F(" vel="));   SerialUART.print(sVel / m, 3);
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
    case MODE_MIT:            return "MIT";
  }
  return "?";
}

bool needsFOC(Mode m)   { return (m == MODE_TORQUE || m == MODE_TORQUE_CURRENT || m == MODE_VELOCITY || m == MODE_MIT); }
// MIT is deliberately NOT a "torque mode" here: isTorqueMode() gates the k/j
// kicks, which write `target` -- and in MIT the law overwrites target every loop.
bool isTorqueMode(Mode m){ return (m == MODE_TORQUE || m == MODE_TORQUE_CURRENT); }
// Modes that run FOC CURRENT control and therefore need the sense chain and
// the sense-mismatch guard. One predicate, so a new current mode cannot be
// added to one of those sites and forgotten at the other.
bool isCurrentMode(Mode m){ return (m == MODE_TORQUE_CURRENT || m == MODE_MIT); }

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
  // 20000 reads (~133 ms): the link fault seen on the bench was bursty, and a
  // short probe usually landed in the quiet. Still a SNAPSHOT -- for a bursty
  // fault use 'E' and let it soak.
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
  // On a STATIONARY shaft raw must not move: a nonzero span is a corrupted frame
  // that PASSED parity.
  const uint16_t span = (hi > lo) ? (uint16_t)(hi - lo) : 0;
  // ALL-ZERO FRAMES: a dead MISO passed every check above (J03, 2026-10-08).
  // zero_run counts consecutive 0x0000 frames (mt6816.h); >= N = every read.
  const bool all_zero = (encoder.zero_run >= N);
  if (errs == 0 && oks == N && span == 0 && !all_zero) {
    SerialUART.println(F("ENC PASS (snapshot; use E to soak a bursty fault)"));
  } else {
    SerialUART.println(F("ENC FAIL: check wiring / slow SPI_HALF_NOPS down"));
    if (span) SerialUART.println(F("  span != 0 on a still shaft = corruption that PASSED parity"));
    if (all_zero) SerialUART.println(F("  every frame 0x0000: MISO dead (or shaft at count 0 -- turn it, re-run)"));
  }
  if (nmg) SerialUART.println(F("!! No_Mag_Warning -- magnet too weak or too far. Angle is GARBAGE."));
}

// ---------------------------------------------------------------------------
// 'E' -- CONTINUOUS HARNESS MONITOR. The instrument for the wiggle test.
// ---------------------------------------------------------------------------
// 'e' is a one-shot snapshot. This repeats until a key is pressed, so both
// hands are free to flex a conductor while watching the console.
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
  SerialUART.println(F("ENC MONITOR -- shaft still. Flex one conductor at a time (connector, then breakout):"));
  SerialUART.println(F("  PB5 CSN, PB6 MOSI, PB7 MISO, PB8 SCK, VCC, GND. Any key stops."));
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
  SerialUART.println(F("--- m: MIT line commands. m<Enter> = MIT mode / status, m ? = help ---"));
}

void startMotor() {
  if (!driver_ok) { SerialUART.println(F("refused: driver init failed")); return; }
  if (needsFOC(mode) && !foc_ready) { SerialUART.println(F("refused: run initFOC (f) first")); return; }
  if (isCurrentMode(mode) && !cs_linked) { SerialUART.println(F("refused: current sense not linked")); return; }
  if (mode == MODE_MIT) {
    target = 0.0f;
    mitOnArm();
    SerialUART.println(F("MIT: p zeroed here; A pd/vd/ff cleared, kp/kd kept, tau_max reset"));
    SerialUART.print(F("MIT: A: ")); printMitCmd(mit_cmd);
    SerialUART.print(F("  | m go -> B = ")); printMitStage(); SerialUART.println();
  }
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

#include "safety.h"   // stopMotor() -- the single disable path

void setMode(Mode m) {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  if (needsFOC(m) && !foc_ready) { SerialUART.println(F("run initFOC (f) first")); return; }
  if (isCurrentMode(m) && !cs_linked) { SerialUART.println(F("current sense not linked")); return; }
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
      motor.PID_velocity.limit = CURR_LIMIT_A_rep;
      target = 2.0f; break;
    case MODE_MIT:
      // Same motor configuration as TORQUE(I); the difference is who writes
      // target. Prime the estimator so p/v are tracked before the first arm.
      motor.torque_controller = TorqueControlType::foc_current;
      motor.controller = MotionControlType::torque; target = 0.0f;
      mitEstPrime(mit_est, encoder.raw);
      mit_s_per_cyc = 1.0f / (float)SystemCoreClock;
      mit_cyc_prev  = DWT->CYCCNT;
      break;
  }
  SerialUART.print(F("mode=")); SerialUART.print(modeName());
  SerialUART.print(F(" target=")); SerialUART.println(target);
}


void adjustTarget(float dir) {
  if (mode == MODE_MIT) {
    SerialUART.println(F("MIT: target is computed by the law -- use m commands (m ?)"));
    return;
  }
  if (mode == MODE_TORQUE)              target = constrain(target + dir*TORQUE_STEP_V, -TORQUE_MAX_V, TORQUE_MAX_V);
  else if (mode == MODE_TORQUE_CURRENT) target = constrain(target + dir*CURR_STEP_A_rep,  -CURR_MAX_A_rep,  CURR_MAX_A_rep);
  else                                  target = constrain(target + dir*VEL_STEP,   -VEL_MAX,   VEL_MAX);
  SerialUART.print(F("target=")); SerialUART.println(target);
}

#include "autocalib.h"          // immediately above void handleSerial()

// Chord tracking for '-' then '5' (swapped-order phase 5, see ac_p5_swap_next
// in autocalib.h). Deliberately narrow: only a '5' arriving within
// AC_CHORD_WINDOW_MS of a '-'/'_' arms the swap, so a '-' typed minutes
// earlier for ordinary target jogging can never silently swap a later,
// unrelated phase-5 run.
static char     ac_last_key    = 0;
static uint32_t ac_last_key_ms = 0;
static const uint32_t AC_CHORD_WINDOW_MS = 800;

// ===========================================================================
// MIT LINE COMMANDS (C5, C8). 'm' or 'M' opens a line; <Enter> runs it.
// ===========================================================================
// WHY A LINE AND NOT KEYS: the MIT command needs numbers, and every printable
// key in this harness is already a command. Typed as single keys, "m kp 41"
// would run AUTOCALIB PHASE 4 on the '4', and "pd -0.005" would arm the '-'/'5'
// phase-5 swap chord. So inside a line NOTHING is a command: handleSerial()
// hands every character here before the chord tracker or the key switch sees
// it. Both cases of 'm' open the line, so a lowercase slip cannot fall through.
// ESC cancels; a line idle for MIT_LINE_TIMEOUT_MS is abandoned, loudly.
//
// LINE ENDINGS. Some monitors send none (VS Code Serial Monitor "None", or a
// paste with no Enter). A line ends on ANY of:
//   CR, LF, or ';'                  explicit -- works in every terminal
//   a whole-line BURST, then idle   the "send line, no ending" monitors: every
//                                   char of the line arrived within
//                                   MIT_LINE_BURST_MS of the 'm' (a 921600-baud
//                                   burst is < 1 ms; USB latency timers can split
//                                   it by ~16 ms), at least 2 non-space chars,
//                                   then MIT_LINE_IDLE_MS of silence.
// Why the burst condition and not plain idle: a HUMAN typing in a char-by-char
// terminal pauses mid-number. Plain idle would run "m kp 4" as kp = 4 -- and
// worse, run "m" alone and hand the rest of the line ("kp 41") to the KEY
// switch, where '4' is AUTOCALIB phase 4. No human types two keys after 'm'
// inside 25 ms, so a typed line still needs Enter (or ';').
// A line idle 2 s without an ending prints a hint saying so, instead of
// silence. The periodic status line is held while a line is open so it cannot
// print through the echo.
static char     mit_line[64];
static uint8_t  mit_line_n  = 0;
static bool     mit_line_on = false;
static uint32_t mit_line_ms = 0;        // last character
static uint32_t mit_line_t0 = 0;        // the opening 'm'
static bool     mit_line_hinted = false;
static const uint32_t MIT_LINE_TIMEOUT_MS = 15000;
static const uint32_t MIT_LINE_BURST_MS   = 25;
static const uint32_t MIT_LINE_IDLE_MS    = 250;
static const uint32_t MIT_LINE_HINT_MS    = 2000;

static void mitHelp() {
  SerialUART.println(F("--- MIT. Output side, SI: rad, rad/s, N.m, N.m/rad, N.m.s/rad ---"));
  SerialUART.println(F("  m                 enter MIT mode (stopped) / print MIT status"));
  SerialUART.println(F("  m kp 41 kd 0.157  set LIVE fields: pd vd kp kd ff, tf <ms>, tmax <N.m>"));
  SerialUART.println(F("  end a line with Enter or ;  (a monitor with no line ending: use ;)"));
  SerialUART.println(F("  m b vd 2          STAGE changes: at m go, B = A + these fields."));
  SerialUART.println(F("                    each m b line replaces the last; m b alone = no changes"));
  SerialUART.println(F("  m go [rev_ms] [decim]   capture; B takes effect after 50 samples;"));
  SerialUART.println(F("                    rev_ms > 0 then reverts to ZERO torque (kp kd vd ff = 0)"));
  SerialUART.println(F("  m zero            LIVE -> zero torque now (pd = current p)"));
  SerialUART.println(F("  m test            clamp-chain + validation self-test, motor DISARMED"));
  SerialUART.println(F("  g arms: p = 0 here, A pd/vd/ff = 0, kp/kd + staged kept, tmax = 0.39. x stops."));
  SerialUART.println(F("  e.g. driven speed:  m kd 0.1;  m b vd 2;  g  m go 0 8;  x  d"));
}

static void mitPrintStatus() {
  SerialUART.print(F("MIT live  A: ")); printMitCmd(mit_cmd);   SerialUART.println();
  SerialUART.print(F("MIT staged B: ")); printMitStage();
  SerialUART.print(F("  -> if m go now: ")); printMitCmd(mitApply(mit_cmd, mit_stage, mit_stage_mask));
  SerialUART.println();
  SerialUART.print(F("MIT Tf_ms="));   SerialUART.print(mit_tf_s * 1000.0f, 3);
  SerialUART.print(F(" tau_max="));    SerialUART.print(mit_tau_max_Nm, 3);
  SerialUART.print(F(" env="));        SerialUART.print(MIT_ENV_A_rep, 2);
  SerialUART.print(F(" A_rep = "));    SerialUART.print(irepToTorqueOut(MIT_ENV_A_rep), 3);
  SerialUART.print(F(" N.m  p="));     SerialUART.print(mitEstP(mit_est, mitDir()), 5);
  SerialUART.print(F(" v="));          SerialUART.print(mit_est.v_out_rads, 3);
  SerialUART.print(F(" cl o/i/r="));   SerialUART.print(mit_n_tau); SerialUART.print('/');
  SerialUART.print(mit_n_iq); SerialUART.print('/'); SerialUART.print(mit_n_rej);
  SerialUART.print(F(" run="));        SerialUART.println(running ? 1 : 0);
}

// C8 -- B12a step a1. Feeds a fixed table through the EXACT compiled clamp
// chain and validator, with the motor DISARMED, and checks every row against
// properties that do not depend on the conversion constant (clamp code, sign,
// bounds, round trip). The magnitudes are printed so they can also be checked
// by hand -- J01 expects 0.10 N.m -> 0.397 A_rep and 0.39 N.m -> 1.549 A_rep.
// On an unbuilt row (e.g. -e J04, Ke = 0) EVERY torque case must REJECT.
static void mitClampTest() {
  if (running) { SerialUART.println(F("m test: stop first (x) -- this runs disarmed")); return; }
  const bool  uncal = !(calKtCmd() > 0.0f);
  const float T = MIT_TAU_MAX_Nm, E = MIT_ENV_A_rep;
  struct Row { const char* name; float tau, tmax, env; IqClamp want; };
  const Row rows[] = {
    { "normal +0.10",        0.10f,     T,    E,    IQ_CLAMP_NONE   },
    { "zero",                0.0f,      T,    E,    IQ_CLAMP_NONE   },
    { "over outer +1.0",     1.0f,      T,    E,    IQ_CLAMP_TAU    },
    { "over outer -1.0",    -1.0f,      T,    E,    IQ_CLAMP_TAU    },
    { "outer 5.0, +1.0",     1.0f,      5.0f, E,    IQ_CLAMP_IQ     },
    { "outer 5.0, -1.0",    -1.0f,      5.0f, E,    IQ_CLAMP_IQ     },
    { "tau NaN",             NAN,       T,    E,    IQ_CLAMP_REJECT },
    { "tau +inf",            INFINITY,  T,    E,    IQ_CLAMP_REJECT },
    { "tau -inf",           -INFINITY,  T,    E,    IQ_CLAMP_REJECT },
    { "tau_max 0",           0.10f,     0.0f, E,    IQ_CLAMP_REJECT },
    { "tau_max -0.39",       0.10f,    -T,    E,    IQ_CLAMP_REJECT },
    { "tau_max NaN",         0.10f,     NAN,  E,    IQ_CLAMP_REJECT },
    { "env 0",               0.10f,     T,    0.0f, IQ_CLAMP_REJECT },
    { "env NaN",             0.10f,     T,    NAN,  IQ_CLAMP_REJECT },
  };
  const uint8_t N = sizeof(rows) / sizeof(rows[0]);
  uint8_t pass = 0;
  SerialUART.print(F("MIT CLAMP TEST  row=")); SerialUART.print(CAL.id);
  SerialUART.print(F("  Nm_per_A_rep="));       SerialUART.print(irepToTorqueOut(1.0f), 5);
  if (uncal) SerialUART.print(F("  UNCALIBRATED ROW: every case must REJECT"));
  SerialUART.println();
  for (uint8_t k = 0; k < N; k++) {
    const Row& w = rows[k];
    const IqClamp want = uncal ? IQ_CLAMP_REJECT : w.want;
    const IqCmd r = tauOutCmdToIq(w.tau, w.tmax, w.env);
    bool ok = (r.clamp == want) && isfinite(r.iq_A_rep);
    if (ok && want == IQ_CLAMP_REJECT) ok = (r.iq_A_rep == 0.0f);
    if (ok && want != IQ_CLAMP_REJECT) {
      ok = fabsf(r.iq_A_rep) <= w.env * (1.0f + 1e-6f);
      if (w.tau != 0.0f) ok = ok && ((r.iq_A_rep > 0.0f) == (w.tau > 0.0f));
      const float tau_back = irepToTorqueOut(r.iq_A_rep);
      if (want == IQ_CLAMP_NONE) ok = ok && fabsf(tau_back - w.tau) < 1e-5f;
      if (want == IQ_CLAMP_TAU)  ok = ok && fabsf(fabsf(tau_back) - w.tmax) < 1e-5f;
      if (want == IQ_CLAMP_IQ)   ok = ok && fabsf(fabsf(r.iq_A_rep) - w.env) < 1e-6f;
    }
    pass += ok;
    SerialUART.print(ok ? F("  PASS  ") : F("  FAIL  "));
    SerialUART.print(w.name);
    SerialUART.print(F("  -> iq="));  SerialUART.print(r.iq_A_rep, 4);
    SerialUART.print(F(" A_rep clamp=")); SerialUART.print((int)r.clamp);
    SerialUART.print(F(" (want "));   SerialUART.print((int)want); SerialUART.println(')');
  }
  // Validator: reject, never repair (a6).
  struct VRow { const char* name; MitCmd c; bool want; };
  const VRow vrows[] = {
    { "valid step cmd",   { 0.0057f, 0.0f, 41.0f, 0.157f, 0.0f }, true  },
    { "kp negative",      { 0.0f,    0.0f, -1.0f, 0.1f,   0.0f }, false },
    { "kd negative",      { 0.0f,    0.0f, 10.0f, -0.1f,  0.0f }, false },
    { "kd NaN",           { 0.0f,    0.0f, 10.0f, NAN,    0.0f }, false },
    { "kp 1e6",           { 0.0f,    0.0f, 1e6f,  0.1f,   0.0f }, false },
    { "p_des 10 rad",     { 10.0f,   0.0f, 10.0f, 0.1f,   0.0f }, false },
    { "v_des inf",        { 0.0f, INFINITY, 0.0f, 0.1f,   0.0f }, false },
    { "tau_ff 2 N.m",     { 0.0f,    0.0f, 0.0f,  0.0f,   2.0f }, false },
  };
  const uint8_t NV = sizeof(vrows) / sizeof(vrows[0]);
  for (uint8_t k = 0; k < NV; k++) {
    const bool got = mitCmdValid(vrows[k].c, MIT_RANGES);
    const bool ok  = (got == vrows[k].want);
    pass += ok;
    SerialUART.print(ok ? F("  PASS  ") : F("  FAIL  "));
    SerialUART.print(vrows[k].name);
    SerialUART.print(got ? F("  -> accepted") : F("  -> rejected"));
    SerialUART.println(vrows[k].want ? F(" (want accepted)") : F(" (want rejected)"));
  }
  SerialUART.print(pass == N + NV ? F("MIT CLAMP TEST PASS ") : F("MIT CLAMP TEST *** FAIL *** "));
  SerialUART.print(pass); SerialUART.print('/'); SerialUART.println(N + NV);
}

// Number parser for the MIT line: [+-]digits[.digits][e[+-]digits], nothing
// else -- no "nan", "inf" or hex, which is what a command line wants anyway.
// Deliberately NOT strtof(): newlib's strtof links the whole strtod/gdtoa
// machinery (~10 KB), and that alone overflowed J01's flash on the first build.
static bool mitParseF(const char* s, float& out) {
  const char* p = s;
  bool neg = false;
  if (*p == '+' || *p == '-') neg = (*p++ == '-');
  uint32_t mant = 0; int16_t exp10 = 0; uint8_t nd = 0;
  for (; *p >= '0' && *p <= '9'; p++, nd++) {
    if (mant < 100000000UL) mant = mant * 10 + (uint32_t)(*p - '0'); else exp10++;
  }
  if (*p == '.') {
    for (p++; *p >= '0' && *p <= '9'; p++, nd++)
      if (mant < 100000000UL) { mant = mant * 10 + (uint32_t)(*p - '0'); exp10--; }
  }
  if (nd == 0) return false;
  if (*p == 'e' || *p == 'E') {
    p++;
    bool eneg = false;
    if (*p == '+' || *p == '-') eneg = (*p++ == '-');
    int16_t e = 0; uint8_t ne = 0;
    for (; *p >= '0' && *p <= '9'; p++, ne++) if (e < 100) e = e * 10 + (*p - '0');
    if (ne == 0) return false;
    exp10 += eneg ? -e : e;
  }
  if (*p != '\0') return false;
  float v = (float)mant;
  for (; exp10 > 0; exp10--) v *= 10.0f;
  for (; exp10 < 0; exp10++) v /= 10.0f;
  if (neg) v = -v;
  if (!isfinite(v)) return false;
  out = v; return true;
}

static void mitGo(char** tok, uint8_t nt) {
  if (!running) { SerialUART.println(F("m go: arm first (g)")); return; }
  if (mit_sw_armed) { SerialUART.println(F("m go: a switch is already pending")); return; }
  float rev_ms = 0.0f, dec = 1.0f;
  if ((nt > 1 && !mitParseF(tok[1], rev_ms)) || (nt > 2 && !mitParseF(tok[2], dec))
      || rev_ms < 0.0f || rev_ms > 5000.0f || dec < 1.0f || dec > 32.0f) {
    SerialUART.println(F("m go: usage  m go [revert_ms 0..5000] [decim 1..32]")); return;
  }
  // Resolve B against A as it is NOW, so the order of `m kd` and `m b` does not
  // matter. Re-validated: A may have changed since `m b` checked it.
  mit_cmd_b = mitApply(mit_cmd, mit_stage, mit_stage_mask);
  if (!mitCmdValid(mit_cmd_b, MIT_RANGES)) {
    SerialUART.println(F("m go: REJECTED -- A + staged changes is out of range (m ?)")); return;
  }
  if (memcmp(&mit_cmd, &mit_cmd_b, sizeof(MitCmd)) == 0)
    SerialUART.println(F("m go: note -- B equals A, this capture has no step"));
  mitWarnInert(mit_cmd_b);
  mit_sw_revert_cyc = (uint32_t)(rev_ms * (float)(SystemCoreClock / 1000U));
  mit_sw_armed = true;                 // BEFORE logStart, so it records B
  logStart((uint8_t)dec);
  SerialUART.print(F("MIT switch after ")); SerialUART.print(MIT_SW_PRETRIG);
  SerialUART.print(F(" samples to B: "));   printMitCmd(mit_cmd_b);
  if (rev_ms > 0.0f) { SerialUART.print(F("  revert after ")); SerialUART.print(rev_ms, 0); SerialUART.print(F(" ms")); }
  SerialUART.println();
}

static void mitExec(char* line) {
  // In-place split on spaces. Not strtok(): newlib's strtok carries an
  // __assert_func that links fiprintf and the whole stdio layer (~3.5 KB).
  char* tok[14]; uint8_t nt = 0;
  for (char* p = line; *p && nt < 14; ) {
    while (*p == ' ' || *p == '\t') *p++ = '\0';
    if (!*p) break;
    tok[nt++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
  }
  if (nt == 0) { if (mode != MODE_MIT) setMode(MODE_MIT); else mitPrintStatus(); return; }
  if (!strcmp(tok[0], "?") || !strcmp(tok[0], "help")) { mitHelp(); return; }
  if (!strcmp(tok[0], "test")) { mitClampTest(); return; }
  if (mode != MODE_MIT) { SerialUART.println(F("MIT: not in MIT mode -- type m <Enter> first")); return; }
  // A capture records the command it was taken under; editing mid-capture
  // would make its header lie.
  if (log_active) { SerialUART.println(F("MIT: capture running -- retry when it is done")); return; }
  if (!strcmp(tok[0], "go"))   { mitGo(tok, nt); return; }
  if (!strcmp(tok[0], "zero")) {
    mit_cmd.kp_Nm_per_rad = 0.0f; mit_cmd.kd_Nms_per_rad = 0.0f;
    mit_cmd.v_des_rads = 0.0f; mit_cmd.tau_ff_Nm = 0.0f;
    mit_cmd.p_des_rad = mitEstP(mit_est, mitDir());
    mit_sw_armed = false; mit_revert_pending = false;
    SerialUART.print(F("MIT A: ")); printMitCmd(mit_cmd); SerialUART.println();
    return;
  }
  const bool staged = !strcmp(tok[0], "b");
  const uint8_t i0 = staged ? 1 : 0;
  if (staged && nt == 1) {
    mit_stage_mask = 0;
    SerialUART.println(F("MIT B = A (no staged changes: m go just records)"));
    return;
  }
  if (((nt - i0) & 1) != 0) { SerialUART.println(F("MIT: expected field/value pairs (m ?)")); return; }
  // Edit COPIES and commit only if the whole line is valid -- a half-applied
  // line is a command nobody typed. A staged line starts EMPTY: it replaces
  // the previous staged changes rather than adding to them.
  MitCmd c = staged ? MitCmd{0.0f, 0.0f, 0.0f, 0.0f, 0.0f} : mit_cmd;
  uint8_t mask = 0;
  float tf_s = mit_tf_s, tmax = mit_tau_max_Nm;
  for (uint8_t i = i0; i < nt; i += 2) {
    float v;
    if (!mitParseF(tok[i + 1], v)) {
      SerialUART.print(F("MIT: bad number '")); SerialUART.print(tok[i + 1]); SerialUART.println('\''); return;
    }
    const char* f = tok[i];
    if      (!strcmp(f, "pd")) { c.p_des_rad      = v; mask |= MIT_F_PD; }
    else if (!strcmp(f, "vd")) { c.v_des_rads     = v; mask |= MIT_F_VD; }
    else if (!strcmp(f, "kp")) { c.kp_Nm_per_rad  = v; mask |= MIT_F_KP; }
    else if (!strcmp(f, "kd")) { c.kd_Nms_per_rad = v; mask |= MIT_F_KD; }
    else if (!strcmp(f, "ff")) { c.tau_ff_Nm      = v; mask |= MIT_F_FF; }
    else if (!staged && !strcmp(f, "tf"))   tf_s = v * 1e-3f;
    else if (!staged && !strcmp(f, "tmax")) tmax = v;
    else { SerialUART.print(F("MIT: unknown field '")); SerialUART.print(f); SerialUART.println('\''); return; }
  }
  if (staged) {
    // Checked against A as it is now; `m go` checks again against A as it is then.
    const MitCmd b = mitApply(mit_cmd, c, mask);
    if (!mitCmdValid(b, MIT_RANGES)) {
      SerialUART.println(F("MIT: REJECTED -- ranges kp 0..200, kd 0..2, |pd| <= 6.28, |vd| <= 5, |ff| <= 1"));
      return;
    }
    mit_stage = c; mit_stage_mask = mask;
    SerialUART.print(F("MIT B = ")); printMitStage();
    SerialUART.print(F("  -> if m go now: ")); printMitCmd(b); SerialUART.println();
    mitWarnInert(b);
    return;
  }
  if (!mitCmdValid(c, MIT_RANGES)) {
    SerialUART.println(F("MIT: REJECTED -- ranges kp 0..200, kd 0..2, |pd| <= 6.28, |vd| <= 5, |ff| <= 1"));
    return;
  }
  if (!(tf_s >= MIT_TF_MIN_S && tf_s <= MIT_TF_MAX_S)) {
    SerialUART.println(F("MIT: REJECTED -- tf must be 0.05..50 ms")); return;
  }
  if (!(tmax > 0.0f && tmax <= MIT_TAU_MAX_CEIL_Nm)) {
    SerialUART.println(F("MIT: REJECTED -- tmax must be > 0 and <= 1.0 N.m")); return;
  }
  mit_cmd = c;
  mit_tf_s = tf_s; mit_tau_max_Nm = tmax;
  SerialUART.print(F("MIT A: ")); printMitCmd(c);
  SerialUART.print(F("  tf_ms=")); SerialUART.print(mit_tf_s * 1000.0f, 3);
  SerialUART.print(F(" tmax="));   SerialUART.println(mit_tau_max_Nm, 3);
  mitWarnInert(c);
  // mitOnArm() clears these, so a target typed while stopped never runs.
  if (!running && (c.p_des_rad != 0.0f || c.v_des_rads != 0.0f || c.tau_ff_Nm != 0.0f))
    SerialUART.println(F("  note: stopped -- g clears A pd/vd/ff. Set them after g, or stage them: m b vd 2"));
  if (mit_tau_max_Nm > irepToTorqueOut(MIT_ENV_A_rep))
    SerialUART.println(F("  !! tmax is above the 1.6 A_rep envelope: the INNER clamp binds first"));
}

static void mitLineFinish() {
  SerialUART.println();
  mit_line[mit_line_n] = '\0';
  mit_line_on = false;
  mitExec(mit_line);
}

static void mitLineBegin() {
  mit_line_on = true; mit_line_n = 0; mit_line_hinted = false;
  mit_line_t0 = mit_line_ms = millis();
  SerialUART.print(F("MIT> "));
}

// Called every handleSerial(), whether or not a character arrived.
static void mitLinePoll() {
  if (!mit_line_on) return;
  const uint32_t idle = millis() - mit_line_ms;
  uint8_t nonsp = 0;
  for (uint8_t i = 0; i < mit_line_n; i++) if (mit_line[i] != ' ' && mit_line[i] != '\t') nonsp++;
  const bool burst = (mit_line_ms - mit_line_t0) <= MIT_LINE_BURST_MS && nonsp >= 2;
  if (burst && idle >= MIT_LINE_IDLE_MS) {
    mitLineFinish();
  } else if (idle > MIT_LINE_TIMEOUT_MS) {
    mit_line_on = false;
    SerialUART.println(F("\nMIT: line abandoned (15 s idle) -- nothing was run"));
  } else if (!mit_line_hinted && idle > MIT_LINE_HINT_MS) {
    mit_line_hinted = true;
    SerialUART.print(F("\nMIT: waiting for Enter. If your monitor sends no line ending, end the line with ';'\nMIT> "));
    for (uint8_t i = 0; i < mit_line_n; i++) SerialUART.print(mit_line[i]);
  }
}

static void mitLineChar(char c) {
  mit_line_ms = millis();
  if (c == '\r' || c == '\n' || c == ';') {
    mitLineFinish();
  } else if (c == 0x1B) {
    mit_line_on = false; SerialUART.println(F(" [cancelled]"));
  } else if (c == 0x08 || c == 0x7F) {
    if (mit_line_n) { mit_line_n--; SerialUART.print(F("\b \b")); }
  } else if (c >= 0x20 && c < 0x7F && mit_line_n < sizeof(mit_line) - 1) {
    mit_line[mit_line_n++] = c; SerialUART.print(c);
  }
}

void handleSerial() {
  mitLinePoll();
  while (SerialUART.available()) {
    char c = (char)SerialUART.read();
    // MIT LINE ENTRY -- FIRST, before the chord tracker and the key switch.
    // Inside a line every character is text; see the block above mitHelp().
    // `continue` also keeps line characters out of ac_last_key.
    if (mit_line_on) { mitLineChar(c); continue; }
    if (c == 'm' || c == 'M') { mitLineBegin(); continue; }
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
      case 'e': encoderSelfTest(); break;              // one-shot, 20000 reads
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
      case 'w': case 'W': acSwingLadder(); break;   // swing ladder, currents AC_SW_I_A_rep, output LOCKED
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
          target = kick_sign * (volts ? KICKV_BASE : KICK_BASE_A_rep);  // out of the dead zone
          kick_at = millis() + 300;                     // settle, then step (in loop)
          SerialUART.print(F("kick armed ")); SerialUART.print(kick_sign > 0 ? '+' : '-');
          SerialUART.print(F(": hold ")); SerialUART.print(target, 3);
          SerialUART.print(volts ? F(" V") : F(" A"));
          SerialUART.print(F(" 300ms, then step to "));
          SerialUART.println(kick_sign * (volts ? KICKV_A : KICK_A_rep), 3);
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
  SerialUART.println(F("=== bench harness: MT6816 SPI, FOC current ==="));
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
  motor.current_limit  = CURR_LIMIT_A_rep;

  // 2.3.1 defaults foc_modulation to SinePWM. SVPWM raises the linear ceiling
  // from V_bus/2 to V_bus/sqrt(3) (+15.5%); at bench modulation depth the phase
  // currents are unchanged and the |I|/Iq = 1.225 check still holds.
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

  printCfgBanner(SerialUART, DRIVER_VOLT_LIMIT, CURR_MAX_A_rep);

  // Direction comes from the row once measured; DIR_STORED = 0 lets initFOC
  // detect it.
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
    // Cast handles micros() wraparound. Rate-limited to 1 kHz (analogRead()
    // blocks for a few us). DORMANT, and NOT valid as written: analogRead() is
    // forbidden after currentSense.init() -- read the DMA buffer before enabling.
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
        // THE RAIL IS NOT THE BUS: setPhaseVoltage() normalises against
        // driver.voltage_limit, so the binding rail is min(DRIVER_VOLT_LIMIT, V_bus).
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
  // foc_current branches, so in TORQUE(V) Iq/Id would be STALE. Refreshed here,
  // unfiltered on purpose: LPF_current (Tf = 250 us) is longer than tau_e
  // (~200 us) and would dominate any current rise-time fit.
  if (cs_linked && mode == MODE_TORQUE) {
    motor.current = currentSense.getFOCCurrents(motor.electrical_angle);
  } else if (mode == MODE_OPENLOOP) {
    // Deliberately zeroed rather than left stale: velocityOpenloop() never writes
    // electrical_angle, so a dq transform here would be meaningless. Read |I|.
    motor.current.q = 0.0f; motor.current.d = 0.0f;
  }

  // MIT LAW (B12a). After loopFOC() so the encoder count and Iq are this
  // loop's; before the guards and motor.move(target), which it feeds.
  if (mode == MODE_MIT) mitService();

  // SENSE-MISMATCH GUARD: raw (unsynchronised) |I| must stay near AMP_INV_MAG*|Iq|.
  // A large divergence means the dq feedback has collapsed and the loop is winding
  // to the voltage rail -- the 8-12 A runaway. Checked every ~2 ms.
  // The raw read is ONE unsynchronised instant of a PWM-rippling current, so it spikes
  // several x the average during legitimate transients (e.g. breaking free of stiction).
  // A real runaway is sustained -> require GUARD_HITS consecutive violations.
  static uint16_t guard_div = 0;
  static uint8_t  guard_hits = 0;
  const uint8_t   GUARD_HITS = 8;          // ~8 x 2 ms = 16 ms of sustained mismatch
  // Both thresholds are compared against the sensed |I|, so they are REPORTED amps
  // by construction -- the unit the guard was tuned in. Do not convert them.
  const float     GUARD_PAD_A_rep   = 0.6f;   // added to the expected |I|: low-Iq noise room
  const float     GUARD_FLOOR_A_rep = 3.0f;   // raw |I| below this never counts as a hit
  if (running && isCurrentMode(mode) && ++guard_div >= 30) {
    guard_div = 0;
    PhaseCurrent_s gc = currentSense.getPhaseCurrents();
    float raw = sqrtf(gc.a*gc.a + gc.b*gc.b + gc.c*gc.c);
    float expect = AMP_INV_MAG * fabsf(motor.current.q) + GUARD_PAD_A_rep;
    if (raw > expect * 3.0f && raw > GUARD_FLOOR_A_rep && fabsf(motor.voltage.q) < VOLT_LIMIT * 0.9f) {
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
                                                  : (kick_zero ? 1.0f : KICK_A_rep));
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
      if (log_mode == MODE_MIT) {        // slot reuse -- see mitDumpBody()
        e.vel_x50  = sat16(mit_est.v_out_rads * 1000.0f);
        e.ud_x1000 = sat16(mit_tau_law_Nm * 10000.0f);
        e.cnt      = (uint16_t)(encoder.raw | ((uint16_t)mit_clamp << 14));
        if (log_i == 0) log_mit_cnt0 = mit_est.cnt - mit_est.cnt_zero;
      }
      if (++log_i >= LOG_N) {
        log_active = false; log_ready = true; log_t1 = micros();
      }
    }
  }

  // Held while an MIT line is open, so telemetry cannot print through the echo
  // (lps just averages over the longer window when it resumes).
  if (!log_active && !mit_line_on && millis() - last_print > print_ms) {
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
    if (mode == MODE_MIT) {
      // Output frame. tau is MEASURED (irepToTorqueOut(Iq)), not commanded.
      // cl=outer/inner/reject: loops each clamp bound since arm. inner > 0 is
      // a FINDING at the default tau_max -- see MIT_TAU_MAX_Nm.
      SerialUART.print(F(" p="));   SerialUART.print(mitEstP(mit_est, mitDir()), 4);
      SerialUART.print(F(" pd="));  SerialUART.print(mit_cmd.p_des_rad, 4);
      SerialUART.print(F(" v="));   SerialUART.print(mit_est.v_out_rads, 3);
      SerialUART.print(F(" tau=")); SerialUART.print(irepToTorqueOut(motor.current.q), 4);
      SerialUART.print(F(" cl="));  SerialUART.print(mit_n_tau); SerialUART.print('/');
      SerialUART.print(mit_n_iq); SerialUART.print('/'); SerialUART.print(mit_n_rej);
      if (mit_n_iq) SerialUART.print(F("!!"));
      // Longest mitService() in this status window, then reset. A MAX: it
      // includes interrupt preemption, so it reads ~2x the mean cost.
      SerialUART.print(F(" svc_us=")); SerialUART.print((float)mit_svc_cyc_max * mit_s_per_cyc * 1e6f, 2);
      mit_svc_cyc_max = 0;
    }
    SerialUART.print(F(" Vb=")); SerialUART.print(vbus_filt, 2);
    // 'seed' not 'vok': with VBUS_LIVE=false, Vb is the boot measurement and is
    // NOT tracking. Printed so no capture can be read as if it were live.
    SerialUART.print(F(" Vb_src=")); SerialUART.print(VBUS_LIVE ? F("live") : F("seed"));
    // DIAGNOSTIC: the DMA path's raw view of the same pin (see Vdma above).
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