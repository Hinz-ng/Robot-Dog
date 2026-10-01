#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include "fleet_config.h"
#include "joint_cal.h"
#include "mt6816.h"
// ============================================================================
// actuator_hw.h -- the actuator's hardware objects.
// ============================================================================
// EXTRACTED VERBATIM from open_test.cpp (2026-09-10) as stage 3a: a pure
// RELOCATION, gated on identical section sizes and an unchanged symbol table.
// Stage 3b then moved the init ORDER, the boot Vbus seed and the CFG banner
// here (actuatorInitHw / actuatorInitMotor / printCfgBanner / runInitFOC). That
// was a restructure, so its gate was the bench, not a hash: BENCH-ACCEPTED on
// J01, 2026-10-01 -- CFG fields all match, parity 0/20000 at 6.65 us, V 2.51
// deg elec, locked |I|/Iq 1.222 at 1.0 A, auto-stop at 20 s (README 15).
//
// Include point is DERIVED, not chosen: it must sit AFTER VBUS_FALLBACK (which
// vbus_filt is initialised from) and BEFORE vbusProbe(), which uses `motor`.
//
// ---------------------------------------------------------------------------
// *** UNIT RISK -- EVERY CURRENT THAT CROSSES THIS BOUNDARY IS REPORTED AMPS. ***
// ---------------------------------------------------------------------------
// And that is the permanent rule, not a pending fix (fleet_config.h, decided
// 2026-10-01). Every limit below -- current_limit, the PI limits, the Imax
// passed to printCfgBanner() -- is A_rep and is never converted. i_scale
// (J01 0.9621, J02 0.9690) enters ONLY through joint_cal.h's torque boundary,
// torqueOutToIrep() / irepToTorqueOut(), and tauOutCmdToIq() for commands. A
// torque in N.m must never reach motor.target or current_limit by any other
// route: that is the exact place a silent 3-4% error would enter the robot.
//
// ---------------------------------------------------------------------------
// ODR -- READ BEFORE INCLUDING THIS FROM A SECOND TRANSLATION UNIT.
// ---------------------------------------------------------------------------
// This header DEFINES these objects rather than declaring them. That is correct
// and harmless while open_test.cpp is the only TU, and it is a MULTIPLE
// DEFINITION error the first time Tier-0 includes it from two. The fix is an
// .h/.cpp pair -- which reopens the static-allocation question this refactor
// deliberately leaves closed (RAM is at 73.9%, ~8.5 kB headroom), so it is
// sequenced after Tier-0 starts, not now.
// ============================================================================

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);
// Clone sense chain is gain-compensated -> genuine constants. Do not change.
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MT6816SPI encoder = MT6816SPI();           // ENC_BITS-bit absolute, ENC_CPR counts/rev

float vbus_filt           = VBUS_FALLBACK;
bool  vbus_valid          = false;

// ---------------------------------------------------------------------------
// THE INIT ORDER. This sequence is load-bearing and was expensive to find:
//   driver config -> driver.init -> linkDriver -> encoder.init -> linkSensor
//     -> [CALLER'S MOTOR POLICY]  -> motor.init -> currentSense.init
//     -> linkCurrentSense -> disable
// It is split into two functions rather than one BECAUSE SimpleFOC forces the
// split: motor.init() consumes voltage_limit, so the caller's gains and limits
// must be set between the two calls. Encoding it this way keeps the ORDER here
// (where Tier-0 inherits it) while leaving POLICY with the caller (where Tier-0
// substitutes its own). Do not merge them.
//
// The Vbus seed is inside phase 1 and BEFORE driver.init() on purpose --
// currentSense.init() reconfigures the ADC, after which analogRead() is
// forbidden on this board. See the SEED-ONLY block in open_test.cpp.
// ---------------------------------------------------------------------------

// Everything the caller must decide. Bench policy does NOT live in this header:
// open_test.cpp's envelope is a harness envelope and the robot will not inherit
// it (claude.md's three-homes table), so it is passed in rather than defined.
struct ActuatorHwCfg {
  float    driver_volt_limit;  // SVPWM MODULATION REFERENCE, not a safety limit
  float    dead_zone;
  long     pwm_hz;
  uint32_t pin_vbus;
  float    vbus_scale;         // V per ADC count -- PER BOARD, from joint_cal.h
  float    vbus_min;           // plausibility window only -- NOT undervoltage
  float    vbus_max;           //   protection. See open_test.cpp's note.
  float    vbus_fallback;
};

// Phase 1: bus-voltage seed, driver, encoder. Returns driver_ok.
static bool actuatorInitHw(const ActuatorHwCfg& cfg, Print& out) {
  // ---- BUS VOLTAGE: seed BEFORE driver.init() and BEFORE currentSense.init().
  // Ordering is deliberate: currentSense.init() reconfigures the ADC, so any
  // Arduino-API analogRead() must either happen before it or be verified against
  // it afterwards (see the |I|/Iq gate in the verification procedure).
  analogReadResolution(12);                  // default is 10-bit; 2 bits for free
  pinMode(cfg.pin_vbus, INPUT_ANALOG);           // detach digital buffer, unload divider
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
    (void)analogRead(cfg.pin_vbus);   // discard: first conversion carries residue
    for (int k = 0; k < 64; k++) acc += analogRead(cfg.pin_vbus);   // average 64
    float v = (acc / 64.0f) * cfg.vbus_scale;
    if (cfg.vbus_scale > 0.0f && v > cfg.vbus_min && v < cfg.vbus_max) {
      vbus_filt = v;  vbus_valid = true;
    } else {
      vbus_filt = cfg.vbus_fallback;  vbus_valid = false;
      out.println(F("!! VBUS read implausible -- using fallback"));
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
  driver.voltage_limit = cfg.driver_volt_limit;
  driver.dead_zone     = cfg.dead_zone;
  // Assigned explicitly so the CFG banner prints a NUMBER. Left unset it stays
  // at NOT_SET and the banner printed -12345 -- a sentinel that reads like data.
  // The STM32 HAL substitutes exactly 25000 when unset, so this changes nothing
  // but the banner. "A library default is a decision nobody made."
  driver.pwm_frequency = cfg.pwm_hz;
  const bool driver_ok = driver.init();
  out.println(driver_ok ? F("driver OK") : F("driver FAILED"));

  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);

  out.println(F("initialising MT6816 SPI encoder..."));
  encoder.init();                            // bit-banged: consults no pin map
  motor.linkSensor(&encoder);
  out.print(F("MT6816 raw=")); out.print(encoder.rawCount());
  out.print(F(" / ")); out.print(ENC_CPR);
  out.print(F("  parity_err=")); out.print(encoder.spi_err);
  out.print(F(" no_mag=")); out.println(encoder.no_mag);
  if (encoder.spi_err) out.println(F("!! SPI parity errors at boot -- check CSN and HVPP->VDD"));
  if (encoder.no_mag)  out.println(F("!! No_Mag_Warning at boot -- magnet weak/far. Fix before running."));
  return driver_ok;
}

// Phase 2: motor + current sense. Call AFTER setting motor policy.
// Returns cs_ok; sets cs_linked.
static bool actuatorInitMotor(Print& out, bool& cs_linked) {
  motor.init();

  const bool cs_ok = currentSense.init();
  out.println(cs_ok ? F("currentSense OK") : F("currentSense FAILED"));
  if (cs_ok) {
    motor.linkCurrentSense(&currentSense);
    cs_linked = true;
    out.println(F("currentSense LINKED"));
  }

  motor.disable();
  return cs_ok;
}

// The CFG banner. driver_volt_limit is a PARAMETER, not a global: it is the
// caller's bench policy, and reading it from open_test.cpp would make this
// header depend upward on the file that includes it.
static void printCfgBanner(Print& out, float driver_volt_limit, float curr_max_A_rep) {
  // "Note which config is actually flashed." A measurement is only comparable to
  // others taken under the SAME four values. On STM32 6-PWM, dead_zone is
  // converted to a timer dead-time register value at driver.init() and quantised,
  // so this echoes what was REQUESTED -- the measured U0 intercept is what the
  // hardware actually does.
  out.print(F("CFG modulation="));
  out.print(motor.foc_modulation == FOCModulationType::SpaceVectorPWM ? F("SVPWM") : F("SinePWM"));
  out.print(F(" dead_zone=")); out.print(driver.dead_zone, 4);
  out.print(F(" pwm_Hz="));    out.print(driver.pwm_frequency);
  out.print(F(" Vbus="));      out.print(driver.voltage_power_supply, 2);
  out.print(F(" vok="));       out.print(vbus_valid ? 1 : 0);
  // The limits that actually bind, echoed because "a limit that does not bind is
  // not protection" and "a clamp in the wrong units is not a clamp". Uq_max is
  // what motor.move() constrains voltage.q to; Uq_ceil is what the modulator can
  // physically synthesise. Uq_max > Uq_ceil would be a limit that does not exist.
  out.print(F(" Uq_max="));    out.print(motor.voltage_limit, 2);
  out.print(F(" Uq_ceil="));   out.print(
      ((driver_volt_limit < driver.voltage_power_supply)
         ? driver_volt_limit : driver.voltage_power_supply) * 0.57735f, 2);
  // TWO current limits, and they bind in DIFFERENT modes -- printing only one
  // was misleading the moment CURR_MAX_A_rep and CURR_LIMIT_A_rep stopped being equal
  // (2026-09-17). Imax is the TORQUE(I) target clamp and is the one that binds
  // there; Ilim is motor.current_limit, which 2.3.1 applies only through
  // PID_velocity.limit, i.e. VELOCITY mode. In TORQUE(I) move() assigns
  // current_sp = target with no constrain, so Ilim is inert.
  out.print(F(" Imax="));      out.print(curr_max_A_rep, 2);
  out.print(F(" Ilim="));      out.print(motor.current_limit, 2);
  // Echo every load-bearing default: a library default is a decision nobody made.
  out.print(F(" v_align="));   out.print(motor.voltage_sensor_align, 2);
  // R_eff is 0.0f on every UNBUILT row (= NOT MEASURED, see joint_cal.h), so the
  // alignment current is genuinely unknown there. Print "?" rather than an inf.
  out.print(F(" (I_align="));
  if (CAL.R_eff > 0.0f) out.print(motor.voltage_sensor_align / CAL.R_eff, 1);
  else                  out.print(F("? R_eff NOT MEASURED"));
  out.print(F("A) spi_nops=")); out.print(SPI_HALF_NOPS);
  out.print(F(" jump_guard=")); out.println(SPI_JUMP_GUARD ? 1 : 0);
}

// Alignment. Reads CAL.zea / CAL.dir from joint_cal.h directly rather than
// open_test.cpp's ZEA_STORED / DIR_STORED aliases, so this stays one-directional.
// foc_ready and target are references because the original mutated the harness
// globals of those names; behaviour is unchanged, the coupling is not.
static void runInitFOC(bool force_align, bool is_running, bool& foc_ready, float& target, Print& out) {
  if (is_running) { out.println(F("stop first (x)")); return; }
  bool use_stored = (!force_align && CAL.zea >= 0.0f && CAL.dir != 0);
  if (use_stored) {
    // Absolute sensor: ZEA is a constant, not a per-session measurement.
    motor.zero_electric_angle = CAL.zea;
    motor.sensor_direction    = (CAL.dir > 0) ? Direction::CW : Direction::CCW;
    out.println(F("initFOC: STORED ZEA -- no alignment, no twitch."));
  } else {
    motor.zero_electric_angle = NOT_SET;
    // Direction must be DETECTED on this board: the SPI angle convention is not
    // the TIM4 count convention. Pin it only once CAL.dir has been measured.
    motor.sensor_direction = (CAL.dir > 0) ? Direction::CW
                           : (CAL.dir < 0) ? Direction::CCW
                                              : Direction::UNKNOWN;
    out.println(F("initFOC: aligning (expect a small twitch)."));
  }
  motor.enable();
  int ok = motor.initFOC();
  motor.disable();
  target = 0.0f;
  if (ok) {
    foc_ready = true;
    out.println(F("initFOC SUCCESS"));
    out.print(F("zero_electric_angle=")); out.println(motor.zero_electric_angle, 4);
    out.print(F("sensor_direction=")); out.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
  } else {
    foc_ready = false;
    out.println(F("initFOC FAILED"));
  }
}
