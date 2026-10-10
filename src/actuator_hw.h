#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include "fleet_config.h"
#include "joint_cal.h"
#include "mt6816.h"
// ============================================================================
// actuator_hw.h -- the actuator's hardware objects and init order.
// ============================================================================
// Shared by the bench harness and Tier 0. Include it AFTER VBUS_FALLBACK is
// defined (vbus_filt is initialised from it).
//
// UNIT RISK: every current crossing this boundary is REPORTED amps
// (current_limit, PI limits, the banner's Imax). i_scale enters only through
// joint_cal.h's torque boundary; a torque in N.m must never reach motor.target
// or current_limit by any other route.
//
// ODR: this header DEFINES its objects. Fine while each build has one
// translation unit (open_test.cpp, tier0_main.cpp); a second TU including it
// needs an .h/.cpp split.
// ============================================================================

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);
// 3 mOhm shunts (R003), same as the genuine board. The absolute scale error is i_scale (M2), not this constructor.
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

// Everything the caller must decide. Policy is passed in, not defined here: the
// harness envelope is not the robot's (CLAUDE.md, three homes).
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
  // ---- FLASH PREFETCH (fleet_config.h). First, so every later step -- and every
  // loop -- runs under the same fetch path. No functional effect; measured no
  // loop-rate effect either (2026-10-02, see fleet_config.h).
  if (FLASH_PREFETCH) __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  // ---- BUS VOLTAGE: seed BEFORE driver.init() and BEFORE currentSense.init().
  // Ordering is deliberate: currentSense.init() reconfigures the ADC, so any
  // Arduino-API analogRead() must either happen before it or be verified against
  // it afterwards (see the |I|/Iq gate in the verification procedure).
  analogReadResolution(12);                  // default is 10-bit; 2 bits for free
  pinMode(cfg.pin_vbus, INPUT_ANALOG);           // detach digital buffer, unload divider
  // 64 samples, first conversion discarded. The remaining boot-to-boot scatter
  // (up to 0.05 V on J02) is a real per-boot offset, not ADC noise, so more
  // averaging cannot remove it. This value is driver.voltage_power_supply and
  // lands 1:1 on M2's g: compare banner vs meter at session start and reboot if
  // they differ by more than 0.03 V.
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
  // driver.voltage_limit is the SVPWM MODULATION REFERENCE, not a safety limit:
  // it sets the phase-voltage ceiling (6.0/sqrt(3) = 3.46 V) and, with
  // modulation_centered, the duty centre (6.0/12.46 = 24%). Raising it leaves
  // R_eff and Ke unchanged (the factor cancels in the differential voltage) but
  // changes U0 and the low-side current-sense window: afterwards re-run phase 3
  // and check phase 1 and the phase-5 |I| ratio (CONSTANTS §8.3).
  driver.voltage_limit = cfg.driver_volt_limit;
  driver.dead_zone     = cfg.dead_zone;
  // Assigned so the banner prints a number (NOT_SET reads -12345); the HAL uses
  // exactly 25000 when it is unset.
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
  out.print(F(" jump_guard=")); out.print(SPI_JUMP_GUARD ? 1 : 0);
  // Read back from the register, not the flag, so every capture is traceable to
  // the fetch path it ran under (FLASH_PREFETCH, fleet_config.h).
  out.print(F(" prefetch=")); out.print((FLASH->ACR & FLASH_ACR_PRFTEN) ? 1 : 0);
  out.print(F(" flash_ws="));  out.println((uint32_t)(FLASH->ACR & FLASH_ACR_LATENCY));
}

// Alignment. Reads CAL.zea / CAL.dir directly so this header stays one-directional.
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
    // Direction is detected unless CAL.dir has been measured.
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
