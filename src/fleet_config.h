#pragma once
#include <Arduino.h>
// ============================================================================
// fleet_config.h  --  CONSTANTS THAT ARE THE SAME ON EVERY JOINT
// ============================================================================
// Three homes exist for a number in this project. Put a constant in the WRONG
// one and it either gets copied twelve times (and then edited eleven), or it
// gets averaged away.
//
//   fleet_config.h   FLEET / CONVENTION. Identical on every assembly by
//                    construction: motor geometry, encoder resolution, the dq
//                    convention, driver hardware configuration. Change one of
//                    these and EVERY joint changes with it.
//   joint_cal.h      PER-UNIT. Measured on one physical assembly and valid only
//                    there: zea, dir, R_eff, U0, Ke, L, vbus_scale, i_scale,
//                    drag, breakaway.
//   open_test.cpp    THIS SKETCH'S bench tuning and safety envelope: control
//                    gains, voltage / current / speed limits, capture settings.
//                    A test harness owns them; they are not shipped to the robot.
//
// A constant belongs here only if it survives the question "would this differ
// between two correctly built joints?" with a NO. If the answer is "yes, by ~2%"
// it is per-unit -- see the vbus_scale note in joint_cal.h for what that costs.
// ============================================================================

// ---------------------------------------------------------------------------
// MOTOR GEOMETRY
// ---------------------------------------------------------------------------
// TYI 4006 KV360, 12N14P. Pole pairs appear in the commutation frame, in the
// electrical-frequency conversion (omega_e = pp * omega_mech) and in every
// elec->mech angle conversion. It was written as a bare 7 in six places, so a
// motor change would have had to find all of them by grep.
static constexpr int   MOTOR_POLE_PAIRS   = 7;
static constexpr float MOTOR_KV_NAMEPLATE = 360.0f;   // rpm/V -- cross-check only

// ---------------------------------------------------------------------------
// dq CONVENTION -- SimpleFOC is amplitude-invariant (peak) throughout
// ---------------------------------------------------------------------------
// Kt = 1.5*Ke is FORCED by the dq power balance in this convention, not
// measured: 1.5*Uq*Iq = tau*omega with Uq = Ke*omega gives tau = 1.5*Ke*Iq.
// Ke = Kt was excluded on the bench at 15 sigma. Kt is therefore DERIVED from
// the stored Ke (calKt() in joint_cal.h) and never stored beside it -- two
// numbers that must agree, with nothing checking them, is a latent edit bug.
static constexpr float KT_PER_KE = 1.5f;

// sqrt(ia^2 + ib^2 + ic^2) = sqrt(3/2) * |I_dq| for a balanced three-phase set.
// This is the integrity check that closes at every rotor position and every
// speed: |I| / sqrt(Iq^2 + Id^2) must sit at 1.2247. Used by the sense-mismatch
// guard, by AUTOCALIB's ratio gate, and to convert phase magnitude -> amplitude.
static constexpr float AMP_INV_MAG     = 1.2247449f;   // sqrt(3/2)
static constexpr float AMP_INV_MAG_INV = 0.8164966f;   // sqrt(2/3)

// ---------------------------------------------------------------------------
// ENCODER -- MT6816, 4-wire SPI, 14-bit absolute
// ---------------------------------------------------------------------------
// One mechanical revolution per wrap. ENC_BITS is used as a shift when binning
// by rotor position, so it and ENC_CPR are derived from each other here on
// purpose: they cannot drift apart.
static constexpr uint8_t  ENC_BITS = 14;
static constexpr uint32_t ENC_CPR  = 1UL << ENC_BITS;              // 16384
static constexpr float    ENC_RAD_PER_COUNT = 6.28318531f / (float)ENC_CPR;

// ---------------------------------------------------------------------------
// DRIVER HARDWARE CONFIG -- the values a measurement is only comparable under
// ---------------------------------------------------------------------------
// PWM frequency. 25 kHz is the SimpleFOC STM32 default (_PWM_FREQUENCY in
// stm32_mcu.h); it is now assigned EXPLICITLY at setup so the boot banner prints
// a number instead of NOT_SET's -12345 sentinel. "A library default is a
// decision nobody made" -- this makes the decision visible without changing it.
// Byte-identical to leaving it unset: the HAL substitutes exactly this value.
static constexpr uint32_t PWM_FREQ_HZ = 25000;

// DEAD ZONE -- FINAL VALUE, set by argument and confirmed by measurement.
// Board-family constant (EG2124A gate driver), not per-unit.
// dead_zone is a fraction of the PWM period, so its cost in lost command voltage
// is dead_zone * V_bus regardless of switching frequency: 0.057 V at 3S, 0.093 V
// at 5S. Referred to foot force at G = N/J = 87.7 that is ~1.0 N / ~1.6 N.
// The EG2124A already has interlock AND internal dead time, so its own dead time
// (a fixed ~200-300 ns = 0.005-0.0075 of a 40 us period) is comparable to or
// larger than this software value -- i.e. 0.005 is probably being absorbed by a
// floor set in hardware. Going to 0.000 would make shoot-through protection
// depend entirely on an undocumented interlock propagation delay on a clone
// board; going to 0.010 doubles the deadband for no extra protection.
// On STM32 6-PWM this value is baked into the timer at driver.init() -- it is NOT
// runtime-mutable, and the requested fraction is quantised. Ground truth is the
// measured U0 intercept from the locked-rotor sweep, NOT this number.
// Long-term fix for the deadband is U0 feedforward, not a smaller dead zone.
static constexpr float DEAD_ZONE = 0.005f;

// ---------------------------------------------------------------------------
// CONTROL TRANSPORT DELAY -- fleet constant, expressed as a RATIO
// ---------------------------------------------------------------------------
// T_delay is NOT per-unit and it is NOT a fixed number of microseconds. It is
// ONE control-loop period, so it scales with 1/f_loop, and any statement of it
// in microseconds is only true at the loop rate it was measured at.
//
// Five parity-separation determinations, two assemblies, two loop rates:
//     0.956 (16.77 kHz) | 0.953, 0.978 (12.91 kHz) | 0.945, 0.974 (13.35 kHz)
//     mean 0.961, sd 0.014
// The old "0.5*T_pwm + 0.5*T_loop" model is 15-29% low: the PWM period (40 us)
// is SHORTER than the loop period, so the duty update lands inside one PWM cycle
// and the loop period dominates outright. LOOP RATE BUYS TRANSPORT DELAY
// ONE-FOR-ONE, which is the reason acService() does not read the phase currents
// twice.
//
// SUPERSEDES "preliminary T ~ 143 us, 40-60 us unaccounted, 3.6% torque loss at
// 270 rad/s". That came from a forward-only incidental capture, which
// structurally cannot separate the EVEN terms (ZEA residual, INL) from the ODD
// one (transport). Actual loss at 270 rad/s and 13.3 kHz is 0.95% -- a 3.8x
// overstatement -- and the angle-lag sweep it motivated is CLOSED, not pending.
//
// Nothing compensates for this and nothing should: 0.95% against Coulomb
// friction at 46%. It lives here so the number has ONE home instead of being
// restated in three README tables.
static constexpr float T_DELAY_PER_LOOP    = 0.961f;
static constexpr float T_DELAY_PER_LOOP_SD = 0.014f;
static inline float tDelayAt(float f_loop_hz) {
  return (f_loop_hz > 1.0f) ? (T_DELAY_PER_LOOP / f_loop_hz) : 0.0f;
}

// ---------------------------------------------------------------------------
// DRIVETRAIN -- frozen 2026-07-29 (README section 17)
// ---------------------------------------------------------------------------
// Geometric, therefore fleet. Force-per-amp is a LEG property, not a joint one,
// and deliberately does not live here -- see M14.
static constexpr float GEAR_RATIO = 9.0f;    // 12T alu pinion -> 108T pulley
