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
// SEVEN parity-separation determinations, three assemblies, three loop rates:
//     0.956 (16.77 kHz) | 0.953, 0.978 (12.91 kHz) | 0.945, 0.974 (13.35 kHz)
//     | 0.937, 0.961 (J02, 13.35 kHz)
//     mean 0.958, sd 0.015
// The mean moved 0.3% and the sd did not grow when a THIRD assembly was added.
// That is the evidence that this is a property of the control loop and not of
// any one build.
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
static constexpr float T_DELAY_PER_LOOP    = 0.958f;   // n = 7
static constexpr float T_DELAY_PER_LOOP_SD = 0.015f;
static inline float tDelayAt(float f_loop_hz) {
  return (f_loop_hz > 1.0f) ? (T_DELAY_PER_LOOP / f_loop_hz) : 0.0f;
}

// ---------------------------------------------------------------------------
// DRIVETRAIN -- frozen 2026-07-29 (README section 17)
// ---------------------------------------------------------------------------
// Geometric, therefore fleet. Force-per-amp is a LEG property, not a joint one,
// and deliberately does not live here -- see M14.
static constexpr float GEAR_RATIO = 9.0f;    // 12T alu pinion -> 108T pulley

// ---------------------------------------------------------------------------
// ROTOR INERTIA -- measured 2026-08-08, belt off (M6a)
// ---------------------------------------------------------------------------
// MOTOR SHAFT, BEFORE the 9:1. Geometric, so it is a fleet constant (~1% unit to
// unit), which is why it lives here and not in JointCal.
//
// THREE METHODS, two of them independent physics:
//     M6a driven step (tau = Kt*Iq - tau_drag = J*alpha)      20.2e-6
//     free coast-down run 1, impulse-momentum vs the drag map 16.31e-6
//     free coast-down run 2 (started mid-coast, 21 rad/s)     24.29e-6
//     coast mean                                              20.30e-6
// A driven measurement and a free-decay measurement landing on the same number
// is the reason this is +-10% and not +-30%.
//
// SANITY, since a physical baseline built on a guessed mass is what went wrong
// earlier in this campaign: J = m*r^2 for a thin ring gives m = J/r^2 =
// 20.2e-6 / 0.0193^2 = 54 g of rotating mass at the magnet radius. The bare
// motor WEIGHS 98 g, and an outrunner's bell + magnets is typically 50-60% of
// that: 49-59 g. The measurement is the expected value, not an anomaly.
//
// The prior ESTIMATE in README section 17 was 2.1e-5. It was 4.0% high, so every
// reflected-inertia figure derived from it stands, corrected down by 4%.
//
// J02 (2026-08-08): 18.17e-6 (impulse-momentum) / 19.20e-6 (angle-trajectory fit,
// rms 47 counts -- nearly twice as clean as J01's 83). Mean 18.7e-6, which is
// 7.9% BELOW J01. That gap is ENTIRELY THE DRAG CORRECTION, and it is provable:
// J02's drag map is 36% higher than J01's, so friction is 31.9% of the impulse
// on J02 against 25% on J01. Rerun J02's impulse integral with J01's drag map
// and it gives 20.10e-6 -- J01's own value to 0.5%. One fact, not two.
//
// So the fleet value STANDS. J is geometric; two units of the same product with
// the same 98 g mass cannot differ by 8% in inertia, and the drag correction's
// own uncertainty (+-20-37%, see FRICTION_TRIAL_SPREAD) covers the gap several
// times over. Tolerance widened 10% -> 12% to carry both determinations.
//
// LIMITING ERROR IS THE DRAG MAP (25-32% of the impulse), NOT THE FIT. A future
// two-step version -- 0.20->0.80 and 0.20->1.60, differenced over the SAME omega
// window -- would cancel friction exactly. Not worth new firmware yet.
//
// The free coast-down cross-check is RETIRED, not merely unused: three attempts
// on two joints all started after the disable because the 'L' keypress lands
// hundreds of ms behind the 'x', by which time the coast is nearly over. J02's
// captured only the last 2.3 rad/s and returned 223e-6, i.e. nonsense. It is not
// viable with keyboard timing and the driven step is the better measurement.
static constexpr float J_ROTOR_KGM2 = 20.2e-6f;      // +-2.4e-6 (12%), n = 2 joints

// FRICTION VARIES +-20% TRIAL TO TRIAL on this plant, and that is not noise in
// the instrument: M4's static breakaway scatter (sd/mean 20.9%) and the two free
// coast-downs (implied friction 1.24x and 0.83x the drag map, +-20%) agree, from
// completely different physics on the same afternoon. That is the signature of
// GREASE REDISTRIBUTION -- a fixed geometric interference (a rub, a bent shaft,
// a brinelled race) would be REPRODUCIBLE, and this is not. Do not treat any
// single friction number here as better than +-20%, and do not chase the spread.
static constexpr float FRICTION_TRIAL_SPREAD = 0.20f;   // fractional, 1 sigma-ish

// ---------------------------------------------------------------------------
// FOOT FORCE PER MOTOR TORQUE
// ---------------------------------------------------------------------------
//     G = N / (dz_foot / dtheta_joint)  =  gear ratio / Jacobian
//
// *** PER MOTOR. A LEG HAS TWO. F_leg = 2 * G * eta * tau_motor. ***
//
// The "161 /m" that a naive back-calculation from A1's stored line produces was
// never a second value of G -- it is 87.7 with the factor of two already folded
// in. A1's record reads "1.05 A -> 4.5 N per leg -> 46% of a 9.81 N standing
// load", and 2 * 87.7 * Kt * 1.05 = 4.90 N against a recorded 4.5 N. The 9%
// residual is the DRIVETRAIN EFFICIENCY term eta that the README formula already
// carries. It is NOT evidence that G is uncertain, and it is not a leg-height
// difference -- but see the provenance warning on DRIVETRAIN_ETA below before
// treating that eta as a measurement, because it is not one.
//
// G IS NOT A CONSTANT. It is a Jacobian and it varies through the stroke, worst
// toward full extension where the Jacobian collapses. ANY force figure quoted in
// newtons MUST state the leg height it was evaluated at.
//
// STILL TO FILL IN: which h produces 87.7. The target is dz/dtheta = N/G =
// 9/87.7 = 102.6 mm. Compute dz/dtheta at h = 115, 134 and 165 mm from the
// frozen geometry (80 mm proximal / 100 mm distal five-bar, hip pivot separation
// from CAD), find which gives 102.6 mm, and record all three. Desk exercise, no
// instruments -- it is unfilled here because the hip pivot separation is not in
// this repository, not because it is hard.
static constexpr float G_FOOT_PER_MOTOR_NM = 87.7f;   // 1/m, at h = ___ mm  <-- FILL IN

// ---------------------------------------------------------------------------
// DRIVETRAIN EFFICIENCY -- *** BACK-SOLVED, NOT MEASURED. CIRCULAR. ***
// ---------------------------------------------------------------------------
// eta = 4.5 / (2 * 87.7 * 0.026626 * 1.05) = 0.918, from A1's stored line.
//
// THAT LINE'S OWN "HOW MEASURED" COLUMN READS `F = 2*G*eta*Kt*I`. The 4.5 N was
// COMPUTED with a formula that already contained eta -- so back-solving eta out
// of it returns the eta that was put in. It is a round trip and it proves
// nothing about the physical drivetrain. NOBODY HAS PUT A LOAD CELL ON A FOOT.
//
// Two consequences of that being exactly circular, and the second is easy to get
// backwards:
//   * KEEP IT ANYWAY. Because the round trip is exact, this value is precisely
//     the eta every existing force figure in the README already assumes, so
//     using it keeps new conversions CONSISTENT with the old ones. It buys
//     consistency, not truth.
//   * The A1 rubbing-magnet contamination does NOT make eta "doubly suspect".
//     The 1.05 A cancels exactly in the back-solve, so contamination cannot
//     reach eta. eta carries zero information either way -- which is worse than
//     suspect, and is the point.
//
// M14 (load cell, assembled leg) measures G*eta*Kt as ONE LUMPED number and
// REPLACES this. Until then this is a structurally identical hazard to i_scale:
// an unverified multiplicative factor applied invisibly to every force output.
//
// CONVENTION -- DO NOT DOUBLE-COUNT. eta is the LOAD-DEPENDENT belt loss (tooth
// engagement, belt bending), proportional to transmitted torque. The
// LOAD-INDEPENDENT loss is drag_c / drag_v in JointCal. Feed footForce*() a
// torque that has ALREADY had drag subtracted, or you subtract friction once and
// then multiply it back out again.
static constexpr float DRIVETRAIN_ETA = 0.92f;

// Foot force from ONE motor's torque, and from a LEG (two motors). The second
// exists so that the factor of two is spent once, here, instead of being
// rediscovered every time a current is converted into newtons.
static inline float footForcePerMotor(float tau_motor_Nm) {
  return G_FOOT_PER_MOTOR_NM * DRIVETRAIN_ETA * tau_motor_Nm;
}
static inline float footForcePerLeg(float tau_motor_Nm) {
  return 2.0f * footForcePerMotor(tau_motor_Nm);
}
