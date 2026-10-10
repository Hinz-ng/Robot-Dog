#pragma once
#include <Arduino.h>
// ============================================================================
// fleet_config.h  --  CONSTANTS THAT ARE THE SAME ON EVERY JOINT
// ============================================================================
// A constant belongs here only if "would this differ between two correctly
// built joints?" is NO ("yes, by ~2%" means per-unit). Per-unit values live in
// joint_cal.h, bench tuning in open_test.cpp, Tier 0 policy in tier0_config.h.
// ============================================================================

// ---------------------------------------------------------------------------
// MOTOR GEOMETRY
// ---------------------------------------------------------------------------
// TYI 4006 KV360, 12N14P.
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

// ---------------------------------------------------------------------------
// CURRENT UNITS (decided 2026-10-01, CONSTANTS §8.1c)
// ---------------------------------------------------------------------------
// The REPORTED amp (suffix _A_rep) is the firmware's current unit. The sense
// under-reads (M2: J01 i_scale 0.9621), so a 6.0 A_rep limit trips at ~6.24 A
// true. Physical torque meets reported amps only at joint_cal.h's
// torqueOutToIrep() / irepToTorqueOut(), for commands AND feedback; every torque
// command goes through tauOutCmdToIq(): clamp tau_max [N.m] -> convert -> clamp
// the demonstrated envelope [A_rep].
// A limit lives in the unit of its source: bench-measured -> _A_rep, never
// converted; contract (tau_max, tau_ff, kp, kd) -> N.m output; datasheet or
// physics (board rating, thermal, B10) -> A_true, converted at init
// (I_rep = I_true * i_scale).
// Option B (correct the sense gain at source) reopens only with a fleet-wide
// re-characterisation AND g shown common across 3-4 boards: it moves R_eff, L,
// drag, breakaway and every amp threshold by 1/g and needs phases 3-4 re-run.
//
// sqrt(ia^2 + ib^2 + ic^2) = sqrt(3/2) * |I_dq| for a balanced set: the
// integrity check used by the sense-mismatch guard, AUTOCALIB's ratio gate and
// the phase-magnitude -> amplitude conversion.
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
// PWM frequency. 25 kHz is SimpleFOC's STM32 default; assigned explicitly so
// the CFG banner prints a number instead of NOT_SET (-12345). Binary-neutral.
static constexpr uint32_t PWM_FREQ_HZ = 25000;

// FLASH PREFETCH. Enabled 2026-10-01; measured no loop-rate effect 2026-10-02.
// Kept because it is harmless and later captures ran with it; the CFG banner
// prints the FLASH->ACR bits. The 8 flash wait states (4 is in spec at 170 MHz)
// are a deferred item (BELT_DRIVE §22.7.7).
static constexpr bool FLASH_PREFETCH = true;

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
// CONTROL TRANSPORT DELAY -- fleet, as a RATIO of the loop period
// ---------------------------------------------------------------------------
// T_delay ~= one control-loop period, so it scales with 1/f_loop. Seven
// parity-separation determinations, three assemblies, three loop rates:
//     0.956 (16.77 kHz) | 0.953, 0.978 (12.91 kHz) | 0.945, 0.974 (13.35 kHz)
//     | 0.937, 0.961 (J02, 13.35 kHz)  ->  mean 0.958, sd 0.015
// The PWM period (40 us) is shorter than the loop period, so the loop period
// dominates: loop rate buys transport delay one-for-one. BELT-OFF value (phase 6
// is invalid belt-on). Torque loss at 270 rad/s and 13.3 kHz is 0.95%: no
// angle compensation.
static constexpr float T_DELAY_PER_LOOP    = 0.958f;   // n = 7
static constexpr float T_DELAY_PER_LOOP_SD = 0.015f;
static inline float tDelayAt(float f_loop_hz) {
  return (f_loop_hz > 1.0f) ? (T_DELAY_PER_LOOP / f_loop_hz) : 0.0f;
}

// ---------------------------------------------------------------------------
// DRIVETRAIN -- frozen 2026-07-29 (ROBOT_DESIGN §17)
// ---------------------------------------------------------------------------
// Geometric, therefore fleet. Force-per-amp is a LEG property, not a joint one,
// and deliberately does not live here -- see M14.
static constexpr float GEAR_RATIO = 9.0f;    // 12T alu pinion -> 108T pulley

// BELT-LINE GEOMETRY. Fleet by construction -- same pinion, same 2 mm GT2 pitch
// on every joint. The PITCH is the primary quantity and the radius is derived
// from it, not the other way round: 12 teeth x 2.000 mm = 24.000 mm of belt per
// motor revolution, EXACTLY, which is why R_PINION_MM lands on 3.8197 and not on
// a round number. Closed form confirmed against the CAD span to 3 um.
static constexpr float BELT_PITCH_MM         = 2.0f;     // GT2
static constexpr uint8_t PINION_TEETH        = 12;
static constexpr float BELT_MM_PER_MOTOR_REV = BELT_PITCH_MM * (float)PINION_TEETH;  // 24.000
static constexpr float R_PINION_MM           = BELT_MM_PER_MOTOR_REV / 6.28318531f;  // 3.8197
// The encoder as a belt-travel gauge. This is what makes a swing test a LENGTH
// measurement rather than an angle one.
static constexpr float BELT_MM_PER_COUNT     = BELT_MM_PER_MOTOR_REV / (float)ENC_CPR;  // 1.4648 um
// One pinion tooth, at the encoder. A swing or return reading that moves by this
// -- or a multiple of it -- is a TOOTH SKIP, not compliance. 16384/12 = 1365.33.
static constexpr float ENC_CNT_PER_TOOTH     = (float)ENC_CPR / (float)PINION_TEETH;

// ---------------------------------------------------------------------------
// ROTOR INERTIA -- M6a, belt off, 2026-08-08. Motor shaft, before the 9:1.
// ---------------------------------------------------------------------------
// Geometric, so fleet (~1% unit to unit). J01: M6a driven step 20.2e-6; free
// coast-downs 16.31 / 24.29e-6 (mean 20.30). J02: 18.7e-6 -- the 7.9% gap is
// its higher drag map (J02's impulse with J01's map gives 20.10e-6). Limiting
// error is the drag map (25-32% of the impulse), not the fit. Sanity: 54 g at
// the 19.3 mm magnet radius, for a 98 g motor. In-situ (B12a a3): a
// direction-independent 14/turn ripple torque requires J = 20.2e-6.
static constexpr float J_ROTOR_KGM2 = 20.2e-6f;      // +-2.4e-6 (12%), n = 2 joints

// Friction varies +-20% trial to trial (M4 scatter and two coast-downs agree):
// handling the shaft redistributes grease. Do not treat one friction number as
// better than +-20%.
static constexpr float FRICTION_TRIAL_SPREAD = 0.20f;   // fractional, 1 sigma-ish

// ---------------------------------------------------------------------------
// FOOT FORCE PER MOTOR TORQUE
// ---------------------------------------------------------------------------
//     G = N / (dz_foot / dtheta_joint)  =  gear ratio / Jacobian
// PER MOTOR. A leg has two: F_leg = 2 * G * eta * tau_motor.
// G is a Jacobian and varies through the stroke: quote a force with the leg
// height it was evaluated at. 87.7 is the ALPHA-AVERAGE over the working stroke
// (alpha 40-70 deg; mean |dh/da| 102.641 mm) -- right for energy, wrong for
// peak force (use G_FOOT_AT_STROKE_TOP).
//
// Coaxial-hip symmetric five-bar, L1 = 80 (proximal), L2 = 100 (distal):
//     h        = L1*cos(a) + sqrt(L2^2 - (L1*sin a)^2)
//     |dh/da|  = L1*sin(a) * (1 + L1*cos(a)/sqrt(L2^2 - (L1*sin a)^2))
//
//   alpha   h_pin   h_ground  |dh/da|      G      F/I     added mass/leg
//    40   147.05   159.05     88.17   102.08   5.000 N/A    0.4210 kg  <- stroke top
//    50   130.44   142.44    101.16    88.96   4.358        0.3198
//   51.53 127.72   139.72    102.62    87.70   4.296        0.3109     <- where Gbar lands
//    55   121.42   133.42    105.34    85.44   4.185        0.2949
//   63.44 105.70   117.70    108.20    83.18   4.074        0.2795     <- Jacobian PEAK
//    70    93.31   105.31    106.37    84.61   4.145        0.2892     <- stroke bottom
// (F/I at Kt = 0.026621 and eta = 0.92; added mass = 2*J_rotor*G^2, i.e. per leg
//  with BOTH motors reflected.) The F/I column uses the pre-M1 Kt: at J01's
// calKt() it is +1.1% per true amp, +5.1% per reported amp. Design-level; M14
// replaces it.
//
// NON-MONOTONIC: G peaks at alpha 63.44 deg, so force per amp is LOWEST
// mid-low stroke and rises at both ends. G swings 22.7% (83.18-102.08): peak
// thrust 487-597 N, force per amp 4.07-5.00 N/A, added mass (4 legs)
// 1.118-1.684 kg -- worst at the stroke top, where a jump leaves the ground.
// h_ground = h_pin + 12 mm contact offset (a vertical shift; dh/da unchanged).
// Confirm from CAD that the contact point translates with the pin rather than
// rotating with a link; if it rotates, every G above changes.
static constexpr float G_FOOT_PER_MOTOR_NM   = 87.70f;   // 1/m, STROKE MEAN over alpha 40-70
static constexpr float G_FOOT_AT_STROKE_TOP  = 102.08f;  // 1/m, alpha 40 deg -- use for PEAK force
static constexpr float G_FOOT_AT_STROKE_MIN  = 83.18f;   // 1/m, alpha 63.44 deg -- weakest point
static constexpr float LEG_L1_PROX_MM        = 80.0f;    // proximal link
static constexpr float LEG_L2_DIST_MM        = 100.0f;   // distal link
static constexpr float LEG_ALPHA_MIN_DEG     = 40.0f;    // stroke TOP,    h_pin 147.05 mm
static constexpr float LEG_ALPHA_MAX_DEG     = 70.0f;    // stroke BOTTOM, h_pin  93.31 mm
static constexpr float LEG_CONTACT_OFFSET_MM = 12.0f;    // foot pin -> ground. Shifts h, not dh/da

// ---------------------------------------------------------------------------
// DRIVETRAIN EFFICIENCY -- BACK-SOLVED, NOT MEASURED (circular)
// ---------------------------------------------------------------------------
// 0.92 was back-solved from A1's 4.5 N, which was itself computed with eta.
// Kept because every existing force figure assumes it (consistency, not truth).
// M14 (load cell on an assembled leg) replaces it with a lumped G*eta*Kt.
// DO NOT DOUBLE-COUNT: eta is the load-dependent belt loss; drag_c / drag_v
// are the load-independent loss. Feed footForce*() a torque with drag already
// subtracted.
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
