#pragma once
#include <Arduino.h>
#include "fleet_config.h"
#include "mit_law.h"      // MitRanges
// ============================================================================
// tier0_config.h -- TIER 0's SHIPPED POLICY. (CAN-T0, 2026-10-03)
// ============================================================================
// The FOURTH home for a constant (CLAUDE.md's table lists three):
//   fleet_config.h   identical on every joint by construction
//   joint_cal.h      per unit, measured on one assembly
//   open_test.cpp    the BENCH HARNESS's tuning and envelope -- not shipped
//   tier0_config.h   what the SHIPPED joint firmware enforces: envelope,
//                    command validation, timeout policy, guards
// Where a value is copied from open_test.cpp it is copied WITH its provenance,
// and the two are independent from here on: changing the harness changes
// nothing in Tier 0, by design. Gains/envelopes are per-PLANT -- mounting the
// leg invalidates them (CLAUDE.md); these are the bench-proven values the leg
// starts from.
// ============================================================================

// ---- DRIVE -- copied from open_test.cpp, unchanged ----
// Modulation reference (NOT a safety limit): sets Uq_ceil = rail/sqrt(3)
// = 3.46 V. Raise before any high-speed work (README 8.3) -- not needed for
// a hanging leg.
const float T0_DRIVER_VOLT_LIMIT = 6.0f;
// Uq clamp. At 2.0 V into R_eff ~0.22 ohm a rail-pinned fault is ~9 A; the
// sense-mismatch guard is the backstop. Also a speed ceiling: ~81-91 rad/s at
// the MOTOR at bench voltage = ~9-10 rad/s at the output. Ample for N0-N6.
const float T0_VOLT_LIMIT        = 2.0f;
// Current PI -- M1-e, 412 Hz, 80% of the transport-delay ceiling. R and L
// survive leg mounting (CLAUDE.md), so these do too.
const float T0_CURQ_P = 0.1f, T0_CURQ_I = 335.0f;
const float T0_CURD_P = 0.1f, T0_CURD_I = 335.0f;
const float T0_CUR_TF = 0.00025f;   // corner 5-10x above loop bandwidth (measured)
// motor.current_limit is INERT in torque/foc_current (open_test.cpp's
// CURR_MAX note: move() assigns current_sp = target unclamped). Set to the
// envelope so the CFG banner's Ilim prints the number that matters.

// ---- BUS VOLTAGE SEED -- copied; actuator_hw.h needs these exact names ----
// actuator_hw.h initialises vbus_filt from VBUS_FALLBACK at file scope, so this
// header MUST be included before it (a hidden includer dependency, recorded).
const uint32_t T0_PIN_VBUS   = PA0;
const float    VBUS_MIN      = 8.0f;    // plausibility window, NOT undervoltage
const float    VBUS_MAX      = 30.0f;   //   protection
const float    VBUS_FALLBACK = 11.30f;  // measured bench pack

// ---- ENVELOPE -- copied from open_test.cpp's C2 block. MOVES AFTER B10'. ----
// INNER: the DEMONSTRATED-safe current, D3's 1.6 A_rep, proven by the swing
// ladder in REPORTED amps. Measured Iq overshoots the clamped command by up to
// 6% (a5, 1.69 vs 1.60) -- any future raise sits >= 6% below the proven value.
const float T0_ENV_A_rep  = 1.6f;
// OUTER, contract units: just under the inner clamp's 0.403 N.m (J01), so the
// outer binds first and an inner hit is a conversion-vs-envelope finding.
// NOTE: 0.39 N.m output is BELOW the ~0.58 N.m a joint carries standing at
// full body weight (9.8 N/leg / 4.30 N/A = 2.3 A). Hanging leg: fine.
// Loaded stance: gated on B10'.
const float T0_TAU_MAX_Nm = 0.39f;

// ---- ESTIMATOR ----
const float T0_MIT_TF_S = 0.001f;   // Tf_mit, B12a a0 (noise 70% of budget, 8.9 deg at 25 Hz)

// ---- COMMAND VALIDATION -- reject, never repair (mit_law.h) ----
// kp/kd = the DEMONSTRATED gain box (b1, clamped output): kp <= 41, kd <= 0.365.
// Not the harness's 200/2 typo-catcher: Tier 0 refuses what has not been shown
// stable. Raise only with a leg test (deferred item, b1).
// |p_des| 2*pi matches the CAN packing range; |v_des| 5 rad/s (45 at the motor,
// inside the Uq ceiling); |tau_ff| 1 N.m (the clamp chain bounds the total).
const MitRanges T0_RANGES = { 41.0f, 0.365f, 6.2832f, 5.0f, 1.0f };

// ---- COMMAND TIMEOUT: HOLD -> DAMP -> DISABLE. Owner-approved 2026-10-03. ----
// Checked EVERY LOOP on the local clock, never from reception, so a dead bus
// fires it.
//   HOLD  last command held          until T_HOLD after the last valid CMD
//   DAMP  kp 0, ff 0, v_des 0, kd = KD_TIMEOUT    until T_OFF
//   OFF   stopMotor("CAN timeout"), fault latched; no auto-resume, ever
// LIVE -> HOLD is REPORTING ONLY (status phase bits): a command older than
// this is late but still applied. 8 ms = 1.6 cycles at 200 Hz.
const uint32_t T0_T_LATE_MS = 8;
const uint32_t T0_T_HOLD_MS = 15;     // 3 cycles at the 200 Hz bench schedule
const uint32_t T0_T_OFF_MS  = 500;
// kd in DAMP. Sag speed ~ load torque / kd: 0.58 N.m standing load at kd 0.1
// would drop at ~5.8 rad/s (a tenth of a second); at 1.0 it is ~0.6 rad/s.
// *** OUTSIDE THE DEMONSTRATED kd <= 0.365 BOX *** -- internal, never received,
// so the validator does not see it; N4a demonstrates it (no buzz, no limit
// cycle). Under today's envelope the damping torque itself clamps at 0.39 N.m.
const float T0_KD_TIMEOUT = 1.0f;
// Resuming from DAMP needs a valid CMD whose p_des is within this of the actual
// p, so a stale target cannot snap the leg. 0.05 rad output = 2.9 deg. From
// HOLD a valid CMD resumes unconditionally.
const float T0_P_RESUME_MARGIN_RAD = 0.05f;

// ---- GUARDS -- every trip goes through stopMotor() (safety.h) ----
// Overspeed, in MOTOR rad/s (the harness's unit; 150 motor = 16.7 output).
// *** UNIT: Tier 0's speed is the OUTPUT-frame estimate; multiply by
// GEAR_RATIO before comparing. *** Raise before the first fast swing.
const float    T0_OVERSPEED_MOTOR_RADS = 150.0f;
const uint32_t T0_OVERSPEED_GRACE_MS   = 300;    // ignore right after arming
// Sense mismatch -- copied verbatim from open_test.cpp's loop() guard. Both
// thresholds compare against sensed |I| and are REPORTED amps by construction.
const uint8_t  T0_GUARD_DIV        = 30;    // check every 30 loops (~2.5 ms)
const uint8_t  T0_GUARD_HITS       = 8;     // ~20 ms of sustained mismatch
const float    T0_GUARD_PAD_A_rep   = 0.6f;
const float    T0_GUARD_FLOOR_A_rep = 3.0f;
// Position envelope relative to the session zero: the CAN packing range.
// Joint limits for the leg belong to the master (it owns geometry).
const float    T0_P_ABS_MAX_RAD = 6.2832f;
// Encoder link: all-zero frames for this long = dead/stuck-low MISO (mt6816.h
// zero_run). Refuses ARM; trips an armed joint (CAN_FAULT_ENCODER).
// AMBIGUOUS BY CONSTRUCTION: a LIVE encoder parked at exactly count 0 sends the
// same word and does not jitter at rest ('e' passes only at span 0), so 1 rotor
// position in 16384 reads as dead. Disarmed that costs a refused ARM (turn the
// shaft a few degrees). Armed it would be a false trip IF the held rotor shows
// no count jitter for the whole window -- NOT YET MEASURED (check: hold armed
// with the shaft on raw 0 for 10 s; any trip = jitter-free hold). A missed dead line costs a frozen commutation
// vector, bounded by the 1.6 A_rep envelope, so the window can be long: 200 ms
// is ~2400 armed loops, and harm over it is bounded, not growing.
const uint32_t T0_ENC_ZERO_MS = 200;

// ---- HANG PROTECTION (owner addition 1) ----
// IWDG on LSI (~32 kHz, spec 29.5-34 kHz): prescaler /4 -> 8 kHz tick.
// Reload 160 -> 20 ms nominal, >= 18.8 ms at the fast LSI limit. The loop
// period is ~85 us and the worst disarmed stall is well under 10 ms, so a
// reset means the loop really stopped. Started only after setup completes.
const uint32_t T0_IWDG_PRESCALER_CODE = 0;      // IWDG_PR = 0 -> /4
const uint32_t T0_IWDG_RELOAD         = 160;    // 20 ms nominal at 32 kHz / 4

// ---- CAN / CONSOLE ----
const uint8_t  T0_CAN_RX_PER_LOOP = 4;     // bounded drain per FOC loop
const uint32_t T0_STATUS_MS       = 500;   // DISARMED only -- nothing prints while armed
