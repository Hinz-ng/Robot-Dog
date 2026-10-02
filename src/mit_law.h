#pragma once
#include <Arduino.h>
#include "fleet_config.h"
#include "mit_types.h"
// ============================================================================
// mit_law.h -- the MIT law and its state estimator. (B12a, 2026-10-01)
// ============================================================================
// PURE: no serial, no SimpleFOC objects, no harness policy. Inputs are the raw
// encoder count, the sensor direction, a timestamp and the command; outputs are
// numbers. That is what lets Tier 0 reuse this file unchanged while the bench
// harness keeps its UI, capture and limits to itself. Limits are PASSED IN.
//
// The command half of the torque boundary is NOT here: the law produces an
// output torque and the caller sends it through joint_cal.h's tauOutCmdToIq().
// ============================================================================

// ---------------------------------------------------------------------------
// STATE ESTIMATOR -- p and v from RAW COUNTS, output frame
// ---------------------------------------------------------------------------
// WHY RAW COUNTS AND NOT motor.shaft_angle / shaft_velocity:
//   * shaft_velocity is LPF_velocity-filtered at VEL_TF = 20 ms -- an 8 Hz
//     corner, ~72 deg late at 25 Hz. kd acting on it ADDS energy. The harness
//     logs and the other modes rely on VEL_TF, so it is left untouched and the
//     MIT law gets its own filter, Tf_mit, chosen by measurement (B12a a0).
//   * shaft_angle ALREADY carries sensor_direction (FOCMotor::shaftAngle(),
//     SimpleFOC 2.3.1 FOCMotor.cpp:64). Applying `dir` to it again would flip
//     the sign twice -- invisible on J01 and J02 (both dir = +1), wrong on the
//     first dir = -1 joint. Here `dir` is applied ONCE, to raw counts.
//   * Differencing an accumulated float angle loses resolution as the angle
//     grows (at ~1000 rad one float ulp is ~1/6 of a count). Integer counts
//     do not.
// Quantisation sets the noise floor: 1 count = 3.835e-4 rad, sigma_v ~=
// q / (sqrt(12) * Tf) = 0.11 rad/s at the MOTOR for Tf = 1 ms, while moving.
// At REST the count does not flicker (span 0), so the floor is NOT measurable
// stationary -- which is why a0 measures it in slow steady motion.
struct MitEstimator {
  int32_t  cnt;          // unwrapped motor counts, SENSOR frame, since prime
  int32_t  cnt_zero;     // cnt at the session zero (arm)
  uint16_t raw_prev;
  float    v_out_rads;   // filtered, OUTPUT frame, sign applied
  bool     primed;
};

static inline void mitEstPrime(MitEstimator& e, uint16_t raw) {
  e.cnt = 0; e.cnt_zero = 0; e.raw_prev = raw;
  e.v_out_rads = 0.0f; e.primed = true;
}

// Call once per control loop, AFTER loopFOC() has refreshed the encoder.
// dt_s is the time since the previous call, measured by the CALLER on whatever
// clock it has (the bench harness uses the DWT cycle counter: one register read,
// cycle resolution -- micros() cost a call chain and quantised dt to 1 us,
// ~1% of a loop). tf_s is the first-order time constant; the form
// a = dt/(Tf+dt) is SimpleFOC's LowPassFilter, so a0's offline simulation
// matches it.
static inline void mitEstUpdate(MitEstimator& e, uint16_t raw, int8_t dir,
                                float dt, float tf_s) {
  if (!e.primed) { mitEstPrime(e, raw); return; }
  int32_t d = (int32_t)raw - (int32_t)e.raw_prev;
  if      (d >  (int32_t)(ENC_CPR / 2)) d -= (int32_t)ENC_CPR;   // wrap through 0
  else if (d < -(int32_t)(ENC_CPR / 2)) d += (int32_t)ENC_CPR;
  e.raw_prev = raw;
  e.cnt += d;
  if (!(dt > 0.0f)) return;
  const float v_raw = (float)(dir * d) * ENC_RAD_PER_COUNT / (GEAR_RATIO * dt);
  const float a = dt / (tf_s + dt);
  e.v_out_rads += a * (v_raw - e.v_out_rads);
}

static inline void mitEstZero(MitEstimator& e) { e.cnt_zero = e.cnt; }

static inline float mitEstP(const MitEstimator& e, int8_t dir) {
  return (float)(dir * (e.cnt - e.cnt_zero)) * ENC_RAD_PER_COUNT / GEAR_RATIO;
}

// ---------------------------------------------------------------------------
// THE LAW. Output torque, BEFORE any clamp. There is no velocity PID and no
// integrator, so nothing winds up while a clamp binds.
// ---------------------------------------------------------------------------
static inline float mitLawTau(const MitCmd& c, const MitState& s) {
  return c.kp_Nm_per_rad  * (c.p_des_rad  - s.p_rad)
       + c.kd_Nms_per_rad * (c.v_des_rads - s.v_rads)
       + c.tau_ff_Nm;
}

// ---------------------------------------------------------------------------
// COMMAND VALIDATION -- reject, never repair. A negative or non-finite gain is
// a caller bug; silently taking fabsf() of it would hide the bug and still
// move the motor. Ranges are the CALLER's policy and are passed in.
// ---------------------------------------------------------------------------
struct MitRanges {
  float kp_max;       // N.m/rad
  float kd_max;       // N.m.s/rad
  float p_abs_max;    // rad
  float v_abs_max;    // rad/s
  float tau_abs_max;  // N.m (tau_ff only -- the clamp chain bounds the total)
};

static inline bool mitCmdValid(const MitCmd& c, const MitRanges& r) {
  if (!isfinite(c.p_des_rad) || !isfinite(c.v_des_rads) || !isfinite(c.kp_Nm_per_rad)
      || !isfinite(c.kd_Nms_per_rad) || !isfinite(c.tau_ff_Nm)) return false;
  if (c.kp_Nm_per_rad  < 0.0f || c.kp_Nm_per_rad  > r.kp_max) return false;
  if (c.kd_Nms_per_rad < 0.0f || c.kd_Nms_per_rad > r.kd_max) return false;
  if (fabsf(c.p_des_rad)  > r.p_abs_max)   return false;
  if (fabsf(c.v_des_rads) > r.v_abs_max)   return false;
  if (fabsf(c.tau_ff_Nm)  > r.tau_abs_max) return false;
  return true;
}
