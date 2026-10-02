#pragma once
// ============================================================================
// mit_types.h -- THE TIER-0 ACTUATOR CONTRACT, as data. (B12a, 2026-10-01)
// ============================================================================
// One header both tiers include (the gait_types.h pattern). Deliberately has
// NO includes and NO logic: Tier 1 (ESP32-S3) must be able to include it
// without dragging in SimpleFOC, Arduino-STM32 or any calibration table.
//
// FRAME AND UNITS -- output side, SI, units in every name:
//   p      rad      OUTPUT angle (after the 9:1). GEAR_RATIO stays inside Tier 0.
//   v      rad/s    OUTPUT speed.
//   tau    N.m      OUTPUT torque, DEFINED as GEAR_RATIO * Kt * I_true.
//                   DRIVETRAIN_ETA is EXCLUDED and friction is NOT subtracted
//                   (fleet_config.h, the 2026-10-01 unit rule). M14 may later
//                   refine a separate estimate; this definition does not move.
//   kp     N.m/rad      output-referred
//   kd     N.m.s/rad    output-referred
//
// SIGN: positive = the motor's positive electrical direction (SimpleFOC
// sensor_direction applied). A belt does not reverse direction, so the output
// turns the same way. Which PHYSICAL direction that is gets written down at
// B12a step a2. A per-joint MOUNTING sign for mirrored legs is NOT applied
// here -- open contract item, decide before the freeze (README 15, 7l).
//
// ZERO: SESSION-RELATIVE for B12 -- p = 0 at the moment the joint is armed.
// The MT6816 is absolute over one MOTOR turn = 40 deg of output, so an absolute
// output angle needs homing or a known boot pose. Open item 7l; until it is
// closed this contract is NOT frozen.
//
// THE LAW Tier 0 applies with these numbers (mit_law.h):
//     tau = kp*(p_des - p) + kd*(v_des - v) + tau_ff
// then joint_cal.h's tauOutCmdToIq(): clamp tau_max [N.m] -> convert ->
// clamp the demonstrated envelope [A_rep]. There is no velocity PID.
// ============================================================================

struct MitCmd {
  float p_des_rad;
  float v_des_rads;
  float kp_Nm_per_rad;
  float kd_Nms_per_rad;
  float tau_ff_Nm;
};

struct MitState {
  float p_rad;
  float v_rads;
  float tau_Nm;     // MEASURED: irepToTorqueOut(Iq), the feedback half of the boundary
};
