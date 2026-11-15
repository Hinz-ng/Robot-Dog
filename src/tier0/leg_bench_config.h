#pragma once
#include <math.h>
#include "can_proto.h"

// Independent Tier-0 cutoffs in raw CONTRACT output radians, not leg-frame q.
// User-observed signs +1 for both; 2026-11-07 widened outer envelope request.
// Match master: +/-120 deg commands, +/-125 deg actual cutoff; manual ZERO.
// A per-joint box does not establish linked-mechanism collision clearance.
struct T0LegPolicy {
  bool configured; float cmd_min,cmd_max,trip_min,trip_max;
  float arm_speed_output,max_speed_output,kp_max,kd_max;
};
struct T0LegReference {
  bool zeroed;
  void onAdmin(uint8_t op,bool running) { if(op==CAN_OP_ZERO && !running) zeroed=true; }
};
#if JOINT_ID == 1
static const T0LegPolicy T0_LEG_POLICY={true,-2.0943951f,2.0943951f,-2.1816616f,2.1816616f,0.05f,0.5f,10.0f,0.1f};
#elif JOINT_ID == 3
static const T0LegPolicy T0_LEG_POLICY={true,-2.0943951f,2.0943951f,-2.1816616f,2.1816616f,0.05f,0.5f,10.0f,0.1f};
#else
#error "Leg bench profile supports J01 and J03 only"
#endif
static inline bool t0LegConfigured(const T0LegPolicy& c) {
  return c.configured && isfinite(c.cmd_min) && isfinite(c.cmd_max) &&
    isfinite(c.trip_min) && isfinite(c.trip_max) &&
    c.trip_min<c.cmd_min && c.cmd_min<0 && 0<c.cmd_max && c.cmd_max<c.trip_max &&
    isfinite(c.arm_speed_output) && c.arm_speed_output>0 &&
    isfinite(c.max_speed_output) && c.max_speed_output>c.arm_speed_output &&
    isfinite(c.kp_max) && c.kp_max>0 && isfinite(c.kd_max) && c.kd_max>=0;
}
// A linked-leg profile never automatically resumes a trajectory after DAMP.
// Keep normal Tier-0's approved position-margin resume behavior unchanged.
static inline bool t0LegMayRefresh(uint8_t phase) { return phase!=CAN_PHASE_DAMP; }
static inline bool t0LegArmAllowed(const T0LegPolicy& c,const T0LegReference& ref,float p,float v) {
  if(!t0LegConfigured(c) || !ref.zeroed || !isfinite(p) || !isfinite(v) ||
     fabsf(v)>c.arm_speed_output || p<c.cmd_min+CAN_LSB_P_DES || p>c.cmd_max-CAN_LSB_P_DES) return false;
  // The first hold will itself be quantized on the wire.
  const MitCmd raw={p,0,0,0,0}; MitCmd decoded; uint8_t bytes[8];
  if(!canPackCmd(raw,bytes)) return false; canUnpackCmd(bytes,decoded);
  return decoded.p_des_rad>=c.cmd_min+CAN_LSB_P_DES && decoded.p_des_rad<=c.cmd_max-CAN_LSB_P_DES;
}
static inline bool t0LegActualInBounds(const T0LegPolicy& c,float p) {
  return t0LegConfigured(c) && isfinite(p) && p>=c.trip_min && p<=c.trip_max;
}
static inline bool t0LegCmdAllowed(const T0LegPolicy& c,const MitCmd& cmd) {
  return t0LegConfigured(c) && isfinite(cmd.p_des_rad) && cmd.p_des_rad>=c.cmd_min && cmd.p_des_rad<=c.cmd_max &&
    isfinite(cmd.kp_Nm_per_rad) && cmd.kp_Nm_per_rad>=0 && cmd.kp_Nm_per_rad<=c.kp_max &&
    isfinite(cmd.kd_Nms_per_rad) && cmd.kd_Nms_per_rad>=0 && cmd.kd_Nms_per_rad<=c.kd_max &&
    cmd.v_des_rads==0.0f && cmd.tau_ff_Nm==0.0f;
}
