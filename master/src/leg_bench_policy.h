#pragma once
#include <math.h>
#include <stdint.h>
#include "can_proto.h"

// Output radians. This policy is pure so the same state machine is host-tested.
struct LegJointConfig {
  uint8_t node; int8_t sign; float q_ref, cmd_min, cmd_max, trip_min, trip_max;
};
struct LegBenchConfig {
  bool configured; LegJointConfig joint[2];
  float kp_max, kd_max, rate_max, arm_speed_max;
  uint32_t fresh_ms, transaction_ms, missed_max; float move_max_s; uint32_t control_gap_ms;
};
struct LegSample { bool seen; uint32_t age_ms, missed, control_age_ms; float p,v; uint8_t status; };
struct LegObservation { LegSample node[2]; uint8_t admin_reply_mask, admin_failed_mask; };
enum LegPhase : uint8_t { LEG_UNREFERENCED, LEG_ZEROING, LEG_READY, LEG_ARMING,
                         LEG_ARMED, LEG_DISARMING, LEG_FAULT };
static inline const char* legPhaseName(LegPhase p) {
  switch(p){case LEG_UNREFERENCED:return "UNREFERENCED";case LEG_ZEROING:return "ZEROING";
    case LEG_READY:return "REFERENCED";case LEG_ARMING:return "ARMING";case LEG_ARMED:return "ARMED";
    case LEG_DISARMING:return "DISARMING";case LEG_FAULT:return "FAULT";default:return "INVALID";}
}
enum LegRequestKind : uint8_t { LEG_NONE, LEG_ZERO, LEG_ARM, LEG_GAINS, LEG_MOVE, LEG_DISARM, LEG_ABORT };
struct LegRequest { LegRequestKind kind; float a,b,duration_s; bool from_phone; };
struct LegActions { uint8_t admin_mask, admin_op; bool estop,rejected; const char* reason; };
struct LegSession {
  LegPhase phase; uint8_t acknowledged; uint32_t transaction_at;
  MitCmd cmd[2]; float hold_q[2], start_q[2], goal_q[2], kp,kd;
  bool moving,gains_set,referenced; uint32_t move_at,move_ms;
};

static inline bool legConfigValid(const LegBenchConfig& c) {
  if (!c.configured || c.joint[0].node!=1 || c.joint[1].node!=3 ||
      !isfinite(c.kp_max) || c.kp_max<=0 || !isfinite(c.kd_max) || c.kd_max<0 ||
      !isfinite(c.rate_max) || c.rate_max<=0 || !isfinite(c.arm_speed_max) || c.arm_speed_max<=0 ||
      !isfinite(c.move_max_s) || c.move_max_s<=0 || c.move_max_s>60 ||
      !c.fresh_ms || !c.transaction_ms || !c.missed_max || !c.control_gap_ms || c.control_gap_ms>=c.fresh_ms) return false;
  for (int i=0;i<2;i++) {
    const auto& j=c.joint[i];
    if ((j.sign!=1 && j.sign!=-1) || !isfinite(j.q_ref) ||
        !isfinite(j.cmd_min) || !isfinite(j.cmd_max) || !isfinite(j.trip_min) || !isfinite(j.trip_max) ||
        !(j.trip_min<j.cmd_min && j.cmd_min<j.q_ref && j.q_ref<j.cmd_max && j.cmd_max<j.trip_max)) return false;
  }
  return true;
}
static inline float legQFromWire(const LegJointConfig& c,float p) { return c.q_ref+c.sign*p; }
static inline bool legWireTargets(const LegBenchConfig& c,const float q[2],float kp,float kd,MitCmd out[2]) {
  if (!legConfigValid(c) || !isfinite(kp) || kp<0 || kp>c.kp_max || !isfinite(kd) || kd<0 || kd>c.kd_max) return false;
  MitCmd tmp[2];
  for (int i=0;i<2;i++) {
    const auto& j=c.joint[i];
    if (!isfinite(q[i]) || q[i]<j.cmd_min || q[i]>j.cmd_max) return false;
    const MitCmd raw={j.sign*(q[i]-j.q_ref),0,kp,kd,0};
    uint8_t bytes[8]; if (!canPackCmd(raw,bytes)) return false;
    canUnpackCmd(bytes,tmp[i]);
    const float decoded=legQFromWire(j,tmp[i].p_des_rad);
    if (decoded<j.cmd_min || decoded>j.cmd_max || tmp[i].kp_Nm_per_rad>c.kp_max || tmp[i].kd_Nms_per_rad>c.kd_max) return false;
  }
  out[0]=tmp[0]; out[1]=tmp[1]; return true;
}
static inline bool legFresh(const LegBenchConfig& c,const LegSample& n) {
  return n.seen && n.age_ms<=c.fresh_ms && n.missed<c.missed_max &&
    canStPhase(n.status)<=CAN_PHASE_DAMP && isfinite(n.p) && isfinite(n.v);
}
static inline bool legCapture(const LegBenchConfig& c,const LegObservation& o,float q[2],MitCmd cmd[2],float kp,float kd) {
  for (int i=0;i<2;i++) {
    q[i]=legQFromWire(c.joint[i],o.node[i].p);
    if (!legFresh(c,o.node[i]) || fabsf(o.node[i].v)>c.arm_speed_max ||
        q[i]<c.joint[i].cmd_min+CAN_LSB_P_DES || q[i]>c.joint[i].cmd_max-CAN_LSB_P_DES) return false;
  }
  if (!legWireTargets(c,q,kp,kd,cmd)) return false;
  for(int i=0;i<2;i++) {
    const float decoded=legQFromWire(c.joint[i],cmd[i].p_des_rad);
    if(decoded<c.joint[i].cmd_min+CAN_LSB_P_DES || decoded>c.joint[i].cmd_max-CAN_LSB_P_DES) return false;
  }
  return true;
}
static inline void legFail(LegSession& s,LegActions& a,const char* why) {
  s.phase=LEG_FAULT; s.moving=false; s.gains_set=false; s.referenced=false; s.kp=s.kd=0;
  a.estop=true; a.reason=why;
}
static inline void legReject(LegActions& a,const char* why) { a.rejected=true; a.reason=why; }

static inline void legSessionStep(LegSession& s,const LegBenchConfig& c,const LegObservation& o,
                                 const LegRequest* request,uint32_t now,LegActions& a) {
  a={};
  if (request && request->kind==LEG_ABORT) { legFail(s,a,"operator/safety stop"); return; }
  const bool active=s.phase!=LEG_UNREFERENCED && s.phase!=LEG_FAULT;
  for(int i=0;i<2;i++) {
    const auto& n=o.node[i]; const bool is_armed=canStArmed(n.status);
    if (active && !legFresh(c,n)) { legFail(s,a,"joint state lost/stale"); return; }
    if (active && canStFault(n.status)) { legFail(s,a,"joint fault"); return; }
    if (s.phase==LEG_ARMED && (!is_armed || canStPhase(n.status)==CAN_PHASE_DAMP)) {
      legFail(s,a,"joint disarmed or DAMP"); return;
    }
    if(s.phase==LEG_ARMED && n.control_age_ms>c.control_gap_ms) {
      legFail(s,a,"control gap: do not resume old trajectory");return;
    }
    if (s.phase==LEG_ARMING && (s.acknowledged&(1u<<i)) && !is_armed) {
      legFail(s,a,"joint dropped during paired ARM"); return;
    }
    if ((s.phase==LEG_READY || s.phase==LEG_UNREFERENCED || s.phase==LEG_ZEROING || s.phase==LEG_FAULT) && legFresh(c,n) && is_armed) {
      legFail(s,a,"joint armed outside pair session"); return;
    }
    if ((s.phase==LEG_ARMED || s.phase==LEG_ARMING) && legConfigValid(c)) {
      const float q=legQFromWire(c.joint[i],n.p);
      if(q<c.joint[i].trip_min || q>c.joint[i].trip_max) { legFail(s,a,"actual position outside trip box"); return; }
    }
  }
  const bool transaction=s.phase==LEG_ZEROING || s.phase==LEG_ARMING || s.phase==LEG_DISARMING;
  if(transaction) {
    if(o.admin_failed_mask) { legFail(s,a,"paired admin exchange failed"); return; }
    for(int i=0;i<2;i++) if(o.admin_reply_mask&(1u<<i)) {
      const auto& n=o.node[i];
      const bool accepted=legFresh(c,n) && !canStFault(n.status) &&
        (s.phase==LEG_ARMING ? canStArmed(n.status) : !canStArmed(n.status)) &&
        (s.phase!=LEG_ZEROING || fabsf(n.p)<=2*CAN_LSB_P);
      if(!accepted) { legFail(s,a,"joint refused paired admin"); return; }
      s.acknowledged|=(1u<<i);
    }
    if(s.acknowledged==3) {
      if(s.phase==LEG_ARMING) {
        if(!legCapture(c,o,s.hold_q,s.cmd,0,0)) { legFail(s,a,"ARM pose changed outside command box"); return; }
        s.phase=LEG_ARMED;
      } else {
        if(s.phase==LEG_ZEROING) s.referenced=true;
        s.phase=s.referenced?LEG_READY:LEG_UNREFERENCED;
      }
    } else if(now-s.transaction_at>=c.transaction_ms) { legFail(s,a,"paired admin timeout"); return; }
  }
  if(s.phase==LEG_ARMED && s.moving) {
    const uint32_t elapsed=now-s.move_at;
    const float fraction=elapsed>=s.move_ms?1.0f:(float)elapsed/s.move_ms;
    float q[2]; for(int i=0;i<2;i++) q[i]=s.start_q[i]+fraction*(s.goal_q[i]-s.start_q[i]);
    if(!legWireTargets(c,q,s.kp,s.kd,s.cmd)) { legFail(s,a,"generated target invalid"); return; }
    s.hold_q[0]=q[0]; s.hold_q[1]=q[1]; if(fraction>=1) s.moving=false;
  }
  if(!request || request->kind==LEG_NONE) return;
  const auto& r=*request;
  if(r.kind==LEG_DISARM) {
    // Even when unreferenced, request acknowledged disable on both nodes.
    s.phase=LEG_DISARMING; s.moving=false; s.gains_set=false; s.kp=s.kd=0;
    s.acknowledged=0; s.transaction_at=now; a.admin_mask=3; a.admin_op=CAN_OP_DISARM; return;
  }
  if(s.phase==LEG_ZEROING || s.phase==LEG_ARMING || s.phase==LEG_DISARMING) {
    legReject(a,"paired transaction busy"); return;
  }
  if(r.kind==LEG_ZERO) {
    for(int i=0;i<2;i++) if(!legFresh(c,o.node[i]) || canStArmed(o.node[i].status) ||
       canStFault(o.node[i].status) || fabsf(o.node[i].v)>c.arm_speed_max) {
      legReject(a,"ZERO needs both healthy stationary disarmed joints"); return;
    }
    s={}; s.phase=LEG_ZEROING; s.transaction_at=now;
    a.admin_mask=3; a.admin_op=CAN_OP_ZERO; return;
  }
  if(r.kind==LEG_ARM) {
    if(s.phase!=LEG_READY || !s.referenced || !legCapture(c,o,s.hold_q,s.cmd,0,0)) {
      legReject(a,"ARM needs configured reference and stationary in-box pose"); return;
    }
    s.phase=LEG_ARMING; s.gains_set=false; s.kp=s.kd=0; s.moving=false;
    s.acknowledged=0; s.transaction_at=now; a.admin_mask=3; a.admin_op=CAN_OP_ARM; return;
  }
  if(s.phase!=LEG_ARMED || s.moving) { legReject(a,"needs armed pair with no move active"); return; }
  if(r.kind==LEG_GAINS) {
    float q[2]={s.hold_q[0],s.hold_q[1]}; MitCmd cmd[2];
    if(!s.gains_set && !legCapture(c,o,q,cmd,r.a,r.b)) { legReject(a,"gain capture pose/gains invalid"); return; }
    if(!legWireTargets(c,q,r.a,r.b,cmd)) { legReject(a,"gains invalid"); return; }
    s.cmd[0]=cmd[0]; s.cmd[1]=cmd[1]; s.hold_q[0]=q[0]; s.hold_q[1]=q[1];
    s.kp=r.a; s.kd=r.b; s.gains_set=true; return;
  }
  if(r.kind==LEG_MOVE) {
    const float q[2]={r.a,r.b}; MitCmd cmd[2];
    if(!s.gains_set || s.kp<=0 || !isfinite(r.duration_s) || r.duration_s<=0 || r.duration_s>c.move_max_s ||
       !legWireTargets(c,q,s.kp,s.kd,cmd)) { legReject(a,"move targets/duration/gains invalid"); return; }
    const uint32_t ms=(uint32_t)ceilf(r.duration_s*1000.0f);
    for(int i=0;i<2;i++) if(fabsf(q[i]-s.hold_q[i])>c.rate_max*(ms*0.001f)+1e-7f) {
      legReject(a,"move exceeds planned rate"); return;
    }
    for(int i=0;i<2;i++) { s.start_q[i]=s.hold_q[i]; s.goal_q[i]=q[i]; }
    s.move_at=now; s.move_ms=ms; s.moving=true; return;
  }
  legReject(a,"unknown leg request");
}
