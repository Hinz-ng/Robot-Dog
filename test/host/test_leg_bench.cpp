#include <cstdio>
#include <cmath>
#include "../../master/src/leg_bench_policy.h"

static int failed=0, checks=0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failed; std::printf("FAIL line %d: %s\n", __LINE__, #x); } } while (0)
static LegBenchConfig config() {
  LegBenchConfig c={}; c.configured=true;
  c.joint[0]={1,+1,0,-0.035f,0.035f,-0.070f,0.070f};
  c.joint[1]={3,-1,0,-0.035f,0.035f,-0.070f,0.070f};
  c.kp_max=10; c.kd_max=0.1f; c.rate_max=0.02f; c.arm_speed_max=0.05f;
  c.fresh_ms=50; c.transaction_ms=100; c.missed_max=3; c.move_max_s=10;
  c.control_gap_ms=10;
  return c;
}
static LegObservation observation(bool armed=false) {
  LegObservation o={};
  for (int i=0;i<2;i++) { o.node[i].seen=true; o.node[i].status=canStatusByte(armed,CAN_PHASE_LIVE,false,CAN_FAULT_NONE); }
  return o;
}
static LegActions run(LegSession& s, const LegBenchConfig& c, LegObservation o,
                      LegRequestKind kind=LEG_NONE, uint32_t now=0, float a=0,float b=0,float d=0) {
  LegRequest r={kind,a,b,d}; LegActions out={};
  legSessionStep(s,c,o,kind==LEG_NONE?nullptr:&r,now,out); return out;
}
static LegSession armed(const LegBenchConfig& c) {
  LegSession s={}; auto o=observation();
  CHECK(run(s,c,o,LEG_ZERO).admin_mask==3);
  o.admin_reply_mask=3; run(s,c,o); CHECK(s.phase==LEG_READY);
  o.admin_reply_mask=0; CHECK(run(s,c,o,LEG_ARM).admin_mask==3);
  o=observation(true); o.admin_reply_mask=3; run(s,c,o); CHECK(s.phase==LEG_ARMED);
  return s;
}
int main() {
  auto c=config(); MitCmd cmds[2]; float q[2]={0.010f,-0.020f};
  CHECK(legConfigValid(c)); CHECK(legWireTargets(c,q,2,0.02f,cmds));
  CHECK(cmds[0].p_des_rad>0 && cmds[1].p_des_rad>0);
  CHECK(std::fabs(legQFromWire(c.joint[1],0.02f)+0.02f)<1e-6f);
  for (int sign=-1;sign<=1;sign+=2) {
    c.joint[1].sign=sign; CHECK(legWireTargets(c,q,2,0.02f,cmds));
    CHECK(std::fabs(legQFromWire(c.joint[1],cmds[1].p_des_rad)-q[1])<=CAN_LSB_P_DES);
  }
  c=config(); auto bad=c; bad.configured=false; CHECK(!legConfigValid(bad));
  bad=c; bad.joint[1].sign=0; CHECK(!legConfigValid(bad));
  bad=c; bad.joint[1].node=1; CHECK(!legConfigValid(bad));
  bad=c; bad.joint[0].trip_min=0; CHECK(!legConfigValid(bad));
  bad=c; bad.rate_max=NAN; CHECK(!legConfigValid(bad));
  q[0]=NAN; CHECK(!legWireTargets(c,q,2,0.02f,cmds)); q[0]=0.010f;
  CHECK(!legWireTargets(c,q,10.1f,0.02f,cmds)); CHECK(!legWireTargets(c,q,2,-0.01f,cmds));
  q[1]=0.04f; CHECK(!legWireTargets(c,q,2,0.02f,cmds));
  bad=c; bad.joint[0].cmd_max=CAN_LSB_P_DES*0.6f;
  q[0]=bad.joint[0].cmd_max; q[1]=0; CHECK(!legWireTargets(bad,q,2,0.02f,cmds));

  LegSession s={}; auto o=observation();
  CHECK(run(s,c,o,LEG_ARM).rejected); CHECK(s.phase==LEG_UNREFERENCED);
  CHECK(run(s,c,o,LEG_ZERO).admin_mask==3);
  o.admin_reply_mask=1; run(s,c,o); CHECK(s.phase==LEG_ZEROING);
  o.admin_reply_mask=0; CHECK(run(s,c,o,LEG_NONE,100).estop); CHECK(s.phase==LEG_FAULT);
  // Clear/fault disappearance alone never creates a reference.
  run(s,c,observation()); CHECK(s.phase==LEG_FAULT);
  bad=c; bad.configured=false; s={}; run(s,bad,observation(),LEG_ZERO);
  o=observation(); o.admin_reply_mask=3; run(s,bad,o); CHECK(s.phase==LEG_READY);
  o.admin_reply_mask=0; CHECK(run(s,bad,o,LEG_ARM).rejected);

  s=armed(c); o=observation(true); o.node[0].p=0.015f;
  run(s,c,o,LEG_GAINS,10,2,0.02f);
  CHECK(s.gains_set); CHECK(std::fabs(s.cmd[0].p_des_rad-0.015f)<=CAN_LSB_P_DES);
  CHECK(run(s,c,o,LEG_MOVE,20,0.025f,0,0.1f).rejected); // Too fast.
  CHECK(!run(s,c,o,LEG_MOVE,20,0.025f,0,0.6f).rejected); CHECK(s.moving);
  CHECK(run(s,c,o,LEG_GAINS,21,3,0.02f).rejected);
  CHECK(run(s,c,o,LEG_MOVE,21,0,0,1).rejected);
  run(s,c,o,LEG_NONE,320); CHECK(s.cmd[0].p_des_rad>0.019f && s.cmd[0].p_des_rad<0.021f);
  run(s,c,o,LEG_NONE,620); CHECK(!s.moving);
  CHECK(std::fabs(s.cmd[0].p_des_rad-0.025f)<=CAN_LSB_P_DES);
  for (int faultCase=0;faultCase<7;faultCase++) {
    s=armed(c); o=observation(true);
    if(faultCase==0)o.node[1].status=canStatusByte(false,0,false,0);
    if(faultCase==1)o.node[1].status=canStatusByte(true,0,false,CAN_FAULT_ENCODER);
    if(faultCase==2)o.node[1].status=canStatusByte(true,CAN_PHASE_DAMP,false,0);
    if(faultCase==3)o.node[1].missed=3;
    if(faultCase==4)o.node[1].age_ms=51;
    if(faultCase==5)o.node[1].p=0.071f;
    if(faultCase==6)o.node[1].status=0x07; // Reserved phase, otherwise armed/fault-free.
    CHECK(run(s,c,o).estop); CHECK(s.phase==LEG_FAULT); CHECK(!s.moving);
  }
  s=armed(c); o=observation(true); o.node[0].status=canStatusByte(true,CAN_PHASE_HOLD,false,0);
  CHECK(!run(s,c,o).estop); CHECK(s.phase==LEG_ARMED);
  s=armed(c); o=observation(true); o.node[0].p=0.05f;
  CHECK(run(s,c,o,LEG_GAINS,0,2,0.02f).rejected);
  s=armed(c); o=observation(true); CHECK(run(s,c,o,LEG_DISARM).admin_mask==3);
  o=observation(); o.admin_reply_mask=1; run(s,c,o); CHECK(s.phase==LEG_DISARMING);
  o.admin_reply_mask=0; CHECK(run(s,c,o,LEG_NONE,100).estop);
  s=armed(c); run(s,c,observation(true),LEG_DISARM);
  o=observation(); o.admin_reply_mask=3; run(s,c,o); CHECK(s.phase==LEG_READY);
  o=observation(); o.node[0].seen=false; CHECK(run(s,c,o).estop);
  // DISARM must never manufacture the reference that only ZERO establishes.
  s={}; o=observation(); run(s,c,o,LEG_DISARM);
  o.admin_reply_mask=3; run(s,c,o); CHECK(s.phase==LEG_UNREFERENCED);
  o.admin_reply_mask=0; CHECK(run(s,c,o,LEG_ARM).rejected);
  // A node armed by another path during ZERO must stop the pair immediately.
  s={}; run(s,c,observation(),LEG_ZERO); CHECK(run(s,c,observation(true)).estop);
  s={};run(s,c,observation(),LEG_ZERO);o=observation();o.admin_reply_mask=3;run(s,c,o);
  run(s,c,observation(),LEG_ARM);o=observation(true);o.node[0].status=0x07;o.admin_reply_mask=3;
  CHECK(run(s,c,o).estop);CHECK(s.phase==LEG_FAULT);
  std::printf("%d checks, %d failed\n",checks,failed); return failed?1:0;
}
