#include <cstdio>
#include "../../src/tier0/leg_bench_config.h"
#include "../../master/src/leg_bench_config.h"
static int fail=0,checks=0;
#define CHECK(x) do { ++checks; if(!(x)){++fail; std::printf("FAIL %d %s\n",__LINE__,#x);} } while(0)
int main(){
  CHECK(t0LegConfigured(T0_LEG_POLICY)); CHECK(legConfigValid(LEG_CONFIG));
  CHECK(LEG_CONFIG.joint[JOINT_ID==1?0:1].node==JOINT_ID);
  CHECK(T0_LEG_POLICY.configured==LEG_CONFIG.configured);
  if(LEG_CONFIG.configured){const auto& j=LEG_CONFIG.joint[JOINT_ID==1?0:1];
    const float a=j.sign*(j.cmd_min-j.q_ref),b=j.sign*(j.cmd_max-j.q_ref);
    const float x=j.sign*(j.trip_min-j.q_ref),y=j.sign*(j.trip_max-j.q_ref);
    CHECK(fabsf(T0_LEG_POLICY.cmd_min-fminf(a,b))<1e-6f);
    CHECK(fabsf(T0_LEG_POLICY.cmd_max-fmaxf(a,b))<1e-6f);
    CHECK(fabsf(T0_LEG_POLICY.trip_min-fminf(x,y))<1e-6f);
    CHECK(fabsf(T0_LEG_POLICY.trip_max-fmaxf(x,y))<1e-6f);
  }
  // Wider real configuration: recorded interior pose accepted, limits still enforced.
  T0LegReference ready={true};
  CHECK(t0LegArmAllowed(T0_LEG_POLICY,ready,1.23018f,0));
  CHECK(t0LegArmAllowed(T0_LEG_POLICY,ready,1.09487f,0));
  MitCmd wide={2.09f,0,2,.02f,0};
  CHECK(t0LegCmdAllowed(T0_LEG_POLICY,wide));
  wide.p_des_rad=-2.09f;CHECK(t0LegCmdAllowed(T0_LEG_POLICY,wide));
  wide.p_des_rad=2.10f;CHECK(!t0LegCmdAllowed(T0_LEG_POLICY,wide));
  wide.p_des_rad=-2.10f;CHECK(!t0LegCmdAllowed(T0_LEG_POLICY,wide));
  CHECK(t0LegActualInBounds(T0_LEG_POLICY,2.18f));
  CHECK(!t0LegActualInBounds(T0_LEG_POLICY,2.19f));
  CHECK(!t0LegActualInBounds(T0_LEG_POLICY,-2.19f));
  T0LegPolicy p={true,-0.035f,0.035f,-0.07f,0.07f,0.05f,0.5f,10.0f,0.1f};
  T0LegReference r={}; CHECK(!t0LegArmAllowed(p,r,0,0));
  r.onAdmin(CAN_OP_CLEAR_FAULT,false); CHECK(!t0LegArmAllowed(p,r,0,0));
  r.onAdmin(CAN_OP_ZERO,true); CHECK(!r.zeroed);
  r.onAdmin(CAN_OP_ZERO,false); CHECK(t0LegArmAllowed(p,r,0,0));
  CHECK(!t0LegArmAllowed(p,r,0.05f,0)); CHECK(!t0LegArmAllowed(p,r,0,0.06f));
  CHECK(!t0LegArmAllowed(p,r,NAN,0)); CHECK(!t0LegArmAllowed(p,r,0,NAN));
  CHECK(t0LegMayRefresh(CAN_PHASE_LIVE));CHECK(t0LegMayRefresh(CAN_PHASE_HOLD));
  CHECK(!t0LegMayRefresh(CAN_PHASE_DAMP));
  CHECK(t0LegActualInBounds(p,0.05f)); CHECK(!t0LegActualInBounds(p,0.071f));
  MitCmd c={0.01f,0,2,0.02f,0}; CHECK(t0LegCmdAllowed(p,c));
  c.p_des_rad=0.04f; CHECK(!t0LegCmdAllowed(p,c)); c.p_des_rad=0;
  c.kp_Nm_per_rad=10.1f; CHECK(!t0LegCmdAllowed(p,c)); c.kp_Nm_per_rad=2;
  c.v_des_rads=0.02f; CHECK(!t0LegCmdAllowed(p,c)); c.v_des_rads=0;
  c.tau_ff_Nm=0.005f; CHECK(!t0LegCmdAllowed(p,c));
  p.configured=false; CHECK(!t0LegArmAllowed(p,r,0,0));
  std::printf("%d checks, %d failed\n",checks,fail); return fail?1:0;
}
