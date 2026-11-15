#include <cstdio>
#include "../../master/src/leg_bench_policy.h"
static LegBenchConfig makeConfig(){LegBenchConfig c={};c.configured=true;c.joint[0]={1,1,0,-.035f,.035f,-.07f,.07f};c.joint[1]={3,-1,0,-.035f,.035f,-.07f,.07f};c.kp_max=10;c.kd_max=.1f;c.rate_max=.02f;c.arm_speed_max=.05f;c.fresh_ms=50;c.transaction_ms=100;c.missed_max=3;c.move_max_s=10;c.control_gap_ms=10;return c;}
static const LegBenchConfig LEG_CONFIG=makeConfig();
enum{REQ_NONE,REQ_ARM,REQ_DISARM,REQ_ZERO,REQ_CLEAR,REQ_IDENT};
static const int M_N_NODES=2;
struct TestNode{uint8_t id,req,st_status;bool session_armed;MitState st;MitCmd cmd;uint32_t st_ms,miss_run;};
static TestNode m_node[2]; static int m_mux=0;
static bool m_leg_request_pending=false,m_leg_abort_pending=false,m_estop=false;
static bool m_leg_phone_owned=false;static uint32_t m_leg_phone_ms=0;
static const uint32_t M_WEB_DEADMAN_MS=1500;
static LegRequest m_leg_request={};static LegSession m_leg_public={};
static uint32_t clock_ms=100;static uint32_t millis(){return clock_ms;}
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
static int stops=0;static void estopBroadcast(){++stops;}
static void allSessionsOff(){for(auto& n:m_node){n.session_armed=false;}m_leg_abort_pending=true;}
static void evt(const char*,...){ }
#include "../../master/src/leg_bench_runtime.h"
static int fail=0,checks=0;
#define CHECK(x) do{++checks;if(!(x)){++fail;std::printf("FAIL %d %s\n",__LINE__,#x);}}while(0)
static void reply(int i,bool armed,uint8_t fault=0){m_node[i].st_ms=clock_ms;m_node[i].st_status=canStatusByte(armed,0,false,fault);m_node[i].session_armed=armed;}
int main(){for(int i=0;i<2;i++){m_node[i].id=i?3:1;reply(i,false);}
 CHECK(legQueue({LEG_ZERO,0,0,0})); CHECK(!legQueue({LEG_ARM,0,0,0})); legService();
 CHECK(m_node[0].req==REQ_ZERO&&m_node[1].req==REQ_ZERO);
 legAdminResult(0,true); CHECK(m_leg_public.phase==LEG_ZEROING);
 legAdminResult(1,true); CHECK(m_leg_public.phase==LEG_READY);
 CHECK(legQueue({LEG_ARM,0,0,0,true}));legService();CHECK(m_leg_phone_owned);MitCmd c;
 reply(0,true);legAdminResult(0,true);CHECK(!legCommandForNode(0,c));
 reply(1,true);legAdminResult(1,true);CHECK(legCommandForNode(0,c));CHECK(c.kp_Nm_per_rad==0);
 CHECK(legQueue({LEG_GAINS,2,.02f,0}));legService();CHECK(legCommandForNode(1,c));CHECK(c.kp_Nm_per_rad==2);
 // Cached STATE is still <50 ms old, but Tier 0 entered DAMP at 15 ms.
 // No new CMD may silently resume the trajectory after this scheduling gap.
 const LegSession saved=m_leg_session;clock_ms+=16;legService();
 CHECK(m_estop);CHECK(!legCommandForNode(0,c));
 clock_ms=100;m_leg_session=saved;m_leg_public=saved;m_estop=false;m_leg_abort_pending=false;
 m_leg_phone_owned=true;m_leg_phone_ms=100;
 CHECK(!legPhoneExpired(1600));CHECK(legPhoneExpired(1601));
 // HTTP/UI loop is stalled, while the bus task still receives LIVE replies.
 for(clock_ms=105;clock_ms<=1600;clock_ms+=5){
   for(int i=0;i<2;i++){reply(i,true);legControlReply(i,clock_ms,m_node[i].st_status);}
   legService();CHECK(!m_estop);
 }
 for(int i=0;i<2;i++){reply(i,true);legControlReply(i,clock_ms,m_node[i].st_status);}
 legService();CHECK(m_estop);CHECK(!legCommandForNode(1,c));
 clock_ms=100;m_leg_session=saved;m_leg_public=saved;m_estop=false;m_leg_abort_pending=false;
 m_leg_phone_owned=false;
 // Simulated STATE from node 0 arrives before node 1's exchange in the cycle.
 reply(0,false,CAN_FAULT_ENCODER);legService();
 CHECK(stops>0&&m_estop);CHECK(!legCommandForNode(1,c));CHECK(m_node[1].req==REQ_NONE);
 CHECK(!m_node[0].session_armed&&!m_node[1].session_armed);
 std::printf("%d checks, %d failed\n",checks,fail);return fail?1:0;}
