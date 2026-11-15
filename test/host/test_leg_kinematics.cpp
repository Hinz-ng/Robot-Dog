#include <cstdio>
#include <cmath>
#define PI 3.1415926535897932384626433832795 // Arduino.h compatibility regression
#include "../../master/src/leg_kinematics_config.h"
static int checks=0,failures=0;
#define CHECK(x) do{++checks;if(!(x)){++failures;std::printf("FAIL %d %s\n",__LINE__,#x);}}while(0)
static bool near(float a,float b,float tolerance=1e-6f){return std::fabs(a-b)<=tolerance;}
int main(){
 using namespace legkin;
 const float pi=3.14159265358979323846f;
 const Geometry g=LEG_GEOMETRY;
 const Branch branch={1,{-1,1}}; // Synthetic outward-knee pose, NOT the rig's reference.
 const float q[2]={-pi/3,-2*pi/3};
 auto r=forward(g,branch,q);
 CHECK(r.status==OK);
 const float knee_x=.0215f+.04f;
 const float expected_z=-.08f*std::sqrt(3.f)/2-std::sqrt(.1f*.1f-knee_x*knee_x);
 CHECK(near(r.foot.x,0)); CHECK(near(r.foot.z,expected_z));
 CHECK(near(r.knee[0].x,knee_x)); CHECK(near(r.knee[1].x,-knee_x));
 for(int a=-4;a<=4;a++)for(int b=-4;b<=4;b++){
  float theta[2]={q[0]+a*.02f,q[1]+b*.02f};auto f=forward(g,branch,theta);CHECK(f.status==OK);
  auto inv=inverse(g,branch,f.foot,theta);CHECK(inv.status==OK);
  CHECK(near(inv.theta[0],theta[0],2e-6f));CHECK(near(inv.theta[1],theta[1],2e-6f));
  for(int i=0;i<2;i++){
   float plus[2]={theta[0],theta[1]},minus[2]={theta[0],theta[1]};plus[i]+=.001f;minus[i]-=.001f;
   auto p=forward(g,branch,plus),m=forward(g,branch,minus);CHECK(p.status==OK&&m.status==OK);
   CHECK(near(f.jacobian[0][i],(p.foot.x-m.foot.x)/.002f,3e-5f));
   CHECK(near(f.jacobian[1][i],(p.foot.z-m.foot.z)/.002f,3e-5f));
  }
 }
 // Wraps must stay near the supplied seed, not jump a revolution.
 float seed[2]={q[0]+2*pi,q[1]-2*pi};auto wrapped=inverse(g,branch,r.foot,seed);
 CHECK(wrapped.status==OK);CHECK(near(wrapped.theta[0],seed[0],2e-6f));CHECK(near(wrapped.theta[1],seed[1],2e-6f));
 CHECK(inverse(g,branch,{1,0},q).status==UNREACHABLE);
 CHECK(inverse(g,branch,{NAN,0},q).status==INVALID_INPUT);
 float bad[2]={INFINITY,q[1]};CHECK(forward(g,branch,bad).status==INVALID_INPUT);
 Geometry invalid=g;invalid.upper[0]=0;CHECK(forward(invalid,branch,q).status==INVALID_CONFIG);
 invalid=g;invalid.margin_rad=NAN;CHECK(forward(invalid,branch,q).status==INVALID_CONFIG);
 Branch unknown={0,{-1,1}};CHECK(forward(g,unknown,q).status==INVALID_CONFIG);
 Branch flipped=branch;flipped.elbow[0]=1;CHECK(forward(g,flipped,q).status==BRANCH_MISMATCH);
 Branch other=branch;other.foot=-1;CHECK(inverse(g,other,r.foot,q).status!=OK);
 // Straight serial chain at the downward reach boundary must be excluded.
 const Vec2 straight={g.spacing_m/2,-(g.upper[0]+g.lower[0])};
 CHECK(inverse(g,branch,straight,q).status!=OK);
 // Lower-link tangency: knees exactly 2*distal apart.
 float tangent[2]={-std::acos((.2f-.043f)/.16f),-pi+std::acos((.2f-.043f)/.16f)};
 CHECK(forward(g,branch,tangent).status!=OK);
 // Independently constructed 14/16-degree boundaries. Zero hip spacing here is
 // a synthetic fixture isolating folded serial / parallel guards, not rig geometry.
 struct Boundary {float spacing,t1,t3;Status expected;};
 const Boundary boundaries[]={
   {.043f,-1.5556039431f,-1.5859887105f,NEAR_SINGULAR}, // serial 14 deg
   {.043f,-1.5364534953f,-1.6051391583f,OK},            // serial 16 deg
   {0,.5293544851f,-3.6709471387f,OK},                 // serial folded: 16 deg from 180
   {0,.6133577038f,-3.7549503574f,NEAR_SINGULAR},      // serial folded: 14 deg from 180
   {0,-1.4178642090f,-1.7237284446f,NEAR_SINGULAR},    // lower links 14 deg
   {0,-1.3959402851f,-1.7456523685f,OK},               // lower links 16 deg
   {.043f,-.2493011396f,-2.8922915140f,OK},            // lower links 164 deg
   {.043f,-.2374851480f,-2.9041075056f,NEAR_SINGULAR}  // lower links 166 deg
 };
 for(const auto& c:boundaries){Geometry test_g=g;test_g.spacing_m=c.spacing;
   const float angle[2]={c.t1,c.t3};CHECK(forward(test_g,branch,angle).status==c.expected);}
 const float coincident[2]={-std::acos(-.043f/.16f),-pi+std::acos(-.043f/.16f)};
 CHECK(forward(g,branch,coincident).status==UNREACHABLE);
 Geometry asymmetric=g;asymmetric.lower[1]=.095f;
 auto asym=forward(asymmetric,branch,q);CHECK(asym.status==OK);
 CHECK(near(std::hypot(asym.foot.x-asym.knee[0].x,asym.foot.z-asym.knee[0].z),.1f));
 CHECK(near(std::hypot(asym.foot.x-asym.knee[1].x,asym.foot.z-asym.knee[1].z),.095f));
 auto ai=inverse(asymmetric,branch,asym.foot,q);CHECK(ai.status==OK);CHECK(near(ai.theta[0],q[0],2e-6f));
 // No hardware state is inferred from geometry; these are pure snapshots.
 Reference ref={true,{q[0],q[1]},branch};
 Observation obs={true,true,{true,true},{true,true},{0,0},{0,0}};
 CHECK(readFoot(g,LEG_KINEMATIC_REFERENCE,obs,50).status==REFERENCE_UNKNOWN);
 CHECK(readFoot(g,ref,obs,50).status==OK);
 obs.q_rad[0]=.01f;auto moved=readFoot(g,ref,obs,50);CHECK(moved.status==OK);CHECK(near(moved.theta[0],q[0]+.01f));
 obs.referenced=false;CHECK(readFoot(g,ref,obs,50).status==UNREFERENCED);obs.referenced=true;
 obs.session_valid=false;CHECK(readFoot(g,ref,obs,50).status==UNREFERENCED);obs.session_valid=true;
 obs.age_ms[1]=51;CHECK(readFoot(g,ref,obs,50).status==STALE_STATE);obs.age_ms[1]=50;
 CHECK(readFoot(g,ref,obs,50).status==OK);
 obs.seen[0]=false;CHECK(readFoot(g,ref,obs,50).status==STALE_STATE);obs.seen[0]=true;
 obs.healthy[1]=false;CHECK(readFoot(g,ref,obs,50).status==FAULTED_STATE);obs.healthy[1]=true;
 obs.q_rad[0]=NAN;CHECK(readFoot(g,ref,obs,50).status==INVALID_INPUT);
 CHECK(!LEG_KINEMATIC_REFERENCE.known);CHECK(std::isnan(LEG_KINEMATIC_REFERENCE.theta_ref[0]));
 std::printf("%d checks, %d failed\n",checks,failures);return failures?1:0;
}
