#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "../master/src/leg_kinematics_config.h"

// PC-only adapter to the SAME math/config header as firmware. No serial/network I/O.
int main(int argc,char** argv){
  using namespace legkin;
  if(argc<2)return 2;
  const bool fk=!std::strcmp(argv[1],"fk"),ik=!std::strcmp(argv[1],"ik"),sample=!std::strcmp(argv[1],"sample");
  const int count=fk?5:7;
  if((!fk&&!ik&&!sample)||argc!=count+2)return 2;
  float a[7];
  for(int i=0;i<count;i++){
    char* end=nullptr;a[i]=std::strtof(argv[i+2],&end);
    if(end==argv[i+2]||*end||!std::isfinite(a[i])){std::fprintf(stderr,"finite numeric inputs required\n");return 2;}
  }
  const int offset=count-3;
  for(int i=offset;i<count;i++)if(a[i]!=-1&&a[i]!=1){std::fprintf(stderr,"branch signs must be -1 or +1\n");return 2;}
  const Branch branch={(int8_t)a[offset],{(int8_t)a[offset+1],(int8_t)a[offset+2]}};
  Result r;
  if(fk){const float theta[2]={a[0]*KIN_PI/180,a[1]*KIN_PI/180};r=forward(LEG_GEOMETRY,branch,theta);}
  else if(ik){const float seed[2]={a[2]*KIN_PI/180,a[3]*KIN_PI/180};r=inverse(LEG_GEOMETRY,branch,{a[0]/1000,a[1]/1000},seed);}
  else{
    const Reference ref={true,{a[0]*KIN_PI/180,a[1]*KIN_PI/180},branch};
    const Observation obs={true,true,{true,true},{true,true},{0,0},{a[2],a[3]}};
    r=readFoot(LEG_GEOMETRY,ref,obs,50);
  }
  if(r.status!=OK){std::fprintf(stderr,"unavailable: %s\n",statusText(r.status));return 2;}
  std::printf("{\"geometry_only\":true,\"foot_mm\":[%.6f,%.6f],\"theta_rad\":[%.9f,%.9f],"
              "\"jacobian_m_per_rad\":[[%.9f,%.9f],[%.9f,%.9f]]}\n",
              r.foot.x*1000,r.foot.z*1000,r.theta[0],r.theta[1],
              r.jacobian[0][0],r.jacobian[0][1],r.jacobian[1][0],r.jacobian[1][1]);
  return 0;
}
