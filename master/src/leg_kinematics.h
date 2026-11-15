#pragma once
#include <math.h>
#include <stdint.h>

// Pure planar geometry. Order J01/J03; x is J03->J01, z toward housing.
// Angles are absolute CCW from +x, in radians. Lengths are metres.
// This module neither commands hardware nor certifies physical clearance.
namespace legkin {
struct Vec2 { float x,z; };
struct Geometry { float upper[2],lower[2],spacing_m,margin_rad; };
// Signs of cross(knee3-knee1,foot-knee1), then cross(upper,lower) per side.
// All three signs are fixed for one assembled configuration, never auto-selected.
struct Branch { int8_t foot,elbow[2]; };
struct Reference { bool known; float theta_ref[2]; Branch branch; };
struct Observation {
  bool referenced,session_valid,seen[2],healthy[2];
  uint32_t age_ms[2]; float q_rad[2];
};
enum Status { OK,INVALID_CONFIG,INVALID_INPUT,UNREACHABLE,BRANCH_MISMATCH,
              NEAR_SINGULAR,REFERENCE_UNKNOWN,UNREFERENCED,STALE_STATE,FAULTED_STATE };
struct Result {
  Status status; Vec2 knee[2],foot; float theta[2];
  float jacobian[2][2]; // rows x,z; columns theta1,theta3; m/rad
};
static constexpr float KIN_PI=3.14159265358979323846f;
static constexpr float LENGTH_EPS_M=1e-7f;
static constexpr float CLOSURE_TOL_M=1e-5f;
inline Vec2 sub(Vec2 a,Vec2 b){return {a.x-b.x,a.z-b.z};}
inline float cross(Vec2 a,Vec2 b){return a.x*b.z-a.z*b.x;}
inline float dot(Vec2 a,Vec2 b){return a.x*b.x+a.z*b.z;}
inline float norm(Vec2 a){return sqrtf(dot(a,a));}
inline bool finite(Vec2 a){return isfinite(a.x)&&isfinite(a.z);}
inline bool signValid(int s){return s==1||s==-1;}
inline int sign(float x){return x>0?1:-1;}
inline Result failure(Status status){
  Result r={};r.status=status;r.foot={NAN,NAN};
  for(int i=0;i<2;i++){r.knee[i]={NAN,NAN};r.theta[i]=NAN;r.jacobian[0][i]=r.jacobian[1][i]=NAN;}
  return r;
}
inline bool valid(const Geometry& g,const Branch& b){
  if(!isfinite(g.spacing_m)||g.spacing_m<0||!isfinite(g.margin_rad)||
     g.margin_rad<=0||g.margin_rad>=KIN_PI/2||!signValid(b.foot))return false;
  for(int i=0;i<2;i++)if(!isfinite(g.upper[i])||!isfinite(g.lower[i])||
     g.upper[i]<=LENGTH_EPS_M||g.lower[i]<=LENGTH_EPS_M||!signValid(b.elbow[i]))return false;
  return true;
}
inline Vec2 hip(const Geometry& g,int i){return {(i==0?1.f:-1.f)*g.spacing_m/2,0};}
inline const char* statusText(Status s){
  switch(s){
    case OK:return "OK (geometry only; clearance/tracking not certified)";
    case INVALID_CONFIG:return "invalid geometry or linkage configuration";
    case INVALID_INPUT:return "non-finite input";
    case UNREACHABLE:return "linkage cannot reach/close";
    case BRANCH_MISMATCH:return "different linkage configuration";
    case NEAR_SINGULAR:return "too close to straight/folded/collinear links";
    case REFERENCE_UNKNOWN:return "reference angles/configuration not measured";
    case UNREFERENCED:return "session has no valid stable reference";
    case STALE_STATE:return "missing/stale joint feedback";
    case FAULTED_STATE:return "joint state faulted/invalid";
    default:return "invalid result";
  }
}
inline Result forward(const Geometry& g,const Branch& b,const float theta[2]){
  if(!valid(g,b))return failure(INVALID_CONFIG);
  if(!isfinite(theta[0])||!isfinite(theta[1]))return failure(INVALID_INPUT);
  Result r={};Vec2 upper[2],lower[2];
  for(int i=0;i<2;i++){
    r.theta[i]=theta[i];upper[i]={g.upper[i]*cosf(theta[i]),g.upper[i]*sinf(theta[i])};
    const Vec2 h=hip(g,i);r.knee[i]={h.x+upper[i].x,h.z+upper[i].z};
  }
  const Vec2 d=sub(r.knee[1],r.knee[0]);const float distance=norm(d);
  if(!isfinite(distance)||distance<=LENGTH_EPS_M||distance>g.lower[0]+g.lower[1]||
     distance<fabsf(g.lower[0]-g.lower[1]))return failure(UNREACHABLE);
  const float along=(g.lower[0]*g.lower[0]-g.lower[1]*g.lower[1]+distance*distance)/(2*distance);
  const float height2=g.lower[0]*g.lower[0]-along*along;
  if(height2<=0)return failure(NEAR_SINGULAR);
  const float offset=b.foot*sqrtf(height2);
  r.foot={r.knee[0].x+(along*d.x-offset*d.z)/distance,
          r.knee[0].z+(along*d.z+offset*d.x)/distance};
  if(!finite(r.foot))return failure(INVALID_INPUT);
  const float min_sine=sinf(g.margin_rad);
  for(int i=0;i<2;i++){
    lower[i]=sub(r.foot,r.knee[i]);const float bend=cross(upper[i],lower[i]);
    if(fabsf(bend)/(g.upper[i]*g.lower[i])<min_sine)return failure(NEAR_SINGULAR);
    if(sign(bend)!=b.elbow[i])return failure(BRANCH_MISMATCH);
  }
  const float det=cross(lower[0],lower[1]);
  if(fabsf(det)/(g.lower[0]*g.lower[1])<min_sine)return failure(NEAR_SINGULAR);
  // Differentiate both fixed distal-length constraints: A*dfoot=B*dtheta.
  const float rhs0=cross(upper[0],lower[0]),rhs1=cross(upper[1],lower[1]);
  r.jacobian[0][0]= lower[1].z*rhs0/det;r.jacobian[0][1]=-lower[0].z*rhs1/det;
  r.jacobian[1][0]=-lower[1].x*rhs0/det;r.jacobian[1][1]= lower[0].x*rhs1/det;
  for(int i=0;i<2;i++)if(!isfinite(r.jacobian[0][i])||!isfinite(r.jacobian[1][i]))return failure(INVALID_INPUT);
  r.status=OK;return r;
}
inline Result inverse(const Geometry& g,const Branch& b,Vec2 foot,const float seed[2]){
  if(!valid(g,b))return failure(INVALID_CONFIG);
  if(!finite(foot)||!isfinite(seed[0])||!isfinite(seed[1]))return failure(INVALID_INPUT);
  float theta[2];
  for(int i=0;i<2;i++){
    const Vec2 d=sub(foot,hip(g,i));const float radius=norm(d);
    if(!isfinite(radius)||radius<=LENGTH_EPS_M||radius>g.upper[i]+g.lower[i]||
       radius<fabsf(g.upper[i]-g.lower[i]))return failure(UNREACHABLE);
    float c=(g.upper[i]*g.upper[i]+radius*radius-g.lower[i]*g.lower[i])/(2*g.upper[i]*radius);
    c=fmaxf(-1.f,fminf(1.f,c));
    const float angle=atan2f(d.z,d.x)-b.elbow[i]*acosf(c);
    // Preserve the seed's revolution; configuration signs choose the elbow, not a nearest-pose guess.
    theta[i]=seed[i]+atan2f(sinf(angle-seed[i]),cosf(angle-seed[i]));
  }
  Result r=forward(g,b,theta);
  if(r.status==OK && norm(sub(r.foot,foot))>CLOSURE_TOL_M)return failure(BRANCH_MISMATCH);
  return r;
}
inline Result readFoot(const Geometry& g,const Reference& ref,const Observation& obs,uint32_t max_age_ms){
  if(!ref.known||!isfinite(ref.theta_ref[0])||!isfinite(ref.theta_ref[1]))return failure(REFERENCE_UNKNOWN);
  if(!obs.referenced||!obs.session_valid)return failure(UNREFERENCED);
  for(int i=0;i<2;i++){
    if(!obs.seen[i]||obs.age_ms[i]>max_age_ms)return failure(STALE_STATE);
    if(!obs.healthy[i])return failure(FAULTED_STATE);
  }
  const float theta[2]={ref.theta_ref[0]+obs.q_rad[0],ref.theta_ref[1]+obs.q_rad[1]};
  return forward(g,ref.branch,theta);
}
} // namespace legkin
