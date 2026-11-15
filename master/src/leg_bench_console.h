#pragma once
#include <stdlib.h>
#include <string.h>
#include "leg_bench_policy.h"

static inline bool legNumber(const char* text,float& value) {
  char* end; value=strtof(text,&end); return end!=text && *end==0 && isfinite(value);
}
static inline bool legFootQuery(char* const* t,int n) {
  return n==2&&!strcmp(t[0],"leg")&&!strcmp(t[1],"foot");
}
static inline bool legParseTokens(char* const* t,int n,LegRequest& r,bool& status) {
  r={}; status=false;
  if(n<2 || strcmp(t[0],"leg"))return false;
  if(n==2) {
    if(!strcmp(t[1],"status")){status=true;return true;}
    if(!strcmp(t[1],"zero"))r.kind=LEG_ZERO;
    else if(!strcmp(t[1],"arm"))r.kind=LEG_ARM;
    else if(!strcmp(t[1],"disarm"))r.kind=LEG_DISARM;
    else return false;
    return true;
  }
  if(!strcmp(t[1],"gains") && n==4) {
    r.kind=LEG_GAINS;return legNumber(t[2],r.a)&&legNumber(t[3],r.b);
  }
  if(!strcmp(t[1],"move") && n==5) {
    r.kind=LEG_MOVE;return legNumber(t[2],r.a)&&legNumber(t[3],r.b)&&legNumber(t[4],r.duration_s);
  }
  return false;
}
static inline bool legRawAllowed(const char* cmd) {
  return !strcmp(cmd,"help")||!strcmp(cmd,"?")||!strcmp(cmd,"ident")||!strcmp(cmd,"status")||
    !strcmp(cmd,"mon")||!strcmp(cmd,"t")||!strcmp(cmd,"log")||!strcmp(cmd,"stop")||
    !strcmp(cmd,"reset")||!strcmp(cmd,"clear");
}
