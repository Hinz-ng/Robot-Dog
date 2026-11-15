#include <cstdio>
#include <cstring>
#include "../../master/src/log_control.h"
static int checks=0,failed=0;
#define CHECK(x) do {++checks;if(!(x)){++failed;std::printf("FAIL line %d\n",__LINE__);}}while(0)
int main(){
  uint32_t n=2;bool on=false;
  logControl("start",true,false,4,n,on);CHECK(on && n==2);
  ++n;logControl("stop",true,false,4,n,on);CHECK(!on && n==3);
  logControl("start",true,false,4,n,on);CHECK(on && n==3);
  logControl("clear",true,false,4,n,on);CHECK(on && n==3);
  logControl("stop",true,false,4,n,on);
  logControl("clear",true,true,4,n,on);CHECK(!on && n==3);
  logControl("typo",true,false,4,n,on);CHECK(!on && n==3);
  n=4;logControl("start",true,false,4,n,on);CHECK(!on && n==4);
  logControl("clear",true,false,4,n,on);CHECK(!on && n==0);
  logControl("start",false,false,4,n,on);CHECK(!on && n==0);
  logControl("start",true,false,4,n,on);CHECK(on && n==0);
  std::printf("%d checks, %d failed\n",checks,failed);return failed?1:0;
}
