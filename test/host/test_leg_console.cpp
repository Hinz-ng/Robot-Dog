#include <cstdio>
#include <cstring>
#include "../../master/src/leg_bench_console.h"
static int fail=0,n=0;
#define CHECK(x) do{++n;if(!(x)){++fail;std::printf("FAIL %d %s\n",__LINE__,#x);}}while(0)
static bool parse(const char* text,LegRequest& r,bool& status){char b[96];std::strcpy(b,text);char* t[16];int k=0;for(char* p=std::strtok(b," ");p&&k<16;p=std::strtok(nullptr," "))t[k++]=p;return legParseTokens(t,k,r,status);}
int main(){LegRequest r={};bool status=false;
 char leg[]="leg",foot[]="foot",extra[]="1",other[]="other";
 char* ft[]={leg,foot,extra};CHECK(legFootQuery(ft,2));CHECK(!legFootQuery(ft,3));CHECK(!legFootQuery(ft,1));
 ft[0]=other;CHECK(!legFootQuery(ft,2));
 CHECK(parse("leg zero",r,status)&&r.kind==LEG_ZERO);
 CHECK(parse("leg arm",r,status)&&r.kind==LEG_ARM);
 CHECK(parse("leg status",r,status)&&status);
 CHECK(parse("leg gains 2 .02",r,status)&&r.kind==LEG_GAINS&&r.a==2);
 CHECK(parse("leg move .01 -.01 .5",r,status)&&r.kind==LEG_MOVE&&r.b<0);
 CHECK(!parse("leg arm 1",r,status)); CHECK(!parse("leg move .01 0",r,status));
 CHECK(!parse("leg gains NaN .1",r,status)); CHECK(!parse("leg gains 2 inf",r,status));
 CHECK(!parse("leg move 0 0 1 garbage",r,status)); CHECK(!parse("leg move 1x 0 1",r,status));
 CHECK(!legRawAllowed("arm")); CHECK(!legRawAllowed("armraw")); CHECK(!legRawAllowed("hold"));
 CHECK(!legRawAllowed("set")); CHECK(!legRawAllowed("pause")); CHECK(!legRawAllowed("testver"));
 CHECK(legRawAllowed("stop")); CHECK(legRawAllowed("status")); CHECK(legRawAllowed("log"));
 std::printf("%d checks, %d failed\n",n,fail);return fail?1:0;}
