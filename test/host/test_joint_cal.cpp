// Host test: joint_cal.h's encoder-chip guard (calZeaUsable) and its banner text.
// Built once per case by run.py with -D JOINT_ID=n -D EXPECT_USABLE=0|1
// -D EXPECT_TEXT="..." (the banner line that must appear).
#include "joint_cal.h"

int main() {
  Print p;
  printJointCal(p);
  int fails = 0;
  if (calZeaUsable() != (EXPECT_USABLE != 0)) {
    printf("FAIL JOINT_ID=%d: calZeaUsable()=%d, expected %d\n", JOINT_ID, calZeaUsable(), EXPECT_USABLE);
    fails++;
  }
  if (p.buf.find(EXPECT_TEXT) == std::string::npos) {
    printf("FAIL JOINT_ID=%d: banner lacks \"%s\"\n", JOINT_ID, EXPECT_TEXT);
    fails++;
  }
  printf("%s JOINT_ID=%d ENC_CHIP=%d zea_enc=%u usable=%d\n", fails ? "FAIL" : "PASS",
         JOINT_ID, ENC_CHIP, CAL.zea_enc, calZeaUsable());
  return fails ? 1 : 0;
}
