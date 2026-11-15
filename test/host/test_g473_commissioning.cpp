// Reviewed J04/J05 bare-motor rows; other G473 boards remain unmeasured.
#include "joint_cal.h"

int main() {
  int fails = 0;
  auto check = [&](bool ok, const char* name) {
    if (!ok) { printf("FAIL %s\n", name); ++fails; }
  };
  char id[8], board[20];
  snprintf(id, sizeof(id), "J%02d", JOINT_ID);
  snprintf(board, sizeof(board), "B-G473-%02d", JOINT_ID - 3);
  check(strcmp(CAL.id, id) == 0, "separate identity");
  check(strcmp(CAL.board_sn, board) == 0, "board label");
  check(strcmp(CAL.motor_sn, JOINT_ID == 4 ? "motor_4" : JOINT_ID == 5 ? "motor_5" : "-") == 0, "motor identity");
  const float expected_scale = JOINT_ID == 4 ? 0.00836325f
                            : JOINT_ID == 5 ? 0.008401368685f : 0.0085f;
  check(fabsf(CAL.vbus_scale - expected_scale) < 1e-8f, "own M1 scale/seed");
  if (JOINT_ID == 4) {
    check(fabsf(CAL.i_scale - 0.97023f) < 1e-6f, "J04 provisional M2 scale");
    check(strcmp(CAL.date, "2026-11-14") == 0 && calZeaUsable(), "reviewed MT6835 row");
    check(CAL.dir == +1 && fabsf(CAL.zea - 3.8209f) < 1e-6f, "repeat alignment");
    check(CAL.R_eff > 0 && CAL.L > 0 && CAL.Ke > 0, "measured electrical data");
    check(fabsf(torqueOutToIrep(irepToTorqueOut(0.5f)) - 0.5f) < 1e-6f,
          "reported-amp torque round trip");
    check(fabsf(torqueOutToIrep(0.1f) - 0.40396205f) < 1e-6f,
          "M2 scale applied in correct direction at output torque boundary");
    const IqCmd capped = tauOutCmdToIq(1.0f, 1.0f, 1.6f);
    check(capped.clamp == IQ_CLAMP_IQ && capped.iq_A_rep == 1.6f,
          "reported-amp inner clamp survives M2 calibration");
    check(tauOutCmdToIq(1.0f, 0.39f, 1.6f).iq_A_rep <= 1.6f,
          "current cap survives new torque conversion");
    check(fabsf(CAL.breakaway_A - 0.3280f) < 1e-6f, "J04 own accepted bare M4 mean");
  } else if (JOINT_ID == 5) {
    check(strcmp(CAL.date, "2026-11-14") == 0 && calZeaUsable(), "J05 reviewed MT6835 row");
    check(CAL.dir == +1 && fabsf(CAL.zea - 2.3225f) < 1e-6f, "J05 own alignment");
    check(CAL.R_eff > 0 && CAL.L > 0 && CAL.Ke > 0, "J05 measured motor data");
    check(fabsf(CAL.i_scale - 0.98252f) < 1e-6f && fabsf(CAL.breakaway_A - 0.3380f) < 1e-6f,
          "J05 provisional M2 and own accepted bare M4 mean");
    check(fabsf(torqueOutToIrep(irepToTorqueOut(0.5f)) - 0.5f) < 1e-6f,
          "J05 torque conversion round trip");
    check(fabsf(torqueOutToIrep(0.1f) - 0.40595303f) < 1e-6f,
          "J05 M2 scale direction at output torque boundary");
    const IqCmd capped = tauOutCmdToIq(1.0f, 1.0f, 1.6f);
    check(capped.clamp == IQ_CLAMP_IQ && capped.iq_A_rep == 1.6f,
          "J05 reported-amp inner clamp remains active");
  } else {
    check(CAL.i_scale == 1.0f, "other boards retain unmeasured current scale");
    check(strcmp(CAL.date, "-") == 0, "unbuilt date");
    check(!calZeaUsable() && CAL.Ke == 0.0f && CAL.R_eff == 0.0f, "T0 calibration gate closed");
    check(irepToTorqueOut(1.0f) == 0.0f, "no invented torque conversion");
  }
  const auto& uid = JOINT_UID[JOINT_ID - 1];
  if (JOINT_ID == 4) {
    check(strcmp(uid.id, CAL.id) == 0 && uid.w[0] == 0x460024 &&
          uid.w[1] == 0x53465003 && uid.w[2] == 0x20333446, "J04 own recorded MCU UID");
  } else {
    check(uid.w[0] == 0 && uid.w[1] == 0 && uid.w[2] == 0, "other boards retain closed UID gate");
  }
  printf("%s G473 commissioning defaults\n", fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
