#pragma once
// ============================================================================
// joint_cal.h  --  PER-JOINT CALIBRATION TABLE  (hand-entered, git-tracked)
// ============================================================================
// ONE row per physical assembly. Selected AT COMPILE TIME by -D JOINT_ID=n from
// platformio.ini, so the constants are never hand-edited before a flash -- you
// pick an ENVIRONMENT, not a number:
//
//     pio run -e J01 -t upload
//
// WHY HAND-ENTERED AND NOT AUTO-SAVED
//   AUTOCALIB prints a pasteable row; YOU paste it. Deliberately manual:
//     * every constant gets read by a human before it becomes authoritative,
//       which is the step that catches a WARN or a bad fit;
//     * a recalibration becomes a reviewable git diff with a date on it, so
//       "when did J07's R change and why" is answerable months later;
//     * flash writes cannot fail halfway and leave a joint with corrupt
//       calibration and no way to tell.
//   The cost is one paste per joint. The routine already formats the row so the
//   paste is mechanical, and NOTHING that failed its verdict is emitted as a
//   number -- unmeasured fields come out as 0.0f with a NOT MEASURED comment.
//
// WHAT BELONGS TO WHAT -- this is why the serial columns exist:
//   motor only     Ke, Kt, L, cogging          (winding + magnet)
//   board only     vbus_scale, U0              (shunts, gate driver, dead time)
//   the PAIRING    zea, dir, R_eff, INL        (how the magnet was glued on,
//                                               plus motor + FETs + shunt in series)
//   Swap a motor between boards and zea/dir/R_eff are INVALID. The serial
//   columns make that visible in the diff instead of silent in the hardware.
//
// SAFE DEFAULT FOR UNFILLED JOINTS: zea = -1.0f, dir = 0. open_test.cpp's
// runInitFOC() already treats ZEA_STORED < 0 as "not yet measured" and falls
// back to a full alignment, so an unfilled row degrades to the old behaviour
// rather than commutating on garbage.
//
// AFTER FLASHING: press 'V'. It does one forced alignment and compares it to the
// stored value. That is the only thing standing between you and a silent
// wrong-joint flash, because storing ZEA removed the per-power-up alignment that
// used to catch it.
// ============================================================================

struct JointCal {
  const char* id;          // "J01" -- must match the label on the BOARD
  const char* board_sn;    // board serial
  const char* motor_sn;    // motor serial
  const char* date;        // calibration date
  const char* belt;        // "OFF" or "10mm-9:1" -- drag numbers depend on it
  float  zea;              // rad elec.  PAIRING.  < 0 = not measured
  int8_t dir;              // +1 CW, -1 CCW.  PAIRING.  0 = not measured
  float  R_eff;            // ohm.       PAIRING (winding + FETs + shunt)
  float  U0;               // V.         BOARD (dead time). Scales with Vbus
  float  Ke;               // V/(rad/s). MOTOR
  float  Kt;               // Nm/A.      MOTOR  = 1.5*Ke
  float  L;                // H.         MOTOR, incremental at ~3 A
  float  vbus_scale;       // V/count.   BOARD
  float  drag_coulomb;     // A.         PAIRING, belt state above
  float  drag_viscous;     // A/(rad/s). PAIRING
};

// ---------------------------------------------------------------------------
// THE FLEET.  Paste AUTOCALIB output here, one row per assembly.
// ---------------------------------------------------------------------------
const JointCal JOINTS[] = {

  // -- J01 -- was "the SPARE"/A2, first SPI assembly, now the reference actuator.
  //    AUTOCALIB rev1 2026-08-07. Multimeter Vbus 12.51 vs firmware 12.52 -> scale OK.
  //    U0 came back 2.4 sigma = NOT DETERMINED. The value below is carried for
  //    completeness ONLY; do not build anything that depends on it being right.
  //    L from the 5-point index-averaged fit; re-run phase 4 after the F4
  //    time-binning fix before trusting it.
  { "J01", "B-SPI-01", "M-SPI-01", "2026-08-07", "OFF",
     6.0516f, +1,  0.22184f, 0.00367f, 0.017851f, 0.026777f, 44.43e-6f,
     0.008358f, 0.0785f, 0.000850f },

  // -- J02 .. J12 -- not yet built. zea<0 and dir=0 make runInitFOC() fall back
  //    to a full alignment, so selecting one of these is safe, just uncalibrated.
  { "J02", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J03", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J04", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J05", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J06", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J07", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J08", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J09", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J10", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J11", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },
  { "J12", "-", "-", "-", "OFF", -1.0f, 0, 0.218f, 0.028f, 0.0177f, 0.0266f, 65e-6f, 0.008358f, 0.0f, 0.0f },

  // -- A1 -- the ORIGINAL ABZ assembly, index 12 (JOINT_ID=13). Kept so the ABZ
  //    fault can still be debugged with the right constants. Its ZEA is
  //    session-relative by construction (incremental encoder), hence -1.
  { "A1",  "B-ABZ-01", "M-ABZ-01", "2026-07-29", "10mm-9:1",
    -1.0f,  0,  0.218f,   0.028f,   0.0177f,  0.0266f,  65e-6f,
     0.008358f, 1.05f, 0.0f },
};

#ifndef JOINT_ID
  #error "Build with -D JOINT_ID=n (see platformio.ini). Refusing a joint-agnostic binary."
#endif
#if (JOINT_ID < 1) || (JOINT_ID > 13)
  #error "JOINT_ID out of range 1..13"
#endif

const JointCal& CAL = JOINTS[JOINT_ID - 1];

// Print at boot so wrong-firmware-on-wrong-board is visible in ONE GLANCE
// instead of inferred later from bad behaviour. The board must carry the same
// physical label as CAL.id.
static inline void printJointCal(Print& out) {
  out.print(F("JOINT ")); out.print(CAL.id);
  out.print(F(" board=")); out.print(CAL.board_sn);
  out.print(F(" motor=")); out.print(CAL.motor_sn);
  out.print(F(" cal=")); out.print(CAL.date);
  out.print(F(" belt=")); out.println(CAL.belt);
  out.print(F("  R=")); out.print(CAL.R_eff, 5);
  out.print(F(" U0=")); out.print(CAL.U0, 5);
  out.print(F(" Ke=")); out.print(CAL.Ke, 6);
  out.print(F(" Kt=")); out.print(CAL.Kt, 6);
  out.print(F(" L=")); out.print(CAL.L*1e6f, 1); out.print(F("uH"));
  out.print(F(" ZEA=")); out.print(CAL.zea, 4);
  out.print(F(" DIR=")); out.println(CAL.dir);
  if (CAL.zea < 0.0f || CAL.dir == 0)
    out.println(F("  !! UNCALIBRATED joint -- 'f' will do a full alignment. Run AUTOCALIB."));
  else
    out.println(F("  press 'V' to verify the stored ZEA against a fresh alignment"));
}