#pragma once
#include <Arduino.h>
#include <string.h>
#include "fleet_config.h"
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
//   motor only     Ke, L, cogging             (winding + magnet)
//   board only     vbus_scale, i_scale, U0    (divider, shunts, gate dead time)
//   the PAIRING    zea, dir, R_eff, INL,      (how the magnet was glued on,
//                  drag, breakaway             plus motor + FETs + shunt in series)
//   Swap a motor between boards and zea/dir/R_eff are INVALID. The serial
//   columns make that visible in the diff instead of silent in the hardware.
//
// WHAT DOES *NOT* BELONG HERE: anything identical on every correctly built
//   joint. Pole pairs, encoder resolution, Kt/Ke = 1.5, gear ratio, dead_zone,
//   PWM frequency and T/T_loop are in fleet_config.h. Kt in particular is
//   DERIVED (calKt()), never stored -- see the note there.
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
  const char* date;        // calibration date. "-" = never calibrated
  const char* belt;        // "OFF" | "10mm-9:1" -- drag AND breakaway depend on it

  // ---- PAIRING: this magnet mount + this motor in this board ----
  float  zea;              // rad elec.  < 0 = not measured
  int8_t dir;              // +1 CW, -1 CCW.  0 = not measured
  float  R_eff;            // ohm at bench ambient, whole drive path. Scales with
                           //   vbus_scale, and with 1/i_scale -- see i_scale

  // ---- BOARD ----
  float  U0;               // V dead-time offset. Scales with Vbus AND vbus_scale.
                           //   The per-fit SE understates the real uncertainty by
                           //   ~2x -- see the J01 row comment before trusting it

  // ---- MOTOR ----
  float  Ke;               // V/(rad/s).  Kt is DERIVED, not stored: calKt()
  float  L;                // H, INCREMENTAL at ~3 A. Inherits R_eff's scaling

  // ---- BOARD ----
  float  vbus_scale;       // V per ADC count. M1, needs a multimeter.
                           //   NOT a fleet constant -- see the warning below
  float  i_scale;          // I_true = I_sensed / i_scale.  M2.  1.0 = uncorrected.
                           //   R_eff and L are BOTH proportional to 1/i_scale and
                           //   Ke is NOT, so changing this invalidates phases 3-4
                           //   but not phase 5

  // ---- PAIRING, belt-state dependent (see .belt) ----
  float  drag_c_fwd;       // A            free-spin Coulomb intercept, |omega| > 0
  float  drag_c_rev;       // A            stored as POSITIVE magnitudes; the
  float  drag_v_fwd;       // A/(rad/s)    consumer applies sign(omega). The
  float  drag_v_rev;       // A/(rad/s)    fwd/rev asymmetry is real (~28% on J01)
                           //              and a single mean field discarded it
  float  breakaway_A;      // A  M4, STATIC threshold. Mean over rotor positions;
                           //    min/max and the direction split go in the row
                           //    comment. Expect > drag_c, which is DYNAMIC
};

// !!! vbus_scale IS NOT A FLEET CONSTANT !!!
// It is a resistor divider -- two parts at (probably) 1% tolerance, so up to
// ~2% board to board -- and R_eff, U0 and Ke ALL scale linearly with it. That
// 2% lands directly on every torque command. Every row below currently reads
// 0.008358 only because it was measured once on B-SPI-01 and copied into the
// placeholders. An unbuilt row will therefore silently inherit the WRONG
// board's divider: M1 is mandatory per board before any row is trusted.

// Kt is a CONVENTION, not a measurement (see KT_PER_KE in fleet_config.h).
// Storing both invites a partial edit that leaves them inconsistent with
// nothing to catch it, so Kt is computed on demand instead.
static inline float calKt(const JointCal& c) { return c.Ke * KT_PER_KE; }

// ---------------------------------------------------------------------------
// THE FLEET.  Paste AUTOCALIB output here, one row per assembly.
// ---------------------------------------------------------------------------
const JointCal JOINTS[] = {

  // -- J01 -- was "the SPARE"/A2, first SPI assembly, the reference actuator.
  //    AUTOCALIB rev2 session 3, 2026-08-07. VERDICT PASS, all six phases.
  //    Firmware Vbus 12.46. MULTIMETER: ______  <-- write it in; R_eff, U0 and Ke
  //    all scale linearly with vbus_scale and this is the only external check.
  //
  //    L SUPERSEDES 44.43 / 44.8 / 45.1 uH. Those came from 5- and 7-point fits on
  //    a phase-locked sample grid and were biased +4% in tau. This is an 18-point
  //    dithered fit, tau = 195.9 us, verified by independent offline refit.
  //
  //    U0 is 6.5 sigma in THIS fit but does NOT reproduce: 0.00367 / 0.00559 /
  //    0.01026 V across three sessions with per-fit SE ~0.0016. Real uncertainty
  //    is +-0.003 V. Harmless: 0.046 A deadband = 0.11 N at the foot, and 0.078 A
  //    = 0.18 N projected to a 5S bus. This board does NOT reopen dead_zone.
  //
  //    Ke's 0.57% session spread tracks bus voltage (12.52 -> 12.46) to 0.09%,
  //    not the motor. Kt = calKt() = 0.026626 vs nameplate KV360 -> 0.026526,
  //    +0.38%. That validates the VOLTAGE scale only; the current scale is M2.
  //
  //    DRAG: fwd/rev asymmetry is real and reproducible (viscous 9.33e-4 fwd vs
  //    7.27e-4 rev, 28%). The old single-mean pair (0.0783, 0.000830) discarded
  //    it and has been replaced by the four fields below.
  //
  //    DIAGNOSTICS, not fields: T/T_loop 0.974 & 0.945 (T = 73.0 us at 13.3 kHz;
  //    0.95% torque loss at 270 rad/s) | INL 1.029 deg mech pk-pk = 1/rev 0.369 +
  //    2/rev 0.204 deg mech | ZEA residual 0.467 deg elec | |I| ratio 1.245-1.313.
  { "J01", "B-SPI-01", "M-SPI-01", "2026-08-07", "OFF",
     6.0542f, +1, 0.22108f,        // zea, dir, R_eff
     0.01026f,                     // U0
     0.017750f, 43.31e-6f,         // Ke, L        (Kt = calKt() = 0.026626)
     0.008358f, 1.0f,              // vbus_scale, i_scale   <-- i_scale PENDING M2
     0.0750f, 0.0816f,             // drag_c fwd, rev
     9.33e-4f, 7.27e-4f,           // drag_v fwd, rev
     0.0f },                       // breakaway_A           <-- PENDING M4

  // -- J02 .. J12 -- not yet built. zea<0 and dir=0 make runInitFOC() fall back
  //    to a full alignment, so selecting one of these is safe, just uncalibrated.
  //    Every number below is INHERITED from B-SPI-01/M-SPI-01 as a plausible
  //    starting point, NOT measured on that joint. vbus_scale in particular is a
  //    per-board divider and WILL be wrong on a new board: run M1 first.
  { "J02", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J03", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J04", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J05", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J06", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J07", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J08", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J09", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J10", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J11", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J12", "-", "-", "-", "OFF", -1.0f, 0, 0.221f, 0.010f, 0.01775f, 43.3e-6f,
    0.008358f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },

  // -- A1 -- the ORIGINAL ABZ assembly, index 12 (JOINT_ID=13). Kept so the ABZ
  //    fault can still be debugged with the right constants. Its ZEA is
  //    session-relative by construction (incremental encoder), hence -1.
  //
  //    *** THIS FIRMWARE CANNOT DRIVE A1. *** The encoder driver in
  //    open_test.cpp is MT6816 4-wire SPI on PB5-PB8; A1 is wired for ABZ
  //    quadrature on TIM4. Building -e A1 produces a binary whose sensor will
  //    fail phase 1 / 'e' immediately. The row exists so that the SEPARATE ABZ
  //    sketch can pick up A1's constants by JOINT_ID; it is not a flashable
  //    target in this repository.
  //
  //    Drag is belt-ON here and not direction-split (single 1.05 A Coulomb
  //    figure from the July map), so the fwd and rev fields carry the same
  //    number. Breakaway was observed over a 0.34-1.34 A band belt-on, but no
  //    position-averaged MEAN was ever recorded, so the field stays 0 = not
  //    measured rather than carrying an invented average.
  { "A1",  "B-ABZ-01", "M-ABZ-01", "2026-07-29", "10mm-9:1",
    -1.0f, 0, 0.218f,              // zea, dir, R_eff
     0.028f,                       // U0
     0.0177f, 65e-6f,              // Ke, L        (Kt = calKt() = 0.02655)
     0.008358f, 1.0f,              // vbus_scale (INHERITED, not measured), i_scale
     1.05f, 1.05f,                 // drag_c fwd, rev   -- belt ON, not split
     0.0f, 0.0f,                   // drag_v fwd, rev   -- not fitted
     0.0f },                       // breakaway_A       -- band 0.34-1.34 A, no mean
};

static constexpr uint8_t JOINT_COUNT = (uint8_t)(sizeof(JOINTS) / sizeof(JOINTS[0]));

#ifndef JOINT_ID
  #error "Build with -D JOINT_ID=n (see platformio.ini). Refusing a joint-agnostic binary."
#endif
// Bounds-checked against the table itself, not against a hand-copied 13, so
// adding a row cannot leave the guard behind.
static_assert(JOINT_ID >= 1 && JOINT_ID <= JOINT_COUNT,
              "JOINT_ID is outside the JOINTS[] table -- see platformio.ini");

// `static` deliberately: a namespace-scope reference has EXTERNAL linkage, so
// without it a second .cpp including this header would collide at link time.
static const JointCal& CAL = JOINTS[JOINT_ID - 1];

// Kt for the selected joint. Derived every time, so it cannot disagree with Ke.
static inline float calKt() { return calKt(CAL); }

// ---------------------------------------------------------------------------
// THE ONE PLACE i_scale IS ALLOWED TO BE APPLIED
// ---------------------------------------------------------------------------
// i_scale = g = I_reported / I_true, from M2.
//
// The current LOOP needs no correction and must not be given one. R_eff and L
// were both MEASURED in reported-amp units, so R_stored = R_true/g and
// L_stored = L_true/g: the plant from volts to REPORTED amps is exactly
// (R_stored + s*L_stored). Their ratio (tau) and the loop's DC gain are already
// right, and the PI gains were tuned in those same units. Dividing R_eff, L or
// the gains by i_scale would DOUBLE-count g and detune a loop that is correct.
//
// g leaks in exactly one place: the torque command. Real torque is Kt * I_true
// = Kt * I_reported / g, and Kt itself is clean (Ke is fit from voltage and
// speed, so it is independent of current-sense gain -- which is why the +0.38%
// Kt-vs-KV agreement confirms the VOLTAGE scale and says nothing about this).
// To actually deliver tau you must therefore ask for g*tau/Kt reported-amps:
//
//     I_command [reported A] = tau_desired / calKtCmd()
//
// i_scale = 1.0 makes this identical to calKt(), so it is inert until M2 runs.
static inline float calKtCmd() {
  return (CAL.i_scale > 0.0f) ? (calKt() / CAL.i_scale) : calKt();
}

// Print at boot so wrong-firmware-on-wrong-board is visible in ONE GLANCE
// instead of inferred later from bad behaviour. The board must carry the same
// physical label as CAL.id.
static inline void printJointCal(Print& out) {
  const bool built = (strcmp(CAL.date, "-") != 0);

  out.print(F("JOINT ")); out.print(CAL.id);
  out.print(F(" board=")); out.print(CAL.board_sn);
  out.print(F(" motor=")); out.print(CAL.motor_sn);
  out.print(F(" cal=")); out.print(CAL.date);
  out.print(F(" belt=")); out.println(CAL.belt);

  out.print(F("  R=")); out.print(CAL.R_eff, 5);
  out.print(F(" U0=")); out.print(CAL.U0, 5);
  out.print(F(" Ke=")); out.print(CAL.Ke, 6);
  out.print(F(" Kt=")); out.print(calKt(), 6); out.print(F("(derived)"));
  out.print(F(" L=")); out.print(CAL.L*1e6f, 1); out.print(F("uH"));
  out.print(F(" ZEA=")); out.print(CAL.zea, 4);
  out.print(F(" DIR=")); out.println(CAL.dir);

  // Board scales. Both are pure multipliers on numbers the torque path uses, and
  // both are silent when wrong, so they get printed rather than assumed.
  out.print(F("  vbus_scale=")); out.print(CAL.vbus_scale, 6);
  out.print(F(" i_scale=")); out.print(CAL.i_scale, 4);
  // Kt_cmd is what a torque command must divide by. It differs from Kt ONLY
  // once M2 has produced an i_scale, and the difference IS the torque error
  // you would otherwise ship silently -- so it is printed, not hidden.
  out.print(F(" Kt_cmd=")); out.print(calKtCmd(), 6);
  if (CAL.i_scale != 1.0f) {
    out.print(F(" (torque cmds corrected by "));
    out.print(100.0f*(1.0f/CAL.i_scale - 1.0f), 2); out.print(F("%)"));
  }
  out.println();

  // Drag is stored as POSITIVE magnitudes per direction; the consumer applies
  // sign(omega). Printed per direction because the asymmetry is the finding.
  out.print(F("  drag fwd=")); out.print(CAL.drag_c_fwd, 4);
  out.print(F("A+")); out.print(CAL.drag_v_fwd, 6); out.print(F("*w"));
  out.print(F("  rev=")); out.print(CAL.drag_c_rev, 4);
  out.print(F("A+")); out.print(CAL.drag_v_rev, 6); out.print(F("*w"));
  out.print(F("  breakaway=")); out.print(CAL.breakaway_A, 4); out.println(F("A"));

  if (CAL.zea < 0.0f || CAL.dir == 0)
    out.println(F("  !! UNCALIBRATED joint -- 'f' will do a full alignment. Run AUTOCALIB."));
  else
    out.println(F("  press 'V' to verify the stored ZEA against a fresh alignment"));

  // A built joint carrying placeholder values is the dangerous case: the row
  // LOOKS calibrated, and nothing downstream can tell that these two were never
  // measured. i_scale = 1.0 puts an undetected common-mode error straight on
  // torque; breakaway = 0 silently zeroes the largest transparency term.
  if (built && CAL.i_scale == 1.0f)
    out.println(F("  !! i_scale = 1.0 (uncorrected) on a BUILT joint -- run M2, current-sense scale."));
  if (built && CAL.breakaway_A == 0.0f)
    out.println(F("  !! breakaway_A = 0 on a BUILT joint -- run M4, stiction threshold."));
}
