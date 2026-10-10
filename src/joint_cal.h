#pragma once
#include <Arduino.h>
#include <string.h>
#include "fleet_config.h"
// ============================================================================
// joint_cal.h  --  PER-JOINT CALIBRATION TABLE  (hand-entered, git-tracked)
// ============================================================================
// ONE row per physical assembly, selected at COMPILE TIME by -D JOINT_ID=n:
// you pick an ENVIRONMENT (pio run -e J01 -t upload), never edit a number.
//
// Hand-entered, not auto-saved: AUTOCALIB phase 7 prints a pasteable row, a
// human reads every value before it becomes authoritative, and each change is
// a dated git diff. Unmeasured fields are emitted as 0.0f, never a number.
//
// WHAT BELONGS TO WHAT (why the serial columns exist):
//   motor only     Ke, L, cogging
//   board only     vbus_scale, i_scale, U0
//   the PAIRING    zea, dir, R_eff, INL, drag, breakaway
// A motor <-> board swap invalidates the pairing fields.
// Fleet constants (pole pairs, encoder, Kt/Ke, gear ratio, dead_zone, PWM,
// T/T_loop) live in fleet_config.h. Kt is derived (calKt()), never stored.
//
// Unfilled row: zea = -1, dir = 0 -> runInitFOC() does a full alignment.
// After flashing, press 'V': with stored ZEA nothing else catches a wrong-joint
// flash or a slipped magnet. Procedure: docs/CALIBRATION.md §21.
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
  float  R_eff;            // ohm, whole drive path, bench ambient. A CHORD SLOPE:
                           //   R falls ~1.5% across the ladder's current range,
                           //   so the value depends on the range that measured
                           //   it. Stored = the 0.46 V ladder (0.28-1.96 A).
                           //   Do not extrapolate to 30 A. CONSTANTS §8.1d

  // ---- BOARD ----
  float  U0;               // V dead-time offset. Scales with Vbus and vbus_scale.
                           //   Real uncertainty ~+-0.003 V (~2x the fit's SE)

  // ---- MOTOR ----
  float  Ke;               // V/(rad/s).  Kt is DERIVED, not stored: calKt()
  float  L;                // H, INCREMENTAL at ~3 A. Inherits R_eff's scaling

  // ---- BOARD ----
  float  vbus_scale;       // V per ADC count. PER BOARD, M1 against a meter.
  float  i_scale;          // g = I_reported / I_true (M2). 1.0 = not measured.
                           //   Applied ONLY in the torque boundary below. Never
                           //   rescale R_eff, L or the current-loop gains by it
                           //   (they were measured in reported amps). Correcting
                           //   the sense gain at source instead would change the
                           //   A_rep unit and require re-running phases 3-4.
                           //   Ke is independent of the current-sense gain

  // ---- PAIRING, belt-state dependent (see .belt) ----
  float  drag_c_fwd;       // A            free-spin Coulomb intercept, |omega| > 0
  float  drag_c_rev;       // A            stored as POSITIVE magnitudes; the
  float  drag_v_fwd;       // A/(rad/s)    consumer applies sign(omega). The
  float  drag_v_rev;       // A/(rad/s)    fwd/rev asymmetry is real (~28% on J01)
                           //              and a single mean field discarded it
  float  breakaway_A;      // A, M4 STATIC threshold, mean over rotor positions
                           //    (n, sd, direction split in the row comment).
                           //    Belt off: > drag_c. Belt on: ~= drag_c.
};

// vbus_scale is PER BOARD (J01 and J02 measured 0.80% apart) and R_eff, U0 and
// Ke all scale with it. Run M1 against a cross-checked meter BEFORE AUTOCALIB.
// Unbuilt rows carry 0.0f, never another board's value.

// Kt is a CONVENTION, not a measurement (see KT_PER_KE in fleet_config.h).
// Storing both invites a partial edit that leaves them inconsistent with
// nothing to catch it, so Kt is computed on demand instead.
static inline float calKt(const JointCal& c) { return c.Ke * KT_PER_KE; }

// ---------------------------------------------------------------------------
// THE FLEET.  Paste AUTOCALIB output here, one row per assembly.
// ---------------------------------------------------------------------------
const JointCal JOINTS[] = {

  // -- J01 -- B-SPI-01 / M-SPI-01, the reference actuator. BELT-ON row (B11),
  //    pulley recipe B, 2026-09-30 (BELT_DRIVE §22.6). -e J01 flashes this.
  //    The belt-off baseline is the LAST row of the table (index 13).
  //    zea, dir, R_eff, U0, Ke, L: belt-off AUTOCALIB 2026-08-07, rescaled
  //    x1.010768 by M1 2026-08-18. i_scale: M2 2026-08-20, 0.9621 +-1.2%.
  //    Belt-on Ke is drag-contaminated and never carried.
  //    Belt-on tripwires 2026-09-30: V 0.62 deg elec; phase 3 R 0.22404 (a
  //    different chord of the same curved R -- not a reason to edit).
  //    DRAG (B3): phase 5 in both orders; each direction pooled over orders,
  //    because the direction run second reads 0.10-0.13 A higher:
  //        fwd first   fwd 0.2265 + 0.004009*|w|   rev 0.3576 + 0.002597*|w|
  //        rev first   rev 0.3084 + 0.002730*|w|   fwd 0.4072 + 0.002466*|w|
  //    BREAKAWAY (B4): 10 positions x 2 dirs, mean 0.295 A, SEM 0.027, range
  //    0.080-0.475 (one reading over the 200-count creep flag; 0.286 without it).
  //    Not fields: backlash G 5.8 counts; k 67.2 kN/m true at its clamp; ring
  //    f_d 65.0 Hz.
  //    Belt-off diagnostics: T/T_loop 0.974 / 0.945; INL 1.029 deg mech pk-pk
  //    (1/rev 0.369, 2/rev 0.204); ZEA residual 0.467 deg elec.
  //    M2 fit: P_bus = 0.8027 + 0.0348*I + 0.34816*I^2;  U_del = 0.01350 + 0.22332*I.
  { "J01", "B-SPI-01", "M-SPI-01", "2026-09-30", "10mm-9:1",
     6.0542f, +1, 0.22346f,        // zea, dir, R_eff      CARRIED (B1, B2 pass)
     0.01037f,                     // U0                   CARRIED
     0.017941f, 43.77e-6f,         // Ke, L                CARRIED -- belt-on Ke never
     0.008448f, 0.9621f,           // vbus_scale, i_scale  CARRIED
     0.3169f, 0.3330f,             // drag_c fwd, rev      RECIPE B, B3 pooled over both orders
     3.24e-3f, 2.66e-3f,           // drag_v fwd, rev      RECIPE B, B3 pooled over both orders
     0.295f },                     // breakaway_A          RECIPE B, B4 n=20 (reported A)

  // -- J02 -- B-ABZ-01 / M-ABZ-01 (the rebuilt A1). Belt-off AUTOCALIB
  //    2026-08-08, rescaled x1.018904 by M1 2026-08-18. board_sn / motor_sn
  //    still unlabelled (docs/README.md §15).
  //    The ESC board failed after P8; the MT6816 board was found dead 2026-10-08
  //    (MISO stuck low) and replaced -- zea/dir belong to the OLD encoder.
  //    R_eff 0.22810 is the 0.28-1.96 A chord; phase 3 on the current 0.68 V
  //    ladder reads ~0.222 on the same hardware -- not a fault.
  //    vbus_scale 0.008516 carries an unresolved 0.32% ambiguity: two banner
  //    pairings imply 0.008489 / 0.008516, below the UT89X's resolution.
  //    i_scale 0.9690 +-2.7%, PROVISIONAL (1.15 sigma). M2 fit:
  //        P_bus = 0.7924 + 0.0982*I + 0.34953*I^2;  U_del = 0.02419 + 0.22580*I
  //    M4: n = 18 over 6 positions, 0.2983 +-0.0259 A, sd 0.1098.
  //    Diagnostics: INL 1.449 deg mech pk-pk (1/rev 0.311, 2/rev 0.519 -- tilt
  //    or channel gain, not centring); ZEA residual 0.240 deg elec; T/T_loop
  //    0.937 / 0.961; M6a J 18.7e-6 (see J_ROTOR_KGM2).
  { "J02", "___", "___", "2026-08-08", "OFF",
     0.3482f, +1, 0.22810f,        // zea, dir, R_eff  @ 0.28-1.96 A  M1-rescaled x1.018904
     0.014937f,                    // U0   M1-rescaled
     0.018097f, 46.25e-6f,         // Ke, L   (Kt = calKt() = 0.027145)  M1-rescaled
     0.008516f, 0.9690f,           // vbus_scale (M1, 0.32% ambiguity above), i_scale PROVISIONAL
     0.1061f, 0.1074f,             // drag_c fwd, rev      unchanged (reported A)
     0.000948f, 0.000845f,         // drag_v fwd, rev      unchanged (reported A)
     0.2983f },                    // breakaway_A  M4, n=18, +-8.7%  (reported A)
  // -- J03 -- board_3 (new) + motor_2 (J02's motor) + a new MT6816 board. Bare
  //    motor, belt off. AUTOCALIB 2026-10-08 (banner 11.98 V vs UT89X 11.97 V):
  //    zea sd 3.5 deg (n = 7); R_eff +-0.75%; INL 1.42 deg mech (1/rev, centring).
  //    vbus_scale: M1 on the SEED path, meter at the pads, 12.02 / 22.70 V; the
  //    two single-point ratios agree to 0.005%. Do not derive M1 from the DMA
  //    buffer: it gave 0.008302, 0.66% low (offset -58 / -49 counts, not constant).
  //    Ke 0.017900 is 1.09% below J02's on the same motor; stored as measured
  //    (J02's 0.32% scale ambiguity points this way).
  //    M2 2026-10-09: g 0.9797 +-1.73%, PROVISIONAL; self-fit R 0.21609, c 0.33086.
  //    M4 2026-10-10: 0.2545 A, n = 10, sd 0.078, fwd 0.243 / rev 0.266.
  //    Raw: docs/cal/J03/.
  { "J03", "board_3", "motor_2", "2026-10-08", "OFF",
     0.3866f, +1, 0.23068f,        // zea, dir, R_eff
     0.01919f,                     // U0
     0.017900f, 45.75e-6f,         // Ke, L   (Kt = calKt() = 0.026850)
     0.008357f, 0.9797f,           // vbus_scale (M1 2026-10-08), i_scale (M2 2026-10-09, PROVISIONAL)
     0.0935f, 0.1058f,             // drag_c fwd, rev   (reported A, belt OFF, bare motor)
     0.001143f, 0.000713f,         // drag_v fwd, rev
     0.2545f },                    // breakaway_A  (M4 2026-10-10, bare motor)
  // -- J04 .. J12 -- NOT BUILT. Every measurable field 0.0f = not measured;
  //    zea -1 / dir 0 force a full alignment; i_scale 1.0 is its inert value.
  //    New joint: docs/cal/BELT_OFF_BASELINE.md (M1, AUTOCALIB, paste, M2, M4).
  { "J04", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J05", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J06", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J07", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J08", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J09", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J10", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J11", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
  { "J12", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },

  // -- A1 -- HISTORICAL; the hardware was rebuilt into J02. Kept at index 12 so
  //    no JOINT_ID shifts. Measured with a rubbing encoder magnet, so every
  //    friction number is contaminated (drag_c 1.05 A = belt + rub). Deliberately
  //    not rescaled by M1 (voltage-derived values read 1.89% low). Inherit
  //    nothing. -e A1 fails to build on purpose (ENCODER_ABZ). Drag is belt-on
  //    and not direction-split; breakaway was never averaged, hence 0.
  { "A1",  "B-ABZ-01", "M-ABZ-01", "2026-07-29", "10mm-9:1",
    -1.0f, 0, 0.218f,              // zea, dir, R_eff
     0.028f,                       // U0
     0.0177f, 65e-6f,              // Ke, L        (Kt = calKt() = 0.02655)
     0.008358f, 1.0f,              // vbus_scale (INHERITED, not measured), i_scale
     1.05f, 1.05f,                 // drag_c fwd, rev   -- belt ON, not split
     0.0f, 0.0f,                   // drag_v fwd, rev   -- not fitted
     0.0f },                       // breakaway_A       -- band 0.34-1.34 A, no mean

  // -- J01 BELT-OFF BASELINE -- index 13 (JOINT_ID 14), moved verbatim from
  //    index 0 by B11 on 2026-09-28. Every belt-on figure is a difference against
  //    it. No env points here; add -D JOINT_ID=14 only to re-read this plant.
  { "J01", "B-SPI-01", "M-SPI-01", "2026-08-07", "OFF",
     6.0542f, +1, 0.22346f,        // zea, dir, R_eff      M1-rescaled
     0.01037f,                     // U0                   M1-rescaled
     0.017941f, 43.77e-6f,         // Ke, L   (Kt = calKt() = 0.026912)  M1-rescaled
     0.008448f, 0.9621f,           // vbus_scale (M1 2026-08-18), i_scale (M2 2026-08-20)
     0.0750f, 0.0816f,             // drag_c fwd, rev      unchanged (reported A)
     9.33e-4f, 7.27e-4f,           // drag_v fwd, rev      unchanged (reported A)
     0.292f },                     // breakaway_A          unchanged (reported A)
};

static constexpr uint8_t JOINT_COUNT = (uint8_t)(sizeof(JOINTS) / sizeof(JOINTS[0]));

// ---------------------------------------------------------------------------
// BOARD IDENTITY -- the STM32 96-bit UID of the board each row was measured on.
// ---------------------------------------------------------------------------
// Tier 0 refuses to ARM unless the running MCU's UID matches (J01's binary on
// J03's board would commutate on J01's ZEA: reduced or reversed torque inside
// a position loop). It prints its own UID at boot, so a new row is a paste.
// {0,0,0} = not recorded -> refuses. A parallel table, not a JointCal field, so
// the harness links without it; the static_assert and the .id string keep it
// in step with JOINTS[].
struct JointUid {
  const char* id;          // must equal JOINTS[same index].id
  uint32_t    w[3];        // HAL_GetUIDw0/1/2(), as printed in the Tier-0 banner
};
const JointUid JOINT_UID[] = {
  { "J01", { 0x460030, 0x34354B0F, 0x30373336 } },  // B-SPI-01 -- T0_J01 boot banner, 2026-10-04 (N1)
  { "J02", { 0, 0, 0 } },  // B-ABZ-01 -- board failed after P8 (7d)
  { "J03", { 0x52003E, 0x34354B0C, 0x33383735 } },   // T0 boot 2026-10-10, board_3
  { "J04", { 0, 0, 0 } },
  { "J05", { 0, 0, 0 } },
  { "J06", { 0, 0, 0 } },
  { "J07", { 0, 0, 0 } },
  { "J08", { 0, 0, 0 } },
  { "J09", { 0, 0, 0 } },
  { "J10", { 0, 0, 0 } },
  { "J11", { 0, 0, 0 } },
  { "J12", { 0, 0, 0 } },
  { "A1",  { 0, 0, 0 } },  // historical, not flashable
  { "J01", { 0x460030, 0x34354B0F, 0x30373336 } },  // J01 belt-off baseline: same board (B-SPI-01) as index 0
};
static_assert(sizeof(JOINT_UID) / sizeof(JOINT_UID[0]) == JOINT_COUNT,
              "JOINT_UID[] must have exactly one entry per JOINTS[] row");

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
// THE TORQUE BOUNDARY -- the only place i_scale is applied
// ---------------------------------------------------------------------------
// Unit rule (2026-10-01, CONSTANTS §8.1c). Two units meet ONLY here:
//   A_rep   the REPORTED amp, the firmware's current unit. Every measured
//           constant (R_eff, L, drag, breakaway, PI gains) and every
//           bench-demonstrated limit is A_rep and is never converted.
//   N.m     OUTPUT torque, tau = GEAR_RATIO * Kt * I_true. DRIVETRAIN_ETA is
//           excluded and friction is not subtracted (the contract's definition).
// Commands AND feedback both cross here, so a commanded and a measured tau
// cannot disagree by g.
//
// i_scale = g = I_reported / I_true. Kt is clean (Ke is fit from voltage and
// speed, independent of the current-sense gain), so at the motor shaft:
//     Iq [A_rep] = tau_motor / calKtCmd()     tau_motor = Iq [A_rep] * calKtCmd()
// The current LOOP needs no correction: R_eff and L were measured in A_rep, so
// the plant from volts to A_rep is already exact. Dividing them or the gains by
// i_scale would double-count g.
//
// Callers: the MIT law (harness and Tier 0) through tauOutCmdToIq(); measured
// tau, the boot banner, the swing ladder's k and autocalib's force print
// through irepToTorqueOut(). Do not call calKtCmd() from control code: a bare
// "/ calKtCmd()" drops GEAR_RATIO. Units: N.m of MOTOR torque (true) per A_rep.
static inline float calKtCmd() {
  return (CAL.i_scale > 0.0f) ? (calKt() / CAL.i_scale) : calKt();
}

// Output torque [N.m] <-> Iq [A_rep]. An unbuilt row has Ke = 0, so Kt_cmd = 0:
// the pair returns 0 (no torque on an uncalibrated joint) rather than inf/NaN,
// and tauOutCmdToIq() flags it. Sign: a belt does not reverse direction, so
// positive motor Iq = positive output torque. Any per-joint MOUNTING sign
// (mirrored legs) is a contract question for the leg, not applied here.
static inline float torqueOutToIrep(float tau_out_Nm) {
  const float nm_per_A_rep = GEAR_RATIO * calKtCmd();
  return (nm_per_A_rep > 0.0f) ? (tau_out_Nm / nm_per_A_rep) : 0.0f;
}
static inline float irepToTorqueOut(float iq_A_rep) {
  return iq_A_rep * GEAR_RATIO * calKtCmd();
}

// THE COMMAND CLAMP CHAIN. Every torque command reaches the current loop through
// this and nothing else:
//
//   tau_out_Nm --[non-finite, bad limit or uncalibrated -> 0]
//              --> clamp |tau| <= tau_max_Nm        (OUTER: the contract, N.m)
//              --> torqueOutToIrep()
//              --> clamp |Iq|  <= iq_cap_A_rep      (INNER: the demonstrated envelope)
//              --> current loop
//
// The inner clamp is in the unit the envelope was DEMONSTRATED in: D3's 1.6 A
// was proven by the swing ladder in reported amps (BELT_DRIVE §22.4.9). It sits
// LAST so that a wrong Kt, i_scale or gear ratio can never command beyond what
// the bench has actually survived. Set tau_max so the OUTER clamp binds first;
// then an inner hit (IQ_CLAMP_IQ) means the conversion and the envelope
// disagree, which is a finding, not a saturation -- log it as such.
//
// A COMMAND LIMIT, NOT A STOP. It never disarms and it adds no e-stop call
// site: faults still go through safety.h's stopMotor() and nowhere else.
// Limits are passed, never defaulted -- the bench harness and Tier-0 each own
// their envelope. A limit that is not finite and > 0 yields zero torque.
enum IqClamp : uint8_t {
  IQ_CLAMP_NONE   = 0,
  IQ_CLAMP_TAU    = 1,   // outer clamp bound (expected under saturation)
  IQ_CLAMP_IQ     = 2,   // inner clamp bound (conversion vs envelope -- a finding)
  IQ_CLAMP_REJECT = 3,   // non-finite command, bad limit, or Kt_cmd = 0 -> 0 A_rep
};
struct IqCmd {
  float   iq_A_rep;
  IqClamp clamp;
};
static inline IqCmd tauOutCmdToIq(float tau_out_Nm, float tau_max_Nm, float iq_cap_A_rep) {
  IqCmd r = { 0.0f, IQ_CLAMP_REJECT };
  // `!(x > 0)` is deliberate: it is also true for NaN.
  if (!isfinite(tau_out_Nm) || !isfinite(tau_max_Nm) || !isfinite(iq_cap_A_rep)
      || !(tau_max_Nm > 0.0f) || !(iq_cap_A_rep > 0.0f) || !(calKtCmd() > 0.0f))
    return r;
  r.clamp = IQ_CLAMP_NONE;
  if      (tau_out_Nm >  tau_max_Nm) { tau_out_Nm =  tau_max_Nm; r.clamp = IQ_CLAMP_TAU; }
  else if (tau_out_Nm < -tau_max_Nm) { tau_out_Nm = -tau_max_Nm; r.clamp = IQ_CLAMP_TAU; }
  float iq = torqueOutToIrep(tau_out_Nm);
  if      (iq >  iq_cap_A_rep) { iq =  iq_cap_A_rep; r.clamp = IQ_CLAMP_IQ; }
  else if (iq < -iq_cap_A_rep) { iq = -iq_cap_A_rep; r.clamp = IQ_CLAMP_IQ; }
  r.iq_A_rep = iq;
  return r;
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
    // Two different percentages: the torque error you would ship uncorrected
    // (1/i_scale - 1) and how the boundary scales commands (i_scale - 1).
    out.print(F(" (uncorrected torque error "));
    out.print(100.0f*(1.0f/CAL.i_scale - 1.0f), 2);
    out.print(F("%; the boundary scales cmds "));
    if (CAL.i_scale > 1.0f) out.print('+');
    out.print(100.0f*(CAL.i_scale - 1.0f), 2);
    out.print(F("%)"));
  }
  out.println();
  // The boundary's conversion, printed as a number so a wrong gear ratio, Ke or
  // i_scale is visible at boot instead of on the bench. J01 expects 0.2517.
  out.print(F("  torque boundary: 1 A_rep = ")); out.print(irepToTorqueOut(1.0f), 4);
  out.println(F(" N.m output (true; eta excluded)"));

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
    out.println(F("  !! i_scale = 1.0 (NOT MEASURED) on a BUILT joint -- run M2 (built boards read 0.96-0.98)."));
  // vbus_scale = 0 is the correct state for an UNBUILT row and a defect on a
  // built one: R_eff, U0 and Ke were all measured THROUGH it, so a built joint
  // without M1 is carrying constants scaled by someone else's divider. That is
  // the failure that cost 1.1% on J01 and 1.9% on J02.
  if (built && CAL.vbus_scale == 0.0f)
    out.println(F("  !! vbus_scale = 0 on a BUILT joint -- run M1. R_eff/U0/Ke all scale with it."));
  // DRIVETRAIN_ETA sits in the same category as i_scale: an unverified
  // multiplicative factor on every force this project quotes. It is announced
  // next to i_scale so a built joint says out loud that BOTH are outstanding,
  // rather than one being visible and the other buried in a header.
  if (built)
    out.println(F("  !! DRIVETRAIN_ETA is back-solved, not measured (M14)."));
  if (built && CAL.breakaway_A == 0.0f)
    out.println(F("  !! breakaway_A = 0 on a BUILT joint -- run M4, stiction threshold."));
}
