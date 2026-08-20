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
//     pio run -e J02 -t upload
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
                           //   R_eff IS NOT A SINGLE NUMBER. Measured 2026-08-20:
                           //   ~0.2173 ohm at 3.05 A rising to ~0.2204 at 1.06 A,
                           //   1.5% of curvature -- real, not noise, and the
                           //   per-board amount differs (J01 -0.99%, J02 -1.59%
                           //   between the 0.68 V and 0.40 V ladder tops). So
                           //   "the" R_eff depends on WHICH CURRENT RANGE
                           //   measured it. The stored values are the 0.46 V
                           //   ladder. M2 is immune: it uses its own
                           //   range-matched R_M2 from the same session

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
  float  i_scale;          // I_true = I_sensed / i_scale.  M2.  1.0 = uncorrected,
                           //   and after 2026-08-20 that means NOT MEASURED --
                           //   both built joints carry a real g of ~0.962-0.969.
                           //   STORING a measured value here does NOT invalidate
                           //   the stored R_eff or L, and you must NOT rescale
                           //   them or the current-loop gains -- see the calKtCmd()
                           //   note below for why. It is applied in exactly ONE
                           //   place: the torque -> current conversion.
                           //   The DIFFERENT case, easily confused with it: if
                           //   the sense gain is corrected AT SOURCE (in the
                           //   LowsideCurrentSense constructor) then the
                           //   reported-amp UNIT itself moves, R_eff and L both
                           //   shift by 1/g, and phases 3-4 MUST be re-run.
                           //   Ke is independent of current-sense gain either way

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

// !!! vbus_scale IS NOT A FLEET CONSTANT -- NOW MEASURED, NOT ESTIMATED !!!
// It is a resistor divider, and R_eff, U0 and Ke ALL scale linearly with it, so
// the error lands directly on every torque command.
//
// M1, 2026-08-18, both built boards against the SAME meter (UT89X):
//     B-SPI-01 (J01)  0.008448          B-ABZ-01 (J02)  0.008516
//                     ------ 0.80% apart, MEASURED ------
// This supersedes the old "(probably) 1% tolerance, so up to ~2% board to board"
// estimate that used to sit here. It is the first time this table has MEASURED a
// per-board divider difference instead of warning about one, so "M1 is mandatory
// per board" is a demonstrated fact, not a precaution.
//
// It cost real numbers, twice, from the same root cause -- one board's divider
// treated as if it were fleet:
//   * J01's own 0.008358 was calibrated against a DT9205A whose DCV gain error
//     was later measured at ~1.11%. Every J01 constant moved x1.010768.
//   * J02 INHERITED that 0.008358 from B-SPI-01. Every J02 constant moved
//     x1.018904 -- the meter error AND the board difference, compounded.
//
// UNBUILT ROWS THEREFORE CARRY 0.0f, NOT A PLACEHOLDER. See the J03..J12 block.

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
  //
  //    *** RESCALED 2026-08-18 BY M1. R_eff, U0, Ke and L below are the
  //    2026-08-07 fits MULTIPLIED BY 1.010768. They were NOT re-measured. ***
  //    M1: banner 12.39 at vbus_scale 0.008489, UT89X 12.33 V -> true 0.008448.
  //    The ORIGINAL 0.008358 was calibrated against a DT9205A, and
  //        0.008358 x 1.0111 (that meter's measured DCV gain error) = 0.0084507
  //        measured true                                            = 0.0084479
  //    -- 0.03% apart. The old M1 error is FULLY explained by the meter with no
  //    residual, which is an independent confirmation of the meter verdict by a
  //    completely different route. (An earlier reading of this attributed the
  //    excess to leftover error in the 2-point fit. That was wrong: the fit was
  //    fine, the meter was not.)
  //    tau_e = L/R is 195.87 us, was 195.90: L and R scale TOGETHER, so THE
  //    CURRENT-LOOP GAINS ARE UNTOUCHED (see the boxed warning in CONTROL.md
  //    section 10). Plant DC gain moves 1.1%, negligible against zeta = 0.60.
  //
  //    L SUPERSEDES 44.43 / 44.8 / 45.1 uH. Those came from 5- and 7-point fits on
  //    a phase-locked sample grid and were biased +4% in tau. This is an 18-point
  //    dithered fit, tau = 195.87 us, verified by independent offline refit.
  //    (The superseded figures are at the OLD scale and are left that way -- the
  //    comparison is within one session and the M1 factor is common-mode.)
  //
  //    U0 is 6.5 sigma in THIS fit but does NOT reproduce: 0.00367 / 0.00559 /
  //    0.01026 V across three sessions with per-fit SE ~0.0016 (old scale;
  //    x1.010768 = 0.00371 / 0.00565 / 0.01037, which is what the row stores).
  //    Real uncertainty is +-0.003 V. Harmless, and the RESCALE DOES NOT REACH
  //    THE DEADBAND -- it is U0/R, a ratio, so the factor cancels exactly:
  //    0.01037/0.22346 = 0.01026/0.22108 = 0.046 A = 0.11 N at the foot, and
  //    0.078 A = 0.18 N projected to a 5S bus. Does NOT reopen dead_zone.
  //
  //    Ke's 0.57% session spread tracks bus voltage (12.52 -> 12.46) to 0.09%,
  //    not the motor. Kt = calKt() = 0.026912 vs nameplate KV360 -> 0.026526,
  //    +1.45% (implied KV 354.8).
  //    *** RETRACTED 2026-08-18: "+0.38%, which validates the VOLTAGE scale". ***
  //    It never could. Ke is COMPUTED from vbus_scale, so a Kt-vs-nameplate
  //    comparison cannot referee the scale it was derived from -- and the tight
  //    +0.38% was a coincidence produced by a divider that was 1.1% low. M1
  //    against a traceable meter is the check; this line is a NAMEPLATE
  //    cross-check only. It still excludes KV380 (+7.1%) by a wide margin, so
  //    README section 24.1 does not reopen -- see the note there on what the
  //    rescale did to the agreement figures.
  //
  //    DRAG: fwd/rev asymmetry is real and reproducible (viscous 9.33e-4 fwd vs
  //    7.27e-4 rev, 28%). The old single-mean pair (0.0783, 0.000830) discarded
  //    it and has been replaced by the four fields below.
  //
  //    BREAKAWAY: M4, 2026-08-08, belt OFF, leg off. Pooled mean 0.2923 A,
  //    sd 0.0611 (20.9%), SE 0.0184 (6.3%). In REPORTED amps that is 7.87
  //    mN.m at the motor (0.2923 x Kt 0.026912); in TRUE amps, 8.18 -- see the
  //    force paragraph below. ONE torque figure per unit system now: this
  //    comment once carried BOTH 7.52 and 7.78 mN.m two lines apart, and
  //    neither matched its own Kt.
  //    THE 20.9% SCATTER IS THE PLANT, NOT THE METHOD. Two free coast-downs the
  //    same afternoon implied friction 1.24x and 0.83x the drag map -- +-20%,
  //    from completely different physics. Grease redistribution: a rub or a bent
  //    shaft would be REPRODUCIBLE. Do not re-run chasing a tighter number, and
  //    do not read a single trial as better than +-20%.
  //    Budget closes with no fault: cogging 1.48 + two greased 5-6 mm bearings
  //    2-6 + assembly preload 0-3 = 3.5-10.5 mN.m, measured 8.18 inside it.
  //    IN TRUE AMPS (i_scale 0.9621, M2 2026-08-20): breakaway is 0.3038 A
  //    true = 8.18 mN.m/motor -> 1.322 N PER LEG (two motors, eta 0.92) =
  //    13.5% of a 9.81 N standing load -- 3.4x better than A1's 46% belt-ON.
  //    At i_scale = 1.0 this read 7.87 mN.m / 1.271 N / 12.95%; the sense
  //    UNDER-reads by 3.9%, so every force here went UP, not down.
  //    (An earlier note read 1.37 N / 13.9%: that omitted eta, while A1's
  //    4.5 N included it, so the comparison was against inconsistent units.
  //    The number is back near 13.5% for a completely unrelated reason.)
  //
  //    COGGING: 1.48 mN.m true (1.42 at i_scale = 1.0), from the fwd/rev
  //    breakaway pair at the same rotor
  //    position. That is 13x the 0.004 A (0.112 mN.m) free-spin figure the
  //    README carried as "negligible, closed" -- see README section 8.2 for why
  //    a spinning measurement structurally cannot see it.
  //
  //    THE FORCE FIGURES ABOVE HAVE BEEN MOVED TWICE, BY TWO DIFFERENT
  //    MEASUREMENTS, AND BOTH ARE NOW DONE:
  //      2026-08-18  M1  x1.010768  (vbus_scale -- Kt was 1.1% low)
  //      2026-08-20  M2  /0.9621    (i_scale -- the sense under-reads 3.9%)
  //    Net on every newton on this joint: x1.0505. The +-(0-6)% "provisional"
  //    band that used to sit on every absolute force is CLOSED and replaced by
  //    +-1.2% (1 sigma), which is M2's own budget.
  //    Still immune, and worth restating because it is what makes the two
  //    corrections tractable: anything in REPORTED AMPS or RADIANS (drag_c,
  //    drag_v, breakaway_A, zea, INL, T/T_loop, every current LIMIT), and every
  //    ratio or percentage-of-a-percentage, where both factors cancel.
  //
  //    DIAGNOSTICS, not fields: T/T_loop 0.974 & 0.945 (T = 73.0 us at 13.3 kHz;
  //    0.95% torque loss at 270 rad/s) | INL 1.029 deg mech pk-pk = 1/rev 0.369 +
  //    2/rev 0.204 deg mech | ZEA residual 0.467 deg elec | |I| ratio 1.245-1.313
  //    | J_rotor 20.2e-6 kg.m^2 -- FLEET, in fleet_config.h, not a field here.
  //    M2, 2026-08-20 -- i_scale = 0.9621, DECISIVE. Bus-power ladder, SELF-FIT
  //    formulation: g = 1.5 * R_M2 / c, where R_M2 is the ladder's OWN
  //    U_delivered-vs-I slope from the SAME session over the SAME current range,
  //    NOT a cross-session phase 3. That change is what removed the last
  //    cross-session dependency from M2 and it is why no bypass clip is needed.
  //        P_bus = 0.8027 + 0.0348*I + 0.34816*I^2     SE(c) = 0.66%
  //        U_del = 0.01350 + 0.22332*I                 SE(R) = 0.27%, rms 0.86 mV
  //        g     = 1.5 * 0.22332 / 0.34816 = 0.9621    +-1.2% (1 sigma)
  //    3.2 SIGMA FROM UNITY. The sense UNDER-reads: I_true = I_reported/0.9621,
  //    so true current is 3.9% HIGHER than reported, and torque delivered at
  //    i_scale = 1.0 was 3.9% OVER commanded.
  //    ALL THREE COEFFICIENTS VALIDATE INDEPENDENTLY, which is the actual
  //    evidence the fit is clean rather than merely well-conditioned:
  //        a  0.803 W fitted vs 0.688 W idle (56.3 mA x 12.22) + ~0.12 W PWM
  //           ripple loss -- ripple is load-independent and belongs in a
  //        b  +0.035 W/A fitted vs 1.5*U0 = 0.020 W/A dead-time conduction --
  //           right order, right sign
  //        U0 0.01350 V self-fit vs 0.01686 V phase-3 cold -- inside the known
  //           +-0.003 V
  //    ERROR BUDGET, ranked -- the point of listing it is that a REPEAT CANNOT
  //    HELP: UT89X 600 mA DCA gain 0.8% (SYSTEMATIC, does not average down) |
  //    SE(c) 0.66% | point-1 Ibus read to 1 mA not 0.1, 0.4% | intra-ladder
  //    heating 0.3% | SE(R_M2) 0.27% | R_b, V_s, seed rounding <0.1%.
  //    RSS 1.18%; a second ladder buys 0.13% and cannot move the verdict.
  //    R_b = 1.547 ohm (4-wire, UNPOWERED -- the powered 1.539 reads the shunt
  //    in parallel with the live loop and is biased low; 0.5% apart, moot).
  //    *** R_b WAS NEVER THE DOMINANT UNKNOWN -- an earlier claim that it gated
  //    this is RETRACTED. *** It enters R_M2 (via U_delivered) AND c (via
  //    P_bus), so it largely cancels in the ratio: +-16% on R_b moves g by
  //    +-0.9%, and the +-0.6% measurement moves it +-0.03%. V_s is more inert
  //    still (12.24 -> 12.34 moves g by 0.0004), because a uniform voltage-scale
  //    error cancels exactly -- which is also why the 0.02 V banner-vs-meter gap
  //    does NOT justify touching vbus_scale.
  { "J01", "B-SPI-01", "M-SPI-01", "2026-08-07", "OFF",
     6.0542f, +1, 0.22346f,        // zea, dir, R_eff      M1-rescaled, see below
     0.01037f,                     // U0                   M1-rescaled
     0.017941f, 43.77e-6f,         // Ke, L   (Kt = calKt() = 0.026912)  M1-rescaled
     0.008448f, 0.9621f,           // vbus_scale (M1 2026-08-18), i_scale (M2 2026-08-20)
     0.0750f, 0.0816f,             // drag_c fwd, rev      unchanged (reported A)
     9.33e-4f, 7.27e-4f,           // drag_v fwd, rev      unchanged (reported A)
     0.292f },                     // breakaway_A          unchanged (reported A)

  // -- J02 -- WAS A1, the legacy ABZ assembly. Same motor and same board: the
  //    lost-count fault was diagnosed as mechanical jitter from a RUBBING encoder
  //    magnet, the magnet was remounted and an MT6816 SPI board fitted.
  //    AUTOCALIB rev2, 2026-08-08. VERDICT PASS, all six phases.
  //
  //    !! FILL IN board_sn AND motor_sn. Write the new labels on the PHYSICAL
  //    parts first -- this hardware still carries A1-era markings, and the header
  //    rule is that .id must match what is written on the board.
  //
  //    *** RESCALED 2026-08-18 BY M1. R_eff, U0, Ke and L below are the
  //    2026-08-08 fits MULTIPLIED BY 1.018904. They were NOT re-measured. ***
  //    THE 2026-08-08 ROW WAS CALIBRATED ON B-SPI-01'S DIVIDER, inherited. This
  //    board is B-ABZ-01 and its own divider is 0.008516 V/count. Two pairings
  //    against the same UT89X reading of 12.33 V:
  //        session 1   banner 12.14 at scale 0.008358  -> implies 0.008489
  //        session 2   banner 12.29 at scale 0.008489  -> implies 0.008516
  //    Those cannot both be right. banner/V_true is a FIXED property of a board
  //    at a fixed scale; it cannot be 1.000 in one sitting and 0.997 in the
  //    next. CONFIRMED 2026-08-18 as ONE sitting with the pack at 12.33 V for
  //    both, which makes session 2 the valid pairing and 0.008516 the stored
  //    value. Session 1's banner was then ~4.7 counts (40 mV) low -- consistent
  //    with the seed being a SINGLE unaveraged pre-init analogRead at 8.5
  //    mV/count on a switching board, and with boot-time inrush latching the
  //    seed while the bulk caps still charge.
  //    THE RESIDUAL 0.32% IS NOT RESOLVED, IT IS BOUNDED. It sits below the
  //    UT89X's own +-0.66% at 12.33 V on the 60 V range, so these instruments
  //    cannot separate it. Recorded in README section 24.14 rather than
  //    silently absorbed. The open item is the BANNER'S OWN REPEATABILITY --
  //    5x power-cycle, nothing else touched, record the banner each time --
  //    because until that exists there is no noise floor to judge a 0.32% shift
  //    against, and every future banner comparison is uninterpretable.
  //    RETRACTED: "same banner as J01 on the same pack, so the divider is
  //    VERIFIED, not recalibrated." The banner is a live measurement of the
  //    PACK, not a constant, and the two dividers have now been measured 0.80%
  //    apart. CLOSED: the old DT9205A-vs-RC3563 "UNRESOLVED" note -- M1 was
  //    redone against a UT89X and the DT9205A's ~1.11% DCV gain error is the
  //    measured explanation for the original 0.008358.
  //    tau_e = L/R is 202.74 us, was 202.75: L and R scale TOGETHER, so THE
  //    CURRENT-LOOP GAINS ARE UNTOUCHED (boxed warning, CONTROL.md section 10).
  //
  //    *** ALL A1-ERA CONSTANTS ARE SUPERSEDED BY THIS ROW. ***
  //    L: A1 read 65 uH / tau_e 300 us on the pre-dither 5-point fit. The SAME
  //    motor and board now read 46.25 uH / 202.7 us on an 18-point dithered fit,
  //    5.7% from J01's 43.77. So 65 uH was a FIT ARTEFACT, not a different motor,
  //    and L is a near-fleet quantity at ~45 uH. This is the cleanest evidence
  //    yet that the phase-locked sampling bias was real. Offline refit of the LSB
  //    block gives tau 206.3 us against the firmware's 202.8 -- the fit band edge
  //    shifts one bin -- so do NOT quote L to better than +-2%. (The J02-vs-J01
  //    L gap widened 4.8% -> 5.7% at the corrected scales, and the ~44 -> ~45 uH
  //    fleet figure moved with it. tau is IMMUNE: it is a ratio.)
  //    U0: A1 read 0.028 V; the same board now reads 0.014937.
  //    A1's own constants are NOT rescaled -- see the note on that row. They
  //    were taken at the 0.008358 belief, so on this board's true divider they
  //    would all read 1.89% higher (0.028 -> 0.0285, 0.0177 -> 0.018034).
  //    drag: A1's 1.05 A was belt-ON *and* rubbing. NOT the M5 belt-drag prior.
  //
  //    *** RETRACTED 2026-08-18 -- the R-vs-Ke argument that used to sit here. ***
  //    It read: "R_eff is 1.26% above J01 while Ke is only 0.06% above. BOTH
  //    scale with the firmware's voltage belief, so a scale error would have
  //    moved them TOGETHER. It did not -> the R difference is REAL." That
  //    assumed the two boards shared ONE scale. They do not. A per-board scale
  //    difference moves R and Ke together WITHIN a board, so a CROSS-board
  //    comparison never tested what it claimed. At the corrected scales:
  //        Ke     J02 vs J01     +0.06%  ->  +0.87%
  //        R_eff  J02 vs J01     +1.26%  ->  +2.08%
  //    The R difference may well still be real (FET Rds_on, shunt, solder
  //    joints, wire length) -- but this row no longer contains an argument that
  //    it is, and "the first time the fleet table separated those two
  //    explanations" goes with it. What actually separated them was M1.
  //    The 0.87% Ke gap now sits above J01's own 0.57% session spread, so
  //    "tighter than J01's own session spread" is also gone; it is still
  //    consistent with two same-model motors, but it is no longer remarkable.
  //
  //    STEELMAN, recorded because it is the one piece of evidence AGAINST the
  //    J01 correction: two independently manufactured motors agreeing on Ke to
  //    0.06% is suspiciously good, and it happens only if the two dividers are
  //    identical. Direct measurement outranks a cross-joint coincidence -- but
  //    this gives a FALSIFIABLE PREDICTION. Re-measure both banner/meter ratios
  //    in ONE sitting: if the corrections are real the Ke gap comes back at
  //    0.87%. If it comes back at 0.06%, the dividers are equal and it is the
  //    M1 reads that are wrong.
  //
  //    Kt = calKt() = 0.027145 vs nameplate KV360 -> 0.026526, +2.33% (implied
  //    KV 351.8; J01 +1.45%). NAMEPLATE cross-check ONLY -- it cannot referee
  //    vbus_scale, because Ke is computed from it. KV380 would be +8.0%, so the
  //    KV360 resolution stands on both joints.
  //
  //    INL 1.449 deg mech pk-pk (J01 1.029). Harmonic fit of the BIN block:
  //    1/rev 0.311 deg (J01 0.369 -- J02 is BETTER centred) and 2/rev 0.519 deg
  //    (J01 0.204 -- 2.5x worse). The extra INL is magnet TILT or sensor channel
  //    gain imbalance, NOT eccentricity, so better centring will not fix it.
  //    Costs 0.45 mm of foot position and 0.39% torque. DO NOT CHASE.
  //    ZEA residual 0.240 deg elec (J01 0.467). The ODD part carries only
  //    0.178 deg of 1/rev against a 3.146 deg mean, which confirms the parity
  //    separation really does isolate transport delay from position error.
  //
  //    T/T_loop 0.937 and 0.961 -> fleet now 0.958 +- 0.015 over n=7.
  //
  //    M4, belt OFF, pulley bare, leg not attached, 2026-08-08: n=18 over 6
  //    positions. mean 0.2983 +- 0.0259 A (+-8.7%), sd 0.1098.
  //    J01 was 0.2923 +- 0.0184 -> the two differ by 0.19 SIGMA. SAME LEVEL.
  //    But cv is 36.8% vs J01's 20.9% -- J02 is 1.8x MORE VARIABLE, which is
  //    exactly what "feels rougher to spin by hand" means: worn or redistributed
  //    grease raises the VARIANCE, not the level. The tactile impression was more
  //    informative than the mean.
  //    NO RUB: per-position S = (fwd+rev)/2 = 0.3357 +- 0.0437 over 5 positions,
  //    nothing above +1.6 sd, and the one high reading (0.4750) is a REVISIT of
  //    the position that read 0.3108 forty minutes earlier -- history, not
  //    geometry. The remount is good.
  //    DIRECTION ASYMMETRY C = (fwd-rev)/2 is NOT significant: -0.0144 +- 0.0332
  //    = 0.43 sigma over all six positions. NOTE: that figure keeps the pair
  //    whose reverse reading was dropped from S. Dropping it consistently gives
  //    -0.0363 +- 0.0306 = 1.19 sigma -- still not significant, so the conclusion
  //    holds either way, but 0.43 is the optimistic version.
  //    DROPPED one reading: rev 0.0350 A at raw 5462 (ramp 1.00 s, travel -53),
  //    after 583 counts of unexplained forward motion. The rotor was left
  //    MID-CREEP by the preceding forward ramp, still elastically loaded, so it
  //    broke away at a quarter of the truth. Settle the rotor between readings.
  //    HANDLING IS THE VARIANCE: three consecutive forward readings with the
  //    shaft untouched gave 0.2450/0.2450/0.2500 -- sd 0.0029 A, cv 1.2%, THIRTY
  //    TIMES tighter than the pooled scatter. So repeats at one position are
  //    pseudo-replication: they measure ONE grease state precisely. Hand-rotating
  //    resets that state, and that is where the +-20-37% comes from.
  //    Force: 0.2983 A reported = 0.3079 A TRUE (i_scale 0.9690) -> 8.36
  //    mN.m/motor -> 1.349 N/leg = 13.75% of standing load (J01 13.5%).
  //    Moved twice: x1.018904 by M1 2026-08-18, then /0.9690 by M2 2026-08-20.
  //    At i_scale = 1.0 this read 8.10 mN.m / 1.307 N / 13.3%.
  //    The AMPS above are unchanged by BOTH corrections: drag, breakaway_A and
  //    the whole M4 scatter analysis are in reported amps, and neither the
  //    voltage belief nor the current-sense gain moves a reported amp. Only the
  //    conversion to torque and newtons does.
  //
  //    M6a: 18.17e-6 (impulse) / 19.20e-6 (trajectory, rms 47 cnt), mean 18.7e-6.
  //    7.9% below J01 -- and that gap is ENTIRELY the drag correction. See
  //    J_ROTOR_KGM2 in fleet_config.h. Coast-down cross-check RETIRED.
  //    FLAGGED, NOT CHASED: if J was derived from commanded torque (J = Kt*I/a)
  //    it inherits Kt's scaling -- 18.7e-6 -> ~19.05e-6 here, and the fleet
  //    20.2e-6 -> ~20.4e-6. That is 1-2% against a stated +-12%, one tenth of
  //    the uncertainty already carried. DO NOT re-measure J for this.
  //
  //    i_scale MEASURED 2026-08-20 -- see the M2 block below. It is PROVISIONAL
  //    on this joint (1.15 sigma), which is a different state from PENDING and
  //    must not be read as either 'unmeasured' or 'settled'.
  //    M2, 2026-08-20 -- i_scale = 0.9690, *** PROVISIONAL, NOT DECISIVE. ***
  //        P_bus = 0.7924 + 0.0982*I + 0.34953*I^2     SE(c) = 2.37%
  //        U_del = 0.02419 + 0.22580*I                 SE(R) = 0.77%, rms 2.48 mV
  //        g     = 1.5 * 0.22580 / 0.34953 = 0.9690    +-2.7% (1 sigma)
  //    THAT IS 1.15 SIGMA FROM UNITY AND FAILS THE PROJECT'S OWN +-2% STANDARD
  //    FOR M2. It is stored anyway because it is the best estimate available and
  //    1.0 is not a better one -- but it must not be quoted as a measurement of
  //    the same weight as J01's. `a` does validate (0.7924 W fitted vs 0.693 W
  //    idle at 56.85 mA x 12.19 plus ~0.10 W ripple -- the same 0.10-0.12 W
  //    excess J01 showed), so the fit is not wrong, it is loose.
  //    CONDITIONING IS 3x WORSE THAN J01, and all three causes are identified
  //    and fixable, which is why a re-run is deferred rather than abandoned:
  //        P residuals  +-9.3 mW  (J01 +-2.8)   U residuals +-3.3 mV (J01 +-1.2)
  //        b = +0.098 (J01 +0.035) -- a bigger linear term leaves LESS quadratic
  //          signal to separate with only 2 degrees of freedom
  //        point 1 was still heating through its averaging window (drift -0.93%
  //          vs J01's -0.63%), smearing the (I_reported, Ibus) pairing
  //        point-1 Ibus read to 1 mA, not 0.1
  //    The 7-point AC_M2_V ladder and the 12 s dwell cap in autocalib.h are the
  //    fix for the first and third of those. R_b and V_s are inert here too
  //    (+-16% on R_b -> +-0.9% on g; +-0.065 V on V_s -> 0.03%), so the noise is
  //    in the ladder, NOT in the burden model.
  //
  //    CROSS-JOINT: J01 0.9621 +-1.18%, J02 0.9690 +-2.69%. Spread 0.72%, INSIDE
  //    J02's own uncertainty. Two boards that differ measurably in divider
  //    (0.80%), U0 (60%) and R_eff both land at ~0.96-0.97. Per-board shunt
  //    tolerance would scatter in BOTH directions; a common offset points at the
  //    current-sense constant assumed in LowsideCurrentSense being ~3.5% off.
  //    *** DO NOT COLLAPSE THIS INTO A FLEET CONSTANT. *** Inverse-variance
  //    pooling gives 0.9632 +-1.10% and it is tempting -- but that is precisely
  //    the move that produced the vbus_scale failure, where an inherited
  //    "verified" value hid a real 0.80% board difference for two weeks. Two
  //    samples cannot establish commonality; they can only fail to reject it.
  //    A KNOWN SYSTEMATIC, recorded because it points the right way: R_eff is
  //    current-dependent (see the R_eff note below), and c is weighted toward
  //    the high-current points where R is lower while R_M2 is a linear
  //    compromise across the range. That biases g HIGH -- true g is likely
  //    slightly FURTHER below unity, not closer to it. Does not threaten either
  //    verdict.
  { "J02", "___", "___", "2026-08-08", "OFF",
     0.3482f, +1, 0.22810f,        // zea, dir, R_eff      M1-rescaled x1.018904
     0.014937f,                    // U0   M1-rescaled -- weak, see the U0 note below
     0.018097f, 46.25e-6f,         // Ke, L   (Kt = calKt() = 0.027145)  M1-rescaled
     0.008516f, 0.9690f,           // vbus_scale (M1 2026-08-18, UT89X -- see the
                                   //   0.32% item, README 24.14)
                                   // i_scale (M2 2026-08-20) PROVISIONAL +-2.7%
     0.1061f, 0.1074f,             // drag_c fwd, rev      unchanged (reported A)
     0.000948f, 0.000845f,         // drag_v fwd, rev      unchanged (reported A)
     0.2983f },                    // breakaway_A  M4, n=18, +-8.7%  (reported A)
  // -- J03 .. J12 -- NOT BUILT. EVERY MEASURABLE FIELD IS 0.0f = NOT MEASURED.
  //    zea = -1 and dir = 0 still make runInitFOC() fall back to a full
  //    alignment, so selecting one of these is safe -- just uncalibrated, and
  //    now loudly so: the boot banner prints R=0.00000 Ke=0.000000, which
  //    nobody can mistake for a calibration.
  //
  //    THESE ROWS USED TO CARRY J01's NUMBERS as "a plausible starting point".
  //    That is exactly the silent-inheritance failure the vbus_scale block at
  //    the top of this file warns about, and 2026-08-18 measured what it costs:
  //    the two built boards' dividers are 0.80% apart, and J02's inherited
  //    divider put every one of its stored constants 1.89% out. Ten rows were
  //    carrying B-ABZ-01's or B-SPI-01's divider under a comment claiming they
  //    were placeholders. A placeholder that LOOKS like a measurement is worse
  //    than no number, so there is no longer a number.
  //
  //    vbus_scale = 0 disables live Vbus entirely -- documented in
  //    open_test.cpp, behaviour byte-identical to a hardcoded divisor, so this
  //    costs a calibrated bus reading on a board that was never calibrated.
  //    i_scale stays 1.0f because that field's documented "unmeasured" value is
  //    1.0 (inert multiplier), not 0.
  //    ORDER: M1 first, then AUTOCALIB, then paste. Not the other way round --
  //    R_eff, U0 and Ke are all measured THROUGH vbus_scale.
  { "J03", "-", "-", "-", "OFF", -1.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
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

  // -- A1 -- HISTORICAL ONLY. *** THIS ASSEMBLY NO LONGER EXISTS. ***
  //    The hardware was rebuilt into J02 (index 1): same motor, same board, the
  //    rubbing magnet remounted and an MT6816 SPI board fitted. Every constant
  //    here is SUPERSEDED by J02's row, which measured the same physical parts
  //    with the fault removed -- see there for the L = 65 -> 46.25 uH and
  //    U0 = 0.028 -> 0.014937 corrections (both at J02's M1-corrected scale).
  //    Kept at index 12 so that no other row's JOINT_ID shifts, and as the
  //    historical record of what the fault looked like. Its ZEA is
  //    session-relative by construction (incremental encoder), hence -1.
  //
  //    *** ALL A1 CONSTANTS WERE MEASURED WITH A RUBBING ENCODER MAGNET. ***
  //    The ABZ lost-count fault is now diagnosed as mechanical jitter from the
  //    magnet contacting. Everything in this row therefore carries an unknown
  //    mechanical friction contribution, and drag_c = 1.05 A in particular is
  //    belt friction PLUS a rub -- it is not a belt-drive baseline.
  //
  //    WHAT DEPENDS ON THAT 1.05 A, and is therefore suspect:
  //      * "4.5 N per leg = 46% of standing load" -- the project's headline
  //        transparency figure, and the basis of "Coulomb friction dominates
  //        transparency, 5x everything else combined".
  //      * The M5 belt-drag prior. DO NOT expect belt-on minus belt-off to be
  //        ~0.98 A. J01 belt-off is 0.075 A moving; a 10 mm GT2 at 9:1 losing
  //        0.98 A was always uncomfortably large, and a rubbing magnet is a far
  //        better explanation than a catastrophically lossy belt. The belt is
  //        probably BETTER than this project has been assuming.
  //    NOT affected, for a reason worth stating precisely: DRIVETRAIN_ETA was
  //    back-solved as 4.5/(2*G*Kt*1.05), and 4.5 was itself COMPUTED as
  //    2*G*eta*Kt*1.05 -- so the 1.05 cancels exactly. Contamination cannot
  //    reach eta. eta is uninformative for a different reason: it is circular.
  //
  //    Superseded by J02 (same motor size, rub fixed, SPI encoder). Kept as the
  //    historical record of what the fault looked like, at index 12 so that no
  //    other row's JOINT_ID shifts.
  //
  //    NOTHING HERE MAY BE INHERITED BY J02. Every field fails at least one test
  //    for reuse: zea/dir are from an incremental encoder on different hardware;
  //    U0 is 2.8x J01's and L is 1.5x, both board/motor properties where one of
  //    the pair must be wrong; vbus_scale was never measured on this board; drag
  //    and breakaway are belt-ON and contaminated. Run M1 and the full AUTOCALIB.
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
  //
  //    NOT RESCALED BY M1 2026-08-18, DELIBERATELY. This board is B-ABZ-01 and
  //    its divider is now measured at 0.008516, so R_eff, U0 and Ke here are
  //    all 1.89% low -- 0.218 -> 0.2221, 0.028 -> 0.0285, 0.0177 -> 0.018034.
  //    They are left alone because this row is a HISTORICAL RECORD of what was
  //    flashed and measured in July, and rewriting it would falsify the record
  //    of the fault. Nothing may be inherited from it in any case (see above),
  //    and the firmware cannot drive A1. The vbus_scale field keeps 0.008358 for
  //    the same reason: that is the value the July numbers were taken under.
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
// i_scale = 1.0 makes this identical to calKt(). *** IT IS NO LONGER INERT: ***
// M2 ran 2026-08-20 and both built rows carry g ~ 0.962-0.969, so calKtCmd()
// now differs from calKt() by ~3-4% on J01 and J02.
//
// AND IT CURRENTLY HAS NO CONSUMER. Nothing in open_test.cpp converts a torque
// to a current -- the bench harness commands voltage or reported amps directly,
// so storing i_scale changes NOTHING at runtime today except the boot banner.
// State it plainly rather than let a stored constant imply a correction that is
// not being applied: TIER-0 IS THE FIRST CONSUMER, and the day it computes
// I_command it must divide by calKtCmd(), not calKt(). That is the whole reason
// this function exists ahead of its caller.
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
    // TWO different percentages, and the label used to name the wrong one.
    // calKtCmd() = Kt / i_scale, so I_command = tau * i_scale / Kt: the COMMAND
    // is multiplied by i_scale (+5% at i_scale = 1.05). The 1/i_scale - 1 figure
    // is the torque ERROR you would have shipped uncorrected (-4.76% at 1.05).
    // Both are worth seeing; neither is a name for the other.
    out.print(F(" (uncorrected torque error "));
    out.print(100.0f*(1.0f/CAL.i_scale - 1.0f), 2);
    out.print(F("%, cmds now scaled "));
    if (CAL.i_scale > 1.0f) out.print('+');
    out.print(100.0f*(CAL.i_scale - 1.0f), 2);
    out.print(F("%)"));
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
    out.println(F("  !! i_scale = 1.0 (NOT MEASURED) on a BUILT joint -- run M2. Both built boards read ~0.96."));
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
    out.println(F("  !! DRIVETRAIN_ETA is BACK-SOLVED, not measured (circular -- see fleet_config.h). M14 replaces it."));
  if (built && CAL.breakaway_A == 0.0f)
    out.println(F("  !! breakaway_A = 0 on a BUILT joint -- run M4, stiction threshold."));
}
