#pragma once
// ============================================================================
// autocalib.h  --  ONE-KEY PER-JOINT CHARACTERISATION
// ============================================================================
// Include this in open_test.cpp IMMEDIATELY BEFORE `void handleSerial()` (it
// must be above handleSerial so the case labels can see acPhase), and add:
//
//     #include "autocalib.h"          // <-- directly above void handleSerial()
//
//     case 'Y': case 'y': acStatus();   break;   // what is done / what is next
//     case '1': acPhase(1); break;   // LINK    zero current, always safe
//     case '2': acPhase(2); break;   // ALIGN   ZEA + direction
//     case '3': acPhase(3); break;   // R/U0    self-locked, rotor still
//     case '4': acPhase(4); break;   // L       self-locked step train
//     case '5': acPhase(5); break;   // SPIN    free-spin both directions
//     case '6': acPhase(6); break;   // T/INL   computation only, no motor
//     case '7': acPhase(7); break;   // REPORT  print the pasteable block
//     case '0': acPhase(0); break;   // RESET   discard all results
//     case 'V': acVerifyZea(); break;   // RESOLVED: lower-case 'v' stays VELOCITY
//                                       // mode, upper-case 'V' verifies ZEA.
//
// ('A' is already logStats, so 'Y' is used for status.)
//
// ONE PHASE PER KEYPRESS, ON PURPOSE. You inspect each result before the next
// phase runs, you can re-run any single phase, and a failure never leaves the
// motor energised or the globals half-mutated. Each phase saves and restores
// everything it touches, and refuses to run if its prerequisites have not
// PASSED -- so out-of-order presses cannot silently produce garbage.
//
// BELT STATE -- it is NOT "belt off" across the board, and saying so blanket
// was actively misleading once M2 got deferred past the belt build:
//     FREE SHAFT REQUIRED : 2 (the alignment must settle unloaded)
//                           5, 6 (free spin)
//     BELT-AGNOSTIC       : 1, 3, 4 and the M2 ladder -- all locked-rotor, the
//                           rotor never turns, so the belt is not in the loop.
//                           The one thing that DOES matter there is that the
//                           rotor must not creep: watch the drift column.
//     BOTH, DELIBERATELY  : M4. Belt-off is the baseline; belt-on at M5 is the
//                           number that decides impedance quality. Its banner
//                           REPORTS the state instead of demanding one.
// The leg links must be off for anything involving friction, in every state --
// gravity torque is position-dependent and cannot be averaged out.
//
// PREREQUISITE CHAIN
//     1 LINK   -> none
//     2 ALIGN  -> 1
//     3 R/U0   -> 2                    (needs correct commutation)
//     4 L      -> 3                    (self-lock; R converts tau to L)
//     5 SPIN   -> 3                    (needs R and U0 to isolate Ke)
//     6 T/INL  -> 4 and 5              (needs R, L, Ke and the +-omega bins)
//     7 REPORT -> whatever has run; missing outputs are printed as NOT MEASURED
//
// WHY A .h AND NOT A .cpp: PlatformIO compiles every .cpp in src/ as its own
// translation unit. A .cpp that is ALSO #included gets compiled twice and fails
// at link with duplicate symbols. As a header included once, this file sees
// motor / driver / currentSense / encoder / mode / running / target directly --
// no extern declarations, no duplicated MT6816SPI definition, and nothing in
// open_test.cpp has to move. Exactly two edits to the main sketch.
//
// ---------------------------------------------------------------------------
// WHAT IT PRODUCES
//   MANDATORY per joint : zea, dir
//   PER-UNIT            : R_eff, U0, Ke, L, drag (Coulomb + viscous, PER DIRECTION)
//   DIAGNOSTIC          : T_delay, INL profile, |I| ratio, SPI link health
//   CARRIED, not measured: vbus_scale (M1), i_scale (M2), breakaway_A (M4) -- the
//                         report re-emits whatever the flashed row holds so a
//                         re-run never silently discards a hand measurement
//   SUGGESTION ONLY     : current-loop gains (printed, NEVER applied)
//   NOT EMITTED         : Kt. It is 1.5*Ke by convention and is derived by
//                         calKt(); storing it would be a second number that must
//                         agree with the first, with nothing checking them.
//
// EXCLUDED ON PURPOSE -- each would be WORSE automated than done by hand:
//   J_rotor       geometric, ~1% between units, and needs friction subtraction
//   breakaway     needs slow +-0.5 rad/s ramps through zero; different regime.
//                 Has a HOME though -- JointCal.breakaway_A, filled by hand (M4)
//   hot/cold R    needs a thermal soak, minutes not seconds
//   cogging map   not used by any planned controller
//   force-per-amp needs a load cell and the assembled leg
//   flash writes  a failed write bricks the session; a pasteable block lives in
//                 git where it is reviewable and diffable
//
// ---------------------------------------------------------------------------
// THE ONE NON-OBVIOUS TRICK: MAGNETIC SELF-LOCK  (phases 3 and 4)
//
// R_eff needs a stationary rotor, which normally means a hand on the pulley.
// Instead: MODE_OPENLOOP with target = 0 makes velocityOpenloop() apply
// motor.voltage_limit at a FIXED electrical angle. The rotor swings into
// alignment with that field and stays there.
//
// The measurement is valid wherever the rotor ends up: at zero speed there is no
// back-EMF, the winding is purely resistive, and
//        U_applied = R_eff * I + U0
// holds for ANY rotor position. Alignment is irrelevant. Only STILLNESS matters,
// and stillness is verified directly from encoder.raw before and after every
// point. A point that moved is DROPPED with a warning -- it cannot corrupt the
// fit silently, which is what makes this safe to run unattended.
//
// Why the lock holds: hold torque = Kt*I, about 0.055 N.m at the top of the
// sweep against ~0.01-0.02 N.m of cogging. The sweep therefore runs DOWNWARD
// from the strongest point, so the rotor is already parked at its equilibrium
// before the hold weakens. Sweeping upward would start at 0.004 N.m, cogging
// would win, and the rotor would jump. Electrical damping through R gives
// zeta ~ 0.5 at omega_n ~ 110 rad/s, so 200 ms is ~3 periods; 350-600 ms used.
//
// PHASE 3/4 CURRENT SOURCE: open_test.cpp's loop() deliberately ZEROES
// motor.current.q/.d in MODE_OPENLOOP, because velocityOpenloop() never writes
// electrical_angle so a dq transform there would be meaningless. This routine
// therefore NEVER reads Iq/Id in openloop. It uses the phase magnitude, which is
// live in every mode:
//        I_amplitude = sqrt(ia^2+ib^2+ic^2) / 1.2247
// the same amplitude-invariant identity as the |I|/Iq = 1.2247 ratio gate.
//
// ---------------------------------------------------------------------------
// SAFETY
//   * A phase BLOCKS loop(), so loop()'s overspeed / auto-stop / sense-mismatch
//     guards do NOT run while it executes. Equivalents live in acService():
//     a hard phase-amplitude abort, an overspeed abort, and an any-key abort.
//   * Direction reversal in phase 5 is the only genuinely dangerous moment. At
//     +110 rad/s, commanding even 0.0 V draws -9.8 A of braking (back-EMF
//     1.95 V across 0.198 ohm); commanding -2.0 V draws -20 A. The routine
//     therefore NEVER commands a reversing voltage: it DISABLES the driver and
//     lets the shaft coast, then re-enables.
//   * Every voltage change is soft-ramped in 0.05 V steps -> 0.25 A transients.
//   * Any serial byte aborts. On abort or FAIL the motor is disabled and every
//     mutated global is restored.
//
// MUTATED AND RESTORED: mode, target, running, motor.voltage_limit,
//   motor.PID_current_{q,d}.limit, motor.zero_electric_angle,
//   motor.sensor_direction, foc_ready.
//
// REVISION 2 (2026-08-07), after the first full run on assembly A2:
//   F1  |I| ratio gate moved to the SQUARED domain and widened to a gross-sanity
//       band. The old 1.20-1.25 gate produced a FALSE FAIL: free-spinning belt-off
//       Iq is only 0.10-0.18 A, and mean(sqrt) of an always-positive rippling
//       quantity is biased up by 4-8% there.
//   F2  VBUS_SCALE is per-board and unmeasurable without an external reference,
//       so the report now DEMANDS a written-in multimeter reading.
//   F3  acService() no longer reads the phase currents twice per iteration. That
//       cost ~18 us and dropped the loop from 16.8 to 12.9 kHz -- which matters
//       because T_delay is ONE LOOP PERIOD, so the reported T was 33% high.
//       f_loop and T/T_loop are now printed alongside T.
//   F4  The L step trace is binned by TIME, not sample index, turning natural
//       loop-phase jitter into equivalent-time sampling: ~17 fit points, not 5.
//   S2  New 'V' command verifies a stored ZEA against a fresh alignment.
//
// RUNTIME per phase: 1 ~0.3 s | 2 ~8 s | 3 ~6 s | 4 ~2 s | 5 ~50 s | 6,7 instant.
//   Settle times are deliberately 2-3x the relevant time constants.
// RAM ~1.6 kB of statics on top of the 19.5 kB log buffer. CHECK THE FREE-RAM
//   FIGURE IN THE BUILD OUTPUT -- a static array colliding with the stack gives
//   a HardFault, not a compile error. Drop LOG_N if it is tight.
// ============================================================================

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
const uint8_t AC_ALIGN_N      = 7;        // alignments for the ZEA median
const float   AC_ZEA_SD_FAIL  = 0.15f;    // rad elec: above this ZEA is unusable
const float   AC_ZEA_SD_WARN  = 0.08f;
const float   AC_ZEA_SE_FAIL  = 0.035f;   // rad elec (2.0 deg) on the median

const uint8_t AC_R_N          = 9;
const float   AC_R_V[AC_R_N]  = { 0.46f, 0.40f, 0.35f, 0.30f, 0.25f,
                                  0.20f, 0.16f, 0.12f, 0.08f };   // DESCENDING
const uint16_t AC_R_SETTLE_MS = 350;
const uint16_t AC_R_MEAS_MS   = 250;
const uint16_t AC_R_STILL_CNT = 20;       // allowed drift per point, counts
const uint8_t  AC_R_MIN_PTS   = 7;
const float    AC_R_RMS_FAIL  = 0.012f;   // V
const float    AC_R_RSE_FAIL  = 0.03f;    // relative SE of the slope

const uint8_t  AC_L_REPS      = 32;       // averaged step repeats (was 24)
const uint8_t  AC_L_NS        = 40;       // samples captured per repeat
const float    AC_L_VBASE     = 0.20f;    // hold between steps (~0.8 A)
const float    AC_L_VSTEP     = 0.70f;    // stepped-to (~3.4 A peak)
const uint16_t AC_L_DECAY_MS  = 4;        // >= 9 tau_e at tau_e ~ 200 us
const uint8_t  AC_L_MIN_PTS   = 6;        // was 8. 6 is the statistical floor for a
                                          // 2-parameter fit with an rms gate; 8 was
                                          // set from F4's WRONG ~17-point prediction.
// F5 -- DELIBERATE PHASE DITHER. F4 assumed the step's phase relative to the loop
// was random. It is NOT: acService() IS the loop, so samples land at step+{0,60,
// 120..} us rigidly and only every 3rd 20 us bin is ever visited. Measured: 7 fit
// points, not 17. FIX: after the sample that APPLIES the step, stall r*2 us so the
// REMAINING samples of that repeat sit at a per-repeat offset. Across 32 reps the
// offset sweeps 0..62 us = one full loop period, so sample k covers
// [60(k-1), 60(k-1)+62] us and the union is contiguous.
// AC_L_DITHER_US * AC_L_REPS must be >= ONE LOOP PERIOD, or the sweep leaves a
// hole and the un-dithered grid shows through. It was 2: 2 x 32 = 64 us against
// a 75 us loop period at 13.3 kHz -- an 11 us hole per period, which is exactly
// why session 3's LSB `n` column still peaked every third bin (21, 15, 17, 15,
// 16...). 3 x 32 = 96 us covers 13.3 kHz with margin and still covers 16.8 kHz.
// Over-covering is harmless: the offsets simply spread over 1.3 loop periods
// and bin occupancy evens out. J01's existing fit is unaffected (that data is
// already taken and the tau it produced was verified offline); this is for J02+.
const uint8_t  AC_L_DITHER_US = 3;
const float    AC_L_RMS_FAIL  = 0.20f;
// F4 -- TIME BINNING.  *** PARTLY SUPERSEDED BY F5 ABOVE -- read that first. ***
// STILL TRUE: averaging by SAMPLE INDEX throws away timing information that is
// useful, and binning the (t, I) pairs by TIME instead is the right move. Index
// averaging left exactly 5 points in the 0.15-0.85 fit band on assembly A2 --
// tau = 200 us against a 58.5 us sample period is only 3.4 samples/tau.
// RETRACTED: F4 originally claimed the step's phase relative to the control loop
// was "effectively random across repeats", and predicted ~17 fit points from
// that free jitter alone. It is NOT random -- acService() IS the loop, so the
// samples land on a rigid grid, and time binning ALONE gave 7 points, not 17.
// The 18 points actually achieved come from F5's DELIBERATE per-repeat dither.
// Time binning is the mechanism; the dither is what supplies the coverage.
const uint8_t  AC_L_TBINS     = 48;       // 48 x 20 us = 960 us of rise
const uint16_t AC_L_TBIN_US   = 20;
const uint8_t  AC_L_BIN_MIN_N = 3;        // samples needed before a bin is used
const uint16_t AC_L_TAIL_US   = 1500;     // t beyond this is steady state -> I_inf

const uint8_t  AC_W_N         = 5;
const float    AC_W_V[AC_W_N] = { 0.50f, 0.90f, 1.30f, 1.70f, 2.00f };
const uint16_t AC_W_SETTLE_MS = 3000;
const uint16_t AC_W_MEAS_MS   = 1000;     // ~15 k samples, ~17 revolutions
const float    AC_W_VLIMIT    = 2.60f;    // raised so 2.00 V is not clamped
const float    AC_KE_RMS_FAIL = 0.015f;   // V
const float    AC_KE_RSE_FAIL = 0.02f;
const float    AC_KE_C_WARN   = 0.030f;   // |intercept| -> R or U0 is off

const uint8_t  AC_BINS        = 32;
const uint8_t  AC_BIN_SPEEDS  = 2;        // top N speeds per direction get binned
const uint16_t AC_BIN_MIN_N   = 20;       // samples needed before a bin is used
const float    AC_T_AGREE     = 0.25f;    // the two T estimates must agree

const float    AC_RAMP_V      = 0.05f;
const uint8_t  AC_RAMP_MS     = 10;
const float    AC_IMAX_ABORT  = 6.0f;     // A amplitude: hard abort
const float    AC_COAST_RADS  = 5.0f;
// F1 -- the |I|/Iq gate. |I| = sqrt(ia^2+ib^2+ic^2) is ALWAYS POSITIVE, so
// mean(|I|) > |mean(I)| whenever there is ripple, and the smaller the DC current
// the larger the relative bias. Free-spinning belt-off, Iq is only 0.10-0.18 A --
// the WORST case. Measured on A2: 1.324 at Iq=0.100 falling monotonically to
// 1.273 at Iq=0.179, i.e. +8.1% -> +3.9%. That is the bias, not a fault, and the
// original 1.20-1.25 gate produced a FALSE FAIL.
// FIX: accumulate in the SQUARED domain. Instantaneously |I|^2 = 1.5*(Iq^2+Id^2),
// so  ratio = sqrt( mean(ia^2+ib^2+ic^2) / (1.5*mean(Iq^2+Id^2)) )  removes the
// sqrt bias entirely. The gate is then a GROSS-SANITY band, because the two sides
// are still sampled at different instants of a PWM-rippling current.
// The definitive 1.22-1.23 check is a LOCKED-ROTOR one -- see README, and do it
// by hand in 'c' mode. Phase 5 cannot assert it.
const float    AC_RATIO_LO    = 1.15f;
const float    AC_RATIO_HI    = 1.40f;
const float    AC_BW_HZ       = 400.0f;   // for the SUGGESTED gains only

// ---------------------------------------------------------------------------
// Verdicts. Every output carries its own, and the run carries the worst.
// ---------------------------------------------------------------------------
// Each output carries its OWN verdict. There is deliberately no single running
// "worst" flag: phase 7 recomputes it from the outputs that actually ran, so a
// stale flag from an earlier attempt cannot leak into a later report.
enum AcV { AC_PASS = 0, AC_WARN = 1, AC_FAIL = 2 };
static bool ac_abort;
// Set ONLY by the keypress branch of acService(), alongside ac_abort. Every
// phase 1-7 ignores it and behaves exactly as before. The M2 ladder uses it to
// tell "operator pressed a key to advance" from "overcurrent/overspeed tripped",
// which acService() previously collapsed into one flag.
static bool ac_key;
// Set ONLY by the overcurrent / overspeed branches. The three abort conditions
// are independent `if`s in the same acService() call, so a keypress arriving in
// the SAME iteration as an overcurrent trip sets ac_key AND ac_abort. A caller
// that clears ac_abort on ac_key alone would then resume after a genuine fault.
// ac_guard is what makes "a human did this" and "a guard tripped" separable
// when both are true: a guard trip is never clearable.
static bool ac_guard;
static const __FlashStringHelper* acVs(AcV v) {
  return (v == AC_PASS) ? F("PASS") : (v == AC_WARN) ? F("WARN") : F("FAIL");
}

// ---------------------------------------------------------------------------
// Least squares y = m*x + c WITH the standard errors that decide whether a
// number is DETERMINED. Reporting these is the point: a 14-point locked-rotor
// sweep once pinned R to +-1.55% while its intercept sat 4.6 sigma from zero
// with a 21.8% standard error -- and both printed as if equally solid.
// ---------------------------------------------------------------------------
struct AcFit { float m, c, rms, se_m, se_c; uint8_t n; bool ok; };

static AcFit acFit(const float* x, const float* y, uint8_t n) {
  AcFit f; f.n = n; f.ok = false; f.m = f.c = f.rms = f.se_m = f.se_c = 0.0f;
  if (n < 3) return f;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (uint8_t i = 0; i < n; i++) {
    sx += x[i]; sy += y[i]; sxx += (double)x[i]*x[i]; sxy += (double)x[i]*y[i];
  }
  double den = (double)n*sxx - sx*sx;
  if (fabs(den) < 1e-12) return f;
  double m = ((double)n*sxy - sx*sy) / den;
  double c = (sy - m*sx) / n;
  double ss = 0;
  for (uint8_t i = 0; i < n; i++) { double r = y[i] - (m*x[i] + c); ss += r*r; }
  double s2 = (n > 2) ? ss / (n - 2) : 0.0;
  f.m = (float)m; f.c = (float)c;
  f.rms  = (float)sqrt(ss / n);
  f.se_m = (float)sqrt(s2 * (double)n / den);
  f.se_c = (float)sqrt(s2 * sxx / den);
  f.ok = true;
  return f;
}

// ---------------------------------------------------------------------------
// State. Statics rather than stack: the bin arrays and the L trace would blow a
// 1 kB stack frame, and results must survive between keypresses.
// ~1.6 kB total. CHECK FREE RAM IN THE BUILD OUTPUT.
// ---------------------------------------------------------------------------
static bool  ac_done[8];                              // phase N has PASSED
static float ac_rx[AC_R_N], ac_ry[AC_R_N];            // R sweep: I, U_commanded
static uint8_t ac_rn, ac_rdrop;
static float    ac_lb_i[AC_L_TBINS];                  // L step: sum of I per time bin
static uint16_t ac_lb_n[AC_L_TBINS];                  // samples per time bin
static float ac_l_I0, ac_l_Iinf, ac_l_tail_n;
static uint8_t ac_l_pts;
static float ac_wx[2*AC_W_N], ac_wy[2*AC_W_N];        // Ke fit: vel, U - R*Iq - U0
static float ac_w_vel[2*AC_W_N], ac_w_iq[2*AC_W_N], ac_w_u[2*AC_W_N];
static float ac_floop[2*AC_W_N];                      // F3: measured loop rate per point
static uint8_t ac_wn;
// [direction][speed slot][bin] ; direction 0 = forward, 1 = reverse
static float    ac_bin_id[2][AC_BIN_SPEEDS][AC_BINS];
static float    ac_bin_iq[2][AC_BIN_SPEEDS][AC_BINS];
static uint16_t ac_bin_n [2][AC_BIN_SPEEDS][AC_BINS];
static float    ac_bin_w [2][AC_BIN_SPEEDS];

// results
static float ac_zea, ac_zea_sd, ac_zea_se;   static int8_t ac_dir;
static float ac_R, ac_U0, ac_R_se, ac_U0_se, ac_R_rms, ac_U0_sig;
static float ac_L, ac_tau, ac_L_rms, ac_L_td;
static float ac_Ke, ac_Ke_se, ac_Ke_rms, ac_Ke_c;
// Drag, fitted PER DIRECTION: Iq = drag_c + drag_v*|omega|, magnitudes only.
// index 0 = forward (omega > 0), 1 = reverse. A single mean pair discarded a
// reproducible 28% fwd/rev asymmetry, which is why JointCal now carries four.
static float ac_drag_c[2], ac_drag_v[2], ac_drag_rms[2];
static uint8_t ac_drag_n[2];
static AcV   ac_v_drag;
static float ac_T[AC_BIN_SPEEDS], ac_T_floop[AC_BIN_SPEEDS], ac_inl_pp, ac_zea_resid;
static float ac_ratio_lo, ac_ratio_hi;
static float ac_link_us; static uint32_t ac_link_err;
static AcV   ac_v_link, ac_v_zea, ac_v_R, ac_v_U0, ac_v_L, ac_v_Ke, ac_v_ratio, ac_v_T;

// ---------------------------------------------------------------------------
// Saved globals, restored by acExit()
// ---------------------------------------------------------------------------
static Mode      acs_mode;   static float acs_target, acs_vlim, acs_pql, acs_pdl;
static float     acs_zea;    static Direction acs_dir;   static bool acs_foc;

static void acEnter() {
  ac_abort  = false;   ac_key = false;   ac_guard = false;
  acs_mode  = mode;   acs_target = target;
  acs_vlim  = motor.voltage_limit;
  acs_pql   = motor.PID_current_q.limit;
  acs_pdl   = motor.PID_current_d.limit;
  acs_zea   = motor.zero_electric_angle;
  acs_dir   = motor.sensor_direction;
  acs_foc   = foc_ready;
}

// keep_align = true means "do not restore the saved alignment on the way out".
// Phase 2 passes true on success because it deliberately INSTALLS the ZEA it
// just measured. Phases 3, 4, 5 and both manual-assist routines also pass true,
// but for the opposite reason: they never touch the alignment, so restoring a
// snapshot of it would be a no-op at best and could only ever undo phase 2's
// install. false is for the paths that DID disturb it and failed -- a failed or
// aborted alignment must never be left installed, because it would silently
// become the commutation offset for every later run in the session.
static void acExit(bool keep_align) {
  motor.disable(); running = false;
  motor.voltage_limit       = acs_vlim;
  motor.PID_current_q.limit = acs_pql;
  motor.PID_current_d.limit = acs_pdl;
  motor.PID_velocity.reset(); motor.PID_current_q.reset(); motor.PID_current_d.reset();
  if (!keep_align) {
    // Never leave a failed or aborted alignment installed: it would silently
    // become the commutation offset for every later run in this session.
    motor.zero_electric_angle = acs_zea;
    motor.sensor_direction    = acs_dir;
    foc_ready                 = acs_foc;
  }
  mode = acs_mode; target = acs_target;
  switch (mode) {
    case MODE_OPENLOOP:       motor.controller = MotionControlType::velocity_openloop; break;
    case MODE_TORQUE:         motor.torque_controller = TorqueControlType::voltage;
                              motor.controller = MotionControlType::torque; break;
    case MODE_TORQUE_CURRENT: motor.torque_controller = TorqueControlType::foc_current;
                              motor.controller = MotionControlType::torque; break;
    case MODE_VELOCITY:       motor.torque_controller = TorqueControlType::foc_current;
                              motor.controller = MotionControlType::velocity;
                              motor.PID_velocity.limit = CURR_LIMIT; break;
  }
}

// ---------------------------------------------------------------------------
// Control-path service. A phase BLOCKS loop(), so nothing calls loopFOC() or
// move() unless we do it here. This mirrors open_test.cpp's loop() control path
// exactly, plus the guards loop() would otherwise have applied.
// ---------------------------------------------------------------------------
static float    ac_i_amp = 0.0f;      // phase-current AMPLITUDE, live in all modes
static uint32_t ac_loops = 0;         // control-loop iterations, for the f_loop report

static inline float acPhaseAmp() {
  PhaseCurrent_s c = currentSense.getPhaseCurrents();
  // sqrt(ia^2+ib^2+ic^2) = 1.2247 * amplitude in the amplitude-invariant frame
  return sqrtf(c.a*c.a + c.b*c.b + c.c*c.c) * AMP_INV_MAG_INV;
}

// F3 -- squared magnitude, for the ratio gate. Returned SQUARED so it can be
// accumulated without a sqrt, which is what removes the averaging bias (F1).
static inline float acPhaseMag2() {
  PhaseCurrent_s c = currentSense.getPhaseCurrents();
  return c.a*c.a + c.b*c.b + c.c*c.c;
}

static void acService() {
  motor.loopFOC();
  // The same refresh open_test.cpp's loop() performs, for the same reasons:
  // 2.3.1's loopFOC() does not touch motor.current in the voltage branch, and
  // velocityOpenloop() never writes electrical_angle so a dq transform in
  // openloop would be meaningless.
  if (cs_linked && mode == MODE_TORQUE) {
    motor.current = currentSense.getFOCCurrents(motor.electrical_angle);
  } else if (mode == MODE_OPENLOOP) {
    motor.current.q = 0.0f; motor.current.d = 0.0f;
  }
  if (running) motor.move(target);
  ac_loops++;

  // F3 -- do NOT read the phase currents again here in closed-loop modes.
  // getFOCCurrents() above already cost one ADC read; a second getPhaseCurrents()
  // every iteration cost ~18 us and dropped the loop from 16.8 kHz to 12.9 kHz.
  // That matters twice over: it degrades every measurement, AND since
  // T_delay ~= ONE FULL LOOP PERIOD (see the T model note in phase 6), it made
  // the T reported here 33% larger than the T the main sketch actually has.
  // In closed-loop modes the dq magnitude IS the phase amplitude, so it is free.
  if (mode == MODE_OPENLOOP) ac_i_amp = acPhaseAmp();
  else                       ac_i_amp = sqrtf(motor.current.q*motor.current.q
                                            + motor.current.d*motor.current.d);
  if (ac_i_amp > AC_IMAX_ABORT) {
    ac_abort = true;  ac_guard = true;
    SerialUART.print(F("\n  !! ABORT overcurrent: ")); SerialUART.print(ac_i_amp, 2);
    SerialUART.println(F(" A amplitude"));
  }
  // motor.shaft_velocity is the MEASURED value in closed-loop modes; in openloop
  // velocityOpenloop() overwrites it with the COMMANDED value (0), so this check
  // is meaningful only outside openloop -- which is exactly where the shaft can
  // actually run away.
  if (mode != MODE_OPENLOOP && fabsf(motor.shaft_velocity) > OVERSPEED_RADS) {
    ac_abort = true;  ac_guard = true;
    SerialUART.println(F("\n  !! ABORT overspeed"));
  }
  if (SerialUART.available()) {
    while (SerialUART.available()) (void)SerialUART.read();
    ac_abort = true;
    ac_key   = true;          // "a human did this", not "a guard tripped"
    // NEUTRAL wording on purpose. A key means ABORT to phases 1-7 but ADVANCE to
    // the M2 ladder, and every phase already prints its own "aborted  FAIL"
    // line. Printing "!! ABORT" here made a normal five-point M2 run announce
    // ABORT five times while working correctly -- which is exactly the message
    // that makes someone stop and redo a good measurement.
    SerialUART.println(F("\\n  -- key received"));
  }
}

static void acRun(uint32_t ms) {
  uint32_t t0 = millis();
  while ((millis() - t0) < ms && !ac_abort) acService();
}

// Soft-ramp `target` so no single step exceeds AC_RAMP_V -> 0.25 A transients.
static void acRampTarget(float to) {
  float from = target;
  int steps = (int)(fabsf(to - from) / AC_RAMP_V) + 1;
  for (int k = 1; k <= steps && !ac_abort; k++) {
    target = from + (to - from) * ((float)k / (float)steps);
    acRun(AC_RAMP_MS);
  }
  if (!ac_abort) target = to;
}

// Arduino's abs() is a macro that double-evaluates, and int32_t is not portably
// `long`, so labs() is not safe either. Explicit helper instead.
static inline uint32_t acAbs32(int32_t v) { return (uint32_t)((v < 0) ? -v : v); }

// Signed count difference with the ENC_CPR wrap handled.
static int32_t acCntDelta(uint16_t a, uint16_t b) {
  int32_t d = (int32_t)b - (int32_t)a;
  if (d >  (int32_t)(ENC_CPR/2)) d -= (int32_t)ENC_CPR;
  if (d < -(int32_t)(ENC_CPR/2)) d += (int32_t)ENC_CPR;
  return d;
}

// Coast to a stop WITHOUT commanding a reversing voltage. At +110 rad/s even
// commanding 0.0 V draws -9.8 A of braking (back-EMF 1.95 V across 0.198 ohm);
// commanding -2.0 V draws -20 A. So disable the driver and let drag do it.
static void acCoast() {
  target = 0.0f;
  motor.disable(); running = false;
  uint32_t t0 = millis();
  while ((millis() - t0) < 8000) {
    encoder.update();
    if (fabsf(encoder.getVelocity()) < AC_COAST_RADS) break;
    if (SerialUART.available()) { while (SerialUART.available()) (void)SerialUART.read(); ac_abort = true; break; }
  }
  delay(250);
}

// The refusals acPhase() has always applied, factored out so the manual-assist
// routines below cannot accidentally skip one.
static bool acReady() {
  if (running) { SerialUART.println(F("stop first (x)")); return false; }
  if (!driver_ok || !cs_linked) {
    SerialUART.println(F("refused: driver or current sense not ready")); return false;
  }
  return true;
}

static bool acNeed(uint8_t n) {
  if (ac_done[n]) return true;
  SerialUART.print(F("refused: phase ")); SerialUART.print(n);
  SerialUART.println(F(" has not PASSED yet"));
  return false;
}

// ===========================================================================
// PHASE 1 -- SPI LINK. Zero current, ~0.3 s. Gates everything downstream:
// calibrating against a broken sensor burns a minute and yields plausible junk.
// ===========================================================================
static void acP1() {
  SerialUART.println(F("[1] LINK  (zero current)"));
  const uint16_t N = 4000;
  uint32_t e0 = encoder.spi_err, k0 = encoder.spi_ok;
  uint8_t  nmg = 0;
  uint32_t t0 = micros();
  for (uint16_t k = 0; k < N; k++) { if (encoder.readAngleRaw()) nmg |= encoder.no_mag; }
  uint32_t dt = micros() - t0;
  uint32_t errs = encoder.spi_err - e0, oks = encoder.spi_ok - k0;
  encoder.readOverSpeed();
  ac_link_us  = (float)dt / N;
  ac_link_err = errs;
  ac_v_link = (errs == 0 && oks == N && !nmg && !encoder.over_speed) ? AC_PASS : AC_FAIL;

  SerialUART.print(F("    n=")); SerialUART.print(N);
  SerialUART.print(F(" ok=")); SerialUART.print(oks);
  SerialUART.print(F(" perr=")); SerialUART.print(errs);
  SerialUART.print(F(" us_per_read=")); SerialUART.print(ac_link_us, 2);
  SerialUART.print(F(" nmg=")); SerialUART.print(nmg);
  SerialUART.print(F(" ovs=")); SerialUART.print(encoder.over_speed);
  SerialUART.print(F("   ")); SerialUART.println(acVs(ac_v_link));
  if (nmg)  SerialUART.println(F("    !! No_Mag_Warning: magnet weak or far. The angle is garbage."));
  if (errs) SerialUART.println(F("    !! parity errors: raise SPI_HALF_NOPS, check CSN and HVPP->3V3."));
  ac_done[1] = (ac_v_link == AC_PASS);
  if (ac_done[1]) SerialUART.println(F("    next: hand-spin the shaft and press 1 again, then press 2"));
}

// ===========================================================================
// PHASE 2 -- ZEA and DIRECTION. AC_ALIGN_N fresh alignments, wrap-safe median.
// Leaves the measured values INSTALLED on success (that is the point).
// ===========================================================================
static void acP2() {
  if (!acNeed(1)) return;
  SerialUART.println(F("[2] ALIGN  (expect AC_ALIGN_N twitches)"));
  acEnter();
  static float z[AC_ALIGN_N];
  uint8_t n = 0; int8_t dirsum = 0;

  for (uint8_t k = 0; k < AC_ALIGN_N; k++) {
    motor.zero_electric_angle = NOT_SET;
    motor.sensor_direction    = Direction::UNKNOWN;
    motor.enable();
    int ok = motor.initFOC();
    motor.disable();
    if (!ok) { SerialUART.println(F("    initFOC FAILED   FAIL")); ac_v_zea = AC_FAIL; acExit(false); return; }
    float v = motor.zero_electric_angle;
    // WRAP-SAFE. Measured values sit ~0.12 rad below 2*pi. One that lands just
    // past the boundary normalises to ~0.0, and a naive mean of
    // {6.05,6.05,6.05,6.05,6.05,6.05,0.01} is 5.19 -- badly wrong and silent.
    // Fold everything below pi up by 2*pi before ordering.
    if (v < _PI) v += _2PI;
    z[n++] = v;
    dirsum += (motor.sensor_direction == Direction::CW) ? +1 : -1;
    SerialUART.print(F("    ")); SerialUART.print(k+1);
    SerialUART.print(F(": ZEA=")); SerialUART.print(motor.zero_electric_angle, 4);
    SerialUART.print(F(" dir=")); SerialUART.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
    if (SerialUART.available()) { while (SerialUART.available()) (void)SerialUART.read(); ac_abort = true; break; }
  }
  if (ac_abort || n < 3) { SerialUART.println(F("    aborted   FAIL")); ac_v_zea = AC_FAIL; acExit(false); return; }

  // insertion sort -> median. Median not mean: two of sixteen alignments in the
  // earlier campaign were +6.5 deg outliers, which a mean would carry.
  for (uint8_t i = 1; i < n; i++) {
    float t = z[i]; int8_t j = (int8_t)i - 1;
    while (j >= 0 && z[j] > t) { z[j+1] = z[j]; j--; }
    z[j+1] = t;
  }
  float med = (n & 1) ? z[n/2] : 0.5f*(z[n/2 - 1] + z[n/2]);
  double s = 0, ss = 0;
  for (uint8_t i = 0; i < n; i++) { s += z[i]; ss += (double)z[i]*z[i]; }
  float mean = (float)(s/n);
  float sd   = (n > 1) ? (float)sqrt((ss - (double)n*mean*mean) / (n - 1)) : 0.0f;
  ac_zea    = (med >= _2PI) ? (med - _2PI) : med;
  ac_zea_sd = sd;
  ac_zea_se = 1.253f * sd / sqrtf((float)n);          // SE of a median
  ac_dir    = (dirsum > 0) ? +1 : -1;
  uint8_t dirmag = (uint8_t)((dirsum < 0) ? -dirsum : dirsum);
  bool dir_split = (dirmag != n);

  ac_v_zea = AC_PASS;
  if (dir_split)                       ac_v_zea = AC_FAIL;
  else if (sd > AC_ZEA_SD_FAIL)        ac_v_zea = AC_FAIL;
  else if (ac_zea_se > AC_ZEA_SE_FAIL) ac_v_zea = AC_FAIL;
  else if (sd > AC_ZEA_SD_WARN)        ac_v_zea = AC_WARN;

  SerialUART.print(F("    n=")); SerialUART.print(n);
  SerialUART.print(F(" ZEA=")); SerialUART.print(ac_zea, 4);
  SerialUART.print(F(" sd=")); SerialUART.print(sd, 4);
  SerialUART.print(F(" (")); SerialUART.print(degrees(sd), 2); SerialUART.print(F(" deg elec)"));
  SerialUART.print(F(" SE=")); SerialUART.print(degrees(ac_zea_se), 2); SerialUART.print(F(" deg"));
  SerialUART.print(F(" dir=")); SerialUART.print(ac_dir > 0 ? F("CW") : F("CCW"));
  SerialUART.print(F("   ")); SerialUART.println(acVs(ac_v_zea));
  if (dir_split) SerialUART.println(F("    !! direction detection DISAGREED between runs -- sensor or wiring."));

  if (ac_v_zea == AC_FAIL) { ac_done[2] = false; acExit(false); return; }

  // Install the measured values so phases 3-6 commutate with them.
  motor.zero_electric_angle = ac_zea;
  motor.sensor_direction    = (ac_dir > 0) ? Direction::CW : Direction::CCW;
  motor.enable(); (void)motor.initFOC(); motor.disable();
  foc_ready = true;
  acExit(true);                       // KEEP the alignment
  ac_done[2] = true;
  SerialUART.println(F("    measured ZEA is now INSTALLED for this session. next: 3"));
}

// ===========================================================================
// PHASE 3 -- R_eff and U0. Magnetic self-lock, rotor STATIONARY, sweep DOWN.
// ===========================================================================
static void acP3() {
  if (!acNeed(2)) return;
  SerialUART.println(F("[3] R/U0  (rotor self-locks; do not touch the shaft)"));
  acEnter();
  mode = MODE_OPENLOOP;
  motor.controller = MotionControlType::velocity_openloop;
  target = 0.0f;                          // zero openloop velocity = FIXED field angle
  motor.voltage_limit = AC_R_V[0];        // strongest hold FIRST
  motor.enable(); running = true; run_started = millis();
  acRun(600);                             // long: the rotor may swing up to ~26 deg mech

  ac_rn = 0; ac_rdrop = 0;
  float vq_err_max = 0.0f;
  for (uint8_t k = 0; k < AC_R_N && !ac_abort; k++) {
    // velocityOpenloop() uses motor.voltage_limit directly. motor.phase_resistance
    // is UNSET in this sketch, so its current-limit branch is not taken.
    motor.voltage_limit = AC_R_V[k];
    acRun(AC_R_SETTLE_MS);
    uint16_t c0 = encoder.raw;
    double acc = 0, accv = 0; uint32_t n = 0;
    uint32_t t0 = millis();
    while ((millis() - t0) < AC_R_MEAS_MS && !ac_abort) {
      acService(); acc += ac_i_amp; accv += motor.voltage.q; n++;
    }
    if (n < 100) {
      // Not a drift drop -- the measurement window produced too few samples
      // (an abort, or a loop rate collapse). Counted separately so that
      // ac_rn + ac_rdrop always reconciles against AC_R_N; a silently vanishing
      // point used to make the tally line lie.
      ac_rdrop++;
      SerialUART.print(F("    DROP U=")); SerialUART.print(AC_R_V[k], 3);
      SerialUART.print(F(" -- only ")); SerialUART.print(n);
      SerialUART.println(F(" samples in the window"));
      continue;
    }
    int32_t moved = acCntDelta(c0, encoder.raw);
    if (acAbs32(moved) > AC_R_STILL_CNT) {
      ac_rdrop++;
      SerialUART.print(F("    DROP U=")); SerialUART.print(AC_R_V[k], 3);
      SerialUART.print(F(" -- rotor moved ")); SerialUART.print(moved);
      SerialUART.println(F(" counts (back-EMF would corrupt this point)"));
      continue;
    }
    float I = (float)(acc / n);
    float vq = (float)(accv / n);         // what move() actually published
    float e = fabsf(vq - AC_R_V[k]);
    if (e > vq_err_max) vq_err_max = e;
    ac_rx[ac_rn] = I;
    ac_ry[ac_rn] = AC_R_V[k];             // COMMANDED voltage is the independent variable
    ac_rn++;
    SerialUART.print(F("    U=")); SerialUART.print(AC_R_V[k], 3);
    SerialUART.print(F(" I=")); SerialUART.print(I, 4);
    SerialUART.print(F(" drift=")); SerialUART.print(moved);
    SerialUART.print(F(" n=")); SerialUART.println(n);
  }
  acExit(true);                           // keep the phase-2 alignment
  if (ac_abort) { SerialUART.println(F("    aborted   FAIL")); ac_v_R = AC_FAIL; ac_done[3] = false; return; }

  AcFit f = acFit(ac_rx, ac_ry, ac_rn);
  ac_R = f.m; ac_U0 = f.c; ac_R_se = f.se_m; ac_U0_se = f.se_c; ac_R_rms = f.rms;
  ac_U0_sig = (ac_U0_se > 1e-9f) ? fabsf(ac_U0)/ac_U0_se : 0.0f;

  ac_v_R = AC_PASS;
  if (!f.ok || ac_rn < AC_R_MIN_PTS)        ac_v_R = AC_FAIL;
  else if (ac_R < 0.05f || ac_R > 0.60f)    ac_v_R = AC_FAIL;
  else if (ac_R_rms > AC_R_RMS_FAIL)        ac_v_R = AC_FAIL;
  else if (ac_R_se/ac_R > AC_R_RSE_FAIL)    ac_v_R = AC_FAIL;
  // U0 is the INTERCEPT and is systematically the badly-conditioned parameter of
  // this fit. It gets its OWN verdict rather than inheriting the slope's.
  ac_v_U0 = (ac_U0_sig >= 3.0f) ? AC_PASS : (ac_U0_sig >= 2.0f) ? AC_WARN : AC_FAIL;

  SerialUART.print(F("    ")); SerialUART.print(ac_rn); SerialUART.print(F("/"));
  SerialUART.print(AC_R_N); SerialUART.print(F(" pts, ")); SerialUART.print(ac_rdrop);
  SerialUART.print(F(" dropped   R=")); SerialUART.print(ac_R, 5);
  SerialUART.print(F(" +-")); SerialUART.print(ac_R_se, 5);
  SerialUART.print(F(" (")); SerialUART.print(100.0f*ac_R_se/ac_R, 2);
  SerialUART.print(F("%) rms=")); SerialUART.print(ac_R_rms*1000.0f, 2);
  SerialUART.print(F("mV   ")); SerialUART.println(acVs(ac_v_R));
  SerialUART.print(F("    U0=")); SerialUART.print(ac_U0, 5);
  SerialUART.print(F(" +-")); SerialUART.print(ac_U0_se, 5);
  SerialUART.print(F(" (")); SerialUART.print(ac_U0_sig, 1);
  SerialUART.print(F(" sigma)   ")); SerialUART.println(acVs(ac_v_U0));
  if (ac_v_U0 != AC_PASS)
    SerialUART.println(F("    !! U0 NOT DETERMINED by this sweep (intercept under 3 sigma of zero)."));
  // Cross-check: move() should publish voltage.q equal to what we commanded via
  // motor.voltage_limit. A real difference means something clamped it (e.g.
  // driver.voltage_limit). Reported, not treated as a failure, because whether
  // 2.3.1 publishes voltage.q in the openloop path is a library detail we do not
  // depend on -- the fit uses the COMMANDED value either way.
  if (vq_err_max > 0.002f) {
    SerialUART.print(F("    note: published Uq differs from commanded by up to "));
    SerialUART.print(vq_err_max*1000.0f, 1);
    SerialUART.println(F(" mV (fit uses the commanded value)"));
  }
  ac_done[3] = (ac_v_R != AC_FAIL);
  if (ac_done[3]) SerialUART.println(F("    next: 4 (L) or 5 (free-spin)"));
}

// ===========================================================================
// PHASE 4 -- L. AC_L_REPS averaged steps, still self-locked.
//
// Confidence comes from three things, not one clever measurement:
//  (a) The step is DIFFERENTIAL, VBASE -> VSTEP, so the rotor stays firmly held
//      throughout. De-energising between repeats would excite the ~18 Hz
//      magnetic-spring resonance and put mechanical motion into the trace.
//  (b) AC_L_REPS repeats are averaged sample-by-sample: noise falls as
//      1/sqrt(24), about 1.3% on a 2.5 A step versus ~15% single-shot.
//  (c) The fit is LINEARISED. ln((Iinf - i)/(Iinf - I0)) is linear in t with
//      slope -1/tau. Iinf and I0 come from the SAME averaged trace, so the
//      result does not depend on phase 3, and the fitted INTERCEPT absorbs the
//      unknown one-sample transport lag instead of biasing tau. Only the slope
//      is used; the intercept is reported as t_d for sanity (expect 1-2 sample
//      periods, i.e. roughly 70-150 us).
// Only samples in the 0.15-0.85 band are fitted: below that the lag dominates,
// above it the noise on a small (Iinf - i) does.
// R converts tau to L, so this phase needs phase 3 -- but only for that scaling.
// ===========================================================================
static void acP4() {
  if (!acNeed(3)) return;
  SerialUART.println(F("[4] L  (step train, rotor stays locked)"));
  acEnter();
  for (uint8_t k = 0; k < AC_L_TBINS; k++) { ac_lb_i[k] = 0.0f; ac_lb_n[k] = 0; }
  double tail_acc = 0; uint32_t tail_n = 0;
  mode = MODE_OPENLOOP;
  motor.controller = MotionControlType::velocity_openloop;
  target = 0.0f;
  motor.voltage_limit = AC_L_VBASE;
  motor.enable(); running = true; run_started = millis();
  acRun(600);                                    // lock at the base voltage
  uint16_t c0 = encoder.raw;
  double i0acc = 0; uint16_t i0n = 0;
  uint8_t reps = 0;

  for (uint8_t r = 0; r < AC_L_REPS && !ac_abort; r++) {
    motor.voltage_limit = AC_L_VBASE;
    acRun(AC_L_DECAY_MS);                        // >= 9 tau_e: fully relaxed
    for (uint8_t q = 0; q < 4 && !ac_abort; q++) { acService(); i0acc += ac_i_amp; i0n++; }
    uint32_t t0 = micros();
    motor.voltage_limit = AC_L_VSTEP;
    for (uint8_t k = 0; k < AC_L_NS && !ac_abort; k++) {
      acService();
      uint32_t dt = micros() - t0;
      // F4: bin by TIME. The step phase relative to the loop is random across
      // repeats, so these land scattered -- equivalent-time sampling for free.
      if (dt < (uint32_t)AC_L_TBINS * AC_L_TBIN_US) {
        uint8_t b = (uint8_t)(dt / AC_L_TBIN_US);
        ac_lb_i[b] += ac_i_amp; ac_lb_n[b]++;
      } else if (dt > AC_L_TAIL_US) {
        tail_acc += ac_i_amp; tail_n++;          // steady state -> I_inf
      }
      // F5 -- see the AC_L_DITHER_US note. Must be AFTER dt is binned, or sample
      // 0's own timestamp gets inflated by the stall.
      if (k == 0 && r) delayMicroseconds((unsigned int)r * AC_L_DITHER_US);
    }
    reps++;
  }
  int32_t moved = acCntDelta(c0, encoder.raw);
  acExit(true);
  if (ac_abort || reps < 6) { SerialUART.println(F("    aborted   FAIL")); ac_v_L = AC_FAIL; ac_done[4] = false; return; }

  ac_l_I0     = (float)(i0acc / i0n);
  ac_l_Iinf   = tail_n ? (float)(tail_acc / tail_n) : 0.0f;
  ac_l_tail_n = (float)tail_n;

  // Linearised fit: ln((Iinf - i)/(Iinf - I0)) = -(t - t_d)/tau. Only the SLOPE is
  // used, so the fitted intercept absorbs the unknown transport lag rather than
  // biasing tau; t_d is reported purely as a sanity number (expect ~1-2 sample
  // periods). I0 and Iinf come from the SAME trace, so tau does not depend on
  // phase 3 -- R only converts tau into L at the end.
  static float lx[AC_L_TBINS], ly[AC_L_TBINS];
  uint8_t n = 0;
  float span = ac_l_Iinf - ac_l_I0;
  if (span > 0.30f && tail_n > 20) {
    for (uint8_t b = 0; b < AC_L_TBINS; b++) {
      if (ac_lb_n[b] < AC_L_BIN_MIN_N) continue;
      float frac = ((ac_lb_i[b] / ac_lb_n[b]) - ac_l_I0) / span;
      if (frac > 0.15f && frac < 0.85f) {
        lx[n] = (b + 0.5f) * AC_L_TBIN_US * 1e-6f;
        ly[n] = logf(1.0f - frac);
        n++;
      }
    }
  }
  ac_l_pts = n;
  AcFit f  = acFit(lx, ly, n);
  ac_tau   = (f.ok && f.m < -1e-6f) ? (-1.0f / f.m) : 0.0f;
  ac_L     = ac_tau * ac_R;
  ac_L_rms = f.rms;
  ac_L_td  = (ac_tau > 0.0f) ? (f.c * ac_tau) : 0.0f;

  ac_v_L = AC_PASS;
  if (n < AC_L_MIN_PTS || !f.ok || ac_tau <= 0.0f) ac_v_L = AC_FAIL;
  else if (ac_L < 15e-6f || ac_L > 400e-6f)        ac_v_L = AC_FAIL;
  else if (ac_L_rms > AC_L_RMS_FAIL)               ac_v_L = AC_FAIL;
  else if (tail_n < 40)                            ac_v_L = AC_WARN;   // weak I_inf
  else if (acAbs32(moved) > 40)                    ac_v_L = AC_WARN;
  else if (ac_L_td < 0.0f || ac_L_td > 400e-6f)    ac_v_L = AC_WARN;

  SerialUART.print(F("    reps=")); SerialUART.print(reps);
  SerialUART.print(F(" fitpts=")); SerialUART.print(ac_l_pts);
  SerialUART.print(F("/")); SerialUART.print(AC_L_TBINS);
  SerialUART.print(F(" tail_n=")); SerialUART.print(tail_n);
  SerialUART.print(F(" I0=")); SerialUART.print(ac_l_I0, 3);
  SerialUART.print(F(" Iinf=")); SerialUART.print(ac_l_Iinf, 3);
  SerialUART.print(F(" tau=")); SerialUART.print(ac_tau*1e6f, 1); SerialUART.print(F("us"));
  SerialUART.print(F(" L=")); SerialUART.print(ac_L*1e6f, 1); SerialUART.print(F("uH"));
  SerialUART.print(F(" rms=")); SerialUART.print(ac_L_rms, 3);
  SerialUART.print(F(" td=")); SerialUART.print(ac_L_td*1e6f, 0); SerialUART.print(F("us"));
  SerialUART.print(F(" drift=")); SerialUART.print(moved);
  SerialUART.print(F("   ")); SerialUART.println(acVs(ac_v_L));
  if (ac_v_L == AC_WARN) SerialUART.println(F("    !! rotor drifted, I_inf weak, or t_d implausible -- refit the bins offline."));
  SerialUART.println(F("    NOTE: L is measured at 0.9 -> 3.2 A, so it is the INCREMENTAL"));
  SerialUART.println(F("    inductance near the operating point. Partial saturation and eddy"));
  SerialUART.println(F("    currents make it legitimately LOWER than a small-signal value."));
  ac_done[4] = (ac_v_L != AC_FAIL);
  if (ac_done[4]) SerialUART.println(F("    next: 5"));
}

// ===========================================================================
// PHASE 5 -- FREE-SPIN SWEEP. Yields Ke, Kt, the drag map, and (binned) the
// data phase 6 turns into T_delay and INL. ~50 s.
// Uses MODE_TORQUE (voltage-torque) so Uq is COMMANDED directly and the speed
// self-regulates to where Iq equals the drag current -- no velocity loop needed.
// ===========================================================================
static void acP5() {
  if (!acNeed(3)) return;
  SerialUART.println(F("[5] FREE-SPIN  (shaft must be free; ~50 s)"));
  acEnter();
  for (uint8_t d = 0; d < 2; d++)
    for (uint8_t s = 0; s < AC_BIN_SPEEDS; s++) {
      ac_bin_w[d][s] = 0.0f;
      for (uint8_t b = 0; b < AC_BINS; b++) {
        ac_bin_id[d][s][b] = 0.0f; ac_bin_iq[d][s][b] = 0.0f; ac_bin_n[d][s][b] = 0;
      }
    }
  ac_wn = 0; ac_ratio_lo = 99.0f; ac_ratio_hi = 0.0f;
  // ac_floop is indexed by (direction, speed index), NOT by ac_wn, so a point
  // skipped for n < 500 leaves its slot untouched. Without this clear that slot
  // would still hold the PREVIOUS run's loop rate and phase 6 would divide a
  // fresh T by a stale f_loop. Statics are zeroed at boot, so only a re-run was
  // ever exposed -- which is exactly what a re-run is for.
  for (uint8_t i = 0; i < 2*AC_W_N; i++) ac_floop[i] = 0.0f;
  for (uint8_t d = 0; d < 2; d++) { ac_drag_c[d] = ac_drag_v[d] = ac_drag_rms[d] = 0.0f; ac_drag_n[d] = 0; }

  mode = MODE_TORQUE;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  // Raised so the top point (2.00 V) is NOT clamped. A saturated Uq makes the
  // Ke fit meaningless -- Uq stops being the independent variable.
  motor.voltage_limit       = AC_W_VLIMIT;
  motor.PID_current_q.limit = AC_W_VLIMIT;
  motor.PID_current_d.limit = AC_W_VLIMIT;

  for (uint8_t d = 0; d < 2 && !ac_abort; d++) {
    float sgn = (d == 0) ? +1.0f : -1.0f;
    SerialUART.println(d == 0 ? F("    -- FORWARD --") : F("    -- REVERSE --"));
    if (d == 1) acCoast();                       // never command a reversing voltage
    if (ac_abort) break;
    target = 0.0f;
    motor.PID_current_q.reset(); motor.PID_current_d.reset();
    motor.enable(); running = true; run_started = millis();

    for (uint8_t k = 0; k < AC_W_N && !ac_abort; k++) {
      float U = sgn * AC_W_V[k];
      acRampTarget(U);
      run_started = millis();
      acRun(AC_W_SETTLE_MS);
      if (ac_abort) break;

      int8_t slot = (k >= AC_W_N - AC_BIN_SPEEDS) ? (int8_t)(k - (AC_W_N - AC_BIN_SPEEDS)) : -1;
      double aIq = 0, aId = 0, aV = 0;
      double aP2 = 0, aD2 = 0;                  // F1: SQUARED accumulators
      uint32_t n = 0, nr = 0, ratio_div = 0;
      ac_loops = 0;
      uint32_t tl0 = millis(), t0 = millis();
      while ((millis() - t0) < AC_W_MEAS_MS && !ac_abort) {
        acService();
        float iq = motor.current.q, id = motor.current.d;
        aIq += iq; aId += id; aV += motor.shaft_velocity; n++;
        // F1 -- ratio gate in the SQUARED domain. |I|^2 = 1.5*(Iq^2+Id^2)
        // instantaneously, so accumulating squares and taking ONE sqrt at the end
        // removes the always-positive averaging bias that made the old per-sample
        // mean(|I|)/|Iq| read 1.27-1.36 instead of 1.2247.
        // Sampled 1-in-4: getPhaseCurrents() costs ~18 us and the loop rate is
        // load-bearing (T_delay ~= one loop period).
        if (++ratio_div >= 4) {
          ratio_div = 0;
          aP2 += acPhaseMag2(); aD2 += (double)iq*iq + (double)id*id; nr++;
        }
        if (slot >= 0) {
          uint8_t b = (uint8_t)(((uint32_t)encoder.raw * AC_BINS) >> ENC_BITS);  // /ENC_CPR
          if (b < AC_BINS) {
            ac_bin_id[d][slot][b] += id;
            ac_bin_iq[d][slot][b] += iq;
            ac_bin_n [d][slot][b]++;
          }
        }
      }
      if (n < 500) continue;
      uint32_t tl_ms = millis() - tl0;
      if (tl_ms > 0) ac_floop[ (d*AC_W_N) + k ] = (float)ac_loops * 1000.0f / (float)tl_ms;
      float mIq = (float)(aIq/n), mId = (float)(aId/n), mV = (float)(aV/n);
      // one sqrt, at the end, on the RATIO of means-of-squares
      float mR = (nr > 50 && aD2 > 1e-9) ? sqrtf((float)(aP2 / (1.5 * aD2))) * AMP_INV_MAG : 0.0f;
      if (mR > 0.5f) { if (mR < ac_ratio_lo) ac_ratio_lo = mR; if (mR > ac_ratio_hi) ac_ratio_hi = mR; }
      if (slot >= 0) ac_bin_w[d][slot] = mV;

      // Ke fit input. Uq - R*Iq - U0*sign(Iq) = Ke*omega. Everything signed, so
      // both directions feed ONE fit that should pass through the origin.
      ac_wx[ac_wn]    = mV;
      ac_wy[ac_wn]    = U - ac_R*mIq - ((mIq >= 0.0f) ? ac_U0 : -ac_U0);
      ac_w_vel[ac_wn] = mV; ac_w_iq[ac_wn] = mIq; ac_w_u[ac_wn] = U;
      ac_wn++;

      SerialUART.print(F("    U=")); SerialUART.print(U, 2);
      SerialUART.print(F(" vel=")); SerialUART.print(mV, 2);
      SerialUART.print(F(" Iq=")); SerialUART.print(mIq, 4);
      SerialUART.print(F(" Id=")); SerialUART.print(mId, 4);
      SerialUART.print(F(" ratio=")); SerialUART.print(mR, 3);
      SerialUART.print(F(" n=")); SerialUART.println(n);
    }
    if (!ac_abort) acRampTarget(0.0f);
    motor.disable(); running = false;
  }
  acCoast();
  acExit(true);
  if (ac_abort) { SerialUART.println(F("    aborted   FAIL")); ac_v_Ke = AC_FAIL; ac_done[5] = false; return; }

  AcFit f = acFit(ac_wx, ac_wy, ac_wn);
  ac_Ke = f.m; ac_Ke_se = f.se_m; ac_Ke_rms = f.rms; ac_Ke_c = f.c;
  // Kt is NOT fitted and NOT stored. Amplitude-invariant dq forces Kt = 1.5*Ke
  // exactly (KT_PER_KE in fleet_config.h); calKt() derives it on demand.

  // ---- DRAG, per direction ----------------------------------------------
  // Iq = drag_c + drag_v*|omega|, fitted on MAGNITUDES so both directions come
  // out positive and the consumer applies sign(omega). Split by the sign of the
  // measured speed rather than by loop index, because a point dropped for
  // n < 500 breaks any index-to-direction mapping.
  // These four numbers used to be fitted by hand off the phase-7 CSV. Twelve
  // joints x four numbers is exactly the arithmetic this routine exists to
  // remove, and hand-fitting was what collapsed them to a single mean in the
  // first place.
  {
    static float dx[AC_W_N], dy[AC_W_N];
    for (uint8_t d = 0; d < 2; d++) {
      uint8_t n = 0;
      for (uint8_t i = 0; i < ac_wn && n < AC_W_N; i++) {
        bool fwd = (ac_w_vel[i] > 0.0f);
        if (fwd != (d == 0)) continue;
        if (fabsf(ac_w_vel[i]) < 1.0f) continue;      // not actually spinning
        dx[n] = fabsf(ac_w_vel[i]);
        dy[n] = fabsf(ac_w_iq[i]);
        n++;
      }
      AcFit g = acFit(dx, dy, n);
      ac_drag_n[d]   = n;
      ac_drag_c[d]   = g.ok ? g.c : 0.0f;
      ac_drag_v[d]   = g.ok ? g.m : 0.0f;
      ac_drag_rms[d] = g.ok ? g.rms : 0.0f;
    }
  }
  // A negative Coulomb intercept is unphysical and means the sweep never got
  // below the speed where viscous drag dominates -- report it, do not hide it.
  ac_v_drag = AC_PASS;
  for (uint8_t d = 0; d < 2; d++) {
    if (ac_drag_n[d] < 3)                          ac_v_drag = AC_FAIL;
    else if (ac_drag_c[d] < 0.0f || ac_drag_v[d] < 0.0f) ac_v_drag = AC_WARN;
  }

  ac_v_Ke = AC_PASS;
  if (!f.ok || ac_wn < 6)                    ac_v_Ke = AC_FAIL;
  else if (ac_Ke < 0.008f || ac_Ke > 0.040f) ac_v_Ke = AC_FAIL;
  else if (ac_Ke_rms > AC_KE_RMS_FAIL)       ac_v_Ke = AC_FAIL;
  else if (ac_Ke_se/ac_Ke > AC_KE_RSE_FAIL)  ac_v_Ke = AC_FAIL;
  // A non-zero intercept means R or U0 is wrong. It is the one INDEPENDENT check
  // on phase 3, so it downgrades Ke rather than passing quietly.
  else if (fabsf(ac_Ke_c) > AC_KE_C_WARN)    ac_v_Ke = AC_WARN;

  ac_v_ratio = (ac_ratio_lo >= AC_RATIO_LO && ac_ratio_hi <= AC_RATIO_HI) ? AC_PASS : AC_FAIL;

  SerialUART.print(F("    Ke=")); SerialUART.print(ac_Ke, 6);
  SerialUART.print(F(" +-")); SerialUART.print(ac_Ke_se, 6);
  SerialUART.print(F(" (")); SerialUART.print(100.0f*ac_Ke_se/ac_Ke, 2);
  SerialUART.print(F("%) rms=")); SerialUART.print(ac_Ke_rms*1000.0f, 2);
  SerialUART.print(F("mV intercept=")); SerialUART.print(ac_Ke_c, 4);
  SerialUART.print(F("   ")); SerialUART.println(acVs(ac_v_Ke));
  if (ac_v_Ke == AC_WARN)
    SerialUART.println(F("    !! Ke intercept far from zero -> R or U0 from phase 3 is off. Re-run 3."));
  SerialUART.print(F("    |I| ratio ")); SerialUART.print(ac_ratio_lo, 3);
  SerialUART.print(F(" .. ")); SerialUART.print(ac_ratio_hi, 3);
  SerialUART.print(F(" (theory 1.2247, squared-domain)   ")); SerialUART.println(acVs(ac_v_ratio));
  SerialUART.println(F("    NOTE: this is a GROSS-SANITY band only. The tight 1.22-1.23 check"));
  SerialUART.println(F("    is a LOCKED-ROTOR measurement -- do it by hand in 'c' mode."));
  for (uint8_t d = 0; d < 2; d++) {
    SerialUART.print(d == 0 ? F("    drag fwd  ") : F("    drag rev  "));
    SerialUART.print(F("Iq = ")); SerialUART.print(ac_drag_c[d], 4);
    SerialUART.print(F(" + ")); SerialUART.print(ac_drag_v[d], 6);
    SerialUART.print(F("*|w|   n=")); SerialUART.print(ac_drag_n[d]);
    SerialUART.print(F(" rms=")); SerialUART.print(ac_drag_rms[d]*1000.0f, 2);
    SerialUART.println(F(" mA"));
  }
  SerialUART.print(F("    ")); SerialUART.println(acVs(ac_v_drag));
  SerialUART.println(F("    DYNAMIC drag only. The STATIC breakaway threshold is larger and"));
  SerialUART.println(F("    is not measurable here -- run M4 by hand and fill breakaway_A."));
  ac_done[5] = (ac_v_Ke != AC_FAIL);
  if (ac_done[5]) SerialUART.println(F("    next: 6"));
}

// ===========================================================================
// PHASE 6 -- T_delay and INL by PARITY SEPARATION. Computation only, no motor.
//
// With Ud forced to 0, sin(delta) = A/Ke + B*omega/Ke, so under omega -> -omega:
//   ZEA residual  constant in sin(delta), SAME sign both directions  -> EVEN
//   INL(theta)    a fixed angle error at a rotor position, SAME sign -> EVEN
//   transport T   delta = omega_e*T, proportional to omega, FLIPS    -> ODD
//   even = (fwd+rev)/2 -> ZEA residual + INL      odd = (fwd-rev)/2 -> T
//
// Forward-only data CANNOT do this. Three sessions of forward-only fitting gave
// T = 85.7 us with a 28.7 us unexplained excess; the two-direction split gave
// 57.0 us, plus the INL profile for free. The two binned speeds give two
// independent T estimates, and T must be speed-INDEPENDENT -- that consistency is
// what makes it a measurement rather than a reading.
//
// T MODEL, CORRECTED 2026-08-07:  T ~= ONE FULL LOOP PERIOD.
// Three points across two loop rates:
//     f_loop 16770 -> T_loop 59.6 us, T 57.0  ->  T/T_loop 0.956
//     f_loop 12911 -> T_loop 77.5 us, T 73.8  ->  T/T_loop 0.953
//     f_loop 12908 -> T_loop 77.5 us, T 75.8  ->  T/T_loop 0.978
// The old "0.5*T_pwm + 0.5*T_loop" model predicts 49.8 / 58.7 / 58.7 and is 15-29%
// low. The PWM period (40 us) is SHORTER than the loop period, so the duty update
// lands inside one PWM cycle and the loop period dominates outright.
// Consequence: LOOP RATE BUYS TRANSPORT DELAY ONE-FOR-ONE, and T measured in here
// is only the sketch's T if the loop rates match -- hence f_loop is reported with
// it, and T/T_loop is the transferable number.
// ===========================================================================
static float acBinAngleDeg(uint8_t d, uint8_t s, uint8_t b) {
  if (ac_bin_n[d][s][b] == 0) return 0.0f;
  float Id = ac_bin_id[d][s][b] / ac_bin_n[d][s][b];
  float Iq = ac_bin_iq[d][s][b] / ac_bin_n[d][s][b];
  float w  = ac_bin_w[d][s];
  if (fabsf(w) < 1.0f) return 0.0f;
  float we = (float)MOTOR_POLE_PAIRS * w;
  float y  = ac_R*Id - we*ac_L*Iq;            // volts attributable to angle error
  float sn = y / (ac_Ke * w);
  if (sn >  0.999f) sn =  0.999f;
  if (sn < -0.999f) sn = -0.999f;
  return degrees(asinf(sn));
}

static void acP6() {
  if (!acNeed(4)) return;
  if (!acNeed(5)) return;
  SerialUART.println(F("[6] T + INL  (parity separation, no motor)"));
  bool any = false;
  float e_lo = 1e9f, e_hi = -1e9f, e_sum = 0.0f; uint8_t e_n = 0;

  for (uint8_t s = 0; s < AC_BIN_SPEEDS; s++) {
    ac_T[s] = 0.0f;
    // Hoisted out of the bin loop. acBinAngleDeg() returns a hard 0.0 when the
    // slot's speed was never established (phase 5 dropped that sweep point), and
    // a bin full of zeros used to be folded into the INL even-part as if it were
    // data -- biasing the profile toward zero -- before the slot was discarded a
    // few lines later. Discard it first.
    float w = 0.5f*(fabsf(ac_bin_w[0][s]) + fabsf(ac_bin_w[1][s]));
    if (w < 1.0f) continue;
    double os = 0, oss = 0; uint8_t on = 0;
    for (uint8_t b = 0; b < AC_BINS; b++) {
      if (ac_bin_n[0][s][b] < AC_BIN_MIN_N || ac_bin_n[1][s][b] < AC_BIN_MIN_N) continue;
      float A = acBinAngleDeg(0, s, b), B = acBinAngleDeg(1, s, b);
      float odd = 0.5f*(A - B), even = 0.5f*(A + B);
      os += odd; oss += (double)odd*odd; on++;
      if (s == AC_BIN_SPEEDS - 1) {                 // profile from the fastest pair
        if (even < e_lo) e_lo = even;
        if (even > e_hi) e_hi = even;
        e_sum += even; e_n++;
      }
    }
    if (on < 8) continue;
    float om  = (float)(os/on);
    float osd = (on > 1) ? (float)sqrt((oss - (double)on*om*om) / (on - 1)) : 0.0f;
    ac_T[s] = radians(om) / ((float)MOTOR_POLE_PAIRS * w);
    any = true;
    // f_loop for the speeds that were binned: the last AC_BIN_SPEEDS of each
    // direction. Average forward and reverse.
    uint8_t idx_f = (AC_W_N - AC_BIN_SPEEDS) + s;
    uint8_t idx_r = AC_W_N + idx_f;
    float fl = 0.5f*(ac_floop[idx_f] + ac_floop[idx_r]);
    ac_T_floop[s] = fl;
    SerialUART.print(F("    w=+-")); SerialUART.print(w, 1);
    SerialUART.print(F(" bins=")); SerialUART.print(on);
    SerialUART.print(F(" odd=")); SerialUART.print(om, 3);
    SerialUART.print(F("+-")); SerialUART.print(osd, 3);
    SerialUART.print(F(" deg -> T=")); SerialUART.print(ac_T[s]*1e6f, 1);
    SerialUART.print(F(" us | f_loop=")); SerialUART.print(fl, 0);
    if (fl > 1.0f) {
      SerialUART.print(F(" T_loop=")); SerialUART.print(1e6f/fl, 1);
      SerialUART.print(F(" us  T/T_loop=")); SerialUART.print(ac_T[s]*fl, 3);
      // T/T_loop is the transferable number; T in microseconds is only true at
      // the loop rate it was taken at. Compare against the fleet constant so a
      // drifting assembly shows up here rather than in a README table later.
      SerialUART.print(F(" (fleet ")); SerialUART.print(T_DELAY_PER_LOOP, 3);
      SerialUART.print(F("+-")); SerialUART.print(T_DELAY_PER_LOOP_SD, 3);
      SerialUART.print(F(")"));
    }
    SerialUART.println();
  }
  ac_inl_pp    = e_n ? (e_hi - e_lo) : 0.0f;
  ac_zea_resid = e_n ? (e_sum / e_n) : 0.0f;
  float Tf = ac_T[AC_BIN_SPEEDS - 1];

  ac_v_T = AC_PASS;
  if (!any || Tf <= 0.0f || Tf > 300e-6f) ac_v_T = AC_FAIL;
  else if (AC_BIN_SPEEDS >= 2 && ac_T[0] > 0.0f) {
    float rel = fabsf(ac_T[0] - ac_T[1]) / (0.5f*(ac_T[0] + ac_T[1]));
    if (rel > AC_T_AGREE) ac_v_T = AC_FAIL;
  }
  else if (AC_BIN_SPEEDS >= 2) ac_v_T = AC_WARN;      // only one estimate survived

  SerialUART.print(F("    INL pk-pk ")); SerialUART.print(ac_inl_pp, 2);
  SerialUART.print(F(" deg elec = ")); SerialUART.print(ac_inl_pp/(float)MOTOR_POLE_PAIRS, 3);
  SerialUART.print(F(" deg MECH | ZEA residual ")); SerialUART.print(ac_zea_resid, 3);
  SerialUART.print(F(" deg elec   ")); SerialUART.println(acVs(ac_v_T));
  if (ac_v_T == AC_FAIL)
    SerialUART.println(F("    !! T inconsistent across speeds -> the fixed-delay model does NOT hold here."));
  if (ac_inl_pp/(float)MOTOR_POLE_PAIRS > 1.5f)
    SerialUART.println(F("    !! INL above the 1.5 deg datasheet max -- check magnet CENTRING (1/rev term)."));
  if (fabsf(ac_zea_resid) > 3.0f)
    SerialUART.println(F("    !! ZEA residual large -- the installed ZEA is off by more than 3 deg elec."));
  ac_done[6] = (ac_v_T != AC_FAIL);
  if (ac_done[6]) SerialUART.println(F("    next: 7 (report)"));
}

// ===========================================================================
// PHASE 7 -- REPORT. Nothing that has not been MEASURED is emitted as a number.
// ===========================================================================
// One line of the verdict table: value, verdict, what it is. NOT pasteable --
// the single pasteable artefact is the JointCal row further down, so there is
// exactly one thing to copy and no way to paste half a calibration.
static void acPrintVal(const __FlashStringHelper* name, float v, uint8_t dp,
                       bool measured, AcV verdict, const __FlashStringHelper* note) {
  SerialUART.print(F("  ")); SerialUART.print(name);
  if (measured) SerialUART.print(v, dp);
  else          SerialUART.print(F("--"));
  SerialUART.print(F("\t[")); SerialUART.print(measured ? acVs(verdict) : F("MISSING"));
  SerialUART.print(F("]  ")); SerialUART.println(note);
}

// A struct field in the pasteable row. Emits a compilable literal in every case:
// a measured value, or the documented safe default plus the phase still to run.
static void acRowF(float v, uint8_t dp, bool measured, uint8_t phase) {
  if (measured) { SerialUART.print(v, dp); SerialUART.print(F("f")); }
  else { SerialUART.print(F("0.0f /* NOT MEASURED - run ")); SerialUART.print(phase);
         SerialUART.print(F(" */")); }
}

static void acP7() {
  AcV worst = AC_PASS;
  bool complete = true;
  for (uint8_t n = 1; n <= 6; n++) if (!ac_done[n]) complete = false;
  AcV vs[8] = { AC_PASS, ac_v_link, ac_v_zea, ac_v_R, ac_v_L, ac_v_Ke, ac_v_T, AC_PASS };
  for (uint8_t n = 1; n <= 6; n++) if (ac_done[n] && vs[n] > worst) worst = vs[n];
  if (ac_done[3] && ac_v_U0    > worst) worst = ac_v_U0;
  if (ac_done[5] && ac_v_ratio > worst) worst = ac_v_ratio;
  // The drag fit feeds four fields of the pasteable row, so its verdict has to
  // reach the headline. A negative Coulomb intercept is a number you must not paste.
  if (ac_done[5] && ac_v_drag  > worst) worst = ac_v_drag;

  SerialUART.println();
  SerialUART.println(F("================ AUTOCALIB REPORT ================"));
  SerialUART.print(F("phases done:"));
  for (uint8_t n = 1; n <= 6; n++) { SerialUART.print(' '); SerialUART.print(n);
    SerialUART.print(ac_done[n] ? '+' : '-'); }
  SerialUART.println();
  SerialUART.print(F("VERDICT: "));
  if (!complete)                SerialUART.println(F("INCOMPLETE -- phases missing, see above. Do NOT paste."));
  else if (worst == AC_FAIL)    SerialUART.println(F("FAIL -- at least one output is NOT determined. Do NOT paste."));
  else if (worst == AC_WARN)    SerialUART.println(F("PARTIAL -- usable, but read every WARN line first."));
  else                          SerialUART.println(F("PASS -- every output determined."));
  SerialUART.print(F("CFG Vbus=")); SerialUART.print(driver.voltage_power_supply, 2);
  SerialUART.print(F(" dead_zone=")); SerialUART.print(driver.dead_zone, 4);
  SerialUART.print(F(" v_align=")); SerialUART.print(motor.voltage_sensor_align, 2);
  SerialUART.print(F(" spi_nops=")); SerialUART.print(SPI_HALF_NOPS);
  SerialUART.print(F(" vbus_scale=")); SerialUART.print(VBUS_SCALE, 6);
  SerialUART.print(F(" mod=")); SerialUART.println(
      motor.foc_modulation == FOCModulationType::SpaceVectorPWM ? F("SVPWM") : F("SinePWM"));
  // F2 -- VBUS_SCALE is PER-BOARD and cannot be measured without an external
  // reference, but every voltage-derived constant scales with it:
  //     R_measured = R_true * (V_assumed / V_true)
  // and the same factor lands on U0 and Ke. This is the one input the routine
  // cannot self-check, so it is demanded explicitly rather than assumed.
  SerialUART.print(F("!! WRITE IN: multimeter Vbus = ______ V   (firmware read "));
  SerialUART.print(driver.voltage_power_supply, 2); SerialUART.println(F(")"));
  SerialUART.println(F("   >1% apart -> recalibrate VBUS_SCALE for THIS BOARD before"));
  SerialUART.println(F("   trusting R_EFF, U0 or KE. They all scale with it."));
  SerialUART.println();

  // ---- verdict table: read this BEFORE pasting anything -------------------
  SerialUART.println(F("---- MEASURED (read every verdict before you paste) ----"));
  acPrintVal(F("zea    "), ac_zea, 4, ac_done[2], ac_v_zea, F("rad elec, wrap-safe median"));
  SerialUART.print(F("  dir    "));
  if (ac_done[2]) SerialUART.print(ac_dir > 0 ? F("CW (+1)") : F("CCW (-1)"));
  else            SerialUART.print(F("--"));
  SerialUART.print(F("\t[")); SerialUART.print(ac_done[2] ? acVs(ac_v_zea) : F("MISSING"));
  SerialUART.println(F("]  inverts torque if wrong"));
  acPrintVal(F("R_eff  "), ac_R,  5, ac_done[3], ac_v_R,  F("ohm, slope of the self-locked sweep"));
  acPrintVal(F("U0     "), ac_U0, 5, ac_done[3], ac_v_U0, F("V, INTERCEPT -- the weak parameter of that fit"));
  acPrintVal(F("Ke     "), ac_Ke, 6, ac_done[5], ac_v_Ke, F("V/(rad/s), both directions in one fit"));
  acPrintVal(F("L (uH) "), ac_L*1e6f, 2, ac_done[4], ac_v_L, F("incremental at ~3 A"));
  if (ac_done[5]) {
    acPrintVal(F("drag_c_fwd "), ac_drag_c[0], 4, true, ac_v_drag, F("A"));
    acPrintVal(F("drag_c_rev "), ac_drag_c[1], 4, true, ac_v_drag, F("A"));
    acPrintVal(F("drag_v_fwd "), ac_drag_v[0], 6, true, ac_v_drag, F("A/(rad/s)"));
    acPrintVal(F("drag_v_rev "), ac_drag_v[1], 6, true, ac_v_drag, F("A/(rad/s)"));
    // Kt is DERIVED and deliberately absent from the row. Printed here only as
    // the one absolute cross-check the routine can make on the VOLTAGE scale.
    SerialUART.print(F("  Kt     ")); SerialUART.print(KT_PER_KE*ac_Ke, 6);
    SerialUART.print(F("\tDERIVED = 1.5*Ke, never stored. vs nameplate KV"));
    SerialUART.print(MOTOR_KV_NAMEPLATE, 0); SerialUART.print(F(" = "));
    SerialUART.print(60.0f/(_2PI*MOTOR_KV_NAMEPLATE), 6);
    SerialUART.print(F("  ("));
    SerialUART.print(100.0f*(KT_PER_KE*ac_Ke*_2PI*MOTOR_KV_NAMEPLATE/60.0f - 1.0f), 2);
    SerialUART.println(F("%) -- confirms the VOLTAGE scale only, see M2"));
  }
  SerialUART.println();

  // ---- the ONE pasteable artefact ----------------------------------------
  // A JointCal row in joint_cal.h's field order. This used to emit standalone
  // `const float R_EFF = ...` declarations, which have not matched the storage
  // schema since joint_cal.h existed: pasting them would not have compiled into
  // anything the firmware actually reads.
  SerialUART.println(F("---- PASTE THIS ROW INTO joint_cal.h (replace the whole row) ----"));
  SerialUART.println(F("// fill in: id (MUST match the board label), board_sn, motor_sn, date, belt"));
  SerialUART.print(F("  { \""));   SerialUART.print(CAL.id);
  SerialUART.print(F("\", \""));   SerialUART.print(CAL.board_sn);
  SerialUART.print(F("\", \""));   SerialUART.print(CAL.motor_sn);
  SerialUART.print(F("\", \"YYYY-MM-DD\", \"")); SerialUART.print(CAL.belt);
  SerialUART.println(F("\","));

  SerialUART.print(F("     "));
  if (ac_done[2]) { SerialUART.print(ac_zea, 4); SerialUART.print(F("f, "));
                    SerialUART.print(ac_dir > 0 ? F("+1") : F("-1")); }
  else            { SerialUART.print(F("-1.0f, 0")); }
  SerialUART.print(F(", ")); acRowF(ac_R, 5, ac_done[3], 3);
  SerialUART.println(F(",        // zea, dir, R_eff"));

  SerialUART.print(F("     ")); acRowF(ac_U0, 5, ac_done[3], 3);
  SerialUART.println(F(",                      // U0"));

  SerialUART.print(F("     ")); acRowF(ac_Ke, 6, ac_done[5], 5);
  SerialUART.print(F(", "));
  if (ac_done[4]) { SerialUART.print(ac_L*1e6f, 2); SerialUART.print(F("e-6f")); }
  else            { SerialUART.print(F("0.0f /* NOT MEASURED - run 4 */")); }
  SerialUART.println(F(",         // Ke, L"));

  // vbus_scale / i_scale / breakaway_A are CARRIED from the flashed row, never
  // measured here. Re-emitting them means a re-run cannot silently discard an
  // M1/M2/M4 result that cost a multimeter and twenty minutes to obtain.
  SerialUART.print(F("     ")); SerialUART.print(CAL.vbus_scale, 6);
  SerialUART.print(F("f, ")); SerialUART.print(CAL.i_scale, 4);
  SerialUART.print(F("f,      // vbus_scale, i_scale  CARRIED (M1 / M2)"));
  if (CAL.i_scale == 1.0f) SerialUART.print(F("  <-- PENDING M2"));
  SerialUART.println();

  SerialUART.print(F("     ")); acRowF(ac_drag_c[0], 4, ac_done[5], 5);
  SerialUART.print(F(", ")); acRowF(ac_drag_c[1], 4, ac_done[5], 5);
  SerialUART.println(F(",         // drag_c fwd, rev"));
  SerialUART.print(F("     ")); acRowF(ac_drag_v[0], 6, ac_done[5], 5);
  SerialUART.print(F(", ")); acRowF(ac_drag_v[1], 6, ac_done[5], 5);
  SerialUART.println(F(",     // drag_v fwd, rev"));

  SerialUART.print(F("     ")); SerialUART.print(CAL.breakaway_A, 4);
  SerialUART.print(F("f },                   // breakaway_A  CARRIED (M4)"));
  if (CAL.breakaway_A == 0.0f) SerialUART.print(F("  <-- PENDING M4"));
  SerialUART.println();
  SerialUART.println();

  SerialUART.println(F("---- DIAGNOSTICS (not per-unit constants) ----"));
  if (ac_done[1]) {
    SerialUART.print(F("SPI link       ")); SerialUART.print(ac_link_us, 2);
    SerialUART.print(F(" us/read, ")); SerialUART.print(ac_link_err);
    SerialUART.println(F(" parity errors in 4000"));
  }
  if (ac_done[6]) {
    float fl = ac_T_floop[AC_BIN_SPEEDS-1];
    SerialUART.print(F("T_delay        ")); SerialUART.print(ac_T[AC_BIN_SPEEDS-1]*1e6f, 1);
    SerialUART.print(F(" us   (2nd estimate ")); SerialUART.print(ac_T[0]*1e6f, 1);
    SerialUART.println(F(" us)"));
    SerialUART.print(F("  measured at f_loop=")); SerialUART.print(fl, 0);
    if (fl > 1.0f) {
      SerialUART.print(F(" -> T/T_loop=")); SerialUART.print(ac_T[AC_BIN_SPEEDS-1]*fl, 3);
      SerialUART.println(F("  (expect ~0.95-0.98: T is ONE loop period)"));
      SerialUART.println(F("  T is NOT per-unit. It scales with 1/f_loop, so the sketch's own"));
      SerialUART.println(F("  T = T/T_loop divided by the sketch's f_loop, not this number."));
    } else SerialUART.println();
    SerialUART.print(F("  torque loss at 270 rad/s = "));
    SerialUART.print(100.0f*(1.0f - cosf((float)MOTOR_POLE_PAIRS*270.0f*ac_T[AC_BIN_SPEEDS-1])), 3);
    SerialUART.println(F(" %"));
    SerialUART.print(F("INL pk-pk      ")); SerialUART.print(ac_inl_pp/(float)MOTOR_POLE_PAIRS, 3);
    SerialUART.println(F(" deg MECH -- fit 1/rev vs 2/rev offline from the bins below"));
    SerialUART.print(F("ZEA residual   ")); SerialUART.print(ac_zea_resid, 3);
    SerialUART.println(F(" deg elec -- independent check on the installed ZEA"));
  }
  if (ac_done[5]) {
    SerialUART.print(F("|I| ratio      ")); SerialUART.print(ac_ratio_lo, 3);
    SerialUART.print(F(" .. ")); SerialUART.print(ac_ratio_hi, 3);
    SerialUART.print(F("  (theory 1.2247)  [")); SerialUART.print(acVs(ac_v_ratio));
    SerialUART.println(F("]"));
    // Tagged with the row's belt field rather than hardcoded: this same block
    // is produced belt-ON at M5, and an untagged "belt OFF" label on belt-on
    // data is how a baseline gets overwritten by its own successor.
    SerialUART.print(F("DRAG MAP (belt ")); SerialUART.print(CAL.belt);
    SerialUART.println(F(")   U_cmd, vel, Iq"));
    for (uint8_t i = 0; i < ac_wn; i++) {
      SerialUART.print(F("  DRAG,")); SerialUART.print(ac_w_u[i], 2);
      SerialUART.print(','); SerialUART.print(ac_w_vel[i], 2);
      SerialUART.print(','); SerialUART.println(ac_w_iq[i], 4);
    }
  }

  // Gains: PRINTED, never applied. Silently mutating gains inside a calibration
  // routine is exactly the hidden-state pattern the failure catalogue is full of.
  SerialUART.println(F("CURRENT-LOOP GAINS"));
  SerialUART.print(F("  configured  P=")); SerialUART.print(CURQ_P, 4);
  SerialUART.print(F(" I=")); SerialUART.print(CURQ_I, 1);
  if (ac_done[3] && ac_done[4]) {
    SerialUART.print(F("   -> implies ")); SerialUART.print(CURQ_I/ac_R/_2PI, 0);
    SerialUART.print(F(" Hz from I, ")); SerialUART.print(CURQ_P/ac_L/_2PI, 0);
    SerialUART.print(F(" Hz from P"));
  }
  SerialUART.println();
  if (ac_done[3] && ac_done[4]) {
    float wbw = _2PI * AC_BW_HZ;
    SerialUART.print(F("  suggested   P=")); SerialUART.print(ac_L*wbw, 4);
    SerialUART.print(F(" I=")); SerialUART.print(ac_R*wbw, 1);
    SerialUART.print(F("   for ")); SerialUART.print(AC_BW_HZ, 0);
    SerialUART.println(F(" Hz.  NOT APPLIED."));
    // Kp = L*w_bw, Ki = R*w_bw is an IDEAL-PLANT formula: it has no transport
    // delay term, and this rig has a measured one of ~T_DELAY_PER_LOOP loop
    // periods. A well-damped loop caps at roughly 1/(3T), and the configured
    // gains were bench-tuned to 412 Hz at zeta = 0.60 -- already ~80% of that
    // ceiling. Raising Ki would eat phase margin the formula cannot see.
    // BENCH EVIDENCE OUTRANKS THE FORMULA. Treat this as a sanity check on
    // R and L, not as a recommendation.
    SerialUART.print(F("  delay ceiling ~1/(3T) = "));
    float fl = ac_done[6] ? ac_T_floop[AC_BIN_SPEEDS-1] : 0.0f;
    float Td = (fl > 1.0f) ? tDelayAt(fl) : 0.0f;
    if (Td > 0.0f) { SerialUART.print(1.0f/(3.0f*Td), 0); SerialUART.print(F(" Hz")); }
    else           { SerialUART.print(F("(run 6 for f_loop)")); }
    SerialUART.println(F("  -- the formula above ignores it. Do not apply blind."));
  }

  if (ac_done[6]) {
    SerialUART.println(F("---- BINS: bin,fwd_deg,rev_deg,even,odd  (offline harmonic fit) ----"));
    uint8_t s = AC_BIN_SPEEDS - 1;
    for (uint8_t b = 0; b < AC_BINS; b++) {
      if (ac_bin_n[0][s][b] < AC_BIN_MIN_N || ac_bin_n[1][s][b] < AC_BIN_MIN_N) continue;
      float A = acBinAngleDeg(0, s, b), B = acBinAngleDeg(1, s, b);
      SerialUART.print(F("BIN,")); SerialUART.print(b);
      SerialUART.print(','); SerialUART.print(A, 3);
      SerialUART.print(','); SerialUART.print(B, 3);
      SerialUART.print(','); SerialUART.print(0.5f*(A+B), 3);
      SerialUART.print(','); SerialUART.println(0.5f*(A-B), 3);
    }
  }
  if (ac_done[4]) {
    SerialUART.println(F("---- L BINS: t_us_centre,I_mean,n,frac  (offline refit) ----"));
    float span = ac_l_Iinf - ac_l_I0;
    for (uint8_t b = 0; b < AC_L_TBINS; b++) {
      if (ac_lb_n[b] == 0) continue;
      float I = ac_lb_i[b] / ac_lb_n[b];
      SerialUART.print(F("LSB,")); SerialUART.print((b + 0.5f) * AC_L_TBIN_US, 1);
      SerialUART.print(','); SerialUART.print(I, 4);
      SerialUART.print(','); SerialUART.print(ac_lb_n[b]);
      SerialUART.print(','); SerialUART.println((span > 1e-6f) ? ((I - ac_l_I0)/span) : 0.0f, 4);
    }
    SerialUART.print(F("LSB_I0,")); SerialUART.print(ac_l_I0, 4);
    SerialUART.print(F(",LSB_Iinf,")); SerialUART.print(ac_l_Iinf, 4);
    SerialUART.print(F(",tail_n,")); SerialUART.println((uint32_t)ac_l_tail_n);
  }
  SerialUART.println(F("=================================================="));
}

// ===========================================================================
// S2 -- ZEA VERIFICATION.  'V'
//
// Storing ZEA removed the only thing that used to re-derive it every power-up.
// That is the benefit, and this is the price: a wrong-joint flash or a slipped
// magnet mount is now SILENT, because nothing recomputes the alignment.
// This does ONE forced alignment, compares it to the value the firmware is
// carrying, then puts the stored value back. It catches both failures.
// Wrap-safe: the comparison folds through 2*pi, so 6.28 vs 0.01 reads as 0.02,
// not 6.27.
// ===========================================================================
void acVerifyZea() {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  if (!driver_ok || !cs_linked) { SerialUART.println(F("refused: driver or CS not ready")); return; }
  if (ZEA_STORED < 0.0f || DIR_STORED == 0) {
    SerialUART.println(F("V: no stored ZEA to verify (ZEA_STORED < 0). Nothing to check."));
    return;
  }
  SerialUART.println(F("[V] ZEA VERIFY  (one twitch; stored value is restored after)"));
  acEnter();
  motor.zero_electric_angle = NOT_SET;
  motor.sensor_direction    = Direction::UNKNOWN;
  motor.enable();
  int ok = motor.initFOC();
  motor.disable();
  if (!ok) { SerialUART.println(F("    initFOC FAILED")); acExit(false); return; }
  float meas = motor.zero_electric_angle;
  int   mdir = (motor.sensor_direction == Direction::CW) ? +1 : -1;
  // shortest signed distance around the circle
  float d = meas - ZEA_STORED;
  while (d >  _PI) d -= _2PI;
  while (d < -_PI) d += _2PI;
  float ddeg = degrees(fabsf(d));

  SerialUART.print(F("    measured ZEA=")); SerialUART.print(meas, 4);
  SerialUART.print(F(" vs stored ")); SerialUART.print(ZEA_STORED, 4);
  SerialUART.print(F("  -> ")); SerialUART.print(ddeg, 2); SerialUART.print(F(" deg elec"));
  SerialUART.print(F(" | dir ")); SerialUART.print(mdir > 0 ? F("CW") : F("CCW"));
  SerialUART.print(F(" vs ")); SerialUART.print(DIR_STORED > 0 ? F("CW") : F("CCW"));
  SerialUART.println();
  if (mdir != DIR_STORED) {
    SerialUART.println(F("    *** FAIL: DIRECTION MISMATCH. Wrong joint's constants, or the"));
    SerialUART.println(F("    *** magnet mount has been rebuilt. Do NOT run this firmware."));
  } else if (ddeg > 15.0f) {
    SerialUART.println(F("    *** FAIL: >15 deg. Almost certainly the WRONG JOINT's constants,"));
    SerialUART.println(F("    *** or the magnet has slipped on the shaft. Re-run autocalib."));
  } else if (ddeg > 8.0f) {
    SerialUART.println(F("    !! WARN: >8 deg, well outside alignment scatter (sd ~3.4 deg)."));
    SerialUART.println(F("    !! Check the magnet mount before trusting torque numbers."));
  } else {
    SerialUART.println(F("    OK: within alignment scatter. Stored ZEA is good."));
  }
  // Put the stored values back -- initFOC just overwrote them.
  motor.zero_electric_angle = ZEA_STORED;
  motor.sensor_direction    = (DIR_STORED > 0) ? Direction::CW : Direction::CCW;
  motor.enable(); (void)motor.initFOC(); motor.disable();
  foc_ready = true;
  acExit(true);
  SerialUART.println(F("    stored ZEA reinstalled."));
}

// ===========================================================================
// M2 ASSIST -- 'N'.  ABSOLUTE CURRENT-SENSE SCALE, bus-power ladder.
//
// WHAT IT DOES NOT DO: it does not compute anything. It holds the rotor still
// at five known voltages, long enough for an EXTERNAL meter to settle, and
// prints the current the firmware THINKS is flowing at each. You supply the
// truth from the bus side. That asymmetry is the whole point -- no internal
// check can see a current-sense gain error, because every internal cross-check
// divides one wrongly-scaled current by another and gets the right answer.
//
// WHY THE BUS AND NOT A PHASE LEAD: at a locked rotor the phase currents are
// DC, but how they split between the three phases depends on where the rotor
// happened to stop, which is unknown. Bus power has no such ambiguity: nothing
// moves, so every watt in comes out as heat, and the firmware claims that heat
// is 1.5*I^2*R_eff. Compare its claim to the wall.
//
// WHY FIVE POINTS AND NOT TWO -- this SUPERSEDES the two-point difference:
// differencing removes the CONSTANT losses (MCU, gate drive, LEDs) but NOT the
// terms proportional to I. Switching loss (~0.15 W at 3.1 A) and dead-time
// body-diode conduction (~1.5*U0*I ~ 0.05 W) both survive it, and together they
// are ~5% of the differenced signal -- the same size as the gain error being
// hunted. A two-point difference would report g ~ 0.95 on a PERFECTLY
// calibrated board. Five points and a quadratic separate them:
//
//     P_bus = a + b*I + c*I^2      a = housekeeping, b = switching + dead time
//     g     = 1.5 * R_eff / c      (fit OFFLINE -- see README section 20.1)
//
// DOWNWARD ladder, same reason as phase 3: the lock is established at the
// strongest hold first, so cogging never gets to win. Thermal: 3.2 W at the top
// point, hence the 20 s hard cap per point and the instruction to bracket the
// whole run with phase 3 so R_eff drift is bounded rather than assumed away.
// ===========================================================================
const uint8_t  AC_M2_N          = 5;
const float    AC_M2_V[AC_M2_N] = { 0.70f, 0.55f, 0.40f, 0.25f, 0.16f };
const uint16_t AC_M2_MAX_MS     = 20000;   // hard cap per point -- thermal
const uint16_t AC_M2_TICK_MS    = 2000;    // running-mean heartbeat

static void acM2Assist() {
  if (!acReady()) return;
  // The self-lock itself does not depend on the alignment -- velocityOpenloop()
  // applies voltage at a fixed ELECTRICAL angle and the rotor pulls into it.
  // Phase 2 is required anyway so that I_reported here is directly comparable
  // to the phase-3 sweep, which is the number the fit is normalised against.
  if (!acNeed(2)) return;

  SerialUART.println(F("[M2] BUS-POWER LADDER.  Rotor self-locks -- HANDS OFF THE SHAFT."));
  SerialUART.println(F("     At each point: let the meter settle (~10 s), write down Vbus and"));
  SerialUART.println(F("     Ibus next to the I_reported line, then press any key for the next."));
  SerialUART.println(F("     Press 3 immediately BEFORE and AFTER this run -- cold and hot R_eff"));
  SerialUART.println(F("     bracket the thermal drift instead of leaving it in the fit."));
  acEnter();
  mode = MODE_OPENLOOP;
  motor.controller = MotionControlType::velocity_openloop;
  target = 0.0f;                      // zero openloop velocity = FIXED field angle

  for (uint8_t k = 0; k < AC_M2_N && !ac_abort; k++) {
    motor.voltage_limit = AC_M2_V[k];
    motor.enable(); running = true; run_started = millis();
    // Mirrors phase 3 exactly, so I_reported here is directly comparable to a
    // phase-3 point: 600 ms initial swing-in, then the same per-point settle.
    // The settle also has to happen BEFORE c0 is latched, or the drift column
    // counts the settling transient of the voltage change as creep and stops
    // meaning what its label claims.
    if (k == 0) acRun(600);
    acRun(AC_R_SETTLE_MS);
    uint16_t c0 = encoder.raw;

    SerialUART.print(F("  point ")); SerialUART.print(k + 1);
    SerialUART.print('/');           SerialUART.print(AC_M2_N);
    SerialUART.print(F("   Uq=")); SerialUART.println(AC_M2_V[k], 3);

    uint32_t t0 = millis(), tp = millis();
    double acc = 0; uint32_t n = 0;
    while ((millis() - t0) < AC_M2_MAX_MS && !ac_abort) {
      acService();
      // A key here means ADVANCE, not abort -- but ONLY if no guard also fired
      // in the same iteration. ac_guard is never cleared, so an overcurrent or
      // overspeed trip that happens to coincide with a keypress still aborts.
      if (ac_key) {
        ac_key = false;
        if (!ac_guard) ac_abort = false;      // clean advance
        break;
      }
      acc += ac_i_amp; n++;
      if ((millis() - tp) > AC_M2_TICK_MS) {
        tp = millis();
        // Safe to print inside the loop ONLY because the rotor is locked: the
        // print-block commutation freeze needs a MOVING rotor to do damage.
        SerialUART.print(F("      I_reported = "));
        SerialUART.print(n ? (float)(acc / n) : 0.0f, 4);
        SerialUART.println(F(" A  (running mean)"));
      }
    }
    int32_t moved = acCntDelta(c0, encoder.raw);
    // Same split as M4: a clean CSV line, then a sentence.
    //     M2,<Uq>,<I_reported>,<n>,<drift_cnt>
    SerialUART.print(F("M2,")); SerialUART.print(AC_M2_V[k], 3);
    SerialUART.print(',');      SerialUART.print(n ? (float)(acc / n) : 0.0f, 4);
    SerialUART.print(',');      SerialUART.print(n);
    SerialUART.print(',');      SerialUART.println(moved);
    SerialUART.print(F("    Uq=")); SerialUART.print(AC_M2_V[k], 3);
    SerialUART.print(F(" I_reported=")); SerialUART.print(n ? (float)(acc / n) : 0.0f, 4);
    SerialUART.print(F(" A, drift ")); SerialUART.print(moved);
    SerialUART.println(F(" cnt   <- write Vbus and Ibus against THIS line"));
    if (acAbs32(moved) > AC_R_STILL_CNT)
      SerialUART.println(F("    !! rotor MOVED -- this point is invalid, redo it"));
  }
  acExit(true);
  if (ac_abort) SerialUART.println(F("[M2] ABORTED by a guard, not by you -- discard the last point."));
  SerialUART.println(F("[M2] done. Press 3 NOW for hot R_eff, then fit P_bus = a + b*I + c*I^2"));
  SerialUART.println(F("     offline and take g = 1.5*R_mean/c. See README section 20.1."));
}

// ===========================================================================
// M4 ASSIST -- 'B' forward / 'b' reverse.  BREAKAWAY (STATIC friction).
//
// The current at which a STATIONARY rotor first moves. This is NOT phase 5's
// drag_c: that is friction while ALREADY MOVING (0.075 A on J01). Breakaway is
// the threshold to GET moving, it is always higher, and it varies around the
// revolution because cogging adds and subtracts. It is the term that decides
// whether impedance control feels alive or dead near zero commanded force --
// belt-on it was 46% of standing leg load on A1, five times every other loss
// combined.
//
// Nothing turns at the start. The rotor is still; the current rises until it
// is not; 0.5 rad/s is only the threshold that counts as "it moved". (An
// earlier description said "ramp Iq through zero at +-0.5 rad/s", which read as
// though something should already be turning. It should not.)
//
// THE RAMP RATE IS THE MEASUREMENT. 0.005 A / 150 ms = 0.033 A/s, so breakaway
// at 0.15 A takes ~4.5 s. Too fast and you measure the ramp, not the friction,
// which is why this is firmware-timed rather than typed by hand -- the elapsed
// time is reported so that failure is visible instead of assumed away.
// ===========================================================================
const float    AC_M4_STEP_A    = 0.005f;   // A per step
const uint16_t AC_M4_DWELL_MS  = 150;      // -> 0.033 A/s
const float    AC_M4_MOVE_RADS = 0.5f;     // "it moved"
const float    AC_M4_ABORT_A   = 0.60f;    // give up: something is rubbing

static void acM4Breakaway(float sgn) {
  if (!acReady()) return;
  // FOC current, so the commutation angle must be right or the number is
  // meaningless. Accepts EITHER path to a valid alignment: a stored ZEA
  // installed by 'f'/'V', or a fresh phase 2. acNeed(2) would wrongly refuse
  // the first and acNeed(1) would wrongly allow neither.
  if (!foc_ready) {
    SerialUART.println(F("refused: no valid alignment. Press 'f' or 'V' (stored ZEA), or run"));
    SerialUART.println(F("         phase 2. This ramp is FOC current -- a wrong ZEA invalidates it."));
    return;
  }
  SerialUART.print(F("[M4] BREAKAWAY ramp, direction "));
  SerialUART.print(sgn > 0 ? F("+") : F("-"));
  SerialUART.print(F("   pulley bare, LEG NOT ATTACHED.  Row says belt="));
  SerialUART.print(CAL.belt);
  SerialUART.print(F(" -- the reading describes whatever is ACTUALLY fitted, so"));
  SerialUART.println(F(" tag it. start raw="));
  SerialUART.print(F("     ")); SerialUART.println(encoder.raw);
  acEnter();
  mode = MODE_TORQUE_CURRENT;                       // 'c' mode -- FOC current
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller        = MotionControlType::torque;
  motor.PID_current_q.reset(); motor.PID_current_d.reset();
  target = 0.0f;
  uint16_t c0 = encoder.raw;
  motor.enable(); running = true; run_started = millis();

  uint32_t tramp = millis();
  float i = 0.0f; bool moved = false;
  while (i < AC_M4_ABORT_A && !ac_abort) {
    i += AC_M4_STEP_A;
    target = sgn * i;
    uint32_t t0 = millis();
    while ((millis() - t0) < AC_M4_DWELL_MS && !ac_abort) {
      acService();
      // shaft_velocity is the MEASURED value here (unlike openloop, where
      // velocityOpenloop() overwrites it with the command), so this is real.
      if (fabsf(motor.shaft_velocity) > AC_M4_MOVE_RADS) { moved = true; break; }
    }
    if (moved) break;
  }
  uint32_t el = millis() - tramp;
  target = 0.0f;
  int32_t travel = acCntDelta(c0, encoder.raw);
  acExit(true);

  if (ac_abort) { SerialUART.println(F("    aborted -- discard")); return; }
  if (!moved) {
    SerialUART.print(F("    NO MOTION up to ")); SerialUART.print(AC_M4_ABORT_A, 3);
    SerialUART.println(F(" A -- something is rubbing. Check bearing preload and"));
    SerialUART.println(F("    that the magnet is not skimming the sensor (gap 0.5-1.0 mm, NEVER zero)."));
    return;
  }
  // Machine-readable, alone on its line and fully comma-delimited, so a whole
  // session pastes straight into docs/cal/*.csv:
  //     M4,<dir>,<amps>,<raw>,<ramp_s>,<travel_cnt>
  SerialUART.print(F("M4,"));  SerialUART.print(sgn > 0 ? '+' : '-');
  SerialUART.print(',');       SerialUART.print(i, 4);
  SerialUART.print(',');       SerialUART.print(c0);
  SerialUART.print(',');       SerialUART.print(el/1000.0f, 2);
  SerialUART.print(',');       SerialUART.println(travel);
  // Human-readable, separately. Mixing the two put "A at raw=" AFTER the count
  // and printed the rotor position twice.
  SerialUART.print(F("    breakaway ")); SerialUART.print(i, 4);
  SerialUART.print(F(" A at raw=")); SerialUART.print(c0);
  SerialUART.print(F("   ramp ")); SerialUART.print(el/1000.0f, 1);
  SerialUART.print(F(" s, travel ")); SerialUART.print(travel);
  SerialUART.println(F(" cnt"));
  if (el < 1000)  SerialUART.println(F("    !! broke away in under 1 s -- you are measuring the RAMP. Halve AC_M4_STEP_A."));
  if (i > 0.40f)  SerialUART.println(F("    !! > 0.4 A belt-off is a FAULT, not a baseline. A1's 0.34-1.34 A was belt-ON."));
  SerialUART.println(F("    Rotate the shaft ~40 deg by hand and repeat -- 5 positions per direction."));
}

// ===========================================================================
// STATUS + DISPATCH
// ===========================================================================
static void acStatus() {
  SerialUART.println(F("--- AUTOCALIB.  0=reset 1..6=phases 7=report ---"));
  const __FlashStringHelper* nm[7] = { F(""), F("1 LINK   zero current"),
    F("2 ALIGN  ZEA + direction"), F("3 R/U0   self-locked, still"),
    F("4 L      step train, locked"), F("5 SPIN   free-spin both dirs (~50 s)"),
    F("6 T/INL  computation only") };
  const uint8_t pre[7] = { 0, 0, 1, 2, 3, 3, 5 };
  for (uint8_t n = 1; n <= 6; n++) {
    SerialUART.print(ac_done[n] ? F("  [x] ") : F("  [ ] "));
    SerialUART.print(nm[n]);
    if (pre[n] && !ac_done[pre[n]]) { SerialUART.print(F("   (needs ")); SerialUART.print(pre[n]); SerialUART.print(')'); }
    if (n == 6 && !ac_done[4]) SerialUART.print(F(" (also needs 4)"));
    SerialUART.println();
  }
  SerialUART.println(F("  Any key aborts a running phase."));
  SerialUART.println(F("  FREE SHAFT + BELT OFF is needed by 2, 5 and 6 only."));
  SerialUART.println(F("  1, 3, 4 are LOCKED-ROTOR and belt-agnostic. Leg links off throughout."));
  SerialUART.println(F("  --- manual-assist, NOT part of the 1..7 chain ---"));
  SerialUART.println(F("  N = M2 bus-power ladder. BELT-AGNOSTIC (locked rotor). Needs 2 and an"));
  SerialUART.println(F("      external METER; bracket it with 3. Rotor must not creep -- watch drift."));
  SerialUART.println(F("  B / b = M4 breakaway ramp, + / - . Needs a valid alignment. Run it"));
  SerialUART.println(F("      BELT-OFF for the baseline and again BELT-ON at M5; tag which."));
  SerialUART.print(F("  stored: ZEA=")); SerialUART.print(ZEA_STORED, 4);
  SerialUART.print(F(" DIR=")); SerialUART.print(DIR_STORED);
  SerialUART.println(F("   ('V' verifies them against a fresh alignment)"));
}

void acPhase(uint8_t n) {
  if (!acReady()) return;
  switch (n) {
    case 0:
      for (uint8_t i = 0; i < 8; i++) ac_done[i] = false;
      SerialUART.println(F("AUTOCALIB state reset (installed ZEA is NOT reverted -- power-cycle for that)"));
      break;
    case 1: acP1(); break;
    case 2: acP2(); break;
    case 3: acP3(); break;
    case 4: acP4(); break;
    case 5: acP5(); break;
    case 6: acP6(); break;
    case 7: acP7(); break;
    default: acStatus(); break;
  }
}