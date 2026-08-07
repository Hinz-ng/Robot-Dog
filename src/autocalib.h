// ============================================================================
// autocalib.h  --  ONE-KEY PER-JOINT CHARACTERISATION  (belt OFF)
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
//
// ('A' is already logStats, so 'Y' is used for status.)
//
// ONE PHASE PER KEYPRESS, ON PURPOSE. You inspect each result before the next
// phase runs, you can re-run any single phase, and a failure never leaves the
// motor energised or the globals half-mutated. Each phase saves and restores
// everything it touches, and refuses to run if its prerequisites have not
// PASSED -- so out-of-order presses cannot silently produce garbage.
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
//   MANDATORY per joint : ZEA_STORED, DIR_STORED
//   PER-UNIT            : R_EFF, U0_V, KE, KT, drag map
//   DIAGNOSTIC          : T_delay, INL profile, |I| ratio, SPI link health
//   SUGGESTION ONLY     : current-loop gains (printed, NEVER applied)
//
// EXCLUDED ON PURPOSE -- each would be WORSE automated than done by hand:
//   J_rotor       geometric, ~1% between units, and needs friction subtraction
//   breakaway     needs slow +-0.5 rad/s ramps through zero; different regime
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
// RUNTIME per phase: 1 ~0.3 s | 2 ~8 s | 3 ~6 s | 4 ~1 s | 5 ~50 s | 6,7 instant.
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

const uint8_t  AC_L_REPS      = 24;       // averaged step repeats
const uint8_t  AC_L_NS        = 40;       // samples per repeat
const float    AC_L_VBASE     = 0.20f;    // hold between steps (~0.8 A)
const float    AC_L_VSTEP     = 0.70f;    // stepped-to (~3.4 A peak)
const uint16_t AC_L_DECAY_MS  = 4;        // >= 9 tau_e at tau_e ~ 330 us
const uint8_t  AC_L_MIN_PTS   = 5;
const float    AC_L_RMS_FAIL  = 0.20f;

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
const float    AC_RATIO_LO    = 1.20f;
const float    AC_RATIO_HI    = 1.25f;
const float    AC_BW_HZ       = 400.0f;   // for the SUGGESTED gains only

// ---------------------------------------------------------------------------
// Verdicts. Every output carries its own, and the run carries the worst.
// ---------------------------------------------------------------------------
// Each output carries its OWN verdict. There is deliberately no single running
// "worst" flag: phase 7 recomputes it from the outputs that actually ran, so a
// stale flag from an earlier attempt cannot leak into a later report.
enum AcV { AC_PASS = 0, AC_WARN = 1, AC_FAIL = 2 };
static bool ac_abort;
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
static float ac_lt[AC_L_NS], ac_li[AC_L_NS];          // L step: mean t_us, mean I
static float ac_l_I0, ac_l_Iinf;
static uint8_t ac_l_pts;
static float ac_wx[2*AC_W_N], ac_wy[2*AC_W_N];        // Ke fit: vel, U - R*Iq - U0
static float ac_w_vel[2*AC_W_N], ac_w_iq[2*AC_W_N], ac_w_u[2*AC_W_N];
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
static float ac_Ke, ac_Kt, ac_Ke_se, ac_Ke_rms, ac_Ke_c;
static float ac_T[AC_BIN_SPEEDS], ac_inl_pp, ac_zea_resid;
static float ac_ratio_lo, ac_ratio_hi;
static float ac_link_us; static uint32_t ac_link_err;
static AcV   ac_v_link, ac_v_zea, ac_v_R, ac_v_U0, ac_v_L, ac_v_Ke, ac_v_ratio, ac_v_T;

// ---------------------------------------------------------------------------
// Saved globals, restored by acExit()
// ---------------------------------------------------------------------------
static Mode      acs_mode;   static float acs_target, acs_vlim, acs_pql, acs_pdl;
static float     acs_zea;    static Direction acs_dir;   static bool acs_foc;

static void acEnter() {
  ac_abort  = false;
  acs_mode  = mode;   acs_target = target;
  acs_vlim  = motor.voltage_limit;
  acs_pql   = motor.PID_current_q.limit;
  acs_pdl   = motor.PID_current_d.limit;
  acs_zea   = motor.zero_electric_angle;
  acs_dir   = motor.sensor_direction;
  acs_foc   = foc_ready;
}

// keep_align = true only for phase 2 on success, which deliberately leaves the
// freshly measured ZEA in place so phases 3-6 commutate with it.
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
static float ac_i_amp = 0.0f;          // phase-current AMPLITUDE, live in all modes

static inline float acPhaseAmp() {
  PhaseCurrent_s c = currentSense.getPhaseCurrents();
  // sqrt(ia^2+ib^2+ic^2) = 1.2247 * amplitude in the amplitude-invariant frame
  return sqrtf(c.a*c.a + c.b*c.b + c.c*c.c) * 0.816497f;
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

  ac_i_amp = acPhaseAmp();
  if (ac_i_amp > AC_IMAX_ABORT) {
    ac_abort = true;
    SerialUART.print(F("\n  !! ABORT overcurrent: ")); SerialUART.print(ac_i_amp, 2);
    SerialUART.println(F(" A amplitude"));
  }
  // motor.shaft_velocity is the MEASURED value in closed-loop modes; in openloop
  // velocityOpenloop() overwrites it with the COMMANDED value (0), so this check
  // is meaningful only outside openloop -- which is exactly where the shaft can
  // actually run away.
  if (mode != MODE_OPENLOOP && fabsf(motor.shaft_velocity) > OVERSPEED_RADS) {
    ac_abort = true;
    SerialUART.println(F("\n  !! ABORT overspeed"));
  }
  if (SerialUART.available()) {
    while (SerialUART.available()) (void)SerialUART.read();
    ac_abort = true;
    SerialUART.println(F("\n  !! ABORT key pressed"));
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

// Signed count difference with the 16384 wrap handled.
static int32_t acCntDelta(uint16_t a, uint16_t b) {
  int32_t d = (int32_t)b - (int32_t)a;
  if (d >  8192) d -= 16384;
  if (d < -8192) d += 16384;
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
    if (n < 100) continue;
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
  for (uint8_t k = 0; k < AC_L_NS; k++) { ac_lt[k] = 0.0f; ac_li[k] = 0.0f; }
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
      ac_lt[k] += (float)(micros() - t0);
      ac_li[k] += ac_i_amp;
    }
    reps++;
  }
  int32_t moved = acCntDelta(c0, encoder.raw);
  acExit(true);
  if (ac_abort || reps < 4) { SerialUART.println(F("    aborted   FAIL")); ac_v_L = AC_FAIL; ac_done[4] = false; return; }

  float N = (float)reps;
  for (uint8_t k = 0; k < AC_L_NS; k++) { ac_lt[k] /= N; ac_li[k] /= N; }
  ac_l_I0 = (float)(i0acc / i0n);
  double tail = 0; for (uint8_t k = AC_L_NS - 6; k < AC_L_NS; k++) tail += ac_li[k];
  ac_l_Iinf = (float)(tail / 6.0);

  static float lx[AC_L_NS], ly[AC_L_NS];
  uint8_t n = 0;
  float span = ac_l_Iinf - ac_l_I0;
  if (span > 0.30f) {
    for (uint8_t k = 0; k < AC_L_NS; k++) {
      float frac = (ac_li[k] - ac_l_I0) / span;
      if (frac > 0.15f && frac < 0.85f) {
        lx[n] = ac_lt[k] * 1e-6f;
        ly[n] = logf(1.0f - frac);               // = -(t - t_d)/tau
        n++;
      }
    }
  }
  ac_l_pts = n;
  AcFit f = acFit(lx, ly, n);
  ac_tau   = (f.ok && f.m < -1e-6f) ? (-1.0f / f.m) : 0.0f;
  ac_L     = ac_tau * ac_R;
  ac_L_rms = f.rms;
  ac_L_td  = (ac_tau > 0.0f) ? (f.c * ac_tau) : 0.0f;

  ac_v_L = AC_PASS;
  if (n < AC_L_MIN_PTS || !f.ok || ac_tau <= 0.0f) ac_v_L = AC_FAIL;
  else if (ac_L < 15e-6f || ac_L > 400e-6f)        ac_v_L = AC_FAIL;
  else if (ac_L_rms > AC_L_RMS_FAIL)               ac_v_L = AC_FAIL;
  else if (acAbs32(moved) > 40)             ac_v_L = AC_WARN;
  else if (ac_L_td < 0.0f || ac_L_td > 400e-6f)    ac_v_L = AC_WARN;

  SerialUART.print(F("    reps=")); SerialUART.print(reps);
  SerialUART.print(F(" fitpts=")); SerialUART.print(ac_l_pts);
  SerialUART.print(F(" I0=")); SerialUART.print(ac_l_I0, 3);
  SerialUART.print(F(" Iinf=")); SerialUART.print(ac_l_Iinf, 3);
  SerialUART.print(F(" tau=")); SerialUART.print(ac_tau*1e6f, 1); SerialUART.print(F("us"));
  SerialUART.print(F(" L=")); SerialUART.print(ac_L*1e6f, 1); SerialUART.print(F("uH"));
  SerialUART.print(F(" rms=")); SerialUART.print(ac_L_rms, 3);
  SerialUART.print(F(" td=")); SerialUART.print(ac_L_td*1e6f, 0); SerialUART.print(F("us"));
  SerialUART.print(F(" drift=")); SerialUART.print(moved);
  SerialUART.print(F("   ")); SerialUART.println(acVs(ac_v_L));
  if (ac_v_L == AC_WARN) SerialUART.println(F("    !! rotor drifted or t_d implausible -- L is suspect, refit the trace offline."));
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
      double aIq = 0, aId = 0, aV = 0, aR = 0; uint32_t n = 0, nr = 0;
      uint32_t t0 = millis();
      while ((millis() - t0) < AC_W_MEAS_MS && !ac_abort) {
        acService();
        float iq = motor.current.q, id = motor.current.d;
        aIq += iq; aId += id; aV += motor.shaft_velocity; n++;
        // |I|/Iq integrity gate, per sample. ac_i_amp is already the amplitude,
        // so compare it against sqrt(Iq^2+Id^2) directly, then scale to the
        // familiar 1.2247 figure for the report.
        float mag = sqrtf(iq*iq + id*id);
        if (mag > 0.15f) { aR += (ac_i_amp / mag) * 1.2247f; nr++; }
        if (slot >= 0) {
          uint8_t b = (uint8_t)(((uint32_t)encoder.raw * AC_BINS) >> 14);   // /16384
          if (b < AC_BINS) {
            ac_bin_id[d][slot][b] += id;
            ac_bin_iq[d][slot][b] += iq;
            ac_bin_n [d][slot][b]++;
          }
        }
      }
      if (n < 500) continue;
      float mIq = (float)(aIq/n), mId = (float)(aId/n), mV = (float)(aV/n);
      float mR  = (nr > 100) ? (float)(aR/nr) : 0.0f;
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
  // Amplitude-invariant dq: Ke = pp*lambda and Kt = 1.5*pp*lambda, so Kt = 1.5*Ke.
  // The master table's 0.0266/0.0177 = 1.503 is the bench confirmation of this.
  ac_Kt = 1.5f * ac_Ke;

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
  SerialUART.print(F(" (theory 1.2247)   ")); SerialUART.println(acVs(ac_v_ratio));
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
// 57.0 us with a 3.6 us excess, plus the INL profile for free. The two binned
// speeds give two independent T estimates, and T must be speed-INDEPENDENT --
// that consistency is what makes it a measurement rather than a reading.
// ===========================================================================
static float acBinAngleDeg(uint8_t d, uint8_t s, uint8_t b) {
  if (ac_bin_n[d][s][b] == 0) return 0.0f;
  float Id = ac_bin_id[d][s][b] / ac_bin_n[d][s][b];
  float Iq = ac_bin_iq[d][s][b] / ac_bin_n[d][s][b];
  float w  = ac_bin_w[d][s];
  if (fabsf(w) < 1.0f) return 0.0f;
  float we = 7.0f * w;
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
    float w   = 0.5f*(fabsf(ac_bin_w[0][s]) + fabsf(ac_bin_w[1][s]));
    if (w < 1.0f) continue;
    ac_T[s] = radians(om) / (7.0f * w);
    any = true;
    SerialUART.print(F("    w=+-")); SerialUART.print(w, 1);
    SerialUART.print(F(" bins=")); SerialUART.print(on);
    SerialUART.print(F(" odd=")); SerialUART.print(om, 3);
    SerialUART.print(F("+-")); SerialUART.print(osd, 3);
    SerialUART.print(F(" deg -> T=")); SerialUART.print(ac_T[s]*1e6f, 1);
    SerialUART.println(F(" us"));
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
  SerialUART.print(F(" deg elec = ")); SerialUART.print(ac_inl_pp/7.0f, 3);
  SerialUART.print(F(" deg MECH | ZEA residual ")); SerialUART.print(ac_zea_resid, 3);
  SerialUART.print(F(" deg elec   ")); SerialUART.println(acVs(ac_v_T));
  if (ac_v_T == AC_FAIL)
    SerialUART.println(F("    !! T inconsistent across speeds -> the fixed-delay model does NOT hold here."));
  if (ac_inl_pp/7.0f > 1.5f)
    SerialUART.println(F("    !! INL above the 1.5 deg datasheet max -- check magnet CENTRING (1/rev term)."));
  if (fabsf(ac_zea_resid) > 3.0f)
    SerialUART.println(F("    !! ZEA residual large -- the installed ZEA is off by more than 3 deg elec."));
  ac_done[6] = (ac_v_T != AC_FAIL);
  if (ac_done[6]) SerialUART.println(F("    next: 7 (report)"));
}

// ===========================================================================
// PHASE 7 -- REPORT. Nothing that has not been MEASURED is emitted as a number.
// ===========================================================================
// Emits either a valid initialiser or an explicit hole -- never a number that
// was not measured, and never something that would silently compile to garbage
// if pasted.
static void acPrintVal(const __FlashStringHelper* decl, float v, uint8_t dp,
                       bool measured, AcV verdict, const __FlashStringHelper* note) {
  SerialUART.print(decl);
  if (measured) { SerialUART.print(v, dp); SerialUART.print(F("f;")); }
  else          { SerialUART.print(F("0.0f; /* NOT MEASURED */")); }
  SerialUART.print(F("  // ")); SerialUART.print(note);
  SerialUART.print(F("  [")); SerialUART.print(measured ? acVs(verdict) : F("MISSING"));
  SerialUART.println(F("]"));
}

static void acP7() {
  AcV worst = AC_PASS;
  bool complete = true;
  for (uint8_t n = 1; n <= 6; n++) if (!ac_done[n]) complete = false;
  AcV vs[8] = { AC_PASS, ac_v_link, ac_v_zea, ac_v_R, ac_v_L, ac_v_Ke, ac_v_T, AC_PASS };
  for (uint8_t n = 1; n <= 6; n++) if (ac_done[n] && vs[n] > worst) worst = vs[n];
  if (ac_done[3] && ac_v_U0    > worst) worst = ac_v_U0;
  if (ac_done[5] && ac_v_ratio > worst) worst = ac_v_ratio;

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
  SerialUART.print(F(" mod=")); SerialUART.println(
      motor.foc_modulation == FOCModulationType::SpaceVectorPWM ? F("SVPWM") : F("SinePWM"));
  SerialUART.println();

  SerialUART.println(F("// ==== JOINT CALIBRATION ====  fill in: joint id / belt=OFF / date"));
  if (ac_done[2]) {
    SerialUART.print(F("const float ZEA_STORED = ")); SerialUART.print(ac_zea, 4);
    SerialUART.print(F("f;   // sd ")); SerialUART.print(degrees(ac_zea_sd), 2);
    SerialUART.print(F(" deg elec, SE ")); SerialUART.print(degrees(ac_zea_se), 2);
    SerialUART.print(F(" deg  [")); SerialUART.print(acVs(ac_v_zea)); SerialUART.println(F("]"));
    SerialUART.print(F("const int   DIR_STORED = ")); SerialUART.print(ac_dir > 0 ? F("+1") : F("-1"));
    SerialUART.print(F(";        // ")); SerialUART.println(ac_dir > 0 ? F("CW") : F("CCW"));
  } else {
    SerialUART.println(F("const float ZEA_STORED = /* NOT MEASURED - run 2 */;"));
    SerialUART.println(F("const int   DIR_STORED = /* NOT MEASURED - run 2 */;"));
  }
  acPrintVal(F("const float R_EFF      = "), ac_R,  5, ac_done[3], ac_v_R,  F("ohm, slope of the self-locked sweep"));
  acPrintVal(F("const float U0_V       = "), ac_U0, 5, ac_done[3], ac_v_U0, F("V, INTERCEPT -- the weak parameter"));
  acPrintVal(F("const float KE         = "), ac_Ke, 6, ac_done[5], ac_v_Ke, F("V/(rad/s), both directions"));
  acPrintVal(F("const float KT         = "), ac_Kt, 6, ac_done[5], ac_v_Ke, F("Nm/A = 1.5*KE (amplitude-invariant dq)"));
  SerialUART.print(F("const float L_H        = "));
  if (ac_done[4]) { SerialUART.print(ac_L*1e6f, 2); SerialUART.print(F("e-6f;")); }
  else            { SerialUART.print(F("0.0f; /* NOT MEASURED - run 4 */")); }
  SerialUART.print(F("  // H, tau=")); SerialUART.print(ac_tau*1e6f, 1);
  SerialUART.print(F(" us  [")); SerialUART.print(ac_done[4] ? acVs(ac_v_L) : F("MISSING"));
  SerialUART.println(F("]"));
  SerialUART.println();

  SerialUART.println(F("---- DIAGNOSTICS (not per-unit constants) ----"));
  if (ac_done[1]) {
    SerialUART.print(F("SPI link       ")); SerialUART.print(ac_link_us, 2);
    SerialUART.print(F(" us/read, ")); SerialUART.print(ac_link_err);
    SerialUART.println(F(" parity errors in 4000"));
  }
  if (ac_done[6]) {
    SerialUART.print(F("T_delay        ")); SerialUART.print(ac_T[AC_BIN_SPEEDS-1]*1e6f, 1);
    SerialUART.print(F(" us   (2nd estimate ")); SerialUART.print(ac_T[0]*1e6f, 1);
    SerialUART.println(F(" us)  -- FIRMWARE-WIDE, depends on f_loop and f_pwm"));
    SerialUART.print(F("  torque loss at 270 rad/s = "));
    SerialUART.print(100.0f*(1.0f - cosf(7.0f*270.0f*ac_T[AC_BIN_SPEEDS-1])), 3);
    SerialUART.println(F(" %"));
    SerialUART.print(F("INL pk-pk      ")); SerialUART.print(ac_inl_pp/7.0f, 3);
    SerialUART.println(F(" deg MECH -- fit 1/rev vs 2/rev offline from the bins below"));
    SerialUART.print(F("ZEA residual   ")); SerialUART.print(ac_zea_resid, 3);
    SerialUART.println(F(" deg elec -- independent check on the installed ZEA"));
  }
  if (ac_done[5]) {
    SerialUART.print(F("|I| ratio      ")); SerialUART.print(ac_ratio_lo, 3);
    SerialUART.print(F(" .. ")); SerialUART.print(ac_ratio_hi, 3);
    SerialUART.print(F("  (theory 1.2247)  [")); SerialUART.print(acVs(ac_v_ratio));
    SerialUART.println(F("]"));
    SerialUART.println(F("DRAG MAP (belt OFF)   U_cmd, vel, Iq"));
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
    SerialUART.println(F(" Hz.  NOT APPLIED -- paste them yourself if you want them."));
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
    SerialUART.println(F("---- L TRACE: k,t_us,I_amp  (offline exponential refit) ----"));
    for (uint8_t k = 0; k < AC_L_NS; k++) {
      SerialUART.print(F("LST,")); SerialUART.print(k);
      SerialUART.print(','); SerialUART.print(ac_lt[k], 1);
      SerialUART.print(','); SerialUART.println(ac_li[k], 4);
    }
  }
  SerialUART.println(F("=================================================="));
}

// ===========================================================================
// STATUS + DISPATCH
// ===========================================================================
static void acStatus() {
  SerialUART.println(F("--- AUTOCALIB (belt OFF).  0=reset 1..6=phases 7=report ---"));
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
  SerialUART.println(F("  Shaft FREE, belt OFF. Any key aborts a running phase."));
}

void acPhase(uint8_t n) {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  if (!driver_ok || !cs_linked) { SerialUART.println(F("refused: driver or current sense not ready")); return; }
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