#pragma once
// ============================================================================
// tier0.h -- TIER 0: the MIT contract over CAN. (CAN-T0, 2026-10-03)
// ============================================================================
// Built ONLY by the T0_Jxx envs (platformio.ini [tier0]); the bench harness
// never sees this file. Reuses, unchanged:
//   actuator_hw.h  init ORDER, objects, CFG banner, runInitFOC (bench-accepted 3b)
//   safety.h       stopMotor() -- THE single disable path. Every trip below
//                  goes through it; no new disable sequence exists here.
//   mit_law.h      estimator, law, validator (B12a a0-a5, b1)
//   joint_cal.h    tauOutCmdToIq() / irepToTorqueOut() -- the torque boundary
//   can_proto.h    the wire contract (contract/, shared with the master)
//
// WHAT TIER 0 ADDS, and nothing else:
//   * FDCAN (can_fdcan.h), POLLED from the control loop -- no ISR.
//   * ADMIN: IDENT / ARM / DISARM / ZERO / CLEAR_FAULT, with refusals.
//   * Command timeout HOLD -> DAMP -> DISABLE, checked EVERY loop on the local
//     clock (owner-approved 2026-10-03; tier0_config.h has the numbers).
//   * Guards: overspeed, sense mismatch (harness copy), |p| envelope, No_Mag.
//   * A non-blocking console that queues nothing while armed (t0_console.h).
//
// NEW stopMotor() CALL SITES -- the design-review list (CLAUDE.md "single
// safety path"): CAN ESTOP, CAN DISARM, command timeout, overspeed, sense
// mismatch, p envelope, encoder No_Mag, console 'x'. All go through t0Stop()
// or stopMotor() directly; none touches the driver itself.
//
// Tier 0 NEVER ALIGNS. A row without zea/dir/Ke/R_eff cannot arm, ever --
// runInitFOC() is not even called, so an uncalibrated joint does not twitch at
// boot. (initFOC with stored ZEA still runs SimpleFOC's current-sense phase
// check -- identical to the bench-accepted harness path; its console result is
// an N1 check item, and skipping it on a leg boot is deferred with a condition.)
// ============================================================================
#include <Arduino.h>
#include <SimpleFOC.h>
#include "fleet_config.h"
#include "joint_cal.h"
#include "tier0_config.h"      // BEFORE actuator_hw.h: it defines VBUS_FALLBACK
#include "actuator_hw.h"
#include "mit_law.h"
#include "can_proto.h"
#include "can_proto_vectors.h"
#include "can_fdcan.h"
#include "t0_console.h"
#include "t0_fault.h"         // IWDG + HardFault: phases off when this code stops running

// safety.h reads these two from its includer (its header records why it was
// not changed). Declared here so this header states its own dependency;
// tier0_main.cpp defines them.
extern HardwareSerial SerialUART;
extern bool running;
#include "safety.h"

static_assert(JOINT_ID >= CAN_NODE_MIN && JOINT_ID <= CAN_NODE_MAX,
              "Tier 0 runs on deployable joints 1..12 only (CAN node = JOINT_ID)");

#ifndef GIT_HASH            // tools/git_hash.py supplies both; a build without
#define GIT_HASH  0u        // it reports hash 0 and DIRTY, which the master
#define GIT_DIRTY 1         // flags rather than trusts.
#endif

// ---- readiness, decided at boot ----
static bool t0_driver_ok = false, t0_cs_ok = false, t0_cs_linked = false;
static bool t0_foc_ready = false, t0_calibrated = false;
static bool t0_uid_ok = false, t0_selftest_ok = false;
static uint32_t t0_uid[3] = { 0, 0, 0 };

// ---- live state ----
static float         target = 0.0f;           // A_rep, what motor.move() gets
static MitEstimator  t0_est = {};
static uint32_t      t0_cyc_prev = 0;
static float         t0_s_per_cyc = 0.0f;
static MitCmd        t0_cmd = { 0, 0, 0, 0, 0 }; // the latched LIVE command
static uint32_t      t0_cmd_ms = 0;           // millis() of the last ACCEPTED CMD (or ARM)
static uint32_t      t0_run_ms = 0;           // millis() at ARM
static uint8_t       t0_phase = CAN_PHASE_LIVE;
static uint8_t       t0_fault = CAN_FAULT_NONE;   // latched until CLEAR_FAULT
static bool          t0_clamped = false;      // a clamp bound since the last STATE
static float         t0_tau_law = 0.0f;

// ---- counters (console status; reset at boot) ----
static uint32_t t0_n_cmd = 0, t0_n_rej = 0, t0_n_admin = 0, t0_n_admin_bad = 0;
static uint32_t t0_n_late = 0, t0_n_damp = 0, t0_n_resume = 0, t0_n_resume_refused = 0;
static uint32_t t0_n_tau = 0, t0_n_iq = 0, t0_n_creject = 0, t0_n_refused = 0;
static uint32_t t0_loops = 0, t0_status_ms = 0;
static uint32_t t0_pump_cyc_max = 0;

static T0ConOut con;                          // ALL post-setup text goes here

static inline int8_t t0Dir()   { return (int8_t)motor.sensor_direction; }
static inline float  t0P()     { return mitEstP(t0_est, t0Dir()); }

// ---------------------------------------------------------------------------
// The one way Tier 0 stops the motor for a FAULT: the single disable path plus
// a latched code. The first fault wins; CLEAR_FAULT is the only way back.
// ---------------------------------------------------------------------------
static void t0Stop(uint8_t fault, const char* reason) {
  stopMotor(reason);
  target = 0.0f;
  t0_phase = CAN_PHASE_LIVE;
  if (t0_fault == CAN_FAULT_NONE) t0_fault = fault;
}

static uint8_t t0StatusFlags() {
  uint8_t f = 0;
  if (GIT_DIRTY)      f |= CAN_SF_DIRTY;
  if (t0_uid_ok)      f |= CAN_SF_UID_OK;
  if (t0_calibrated)  f |= CAN_SF_CALIBRATED;
  if (can_hse_ok)     f |= CAN_SF_HSE_OK;
  if (t0_foc_ready)   f |= CAN_SF_FOC_READY;
  if (t0_cs_linked)   f |= CAN_SF_CS_OK;
  if (running)        f |= CAN_SF_ARMED;
  if (t0_selftest_ok) f |= CAN_SF_SELFTEST;
  return f;
}

// ---------------------------------------------------------------------------
// Replies. Only ever sent in answer to a frame addressed to this node.
// ---------------------------------------------------------------------------
static void t0SendState() {
  // tau is MEASURED. While disarmed loopFOC() returns before reading the
  // current (SimpleFOC 2.3.1), so motor.current.q is stale -- report 0.
  const MitState s = { t0P(), t0_est.v_out_rads,
                       running ? irepToTorqueOut(motor.current.q) : 0.0f };
  uint8_t b[8];
  canPackState(s, canStatusByte(running, t0_phase, t0_clamped, t0_fault), b);
  canSend(canIdState(JOINT_ID), b, 8);
  t0_clamped = false;
}

static void t0SendStatus() {
  const CanStatus st = { CAN_BUILD_TIER0, CAN_CONTRACT_VERSION, (uint32_t)GIT_HASH,
                         (uint8_t)JOINT_ID, t0StatusFlags() };
  uint8_t b[8];
  canPackStatus(st, b);
  canSend(canIdStatus(JOINT_ID), b, 8);
}

// ---------------------------------------------------------------------------
// ARM. Every precondition is checked, in order, and the first failure is
// named on the console. The master can also see most of them in STATUS flags.
// ---------------------------------------------------------------------------
static bool t0Refuse(const __FlashStringHelper* why) {
  t0_n_refused++;
  con.print(F("ARM REFUSED: ")); con.println(why);
  return false;
}

static bool t0TryArm(uint8_t version) {
  if (running)                         return t0Refuse(F("already armed -- DISARM first"));
  if (t0_fault != CAN_FAULT_NONE)      return t0Refuse(F("fault latched -- CLEAR_FAULT first"));
  if (version != CAN_CONTRACT_VERSION) return t0Refuse(F("contract version mismatch"));
  if (!t0_selftest_ok)                 return t0Refuse(F("can_proto self-test failed on this build"));
  if (!can_up)                         return t0Refuse(F("CAN not up (HSE / timing / mode)"));
  if (!t0_driver_ok)                   return t0Refuse(F("driver init failed"));
  if (!t0_cs_linked)                   return t0Refuse(F("current sense not linked"));
  if (!t0_calibrated)                  return t0Refuse(F("joint_cal row not calibrated (zea/dir/Ke/R_eff)"));
  if (!t0_uid_ok)                      return t0Refuse(F("board UID does not match JOINT_UID[] -- wrong board or not recorded"));
  if (!t0_foc_ready)                   return t0Refuse(F("initFOC failed"));
  if (encoder.no_mag)                  return t0Refuse(F("encoder No_Mag"));
  if (fabsf(t0P()) > T0_P_ABS_MAX_RAD) return t0Refuse(F("|p| outside envelope -- ZERO first"));

  // "Torque modes arm at zero": p_des = p, every gain zero -> tau = 0 until a
  // CMD arrives. Unlike the harness, p is NOT re-zeroed here -- ZERO is an
  // explicit ADMIN op, so the master's frame of reference survives a re-arm.
  t0_cmd = { t0P(), 0.0f, 0.0f, 0.0f, 0.0f };
  t0_cmd_ms = t0_run_ms = millis();
  t0_phase = CAN_PHASE_LIVE;
  t0_clamped = false;
  target = 0.0f;
  // Same arming sequence as the harness's startMotor() for current modes.
  motor.PID_current_q.reset();
  motor.PID_current_d.reset();
  encoder.update();
  motor.enable();
  running = true;
  return true;
}

static void t0OnAdmin(const CanRxFrame& fr) {
  // Byte 1 must echo our node: an ID typo in the master cannot address us.
  if (fr.dlc != CAN_ADMIN_DLC || fr.data[1] != JOINT_ID) { t0_n_admin_bad++; return; }
  t0_n_admin++;
  switch (fr.data[0]) {
    case CAN_OP_IDENT:
      t0SendStatus();
      return;
    case CAN_OP_ARM:
      if (t0TryArm(fr.data[2])) con.println(F("ARMED over CAN (cmd = hold here, zero gains)"));
      break;
    case CAN_OP_DISARM:
      if (running) { stopMotor("CAN DISARM"); target = 0.0f; t0_phase = CAN_PHASE_LIVE; }
      break;
    case CAN_OP_ZERO:
      if (running) con.println(F("ZERO refused: armed"));
      else         mitEstZero(t0_est);
      break;
    case CAN_OP_CLEAR_FAULT:
      if (running) con.println(F("CLEAR refused: armed"));
      else         t0_fault = CAN_FAULT_NONE;
      break;
    case CAN_OP_POLL:                    // monitoring only: reply STATE, change nothing
      break;
    default:
      t0_n_admin_bad++;
      break;
  }
  t0SendState();
}

static void t0OnCmd(const CanRxFrame& fr) {
  t0_n_cmd++;
  if (fr.dlc != 8) { t0_n_rej++; t0SendState(); return; }
  MitCmd c;
  canUnpackCmd(fr.data, c);
  if (!mitCmdValid(c, T0_RANGES)) {
    // Not received, as far as the timeout is concerned: an invalid CMD does
    // not refresh it. Reject, never repair.
    t0_n_rej++;
  } else if (running) {
    if (t0_phase == CAN_PHASE_DAMP && fabsf(c.p_des_rad - t0P()) > T0_P_RESUME_MARGIN_RAD) {
      // A stale target must not snap the leg: stay in DAMP, let T_OFF run out.
      t0_n_resume_refused++;
    } else {
      if (t0_phase != CAN_PHASE_LIVE) t0_n_resume++;
      t0_cmd = c;
      t0_cmd_ms = millis();
      t0_phase = CAN_PHASE_LIVE;
    }
  }
  // Disarmed: a valid CMD is ignored (it is how the master polls state).
  t0SendState();
}

static void t0OnFrame(const CanRxFrame& fr) {
  // ESTOP: no reply. The master repeats it every cycle while its e-stop is
  // latched, so act once -- when armed, or to latch the fault -- rather than
  // printing "STOPPED" to a blocking UART 200 times a second.
  if (fr.id == CAN_ID_ESTOP) {
    if (running || t0_fault == CAN_FAULT_NONE) t0Stop(CAN_FAULT_ESTOP, "CAN ESTOP");
    return;
  }
  if (fr.id == canIdAdmin(JOINT_ID)) { t0OnAdmin(fr); return; }
  if (fr.id == canIdCmd(JOINT_ID))   { t0OnCmd(fr);   return; }
}

// ---------------------------------------------------------------------------
// Armed control: timeout -> guards -> law -> move. Every loop.
// ---------------------------------------------------------------------------
static uint8_t t0_guard_div = 0, t0_guard_hits = 0;

static void t0Armed() {
  const uint32_t now = millis();

  // ---- COMMAND TIMEOUT, local clock, independent of reception ----
  const uint32_t age = now - t0_cmd_ms;
  if (age >= T0_T_OFF_MS) { t0Stop(CAN_FAULT_CMD_TIMEOUT, "CAN timeout"); return; }
  if (age >= T0_T_HOLD_MS) {
    if (t0_phase != CAN_PHASE_DAMP) { t0_phase = CAN_PHASE_DAMP; t0_n_damp++; }
  } else if (age >= T0_T_LATE_MS) {
    if (t0_phase == CAN_PHASE_LIVE) { t0_phase = CAN_PHASE_HOLD; t0_n_late++; }
  }

  // ---- GUARDS ----
  const float p = t0P();
  const float v = t0_est.v_out_rads;
  // UNIT: v is OUTPUT rad/s; the threshold is MOTOR rad/s (tier0_config.h).
  if (now - t0_run_ms > T0_OVERSPEED_GRACE_MS &&
      fabsf(v) * GEAR_RATIO > T0_OVERSPEED_MOTOR_RADS) { t0Stop(CAN_FAULT_OVERSPEED, "OVERSPEED"); return; }
  if (fabsf(p) > T0_P_ABS_MAX_RAD)                   { t0Stop(CAN_FAULT_P_ENVELOPE, "P ENVELOPE"); return; }
  if (encoder.no_mag)                                { t0Stop(CAN_FAULT_ENCODER, "ENCODER No_Mag"); return; }
  // Sense mismatch -- open_test.cpp's loop() guard, verbatim in logic: raw
  // (unsynchronised) |I| far above sqrt(3/2)|Iq| for ~20 ms means the dq
  // feedback has collapsed and the loop is winding to the rail.
  if (++t0_guard_div >= T0_GUARD_DIV) {
    t0_guard_div = 0;
    PhaseCurrent_s gc = currentSense.getPhaseCurrents();
    const float raw = sqrtf(gc.a * gc.a + gc.b * gc.b + gc.c * gc.c);
    const float expect = AMP_INV_MAG * fabsf(motor.current.q) + T0_GUARD_PAD_A_rep;
    if (raw > expect * 3.0f && raw > T0_GUARD_FLOOR_A_rep && fabsf(motor.voltage.q) < T0_VOLT_LIMIT * 0.9f) {
      if (++t0_guard_hits >= T0_GUARD_HITS) {
        t0_guard_hits = 0;
        t0Stop(CAN_FAULT_SENSE_MISMATCH, "SENSE MISMATCH (sustained raw |I| >> dq Iq)");
        return;
      }
    } else {
      t0_guard_hits = 0;
    }
  }

  // ---- LAW ----
  MitCmd eff = t0_cmd;
  if (t0_phase == CAN_PHASE_DAMP) eff = { p, 0.0f, 0.0f, T0_KD_TIMEOUT, 0.0f };
  const MitState s = { p, v, 0.0f };
  t0_tau_law = mitLawTau(eff, s);
  const IqCmd r = tauOutCmdToIq(t0_tau_law, T0_TAU_MAX_Nm, T0_ENV_A_rep);
  if      (r.clamp == IQ_CLAMP_TAU)    { t0_n_tau++; t0_clamped = true; }
  else if (r.clamp == IQ_CLAMP_IQ)     { t0_n_iq++;  t0_clamped = true; }
  else if (r.clamp == IQ_CLAMP_REJECT) { t0_n_creject++; }
  target = r.iq_A_rep;
  motor.move(target);
}

// ---------------------------------------------------------------------------
// Console. Immediate keys when no line is open: x s b t ?
// Lines (Enter): `uid`, and the N4e test lines `hang!` / `fault!` (work armed).
// Nothing is queued while armed.
// ---------------------------------------------------------------------------
static char    t0_line[16];
static uint8_t t0_line_n = 0;

static void t0PrintUidRow(Print& out) {
  out.print(F("  { \"")); out.print(CAL.id); out.print(F("\", { 0x"));
  out.print(t0_uid[0], HEX); out.print(F(", 0x")); out.print(t0_uid[1], HEX);
  out.print(F(", 0x")); out.print(t0_uid[2], HEX); out.println(F(" } },"));
}

static void t0SelfTest(Print& out) {
  uint16_t first = 0;
  const uint16_t fails = canProtoSelfTest(&first);
  t0_selftest_ok = (fails == 0);
  out.print(F("can_proto self-test: "));
  if (fails == 0) out.println(F("PASS"));
  else { out.print(fails); out.print(F(" FAILED, first at check ")); out.println(first); }
}

static void t0PrintStatus() {
  const uint32_t now = millis();
  const uint32_t dt = now - t0_status_ms;
  const uint32_t lps = dt ? (t0_loops * 1000UL) / dt : 0;
  t0_loops = 0; t0_status_ms = now;
  const CanHealth h = canHealth();
  con.print(F("T0 ")); con.print(CAL.id);
  con.print(F(" run=")); con.print(running ? 1 : 0);
  con.print(F(" fl=")); con.print(t0_fault);
  con.print(F(" rdy=")); con.print(t0_uid_ok); con.print(t0_calibrated); con.print(can_up);
  con.print(t0_foc_ready); con.print(t0_cs_linked); con.print(t0_selftest_ok);
  con.print(F(" p=")); con.print(t0P(), 4);
  con.print(F(" v=")); con.print(t0_est.v_out_rads, 3);
  con.print(F(" cmd=")); con.print(t0_n_cmd);
  con.print(F(" rej=")); con.print(t0_n_rej);
  con.print(F(" adm=")); con.print(t0_n_admin); con.print('/'); con.print(t0_n_admin_bad);
  con.print(F(" late=")); con.print(t0_n_late);
  con.print(F(" damp=")); con.print(t0_n_damp);
  con.print(F(" res=")); con.print(t0_n_resume); con.print('/'); con.print(t0_n_resume_refused);
  con.print(F(" cl=")); con.print(t0_n_tau); con.print('/'); con.print(t0_n_iq); con.print('/'); con.print(t0_n_creject);
  con.print(F(" TEC=")); con.print(h.tec);
  con.print(F(" REC=")); con.print(h.rec);
  con.print(F(" LEC=")); con.print(canLecName(h.lec));
  con.print(F(" EP=")); con.print(h.ep); con.print(F(" BO=")); con.print(h.bo);
  con.print(F(" boff=")); con.print(can_busoff_n);
  con.print(F(" rx=")); con.print(can_rx_n);
  con.print(F(" tx=")); con.print(can_tx_n); con.print('/'); con.print(can_tx_full);
  con.print(F(" lps=")); con.print(lps);
  // Longest single pump() call since the last line, in us. The non-blocking
  // claim is verified, not assumed: it must stay a few us, never ~1.5 ms.
  con.print(F(" pump_us=")); con.println((float)t0_pump_cyc_max * t0_s_per_cyc * 1e6f, 1);
  t0_pump_cyc_max = 0;
}

static void t0Banner(Print& out);

static void t0ExecLine() {
  t0_line[t0_line_n] = 0;
  if (strcmp(t0_line, "uid") == 0) t0PrintUidRow(con);
  // N4e ONLY -- deliberate failures, typed in full, console only (never CAN).
  // Both must end in an IWDG reset that reboots the joint DISARMED.
  else if (strcmp(t0_line, "hang!") == 0)  { for (;;) { } }         // loop stops kicking
  else if (strcmp(t0_line, "fault!") == 0) { __builtin_trap(); }   // UDF -> HardFault
  else if (t0_line_n) { con.print(F("? ")); con.println(t0_line); }
  t0_line_n = 0;
}

static void t0Console() {
  while (SerialUART.available()) {
    const char c = (char)SerialUART.read();
    if (t0_line_n == 0 && c == 'x') {        // LOCAL STOP: works armed, never queued text
      if (running) t0Stop(CAN_FAULT_LOCAL_STOP, "console x");
      continue;
    }
    if (t0_line_n == 0 && !running) {
      if (c == 's') { t0PrintStatus(); continue; }
      if (c == 'b') { t0Banner(con);   continue; }
      if (c == 't') { t0SelfTest(con); continue; }
      if (c == '?') { con.println(F("x stop | s status | b banner | t self-test | uid<Enter>")); continue; }
    }
    if (c == '\r' || c == '\n') { t0ExecLine(); continue; }
    if (t0_line_n < sizeof(t0_line) - 1) t0_line[t0_line_n++] = c;
  }
}

static void t0Banner(Print& out) {
  out.print(F("=== TIER 0  ")); out.print(CAL.id);
  out.print(F("  build T  contract v")); out.print(CAN_CONTRACT_VERSION);
  out.print(F("  git ")); out.print((uint32_t)GIT_HASH, HEX);
  out.println(GIT_DIRTY ? F(" DIRTY ===") : F(" ===="));
  out.print(F("UID ")); out.print(t0_uid[0], HEX); out.print(' ');
  out.print(t0_uid[1], HEX); out.print(' '); out.print(t0_uid[2], HEX);
  out.println(t0_uid_ok ? F("  MATCHES JOINT_UID[]") : F("  !! NOT IN JOINT_UID[] -- ARM REFUSED. Row to paste:"));
  if (!t0_uid_ok) t0PrintUidRow(out);
  out.print(F("envelope: inner ")); out.print(T0_ENV_A_rep, 2);
  out.print(F(" A_rep, outer ")); out.print(T0_TAU_MAX_Nm, 3);
  out.print(F(" N.m; validator kp<=")); out.print(T0_RANGES.kp_max, 1);
  out.print(F(" kd<=")); out.print(T0_RANGES.kd_max, 3);
  out.print(F(" |vd|<=")); out.print(T0_RANGES.v_abs_max, 1);
  out.print(F(" |ff|<=")); out.println(T0_RANGES.tau_abs_max, 2);
  out.print(F("timeout: late ")); out.print(T0_T_LATE_MS);
  out.print(F(" ms, damp at ")); out.print(T0_T_HOLD_MS);
  out.print(F(" ms (kd ")); out.print(T0_KD_TIMEOUT, 2);
  out.print(F("), off at ")); out.print(T0_T_OFF_MS);
  out.print(F(" ms; resume margin ")); out.print(T0_P_RESUME_MARGIN_RAD, 3);
  out.println(F(" rad"));
  out.print(F("ready: uid=")); out.print(t0_uid_ok); out.print(F(" cal=")); out.print(t0_calibrated);
  out.print(F(" can=")); out.print(can_up); out.print(F(" foc=")); out.print(t0_foc_ready);
  out.print(F(" cs=")); out.print(t0_cs_linked); out.print(F(" selftest=")); out.println(t0_selftest_ok);
}

static void t0Led() {
  static uint32_t last = 0; static bool on = false;
  const bool ready = t0_uid_ok && t0_calibrated && can_up && t0_foc_ready && t0_cs_linked
                     && t0_selftest_ok && t0_fault == CAN_FAULT_NONE;
  if (running) { if (!on) { on = true; digitalWrite(LED_BUILTIN, HIGH); } return; }
  const uint32_t period = ready ? 500 : 80;          // fast blink = cannot arm
  if (millis() - last > period) { on = !on; digitalWrite(LED_BUILTIN, on); last = millis(); }
}

// ---------------------------------------------------------------------------
// SETUP -- blocking prints are fine here: nothing is armed and the bus has not
// been joined until canInit() at the end.
// ---------------------------------------------------------------------------
static void t0Setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  SerialUART.begin(921600);
  _delay(500);
  SimpleFOCDebug::enable(&SerialUART);
  SerialUART.println(F("=== TIER 0 joint firmware (CAN-T0) ==="));
  SerialUART.print(F("last reset: ")); SerialUART.println(t0ResetCause());
  printJointCal(SerialUART);

  const ActuatorHwCfg hw = {
    T0_DRIVER_VOLT_LIMIT, DEAD_ZONE, (long)PWM_FREQ_HZ,
    T0_PIN_VBUS, CAL.vbus_scale, VBUS_MIN, VBUS_MAX, VBUS_FALLBACK
  };
  t0_driver_ok = actuatorInitHw(hw, SerialUART);

  // ---- MOTOR POLICY -- Tier 0's, set between the two init phases because
  // motor.init() consumes voltage_limit (actuator_hw.h). The harness's MIT mode
  // runs this same configuration (torque / foc_current), entered later.
  motor.controller        = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;   // explicit: 2.3.1 defaults to Sine
  motor.voltage_limit     = T0_VOLT_LIMIT;
  motor.current_limit     = T0_ENV_A_rep;                        // inert in torque mode; banner honesty
  motor.PID_current_q.P = T0_CURQ_P; motor.PID_current_q.I = T0_CURQ_I; motor.PID_current_q.D = 0.0f;
  motor.PID_current_d.P = T0_CURD_P; motor.PID_current_d.I = T0_CURD_I; motor.PID_current_d.D = 0.0f;
  motor.LPF_current_q.Tf = T0_CUR_TF;
  motor.LPF_current_d.Tf = T0_CUR_TF;
  motor.PID_current_q.limit = T0_VOLT_LIMIT;
  motor.PID_current_d.limit = T0_VOLT_LIMIT;
  motor.voltage_sensor_align = 1.0f;     // harness value; used by the current-sense check only

  t0_cs_ok = actuatorInitMotor(SerialUART, t0_cs_linked);
  printCfgBanner(SerialUART, T0_DRIVER_VOLT_LIMIT, T0_ENV_A_rep);

  // ---- IDENTITY: board UID against joint_cal.h's JOINT_UID[] ----
  t0_uid[0] = HAL_GetUIDw0(); t0_uid[1] = HAL_GetUIDw1(); t0_uid[2] = HAL_GetUIDw2();
  {
    const JointUid& u = JOINT_UID[JOINT_ID - 1];
    const bool recorded = (u.w[0] | u.w[1] | u.w[2]) != 0;
    t0_uid_ok = recorded && strcmp(u.id, CAL.id) == 0 &&
                u.w[0] == t0_uid[0] && u.w[1] == t0_uid[1] && u.w[2] == t0_uid[2];
  }

  // ---- STORED ZEA ONLY. Tier 0 never aligns. ----
  t0_calibrated = (CAL.zea >= 0.0f && CAL.dir != 0 && CAL.Ke > 0.0f && CAL.R_eff > 0.0f);
  if (t0_calibrated) {
    runInitFOC(false, false, t0_foc_ready, target, SerialUART);
    // What SimpleFOC's current-sense alignment LEFT the gains at. "MOT: Success:
    // 3" (N1, 2026-10-04) only says at least one gain ended negative -- it
    // cannot tell "unchanged from the constructor" from "one phase flipped".
    // UNIT: gain_x is the V->A ratio 1/(shunt * amp gain), NOT the amp gain:
    // 1/(0.003 * -64/7) = -36.458333. (Corrected 2026-10-04: the first version
    // of this line said -9.142857, which is the amp gain itself.)
    // All three at -36.458333 = alignment changed nothing -- measured on J01
    // 2026-10-04 (N2d boot) -- the evidence skip_align needs, per board.
    SerialUART.print(F("cs gains after align: "));
    SerialUART.print(currentSense.gain_a, 6); SerialUART.print(' ');
    SerialUART.print(currentSense.gain_b, 6); SerialUART.print(' ');
    SerialUART.print(currentSense.gain_c, 6);
    SerialUART.println(F("  (unchanged = -36.458333 each = 1/(0.003 * -64/7))"));
  } else {
    SerialUART.println(F("!! joint_cal row NOT calibrated -- initFOC skipped, ARM REFUSED for this boot."));
  }

  // ---- SELF-TEST the wire contract on THIS compiler, before joining the bus ----
  t0SelfTest(SerialUART);

  // ---- ESTIMATOR ----
  t0_s_per_cyc = 1.0f / (float)SystemCoreClock;
  mitEstPrime(t0_est, encoder.raw);
  t0_cyc_prev = DWT->CYCCNT;

  // ---- CAN LAST: join the bus only once everything else is settled ----
  canInit((uint8_t)JOINT_ID, SerialUART);

  t0Banner(SerialUART);
  SerialUART.println(F("Motor DISARMED. Arms only over CAN. Console: ? for keys."));
  t0_status_ms = millis();
  // LAST: from here a loop that stops kicking for ~20 ms resets the chip.
  t0IwdgStart();
  SerialUART.print(F("IWDG armed: reload ")); SerialUART.print(T0_IWDG_RELOAD);
  SerialUART.println(F(" at LSI/4 = ~20 ms"));
}

static void t0Loop() {
  motor.loopFOC();                               // refreshes the encoder even disarmed
  t0_loops++;

  const uint32_t c0 = DWT->CYCCNT;
  const float dt = (float)(uint32_t)(c0 - t0_cyc_prev) * t0_s_per_cyc;
  t0_cyc_prev = c0;
  mitEstUpdate(t0_est, encoder.raw, t0Dir(), dt, T0_MIT_TF_S);

  canService();
  for (uint8_t i = 0; i < T0_CAN_RX_PER_LOOP; i++) {
    CanRxFrame fr;
    if (!canPoll(fr)) break;
    t0OnFrame(fr);
  }

  if (running) t0Armed();

  // Console: nothing is queued while armed (owner addition 2).
  con.enabled = !running;
  t0Console();
  if (!running && millis() - t0_status_ms >= T0_STATUS_MS) t0PrintStatus();
  {
    const uint32_t p0 = DWT->CYCCNT;
    con.pump(SerialUART);
    const uint32_t dp = DWT->CYCCNT - p0;
    if (dp > t0_pump_cyc_max) t0_pump_cyc_max = dp;
  }
  t0Led();
  t0IwdgKick();                                  // the ONLY kick site
}
