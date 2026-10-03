#pragma once
#include <Arduino.h>
#include "can_proto.h"
#include "can_proto_vectors.h"
#include "master_config.h"
#include "nodes.h"
#include "poll.h"
// ============================================================================
// console.h -- the operator's console, in loop() on core 1. Line-based (Enter).
// ============================================================================
// Nothing here touches the bus directly except `stop` (ESTOP) and the N2 raw
// test; everything else is a request the poll task executes in its next cycle.
// ============================================================================

#ifndef GIT_HASH
#define GIT_HASH  0u
#define GIT_DIRTY 1
#endif

static char    c_line[96];
static uint8_t c_n = 0;
static bool    c_mon = false;
static uint32_t c_mon_ms = 0;

static void conHelp() {
  Serial.println(F(
    "-- leg master (CAN-T0). Nodes are JOINT_IDs; 'all' where shown.\n"
    "  ident                  STATUS of every node (build, contract, git, flags)\n"
    "  status | mon           one table / toggle 1 Hz table\n"
    "  arm <n|all>            needs a fresh matching STATUS; arms at zero torque\n"
    "  disarm <n|all>   zero <n|all>   clear <n|all>\n"
    "  set <n> [pd x] [vd x] [kp x] [kd x] [ff x]   contract units, output side\n"
    "  hold <n|all>           pd = current p, vd = ff = 0, gains kept\n"
    "  step <n> <dp>          pd += dp (rad)\n"
    "  sine <n> <amp> <f>     pd = centre + amp*sin(2 pi f t);  sine <n> off\n"
    "  pause <ms>             send NOTHING for ms (timeout test N4)\n"
    "  stop                   ESTOP to all, latched.   reset  (button closed)\n"
    "  log | log dump | log stop      t  self-test\n"
    "  testver <v|off>        N2: pretend our contract version is v\n"
    "  armraw <n> <ver> [echo]   N2: raw ARM frame, no master checks"));
}

static void printIdent(const Node& n) {
  Serial.printf("J%02u STATUS ", n.id);
  if (!n.ident_ms) { Serial.println("never received"); return; }
  const CanStatus& s = n.ident;
  Serial.printf("age %lums build '%c' contract v%u git %08lX%s joint %u flags[%s%s%s%s%s%s%s]\n",
    (unsigned long)(millis() - n.ident_ms), s.build_type ? s.build_type : '?', s.contract_version,
    (unsigned long)s.git_hash, (s.flags & CAN_SF_DIRTY) ? " DIRTY" : "", s.joint_id,
    (s.flags & CAN_SF_UID_OK) ? " uid" : " !UID", (s.flags & CAN_SF_CALIBRATED) ? " cal" : " !CAL",
    (s.flags & CAN_SF_HSE_OK) ? " hse" : " !HSE", (s.flags & CAN_SF_FOC_READY) ? " foc" : " !FOC",
    (s.flags & CAN_SF_CS_OK) ? " cs" : " !CS", (s.flags & CAN_SF_SELFTEST) ? " selftest" : " !SELFTEST",
    (s.flags & CAN_SF_ARMED) ? " ARMED" : "");
}

static void printStatus() {
  const BusHealth h = busHealth();
  Serial.printf("bus %s TEC=%lu REC=%lu buserr=%lu arblost=%lu txfail=%lu/%lu rxmiss=%lu stray=%lu boff=%lu | "
                "cycles=%lu max=%luus overrun=%lu%s%s\n",
    h.state, (unsigned long)h.tec, (unsigned long)h.rec, (unsigned long)h.buserr,
    (unsigned long)h.arblost, (unsigned long)h.txfail, (unsigned long)m_tx_fail,
    (unsigned long)h.rxmiss, (unsigned long)m_rx_stray, (unsigned long)m_busoff_n,
    (unsigned long)m_cycles, (unsigned long)m_cycle_us_max, (unsigned long)m_overruns,
    m_estop ? "  *** ESTOP LATCHED ***" : "", m_log_on ? "  [logging]" : "");
  m_cycle_us_max = 0;
  for (uint8_t i = 0; i < M_N_NODES; i++) {
    portENTER_CRITICAL(&m_mux);
    const Node n = m_node[i];
    const SineGen sg = m_sine[i];
    portEXIT_CRITICAL(&m_mux);
    Serial.printf("J%02u %s arm=%d ph=%s fl=%s cl=%d p=%+.4f v=%+.3f tau=%+.4f | "
                  "pd=%+.4f vd=%+.2f kp=%.1f kd=%.3f ff=%+.3f%s | sent=%lu rep=%lu miss=%lu run=%u rej=%lu lim=%lu age=%lums\n",
      n.id, n.session_armed ? "CMD " : "POLL", canStArmed(n.st_status) ? 1 : 0,
      phaseName(canStPhase(n.st_status)), faultName(canStFault(n.st_status)), canStClamped(n.st_status) ? 1 : 0,
      n.st.p_rad, n.st.v_rads, n.st.tau_Nm,
      n.cmd.p_des_rad, n.cmd.v_des_rads, n.cmd.kp_Nm_per_rad, n.cmd.kd_Nms_per_rad, n.cmd.tau_ff_Nm,
      sg.on ? " SINE" : "",
      (unsigned long)n.sent, (unsigned long)n.replies, (unsigned long)n.missed, n.miss_run,
      (unsigned long)n.rejected_pack, (unsigned long)n.soft_limited,
      (unsigned long)(n.st_ms ? millis() - n.st_ms : 0));
  }
}

// Every node named by `arg` ("all" or a JOINT_ID). Returns the count.
static uint8_t targets(const char* arg, int out[M_N_NODES]) {
  uint8_t k = 0;
  if (!arg) return 0;
  if (strcmp(arg, "all") == 0) { for (uint8_t i = 0; i < M_N_NODES; i++) out[k++] = i; return k; }
  const int i = nodeIndex((uint8_t)atoi(arg));
  if (i >= 0) out[k++] = i;
  else Serial.printf("no node '%s' on this bus\n", arg);
  return k;
}

// The master-side ARM gate. Tier 0 checks again on its side.
static bool armAllowed(int i) {
  portENTER_CRITICAL(&m_mux);
  const Node n = m_node[i];                // snapshot: the poll task writes it
  portEXIT_CRITICAL(&m_mux);
  const int ver = (m_ver_override >= 0) ? m_ver_override : CAN_CONTRACT_VERSION;
  if (m_estop) { Serial.println("ARM refused: ESTOP latched (close the button, then `reset`)"); return false; }
  if (!n.ident_ms || millis() - n.ident_ms > M_IDENT_FRESH_MS) {
    Serial.printf("J%02u ARM refused: no fresh STATUS (is it running Tier 0?)\n", n.id); return false; }
  const CanStatus& s = n.ident;
  if (s.build_type != CAN_BUILD_TIER0) { Serial.printf("J%02u ARM refused: build '%c' is not Tier 0\n", n.id, s.build_type); return false; }
  if (s.contract_version != ver) {
    Serial.printf("J%02u ARM refused: node contract v%u, master v%d -- stale binary on one side\n",
                  n.id, s.contract_version, ver); return false; }
  if (s.joint_id != n.id) { Serial.printf("J%02u ARM refused: node reports joint %u\n", n.id, s.joint_id); return false; }
  const uint8_t need = CAN_SF_UID_OK | CAN_SF_CALIBRATED | CAN_SF_HSE_OK | CAN_SF_FOC_READY | CAN_SF_CS_OK | CAN_SF_SELFTEST;
  if ((s.flags & need) != need) { Serial.printf("J%02u ARM refused: node not ready -- ", n.id); printIdent(n); return false; }
  if (canStArmed(n.st_status) && !n.session_armed) {
    Serial.printf("J%02u ARM refused: node is ALREADY ARMED by an earlier session -- disarm first\n", n.id); return false; }
  if (s.git_hash != (uint32_t)GIT_HASH || (s.flags & CAN_SF_DIRTY) || GIT_DIRTY)
    Serial.printf("J%02u WARNING: git %08lX%s vs master %08lX%s -- builds differ or are dirty\n", n.id,
                  (unsigned long)s.git_hash, (s.flags & CAN_SF_DIRTY) ? "+dirty" : "",
                  (unsigned long)GIT_HASH, GIT_DIRTY ? "+dirty" : "");
  return true;
}

static bool parseF(const char* s, float& v) {
  if (!s) return false;
  char* e; v = strtof(s, &e);
  return e != s && *e == 0 && isfinite(v);
}

static void conSet(int i, char** tok, int nt) {
  portENTER_CRITICAL(&m_mux);
  MitCmd c = m_node[i].cmd;
  portEXIT_CRITICAL(&m_mux);
  bool pd_set = false;
  for (int k = 0; k + 1 < nt; k += 2) {
    float v;
    if (!parseF(tok[k + 1], v)) { Serial.printf("bad number '%s'\n", tok[k + 1]); return; }
    if      (!strcmp(tok[k], "pd")) { c.p_des_rad = v; pd_set = true; }
    else if (!strcmp(tok[k], "vd")) c.v_des_rads = v;
    else if (!strcmp(tok[k], "kp")) c.kp_Nm_per_rad = v;
    else if (!strcmp(tok[k], "kd")) c.kd_Nms_per_rad = v;
    else if (!strcmp(tok[k], "ff")) c.tau_ff_Nm = v;
    else { Serial.printf("unknown field '%s'\n", tok[k]); return; }
  }
  uint8_t b[8];
  if (!canPackCmd(c, b)) { Serial.println("refused: not representable on the wire (range / NaN)"); return; }
  if (c.p_des_rad < M_PDES_MIN[i] || c.p_des_rad > M_PDES_MAX[i]) { Serial.println("refused: pd outside master soft limits"); return; }
  if (c.kp_Nm_per_rad == 0.0f && fabsf(c.p_des_rad - m_node[i].st.p_rad) > 1e-3f)
    Serial.println("!! pd has no effect with kp = 0");
  if (c.kd_Nms_per_rad == 0.0f && c.v_des_rads != 0.0f)
    Serial.println("!! vd has no effect with kd = 0");
  portENTER_CRITICAL(&m_mux);
  m_node[i].cmd = c;
  if (pd_set) m_sine[i].on = false;
  portEXIT_CRITICAL(&m_mux);
  Serial.printf("J%02u cmd pd=%+.4f vd=%+.2f kp=%.1f kd=%.3f ff=%+.3f%s\n", m_node[i].id, c.p_des_rad,
                c.v_des_rads, c.kp_Nm_per_rad, c.kd_Nms_per_rad, c.tau_ff_Nm,
                m_node[i].session_armed ? "" : "  (not armed: stored, not sent)");
}

static void logDump() {
  m_log_on = false;
  Serial.printf("# CAN-T0 master log, %lu records, git %08lX%s, cycle %lu ms\n",
                (unsigned long)m_log_n, (unsigned long)GIT_HASH, GIT_DIRTY ? "+dirty" : "", (unsigned long)M_CYCLE_MS);
  Serial.println("t_us,node,flags,armed,phase,clamp,fault,p_des,v_des,kp,kd,ff,p,v,tau");
  for (uint32_t k = 0; k < m_log_n; k++) {
    const LogRec& r = m_log[k];
    Serial.printf("%lu,%u,%u,%d,%u,%d,%u,%.5f,%.3f,%.1f,%.3f,%.3f,%.5f,%.4f,%.4f\n",
      (unsigned long)r.t_us, r.node, r.flags, canStArmed(r.status), canStPhase(r.status),
      canStClamped(r.status), canStFault(r.status), r.p_des, r.v_des, r.kp, r.kd, r.ff, r.p, r.v, r.tau);
    if ((k & 63) == 0) delay(1);           // let USB drain; the poll task runs on core 0
  }
  Serial.println("# end");
}

static void selfTest() {
  uint16_t first = 0;
  const uint16_t f = canProtoSelfTest(&first);
  if (f == 0) Serial.println("can_proto self-test: PASS");
  else Serial.printf("can_proto self-test: %u FAILED, first at check %u\n", f, first);
}

static void conExec(char* line) {
  char* tok[16]; int nt = 0;
  for (char* p = strtok(line, " \t;"); p && nt < 16; p = strtok(nullptr, " \t;")) tok[nt++] = p;
  if (!nt) return;
  int idx[M_N_NODES]; uint8_t k;
  const char* cmd = tok[0];

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) conHelp();
  else if (!strcmp(cmd, "status")) printStatus();
  else if (!strcmp(cmd, "mon")) { c_mon = !c_mon; Serial.printf("monitor %s\n", c_mon ? "ON" : "off"); }
  else if (!strcmp(cmd, "t")) selfTest();
  else if (!strcmp(cmd, "ident")) {
    portENTER_CRITICAL(&m_mux); for (uint8_t i = 0; i < M_N_NODES; i++) m_node[i].req = REQ_IDENT; portEXIT_CRITICAL(&m_mux);
    delay(3 * M_CYCLE_MS);
    for (uint8_t i = 0; i < M_N_NODES; i++) printIdent(m_node[i]);
  }
  else if (!strcmp(cmd, "arm") && nt >= 2) {
    k = targets(tok[1], idx);
    for (uint8_t j = 0; j < k; j++) {
      const int i = idx[j];
      if (!armAllowed(i)) continue;
      portENTER_CRITICAL(&m_mux);
      // Arm at zero torque: hold where it is, every gain zero. Tier 0 does the same.
      m_node[i].cmd = { m_node[i].st.p_rad, 0.0f, 0.0f, 0.0f, 0.0f };
      m_sine[i].on = false;
      m_node[i].req = REQ_ARM;
      portEXIT_CRITICAL(&m_mux);
    }
  }
  else if ((!strcmp(cmd, "disarm") || !strcmp(cmd, "zero") || !strcmp(cmd, "clear")) && nt >= 2) {
    const uint8_t r = !strcmp(cmd, "disarm") ? REQ_DISARM : !strcmp(cmd, "zero") ? REQ_ZERO : REQ_CLEAR;
    k = targets(tok[1], idx);
    for (uint8_t j = 0; j < k; j++) {
      if (r != REQ_DISARM && m_node[idx[j]].session_armed) { Serial.printf("J%02u: %s refused while armed\n", m_node[idx[j]].id, cmd); continue; }
      portENTER_CRITICAL(&m_mux); m_node[idx[j]].req = r; portEXIT_CRITICAL(&m_mux);
    }
  }
  else if (!strcmp(cmd, "set") && nt >= 4) {
    k = targets(tok[1], idx);
    if (k == 1) conSet(idx[0], tok + 2, nt - 2);
  }
  else if (!strcmp(cmd, "hold") && nt >= 2) {
    k = targets(tok[1], idx);
    for (uint8_t j = 0; j < k; j++) {
      portENTER_CRITICAL(&m_mux);
      Node& n = m_node[idx[j]];
      n.cmd.p_des_rad = n.st.p_rad; n.cmd.v_des_rads = 0.0f; n.cmd.tau_ff_Nm = 0.0f;
      m_sine[idx[j]].on = false;
      portEXIT_CRITICAL(&m_mux);
    }
  }
  else if (!strcmp(cmd, "step") && nt >= 3) {
    float dp; k = targets(tok[1], idx);
    if (k == 1 && parseF(tok[2], dp)) {
      char pd[24]; snprintf(pd, sizeof(pd), "%.6f", m_node[idx[0]].cmd.p_des_rad + dp);
      char f0[] = "pd"; char* t2[2] = { f0, pd };
      conSet(idx[0], t2, 2);
    }
  }
  else if (!strcmp(cmd, "sine") && nt >= 3) {
    k = targets(tok[1], idx);
    if (k != 1) return;
    if (!strcmp(tok[2], "off")) { portENTER_CRITICAL(&m_mux); m_sine[idx[0]].on = false; portEXIT_CRITICAL(&m_mux); return; }
    float a, f;
    if (nt < 4 || !parseF(tok[2], a) || !parseF(tok[3], f)) { Serial.println("sine <n> <amp rad> <f Hz>"); return; }
    const float c = m_node[idx[0]].cmd.p_des_rad;
    if (c - fabsf(a) < M_PDES_MIN[idx[0]] || c + fabsf(a) > M_PDES_MAX[idx[0]]) { Serial.println("refused: sine leaves soft limits"); return; }
    portENTER_CRITICAL(&m_mux);
    m_sine[idx[0]] = { true, c, a, f, millis() };
    portEXIT_CRITICAL(&m_mux);
  }
  else if (!strcmp(cmd, "pause") && nt >= 2) {
    const uint32_t ms = (uint32_t)atol(tok[1]);
    m_pause_until_ms = millis() + ms;
    Serial.printf("bus silent for %lu ms\n", (unsigned long)ms);
  }
  else if (!strcmp(cmd, "stop")) {
    m_estop = true; allSessionsOff(); estopBroadcast();
    Serial.println("ESTOP sent, latched. `reset` to release (button must be closed).");
  }
  else if (!strcmp(cmd, "reset")) {
    if (digitalRead(M_PIN_ESTOP) == HIGH) Serial.println("reset refused: e-stop button (or its wire) is OPEN");
    else { m_estop = false; Serial.println("ESTOP released. Joints stay faulted: `clear` then `arm`."); }
  }
  else if (!strcmp(cmd, "log")) {
    if (nt >= 2 && !strcmp(tok[1], "dump")) logDump();
    else if (nt >= 2 && !strcmp(tok[1], "stop")) { m_log_on = false; Serial.printf("log stopped at %lu\n", (unsigned long)m_log_n); }
    else if (m_log) { m_log_n = 0; m_log_on = true; Serial.printf("logging (%lu records max)\n", (unsigned long)m_log_cap); }
  }
  else if (!strcmp(cmd, "testver") && nt >= 2) {
    m_ver_override = !strcmp(tok[1], "off") ? -1 : atoi(tok[1]);
    Serial.printf("contract version override: %d (-1 = off)\n", m_ver_override);
  }
  else if (!strcmp(cmd, "armraw") && nt >= 3) {
    const uint8_t id = (uint8_t)atoi(tok[1]);
    const uint8_t b[3] = { CAN_OP_ARM, (uint8_t)(nt >= 4 ? atoi(tok[3]) : id), (uint8_t)atoi(tok[2]) };
    busSend(canIdAdmin(id), b, 3);
    Serial.printf("raw ARM -> 0x%03X ver %u echo %u (watch the T0 console / next STATE)\n", canIdAdmin(id), b[2], b[1]);
  }
  else Serial.printf("? %s  (help)\n", cmd);
}

static void conPoll() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') { c_line[c_n] = 0; if (c_n) conExec(c_line); c_n = 0; }
    else if (c_n < sizeof(c_line) - 1) c_line[c_n++] = c;
  }
  // Events from the poll task.
  for (;;) {
    char b[72];
    portENTER_CRITICAL(&m_mux);
    const bool have = (m_evt_tail != m_evt_head);
    if (have) { memcpy(b, m_evt[m_evt_tail], sizeof(b)); m_evt_tail = (uint8_t)((m_evt_tail + 1) & 7); }
    portEXIT_CRITICAL(&m_mux);
    if (!have) break;
    Serial.printf("** %s\n", b);
  }
  if (c_mon && millis() - c_mon_ms >= 1000) { c_mon_ms = millis(); printStatus(); }
}
