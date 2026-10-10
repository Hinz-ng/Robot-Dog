#pragma once
#include <Arduino.h>
#include "can_proto.h"
#include "master_config.h"
#include "twai_bus.h"
#include "nodes.h"
// ============================================================================
// poll.h -- the polled bus schedule. A FreeRTOS task pinned to core 1 (WiFi owns core 0).
// ============================================================================
// Every M_CYCLE_MS, for each node in turn: [IDENT -> STATUS once a second]
// [pending admin -> STATE] then the regular exchange -> STATE:
//   * CMD   if THIS master armed the node in this session, else
//   * POLL  (monitor only; changes nothing on the joint).
// A joint never transmits unless asked, so the bus has no contention.
//
// OWNER ADDITION 3: the master BOOTS POLL-ONLY. A CMD frame goes only to a
// node this master armed, so a master that reboots inside a joint's DAMP
// window cannot resume it -- it sends no CMD, its ARM is refused while the
// joint is still armed, and the joint runs on to DISABLED.
//
// LOST NODE: an armed node that misses M_LOST_N consecutive replies ->
// ESTOP to every joint, everyone back to POLL. Tier 0 times out on its own
// as well; this is the second, independent layer.
//
// The task never prints: it queues short event strings for the console.
// ============================================================================

// ---- events, task -> console ----
static char     m_evt[8][72];
static uint8_t  m_evt_head = 0, m_evt_tail = 0;
static void evt(const char* fmt, ...) {
  char b[72];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  portENTER_CRITICAL(&m_mux);
  const uint8_t n = (uint8_t)((m_evt_head + 1) & 7);
  if (n != m_evt_tail) { memcpy(m_evt[m_evt_head], b, sizeof(b)); m_evt_head = n; }
  portEXIT_CRITICAL(&m_mux);
}

// ---- log: one record per node per cycle, real per-sample timestamp ----
struct LogRec {
  uint32_t t_us;                 // micros() when the reply arrived (or timed out)
  uint8_t  node, status, flags, pad;   // flags: 1 reply, 2 CMD sent, 4 POLL sent
  float    p_des, v_des, kp, kd, ff;
  float    p, v, tau;
};
static LogRec*  m_log = nullptr;
static uint32_t m_log_cap = 0, m_log_n = 0;
static volatile bool m_log_on = false;

static void logAlloc(Print& out) {
  for (uint32_t cap = M_LOG_RECORDS; cap >= 250; cap /= 2) {
    m_log = (LogRec*)malloc(cap * sizeof(LogRec));
    if (m_log) { m_log_cap = cap; break; }
  }
  out.printf("log: %lu records (%lu B)\n", (unsigned long)m_log_cap,
             (unsigned long)(m_log_cap * sizeof(LogRec)));
}

// ---- schedule state ----
static volatile bool     m_estop = false;          // latched: button or console `stop`
static volatile uint32_t m_pause_until_ms = 0;
static volatile uint32_t m_cycles = 0, m_cycle_us_max = 0, m_overruns = 0;
static volatile int      m_ver_override = -1;      // N2 test: pretend our contract is this

static void estopBroadcast() {
  // Retransmission allowed: an ESTOP is never stale.
  busSend(CAN_ID_ESTOP, nullptr, 0, /*single_shot=*/false);
}

// Everyone back to POLL. Called with or without the mutex NOT held.
static void allSessionsOff() {
  portENTER_CRITICAL(&m_mux);
  for (uint8_t i = 0; i < M_N_NODES; i++) { m_node[i].session_armed = false; m_sine[i].on = false; }
  portEXIT_CRITICAL(&m_mux);
}

static void sendAdmin(uint8_t id, uint8_t op) {
  uint8_t b[CAN_ADMIN_DLC];
  canPackAdmin(op, id, b);
  if (m_ver_override >= 0) b[2] = (uint8_t)m_ver_override;
  busSend(canIdAdmin(id), b, CAN_ADMIN_DLC);
}

// Handle one STATE reply for node i. Returns the status byte.
static uint8_t takeState(int i, const uint8_t d[8]) {
  MitState s; uint8_t st;
  canUnpackState(d, s, st);
  portENTER_CRITICAL(&m_mux);
  Node& n = m_node[i];
  n.st = s; n.st_status = st; n.st_ms = millis();
  n.replies++; n.miss_run = 0;
  const bool dropped = n.session_armed && !canStArmed(st);
  if (dropped) { n.session_armed = false; m_sine[i].on = false; }
  portEXIT_CRITICAL(&m_mux);
  if (dropped) evt("J%02u DISARMED ITSELF: fault %s", m_node[i].id, faultName(canStFault(st)));
  return st;
}

static void pollNode(int i, uint32_t now_ms) {
  // Snapshot what the console owns.
  portENTER_CRITICAL(&m_mux);
  Node& n = m_node[i];
  const uint8_t id = n.id;
  const uint8_t req = n.req; n.req = REQ_NONE;
  MitCmd cmd = n.cmd;
  const bool armed = n.session_armed;
  SineGen sg = m_sine[i];
  const bool ident_due = (now_ms - n.last_ident_poll_ms >= M_IDENT_PERIOD_MS) || req == REQ_IDENT;
  if (ident_due) n.last_ident_poll_ms = now_ms;
  portEXIT_CRITICAL(&m_mux);

  uint8_t d[8], dlc;

  // ---- 1. IDENT -> STATUS ----
  if (ident_due) {
    busDrain();
    sendAdmin(id, CAN_OP_IDENT);
    if (busWaitFor(canIdStatus(id), d, dlc, M_REPLY_TIMEOUT_US) && dlc == 8) {
      CanStatus s; canUnpackStatus(d, s);
      portENTER_CRITICAL(&m_mux);
      n.ident = s; n.ident_ms = millis();
      portEXIT_CRITICAL(&m_mux);
    }
  }

  // ---- 2. pending admin request -> STATE ----
  if (req >= REQ_ARM && req <= REQ_CLEAR) {
    const uint8_t op = (req == REQ_ARM) ? CAN_OP_ARM : (req == REQ_DISARM) ? CAN_OP_DISARM
                     : (req == REQ_ZERO) ? CAN_OP_ZERO : CAN_OP_CLEAR_FAULT;
    if (req == REQ_DISARM) {                 // stop commanding BEFORE asking
      portENTER_CRITICAL(&m_mux); n.session_armed = false; m_sine[i].on = false; portEXIT_CRITICAL(&m_mux);
    }
    busDrain();
    sendAdmin(id, op);
    if (busWaitFor(canIdState(id), d, dlc, M_REPLY_TIMEOUT_US) && dlc == 8) {
      const uint8_t st = takeState(i, d);
      portENTER_CRITICAL(&m_mux);
      n.last_req_result = st; n.last_req_done = true;
      if (req == REQ_ARM && canStArmed(st)) n.session_armed = true;
      portEXIT_CRITICAL(&m_mux);
      if (req == REQ_ARM) evt(canStArmed(st) ? "J%02u ARMED" : "J%02u ARM REFUSED (see its STATUS flags / T0 console)", id);
    } else {
      portENTER_CRITICAL(&m_mux); n.last_req_done = true; n.last_req_result = 0xFF; portEXIT_CRITICAL(&m_mux);
      evt("J%02u: no reply to admin op %u", id, op);
    }
    return;                                  // one exchange per node per cycle
  }

  // ---- 3. the regular exchange ----
  uint8_t flags = 0;
  busDrain();
  if (armed) {
    if (sg.on) cmd.p_des_rad = sg.centre + sg.amp * sinf(6.2831853f * sg.freq_hz * (now_ms - sg.t0_ms) * 1e-3f);
    const bool in_limits = cmd.p_des_rad >= M_PDES_MIN[i] && cmd.p_des_rad <= M_PDES_MAX[i];
    if (!in_limits || !canPackCmd(cmd, d)) {
      // Reject, never repair: send NOTHING. Tier 0 will HOLD, then DAMP.
      portENTER_CRITICAL(&m_mux);
      if (!in_limits) n.soft_limited++; else n.rejected_pack++;
      portEXIT_CRITICAL(&m_mux);
      return;
    }
    busSend(canIdCmd(id), d, 8);
    flags |= 2;
  } else {
    uint8_t b[CAN_ADMIN_DLC];
    canPackAdmin(CAN_OP_POLL, id, b);
    busSend(canIdAdmin(id), b, CAN_ADMIN_DLC);
    flags |= 4;
  }
  portENTER_CRITICAL(&m_mux); n.sent++; portEXIT_CRITICAL(&m_mux);

  uint8_t st = 0;
  MitState s = {};
  if (busWaitFor(canIdState(id), d, dlc, M_REPLY_TIMEOUT_US) && dlc == 8) {
    st = takeState(i, d);
    canUnpackState(d, s, st);
    flags |= 1;
  } else {
    bool lost = false;
    portENTER_CRITICAL(&m_mux);
    n.missed++; n.miss_run++;
    lost = armed && n.miss_run >= M_LOST_N;
    portEXIT_CRITICAL(&m_mux);
    if (lost) {
      estopBroadcast();
      allSessionsOff();
      evt("J%02u LOST (%u missed) -> ESTOP to all", id, M_LOST_N);
    }
  }

  if (m_log_on && m_log && m_log_n < m_log_cap) {
    LogRec& r = m_log[m_log_n++];
    r.t_us = micros(); r.node = id; r.status = st; r.flags = flags; r.pad = 0;
    r.p_des = cmd.p_des_rad; r.v_des = cmd.v_des_rads; r.kp = cmd.kp_Nm_per_rad;
    r.kd = cmd.kd_Nms_per_rad; r.ff = cmd.tau_ff_Nm;
    r.p = s.p_rad; r.v = s.v_rads; r.tau = s.tau_Nm;
    if (m_log_n >= m_log_cap) { m_log_on = false; evt("log full (%lu records)", (unsigned long)m_log_cap); }
  }
}

static void pollTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(M_CYCLE_MS));
    const uint32_t c0 = micros();
    const uint32_t now = millis();
    busService();

    // E-stop button: NC to GND, pull-up. HIGH = pressed OR wire broken.
    if (digitalRead(M_PIN_ESTOP) == HIGH && !m_estop) {
      m_estop = true;
      evt("ESTOP BUTTON (or its wire) OPEN -> ESTOP to all");
    }
    if (m_estop) {
      allSessionsOff();
      estopBroadcast();                       // every cycle while latched
    }

    if ((int32_t)(m_pause_until_ms - now) > 0) continue;   // N4a: total silence

    for (uint8_t i = 0; i < M_N_NODES; i++) pollNode(i, now);

    const uint32_t dt = micros() - c0;
    if (dt > m_cycle_us_max) m_cycle_us_max = dt;
    if (dt > M_CYCLE_MS * 1000UL) m_overruns++;
    m_cycles++;
  }
}
