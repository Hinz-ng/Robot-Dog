#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "master_config.h"
#include "nodes.h"
#include "poll.h"
#include "con_out.h"
#include "console.h"
#include "web_page.h"
// ============================================================================
// web_term.h -- the phone terminal. (CAN-T0, 2026-10-04)
// ============================================================================
// The ESP32 runs its own WiFi network (softAP). The phone joins it and opens
// http://192.168.4.1 -- a terminal that types the same commands as the USB
// console and shows the same output.
//
// WHY THE BUILT-IN WebServer AND POLLING, not async WebSockets: no extra
// libraries on a pinned toolchain, and every handler runs inside loop() on
// core 1 -- the same place the USB console runs -- so the web side adds no
// concurrency at all. The page asks for new lines 4x a second; a terminal
// does not need more.
//
// CORE PLACEMENT: the WiFi stack lives on core 0, so the 200 Hz bus task was
// moved to core 1 (main.cpp). Radio activity cannot then stretch a cycle or
// eat the 500 us reply window.
//
// ENDPOINTS
//   GET /                      the page (web_page.h)
//   GET /poll?since=N          new console lines + a live node summary (JSON)
//   GET /cmd?c=<line>          queue one command line (runs in conPoll())
//   GET /stop                  ESTOP to all, latched -- runs immediately
//   GET /log.csv               the RAM log as a CSV download
//
// DEAD-MAN: if a joint was armed FROM THE PHONE and the phone stops polling
// for M_WEB_DEADMAN_MS, every armed joint is sent DISARM (not ESTOP: nothing
// is faulted, re-arming is allowed). Joints armed from USB are unaffected.
// ============================================================================

static WebServer wt_server(80);
static uint32_t  wt_last_poll_ms = 0;
static bool      wt_up = false;

// ---- JSON helpers -----------------------------------------------------------
// A console line may contain quotes; JSON needs them escaped. Control chars
// cannot occur (con_out strips CR and splits on LF) but are blanked anyway.
static void wtJsonStr(String& o, const char* s) {
  o += '"';
  for (; *s; s++) {
    const char c = *s;
    // INTENTIONAL backslashes (CLAUDE.md grep rule): each '\\' is ONE backslash,
    // the JSON escape character. Not a doubled-escape regression.
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  o += '"';
}

static bool wtAnyArmed() {
  bool any = false;
  portENTER_CRITICAL(&m_mux);
  for (uint8_t i = 0; i < M_N_NODES; i++) any |= m_node[i].session_armed;
  portEXIT_CRITICAL(&m_mux);
  return any;
}

// ---- handlers ---------------------------------------------------------------
static void wtPage() {
  wt_server.send_P(200, "text/html", WEB_PAGE);
}

static void wtPoll() {
  wt_last_poll_ms = millis();
  uint32_t since = wt_server.hasArg("since") ? (uint32_t)strtoul(wt_server.arg("since").c_str(), nullptr, 10) : 0;
  const uint32_t first = wlFirst();
  const uint32_t dropped = (since < first) ? first - since : 0;
  if (since < first) since = first;
  if (since > wl_next) since = first;       // page from before a reboot: resend from the oldest line
  const uint32_t MAX_LINES = 60;            // bound one response; the page asks again
  const uint32_t end = (wl_next - since > MAX_LINES) ? since + MAX_LINES : wl_next;

  String o;
  o.reserve(512 + (end - since) * 120);
  o += "{\"next\":"; o += end;
  o += ",\"more\":"; o += (end < wl_next) ? 1 : 0;
  o += ",\"dropped\":"; o += dropped;
  o += ",\"lines\":[";
  for (uint32_t s = since; s < end; s++) {
    if (s != since) o += ',';
    wtJsonStr(o, wlLine(s));
  }
  o += "],\"estop\":"; o += m_estop ? 1 : 0;
  o += ",\"log\":"; o += m_log_on ? 1 : 0;
  o += ",\"logn\":"; o += m_log_n;
  o += ",\"bus\":";
  { const BusHealth h = busHealth(); wtJsonStr(o, h.state); o += ",\"tec\":"; o += h.tec; }
  o += ",\"nodes\":[";
  for (uint8_t i = 0; i < M_N_NODES; i++) {
    portENTER_CRITICAL(&m_mux);
    const Node n = m_node[i];
    portEXIT_CRITICAL(&m_mux);
    char b[200];
    snprintf(b, sizeof(b),
      "%s{\"id\":%u,\"cmd\":%d,\"armed\":%d,\"ph\":\"%s\",\"fl\":\"%s\",\"p\":%.4f,\"v\":%.3f,"
      "\"tau\":%.4f,\"miss\":%lu,\"rep\":%lu,\"age\":%lu,\"t0\":%d}",
      i ? "," : "", n.id, n.session_armed ? 1 : 0, canStArmed(n.st_status) ? 1 : 0,
      phaseName(canStPhase(n.st_status)), faultName(canStFault(n.st_status)),
      n.st.p_rad, n.st.v_rads, n.st.tau_Nm,
      (unsigned long)n.missed, (unsigned long)n.replies,
      (unsigned long)(n.st_ms ? millis() - n.st_ms : 999999UL),
      (n.ident_ms && millis() - n.ident_ms < M_IDENT_FRESH_MS) ? 1 : 0);
    o += b;
  }
  o += "]}";
  wt_server.sendHeader("Cache-Control", "no-store");
  wt_server.send(200, "application/json", o);
}

static void wtCmd() {
  wt_last_poll_ms = millis();
  if (!wt_server.hasArg("c")) { wt_server.send(400, "text/plain", "missing c"); return; }
  String c = wt_server.arg("c");
  c.trim();
  if (!c.length()) { wt_server.send(200, "text/plain", "empty"); return; }
  if (!webQueuePush(c.c_str())) { wt_server.send(503, "text/plain", "busy"); return; }
  wt_server.send(200, "text/plain", "ok");
}

// STOP does not wait for the queue: it acts here, now.
static void wtStop() {
  m_estop = true;
  allSessionsOff();
  estopBroadcast();
  c_web_armed = false;
  con.println("** ESTOP from phone STOP button -- latched. `reset` to release (button closed).");
  wt_server.send(200, "text/plain", "stopped");
}

static void wtLogCsv() {
  m_log_on = false;                         // freeze the log while it is read
  wt_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  wt_server.sendHeader("Content-Disposition", "attachment; filename=\"leg_log.csv\"");
  wt_server.send(200, "text/csv", "");
  char b[160];
  snprintf(b, sizeof(b), "# CAN-T0 master log, %lu records, git %08lX%s, cycle %lu ms\n"
           "t_us,node,flags,armed,phase,clamp,fault,p_des,v_des,kp,kd,ff,p,v,tau\n",
           (unsigned long)m_log_n, (unsigned long)GIT_HASH, GIT_DIRTY ? "+dirty" : "",
           (unsigned long)M_CYCLE_MS);
  wt_server.sendContent(b);
  String chunk;
  chunk.reserve(2200);
  for (uint32_t k = 0; k < m_log_n; k++) {
    const LogRec& r = m_log[k];
    snprintf(b, sizeof(b), "%lu,%u,%u,%d,%u,%d,%u,%.5f,%.3f,%.1f,%.3f,%.3f,%.5f,%.4f,%.4f\n",
      (unsigned long)r.t_us, r.node, r.flags, canStArmed(r.status), canStPhase(r.status),
      canStClamped(r.status), canStFault(r.status), r.p_des, r.v_des, r.kp, r.kd, r.ff, r.p, r.v, r.tau);
    chunk += b;
    if (chunk.length() > 2000) { wt_server.sendContent(chunk); chunk = ""; }
  }
  chunk += "# end\n";
  wt_server.sendContent(chunk);
  wt_server.sendContent("");                // end of chunked response
  con.printf("log.csv sent to phone: %lu records\n", (unsigned long)m_log_n);
}

// ---- lifecycle --------------------------------------------------------------
static void webTermBegin(Print& out) {
  WiFi.mode(WIFI_AP);
  // Lower TX power: the phone is within a few metres, and smaller radio
  // current bursts mean less supply noise near the CAN transceiver.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  if (!WiFi.softAP(M_WIFI_SSID, M_WIFI_PASS, M_WIFI_CHANNEL, 0, 2)) {
    out.println("!! WiFi softAP FAILED -- phone terminal unavailable, USB console still works");
    return;
  }
  WiFi.setSleep(false);                     // no power-save latency on the terminal
  wt_server.on("/", HTTP_GET, wtPage);
  wt_server.on("/poll", HTTP_GET, wtPoll);
  wt_server.on("/cmd", HTTP_GET, wtCmd);
  wt_server.on("/stop", HTTP_GET, wtStop);
  wt_server.on("/log.csv", HTTP_GET, wtLogCsv);
  wt_server.onNotFound([]() { wt_server.send(404, "text/plain", "not found"); });
  wt_server.begin();
  wt_up = true;
  out.printf("phone terminal: join WiFi \"%s\" (password \"%s\"), open http://%s\n",
             M_WIFI_SSID, M_WIFI_PASS, WiFi.softAPIP().toString().c_str());
}

// Call every loop(), before conPoll() so a queued phone command runs promptly.
static void webTermService() {
  if (!wt_up) return;
  wt_server.handleClient();

  // Dead-man. Only armed-from-phone sessions are covered; cleared once
  // nothing is armed any more.
  if (c_web_armed) {
    if (!wtAnyArmed()) c_web_armed = false;
    else if (millis() - wt_last_poll_ms > M_WEB_DEADMAN_MS) {
      portENTER_CRITICAL(&m_mux);
      for (uint8_t i = 0; i < M_N_NODES; i++)
        if (m_node[i].session_armed) m_node[i].req = REQ_DISARM;
      portEXIT_CRITICAL(&m_mux);
      c_web_armed = false;
      con.printf("** phone silent for > %lu ms -> DISARM all (dead-man)\n",
                 (unsigned long)M_WEB_DEADMAN_MS);
    }
  }
}
