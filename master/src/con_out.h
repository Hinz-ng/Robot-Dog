#pragma once
#include <Arduino.h>
// ============================================================================
// con_out.h -- every console line goes to BOTH the USB serial and a line ring
// the phone terminal reads (web_term.h). (CAN-T0 web terminal, 2026-10-04)
// ============================================================================
// WHY A TEE: the USB-CDC console has been unreliable at the bench (and on the
// humanoid). Rather than choose, everything printed through `con` lands in
// both places, so either one can be used and they never disagree.
//
// SINGLE-WRITER, BY DESIGN: only loop() (core 1) prints -- the console, the
// event drain, and the web handlers all run there. The poll task never prints
// (it queues events). So the ring needs no lock.
//
// Lines longer than WL_W-1 are wrapped, not truncated. The ring keeps the last
// WL_N lines; a phone that falls further behind is told how many it missed.
// ============================================================================

static const uint16_t WL_N = 200;          // lines kept for the phone
static const uint16_t WL_W = 224;          // chars per line incl. NUL (status rows ~200)

static char     wl_buf[WL_N][WL_W];
static uint32_t wl_next = 0;               // sequence number of the next line to commit
static char     wl_cur[WL_W];
static uint16_t wl_cur_n = 0;

static void wlCommit() {
  wl_cur[wl_cur_n] = 0;
  memcpy(wl_buf[wl_next % WL_N], wl_cur, wl_cur_n + 1);
  wl_next++;
  wl_cur_n = 0;
}

// Oldest sequence number still in the ring.
static inline uint32_t wlFirst() { return (wl_next > WL_N) ? wl_next - WL_N : 0; }
static inline const char* wlLine(uint32_t seq) { return wl_buf[seq % WL_N]; }

class ConOut : public Print {
public:
  size_t write(uint8_t c) override {
    Serial.write(c);                       // HWCDC with TX timeout 0: drops, never blocks
    feed(c);
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
    for (size_t i = 0; i < n; i++) feed(b[i]);
    return n;
  }
  using Print::write;

  // Feed the phone ring only (used for echoes and notes that need not go to USB).
  void webOnly(const char* s) { while (*s) feed((uint8_t)*s++); }

private:
  static void feed(uint8_t c) {
    if (c == '\r') return;
    if (c == '\n') { wlCommit(); return; }
    if (wl_cur_n >= WL_W - 1) wlCommit();  // wrap a long line
    wl_cur[wl_cur_n++] = (char)c;
  }
};

static ConOut con;

// ---------------------------------------------------------------------------
// Command lines from the phone. The HTTP handler (loop()) pushes, conPoll()
// (loop(), just after) pops and executes -- same task, so commands from the
// phone and from USB run in exactly the same place, one at a time.
// ---------------------------------------------------------------------------
static const uint8_t WQ_N = 8, WQ_W = 96;
static char    wq_buf[WQ_N][WQ_W];
static uint8_t wq_head = 0, wq_tail = 0;

static bool webQueuePush(const char* s) {
  const uint8_t n = (uint8_t)((wq_head + 1) % WQ_N);
  if (n == wq_tail) return false;          // full: the phone retries
  strncpy(wq_buf[wq_head], s, WQ_W - 1);
  wq_buf[wq_head][WQ_W - 1] = 0;
  wq_head = n;
  return true;
}
static bool webQueuePop(char* out) {
  if (wq_tail == wq_head) return false;
  memcpy(out, wq_buf[wq_tail], WQ_W);
  wq_tail = (uint8_t)((wq_tail + 1) % WQ_N);
  return true;
}
