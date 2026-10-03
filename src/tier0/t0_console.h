#pragma once
#include <Arduino.h>
// ============================================================================
// t0_console.h -- NON-BLOCKING console output for Tier 0. (CAN-T0, 2026-10-03)
// ============================================================================
// WHY: STM32duino's HardwareSerial TX buffer is 64 bytes, so a status line
// BLOCKS once it fills -- the harness measured pr_us = 1.2-1.8 ms per line.
// In Tier 0 that stall would (a) miss the master's 500 us reply window, so a
// 2 Hz status print alone would fail N1's "0 missed replies", and (b) stop the
// FOC loop for 1.5 ms. So text goes into this RAM ring and conPump() moves at
// most availableForWrite() bytes per loop: HardwareSerial::write() then never
// waits.
//
// RULE (owner addition 2): NOTHING IS QUEUED WHILE ARMED -- tier0.h sets
// `enabled = !running` every loop. STATE/STATUS carry everything over CAN.
// The pump keeps draining what was queued before arming; that is bounded by
// availableForWrite() and cannot block. stopMotor()'s own "STOPPED" line goes
// straight to SerialUART (safety.h is unchanged) and prints with running
// already false.
// ============================================================================

class T0ConOut : public Print {
public:
  bool     enabled = true;
  uint32_t dropped = 0;           // bytes refused: ring full, or armed

  size_t write(uint8_t b) override {
    if (!enabled) { dropped++; return 1; }
    const uint16_t n = (uint16_t)((head_ + 1) % N);
    if (n == tail_) { dropped++; return 1; }
    buf_[head_] = b; head_ = n;
    return 1;
  }
  using Print::write;

  // Never blocks: writes only what the UART can take right now.
  void pump(HardwareSerial& s) {
    int room = s.availableForWrite();
    while (room-- > 0 && tail_ != head_) {
      s.write(buf_[tail_]);
      tail_ = (uint16_t)((tail_ + 1) % N);
    }
  }
  bool empty() const { return head_ == tail_; }

private:
  static const uint16_t N = 1024;
  uint8_t  buf_[N];
  uint16_t head_ = 0, tail_ = 0;
};
