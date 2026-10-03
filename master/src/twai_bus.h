#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include "master_config.h"
// ============================================================================
// twai_bus.h -- raw ESP-IDF TWAI, NORMAL mode, 1 Mbit/s. (CAN-T0)
// ============================================================================
// The raw driver, as at S0-S2 (the ESP32-TWAI-CAN wrapper hides the mode and
// the single-shot flag). Control frames go out SINGLE-SHOT (ss = 1, CAN_BRINGUP
// 23.4b): a late command is stale, and on a polled bus a dropped frame shows up
// as a missed reply, which is counted. ESTOP is the one exception -- it is never
// stale, so it may retransmit.
// ============================================================================

static bool m_bus_up = false;
static uint32_t m_tx_fail = 0;

static bool busInit(Print& out) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(M_PIN_CAN_TX, M_PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 8;
  g.rx_queue_len = 32;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  // Accept all; the poll engine discards anything that is not the reply it is
  // waiting for. Only joints transmit to us, and only when asked.
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) { out.println("!! TWAI driver_install FAILED"); return false; }
  if (twai_start() != ESP_OK)                    { out.println("!! TWAI start FAILED");          return false; }
  m_bus_up = true;
  return true;
}

static bool busSend(uint16_t id, const uint8_t* data, uint8_t dlc, bool single_shot = true) {
  twai_message_t m = {};
  m.identifier = id;
  m.data_length_code = dlc;
  m.ss = single_shot ? 1 : 0;
  for (uint8_t i = 0; i < dlc; i++) m.data[i] = data[i];
  if (twai_transmit(&m, 0) != ESP_OK) { m_tx_fail++; return false; }
  return true;
}

// Wait up to timeout_us for a frame with exactly this id. Other frames are
// dropped (counted). Busy-waits: the poll task owns core 0 for ~1 ms per cycle.
static uint32_t m_rx_stray = 0;
static bool busWaitFor(uint16_t id, uint8_t out[8], uint8_t& dlc, uint32_t timeout_us) {
  const uint32_t t0 = micros();
  twai_message_t m;
  do {
    while (twai_receive(&m, 0) == ESP_OK) {
      if (m.identifier == id && !m.extd && !m.rtr) {
        dlc = m.data_length_code;
        for (uint8_t i = 0; i < 8; i++) out[i] = (i < dlc) ? m.data[i] : 0;
        return true;
      }
      m_rx_stray++;
    }
  } while (micros() - t0 < timeout_us);
  return false;
}

// Drain anything late (a reply that missed its window) so it cannot be taken
// for the NEXT exchange's answer.
static void busDrain() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) m_rx_stray++;
}

struct BusHealth { const char* state; uint32_t tec, rec, buserr, arblost, txfail, rxmiss; };
static BusHealth busHealth() {
  BusHealth h = { "?", 0, 0, 0, 0, 0, 0 };
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return h;
  switch (st.state) {
    case TWAI_STATE_STOPPED:    h.state = "STOPPED";    break;
    case TWAI_STATE_RUNNING:    h.state = (st.tx_error_counter >= 128) ? "ERR-PASSIVE" : "RUNNING"; break;
    case TWAI_STATE_BUS_OFF:    h.state = "BUS_OFF";    break;
    case TWAI_STATE_RECOVERING: h.state = "RECOVERING"; break;
  }
  h.tec = st.tx_error_counter; h.rec = st.rx_error_counter;
  h.buserr = st.bus_error_count; h.arblost = st.arb_lost_count;
  h.txfail = st.tx_failed_count; h.rxmiss = st.rx_missed_count;
  return h;
}

// Bus-off is reachable only from bit/form errors (23.4b) -- wiring or a
// babbler, never a missing partner. Recover, but it is a finding.
static uint32_t m_busoff_n = 0;
static void busService() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) return;
  if (st.state == TWAI_STATE_BUS_OFF)      { m_busoff_n++; twai_initiate_recovery(); }
  else if (st.state == TWAI_STATE_STOPPED) { twai_start(); }
}
