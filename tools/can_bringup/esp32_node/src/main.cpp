/* ============================================================================
 * CAN TRANSPORT BRING-UP -- ESP32-S3 SIDE                rev 3  (2026-08-14)
 * ============================================================================
 * SCOPE: diagnostic instrument, NOT production structure. This deliberately
 *   violates "no logic in main.cpp" because it is a throwaway. Tier 1 gets
 *   built fresh once transport is proven. DO NOT GROW THIS INTO TIER 1.
 *
 * PURPOSE: prove the CAN physical + data-link layer with no motor and no
 *   SimpleFOC anywhere in the picture.
 *
 * STAGES (set STAGE below):
 *   0 = this ESP32 alone.  TWAI_MODE_NO_ACK. Endless clean frames on the wire.
 *   1 = ESP32 <-> ESP32, or ESP32 <-> ESC1.  TWAI_MODE_NORMAL, real ACK.
 *
 * ---------------------------------------------------------------------------
 * CHANGELOG vs rev 1 (the ESP32-TWAI-CAN version from 2026-08-05)
 *   1. DROPPED the ESP32-TWAI-CAN wrapper for the raw ESP-IDF TWAI driver.
 *      Reason: the wrapper does not expose the operating mode, and Stage 0
 *      REQUIRES no-ACK mode. See item 2.
 *   2. *** REV 2'S CORRECTION IS ITSELF RETRACTED. Rev 1 WAS RIGHT. ***
 *      Rev 1 said "endless retransmission is the CORRECT result" for a lone
 *      node. It is. Rev 2 replaced that with "TEC > 255, BUS-OFF within a few
 *      milliseconds", and that is wrong -- measured 2026-08-14 on the S1
 *      negative control (docs/CAN_BRINGUP.md 23.4b).
 *      A lone node climbs TEC to 128 in 16 frames (8 per ACK error), becomes
 *      ERROR-PASSIVE, and FREEZES there forever. CAN fault confinement does
 *      NOT increment TEC when an error-passive transmitter detects an ACK
 *      error and sees no dominant bit while sending its passive error flag.
 *      Bus-off needs TEC > 255, reachable only from bit/form errors -- so
 *      BUS-OFF FROM MISSING ACK ALONE IS IMPOSSIBLE. Measured: TEC 0 -> 128,
 *      frozen; state still RUNNING; recovered by itself when the partner came
 *      back, with no 'r' needed.
 *      NO_ACK is still the right Stage 0 instrument, but for a DIFFERENT
 *      reason than rev 2 gave: in NORMAL mode the analyzer would see ONE frame
 *      retransmitted at 7.7 kHz with an error flag on every attempt, instead
 *      of clean isolated frames at 200 Hz.
 *   3. Bus-off detection plus explicit recovery on 'r'. Rev 2 justified this
 *      with "Stage 1 with only one node powered will hit bus-off" -- retracted,
 *      see item 2; that configuration parks at error-passive instead. The watch
 *      is KEPT because bus-off is still reachable from bit and form errors --
 *      bit-rate mismatch, mis-wiring, noise -- which is exactly what S1c and S2
 *      are capable of producing.
 *   4. Boot banner prints the operating mode, both GPIOs, the bit rate, and
 *      the expected bit time in microseconds -- the number you will measure
 *      on the analyzer. A library default is a decision nobody made.
 *
 * CHANGELOG vs rev 2  (rev 3, 2026-08-14, after S1)
 *   5. ERROR-PASSIVE WATCH added to the periodic status block. twai_state_t
 *      has no error-passive value -- STOPPED / RUNNING / BUS_OFF / RECOVERING
 *      is the whole enum -- so the state field reads RUNNING the entire time a
 *      node is being ignored. TEC >= 128 is the ONLY tell, and rev 2 had no
 *      check for it because it wrongly expected bus-off instead.
 *   6. SINGLE-SHOT TX IS DELIBERATELY *NOT* ENABLED HERE. This sketch is the
 *      instrument that exhibits the retransmission behaviour; turning it off
 *      would remove the thing being measured. The CONTROL path is the opposite
 *      case -- Tier 0 / Tier 1 control frames MUST be single-shot (ESP32:
 *      twai_message_t.ss = 1; G431: DAR in FDCAN_CCCR), because a control frame
 *      that missed its slot is stale by the time it lands and acting on a stale
 *      {p_des, v_des, kp, kd, tau_ff} is worse than dropping it. Recorded as a
 *      requirement in docs/CAN_BRINGUP.md 23.4b. DO NOT copy this file's TX
 *      settings into Tier 1.
 *
 * HARDWARE: ESP32-S3 + SN65HVD230 breakout (3V3 native, no level shifting).
 *
 * BUILD: ONE copy of this file, TWO environments in platformio.ini. STAGE and
 *   NODE_TX_ID arrive as -D flags, so the two nodes are never separate edits.
 *     pio run -e nodeA -t upload     ->  STAGE=1, ID 0x100
 *     pio run -e nodeB -t upload     ->  STAGE=1, ID 0x101
 *     pio run -e solo  -t upload     ->  STAGE=0, ID 0x100  (S0 regression)
 *   See platformio.ini for the build_flags inheritance trap.
 * ========================================================================= */

#include <Arduino.h>
#include "driver/twai.h"

/* ---------------------------------------------------------------------------
 * CONFIG -- every load-bearing constant lives here and is echoed at boot.
 * ------------------------------------------------------------------------ */

// 0 = alone on the bus (NO_ACK).  1 = a partner exists (NORMAL).
// Set by platformio.ini build flags. The defaults below apply only if no flag
// is given, so a bare build still compiles as a Stage 0 node. Without the
// #ifndef guards these unconditional defines would come AFTER the command-line
// -D and silently override it (with a "macro redefined" warning nobody reads).
#ifndef STAGE
#define STAGE                 0
#endif
#ifndef NODE_TX_ID
#define NODE_TX_ID            0x100
#endif

// GPIO choice on the ESP32-S3-DevKitC-1 (N16R8).
//   AVOID: 0/3/45/46 (strapping), 19/20 (native USB), 26..32 (SPI flash),
//   33..37 (octal PSRAM on the R8 part -- this is the one people forget).
static const gpio_num_t PIN_CAN_TX   = GPIO_NUM_5;   // -> transceiver TXD / D / CTX
static const gpio_num_t PIN_CAN_RX   = GPIO_NUM_4;   // <- transceiver RXD / R / CRX

static const uint32_t   TX_PERIOD_MS = 5;            // 200 Hz. Slow enough to
                                                     // find frames by eye in a
                                                     // 41 ms analyzer window.
static const uint32_t   TX_ID        = NODE_TX_ID;   // 11-bit standard ID
static const uint32_t   STATUS_MS    = 1000;

/* ------------------------------------------------------------------------ */

/* --- 1. move the mode selection to file scope, above setup() --- */
#if STAGE == 0
static const twai_mode_t MODE      = TWAI_MODE_NO_ACK;
static const char*       MODE_NAME = "NO_ACK (Stage 0: alone, no partner needed)";
#else
static const twai_mode_t MODE      = TWAI_MODE_NORMAL;
// The old text read "a partner MUST answer or we go bus-off". That is the rev 2
// claim retracted in item 2 -- an unanswered node parks at error-passive
// (TEC = 128) and retransmits forever. Corrected here because the banner is the
// most-read line in the file and a wrong banner outlives a wrong comment.
static const char*       MODE_NAME = "NORMAL (Stage 1/2: a partner must ACK, or TEC parks at 128 = error-passive)";
#endif

static uint32_t tx_count = 0, rx_count = 0;
static uint32_t last_tx = 0, last_status = 0;
static bool     tx_enabled = true;

// Receive-side instrumentation. dump_remaining starts at 3 so each node prints
// its first three received frames for an eyeball check, then goes quiet.
static uint32_t rx_bad = 0, last_rx_id = 0, dump_remaining = 3;

// The partner sends data[i] = tx_count + i, so the eight bytes must ascend by
// exactly one (mod 256). Checking that here turns "frames are arriving" into
// "frames are arriving uncorrupted" without anyone reading hex off a terminal.
static bool payloadOK(const twai_message_t& m) {
  if (m.data_length_code != 8) return false;
  for (int i = 1; i < 8; i++)
    if ((uint8_t)(m.data[i] - m.data[i - 1]) != 1) return false;
  return true;
}

static const char* stateName(twai_state_t s) {
  switch (s) {
    case TWAI_STATE_STOPPED:    return "STOPPED";
    case TWAI_STATE_RUNNING:    return "RUNNING";
    case TWAI_STATE_BUS_OFF:    return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
  }
  return "?";
}

/* --- 2. extract the banner so it is retrievable without a reflash --- */
static void banner() {
  Serial.println();
  Serial.println("==== CAN BRING-UP  ESP32-S3  rev3 ====");
  Serial.printf("  mode      : %s\n", MODE_NAME);
  Serial.printf("  TX gpio   : %d      RX gpio : %d\n", (int)PIN_CAN_TX, (int)PIN_CAN_RX);
  Serial.println("  bit rate  : 1.000 Mbit/s  -> ONE BIT = 1.000 us");
  Serial.printf("  tx period : %lu ms   ID = 0x%03lX, 8 data bytes\n",
                (unsigned long)TX_PERIOD_MS, (unsigned long)TX_ID);
  Serial.println("  keys      : s=status  p=pause TX  r=recover  b=banner  d=dump 3 frames");
  Serial.println("======================================");
}

static void printStatus() {
  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK) { Serial.println("status read FAILED"); return; }
  Serial.printf("[st] %s  TEC=%lu REC=%lu  qtx=%lu qrx=%lu  "
                "txfail=%lu rxmiss=%lu buserr=%lu arblost=%lu  | sent=%lu recv=%lu "
                "rxbad=%lu lastid=0x%03lX\n",
                stateName(st.state),
                (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter,
                (unsigned long)st.msgs_to_tx,       (unsigned long)st.msgs_to_rx,
                (unsigned long)st.tx_failed_count,  (unsigned long)st.rx_missed_count,
                (unsigned long)st.bus_error_count,  (unsigned long)st.arb_lost_count,
                (unsigned long)tx_count, (unsigned long)rx_count,
                (unsigned long)rx_bad,    (unsigned long)last_rx_id);
}

void setup() {
  Serial.begin(921600);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);   // wait for the host
  delay(1200);                                          // let the monitor settle

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, MODE);
  g.tx_queue_len = 8;
  g.rx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  banner();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("!! driver_install FAILED"); return; }
  if (twai_start() != ESP_OK)                    { Serial.println("!! twai_start FAILED");     return; }
  Serial.println("driver installed and started.");
  printStatus();
}

void loop() {
  // ---- serial keys -------------------------------------------------------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') printStatus();
    else if (c == 'p') { tx_enabled = !tx_enabled;
                         Serial.printf("TX %s\n", tx_enabled ? "RESUMED" : "PAUSED"); }
    else if (c == 'r') { Serial.println("initiating bus-off recovery...");
                         twai_initiate_recovery(); }
    else if (c == 'b') banner();
    else if (c == 'd') { dump_remaining = 3; Serial.println("dumping next 3 frames"); }
  }

  // ---- transmit ----------------------------------------------------------
  uint32_t now = millis();
  if (tx_enabled && now - last_tx >= TX_PERIOD_MS) {
    last_tx = now;
    twai_message_t m = {};
    m.identifier       = TX_ID;
    m.data_length_code = 8;
    m.extd = 0; m.rtr = 0;
    for (int i = 0; i < 8; i++) m.data[i] = (uint8_t)(tx_count + i);
    if (twai_transmit(&m, 0) == ESP_OK) tx_count++;
  }

  // ---- receive (non-blocking drain) --------------------------------------
  twai_message_t r;
  while (twai_receive(&r, 0) == ESP_OK) {
    rx_count++;
    // NOT cosmetic. Printing every frame is 200 Serial.printf/s; USB CDC writes
    // block when the host buffer is full, that stalls loop(), and the 16-deep
    // rx queue overflows. rxmiss would then climb and read as a bus fault when
    // it is really a printing fault.
    if (!payloadOK(r)) rx_bad++;
    last_rx_id = r.identifier;
    if (dump_remaining) {
      dump_remaining--;
      Serial.printf("[rx] id=0x%03lX dlc=%u data=", (unsigned long)r.identifier, r.data_length_code);
      for (int i = 0; i < r.data_length_code; i++) Serial.printf("%02X ", r.data[i]);
      Serial.println();
    }
  }

  // ---- periodic status + bus-off watch -----------------------------------
  if (now - last_status >= STATUS_MS) {
    last_status = now;
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
      if (st.state == TWAI_STATE_BUS_OFF) {
        Serial.println("!! BUS_OFF -- this is NOT a missing partner. A missing ACK parks at");
        Serial.println("   TEC=128 and never reaches bus-off. TEC>255 means BIT or FORM");
        Serial.println("   errors: bit-rate mismatch, mis-wiring, or noise. Press 'r'.");
      } else if (st.state == TWAI_STATE_STOPPED) {
        Serial.println("recovery complete, restarting...");
        twai_start();
      } else if (st.tx_error_counter >= 128) {
        // Ordered AFTER the STOPPED branch deliberately: a lingering TEC must
        // never shadow the auto-restart above. Reached only while RUNNING.
        // TEC >= 128 is the ONLY way to observe error-passive -- twai_state_t
        // has no value for it, so st.state reads RUNNING throughout.
        Serial.println("!! ERROR-PASSIVE (TEC>=128) -- nobody is acknowledging us. The state");
        Serial.println("   field still reads RUNNING; TEC is the only tell. One frame is");
        Serial.println("   being retransmitted continuously (7.7 kHz measured alone on the");
        Serial.println("   bus). Check the partner -- this self-heals when it returns.");
      }
    }
    printStatus();
  }
}