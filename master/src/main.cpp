// ============================================================================
// main.cpp -- WIRING ONLY. ESP32-S3 leg master for CAN-T0.
// ============================================================================
// The bus schedule runs in its own task (poll.h) pinned to CORE 1: the WiFi
// stack for the phone terminal lives on core 0, and radio activity there must
// not stretch a 200 Hz cycle or eat a 500 us reply window. The console and
// the phone terminal both run in loop() (also core 1, lowest priority), so
// the poll task always pre-empts them.
// ============================================================================
#include <Arduino.h>
#include "master_config.h"
#include "twai_bus.h"
#include "nodes.h"
#include "poll.h"
#include "con_out.h"
#include "console.h"
#include "web_term.h"

void setup() {
  Serial.begin(921600);
  Serial.setTxTimeoutMs(0);                 // never block on a host that is not reading
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);   // short: the phone is the primary console now

  con.printf("\n=== LEG MASTER (CAN-T0)  contract v%u  git %08lX%s ===\n",
             CAN_CONTRACT_VERSION, (unsigned long)GIT_HASH, GIT_DIRTY ? " DIRTY" : "");
  con.printf("TWAI tx GPIO%d rx GPIO%d, 1 Mbit/s NORMAL, single-shot; cycle %lu ms; nodes:",
             (int)M_PIN_CAN_TX, (int)M_PIN_CAN_RX, (unsigned long)M_CYCLE_MS);
  for (uint8_t i = 0; i < M_N_NODES; i++) con.printf(" J%02u", M_NODES[i]);
  con.println("\nBOOTS POLL-ONLY: no node is commanded until `arm` in THIS session.");
  selfTest();

  pinMode(M_PIN_ESTOP, INPUT_PULLUP);
  nodesInit();
  logAlloc(con);
  if (!busInit(con)) { con.println("!! no bus -- halted"); for (;;) delay(1000); }
  xTaskCreatePinnedToCore(pollTask, "poll", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
  webTermBegin(con);
  con.println("type `help`");
}

void loop() {
  webTermService();
  conPoll();
  delay(1);
}
