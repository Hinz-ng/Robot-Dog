// ============================================================================
// main.cpp -- WIRING ONLY. ESP32-S3 leg master for CAN-T0.
// ============================================================================
// The bus schedule runs in its own task on core 0 (poll.h), so nothing the
// console prints on core 1 can stretch a cycle. See master/platformio.ini.
// ============================================================================
#include <Arduino.h>
#include "master_config.h"
#include "twai_bus.h"
#include "nodes.h"
#include "poll.h"
#include "console.h"

void setup() {
  Serial.begin(921600);
  Serial.setTxTimeoutMs(0);                 // never block on a host that is not reading
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(300);

  Serial.printf("\n=== LEG MASTER (CAN-T0)  contract v%u  git %08lX%s ===\n",
                CAN_CONTRACT_VERSION, (unsigned long)GIT_HASH, GIT_DIRTY ? " DIRTY" : "");
  Serial.printf("TWAI tx GPIO%d rx GPIO%d, 1 Mbit/s NORMAL, single-shot; cycle %lu ms; nodes:",
                (int)M_PIN_CAN_TX, (int)M_PIN_CAN_RX, (unsigned long)M_CYCLE_MS);
  for (uint8_t i = 0; i < M_N_NODES; i++) Serial.printf(" J%02u", M_NODES[i]);
  Serial.println("\nBOOTS POLL-ONLY: no node is commanded until `arm` in THIS session.");
  selfTest();

  pinMode(M_PIN_ESTOP, INPUT_PULLUP);
  nodesInit();
  logAlloc(Serial);
  if (!busInit(Serial)) { Serial.println("!! no bus -- halted"); for (;;) delay(1000); }
  xTaskCreatePinnedToCore(pollTask, "poll", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 0);
  Serial.println("type `help`");
}

void loop() {
  conPoll();
  delay(1);
}
