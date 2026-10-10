// ============================================================================
// tier0_main.cpp -- WIRING ONLY. Tier 0 joint firmware (CAN-T0).
// ============================================================================
// Built by the T0_Jxx envs only (platformio.ini [tier0] whitelists src/tier0/).
// The one file in this build that defines setup()/loop(). Everything else is
// in tier0.h and the shared headers it includes.
// ============================================================================
#include <Arduino.h>

// The two globals safety.h's stopMotor() reads from its includer (see the
// "TWO UPWARD DEPENDENCIES" note there). Defined here so the single disable
// path is used UNCHANGED -- changing its signature needs design review.
HardwareSerial SerialUART(PB4, PB3);   // (RX, TX) -- USART2, the only UART out
bool running = false;

#include "tier0.h"

void setup() { t0Setup(); }
void loop()  { t0Loop(); }
