#include <Arduino.h>
#include <SimpleFOC.h>

// Serial over USART2: TXD2=PB3 -> STLink VCP RX, RXD2=PB4 <- STLink VCP TX
HardwareSerial SerialUART(PB4, PB3);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);

// CORRECTED for this clone board: 20 mOhm shunts (R020), not the genuine 3 mOhm.
// Gain kept from the genuine design assumption; if absolute values look off by a
// constant factor later, the amp gain is the remaining unknown to calibrate.
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// ---- test parameters ----
float target_velocity = 2.0f;        // rad/s
const float VEL_STEP   = 1.0f;       // rad/s per +/- keypress
const float VEL_MAX    = 20.0f;
const unsigned long AUTO_STOP_MS = 30000;  // auto-disable after 30 s of running (burn insurance)

bool driver_ok = false, cs_ok = false;
bool running = false;                // motor state, boots DISABLED
unsigned long run_started = 0;
unsigned long last_blink = 0, last_print = 0;
bool led_state = false;

void printHelp() {
  SerialUART.println(F("--- commands ---"));
  SerialUART.println(F(" g : go (enable + spin)"));
  SerialUART.println(F(" x : stop (disable, FETs off)"));
  SerialUART.println(F(" + : velocity +1 rad/s"));
  SerialUART.println(F(" - : velocity -1 rad/s"));
  SerialUART.println(F(" ? : this help"));
  SerialUART.println(F("----------------"));
}

void startMotor() {
  if (!driver_ok) { SerialUART.println(F("refused: driver init failed")); return; }
  motor.enable();
  running = true;
  run_started = millis();
  SerialUART.print(F("RUNNING  target=")); SerialUART.print(target_velocity); SerialUART.println(F(" rad/s"));
}

void stopMotor(const char* reason) {
  motor.disable();                   // all FETs off -> coast
  running = false;
  SerialUART.print(F("STOPPED (")); SerialUART.print(reason); SerialUART.println(F(")"));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  SerialUART.begin(115200);
  _delay(2000);
  SimpleFOCDebug::enable(&SerialUART);

  SerialUART.println(F("=== boot (safe open-loop + current, keyboard ctrl) ==="));

  driver.voltage_power_supply = 11.4f;
  driver.voltage_limit = 6.0f;
  driver.dead_zone = 0.05f;          // widened dead-time: insurance for EG2124A interlock,
                                     // and the direct test of the shoot-through theory
  driver_ok = driver.init();
  SerialUART.println(driver_ok ? F("driver.init() -> SUCCESS") : F("driver.init() -> FAILED"));

  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);
  cs_ok = currentSense.init();
  SerialUART.println(cs_ok ? F("currentSense.init() -> SUCCESS") : F("currentSense.init() -> FAILED"));

  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 1.0f;        // ~3 A peak into these windings; enough to spin, not a heater
  motor.velocity_limit = VEL_MAX;
  motor.init();

  motor.disable();                   // SAFE BOOT: no drive until 'g'
  running = false;

  SerialUART.println(F("setup complete. Motor DISABLED."));
  printHelp();
}

void handleSerial() {
  while (SerialUART.available()) {
    char c = (char)SerialUART.read();
    switch (c) {
      case 'g': case 'G': startMotor(); break;
      case 'x': case 'X': case 's': case 'S': stopMotor("user"); break;
      case '+': case '=':
        target_velocity = min(target_velocity + VEL_STEP, VEL_MAX);
        SerialUART.print(F("target=")); SerialUART.println(target_velocity);
        break;
      case '-': case '_':
        target_velocity = max(target_velocity - VEL_STEP, 0.0f);
        SerialUART.print(F("target=")); SerialUART.println(target_velocity);
        break;
      case '?': printHelp(); break;
      default: break;                // ignore newlines etc.
    }
  }
}

void loop() {
  handleSerial();

  // heartbeat: slow = healthy, fast = init failed
  unsigned long period = (driver_ok && cs_ok) ? 500 : 80;
  if (millis() - last_blink > period) {
    led_state = !led_state; digitalWrite(LED_BUILTIN, led_state); last_blink = millis();
  }

  if (running) {
    // burn insurance: bounded run time, restart with 'g'
    if (millis() - run_started > AUTO_STOP_MS) {
      stopMotor("auto 30s");
    } else {
      motor.move(target_velocity);
    }
  }

  if (millis() - last_print > 300) {
    PhaseCurrent_s c = currentSense.getPhaseCurrents();
    float mag = sqrtf(c.a*c.a + c.b*c.b + c.c*c.c);
    SerialUART.print(F("run=")); SerialUART.print(running ? 1 : 0);
    SerialUART.print(F(" angle=")); SerialUART.print(motor.shaft_angle, 2);
    SerialUART.print(F("  Ia=")); SerialUART.print(c.a, 2);
    SerialUART.print(F(" Ib=")); SerialUART.print(c.b, 2);
    SerialUART.print(F(" Ic=")); SerialUART.print(c.c, 2);
    SerialUART.print(F("  |I|=")); SerialUART.println(mag, 2);
    last_print = millis();
  }
}