#include <Arduino.h>
#include <SimpleFOC.h>

// ============================================================================
// ACTUATOR BASELINE — all proven functionality consolidated.
// Board: B-G431B-ESC1 clone (EG2124A). SimpleFOC pinned 2.3.1, platform 17.6.0.
// Console: USART2 PB3(TX)->VCP RX, PB4(RX)->VCP TX.  Encoder: ABZ PB6/PB7, no Z.
//
// COMMANDS (serial monitor):
//   g = go (enable)          x/s = stop (disable, FETs off)
//   + / - = adjust target    (1.0 rad/s in velocity modes, 0.1 V in torque)
//   o = open-loop velocity mode (boot default, proven)
//   f = run initFOC once per power-up (belt OFF; expect a small twitch)
//   t = closed-loop torque (voltage) mode   [requires f]
//   v = closed-loop velocity mode           [requires f]
//   ? = help
// Mode changes only while stopped. Boots DISABLED.
// ============================================================================

HardwareSerial SerialUART(PB4, PB3);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);
// Clone sense chain is gain-compensated -> genuine constants. Do not change.
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

Encoder encoder = Encoder(PB6, PB7, 1024);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// ---- safety / tuning constants (bench standard) ----
const float VOLT_LIMIT      = 2.0f;    // ~7 A into these windings; raise deliberately, never casually
const float VEL_MAX         = 20.0f;   // rad/s
const float VEL_STEP        = 1.0f;    // rad/s per keypress
const float TORQUE_STEP     = 0.02f;    // volts per keypress (torque-voltage mode)
const float TORQUE_MAX      = 1.0f;    // clamp for torque target
const unsigned long AUTO_STOP_MS = 20000;

// conservative starting gains for 4096 CPR encoder (tune in the initFOC sessions)
const float VEL_P  = 3.0f;
const float VEL_I  = 0.0f;
const float VEL_D  = 0.0f;
const float VEL_TF = 0.02f;            // velocity LPF; larger = smoother, laggier

enum Mode { MODE_OPENLOOP, MODE_TORQUE, MODE_VELOCITY };
Mode mode = MODE_OPENLOOP;
bool foc_ready = false;                // set true after successful initFOC
bool running = false;
float target = 2.0f;                   // rad/s in velocity modes, V in torque mode

bool driver_ok=false, cs_ok=false;
unsigned long run_started=0, last_blink=0, last_print=0;
bool led_state=false;

const char* modeName() {
  switch (mode) {
    case MODE_OPENLOOP: return "OPENLOOP";
    case MODE_TORQUE:   return "TORQUE(V)";
    case MODE_VELOCITY: return "VELOCITY";
  }
  return "?";
}

void printHelp() {
  SerialUART.println(F("--- g:go x:stop +/-:target o:openloop f:initFOC once per power-up t:torque v:velocity ?:help ---"));
}

void startMotor() {
  if (!driver_ok) { SerialUART.println(F("refused: driver init failed")); return; }
  if ((mode == MODE_TORQUE || mode == MODE_VELOCITY) && !foc_ready) {
    SerialUART.println(F("refused: run initFOC (f) first")); return;
  }
  motor.enable();
  running = true; run_started = millis();
  SerialUART.print(F("RUNNING ")); SerialUART.print(modeName());
  SerialUART.print(F(" target=")); SerialUART.println(target);
}

void stopMotor(const char* reason) {
  motor.disable(); running = false;
  SerialUART.print(F("STOPPED (")); SerialUART.print(reason); SerialUART.println(F(")"));
}

void setMode(Mode m) {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  if ((m == MODE_TORQUE || m == MODE_VELOCITY) && !foc_ready) {
    SerialUART.println(F("run initFOC (f) first")); return;
  }
  mode = m;
  switch (mode) {
    case MODE_OPENLOOP:
      motor.controller = MotionControlType::velocity_openloop;
      target = 2.0f; break;
    case MODE_TORQUE:
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::torque;
      target = 0.0f; break;                       // start at zero torque
    case MODE_VELOCITY:
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::velocity;
      target = 2.0f; break;
  }
  SerialUART.print(F("mode=")); SerialUART.print(modeName());
  SerialUART.print(F(" target=")); SerialUART.println(target);
}

void runInitFOC() {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  SerialUART.println(F("initFOC: aligning (expect a small twitch). Belt OFF for first run."));
  // force a FRESH offset calibration on every 'f' (direction is persistent)
  motor.zero_electric_angle = NOT_SET;
  motor.enable();                                // alignment needs a live driver
  int ok = motor.initFOC();
  motor.disable();                               // move the existing disable to immediately after
  if (ok) {
    foc_ready = true;
    SerialUART.println(F("initFOC SUCCESS"));
    // print alignment results so they can be hardcoded for instant future boots:
    //   motor.zero_electric_angle = <ZEA>; motor.sensor_direction = <DIR>;
    SerialUART.print(F("zero_electric_angle=")); SerialUART.println(motor.zero_electric_angle, 4);
    SerialUART.print(F("sensor_direction=")); SerialUART.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
  } else {
    foc_ready = false;
    SerialUART.println(F("initFOC FAILED (check magnet/encoder, free the shaft)"));
  }
  motor.disable();   // stay safe after alignment
}

void adjustTarget(float dir) {
  if (mode == MODE_TORQUE) {
    target = constrain(target + dir * TORQUE_STEP, -TORQUE_MAX, TORQUE_MAX);
  } else {
    target = constrain(target + dir * VEL_STEP, -VEL_MAX, VEL_MAX);
  }
  SerialUART.print(F("target=")); SerialUART.println(target);
}

void handleSerial() {
  while (SerialUART.available()) {
    char c = (char)SerialUART.read();
    switch (c) {
      case 'g': case 'G': startMotor(); break;
      case 'x': case 'X': case 's': case 'S': stopMotor("user"); break;
      case '+': case '=': adjustTarget(+1); break;
      case '-': case '_': adjustTarget(-1); break;
      case 'o': case 'O': setMode(MODE_OPENLOOP); break;
      case 't': case 'T': setMode(MODE_TORQUE); break;
      case 'v': case 'V': setMode(MODE_VELOCITY); break;
      case 'f': case 'F': runInitFOC(); break;
      case '?': printHelp(); break;
      default: break;
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // PC6 (STATUS)
  SerialUART.begin(115200);
  _delay(2000);
  SimpleFOCDebug::enable(&SerialUART);
  SerialUART.println(F("=== actuator baseline ==="));

  driver.voltage_power_supply = 11.4f;
  driver.voltage_limit = 6.0f;
  driver.dead_zone = 0.05f;                 // EG2124A insurance
  driver_ok = driver.init();
  SerialUART.println(driver_ok ? F("driver OK") : F("driver FAILED"));

  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);
  cs_ok = currentSense.init();
  SerialUART.println(cs_ok ? F("currentSense OK") : F("currentSense FAILED"));

  encoder.quadrature = Quadrature::ON;      // 4096 CPR
  encoder.pullup     = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);
  SerialUART.println(F("encoder OK"));

  // control config
  motor.controller = MotionControlType::velocity_openloop;   // proven boot mode
  motor.voltage_limit = VOLT_LIMIT;
  motor.velocity_limit = VEL_MAX;
  motor.PID_velocity.P = VEL_P;
  motor.PID_velocity.I = VEL_I;
  motor.PID_velocity.D = VEL_D;
  motor.LPF_velocity.Tf = VEL_TF;
  
  // alignment voltage kept modest (default 3V is a lot at 0.3 ohm):
  motor.voltage_sensor_align = 2.0f;
  motor.init();
  motor.disable();                          // SAFE BOOT (unchanged)
  motor.sensor_direction = Direction::CCW;  // persistent: keep
  // NOTE: zero_electric_angle is session-relative with an incremental ABZ
  // encoder (no index) -> NEVER hardcode. Run 'f' once per power-up.
  foc_ready = false;
  SerialUART.println(F("Motor DISABLED. o=openloop(default) f=initFOC once per power-up t=torque v=velocity"));
  printHelp();
}

void loop() {
  handleSerial();

  // heartbeat: slow = init OK, fast = something failed
  unsigned long period = (driver_ok && cs_ok) ? 500 : 80;
  if (millis() - last_blink > period) { led_state=!led_state; digitalWrite(LED_BUILTIN, led_state); last_blink=millis(); }

  // FOC inner loop (no-op in open-loop mode); keeps sensor updated when closed-loop
  motor.loopFOC();
  if (!foc_ready || mode == MODE_OPENLOOP) encoder.update();   // manual sensor update outside FOC path

  if (running) {
    if (millis() - run_started > AUTO_STOP_MS) stopMotor("auto 20s");
    else motor.move(target);
  }

  if (millis() - last_print > 150) {          // was 300 — finer position resolution
    PhaseCurrent_s c = currentSense.getPhaseCurrents();
    float mag = sqrtf(c.a*c.a + c.b*c.b + c.c*c.c);
    SerialUART.print(F("m=")); SerialUART.print(modeName());
    SerialUART.print(F(" run=")); SerialUART.print(running?1:0);
    SerialUART.print(F(" tgt=")); SerialUART.print(target, 2);
    SerialUART.print(F(" enc_a=")); SerialUART.print(encoder.getAngle(), 3);
    SerialUART.print(F(" vel=")); SerialUART.print(motor.shaft_velocity, 2);   // filtered — what the PID sees
    SerialUART.print(F(" Uq=")); SerialUART.print(motor.voltage.q, 3);         // controller output (saturation check)
    SerialUART.print(F(" |I|=")); SerialUART.println(mag, 2);
    last_print = millis();
  }
}