#include <Arduino.h>
#include <SimpleFOC.h>

// --- Motor: TYI 4006, 14 rotor poles -> 7 pole pairs ---
BLDCMotor motor = BLDCMotor(7);

// --- Driver: 6-PWM, using B-G431B-ESC1 board pin macros ---
// (provided by the disco_b_g431b_esc1 board variant)
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);

// Commanded open-loop speed (rad/s). 5 rad/s ~= 48 rpm: slow, clearly visible.
float target_velocity = 5.0f;

void setup() {
  Serial.begin(115200);
  _delay(1000);

  // ---- DRIVER ----
  // MUST match the voltage your buck actually delivers to the board:
  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit = 6.0f;            // absolute ceiling the driver will output
  if (!driver.init()) {
    Serial.println("Driver init FAILED");
    return;
  }
  motor.linkDriver(&driver);

  // ---- MOTOR (open loop) ----
  motor.controller = MotionControlType::velocity_openloop;
  // *** SAFETY KNOB *** applied voltage magnitude. ~1V / 0.17ohm ~= 6A. START HERE.
  motor.voltage_limit = 1.0f;
  motor.velocity_limit = 20.0f;

  motor.init();   // NO initFOC() — open loop uses no sensor

  Serial.println("Open-loop velocity test ready");
  _delay(500);
}

void loop() {
  motor.move(target_velocity);   // keep loop() light; call move() as often as possible
}