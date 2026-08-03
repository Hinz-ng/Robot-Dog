#include <Arduino.h>
#include <SimpleFOC.h>

// ============================================================================
// ACTUATOR BASELINE + FOC CURRENT MODE + TIM4 HARDWARE ENCODER (register-level)
// Board: B-G431B-ESC1 clone (EG2124A). SimpleFOC 2.3.1, platform ststm32@17.6.0.
//
// ENCODER: TIM4 silicon quadrature decoder, PB6=TIM4_CH1, PB7=TIM4_CH2 (AF2).
//   Configured by direct register writes — the Arduino pinmap (PeripheralPins_
//   B_G431B_ESC1.c) is vendor-pruned and hangs in Error_Handler() on lookup
//   miss, which is what killed STM32HWEncoder and hardware SPI on this board.
//   Zero interrupts, zero CPU, cannot lose counts. No Z/index (PB8 = BOOT0).
//   ZEA stays session-relative: run 'f' once per power-up.
//
// NOTE: the high-pitched whine is MECHANICAL (audible when backdriving by hand) --
//   suspect motor or pulley bearing. Do not attribute it to the current loop.
//   Separate from the CURQ_I=2000 d-axis SCREECH, which was electrically real
//   (Id +-1.28 A, Ud railed, |I| 9.7 A).
//
// COMMANDS: g=go x/s=stop +/-=target | o=openloop t=torque(V) c=torque(I)
//           v=velocity | f=initFOC | ?=help
//   logger: l=fast capture  L=slow capture  k=kick-step  j=zero-step
//           d=dump CSV      a=stats (mean of last capture)
// Boots DISABLED. 20 s auto-stop. 150 rad/s overspeed cutoff. Torque modes arm at 0.
//
// ---------------------------------------------------------------------------
// CHARACTERISATION BUILD -- changes from the previous sketch, all declared:
//   1. motor.foc_modulation = SpaceVectorPWM (was library-default SinePWM).
//   2. Boot CFG banner echoes modulation / dead_zone / pwm_Hz / Vbus.
//   3. Burst log stores a REAL per-sample dt_us; LOG_N is 1000 with 18 B/sample.
//      logDump() now reports dt jitter and accumulates true time, not k*dt_mean.
//   4. k / j step-and-capture now work in TORQUE(V) as well as TORQUE(I).
//   5. 'a' prints the mean of the capture buffer -- sweep points without CSV.
//   6. motor.current is refreshed explicitly in TORQUE(V). 2.3.1's loopFOC()
//      does NOT touch motor.current in the voltage branch, so Iq/Id were STALE
//      in TORQUE(V) and OPENLOOP -- a frozen value that reads like a measurement.
//   7. Header comment corrected: overspeed cutoff is 150 rad/s (was documented
//      as 60; OVERSPEED_RADS has been 150).
//   8. DEAD_ZONE comment rewritten -- the old "step down 0.05->0.02->0.01"
//      instruction was already completed and had gone stale.
//   9. logStats() distinguishes "capture still running" from "buffer empty".
// No behavioural change to: the safety path, the sense-mismatch guard, the
// overspeed / auto-stop guards, initFOC, mode transitions, or any gain.
// ============================================================================

HardwareSerial SerialUART(PB4, PB3);

// ---------------------------------------------------------------------------
// TIM4 hardware quadrature sensor
// ---------------------------------------------------------------------------
class TIM4Encoder : public Sensor {
public:
  TIM4Encoder(int32_t ppr) : cpr(4 * ppr) {}

  void init() {
    // ---- GPIO: PB6/PB7 -> alternate function 2 (TIM4_CH1/CH2) ----
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    // MODER: 10 = alternate function
    GPIOB->MODER &= ~((3UL << (6 * 2)) | (3UL << (7 * 2)));
    GPIOB->MODER |=  ((2UL << (6 * 2)) | (2UL << (7 * 2)));
    // PUPDR: 01 = pull-up (encoder outputs are push-pull; harmless, noise margin)
    GPIOB->PUPDR &= ~((3UL << (6 * 2)) | (3UL << (7 * 2)));
    GPIOB->PUPDR |=  ((1UL << (6 * 2)) | (1UL << (7 * 2)));
    // OSPEEDR: 11 = very high speed
    GPIOB->OSPEEDR |= ((3UL << (6 * 2)) | (3UL << (7 * 2)));
    // AFR[0] holds pins 0-7, 4 bits each. AF2 = TIM4.
    GPIOB->AFR[0] &= ~((0xFUL << (6 * 4)) | (0xFUL << (7 * 4)));
    GPIOB->AFR[0] |=  ((0x2UL << (6 * 4)) | (0x2UL << (7 * 4)));

    // ---- TIM4: encoder mode ----
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
    TIM4->CR1 = 0;                       // disable while configuring
    // CC1S=01 (IC1 on TI1), CC2S=01 (IC2 on TI2), max input filters (0xF)
    TIM4->CCMR1 = (1UL << 0) | (0xFUL << 4) | (1UL << 8) | (0xFUL << 12);
    TIM4->CCER  = 0;                     // both inputs non-inverted
    TIM4->SMCR  = 3;                     // encoder mode 3: count both edges (x4)
    TIM4->ARR   = (uint32_t)(cpr - 1);   // wrap once per mechanical revolution
    TIM4->CNT   = 0;
    TIM4->EGR   = TIM_EGR_UG;            // latch registers
    TIM4->CR1  |= TIM_CR1_CEN;           // start counting

    this->Sensor::init();                // seed base-class state
  }

  // SimpleFOC calls this; must return 0..2PI (base class tracks full rotations)
  float getSensorAngle() override {
    return ((float)TIM4->CNT / (float)cpr) * _2PI;
  }

  // diagnostics
  int32_t rawCount() { return (int32_t)TIM4->CNT; }
  // call if counts run backwards vs. motor convention (inverts CH1 polarity)
  void invertDirection() { TIM4->CCER ^= TIM_CCER_CC1P; }

private:
  int32_t cpr;
};

// ---------------------------------------------------------------------------

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);
// Clone sense chain is gain-compensated -> genuine constants. Do not change.
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

TIM4Encoder encoder = TIM4Encoder(1024);   // 1024 PPR -> 4096 CPR

// ---- safety / tuning constants ----
// Uq rails -> real current = VOLT_LIMIT / R_eff(0.218). At 2.0 V that is ~9.2 A / 18 W
// for the ~1 s it takes to react: thermally trivial. Debounced sense guard is the backstop.
const float VOLT_LIMIT      = 3.5f;
const float VEL_MAX         = 20.0f;
const float OVERSPEED_RADS  = 150.0f;   // torque mode has NO built-in speed limit
const float VEL_STEP        = 1.0f;
const float TORQUE_STEP     = 0.01f;
// Raised for the angle-lag sweep: TORQUE(V) at 130 rad/s needs Uq = 2.56 V.
// Ceiling is 2.6 V, NOT 3.5 -- Uq = 3.5 settles at 183 rad/s, past the
// 150 rad/s overspeed guard. current_limit does not bind in voltage mode (§12).
const float TORQUE_MAX      = 2.6f;
const unsigned long AUTO_STOP_MS = 20000;
const unsigned long OVERSPEED_GRACE_MS = 300;   // ignore overspeed right after arming

const float CURR_STEP   = 0.1f;
const float CURR_MAX    = 2.0f;
const float CURR_LIMIT  = 2.0f;

// DEAD ZONE -- FINAL VALUE, set by argument and confirmed by measurement.
// dead_zone is a fraction of the PWM period, so its cost in lost command voltage
// is dead_zone * V_bus regardless of switching frequency: 0.057 V at 3S, 0.093 V
// at 5S. Referred to foot force at G = N/J = 87.7 that is ~1.0 N / ~1.6 N.
// The EG2124A already has interlock AND internal dead time, so its own dead time
// (a fixed ~200-300 ns = 0.005-0.0075 of a 40 us period) is comparable to or
// larger than this software value -- i.e. 0.005 is probably being absorbed by a
// floor set in hardware. Going to 0.000 would make shoot-through protection
// depend entirely on an undocumented interlock propagation delay on a clone
// board; going to 0.010 doubles the deadband for no extra protection.
// On STM32 6-PWM this value is baked into the timer at driver.init() -- it is NOT
// runtime-mutable, and the requested fraction is quantised. Ground truth is the
// measured U0 intercept from the locked-rotor sweep, NOT this number.
// Long-term fix for the deadband is U0 feedforward, not a smaller dead zone.
const float DEAD_ZONE   = 0.005f;

const float CURQ_P = 0.1f,  CURQ_I = 335.0f;
const float CURD_P = 0.1f,  CURD_I = 335.0f;
// MEASURED: with CUR_TF=0.005 (32 Hz corner) the current loop achieved only 80 Hz
// bandwidth (tau 2.0 ms) with 83% overshoot -- the filter was the dominant lag inside
// a loop measured at 412 Hz. Filter corner must sit 5-10x ABOVE loop bandwidth.
const float CUR_TF = 0.00025f;    // validated value; the filter-corner bisection is closed.

const float VEL_P  = 0.2f;   // A/(rad/s) -- velocity closes on current; sweep fresh
const float VEL_I  = 1.5f;   // 0 for the P_crit sweep; restore after
const float VEL_D  = 0.0f;
const float VEL_TF = 0.02f;

// ---------------------------------------------------------------------------
// BUS VOLTAGE SENSING
// ---------------------------------------------------------------------------
// WHY: driver.voltage_power_supply is the DIVISOR SimpleFOC uses to convert a
// requested voltage into a PWM duty cycle. It was a hardcoded 11.4 f -- a belief,
// not a measurement. Bench pack measured 11.30 V (2026-__-__): 0.9% error, inside
// the R_eff scatter, so the master table stands. On the robot at ~230 A the pack
// sags 3-7 V and a hardcoded divisor causes:
//   (a) delivered voltage != commanded voltage,
//   (b) PID_current limits sitting ABOVE the achievable modulation ceiling, so
//       the integrator winds up against a ceiling that does not physically exist,
//   (c) every logged Uq becoming a REQUEST rather than a DELIVERY, which silently
//       breaks the Uq = R*Iq + U0 + Ke*w cross-check. Same failure family as the
//       stale motor.current in the voltage branch: a value nobody updates that
//       reads exactly like data.
//
// PIN: PA0. CONFIRMED on the bench, not inferred -- 11.30 V -> 1351 counts,
// 22.50 V -> 2694 counts. Voltage ratio 1.991, count ratio 1.994 (0.15% apart).
// PA1 / PB12 were flat; PB14 moved the wrong way (it is the board thermistor).
//
// SCALE: measured, not from a datasheet -- the divider ratio is undocumented on
// this clone. Pure proportional fit; the 33 mV offset from a 2-point line fit is
// smaller than the +-0.05 V rounding in the 22.5 V meter reading, so it is not
// resolvable and is discarded. Back-predicts both points to within 0.07%.
//   full scale  = 4095 * 0.008358 = 34.23 V bus
//   resolution  = 8.36 mV / count
//   6S at 25.2 V = 3015 counts = 74% of range -> NO PB10 / 48V_EN change needed.
// Re-calibrate if PB10 (48V_EN) is ever driven: it switches the divider range.
//
// VBUS_SCALE = 0 disables this entire feature. Behaviour then is byte-identical
// to the old hardcode, which makes rollback a one-character edit.
const uint32_t PIN_VBUS   = PA0;
const float VBUS_SCALE    = 0.008358f;  // V per ADC count -- MEASURED 2026-__-__
// SEED-ONLY. Arduino analogRead() returns 0 on any pin of this ADC once
// currentSense.init() has armed the injected-conversion group -- confirmed
// 2026-__-__ by vraw=0 from an isolated call in the print block, while the
// pre-init setup read works every time. Not a contention-between-two-calls
// issue: ONE call fails. HAL_ADC_Start returns BUSY because the peripheral is
// never in the READY state again.
// Live tracking requires a register-level REGULAR-group conversion on PA0.
// Regular and injected groups coexist by design: injected preempts, regular
// resumes. No pausing, no blind interval in the current loop.
// Deferred -- the bench has no sag to track. See README 8.3.
const bool  VBUS_LIVE     = false;
const float VBUS_TF       = 0.020f;     // 20 ms. Noise here becomes motor current.
const float VBUS_MIN      = 8.0f;       // PLAUSIBILITY window only -- NOT a
const float VBUS_MAX      = 30.0f;      //   low-voltage cutoff. See note below.
const float VBUS_FALLBACK = 11.30f;     // measured bench pack, used if read fails
float vbus_filt           = VBUS_FALLBACK;
bool  vbus_valid          = false;
// NOTE ON VBUS_MAX: the ADC saturates at 34.2 V, so a genuine overvoltage above
// that would read as exactly 34.2 and be REJECTED by this window -- the filter
// would then hold its last good value rather than reporting the fault. That is
// the correct behaviour for a divisor, but it means this window is not, and must
// not be mistaken for, overvoltage protection.

enum Mode { MODE_OPENLOOP, MODE_TORQUE, MODE_TORQUE_CURRENT, MODE_VELOCITY };
Mode mode = MODE_OPENLOOP;
bool foc_ready = false;
bool running = false;
float target = 2.0f;

bool driver_ok=false, cs_ok=false, cs_linked=false;
unsigned long run_started=0, last_blink=0, last_print=0;
bool led_state=false;

volatile uint32_t g_loops = 0;
uint32_t lps = 0;
uint32_t pr_us = 0;
uint16_t print_ms = 300;   // 'q' toggles 300 <-> 3000 for the loop-rate test

// ---------------------------------------------------------------------------
// BURST LOGGER -- captures every loop iteration into RAM, dumps afterwards.
// 300 ms serial telemetry aliases everything above ~1.7 Hz; this does not.
//   l = fast capture  (decim 1  -> ~65 ms  @15.5kHz, for current-loop steps)
//   L = slow capture  (decim 8  -> ~520 ms,           for judder / resonance)
//   k = kick: step target BASE -> STEP and fast-capture it. TORQUE(V) or (I).
//   j = kick from ZERO -- deliberately includes the dead-zone traverse.
//   d = dump last capture as CSV
//   a = mean of last capture (one line per sweep point; no CSV needed)
// ---------------------------------------------------------------------------
#define LOG_N 1000
struct LogSample {
  uint16_t dt_us;         // us since the PREVIOUS sample. logDump() used to
                          // reconstruct t = k * dt_mean, which ASSUMES uniform
                          // sampling. The sense guard's extra ADC read (every 30
                          // loops), the LED blink and handleSerial all jitter the
                          // loop; an inertia fit against an assumed-uniform clock
                          // is biased by exactly that jitter. Also the decisive
                          // datum for the 57 kHz vs 15.5 kHz lps discrepancy.
                          // Saturates at 65.5 ms; decim 8 @15.5 kHz is ~516 us.
  int16_t  vel_x50;       // rad/s * 50 -- FILTERED by LPF_velocity (Tf=20 ms).
                          // Do NOT fit inertia from this column; use cnt.
  int16_t  iq_x1000;      // A * 1000
  int16_t  id_x1000;      // A * 1000
  int16_t  uq_x1000;      // V * 1000
  int16_t  ud_x1000;      // V * 1000 -- d-axis PI output. In current mode this IS
                          // the cross-coupling term (w_e*L*Iq) plus any angle-error
                          // contribution: the direct measurement of angle lag.
  int16_t  sp_x1000;      // current SETPOINT * 1000
  int16_t  raw_x100;      // RAW unsynchronised |I| * 100 -- sees PWM-rate ripple that
                          // the synchronously sampled dq path is blind to
  uint16_t cnt;           // raw TIM4 count (ground truth for velocity).
                          // uint16, wraps at 4095 = one motor revolution: UNWRAP
                          // in post-processing before differentiating.
};
LogSample logbuf[LOG_N];
volatile uint16_t log_i = 0;
volatile bool     log_active = false;
bool     log_ready = false;
bool     log_announced = false;
uint8_t  log_decim = 1;
uint8_t  log_skip  = 0;
uint32_t log_t0 = 0, log_t1 = 0;
uint32_t log_t_prev = 0;        // timestamp of the previous stored sample
// Conditions the capture was taken under. Gate D was invalidated by a capture
// taken with the motor disarmed -- the STATS line showed a stale Uq=2.0 and
// vel=2.0 and looked entirely healthy. Record the state so a capture can never
// be separated from the conditions that produced it.
Mode     log_mode = MODE_OPENLOOP;
bool     log_running = false;
// Step between two NONZERO currents: stepping from 0 puts the 532 us dead-zone
// traverse in the measurement and hides the true electrical bandwidth.
const float KICK_BASE = 0.5f;   // pre-step hold current
const float KICK_A    = 1.5f;   // post-step current
// TORQUE(V) step pair, for the inertia (J) and electrical-time-constant (L)
// measurements. A Uq step is a KNOWN excitation even when the current loop is
// untuned, so those tests do not depend on the open CUR_TF bisection. Iq is
// logged every sample, so torque is reconstructed rather than assumed constant.
// 0.20 V clears the dead-zone offset and keeps the rotor moving (stiction out of
// the picture); 0.80 V gives dIq = 0.6/0.218 = 2.75 A of step at ~4 W.
const float KICKV_BASE = 0.20f;   // V -- pre-step hold voltage
const float KICKV_A    = 0.80f;   // V -- post-step voltage
uint32_t kick_at = 0;           // scheduled kick (two-stage: zero -> settle -> step)
bool kick_zero = false;

void logStart(uint8_t decim) {
  log_decim = decim; log_skip = 0; log_i = 0;
  log_ready = false; log_announced = false;
  log_mode = mode; log_running = running;
  log_t0 = micros(); log_t_prev = log_t0; log_active = true;
  SerialUART.print(F("CAPTURE start decim=")); SerialUART.println(decim);
}

void logDump() {
  if (!log_ready) { SerialUART.println(F("no capture in buffer")); return; }
  float dt_us = (float)(log_t1 - log_t0) / (float)(log_i > 1 ? (log_i - 1) : 1);
  SerialUART.println(F("# BURST DUMP"));
  SerialUART.print(F("# samples=")); SerialUART.print(log_i);
  SerialUART.print(F(" decim=")); SerialUART.print(log_decim);
  SerialUART.print(F(" dt_us=")); SerialUART.print(dt_us, 2);
  SerialUART.print(F(" fs_Hz=")); SerialUART.println(1e6f / dt_us, 1);
  SerialUART.print(F("# mode=")); SerialUART.print(log_mode == MODE_OPENLOOP ? "OL"
  : log_mode == MODE_TORQUE ? "TV"
  : log_mode == MODE_TORQUE_CURRENT ? "TI" : "VEL");
  SerialUART.print(F(" run=")); SerialUART.println(log_running ? 1 : 0);
  // Sampling jitter. If max/min spread more than ~20%, every downstream fit must
  // use the per-sample t_us column, not the mean.
  uint16_t dt_min = 0xFFFF, dt_max = 0;
  for (uint16_t k = 1; k < log_i; k++) {      // k=0 spans logStart -> first sample
    if (logbuf[k].dt_us < dt_min) dt_min = logbuf[k].dt_us;
    if (logbuf[k].dt_us > dt_max) dt_max = logbuf[k].dt_us;
  }
  SerialUART.print(F("# dt_us min=")); SerialUART.print(dt_min);
  SerialUART.print(F(" max="));        SerialUART.println(dt_max);
  SerialUART.println(F("i,t_us,vel,Iq,Id,Uq,Ud,sp,rawI,cnt"));
  uint32_t t_acc = 0;
  for (uint16_t k = 0; k < log_i; k++) {
    t_acc += logbuf[k].dt_us;               // true elapsed, not k * dt_mean
    SerialUART.print(k); SerialUART.print(',');
    SerialUART.print(t_acc); SerialUART.print(',');
    SerialUART.print(logbuf[k].vel_x50 / 50.0f, 2); SerialUART.print(',');
    SerialUART.print(logbuf[k].iq_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].id_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].uq_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].ud_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].sp_x1000 / 1000.0f, 3); SerialUART.print(',');
    SerialUART.print(logbuf[k].raw_x100 / 100.0f, 2); SerialUART.print(',');
    SerialUART.println(logbuf[k].cnt);
  }
  SerialUART.println(F("# END"));
}

// Mean of the capture buffer. The R_eff and Ke sweeps are ~10 steady-state points
// each; reading every point off a 1000-row CSV is the dominant time cost of the
// campaign. Skips the first 25% so an arming transient never averages into a
// steady-state point. For k/j step captures, this reports the settled value, not
// the transient. Echoes dead_zone so a measurement can never get separated
// from the condition it was taken under.
void logStats() {
  if (log_active)  { SerialUART.println(F("capture still running -- wait")); return; }
  if (!log_ready)  { SerialUART.println(F("no capture in buffer")); return; }
  uint16_t n = log_i;
  if (n < 8)       { SerialUART.println(F("capture too short")); return; }
  uint16_t k0 = n / 4, m = n - k0;
  float sIq=0, sId=0, sUq=0, sUd=0, sVel=0, sRaw=0, iq_lo=1e6f, iq_hi=-1e6f;
  for (uint16_t k = k0; k < n; k++) {
    float iq = logbuf[k].iq_x1000 / 1000.0f;
    sIq  += iq;
    sId  += logbuf[k].id_x1000 / 1000.0f;
    sUq  += logbuf[k].uq_x1000 / 1000.0f;
    sUd  += logbuf[k].ud_x1000 / 1000.0f;
    sVel += logbuf[k].vel_x50 / 50.0f;
    sRaw += logbuf[k].raw_x100 / 100.0f;
    if (iq < iq_lo) iq_lo = iq;
    if (iq > iq_hi) iq_hi = iq;
  }
  float mIq = sIq / m, mRaw = sRaw / m;
  SerialUART.print(F("STATS n="));  SerialUART.print(m);
  SerialUART.print(F(" m="));       SerialUART.print(log_mode == MODE_OPENLOOP ? "OL"
  : log_mode == MODE_TORQUE ? "TV"
  : log_mode == MODE_TORQUE_CURRENT ? "TI" : "VEL");
  SerialUART.print(F(" run="));     SerialUART.print(log_running ? 1 : 0);
  SerialUART.print(F(" dz="));      SerialUART.print(driver.dead_zone, 4);
  SerialUART.print(F(" Uq="));      SerialUART.print(sUq / m, 4);
  SerialUART.print(F(" Ud="));      SerialUART.print(sUd / m, 4);
  SerialUART.print(F(" Iq="));      SerialUART.print(mIq, 4);
  SerialUART.print(F(" Id="));      SerialUART.print(sId / m, 4);
  SerialUART.print(F(" vel="));     SerialUART.print(sVel / m, 3);
  SerialUART.print(F(" |I|="));     SerialUART.print(mRaw, 3);
  SerialUART.print(F(" Iq_pp="));   SerialUART.print(iq_hi - iq_lo, 3);
  // Angle + calibration integrity: |I| / |Iq| must sit near sqrt(3/2) = 1.225.
  SerialUART.print(F(" ratio="));
  SerialUART.println(fabsf(mIq) > 0.05f ? mRaw / fabsf(mIq) : 0.0f, 3);
}

const char* modeName() {
  switch (mode) {
    case MODE_OPENLOOP:       return "OPENLOOP";
    case MODE_TORQUE:         return "TORQUE(V)";
    case MODE_TORQUE_CURRENT: return "TORQUE(I)";
    case MODE_VELOCITY:       return "VELOCITY";
  }
  return "?";
}

bool needsFOC(Mode m)   { return (m == MODE_TORQUE || m == MODE_TORQUE_CURRENT || m == MODE_VELOCITY); }
bool isTorqueMode(Mode m){ return (m == MODE_TORQUE || m == MODE_TORQUE_CURRENT); }

void printHelp() {
  SerialUART.println(F("--- g:go x:stop +/-:target | o t c v modes | f:initFOC ---"));
  SerialUART.println(F("--- logger: l=fast L=slow k=kick j=zero-kick d=dump a=stats ---"));
}

void startMotor() {
  if (!driver_ok) { SerialUART.println(F("refused: driver init failed")); return; }
  if (needsFOC(mode) && !foc_ready) { SerialUART.println(F("refused: run initFOC (f) first")); return; }
  if (mode == MODE_TORQUE_CURRENT && !cs_linked) { SerialUART.println(F("refused: current sense not linked")); return; }
  if (isTorqueMode(mode) && target != 0.0f) {
    target = 0.0f;
    SerialUART.println(F("torque mode: target reset to 0 on arm (ramp with +)"));
  }
  // clear stale controller state so a previous run cannot leak into this one
  motor.PID_velocity.reset();
  motor.PID_current_q.reset();
  motor.PID_current_d.reset();
  encoder.update();                 // refresh sensor before the loop engages
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
  if (needsFOC(m) && !foc_ready) { SerialUART.println(F("run initFOC (f) first")); return; }
  if (m == MODE_TORQUE_CURRENT && !cs_linked) { SerialUART.println(F("current sense not linked")); return; }
  mode = m;
  switch (mode) {
    case MODE_OPENLOOP:
      motor.controller = MotionControlType::velocity_openloop; target = 2.0f; break;
    case MODE_TORQUE:  // legacy voltage-torque, kept for A/B
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::torque;
      target = 0.0f; break;
    case MODE_TORQUE_CURRENT:
      motor.torque_controller = TorqueControlType::foc_current;
      motor.controller = MotionControlType::torque; target = 0.0f; break;
    case MODE_VELOCITY:
      // velocity closes on the CURRENT loop: current_limit binds here (stall = 2 A,
      // not 7-9 A) and Iq/Id telemetry is live. Velocity PID output is in AMPS.
      motor.torque_controller = TorqueControlType::foc_current;
      motor.controller = MotionControlType::velocity;
      motor.PID_velocity.limit = CURR_LIMIT;
      target = 2.0f; break;
  }
  SerialUART.print(F("mode=")); SerialUART.print(modeName());
  SerialUART.print(F(" target=")); SerialUART.println(target);
}

void runInitFOC() {
  if (running) { SerialUART.println(F("stop first (x)")); return; }
  SerialUART.println(F("initFOC: aligning (expect a small twitch)."));
  motor.zero_electric_angle = NOT_SET;
  motor.enable();
  int ok = motor.initFOC();
  motor.disable();
  target = 0.0f;
  if (ok) {
    foc_ready = true;
    SerialUART.println(F("initFOC SUCCESS"));
    SerialUART.print(F("zero_electric_angle=")); SerialUART.println(motor.zero_electric_angle, 4);
    SerialUART.print(F("sensor_direction=")); SerialUART.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
  } else {
    foc_ready = false;
    SerialUART.println(F("initFOC FAILED"));
  }
}

void adjustTarget(float dir) {
  if (mode == MODE_TORQUE)              target = constrain(target + dir*TORQUE_STEP, -TORQUE_MAX, TORQUE_MAX);
  else if (mode == MODE_TORQUE_CURRENT) target = constrain(target + dir*CURR_STEP,  -CURR_MAX,  CURR_MAX);
  else                                  target = constrain(target + dir*VEL_STEP,   -VEL_MAX,   VEL_MAX);
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
      case 'c': case 'C': setMode(MODE_TORQUE_CURRENT); break;
      case 'v': case 'V': setMode(MODE_VELOCITY); break;
      case 'f': case 'F': runInitFOC(); break;
      case 'l': logStart(1); break;                     // fast capture (~65 ms)
      case 'L': logStart(8); break;                     // slow capture (~520 ms)
      case 'd': case 'D': logDump(); break;
      case 'a': case 'A': logStats(); break;            // mean of last capture
      case 'q': case 'Q':
        print_ms = (print_ms == 300) ? 3000 : 300;
        SerialUART.print(F("print_ms=")); SerialUART.println(print_ms); break;
      case 'k': case 'K':                               // step + capture
        if (!running || !isTorqueMode(mode)) {
          SerialUART.println(F("k: need TORQUE(V) or TORQUE(I) running"));
        } else {
          bool volts = (mode == MODE_TORQUE);
          target = volts ? KICKV_BASE : KICK_BASE;      // pre-load OUT of the dead zone
          kick_at = millis() + 300;                     // settle, then step (in loop)
          SerialUART.print(F("kick armed: hold ")); SerialUART.print(target, 3);
          SerialUART.print(volts ? F(" V") : F(" A"));
          SerialUART.print(F(" 300ms, then step to "));
          SerialUART.println(volts ? KICKV_A : KICK_A, 3);
        }
        break;
      case 'j': case 'J':                               // zero-based step: measures dead-zone traverse
        if (!running || !isTorqueMode(mode)) {
          SerialUART.println(F("j: need TORQUE(V) or TORQUE(I) running"));
        } else {
          target = 0.0f;
          kick_at = millis() + 300;
          kick_zero = true;
          SerialUART.println(F("kick armed from ZERO (dead-zone traverse)"));
        }
        break;
      case '?': printHelp(); break;
      default: break;
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // PC6 (STATUS)
  SerialUART.begin(921600);
  _delay(2000);
  SimpleFOCDebug::enable(&SerialUART);
  SerialUART.println(F("=== actuator + current mode + TIM4 HW encoder ==="));

  // ---- BUS VOLTAGE: seed BEFORE driver.init() and BEFORE currentSense.init().
  // Ordering is deliberate: currentSense.init() reconfigures the ADC, so any
  // Arduino-API analogRead() must either happen before it or be verified against
  // it afterwards (see the |I|/Iq gate in the verification procedure).
  analogReadResolution(12);                  // default is 10-bit; 2 bits for free
  pinMode(PIN_VBUS, INPUT_ANALOG);           // detach digital buffer, unload divider
  {
    uint32_t acc = 0;
    (void)analogRead(PIN_VBUS);   // discard: first conversion carries residue
    for (int k = 0; k < 64; k++) acc += analogRead(PIN_VBUS);   // average 64
    float v = (acc / 64.0f) * VBUS_SCALE;
    if (VBUS_SCALE > 0.0f && v > VBUS_MIN && v < VBUS_MAX) {
      vbus_filt = v;  vbus_valid = true;
    } else {
      vbus_filt = VBUS_FALLBACK;  vbus_valid = false;
      SerialUART.println(F("!! VBUS read implausible -- using fallback"));
    }
  }
  driver.voltage_power_supply = vbus_filt;
  driver.voltage_limit = 6.0f;
  driver.dead_zone = DEAD_ZONE;
  driver_ok = driver.init();
  SerialUART.println(driver_ok ? F("driver OK") : F("driver FAILED"));

  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);

  SerialUART.println(F("initialising TIM4 encoder..."));
  encoder.init();                            // register-level, cannot hang on pinmap
  motor.linkSensor(&encoder);
  SerialUART.print(F("TIM4 encoder OK, raw count=")); SerialUART.println(encoder.rawCount());

  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit  = VOLT_LIMIT;
  motor.velocity_limit = VEL_MAX;
  motor.current_limit  = CURR_LIMIT;

  // 2.3.1 defaults foc_modulation to SinePWM; this sketch inherited that silently.
  // SVPWM raises the linear ceiling from V_bus/2 to V_bus/sqrt(3) (+15.5% of usable
  // voltage). Only the CEILING changes: at bench modulation depth (2 V of 11.4 V)
  // the fundamental phase voltage for a given Uq is identical, so the phase
  // currents should NOT change. The zero-sequence component SVPWM adds is
  // common-mode; with no neutral connection the phase currents are unaffected, so
  // the |I|/Iq = 1.225 integrity check still holds. A/B this against the ammeter
  // baseline before trusting anything downstream of it.
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.PID_velocity.P = VEL_P;
  motor.PID_velocity.I = VEL_I;
  motor.PID_velocity.D = VEL_D;
  motor.LPF_velocity.Tf = VEL_TF;

  motor.PID_current_q.P = CURQ_P;  motor.PID_current_q.I = CURQ_I;  motor.PID_current_q.D = 0.0f;
  motor.PID_current_d.P = CURD_P;  motor.PID_current_d.I = CURD_I;  motor.PID_current_d.D = 0.0f;
  motor.LPF_current_q.Tf = CUR_TF;
  motor.LPF_current_d.Tf = CUR_TF;
  motor.PID_current_q.limit = VOLT_LIMIT;
  motor.PID_current_d.limit = VOLT_LIMIT;

  motor.voltage_sensor_align = 2.0f;
  motor.init();

  cs_ok = currentSense.init();
  SerialUART.println(cs_ok ? F("currentSense OK") : F("currentSense FAILED"));
  if (cs_ok) {
    motor.linkCurrentSense(&currentSense);
    cs_linked = true;
    SerialUART.println(F("currentSense LINKED"));
  }

  motor.disable();

  // "Note which config is actually flashed." A measurement is only comparable to
  // others taken under the SAME four values. On STM32 6-PWM, dead_zone is
  // converted to a timer dead-time register value at driver.init() and quantised,
  // so this echoes what was REQUESTED -- the measured U0 intercept is what the
  // hardware actually does.
  SerialUART.print(F("CFG modulation="));
  SerialUART.print(motor.foc_modulation == FOCModulationType::SpaceVectorPWM ? F("SVPWM") : F("SinePWM"));
  SerialUART.print(F(" dead_zone=")); SerialUART.print(driver.dead_zone, 4);
  SerialUART.print(F(" pwm_Hz="));    SerialUART.print(driver.pwm_frequency);
  SerialUART.print(F(" Vbus="));      SerialUART.print(driver.voltage_power_supply, 2);
  SerialUART.print(F(" vok="));       SerialUART.println(vbus_valid ? 1 : 0);

  // TIM4 counting convention confirmed CCW (matches the previous software
  // encoder). Preset it so 'f' skips direction detection -> short twitch.
  motor.sensor_direction = Direction::CCW;
  foc_ready = false;

  SerialUART.println(F("Motor DISABLED. Run 'f' once per power-up."));
  printHelp();
}

void loop() {
  handleSerial();

  // ---- BUS VOLTAGE: 1 kHz sample -> filter -> republish derived limits ----
  if (VBUS_SCALE > 0.0f && VBUS_LIVE) {
    static uint32_t vbus_last_us = 0;
    uint32_t now_us = micros();
    // Cast handles micros() wraparound at ~71 min. Rate-limited to 1 kHz because
    // analogRead() blocks for a few us; once per 12.5 loops is 0.5% overhead,
    // once per loop would be ~6%. The cost lands in dt_us, which is logged.
    if ((uint32_t)(now_us - vbus_last_us) >= 1000) {
      vbus_last_us = now_us;
      (void)analogRead(PIN_VBUS);
      float v = (float)analogRead(PIN_VBUS) * VBUS_SCALE;
      if (v > VBUS_MIN && v < VBUS_MAX) {
        // One-pole low-pass, 1 ms sample period, 20 ms time constant -> each new
        // reading contributes 4.8%. HEAVILY filtered on purpose: this value is the
        // divisor in every duty-cycle calculation, so ADC noise here is injected
        // straight into the motor current and from there into the current loop.
        const float a = 0.001f / (VBUS_TF + 0.001f);
        vbus_filt += a * (v - vbus_filt);
        vbus_valid = true;
        driver.voltage_power_supply = vbus_filt;
        // SVPWM can only synthesise V_bus/sqrt(3) = 0.57735*V_bus. A PI output
        // limit ABOVE that is not a limit: the integrator winds up against a
        // ceiling that does not exist, then dumps the windup when the bus
        // recovers. Inert on the 3S bench (6.52 V ceiling vs VOLT_LIMIT 2.0);
        // this exists for the robot.
        float ceiling = vbus_filt * 0.57735f;
        float lim = (VOLT_LIMIT < ceiling) ? VOLT_LIMIT : ceiling;
        motor.voltage_limit       = lim;
        motor.PID_current_q.limit = lim;
        motor.PID_current_d.limit = lim;
      } else {
        vbus_valid = false;              // implausible: hold last good value
      }
    }
  }

  unsigned long period = (driver_ok && cs_ok) ? 500 : 80;
  if (millis() - last_blink > period) { led_state=!led_state; digitalWrite(LED_BUILTIN, led_state); last_blink=millis(); }

  motor.loopFOC();
  g_loops++;
  // loopFOC() updates the sensor ONLY when the motor is enabled AND in a
  // closed-loop mode. Otherwise update it here, or shaft_velocity freezes at
  // its last value -- which latched the overspeed trip across runs.
  if (!running || mode == MODE_OPENLOOP) encoder.update();

  // 2.3.1's loopFOC() refreshes motor.current ONLY in the dc_current and
  // foc_current branches; the voltage branch returns without touching it. In
  // TORQUE(V), Iq/Id were therefore STALE -- left behind by initFOC or the last
  // TORQUE(I) run -- and every Uq-vs-Iq measurement would have read a frozen
  // number that looks exactly like data. (|I| from getPhaseCurrents() was always
  // live, which is why this never showed up before.)
  // Unfiltered on purpose: LPF_current (Tf = 500 us) is ~2x the electrical time
  // constant and would dominate any current rise-time fit.
  if (cs_linked && mode == MODE_TORQUE) {
    motor.current = currentSense.getFOCCurrents(motor.electrical_angle);
  } else if (mode == MODE_OPENLOOP) {
    // Deliberately zeroed rather than left stale: velocityOpenloop() never writes
    // electrical_angle, so a dq transform here would be meaningless. Read |I|.
    motor.current.q = 0.0f; motor.current.d = 0.0f;
  }

  // SENSE-MISMATCH GUARD: raw (unsynchronised) |I| must stay near 1.225*|Iq|.
  // A large divergence means the dq feedback has collapsed and the loop is winding
  // to the voltage rail -- the 8-12 A runaway. Checked every ~2 ms.
  // The raw read is ONE unsynchronised instant of a PWM-rippling current, so it spikes
  // several x the average during legitimate transients (e.g. breaking free of stiction).
  // A real runaway is sustained -> require GUARD_HITS consecutive violations.
  static uint16_t guard_div = 0;
  static uint8_t  guard_hits = 0;
  const uint8_t   GUARD_HITS = 8;          // ~8 x 2 ms = 16 ms of sustained mismatch
  if (running && mode == MODE_TORQUE_CURRENT && ++guard_div >= 30) {
    guard_div = 0;
    PhaseCurrent_s gc = currentSense.getPhaseCurrents();
    float raw = sqrtf(gc.a*gc.a + gc.b*gc.b + gc.c*gc.c);
    float expect = 1.225f * fabsf(motor.current.q) + 0.6f;
    if (raw > expect * 3.0f && raw > 3.0f && fabsf(motor.voltage.q) < VOLT_LIMIT * 0.9f) {
      if (++guard_hits >= GUARD_HITS) {
        guard_hits = 0;
        stopMotor("SENSE MISMATCH (sustained raw |I| >> dq Iq)");
      }
    } else {
      guard_hits = 0;                      // any good sample clears the count
    }
  }

  if (running) {
    unsigned long run_ms = millis() - run_started;
    if (run_ms > OVERSPEED_GRACE_MS && fabsf(motor.shaft_velocity) > OVERSPEED_RADS)
                                                          stopMotor("OVERSPEED");
    else if (run_ms > AUTO_STOP_MS)                        stopMotor("auto 20s");
    else                                                   motor.move(target);
  }

  // scheduled kick: base settled -> start capture, then step (capture sees the edge)
  if (kick_at && millis() >= kick_at) {
    kick_at = 0;
    if (running && isTorqueMode(mode)) {
      logStart(1);
      target = (mode == MODE_TORQUE) ? KICKV_A
                                     : (kick_zero ? 1.0f : KICK_A);
      kick_zero = false;
    }
  }

  // burst capture: one sample per (decim) loop iterations, no serial in the path
  if (log_active) {
    if (++log_skip >= log_decim) {
      log_skip = 0;
      uint32_t t_now = micros();
      LogSample &e = logbuf[log_i];
      e.dt_us    = (uint16_t)(t_now - log_t_prev);
      log_t_prev = t_now;
      e.vel_x50  = (int16_t)(motor.shaft_velocity * 50.0f);
      e.iq_x1000 = (int16_t)(motor.current.q * 1000.0f);
      e.id_x1000 = (int16_t)(motor.current.d * 1000.0f);
      e.uq_x1000 = (int16_t)(motor.voltage.q * 1000.0f);
      e.ud_x1000 = (int16_t)(motor.voltage.d * 1000.0f);
      e.sp_x1000 = (int16_t)(motor.current_sp * 1000.0f);
      PhaseCurrent_s lc = currentSense.getPhaseCurrents();
      e.raw_x100 = (int16_t)(sqrtf(lc.a*lc.a + lc.b*lc.b + lc.c*lc.c) * 100.0f);
      e.cnt      = (uint16_t)TIM4->CNT;
      if (++log_i >= LOG_N) {
        log_active = false; log_ready = true; log_t1 = micros();
      }
    }
  }

  if (!log_active && millis() - last_print > print_ms) {
    uint32_t pr_start = micros();
    unsigned long now = millis();
    unsigned long dt  = now - last_print;
    lps = (dt > 0) ? (uint32_t)((g_loops * 1000UL) / dt) : 0;
    g_loops = 0;

    SerialUART.print(F("m=")); SerialUART.print(modeName());
    SerialUART.print(F(" run=")); SerialUART.print(running?1:0);
    SerialUART.print(F(" tgt=")); SerialUART.print(target, 2);
    SerialUART.print(F(" cnt=")); SerialUART.print(encoder.rawCount());
    SerialUART.print(F(" vel=")); SerialUART.print(motor.shaft_velocity, 2);

    // Print dq in every mode: Id is the desync detector, not a current-mode luxury.
    // In OPENLOOP these are forced to 0 above -- a deliberate zero, not a stale read.
    SerialUART.print(F(" Iq=")); SerialUART.print(motor.current.q, 2);
    SerialUART.print(F(" Id=")); SerialUART.print(motor.current.d, 2);
    PhaseCurrent_s c = currentSense.getPhaseCurrents();
    SerialUART.print(F(" |I|=")); SerialUART.print(sqrtf(c.a*c.a + c.b*c.b + c.c*c.c), 2);

    SerialUART.print(F(" Uq=")); SerialUART.print(motor.voltage.q, 3);
    SerialUART.print(F(" Ud=")); SerialUART.print(motor.voltage.d, 3);
    SerialUART.print(F(" Vb=")); SerialUART.print(vbus_filt, 2);
    // 'seed' not 'vok': with VBUS_LIVE=false, Vb is the boot measurement and is
    // NOT tracking. Printed so no capture can be read as if it were live.
    SerialUART.print(F(" Vb_src=")); SerialUART.print(VBUS_LIVE ? F("live") : F("seed"));
    SerialUART.print(F(" lps=")); SerialUART.print(lps);
    SerialUART.print(F(" pr_us=")); SerialUART.println(pr_us);   // cost of the PREVIOUS print block
    if (log_ready && !log_announced) {
      SerialUART.println(F("CAPTURE done -- press d to dump")); log_announced = true;
    }
    pr_us = micros() - pr_start;      // how long this telemetry block actually took
    last_print = millis();            // start the next window AFTER printing
    g_loops = 0;                      // so lps measures control-loop rate only
  }
}
