#pragma once
// ============================================================================
// mt6816.h -- MT6816 14-bit absolute encoder, 4-wire SPI, bit-banged.
// ============================================================================
// Layer: bottom. Depends ONLY on Arduino GPIO, SimpleFOC's Sensor base and
// ENC_RAD_PER_COUNT / ENC_CPR from fleet_config.h. It includes NOTHING upward --
// in particular it does not touch SerialUART, motor, driver or currentSense, so
// Tier-0 can take it as-is.
//
// *** UNIT RISK -- THREE ANGLE UNITS MEET HERE. ***
//   rawCount() / raw   COUNTS,          0..ENC_CPR-1 over ONE MECHANICAL rev
//   getSensorAngle()   MECHANICAL rad,  0..2*PI  (counts * ENC_RAD_PER_COUNT)
//   what FOC commutates on is ELECTRICAL rad = mechanical * MOTOR_POLE_PAIRS,
//   which is 7x on this motor. SimpleFOC does that multiply itself; do NOT
//   pre-multiply here. 1 count = 0.02197 deg mech = 0.1538 deg elec.
// The log field `cnt` is this raw count and WRAPS at ENC_CPR-1 -- unwrap before
// differentiating, or a single wrap reads as a huge velocity spike.
// ============================================================================

// ---------------------------------------------------------------------------
// MT6816 absolute encoder, 4-wire SPI, BIT-BANGED
// ---------------------------------------------------------------------------
// Bit-banged on purpose. Arduino-API hardware SPI hangs on this clone: the
// vendor-pruned PeripheralPins_B_G431B_ESC1.c lookup miss lands in
// Error_Handler(), an infinite loop -- the same trap that killed STM32HWEncoder.
// Bit-banging consults no pin map at all, and MODE3 bit-bang was already proven
// working on this hardware. It also leaves the telemetry UART on PB4/PB3.
//
// ALL FOUR PINS MUST BE ON GPIOB (the fast path touches GPIOB->BSRR / ->IDR).
// Remapping is a four-line edit here; keep SCK on PB8 unless you re-read the
// BOOT0 note in the header.
const uint8_t SPI_CSN_BIT  = 5;    // PB5
const uint8_t SPI_MOSI_BIT = 6;    // PB6  (chip pin 5, was ABZ 'A')
const uint8_t SPI_MISO_BIT = 7;    // PB7  (chip pin 6, was ABZ 'B')
const uint8_t SPI_SCK_BIT  = 8;    // PB8  (chip pin 7, was ABZ 'Z') -- BOOT0

// Half-period padding. 170 MHz -> 5.88 ns/cycle. Datasheet minima: TSCK 64 ns,
// TSCKL/TSCKH 30 ns each. Measured frame times (2 frames per angle read):
//    8 NOPs -> SCK 7.7 MHz, 2.1 us/frame     20 NOPs -> 3.7 MHz, 4.3 us/frame
//   30 NOPs -> SCK 2.6 MHz, 6.2 us/frame     40 NOPs -> 2.0 MHz, 8.1 us/frame
// If parity errors appear, slow down and re-run 'e'.
const uint8_t SPI_HALF_NOPS = 1; // 6.65 us/read measured.

static inline void spiHalf() {
  for (uint8_t i = 0; i < SPI_HALF_NOPS; i++) __asm__ volatile ("nop");
}

// Reject an angle step larger than this (radians, mechanical) as corruption.
// DEFAULT OFF for bring-up: one variable at a time. At 150 rad/s and a 74 us
// loop the true step is 0.011 rad, but a 1 ms serial block makes it 0.15 rad,
// so the threshold must clear that with margin. Enable only AFTER the parity
// error rate from 'e' is known to be zero.
const bool  SPI_JUMP_GUARD  = false;
const float SPI_MAX_JUMP    = 1.0f;    // rad mechanical between consecutive reads
// The guard FORCE-ACCEPTS after this many consecutive rejections. Without it, a
// rest position more than SPI_MAX_JUMP from the last accepted sample froze the
// angle until reboot -- with the motor armed, a frozen commutation vector
// (FAILURE_MODES §12). A corrupted sample is held at most ~0.2 ms.
const uint8_t SPI_JUMP_MAX_RUN = 3;

class MT6816SPI : public Sensor {
public:
  void init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // CSN / MOSI / SCK -> push-pull outputs, very high speed.
    // MISO -> input. No pull: the MT6816 drives it push-pull in SPI mode.
    const uint8_t outs[3] = { SPI_CSN_BIT, SPI_MOSI_BIT, SPI_SCK_BIT };
    for (uint8_t k = 0; k < 3; k++) {
      uint8_t p = outs[k];
      GPIOB->MODER   &= ~(3UL << (p * 2));
      GPIOB->MODER   |=  (1UL << (p * 2));    // 01 = general purpose output
      GPIOB->OTYPER  &= ~(1UL << p);          // push-pull
      GPIOB->OSPEEDR |=  (3UL << (p * 2));    // very high speed
      GPIOB->PUPDR   &= ~(3UL << (p * 2));    // no pull
    }
    GPIOB->MODER &= ~(3UL << (SPI_MISO_BIT * 2));   // 00 = input
    GPIOB->PUPDR &= ~(3UL << (SPI_MISO_BIT * 2));

    // Idle state: CSN high (deselected), SCK HIGH (mode 3 requires CPOL=1).
    GPIOB->BSRR = (1UL << SPI_CSN_BIT) | (1UL << SPI_SCK_BIT);
    GPIOB->BSRR = (1UL << (SPI_MOSI_BIT + 16));

    // TPwrUp is 16 ms from VDD: wait explicitly rather than rely on boot delays.
    delay(20);

    last_ok_rad = 0.0f;
    (void)readAngleRaw();          // prime last_ok_rad and the error counters
    last_ok_rad = raw * ENC_RAD_PER_COUNT;

    this->Sensor::init();
  }

  // ---- one 16-bit mode-3 transfer -------------------------------------------
  // CPOL=1 CPHA=1: SCK idles high; data changes on the falling edge and is
  // sampled on the rising edge (datasheet 8.6.2 / figure 17).
  uint16_t xfer16(uint16_t out) {
    uint16_t in = 0;
    GPIOB->BSRR = (1UL << (SPI_CSN_BIT + 16));      // CSN low -> start
    spiHalf();                                      // TL: CSN fall -> first SCK fall
    for (int8_t i = 15; i >= 0; i--) {
      GPIOB->BSRR = (1UL << (SPI_SCK_BIT + 16));    // SCK falling edge
      if (out & (1UL << i)) GPIOB->BSRR = (1UL << SPI_MOSI_BIT);
      else                  GPIOB->BSRR = (1UL << (SPI_MOSI_BIT + 16));
      spiHalf();
      GPIOB->BSRR = (1UL << SPI_SCK_BIT);           // SCK rising edge -> sample
      spiHalf();
      in <<= 1;
      if (GPIOB->IDR & (1UL << SPI_MISO_BIT)) in |= 1;
    }
    spiHalf();                                      // TH: last SCK rise -> CSN rise
    GPIOB->BSRR = (1UL << SPI_CSN_BIT);             // CSN high -> stop
    return in;
  }

  // ---- read 0x03 + 0x04, check parity, extract the 14-bit angle --------------
  // Returns true if parity passed. On failure `raw` is left at its previous value.
  bool readAngleRaw() {
    uint8_t d03 = (uint8_t)(xfer16(0x8300) & 0xFF);   // 0x80 = read, addr 0x03
    uint8_t d04 = (uint8_t)(xfer16(0x8400) & 0xFF);
    uint16_t word = ((uint16_t)d03 << 8) | d04;
    // PC (0x04[0]) is EVEN parity over 0x03[7:0] + 0x04[7:1], so the whole
    // 16-bit word must have even parity. This is the corruption detector that
    // ABZ structurally could not provide.
    uint16_t p = word;
    p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    if (p & 1) { spi_err++; return false; }          // ODD -> corrupted frame
    raw     = (uint16_t)(((uint16_t)d03 << 6) | (d04 >> 2));   // 14 bits
    no_mag  = (d04 >> 1) & 1;
    // ALL-ZERO FRAME. 0x0000 has even parity and No_Mag = 0, so a chip that
    // never drives MISO reads as a perfect magnet parked at count 0 -- 'e'
    // printed PASS on exactly that (J03, 2026-10-08). The mirror case, MISO
    // stuck HIGH, is 0xFFFF: also even parity, but No_Mag = 1, so it is caught.
    // The driver only COUNTS; callers decide, because a live chip parked at
    // exactly count 0 sends the same word and does not jitter at rest.
    // Untouched on a parity failure: a failed frame toggled bits, but it is
    // not evidence either way about the angle.
    if (word == 0) { if (zero_run != UINT32_MAX) zero_run++; }
    else           zero_run = 0;
    spi_ok++;
    return true;
  }

  // SimpleFOC calls this every loopFOC(); must return 0..2PI.
  float getSensorAngle() override {
    if (!readAngleRaw()) return last_ok_rad;         // stale for ONE cycle, then recovers
    float a = raw * ENC_RAD_PER_COUNT;
    if (SPI_JUMP_GUARD) {
      float d = a - last_ok_rad;
      while (d >  _PI) d -= _2PI;                    // wrap to +-PI
      while (d < -_PI) d += _2PI;
      if (fabsf(d) > SPI_MAX_JUMP && jump_run < SPI_JUMP_MAX_RUN) {
        jump_run++; spi_jump++; return last_ok_rad;
      }
      // Either the step was plausible, or it has repeated SPI_JUMP_MAX_RUN times
      // and is therefore where the shaft actually IS. Re-seed and carry on --
      // never hold a reference the plant has left behind.
      jump_run = 0;
    }
    last_ok_rad = a;
    return a;
  }

  // Over_Speed lives in 0x05 and is not needed at loop rate. Poll it from the
  // telemetry block instead of paying for a third frame every cycle.
  bool readOverSpeed() {
    uint8_t d05 = (uint8_t)(xfer16(0x8500) & 0xFF);
    over_speed = (d05 >> 3) & 1;
    return over_speed;
  }

  // diagnostics
  int32_t  rawCount()   { return (int32_t)raw; }     // 0..ENC_CPR-1, one mech rev
  uint16_t raw        = 0;
  uint8_t  no_mag     = 0;
  uint8_t  over_speed = 0;
  uint32_t spi_ok     = 0;
  uint32_t spi_err    = 0;
  uint32_t spi_jump   = 0;
  uint32_t zero_run   = 0;      // consecutive parity-passing 0x0000 frames (see readAngleRaw)

private:
  float   last_ok_rad = 0.0f;
  uint8_t jump_run    = 0;      // consecutive jump rejections; bounds the guard
};
