#pragma once
// ============================================================================
// mt6835.h -- MT6835 21-bit absolute encoder, 4-wire SPI, bit-banged, READ-ONLY.
// ============================================================================
// Presents EXACTLY MT6816SPI's interface (mt6816.h) at ENC_BITS resolution, so
// nothing above the driver changes: raw = ANGLE[20:0] >> (21 - ENC_BITS). The
// estimator, contract LSBs, AUTOCALIB, logs and every count-unit constant keep
// their meaning. Native 21-bit is stage 2, not this file.
//
// Layer: bottom, same as mt6816.h. Depends ONLY on Arduino GPIO, SimpleFOC's
// Sensor base and ENC_BITS / ENC_RAD_PER_COUNT from fleet_config.h.
//
// *** UNIT RISK -- the same three angle units as mt6816.h, plus one. ***
//   angle21            COUNTS at 21 bits, 0..2^21-1 per MECHANICAL rev.
//                      DIAGNOSTIC ONLY -- nothing downstream may use it.
//   rawCount() / raw   COUNTS at ENC_BITS, 0..ENC_CPR-1 per MECHANICAL rev
//   getSensorAngle()   MECHANICAL rad, 0..2*PI. SimpleFOC multiplies by pole pairs.
//
// ---------------------------------------------------------------------------
// CHIP FACTS -- PROVENANCE: NOT a datasheet in hand. From SimpleFOC's
// Arduino-FOC-drivers MT6835 driver (src/encoders/mt6835, fetched 2026-10-12)
// and MagnTek product summaries. A wrong frame layout or CRC reads as ~100%
// CRC errors in 'e', not as a plausible angle, so bench gate E2 confirms them.
//   * SPI mode 3 (CPOL=1, CPHA=1), MSB first. SCK max 16 MHz. VDD 3.0-5.5 V.
//   * Burst angle read, ONE CS frame of 48 clocks: out 0xA0 0x03 (opcode
//     0b1010, address 0x003), then 4 bytes in:
//        b0 = ANGLE[20:13]   b1 = ANGLE[12:5]
//        b2 = ANGLE[4:0] << 3 | STATUS[2:0]     b3 = CRC
//     STATUS bit0 over-speed, bit1 weak field, bit2 under-voltage.
//   * CRC-8 over b0 b1 b2: x^8 + x^2 + x + 1 (0x07), init 0x00, no final XOR.
//   * Register read, 24 clocks: out (0b0011 << 4 | addr >> 8), addr & 0xFF;
//     1 data byte in.
//   * Registers read here: 0x009 ZERO[11:4]; 0x00A ZERO[3:0] in bits 7:4;
//     0x00D ROT_DIR bit3, HYST bits 2:0; 0x011 BW bits 2:0.
//
// READ-ONLY BY CONSTRUCTION. Only opcodes 0b1010 (angle) and 0b0011 (register
// read) exist in this file. The chip's ZERO, ROT_DIR and EEPROM-program
// commands would silently move the angle that zea and dir (joint_cal.h) were
// measured through -- a wrong commutation angle on a joint that looks
// calibrated.
// ============================================================================

// ---------------------------------------------------------------------------
// PINS -- DUPLICATED from mt6816.h on purpose: enc_select.h compiles exactly
// one driver per build, and the MT6816 path is deleted when the last joint
// migrates. Remap BOTH files or neither. All four on GPIOB (BSRR / IDR fast
// path); SCK stays on PB8 (BOOT0 -- see the wiring note in open_test.cpp).
// ---------------------------------------------------------------------------
const uint8_t SPI_CSN_BIT  = 5;    // PB5
const uint8_t SPI_MOSI_BIT = 6;    // PB6
const uint8_t SPI_MISO_BIT = 7;    // PB7
const uint8_t SPI_SCK_BIT  = 8;    // PB8 -- BOOT0

// SCK timing, from the datasheet (Rev 1.3 sec 7.6.2), NOT copied from the MT6816:
//   TSCKL, TSCKH >= 30 ns; MOSI setup TST >= 10 ns before the rising edge;
//   MISO valid TDV <= 15 ns after the falling edge AT 20 pF (Dupont is more).
// HIGH phase: SPI_HALF_NOPS plus the loop's own ~8 cycles (~47 ns, >= TSCKH).
// LOW phase: a countdown in inline asm, ~3 cycles per count -- asm because the
// compiler unrolls a NOP loop and the core may drop a NOP before it executes,
// so NOP counts are not a delay. With no padding the inlined loop held SCK low
// ~3 cycles (~18 ns) and read 99.65% CRC errors on J01 (2026-10-13); the earlier
// 16 us/read build passed only because call overhead stretched each half-period.
// 5 counts + ~4 cycles of stores -> ~19 cycles (~110 ns) low: ~3.5x TSCKL,
// ~7x TDV. If CRC errors appear, raise MT35_SCK_LOW_SPIN.
const uint8_t  SPI_HALF_NOPS     = 1;
const uint32_t MT35_SCK_LOW_SPIN = 5;
// CSN setup (fall -> first SCK) and CSN high time between frames. The MT6835's
// minima are not in hand; ~190 ns each costs 0.4 us per read and keeps 'e''s
// back-to-back reads from violating either. Unverified margin, not a datasheet value.
const uint8_t MT35_CS_NOPS = 32;

// always_inline: the build optimises for size, and without it these became real
// calls -- 48 calls of clk() per read, 16.09 us/read on J01 (2026-10-12) against
// the MT6816's 6.67 us for 32 clocks.
static inline __attribute__((always_inline)) void spiHalf() {
  for (uint8_t i = 0; i < SPI_HALF_NOPS; i++) __asm__ volatile ("nop");
}
static inline __attribute__((always_inline)) void mt35SckLow() {
  uint32_t n = MT35_SCK_LOW_SPIN;
#if defined(__arm__)
  __asm__ volatile ("1: subs %0, %0, #1\n\tbne 1b" : "+r"(n) : : "cc");
#else
  while (n) { __asm__ volatile ("" : "+r"(n)); n--; }   // host unit test only
#endif
}
static inline __attribute__((always_inline)) void mt35CsGap() {
  for (uint8_t i = 0; i < MT35_CS_NOPS; i++) __asm__ volatile ("nop");
}

// Jump guard: same logic and the same default (OFF) as mt6816.h -- see there
// for why it force-accepts after SPI_JUMP_MAX_RUN rejections.
const bool    SPI_JUMP_GUARD   = false;
const float   SPI_MAX_JUMP     = 1.0f;    // rad mechanical between consecutive reads
const uint8_t SPI_JUMP_MAX_RUN = 3;

// Driver identity for the shared console text (same names as mt6816.h).
#define ENC_NAME  "MT6835"
#define ENC_CHECK "crc"
#define ENC_FLAG  "weak field or VDD under-voltage"
// 'e' / 'E' still-shaft span limit. 21 bits truncated to ENC_BITS: a shaft
// parked on a count boundary flickers by one count from the chip's own noise.
// Two or more counts on a still shaft is corruption.
const uint16_t ENC_STILL_SPAN_MAX = 1;

// raw is uint16_t and the chip has 21 bits.
static_assert(ENC_BITS <= 16, "MT6835 raw is uint16_t: ENC_BITS > 16 needs the stage-2 type changes");
const uint8_t MT35_SHIFT = 21 - ENC_BITS;

const uint16_t MT35_PWRUP_MS = 100;   // wait at most this long for the first CRC-valid frame

// ---------------------------------------------------------------------------
// CRC-8, poly 0x07, init 0x00. Table generated from a Python port of the
// reference calcCrc() and cross-checked against it on 10,000 random frames;
// the static_assert below re-derives every entry at compile time.
// ---------------------------------------------------------------------------
static constexpr uint8_t MT35_CRC_TABLE[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
};

// C++11 constexpr (gnu++11 here): single-expression recursion only.
constexpr uint8_t mt35CrcBits(uint8_t c, uint8_t k) {
  return k ? mt35CrcBits((uint8_t)((c & 0x80) ? ((c << 1) ^ 0x07) : (c << 1)), (uint8_t)(k - 1)) : c;
}
constexpr bool mt35CrcTableOk(uint16_t i) {
  return i == 256 ? true
       : (MT35_CRC_TABLE[i] == mt35CrcBits((uint8_t)i, 8) && mt35CrcTableOk((uint16_t)(i + 1)));
}
static_assert(mt35CrcTableOk(0), "MT35_CRC_TABLE does not match poly 0x07");
constexpr uint8_t mt35Crc3(uint8_t b0, uint8_t b1, uint8_t b2) {
  return MT35_CRC_TABLE[MT35_CRC_TABLE[MT35_CRC_TABLE[b0] ^ b1] ^ b2];
}
// THE ALL-ZERO TRAP, carried over from the MT6816: a MISO that never drives
// returns 00 00 00 00, and CRC8(00 00 00) is 00 -- it PASSES, exactly as it
// passed parity. zero_run below is the only detector for it.
static_assert(mt35Crc3(0x00, 0x00, 0x00) == 0x00, "all-zero frame passes the CRC -- zero_run must stay");
// MISO stuck HIGH (FF FF FF FF) FAILS: CRC8(FF FF FF) = 0x0F, not 0xFF.
static_assert(mt35Crc3(0xFF, 0xFF, 0xFF) == 0x0F, "stuck-high MISO must fail the CRC");

class MT6835SPI : public Sensor {
public:
  void init() {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // CSN / MOSI / SCK -> push-pull outputs, very high speed. MISO -> input, no
    // pull (same as the MT6816: one variable at a time).
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

    // Idle: CSN high (deselected), SCK HIGH (mode 3), MOSI low.
    GPIOB->BSRR = (1UL << SPI_CSN_BIT) | (1UL << SPI_SCK_BIT);
    GPIOB->BSRR = (1UL << (SPI_MOSI_BIT + 16));

    // POWER-UP. The MT6835's power-up time is not in hand, so wait for the
    // first CRC-valid frame instead of guessing a delay. Failures while it
    // starts are not link errors and are discarded -- but only on success: if
    // nothing passes in MT35_PWRUP_MS the counters keep the evidence.
    const uint32_t t0 = millis();
    while (!readAngleRaw() && millis() - t0 < MT35_PWRUP_MS) { }
    pwrup_ms = (uint16_t)(millis() - t0);
    boot_ok  = (spi_ok > 0);
    if (boot_ok) { spi_err = 0; spi_ok = 0; }
    last_ok_rad = raw * ENC_RAD_PER_COUNT;

    // CHIP CONFIG, read once for the boot banner: these set where the angle's
    // zero is, which way it counts, and its filter lag -- all things zea, dir
    // and T/T_loop were measured under. Register reads carry no CRC: a
    // diagnostic, never acted on.
    const uint8_t z_hi = readReg(0x009), z_lo = readReg(0x00A);
    const uint8_t o3   = readReg(0x00D), o5   = readReg(0x011);
    cfg_zero    = (uint16_t)(((uint16_t)z_hi << 4) | (z_lo >> 4));
    cfg_rot_dir = (o3 >> 3) & 1;
    cfg_hyst    = o3 & 0x07;
    cfg_bw      = o5 & 0x07;

    this->Sensor::init();
  }

  // ---- one burst angle read: 16 clocks out, 32 in, one CS frame -------------
  // Returns true if the CRC passed. On failure raw is left at its previous value.
  bool readAngleRaw() {
    GPIOB->BSRR = (1UL << (SPI_CSN_BIT + 16));      // CSN low -> start
    mt35CsGap();
    const uint16_t cmd = 0xA003;                    // opcode 0b1010, address 0x003
    for (int8_t i = 15; i >= 0; i--) (void)clk((cmd >> i) & 1);
    uint32_t w = 0;
    for (uint8_t i = 0; i < 32; i++) w = (w << 1) | clk(false);
    spiHalf();
    GPIOB->BSRR = (1UL << SPI_CSN_BIT);             // CSN high -> stop
    mt35CsGap();

    const uint8_t b0 = (uint8_t)(w >> 24), b1 = (uint8_t)(w >> 16);
    const uint8_t b2 = (uint8_t)(w >> 8),  b3 = (uint8_t)w;
    if (mt35Crc3(b0, b1, b2) != b3) { spi_err++; return false; }
    angle21    = ((uint32_t)b0 << 13) | ((uint32_t)b1 << 5) | (uint32_t)(b2 >> 3);
    raw        = (uint16_t)(angle21 >> MT35_SHIFT);
    status     = b2 & 0x07;
    over_speed = status & 0x01;
    weak_field = (status >> 1) & 1;
    under_volt = (status >> 2) & 1;
    // no_mag is the name every caller (Tier-0 guard, 'e', AUTOCALIB) checks.
    // Under-voltage is in it too: an angle measured at low VDD is not trusted.
    no_mag     = weak_field | under_volt;
    if (w == 0) { if (zero_run != UINT32_MAX) zero_run++; }   // see the all-zero trap above
    else        zero_run = 0;
    spi_ok++;
    return true;
  }

  // SimpleFOC calls this every loopFOC(); must return 0..2PI.
  float getSensorAngle() override {
    if (!readAngleRaw()) return last_ok_rad;         // stale for ONE cycle, then recovers
    float a = raw * ENC_RAD_PER_COUNT;
    if (SPI_JUMP_GUARD) {
      float d = a - last_ok_rad;
      while (d >  _PI) d -= _2PI;
      while (d < -_PI) d += _2PI;
      if (fabsf(d) > SPI_MAX_JUMP && jump_run < SPI_JUMP_MAX_RUN) {
        jump_run++; spi_jump++; return last_ok_rad;
      }
      jump_run = 0;
    }
    last_ok_rad = a;
    return a;
  }

  // Over-speed arrives in EVERY frame on this chip, so there is nothing to
  // fetch: returns the flag from the last CRC-valid frame (the MT6816 spends a
  // third frame on register 0x05 here).
  bool readOverSpeed() { return over_speed; }

  // Read-only register access (opcode 0b0011). Diagnostics only.
  uint8_t readReg(uint16_t addr) {
    GPIOB->BSRR = (1UL << (SPI_CSN_BIT + 16));
    mt35CsGap();
    const uint16_t hdr = (uint16_t)((0x3u << 12) | (addr & 0x0FFF));
    for (int8_t i = 15; i >= 0; i--) (void)clk((hdr >> i) & 1);
    uint8_t d = 0;
    for (uint8_t i = 0; i < 8; i++) d = (uint8_t)((d << 1) | clk(false));
    spiHalf();
    GPIOB->BSRR = (1UL << SPI_CSN_BIT);
    mt35CsGap();
    return d;
  }

  // diagnostics -- same names as MT6816SPI, plus the MT6835's own
  int32_t  rawCount()   { return (int32_t)raw; }     // 0..ENC_CPR-1, one mech rev
  uint16_t raw        = 0;
  uint8_t  no_mag     = 0;
  uint8_t  over_speed = 0;
  uint32_t spi_ok     = 0;
  uint32_t spi_err    = 0;       // CRC failures
  uint32_t spi_jump   = 0;
  uint32_t zero_run   = 0;       // consecutive CRC-passing all-zero frames
  uint32_t angle21    = 0;       // full resolution, DIAGNOSTIC ONLY
  uint8_t  status     = 0;
  uint8_t  weak_field = 0;
  uint8_t  under_volt = 0;
  bool     boot_ok    = false;   // a CRC-valid frame arrived within MT35_PWRUP_MS
  uint16_t pwrup_ms   = 0;
  uint16_t cfg_zero   = 0;       // chip ZERO_POS register, 12 bits
  uint8_t  cfg_rot_dir = 0, cfg_hyst = 0, cfg_bw = 0;

private:
  // One SCK cycle, mode 3: drive MOSI on the falling edge, sample MISO after
  // the rising edge -- the same edge placement as MT6816SPI::xfer16().
  static inline __attribute__((always_inline)) uint32_t clk(bool mosi) {
    GPIOB->BSRR = (1UL << (SPI_SCK_BIT + 16));                     // SCK fall
    GPIOB->BSRR = mosi ? (1UL << SPI_MOSI_BIT) : (1UL << (SPI_MOSI_BIT + 16));
    mt35SckLow();                                                  // TSCKL, TST, TDV
    GPIOB->BSRR = (1UL << SPI_SCK_BIT);                            // SCK rise
    spiHalf();
    return (GPIOB->IDR >> SPI_MISO_BIT) & 1UL;
  }

  float   last_ok_rad = 0.0f;
  uint8_t jump_run    = 0;
};
