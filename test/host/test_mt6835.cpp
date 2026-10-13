// Host test for src/mt6835.h: the REAL driver source on mocked GPIO registers,
// talking to a simulated MT6835 slave (bit-level, SPI mode 3).
//   frames      command words, 48-clock frame, angle/status assembly, ENC_BITS
//               truncation, CRC (reference bitwise form, not the table)
//   failures    every single-bit error, random multi-bit escape rate, all-zero
//               and stuck-high frames, power-up give-up and recovery
//   link health linkTripped() / linkArmable(): glitch, runs, all-zero, rates
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <climits>
#include <random>

// ---- Arduino / SimpleFOC / fleet_config stand-ins -------------------------
static uint32_t fake_ms = 0;
static bool     auto_tick = true;            // power-up loop needs time to pass
static uint32_t millis() { return auto_tick ? fake_ms++ : fake_ms; }
#define _PI  3.14159265359f
#define _2PI 6.28318530718f
class Sensor { public: virtual float getSensorAngle() = 0; virtual void init() {} virtual ~Sensor() {} };
static constexpr uint8_t  ENC_BITS = 14;
static constexpr uint32_t ENC_CPR  = 1UL << ENC_BITS;
static constexpr float    ENC_RAD_PER_COUNT = 6.28318531f / (float)ENC_CPR;

// ---- reference CRC: SimpleFOC MT6835::calcCrc, bitwise ----------------------
static uint8_t refCrc(uint32_t angle, uint8_t status) {
  uint8_t crc = 0;
  const uint8_t in[3] = { (uint8_t)(angle >> 13), (uint8_t)((angle >> 5) & 0xFF),
                          (uint8_t)(((angle << 3) & 0xFF) | (status & 7)) };
  for (int b = 0; b < 3; b++) {
    crc ^= in[b];
    for (int k = 8; k > 0; k--) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}
static uint32_t frameWord(uint32_t a, uint8_t st) {
  const uint8_t b0 = a >> 13, b1 = (a >> 5) & 0xFF, b2 = ((a << 3) & 0xFF) | (st & 7);
  return ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | refCrc(a, st);
}

// ---- slave model --------------------------------------------------------------
struct Slave {
  bool cs = true, sck = true, mosi = false, miso = false;
  int  rises = 0, falls = 0;
  uint32_t rx = 0, tx = 0; int tx_bits = 0;
  uint32_t angle21 = 0; uint8_t status = 0;
  bool force_raw = false; uint32_t raw_word = 0;   // answer this 32-bit word verbatim
  uint32_t xor_mask = 0;                           // corrupt the answer
  uint8_t regs[0x200] = {0};
  uint32_t last_cmd = 0; int last_clocks = 0; int bad_idle = 0;
  void csFall() { if (!sck) bad_idle++; rises = falls = 0; rx = 0; tx_bits = 0; }
  void csRise() { if (!sck) bad_idle++; last_clocks = rises; }
  void sckFall() {
    if (cs) return;
    if (falls == 16) {                       // command complete: prepare the answer
      last_cmd = rx & 0xFFFF;
      const uint32_t op = last_cmd >> 12, addr = last_cmd & 0x0FFF;
      if (op == 0xA && addr == 0x003) { tx = (force_raw ? raw_word : frameWord(angle21, status)) ^ xor_mask; tx_bits = 32; }
      else if (op == 0x3)             { tx = regs[addr & 0x1FF]; tx_bits = 8; }
      else                            { tx = 0; tx_bits = 0; }
    }
    if (falls >= 16) { const int i = falls - 16; miso = (i < tx_bits) ? ((tx >> (tx_bits - 1 - i)) & 1) : false; }
    falls++;
  }
  void sckRise() { if (cs) return; if (rises < 16) rx = (rx << 1) | (mosi ? 1 : 0); rises++; }
};
static Slave slave;

// ---- GPIO / RCC register mocks --------------------------------------------------
struct PlainReg {
  uint32_t v = 0;
  PlainReg& operator=(uint32_t x)  { v = x;  return *this; }
  PlainReg& operator|=(uint32_t x) { v |= x; return *this; }
  PlainReg& operator&=(uint32_t x) { v &= x; return *this; }
  operator uint32_t() const { return v; }
};
static void pinSet(int bit, bool level) {
  if (bit == 5 && slave.cs != level)  { slave.cs = level;  level ? slave.csRise()  : slave.csFall(); }
  if (bit == 6)                         slave.mosi = level;
  if (bit == 8 && slave.sck != level) { slave.sck = level; level ? slave.sckRise() : slave.sckFall(); }
}
struct BsrrReg { void operator=(uint32_t v) {
  for (int b = 0; b < 16; b++) if (v & (1u << (b + 16))) pinSet(b, false);
  for (int b = 0; b < 16; b++) if (v & (1u << b))        pinSet(b, true);
} };
struct IdrReg { operator uint32_t() const { return slave.miso ? (1u << 7) : 0u; } };
struct GPIO_TypeDef { PlainReg MODER, OTYPER, OSPEEDR, PUPDR; BsrrReg BSRR; IdrReg IDR; };
struct RCC_TypeDef  { PlainReg AHB2ENR; };
static GPIO_TypeDef gpiob; static RCC_TypeDef rcc;
#define GPIOB (&gpiob)
#define RCC   (&rcc)
#define RCC_AHB2ENR_GPIOBEN 2u

#include "mt6835.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; if (fails < 25) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static void good(uint32_t a = 1000)   { slave.force_raw = false; slave.xor_mask = 0; slave.angle21 = a; slave.status = 0; }
static void crcBad()                  { slave.force_raw = false; slave.xor_mask = 1u << 9; slave.angle21 = 1000; }
static void allZero()                 { slave.force_raw = true;  slave.raw_word = 0; slave.xor_mask = 0; }

int main() {
  std::mt19937 rng(12345);

  // ---- 1. init: valid at once, config registers decoded ----------------------
  slave.regs[0x009] = 0xAB; slave.regs[0x00A] = 0xC5; slave.regs[0x00D] = 0x0B; slave.regs[0x011] = 0x05;
  good(0x12345);
  MT6835SPI enc; enc.init();
  CHECK(enc.boot_ok && enc.spi_err == 0, "init boot_ok=%d err=%u", enc.boot_ok, enc.spi_err);
  CHECK(enc.cfg_zero == 0xABC && enc.cfg_rot_dir == 1 && enc.cfg_hyst == 3 && enc.cfg_bw == 5,
        "cfg zero=%X rot=%u hyst=%u bw=%u", enc.cfg_zero, enc.cfg_rot_dir, enc.cfg_hyst, enc.cfg_bw);
  CHECK(slave.last_cmd == 0x3011 && slave.last_clocks == 24, "reg read cmd=%X clocks=%d", slave.last_cmd, slave.last_clocks);

  // ---- 2. random valid frames ---------------------------------------------------
  for (int n = 0; n < 20000; n++) {
    const uint32_t a = rng() & 0x1FFFFF; const uint8_t st = rng() & 7;
    slave.angle21 = a; slave.status = st;
    CHECK(enc.readAngleRaw(), "valid frame rejected a=%X", a);
    CHECK(slave.last_cmd == 0xA003 && slave.last_clocks == 48, "burst cmd=%X clocks=%d", slave.last_cmd, slave.last_clocks);
    CHECK(enc.angle21 == a && enc.raw == (a >> (21 - ENC_BITS)), "angle21 %X raw %u", enc.angle21, enc.raw);
    CHECK(enc.over_speed == (st & 1) && enc.weak_field == ((st >> 1) & 1) && enc.under_volt == ((st >> 2) & 1)
          && enc.no_mag == (((st >> 1) | (st >> 2)) & 1), "status %u", st);
  }
  good(0x1FFFFF);
  { const float ang = enc.getSensorAngle();
    CHECK(fabsf(ang - (0x1FFFFF >> 7) * ENC_RAD_PER_COUNT) < 1e-6f && ang < _2PI, "angle %f", ang); }

  // ---- 3. every single-bit error is rejected and raw is held ---------------------
  for (int bit = 0; bit < 32; bit++) {
    good(0x0ABCDE); enc.readAngleRaw(); const uint16_t held = enc.raw; const uint32_t e0 = enc.spi_err;
    slave.angle21 = 0x155555; slave.xor_mask = 1u << bit;
    CHECK(!enc.readAngleRaw() && enc.spi_err == e0 + 1 && enc.raw == held, "bit %d flip accepted", bit);
  }
  int escaped = 0; const int NC = 100000;
  for (int n = 0; n < NC; n++) {
    slave.force_raw = false; slave.angle21 = rng() & 0x1FFFFF; slave.status = rng() & 7;
    uint32_t m = rng(); slave.xor_mask = m ? m : 1;
    if (enc.readAngleRaw()) escaped++;
  }
  printf("  random multi-bit corruption accepted: %d / %d (%.3f%%; CRC-8 ideal 0.39%%)\n", escaped, NC, 100.0 * escaped / NC);
  CHECK(escaped < NC / 200, "too many corrupt frames accepted: %d", escaped);

  // ---- 4. all-zero passes the CRC and is counted; stuck-high fails ----------------
  allZero();
  for (int k = 0; k < 5; k++) CHECK(enc.readAngleRaw(), "all-zero frame should pass the CRC");
  CHECK(enc.zero_run == 5 && enc.raw == 0, "zero_run %u", enc.zero_run);
  good(); CHECK(enc.readAngleRaw() && enc.zero_run == 0, "zero_run not cleared");
  slave.force_raw = true; slave.raw_word = 0xFFFFFFFF;
  CHECK(!enc.readAngleRaw(), "stuck-high frame accepted");

  // ---- 5. power-up: gives up and keeps evidence; recovers when valid ---------------
  { MT6835SPI e2; slave.force_raw = true; slave.raw_word = 0xDEADBEEF; e2.init();
    CHECK(!e2.boot_ok && e2.spi_err > 0 && e2.pwrup_ms >= MT35_PWRUP_MS, "dead chip: boot_ok=%d err=%u", e2.boot_ok, e2.spi_err);
    good(777); MT6835SPI e3; e3.init();
    CHECK(e3.boot_ok && e3.spi_err == 0 && e3.raw == (777 >> 7), "recovery boot"); }

  // ---- 6. LINK HEALTH ----------------------------------------------------------------
  auto_tick = false; fake_ms = 100000;
  MT6835SPI h; good(); h.init();             // auto_tick off: init sees no time pass
  fake_ms += 2000;
  for (int k = 0; k < 100; k++) h.readAngleRaw();
  CHECK(!h.linkTripped() && h.linkArmable(fake_ms), "healthy link: tripped=%d armable=%d", h.linkTripped(), h.linkArmable(fake_ms));
  // one glitch: never trips; blocks arming for exactly ENC_ARM_CLEAN_MS
  crcBad(); h.readAngleRaw(); const uint32_t t_bad = fake_ms;
  CHECK(h.bad_run == 1 && !h.linkTripped() && !h.linkArmable(fake_ms), "glitch: bad_run=%u", h.bad_run);
  good(); h.readAngleRaw();
  CHECK(h.bad_run == 0 && !h.linkTripped(), "glitch recovery: bad_run=%u", h.bad_run);
  fake_ms = t_bad + ENC_ARM_CLEAN_MS - 1; CHECK(!h.linkArmable(fake_ms), "armable before the clean window");
  fake_ms = t_bad + ENC_ARM_CLEAN_MS;     CHECK(h.linkArmable(fake_ms),  "not armable after the clean window");
  // runs: 3 in a row does not trip, 4 does; one good frame clears the trip
  crcBad(); for (int k = 0; k < 3; k++) h.readAngleRaw();
  CHECK(!h.linkTripped(), "3 in a row tripped");
  h.readAngleRaw(); CHECK(h.linkTripped(), "4 in a row did not trip");
  good(); h.readAngleRaw(); CHECK(!h.linkTripped(), "trip did not clear on a good frame");
  // all-zero frames count as invalid
  allZero(); for (int k = 0; k < ENC_TRIP_BAD_RUN; k++) h.readAngleRaw();
  CHECK(h.linkTripped(), "all-zero frames did not trip");
  good(); h.readAngleRaw();
  // saturation
  crcBad(); for (int k = 0; k < 300; k++) h.readAngleRaw();
  CHECK(h.bad_run == 255 && h.linkTripped(), "bad_run saturation: %u", h.bad_run);
  good(); h.readAngleRaw();
  CHECK(h.bad_run == 0, "saturated run not cleared");

  // rates: reads to the first trip vs the run-of-4 expectation, and how often a
  // 1 s window (11,500 reads) is clean enough to arm
  const double rates[] = { 0.9965, 0.5, 0.2, 0.05 };
  std::bernoulli_distribution coin;
  for (double p : rates) {
    coin = std::bernoulli_distribution(p);
    const int trials = 400; double sum = 0;
    for (int t = 0; t < trials; t++) {
      good(); h.readAngleRaw();
      long n = 0;
      while (!h.linkTripped() && n < 20000000) { if (coin(rng)) crcBad(); else good(); h.readAngleRaw(); n++; }
      sum += n;
    }
    const double mean = sum / trials;
    const double expect = (1 - pow(p, 4)) / ((1 - p) * pow(p, 4));   // reads until 4 in a row
    const double p_arm = pow(1 - p, 11500);
    printf("  p_err %6.4f: mean %9.0f reads to trip (%8.2f ms at 11.5 kHz; run-of-4 theory %9.0f); P(clean 1 s) %.2e\n",
           p, mean, mean / 11.5, expect, p_arm);
    CHECK(fabs(mean - expect) / expect < 0.25, "p=%.4f mean %.0f vs %.0f", p, mean, expect);
  }
  // Arming is stricter: at error rates where a run of 4 takes hours, a clean
  // 1 s window is still improbable, so a marginal link is refused, not armed.
  for (double p : { 1e-3, 1e-4, 1e-5 })
    printf("  p_err %.0e: P(no invalid read in 1 s = 11,500 reads) = %.3f\n", p, pow(1 - p, 11500));
  CHECK(slave.bad_idle == 0, "CS toggled with SCK low %d times (mode 3 idles high)", slave.bad_idle);

  printf(fails ? "*** %d FAILURES ***\n" : "ALL PASS (driver + link health)\n", fails);
  return fails ? 1 : 0;
}
