/* ============================================================================
 * S1c -- FDCAN LOOPBACK SELF-TEST + HSE FREQUENCY MEASUREMENT
 *                                              rev 2   ESC1 clone / STM32G431
 * ============================================================================
 * SCOPE: diagnostic instrument. Standalone. NO SimpleFOC, NO open_test.cpp,
 *   NO joint_cal.h. Throwaway -- do not grow it into Tier 0.
 *
 * rev 2 adds mode 'n' (NORMAL, for S2), lastid, EP/EW/BO decode, and the Tx
 *   Event FIFO drop counter (txok= / drop=). The drop counter is what measured
 *   0 drops in ~10,000 frames and confirmed DAR=1 acting in both directions.
 *
 * *** THIS FILE IS MIRRORED INTO THE M0 Actuator REPO at
 *     tools/can_bringup/esc1_s1c/. It is an INSTRUMENT, not a throwaway any
 *     more: it re-validates every board during the 12-board termination
 *     rework (docs/CAN_BRINGUP.md 23.2). Edit here, then re-copy. ***
 *
 * *** MOTOR MUST BE DISCONNECTED. ***
 * *** NOTHING CONNECTED TO CANH / CANL. Single node, its own 121 ohm. ***
 *
 * ---------------------------------------------------------------------------
 * THREE MODES, EACH ADDING EXACTLY ONE UNKNOWN
 *
 *   'i'  S1c-i  INTERNAL loopback, 1 Mbit at the assumed HSE
 *               TEST.LBCK = 1, CCCR.MON = 1.
 *               Proves: HSE reaches the peripheral, FDCAN configures, message
 *               RAM and FIFO work.
 *               Does NOT prove: pin mux, transceiver, or the HSE FREQUENCY --
 *               TX and RX share one clock, so this passes at ANY crystal value.
 *               *** PB9 IS HELD RECESSIVE IN THIS MODE (AN5348). The analyzer
 *               will show NOTHING. That is correct, not a failure. ***
 *
 *   'e'  S1c-e  EXTERNAL loopback, 1 Mbit at the assumed HSE
 *               TEST.LBCK = 1, CCCR.MON = 0.
 *               Adds: the TX pin is DRIVEN. *** BUT YOU CANNOT WATCH IT.
 *               PB9 AND PA11 ARE PACKAGE PINS WITH NO PAD on this board --
 *               the exposed edge is GND 3V SWD SCK PWM BUTTON POTEN TXD RXD
 *               GND 5V CANL CANH PB5 RST, and CANH/CANL are the only
 *               CAN-carrying pads. Neither decodes on the FX2: at VCC 5 V,
 *               CANH swings 2.48 -> 3.88 V (both above Vih 2.0, so it never
 *               changes state) and CANL swings 2.48 -> 1.12 V (dominant lands
 *               in the undefined zone between Vil 0.8 and Vih 2.0). No
 *               resistor divider fixes either: a divider scales both levels
 *               by the same factor and the two required thresholds are
 *               contradictory. To see ESC1 frames on the analyzer, tap the
 *               ESP32 breakout's RXD -- clean 3.3 V logic. That is S2. ***
 *               *** RETRACTED: this does NOT prove the AF9 mux or the
 *               transceiver path. The M_CAN spec is explicit that in EXTERNAL
 *               loopback "the M_CAN performs an internal feedback from its Tx
 *               output to its Rx input. The actual value of the CAN_RX input
 *               pin is disregarded." BOTH loopback modes feed the receiver
 *               internally; the only difference is whether TX drives. So 'e'
 *               is expected to be byte-identical to 'i' EVEN IF the mux is
 *               wrong and EVEN IF the transceiver is absent. Use 'x'. ***
 *
 *   'm'  S1c-m  EXTERNAL loopback, prescaler x8 -> 125 kbit at the assumed HSE
 *               THE FREQUENCY MEASUREMENT. 8 us bits instead of 1 us, so a
 *               10-bit cursor span at 24 MSa/s resolves to ~0.05% -- easily
 *               separating 24 MHz from 25 MHz.
 *                     f_HSE = 8 MHz x (80.0 us / measured 10-bit span)
 *               SUPERSEDED by 'q': the timestamp counter settled the crystal at
 *               7.983 MHz = 8.000 MHz (-0.21%, runner-up 12 MHz is +50% away).
 *
 *   'n'  S2     NORMAL mode, 1 Mbit. Needs a partner node and real wiring.
 *               The first mode that is not loopback, so the first that
 *               exercises ACK, arbitration and the transceiver loop delay.
 *
 *   'x'  S1c-x  THE MUX TEST. Floods TX in external loopback and samples
 *               TEST.RX (bit 7), which monitors the ACTUAL FDCAN_RX pin rather
 *               than the internal feedback. A dominant reading requires
 *               PB9 mux -> transceiver -> bus -> PA11 mux -> peripheral, so
 *               this is what 'e' was wrongly credited with proving.
 *               *** REQUIRES CCCR.TEST = 1, which the HAL sets ONLY in the
 *               loopback modes -- hence the 'e'/'m' guard. In mode 'n' the
 *               TEST register reads reset values, not the pin. ***
 *
 * Loopback ignores ACK errors (AN5348), so no partner is needed in any mode.
 *
 * Transceiver loop delay measured at S1b = 125 ns, against an 875 ns sample
 * point at 1 Mbit. AN5348 warns this delay sets a lower limit on TSEG1; 7x
 * margin here, and 56x in 'm'. Not a concern, but it is the reason S1b's
 * loop-delay number was worth taking.
 *
 * ---------------------------------------------------------------------------
 * WHY ST's HAL AND NOT BARE REGISTERS
 *   The clock is the thing that goes wrong, so the clock is done BY HAND here
 *   -- HSEON, HSERDY poll, FDCANSEL -- and every value is printed. The message
 *   RAM layout is where recall is weakest (~70%), and it is exactly where ST's
 *   driver is authoritative. The compromise: let HAL do the allocation, then
 *   READ NBTP/CCCR/TEST BACK AND PRINT THEM, so the library's arithmetic is
 *   checked against ours rather than trusted. A library default is a decision
 *   nobody made -- unless you read it back.
 *
 * ---------------------------------------------------------------------------
 * BUILD
 *   Separate PlatformIO project (NOT an env in M0 Actuator -- src/ there is
 *   picked up by all 13 joint envs and would collide with open_test.cpp).
 *   Copy the whole [env:J01] block, rename it, delete lib_deps, and ADD:
 *
 *     build_flags = ${env:J01.build_flags} -D HAL_FDCAN_MODULE_ENABLED
 *
 *   If the linker still cannot find HAL_FDCAN_Init, create a file named
 *   hal_conf_extra.h in the project root containing:
 *       #define HAL_FDCAN_MODULE_ENABLED
 *   That is the Arduino-STM32 core's supported override path.
 *
 * KNOWN HAL VERSION TRAP
 *   FDCAN_TxHeaderTypeDef.DataLength is FDCAN_DLC_BYTES_8 (== 8 << 16) on
 *   older G4 HAL and plain 8 on newer. If it will not compile, or if the
 *   received DLC prints as something other than 8, swap TX_DLC below.
 * ========================================================================= */

#include <Arduino.h>

/* Copy this line EXACTLY from your working open_test.cpp. */
HardwareSerial SerialUART(PB4, PB3);   // (RX, TX) -- USART2, the only one out

/* ---------------------------------------------------------------------------
 * CONFIG
 * ------------------------------------------------------------------------ */

/* The crystal frequency is UNKNOWN -- no resolvable marking. Everything below
 * is computed from this assumption, and mode 'm' measures the truth. Leave it
 * at 8 MHz until the analyzer says otherwise; the assumption only scales the
 * answer, it cannot make the test fail. */
static const uint32_t HSE_ASSUMED_HZ = 8000000UL;

/* One bit = 8 tq: 1 sync + 6 (TSEG1) + 1 (TSEG2). Sample point 87.5%. */
static const uint32_t TSEG1 = 6, TSEG2 = 1, SJW = 1;
static const uint32_t TQ_PER_BIT = 1 + TSEG1 + TSEG2;          // 8

static const uint32_t PRESC_FAST = 1;   // -> 1.000 Mbit at 8 MHz assumed
static const uint32_t PRESC_SLOW = 8;   // -> 125.0 kbit at 8 MHz assumed

static const uint32_t TX_ID      = 0x200;   // ESC1 transmits
static const uint32_t PARTNER_ID = 0x100;   // ESP32 nodeA transmits
/* ESP32 on the LOWER id wins every arbitration collision, which keeps the
 * ESC1's behaviour the interesting one to watch. */
static const uint32_t TX_PERIOD_MS = 5;      // 200 Hz, same as the ESP32 side
static const uint32_t STATUS_MS    = 1000;

#define TX_DLC  FDCAN_DLC_BYTES_8            // see KNOWN HAL VERSION TRAP

/* PC11 = CAN_SHD -> transceiver pin 8 (S). MEASURED at S1b: LOW = Normal,
 * HIGH = Standby, FLOATING = Standby. Not optional, and invisible when
 * missing: in Standby the transmitter is off and RXD never mirrors TXD,
 * which presents exactly like a dead transceiver. */
#define MODE_PORT  GPIOC
#define MODE_BIT   11

/* ------------------------------------------------------------------------ */

static FDCAN_HandleTypeDef h;
static uint32_t tx_count = 0, rx_count = 0, rx_bad = 0;
static uint32_t last_rx_id = 0;                 // which node we last heard from
static uint32_t tx_ok = 0;                      // frames actually TRANSMITTED (Tx Event
                                                // FIFO), vs tx_count merely queued
static uint32_t last_tx = 0, last_status = 0;
static bool     running = false, tx_enabled = true;
static char     cur_mode = '-';

static void p(const char* s) { SerialUART.println(s); }

/* ---- clock: done by hand, because this is what goes wrong -------------- */

static bool enableHSE() {
  bool was_on = (RCC->CR & RCC_CR_HSEON) != 0;
  if (!was_on) RCC->CR |= RCC_CR_HSEON;
  uint32_t t0 = millis();
  while (!(RCC->CR & RCC_CR_HSERDY)) {
    if (millis() - t0 > 100) return false;
  }
  return true;
}

/* Read NBTP back and derive the bit rate the hardware will ACTUALLY produce,
 * from the assumed kernel clock. This is the check on the HAL, not a repeat
 * of our own input. */
static void printTiming() {
  uint32_t nbtp  = FDCAN1->NBTP;
  uint32_t nsjw  = ((nbtp >> 25) & 0x7F) + 1;
  uint32_t nbrp  = ((nbtp >> 16) & 0x1FF) + 1;
  uint32_t nts1  = ((nbtp >>  8) & 0xFF)  + 1;
  uint32_t nts2  = ( nbtp        & 0x7F)  + 1;
  uint32_t tq    = 1 + nts1 + nts2;
  float    rate  = (float)HSE_ASSUMED_HZ / (float)(nbrp * tq);

  SerialUART.print("  NBTP = 0x"); SerialUART.println(nbtp, HEX);
  SerialUART.print("    NBRP="); SerialUART.print(nbrp);
  SerialUART.print("  TSEG1="); SerialUART.print(nts1);
  SerialUART.print("  TSEG2="); SerialUART.print(nts2);
  SerialUART.print("  SJW=");   SerialUART.print(nsjw);
  SerialUART.print("  -> ");    SerialUART.print(tq);
  SerialUART.print(" tq/bit, sample point ");
  SerialUART.print(100.0f * (1 + nts1) / tq, 1); SerialUART.println("%");
  SerialUART.print("    IF HSE = "); SerialUART.print(HSE_ASSUMED_HZ / 1000000UL);
  SerialUART.print(" MHz  ->  bit rate "); SerialUART.print(rate / 1000.0f, 2);
  SerialUART.print(" kbit/s, one bit = ");
  SerialUART.print(1000000.0f / rate, 3); SerialUART.println(" us");
  SerialUART.print("    MEASURE 10 BITS ON THE ANALYZER: expect ");
  SerialUART.print(10.0f * 1000000.0f / rate, 2); SerialUART.println(" us");

  SerialUART.print("  CCCR = 0x"); SerialUART.print(FDCAN1->CCCR, HEX);
  SerialUART.print("   MON=");     SerialUART.print((FDCAN1->CCCR >> 5) & 1);
  SerialUART.print(" TEST=");      SerialUART.print((FDCAN1->CCCR >> 7) & 1);
  SerialUART.print("   TEST reg = 0x"); SerialUART.print(FDCAN1->TEST, HEX);
  SerialUART.print("   LBCK=");    SerialUART.println((FDCAN1->TEST >> 4) & 1);
}

/* ---- bring FDCAN up in one of the three modes -------------------------- */

static bool startFDCAN(char mode) {
  if (running) { HAL_FDCAN_Stop(&h); HAL_FDCAN_DeInit(&h); running = false; }

  SerialUART.println();
  SerialUART.print("=== starting mode '"); SerialUART.print(mode);
  SerialUART.println("' ===");

  if (!enableHSE()) {
    p("!! HSERDY TIMEOUT -- the crystal is not oscillating.");
    p("   FDCANSEL selects HSE, so the peripheral would have NO CLOCK and every");
    p("   register write would still appear to succeed. ABORTING ON PURPOSE.");
    return false;
  }
  p("  HSE ready.");

  /* FDCANSEL = 00 = HSE. It is already the reset default, but the Arduino core
   * never enables HSE, so the default is a trap: FDCAN configured to clock from
   * a switched-off oscillator. Written explicitly so the intent is visible. */
  RCC->CCIPR = (RCC->CCIPR & ~(3UL << 24)) | (0UL << 24);
  SerialUART.print("  FDCANSEL set to 0 (HSE). RCC_CCIPR = 0x");
  SerialUART.println(RCC->CCIPR, HEX);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_FDCAN_CLK_ENABLE();

  /* Transceiver OUT OF STANDBY before anything else touches the bus. */
  GPIO_InitTypeDef g = {};
  g.Pin = GPIO_PIN_11; g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MODE_PORT, &g);
  HAL_GPIO_WritePin(MODE_PORT, GPIO_PIN_11, GPIO_PIN_RESET);
  p("  PC11 driven LOW -> transceiver Normal mode (measured at S1b).");

  /* PB9 = FDCAN1_TX, PA11 = FDCAN1_RX, both AF9. */
  g.Mode = GPIO_MODE_AF_PP; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_FDCAN1;
  g.Pin = GPIO_PIN_9;  HAL_GPIO_Init(GPIOB, &g);
  g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOA, &g);

  h.Instance                  = FDCAN1;
  h.Init.ClockDivider         = FDCAN_CLOCK_DIV1;   // remove if it won't compile
  h.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  h.Init.Mode = (mode == 'n') ? FDCAN_MODE_NORMAL
              : (mode == 'i') ? FDCAN_MODE_INTERNAL_LOOPBACK
                              : FDCAN_MODE_EXTERNAL_LOOPBACK;
  h.Init.AutoRetransmission   = DISABLE;   // matches the single-shot decision
  h.Init.TransmitPause        = DISABLE;
  h.Init.ProtocolException    = DISABLE;
  h.Init.NominalPrescaler     = (mode == 'm') ? PRESC_SLOW : PRESC_FAST;
  h.Init.NominalSyncJumpWidth = SJW;
  h.Init.NominalTimeSeg1      = TSEG1;
  h.Init.NominalTimeSeg2      = TSEG2;
  h.Init.DataPrescaler        = 1;         // unused: classic frames only
  h.Init.DataSyncJumpWidth    = 1;
  h.Init.DataTimeSeg1         = TSEG1;
  h.Init.DataTimeSeg2         = TSEG2;
  h.Init.StdFiltersNbr        = 1;
  h.Init.ExtFiltersNbr        = 0;
  h.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&h) != HAL_OK) { p("!! HAL_FDCAN_Init FAILED"); return false; }

  /* Timestamp counter increments once per CAN bit time. Used by 'q' to
   * identify the crystal with no external instrument. Must be written while
   * CCE=1, i.e. after Init and before Start -- both HAL calls reject any
   * state other than HAL_FDCAN_STATE_READY. */
  HAL_FDCAN_ConfigTimestampCounter(&h, FDCAN_TIMESTAMP_PRESC_1);
  HAL_FDCAN_EnableTimestampCounter(&h, FDCAN_TIMESTAMP_INTERNAL);

  FDCAN_FilterTypeDef f = {};
  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterIndex  = 0;
  f.FilterType   = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = 0x000;
  f.FilterID2    = 0x000;                  // mask 0 -> accept everything
  if (HAL_FDCAN_ConfigFilter(&h, &f) != HAL_OK) { p("!! ConfigFilter FAILED"); return false; }
  HAL_FDCAN_ConfigGlobalFilter(&h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  if (HAL_FDCAN_Start(&h) != HAL_OK) { p("!! HAL_FDCAN_Start FAILED"); return false; }

  printTiming();
  if (mode == 'i')
    p("  NOTE: internal loopback holds PB9 RECESSIVE.");
  else
    p("  NOTE: PB9 is driving, but PB9 has NO PAD on this board and CANH/CANL");
  p("        do not decode on the FX2 at 5 V. The analyzer shows NOTHING in any\n"
    "        single-node mode -- that is the board, not a failure. Use 'x' for\n"
    "        the pin-level check; the analyzer's first real look is S2, tapping\n"
    "        the ESP32 breakout's RXD.");

  running = true; cur_mode = mode; tx_count = rx_count = rx_bad = 0; tx_ok = 0;
  return true;
}

/* Payload must ascend by one, same convention as the ESP32 sketch. */
static bool payloadOK(const uint8_t* d, uint32_t dlc) {
  if (dlc != 8) return false;
  for (int i = 1; i < 8; i++) if ((uint8_t)(d[i] - d[i-1]) != 1) return false;
  return true;
}

/* LEC is the single most diagnostic field in S2, and a bare numeral is not a
 * diagnosis at 3 a.m. -- STUFF/FORM means the bit rate is wrong, ACK means the
 * partner is absent, BIT0/BIT1 means the driver or the bus is.
 *
 * Two behaviours make LEC readable, and both are the reason printStatus()
 * snapshots PSR ONCE into a local rather than re-reading the register per
 * field. LEC self-sets to 7 (no-change) on a CPU read of PSR, so each status
 * line reports the last error SINCE THE PREVIOUS LINE rather than a stale one.
 * And it is cleared to 0 by any transfer that completes without error -- so
 * LEC=none is a POSITIVE indicator that traffic is moving, not merely the
 * absence of a complaint. */
static const char* lecName(uint32_t lec) {
  switch (lec) {
    case 0: return "none";       // and a transfer completed cleanly to clear it
    case 1: return "STUFF";      // bit-timing mismatch corrupting frames
    case 2: return "FORM";       // ditto
    case 3: return "ACK";        // nobody acknowledged: partner absent/mistimed
    case 4: return "BIT1";       // sent recessive, read back dominant
    case 5: return "BIT0";       // sent dominant, read back recessive
    case 6: return "CRC";
    default: return "no-change"; // 7: nothing new since the last read
  }
}

static void printStatus() {
  uint32_t psr = FDCAN1->PSR, ecr = FDCAN1->ECR;
  SerialUART.print("[st] mode="); SerialUART.print(cur_mode);
  SerialUART.print("  sent=");    SerialUART.print(tx_count);
  SerialUART.print(" recv=");     SerialUART.print(rx_count);
  SerialUART.print(" rxbad=");    SerialUART.print(rx_bad);
  SerialUART.print("  TEC=");     SerialUART.print(ecr & 0xFF);
  SerialUART.print(" REC=");      SerialUART.print((ecr >> 8) & 0x7F);
  SerialUART.print("  LEC=");     SerialUART.print(lecName(psr & 0x7));
  SerialUART.print(" ACT=");      SerialUART.print((psr >> 3) & 0x3);
  /* EP/EW/BO are self-checking against ECR: TEC >= 128 must coincide with
   * EP=1, and BO=1 must never appear from a missing ACK alone (23.4b). */
  SerialUART.print(" EP=");       SerialUART.print((psr >> 5) & 1);
  SerialUART.print(" EW=");       SerialUART.print((psr >> 6) & 1);
  SerialUART.print(" BO=");       SerialUART.print((psr >> 7) & 1);
  SerialUART.print("  lastid=0x");SerialUART.print(last_rx_id, HEX);
  /* sent - tx_ok = frames discarded by DAR=1, i.e. arbitration losses. */
  SerialUART.print(" txok=");     SerialUART.print(tx_ok);
  SerialUART.print(" drop=");     SerialUART.print(tx_count - tx_ok);
  SerialUART.println();
}

/* ---- 'q': identify the crystal with no external instrument ------------- *
 * The FDCAN timestamp counter ticks once per CAN bit time off the SAME HSE
 * that clocks the peripheral. Count it against millis(), which runs off
 * HSI16 (spec +/-1%), and the ratio gives f_HSE. Independent of the analyzer
 * and of mode 'm'. */
static void measureHSE() {
  if (!running) { p("start a mode first -- 'i' is fine"); return; }
  p("counting CAN bit times for 5 s against millis()...");
  uint16_t prev = (uint16_t)FDCAN1->TSCV;
  uint32_t total = 0, t0 = millis();
  while (millis() - t0 < 5000) {
    delay(5);
    uint16_t n = (uint16_t)FDCAN1->TSCV;
    total += (uint16_t)(n - prev);          // 16-bit wrap is handled by the cast
    prev = n;
  }
  uint32_t el = millis() - t0;
  if (total == 0) { p("!! TSCV never incremented -- not supported. Use method B."); return; }
  uint32_t nbtp = FDCAN1->NBTP;
  uint32_t nbrp = ((nbtp >> 16) & 0x1FF) + 1;
  uint32_t tq   = 1 + (((nbtp >> 8) & 0xFF) + 1) + ((nbtp & 0x7F) + 1);
  float bits_s  = 1000.0f * total / el;
  SerialUART.print("  "); SerialUART.print(total);
  SerialUART.print(" bit times in "); SerialUART.print(el);
  SerialUART.print(" ms -> "); SerialUART.print(bits_s / 1000.0f, 3);
  SerialUART.print(" kbit/s x "); SerialUART.print(tq);
  SerialUART.print(" tq x NBRP "); SerialUART.print(nbrp);
  SerialUART.print("  =>  f_HSE = "); SerialUART.print(bits_s * tq * nbrp / 1e6f, 3);
  SerialUART.println(" MHz");
  p("  reference is HSI16 via millis(), +/-1% -- enough to IDENTIFY a standard");
  p("  crystal (candidates >4% apart), not to calibrate one.");
}

/* ---- 'x': the AF9 mux test, which 'e' does NOT perform ----------------- *
 * TEST.RX (bit 7) monitors the ACTUAL FDCAN_RX pin, not the internal feedback.
 * Both loopback modes disregard the pin (M_CAN spec), so neither proves the
 * AF9 mux on PB9/PA11. This does. Floods the TX FIFO so the bus is busy
 * rather than 2.2% occupied -- muxTest() blocks loop(), so nothing else queues.
 *
 * The 'e'/'m' guard is load-bearing, not cosmetic: the HAL sets CCCR.TEST only
 * for the loopback modes, and with CCCR.TEST = 0 the whole TEST register reads
 * reset values. Running this in mode 'n' would report 0% and declare a mux
 * fault on a perfectly good bus.
 *
 * THE VERDICT IS THREE-WAY. "Any dominant at all = pass" has a false pass in
 * it: if PA11's AF9 routing fails such that the peripheral samples a disabled
 * input buffer, it reads a constant 0 -- DOMINANT -- and a 1% threshold would
 * wave through the exact fault this test exists to catch. A frame stream is
 * 40-55% dominant, so <1% and >90% are both faults and only the band between
 * them proves the pin is being driven BOTH ways by real hardware. */
static void muxTest() {
  if (!running || (cur_mode != 'e' && cur_mode != 'm')) {
    p("run 'e' first -- needs EXTERNAL loopback ('e' or 'm'):");
    p("  'i' holds PB9 recessive, and 'n' leaves CCCR.TEST = 0 so TEST.RX");
    p("  reads its reset value instead of the pin.");
    return;
  }
  p("flooding TX and sampling TEST.RX for 2 s...");
  uint32_t dom = 0, tot = 0, t0 = millis();
  uint8_t d[8];
  FDCAN_TxHeaderTypeDef tx = {};
  tx.Identifier = TX_ID;            tx.IdType = FDCAN_STANDARD_ID;
  tx.TxFrameType = FDCAN_DATA_FRAME; tx.DataLength = TX_DLC;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE; tx.BitRateSwitch = FDCAN_BRS_OFF;
  tx.FDFormat = FDCAN_CLASSIC_CAN;   tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  while (millis() - t0 < 2000) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(&h) > 0) {
      for (int i = 0; i < 8; i++) d[i] = (uint8_t)(tx_count + i);
      if (HAL_FDCAN_AddMessageToTxFifoQ(&h, &tx, d) == HAL_OK) tx_count++;
    }
    if (!((FDCAN1->TEST >> 7) & 1)) dom++;
    tot++;
  }
  SerialUART.print("  dominant on "); SerialUART.print(dom);
  SerialUART.print(" / ");            SerialUART.print(tot);
  SerialUART.print(" samples = ");    SerialUART.print(100.0f * dom / tot, 2);
  SerialUART.println(" %");
  /* ~50%, not the "20-50%" this line used to print -- that was a guess with no
   * arithmetic behind it, and the real run landed at 52.05%, OUTSIDE it. Count
   * the frame: SOF 1 + id 0x200 10 + RTR/IDE/r0 3 + DLC 3 + data ~32 + CRC ~7.5
   * + ~1.5 stuff = ~58 dominant of ~116 bits = 50.0%. Landing within 2 points
   * of a frame's own duty cycle is a far stronger pass than clearing a floor. */
  p("  expected ~50% (frame duty cycle). Both tails are faults: verdict below.");
  /* THREE-WAY, not two. A healthy frame stream is 40-55% dominant, so BOTH
   * tails are faults and only the middle band is a pass. See the header note
   * on why a stuck-dominant pin is the likelier of the two failures. */
  if (dom < tot / 100)
    p("  -> PA11 NEVER DOMINANT. Loopback still passes because the Rx pin is\n"
      "     disregarded -- so this is a MUX or TRANSCEIVER fault, not a\n"
      "     peripheral fault. Do not proceed to S2.");
  else if (dom > (tot / 10) * 9)
    p("  -> PA11 STUCK DOMINANT. *** THIS IS NOT A PASS. *** A real frame\n"
      "     stream is 40-55% dominant; >90% means the receiver is reading a\n"
      "     constant 0, which is exactly what an unrouted AF9 looks like when\n"
      "     the pin's input buffer is disabled. Same reading would come from a\n"
      "     CANH-CANL short or a transceiver holding the bus dominant.\n"
      "     Do not proceed to S2.");
  else
    p("  -> PA11 GOES DOMINANT, AND RECESSIVE IN BETWEEN. AF9 mux confirmed on\n"
      "     BOTH pins, and the TX -> transceiver -> bus -> RX loop is closed\n"
      "     through real hardware.");
}

static void banner() {
  p("");
  p("==== S1c  FDCAN LOOPBACK SELF-TEST  rev1 ====");
  p("  MOTOR DISCONNECTED. NOTHING ON CANH/CANL.");
  p("");
  p("  i = INTERNAL loopback, 1 Mbit  (no pins, no transceiver)");
  p("  e = EXTERNAL loopback, 1 Mbit  (TX pin driven -- but PB9 has NO PAD,");
  p("      and CANH/CANL do not decode on the FX2. Analyzer stays blank.)");
  p("  m = EXTERNAL loopback, 125 kbit -- superseded by 'q'");
  p("  n = NORMAL mode, 1 Mbit -- S2, needs a partner node and real wiring");
  p("");
  p("  x = MUX TEST: samples TEST.RX, the actual pin. Run 'e' THEN 'x'.");
  p("      This is the only check that proves the AF9 mux and transceiver;");
  p("      'e' passing proves neither (M_CAN disregards the Rx pin).");
  p("  q = measure f_HSE from the FDCAN timestamp counter (no analyzer)");
  p("  p = pause/resume TX    s = status    b = banner");
  p("=============================================");
}

void setup() {
  SerialUART.begin(921600);
  delay(500);
  banner();
  p("press 'i' first. Do not skip to 'e'.");
}

void loop() {
  while (SerialUART.available()) {
    char c = SerialUART.read();
    if (c == 'i' || c == 'e' || c == 'm' || c == 'n') startFDCAN(c);
    else if (c == 'p') { tx_enabled = !tx_enabled;
                         SerialUART.print("TX "); p(tx_enabled ? "RESUMED" : "PAUSED"); }
    else if (c == 's') printStatus();
    else if (c == 'q') measureHSE();
    else if (c == 'x') muxTest();
    else if (c == 'b') banner();
  }
  if (!running) return;

  uint32_t now = millis();
  if (tx_enabled && now - last_tx >= TX_PERIOD_MS) {
    last_tx = now;
    FDCAN_TxHeaderTypeDef tx = {};
    tx.Identifier          = TX_ID;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = TX_DLC;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    /* STORE, not NO_TX_EVENTS: the Tx Event FIFO records only transmissions
     * that COMPLETED. With DAR = 1 a frame that loses arbitration is discarded
     * and generates no event, so (sent - tx_ok) IS the drop count -- measured
     * on ONE board against ONE clock. The cross-node method it replaces
     * (ESC1 sent vs ESP32 recv) is swamped: status windows misalign by up to
     * ~1 s = +/-200 frames, and the two timebases differ by 0.141%, against a
     * signal of ~15 frames in 60,000. See 23.9.
     * muxTest() deliberately keeps NO_TX_EVENTS -- it floods 17k frames while
     * blocking loop(), so a 3-deep FIFO would overflow and make tx_ok a lie. */
    tx.TxEventFifoControl  = FDCAN_STORE_TX_EVENTS;
    tx.MessageMarker       = 0;
    uint8_t d[8];
    for (int i = 0; i < 8; i++) d[i] = (uint8_t)(tx_count + i);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&h, &tx, d) == HAL_OK) tx_count++;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(&h, FDCAN_RX_FIFO0) > 0) {
    FDCAN_RxHeaderTypeDef rx; uint8_t d[8];
    if (HAL_FDCAN_GetRxMessage(&h, FDCAN_RX_FIFO0, &rx, d) != HAL_OK) break;
    rx_count++;
    last_rx_id = rx.Identifier;
    uint32_t dlc = (rx.DataLength > 8) ? (rx.DataLength >> 16) : rx.DataLength;
    bool id_ok = (rx.Identifier == TX_ID) || (rx.Identifier == PARTNER_ID);
    if (!id_ok || !payloadOK(d, dlc)) rx_bad++;
    if (rx_count <= 3) {
      SerialUART.print("[rx] id=0x"); SerialUART.print(rx.Identifier, HEX);
      SerialUART.print(" dlc=");      SerialUART.print(dlc);
      SerialUART.print(" data=");
      for (int i = 0; i < 8; i++) { SerialUART.print(d[i], HEX); SerialUART.print(' '); }
      SerialUART.println();
    }
  }

  /* Drain the Tx Event FIFO EVERY pass. It is only 3 deep on the G4, so a
   * slower drain silently loses events and undercounts tx_ok -- which would
   * read as a drop rate that is not there. */
  /* NOTE: there is no HAL_FDCAN_GetTxEventFifoFillLevel() -- the G4 HAL ships
   * only the Rx equivalent -- and HAL_FDCAN_GetTxEvent() takes TWO arguments,
   * with no FIFO selector. Read the fill level from TXEFS directly, which is
   * also how this file reads PSR/ECR/NBTP/TSCV. Depth is SRAMCAN_TEF_NBR = 3. */
  uint32_t nev = (FDCAN1->TXEFS & FDCAN_TXEFS_EFFL_Msk) >> FDCAN_TXEFS_EFFL_Pos;
  while (nev--) {
    FDCAN_TxEventFifoTypeDef ev;
    if (HAL_FDCAN_GetTxEvent(&h, &ev) != HAL_OK) break;
    tx_ok++;
  }

  if (now - last_status >= STATUS_MS) { last_status = now; printStatus(); }
}
