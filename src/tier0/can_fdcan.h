#pragma once
#include <Arduino.h>
#include "can_proto.h"
// ============================================================================
// can_fdcan.h -- Tier-0 FDCAN1 driver. (CAN-T0, 2026-10-03)
// ============================================================================
// WRITTEN FROM docs/CAN_BRINGUP.md 23.6's ORDERED CHECKLIST, NOT RE-DERIVED.
// Every value there is measured; the code is tools/can_bringup/esc1_s1c's
// startFDCAN() reduced to NORMAL mode only.
//
//   1 HSE on, HSERDY polled with a timeout, LOUD failure   (23.5)
//   2 RCC_CCIPR.FDCANSEL = 0 -> HSE is the kernel clock    (23.5: NOT PLLQ, so
//     the bit rate never couples to SYSCLK / PWM / loop rate)
//   3 FDCAN peripheral clock
//   4 PB9 (TX) / PA11 (RX) = AF9 -- BEFORE step 5 (recommended, 23.10 Test C)
//   5 PC11 LOW -> SIT1042 Normal. FLOATING = Standby, measured (23.3): forget
//     this and the node configures, queues and reports no fault while nothing
//     reaches the wire
//   6 NBTP: NBRP 1, TSEG1 6, TSEG2 1, SJW 1 -> 8 tq, 87.5% (23.8; forced by
//     the 8.000 MHz crystal, HSI16 would violate the tolerance budget by 2x)
//   7 CCCR.DAR = 1 -- single-shot (23.4b: a late control frame is stale)
//   8 NORMAL mode. NEVER loopback: a loopback node ignores its Rx pin, cannot
//     arbitrate and babbles over the bus -- Test A drove the partner BUS_OFF.
//     So there is no loopback power-on self-test here at all: a quiescent bus
//     cannot be guaranteed on a robot.
//
// POLLED, NO INTERRUPTS. Tier 0 drains the RX FIFO from its control loop, so
// no ISR touches the command latch and the command timeout is checked on the
// loop's own clock -- a dead bus still fires it.
//
// PB10 (48V_EN) is NOT touched: driving it moves vbus_scale ~1.9x silently
// (HARDWARE.md 3). Only PB9, PA11 and PC11 are configured here, pin by pin.
// ============================================================================

// *** HSE_VALUE TRAP. *** The board variant defines HSE_VALUE = 24000000 -- the
// GENUINE B-G431B-ESC1's crystal. This clone's crystal is 8.000 MHz (23.7, 23.9).
// Nothing here reads HSE_VALUE: the bit timing is set by prescalers against a
// MEASURED 8 MHz, and the core runs SYSCLK from HSI16 (S1c measured HSEON off at
// boot). Any future call to HAL_RCCEx_GetPeriphCLKFreq(FDCAN) or similar would
// report 24 MHz and be 3x wrong.
static const uint32_t CAN_HSE_TIMEOUT_MS = 100;   // HSERDY poll -- S1c's value
// Expected NBTP for NBRP 1 / TSEG1 6 / TSEG2 1 / SJW 1: fields are stored
// minus one, so NTSEG1 = 5 at bit 8 -> 0x500. S1c read exactly this back.
static const uint32_t CAN_NBTP_EXPECTED = 0x00000500UL;

struct CanRxFrame {
  uint16_t id;
  uint8_t  dlc;
  uint8_t  data[8];
};

struct CanHealth {          // ONE snapshot of PSR/ECR -- reading PSR resets LEC
  uint8_t tec, rec, lec;
  bool    ep, ew, bo;
};

static FDCAN_HandleTypeDef can_h;
static bool     can_hse_ok   = false;   // crystal came up -> bit clock is right
static bool     can_up       = false;   // configured and started in NORMAL
static uint32_t can_rx_n     = 0;       // frames drained
static uint32_t can_tx_n     = 0;       // frames queued
static uint32_t can_tx_full  = 0;       // TX FIFO full -> frame not queued
static uint32_t can_busoff_n = 0;       // bus-off recoveries started
static uint32_t can_busoff_last_ms = 0;
static const uint32_t CAN_BUSOFF_RETRY_MS = 100;

static const char* canLecName(uint8_t lec) {
  switch (lec) {     // 23.9's failure-branch table keys on these
    case 0: return "none";  case 1: return "STUFF"; case 2: return "FORM";
    case 3: return "ACK";   case 4: return "BIT1";  case 5: return "BIT0";
    case 6: return "CRC";   default: return "-";
  }
}

// FDCAN_DLC_BYTES_n is (n << 16) on this HAL generation and plain n on newer
// ones (S1c's "KNOWN HAL VERSION TRAP"); a table keeps both correct.
static const uint32_t CAN_DLC_CODE[9] = {
  FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
  FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
  FDCAN_DLC_BYTES_8 };

static uint8_t canDlcBytes(uint32_t code) {
  for (uint8_t i = 0; i <= 8; i++) if (CAN_DLC_CODE[i] == code) return i;
  return 0xFF;                              // FD lengths: not ours, reject
}

// Returns true only if the node is on the bus in NORMAL mode. On false, the
// joint must refuse to arm -- "loud" means that, not a log line (23.6 #4).
static bool canInit(uint8_t node, Print& out) {
  can_up = false;

  // 1. HSE. The Arduino core never enables it, and FDCANSEL's reset default
  //    (00 = HSE) then clocks FDCAN from a switched-off oscillator with every
  //    register write still appearing to succeed.
  if (!(RCC->CR & RCC_CR_HSEON)) RCC->CR |= RCC_CR_HSEON;
  const uint32_t t0 = millis();
  while (!(RCC->CR & RCC_CR_HSERDY)) {
    if (millis() - t0 > CAN_HSE_TIMEOUT_MS) {
      can_hse_ok = false;
      out.println(F("!! CAN: HSERDY TIMEOUT -- crystal not oscillating. CAN DOWN, ARM REFUSED."));
      return false;
    }
  }
  can_hse_ok = true;

  // 2. Kernel clock = HSE.
  RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL_Msk) | (0UL << RCC_CCIPR_FDCANSEL_Pos);

  // 3. Clocks.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // 4. PB9 = FDCAN1_TX, PA11 = FDCAN1_RX, AF9 -- before the transceiver wakes.
  GPIO_InitTypeDef g = {};
  g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF9_FDCAN1;
  g.Pin = GPIO_PIN_9;  HAL_GPIO_Init(GPIOB, &g);
  g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOA, &g);

  // 5. PC11 LOW -> transceiver Normal. Measured polarity (23.3).
  g = GPIO_InitTypeDef{};
  g.Pin = GPIO_PIN_11; g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &g);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

  // 6-8. Timing, single-shot, NORMAL.
  can_h.Instance                  = FDCAN1;
  can_h.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
  can_h.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  can_h.Init.Mode                 = FDCAN_MODE_NORMAL;
  can_h.Init.AutoRetransmission   = DISABLE;          // CCCR.DAR = 1
  can_h.Init.TransmitPause        = DISABLE;
  can_h.Init.ProtocolException    = DISABLE;
  can_h.Init.NominalPrescaler     = 1;
  can_h.Init.NominalSyncJumpWidth = 1;
  can_h.Init.NominalTimeSeg1      = 6;
  can_h.Init.NominalTimeSeg2      = 1;
  can_h.Init.DataPrescaler        = 1;                // classic frames only
  can_h.Init.DataSyncJumpWidth    = 1;
  can_h.Init.DataTimeSeg1         = 6;
  can_h.Init.DataTimeSeg2         = 1;
  can_h.Init.StdFiltersNbr        = 2;
  can_h.Init.ExtFiltersNbr        = 0;
  can_h.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&can_h) != HAL_OK) {
    out.println(F("!! CAN: HAL_FDCAN_Init FAILED. CAN DOWN, ARM REFUSED.")); return false;
  }

  // Accept ONLY what is addressed to this node: {ESTOP, ADMIN+n} and {CMD+n}.
  // Everything else -- other joints' traffic, every STATE reply -- is dropped
  // in hardware, so a 12-node bus does not cost this loop 23 useless drains.
  FDCAN_FilterTypeDef f = {};
  f.IdType = FDCAN_STANDARD_ID; f.FilterType = FDCAN_FILTER_DUAL;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterIndex = 0; f.FilterID1 = CAN_ID_ESTOP;    f.FilterID2 = canIdAdmin(node);
  if (HAL_FDCAN_ConfigFilter(&can_h, &f) != HAL_OK) { out.println(F("!! CAN: filter 0 FAILED")); return false; }
  f.FilterIndex = 1; f.FilterID1 = canIdCmd(node);  f.FilterID2 = canIdCmd(node);
  if (HAL_FDCAN_ConfigFilter(&can_h, &f) != HAL_OK) { out.println(F("!! CAN: filter 1 FAILED")); return false; }
  HAL_FDCAN_ConfigGlobalFilter(&can_h, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  if (HAL_FDCAN_Start(&can_h) != HAL_OK) {
    out.println(F("!! CAN: HAL_FDCAN_Start FAILED. CAN DOWN, ARM REFUSED.")); return false;
  }

  // READ BACK, do not trust: the HAL's arithmetic is checked against ours.
  const uint32_t nbtp = FDCAN1->NBTP, cccr = FDCAN1->CCCR;
  out.print(F("CAN: HSE ready, FDCANSEL=HSE, NBTP=0x")); out.print(nbtp, HEX);
  out.print(F(" CCCR=0x")); out.print(cccr, HEX);
  out.print(F(" DAR=")); out.print((cccr & FDCAN_CCCR_DAR) ? 1 : 0);
  out.print(F(" node=")); out.print(node);
  out.print(F(" rx ids 0x000 0x0")); out.print(canIdAdmin(node), HEX);
  out.print(F(" 0x")); out.println(canIdCmd(node), HEX);
  if (nbtp != CAN_NBTP_EXPECTED) {
    out.println(F("!! CAN: NBTP is not 0x500 -- bit timing is not 8 tq / 87.5%. ARM REFUSED."));
    return false;
  }
  if (!(cccr & FDCAN_CCCR_DAR)) {
    out.println(F("!! CAN: DAR not set -- control frames would retransmit stale. ARM REFUSED."));
    return false;
  }
  if (cccr & (FDCAN_CCCR_TEST | FDCAN_CCCR_MON)) {
    out.println(F("!! CAN: TEST/MON set -- loopback or monitor mode on a live bus. ARM REFUSED."));
    return false;
  }
  can_up = true;
  return true;
}

// Bounded, non-blocking: one frame per call, false when the FIFO is empty.
static bool canPoll(CanRxFrame& fr) {
  if (!can_up) return false;
  if (HAL_FDCAN_GetRxFifoFillLevel(&can_h, FDCAN_RX_FIFO0) == 0) return false;
  FDCAN_RxHeaderTypeDef rx;
  if (HAL_FDCAN_GetRxMessage(&can_h, FDCAN_RX_FIFO0, &rx, fr.data) != HAL_OK) return false;
  fr.id  = (uint16_t)rx.Identifier;
  fr.dlc = canDlcBytes(rx.DataLength);
  can_rx_n++;
  return true;
}

// Single-shot (DAR = 1): a frame that cannot go out promptly is dropped, which
// on a polled bus means the master sees a missed reply and counts it.
static bool canSend(uint16_t id, const uint8_t* data, uint8_t dlc) {
  if (!can_up || dlc > 8) return false;
  if (HAL_FDCAN_GetTxFifoFreeLevel(&can_h) == 0) { can_tx_full++; return false; }
  FDCAN_TxHeaderTypeDef tx = {};
  tx.Identifier = id;                 tx.IdType = FDCAN_STANDARD_ID;
  tx.TxFrameType = FDCAN_DATA_FRAME;  tx.DataLength = CAN_DLC_CODE[dlc];
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE; tx.BitRateSwitch = FDCAN_BRS_OFF;
  tx.FDFormat = FDCAN_CLASSIC_CAN;    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&can_h, &tx, const_cast<uint8_t*>(data)) != HAL_OK) {
    can_tx_full++; return false;
  }
  can_tx_n++;
  return true;
}

static CanHealth canHealth() {
  const uint32_t psr = FDCAN1->PSR, ecr = FDCAN1->ECR;
  CanHealth h;
  h.tec = (uint8_t)(ecr & 0xFF);
  h.rec = (uint8_t)((ecr >> 8) & 0x7F);
  h.lec = (uint8_t)(psr & 0x7);
  h.ep  = (psr >> 5) & 1;  h.ew = (psr >> 6) & 1;  h.bo = (psr >> 7) & 1;
  return h;
}

// Bus-off: M_CAN sets CCCR.INIT and stops. Recovery starts when software
// clears INIT; the controller then waits 128 x 11 recessive bits by itself.
// Rate-limited and counted. Bus-off is reachable only from bit/form errors
// (23.4b, 23.10) -- a missing partner parks at TEC 128 and never gets here --
// so a climbing can_busoff_n is a wiring or babbler finding, not a timeout.
static void canService() {
  if (!can_up) return;
  if ((FDCAN1->PSR & FDCAN_PSR_BO) && (FDCAN1->CCCR & FDCAN_CCCR_INIT)) {
    const uint32_t now = millis();
    if (now - can_busoff_last_ms >= CAN_BUSOFF_RETRY_MS) {
      can_busoff_last_ms = now;
      can_busoff_n++;
      FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    }
  }
}
