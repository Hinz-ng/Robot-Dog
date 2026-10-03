#pragma once
#include <Arduino.h>
#include "tier0_config.h"
// ============================================================================
// t0_fault.h -- HANG PROTECTION: IWDG + HardFault. (CAN-T0, owner addition 1)
// ============================================================================
// THE FAILURE: if Tier 0 hangs (HardFault, infinite loop, a stuck wait), TIM1
// keeps generating the LAST duty cycle. A frozen voltage vector acts like an
// alignment: the rotor locks to it and draws up to VOLT_LIMIT / R_eff ~= 2.0 /
// 0.22 ~= 9 A, indefinitely. The CAN command timeout cannot catch this -- it
// runs in the loop that froze -- and the master's ESTOP cannot be obeyed.
//
// TWO INDEPENDENT LAYERS:
//   1. IWDG, its own LSI clock, kicked ONLY from t0Loop(). A reset returns the
//      gate-driver pins to their reset state (inputs) and the joint reboots
//      DISARMED. Debug-halt freeze set so ST-Link debugging does not reset.
//   2. HardFault_Handler: phases off FIRST (TIM1 MOE cleared, then all six
//      gate inputs forced to GPIO LOW by register writes), then spin until the
//      IWDG fires. No HAL, no Serial, no stack-heavy calls -- the core state is
//      untrusted. LOW on both HIN and LIN = both FETs of every leg off on the
//      EG2124A (confirm at N4e: motor limp after `fault!`, before the reset).
//
// NOT part of the disable path (safety.h): this is what happens when the
// software that owns the disable path is no longer running.
// ============================================================================

// Phase pins, B-G431B-ESC1 variant: UH PA8, VH PA9, WH PA10 (TIM1 CH1-3),
// UL PC13, VL PA12, WL PB15 (CH1N-3N). The handler cannot use the Arduino
// pin map, so the ports and bits are written out -- and checked against the
// variant here, so a variant change breaks the BUILD, not the fault path.
static_assert(A_PHASE_UH == PA8 && A_PHASE_VH == PA9 && A_PHASE_WH == PA10 &&
              A_PHASE_UL == PC13 && A_PHASE_VL == PA12 && A_PHASE_WL == PB15,
              "phase pin map changed -- update t0PhasesOffRaw()");

static inline void t0PinLowRaw(GPIO_TypeDef* port, uint32_t bit) {
  port->BSRR   = (1UL << (bit + 16));                       // output latch LOW first
  port->MODER  = (port->MODER & ~(3UL << (bit * 2))) | (1UL << (bit * 2));  // 01 = output
}

// Callable from any context. Idempotent.
static inline void t0PhasesOffRaw() {
  TIM1->BDTR &= ~TIM_BDTR_MOE;                              // timer stops driving
  t0PinLowRaw(GPIOA, 8);  t0PinLowRaw(GPIOA, 9);  t0PinLowRaw(GPIOA, 10);
  t0PinLowRaw(GPIOC, 13); t0PinLowRaw(GPIOA, 12); t0PinLowRaw(GPIOB, 15);
}

extern "C" void HardFault_Handler(void) {
  t0PhasesOffRaw();
  for (;;) { }                                              // IWDG resets us
}

static bool t0_iwdg_on = false;

static void t0IwdgStart() {
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;        // freeze while halted
  IWDG->KR  = 0xCCCC;                                       // start (enables LSI)
  IWDG->KR  = 0x5555;                                       // unlock PR/RLR
  IWDG->PR  = T0_IWDG_PRESCALER_CODE;
  IWDG->RLR = T0_IWDG_RELOAD;
  const uint32_t t0 = millis();
  while (IWDG->SR && millis() - t0 < 50) { }                // PVU/RVU settle
  IWDG->KR  = 0xAAAA;                                       // first kick
  t0_iwdg_on = true;
}

static inline void t0IwdgKick() { IWDG->KR = 0xAAAA; }

// Why the LAST reset happened -- printed in the banner, then cleared, so N4e
// can see "IWDG" after `hang!` / `fault!`.
static const __FlashStringHelper* t0ResetCause() {
  const uint32_t csr = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
  if (csr & RCC_CSR_IWDGRSTF) return F("IWDG (loop hang or HardFault)");
  if (csr & RCC_CSR_WWDGRSTF) return F("WWDG");
  if (csr & RCC_CSR_SFTRSTF)  return F("software");
  if (csr & RCC_CSR_BORRSTF)  return F("brown-out / power-on");
  if (csr & RCC_CSR_PINRSTF)  return F("reset pin");
  return F("unknown");
}
