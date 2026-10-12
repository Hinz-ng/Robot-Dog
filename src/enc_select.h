#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include "fleet_config.h"
#include "joint_cal.h"      // ENC_CHIP: the chip fitted to THIS assembly
// ============================================================================
// enc_select.h -- compile exactly ONE encoder driver: the one this joint carries.
// ============================================================================
// Both drivers define the same pin, timing and label names, so including both
// would not compile -- and must not: the wrong driver on a joint reads garbage
// as an angle. EncoderSPI is the type actuator_hw.h instantiates.
// ============================================================================
#if   ENC_CHIP == ENC_MT6835
  #include "mt6835.h"
  typedef MT6835SPI EncoderSPI;
#elif ENC_CHIP == ENC_MT6816
  #include "mt6816.h"
  typedef MT6816SPI EncoderSPI;
#else
  #error "ENC_CHIP must be ENC_MT6816 or ENC_MT6835 (joint_cal.h)"
#endif
