#pragma once
#include <Arduino.h>
#include "driver/gpio.h"
// ============================================================================
// master_config.h -- the leg master's bench policy. (CAN-T0, 2026-10-03)
// ============================================================================

// ---- pins (ESP32-S3-DevKitC-1). Avoid 0/3/45/46 strapping, 19/20 USB,
// 26..32 flash, 33..37 octal PSRAM (CAN bring-up sketch's list). ----
static const gpio_num_t M_PIN_CAN_TX = GPIO_NUM_5;   // -> transceiver TXD
static const gpio_num_t M_PIN_CAN_RX = GPIO_NUM_4;   // <- transceiver RXD
// NORMALLY-CLOSED button to GND, internal pull-up: pressed (open) or a broken
// wire both read HIGH = ESTOP. A normally-open button would fail silent.
static const uint8_t    M_PIN_ESTOP  = 6;

// ---- the joints on this bus. Node number = JOINT_ID. ----
static const uint8_t M_NODES[]  = { 1, 3 };
static const uint8_t M_N_NODES  = sizeof(M_NODES) / sizeof(M_NODES[0]);

// ---- schedule ----
// 200 Hz: the traffic rate S2 and Tests A-D proved (CAN_BRINGUP 23.9-23.10).
// Two joints poll-and-wait at ~0.7 ms per cycle = ~14% of the bus.
static const uint32_t M_CYCLE_MS        = 5;
static const uint32_t M_REPLY_TIMEOUT_US = 500;   // a joint replies within ~1 FOC loop (~85 us) + frame
static const uint32_t M_IDENT_PERIOD_MS = 1000;   // STATUS refresh per node
static const uint32_t M_IDENT_FRESH_MS  = 2000;   // ARM needs a STATUS younger than this
// An ARMED node that misses this many consecutive replies is LOST: the
// master broadcasts ESTOP to every joint and drops to POLL-only. A leg with
// one joint gone must not keep driving the other into it.
static const uint8_t  M_LOST_N          = 3;

// ---- geometry the master owns (Tier 0 stays motor-positive) ----
// MOUNT_SIGN: the open contract item "per-joint mounting sign for mirrored
// legs" lives HERE, not in Tier 0. Leg-frame angle = MOUNT_SIGN * contract p.
// +1 for every node until the leg is mounted and the sign is observed.
static const int8_t M_MOUNT_SIGN[] = { +1, +1 };
// Soft limits on p_des in the CONTRACT frame (rad, output, session zero).
// Wide open until 7l and the leg geometry are known; the master refuses to
// send a p_des outside them.
static const float M_PDES_MIN[] = { -6.28f, -6.28f };
static const float M_PDES_MAX[] = { +6.28f, +6.28f };
static_assert(sizeof(M_MOUNT_SIGN) == M_N_NODES && sizeof(M_PDES_MIN) / sizeof(float) == M_N_NODES
              && sizeof(M_PDES_MAX) / sizeof(float) == M_N_NODES, "one entry per node");

// ---- log ----
static const uint32_t M_LOG_RECORDS = 4000;   // x 40 B = 160 kB internal RAM (halved until malloc succeeds)
                                              // = 10 s of 2 nodes at 200 Hz
