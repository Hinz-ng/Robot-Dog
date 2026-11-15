# CAN bring-up instruments

Diagnostic sketches used during CAN transport development. The
[technical reference](../../docs/technical.md) describes their place in the control
architecture. Detailed wiring, operating procedures and original results are
maintained separately from the public repository.

| Folder | Board | Rev | What it is |
|---|---|---|---|
| `esc1_s1c/` | ESC1 clone / STM32G431 | 2 | FDCAN loopback self-test, mux test, HSE frequency measurement, NORMAL mode for S2. Modes `i` / `e` / `m` / `n`; probes `x` (mux), `q` (HSE), `s` (status), `b` (banner), `p` (TX toggle) |
| `esp32_node/` | ESP32-S3 DevKitC-1 | 3 | Raw ESP-IDF TWAI partner node. `nodeA` / `nodeB` for S1, `solo` for S0's `TWAI_MODE_NO_ACK` |

These are retained diagnostic snapshots. Their `platformio.ini` files preserve the
toolchain pins and build flags used with the sketches. They are separate from the
CAN-facing actuator runtime in [`src/tier0/`](../../src/tier0/).

`esc1_s1c` needs **`-D HAL_FDCAN_MODULE_ENABLED`** (the STM32 Arduino core omits the FDCAN HAL by
default; the failure is a link error).

The earlier standalone GPIO-only probe is not included in this repository.
