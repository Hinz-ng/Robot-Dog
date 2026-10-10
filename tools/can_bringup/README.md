# CAN bring-up instruments

Mirrors of the diagnostic sketches behind the §23 ladder. Results and method:
[`docs/CAN_BRINGUP.md`](../../docs/CAN_BRINGUP.md). They are also the tools that re-validate each
board during the 12-board termination rework (§23.2).

| Folder | Board | Rev | What it is |
|---|---|---|---|
| `esc1_s1c/` | ESC1 clone / STM32G431 | 2 | FDCAN loopback self-test, mux test, HSE frequency measurement, NORMAL mode for S2. Modes `i` / `e` / `m` / `n`; probes `x` (mux), `q` (HSE), `s` (status), `b` (banner), `p` (TX toggle) |
| `esp32_node/` | ESP32-S3 DevKitC-1 | 3 | Raw ESP-IDF TWAI partner node. `nodeA` / `nodeB` for S1, `solo` for S0's `TWAI_MODE_NO_ACK` |

**These are copies.** The live projects are siblings of this repo (`../CAN Bringup`,
`../ESP32 CAN Bringup`): edit and flash there, then re-copy here. The `platformio.ini` files are
mirrored so the toolchain pin and build flags travel with the source.

`esc1_s1c` needs **`-D HAL_FDCAN_MODULE_ENABLED`** (the STM32 Arduino core omits the FDCAN HAL by
default; the failure is a link error).

The S1b GPIO-only ESC1 probe (`x`, `r`, `t`) is lost (not in either project; VS Code Local History
unchecked). Its results are in §23.3. Recreate it, or use `esc1_s1c`'s `x` probe, before the rework.
