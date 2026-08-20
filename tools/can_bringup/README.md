# CAN bring-up instruments

**Mirrors of the two diagnostic sketches that built the §23 ladder.** They lived in
throwaway PlatformIO projects outside version control, which stopped being acceptable the
moment they became the tools that **re-validate every board during the 12-board termination
rework** (`docs/CAN_BRINGUP.md` §23.2).

Results, method and every retraction: **`docs/CAN_BRINGUP.md` §23–§23.10.**

| Folder | Board | Rev | What it is |
|---|---|---|---|
| `esc1_s1c/` | ESC1 clone / STM32G431 | 2 | FDCAN loopback self-test, mux test, HSE frequency measurement, and NORMAL mode for S2. Modes `i` / `e` / `m` / `n`; probes `x` (mux), `q` (HSE), `s` (status), `b` (banner), `p` (TX toggle) |
| `esp32_node/` | ESP32-S3 DevKitC-1 | 3 | Raw ESP-IDF TWAI partner node. `nodeA` / `nodeB` for S1, `solo` for S0's `TWAI_MODE_NO_ACK` |

## These are mirrors, not the build location

Both are **copies**. The live projects are siblings of this repo — `../CAN Bringup` and
`../ESP32 CAN Bringup` — and that is where they are edited and flashed from. **Edit there,
then re-copy here.** Keeping them buildable in place would mean a second PlatformIO project
inside the joint firmware repo, which is worse than a stale copy: the `platformio.ini`
files are mirrored alongside so the exact toolchain pin and build flags travel with the
source.

`esc1_s1c` needs **`-D HAL_FDCAN_MODULE_ENABLED`** — the STM32 Arduino core leaves the
FDCAN HAL module out by default and the failure is a link error, not a warning.

## A third sketch existed and is gone

**The S1b GPIO-only ESC1 probe** — the one that measured the 125 ns transceiver loop delay
and confirmed `S`-pin polarity — **is not present in either bring-up project.**
`../CAN Bringup` holds exactly one `main.cpp` (the S1c sketch) and has **no git history**;
the ESP32 project does. The one place still worth looking before giving up is **VS Code's
Local History**; that has not been checked.

Its results are all recorded in §23.3, so nothing measured is lost. What is lost is the
ability to re-run it, which matters because §23.2's per-board acceptance test names the
**S1b probe (`x`, `r`, `t`)** as the functional half. That test now has to be re-created,
or served by `esc1_s1c`'s `x` (mux) probe instead — decide before the rework, not during.

**This directory exists so that does not happen a second time.**
