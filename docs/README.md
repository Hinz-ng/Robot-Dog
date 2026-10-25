# M0/M1 Actuator — Quadruped QDD Bring-Up

Single-actuator bench platform for a ~4 kg, 12-DOF dynamic quadruped. Hardware, firmware
contract, measured plant constants, procedures, and the failure modes that cost the most time.

**Status lives only in the §15 task board below.**

| Before you… | Read |
|---|---|
| flash anything | §4 firmware contract · §11 session workflow |
| believe any number | §12 failure-mode catalogue |
| quote any constant | **§8, the master table** — everything else defers to it |
| bring up a new joint | [`docs/cal/BELT_OFF_BASELINE.md`](cal/BELT_OFF_BASELINE.md), then §22 |
| write STM32 CAN code | §23.6 Tier-0 constraints and init checklist |

---

## Where everything lives

Section numbers are stable across files; find the number, then the file.

| § | Topic | File |
|---|---|---|
| 0 | Dated changelog, newest first | [`docs/CHANGELOG.md`](CHANGELOG.md) |
| 1, 1a, 2, 2a, 3, 14, 16 | Hardware · clone vs genuine · vendor docs · pin map · tooling · thermal | [`docs/HARDWARE.md`](HARDWARE.md) |
| 4, 11, 18 | Firmware contract · session workflow and telemetry · sketch gaps | [`docs/FIRMWARE.md`](FIRMWARE.md) |
| 5, 6, 7 | Encoder · alignment (ZEA) · current sense and INL | [`docs/SENSING.md`](SENSING.md) |
| **8, 9** | **Master table** · drivetrain health | [`docs/CONSTANTS.md`](CONSTANTS.md) |
| 10 | Control: current loop, per-unit policy | [`docs/CONTROL.md`](CONTROL.md) |
| 12, 13 | Failure-mode catalogue · diagnostic ladder | [`docs/FAILURE_MODES.md`](FAILURE_MODES.md) |
| 15, 24 | Status and task board · open documentation conflicts | this file |
| 17, 19 | Robot-level design · battery | [`docs/ROBOT_DESIGN.md`](ROBOT_DESIGN.md) |
| 20, 21 | Manual calibration (M1–M15) · per-joint storage | [`docs/CALIBRATION.md`](CALIBRATION.md) |
| — | New-joint belt-off command sheet | [`docs/cal/BELT_OFF_BASELINE.md`](cal/BELT_OFF_BASELINE.md) |
| 22 | Belt routine B0–B13 · idler spec · pulley recipe of record (22.6) · B12 MIT law (22.7) · deferred items (22.8) | [`docs/BELT_DRIVE.md`](BELT_DRIVE.md) |
| 23 | CAN bring-up results · Tier-0 init checklist (23.6) · termination rework (23.2) | [`docs/CAN_BRINGUP.md`](CAN_BRINGUP.md) |
| — | CAN bring-up sketches (mirrors) | [`tools/can_bringup/`](../tools/can_bringup/) |

Method and the catalogue of ways we have been fooled: `project_context.md`. Working rules,
including how to write these docs: `CLAUDE.md`. Neither restates measured numbers.

**Step prefixes:** M1–M15 manual calibration (§20) · B0–B13 belt build (§22) · S0–S2, A–D CAN
bring-up (§23) · N0–N6 CAN-T0 ladder · D/O deferred items and observations (§22.8).

---

## 15. Status and roadmap

**Binding milestone:** MIT contract over CAN on two joints (CAN-T0), then the leg.
**Now:** J01 fully characterised belt-on (B12 closed 2026-10-03), belt off since 2026-10-10 (bare
motor); CAN-T0 N0–N4 closed on J01 (2026-10-10); J01 and J03 on the MT6835 at 16 bits (J03 accepted
2026-10-24). Final assembly done; J01 belt-on baselines B1–B6b taken 2026-10-25 on the
long-shaft output pulley (extra bearing; no long-shaft band set, BELT_DRIVE §22). **Next: J03
baselines, B11 rows for both, then N5/N6.**

### Closed

- **M0** toolchain, own-firmware spin, e-stop, current-sense calibration, two working clone drivers.
- **Encoder:** MT6835 SPI, 21-bit read at 16 (65536 counts/rev; J01 acceptance 2026-10-19), CRC-8, link-health trip, permanent
  magnet mount (SENSING §5). MT6816 retired 2026-10-13.
- **Electrical model** (Kt, Ke, R_eff, L, τ_e, U0), `dead_zone` 0.005, current loop 412 Hz / 9.3%,
  control delay 0.958 loop periods with the MT6816 / 84–99 µs with the MT6835 (`T_DELAY_S` 91.5 ± 7.5 µs),
  INL 0.93 / 0.85–0.94° mech.
- **M1 `vbus_scale`, M2 `i_scale`** on J01, J02, J03 (§8.1c).
- **J01 and J02 belt-off characterisation**, `J_rotor` 20.2 ± 2.4 × 10⁻⁶ kg·m².
- **Belt drive:** idlers rebuilt, pulley recipe B of record, J01 B1–B6b + B11 (BELT_DRIVE §22.6).
- **B12 MIT law** on J01: contract, signs, units, both clamps, gain box kp ≤ 41 / kd ≤ 0.365
  (§22.7).
- **Units decided** (option A, CONSTANTS §8.1c).
- **CAN transport ladder** S0–S2 and tests A, C, D (§23).
- **Header extraction** (encoder driver, `actuator_hw.h`, `safety.h`), stage 3b bench-accepted
  2026-10-01.

### Task board

| # | Task | Blocked by | Status |
|---|---|---|---|
| **CAN-T0** | **MIT contract over CAN, ESP32 ↔ J01 + J03** (branch `can-t0`) | — | 🟡 **N0–N4 closed on J01, 2026-10-04/06/10.** N1 0 bus errors, 1 late reply in 65k polls (status-print latency); one motor rev CW = −0.6998 rad. N2 all four refusals. N3 v +1.168 / −1.121 rad/s, step rest error 0.6 / 0.3 mrad, armed lps 11.5–12.2 k, 0 misses armed. N4 HOLD/DAMP/resume, cable pull, ESTOP, phone STOP, master reboot, phone dead-man. N4d (bare motor): resume with p_des 0.199 rad off → 59 refusals, CMD_TIMEOUT at 500 ms; hang and HardFault builds → master LOST → IWDG reset → reboots disarmed. Current-sense align changes nothing on J01 (gains = constructor −36.458333) → evidence for `skip_align` at leg boot. Tier 0 refuses to arm / trips on encoder link loss (4 invalid reads; arm after 1 s clean). **Next: N5/N6, after the belt-on assembly.** Tier 0 `-e T0_J01/T0_J03`; master `pio run -d master`, operated from the phone (WiFi `LegMaster`, http://192.168.4.1); contract `can_protocol/` (v0, not frozen). Bus: J01 and J03 keep R22 at the two ends, ESP32 breakout (jumper OFF) on a short mid stub, ≈ 60.5 Ω unpowered. Plan: `C:\Users\ACER\.claude\plans\read-the-attached-context-synchronous-aho.md` until §23.11 is written |
| **ENC** | **Encoder link check steps 2–5 on J01, once (FIRMWARE §11 encoder link health)** | — | ⏸ before the first armed run over CAN. Step 1 passed 2026-10-19 (`encok=1 crc=0`, `lps` 40.8 k) |
| — | Angle extrapolation θ + ω·T for commutation (torque loss 1.45% at 270 rad/s, 6.8% at 600) | — | ⏸ before the first dynamic (sprint / jump) test |
| — | MT6835 user auto-calibration (INL ~±0.07°) | — | ⏸ when an MT6835 joint reads INL > 1.5° mech pk-pk, or leg tracking error shows a 1×/2× motor-rev component that matters. Datasheet §9.2: CAL_EN on a temporary jumper, spin 400–800 rpm in open loop for > 64 rev, then AUTOCALIB |
| 7l | Output position: homing or a known boot pose (the encoder is absolute over one motor turn = 40° of output) | — | ⏸ before the contract freezes and before the leg. Then set per-joint P travel limits (now ±2π) |
| B10 | Tooth-skip threshold (static lever + spring gauge) | — | ⏸ before any command above 1.6 A and before the leg; raises the envelope (≥ 6% below demonstrated-safe current) |
| — | CAN message spec + RL observation/action vector: frame IDs, packing and scaling, poll schedule | — | ⏸ freeze together (§23.6) |
| S1e | Measure the real control frame width | message spec frozen | ⏸ |
| 7d | J02 rebuild as a new joint: ESC board + MT6835 → BELT_OFF_BASELINE → recipe-B pulley → B4 + B6b → J01-vs-J02 15% comparison | hardware | 🔴 J02 retired 2026-10-13: board failed after P8 (D12, D15), motor now J03's, row reset |
| 7e | Belt-length histogram (20 belts, §22.3 protocol) → pulley-compensation bins | — | 🟡 desk work; plus one short-belt joint on recipe B (D4) |
| 7j | Record the recipe-B G-code md5 (`certutil -hashfile <file>.gcode MD5`) | — | 🟡 2 min |
| 7m | Backfill the stage-3b acceptance log with the UT89X reading and auto-stop line | — | 🟡 2 min |
| 2b | Read the ESC1 silkscreen version (V1.0 or V2.0) — §24.7 | — | 🟡 2 min |
| — | 12-board termination rework (§23.2) | first bus with > 2 ESC1s | ⏸ |
| — | 478 mm apex mass question (§17) | — | ⏸ before the controller energy budget |

Deferred belt and control items with promotion conditions: BELT_DRIVE §22.8. Deferred electrical
items: CONSTANTS §8.3.

---

## 24. Open documentation conflicts

Each needs a decision or a short observation; none should be resolved by inference.

| # | Conflict | Resolution |
|---|---|---|
| 24.2 | **Target mass 3.0 or 4.0 kg?** Working instructions say 3 kg; §17 freezes 4.0 kg. Every "% of standing load" divides by 9.81 N (4.0 kg); at 3.0 kg they rise 33% | decide once, in §17 |
| 24.3 | **Pack 5S frozen or 6S live?** `CLAUDE.md` freezes 5S; §17/§19 treat 6S as re-opened | re-freeze 5S (mark §19's 6S rows unexercised) or unfreeze with the evidence named (HG5511D 60 V, a 22.5 V run, 34.2 V divider) |
| 24.6 | `DRIVETRAIN_ETA = 0.92` is circular; it multiplies every force figure | M14, load cell on an assembled leg |
| 24.7 | **ESC1 revision V1.0 or V2.0?** §1 says V2.0; the vendor schematic (sole source for `CAN_SHD` = PC11, PB14, the divider factor, HSE routing) is V1.0 | read the silkscreen; if V2.0, get the V2.0 schematic |
| 24.8 | **PB14 thermistor sign.** Vendor: +0.019 V/°C. Cross-session `analogRead` readings fell as the board warmed; a single DMA session rose (vendor's sign) | one continuous warm-up run, 1.5 A for 2 min, watching PB14 (§16) |
| 24.9 | **Vendor rating below the design point:** < 10 A continuous, < 40 A instantaneous, vs 30 A peak per motor | instrument: M10 hot/cold `R_eff` + FET temperature after a realistic duty cycle (§16) |
