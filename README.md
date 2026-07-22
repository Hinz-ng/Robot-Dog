# M0/M1 Actuator — Quadruped QDD Bring-Up

Single-actuator bench platform for a ~3 kg dynamic quadruped (ODRI Solo / Mini Cheetah class).
Motor + driver + encoder bring-up notes, hard-won. **Read "Firmware Contract" before flashing anything.**

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Motor | TYI 4006, KV360, 14P/7PP | Phase-to-phase R = 0.5–0.7 Ω (≈0.3 Ω/phase), Kt ≈ 0.0265 Nm/A. 18 A/60 s rating assumes **propeller airflow** — see Thermal. |
| Driver | B-G431B-ESC1 **clone** ("火柴"/Matches FOC V2.0) ×2 | NOT a 1:1 copy — see table. ~89 RMB. |
| Reference driver | Genuine B-G431B-ESC1 | Acceptance-test standard. Onboard genuine ST-Link. |
| Encoder | **MT6816** (AMR) on XJX-135 breakout — *not* MT6701 (early mistake; chip marking is ground truth) | **ABZ interface**, 1024 PPR × 4 = 4096 CPR (~0.088°). Rotor-side, mandatory for FOC. |
| Magnet | N35, D6 × 2.5 mm, diametric (ships with breakout) | **No ferromagnetic material through/behind it** (steel screw = corrupted field, proven). |
| Reduction | 9:1 GT2 belt, 12T alu pinion → 108T printed pulley | |
| Programmer | Clone ST-Link V2.1 | PlatformIO/OpenOCD ✓. **Never attempt ST FW upgrade — brick risk (GoToUSBLoader failure).** |
| Power | 3S LiPo (~11.4 V) | Board defaults 24 V mode; >24 V needs PB10 / VBUS_PARTITIONING_FACTOR change per seller. |

### Clone vs. genuine — measured differences

| | Genuine | This clone |
|---|---|---|
| Gate driver | 3× L6387D (no interlock/dead-time) | **1× EG2124A** (interlock + internal dead-time + pulse filter, HIN/LIN active-high, VCC measured 9.5 V at K36 node) |
| MOSFETs | 6× STL180N6F7 | 6× HG5511D |
| Shunts | 3 mΩ | **20 mΩ (R020), gain-compensated** — see Calibration |
| MCU | STM32G431CB | STM32G431CBU6 (genuine ST silicon) |

"IO-compatible" = external pads only. The firmware↔gate-driver contract differs — root cause of the M0 saga.

---

## Board Pin Truths (corrections earned the hard way)

- **LED_BUILTIN = PC6** (STATUS on schematic). It is **NOT PB8** — bare-PB8 blink fails, LED_BUILTIN works.
- **PB8 = BOOT0** on the G431. If PB8 is high at reset, the MCU boots into the ROM bootloader and firmware never runs. **Never wire the encoder Z/index (or anything that can idle high) to PB8.** We run without index.
- **Only exposed hardware UART is USART2 on PB3/PB4** (TXD/RXD pads). PB6/PB7 have no UART on this board — `HardwareSerial(PB7,PB6)` hangs the MCU. USB-CDC impossible (PA12 is a phase pin).
- **Hardware SPI on PB3/PB4/PB5 does not work via the Arduino API on this variant** — `SPI.begin()` returns but the first `transfer16` never returns (suspected variant pin-map binding). **Bit-banged MODE3 SPI on the same pins works**, proving chip + wiring were fine. Path abandoned in favor of ABZ; do not re-fight it.
- Silkscreen pad table = mode presets (ABI/Z, SPI, I2C), not simultaneous functions.

## Encoder — Working Configuration (ABZ)

- XJX-135 **JP1**: A → **HA/A (PB6)**, B → **HB/B (PB7)**, VDD → 3V, GND → GND. **Z unconnected** (BOOT0 trap). **HVPP → GND** (required for ABZ per seller doc).
- SimpleFOC core `Encoder(PB6, PB7, 1024)` + `enableInterrupts(doA, doB)`, `Quadrature::ON`.
- Interrupt load: ~4096 edges/rev. Bring-up speeds ≈ 1% CPU — irrelevant. Sprint speeds ≈ 15–25% CPU → **migrate to `STM32HWEncoder` (TIM4 hardware quadrature on PB6/PB7, zero CPU) before the dynamic phase.**
- **No-field behavior (expected, by design):** without a saturating field (~300 G, AMR), the angle engine outputs garbage → ABZ emits random edges → counter drifts confidently. The chip's no-magnet flag (SPI bit 0x0002) exists for this reason. Consequence: **a detached magnet at runtime looks like plausible motion, not zeros** — future firmware should plausibility-check velocity.
- Magnet mount spec: diametric, centered ≤0.1–0.2 mm, gap ~1–1.5 mm, tilt <3°, non-ferromagnetic mount/standoff, glue with a shaft-piloted jig.
- Acceptance tests: full turn = ±6.28 rad both directions, no jumps; standstill dead-flat; `enc_vel` matches commanded open-loop velocity (cross-validation vs known motion source).

---

## Firmware Contract

Important operating change (ABZ / no index): the firmware no longer runs an initFOC alignment at boot. On power-up it boots disabled with the direction preset to CCW and `foc_ready = false`. Run `f` once per power-up to relearn a fresh `zero_electric_angle` for that session. That keeps the alignment twitch smaller and faster, and it avoids hardcoding a session-relative offset.

**M0 symptom:** init SUCCESS everywhere, angle increments, motor never moves, battery flat ~40 mA regardless of commanded voltage → all six FETs off.
**Root cause:** SimpleFOC **latest** 6-PWM output violates the EG2124A input contract; **2.3.1 works**. Genuine L6387 tolerates both → no upstream bug reports.

### Known-good `platformio.ini`

```ini
[env:disco_b_g431b_esc1]
platform = ststm32@17.6.0
board = disco_b_g431b_esc1
framework = arduino
monitor_speed = 115200
lib_archive = false
build_flags =
    -DHAL_OPAMP_MODULE_ENABLED
    -DSIMPLEFOC_STM32_DEBUG
lib_deps =
    askuric/Simple FOC @ 2.3.1
    SPI
    Wire
```

Load-bearing rules:
- **`platform = ststm32@17.6.0` pinned** (→ core 2.8.1). Unpinned resolves to latest core and has silently broken working sketches. Verify the build log says 17.6.0 / 2.8.1 every time.
- **`Simple FOC @ 2.3.1` exact — no caret.** `^2.3.1` resolves to latest = dead motor.
- **`lib_archive = false` always** (linker drops the STM32 6-PWM impl without it).
- Any lib/platform change → **delete `.pio`**, rebuild, **read the resolved versions in the Dependency Graph** — verify what resolved, not what you requested.
- SimpleFOCDrivers (if used) must be 2.3.1-era (1.0.5 verified compiling) or expect `stm32_reserveTimer` link errors.
- **Exactly ONE file in `src/` may define `setup()/loop()`.** A stray `main.cpp` gives clean build + verified flash + wrong firmware running. Check the compile list.
- **Wiring and sketch must describe the same configuration.** Mixed config (encoder on serial pins + console sketch) caused phantom serial bytes decoded as `g` commands → motor self-starting. If pins are repurposed, the old function's code must be gone.

### Serial console (restored, known-good)
- USART2: `HardwareSerial SerialUART(PB4, PB3)` — board TXD(PB3) → ST-Link VCP RX, RXD(PB4) → VCP TX, common GND. 115200.
- OpenOCD "target voltage may be too low" = clone ST-Link VREF quirk, harmless.

---

## Current-Sense Calibration (do not "fix" again)

```cpp
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
```
Clone uses 20 mΩ shunts with proportionally reduced amp gain → ADC volts-per-amp equals genuine → **use genuine constants (0.003f)**. Validated: 1 V open loop → |I| ≈ 3 A displayed, battery ≈ 0.35 A (P ≈ 4 W, consistent). The "19 A" burn-incident readings were real (6 V / 0.3 Ω ≈ 20 A ≈ 160 W).

## Safe Test Protocol

- Acceptance sketch: open-loop velocity + current + encoder telemetry + keyboard (`g`/`x`/`+`/`-`), boots **DISABLED**, 30 s auto-stop.
- `motor.voltage_limit = 1.0 V` bench standard (~3 A). **6 V open loop = 160 W stall heater.**
- `driver.dead_zone = 0.05` (EG2124A insurance).
- Battery-side DC ammeter in the pack + line for every characterization run.
- Open loop = fixed V → fixed I → fixed torque; overload = pole slip, NOT a current limit. Closed loop inverts this; the FOC current limit becomes the real thermal protection.

## Diagnostic Ladder

1. Battery-side ammeter vs commanded voltage — does current scale?
2. Drag A/B (powered vs unpowered hand-spin) — low-sides conducting vs all-off.
3. Phase-pad → GND DC average — switching / pinned / floating.
4. Gate-driver VCC at K36 common node (5–20 V).
5. **Read actual IC markings** before trusting "compatible".
6. Firmware A/B against the pinned recipe, one variable at a time, `.pio` deleted.
7. Sensor buses: **raw/bit-bang transaction test before driver classes** — tests the bus, not the library.
8. Make faults visible: step-marker blinks + HardFault strobe handler; a silent LED has meant four different non-hardware causes.

Meta-lessons: instrument-first beats hypothesis iteration; two identical failures = common cause, not two dead units; volume products that "don't work" = contract mismatch; verify resolved versions; chip marking is ground truth over listing titles; observation discipline (power-cycle and watch from t=0) — miscounted LED patterns cost a round each.

## ST Tooling & Clone Hardware

- Clone ST-Link: PlatformIO/OpenOCD ✓; CubeProgrammer ✗; FW upgrade ✗ (**abort — brick risk**); Motor Pilot over clone VCP: not achievable (1.84 Mbaud bidirectional + matched ASPEP firmware; clone ceiling).
- MCSDK path on genuine hardware: 6.3.1 + Motor Pilot 1.2.11, profiler hex `B-G431B-ESC1#...3Sh#.hex` via CubeProgrammer.

## Status & Roadmap

- **M0 — CLOSED.** Toolchain, spin under own firmware, e-stop, thermal sanity, calibration, two working clone drivers.
- **Encoder bring-up — CLOSED pending final acceptance** (full-turn ±6.28 rad check + open-loop cross-validation). ABZ on PB6/PB7 working with temporary magnet mount.
- **M1 next:** permanent magnet mount (jig-centered, spec above) → `initFOC()` alignment (expect twitch; low voltage_limit; e-stop ready) → closed-loop torque(voltage) → closed-loop velocity → current control → impedance behavior.
- Dynamic-phase upgrades (deferred until needed): `STM32HWEncoder` swap; oscilloscope at first dynamic anomaly; heatsink only if measured FET temps demand.
- Genuine board on arrival: acceptance sketch under 2.3.1 AND latest → completes the EG2124A evidence table.
- Parallel mechanical: pivot stack / carriage CAD, bond coupons.

## Thermal Ground Rules

- Binding constraint is **motor copper**: P_cu ≈ 1.5·R_ph·I². 3 A ≈ 4 W (warm); 19 A ≈ 160 W (smoke in seconds).
- Vendor 18 A/60 s assumes propeller airflow at speed. Stall/low speed: no airflow, current parked in fixed windings → derate hard; >5 A sustained stall = instrumented-only.
- Listing "no-load 0.3 A @ 10 V" = friction/windage/iron at ~3600 rpm; unrelated to bus draw at 1 V open loop. Bus current ≠ phase current: the inverter is a buck converter; V_bus·I_bus ≈ P_copper + P_mech.