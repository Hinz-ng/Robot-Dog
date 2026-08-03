# M0/M1 Actuator — Quadruped QDD Bring-Up

Single-actuator bench platform for a ~4 kg dynamic quadruped (ODRI Solo / Mini Cheetah class).
Hardware, firmware contract, measured plant characteristics, and the failure modes that cost the most time.

**Before flashing anything:** read *Firmware Contract* and *Session Workflow*.
**Before believing anything:** read *Failure-Mode Catalogue*.
**For any number:** §8 is the master table. Everything else defers to it.

---

## 0. Changelog — 2026-08-01 bus-sense / M1-d session

Full file replacement. Every change declared.

| § | Change | Why |
|---|---|---|
| 1 | New row: **bus voltage sense on PA0**, calibrated | Divisor is now a measurement, not a hardcode |
| 1 | MOSFET row: **HG5511D = 60 V / 40 A / 120 A pulse / 11 mΩ** | Datasheet obtained. Clears the FET gate for 6S |
| 1 | Power row: 6S evidence recorded | Board demonstrably ran at 22.5 V |
| 3 | **Deleted** the `PB10 / VBUS_PARTITIONING_FACTOR` note | That constant is ST MCSDK; it does not exist in this firmware. Divider already spans 34.2 V |
| 3 | PA0 / PA1 / PB12 / PB14 identified by measurement | Four-channel probe sweep |
| **8.1** | **R_eff: fifth independent confirmation, 0.2183 ± 0.0034 Ω** | 14-point locked-rotor fit, incidental to M1-d |
| 8.1 | Bus-sense entries; disarmed loop rate 126 kHz | New measurements |
| 8.3 | **Angle lag: preliminary T ≈ 143 µs** (was "0–240 µs") | Extracted from an incidental capture. Promotion condition met |
| 8.3 | New deferred: live Vbus sampling; U₀ feedforward with a bus-voltage trigger | |
| **11** | Telemetry line rewritten; **new session-header protocol** | `Vb`, `Vb_src` added; multimeter reading now recorded per session |
| 11 | **Deleted** "`Ud` is NOT currently printed" | Stale — it is printed in three places |
| **12** | Five new failure modes, **one retraction** | All cost time this session |
| 15 | **M1-d closed.** M1-c and the angle-lag sweep are next | |
| 17 | Peak bus current corrected to **230 A, independent of pack voltage**; sag table added; battery spec | Arithmetic error found in the earlier estimate |
| **19** | **New section: battery specification** | Sag, not energy, is the binding constraint |

**Superseded and deleted:** the claim that `Vb` rose 1.1% under load (it never updated at all — see §12); the `U₀ = 0.0194 V` intercept from this session's sweep (ill-conditioned — §8.1); `dead_zone` as permanently closed (it re-opens above ~15 V bus — §8.3).

---

## 0b. Changelog — 2026-07-29 characterisation session

Substantial rewrite. Every change declared, so nothing regresses silently.

| § | Change | Why |
|---|---|---|
| 1 | `Kt = Ke ≈ 0.0265` **corrected** to Kt = 0.0266, Ke = 0.0177 | The dq power balance forbids Kt = Ke in SimpleFOC's convention. Measured. |
| 1 | Magnet slip moved from "open" to "closed" | Permanent mount, pen mark intact across many sessions |
| 1 | Bearing whine entry updated with the 1/rev measurement | Now has a number and a discriminating test |
| 4 | `monitor_speed` 115200 → **921600**; SVPWM added to the contract | Print-block commutation freeze (§12); SVPWM is +15.5% voltage ceiling |
| 7 | Current-sense section: added the 1.224 verification across a 3.6× range and the 7/rev + 14/rev artefacts | New measurement |
| **8** | **Master table rewritten from scratch** | Almost every entry was an estimate; nearly all are now measured |
| 9 | 1/rev disturbance confirmed by position-fold (r = 0.98); cogging amplitude added and found negligible | Replaces "attributed to the pinion, strictly unproven" |
| 10 | Current-loop section rewritten: new gains, new measured response | 203 Hz / 83% overshoot → 412 Hz / 9.3% |
| 11 | Commands `a`, `j`; log struct v2; `Ud` flagged as missing | Sketch changed |
| 12 | Six new failure modes added | All cost time this session |
| **17** | **New section: robot-level design decisions** | Links, ratio, battery frozen on measured constants |
| 15 | Status and roadmap rewritten | M1-a/b closed; M1-c/d are next |

Superseded and deleted: the `L ≈ 26 µH` inference (circular — it was `Kp·R/Ki` restated), `R_eff 0.23–0.35 Ω use 0.3`, `PWM ripple 3–4 A pp` (followed from the wrong L), the `CURQ_I 900 → 600` damping trim (obsolete), `dead_zone` tuning as an open task.

---

## 1. Hardware

| Component | Part | Notes |
|---|---|---|
| Motor | TYI 4006, KV360, 12N14P (7 pole pairs) | **Kt = 0.0266 Nm/A per Iq; Ke = 0.0177 V/(rad/s) = Kt/1.5.** Measured — see §8. Nameplate 18 A/60 s assumes **propeller airflow** — see §16. |
| Driver | B-G431B-ESC1 **clone** ("火柴"/Matches FOC V2.0) ×2 | Not a 1:1 copy — see §2. ~89 RMB. Seller lists 48 V capability. |
| MOSFETs | **HG5511D** ×6 | **60 V V(BR)DSS, 40 A cont (25 °C) / 25 A (100 °C), 120 A pulsed, R_DS(on) 11 mΩ @ V_GS = 10 V, R_θJA 35 °C/W.** Datasheet 2026-08-01. **Clears the FET gate for 6S** — 25.2 V bus with 1.5× switching overshoot = 38 V = 63% of breakdown. |
| **Bus voltage sense** | **PA0**, resistive divider | **0.008358 V/count** (12-bit). Measured 2-point, not from a datasheet: 11.30 V → 1351, 22.50 V → 2694. Voltage ratio 1.991 vs count ratio 1.994. Full scale **34.23 V**, 8.36 mV/count. **6S at 25.2 V = 74% of range → no divider change needed.** Seed-only — see §12. |
| Reference driver | Genuine B-G431B-ESC1 | Acceptance test still not run. Has genuine onboard ST-Link. |
| Encoder | **MT6816** (AMR) on XJX-135 breakout | *Not* MT6701 — chip marking is ground truth. **ABZ mode**, 1024 PPR ×4 = **4096 CPR** (0.088°). Possible angle latency — §8 deferred list. |
| Encoder interface | **TIM4 hardware quadrature**, register-level (§5) | Zero interrupts, zero CPU, cannot lose counts. |
| Magnet | N35 **D6 × 3 mm** diametric, jig-centred, CA-bonded | **CLOSED.** Permanent mount; pen mark intact across many sessions. **No ferromagnetic material in/behind the field path.** History in §12. |
| Reduction | 9:1 GT2 belt, 12T alu pinion → 108T printed pulley | **6 mm belt is the force ceiling — moving to 10 mm.** See §17. |
| Programmer | Clone ST-Link V2.1 | PlatformIO/OpenOCD ✓. **Never attempt ST FW upgrade — brick risk.** |
| Power | 3S LiPo (~11.3 V) on the bench; **6S re-opened for the robot** | Divider spans 34.2 V, so 5S *and* 6S both fit with no board change. **Board demonstrably powered and ran at 22.5 V** (2026-08-01, unloaded). Remaining untested gate: switching at 25.2 V under load — see §19. |

**Open hardware issue — bearing whine, now quantified.** Audible when backdriving by hand, motor unpowered, and it survived a shaft-bearing swap. A position-resolved capture (§9) shows a **once-per-motor-revolution drag disturbance of 0.185 A amplitude (0.79 N at the foot), repeatable at r = 0.98.** Whether that and the whine share a cause is unproven. Candidates: motor internal bearing, pinion bore runout, rotor/magnet eccentricity.

> **Discriminating test (10 min, no parts): belt off, TORQUE(V) at 0.6 V, `L` capture, fold Iq against unwrapped `cnt`.**
> 1/rev survives → motor or its magnet; swap justified (60 RMB).
> 1/rev vanishes → pinion or belt; a motor swap would be wasted.
>
> Catch this during the 10 mm-belt rebuild while the assembly is apart. **Do it before building any position-indexed feedforward table**, or the table encodes the defect.

## 2. Clone vs. genuine (measured)

| | Genuine | This clone |
|---|---|---|
| Gate driver | 3× L6387D (no interlock, no internal dead-time) | **1× EG2124A** — interlock + internal dead-time + pulse filter, HIN/LIN active-high, VCC measured 9.5 V at the K36 node |
| MOSFETs | 6× STL180N6F7 | 6× HG5511D |
| Shunts | 3 mΩ | **20 mΩ (R020), amp gain scaled to compensate** — §7 |
| MCU | STM32G431CB | STM32G431CBU6 (genuine ST silicon, flashes normally) |

"IO-compatible" means external pads only. The firmware↔gate-driver contract differs — that was the root cause of the entire M0 saga.

---

## 3. Board Pin Truths

- **`LED_BUILTIN` = PC6** (STATUS). **Not PB8** — a bare PB8 blink fails while `LED_BUILTIN` works.
- **PB8 = BOOT0.** High at reset → MCU enters the ROM bootloader and firmware never runs. **Never wire the encoder index (or anything that can idle high) to PB8.** We run without index.
- **The only exposed hardware UART is USART2 on PB3/PB4.** PB6/PB7 have no UART on this board — `HardwareSerial(PB7, PB6)` hangs the MCU. USB-CDC is impossible (PA12 is a phase pin).
- **Arduino-API hardware SPI on PB3/PB4/PB5 does not work on this variant.** `SPI.begin()` returns but the first `transfer16()` never does. Bit-banged MODE3 SPI on the same pins *does* work, proving chip + wiring were fine. Cause: the vendor-pruned `PeripheralPins_B_G431B_ESC1.c` pin map; a lookup miss lands in `Error_Handler()`, which is an infinite loop. **Do not re-fight this.**
- Same pruning killed `STM32HWEncoder` — it hangs inside `init()`. Solved by configuring TIM4 directly (§5).
- Silkscreen pad labels are *mode presets* (ABZ / SPI / I²C), not simultaneous functions.
- **Analogue pins identified by measurement** (four-channel probe, 3S then 6S):

| Pin | 11.30 V | 22.50 V | Identity |
|---|---|---|---|
| **PA0** | 1351 | 2694 | **VBUS divider.** Tracks the bus 1:1 |
| PA1 | 155 | 155 | Current-sense op-amp input. Flat |
| PB14 | 1431 | **1267** | **Board thermistor (probable).** Moved *down* between sessions as the board warmed — wrong direction and wrong proportion for a voltage channel. See §16 |
| PB12 | 2126 | 2127 | Unpopulated speed-pot input, parked at mid-rail. No use |

- **`PB10` = `48V_EN` switches the bus divider RANGE. Do not touch it.** It is a *measurement* setting, not a power setting — it gates nothing and does not limit what the board can run on. The divider as shipped spans 34.2 V, which covers 6S with headroom; switching to the 48 V range would only cost resolution and invalidate the §1 calibration. `VBUS_PARTITIONING_FACTOR` is an **ST MCSDK** symbol and does not exist in this Arduino/SimpleFOC firmware — `VBUS_SCALE` does its job and is measured rather than derived.
- **`driver.pwm_frequency` reads back `-12345`** = SimpleFOC's `NOT_SET` sentinel. The platform default (25 kHz on STM32) is in force; the member is simply never written back. **Do not "fix" it by assigning a value** — you would be changing a variable to one you only believe is already active.

---

## 4. Firmware Contract

**M0 symptom:** every init reports SUCCESS, angle increments, motor never moves, battery flat at ~40 mA regardless of commanded voltage → all six FETs off.
**Root cause:** SimpleFOC *latest* emits 6-PWM that violates the EG2124A input contract. **2.3.1 works.** The genuine board's L6387 tolerates both, which is why no upstream bug exists.

### Known-good `platformio.ini`

```ini
[env:disco_b_g431b_esc1]
platform = ststm32@17.6.0          ; -> core 2.8.1. PIN THIS.
board = disco_b_g431b_esc1
framework = arduino
monitor_speed = 921600             ; raised from 115200 — see §12 print-block freeze
lib_archive = false
build_flags =
    -DHAL_OPAMP_MODULE_ENABLED
    -DSIMPLEFOC_STM32_DEBUG
lib_deps =
    askuric/Simple FOC @ 2.3.1     ; EXACT. No caret.
    SPI
    Wire
```

Load-bearing rules:

- **Pin the platform.** Unpinned resolves to the latest core (19.6.0 / 2.12.0) and has silently broken working sketches. Verify `17.6.0 / 2.8.1` in the build log every time.
- **`Simple FOC @ 2.3.1` exact.** `^2.3.1` resolves to latest = dead motor.
- **`lib_archive = false`** or the linker drops the STM32 6-PWM implementation.
- After any lib/platform change: **delete `.pio`**, rebuild, and **read the resolved versions** in the Dependency Graph. Verify what resolved, not what you requested.
- **Exactly one file in `src/` may define `setup()`/`loop()`.** A stray `main.cpp` produces a clean build, a verified flash, and the wrong firmware running. Check the compile list.
- **Wiring and sketch must describe the same configuration.** Encoder on the serial pins + a console sketch produced phantom serial bytes decoded as `g` → motor self-starting.
- **`motor.foc_modulation = SpaceVectorPWM` must be set explicitly.** 2.3.1 defaults to `SinePWM`, which caps the linear range at `V_bus/2` instead of `V_bus/√3` — **13.4% of your voltage, silently.** Verified on the bench: at bench modulation depth the phase currents are identical (SVPWM changes the ceiling, not the gain), and `|I|/Iq = 1.225` survives because the zero-sequence component SVPWM adds is common-mode.
- SimpleFOCDrivers, if used, must be 2.3.1-era (1.0.5 compiles). Nothing currently needs it.

### Console
USART2: `HardwareSerial SerialUART(PB4, PB3)` — board TXD(PB3) → ST-Link VCP RX, RXD(PB4) → VCP TX, common ground, **921600**.
OpenOCD's "target voltage may be too low" is a clone ST-Link VREF quirk. Harmless.

---

## 5. Encoder — TIM4 hardware quadrature

Wiring: XJX-135 **JP1**: A → HA/A (**PB6** = TIM4_CH1), B → HB/B (**PB7** = TIM4_CH2), VDD → 3V, GND → GND. **Z unconnected** (BOOT0 trap). **HVPP → GND** (required for ABZ per seller doc).

Implementation: custom `TIM4Encoder : public Sensor` configuring GPIO AF2 + TIM4 encoder-mode 3 by direct register writes, `ARR = 4095` so the counter wraps once per mechanical revolution. Bypasses the Arduino pin map entirely, so the pruned-variant hang cannot occur.

- **CPR = 4096 verified**: 1 count of error accumulated over 23.65 revolutions. The seller's "AB: 1025 pulses/rev" is a typo; 1024 PPR is correct.
- **Velocity estimation needs a minimum sampling window.** The base `Sensor` class differences position on every call; at 15 kHz and 2 rad/s that is 0.089 counts per sample, so each estimate is either 0 or 22.6 rad/s — quantisation garbage that swung ±4 rad/s after filtering. Overriding `getVelocity()` with a **2 ms minimum window** fixed it: ±0.15 rad/s.
- **No-field behaviour:** without a saturating field (~300 G, AMR), the angle engine outputs garbage → ABZ emits random edges → the counter drifts confidently. A detached magnet at runtime looks like plausible motion, not zeros.
- Magnet mount spec: diametric, centred ≤0.1–0.2 mm, gap ~1–1.5 mm, tilt <3°, non-ferromagnetic mount, bonded with a shaft-piloted jig.
- **Angle latency — the ENCODER IS EXCLUDED.** MT6816 datasheet Rev 2.1 gives propagation delay **1 µs typ / 3 µs max**; the TIM4 input filter at `0xF` adds ≈1.5 µs. Together ~3% of the observed budget. The lag is in the control path, not the sensor. Preliminary **T ≈ 143 µs** against a measured 80–100 µs loop transport delay — see §8.3 for the settling test. *(The old note here said "read the MT6816 datasheet to settle whether the encoder is the source." That is now spent.)*
- Retired: software `Encoder` class — it lost counts above ~100 k edges/s (193 rad/s), collapsing `lps` 16 k → 5 k and silently corrupting ZEA. Structurally impossible now.

---

## 6. Alignment (ZEA) — session-relative, never hardcode

`zero_electric_angle` is measured relative to wherever the shaft sat at power-up. With an incremental encoder and no index that origin is arbitrary, and a shift of δ mechanical shifts ZEA by **7δ**. Hardcoding it across boots produced a locked rotor at full current.

- **`sensor_direction = Direction::CCW` IS persistent** (a wiring fact). Preset it; `f` then skips direction detection.
- **`zero_electric_angle` must be relearned every power-up** via `f`, which forces `NOT_SET` first.
- **`initFOC()` requires an ENABLED driver.** Calling it after `motor.disable()` gives `Failed to notice movement`.
- **`MOT: Skip dir calib` / `Skip offset calib` means no new measurement was taken.**
- **Belt-on alignment is acceptable in practice** (direction preset + `voltage_sensor_align = 2.0`), validated across many sessions. Quality gates: `Id ≈ 0` and `|I|/Iq ≈ 1.225`.
- Repeatability belt-off, same session: 1–5 encoder counts. Torque cost <0.15%. Don't chase it.
- **Alignment quality is now independently confirmed**: `|I|/Iq` = 1.224, 1.223, 1.224 at 1.24 / 2.64 / 4.45 A — within **0.16%** of √(3/2) across a 3.6× range, and within 1% at every speed up to 42 rad/s using the general form below. Any residual angle error is under ~1° electrical.

---

## 7. Current-Sense Calibration (do not "fix" again)

```cpp
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
```

The clone uses 20 mΩ shunts with proportionally reduced amp gain, so ADC volts-per-amp matches the genuine board → **use the genuine constants.**

**Link order matters:**

```cpp
driver.init();
currentSense.linkDriver(&driver);
motor.linkDriver(&driver);
motor.linkSensor(&encoder);
motor.init();                            // motor FIRST
currentSense.init();                     // then current sense
motor.linkCurrentSense(&currentSense);   // enables foc_current + CS alignment in initFOC
```

### The integrity check, in its general form

The familiar `|I|/Iq = 1.2247` is a **special case valid only when Id ≈ 0.** The correct relation is:

```
|I|  =  1.2247 × √(Id² + Iq²)
```

At standstill Id is negligible and the two agree. Once the shaft spins, Id grows and the naive ratio climbs — that is **not** degradation. Measured at five speeds up to 42 rad/s, the general form holds to **0.2–0.9%** while the naive ratio drifts from 1.230 to 1.289. Use the general form; it is the only cross-check that works in every mode at every speed.

### Known channel imbalance (measured, deferred)

A position-resolved fold of Iq while spinning shows two textbook signatures:

| Order | Amplitude | Cause |
|---|---|---|
| 7/rev (electrical fundamental) | 0.042 A | **Offset** error between sense channels (~0.04 A) |
| 14/rev (2× electrical) | 0.070 A | **Gain** mismatch between channels (~9%) |

In voltage mode these are measurement errors only. **In current mode the loop chases them and converts them into real torque ripple** (0.93 N pp at the foot, ~9.5% of a standing leg load). Below friction, so deferred — see §8.

---

## 8. MASTER TABLE — Measured Plant Characteristics

Everything here is bench-measured unless marked. **This table outranks any other number in this document.**

### 8.1 Electrical — closed

| Quantity | Value | How measured | Confirmations |
|---|---|---|---|
| **Kt** (per Iq) | **0.0266 Nm/A** | = 1.5 × Ke | Matches `60/(2π·KV)` = 0.02653 to 0.2% |
| **Ke** | **0.0177 V/(rad/s)** | 5-point `Uq = R·Iq + U₀ + Ke·ω` fit, free pulley | 0.9% scatter; 2-var fit gives 0.01768 |
| **Ke/Kt relation** | **Ke = Kt / 1.5** | Forced by dq power balance in SimpleFOC's amplitude-invariant convention | `Ke = Kt` excluded at 15σ |
| **R_eff** (whole drive path) | **0.218 Ω** | Locked-rotor `Uq`-vs-`Iq` slope | **Five independent determinations.** 3-point locked rotor 0.218; free-spin 2-var fit 0.219; July 0.226; open-loop SVPWM check; **2026-08-01 14-point locked-rotor fit 0.2183 ± 0.0034 (R² = 0.997, 0.10σ from the table value)** |
| **U₀** (dead-time offset) | **0.028 V** at `dead_zone = 0.005`, `V_bus = 11.4 V` | Locked-rotor intercept, dedicated 3-point sweep | Predicted 0.033 from the July dead-zone table. **The 2026-08-01 14-point sweep returns 0.0194 ± 0.0042 V — only 4.6σ from zero, with 0.028 just 2.06σ away and not excluded. That dataset is ill-conditioned for the intercept (`R` and `U₀` trade off; per-point apparent U₀ scatters 0.007–0.034 with no trend) and does NOT supersede this entry.** Scales with bus voltage — §8.3 |
| Deadband in current | **0.129 A** = U₀/R_eff | derived | **Below the 0.16 A sense noise floor — closed** |
| **L** | **65 µH** (range 59–74) | Locked-rotor Uq step, 3 fit methods (302 / 270 / 340 µs) | Step amplitude matches `ΔUq/R` to 4% |
| **τ_e** = L/R | **300 µs** | as above | |
| Loop transport delay | **~80–100 µs** | Identified from two step responses at different `CUR_TF` | ≈ ½ PWM + ½ loop period |
| PWM frequency | **25 kHz** (library default) | `pwm_frequency` reads `NOT_SET`; STM32 default | Not scope-verified |
| Modulation | **SVPWM** | Boot banner + ammeter A/B | Ceiling `V_bus/√3` = 6.58 V at 3S |
| Loop rate | **13.5 kHz** TORQUE(I) armed, **15 kHz** TORQUE(V), **21 kHz** OPENLOOP-armed, **126 kHz** OPENLOOP disarmed | `dt_us` per sample, cross-checked against `lps` to 9% | Anomaly closed. The disarmed figure completes the set and confirms it was always armed-vs-disarmed loop content |
| Telemetry print cost | **~950 µs** at 921600 (was 5700 µs; ~800 before `Vb`/`Vb_src` were added) | `pr_us` | 6× improvement. Float formatting dominates |
| **Bus-sense scale** | **0.008358 V/count** | 2-point vs multimeter, back-predicts both to 0.07% | Full scale 34.23 V |
| Bus-sense seed accuracy | **±1.3% boot-to-boot** | 11.28–11.43 V across 7 boots vs 11.26–11.30 true | Per-boot scale factor; record the multimeter reading per session (§11) |

### 8.2 Mechanical / drivetrain

| Quantity | Value | How measured |
|---|---|---|
| CPR | 4096 | 1 count error over 23.65 rev |
| Coulomb friction (belt on, leg off) | **1.05 A ≈ 0.028 Nm at the motor** | Drag map from the Ke sweep; flat above ~19 rad/s |
| — referred to the foot | **4.5 N per leg (46% of a standing leg load)** | `F = 2·G·η·Kt·I`, G = N/J̄ = 87.7 |
| Breakaway / stiction band | 0.34 – 1.34 A | July drag map |
| **1/rev disturbance** | **0.185 A → 0.79 N** | Position-fold, **r = 0.98** between consecutive revolutions; `corr(Iq, vel) = −0.78` proves it is real load, not measurement |
| **Cogging (84/rev)** | **0.004 A → 0.02 N** | Position-fold. **Negligible — 50× smaller than 1/rev** |
| 6th electrical harmonic (42/rev) | 0.029 A → 0.12 N | Position-fold; dead time + back-EMF shape |
| Current-mode torque ripple floor | **0.217 A pp → 0.93 N pp** | Iq_pp at 35 rad/s in TORQUE(I) |
| **J_total** | **not measured** | Deferred to the 10 mm-belt actuator |

**Transparency budget at the foot** (against 9.8 N for one leg of a 4 kg robot):

| Source | Foot force | % | Fix |
|---|---|---|---|
| **Coulomb friction** | **4.5 N** | **46%** | **Constant feedforward — highest leverage, ~3 lines** |
| Current-mode ripple (7/rev + 14/rev) | 0.93 N pp | 9.5% | Sense calibration |
| 1/rev mechanical | 0.79 N | 8% | Hardware — localise first |
| Dead-time deadband | 0.55 N | 5.6% | Closed (below noise floor) |
| Cogging | 0.02 N | 0.2% | **Do not build a cogging table** |

**Structural note that closes an argument:** friction referred to the foot scales with `G` exactly as useful torque does, so `F_friction/F_max = I_friction/I_max` is **independent of gear ratio and link length.** Changing the reduction cannot improve transparency on the friction axis. Only reducing the friction or cancelling it in firmware can.

### 8.3 Deferred, with promotion conditions

| Item | Size | Promotes when |
|---|---|---|
| **Angle lag / d-axis decoupling** | **T ≈ 143 µs (PRELIMINARY)** → 3.6% torque loss at 270 rad/s | **PROMOTED — sweep scheduled.** See the box below |
| **J_total** | — | With the 10 mm-belt actuator |
| **Force per amp** | Predicted 4.13 / 4.32 / 4.98 N/A at α = 70/55/40° | When the 80/100 leg exists. Also validates the five-bar Jacobian model |
| Sense gain mismatch (~9%) | 0.93 N pp | If impedance force ripple above ~1 N proves to matter |
| 1/rev source | 0.79 N | Belt-off capture during the rebuild |
| Higher current-loop bandwidth | ~100 Hz available at best | Effectively closed — at 80% of the transport-delay ceiling |
| Genuine-board acceptance run | — | Completes the EG2124A evidence table |
| **Live Vbus sampling** | Bench: none. Robot: 3–7 V of sag | **When the first multi-actuator bus is assembled.** Needs a register-level REGULAR-group conversion on PA0 — §12 |
| **U₀ feedforward** | 0.57 N at 11.3 V → **1.26 N (12.9%) at 25.2 V** | **When the bench bus exceeds ~15 V.** Deadband = U₀/R_eff scales with bus: 0.133 A at 3S (below the 0.16 A noise floor, hence closed) but 0.297 A at 6S (above it). **Fix is U₀ feedforward, NOT a smaller `dead_zone`** — the EG2124A hardware dead time sets the floor. Re-measure U₀ with a locked-rotor sweep at the new bus before implementing |

### Angle lag — preliminary result and the test that settles it

**Preliminary `T ≈ 143 µs`**, extracted from an *incidental* 750-sample capture at 97 rad/s (`Ud = −0.2032`, `Iq = 0.8141`, `Id ≈ 0`):

```
ω_e            = 7 × 97.154 = 680 rad/s
cross-coupling = −ω_e·L·Iq = −680 × 65e-6 × 0.8141 = −0.0360 V
angle part     = −0.2032 + 0.0360 = −0.1672 V
T              = 0.1672 / (Ke·ω·ω_e) = 0.1672 / 1169 = 143 µs
```

Insensitive to `L`: sweeping 59–74 µH moves `T` only 146 → 139 µs.

**Treat as preliminary — four reasons.** (1) One capture spanning ~1 revolution, so the 1/rev disturbance is *in* the answer. (2) `Uq` was pinned at `VOLT_LIMIT` — saturated plant. (3) One speed, so no consistency check. (4) Single telemetry lines scatter 100–352 µs; only the 750-sample mean means anything.

**Why it matters:** measured loop transport delay is 80–100 µs. If T really is 143 µs, **40–60 µs is unaccounted for** — and finding that source is worth more than blindly compensating.

**The settling test** (velocity mode, `VOLT_LIMIT = 3.5`, `L` captures at 20/40/60/80/100/130 rad/s). Two terms sit on `Ud` and were the same size at 42 rad/s, which is why the original estimate could only bracket 0–240 µs:

```
Ud = −ω_e·L·Iq  −  Ke·ω·ω_e·T
      ↑ scales with Iq        ↑ independent of Iq, grows as ω²
```

Hold `Iq` roughly constant (velocity mode does this — friction sets it near 1.05 A, flat above 19 rad/s) and sweep speed. Then per point:

```
T = (−Ud − ω_e·L·Iq) / (Ke·ω·ω_e)
```

**`T` must come out identical at every speed.** Consistency is the falsification test; drift means the fixed-delay model is wrong, which would be a new finding.

| Outcome | Loss at 270 rad/s | Action |
|---|---|---|
| T consistent, <120 µs | <2.6% | **Close.** Don't write compensation |
| T consistent, 130–160 µs | ~3.6% | Locate the extra 40–60 µs before compensating |
| T consistent, >180 µs | >5.7% | Fix, and find the source |
| **T drifts with speed** | — | Model wrong. Stop and reconcile |

**If compensation is eventually written:** `angle += 7·ω·T`. Three traps — use raw `cnt`-derived velocity (`shaft_velocity` is filtered at 20 ms, and a jump accelerates in ~35 ms); verify the sign at 20 rad/s first, because backwards it *doubles* the error; watch `|I| = 1.2247·√(Id²+Iq²)` throughout.

**Encoder is excluded as a source.** MT6816 datasheet Rev 2.1: propagation delay **1 µs typ / 3 µs max**. Plus the TIM4 input filter at `0xF` (≈1.5 µs) that is ~3% of the budget. No encoder upgrade helps — MT6826S is *worse* (10 µs propagation, 100 µs step response). Datasheet also gives INL ±0.75° typ (±5.25° electrical, 0.4% torque loss) and ABFreq 1.024 MHz = 6283 rad/s, so the old software-encoder failure at ~193 rad/s was always the MCU, never the sensor.

**Known blind spot:** `No_Mag_Warning` (0x04[1]) and `Over_Speed` (0x05[3]) are SPI-only. `No_Mag_Warning` would have caught the detached-magnet failure in §5 directly. Not actionable — SPI is closed on this variant.

### 8.4 How to interpret `|I|`

- **At standstill:** trustworthy. `|I|/Iq = 1.224`.
- **While spinning, inside a burst capture:** trustworthy. Averaged, it satisfies the general form to <1%.
- **While spinning, in a single telemetry line:** **not trustworthy.** It is one unsynchronised instant of a rippling current, and it is additionally corrupted by the print-block commutation freeze (§12). Observed 6.9–15.5 A when the true value was 1.2 A.
- **When `Uq` is at the limit:** meaningless. Check `Uq` before interpreting.

---

## 9. Drivetrain Health

Baseline method: velocity mode, 2 rad/s, belt on, position-resolved friction map.

| metric | original pinion+belt | after replacement |
|---|---|---|
| mean drag current | 1.54 A | **0.81 A** |
| peak drag | 3.19 A | **1.34 A** |
| ripple ratio | 12.0× | **4.0×** |
| worst-case `Uq` | 1.05 V | 0.73 V |

**Position-resolved decomposition (2026-07-29), Iq amplitude by mechanical order:**

| Order | Amplitude | Interpretation |
|---|---|---|
| **1/rev** | **0.185 A** | **Dominant. Mechanical, on the motor shaft.** r = 0.98 across revolutions |
| 14/rev | 0.070 A | Current-sense gain mismatch |
| 7/rev | 0.042 A | Current-sense offset |
| 42/rev | 0.029 A | 6th electrical harmonic |
| 84/rev | 0.004 A | Cogging — **negligible** |

Velocity modulation at 1/rev: 1.80 rad/s on a 22.55 rad/s mean — **8% speed ripple, once per revolution.**

- This **confirms** the earlier 1/rev finding with proper sampling. The July attribution to the pinion remains unproven; the belt-off test in §1 settles it.
- **Reflected cogging is inherent** but, now measured, it is not the problem anyone thought it was.
- Electrical braking is *not* a factor when disabled: back-EMF at hand speeds cannot overcome bus + 2 diode drops. **No phase-disconnect relay needed.**

---

## 10. Control — Tuned and Measured

### Current loop (`foc_current`) — CLOSED

| | value |
|---|---|
| `PID_current_q/d.P` | **0.1** |
| `PID_current_q/d.I` | **335** |
| `LPF_current_q/d.Tf` | **0.00025** |
| `PID_current_q/d.limit` | `voltage_limit` |
| `current_limit` | 2.0 A |

**Design rule, now anchored on measurement.** For pole-zero cancellation the gain ratio must equal the electrical time constant:

```
CURQ_P / CURQ_I  =  L / R  =  300 µs        bandwidth ω_c = CURQ_I / R = CURQ_P / L
```

The old `900` gave a ratio of 111 µs — 2.7× too small, i.e. integral-dominated. That single number was the cause of the 83% overshoot, and fixing it was worth more than every filter change combined.

**Measured step response, 0.5 → 1.5 A:**

| config | overshoot | rise 10–90% | bandwidth | notes |
|---|---|---|---|---|
| `I=900, TF=0.005` | 83% | — | 80 Hz | filter-dominated |
| `I=900, TF=0.0005` | 83% | ~2.0 ms | 203 Hz | previous baseline |
| `I=335, TF=0.0005` | 15.6% | 1022 µs | 337 Hz | gain fix alone |
| **`I=335, TF=0.00025`** | **9.3%** | **850 µs** | **412 Hz** | **adopted.** ζ = 0.60, zero steady-state error, Iq sd 0.0094 A (0.6%), locked rotor |
| `I=900, TF=0.0002` | — | — | — | historical runaway with the *old* gains |

**Why to stop here.** Lowering `CUR_TF` further buys damping, not bandwidth — the modelled optimum is at 250 µs and bandwidth *declines* below it. And a transport delay of ~80–100 µs caps a well-damped loop at roughly `1/(3T)` ≈ 500 Hz. **412 Hz is ~80% of the hard ceiling.** More would need a faster control loop, not different gains.

**Why 412 Hz is enough.** Required current bandwidth ≈ 5× the natural frequency of the foot against commanded stiffness, with ~1.32 kg effective mass at the foot:

| Foot stiffness | Natural freq | Needed |
|---|---|---|
| 5 N/mm | 9.8 Hz | 49 Hz |
| 20 N/mm | 19.6 Hz | 98 Hz |
| 50 N/mm | 30.9 Hz | 155 Hz |

Legged robots run 5–30 N/mm. **You have 2–8× margin.**

### Velocity loop — voltage-mode numbers, NOT yet retuned on current

| gain | value |
|---|---|
| `PID_velocity.P` | 0.45 V/(rad/s) |
| `PID_velocity.I` | 2.0 |
| `PID_velocity.D` | **0** — never use D: quantised encoder + filtered velocity = noise amplifier |
| `LPF_velocity.Tf` | 0.02 |

- P_crit ≈ 1.0. Tracks 2–5 rad/s to ±0.02; steps settle <300 ms with 8–12% overshoot.
- Disturbance rejection verified 2–10 rad/s.
- **These gains are in VOLTS. On the current loop the PID output is AMPS.** Rescale by 1/R for the same DC gain — but that does *not* preserve stability margin, so sweep P_crit fresh.
- **Scope this deliberately.** The Tier-0 contract is `τ = kp(q_d−q) + kd(v_d−v) + τ_ff` → current loop. **There is no cascaded velocity PID in the shipping architecture.** Velocity mode is a test harness; tune it to "usable instrument" and stop. Test with **steps, not ramps**.

---

## 11. Session Workflow & Telemetry

Every power-up:

1. Power on. Motor boots **DISABLED**, mode = OPENLOOP, `foc_ready = false`.
2. **Read the `CFG` banner** — `modulation / dead_zone / pwm_Hz / Vbus`. A measurement is only comparable to others taken under the same four values.
2a. **Put the multimeter on the pack terminals and write the reading in the session header, next to the banner `Vbus`.** The seed is a per-boot scale factor on every voltage the firmware reports, and it drifts 1.3% boot to boot (11.28–11.43 V observed vs 11.26–11.30 V true). Recording it costs 30 s and lets any session be rescaled in post-processing. Observed worst case 0.53%, which propagates to 0.5% on `R_eff` — inside the existing scatter, so this is bookkeeping, not a blocker.
3. Press **`f`** → confirm a *real* alignment (twitch visible, no `Skip offset calib`).
4. `v` / `t` / `c` → `g`.
5. **Check `m=` before interpreting anything.**

Commands: `g` go · `x`/`s` stop · `+`/`-` target · `o` open-loop · `t` torque(V) · `c` torque(I) · `v` velocity · `f` align · `l`/`L` burst capture fast/slow · `k` step+capture · **`j` zero-based step** · `d` dump · **`a` stats** · `q` toggle print interval · `?` help.

Guards: boots disabled, 20 s auto-stop, 150 rad/s overspeed, torque modes arm at 0, PID reset on arm, debounced sense-mismatch trip.

### Telemetry line
```
m=<mode> run=<0/1> tgt= cnt= vel= Iq= Id= |I|= Uq= Ud= Vb= Vb_src= lps= pr_us=
```
- **`Ud` IS printed** — in the telemetry line, in `logStats()`, and in the burst CSV. (The old note claiming otherwise was stale and has been deleted.) It is the angle-lag channel — §8.3.
- **`Vb` is the boot-time seed, NOT live.** `Vb_src=seed` says so on every line. It does not track sag. See §12.
- **`Uq` is the saturation check.** Tuning while `Uq` is pinned is tuning a clamp. At 3S, `VOLT_LIMIT = 2.0` saturates above ~90 rad/s: `Uq = R·Iq + U₀ + Ke·ω` = 2.08 V at 1.5 A and 98 rad/s. **Any free-spin run above ~90 rad/s at 2.0 V is saturated and its `Iq` is meaningless.**
- **`ratio=` is only a valid calibration gate when `Uq` is unsaturated AND speed is low.** `|I|` is a square root of squares, so averaging an always-positive rippling quantity biases the mean *upward*. Measured: 1.255 at 97 rad/s (`Iq_pp` = 0.533 A) vs **1.224 at locked rotor** (`Iq_pp` = 0.062 A), same calibration. Gate it at locked rotor.
- **`motor.shaft_velocity` is written only inside `motor.move()`**, skipped while stopped → freezes. Compute fresh when stopped.

### Burst logger
RAM ring buffer, 1000 samples × 16 B = 16.0 kB.
- `l` = decim 1 (~65 ms) — current-loop steps.
- `L` = decim 8 (~520 ms) — judder, resonance, position folds.
- `k` = pre-load to base, settle 300 ms, step while capturing. Works in **TORQUE(V) and TORQUE(I)**.
- `a` = mean of the last capture, one line per sweep point. Echoes `m=`, `run=`, `dz=` so a measurement can never be separated from its conditions.
- **Per-sample `dt_us` is stored**, not assumed. `logDump` reports min/max jitter; use the `t_us` column for any fit.
- Periodic printing is suppressed during capture.
- **Step between two nonzero currents.** Stepping from 0 puts the dead-zone traverse inside the measurement.
- **`vel` is filtered at 20 ms — never fit inertia from it. Use `cnt`** (uint16, wraps at 4095; unwrap before differentiating).

---

## 12. Failure-Mode Catalogue

Each of these cost at least one session.

**Toolchain / build**
- Unpinned platform silently upgrades the core.
- `^2.3.1` → dead motor (EG2124A contract).
- Two files with `setup()`/`loop()` → clean build, wrong firmware.
- Missing `lib_archive = false` → linker drops 6-PWM.
- **Library defaults are silent decisions.** `foc_modulation` defaulted to `SinePWM` for the whole project, costing 13.4% of the voltage ceiling with nothing in any log to say so. Echo every load-bearing default in the boot banner.

**Board / pin map**
- Vendor-pruned pin map → `Error_Handler()` infinite loop, not an error return.
- PB8 = BOOT0.
- **`NOT_SET` reads as `-12345`, not as an error.** `pwm_frequency` printing `-12345` means "never assigned", and the platform default is silently in force.

**Sensor / alignment**
- ZEA hardcoded across boots → rotor locks at full current.
- `Skip … calib` = stored value reported back, not a measurement.
- `initFOC()` with the driver disabled → "Failed to notice movement".
- Software quadrature loses counts above ~100 k edges/s → silent ZEA corruption.
- **Magnet slipping in its mount** — CLOSED, kept as history. Presented as escalating current at constant speed ending in a stall that realignment "fixed". CA was what failed. **Keep the pen mark.**
- Steel screw through a diametric magnet corrupts the field entirely.

**Control / measurement**
- **`motor.current` is NOT updated in voltage mode.** 2.3.1's `loopFOC()` returns from the `voltage` branch without touching it, so `Iq`/`Id` in TORQUE(V) and OPENLOOP are whatever `initFOC` or the last current-mode run left behind. **This would have silently invalidated every Uq-vs-Iq measurement in the campaign** — a frozen number that reads exactly like data. Fixed by refreshing `motor.current` explicitly. `|I|` was always live, which is why it hid for so long.
- **A blocking telemetry print freezes commutation.** During the print block `loopFOC()` does not run, so `setPhaseVoltage()` stops updating while the PWM timer keeps its last duty cycles — the voltage vector is **frozen in space while the rotor keeps turning.** At 96 rad/s and a 5.7 ms print, the rotor sweeps 61% of an electrical revolution and the back-EMF comes into anti-phase with the frozen vector: `(Uq + E)/R = (2.00 + 1.70)/0.218 = 17 A`. Observed `|I|` up to 15.5 A while `Iq` read 1.0 A. **Mitigated 7× by 921600 baud; the proper fix is a chunked non-blocking emit.** Also the most likely cause of the unexplained mid-run load step in the Ke sweep.
- **A capture taken with the motor disarmed looks perfectly healthy.** `Uq = 2.0` and `vel = 2.0` are both stale-but-plausible; only `|I| = 0.053` gave it away. Fixed by recording `m=` and `run=` at `logStart` and printing them with every dump and stats line.
- **A measurement filter inside a loop is part of the loop dynamics** — but it is a *damping* lever, not a bandwidth lever. Below the optimum, lowering it makes the loop slower.
- **Gain ratio beats filter tuning.** Three sessions were spent on `CUR_TF` while `CURQ_P/CURQ_I` was 2.7× off the plant. Measure `L`, compute the ratio, then tune the filter.
- Dead time is a **voltage dead zone**, not just a safety margin — but at `dead_zone = 0.005` it is now **below the sense noise floor** and the question is closed.
- **`Id` cannot detect commutation drift in current mode.** The d-axis loop regulates the *apparent* Id to zero in its own rotated frame and succeeds even when misaligned. **The symptom disappears; the `cos(δ)` torque loss remains.** In *voltage* mode Id is an honest open-loop diagnostic — the original lesson applies to current mode only.
- **`current_limit` does not bind in voltage torque mode.**
- Torque mode has **no speed limit**.
- Stale state latches. **Always ask who updates a value and when.**
- Raw `|I|` is meaningless when the drive saturates.
- A ramp test hides marginal stability; a **step** test exposes it.
- A guard on a noisy signal needs debouncing.

**ADC sharing / silent stale values (2026-08-01)**
- **Arduino `analogRead()` returns 0 on ANY pin of the ADC that `LowsideCurrentSense` owns, from the moment `currentSense.init()` runs.** Not a contention-between-two-calls problem — a *single isolated* call fails. Proven by `vraw=0` printed from the telemetry block while the pre-init seed read in `setup()` works every boot. Cause: the current sense arms the ADC for TIM1-triggered **injected** conversions and leaves it there, so `HAL_ADC_Start` returns BUSY and yields 0. **The PWM timer runs whether or not the motor is enabled**, so this fails even in disabled OPENLOOP. Same family as the pruned pin map: the Arduino layer fails silently instead of erroring.
- **RETRACTED — the "+1.1% `Vbus` rise under load" was never real.** Because the loop sampler never updated once, every `Vb` ever displayed was that boot's `setup()` seed. Two *different boots* (11.30 and 11.42) were compared as if they were one run, and a sample-and-hold residue mechanism was invented to explain the difference. **"Zero drift over 20 s" was the tell and was read as a pass — a frozen value looks exactly like a perfectly stable measurement.** Ask who *writes* a value and when, not what it reads.
- **A dummy `analogRead()` added to flush that imaginary residue broke nothing further, because nothing was working.** Lesson: a fix for a 1.1% artefact was shipped against a 100% failure that was already present and invisible.
- **The remedy is a flag only one branch writes.** `vbus_valid` starts `true` in `setup()` and is only ever cleared by the sampler, so a single telemetry field answers "is the sampler running?" — it resolved in one glance what three turns of hypothesising could not. Design guards this way deliberately.
- **A diagnostic that mutates the state it reports makes its own "before" reading unreliable.** The temporary `z` probe set `vbus_valid = false` as its test action, so a second press showed a misleading `before:` line. Prefer a passive telemetry field over a state-disturbing probe.
- **Correct sharing mechanism, for when this is implemented properly:** STM32 ADCs run **regular** and **injected** groups concurrently on one peripheral. Injected preempts, regular resumes. VBUS belongs on a register-level regular conversion — **no pausing of the current sense, no blind interval in the current loop.** Pausing would be an instrument that disturbs what it measures (§6 of the working-context doc), and at 30 A a blind interval is not cosmetic.

**Interpretation**
- 300 ms telemetry aliases everything above ~1.7 Hz.
- **A fit can measure one parameter superbly and another not at all.** The 2026-08-01 14-point locked-rotor sweep pins `R_eff` to ±1.55% (0.10σ from the table) while its intercept `U₀` lands 4.6σ from zero with a 21.8% standard error; subsetting the points swings `U₀` from 0.019 to 0.042 while `R` moves the other way. **Report the well-conditioned parameter and say plainly that the other is not resolved** — do not overwrite a dedicated measurement with a byproduct.
- **Aliased telemetry can still be right for the wrong reason.** The 1/rev disturbance was correctly guessed from 2.11-samples-per-revolution telemetry — right at Nyquist. The guess only became a finding after a proper position fold. Don't promote a marginal reading to a conclusion.
- Free-shaft runs hit the **voltage ceiling** and look like runaway. At 96 rad/s: `E = 0.0177 × 96 = 1.70 V`, `+R·I +U₀ = 1.95 V ≈ VOLT_LIMIT`. Compute back-EMF first.
- **A frequency-domain FFT smears when the speed varies.** Fold against integrated position instead; ±10% speed variation destroyed the spectral peaks that the position fold resolved cleanly.
- **`|I|/Iq = 1.225` is a special case.** Use `|I| = 1.2247·√(Id²+Iq²)`.

---

## 13. Diagnostic Ladder

1. Battery-side DC ammeter vs commanded voltage — does current scale?
2. Drag A/B: hand-spin powered vs unpowered.
3. Phase-pad → GND DC average.
4. Gate-driver VCC at the K36 common node.
5. **Read the actual IC markings.**
6. Firmware A/B against the pinned recipe, one variable at a time, `.pio` deleted.
7. Sensor buses: **raw/bit-bang transaction test before driver classes.**
8. Make faults visible: step-marker blinks, HardFault strobe.
9. **Position-resolved current map** for any "it feels rough" question. Fold against integrated `cnt`, not time.
10. **Burst capture** for anything dynamic.
11. **Check the mode and armed state of the capture itself** before interpreting it.

Meta-lessons: instrument-first beats hypothesis iteration; verify resolved versions, not requested ones; chip markings over listings; one variable per test; check `m=` first; **echo every load-bearing default at boot.**

---

## 14. ST Tooling & Clone Hardware

- Clone ST-Link: PlatformIO/OpenOCD ✓; CubeProgrammer ✗; FW upgrade ✗ (**abort — brick risk**); Motor Pilot over the clone VCP not achievable.
- MCSDK path on genuine hardware: 6.3.1 + Motor Pilot 1.2.11.

---

## 15. Status & Roadmap

**Closed**
- **M0** — toolchain, spin under own firmware, e-stop, thermal sanity, current-sense calibration, two working clone drivers.
- **Encoder** — TIM4 hardware quadrature, 4096 CPR, permanent magnet mount, min-window velocity estimator.
- **Closed-loop torque (voltage) + velocity (voltage mode)** — characterised, disturbance rejection verified.
- **M1-a — actuator electrical model** — Kt, Ke, R_eff, L, τ_e, U₀ all measured. §8.
- **M1-b — `dead_zone`** — final at 0.005; deadband below the sense noise floor.
- **M1-e — current loop** — 412 Hz, 9.3% overshoot, zero steady-state error, at 80% of the transport-delay ceiling.
- **M1-d — stall clamp (2026-08-01)** — locked rotor, `tgt = 2.00 A` → `Iq` = 1.98–2.03 (mean 2.000), `|I|` mean 2.446 vs 2.449 predicted (0.1%), `Uq` = 0.4604 vs 0.464 predicted (0.8%), `Id` ≈ 0. **`current_limit` binds.** Locked-rotor `ratio` = **1.224**, `Iq_pp` = 0.062 A. `VOLT_LIMIT = 3.5` is authorised for the angle-lag sweep on this evidence.
- **Bus-voltage sense** — PA0 identified and calibrated to 0.07%; divisor is a measurement. Seed-only; live sampling deferred (§8.3).
- **Modulation** — SVPWM verified two independent ways.
- **Instrumentation** — burst logger v2 (per-sample `dt`, `a` stats, capture-condition recording), 921600 telemetry.

**Immediate next**
1. **M1-c — velocity harness** (30 min). `VEL_P` from 0.1, double to `P_crit` with I = 0, take 0.45×, then `VEL_I = VEL_P/0.15`. **Steps, not ramps.** Stop at "usable instrument" — the shipping architecture has no cascaded velocity PID. Sweep at 20–80 rad/s (above ~90 the 2.0 V limit saturates).
2. **Angle-lag sweep** (30 min). `VOLT_LIMIT = 3.5` → `L` + `a` captures at 20/40/60/80/100/130 rad/s → compute `T` per point and check consistency → revert `VOLT_LIMIT`. See §8.3. Prediction on record: **T = 130–160 µs, consistent to ±15%.**
3. **6S staged commissioning** (30 min, separate session — one variable per test, so keep 3S for the tasks above). §19.
4. **M1 proper — impedance controller** + **constant friction feedforward** (~1.05 A, ±0.5 rad/s linear ramp through zero) + **U₀ feedforward if the bus goes above ~15 V**.
5. Belt-off 1/rev capture — opportunistic during the 10 mm rebuild.
6. J_total and J_rotor on the new actuator; force-per-amp when the 80/100 leg exists.

**Deferred with reasons** — see §8.3 for the full table with promotion conditions.

---

## 16. Thermal Ground Rules

- The binding constraint is **motor copper**: `P_cu ≈ 1.5·R_ph·I²` with R_ph ≈ 0.218 Ω. 1.5 A ≈ 0.7 W; 4.5 A ≈ 6.6 W; 8.7 A ≈ 25 W; 19 A ≈ 118 W.
- Vendor 18 A/60 s assumes propeller airflow. At stall there is no airflow → derate hard; >5 A sustained stall is instrumented-only.
- **Open-loop mode applies `voltage_limit` directly with no current limit.** At `VOLT_LIMIT = 2.0` that is 8.6 A / 25 W with no throttle. Keep open-loop runs short or drop the limit to 1.0 for them.
- Switching ripple contributes real RMS heating even at zero average current.
- Bus current ≠ phase current: the inverter is a buck converter.
- **Hot-vs-cold R_eff has not been measured.** Copper is +0.39%/K. If `R_hot/R_cold > 1.15`, peak force sags during a jump sequence. 10-minute test whenever convenient.
- **PB14 is a board thermistor channel (probable)** and is already wired — 1431 → 1267 counts as the board warmed between two sessions. It makes the hot-vs-cold test nearly free. **Caveat: it measures the BOARD, not the winding.** The binding constraint is motor copper, so treat PB14 as an indicator, not a substitute for measuring `R_eff` warm. Confirm the identification first: run at 1.5 A for two minutes and watch it fall further. **Reading it requires the register-level ADC work in §8.3** — `analogRead()` cannot reach it once the current sense is initialised (§12).

---

## 17. Robot-Level Design Decisions (frozen 2026-07-29, battery re-opened 2026-08-01)

Frozen on measured constants, not estimates. Re-open only with new bench data.

**Re-opened, with the data that did it:** the battery row. HG5511D datasheet (60 V), a bench run at 22.5 V, and the measured 34.2 V divider span together removed both objections to 6S. Everything else in this table stands.

| Parameter | Value | Reasoning |
|---|---|---|
| Target mass | **4.0 kg**, 12 DOF | Mass increase is nearly free for jumping — the limit is voltage, not torque |
| **Proximal link** | **80 mm** | L₁ sets stroke |
| **Distal link** | **100 mm** | **Longer distal links *reduce* stroke** (h → L₁cos α + L₂ asymptotically). Keep L₂/L₁ ≈ 1.25 |
| **Reduction** | **9:1 — unchanged** | Jump optimum is ~10:1; 9:1 within 2%. Optimal under both modulation assumptions |
| **Battery** | **5S or 6S — RE-OPENED 2026-08-01** | The old "6S buys 3% and needs a PB10 change" line was wrong on both counts. **No PB10 change is needed** (divider spans 34.2 V, §3) and the gain is not 3% — it is large, because the `R·I` toll is subtracted *before* anything buys speed. See §19 |
| **Belt** | **10 mm GT2** | Tooth load `Kt·I/r_pinion` = 208 N at 30 A on 6 mm — over the limit. Independent of gear ratio |
| Workspace | α ∈ [40°, 70°] | 40° of snap-through margin; reflected inertia worsens toward full extension |

**Resulting envelope:** 53.7 mm usable stroke, 93–147 mm working hip height, 513 N peak thrust (13.1× BW), **~478 mm jump apex**, backflip with ~2.8× angular margin (now a control problem, not a hardware one).

**Reflected inertia:** 0.161 kg per motor, **0.323 kg per leg, 1.29 kg across the robot** — 24% of system mass. Scales as `J_rotor·(N/J̄)²`, and it is *worse* toward full extension where J̄ collapses.

**Accepted weakness:** swing clearance ≈ 34 mm, landing absorption over 54 mm. **This is a flat-ground sprinter and jumper, not a terrain robot.** An explicit choice.

**Not measured yet, and the design rests on it:** `J_rotor` is estimated at 2.1×10⁻⁵ kg·m². Every reflected-inertia figure scales on it — and reflected inertia is 24% of effective mass, hence a major driver of the 30 A peak. **Measure it on the 10 mm-belt actuator before buying a battery.**

### Peak bus current — corrected 2026-08-01

The earlier 150 A estimate was arithmetic error. At full modulation the bus voltage **cancels out**:

```
I_bus/motor = 1.5·Uq·Iq/(V_bus·η)   with Uq = V_bus/√3
            = 1.5·Iq/(√3·0.95) = 0.911·Iq
```

At `Iq` = 30 A: **27.3 A per motor, ×8 sagittal actuators = 219 A, +11 A servos/electronics = 230 A**, independent of pack voltage. Peak electrical power ≈ 3.9 kW, of which 2.35 kW is copper loss — **actuator efficiency at peak is ~40%.**

**The 478 mm apex figure above contains NO sag term.** A no-sag reconstruction gives 653 mm at 21.0 V and 358 mm at 18.5 V; 478 sits between them and assumes a terminal voltage that no realistic pack holds at 230 A. Treat 478 mm as an upper bound and use §19 for what a real pack delivers.

**Mass exchange rate, qualifying the "mass is nearly free" note above:** once force-limited at 30 A by belt tooth load, `h ≈ F_max·s/(m·g) − s`, so apex goes as **1/m**. 300 g of extra battery costs ~40 mm of apex; going from 45 mΩ to 25 mΩ of pack resistance buys ~190 mm. **The heavier, stiffer pack wins ~5:1. Spend the mass.**

---

## 18. Sketch — Known Gaps

Reviewed 2026-07-29. No functional bugs found. Outstanding items:

- `logStats()` skips the first 25% of the buffer — correct for steady-state sweeps, misleading after a `k` step.
- `VBUS_LIVE = false`. `Vb` is a boot seed and does not track. §12 has the reason; §8.3 has the promotion condition.
- Bus voltage is not in the burst log. Add `uint16_t vbus_x100` (drop `LOG_N` 1000 → 900 to hold 18.0 kB, and **read the free-RAM figure in the build output first** — a static buffer that collides with the stack gives a HardFault, not a compile error). Only worth doing once sampling is live.

---

## 19. Battery Specification (opened 2026-08-01)

**The binding constraint is sag, not energy.** One jump costs ~85 J = 0.024 Wh against a ~33–40 Wh pack — three orders of magnitude of margin. Heating is also a non-issue: 394 W dissipated inside the pack for 35 ms raises it 0.05 °C. **The pack is sized entirely by the voltage it retains at 230 A.**

### Why voltage leverages jump height so hard

```
Voltage needed = R_eff·Iq (a fixed toll, buys zero speed) + Ke·ω (buys speed)
               = 0.218 × 30 = 6.57 V                      + 0.0177·ω
Available      = V_bus/√3   (SVPWM ceiling)
```

The toll comes off the top, so sag eats the *remainder*, not the total — and apex goes as speed squared. **A 29% voltage loss becomes a 62% speed loss and an 86% apex loss.**

| Pack | Pulse R inc. wiring | Bus at 230 A | Ceiling | Foot speed at 30 A | **Apex** |
|---|---|---|---|---|---|
| 5S 2200 mAh 35C | ~53 mΩ | 8.8 V | 5.09 V | **cannot reach 30 A** | — |
| 5S 1800 mAh 110C ×1 | ~28.5 mΩ | 14.5 V | 8.34 V | 1.14 m/s | 66 mm |
| **5S 1800 110C ×2 parallel** | ~16.2 mΩ | 17.3 V | 9.97 V | 2.19 m/s | **244 mm** |
| **6S 1800 110C ×1** | ~33.4 mΩ | 17.5 V | 10.11 V | 2.28 m/s | **266 mm** |
| **6S 1800 110C ×2 parallel** | ~18.7 mΩ | 20.9 V | 12.07 V | 3.54 m/s | **639 mm** |

Note row 1: that pack's ceiling falls **below the 6.57 V toll**, so it cannot make 30 A flow at any speed. Fine for standing, walking, trotting; cannot jump at design force. **Every resistance figure above is an estimate — the RC3563 replaces them with measurements.**

### Target spec

| Parameter | Target | Rationale |
|---|---|---|
| Cells | **6S if commissioning passes, else 5S** | §3 shows no board change is needed either way |
| **Internal resistance** | **≤20 mΩ total pack** | *The* spec. Sets apex directly. Ask the seller (*内阻多少毫欧?*); if they cannot answer, it is not a high-C pack |
| Capacity | 1800–3000 mAh | <1800 → runtime impractical; >3000 → mass penalty beats the sag benefit |
| Advertised C | ≥100C @1800 mAh | Assume ~50% derate on advertised claims |
| Connector | **XT90 or direct 10 AWG solder** | XT60 is ~60 A cont / ~120 A burst — inadequate at 230 A |
| Trunk wire | 10 AWG minimum | Per-ESC branches at 30 A are fine on 16 AWG |
| Mass | ≤700 g (17.5% of 4 kg) | Exchange rate favours mass — §17 |

**Parallel pairs are the best value.** Doubling electrode area halves cell resistance, doubles capacity, and costs two cheap packs instead of one exotic one. Match voltage within ~0.1 V/cell before connecting (use a parallel board) and keep the packs the same model and age.

**Do not try to fix sag with capacitors.** 200 A for 35 ms at 1 V droop needs `C = I·Δt/ΔV` = **7 F**. Bulk caps handle switching ripple only.

### Measurement protocol (RC3563, four-wire, 1 kHz)

1. Charge to ~3.8 V/cell, rest 30 min, room temperature. Resistance depends on charge state and temperature — roughly doubles at 0 °C.
2. **Measure each cell individually through the balance lead, then sum.** A pack with one bad cell reads only slightly high overall, but that cell is what limits you and what fails first.
3. Add ~4 mΩ for wiring and connector.
4. **Multiply by ~1.4** to get the resistance that matters for a 35 ms pulse. The 1 kHz AC method sees only the fast ohmic part; charge-transfer and diffusion add more on the jump timescale. Factor is approximate and chemistry-dependent.
5. Re-measure every few months. Rising internal resistance is how a LiPo announces its death, long before capacity drops.

| Total pulse resistance | Verdict |
|---|---|
| ≤20 mΩ | Excellent — supports the full design jump |
| 20–30 mΩ | Good — roughly half design apex |
| 30–45 mΩ | Marginal — fine for trotting, weak on jumps |
| >45 mΩ | Cannot support the design operating point |

### 6S staged commissioning

Gates cleared: **FETs 60 V** (datasheet); **regulator, 3.3 V rail, bulk capacitance** (board demonstrably ran at 22.5 V); **ADC range** (34.2 V full scale). Not cleared: **electrolytic capacitor markings unreadable**, and **switching at 25.2 V under load untested**.

Abort at the first sign of anything warm that should not be:

1. Power only, no motor, 22.5 V, 10 min. Feel the electrolytics — they should be at ambient.
2. Power only, full 25.2 V, 10 min. **This is the capacitor test.**
3. Motor connected, open-loop, `VOLT_LIMIT = 1.0`, 30 s. First switching at 6S.
4. TORQUE(I) at 1.0 A, 60 s. **`|I|/Iq` must stay at 1.22–1.23.**
5. Only then run anything longer. **Re-measure U₀ at the new bus (§8.3).**

Expect switching overshoot around 1.3–1.5× the bus: 38 V worst case against 60 V breakdown = 63%. Acceptable margin, not a measurement.