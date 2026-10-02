# Sensing — encoder, alignment, current sense

§5 encoder (MT6816 SPI on the reference actuator, TIM4 quadrature on the retired
ABZ build) · §6 alignment / ZEA · §7 current-sense calibration and sensor INL.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 5. Encoder — TIM4 quadrature (ORIGINAL) / MT6816 SPI (SPARE)

Wiring: XJX-135 **JP1**: A → HA/A (**PB6** = TIM4_CH1), B → HB/B (**PB7** = TIM4_CH2), VDD → 3V, GND → GND. **Z unconnected** (BOOT0 trap). **HVPP → GND** (required for ABZ per seller doc).

Implementation: custom `TIM4Encoder : public Sensor` configuring GPIO AF2 + TIM4 encoder-mode 3 by direct register writes, `ARR = 4095` so the counter wraps once per mechanical revolution. Bypasses the Arduino pin map entirely, so the pruned-variant hang cannot occur.

- **CPR = 4096 verified**: 1 count of error accumulated over 23.65 revolutions. The seller's "AB: 1025 pulses/rev" is a typo; 1024 PPR is correct.
- **Velocity estimation needs a minimum sampling window.** The base `Sensor` class differences position on every call; at 15 kHz and 2 rad/s that is 0.089 counts per sample, so each estimate is either 0 or 22.6 rad/s — quantisation garbage that swung ±4 rad/s after filtering. Overriding `getVelocity()` with a **2 ms minimum window** fixed it: ±0.15 rad/s.
- **Magnet air gap — likely root cause of the ABZ count loss (2026-08-06, ORIGINAL assembly).** The magnet was mounted with essentially **zero air gap to the MT6816 package** and was rubbing or intermittently contacting it. Lifting it ~1 mm made a −2.07 V run that had stalled twice complete a clean 20 s. Mechanism is **mechanical coupling, not field strength** — the AMR element responds to field direction, contact is still inside the 30–1000 mT window, and the die sits 0.5–0.8 mm inside the package. Vibration and micro-deflection at the sensing element make the computed angle jitter; the ABZ interpolator emits spurious edges; the external counter accumulates them. Speed-dependent (deflection grows with speed), progressive within a run (motor heats → axial expansion → worse contact → more current → more heat), and recoverable only by re-running `f`. **Not yet closed — one un-replicated test that also changed the power cycle. Confirm with the ZEA-delta test (§15).**

**RETRACTED:** *"the high-pitched whine is MECHANICAL (audible when backdriving by hand) — suspect motor or pulley bearing."* A magnet rubbing a chip package is exactly a whine audible on backdrive. The bearing attribution had no evidence behind it and this hypothesis explains the observation it was not invented for.

**No-field behaviour:** without a saturating field (~300 G, AMR), the angle engine outputs garbage → ABZ emits random edges → the counter drifts confidently. A detached magnet at runtime looks like plausible motion, not zeros.
- Magnet mount spec: diametric, centred ≤0.1–0.2 mm, gap ~1–1.5 mm, tilt <3°, non-ferromagnetic mount, bonded with a shaft-piloted jig.
- **Angle latency — the ENCODER IS EXCLUDED.** MT6816 datasheet Rev 2.1 gives propagation delay **1 µs typ / 3 µs max**; the TIM4 input filter at `0xF` adds ≈1.5 µs. Together ~3% of the observed budget. The lag is in the control path, not the sensor. ~~Preliminary **T ≈ 143 µs**~~ — **SUPERSEDED. The answer is `T/T_loop` = 0.958 ± 0.015, i.e. one control-loop period, and the sweep was cancelled rather than run (§8.3).** *(The old note here said "read the MT6816 datasheet to settle whether the encoder is the source." That is now spent.)*
- Retired: software `Encoder` class — it lost counts above ~100 k edges/s (193 rad/s), collapsing `lps` 16 k → 5 k and silently corrupting ZEA. Structurally impossible now.

---

## 6. Alignment (ZEA) — session-relative on ABZ, a STORED CONSTANT on SPI

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

### Sensor INL — measured on the SPARE (SPI), 2026-08-06

Parity-separated EVEN part of the angle error, two 1000-sample captures at ±109.5 rad/s, 32 position bins:

| Harmonic | Amplitude (elec) | Amplitude (MECH) | Phase |
|---|---|---|---|
| **1/rev** | **2.298°** | **0.328°** | +120.3° |
| 2/rev | 1.297° | 0.185° | +8.5° |
| 3/rev | 0.092° | 0.013° | −110.3° |
| **Total pk-pk** | **6.51°** | **0.93°** | — |

Phases agree between the two directions to within 9°, confirming the error is **fixed in rotor position** — a genuine sensor/mechanical property, not a control artefact.

**1/rev dominates, not 2/rev.** 1/rev is magnet eccentricity, shaft runout, or off-axis mounting (DISP) — **mechanical, therefore reducible by better centring.** 2/rev would be AMR bridge mismatch, which is not reducible. Total is inside the datasheet's ±1.5° max (quoted for a Ø10 magnet; this rig runs Ø6).

**Consequence:** INL is 7.7× the ZEA calibration residual and slightly exceeds the whole transport-delay error at takeoff, so **it is now the dominant angle-error term.** It is repeatable and position-dependent, so it does not accumulate and needs no compensation at this amplitude — but it sets the floor for any future angle measurement.

> ### ✅ The prediction that belt tension would grow the 1/rev INL term is NOT supported — 2026-08-12
>
> §7 previously said INL **must** be re-measured after the belt is fitted, because belt side-load moves the magnet off-axis (DISP) and the 1/rev term "will grow". Measured across the belt change on J01:
>
> | | Belt off | Belt on, no idlers | Δ |
> |---|---|---|---|
> | INL 1/rev | 0.3690° mech | **0.3569°** | **−3.3%** |
> | INL 2/rev | 0.2035° | **0.1977°** | −2.9% |
> | Phase-1 parity errors | 0/4000 | **0/4000** | air gap intact |
>
> Both terms moved **down**, by less than the measurement's own repeatability. And the **even-part correlation between the two runs is r = 0.9986 across all 32 bins** — the same physical magnet-and-sensor imperfection measured twice through a changed drivetrain.
>
> **Two conclusions.** INL is genuinely a *sensor* property and the measurement is highly repeatable, which is a stronger validation than either run alone. And the re-measurement (M7) is **demoted from mandatory to a cheap confirmation** — but it is not yet retired, because this run was at **1.04 mm of slack with no idlers** (slack corrected 2026-09-02, §22.2), which is not the tensioned state. Re-run it at real tension (B5) and then close it.

*Superseded: a 2.95° mechanical figure measured on the ORIGINAL assembly, whose near-zero air gap was probably distorting it.*

---