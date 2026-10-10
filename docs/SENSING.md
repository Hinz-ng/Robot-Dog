# Sensing — encoder, alignment, current sense

§5 encoder (MT6816 SPI) · §6 alignment / ZEA · §7 current-sense calibration and sensor INL.

*Hub: [`README.md`](../README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 5. Encoder — MT6816, 4-wire SPI

14-bit, 16384 cnt/rev, bit-banged mode 3: PB5 = CSN, PB6 = MOSI, PB7 = MISO, PB8 = SCK; HVPP → 3V3.
Parity is checked per read; `No_Mag_Warning` and `Over_Speed` are available over SPI. Propagation
delay 1 µs typ / 3 µs max (not a delay source).

- **Magnet:** diametric N35 D6 × 3 mm, centred ≤ 0.1–0.2 mm, **gap 0.5–1.0 mm, never zero**, tilt
  < 3°, non-ferromagnetic mount, bonded with a shaft-piloted jig. A zero gap (magnet rubbing the
  package) caused A1's count loss, its "bearing whine" and its 1/rev disturbance.
- **No field:** without a saturating field (~300 G) the angle engine outputs garbage that looks
  like plausible motion, not zeros.
- **A dead MISO returns all-zero frames, which pass parity and `No_Mag`** (J03's original encoder;
  J02's encoder board, 2026-10-08). The harness `e` test now FAILs when every frame is 0x0000, and
  Tier 0 refuses to arm / trips on them. Turn the shaft (`E`) to confirm `raw` moves.
- **Velocity needs a minimum window:** differencing every loop at 15 kHz and 2 rad/s is 0.089
  counts per sample (each estimate 0 or 22.6 rad/s). A 2 ms minimum window gives ±0.15 rad/s.
- Retired paths: ABZ via TIM4 quadrature (4096 CPR; counts noise, cannot detect corruption) and the
  software `Encoder` class (lost counts above ~100 k edges/s).

---

## 6. Alignment (ZEA) — a stored constant

`zea` and `dir` are stored per joint in `joint_cal.h` and reproduce across power cycles (J01: 7
alignments, SE 1.57° elec). `f` forces a fresh alignment; `V` verifies the stored value (§21).

- **Align belt-off.** Alignment settles where alignment torque balances friction: J01 read +2.88°
  elec with sliding-post idlers, +0.15° without. Belt-on, use `V` as a check (≤ 8° elec; J01
  recipe B: 0.62°).
- **`initFOC()` needs an enabled driver.** After `motor.disable()` it fails with `Failed to notice
  movement`.
- **`MOT: Skip dir calib` / `Skip offset calib` means no new measurement was taken.**
- Quality gates: `Id ≈ 0` and `|I|/Iq ≈ 1.2247` (J01: 1.224 / 1.223 / 1.224 at 1.24 / 2.64 / 4.45 A).
- Same-session repeatability belt-off: 1–5 counts, torque cost < 0.15%. Do not chase it.

---

## 7. Current-sense calibration (do not "fix" again)

```cpp
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
```

The clone's shunts are 3 mΩ (`R003`), the same as the genuine board. The absolute scale error is
`i_scale` per board (M2, §8.1c), not a constructor change.

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

**Integrity check, general form:** `|I| = 1.2247 × √(Id² + Iq²)`. The familiar `|I|/Iq = 1.2247`
holds only when Id ≈ 0 (standstill); spinning, Id grows and the naive ratio climbs (1.230 → 1.289
up to 42 rad/s) while the general form holds to 0.2–0.9%.

**Channel imbalance (deferred):**

| Order | Amplitude | Cause |
|---|---|---|
| 7/rev (electrical fundamental) | 0.031–0.042 A | offset between sense channels |
| 14/rev | 0.070 A | ~9% gain mismatch between channels |

In current mode the loop chases these and turns them into real torque ripple (0.93 N pp at the
foot on A1). Below friction; deferred (§8.3).

### Sensor INL (J01, SPI, 2026-08-06)

Parity-separated even part of the angle error, ±109.5 rad/s, 32 bins:

| Harmonic | Electrical | Mechanical | Phase |
|---|---|---|---|
| **1/rev** | 2.298° | **0.328°** | +120.3° |
| 2/rev | 1.297° | 0.185° | +8.5° |
| 3/rev | 0.092° | 0.013° | −110.3° |
| **Total pk-pk** | 6.51° | **0.93°** | — |

- Fixed in rotor position (phases agree between directions within 9°). 1/rev dominates → magnet
  eccentricity / runout, reducible by better centring (M8). Inside the datasheet's ±1.5°.
- INL is the dominant angle-error term (7.7× the ZEA residual) but repeatable and non-accumulating:
  no compensation needed.
- Belt tension does not grow it: J01 belt off → on (no idlers) 1/rev 0.3690° → 0.3569°, 2/rev
  0.2035° → 0.1977°, even-part r = 0.9986 over 32 bins. Re-measure at real tension as a cheap
  confirmation (B5 / M7).
