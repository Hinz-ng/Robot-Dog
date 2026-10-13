# Sensing — encoder, alignment, current sense

§5 encoder (MT6835 SPI) · §6 alignment / ZEA · §7 current-sense calibration and sensor INL.

*Hub: [`README.md`](README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 5. Encoder — MT6835, 4-wire SPI

21-bit, read at 14 bits (16384 counts/rev; the driver truncates `ANGLE >> 7`), bit-banged SPI
mode 3, read-only. Wiring: [`HARDWARE.md`](HARDWARE.md). Chip facts with datasheet sections
(MagnTek Rev 1.3): `src/mt6835.h`. At 14 bits 1 count = 0.0220° at the motor = 0.0426 mrad at the
output = 1.465 µm of belt.

| Quantity | Value | Source |
|---|---|---|
| Read (48-clock burst + CRC-8) | 11.68 µs | J01, `e`, 2026-10-13 |
| Integrity | CRC-8 over angle + status: every single-bit error caught, 0.39% of random garbage passes | host test |
| Status in every frame | over-speed, weak field, under-voltage (`no_mag` = weak OR under-voltage) | datasheet §7.6 |
| Propagation delay | 10 µs typ | datasheet §5 |
| Hysteresis | 0.011° (HYST = 7, factory) = half a 14-bit count | datasheet, J01 boot |
| Transition noise | 0.0015° rms at BW = 5 (factory) | datasheet §5 |
| Commutation delay T (phase 6, belt off) | 84.3 / 88.1 µs at 12.08 kHz; 86.6 / 90.2 µs at 12.62 kHz | J01, 2026-10-13 |
| INL | 0.845 / 0.868° mech pk-pk (two runs) | J01 |
| Field range | 30–1000 mT; below it the chip flags weak field | datasheet §4 |

**Link health** (`mt6835.h`, used by Tier 0, the harness run loop and AUTOCALIB): an invalid read
is a CRC failure or the all-zero word. Four in a row trips an armed joint (~0.35 ms of angle age);
arming needs 1 s with no invalid read. Host model: at 99.65 / 50 / 20 / 5% errors the trip comes
after 0.35 ms / 2.8 ms / 67 ms / 15 s, and arming is refused from ~10⁻³ up.

- **Magnet:** diametric N35 D6 × 3 mm, centred ≤ 0.1–0.2 mm, **gap 0.5–1.0 mm, never zero**, tilt
  < 3°, non-ferromagnetic mount, bonded with a shaft-piloted jig. A zero gap (magnet rubbing the
  package) caused A1's lost motion, its "bearing whine" and its 1/rev disturbance.
- **A dead MISO returns all-zero frames, which pass parity and CRC alike** (J03's and J02's MT6816
  boards, 2026-10-08). `e`, AUTOCALIB phase 1 and the link check all reject them.
- **The span gate is 1 count, not 0:** 21 bits truncated to 14 can flicker at a count boundary. J01
  read span 1 in 3 of 156 still-shaft windows.
- **Velocity is quantised:** at 14 bits and ~11.5 kHz one count per loop is 4.4 rad/s at the motor.
  The MIT estimator's 1 ms filter turns one count step into 0.039 rad/s, which at kd 0.365 is the
  14 mN·m "kd flicker blip" measured at b1 (BELT_DRIVE §22.7).
- Retired: ABZ via TIM4 (4096 counts/rev; no corruption check), the software `Encoder` class (lost
  edges above ~100 k/s), and the MT6816 (14-bit, 1-bit parity; replaced 2026-10-13).

### MT6816 → MT6835 (2026-10-13, J01 belt off)

| | MT6816 | MT6835 |
|---|---|---|
| Integrity check | 1-bit parity: ~50% of garbage passes | CRC-8: 0.39% passes |
| Read time | 6.65 µs | 11.68 µs |
| Harness loop rate, disarmed | 63.9 k/s | 47.3 k/s |
| Commutation delay T | 71–76 µs at 12.9–13.4 kHz (0.958 × loop period) | 84–90 µs at 12.1–12.6 kHz |
| Torque loss from T at 270 / 600 rad/s | 0.95% / 4.8% | 1.45% / 6.8% |
| INL (J01) | 1.03° mech pk-pk | 0.85–0.87° |
| Link failure seen as | a noisy angle (parity passes half) | a stale angle, tripped in 4 reads |

The MT6835's T did not drop when the loop sped up from 12.08 to 12.62 kHz (a per-loop delay predicts
−3.4 µs): the excess over the MT6816 is mostly fixed (10 µs propagation plus the longer read).

---

## 6. Alignment (ZEA) — a stored constant

`zea` and `dir` are stored per joint in `joint_cal.h`, tagged with the encoder chip they were
measured through (`zea_enc`; a mismatch is ignored, CALIBRATION §21), and reproduce across power
cycles (J01 MT6835: 7 alignments, sd 2.04°, SE 0.97° elec; `V` 1.54° and 1.49° on later boots).
`f` forces a fresh alignment; `V` verifies the stored value (§21).

- **Align belt-off.** Alignment settles where alignment torque balances friction: J01 read +2.88°
  elec with sliding-post idlers, +0.15° without. Belt-on, use `V` as a check (≤ 8° elec; J01
  recipe B: 0.62°).
- **`initFOC()` needs an enabled driver.** After `motor.disable()` it fails with `Failed to notice
  movement`.
- **`MOT: Skip dir calib` / `Skip offset calib` means no new measurement was taken.**
- Quality gates: `Id ≈ 0` and `|I|/Iq ≈ 1.2247` (J01: 1.224 / 1.223 / 1.224 at 1.24 / 2.64 / 4.45 A).
- Same-session repeatability belt-off: 0.15–0.77° elec (1–5 counts), torque cost < 0.15%. Do
  not chase it.

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

### Sensor INL

Direction-separated even part of the angle error, phase 5 bins (32 per rev), J01 belt off:

| Harmonic (mech) | MT6816, 2026-08-06, ±109.5 rad/s | MT6835, 2026-10-13, ±108 rad/s |
|---|---|---|
| 1/rev | **0.328°** (2.298° elec) | 0.070° |
| 2/rev | 0.185° | **0.326°** |
| 3/rev | 0.013° | 0.012° |
| Total pk-pk | 0.93° (6.51° elec) | 0.845° / 0.868° (two runs) |

- Fixed in rotor position (MT6816: phases agree between directions within 9°). MT6816 1/rev →
  magnet eccentricity, reducible by centring (M8); MT6835 2/rev → tilt or channel gain, centring
  good. Both inside their datasheets (MT6816 ±1.5°, MT6835 ±1.0° max).
- INL is the dominant angle-error term (~7.6× the ZEA residual on both chips) but repeatable and
  non-accumulating: no compensation needed. The MT6835's user auto-calibration (~±0.07°) is
  deferred ([`README.md`](README.md) §15).
- Belt tension does not grow it: J01 belt off → on (no idlers) 1/rev 0.3690° → 0.3569°, 2/rev
  0.2035° → 0.1977°, even-part r = 0.9986 over 32 bins. Re-measure at real tension as a cheap
  confirmation (B5 / M7).
