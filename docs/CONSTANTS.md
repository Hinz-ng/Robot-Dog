# Master table — measured constants

**This file outranks every other number in the doc set.** §8 is the master table; §9 is
drivetrain health. Per-joint values live in `src/joint_cal.h`; fleet values in
`src/fleet_config.h`. Belt-on plant states are per-plant and live in
[`BELT_DRIVE.md`](BELT_DRIVE.md).

*Hub: [`README.md`](../README.md).*

---

## 8. Assemblies

| Name | Motor / board | Status |
|---|---|---|
| A1 | `M-ABZ-01` on `B-ABZ-01`, ABZ encoder | historical: rubbing encoder magnet contaminated every friction number (`drag_c` 1.05 A, "4.5 N / 46%"). Not used; voltage-derived numbers 1.89% low |
| **J01** (= A2) | `B-SPI-01`, MT6816 SPI | reference actuator, belt on (recipe B) |
| **J02** (= rebuilt A1) | `M-ABZ-01` on `B-ABZ-01`, MT6816 SPI | belt on (SCB pulley). ESC board failed after P8; its MT6816 board found dead 2026-10-08 (MISO stuck low) and replaced, so its stored `zea`/`dir` belong to the old encoder |
| J03 | board_3 + motor_2 | belt-off baseline complete (2026-10-10) |

`zea` and `dir` never transfer between joints. `R_eff`, `U0`, `Ke`, `L`, `vbus_scale`, `i_scale`
are per board (§21).

### 8.1a — J01, belt off (AUTOCALIB 2026-08-07, rescaled by M1 ×1.010768 on 2026-08-18)

| Constant | Value | Quality |
|---|---|---|
| `zea` | **6.0542 rad elec** | 7 alignments, SE 1.57° |
| `dir` | CW | 7/7 |
| `R_eff` | **0.22346 Ω** | 9 pts, rms 1.84 mV, SE 0.57%; 0.46 V ladder (§8.1d) |
| `U0` | **0.01037 V** | real uncertainty ±0.003 V (fit SE understates ~2×); see below |
| `Ke` | **0.017941 V/(rad/s)** | 10 pts both directions, SE 0.05% |
| `Kt` | 0.026912 N·m/A_true | derived, 1.5·Ke. vs KV360: +1.45% |
| `L` | **43.77 µH**, τ_e **195.87 µs** | 18-point dithered fit; incremental inductance at 0.9–3.2 A |
| Drag, belt off | fwd `0.0749 + 9.33e-4·ω`, rev `0.0816 + 7.27e-4·ω` | fit rms 0.70 / 0.74 mA |
| `\|I\|/Iq` | 1.245–1.313 | sanity band 1.15–1.40 |
| INL | 1.029° mech pk-pk | 1/rev 0.369°, 2/rev 0.204° |
| ZEA residual | +0.467° elec | parity even part |

The `Kt`-vs-KV360 comparison cannot check `vbus_scale` (Ke derives from it); only M1 against
an external meter does. It does exclude KV380 (7–8% out).

**`U0` is unsettled.** Stored 0.01037 V. The better-conditioned 0.68 V ladder (2026-08-20) gives
0.01686 V cold; M2's self-fit gives 0.01350 V; the M6a step prefers 0.003–0.008 V. Use
**0.010 ± 0.005 V**. It does not matter for `dead_zone`: the deadband is the ratio `U0/R` =
**0.046 A** (0.11 N at the foot), unchanged by any rescale. J02: stored 0.014937 V; the
2026-08-20 ladder read 0.0269 V (per-board dead-time difference).

### 8.1b — J02, belt off (AUTOCALIB 2026-08-08, rescaled by M1 ×1.018904)

Full row in `joint_cal.h`.

| | J01 (`B-SPI-01`) | J02 (`B-ABZ-01`) | Gap |
|---|---|---|---|
| `vbus_scale` | 0.008448 | 0.008516 | +0.80% |
| `R_eff` | 0.22346 Ω | 0.22810 Ω | +2.08% |
| `Ke` | 0.017941 | 0.018097 | +0.87% |
| `L` | 43.77 µH | 46.25 µH | +5.7% |
| τ_e = L/R | 195.87 µs | 202.74 µs | — |
| `Kt` = 1.5·Ke | 0.026912 | 0.027145 | vs KV360 +1.45% / +2.33% |
| Breakaway (M4) | 0.2923 A_rep (n = 11) | 0.2983 A_rep (n = 18) | 0.19σ |

Falsifiable check on the M1 corrections: re-measure both banner/meter ratios in one sitting; the
`Ke` gap should come back at 0.87%. If it comes back at 0.06%, the dividers are equal and the M1
reads are wrong.

### 8.1c — `i_scale` (M2, 2026-08-20)

**`I_true = I_reported / i_scale`.** Locked-rotor bus-power balance, `g = 1.5·R/c` with the
ladder's own `R` ([`CALIBRATION.md`](CALIBRATION.md) §20.1).

| | `R` (self-fit) | `c` | **`i_scale`** | ±1σ | σ from 1.0 |
|---|---|---|---|---|---|
| J01 | 0.22332 | 0.34816 | **0.9621** | 1.18% | 3.2 |
| J02 | 0.22580 | 0.34953 | **0.9690** | 2.69% | 1.2 (provisional) |
| J03 | — | — | **0.9797** | 1.73% | provisional |

True current is 2–4% above reported. **Do not pool into a fleet constant** (two or three samples
cannot establish commonality); `i_scale` stays per row. Known bias: true `g` is likely slightly
further below 1 than measured.

**Units (option A, 2026-10-01).** Reported amps (`_A_rep`) are the firmware's current unit.
`R_eff`, `L`, drag, breakaway, PI gains and every bench-demonstrated limit are A_rep and are never
converted. Physical torque **τ = N × Kt × I_true at the output, η excluded** meets them only
through `torqueOutToIrep()` / `irepToTorqueOut()`, in both directions. Commands go through
`tauOutCmdToIq()`: clamp τ_max (N·m) → convert → clamp the A_rep envelope **last**.

| Limit's source | Unit | Example |
|---|---|---|
| Measured on the bench | A_rep, never converted | 1.6 A cap, ladder currents, `AC_IMAX_ABORT_A_rep`, sense guard |
| Control contract | N·m output, converted once | τ_max, τ_ff, kp, kd |
| Physics / datasheet | A_true or N, converted at init (I_rep = I_true × `i_scale`) | board rating, thermal, B10 skip force |

**J01: 1 A_rep = 0.2517 N·m output** (`calKtCmd()` 0.027972 × 9). Option B (correct the sense gain
at source) reopens only with a fleet-wide re-characterisation **and** `g` shown common across 3–4
boards.

### 8.1d — `R_eff` depends on the current range that measured it

| Joint | Extended ladder (to 0.68 V, ~3 A) | Old range (to 0.40 V) | Range effect | Stored |
|---|---|---|---|---|
| J01 | 0.21795 | 0.22013 | −0.99% | 0.22346 |
| J02 | 0.21894 | 0.22247 | −1.59% | 0.22810 |

J01 local slope 0.2316 Ω at 0.5 A → 0.2097 Ω at 3.0 A (quadratic term reproduces to 2.7%).
**Stored values correspond to the 0.46 V ladder and are not updated** (changing R without L would
corrupt τ_e and the loop-gain provenance). **A phase-3 reading of ~0.222 Ω on the 0.68 V ladder
is not a fault.** M2 uses its own range-matched R.

### 8.1 Electrical — shared

| Quantity | Value | Note |
|---|---|---|
| Kt | `calKt()` = 1.5 × Ke per row (never stored). Per reported amp `calKtCmd()` = Kt / `i_scale`: J01 0.027972, J02 0.028013 | Kt ≠ Ke in SimpleFOC's amplitude-invariant convention (Ke = Kt excluded at 15σ) |
| **`T/T_loop`** | **0.958 ± 0.015**, n = 7, 3 assemblies | transport delay = one loop period. `T_DELAY_PER_LOOP` in `fleet_config.h`. **Belt-off only.** Torque loss at 270 rad/s 0.95%: **no angle compensation** |
| PWM | 25 kHz, SVPWM | ceiling V_bus/√3 |
| Loop rate | 13.5 kHz TORQUE(I) armed, 15 kHz TORQUE(V), 21 kHz OPENLOOP armed, 126 kHz disarmed | J01 MIT build: ~12.4 kHz TORQUE(I) armed (BELT_DRIVE §22.7.7) |
| Telemetry print cost | ~950 µs at 921600 baud | float formatting dominates |
| Bus-sense scale | **per board:** J01 0.008448, J02 0.008516 (M1, UT89X at the pads) | full scale 34.60 / 34.87 V |
| Seed linearity | J02 seed vs UT89X at 12.24 / 12.25 / 22.73 V: −0.16 / −0.11 / +0.04% | |
| Seed boot-to-boot | per-boot offset up to 0.05 V | check banner vs meter at session start (±0.03 V, else reboot) |

Encoder excluded as a delay source (MT6816 1 µs typ / 3 µs max).

### 8.2 Mechanical / drivetrain

**Geometry:**

| Quantity | Value |
|---|---|
| Pinion pitch radius r₁ | 3.8197 mm (12T GT2) |
| Output pitch radius r₂ | 34.3775 mm (108T); 9:1 exact |
| Centre distance C | 44.5 mm |
| No-idler belt path | 230.96 mm; slack 1.04 mm on a 232 mm belt |
| Belt travel per motor rev | 24.000 mm |
| Encoder at the belt | 1 count = 1.4648 µm (16384 CPR) |
| Belt batch, n = 10 | σ(L) 0.158 mm, range 0.60 mm |
| Idler bearing | OD 8.99 mm, width 4.96 mm |

**Inertia and friction, belt off:**

| Quantity | Value | Note |
|---|---|---|
| **`J_rotor`** | **20.2 ± 2.4 × 10⁻⁶ kg·m² (±12%)** | M6a driven step (`t` mode), J01 20.3e-6, J02 18.7e-6. J02's gap is its higher drag map (rerun with J01's map: 20.10e-6). Limiting error is the drag map. Fleet constant |
| J_total | not measured | M6b, `t` mode |
| Breakaway, J01 | 0.2923 A_rep ± 0.0184, sd 20.9% | **0.3038 A true = 8.18 mN·m → 1.322 N/leg = 13.5%** of a 9.81 N standing load (η 0.92) |
| Breakaway, J02 | 0.2983 A_rep ± 0.0259, sd 36.8% | 0.3079 A true = 8.36 mN·m → 1.349 N/leg = 13.75% |
| Cogging (84/rev) | **1.48 mN·m → 0.237 N/leg (2.4%)** | half the M4 fwd/rev difference at one position. No cogging table |
| 6th electrical harmonic (42/rev) | 0.029 A | dead time + back-EMF shape |
| Current-mode torque ripple floor | 0.217 A pp | Iq_pp at 35 rad/s, TORQUE(I) |
| Friction trial-to-trial | ±20% (J01), ±37% (J02) | handling the shaft resets it (below) |

**J01 belt-off terms per leg** (η included): breakaway 1.321 N (13.5%) · mechanical without cogging
1.276 N (13.0%) · cogging 0.237 N (2.4%) · dynamic drag (moving, 0.078 A) 0.336 N (3.4%).

**Belt on (J01, B11, recipe B):** drag_c fwd 0.3169 / rev 0.3330 A, drag_v 3.24e-3 / 2.66e-3
A/(rad/s); breakaway 0.295 A; backlash G 5.8 counts (0.014° output); k_beltline 67.2 kN/m at its
clamp (±10% clamp-dependent); ring f_d 65.0 Hz, f_n 69.8 Hz, ζ 0.36. Detail: BELT_DRIVE §22.6.

**Handling rule.** Three untouched consecutive readings: sd 0.0029 A (cv 1.2%), 30× tighter than
the pooled scatter. Rotating the shaft by hand redistributes grease and resets friction, so
repeats at one position are pseudo-replicates. For ± decompositions, return the rotor to the same
count. Let the rotor settle into a cogging detent before each M4 reading (a rotor left mid-creep
reads far low in the opposite direction).

**Forward/reverse drag asymmetry, belt off:** Coulomb 0.0749 / 0.0816 A (9%), viscous 9.333e-4 /
7.272e-4 A/(rad/s) (28.3%), fit rms 0.70 / 0.74 mA. Real, cause unknown, 0.04 N at the foot. Not
chased; `JointCal` carries four drag fields.

**`DRIVETRAIN_ETA` = 0.92 is circular** (back-solved from a force computed with it). It keeps new
conversions consistent with old ones; M14 (load cell on an assembled leg) replaces it with a
lumped `G·η·Kt`. **Do not double-count:** η is the load-dependent belt loss; `drag_c`/`drag_v` are
the load-independent loss. Feed `footForcePerMotor()` a torque that already has drag subtracted.

Friction referred to the foot scales with G exactly as useful torque does, so the gear ratio and
link lengths cannot improve transparency on the friction axis. Only lower friction or firmware
cancellation can.

### 8.3 Deferred, with promotion conditions

| Item | Size | Promotes when |
|---|---|---|
| J_total | — | M6b on a built actuator |
| Force per amp | 4.145 / 5.000 N/A at α = 70 / 40° (closed-form Jacobian); 4.186 at 55° (the old 4.32 is the stroke mean, 4.296) | needs the real leg (M14) to validate η |
| Sense gain mismatch (14/rev) | ~9% | if impedance force ripple above ~1 N matters |
| Genuine-board acceptance run | — | completes the EG2124A evidence table |
| **`DRIVER_VOLT_LIMIT` 6.0 → ~V_bus** | phase voltage capped at **3.46 V** (Uq_max = 0.5774 × 6.0); rig tops out near 190 rad/s unloaded | **before the first commanded velocity above 150 rad/s.** Raising it invalidates only `U0` and the low-side current-sense window (`R_eff` and `Ke` cancel). Afterwards re-run phase 3 and check phase 1 and the phase-5 \|I\| ratio |
| Live Vbus (`VBUS_LIVE`) | robot sag 3–7 V; at 10 A, 2.4% on R/U0/Ke | PA0 is already ADC1 rank 5 in the DMA buffer, but that converter is uncalibrated (`CALFACT = 0`, reads ~60 counts low). Promote when bench currents exceed ~10 A, the first multi-actuator bus, or constants must be compared across sessions. **Acceptance: `Vdma` vs a terminal meter at two voltages ≥ 5 V apart** (not seed vs DMA) |
| `CALFACT` noise floor | never measured | before comparing two `CALFACT` values. Cost: 5 power cycles |
| PB14 thermistor | — | when a joint runs unattended. Hot/cold `R_eff` is M10 (phase 3 before/after 60 s of load) |
| U₀ feed-forward | J01: 0.078 A ≈ 3% of standing load at 5S | when a board measures `U0` > 0.03 V. `dead_zone = 0.005` stays |

### 8.4 How to interpret `|I|`

- At standstill: trustworthy (`|I|/Iq` = 1.224).
- Spinning, inside a burst capture: trustworthy when averaged.
- Spinning, in one telemetry line: **not** trustworthy (one unsynchronised instant; print-block
  commutation freeze, §12). Seen 6.9–15.5 A against a true 1.2 A.
- `Uq` at its limit: meaningless. Check `Uq` first.

---

## 9. Drivetrain health

### 9a. 1/rev disturbance — not motor-internal (J02, belt off, pinion off, 2026-08-13)

| Quantity | Capture 1 (Uq 1.30 V) | Capture 2 (Uq 0.60 V) | A1 (belt on, rubbing magnet) |
|---|---|---|---|
| Mean velocity | 70.34 rad/s | 31.45 rad/s | 22.55 rad/s |
| **1/rev amplitude** | **0.0262 A** | **0.0097 A** | 0.185 A |
| 7/rev (sense offset) | 0.0311 A | 0.0385 A | 0.042 A |
| 14/rev (sense gain) | 0.0250 A | 0.0217 A | 0.070 A |
| `corr(Iq, vel)` | −0.042 | −0.103 | −0.78 |
| Folded 1/rev velocity ripple | 0.3% | 0.5% | 8% |

1/rev = 0.042–0.113 N at the foot (0.4–1.1% of standing load). Motor swap cancelled; no 1/rev
feed-forward table. 7/rev (current-sense offset) now dominates. Fleet cross-check: J02 70.34 rad/s
at Uq 1.30 V vs J01 71.02 (1%).

### 9b. Output-rate drag modulation (J02, belt on, 2026-09-02 / 09-05)

TORQUE(I) 1.00 A setpoint (Uq saturated at 2.0 V, so voltage-limited, Iq ~0.51 A), output free.
Three captures per belt, harmonics fitted together at output orders 1, 2, 9, 18 against unwrapped
output angle.

| Order | Belt 105.0 (n = 3) | Belt 104.7 (n = 3) | Change | p |
|---|---|---|---|---|
| **h1** — 1/output rev | 0.0655 ± 0.0053 A | 0.0621 ± 0.0034 | −5.1% | 0.42 |
| **h2** — 2/output rev | 0.0055 ± 0.0027 | **0.0323 ± 0.0025** | +490% | 0.0002 |
| h9 — 1/motor rev | 0.0674 ± 0.0029 | 0.0594 ± 0.0065 | −11.9% | 0.15 |
| h18 — 2/motor rev | 0.0862 ± 0.0004 | 0.0820 ± 0.0029 | −4.9% | 0.13 |

- **h1 (0.065 A = 3.0% of standing load, 0.29 N at the foot) is on the output side** — pulley
  eccentricity or output-shaft bearing. A belt-borne share > ~0.013 A is excluded. Separating
  pulley vs bearing needs an indexed-runout test (pulley reprint with 4 threaded M3 holes at 90°,
  r ≈ 28–30 mm).
- **h2 is belt-borne** (belt 104.7: 0.032 A = 0.14 N). **Fleet screen: an unexplained 2/rev on a
  joint → swap the belt first.**
- Output rate (9.000 motor revs) and belt rate (9.667) are only 7.4% apart; separating them by
  frequency would need ~12,800 samples (`LOG_N` is 1000). Use a belt swap instead.
- Phase is not comparable across captures (no absolute output index); amplitude is.
