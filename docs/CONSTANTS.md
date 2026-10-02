# MASTER TABLE — measured constants

**This file outranks every other number in the doc set.** §8 and its subsections
are the master table; §9 is drivetrain health.

Belt-*on* plant states live in [`BELT_DRIVE.md`](BELT_DRIVE.md) §22.1 — they are
per-plant and expire on every mechanical change, so they are deliberately not
mixed into the per-assembly tables here.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 8. MASTER TABLE — MOTOR 1 / ASSEMBLY A1 ONLY

> ### ⚠ THIS TABLE DESCRIBES ONE PHYSICAL ACTUATOR
> Every number below was measured on **A1** — the original ABZ assembly, motor
> M-ABZ-01 on board B-ABZ-01. **It is not a fleet specification.** A2 (the SPI
> reference actuator, now J01) has its own numbers in §8.1a, and each of the
> twelve robot joints will have its own row in `joint_cal.h` (§21).
>
> Measured spread between A1 and A2 so far: `R_eff` **+1.8%**, `Ke` **+0.9%** —
> small, but `ZEA` and `sensor_direction` differ *completely* and are not
> transferable at all (§21).
>
> ⚠ **2026-08-18: every number in §8.1/§8.2 below was taken at `vbus_scale`
> 0.008358, which is B-SPI-01's *original and wrong* divider.** A1 sits on
> `B-ABZ-01`, whose measured divider is **0.008516** — so every voltage-derived
> A1 constant here (`R_eff`, `U0`, `Ke`, `L`, and every `Kt`-derived force) reads
> **1.89% low**. They are **deliberately not rescaled**: this is the historical
> record of a superseded assembly with a rubbing magnet, and rewriting it would
> falsify the record. **Do not compare an A1 number with a J01/J02 number without
> applying the factor** — the A1-vs-A2 spreads quoted just above are exactly that
> kind of comparison and are unreliable to ~1–2%.

### 8.1a — ASSEMBLY A2 (= J01), the SPI reference actuator

**AUTOCALIB rev2, session 3, 2026-08-07, belt OFF. VERDICT PASS, all six phases.** Firmware Vbus 12.46 V.

> ### ⚠ RESCALED 2026-08-18 by M1 — ×1.010768
> `R_eff`, `U0`, `Ke` and `L` below are the 2026-08-07 fits **multiplied by 1.010768**. They were **not re-measured.** M1 on `B-SPI-01`: banner 12.39 V at `vbus_scale` 0.008489 against UT89X **12.33 V** → true scale **0.008448**, superseding the original 0.008358.
>
> **The original error was the meter, not the fit.** `0.008358 × 1.0111` (the DT9205A's measured DCV gain error) `= 0.0084507` against `0.0084479` measured — **0.03% apart.** The old M1 error is fully explained with no residual, which is an independent confirmation of the meter verdict by a different route.
>
> **`τ_e = L/R` is unchanged at 195.87 µs** (was 195.90). `L` and `R` scale together, so **the current-loop gains are untouched** — the boxed warning in [`CONTROL.md`](CONTROL.md) §10 is unaffected. Plant DC gain moves 1.1%, negligible against ζ = 0.60.
>
> **Every `Kt`-derived force and torque figure on this joint moved ×1.010768.** Anything in reported amps or radians did not.

| Constant | Value | Quality | Verdict |
|---|---|---|---|
| `zea` | **6.0542 rad elec** | 7 alignments, sd 3.32° elec, SE **1.57°** (78% of the 2.0° budget) | PASS |
| `dir` | **CW** | 7/7 agree | PASS |
| **`R_eff`** | **0.22346 Ω** | 9 pts, rms **1.84 mV**, SE **0.57%**, 0 dropped. 3-session spread 0.6%. **RESCALED** from 0.22108 | PASS |
| `U0` | **0.01037 V** | SE 0.0016, 6.5σ **in this fit** — but see below. **RESCALED** from 0.01026 | **PASS, overconfident** |
| **`Ke`** | **0.017941 V/(rad/s)** | 10 pts both directions, SE **0.05%**, intercept 0.8 mV. **RESCALED** from 0.017750 | PASS |
| `Kt` | 0.026912 N·m/A | **DERIVED** = 1.5·Ke, not stored. vs KV360 → 0.026526, **+1.45%** (implied KV 354.8). ⚠ **NAMEPLATE check only — see the retraction below** | PASS |
| **`L`** | **43.77 µH** (τ **195.87 µs**) | **18 fit points**, dithered grid, t_d 80 µs. **RESCALED** from 43.31 µH; **τ is a ratio and did not move** | PASS |
| Drag (belt OFF) | fwd `0.0750 + 9.33e-4·ω` · rev `0.0816 + 7.27e-4·ω` | **28% viscous asymmetry, reproducible** | — |
| `\|I\|`/`Iq` | 1.245 – 1.313 | gross-sanity band 1.15–1.40 | PASS |
| `T/T_loop` | 0.974 / 0.945 | T = 73.0 / 70.8 µs at f_loop 13347. Estimates agree to 3.1% | PASS |
| INL | **1.029° mech pk-pk** | 1/rev 0.369°, 2/rev 0.204° mech | PASS |
| ZEA residual | +0.467° elec | independent check via parity even-part | PASS |

> ### ⚠ RETRACTED 2026-08-18 — "`Kt` +0.38% vs KV360 confirms the voltage scale"
> That line stood in this table, in the `joint_cal.h` row comment and in AUTOCALIB's printed report. **It is circular.** `Ke` is computed *from* `vbus_scale`, so a `Kt`-vs-nameplate comparison cannot referee the scale it was derived from. The tight +0.38% was a **coincidence produced by a divider that was 1.1% low**; at the corrected scale the same joint reads **+1.45%**, and J02 reads **+2.33%**.
>
> **Only M1 against an external meter checks `vbus_scale`.** What the comparison is still good for is excluding a grossly wrong nameplate KV — KV380 lands 7–8% out, not 1–2% — so **§24.1's KV360 resolution does not reopen**; see the note there on what the rescale did to the agreement figures.

**`L` = 43.77 µH SUPERSEDES 44.43 / 44.8 / 45.1 µH.** Those came from 5- and 7-point fits on a sample grid that was *phase-locked to the control loop*, not randomly scattered as assumed: `acService()` **is** the loop, so samples landed at step + {0, 60, 120…} µs and only every third 20 µs bin was ever visited. The earliest fitted sample therefore sat at a fixed sub-band time, which biases τ **high**. The F5 per-repeat dither fixed it — 18 fit points, τ = 195.87 µs, **reproduced by an independent offline refit at 195.92 µs** with t_d 79.7 vs 80 µs. The two sub-band bins at 50 and 70 µs sit at frac ≈ 0.000 and −0.001, which independently confirms the ~80 µs step-onset lag that `t_d` absorbs.

Measured at 0.9 → 3.2 A, so this is the *incremental* inductance near the operating point; partial saturation and lamination eddy currents make it legitimately lower than a small-signal value.

> **`U0` PASSES at 6.5σ and still does not reproduce, and 2026-08-20 made that worse rather than better.** The extended 0.68 V ladder determines `U0` far more tightly (**13.0σ / 12.1σ**, against 9.1σ / **2.8σ WARN** on the old 0.46 V top point) — and it returns **0.01686 V cold on J01**, against the stored 0.01037. M2's own independent self-fit on the same joint the same day gives **0.01350 V**. ⚠ **The stored value was deliberately left alone** (the whole 2026-08-20 session changed only `i_scale`), **but a better-conditioned fit disagreeing with the stored value by 63% is a live question, not a closed one** — it is larger than the ±0.003 V band this box recommends, and it is the one number from that session that was not reconciled. J02's new reading is **0.0269 V, 60% above J01's 0.0169** — a real per-board difference in dead-time behaviour, consistent with two boards that also differ 0.80% in divider. **What does not move: the deadband `U0/R`**, a ratio, and therefore `dead_zone`. Earlier sessions: 0.00367 / 0.00559 / **0.01026 V** *(at the pre-2026-08-18 scale; ×1.010768 gives 0.00371 / 0.00565 / **0.01037**, which is what the row now stores — the rescale is common-mode and changes nothing about the spread)*, an RMS spread of 0.0033 V against a per-fit SE of ~0.0016. **The fit's SE understates the real uncertainty by ~2×.** `U0` is the intercept of a sweep spanning only 0.08–0.46 V; any mild curvature — current-sense gain nonlinearity, or a dead-time voltage that itself depends on current — redistributes between slope and intercept, and the two are strongly anti-correlated. Cross-check: at U = 0.460 the measured current went 2.0690 → 2.0439 between sessions, a 1.2% impedance rise consistent with a few °C of winding warming, and the fit absorbed part of that into the intercept. **Use ±0.003 V, not ±0.0016.**
>
> **It does not matter, and the rescale does not reach it.** The deadband is a RATIO, so the factor cancels exactly: 0.01037 / 0.22346 = 0.01026 / 0.22108 = **0.046 A deadband** → 0.11 N at the foot. At 5S, ~0.017 V → 0.078 A → 0.18 N. See §8.3 for what this does to the `dead_zone` promotion condition.

**The belt costs ~0.97 A of Coulomb friction** — A1 belt-on was 1.05 A, A2 belt-off is 0.079 A. That subtraction is only possible because the belt-off baseline was taken *before* the belt went on, and it cannot be recovered afterwards.


Everything here is bench-measured unless marked. **This table outranks any other number in this document.**

### 8.1b — ASSEMBLY J02 (= rebuilt A1), `M-ABZ-01` on `B-ABZ-01`

**AUTOCALIB rev2, 2026-08-08, belt OFF. VERDICT PASS, all six phases.** **RESCALED 2026-08-18 by M1, ×1.018904** — the 2026-08-08 fits had been taken against `B-SPI-01`'s inherited 0.008358, and this board's own divider is **0.008516**. The constants and the full provenance live in **`src/joint_cal.h`**, which is the source of truth for per-joint values; restating them here is what the one-source-of-truth rule exists to prevent. What belongs in the master table:

| | J01 (`B-SPI-01`) | J02 (`B-ABZ-01`) | Gap |
|---|---|---|---|
| `vbus_scale` | 0.008448 | 0.008516 | **+0.80%, measured** |
| rescale applied 2026-08-18 | ×1.010768 | ×1.018904 | — |
| `R_eff` | 0.22346 Ω | 0.22810 Ω | **+2.08%** (was +1.26%) |
| `Ke` | 0.017941 | 0.018097 | **+0.87%** (was +0.06%) |
| `L` | 43.77 µH | 46.25 µH | +5.7% (was +4.8%) |
| `τ_e = L/R` | 195.87 µs | 202.74 µs | **ratios — did not move at all** |
| `Kt` = 1.5·Ke | 0.026912 | 0.027145 | vs KV360: +1.45% / +2.33% |

> **⚠ RETRACTED 2026-08-18 — the `R`-vs-`Ke` argument.** It read: *"`R_eff` is 1.26% above J01 while `Ke` is only 0.06% above; both scale with the voltage belief, so a scale error would have moved them together; it did not, so the `R` difference is real."* **That assumed the two boards shared one scale. They do not.** A per-board scale difference moves `R` and `Ke` together *within* a board, so the cross-board comparison never tested what it claimed — and "the first time the fleet table separated those two explanations" goes with it. **What actually separated them was M1.** The `R` difference may still be real (FET `R_ds(on)`, shunt, solder joints, wire length); there is simply no longer an argument here that it is.
>
> **Steelman, recorded because it is the one piece of evidence *against* the correction:** two independently manufactured motors agreeing on `Ke` to 0.06% is suspiciously good, and happens only if the dividers are equal. Direct measurement outranks a cross-joint coincidence — but it yields a **falsifiable prediction. Re-measure both banner/meter ratios in one sitting: if the corrections are real, the `Ke` gap comes back at 0.87%. If it comes back at 0.06%, the dividers are equal and the M1 reads are wrong.**

**Breakaway (M4, n=18):** 0.2983 A reported ± 8.7% — amps, unchanged by either correction. **0.3079 A true** (`i_scale` 0.9690) → **8.36 mN·m → 1.349 N/leg = 13.75% of standing load** (J01 13.5%). Moved ×1.018904 by M1 2026-08-18, then ÷0.9690 by M2 2026-08-20; at `i_scale` = 1.0 it read 8.10 mN·m / 1.307 N / 13.3%.

### 8.1c — `i_scale`: the current sense under-reads on both boards (M2, 2026-08-20)

**`I_true = I_reported / i_scale`.** Measured by locked-rotor bus-power balance, self-fit formulation (`g = 1.5·R/c` using the ladder's own `R`, not a cross-session phase 3 — see [`CALIBRATION.md`](CALIBRATION.md) §20.1).

| | `R` (self-fit) | `c` | **`g` = `i_scale`** | ±1σ | σ from 1.0 | Verdict |
|---|---|---|---|---|---|---|
| **J01** | 0.22332 | 0.34816 | **0.9621** | **1.18%** | **3.2** | **DECISIVE** |
| **J02** | 0.22580 | 0.34953 | **0.9690** | **2.69%** | 1.2 | ⚠ **PROVISIONAL** — fails the ±2% standard |

**The sense under-reads by 3.9% (J01) and 3.2% (J02)**, so true current is *higher* than reported and torque commanded at `i_scale = 1.0` was delivered ~3–4% **over**.

**Spread 0.72%, inside J02's own uncertainty.** Two boards that differ measurably in divider (0.80%), `U0` (60%) and `R_eff` both land at ~0.96–0.97. Per-board shunt tolerance would scatter in *both* directions; a common offset points at the current-sense constant assumed in `LowsideCurrentSense` being ~3.5% off.

> ### 🔴 DO NOT COLLAPSE THIS INTO A FLEET CONSTANT
> Inverse-variance pooling gives **0.9632 ± 1.10%** and it is tempting. **That is precisely the move that produced the `vbus_scale` failure**, where an inherited "verified" value hid a real 0.80% board difference for two weeks. **Two samples cannot establish commonality; they can only fail to reject it.** `i_scale` stays per-row in `joint_cal.h`.

**A known systematic, recorded because it points the right way:** `R_eff` is current-dependent (below), and `c` is weighted toward the high-current points where `R` is lower, while the self-fit `R` is a linear compromise across the range. **That biases `g` high — true `g` is likely slightly *further* below unity, not closer.** It does not threaten either verdict.

✅ **Units decided 2026-10-01 — option A** (record in `src/fleet_config.h`; README §15 7k). **Reported amps (`_A_rep`) stay the firmware's current unit.** `R_eff`, `L`, drag, breakaway, the PI gains and every bench-demonstrated limit are A_rep and are **never converted**. Physical torque — **τ = N × Kt × I_true at the output, η excluded, friction not subtracted** — meets them only through `joint_cal.h`'s `torqueOutToIrep()` / `irepToTorqueOut()`, in **both** directions, so a commanded and a measured τ cannot disagree by `g`. Commands go through `tauOutCmdToIq()`: clamp τ_max (N·m) → convert → clamp the demonstrated envelope (A_rep) **last**, so a wrong conversion can never command past what the bench survived.

| Limit's source | Unit | Example |
|---|---|---|
| Measured on the bench | **A_rep**, never converted | D3 1.6 A cap, ladder currents, `AC_IMAX_ABORT_A_rep`, the sense guard |
| The control contract | **N·m (output)**, converted once at the boundary | τ_max, τ_ff, kp, kd |
| Physics / datasheet | **A_true** or N, converted at init (I_rep = I_true × `i_scale`) | board rating, thermal, B10 skip force — none written yet |

**J01: 1 A_rep = 0.2517 N·m at the output** (`calKtCmd()` 0.027972 × 9). The banner prints it. **Option B** (correct the sense gain at source, so reported = true) reopens only when **both** a fleet-wide re-characterisation is happening anyway **and** M2 has shown `g` common across 3–4 boards — it moves `R_eff`, `L`, drag, breakaway and every threshold by 1/g, and J01's belt-on phase 4 cannot be re-run (BELT_DRIVE R25).

### 8.1d — `R_eff` is not a single number: it depends on the current range that measured it

**New 2026-08-20, from the extended ladder's per-point data.** On J01, `R_eff` runs **~0.2173 Ω at 3.05 A rising to ~0.2204 Ω at 1.06 A** — about **1.5% of curvature**, and it is real, not noise. Comparing the same joint fitted over two different spans:

| | Extended (0.68→0.08 V) | Old range (0.40→0.08 V) | Range effect | **Stored** | vs like-for-like |
|---|---|---|---|---|---|
| J01 | 0.21795 | 0.22013 | **−0.99%** | 0.22346 | −1.49% |
| J02 | 0.21894 | 0.22247 | **−1.59%** | 0.22810 | −2.47% |

**The stored values are NOT updated, and the reason is not conservatism.** Once compared like-for-like the stored numbers sit 1.5–2.5% above the new session — about 4–6 K of ambient, i.e. **a different day, not a calibration error.** More importantly, **updating `R_eff` without re-measuring `L` would silently corrupt `τ_e` and the loop-gain provenance** ([`CONTROL.md`](CONTROL.md) §10 rests on `L/R` = 195.87 µs). The stored values correspond to the 0.46 V ladder.

**M2 is unaffected** — it uses its own range-matched `R` from the same session, which is one of the three reasons the self-fit formulation replaced the phase-3 borrow.

### 8.1 Electrical — closed

| Quantity | Value | How measured | Confirmations |
|---|---|---|---|
| **Kt** (per Iq) | **Derived, never stored: `calKt()` = 1.5 × `Ke` per row** — J01 **0.026912**, J02 **0.027145** N·m per *true* A. Per **reported** A (what the firmware commands), `calKtCmd()` = Kt / `i_scale`: J01 **0.027972**, J02 **0.028013**. *~~0.0266~~ was A1's pre-M1 value, kept here as a stored number by mistake (R26, 2026-10-01)* | = 1.5 × Ke | ⚠ ~~Matches `60/(2π·KV)` = 0.02653 to 0.2%~~ **RETRACTED 2026-08-18 as circular** — `Ke` is computed from `vbus_scale`, so this cannot check it. See the retraction box in §8.1a |
| **Ke** | **0.0177 V/(rad/s)** *(A1, pre-M1 — history. Live per-row values are in `joint_cal.h`: J01 0.017941, J02 0.018097)* | 5-point `Uq = R·Iq + U₀ + Ke·ω` fit, free pulley | 0.9% scatter; 2-var fit gives 0.01768 |
| **Ke/Kt relation** | **Ke = Kt / 1.5** | Forced by dq power balance in SimpleFOC's amplitude-invariant convention | `Ke = Kt` excluded at 15σ |
| **R_eff** (whole drive path) | **0.218 Ω** *(A1)* | Locked-rotor `Uq`-vs-`Iq` slope | **Five independent determinations.** 3-point locked rotor 0.218; free-spin 2-var fit 0.219; July 0.226; open-loop SVPWM check; **2026-08-01 14-point locked-rotor fit 0.2183 ± 0.0034 (R² = 0.997, 0.10σ from the table value)** |
| **U₀** (dead-time offset) | **0.028 V** at `dead_zone = 0.005`, `V_bus = 11.4 V` | Locked-rotor intercept, dedicated 3-point sweep | Predicted 0.033 from the July dead-zone table. **The 2026-08-01 14-point sweep returns 0.0194 ± 0.0042 V — only 4.6σ from zero, with 0.028 just 2.06σ away and not excluded. That dataset is ill-conditioned for the intercept (`R` and `U₀` trade off; per-point apparent U₀ scatters 0.007–0.034 with no trend) and does NOT supersede this entry.** Scales with bus voltage — §8.3 |
| Deadband in current | **0.129 A** = U₀/R_eff | derived | **Below the 0.16 A sense noise floor — closed** |
| **L** | **65 µH** (range 59–74) | Locked-rotor Uq step, 3 fit methods (302 / 270 / 340 µs) | Step amplitude matches `ΔUq/R` to 4% |
| **τ_e** = L/R | **300 µs** | as above | |
| ~~Loop transport delay ~80–100 µs~~ | **SUPERSEDED** by the `T_delay` row below | Identified from two step responses at different `CUR_TF` | ~~≈ ½ PWM + ½ loop period~~ — that model is 15–29% low |
| PWM frequency | **25 kHz** (library default) | `pwm_frequency` reads `NOT_SET`; STM32 default | Not scope-verified |
| Modulation | **SVPWM** | Boot banner + ammeter A/B | Ceiling `V_bus/√3` = 6.58 V at 3S |
| **`T_delay`** | **`T/T_loop` = 0.958 ± 0.015** | **7 points, 3 assemblies:** 0.956 (16.77 kHz) · 0.953, 0.978 (12.91 kHz) · 0.945, 0.974 (13.35 kHz) · **0.937, 0.961 (J02)**. The mean moved 0.3% and **the sd did not grow** when a third assembly was added — that is the evidence it is a property of the loop, not of a build | **Supersedes ½·T_pwm + ½·T_loop** (15–29% low) **and any statement of T in microseconds.** The PWM period (40 µs) is *shorter* than the loop period, so the duty update lands inside one PWM cycle and the loop period dominates. **Loop rate buys transport delay one-for-one.** Not per-unit — it lives in `src/fleet_config.h` as `T_DELAY_PER_LOOP`, not in `JointCal` |
| Loop rate | **13.5 kHz** TORQUE(I) armed, **15 kHz** TORQUE(V), **21 kHz** OPENLOOP-armed, **126 kHz** OPENLOOP disarmed | `dt_us` per sample, cross-checked against `lps` to 9% | Anomaly closed. The disarmed figure completes the set and confirms it was always armed-vs-disarmed loop content |
| Telemetry print cost | **~950 µs** at 921600 (was 5700 µs; ~800 before `Vb`/`Vb_src` were added) | `pr_us` | 6× improvement. Float formatting dominates |
| **Bus-sense scale** | **PER BOARD — 0.008448 (`B-SPI-01`/J01) · 0.008516 (`B-ABZ-01`/J02)** | **M1 2026-08-18, UT89X at the board terminals.** Supersedes the single 0.008358 this row used to carry | **0.80% apart, measured.** Full scale 34.60 / 34.87 V. The old 0.008358 was 1.1% low — **the DT9205A's ~1.11% DCV gain error, not the fit**, which back-predicted its own two points to 0.07% and was never the problem |
| Bus-sense seed accuracy | **±1.3% boot-to-boot** | 11.28–11.43 V across 7 boots vs 11.26–11.30 true | Per-boot scale factor; record the multimeter reading per session (§11) |

### 8.2 Mechanical / drivetrain

**Drivetrain geometry — closed form, confirmed against CAD 2026-09-02:**

| Quantity | Value | Source |
|---|---|---|
| Pinion pitch radius r₁ | 3.8197 mm (12T GT2) | geometry |
| Output pitch radius r₂ | **34.3775 mm (108T)** | back-solved from the CAD span 32.352 mm; confirms **9:1 exact** |
| Centre distance C | 44.5 mm | design |
| Undeflected span | **32.349 mm** (CAD measures 32.352) | closed form, agreeing to **3 µm** |
| **Exact no-idler belt path** | **230.96 mm** | ⚠ supersedes **229.98** |
| **Slack with a 232 mm belt** | **1.04 mm** | ⚠ supersedes **2.00** — see §22.2 for the retraction |
| Belt travel per motor revolution | **24.000 mm** | π·2r₁ |
| **Encoder scale at the belt** | **1 count = 1.46 µm of belt** | 24.000 mm / 16384 |
| Belt batch spread, n = 10 | **σ(L) = 0.158 mm, range 0.60 mm** | §22.3, 2026-08-31 |
| Bearing, idler | **OD 8.99 mm, width 4.96 mm** | calipers, 2026-09-02 |

| Quantity | Value | How measured |
|---|---|---|
| CPR | 4096 | 1 count error over 23.65 rev |
| Coulomb friction (belt on, leg off) | **1.05 A ≈ 0.028 Nm at the motor** | Drag map from the Ke sweep; flat above ~19 rad/s |
| — referred to the foot | **4.5 N per leg (46% of a standing leg load)** | `F = 2·G·η·Kt·I`, G = N/J̄ = 87.7. **⚠ COMPUTED, not measured — see the η box below** |
| **Breakaway / stiction (J01, belt OFF)** | **0.2923 A reported ± 0.0184 (6.3% SE), sd 20.9%** — amps, unaffected by *either* correction | **M4, 2026-08-08.** **0.3038 A true** (`i_scale` 0.9621) = **8.18 mN·m** at the motor → **0.661 N per motor, 1.322 N per leg, 13.5% of standing load.** ⚠ Three corrections have landed here: `×1.010768` (M1, `vbus_scale`), `÷0.9621` (M2, `i_scale`), and the force figures **include η = 0.92** — the original 1.37 N / 13.9% omitted η while A1's 4.5 N included it. Traceable history: 7.52 → 7.87 → **8.18 mN·m**; 1.257 → 1.271 → **1.322 N** |
| Breakaway / stiction band (A1, belt ON) | 0.34 – 1.34 A | July drag map. Belt-**on** — not the comparison for J01's belt-off number |
| **`J_rotor`** | **20.2 ± 2.4 × 10⁻⁶ kg·m² (±12%)** — ⚠ **flagged, not chased:** if `J` was derived from commanded torque (`J = Kt·I/α`) it inherits `Kt`'s scaling, → ~20.4e-6 (J01 20.5e-6, J02 19.05e-6). **1–2% against a stated ±12%** — one tenth of the uncertainty already carried, so **do not re-measure for this** | **M6a on two joints, 2026-08-08.** J01 20.3e-6, J02 18.7e-6. Motor shaft, before the 9:1. **The 7.9% gap is entirely the drag correction** — see the box below. Fleet constant, `fleet_config.h`. §17 |
| ~~Free coast-down cross-check~~ | — | **RETIRED.** Three attempts on two joints all began *after* the disable: the `L` keypress lands hundreds of ms behind `x`, by which time the coast is nearly over. J02's captured only the last 2.3 rad/s and returned 223e-6, i.e. nonsense. Not viable with keyboard timing; the driven step is the better measurement anyway |
| ~~**1/rev disturbance**~~ | ~~0.185 A → 0.79 N~~ **RETIRED as an A1 artefact** | **CLOSED 2026-08-13 (§9a).** On J02, belt off and pinion off: **0.0262 A max → 0.042–0.113 N = 0.4–1.1% of standing load**, `corr(Iq, vel)` = −0.042, folded velocity ripple 0.3%. The A1 figure was the rubbing magnet. **Motor swap cancelled** |
| **Belt-on drag (J01, no idlers)** | **0.2752 A → 1.18 N/leg (12.1%)** | **§22.1, 2026-08-12.** Belt mesh + output bearing + belt bending = **+0.197 A** over belt-off. Per-plant, expires on any mechanical change — full three-state decomposition in §22.1, not here |
| **Belt-on drag (J01, current faulty idlers)** | 0.6605 A → 2.84 N/leg (28.9%) | Same run. **The sliding brass idlers alone are +0.385 A = 58% of the total.** Being replaced (§22.2) — do not treat this as the belt's number |
| **Belt-on drag, SHIPPED (J01, B11)** | **drag_c fwd 0.317 / rev 0.333 A (mean 0.325)**; viscous fwd 3.24e-3 / rev 2.66e-3 A/(rad/s) — reported amps | **B3 2026-09-30 on pulley recipe B, §22.6.6**, each direction pooled over both run orders (the direction run second reads 0.10–0.13 A higher, D18). Rolling idlers at (1.72, ±10.00), belt 104.85. First-run basis 0.292 A, the same as recipe A's 0.291 (2026-09-28, first-run convention). **In `joint_cal.h`.** Supersedes the two 2026-08-12 diagnostic rows above as *the* J01 belt-on figure |
| **Breakaway, belt ON (J01, B11)** | **0.295 A mean** (recipe B, n = 20, 10 × 2), 0.080–0.475 A — reported amps | **B4 2026-09-30, §22.6.4**, raw rows in `docs/cal/pulley acceptance/`. Recipe A read 0.288 (5 × 2). Mean ≈ drag_c: belt-on there is no extra average stiction. Position sd 0.095 A; direction-biased part up to ±0.18 A (D16). `breakaway_A` in `joint_cal.h` |
| **Backlash G (J01, recipe B)** | **5.8 counts = 8.5 µm belt = 0.014° at the output** | Friction-corrected ladder, 3 ladders / 2 boots, I_f 0.295, §22.6.3. **≤ 20-count gate PASSED.** *(Recipe A read 31–48 under a gate exception, closed 2026-09-30)* |
| **Belt-line stiffness (J01, recipe B)** | k_beltline 64.7 kN/m (ladder, at its clamp); ring f_d 65.0 Hz, f_n 69.8 Hz, ζ 0.36 | §22.6.3 / §22.6.5. **Clamp-dependent ~±10% in k** (§22.5.4); ring stiffness is amplitude-dependent (−0.47 Hz/count) |
| **Cogging (84/rev)** | **1.4 mN·m → 0.25 N per leg (2.5%)** | **REOPENED 2026-08-08.** M4 forward/reverse breakaway pair at the same rotor position. **13× the free-spin figure** — see the box below |
| ~~Cogging, free-spin position-fold~~ | ~~0.004 A → 0.02 N~~ | **SUPERSEDED.** 0.107 mN·m. That measurement was structurally blind, not merely noisy |
| 6th electrical harmonic (42/rev) | 0.029 A → 0.12 N | Position-fold; dead time + back-EMF shape |
| Current-mode torque ripple floor | **0.217 A pp → 0.93 N pp** | Iq_pp at 35 rad/s in TORQUE(I) |
| **J_total** | **not measured** | Deferred to the 10 mm-belt actuator (M6b). Method is `t` mode (voltage step), like M6a (confirmed 2026-09-28, CALIBRATION §20.2) |
| **Friction trial-to-trial variability** | **±20% (J01), ±37% (J02)** | **Static breakaway (sd/mean 20.9%) and two free coast-downs (1.24× and 0.83× the drag map) agree.** Two different physics, same afternoon, same plant |

> ### ⚠ EVERY FORCE FIGURE BELOW CARRIES AN UNMEASURED `i_scale`
> No force number in this document has been validated against an absolute current reference. `F = 2·G·η·Kt·I` uses the **reported** current, and `τ_actual = τ_des / g` where `g = I_reported / I_true` is **unmeasured until M2 runs** (§20.1). A 5%-high current sense means every force here is 5% optimistic, uniformly and silently — it does not show up as scatter, because every internal cross-check divides one wrongly-scaled current by another.
>
> The `Kt` vs `60/(2π·KV)` agreement (+0.38%) confirms the **voltage** scale only; `Ke` is fit from voltage and speed and is independent of current-sense gain.
>
> **Treat every N and N/A in §8.2, §17 and §19 as carrying an unquantified ±(0–6)% common-mode factor** until `i_scale` is measured. It does not change any *ratio* — the 46% friction share, the transparency percentages and the gear-ratio argument are all immune, because the factor cancels. It changes the absolute numbers only.

> ### `J_rotor`: J02 is 7.9% low, and the gap is entirely the drag correction
>
> | Method | J01 | J02 |
> |---|---|---|
> | Impulse–momentum | 19.6e-6 | **18.17e-6** |
> | Angle-trajectory fit | 21.0e-6 | **19.20e-6** |
> | Fit residual | 83 counts | **47 counts** |
> | **Mean** | **20.3e-6** | **18.7e-6** |
>
> J02's trajectory fit is nearly **twice as clean**, yet its mean is 7.9% lower. That is not two facts, it is one: **J02's drag map is 36% higher**, so friction is **31.9% of the impulse** on J02 against 25% on J01. Rerun J02's impulse integral with **J01's** drag map:
>
> ```
> J02 with its own drag map (0.1061 + 9.48e-4·ω) :  18.17e-6
> J02 with J01's drag map   (0.0783 + 8.30e-4·ω) :  20.10e-6   ← J01 measured 20.2e-6
> ```
>
> **J01's own value to 0.5%.** `J` is geometric — two units of the same product at the same 98 g cannot differ by 8% in inertia — so the fleet value stands, with the tolerance widened to ±12%.
>
> **The limiting error is the drag map (25–32% of the impulse), not the fit.** A future two-step version — 0.20→0.80 and 0.20→1.60, differenced over the *same* ω window — would cancel friction exactly. Not worth new firmware yet.
>
> **And the step is a fourth independent probe of `U0`.** Solving the two endpoint speeds for the drag, holding `Ke`, `R` and `U0`, gives implied `drag_c` of 0.0636 (low endpoint) and 0.0436 (high) against phase 5's direct 0.1061 — both far too low *and* disagreeing with each other, the signature of an amplified small difference (`R·Iq_drag` is 0.024 V out of a commanded 0.80 V, so a 0.2% error in `Ke·ω` moves the inferred drag by 8%). Set `U0 = 0` instead of 0.014937 and the endpoints give **0.129 and 0.109, straddling phase 5's 0.1061**. So the step prefers `U0` ≈ 0.003–0.008 V. **Treat `U0` as 0.010 ± 0.005 V.**
>
> *One caveat on "four determinations": three are B-SPI-01 and one is the other board, and `U0` is a per-board property — so that spread mixes within-board reproducibility with board-to-board variation. The within-board evidence stands on its own though: B-SPI-01 gave 0.0037/0.0056/0.0103 against per-fit SEs of ~0.0016, and the same board that read 0.028 as A1 now reads 0.014937 as J02. Both of those are also now known to be at DIFFERENT voltage scales — A1's 0.028 was taken at 0.008358, J02's at a corrected 0.008516 — which is the same board-to-board confound this caveat is about, one layer down.*

> ### Why the free-spin cogging measurement was blind, not just noisy
>
> The 0.004 A position-fold figure was never a measurement of cogging **torque**. It measured how little a 1.4 mN·m ripple perturbs a *spinning* rotor, which is a different question with a much smaller answer.
>
> At 100 rad/s, 84/rev puts cogging at **1337 Hz**. Three independent attenuations stack there:
> - **Rotor inertia dominates.** The speed ripple is `τ/(J·ω_ripple)` = 1.4e-3 / (20.2e-6 × 2π×1337) = **8 × 10⁻³ rad/s**. The rotor simply does not respond, so almost nothing appears in `Iq`.
> - **The current loop cannot track it.** 1337 Hz against a 412 Hz closed-loop bandwidth and an 812 Hz electrical pole.
> - **The logger aliases it.** Decim 8 (~516 µs) has a Nyquist of 969 Hz; even decim 1 gives ~11 samples per cycle.
>
> **M4's forward/reverse pair has none of that.** The rotor is stationary, there are no dynamics at all, and cogging adds to breakaway in one direction and subtracts in the other — so half the difference at a fixed position *is* the cogging torque.
>
> **The conclusion does not change: still do not build a cogging table.** 2.5% of a standing leg load against friction at 13.9%. What changes is that "negligible, closed" was resting on a number that could not have detected the real value, and this is the third time in this project a measurement has agreed with itself while being blind to what it claimed to measure.

**Transparency budget at the foot** (against 9.8 N for one leg of a 4 kg robot):

> ### ⚠ A1's ENTIRE MEASUREMENT SET WAS TAKEN ON A RUBBING ASSEMBLY
>
> The ABZ lost-count fault is now diagnosed as **mechanical jitter from the encoder magnet contacting**. Every A1 number therefore carries an unknown mechanical friction contribution, and **`drag_c` = 1.05 A is belt friction *plus* a rub**.
>
> | Downstream of that 1.05 A | Status |
> |---|---|
> | "4.5 N per leg, **46% of standing load**" — the headline transparency limit | **Suspect.** Part of it is a rub, not belt friction |
> | "Coulomb friction dominates transparency, 5× everything else combined" — a key project learning | **Suspect** — same source |
> | The **M5 belt-drag prior** (`belt-on − belt-off`) | **Do not use 1.05 A as the expectation.** It would make a healthy belt look good |
>
> **The belt is probably better than this project has been assuming.** J01 belt-off is 0.075 A while moving; A1 belt-on was 1.05 A, implying ~0.98 A of belt drag, which was always uncomfortably large for a 10 mm GT2 at 9:1. **A rubbing magnet is a much better explanation than a catastrophically lossy belt.**
>
> **`DRIVETRAIN_ETA` is *not* additionally damaged by this**, for a reason worth stating precisely: η was back-solved as `4.5/(2·G·Kt·1.05)`, and the 4.5 N was itself *computed* as `2·G·η·Kt·1.05` — so **the 1.05 A cancels exactly** and contamination cannot reach η. η is uninformative for a different and worse reason: **it is circular**. See the next box.

**A1, belt ON** (the historical budget — this is what the belt costs, *plus a rub*):

| Source | Foot force | % | Fix |
|---|---|---|---|
| **Coulomb friction** | **4.5 N** | **46%** | **Constant feedforward — highest leverage, ~3 lines** |
| Current-mode ripple (7/rev + 14/rev) | 0.93 N pp | 9.5% | Sense calibration |
| 1/rev mechanical | 0.79 N | 8% | Hardware — localise first |
| Dead-time deadband | 0.55 N | 5.6% | Closed (below noise floor) |
| ~~Cogging~~ | ~~0.02 N~~ | ~~0.2%~~ | Superseded — see below |

**Belt OFF, 2026-08-08 — TWO independently built assemblies.** This is the first real fleet number the project has for transparency. All figures use `F = 2·G·η·Kt·I` with η = 0.92, the same formula as the A1 row above, so the two are directly comparable.

| | **J01** | **J02** |
|---|---|---|
| Breakaway, `n` | 11 | **18** |
| Mean | 0.2923 A | **0.2983 A** |
| SE | ±0.0184 (6.3%) | ±0.0259 (8.7%) |
| sd | 0.0611 | **0.1098** |
| **Coefficient of variation** | **20.9%** | **36.8%** |
| **True** mean (`I_reported`/`i_scale`) | **0.3038 A** | **0.3079 A** |
| Torque per motor | **8.18 mN·m** | **8.36 mN·m** |
| **Foot force per leg** | **1.322 N** | **1.349 N** |
| **% of a 9.81 N standing load** | **13.5%** | **13.75%** |
| *`Kt` (M1-corrected 2026-08-18)* | *0.026912* | *0.027145* |
| *`i_scale` (M2 2026-08-20)* | *0.9621* | *0.9690 — provisional* |
| *at `i_scale` = 1.0, for tracing old notes* | *7.87 mN·m, 1.271 N, 12.95%* | *8.10 mN·m, 1.307 N, 13.3%* |

**The means differ by 0.0060 ± 0.0318 A = 0.19σ — statistically indistinguishable.** Two independently built assemblies both land at **13–14% belt-off**, which is **3.4× better** than A1's contaminated 46%. *The amps and the 0.19σ are untouched by BOTH corrections; only the torque and force rows moved — ×1.010768 / ×1.018904 by M1, then ÷0.9621 / ÷0.9690 by M2. Net on the newtons: ×1.0505 (J01), ×1.0515 (J02).*

> **But J02's scatter is 1.8× larger, and that is the real finding.** *"Feels a little rougher to spin by hand"* was the correct read: **worn or redistributed grease raises the variance, not the level.** The tactile impression was more informative than the mean — see the handling box below.

**J01's other belt-off terms** (per leg, ×2 motors, η included):

| Source | Per leg | % of 9.81 N | Note |
|---|---|---|---|
| **Breakaway (static)** | **1.321 N** | **13.5%** | 0.2923 A |
| — mechanical only, cogging removed | 1.276 N | 13.0% | 0.2825 A |
| **Cogging** | **0.237 N** | **2.4%** | 1.48 mN·m |
| Dynamic drag (already moving) | 0.336 N | 3.4% | `drag_c` ≈ 0.078 A |

*Newtons carry both corrections: ×1.010768 (M1, 2026-08-18) then ÷0.9621 (M2, 2026-08-20), net ×1.0505. **The amps in the last column are what the firmware reported and did not move under either** — that invariance is what made two independent corrections tractable at all. At `i_scale` = 1.0 these read 1.271 / 1.228 / 0.228 / 0.323 N.*

> ### ±20–37% "plant variability" is created by HANDLING THE SHAFT
>
> Three consecutive J02 forward readings **with the shaft untouched**: 0.2450, 0.2450, 0.2500 — **sd 0.0029 A, cv 1.2%.** That is **thirty times tighter** than the 36.8% pooled scatter.
>
> So the spread is neither measurement noise nor moment-to-moment drift: **rotating the shaft by hand redistributes the grease and resets the friction state.** Two consequences:
>
> - **Repeats at one position are pseudo-replication.** Eight readings without handling are close to *one* independent sample, not eight. A "7.2σ direction asymmetry" computed from such a block was counting pseudo-replicates; done properly over six independent positions it is **C = −0.0144 ± 0.0332 A = 0.43σ, not significant.** Cogging averages to zero across position exactly as it must, and there is no direction-dependent mechanical friction to explain.
> - **"Same position, both directions" is not achievable by simply not touching the shaft.** The overrun after each detection moves the rotor — J02's forward readings clustered at raw ≈ 11320 and the reverse ones at ≈ 11398, **79 counts apart = 0.40 of a cogging cycle.** Any future ± decomposition needs the rotor deliberately returned to the same count, or pairs that happen to land within ~10 counts.
>
> **Protocol fix, already in the firmware:** M4's trailing instruction now tells you to let the rotor settle into a cogging detent between readings. A rotor left mid-creep is still elastically loaded and breaks away far too easily in the *opposite* direction — J02 produced a 0.0350 A reading that way, **a quarter of the truth**, after 583 counts of unexplained forward motion. That reading is dropped.

### Forward/reverse drag asymmetry — real, unexplained, and small

Session 3's phase-5 data shows a **28% forward/reverse split in the viscous coefficient** (9.333e-4 vs 7.272e-4 A/(rad/s)) and 8.9% in Coulomb (0.07495 vs 0.08159 A). It is confirmed by two independent observables: reverse spins 0.4% *faster* at the same commanded voltage **and** draws 10% *less* `Iq` at the top point. **It is real, not a fit artefact** — which is why `JointCal` now carries four drag fields instead of a mean.

Quantitatively excluded as causes:

| Candidate | Predicted asymmetry | Observed |
|---|---|---|
| Commutation angle error (ZEA residual + transport delay) | 0.09% | 10.8% ✗ |
| Iron loss from the `Id` asymmetry (0.60 vs 0.44 A) | 0.6% | ✗ |
| Thermal (grease viscosity) — only ~0.5 W of mechanical loss over ~30 s | weak | cannot rule out, cannot size |

**Absolute impact belt-off: 0.017 A = 0.04 N at the foot.** Below every other error in the budget above. **So: keep the four fields, do not chase it now**, and add one cheap discriminator at M5 — **run phase 5 once with the direction order swapped (reverse first).** If the asymmetry follows the *order*, it is thermal and the four fields are freezing a transient as though it were a property. If it follows the *direction*, it is mechanical.

The belt-off fit was re-run cleanly on 2026-08-13 and the asymmetry survives at sub-milliamp residual, so it is not scatter:

| Direction | Coulomb (A) | Viscous (A per rad/s) | Fit rms |
|---|---|---|---|
| Forward | 0.0749 | 9.333 × 10⁻⁴ | **0.70 mA** |
| Reverse | 0.0816 | 7.272 × 10⁻⁴ | **0.74 mA** |
| Asymmetry | 9% | **28.3%** | — |

> ### The prediction that it would grow ~10× belt-on was wrong, in an informative direction — 2026-08-12
>
> | Plant state | Coulomb asymmetry | Viscous asymmetry | Fit rms fwd / rev |
> |---|---|---|---|
> | Belt off | 8.4% | 24.8% | 0.70 / 0.74 mA |
> | **Belt on, no idlers** | **0.3%** | **9.8%** | **5.96 / 9.87 mA** |
> | Belt on, sliding brass idlers | 32.9% | **167.5%** | 14.6 / 66.8 mA |
>
> **The idler-free belt-on state is the most symmetric drag measurement in this project's history — better than belt-off.** So the belt does not amplify the asymmetry; the *faulty idlers* create a much larger one, and they were predicted to, by the thread-walking mechanism whose axial bias reverses with belt direction (§22.2). Removing them removed the direction dependence entirely — confirmation by removal, which is the strongest form available.
>
> **What this leaves genuinely open** is smaller and stranger than before: why is *belt-off* less symmetric than belt-on-no-idlers? Adding a drivetrain should not improve symmetry. Candidates: the 0.017 A absolute difference is near the fit's own resolution at belt-off currents; or the belt's mesh drag swamps a small motor-side asymmetry rather than adding to it. **Impact is 0.04 N at the foot. Do not spend a session on it** — but the direction-order swap at B3 is still the right free test, and it now has a second question to answer.

> **A cross-check that was offered, and why it is not being added.** The `Id` asymmetry itself is fully accounted for by `Ud = Uq·sin(δ)` with `δ_fwd = 0.467° + 3.22° = 3.69°` and `δ_rev = 0.467° − 3.22° = −2.75°`, giving predicted `Id` = 0.583 / 0.434 A against measured **0.5995 / 0.4398** — both within 3%. It was suggested as a free independent confirmation of phase 6.
>
> **It is not independent.** `acBinAngleDeg()` already computes `sin δ = (R·Id − ω_e·L·Iq)/(Ke·ω)` *from* the measured `Id`, and phase 6 splits that δ into the even part (ZEA residual) and the odd part (`T`). Reconstructing `Id` from those two outputs inverts the same equation and returns the `Id` you started from. The only non-circular ingredient is using commanded `Uq` = 2.00 V in place of `Ke·ω` = 1.95 V — a 2.5% substitution on a term that is 95% of the total, which is precisely the size of the "3% agreement". **It tests `Ke`, weakly, and phase 5's fit residual (rms 2.36 mV on ~2 V) already tests `Ke` far better.**
>
> Recorded here as a **consistency** observation. Printing it as a "cross-check" would manufacture the appearance of independent evidence, which is §2's corollary — *a number derived by rearranging your own settings is not a measurement* — with extra steps.

> ### ⚠ `DRIVETRAIN_ETA` = 0.92 is BACK-SOLVED and CIRCULAR
>
> `η = 4.5 / (2 × 87.7 × 0.026626 × 1.05) = 0.918` *(the 0.026626 is the pre-2026-08-18 `Kt`, and the 1.05 A is a **reported** amp; correcting both gives ~0.87, which changes nothing below)*. But **the 4.5 N row's own "How measured" column reads `F = 2·G·η·Kt·I`** — it was computed with a formula that already contained η. Back-solving η out of it returns the η that was put in. **It is a round trip and it proves nothing about the physical drivetrain. Nobody has put a load cell on a foot.**
>
> **It is kept anyway, and the reason is not sentiment.** Because the round trip is *exact*, 0.92 is precisely the η every existing force figure in this document already assumes — so using it keeps new conversions **consistent** with the old ones. It buys consistency, not truth.
>
> **M14** (load cell, assembled leg) measures `G·η·Kt` as **one lumped number** and replaces it. Until then this is structurally the same hazard as `i_scale`: an unverified multiplicative factor applied invisibly to every force output. **The boot banner now announces both** on a built joint.
>
> **Convention — do not double-count.** η is the **load-dependent** belt loss (tooth engagement, belt bending), proportional to transmitted torque. The **load-independent** loss is `drag_c` / `drag_v` in `JointCal`. Feed `footForcePerMotor()` a torque that has **already had drag subtracted**, or you subtract friction once and then multiply it back out.

**Structural note that closes an argument:** friction referred to the foot scales with `G` exactly as useful torque does, so `F_friction/F_max = I_friction/I_max` is **independent of gear ratio and link length.** Changing the reduction cannot improve transparency on the friction axis. Only reducing the friction or cancelling it in firmware can.

### 8.3 Deferred, with promotion conditions

| Item | Size | Promotes when |
|---|---|---|
| ~~Angle lag / d-axis decoupling~~ | ~~T ≈ 143 µs~~ | ✅ **CLOSED 2026-08-06 — see the box below** |
| ~~`J_rotor`~~ | ~~—~~ | ✅ **CLOSED 2026-08-08** — **20.2 ± 2.4 × 10⁻⁶ kg·m² (±12%)**, three methods, two joints. Tolerance widened from ±2.0/±10% when J02's determination came in; the limiting error is the drag map, not the fit (§8.2). `fleet_config.h` |
| **`J_total`** | — | With the 10 mm-belt actuator (M6b) |
| ~~Which `h` gives G = 87.7 /m~~ | ~~—~~ | ✅ **CLOSED 2026-08-12 — the question was mis-framed. 87.7 is the STROKE MEAN and is not evaluated at any height.** The α-average of \|dh/dα\| over α = 40–70° is 102.641 mm → G = 87.68, matching the stored 87.70 to 0.02%. It was never blocked on CAD either: the five-bar is **coaxial-hip**, so the pivot separation is zero by construction. `fleet_config.h` |
| **Force per amp** | Predicted 4.13 / ~~4.32~~ / 4.98 N/A at α = 70/~~55~~/40° | **Endpoints now validated against the closed-form Jacobian to 0.4%** (model: 4.145 / 5.000). **The middle value does not reproduce** — the model gives 4.186 at α = 55°, and 4.32 needs α = 50.9°. Most likely it is the stroke mean (4.296) that acquired an angle label. Still needs the real leg to validate η |
| Sense gain mismatch (~9%) | 0.93 N pp | If impedance force ripple above ~1 N proves to matter |
| ~~1/rev source~~ | ~~0.79 N~~ | ✅ **CLOSED 2026-08-13 (§9a).** Belt-off, pinion-off capture on J02: 1/rev is **0.042–0.113 N**, not 0.79 N, and `corr(Iq, vel)` collapsed to −0.042. Not motor-internal. **Motor swap cancelled.** One residual: **A3b**, a pinion-seated capture to separate bore runout — 10 min, **expires at B0** |
| **`T/T_loop` / phase 6 belt-on validity** | Reads 1.29 belt-on vs 0.958 belt-off | ✅ **RESOLVED as a scope limit, not a constant: phase 6 is BELT-OFF ONLY.** The excess tracks drag at roughly +0.57 per amp and is **not** an alignment artefact (ZEA was at baseline). See §22.1. `T_DELAY_PER_LOOP = 0.958` is a belt-off constant and needs that note at the constant and in the phase-6 output |
| Higher current-loop bandwidth | ~100 Hz available at best | Effectively closed — at 80% of the transport-delay ceiling |
| Genuine-board acceptance run | — | Completes the EG2124A evidence table |
| **`DRIVER_VOLT_LIMIT` = 6.0 V → ~V_bus** | **Caps phase voltage at 3.46 V — 28% of a 12.46 V bus.** Tops the rig out near **190 rad/s unloaded**, less under load | **Before the first commanded velocity above 150 rad/s.** See the box below |
| **`open_test.cpp` holds all logic** | Against §9 of the working context ("no logic in `main.cpp`; wiring only") | **Before CAN / Tier-1 integration.** The Tier-0 boundary is the frozen contract, and it cannot be drawn cleanly through a single 1100-line sketch that is also the bench harness |
| **Live Vbus sampling** | Bench: none. Robot: 3–7 V of sag | ⚠ **2026-08-21: the mechanism is no longer the obstacle — the NUMBER is.** PA0 is rank 5 of ADC1's DMA'd regular sequence, so a live reading costs a RAM read and zero ADC configuration; contention, blind intervals and the register-level work are all moot (§0, §3). What blocks it now is that the DMA path reads ~60 counts (0.51 V) low because its converter is uncalibrated, and the fix as first written over-corrects by 1.97×. **`VBUS_LIVE` stays `false`; nothing may consume the buffer until the residual is ≤ 8 counts and re-verified per board.** Everything below still applies. **STILL DEFERRED, condition WIDENED 2026-08-07 (session 3).** Whichever comes first of *(a)* bench currents routinely above ~10 A, *(b)* the first multi-actuator bus, or ***(c)* whenever constants are compared ACROSS SESSIONS** — `Ke`'s entire 0.57% session drift turned out to be the pack's state of charge (§10), so cross-session comparability of `R`, `U0` and `Ke` all hinge on the bus reading being right *during* the measurement |
| **`CALFACT` noise floor** | ⚠ **Never measured — there is no baseline** | **Before any future `CALFACT` value is compared to another.** The same structural gap as §24.14's banner repeatability: 117 and 71 are single determinations, so a later "71 vs 65" would be uninterpretable. **Cost: 5 power cycles, logging the precal line. Zero bench setup** (§0, §3) |
| **Live Vbus — promotion condition, written 2026-08-30** | — | `Vdma` against a **terminal meter** at two bus voltages ≥ 5 V apart. This also closes H7 for free, since §20.1 already instructs recording terminal voltage at M2 points 1 and 8. **Seed-vs-DMA agreement is NOT the acceptance test** — the seed is a proxy for truth and the meter is truth; at 12.21 V the two paths differ by 0.05 V, which is inside the UT89X's own tolerance, so **these instruments cannot settle it** |
| **PB14 thermistor** | Was the blocker for hot/cold `R_eff` | **DEFERRED, and no longer blocking anything.** See the review below |
| **U₀ feedforward** | A1: 1.26 N (12.9%) projected at 25.2 V. **B-SPI-01: 0.18 N (~3%) at 5S** | **CONDITION REWRITTEN 2026-08-07 (session 3): when a BOARD measures `U0` > 0.03 V.** It was *"when the bench bus exceeds ~15 V"*, which was built on A1's `U0` = 0.028 V and projected a 0.297 A deadband. **`U0` is per-board and B-SPI-01's is 3× smaller** (0.01037 V → 0.046 A now — **the deadband is `U0/R`, a ratio, so the 2026-08-18 rescale cancels out of it exactly and the 0.03 V threshold is unaffected at this magnitude**, 0.078 A at 21 V ≈ 3% of standing leg load, not 12.9%). Bus voltage alone does not decide this; the board's own dead-time offset does. **`dead_zone = 0.005` stays closed for B-SPI-01.** Fix remains U₀ feedforward, not a smaller `dead_zone` — the EG2124A hardware dead time sets the floor |

### ⚠ `DRIVER_VOLT_LIMIT` — the phase-voltage ceiling nobody set on purpose

`driver.voltage_limit = 6.0` is **not** a safety limit. It is the **SVPWM modulation reference**. SimpleFOC computes `Uout = Uq / driver->voltage_limit`, then `T1 = √3·sin(…)·Uout` and `T2 = √3·sin(…)·Uout`. Worst case is mid-sector where both sines are 0.5:

```
T1 + T2 = √3 · Uout   must stay ≤ 1   →   Uout ≤ 1/√3 = 0.5774
Uq_max = 0.5774 × driver.voltage_limit = 0.5774 × 6.0 = 3.46 V
```

The 12.46 V bus never binds, because 6.0 < 12.46. **The board has been running on 3.46 V of usable phase voltage out of a 12.46 V bus — 28% of it.**

**Nothing measured so far was clipped.** The highest `Uq` ever *commanded* is 2.60 V (`AC_W_VLIMIT`); the highest *reached* is 2.00 V; phase 4's step is 0.70 V. All well under 3.46. **Every constant in the J01 row stands.**

**But it is a hard speed ceiling.** Required `Uq` at 270 rad/s ≈ `Ke·ω + R·Iq` = 4.79 V unloaded, ~7.0 V at 10 A. Against 3.46 V the rig tops out near **190 rad/s unloaded** and well below that under load — with the 9:1 belt, a real limit on sprint and jump.

**What raising it would and would not invalidate.** An earlier note in the source said *"it invalidates `R_eff` and `U0`"*. **The `R_eff` half of that was wrong:**

| Constant | Depends on `driver.voltage_limit`? | Why |
|---|---|---|
| **`R_eff`** | **No** | `Ua = Ta · driver_vl` while `Ta ∝ Uout = Uq / driver_vl` — the factor cancels. The **differential** phase voltage depends on commanded `Uq` alone, and with a floating star point only the differential drives current |
| **`Ke`** | **No** | Fit against commanded `Uq` vs ω; same cancellation |
| **`U0`** | **Yes** | `modulation_centered = 1` puts the duty centre at `driver_vl/(2·V_bus)`. Raising 6.0 → 12.46 moves it **24% → 50%**, changing the dead-time / body-diode regime the intercept describes |
| **Low-side current sensing** | **Yes — watch this one** | `LowsideCurrentSense` samples while the low-side FETs conduct. At a 24% centre the low side is on ~76% of the time: a comfortable window. At a 50% centre with high modulation that window shrinks, a known failure mode on this board family |

**So the instruction is not "do not touch it".** It is: *raising it unlocks ~2× the phase voltage and invalidates only `U0` and the current-sense sampling window. Re-run phase 3 afterwards — which re-measures `R_eff` too, and so **tests** the cancellation argument above instead of assuming it — and confirm phase 1 and the phase-5 `|I|` ratio for sense health.*

### Deferral review — live Vbus and the PB14 thermistor (2026-08-07)

~~Both were blocked by the same single piece of work: `analogRead()` returns 0 on any pin of the ADC that `LowsideCurrentSense` owns, so reading either needs a **register-level REGULAR-group conversion**. One task, two payoffs — which is exactly why it looked attractive.~~

> ### ⚠ RETRACTED 2026-08-21 — neither was blocked, and the blocker never existed
>
> **Both channels were already being converted every PWM period and landing in RAM.**
> ADC1's regular sequence is 5 ranks and includes **PA0 as rank 5** and **PB14 as rank 4**,
> DMA'd to a circular buffer (§3 for the map, §0 for the dump). Reading either is a RAM
> read. The "register-level conversion" this paragraph called for was never needed, and the
> premise underneath it — that the current sense owns the *injected* group — was wrong
> (§12).
>
> **The real blocker turned out to be a different one, and it is now named:** the converter
> feeding that buffer has never been calibrated (`CALFACT = 0`; `analogRead()` calibrates on
> every call, SimpleFOC's init never does), so the DMA path reads ~60 counts low. §0 carries
> the mechanism, the four eliminated hypotheses, and the outstanding defect in the fix.
>
> **The deferral below still stands** — on the analysis in it, not on the blocker above.

**Live Vbus — still deferred, and the seed is demonstrably sufficient.** Two independent multimeter checks now confirm it: 11.29 vs 11.30, and **12.51 vs 12.52**. The seed is measured at boot and correct at boot; a 3S pack drops by millivolts over a 90 s AUTOCALIB run at 0.1–2 A. **Bench sag is not yet a real quantity.**

The revised trigger matters though. At **10 A** through a ~30 mΩ pack the sag is 0.3 V = **2.4%**, and that lands directly on `R_eff`, `U0` and `Ke` via `R_measured = R_true·(V_assumed/V_true)`. The 10 mm actuator with a leg attached will reach those currents. **So the promotion condition is now a current threshold, not a joint count.**

**Thermistor — deferred, and it stopped being a blocker.** The only reason it mattered was the open hot-vs-cold `R_eff` item (§16). **AUTOCALIB phase 3 measures `R_eff` in 6 seconds**, fast enough that the motor barely cools: phase 3 cold → load hard for 60 s → phase 3 again. That answers the actual question with no temperature reading at all (§20.3, M10).

What a thermistor would still buy is *continuous* monitoring — valuable on a robot, worth little on a bench where a fingertip works. **Promotion condition: when a joint runs unattended.**

**Net: neither is worth a session now**, and the register-level ADC work drops from "one task, two payoffs" to "one deferred payoff plus a nice-to-have."

---

### ✅ ANGLE LAG — CLOSED 2026-08-06, re-confirmed 2026-08-07 (session 3)

**The transferable constant is `T/T_loop` = 0.961 ± 0.014**, i.e. the transport delay is **one control-loop period**. It lives in `src/fleet_config.h` as `T_DELAY_PER_LOOP`; it is *not* per-unit and does *not* belong in `JointCal`.

| | Value |
|---|---|
| Session-3 measurement | **T = 73.0 µs** (2nd estimate 70.8) at f_loop 13,347 → T/T_loop **0.974 / 0.945**, agreeing to **3.1%** |
| Five-point history | 0.956, 0.953, 0.978, 0.945, 0.974 → **mean 0.961, sd 0.014**, across **2 assemblies and 2 loop rates** |
| **Torque loss at 270 rad/s** | **0.95%** (7 × 270 × 73 µs = 7.9° elec) |

> **Retraction.** An earlier note carried *"preliminary T ≈ 143 µs, implying 40–60 µs unaccounted delay and 3.6% torque loss at 270 rad/s."* That came from a **forward-only incidental capture**, which structurally cannot separate the EVEN terms (ZEA residual, INL) from the ODD one (transport). Two-direction parity separation gives T = 0.96 × one loop period **with no unexplained excess**, and the loss is **0.95%, not 3.6% — a 3.8× overstatement.** The angle-lag sweep that this figure had justified **no longer needs to be run.**

Historical detail, kept because the method is the reusable part: the first SPI determination was **`T = 57.0 ± 6.5 µs`** at f_loop 14,970. Theory `0.5/f_pwm + 0.5/f_loop` gives 53.4 µs — 7% agreement — but that model only *appears* to work at one loop rate and is 15–29% low across the set. Use the ratio.

**Method — parity separation, which is what finally worked.** In voltage mode with `Ud` forced to 0, the measured angle error is `sin δ = A/Ke + B·ω/Ke`. Then:

| Term | Under ω → −ω | Parity |
|---|---|---|
| ZEA residual (constant offset) | unchanged | **EVEN** |
| INL(θ) (fixed error at a rotor position) | unchanged | **EVEN** |
| Transport delay (`δ = ω_e·T`) | **flips sign** | **ODD** |

Two 1000-sample captures at ±109.5 rad/s (each spanning **9.5 revolutions**), binned into 32 positions:

```
ODD  part = +2.506 +- 0.287 deg elec  ->  T = 57.0 us, position-independent (sd/mean 11%)
EVEN part = +0.121 deg elec mean      ->  ZEA residual: ZEA_STORED is right to 0.12 deg
            6.51 deg elec pk-pk       ->  INL = 0.93 deg MECHANICAL (see section 7)
```

**Torque cost — this is why no compensation gets written:**

| ω | δ | Loss |
|---|---|---|
| 109 rad/s | 2.49° elec | 0.095% |
| **270 (takeoff)** | **6.18°** | **0.581%** |
| 314 | 7.18° | 0.785% |

**Do NOT implement angle compensation.** 0.58% at the design operating point, against Coulomb friction at 46%. Promotion condition retired.

**Why the ABZ campaign never converged.** It reported T = 85.7 µs and δ₀ = 2.17° — a 28.7 µs excess over theory that motivated three sessions of searching. Three reasons it was wrong: forward-only data cannot separate even from odd terms; ZEA was redrawn between the forward and reverse sweeps so they could not share a `δ₀`; and the counter was probably leaking during the runs. **The SPI measurement's excess is 3.6 µs. The 28.7 µs was an artefact of the method, not a physical delay.**

**Encoder excluded as a source** — MT6816 propagation delay 1 µs typ / 3 µs max, plus a TIM4 input filter of ≈1.5 µs on the old path. No encoder upgrade helps; MT6826S is worse (10 µs propagation, 100 µs step response).

**Known blind spot, now half-closed:** `No_Mag_Warning` (0x04[1]) and `Over_Speed` (0x05[3]) are SPI-only. **Available on the SPARE**; still absent on the ORIGINAL, which is why its magnet failure mode was invisible.

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

- This **confirms** the earlier 1/rev finding with proper sampling. ~~The July attribution to the pinion remains unproven; the belt-off test in §1 settles it.~~ → **settled, see §9a: it is neither pinion nor bearing on J02.**
- **Reflected cogging is inherent** but, now measured, it is not the problem anyone thought it was.
- Electrical braking is *not* a factor when disabled: back-EMF at hand speeds cannot overcome bus + 2 diode drops. **No phase-disconnect relay needed.**

### 9b. Output-rate drag modulation on J02 — observed 2026-09-02, cause NOT localised

> ### ⚠ This is NOT the §9 / §9a 1/rev. Do not cite one as corroboration of the other.
>
> §9a's 1/rev was at **motor rate**, measured **belt-off and pinion-off**, and closed as a
> rubbing encoder magnet. **This is at one-ninth of motor rate, with the output pulley in the
> loop.** Different frequency, different plant, different part.

**Method.** J02, belt on, **one idler at the innermost slot position**, output free, leg links
off. TORQUE(I) at a 1.00 A setpoint — ⚠ **`Uq` saturated at 2.000 V, so the run is
voltage-limited and `Iq` never reached setpoint (~0.51 A).** Fine for this purpose, because a
load change shows in both `Iq` *and* speed, **but record it in the archive header so nobody
later reads it as a current-controlled sweep.** Three `L` captures (decim 8, 1000 samples,
fs ≈ 1525 Hz, 0.654 s each), spanning 10.6 motor revolutions = **1.18 output revolutions.**
`cnt` unwrapped, folded against output angle, and fitted with harmonics at output orders
**1, 2, 9 and 18 simultaneously**, so motor-rate content cannot leak into the output-rate terms.

| Order | Physical rate | `Iq` amplitude | vs noise | `Iq`–vel phase |
|---|---|---|---|---|
| **h1** | 1 / **output** rev | **0.065 ± 0.003 A** | 18–20σ | **168–175° (antiphase)** |
| h2 | 2 / output rev (ovality) | 0.006 A | 0.8–2.6σ | — |
| h9 | 1 / motor rev | 0.067 A | 18–21σ | 130–144° |
| **h18** | 2 / motor rev | **0.086 A** | 25–26σ | 180–185° (antiphase) |

**h1 is real — three independent confirmations.** Amplitude reproduces across three captures to
**4.6% cv**; it survives polynomial detrending to order 2; and it is **antiphase with velocity**
— under a saturated `Uq` a genuine drag rise *must* slow the motor, and it does. The
per-motor-revolution `Iq` means show a single clean hump.

⚠ **Phase is not comparable across captures** — each starts at a random output angle and there
is no absolute output reference. **Only amplitude is comparable**, and it is tight.

**Magnitude: 0.065 A amplitude = 3.0% of standing load = 0.29 N at the foot** (0.120 A pk-pk =
5.5% = 0.54 N). That is **22% of J02's belt-off breakaway** and **3.3×** the top-plate delta
already logged as operationally negligible. **Real, worth recording, not a blocker.**

**h2 is NOT significant — the pulley is round.** Ovality is excluded.

**h18 (2/motor-rev) is the largest single harmonic at 0.086 A**, antiphase, consistent with the
two-point contact signature already recorded in the J02 top-plate diagnostic. **h9 is *not*
cleanly antiphase** (130–144°), consistent with a sensor/INL angle contribution rather than pure
load — which is why fitting all four orders together mattered.

#### ✅ RESOLVED 2026-09-05 by belt swap: h1 is on the OUTPUT SIDE, not the belt

The frequency route was arithmetically impossible (below), so the discriminator was run
instead: **three captures on belt 105.0, three on belt 104.7**, everything else identical.

| Order | Belt 105.0 (n=3) | Belt 104.7 (n=3) | Change | p |
|---|---|---|---|---|
| **h1** — 1/output rev | 0.0655 ± 0.0053 | 0.0621 ± 0.0034 | −5.1% | **0.42** |
| **h2** — 2/output rev | 0.0055 ± 0.0027 | **0.0323 ± 0.0025** | **+490%** | **0.0002** |
| h9 — 1/motor rev | 0.0674 ± 0.0029 | 0.0594 ± 0.0065 | −11.9% | 0.15 |
| h18 — 2/motor rev | 0.0862 ± 0.0004 | 0.0820 ± 0.0029 | −4.9% | 0.13 |

**h1 did not follow the belt. h2 did, dramatically.**

> **Why this null carries weight: h2 is an internal positive control.** A null is normally
> weak because "no change" and "the test cannot see change" look identical. **Here they do
> not** — the same three captures, the same fit, the same window detected a **5.9× change at
> h2 (d = 10.3)**. The method demonstrably has the sensitivity to see a belt-borne effect, and
> at h1 it saw nothing. Power check: pooled sd 0.0045, n = 3 each, so a belt-borne contribution
> larger than **~0.013 A (about 20% of the observed h1)** is excluded.
>
> **h1 also survived a full teardown, belt swap and reassembly.** A belt-borne defect would
> not have.

**Conclusion: h1 is fixed to the output side — pulley eccentricity or output-shaft bearing.**
Still to separate those two, and the pulley is now bonded, so the indexed-runout route needs
the reprint with 4 threaded holes at 90° (§22.3).

⚠ **The +6.4% mean-drag difference between the two sets is NOT evidence that the shorter belt
ran tighter.** Its p = 0.044 was computed against within-set scatter from three back-to-back
captures on one assembly, which badly understates **reassembly** variance — documented at
**±20%** for this drag, and the swap required a teardown and an idler re-mount. **Read the mean
as unchanged.** The caveat does not touch h1 or h2: reassembly moves mean drag, not the
amplitude of a specific harmonic order.

#### h2 is belt-borne — and it is a fleet QC screen

Belt 104.7 carries a 2/output-rev feature that belt 105.0 does not: **0.032 A = 1.5% of
standing load = 0.14 N at the foot.** Small, not disqualifying, but real and measurable. Either
two defects roughly opposite each other on the loop, or the belt sitting with a twist.

**Bank this: elevated h2 is a cheap defect screen.** Not worth running on all 20 belts, but
**if a joint later shows an unexplained 2/rev, the belt is the first thing to swap.**

#### ⚠ Why the frequency route was never available

```
output revolution = 9.000 motor revolutions
belt revolution   = 232 / 24.000 = 9.667 motor revolutions
                    -> only 7.4% apart
```

Separating them needs frequency resolution better than 7.4%, i.e. ~14 cycles of the slower one
= **135 motor revolutions ≈ 8.4 s ≈ 12,800 samples. `LOG_N` is 1000.** **Not resolvable with
the current capture buffer, at any speed** — see §12.

**The idler is excluded by frequency:** an 8.99 mm roller turns once per **1.177** motor
revolutions, nowhere near h1.

> ✅ **The discriminator was run on 2026-09-05, in the window before the pulley was bonded, and
> it answered — see above.** Belt thickness variation is excluded for h1 and confirmed for h2.
> **What remains for h1 is pulley eccentricity vs output-shaft bearing**, and separating those
> needs the indexed-runout test, which now requires the pulley reprint (§22.3).

### 9a. A3 — the 1/rev disturbance localised, and the motor swap cancelled (2026-08-13)

**Configuration: J02, belt OFF, pinion OFF the shaft** — the level-0 state, which isolates motor bearings plus rotor/magnet eccentricity from everything else. Two `L` captures folded against unwrapped `cnt` into 32 bins.

| Quantity | Capture 1 (Uq 1.30 V) | Capture 2 (Uq 0.60 V) | A1 reference (belt ON) |
|---|---|---|---|
| Mean velocity | 70.34 rad/s | 31.45 rad/s | 22.55 rad/s |
| Revolutions / samples per rev | 6.19 / 162 | 2.77 / 361 | — |
| Mean `Iq` | 0.1711 A | 0.1290 A | 0.81 A |
| **1/rev amplitude** | **0.0262 A** | **0.0097 A** | **0.185 A** |
| 2/rev | 0.0136 A | 0.0022 A | — |
| **7/rev** (sense offset) | **0.0311 A** | **0.0385 A** | 0.042 A |
| 14/rev (sense gain) | 0.0250 A | 0.0217 A | 0.070 A |
| Harmonic noise floor (est.) | 0.005 A | 0.003 A | — |
| Consecutive-revolution `r` | 0.83–0.85 | 0.92 | 0.98 |
| **`corr(Iq, vel)`** | **−0.042** | **−0.103** | **−0.78** |
| Folded 1/rev velocity ripple | 0.193 rad/s (**0.3%**) | 0.155 rad/s (**0.5%**) | 1.80 rad/s (**8%**) |

**Verdict: the 1/rev source is not motor-internal on J02.** Three independent lines agree, which is why this closes rather than merely reduces:

1. **Amplitude is 7–19× smaller.** Referred to the foot at 4.295 N/A (still carrying the unverified η and unmeasured `i_scale`): **0.042–0.113 N = 0.4–1.1% of a standing leg load.** That is ~8× below the current-mode torque-ripple floor and ~3× below the drag asymmetry. It is not a force the impedance controller will ever express.
2. **The velocity ripple collapsed from 8% to 0.3–0.5%.** A real once-per-revolution load modulates speed; this one does not.
3. **`corr(Iq, vel)` went from −0.78 to −0.042.** That correlation was the original evidence the disturbance was *load* rather than measurement. It is gone.

**7/rev now dominates** — that is current-sense channel offset (§7), a measurement artefact, not a mechanical one. The remaining rev-to-rev `r` of 0.83–0.85 is consistent with a fixed-in-position electrical pattern rather than a mechanical defect.

**Decisions this closes:**

| Decision | Outcome |
|---|---|
| Motor swap (60 RMB + one session) | ❌ **Cancelled.** There is nothing to swap for |
| "Whine = bearing" | Already retracted (§5); this removes the last quantitative support for it |
| Position-indexed feedforward table | **Not needed for 1/rev.** The original rule — swap before building one, or the table encodes the defect — is moot |
| A1's 0.185 A | **Reclassified as rubbing-magnet contamination**, joining `drag_c` = 1.05 A and the 4.5 N transparency figure |

**Free cross-check taken from the same captures:** J02 reached **70.34 rad/s at Uq = 1.30 V against J01's 71.02 rad/s — 1% agreement.** Two independently built assemblies at the same commanded voltage landing within 1% is fleet consistency measured rather than assumed, and it cost nothing.

**Method note worth keeping.** The plan had been to fold the *archived* J01 autocalib CSV instead of running a capture. That archive holds **INL angle bins, not a current fold** — the quantity was never in it. **Check what an archive actually contains before planning a session around it**, which is the reverse of the earlier failure of planning measurements for quantities already recorded.

---