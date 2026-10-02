# Belt drive — build routine, measured belt-on states, idler design

§22 the belt-on routine for J01 · §22.1 the three measured plant states
(2026-08-12) · §22.2 the idler rebuild specification · §22.3 tension as a fleet
problem · §22.4 J02 pulley seating and idler position (closure) · §22.5 J01 output-pulley
compensation −0.10 → −0.12, recipe A (superseded history) · **§22.6 J01 pulley recipe B —
the pulley of record (closure).**

**The belt-off baselines this file differences against cannot be recovered after
B0.** Archive them before touching a tensioner.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 22. THE BELT-ON ROUTINE FOR J01

Everything below is a **difference against the belt-off baseline**, which is why the baseline has to be complete and saved before B0. Fitting the belt destroys it and it cannot be recovered afterwards.

**Prerequisites, all belt-off:** M2 · M4 · M6a · `docs/cal/J01_2026-08-07.csv` saved.

> ### ⚠ B0 WAS RUN TWICE ON 2026-08-12 AND ROLLED BACK. NEITHER RUN COUNTS AS A BASELINE.
>
> The 10 mm belt was fitted, characterised, then characterised again with the idlers removed. **Both runs are diagnostic, not baseline** — the idlers are faulty (§22.2) and the no-idler state has **1.04 mm** of slack (corrected 2026-09-02, §22.2) and will never exist on the robot. Three consequences:
>
> 1. **Neither belt-on row goes into `joint_cal.h`.** Archive both with a header saying what plant they describe. `Ke` and the drag map are contaminated (§22.1).
> 2. **The ladder restarts at B0 after the idler rebuild.** Everything from B1 down is a property of the *contact*, so it all expires with the idler change.
> 3. **The belt-off baseline survived intact**, which is the whole reason these two runs are interpretable. `R_eff` moved −0.47%, `L` +2.1%, parity errors 0/4000.
>
> **B0 cannot start until the idler rebuild and plate revision (§22.2) are done.** Whether that is blocked, and on what, lives in the task board in [`README.md`](../README.md) — not here.
>
> **Numbering note:** the session that produced §22.1 called these steps S1–S7. They are the same steps as B1–B7 here, and **B-numbering is authoritative** — `S`-numbers now belong to the CAN ladder (§23) only. The mapping is S1→B1, S2→B2, S3→B3, S4→§9a, S5→B4, S6a/S6b→B6a/B6b, S7→B7.

| # | Step | Time | Gate / what it produces | Why here |
|---|---|---|---|---|
| **B0** | **Rebuild the idlers per §22.2, THEN fit the 10 mm GT2 belt. Output pulley BARE — leg links NOT attached.** Set centre distance, note it. Belt 232 mm, C = 44.5 mm, **take-up budget 1.04 mm** (corrected 2026-09-02) | — | a plant that will still exist on the robot | One plant change at a time. Leg gravity torque would swamp every friction number below, position-dependently. **⚠ Do not run B1–B7 on the current sliding idlers again — that work has been done twice and is not reusable** |
| **B1** | Press **`V`** | 1 min | ZEA within 8° of 6.0542 | **Belt preload can move the magnet mount.** If this fails, nothing else in the list is valid. **Measured 2026-08-12: +2.88° with the faulty idlers, +0.15° without — the shift was friction-biased alignment, not a moved mount (§22.1). A pass here is necessary and not sufficient** |
| **B2** | **`1`** then **`3`** | 7 min | `R_eff` unchanged | Electrical — a change means a wiring fault from the rebuild. Cheap tripwire before spending 50 s per test |
| **B3** | **M5 — phase 5, belt on.** Then once more with the **direction order swapped** (firmware: `-` then `5`) | 3 min | belt drag = M5 − belt-off. **And the asymmetry discriminator** (§8.2) | Belt drag is the headline transparency number. **Target after the rebuild, J01: `drag_c` 0.30–0.40 A** = 1.36–1.81 N/leg = **13.8–18.5%** of standing load — **J02 does NOT share this band — it projects 0.41–0.50 A; the single statement of that is the cross-joint note in §22.2, which also gives the reason. Do not restate it here.** *(Percentages restated 2026-08-20 for `i_scale`; **the amp targets are unchanged and are the real gate** — amps are what is measured.)* ⚠ **`Ke` from this phase is drag-contaminated — never carry it into `joint_cal.h`** (§22.1) |
| **B4** | **M4b — breakaway, belt on** (`B`/`b`), 5 positions × 2 directions | 20 min | **the number that decides impedance-control quality** | Belt-off is 0.2923 A reported / **13.5%** (`i_scale`-corrected 2026-08-20; it read 12.95% before). ⚠ **The amps are the measurement and the percentages are not** — a belt-on figure goes through the same `Kt` and the same `i_scale`, so it will land ~5% higher than the same run would have read before 2026-08-20. **Judge the bands below on amps, or restate them; do not compare a corrected percentage against a band set before the correction.** **Predicted belt-on 40–60% of standing load.** Bands: **<30% good · 30–45% acceptable, belt dominates transparency · >45% = A1's number without a rub, i.e. a tension or idler problem to fix before the legs go on.** A1's 1.05 A is contaminated and is *not* the comparison. **Procedure:** output **free** (not clamped); between presses rotate ~40° by hand and **let the rotor settle into a cogging detent** — a rotor left mid-creep reads far low in the opposite direction. The firmware flags `travel` > 200 counts (reading biased high), > 0.60 A (outlier) and `NO MOTION` at 0.80 A. **→ Measured J01, −0.12 pulley, 2026-09-28: mean 0.288 A, range 0.085–0.49 A — §22.5.** The mean equals phase 5's `drag_c`; the "predicted 40–60%" band was set against a breakaway expected to sit well above drag_c, which did not happen. **→ J01 recipe B, 2026-09-30: mean 0.295 A (n = 20, 10 × 2), range 0.080–0.475 — §22.6.4.** **For A/B comparisons (added 2026-09-30): take the readings at the SAME raw positions as the reference session and compare pairwise** — position sd ≈ 0.095 A otherwise swamps the difference being tested. Compare breakaway with breakaway, never with phase 5's `drag_c` (R22) |
| **B5** | **M7 — phases 5+6 again** | 3 min | INL after belt tension. **⚠ Phase 6's `T/T_loop` is invalid belt-on — read INL only** | ~~Belt side-load moves the magnet off-axis; the 1/rev term will grow~~ — **NOT SUPPORTED: at 1.04 mm slack the 1/rev term fell 3.3% and even-part `r` = 0.9986 (§7).** Re-run at real tension to close it, but expect confirmation rather than a finding |
| **B6a** | **Backlash / dead zone.** ~~Output **rigidly clamped**, `c` mode. Step the target `0 → +0.30 A` in 0.05 A steps, back to 0, then `−0.30 A`, back to 0. Record `cnt` at each step and plot against commanded current~~ **→ Procedure superseded: key `w` (swing ladder), output locked (2 × M3×10 countersunk into the top plate, perpendicular), §22.4.4. Pass rule = friction-corrected G ≤ 20 counts, ladders at one clamp setting.** *(A 3-position mean was adopted and reversed on 2026-09-28, R17.)* **J01 recipe B: G = 5.8 counts, PASS (§22.6.3).** *(Recipe A read 31–48 under a gate exception, closed 2026-09-30.)* | 15 min per ladder | **the tension acceptance criterion — see §22.3** | **≈0 counts of free play = preload sufficient. ~68 counts = both spans going slack near zero torque** (GT2 tooth backlash ~0.1 mm at r_pinion 3.82 mm = 0.026 rad = 68 counts). 10–40 = marginal. **This replaces the pluck method and it replaces B9.** → *Readout superseded by the friction-corrected ladder intercept, §22.4.4. The 68-count backlash figure does not describe a seated slicer-compensated pulley (§22.4.5).* |
| **B6b** | **M15 — belt stiffness.** Output **rigidly clamped**. ~~Exactly the M6a keystrokes in `c` mode: `c` → `g` → `k` → 1 s → `x` → `d`~~ **Corrected 2026-09-28 — voltage mode, per the §22.3 ring box:** `t` → `g` → `k` (positive step) or `K` (negative step) → wait for `CAPTURE done` → `x` → `d`. **4 bursts with `k` + 4 with `K`** (the D1 method). Fit the damped frequency `f_d` from the `cnt` column against `t_us` (cycle count or damped-sinusoid fit), then `f_n = f_d/√(1−ζ²)`. Record the boot each burst came from — boot-to-boot scatter is ~8× within-boot | 15 min per joint | `k_belt`, and the resonance to stay below | **High leverage:** belt compliance dominates the sim-to-real gap. Needs `J_rotor` from M6a. ~~**Predicted `f_n` 100–230 Hz**~~ **Superseded: measured 59–72 Hz (§22.3), amplitude-dependent. J01 −0.12: f_d 64.3 ± 0.4 Hz, f_n ≈ 69.5 Hz (§22.5, recipe A); recipe B f_d 65.0 Hz, f_n 69.8 Hz (§22.6.5).** Voltage, not current, mode: 2.8× larger kick and independent of current-loop tuning (§22.3). ⚠ Use the same mode on every joint being compared |
| **B7** | **M6b — `J_total`, belt on.** Output pulley bare and **unclamped**, shaft free. `t` → `g` → `k` → 1 s → `x` → `d`. *Mode confirmed 2026-09-28: `t` is what produced `J_rotor` (CONSTANTS §8.2, the 0.20 → 0.80 V step); CALIBRATION M6a/M6b said `c` and was corrected* | 15 min | rotor + belt + output pulley inertia | Per-plant; needs B3's drag map. **Predicted 20.5–22 × 10⁻⁶** (rotor 20.2 + the 108T pulley reflected through 9:1 ≈ 0.23e-6 + the pinion). **Friction is 40–60% of the impulse here, up from 32% belt-off, so treat it as ±20% and do not chase the difference from `J_rotor`.** Coming back below 20e-6 means the drag map is too high — the same effect that produced J02's 7.9% shortfall |
| **B8** | **M10 — hot vs cold `R_eff`.** Phase 3 cold → load hard 60 s → phase 3 | 10 min | ΔR/R. >15% means peak force sags mid-jump | Nearly free now that phase 3 takes 6 s. No thermistor needed |
| ~~**B9**~~ | ~~Preload check / low-preload dead-zone check~~ **→ MERGED INTO B6a.** What survives separately: backdrive breakaway felt at the *output* pulley, as a qualitative hand check | 5 min | — | The dead-zone sweep *is* B6a, done properly with a clamp and a count readout instead of by feel. Keeping both invited the skip described in the hazard note below |
| **B10** | **Tooth-skip threshold.** ⚠ **Method changed — this cannot be motor-driven.** Static lever at the output pulley plus a spring gauge. See the box below | 20 min | the hard force ceiling | **Do this last.** It is the only step that can damage the belt |
| **B11** | Update `joint_cal.h`: `belt = "10mm-9:1"`, all four drag fields, `breakaway_A`. **A NEW ROW, not an edit** | 10 min | git diff with a date | The belt-off row is a baseline you can never re-measure. Keep it |
| **B12** | **MIT impedance controller** on J01. `{p_des, v_des, kp, kd, τ_ff} → {p, v, τ}`. **Split 2026-10-01: B12a bare output** (contract, signs, velocity noise floor, saturation, friction band) **→ B12b defined load** (clamped output, then a balanced inertia bar). A bare output cannot test the belt mode or the lost motion: the loop closes on the motor encoder and the output pulley is ~1% of the rotor's inertia (R27). **Units decided (option A, README §15 7k):** τ in N·m at the output, η excluded; commands enter only through `tauOutCmdToIq()` | — | the frozen Tier-0 contract on real hardware | Everything above exists to make this honest |
| **B13** | **Build and characterise J02.** M1 → AUTOCALIB → M2 → M4 → belt | — | first fleet-spread data on a second board | 🔴 **Difference J02 against J02's OWN belt-off baseline, never against J01's.** The inter-joint drag spread is ~25% and real, so subtracting J01's baseline would attribute a board difference to the belt. J02's belt-off row already exists — use it |

Then, with two characterised actuators and impedance control validated: leg links on, **M14 force-per-amp** (load cell), and Jacobian characterisation.

> **Noted now so it is not rediscovered later:** the contact offset ~12 mm below the foot pin must appear **identically** in the IK, the sim model, and foot-position logging. Three places, one number.

### B10 cannot be run motor-driven — the actuator cannot reach the skip threshold

**Found 2026-08-15, before the run rather than during it.** The method "ramp torque until it
skips" assumes the motor can get there. It cannot, and not marginally.

| | |
|---|---|
| Motor torque at `current_limit = 2 A` | 2 A × `Kt` 0.026625 = **0.0533 N·m at the rotor** |
| At the output, ×9 | **0.48 N·m** |
| GT2 10 mm tooth-skip threshold | **3.4 – 5.7 N·m** |
| Shortfall | **7 – 12×** |

**The 0.48 N·m is computed from measured constants** (`Kt` = 1.5 · `Ke`, §8) and ignores
`DRIVETRAIN_ETA`, so it is the optimistic figure. **The 3.4–5.7 N·m is spec-derived and
therefore the weaker number** — but the conclusion survives a large error in it, which is
the point: raising `current_limit` far enough to close a 7–12× gap is not on the table.

**Replacement method: a static lever at the output pulley plus a spring gauge.** Output
rigidly held, known lever arm, pull until the belt skips, read the force. No firmware, no
thermal limit, and the failure is a skipped tooth rather than a stalled motor at 4× the
continuous board rating (§24.9).

> **The useful reframe:** the motor being unable to skip the belt is not an obstacle to the
> measurement, it is **a result about the drivetrain.** It means the belt cannot be damaged
> by any commanded torque the actuator can produce, so B10 stops being the dangerous step
> and becomes a characterisation of the mechanical margin.

### Two hidden failure modes in this plan

**B0 assumes the belt can be tensioned without the leg links.** If the carriage design needs the links installed to reach the tensioner, then B3, B4, B6 and B9 all get gravity torque folded in, and **no amount of averaging separates it from friction.** Check the CAD *before* starting. If the links must be on, the workaround is to measure at two rotor positions 180° apart and difference — but that costs accuracy and doubles the run, so five minutes in CAD now is worth it.

**B6a is the step most likely to be skipped and the most expensive to skip.** Preload that is too low creates a torque-dependent stiffness dead zone near zero torque — unseated teeth — which is *harder to diagnose than skip* and undermines exactly the impedance quality B12 exists to demonstrate. **If you find yourself deferring B6a because B10 looks like the real test, that is the failure mode.** It was previously B9, sitting after five other steps; it has been moved up and merged precisely because its position in the list was an invitation to drop it.

### Three physical prerequisites before B0

| # | Prerequisite | Why it blocks |
|---|---|---|
| 1 | **Idler rebuild** — bearings + plate revision (§22.2) | Every number B1–B7 produces is a property of the idler contact. Running the ladder first means running it twice |
| 2 | **An output-pulley clamp**, designed and printed | **B6a and B6b are impossible without one**, and it is far easier to design while the actuator is bare on the bench than after the legs are on. ~1 h |
| 3 | **CAD check: can the belt be tensioned with the leg links OFF?** | Five minutes in Fusion. If the tensioner is only reachable with the links installed, B3/B4/B6/B7 all get gravity torque folded in — see the hazard note above |

---

## 22.1 The three measured plant states — J01, 2026-08-12

Two belt-on AUTOCALIB runs plus the belt-off baseline give a **one-variable-at-a-time chain**, which is the only reason the terms can be separated at all. Belt 232 mm at C = 44.5 mm, output pulley bare, leg links off, **~1.04 mm** of slack in the no-idler state (corrected 2026-09-02, §22.2).

### The decomposition

| Plant state | `drag_c` (direction mean) | Belt force | Δ from previous | Per leg | % of a 9.81 N standing load |
|---|---|---|---|---|---|
| Motor alone, belt off | **0.0783 A** | 0.58 N | — | 0.353 N | **3.58%** |
| **+ belt, no idlers** | **0.2752 A** | 2.02 N | **+0.197 A** | 1.242 N | **12.7%** |
| + the sliding brass posts | **0.6605 A** | 4.83 N | **+0.385 A** | 2.982 N | **30.4%** |
| A1 historical (rubbing magnet) | 1.05 A | 7.32 N | — | 4.511 N | 46.0% |

All figures via `F = 2·G·η·Kt·I` with `Ḡ` = 87.7 /m and η = 0.92, so they carry the same unverified η and unmeasured `i_scale` as everything else (§8.2). Forward/reverse split with the posts fitted was 0.5518 / 0.7692 A.

> **Newtons and percentages carry TWO corrections, both on `B-SPI-01`/J01:** `×1.010768` (M1 2026-08-18 — `vbus_scale` 0.008358 → 0.008448, moving `Kt` 0.026626 → 0.026912) and `÷0.9621` (M2 2026-08-20 — `i_scale`, the sense under-reads by 3.9%). **Net ×1.0505.** History, so an old note can be traced: 0.336 / 1.182 / 2.838 N = 3.4 / 12.1 / 28.9% originally, then 0.340 / 1.195 / 2.869 N = 3.44 / 12.2 / 29.2% after M1.
>
> **The amperes did not move under either correction** — `drag_c` is measured in reported amps, and neither the voltage belief nor the current-sense gain moves a reported amp. **A1's row is deliberately NOT corrected**: it sits on the other board at a divider that was never measured for it, and at an `i_scale` that has never been measured for it either (see the box at the head of §8).

**The idlers are 0.385 A — 58% of total belt-on drag.** The mesh, output bearing and belt bending together are **0.197 A**, and that is now measured rather than assumed. It is the number the rebuild has to be judged against.

> **A prediction of mine that was 2–6× high, recorded because the direction of the error matters.** The capstan estimate for two sliding posts was 0.87–2.19 A; the truth is 0.385 A. The *mechanism* was right and confirmed by removal; the *magnitude* was overstated, most likely because the posts turn intermittently rather than sliding continuously. A mechanism confirmed by removal is strong evidence; a magnitude computed from an assumed friction coefficient is not.

### Prediction scorecard for the no-idler run

Five predictions were stated in numbers before the run. Four held.

| # | Quantity | Predicted | Measured | |
|---|---|---|---|---|
| 1 | ZEA vs stored 6.0542 rad | within 1.5° elec | **+0.15° elec** | ✅ 10× inside |
| 2 | `Ke` | 0.01775 ± 0.0003 | **0.017952** (+1.14%) | ⚠️ missed |
| 3 | `Id` at U = +2.00 V | 0.55 – 0.75 A | **0.6436 A** | ✅ dead centre |
| 4 | `T/T_loop` | 0.95 – 0.98 | **1.038, 1.046** | ⚠️ **missed — and this is the finding** |
| 5 | `\|I\|/Iq` | widens to 1.24–1.31 | **1.238 – 1.249** | ✅ |
| — | `drag_c` | 0.10 – 0.18 A | **0.2752 A** | ⚠️ 1.5× high |

### Finding 1 — friction-biased alignment, confirmed by removal

| State | ZEA | vs stored |
|---|---|---|
| Belt-off stored | 6.0542 rad | — |
| **No idlers** | 6.0569 rad | **+0.15°** |
| Idlers on | 6.1045 rad | **+2.88°** |

Alignment settles where the alignment torque balances **friction**, so the ZEA it returns is biased by the plant's friction state. Remove the friction and the alignment returns to the belt-off value. **The +2.88° shift was the idlers' sliding friction, not a mount that moved.**

> ⚠️ **This resolves an open provenance dispute.** The changelog entry of 2026-08-12 (§0) argued that phases 3–6 should not force a re-alignment, and flagged that the report justifying it cited a belt-on session with "2.88° elec above belt-off" that **did not exist in the repository at the time**. It exists now, and it is this run. The firmware change stands on what it always stood on — phase 3's open-loop path provably ignores the ZEA — and the 2.88° figure is now **measured, on J01, belt-on, with the faulty idlers fitted.** The `acHaveCommutation()` change is retrospectively justified by data as well as by argument. **The parity cross-check named in that entry has not been run and is still worth taking.**

### Finding 2 — `T/T_loop` is drag-contaminated, so phase 6 is BELT-OFF ONLY

| State | `drag_c` | `T/T_loop` | ZEA offset |
|---|---|---|---|
| Belt off (n = 7, two assemblies) | 0.078 A | **0.958 ± 0.015** | 0 |
| No idlers | 0.275 A | **1.042** (1.038, 1.046) | +0.15° |
| Idlers on | 0.661 A | **1.291** (1.305, 1.277) | +2.88° |

The idlers-on figure sits **5.6σ above the fleet band**, and the no-idler figure is still out of band **with essentially zero ZEA error** — so the anomaly is *not* an alignment artefact. The excess tracks drag at roughly **+0.57 per amp, linearly.**

Both runs pass the internal consistency check: the odd part scales linearly with speed (odd ratio 1.193 against a speed ratio 1.184), so the parity separation arithmetic is working. **It is the interpretation as pure transport delay that fails, not the measurement.**

**Best remaining explanation, and it is a hypothesis, not a conclusion:** phase 6 divides by `Ke·ω`, but under high drag a much larger share of the applied voltage goes into `R·Iq`, so the denominator is wrong in a drag-dependent way. That has not been verified to produce the right coefficient.

| Action | |
|---|---|
| `T_DELAY_PER_LOOP = 0.958` | **Label it belt-off in `fleet_config.h`.** It is still the right number — 0.95% torque loss at 270 rad/s stands |
| Phase 6 output | Print a warning when `CAL.belt` is not `OFF`, the same way phase 7's drag-map label was fixed (§0b2 F7) |
| B5 | Read INL from phases 5+6; **ignore the `T/T_loop` line** |

### Finding 3 — `Ke` drifts with drag, so belt-on `Ke` must never be carried

| Plant state | `Ke` | vs belt-off truth |
|---|---|---|
| Belt off | **0.017750** *(0.017941 at the corrected scale)* | — |
| No idlers (firmware fit) | 0.017952 | **+1.14%** |
| Idlers on | 0.018089 | **+1.9%** |

An offline refit of the no-idler set gives 0.018035, and subtracting the omitted cross-coupling term `ω_e·L·Id` (21.2 mV here, 32.9 mV with the idlers) brings it to 0.017907 — **leaving +0.89% unexplained.** The correction also makes the residual rms *worse* (6.18 → 8.93 mV), which says the model has more than one blind spot at elevated `Iq`.

> **Provenance flag:** 0.017952 is the firmware's own fit and 0.018035 is the offline refit of the same run. They should agree and they do not, by 0.46%. **The archived CSV settles which is which** — worth five minutes before either number is quoted again. Neither changes the conclusion.

**`Ke` error tracks drag monotonically. The belt-off value is the one to keep**, which is what `joint_cal.h` already does.

### Finding 4 — everything electrical held across the belt change

| | Belt off | No idlers | Δ |
|---|---|---|---|
| `R_eff` | 0.22110 Ω | **0.22006 Ω** | **−0.47%** |
| `L` | 43.31 µH | **44.23 µH** | +2.1% |
| `τ_e` | 195.9 µs | 201.0 µs | +2.6% |

*Both columns are at the pre-2026-08-18 `vbus_scale` of 0.008358 and are left that way: this table is a **within-session Δ**, the M1 factor ×1.010768 is common-mode across it, and every Δ in it — including `τ_e`, a ratio — is unchanged by the correction. Multiply the absolute `R_eff`, `L` and `Ke` figures by 1.010768 before comparing them to anything in `joint_cal.h`.*
| Phase-1 parity errors | 0/4000 | **0/4000** | air gap intact |
| `fitpts` | 18/48 | 18/48 | — |
| INL 1/rev | 0.3690° mech | **0.3569°** | −3.3% |
| INL 2/rev | 0.2035° | **0.1977°** | −2.9% |

An offline refit of the `L` block reproduces the firmware to **0.02% on τ and 0.00% on `L`.** The INL even-part correlation between the two runs is **r = 0.9986 across all 32 bins** — see §7 for what that does to the M7 re-measurement requirement.

**The electrical constants are plant-independent, demonstrated rather than assumed.** That is the property that made the whole three-state decomposition interpretable: only the mechanical terms moved.

### What must NOT be carried out of these runs

> **Do not paste either belt-on row into `joint_cal.h`.** The drag map describes a **1.04 mm-slack, idler-free state that will never exist on the robot**, and `Ke`/`Kt` are drag-contaminated. Archive both to `docs/cal/` with a header stating the plant state, and let B0–B7 produce the row that ships.

---

## 22.2 The idler rebuild — specification and the reasoning behind it

### What is wrong with the current design

Two brass standoff posts, ~5 mm OD, clamped between the plates by M3 **threaded** screws. Five independent defects, in rough order of cost:

| # | Defect | Consequence |
|---|---|---|
| 1 | **They slide instead of rolling.** A plain brass bore on a steel thread has high starting friction and stick-slips rather than rolling cleanly | **0.385 A = 58% of belt-on drag** (§22.1) |
| 2 | **On a thread, a spinning roller walks axially — fast.** At 99.32 rad/s motor the belt runs 379 mm/s, a 5 mm OD roller turns 24.2 rev/s, and on a 0.5 mm-pitch M3 that is **12.1 mm/s of axial walk.** The bias direction **reverses with belt direction** | Predicts direction-asymmetric drag — **measured at 32.9% Coulomb / 167.5% viscous, and it vanished on removal** (§8.2). Then it reaches end of travel and grinds |
| 3 | **The bend radius is worse than the pinion's.** GT2 thickness ~1.38 mm: `t/R` = 1.38/2.5 = **0.55** at the 5 mm idler vs 0.36 at the 12T pinion | The idler bends the belt **tighter than the pinion does, in the reverse direction, twice per belt revolution.** Hysteresis loss goes as roughly `(t/R)²` |
| 4 | **~4–5 mm of contact on a 10 mm belt** | Loads part of the belt width; uneven cord loading, uneven fatigue |
| 5 | **An off-centre normal force is a moment about the belt's long axis** | The belt walks toward the loaded side, which moves the contact, which steers it further — the reported "becomes not centred" is this feedback loop |

**Belt life is the real argument for diameter, not drag.** For 2 mm-pitch belts the minimum practical sprocket is 12T = 7.64 mm pitch diameter, and published guidance for a **back-side** idler is typically 1.5× that, so ~11 mm, with the absolute floor at the sprocket diameter itself.

| OD | vs 12T pitch dia | Life vs 5 mm (`R²`) | Life vs 5 mm (`R³`) |
|---|---|---|---|
| **5 mm (current)** | **0.65×** | 1.0 | 1.0 |
| 8 mm | 1.05× | 2.6× | 4.1× |
| **9 mm** | **1.18×** | **3.2×** | **5.8×** |
| 10 mm | 1.31× | 4.0× | 8.0× |

The current 5 mm is roughly **half the practical floor.** The fatigue exponent is somewhere between 2 and 3 depending on whose model you use, so call 5 → 9 mm worth **3–6× the belt life.**

> **Keep the two effects separate.** Rolling versus not rolling is a **25–100×** effect on drag. Diameter is a **2×** effect on a term worth about 2%. **Make it roll; get the diameter right for belt life.**

### The constraint that makes this a plate revision, not a parts swap

**The take-up budget is 1.04 mm** — belt 232 mm, exact no-idler path at C = 44.5 mm is
**230.96 mm**.

```
L = 2√(C² − (r₂−r₁)²) + r₁(π − 2α) + r₂(π + 2α),   α = arcsin((r₂−r₁)/C)
  r₁ = 3.8197 mm (12T)   r₂ = 34.3775 mm (108T)   C = 44.5 mm
  span = 32.349 mm   →   L = 230.96 mm   →   slack = 1.04 mm
```

Cross-check: the CAD sketch measures the undeflected span at **32.352 mm**, agreeing with the
closed form to **3 µm**. That independently confirms r₂ = 34.377 mm, i.e. 108T and an exact 9:1.

> ~~The take-up budget is 2.00 mm — no-idler path is 229.98 mm~~ — **RETRACTED 2026-09-02.**
> That figure came from the *approximate* belt-length formula
> `L ≈ 2C + π(r₁+r₂) + (r₂−r₁)²/C`, which reproduces 229.98 exactly. The approximation is a
> series expansion in `(r₂−r₁)/C`, valid to roughly **0.3**. **This drive runs at 0.687** —
> more than double the limit. The error is 0.97 mm: only 0.4% of belt length, but **48% of the
> take-up budget.** See §12 — an approximation error is invisible precisely because the number
> looks reasonable.

Growing the roller while leaving the screw where it is pushes the belt line outward by
`(R_new − 2.5)` and lengthens the path:

| New OD | Intrusion at a fixed screw | Extra belt path | Required extra stretch |
|---|---|---|---|
| 5 mm | 4.7 mm | 2.68 mm | — |
| 8 mm | 6.2 mm | 4.59 mm | **+1.91 mm** |
| 9 mm | 6.7 mm | 5.33 mm | **+2.65 mm** |

2.65 mm of stretch on a 232 mm belt is 1.1% strain = **110–570 N** against a working rating around 150 N. You would probably not get the belt on, and forcing it would put it at or past its limit.

```
The screw axis must move OUTWARD by (R_new - 2.5) mm:
    -> 8 mm OD : move out 1.5 mm
    -> 9 mm OD : move out 2.0 mm
```

### Two bearings or three

| | **2 × 3×9×5** | 3 × 3×8×4 |
|---|---|---|
| OD / bend radius | **9 mm / 4.5 mm** | 8 mm / 4.0 mm |
| Stack vs a 10 mm belt | **10.0 mm — exact** | 12.0 mm — 2 mm margin |
| Junctions in the belt path | **1** | 2 |
| Life vs 5 mm | **3.2–5.8×** | 2.6–4.1× |
| Computed drag | 0.0099 A | 0.0117 A |
| Screw must move out | 2.0 mm | **1.5 mm** |
| Bearings for two idlers | **4** | 6 |

**The drag difference is 0.0018 A — 0.3% of the problem being fixed. Ignore it.** Choose on stack height and junction count.

### Final specification

```
IDLER, per side (x2):

  Roller       2x bearing, 3 mm bore x 9 mm OD x 5 mm wide, ZZ shielded
               -> 9 mm OD, 10.0 mm stack, exact belt width, one junction
  Shaft        M3 partially-threaded socket screw with >= 10 mm PLAIN shank,
               or a 3 mm ground dowel pin with a nut
               *** NO THREAD IN THE BEARING BORE ***
  End washers  3 x 6 mm shim  (NOT a standard 7 mm M3 washer -- it would
               bridge onto the outer race and lock the roller, which is
               exactly the present failure)
  Clamp path   THROUGH THE INNER RACES ONLY. Two 3.2 mm ID / 4.8 mm OD tube
               spacers, one each end, sized so spacers + bearings = the
               plate gap exactly
  Screw axis   move OUTWARD 2.0 mm from its current position
  Flanges      none. The output pulley guides the belt

IF 9 mm OD does not fit the envelope:
  3x 3 x 8 x 4 ZZ, 12.0 mm stack, screw axis out 1.5 mm, two shims between

MEASURE THE BEARINGS WITH CALIPERS ON ARRIVAL. Miniature bearing designations
are inconsistent -- 3x9x4 is a more common size than 3x9x5, and two of those
give an 8 mm stack (80% belt coverage), which changes the answer to three.
```

**Acceptance test, non-negotiable:**

1. **Flick the roller — it must coast for a second or more.** If it does not, the clamp load is reaching an outer race.
2. **Pen-mark it.** Run phase 5. Confirm the mark **rotated** and the roller has **not walked axially.**

That ten-second test would have caught the present problem months ago. It is now a permanent step in B0.

### Bearings and washers — MEASURED 2026-09-02, the spec above is confirmed

| Item | Nominal | **Measured** | Consequence |
|---|---|---|---|
| Bearing OD | 9 mm | **8.99 mm** | 9 mm OD branch of the spec stands; the three-bearing fallback is not needed |
| Bearing width | 5 mm | **4.96 mm** | Two stacked = **9.92 mm** against a 10 mm belt → **0.04 mm overhang per side** |
| End washers | 3 × 6 mm shim | **conical M3, proven** | Bearings spin cleanly with the clamp at final torque |

**Conical M3 washers are adopted in place of the 3 × 6 mm shim.** They contact the inner race
only and were verified spinning free at final clamp torque on 2026-09-02. ⚠ The prohibition
above is unchanged and is the reason: **a flat Ø7 DIN 125 washer bridges onto the outer race
and locks the roller** — that is the present failure mode, not a variant of it.

**The idler provides no belt tracking.** 9.92 mm of roller under a 10 mm belt leaves 0.04 mm
per side; tracking remains a **pulley-flange** job. Do not let the idler be designed as if it
guides the belt.

### Idler mounting: FIXED M3 HOLES IN BOTH PLATES. No slot, no adjuster

**Decided 2026-09-02.** This supersedes the slot proposal that stood in §22.3.

Reasoning, in the order it should be weighted:

1. **The idler is in double shear across two plates.** Two slots must be set to the *same*
   position or the idler axis tilts, which edge-loads both bearings and voids the "coasts ≥ 1 s"
   acceptance test the whole rebuild exists to pass. **There is no practical way to guarantee
   two slots match by hand, ×24 idlers.**
2. With fixed holes the adjustment mechanism is **the printer**. A plate reprint is ~90 minutes
   and a few cents. A slot costs assembly time and an alignment risk on every joint, forever.
3. Belt-length tolerance is absorbed by **selective assembly** (§22.3), not by hardware.

> ### Withdrawn 2026-09-02 — do not re-propose
>
> | Proposal | Why withdrawn |
> |---|---|
> | Jackscrew / grub screw on a slotted idler | Sets **position, not force.** Belt-length tolerance passes straight through it. Also needs a plastic thread under sustained 55 N, which creeps |
> | Printed spring cartridge (force-setting) | Pushes on one carrier only, so it **skews a double-shear idler.** Dies with the slot |
> | M2 / M2.5 grub screw | Thread shear area halves against M3; a 0.9 mm hex key across 12 joints; M2 heat-set inserts split thin bosses |
> | Tape wrapped on the idler OD | A compliant viscoelastic layer **in series with the belt**, which corrupts the `f_n` → EA measurement the whole tension method rests on. Adds a rotating disturbance to a project that has spent multiple sessions chasing per-rev disturbances. **Masks a seized bearing instead of exposing it** |

**Grip is not why idlers fail to roll, and this is the load-bearing argument.** At T = 80 N and
a 20° wrap the idler carries 55 N. Bearing friction needs **0.037 N** of rim traction against
**~27 N** available — a **740× margin**. **A free bearing rolls. One that does not is clamped**,
not under-gripped. The sliding brass posts failed because they were *sliding surfaces*, and any
fix aimed at grip is aimed at the wrong term.

### Two build-spec items to settle at the same teardown

**Added 2026-08-15.** Neither is worth a teardown of its own, and both become expensive
once twelve joints exist. The idler rebuild opens the assembly anyway — settle them then.

| # | Item | Why it belongs in the build spec, not in the assembler's hands |
|---|---|---|
| 1 | **Shim the top-plate clearance** | This is a **wear** problem, not a friction one. 0.0196 A of drag is negligible against a 0.30–0.40 A target and would never justify the work on its own — but the **debris** it generates is not negligible, and it lands in the bearings the rest of this section exists to protect |
| 2 | **Determine whether the M4 screw axially preloads the pinion bearing**, then define a shim stack or a screw depth | 🔴 **Preload set by feel = twelve joints with twelve different drags.** The whole point of §22.3 is that tension cannot be hit by feel; the same argument applies here and for the same reason. If the screw *does* preload the race, an undocumented torque spec silently becomes a per-unit constant |

Item 2 is the one with fleet consequences: it decides whether pinion-bearing drag is a
`fleet_config.h` quantity or a `joint_cal.h` one, and that question should be answered by
the design rather than discovered from a 25% spread across twelve assembled joints.

**Prediction after the rebuild**, stated so the run can falsify it:

| Quantity | Prediction |
|---|---|
| `drag_c` | **0.30 – 0.40 A** = mesh/bearing/bending 0.197 + rolling idlers ~0.010 + belt-off 0.078 + the tension-dependent mesh increase as the 1.04 mm of slack goes away |
| Per leg | **1.29 – 1.72 N = 13 – 18% of standing load** |
| Coulomb asymmetry | **below 20%** (from 39% with the posts) |
| Reverse drag fit rms | **below 10 mA** (from 66.8 mA) |

*Revised upward from an earlier 0.15–0.35 A: that estimate forgot that removing the idlers also removed the tension.* **If drag stays above 0.4 A with a verifiably free-rolling idler, it is tension rather than the idler, and B6a becomes the priority.**

**J02 measured ⟨V8⟩: drag_c 0.340 A, asymmetry 8.0% pooled ✅, reverse rms 10–13 mA ❌ — §22.4.8.**

> ### ⚠ The band above is J01's, not fleet — restated per joint 2026-08-18, percentages again 2026-08-20
>
> J02's belt-on-no-idlers diagnostic run (top ON, §24.1's Kt cross-check) gives a second
> data point, and it does not land in the same place:
>
> | State | J01 (2026-08-12) | J02 (run A, top on) |
> | --- | --- | --- |
> | Assembled, belt off | 0.0783 A | ~0.143 A |
> | Belt on, no idlers | 0.2752 A | 0.381 A |
> | **Belt increment** | **0.197 A** | **0.238 A** — **+21% on J01's** |
>
> The belt itself reproduces cross-joint far tighter than the joints it is mounted on — a
> good sign for fleet scaling, and the one thing these diagnostic runs genuinely
> established. But it means **J02's post-rebuild `drag_c` projects to 0.41–0.50 A = 18.9–23.1%
> of standing load, not 13.8–18.5%.** J02 is already at 17.6% (0.381 A) with no idlers fitted
> and a slack belt — this is the **36% J01/J02 drag offset (§22.1) plus a slightly larger
> belt increment**, not an idler failure, and the rebuild does not fix it.
>
> **Action: read the 0.30–0.40 A / 13.8–18.5% prediction above as J01's own number, not a
> fleet target — and judge the run on the AMPS.** The percentages have now been restated
> twice for calibration corrections (M1 2026-08-18, M2 2026-08-20) while the amp targets
> never moved once. That is the whole argument for gating on amps. Restate per joint as each joint's B3 run lands — do not let one joint's
> figure harden into fleet dogma, which is the failure this note exists to head off.
>
> **J02 measured ⟨V8⟩: drag_c 0.340 A, asymmetry 8.0% pooled ✅, reverse rms 10–13 mA ❌ — §22.4.8.**

---

## 22.3 Tension is a fleet problem, and it is measured by consequence

### Why the pluck method is retired

Plucking the free span and reading the pitch is ill-posed on **this** design, not badly executed:

| Span assumed | at 1500 Hz | at 3500 Hz |
|---|---|---|
| 20 mm | 23 N | 127 N |
| 30 mm | 53 N | 287 N |
| 44 mm | 113 N | 617 N |

**The idlers subdivide every span**, so "which segment am I plucking?" is unanswerable, and the span-length ambiguity alone spans a 5× tension range before the microphone contributes anything. At 1.5–3.5 kHz you are also at the top of phone-mic usefulness with a very short decay. **Retire it. It is a geometry mismatch, not a calibration error.**

### Why a target tension cannot be hit by printing accurately

```
d(tension)/d(intrusion)  =  48 to 240 N per mm      (EA 10-50 kN, 232 mm belt)
a 0.15 mm print tolerance                          =  7 to 36 N of tension
a +-0.5 mm belt length tolerance                   =  22 to 108 N of tension
```

**The printed parts are not the dominant variable — the belt is.** A rigid tensioner with a **1.04 mm** take-up budget amplifies an ordinary ±0.5 mm closed-loop belt tolerance into tens of newtons on a tension nobody can measure. **Twelve joints built from identical parts, with belts from the same bag, could still span a wide tension range.**

This is **not** an argument for adding a tensioner. It is an argument that the *consequence* has to be measured.

**J02's hole position was accepted by measurement at the existing y = 10.00 — §22.4.2 / §22.4.6.**

### What replaces it: B6a and B6b become fleet QC on every joint

| | Measures | Pass condition |
|---|---|---|
| **B6a** backlash | whether both spans stay taut through zero torque | ~~**≈0 counts** of free play. ~68 counts = slack. 10–40 = marginal~~ → friction-corrected ladder G ≤ 20 counts, §22.4.4 |
| **B6b** resonance | drivetrain stiffness, and the mode the impedance loop must stay under | ~~**`f_n` 100–230 Hz**~~ → measured 59–72 Hz (below); no pass band is set. Its fleet use is the J01-vs-J02 comparison |

> **Superseded 2026-09-05 / annotated 2026-09-28.** The 100–230 Hz derivation and the result
> table below assumed EA = 10–50 kN acting on a linear belt. The measurement came in at 59–72 Hz
> and amplitude-dependent ("Ring test — RESULTS", below), and the "< 80 Hz → stiffer clamp"
> branch was exercised and the fixture passed. Kept for provenance; **do not gate on it.**

**B6b derivation and the correction to an earlier estimate:**

```
k_motor = k_belt * r_pinion^2 ,   r_pinion = 3.82 mm
k_belt  = 2 * EA / L_span ,       L_span ~ 35 mm ,  EA = 10-50 kN
        -> k_motor = 8.3 .. 41.7 N.m/rad
f_n     = sqrt(k_motor / J_rotor) / 2pi ,  J_rotor = 20.2e-6
        -> 100 .. 230 Hz
```

*An earlier prediction of 40–55 Hz came from a `k_belt` estimate of 80–160 kN/m; recomputing from EA gives 570–2860 kN/m, so that figure was low by roughly 5×.* Expect 7.5–17 cycles in the 75 ms window at 58–134 samples per cycle, ring amplitude 3–17 counts static.

| Result | Meaning |
|---|---|
| **100–230 Hz** | ✅ In band. **Keep the impedance loop bandwidth below ~`f_n`/3** so it does not excite the mode |
| **< 80 Hz** | Either the belt is slacker than expected **or the clamp is the spring.** Repeat with a deliberately stiffer clamp — if the frequency moves, you measured the clamp |
| > 250 Hz | Very stiff, and above any bandwidth you would want |

**Measure a frequency, not a displacement.** That is why this works despite static deflection being only 3–17 counts.

### Consequences for the fleet

**B6a and B6b run on every joint, not just J01.** They are the only thing standing between the current design and twelve differently-tensioned actuators. The earlier plan to give J02 a reduced run (slope only) is withdrawn.

| Step | J01 | J02 | Why |
|---|---|---|---|
| B1 `V` | full | **full** | Non-negotiable gate |
| B2 phases 1, 3 | full | **full** | 10 min, catches a closed air gap |
| B3 belt drag | full | **full** | Per-assembly |
| B4 breakaway | full | **full** | Per-assembly |
| **B6a, B6b** | full | **full** | **Tension QC — the whole point** |
| B7 `J_total` | full | **skip** | Fleet quantity, drag-dominated; a second one carries no information |

Roughly 2 h for J01 and 1 h 45 for J02. **The only saving is B7.**

**The comparison that decides whether the no-tensioner design survives:**

| | Pass condition |
|---|---|
| **B6b ring frequency, J01 vs J02** | **within 15%** — this is the tension-repeatability answer |
| B6a backlash | both ≈ 0 counts |
| B3 belt drag | within 30% (the plant term is itself ±20%) |
| B4 breakaway | within 30% |

**If B6b agrees within 15%, the fixed-geometry design is vindicated** and the ±0.5 mm belt-tolerance concern is bounded by measurement rather than argument. If it does not, that is found on joint two of twelve rather than joint twelve.

> **2026-09-28 — this comparison is blocked, not failed.** J01's side exists (§22.5: f_d 64.3 ±
> 0.4 Hz, 4 + 4 bursts). J02's B6b and B4 do not exist in comparable form: its only rings are
> P8's 2 bursts (§22.4.7), and its ESC board failed after P8. Deferred as **D15**. ⚠ **J01
> and J02 now run different pulley recipes** (J01 −0.12 + precise wall, J02 SCB −0.10), so
> a pass or fail is no longer purely a tension-repeatability answer. Record both recipes with
> the comparison.

### ~~Since the plate has to be revised anyway: slot one idler hole~~ — WITHDRAWN 2026-09-02

> ~~**A — slot one idler hole** ✅ recommended. B6a supplies the setting criterion: back it off
> until backlash appears, then advance until it just disappears, then a little more.~~
>
> **WITHDRAWN. The decision is FIXED M3 HOLES IN BOTH PLATES — §22.2 carries it and the
> reasoning.** The defect in this proposal is that it treated the idler as if it were mounted
> on one plate. **It is in double shear across two**, so a slot must be set identically on both
> or the idler axis tilts and edge-loads the bearings — voiding the very acceptance test the
> rebuild exists to pass. Belt-length tolerance moves to **selective assembly** (below).
>
> ⚠ **Option B is also withdrawn, and its own note said why:** a 230 mm belt against an exact
> no-idler path of **230.96 mm** (corrected, §22.2) is not "just fits" — **it is shorter than
> the path.** At 229.98 mm the option looked marginal; at the correct figure it is impossible.
> A case of an arithmetic error making a bad option look merely risky.

### Belt length variance — measured 2026-08-31, n = 10, same order

**The "same batch → minimal variation" assumption is falsified.**

**Method.** Two printed ⌀10 mm shafts clipped to the caliper jaws, belt looped over both,
**1.475 kg** hung from the lower jaw, caliper read outside-to-outside. Vertical, so gravity
sets the force — no hand pressure.

```
L = 2C + π·d_eff   and   C = R − d_shaft
  → L = 2R + (π·d_eff − 2·d_shaft)
         ╰─── constant, identical for every belt ───╯
  → ΔL = 2 × ΔR
```

**Absolute length is never needed.** Shaft diameter, belt neutral-axis offset, elastic stretch
under the 7.35 N/span, and any shaft-angle cosine error are all **common-mode and cancel in the
ranking.** That is what makes a printed-shaft jig adequate for a job that looks like it needs
a length gauge.

**Results** (0.1 mm resolution): 104.7 ×1, 104.8 ×3, 104.9 ×5, 105.0 ×1.

```
mean 104.86 mm    s = 0.0843 mm    Sheppard-corrected (h = 0.1) s = 0.0792 mm
σ(L)     = 2 × 0.0792 = 0.158 mm
range(L) = 2 × 0.300  = 0.60 mm
```

**The batch spans 0.60 mm.** Inside the ±0.5 mm catalog figure, but not negligible:

```
ΔT = EA × ΔL / L,   L = 232 mm
  EA 20 kN →  52 N        EA 50 kN → 129 N
```

Against a working preload near 80 N that is **±32% to ±81%. `EA` is the dominant unknown**, and
one `f_n` measurement closes it.

**Jig repeatability:** repeatable to 0.1 mm; fresh-shaft cross-check passed; each belt measured
with both faces toward the calipers. **Known wear mode:** caliper edges creep into the PLA
shafts by a few 0.01 mm — replace the shafts periodically.

> **Protocol fixes before the 20-belt batch.** Record to **2 d.p.** — at 0.1 mm there are only
> four bins across the entire range, which is why the Sheppard correction was needed at all.
> **Randomise measurement order**: PLA creep is a monotonic drift, and measuring in sequence
> makes it correlate with belt identity — a systematic error that would look exactly like a
> real length trend. Re-read one **reference belt** at the start, every 5, and at the end.
> **Never merge readings taken across different shaft installs** without a reference belt tying
> them together.

**Calibrate the idler hole position on the LONGEST belt, not the median or the mode.** The
failure modes are asymmetric:

- **Too long → slack → unseated teeth near zero torque**, which wrecks impedance control and is
  hard to diagnose from the outside.
- **Too short → more drag and bearing load** — graceful, bounded, and visible in `drag_c`.

Setting the hole for the longest belt puts every shorter belt on the safe side.

**Satisfied on J02: belt 105.0 at y = 10.00 — §22.4.1.**

**Selective assembly fallback ladder:**

| Measured spread of the chosen 12 | Action |
|---|---|
| ≤ ±0.15 mm | one hole position, one plate variant |
| ≤ ±0.35 mm | two plate variants A/B, hole centres offset by half the spread |
| > ±0.35 mm | rigid stepped sleeve on the idler OD (symmetric, **cannot tilt** — unlike a slot) |
| all of the above fail | slot both plates with a **shared spacer block** forcing both to one position |

→ **Belt-length sensitivity table: §22.4.6.** The batch range of 0.60 mm of loop = 0.77 mm of
equivalent y, and y = 10.00 is predicted to hold every measured belt taut — a prediction, not
yet measured on a shorter belt (D4).

### What replaces the pluck test: two keystroke procedures

~~Both are **assembled from existing keys — neither is a named firmware command.** Nothing in
`open_test.cpp` needs changing to run them.~~ **Corrected 2026-09-28:** the swing test is now
a firmware command, **`w`** (acSwingLadder, §22.4.4), because the ~90 keystrokes of a
three-current manual version kept producing mis-counted jogs. The ring test is still assembled
from existing keys (`t`, `g`, `k`/`K`, `x`, `d`).

> ### Swing test — the SEATING test
>
> Output pulley locked. `c` → `g` → `+`×15 (to +1.5 A), read `cnt` from the periodic telemetry
> line → `-`×30 (to −1.5 A), read `cnt` again → `+`×15 to confirm repeatability.
> **Swing = |C⁺ − C⁻|**, in motor encoder counts.
>
> → *Keystroke procedure superseded by `w` (acSwingLadder), §22.4.4.*
>
> ⚠ **`a` (logStats) does NOT report encoder counts.** It prints `Uq`, `Ud`, `Iq`, `Id`, `vel`,
> `|I|` and `ratio` only. Use the live **`cnt=`** field on the periodic line. No CSV needed.
>
> Scale: 16384 counts/motor-rev, pinion pitch circumference 24.000 mm →
> **1 count = 1.46 µm of belt travel.**
>
> | Swing | Belt free play | Reading |
> |---|---|---|
> | ~~> 200 counts~~ | ~~> 0.3 mm~~ | ~~genuinely slack~~ |
> | ~~\~70~~ | ~~0.10 mm~~ | ~~one tooth clearance~~ |
> | ~~10–40~~ | ~~0.015–0.06 mm~~ | ~~marginal~~ |
> | ~~5–12, not shrinking further~~ | ~~elastic only~~ | ~~**seated → P_seat**~~ |
>
> **SUPERSEDED 2026-09-24 → §22.4.4 (R13).** A single-current swing mixes slack, lost motion
> and elastic stretch; the readout is now the friction-corrected ladder intercept.
>
> ### ✅ RUN 2026-09-05 — 835 counts, and the corrected arithmetic is confirmed
>
> Belt 105.0, idlers out, ±2.0 A (`CURR_MAX`), read 5 s after each limit.
>
> | | + limit | − limit | swing |
> |---|---|---|---|
> | run 1 / run 3 | 15591 / 15590 | — | |
> | run 2 / run 4 | — | 14755 / 14756 | |
> | | | | **835 counts, reproducible to ±1** |
>
> | Model | Predicted | vs measured 835 |
> |---|---|---|
> | **Corrected, path 230.96 mm** | ~980 | **−15%** |
> | ~~Retracted, path 229.98 mm~~ | ~1570 | **−47%** |
>
> **The corrected closed form stands; the approximate formula is excluded by nearly 2×.**
> The residual 15% says the batch runs ~0.2 mm shorter than the **232.00 nominal**, which was
> always an assumption rather than a measurement — the one remaining soft input.
>
> ```
> 835 ct x 1.4648 um/ct = 1.223 mm total free travel
>   - elastic at +-2.0 A (2 x 38.6, at the measured stiffness)  =  77 ct
>   - tooth clearance                                           =  0 .. 68 ct
>   -> slack = 690 .. 758 ct = 1.01 .. 1.11 mm
> ```
>
> **Inverting makes the drivetrain an absolute length gauge:** belt 105.0 =
> **231.97 .. 232.07 mm**, batch mean (104.86) = **231.69 .. 231.79 mm**. The caliper jig and
> the encoder now agree on a belt-length scale — two fully independent instruments.
>
> ⚠ **The inversion assumed the STD pulley seated at pitch. Uncompensated prints ride
> 0.1–0.3 mm high (§22.4.3), so 231.97–232.07 mm carries an unquantified bias. The
> belt-difference test (D7) remains the robust check.**
>
> ⚠ **Still outstanding, and it is the stronger test:** the swing **difference** between two
> belts depends on neither the 232 nominal nor the 230.96 path. Predicted
> `0.60 mm / 24.000 mm x 16384 = 410 counts` between belts 105.0 and 104.7. Agreement within
> ~30 counts validates the caliper jig as a fleet tool. **Not yet run.**
>
> ### Tooth skip: `I_skip` > 2.0 A on this plant
>
> → *Plant- and procedure-specific: the SCB no-idler plant ratcheted at 1.6 A under stepped
> reversal — §22.4.9.*
>
> Same run. At ±2.0 A — 0.053 N·m at the pinion, **13.9 N of belt force** — the belt **did not
> skip**; position settled and held at both limits, twice each, to one count. **The motor
> cannot reach the skip threshold even with the belt slack.** An earlier hand observation of
> skipping simply exceeded 2.0 A-equivalent, which is easy: 13.9 N is a firm push at the
> output rim. B10's static-lever method stands, and the 3.1-teeth-in-mesh concern still
> applies to the *tensioned* case.
>
> ⚠ **Protocol: read the count 5 s after the limit.** ~10 counts of creep (15 µm of belt)
> appear over the first ~4 s and then converge. That is not skip.

> ### Ring test — the STIFFNESS test
>
> Output pulley locked. `x` → `t` (TORQUE(V)) → `g` → `k` → **`x` as soon as `CAPTURE done`
> appears** (`target` stays at 0.80 V = 2.77 A stalled after the capture) → `d`. Count cycles
> in the `cnt` column against `t_us`: **`f_d = N / (t_last − t_first)`** — this is the
> *damped* frequency; `f_n = f_d/√(1−ζ²)`.
>
> *Corrected 2026-09-28:* this box said "`x` immediately", which contradicted the "press `x`
> only after `CAPTURE done`" finding further down this section. Pressing `x` before the
> capture completes truncates it. It also labelled the cycle count `f_n`; at the ζ of 0.3–0.7
> measured since, `f_d` is 5–30% below `f_n`. **`K` steps the kick negative** (added
> 2026-09-05; `k` is positive). A B6b run is 4 × `k` + 4 × `K`, and every dump carries
> `cap=N` in its header — `!! RE-DUMP` means the same capture was dumped twice.
>
> **Voltage mode, not current mode.** 0.20 → 0.80 V is 2.77 A stalled against 0.2168 Ω — a
> **2.8× larger kick** than current mode's 0.5 → 1.5 A, giving 9–46 counts of ring instead of a
> marginal 2–8. A `Uq` step is also an excitation **independent of current-loop tuning**, so the
> result does not move when the gains do.
>
> **Cycle counting beats FFT here.** The capture is ~65–80 ms, so FFT bins are 12.5 Hz — 5–12%
> at these frequencies — while counting 8–15 cycles against per-sample timestamps gives 1–2%.
> (The burst logger stores a real per-sample `dt_us`, which is what makes this work.)
>
> ```
> EA ≈ 0.96 × f_n²  newtons        (f_n in Hz)
>   from  k_belt = k_motor / r_pinion²,  k_motor = (2πf_n)²·J_rotor
>   r_pinion = 3.82 mm,  J_rotor = 20.2 µkg·m²,  L_span = 35 mm
> ```
>
> ~~Expected band 100–230 Hz. `f_n` < 80 Hz → repeat with a stiffer clamp.~~
> **SUPERSEDED 2026-09-05 — measured 59–72 Hz, and `f_n` is not a single number.** See the
> box below before running this again. The 80 Hz gate did fire, the fixture check was run,
> and **the fixture passed** — the low frequency is the drive, not the clamp.

#### ⚠ Fixture stiffness is a first-class requirement, not a detail

A compliant lock is a spring **in series** with the belt:

```
f_n(measured) = f_n(true) × √(1 / (1 + k_belt/k_fix))
  <5% error → k_fix >= 10 x k_belt        <3% error → k_fix >= 16 x k_belt
```

⚠ **The requirement is an order of magnitude lower than first quoted, because measured
`k_belt` came in far below the assumed EA.** ~~k_belt at the output = 672 .. 3377 N.m/rad →
requirement 6,700 .. 33,800~~ — **RETRACTED 2026-09-05.** Measured `k_motor` is
**2.8 .. 6.0 N·m/rad** (amplitude-dependent, see below), so at the output that is
**227 .. 486 N·m/rad** and the requirement is **≈2,300 .. 4,900 N·m/rad.** A CA rim bond at
r ≈ 34 mm plausibly clears that by an order of magnitude — **and it was then confirmed
empirically, which is what actually settles it.**

**Torsional stiffness scales with the SQUARE of the radius you react at.** Gripping the 3 mm
shaft rather than the 34.4 mm rim is a **131× penalty in the same material.** Worse, a shaft
clamp puts the pulley web in the load path; **a rim bond short-circuits it** — torque enters at
the pitch radius and leaves at the same radius.

| Fixture | Extra swing contributed at ±1.5 A |
|---|---|
| Shaft clamp (~19 N·m/rad, "a few degrees") | **442 counts** — larger than the 790-count slack signal it is meant to measure |
| Rim bond (~50,000 N·m/rad, **estimated, ±3×**) | **0.17 counts** |

**Fixture acceptance test — run it, do not trust the estimate.** Measure `f_n`, add stiffness
(one more bond spot, or a thin steel strap), re-measure. **< 3% shift = the fixture is out of
the loop. > 10% = you are still measuring the fixture** — escalate to the second pulley face
then, and only then.

> ### Bonding the pulley for the fixture: ONE side, at maximum radius
>
> **Gel CA, non-toothed face, largest radius available, off the tooth band. 3–4 spots.**
>
> Reasons to stop at one side: rim bonding already wins on **load path, not adhesive area**, so
> a second face doubles area without shortening the path; access and reversibility; and the
> tilting couple from single-sided bonding is small — **0.66 N·m over ~5 mm of offset** — which
> the output bearings take.
>
> If there is an air gap between the pulley face and the plate, **bond small PLA blocks bridging
> it rather than building a thick fillet. A thick bondline is a spring**, which is the one thing
> this fixture must not contain.
>
> **Do not glue if the indexed runout test is still wanted** — it needs the pulley *clamped and
> indexable*, and glue makes those mutually exclusive. **On the next pulley reprint, design 4
> threaded holes at 90° into the pulley face** (r ≈ 28–30 mm, heat-set M3) with one matching
> clearance hole in the top plate. That makes the indexed test free later, gives a stiffer
> fixture than glue, and needs no adhesive at all.

### 🔴 Ring test — RESULTS, and why this plant is now retired from it (2026-09-05)

**Twelve captures across four sessions. The headline is not a stiffness number — it is that
there isn't one.**

> #### The finding: `f_n` is set by ring AMPLITUDE, not by the fixture
>
> ```
> f_d = 86.9 - 0.522 x A          r = -0.982        (A = ring amplitude, counts)
> ```
>
> | | sd of f_d |
> |---|---|
> | Raw, all 12 captures | **4.86 Hz** |
> | After removing amplitude | **0.91 Hz** |
>
> **Ring amplitude explains 96.5% of the variance.** The drive is a **softening spring** — the
> larger the oscillation, the lower the frequency.
>
> | Ring amplitude | Belt travel at pinion | `f_d` | `k_motor` |
> |---|---|---|---|
> | 54 counts | 79 µm | 59 Hz | 2.8 N·m/rad |
> | 29 counts | 42 µm | 72 Hz | 4.2 |
> | → 0 **(extrapolated)** | — | **≈87 Hz** | **6.0** |
>
> ⚠ **The zero-amplitude intercept is an extrapolation past the data. Indicative, not
> measured.**
>
> **This is the signature of tooth engagement at 3.1 teeth in mesh.** Cord stretch is linear;
> tooth contact climbing under load is not. Second independent line of evidence for the
> pinion-wrap concern.

> #### ✅ Fixture check: PASS — and the bond was never in the load path
>
> Residuals of the amplitude fit, by bond count — **this is the actual fixture test**, because
> it compares sets at *matched* amplitude:
>
> | Set | mean residual |
> |---|---|
> | 3 spots | +0.39 Hz |
> | 5 spots | −0.17 Hz |
> | 7 spots | +0.45 Hz |
> | 7 spots, repeat | −0.66 Hz |
>
> Spread 1.11 Hz against a residual sd of 0.91 Hz — **indistinguishable.** Going from 3 to 7
> bond spots changes nothing. **One side, 3 spots at maximum radius, is sufficient** (§22.2).

> #### 🔴 STRUCK: the "5–8% per session drift"
>
> That claim was measurement scatter plus the amplitude effect, **not a physical process.** No
> creep, no CA shrinkage, no progressive tooth unseating. **Delete it from any working notes.**

> #### 🔴 Boot-to-boot scatter is ~8× within-boot scatter
>
> A repeat with **nothing touched** — same hardware, 20 minutes apart, new boot — moved the
> frequency **+3.96%, "p = 0.0042"**: 59.63 ± 0.49 Hz → 61.99 ± 0.29 Hz. All six 7-spot
> captures pooled: **60.81 ± 1.34 Hz, cv 2.21%.**
>
> **Every p-value quoted on the 3 → 5 → 7 bond comparison was computed against the wrong error
> term.** Within-boot scatter understates the real measurement uncertainty by about 3×. See
> §12 — this is now a catalogued failure mode.

> #### ⚠ The ring test can never give cord stiffness — and that is why the caliper is right
>
> The ring measures **cord, teeth, hub and fixture in series**, and the series total is
> dominated by a **nonlinear element that is not the cord**. `EA ≈ 4 kN` was stated in an
> earlier session without an amplitude and is **withdrawn as a standalone number**; it is a
> lower bound at one excursion, not a property of the belt.
>
> **A length mismatch is taken up by CORD stretch, which tooth compliance does not relieve** —
> so the ring number cannot be used for the plate/hole decision at all. **The A/B plate
> question therefore stays OPEN**, not closed as an earlier session had it.

> #### ✋ STOP RING-TESTING THIS PLANT
>
> → *Applies to the STD glued plant. SCB rings are recorded in §22.4.7.*
>
> The fixture question is closed, the amplitude relation is characterised, and further captures
> here have **no decision value.** Two things were checked and excluded along the way, recorded
> so they are not re-run:
>
> - **`k` → `x` timing is not a contributor.** In all twelve captures `Uq` holds 0.800 for the
>   whole window; `Iq` drops below 1 A in exactly two, at **68.5 ms and 71.1 ms** — both well
>   past the 8–50 ms fit window. ⚠ It *would* matter if the fit window were widened. **Press
>   `x` only after `CAPTURE done` appears.**
> - **LiPo droop is real but far too small to explain it**, and it surfaced something else:
>   `Vb` fell 12.100 → 12.020 (−0.66%, monotonic) while end-of-step `Iq` fell 3.665 → 3.486
>   (**−4.8%, 7× more than the bus can account for, and a step not a trend**). Backing out
>   resistance: 0.2183 → 0.2294 Ω, **+5.1% ⇒ ≈13 °C of winding warm-up** at copper's
>   0.393%/°C. **A free thermal measurement nobody was looking for** — and a caution that
>   back-to-back captures are not thermally identical.

### Next instrument: the caliper two-weight test for cord EA

**SUPERSEDED for J02 ⟨V8⟩: hole position accepted by measurement at y = 10.00 (§22.4.2).
EA test demoted to D8.**

Off the machine, no firmware, immune to everything that contaminated the ring test.

```
EA_cord = (W2 - W1) x L / (4 x dR_true)       dR_true = dR_belt - dR_wire
```

Same belt at **1.475 kg and 5 kg**, with a **steel-wire loop as a splay reference** so jaw and
shaft deflection cancel. **Three reads per condition, randomised order** (§22.3's protocol
fixes apply — PLA creep is monotonic and will otherwise correlate with condition).

Then compute the idler depth `d`, **print three plate pairs at `d − 0.5 / d / d + 0.5`,
swing-test each, and keep the tightest that seats.** That is the decision path to the hole
position, and it does not depend on the ring number.

---

## 22.4 J02 — pulley seating and idler position: closure (2026-09-05 → 2026-09-23)

**What opened it.** On the uncompensated Arachne-printed output pulley, the existing
y = 10.00 idler holes felt over-tight, and the no-idler swing, the caliper and the belt-path
model disagreed on the slack.

**What closed it.**

1. **Root cause:** printed 108T grooves hold the belt's tension cord above pitch radius.
2. **Fix:** slicer X-Y contour compensation of −0.10 mm. Neither Fusion offset route reproduced it.
3. **Acceptance at the existing y = 10.00 holes**, on the fixed pulley with the batch's longest belt:
   B6a geometric term ≈ 0, drag_c 0.340 A, clean to 1.6 A. **No plate reprint.**
4. The §22.3 plan — EA two-weight test, then three plate pairs — is **superseded for J02**.

Session IDs used below:

| ID | Pulley | Idlers | What ran |
|---|---|---|---|
| P0 | STD (§22.3) | — | swing 835, rings |
| P1 | A2 | — | swing 373, ring |
| P2–P4 | all variants | — | caliper surveys |
| P5 | SCB | none | ramped swing, ring |
| P6 | SCB | none | ladders |
| P7 | SCB | none | phase 5 |
| P8 | SCB | y = 10 | full acceptance |

### 22.4.1 Frozen J02 configuration

| Item | Value | Source |
|---|---|---|
| Output pulley | 108T GT2, **ID SCB**: Arachne walls, **slicer X-Y contour compensation −0.10 mm** (stored in the 3MF project and the filename), CAD boss parameter **+0.20 mm**. ⚠ **Sliced on the 0.6 nozzle profile (recipe A)**, found 2026-09-30. The fleet recipe is now B (§22.6.1), so a J02 reprint is a new recipe and needs re-acceptance | §22.4.3 |
| Pulley mount | Screwed, not glued | P1 |
| Bearing seat | Printed **14.95 mm** against A's 15.10 → 0.15 mm undersize. **Shim: none fitted.** Owner reports the fit as tight with no perceptible play (hand assessment, 2026-09-24) — **not reconciled** with the 0.15 mm-undersize caliper reading; see §22.4.5 | §22.4.3 |
| Next print of this pulley | Boss parameter **+0.35 mm** (predicted to print 15.10). Re-verify per §22.4.3 before fitting. ⚠ **That target assumed the 14.95 seat is loose. The owner reports it tight, so +0.35 may give an interference fit — dry-fit before adopting it (D12)** | §22.4.3 |
| Idlers | Per side: 2 × 3×9×5 ZZ (measured 8.99 / 4.96), conical M3 washers, **fixed holes at (1.72, ±10.00) mm** (owner-confirmed). **Adopted as the fleet hole position for every joint, 2026-09-28** (§22.5.1, D4) | §22.2 |
| Belt | 116T GT2, 10 mm, **caliper-jig 105.0 — longest of the n = 10 batch**, loop ≈ 232.00 mm. **The same belt was fitted for A2 and for every SCB session P5–P8** (owner-confirmed) | §22.3 |
| Centre distance C | 44.5 mm | §22.2 |
| Pinion wrap at y = 10 | **137.6° = 4.59 teeth** (93.3° = 3.11 teeth with no idlers) | §22.4.6 |
| Belt stretch at y = 10, this belt | **0.20–0.37 mm.** Lower bound: a 232.00 belt on a pitch-seated pulley. Upper bound: measured raw no-idler geometric term | §22.4.6 |

⚠ **J02 runs the single longest belt of the batch, so its preload is the fleet *minimum* at
this hole position, not the typical value.** That is the correct calibration case under
§22.3's "calibrate the idler hole position on the LONGEST belt" rule, and this build satisfies it.

### 22.4.2 Acceptance at the frozen configuration (P8)

Predictions were stated before the runs.

| Criterion | Prediction | Measured | Verdict |
|---|---|---|---|
| **B6a — friction-corrected ladder intercept G** | ≈ 0; at most lost motion, 0–100 counts | **−6 to +13 counts = −0.009 to +0.018 mm** (I_f bracket 0.30–0.40 A) | ✅ **PASS** — ⚠ I_f bracketed (0.30–0.40 A), not measured; at J01's I_f (0.20–0.29) the same ladders give −25 to −8. Compare joints only at matched I_f (§22.4.4, §22.5.5) |
| Swing at ±0.6 A | 100–250 counts | **52 / 59** | ✅ Better than predicted |
| Stepped ladder to 1.6 A (11.4 N) | Clean | Repeats −1 / −1 | ✅ (≥ 2.0 A not attempted) |
| drag_c | J02 projection 0.41–0.50 A (§22.2 box); J01 band 0.30–0.40 A | **0.340 A** (range 0.306–0.400) | ✅ Below J02's projection |
| Coulomb asymmetry | < 20% | **8.0%** pooled across run orders; 22% in one single-order run | ✅ / ⚠ |
| Reverse drag fit rms | < 10 mA | **10.2 / 13.4 mA** (forward 6.0 / 4.0) | ❌ Marginal miss — D10 |
| Idler roller coast / pen mark | ≥ 1 s, rotates, no walk | ⟨V13⟩ | — |
| Ring f_n | Above 82.3 Hz with idlers | **72.5 / 70.1 Hz** | ❌ Falsified; not an acceptance criterion — §22.4.7 |
| Hand check | — | Taut, back-drivable, no perceptible backlash; the 1/rev disturbance is no longer felt | Qualitative — D11 |
| Belt teeth after the no-idler ratchet events | — | Inspected intact | ✅ |

### 22.4.3 Pulley seating — the defect, the fix, and the evidence

#### The defect

A belt seated freely in 108 grooves has its cord pinned at
**r_pitch = 108 × 2.000 / 2π = 34.3775 mm**, because the cord between adjacent teeth is a
fixed 2.000 mm.

Holding the cord higher requires one of two things:

- **cord strain** — 0.41% per 0.14 mm of height, or
- **teeth walking off-centre up the flanks.**

So "riding high" is a flank-wedged state whose height depends on tension. It costs:

- **Belt path:** dL/dr₂ = 4.655 mm per mm. Just 0.1 mm of ride-high consumes 45% of the
  1.04 mm take-up budget.
- **Load position:** the belt load moves onto the rounded tooth shoulders.

**Tip OD is not the path datum.** Printed tooth tips come out 0.4–0.7 mm under the GT2
nominal 68.25 mm (a 0.4 mm nozzle rounds them). A gap between belt land and tooth tip
therefore exists even when the belt is fully seated.

#### Measurement methods

| ID | Quantity | Procedure | Status |
|---|---|---|---|
| **T** | Tip OD | Tooth tip to tooth tip across a diameter, n ≥ 3, middle value | Authoritative, ±0.08 mm |
| **M-B** | Over-belt OD | Belt fitted and hand-tensioned on the pulley; caliper closed to **light contact only**, no deformation; n = 3, middle value | Authoritative, ±0.08 mm |
| **M-A** | Over-belt OD | Caliper pressed into the belt back | **RETIRED.** On FV1, two reading sets each repeated within 0.05 mm but differed from each other by 0.34 mm — larger than the effects being measured. **Never compare M-A to M-B** |

Derived quantities:

- **Cord radius:** `r_cord = OB/2 − 0.386` mm, where 0.386 = measured belt land 0.640 − GT2 pitch-line offset 0.254.
- **Path difference between two pulleys:** `ΔS = (OB₁ − OB₂)/2 × 4.655`. The method's common-mode bias cancels.
- **Pairwise uncertainty:** ±0.26 mm of path = ±180 counts.

#### Variant register

> **Slicer profile (owner-confirmed 2026-09-30):** every print in this register except
> **J01-P12B** was sliced under the **0.6 mm nozzle machine profile** with 0.4-tuned settings
> (recipe A, §22.6.1). So every measured print dimension here is a 0.6-profile number,
> including SCB, the pulley J02 was accepted on.

| ID | Build | Tip OD (T) | Over-belt (M-B) | r_cord raw | ΔS vs A | σ | Hand skip | Status |
|---|---|---|---|---|---|---|---|---|
| **A** | Arachne, no compensation | 67.76 | 70.10 | 34.664 | 0 | — | Holds | Reference |
| A2 | Sibling of A — same print session and settings (owner-confirmed), screwed | 67.67 | — | — | — | — | — | Swing 373, ring 94–97 Hz; retired |
| B, C | Arachne, no compensation | 67.87 / 67.60 (early T) | — | — | — | — | — | M-A only |
| SC1 | Slicer −0.10, boss uncompensated | 67.50 | — | — | — | — | **Best** | Boss loose; superseded by SCB |
| **SCB** | Slicer −0.10 + CAD boss +0.20 | **67.53** | **69.60** | **34.414** | **+1.16 mm** (+795 counts) | **4.4** | (as SC1) | **Fitted to J02** |
| FV2 | Fusion: offset in the single-tooth sketch before the circular pattern | 67.75 | 69.95 | 34.589 | +0.35 mm | 1.3 | — | Rejected |
| FV1 | Fusion: Offset Face on 5 groove faces, both tip-fillet faces excluded | 67.80 | 70.08 | 34.654 | +0.05 mm | 0.2 | **Worse than A** | Rejected |
| STD | Legacy standard walls, glued (§22.3 plant) | 67.2–67.5 (legacy T) | — | — | — | — | — | Swing 835, ring 59–72 Hz; retired |
| **J01-P10** | J01's first pulley: slicer −0.10, standard walls (the J02 compensation; boss parameter not recorded) | 67.55–67.58 | — | — | — | — | "a tiny bit tight" by hand | drag_c 0.414 A on belt 104.85; **replaced** mid-session S1 (§22.5) |
| J01-P12 | Slicer −0.12 + precise wall, **recipe A** (0.6 nozzle machine profile, §22.6.1), boss CAD 15.24 | not recorded (precise wall alone: unchanged within scatter, n = 1) | — | — | — | — | — | G 31–48 counts. **Superseded 2026-09-30 by J01-P12B**; its gate exception closed (§22.5, §22.6) |
| **J01-P12B** | Slicer **−0.12** + **precise wall**, **recipe B** (0.4 nozzle profile, §22.6.1), boss CAD **15.26**, printed **14.95** | within 0.05 of J01-P12 (value not recorded) | — | — | — | — | — | **Fitted to J01 — pulley of record, accepted 2026-09-30** (G 5.8 counts, §22.6). Identity: G-code md5 ⟨V22⟩ |

The hand-skip ranking was felt while tensioning the belt for the M-A reads. It is
unblinded and qualitative.

**Tip-OD fingerprint.** A slicer contour offset of −0.10 mm per side must drop tip OD by
about 0.20 mm.

- SC1 and SCB: **−0.26 / −0.23 mm ✅**
- FV1 and FV2: **+0.04 / −0.01 mm — tips unmoved.** Neither Fusion route reproduced the slicer geometry.

**Check the fingerprint on every compensated print before trusting any belt reading.**

#### Cross-check: caliper against swing

| Pair | Method | Result | Cord radius shift |
|---|---|---|---|
| A2 → SCB | No-idler swing, both ramped ±2.0 A, same belt (owner-confirmed) | 373 → **984 ± 3.5** counts = +611 counts = **+0.895 mm** of path | SCB 0.192 mm lower |
| A → SCB | Caliper M-B, A standing in for A2 (sibling print, owner-confirmed) | **+1.16 mm** of path (+795 counts) | SCB 0.25 mm lower |

The two agree within the pairwise ±180 counts.

The legacy STD → A2 change was 835 → 373 counts = −462 counts, which puts A2's cord 0.145 mm
*higher* than STD's. At the time this was attributed to Arachne printing a +0.43 mm larger
tip OD (R11).

#### Absolute seating — conditional on belt length

- **Measured:** the friction-corrected no-idler geometric term is **G = 0.996 mm** (raw 0.872 mm; §22.4.5).
- **Bound:** the fully-seated maximum slack for a 232.00 mm belt is **1.042 mm**.
- **Implies:** SCB's cord sits at **34.39–34.41 mm** (mesh lost motion 0–0.10 mm) — **within about 0.04 mm of pitch.**
- **By-product:** M-B's raw cord reading of 34.414 means **the M-B common-mode bias is ≈ 0.00–0.02 mm.**

⚠ **This rests on the 105.0 belt being 232.00 mm long.** That value is nominal from the
tooth count (116 × 2.000). §22.3's absolute-length inversion (231.97–232.07 mm) assumed the
STD pulley was seated at pitch, which uncompensated prints are now known not to be.
Treat the absolute length as soft by ±0.1–0.2 mm; every 0.1 mm of belt length moves the
inferred cord radius by 0.021 mm. **The relative results above do not depend on it.**
It also rests on the I_f used for the friction correction — see the caveats under §22.4.5.

#### Print scale and effective compensation

| Feature | CAD | A measured | SCB CAD | SCB measured | Loss vs A's print behaviour, per side |
|---|---|---|---|---|---|
| Flange OD | 70.25 | 70.30 (+0.07%) | 70.25 | 69.95 | −0.175 mm |
| Boss (hub) OD | 15.10 | 15.10 | 15.30 | 14.95 | −0.175 mm |
| Tooth-tip OD | — | 67.76 | — | 67.53 | −0.115 mm |

- **The printer is not undersize** (A: +0.07% on the flange, 0.00 on the boss). The pitch-radius floor is therefore usable.
- **The effective contour offset is not the nominal −0.10 mm per side.** It is −0.175 mm on smooth convex cylinders and −0.115 mm on nozzle-rounded tips.
- **Boss build parameter +0.35 mm** is predicted to print 15.10. Verify on the print — and see the tight-fit caveat in §22.4.1 before adopting it.
- The boss reading is more repeatable than the flange reading, because the smaller diameter seats the caliper more consistently.

#### Metal pinion control (taken under M-A; valid as a qualitative control)

- **Over-belt 8.26 mm**, against a seated range of 8.12 mm (flats) to 8.41 mm (corners) for the 12-sided belt polygon.
  → The belt seats fully on metal; **the defect is specific to the printed pulley.**
- **Pinion tip OD 7.10 mm** (nominal 7.131). It reads 6.94 when rotated — the same 12-gon effect (7.10 × cos 15° = 6.86, plus finite tip lands).
- **Not usable to calibrate 108T readings:** the polygon error is up to 0.29 mm on 12T, against 0.03 mm on 108T.

#### Belt land thickness

- Measured **0.640 mm** (GT2 spec 0.63), giving cord-to-back 0.386 mm.
- Idler cord radius = 4.495 + 0.386 = **4.881 mm.**

### 22.4.4 Swing ladder — the revised B6a readout

**Model:**

```
swing(I) = G + s·(I − I_f)
  G    = slack + mesh lost motion              <- the deliverable
  s    = elastic compliance, counts per amp
  I_f  = static breakaway. The shaft stops when belt force reaches (I − I_f)·Kt/r
raw intercept = G − s·I_f      ->  ALWAYS friction-correct before interpreting
```

**Why a ladder.** A single-current swing mixes slack, lost motion and elastic stretch.

- The ramped ±2.0 A, 984-count point gave **G = 1.366 mm**.
- That exceeds the 1.042 mm physical maximum, because the elastic term was subtracted using ring stiffness (51 counts).
- The ladder slope puts the true elastic term at **about 13× that.**

**Procedure** — key `w` (acSwingLadder), entered via `e → f → w` (firmware provenance: §22.4.12):

1. **Setup.**
   - Output locked. **J02, P5–P8: two screws** — the owner assesses the lock as very secure, no play, comparable to the glued rim bond. That is a hand assessment: §22.3's add-stiffness fixture test was run on the glued plant only and has not been repeated on the screwed lock.
   - **Fleet clamp (2026-09-28): two screws into the top plate, perpendicular to each other**, standardised screws: **2 × M3×10 countersunk** (§22.6.7). The holes belong to each top-plate assembly, so there is no shared jig.
   - **Record with every ladder whether the output was re-clamped since the last one.** A re-clamp on J01 moved the slope +8–12% and k −11% while leaving the intercept within noise (§22.5.4). k is only comparable at the same clamp.
   - Bearing seat as fitted — **J02 ran unshimmed** (§22.4.1). Record the seat state with every run.
   - Record pulley ID, idler state and belt ID **by hand**. The banner's belt field comes from `joint_cal.h`: J02's reads OFF (no belt row); J01's reads `10mm-9:1` since B11 (2026-09-28), but it does not carry the pulley variant.
2. **Current points.** ±0.6, 1.0, 1.4, 1.6 A.
   - Bottom = 2 × the 0.2983 A breakaway, so every leg completes its traverse.
   - Top = limited by the no-idler ratchet.
3. **Four legs per point.** `+cond` (discarded) → `−` → `+` → `−rpt`.
   - 5 s dwell per leg with a 1 Hz heartbeat; about 10 counts of creep converges in ~4 s.
   - `swing_a = + − m1`, `swing_b = + − m2`, `repeat = m2 − m1`.
4. **Skip detector.** |repeat| ≥ 0.5 tooth (ENC_CPR/12 = **1365.3 counts per tooth**) discards the point **and stops the ladder** — the belt has moved, so later points would measure a different assembly.
5. **Output.**
   - CSV row: `SW,<amps>,<minus1>,<plus>,<minus2>,<swing_a>,<swing_b>,<swing>,<repeat>`.
   - The firmware's printed fit is a convenience; refit offline.
   - Runtime about 80 s. The `lps` value on the first telemetry line after the ladder is an artefact (the main loop did not run) — ignore it.

**Scale.** 1 count = **1.4648 µm** of belt at the pinion pitch line. **7.107 N** of belt force
per reported amp, from Kt = `calKt(CAL)` = 0.018097 × 1.5 = **0.027146 N·m per reported amp**
(J02 row, M1-rescaled — confirmed from the firmware). That is about 2% above the 0.0266
carried in CONSTANTS §8 and HARDWARE §1, so every N and kN/m figure in this section inherits
~2%; nothing in counts or mm does.

⚠ **Superseded v1 leg order** (`+` from wherever the previous point left the plant, `−`, `+ret`):

- For points 2 onward, the first `+` leg was not a full traverse.
- This biased the swing low and produced spurious 30–37-count return errors.
- **The valid swing is recoverable as `cp2 − cm`** (both limits reached by a full traverse). All v1 numbers below use that.

**Proposed B6a pass rule** — provisional, derived from one joint:

- friction-corrected |G| ≤ 20 counts (0.03 mm) = **pass**
- 20–40 counts = **marginal**
- > 40 counts = **slack**

~~It needs the plant's I_f, measured by M4 by hand (D9).~~ It needs the plant's I_f, measured
by the **`B` / `b` firmware ramp** (B4 / M4; the ramp has been firmware-timed since it was
written, so "by hand" was wrong).

**How G is taken (unchanged rule, restated 2026-09-28 after the reversal below):** all ladders
at **one clamp setting, output not moved** between them. G = intercept + slope · I_f for each
ladder. When more than one ladder is run, report the mean and the spread. **Measured
single-ladder noise: ±7 counts** (J01 L4–L6, sd, n = 3), so one ladder cannot resolve G finer
than that, and a G within ~7 counts of 20 is not a clear pass or fail. **I_f:** state which
value was used. The B4 mean is the only measured one. On J01 the only measured motor-side
static figure (belt-off breakaway 0.2923 A) is as high as the B4 mean, so a lower "motor-side"
I_f needs a measurement behind it (§22.5.3, R20). **Discard ladders that ran while the
intercept was still climbing** (J01 L1–L3, bedding-in).

> #### ↩️ ~~METHOD CHANGE 2026-09-28 — G is a 3-position mean~~ — REVERSED the same day (R17)
>
> ~~G_joint = mean over ladders at 3 output positions (0°, +120°, +240°), with I_f = mean of
> the B4 readings.~~
>
> **Why it was proposed:** B4 breakaway ranged 0.085–0.49 A around the rotor, so I_f at any one
> ladder's position looked uncertain by ±0.13 A = **±22 counts of G**, wider than the pass band.
>
> **Why it is reversed (owner decision, with the measurement that supports it):**
> 1. **Moving the output position requires unscrewing the clamp**, and a re-clamp is itself a
>    variable: L3 → L4 across a re-clamp moved the slope +8% (mean-to-mean +12%) and k −11%
>    (§22.5.4). Three positions would be three clamps.
> 2. **The premise did not hold.** L3 and L4–L6 ran at different output positions and their
>    intercepts agree within 2–3 counts (§22.5.3). Position-dependent friction is much smaller
>    in the ladder than the B4 spread suggested.
>
> **Carried over from it:** the known-bias note, restated with its evidence. With the output
> clamped the output bearing does not turn, so ladder friction *could* be lower than the
> whole-joint B4 mean, which would push G high. On J01 the motor-alone belt-off breakaway
> (0.2923 A) is not lower than the B4 mean (0.288 A), so there is no measured support for a
> large bias. Not corrected.
>
> **J02 (§22.4.2):** the "single-position" part of the caveat added with the method change is
> withdrawn with it. **What remains:** J02's I_f was **bracketed (0.30–0.40 A), not measured**,
> and it sits higher than J01's 0.20–0.29 bracket. Compare G across joints only at matched I_f
> (§22.5.5).

**Curvature.** Residuals are convex in every run: the slope rises with current, i.e. the drive
softens as load increases. Linear fits over 0.6–1.6 A are conveniences; four points give
2 degrees of freedom to see the curvature.

### 22.4.5 Swing results by state (SCB pulley, belt 105.0)

#### No idlers (P5, P6)

| Current | v1 run 1 (cp2 − cm) | v1 run 2 | v2 (swing_a, swing_b) | Pooled | sd |
|---|---|---|---|---|---|
| 0.6 A | 761 | 769 | 765, 766 | **765.2** | 3.3 |
| 1.0 A | 875 | 876 | 881, 882 | **878.5** | 3.5 |
| 1.4 A | 1021 | 1027 | 1054, 1114 ⚠ creep | 1024 (v1 only) | 4.2 |
| 1.6 A | — | — | Ratchet ~0.96 tooth | — | — |

Also recorded:

- **Ramped ±2.0 A** (§22.3 procedure), three runs: 980 / 986 / 986 → **984 ± 3.5 counts.**
- **First ladder attempt (stepped 1/2/3 A):** 1.0 A gave 841 reported (854 as cp2 − cm), then 2.0 A ratcheted continuously. Excluded from the fits.

Fits (1.4 A excluded from the primary fit as incipient tooth climb):

| Fit | Slope | Raw intercept | G, friction-corrected (I_f 0.2983 A) |
|---|---|---|---|
| **2-point pooled, 0.6 / 1.0 A** | **283 cnt/A** | 595 counts = 0.872 mm | **680 counts = 0.996 mm** |
| 3-point pooled linear (incl. v1's 1.4 A) | 323 | 566 = 0.829 mm | 662 = 0.970 mm |
| 3-point quadratic (0 dof) | — | 656 = 0.961 mm | — |
| Firmware v2, 2-point | 290 | 591 = 0.866 mm | 678 = 0.992 mm |

- **G = 0.87 mm (raw) to 0.996 mm (corrected)** — below the 1.042 mm fully-seated bound.
- Stiffness from the slope: **k_beltline 30–34 kN/m.**

**Two caveats on the no-idler G, recorded from the owner verification of 2026-09-24:**

- **I_f provenance (V9).** 0.2983 A is J02's **belt-off** static breakaway — M4, n = 18,
  ±8.7%, `joint_cal.h` J02 row (cal 2026-08-08, belt OFF). It is not a measurement on this
  plant. The no-idler plant's own dynamic Coulomb drag is 0.292 A (P7, §22.4.8), close to it,
  but its static breakaway is unmeasured (D9). Sensitivity: each +0.1 A of I_f raises G by
  ~28 counts (0.041 mm), and **G reaches the 1.042 mm physical bound at I_f ≈ 0.41 A** — so
  the "within 0.04 mm of pitch" conclusion in §22.4.3 holds only while I_f stays near its
  proxy value.
- **Bearing seat unshimmed (V3).** A 0.15 mm-undersize boss could float by up to ±0.10 mm of
  centre = ±0.145 mm of path ≈ **±99 counts** in the no-idler state, where nothing preloads
  the seat (dL/dC = 1.454). The owner reports a tight fit with no perceptible play, so the
  float is probably not exercised — but that is hand assessment, not measurement. At y = 10
  the belt preload loads the seat one way, so the P8 result below most likely stands
  (inference).

#### Idlers at y = 10 (P8)

| Current | Run 1 (repeat) | Run 2 (repeat) | Pooled |
|---|---|---|---|
| 0.6 A | 52 (−2) | 59 (+2) | 55.5 |
| 1.0 A | 113 (−1) | 126 (+3) | 119.5 |
| 1.4 A | 189 (−10) | 202 (−3) | 195.5 |
| 1.6 A | 245 (−1) | 249 (−1) | 247 |

| Fit | Intercept | Slope | k_beltline | Residuals |
|---|---|---|---|---|
| Run 1 | −68.5 | 189.7 | 51.1 kN/m | +6.6, −8.3, −8.2, +9.9 |
| Run 2 | −57.7 | 188.5 | 51.5 kN/m | +3.7, −4.7, −4.1, +5.2 |
| **Pooled** | **−63.1** | **189.1** | **51.3 kN/m** | — |

- **Friction-corrected G:** −6 counts (I_f 0.30), +1 (0.34), +13 (0.40) → **−0.009 to +0.018 mm.**
- **Chord slopes:** 160 → 190 → 258 cnt/A (still softening with load).

**What it means.**

- At y = 10, **slack and mesh lost motion are both ≈ 0** (at most about 13 counts = 0.02 mm).
- The §22.3 B6a assumption of "~68 counts of GT2 backlash" does **not** describe a seated slicer-compensated pulley.

**Three independent chains agree.** Subject to the belt-length caveat in §22.4.3, all three
land within 0.05 mm:

| Chain | Result |
|---|---|
| Caliper jig | Belt 105.0 → ≈ 232.00 mm loop |
| Path solver | 1.042 mm of slack at pitch radius |
| No-idler ladder, friction-corrected | 0.996 mm |

### 22.4.6 Idler geometry solver and belt-length sensitivity

**Solver setup.** Four-circle, signed-radius belt path:

| Element | Position | Radius |
|---|---|---|
| Pinion | origin | r₁ = 3.8197 mm |
| Output pulley | (44.5, 0) | r₂ = 34.3775 mm |
| Idlers | (1.72, ±y) | −4.881 mm (back side of the belt) |

**Validation.**

- No-idler path = 230.958 mm (matches the §22.2 closed form).
- Take-up at y = 10 with the old 0.376 mm offset = 1.228 mm (matches the earlier session).

**Sensitivities.**

- dL/dr₂ = 4.655 mm per mm.
- dL/dC = 1.454 (no idlers) / 1.281 (idlers at y = 10).
- Idler and pinion cord circles collide below **y = 8.53** at x = 1.72.

**How to use the model.** Increments are robust: a 0.36 mm error in r₂ moves take-up-vs-y by
less than 5%. Absolute values need a measured anchor.

| y (mm) | Take-up (mm) | dTU/dy (backward, 0.5 mm) | Pinion wrap | Teeth in mesh |
|---|---|---|---|---|
| 9.0 | 2.275 | — | 165.1° | 5.50 |
| 9.5 | 1.695 | −1.16 | 149.2° | 4.97 |
| **10.0** | **1.238** | **−0.91** | **137.6°** | **4.59** |
| 10.5 | 0.875 | −0.73 | 128.3° | 4.28 |
| 11.0 | 0.590 | −0.57 | 120.6° | 4.02 |
| 11.5 | 0.370 | −0.44 | 113.9° | 3.80 |
| 12.0 | 0.207 | −0.33 | 108.0° | 3.60 |

- **Local slope at y = 10:** −0.78 mm per mm (10.0 → 10.2).
- **Direction:** smaller y = more take-up = tighter.
- **Pinion wrap is set by the idler position, not by the pulley.** The pulley decides how much stretch a given y produces: 0.1 mm of cord ride-high = 0.47 mm of path.

**Belt-length sensitivity.** Assumes a pitch-seated pulley; loop length = 2 × Δ(caliper reading), per §22.3.

| Caliper reading | Loop (mm) | No-idler slack | Stretch @ y = 10.0 | Stretch @ y = 10.2 |
|---|---|---|---|---|
| **105.00 (J02)** | 232.00 | 1.04 | **0.20** | **0.04** |
| 104.90 | 231.80 | 0.84 | 0.40 | 0.24 |
| 104.86 (batch mean) | 231.74 | 0.78 | 0.46 | 0.30 |
| 104.80 | 231.60 | 0.64 | 0.60 | 0.44 |
| 104.70 (shortest) | 231.40 | 0.44 | 0.80 | 0.64 |

**The batch range of 0.60 mm of loop = 0.77 mm of equivalent y.**

**Tension is deliberately not stated as a number.** Cord EA is unmeasured (the two-weight test
has not run), and ring-derived EA_eff depends on state and amplitude (§22.3). Indicative
only, at EA_eff 4–8.5 kN:

- 0.20 mm of stretch ≈ **3–7 N**
- 0.80 mm of stretch ≈ **14–29 N**

**Conclusions.**

- **y = 10.00 is frozen.** Moving to 10.2 would cut this belt's stretch to 0.04 mm — preload effectively gone.
- **y = 10.00 is predicted to hold every measured belt taut** (0.20–0.80 mm of stretch). None comes near pulley A's y = 10 condition (1.42 mm, "barely installable"). This is a prediction; it has not yet been measured on a shorter belt (D4).
- **Bedding-in headroom is thin on this belt (0.20 mm).** The remedy is a mid-batch belt, which gives 0.40–0.60 mm at the same y — **not a hole move.** Trigger: D2.

### 22.4.7 Ring results by state — no stiffness number is carried forward

| State | Pulley / mount | Idlers | Frequency (Hz) | ζ | Ring amplitude | n | k_motor (N·m/rad) | k_beltline (kN/m) |
|---|---|---|---|---|---|---|---|---|
| P0 (§22.3) | STD, glued | None, slack | f_d 59.6–72.0 | — | 29–54 counts | 12 | 2.8–4.2 | 190–290 |
| P1 | A2, screwed | None | f_d 93.6–97.1 (cap 0: 62.9, traversed slack) | — | 5.6–8.5 counts | 3 (+1) | 7.46–7.76 | 510–530 |
| P5 | SCB, screwed | None | **f_n 82.3 ± 0.4** (f_d 75.7) | 0.39 | Fitted 46 / overshoot 11–12 counts | 4 | 5.40 | 370 |
| P8 | SCB, screwed | y = 10 | **f_n 72.5 (+) / 70.1 (−)** | 0.41 / 0.70 | Overshoot 15 / 4 counts | 2 | 4.19 / 3.92 | 287 / 269 |

- **Conversions:** k_motor = (2πf)² · J_rotor (J_rotor = 20.2 × 10⁻⁶ kg·m²); k_beltline = k_motor / r₁².
- **Fit methods differ.** P0 and P1 used cycle counting and earlier fits. P5 and P8 used damped-sinusoid fits over 8–55 ms and 8–60 ms respectively, residual rms 1.0–1.8 counts.
- **Travel under the 0.2 → 0.8 V kick:** P5 221–224 counts; P8 +221 / −271 counts.

**Findings.**

- **Falsified:** the prediction that tensioning would raise f_n above 82.3 Hz. The tensioned plant reads **12–15% lower.**
- **Two stiffnesses, far apart.** In both states the ring (small amplitude) reads 5–12× stiffer than the swing slope (large excursion): P5 370 vs 34 kN/m; P8 269–287 vs 51 kN/m. Consistent with an amplitude- or hysteresis-dependent mesh. **Not resolved.**
- **P8 is the weakest dataset:** 2 bursts with ζ of 0.41 and 0.70, against P5's 4 bursts at ζ 0.39 ± 0.01.
- **Untested causes of the drop:** idler-mount compliance in series with the belt; added back-bend hysteresis; fit conditioning.

**Consequence.** None of these frequencies may be carried into the CONTROL.md §10 Tier-2
ceiling. At f_n/3 they would imply about 23–24 Hz (P8) to about 27 Hz (P5). **Held pending D1.**

The A2-plant conclusion — "2× stiffer, EA_eff 4 → 8.5 kN with tension, Tier-2 ~33 Hz" —
is **withdrawn** (R10).

### 22.4.8 Drag — phase 5 (SCB pulley, belt 105.0)

| State | Run order | Forward: Coulomb A + viscous mA/(rad/s) | Reverse: Coulomb A + viscous | Fit rms fwd / rev (mA) | Ke |
|---|---|---|---|---|---|
| No idlers (P7) | Forward first | 0.2773 + 1.948 | 0.2721 + 1.743 | 3.04 / 3.88 | 0.017867 ± 0.000025 |
| No idlers (P7) | Reverse first | 0.3192 + 1.599 | 0.2977 + 1.466 | 4.19 / 2.40 | 0.017870 ± 0.000025 |
| y = 10 (P8) | Forward first | 0.3200 + 3.082 | 0.4003 + 1.561 | 5.99 / **10.20** | 0.017883 ± 0.000023 |
| y = 10 (P8) | Reverse first | 0.3320 + 2.922 | 0.3060 + 2.535 | 3.97 / **13.37** | 0.017881 ± 0.000022 |

| Summary | No idlers | y = 10 | Change |
|---|---|---|---|
| **drag_c** (mean of 4 intercepts) | **0.2916 A** | **0.3396 A** | **+0.048 A** |
| Viscous, mean (mA per rad/s) | 1.69 | 2.53 | × 1.49 |
| Coulomb asymmetry, pooled across orders | 4.6% | 8.0% | — |
| \|I\| ratio (sanity band; theory 1.2247) | 1.232–1.243 | 1.226–1.237 | — |

**Against the archive:**

- **Idler contribution:** brass sliding posts 0.385 A (J01, §22.1) → rolling bearing idlers **0.048 A = −88%.**
- **J02 belt-on, no idlers:** 0.381 A (old pulley, run A, top plate on) → **0.292 A (SCB)**, i.e. −0.089 A (−23%). The assembly state matches — top plate on, the same state as the archived run (owner-confirmed) — so the drop is attributed to the pulley. ⚠ **A pulley swap is a full teardown, and handling is what resets J02's friction state: its trial-to-trial variability is ±37% (CONSTANTS §8.2 and the handling box there). −23% sits inside that, so read the attribution as suggestive, not established.**
- **J02 post-rebuild projection** 0.41–0.50 A (§22.2 box) → **0.340 A measured:** below projection and inside J01's 0.30–0.40 A band. Judged on amps, per §22.2.

**Notes on the fits:**

- **Pool both run orders.** Single-direction intercepts shift by 0.04–0.08 A depending on run order, with no consistent winner.
- **Reverse-fit rms is suspect.** With idlers it rises about 3× over the no-idler value and exceeds the 10 mA prediction. A periodic drag term from the idlers is suspected; not diagnosed (D10).
- **Ke is not carried.** Belt state moves Ke by only +0.08% (0.017868 → 0.017882), but belt-on reads +0.74% above the belt-off authoritative 0.017750. Consistent with §22.1 Finding 3.
- **Phase 5 measures dynamic drag only.** Static breakaway (M4, `breakaway_A`) has not been measured for this plant (D9).

### 22.4.9 Skip and ratchet observations

| Plant | Procedure | Held clean up to | Event |
|---|---|---|---|
| STD, no idlers (§22.3) | Ramped ±2.0 A | 13.9 N | — |
| SCB, no idlers, P5 | Ramped ±2.0 A, three runs | 14.2 N | — |
| SCB, no idlers, first ladder attempt | Stepped 1/2/3 A | 1.0 A (7.1 N) | **2.0 A: continuous ratchet**, shaft to ~72 rad/s, hand abort; belt re-caught one tooth over |
| SCB, no idlers, ladder v1 × 2 | Stepped 0.6/1.0/1.4 A | 1.4 A (9.9 N) | — |
| SCB, no idlers, ladder v2 | Stepped, 4-leg | 1.0 A | **1.4 A: 60-count monotonic creep** (repeat −60); **1.6 A (11.4 N): ratchet ~0.96 tooth**, ladder stopped |
| **SCB, y = 10, two runs** | Stepped, 4-leg | **1.6 A (11.4 N)**, repeats ≤ 10 | — (2.0 A not attempted) |

The belt teeth were inspected afterwards and are intact.

**No idlers (3.11 teeth in mesh).**

- The ratchet threshold was **10–14 N, and falling across sessions.**
- At 1.4 A the swing climbed monotonically (1021 → 1027 → 1054) while the 0.6 A value stayed flat. That is load-dependent, which points to **mesh degradation or step-loading, not belt elongation** (elongation would shift every point equally).
- Stepped versus ramped loading was never isolated.

**Idlers at y = 10 (4.59 teeth in mesh).**

- Clean at the highest load tested.
- **The skip threshold on the tensioned plant is unknown.** The design point is **213 N of belt force at 30 A Iq** (× 7.1 N/A).
- B10's static-lever method is the instrument (D3).

§22.3's "I_skip > 2.0 A on this plant" is specific to that plant and procedure. **Do not generalise it.**

### 22.4.10 Why the Fusion offset routes failed, and the fleet CAD route

**Slicer behaviour.**

- A slicer contour offset shrinks **every** outer contour — the tooth ring, but also the bearing boss, the flange and any register boss. Holes are unaffected.
- In Bambu Studio it is **global**: per user reports it cannot be set per modifier or per object (OrcaSlicer unverified).
- A hole that breaks a layer's topology is treated as outer contour on that layer.

**FV1.** Offset Face on the 5 groove faces.

- Adding the 2 tip-fillet faces (7 faces total) errors with *"An existing fillet or chamfer could not be deleted."*
- Printed tips unmoved.

**FV2.** Offset applied to the single-tooth sketch before the circular pattern. Tips are
unmoved by construction.

**Hypothesis — not proven.**

- The tip fillets set the groove mouth, and an unmoved mouth caps how far the belt can enter.
- Moving the flanks away while leaving the mouth where it was leaves the tooth resting on the shoulders with less flank support. That would also explain FV1's worse hand-skip feel.
- **CAD section check not run:** compare v9 vs v10 groove width at the tip circle and 0.3 mm below it, on a groove 180° from the seed.
- **Alternatives not excluded:** a Features-type pattern not carrying the offset; a reversed sign.

**Chosen for J02:** the slicer route, plus a CAD boss parameter `slicer_contour_comp`. The
part is correct only when sliced with the stored setting (§22.4.11).

**Fleet CAD route (D6).**

1. Roll the timeline back to before the tip-fillet feature.
2. Offset Face the groove faces **and** the OD cylinder by −`gt2_groove_offset`.
3. Set the tip fillet radius to `tip_r − offset` (only if the original radius is larger than the offset).
4. Confirm the pattern carries the offset.
5. **Accept only if** a print matches SCB on all three:
   - tip-OD fingerprint of −0.20 to −0.26 mm;
   - M-B over-belt of 69.60 ± 0.1 mm;
   - hand skip at least as good as A.

**Do not increase the offset (0.15 or 0.2 mm) without a failing seating result.**

- At 0.2 mm per side, each pulley tooth loses 0.4 mm of width — one full extrusion.
- Once the belt is seated, extra groove room buys backlash, not path.

### 22.4.11 Standing warnings (new)

- **The slicer contour compensation is not part of the model file.** Reslicing SCB without
  −0.10 silently removes ~1.1 mm of slack — the whole take-up budget — with no error and
  no symptom. Same hazard class as PB10 / 48V_EN (HARDWARE.md). Mitigations: the setting is
  stored in the 3MF project **and** carried in the filename; the CAD parameter
  `slicer_contour_comp = −0.10` drives the boss compensation.
- **A global contour offset loosens every outer-contour mating feature.** Bolts hide a
  loosened register boss at assembly; it surfaces later as output backlash.
- **A loose bearing seat corrupts belt measurements.** 0.10 mm of centre float = 0.145 mm
  of path ≈ 99 counts (dL/dC = 1.454).
- **Tip OD is the geometry fingerprint, not the path datum.**
- **Never compare M-A readings to M-B readings.**

### 22.4.12 Data integrity and session provenance

All sessions: J02, bench supply at about 12.5 V (not the 5S pack), `vbus_scale` 0.008516
unchanged, `VBUS_LIVE` FALSE.

**Firmware (V7).** The P6–P8 binaries were built from an **uncommitted working tree** on
`refactor/extract-headers`, on top of stage 3b (`7a428a1`, **not bench-accepted**), so no
commit identifies them. Build sizes, in the order they were produced: 108,372 B (ladder
1/2/3 A, first attempt) → 108,636 B (0.6/1.0/1.4 A, stop-on-skip; P6a) → 108,708 B (4-leg,
+1.6 A; P6b–P8, assuming no reflash in between). RAM 24,228 B throughout. The first commit
containing the ladder is `b2a562e` (2026-09-24): its `acSwingLadder` matches the P6b–P8
build, but the commit also carries later changes those sessions did not run (the `E` encoder
monitor, the `e` verdict fix, the jump-guard latch fix). md5 of the flashed binaries: ⟨V7⟩.

| Session | Date | Pulley | Idlers | Tests | Banner / UT89X (V) | Δ | perr start → end | nmg | ovs |
|---|---|---|---|---|---|---|---|---|---|
| P5 | ⟨V8⟩ | SCB | None | Ramped swing ×3, ring ×4 | 12.56 / 12.59 | −0.24% | 105,910 → 106,203 | 0 | 0 |
| P6a | ⟨V8⟩ | SCB | None | Ladder v1 ×2 | 12.53 / 12.57 | −0.32% | 0 → 0 | 0 | 0 |
| P6b | ⟨V8⟩ | SCB | None | Ladder v2 | 12.51 / 12.54 | −0.24% | 0 → 247 | 0 | 0 |
| P7 | ⟨V8⟩ | SCB | None | Phase 5 ×2 | 12.49 / 12.53 | −0.32% | 0 → 1,164 | 0 | 0 |
| P8 | ⟨V8⟩ | SCB | y = 10 | Phase 5 ×2, ladder ×2, ring ×2 | 12.47 / 12.51 | −0.32% | 2,699,720 → 2,699,728 | 0 | 0 |

**vbus.**

- The proportional error is flat at −0.24% to −0.32%, within the 0.01 V quantisation of both instruments.
- The absolute supply voltage fell from 12.59 to 12.51 V. **This is the supply, not scale drift.**
- CALIBRATION §20.1's absolute 0.03 V gate sits at the instrument resolution floor at 12.5 V (a percentage restatement is proposed for CALIBRATION.md §20.1; not yet applied).

**perr.**

- 2.7 million errors accumulated during setup handling between P7 and P8, while `no_mag` warnings were being seen.
- During P8 itself: **+8** across all tests. `nmg` and `ovs` were 0 on every line of every session.
- **P8's data are treated as clean.** What the counters mean (V11, confirmed from `mt6816.h`):
  `perr` counts **parity failures only** (`spi_err`, incremented in `readAngleRaw()` on odd
  parity; the angle is then held for one cycle). `nmg` is a **live flag, not a counter**, and
  is written **only on a successful read**. Angle-jump rejects are a separate counter
  (`spi_jump`, printed as `jrej`) that was **never active** — `SPI_JUMP_GUARD` was false for
  P5–P8. So parity catches only odd bit-error counts and `perr` is a lower bound on corrupted
  frames, and `nmg = 0` across a fault does not exclude a field problem. P8's cleanliness
  therefore rests on +8 parity events **and** on the internal reproducibility in §22.4.5 and
  §22.4.8 — not on `perr` alone.
- The fault tracks handling of the assembly (moving or flipping it), not rotation or load. The magnet was inspected and is intact.
- Suspected cause: the SPI harness or connector (PB5–PB8, VCC, GND). Tracked outside this file. **It gates B0.**
  ⚠ **Added 2026-09-24:** HVPP (chip pin 2, internal 150 kΩ pulldown, selects SPI vs ABZ)
  belongs on that list and was omitted. After P8 the J02 ESC board failed — hot at idle on
  the 3S pack, no serial on boot — and a sagging 3V3 rail pulling HVPP low is a candidate
  cause of the whole SPI fault. Unresolved. It does not reopen P8: the +8 parity events and
  the reproducibility above argue against contamination.

**dq convention.** In P5, |I|/Iq = 2.450 / 2.000 = **1.2250**, against √(3/2) = 1.2247 — the
amplitude-invariant dq convention is confirmed.

### 22.4.13 Retraction register

Conversation-level claims from the sessions behind this section, recorded so they are not rediscovered.

| # | Claim | Status | Reason |
|---|---|---|---|
| R1 | "Tooth engagement 24% / 52%" | ❌ Retracted | Used tip OD as the datum, but printed tips are undersize. The right metric is cord radius vs 34.3775 mm |
| R2 | "Belt pre-stretched 0.47 mm with no idlers (interference)" | ❌ Retracted | Rested on a single 70.20 mm M-A reading |
| R3 | "373 counts = 0.49 mm of tooth backlash from shallow engagement" | ❌ Retracted | Narrow grooves wedge the tooth against both flanks, which means less clearance, not more |
| R4 | "Three hole pairs on one plate, 0.5 mm apart" | ❌ Retracted | M3 clearance holes at that spacing merge into a slot — the failure fixed holes exist to avoid |
| R5 | "A Fusion CAD offset reproduces slicer contour compensation" | ❌ Retracted | §22.4.10 |
| R6 | M-A-based cord-radius / slack / y / teeth table (comp −0.10 mm below pitch; A +0.14; B +0.30; C +0.15) | ❌ Not established | Method retired |
| R7 | "The compensated pulley buys +0.6–1.0 pinion teeth" | 🔁 Reframed | At fixed y, pinion wrap is set by the idler; the pulley changes the stretch at that y |
| R8 | "N ≈ 1100 counts if the caliper value is right" | ❌ Retracted | Placed the cord below pitch radius, which is impossible |
| R9 | Elastic subtraction of 51 counts at ±2.0 A using ring stiffness, giving G = 1.366 mm | ❌ Retracted | Exceeds the 1.042 mm bound; the ladder slope gives about 13× |
| R10 | "Drive 2× stiffer; EA_eff 4 → 8.5 kN under tension; Tier-2 ceiling ~33 Hz" (A2 plant) | ❌ Withdrawn | Confounded by pulley change, mount change (glue → screws) and ring amplitude. The tensioned SCB plant reads 70–73 Hz |
| R11 | "Arachne prints the pulley +0.43 mm larger, which explains the 835 → 373 swing change" | 🔁 Reinterpreted | The change is cord radius (+0.145 mm on A2), not tip OD |
| R12 | "Ring f_n rises above 82.3 Hz with idlers" | ❌ Falsified | §22.4.7 |
| R13 | §22.3 swing reading table (> 200 slack / ~70 / 10–40 / 5–12 seated) | ⤳ Superseded | Replaced by the friction-corrected ladder intercept (§22.4.4) |
| R14 | Pitch floor as a hard bound on caliper bias (≤ 0.037 mm) | ⚠ Softened | Softened for print scale, then supported by the scale check (A: +0.07%). Stands at about ±0.05 mm, conditional on the belt's absolute length |
| R15–R20 | J01 −0.12 campaign: raw-swing reject rule; "breakaway always > drag_c"; the 3-position G method (reversed); position-explains-spread; geometry-matches at mismatched I_f; the 0.20 A I_f | ❌ / 🔁 / ↩️ | §22.5.12 |
| R21–R25 | J01 recipe B: backlash-is-the-compensation and its trade-off line; breakaway compared with drag_c; outer-wall-width difference; settings-copy reproduces the toolpath; phase 4 "belt-agnostic" | ❌ / 🔁 | §22.6.9 |
| R26–R28 | Units / B12 scoping, 2026-10-01: the stale Kt 0.0266 in §22.6.3–4; "lost motion below encoder resolution"; "1.6 A ≈ 0.38 N·m" | 🔁 / ❌ | §22.6.9 |

### 22.4.14 Deferred items and promotion conditions

| # | Item | Promote when |
|---|---|---|
| **D1** | Ring re-run, 4 bursts per direction, on the tensioned plant | Before any CONTROL.md §10 Tier-2 ceiling update or impedance-bandwidth decision. The §22.3 J01-vs-J02 B6b comparison also needs this method settled first. **2026-09-28: method run on J01 (−0.12 pulley, 4 + 4, f_d 64.3 ± 0.4 Hz, §22.5).** Still open for the ceiling decision: the ring amplitudes were not in the summary, so the result cannot yet be placed on §22.3's amplitude relation, and all 8 bursts come from one boot. **2026-09-30, recipe B:** 4 + 4 **with amplitudes recorded** (A₀ 18 / 28 counts; slope −0.47 ± 0.08 Hz/count, matching §22.3's −0.522; f_d 64.9 Hz at A₀ = 23), §22.6.5. The amplitude placement now exists for one joint; the ceiling decision still waits on a second joint and more than one boot |
| **D2** | B6a ladder repeat after B0's first running hours | Scheduled with B0. If friction-corrected G > 40 counts, fit a mid-batch belt (104.8–104.9) **before** touching the plate. *2026-09-28: take G per §22.4.4 at one clamp setting; the plate is now fixed fleet-wide at (1.72, ±10.00), so "fit a mid-batch belt" or a pulley-compensation change are the only levers.* *2026-09-30: also the check that J01 recipe B's G = 5.8 holds in a later session (A's biggest bedding-in rise came across sessions, §22.6.3)* |
| **D3** | Skip threshold on the tensioned plant (B10 static lever) | Before any command above ~11 N of belt force (1.6 A), and before leg attachment |
| **D4** | Fleet hole position on a shorter belt | ~~On the first joint built with a belt under 105.0: ladder G ≈ 0 and drag_c in band.~~ **First half ANSWERED 2026-09-28 by J01 (belt 104.85, §22.5):** on the −0.10 pulley drag_c was 0.414 A, just over the 0.40 band, and G was near zero (raw intercept −57.3; the S1a slope was not recorded, so G is estimated only). **Owner decision: the hole position is FIXED fleet-wide at (1.72, ±10.00)**, and belt length is absorbed by pulley compensation (−0.12 recipe B on J01, §22.6.7). *(The 2026-09-30 acceptance used the same 104.85 belt, so it is not (b)'s data point.)* **Still open:** (a) confirm the 20-belt batch maximum is ≤ 105.0, since a longer belt would become the new loose case; (b) one joint on a genuinely short belt (≤ 104.7) with its binned compensation: drag_c in band **and** G against the B6a gate. Promote with the 20-belt batch measurement / joint 3 |
| **D5** | Fleet pulley repeatability: M-B repeatability (one pulley, 5 refits) plus a second slicer-compensated print (tip-OD fingerprint, over-belt) | Before printing any fleet pulley. *2026-09-30: recipe A → B is not a repeatability sample (the toolpath changed, R24). Fleet repeatability means two prints of the **same md5** (§22.6.1). Every tip-OD / M-B / offset reference in §22.4.3 is a 0.6-profile number, so **re-baseline the fingerprint on recipe-B prints first** (§22.6.6)* |
| **D6** | Fleet CAD route (§22.4.10) plus the CAD section check | Only if the slicer-setting coupling is to be removed from the fleet build |
| **D7** | Belt-difference swing, 105.0 vs 104.7 belt, predicted 410 counts | Before selective assembly relies on the caliper jig. Also resolves the pulley assumption behind §22.3's absolute-length inversion |
| **D8** | Cord EA two-weight test | No longer on the hole-position path. Promote only if a tension in newtons becomes load-bearing (bearing load, belt fatigue budget) |
| **D9** | M4 static breakaway on the tensioned plant (fill `breakaway_A`) | With B0. Required before the ladder's friction correction is used to decide anything on another joint. Also closes the I_f-proxy caveat on the no-idler G (§22.4.5). **J01 half CLOSED 2026-09-28:** B4 on the −0.12 plant, mean 0.288 A (§22.5); **re-measured on recipe B 2026-09-30: 0.295 A, n = 20, now in `breakaway_A`** (§22.6.4). **J02 half still open**, and blocked on the failed J02 board |
| **D15** | B6b J01-vs-J02 15% comparison (§22.3) | When J02 has a working board: run B6b 4 + 4 in `t` mode, and B4, on J02. Record both pulley recipes with the result. ⚠ Clamps are per-assembly (§22.6.7), worth ~±5% in f on their own; re-run J01's ring at its current clamp in the same sitting rather than reusing a stored value (recipe B: 65.0 Hz, §22.6.5) |
| **D16** | Separate cogging from position-dependent preload in B4's direction-biased component (±0.18 A) | Only if the sim friction model (§22.5.9) needs the ripple's period or phase. 10 readings ~37° apart cannot resolve a cogging period of a few degrees; this needs a dense angle sweep |
| **D19** | **⬆ PROMOTED 2026-10-02 (a3 part 2, §22.7.11):** 44.4 / 48.2 mN·m output (4.9 / 5.4 mN·m motor) in current mode, direction-independent phase, speed-independent. **Active item: characterise per joint after B12a** (amplitude and phase vs electrical angle) and evaluate a Tier-0 feed-forward. **Scope added 2026-10-02 (a4, O4, §22.7.13):** the washboard sets every rest position (detent ≈ 218 counts mod 1170 on J01, stick band ±70), and friction near standstill (~0.02–0.05 N·m) is the same size as the washboard. The characterisation must therefore include an **at-rest torque–angle sweep**: a quasi-static p_des ramp at kp ~41 over ≥ 1 period (1170 counts, 50 mrad output) in both directions. The mean curve gives the washboard shape at rest, half the hysteresis width gives static friction, and the result tests O3/O1's H1 per capture. Original entry follows. — **Position-locked torque ripple, 14 per motor turn** (2 per electrical cycle; a0, §22.7.2): ~4.3–5.5 mN·m motor (~39–49 mN·m output), phase-locked to rotor angle, only ~30% visible as current ripple. Probably the 09-30 breakaway "position scatter" (+ direction fits at R² 0.92, 0.25 A; − direction does not fit cleanly — strong hypothesis, not established) | **Promote if** a3 shows it persists in current mode at ≥ 3 mN·m (motor): characterise per joint (amplitude + phase vs electrical angle) and evaluate a Tier-0 feed-forward. Adds a contract-freeze item: does Tier 0 cancel it, and does τ then include it? |
| **D18** | **Drag run-order effect:** in phase 5 the direction run second reads 0.10–0.13 A higher (J01 recipe B, §22.6.6). It is not monotonic warm-up; the hypothesis is reversal straight after high speed | Only if a feedforward drag term needs better than the ±20% plant spread. Pooling both orders (the stored convention) cancels it. Discriminating test: one direction run twice with a rest between, vs twice back-to-back |
| **D17** | Control-side backlash tolerance test (high-gain position hold on the impedance loop, raising kp in steps and watching for a limit cycle, ~10 min) | **DEMOTED 2026-09-30.** ~~Closes the J01 B6a gate exception~~: that exception closed when recipe B passed B6a at G = 5.8 counts (§22.6.7). **Promote if** any fleet joint fails G ≤ 20 counts, or before the B6a gate is re-derived from controller tolerance instead of from J02's result |
| **D10** | Reverse-fit rms of 10–13 mA and the run-order drag shifts | Inspect the B0 drag map for per-idler-revolution or per-output-revolution terms |
| **D11** | "The 1/rev disturbance is gone" (hand feel only) | Confirm the h1 term in the B0 drag map |
| **D12** | J02 build pulley with boss +0.35 | Replace SCB only with re-verification (tip-OD fingerprint, ladder G ≈ 0). Until then, SCB — **unshimmed**, with an owner-reported tight fit — is the characterised plant. ⚠ If 14.95 is already tight, +0.35 (predicted 15.10) may be an interference fit: dry-fit the print before adopting it. Promote if the seat develops play, or when the joint moves to leg-attached testing. ⚠ **2026-09-30: SCB is a 0.6-profile (recipe A) print, and the +0.35 → 15.10 prediction came from 0.6-profile behaviour, so it is VOID.** Any J02 reprint is on recipe B (0.4 profile); take its boss offset from the recipe-B print (CAD 15.26 → 14.95 at −0.12) and re-accept J02 before D15 |
| **D13** | Chalk witness test for tooth contact | Only if a future pulley fails seating |
| **D14** | Stepped vs ramped loading at matched current, for the no-idler ratchet | Only if the no-idler swing is used again as a QC step |

### 22.4.15 In-place annotations required elsewhere in BELT_DRIVE.md

Annotate only; do not delete.

**§22, table row B6a — pass condition "≈ 0 counts of free play … ~68 counts":**
append *"→ readout superseded by the friction-corrected ladder intercept, §22.4.4. The
68-count backlash figure does not describe a seated slicer-compensated pulley (§22.4.5)."*

**§22.2, "Prediction after the rebuild" and the J02 box:** add the result line:
*"J02 measured ⟨V8⟩: drag_c 0.340 A, asymmetry 8.0% pooled ✅, reverse rms 10–13 mA ❌ —
§22.4.8."*

**§22.3, "Why a target tension cannot be hit by printing accurately":** add *"J02's hole
position was accepted by measurement at the existing y = 10.00 — §22.4.2 / §22.4.6."*

**§22.3, "Calibrate the idler hole position on the LONGEST belt":** add *"Satisfied on J02:
belt 105.0 at y = 10.00 — §22.4.1."*

**§22.3, selective assembly ladder:** add a pointer to the §22.4.6 belt-sensitivity table
(batch range 0.60 mm = 0.77 mm of equivalent y; y = 10 predicted to cover the batch — D4).

**§22.3, swing test box:**

- Reading table → ~~strike~~, *"SUPERSEDED → §22.4.4 (R13)."*
- Keystroke procedure → *"superseded by `w` (acSwingLadder), §22.4.4."*
- "RUN 2026-09-05 — 835 counts", absolute-length inversion → add *"⚠ The inversion assumed
  the STD pulley seated at pitch. Uncompensated prints ride 0.1–0.3 mm high (§22.4.3), so
  231.97–232.07 mm carries an unquantified bias. The belt-difference test (D7) remains the
  robust check."*
- "Tooth skip: I_skip > 2.0 A on this plant" → add *"Plant- and procedure-specific: the SCB
  no-idler plant ratcheted at 1.6 A under stepped reversal — §22.4.9."*

**§22.3, "STOP RING-TESTING THIS PLANT":** add *"Applies to the STD glued plant. SCB rings
are recorded in §22.4.7."*

**§22.3, "Next instrument: the caliper two-weight test for cord EA" and its three-plate-pair
plan:** add *"SUPERSEDED for J02 ⟨V8⟩: hole position accepted by measurement at y = 10.00
(§22.4.2). EA test demoted to D8."*

**File header index line:** add §22.4.

*All annotations above were applied on 2026-09-24.*

---

## 22.5 J01 — output-pulley compensation −0.10 → −0.12, recipe A (2026-09-27 → 2026-09-28) — SUPERSEDED by §22.6

> ### ⤳ SUPERSEDED 2026-09-30 — this closure is history; §22.6 is the pulley of record
>
> Every pulley in this section was printed under the **0.6 mm nozzle machine profile** with
> 0.4-tuned settings ("recipe A", §22.6.1). J01-P12 was reprinted on the correct 0.4 profile
> (recipe B, same −0.12 compensation) and re-accepted: **G fell from 31–48 to ~6 counts**, so
> **the B6a gate exception below is CLOSED** and the attribution of the extra backlash to the
> −0.12 compensation is **withdrawn (R21)**.
>
> **Still valid and still referenced:** §22.5.3 (bedding-in, single-ladder noise, I_f choice),
> §22.5.4 (the clamp moves stiffness, not backlash), §22.5.9 (breakaway structure), §22.5.12
> (R15–R20). **Replaced by §22.6:** §22.5.5, §22.5.6, §22.5.7, §22.5.8, §22.5.11, §22.5.13 (now
> one-line pointers). Numbers below describe recipe A.

**What opened it.** J01's first belt-on assembly, on the J02 recipe (slicer contour
compensation −0.10 mm), felt slightly tight by hand. The run doubled as the D4 test: does a
belt shorter than 105.0 pass on the standard pulley at the fixed idler holes, with no tensioner?

**What closed it.**

1. **D4 answer, standard pulley:** marginal. drag_c 0.414 A, just above the 0.30–0.40 A band.
2. **Fix:** slicer compensation −0.12 mm + precise wall. drag_c **0.291 A (−30%)**, session drift
   ~7× smaller, stiffness unchanged. Ladder intercept stable once bedded in (L4–L6 vs L3).
3. **Cost:** backlash **G = 31–48 counts (0.07–0.12° at the output)**, above the ≤ 20-count B6a gate.
   The cross-joint comparison attributes it to the pulley geometry (§22.5.5).
4. **Decision (owner, 2026-09-28): −0.12 LOCKED for J01 with a VISIBLE GATE EXCEPTION on B6a**,
   pending the control-side backlash test (D17). Fallback if that fails: −0.11 (§22.5.6).
5. **B11 written:** `joint_cal.h` J01 row, belt-on (§22.5.8). The belt-off baseline row is
   kept verbatim at the end of the table.
6. **Reversed the same day:** the 3-position ladder method (§22.4.4). Moving the output needs
   the clamp unscrewed, and a re-clamp was measured to move the ladder slope ~8–12%. It adds
   a variable rather than removing one.

> ⚠ **Provenance.** Every number below is transcribed from the session analysis summaries.
> None was refit from raw rows in this repo. The raw `SW,` rows, `M4,` rows and ring dumps
> are **not archived here ⟨V16⟩**, and `docs/` is still gitignored (README §24.13). Bench
> dates are not in the summaries, so sessions are dated by when they were reported. The
> slope and k columns in §22.5.3 were cross-checked here: G = intercept + slope·I_f and
> k = 2·(Kt/r₁)/(slope·1.4648 µm) reproduce every tabulated G and k to ±0.5.

### 22.5.1 Plant state

| Item | Value |
|---|---|
| Joint / board | J01, `B-SPI-01` (`vbus_scale` 0.008448) |
| Belt | Caliper jig **104.85** (as recorded 2026-09-27; the 2026-09-28 summary rounds it to 104.9). 0.15 mm shorter than J02's 105.0. **Batch position** (§22.3, n = 10, 0.1 mm bins: 104.7 ×1 · 104.8 ×3 · 104.9 ×5 · 105.0 ×1, mean 104.86): in the modal 104.9 bin, with only one belt longer. So it is **long relative to 4 of 10 and at the batch mean**. It is a longer-than-typical belt, not the fleet extreme. The earlier "mid-batch" (2026-09-27 plan) and "near the long end" (2026-09-28 summary) descriptions are both partly right |
| Idlers | 2 × 3×9×5 ZZ per side, **fixed holes (1.72, ±10.00) mm** (owner-confirmed 2026-09-28; ⟨V17⟩ closed). Same idlers and holes for both pulleys. **This hole position is now the fleet standard for every joint** (owner decision 2026-09-28): belt-length variation is absorbed by pulley compensation, not by the holes. See D4 |
| Pulley, session S1a | Slicer −0.10, standard walls (the J02 recipe). Tip OD 67.55–67.58 |
| Pulley, S1b–S3 | **Slicer −0.12 + precise wall** — variant **J01-P12** in the §22.4.3 register. Boss CAD planned at 15.24, to hold the 15.00 fit against the extra compensation. **Printed boss as measured ⟨V18⟩ — owner to confirm**; the B11 record carries the measured value, not the CAD value. Tip OD of this print not recorded |
| Precise wall, alone | Tip OD unchanged within scatter, **n = 1**. Adopted as a recipe change with no dimensional effect |
| Top screw / pinion top support | **Off** in every session. The planned top-plate bearing + M4 changes belt tension, so these numbers become the reference for that comparison |
| Output clamp | **Two screws into the top plate, perpendicular to each other** (owner-confirmed 2026-09-28 for J01; ⟨V19⟩ closed). The holes are part of each top-plate assembly, so no two joints share a clamp: **the standard is the screw spec, not a jig** (§22.5.7). B4 needs the output free, so every B4 means an unclamp/re-clamp. L4 followed a re-clamp |
| Supply check | S1b: banner 12.40 V = UT89X 12.40 V ✅ |

| Session | Pulley | What ran |
|---|---|---|
| **S1a** | −0.10 | B1, B2, B3 × 4, one ladder |
| **S1b** | −0.12 | B1, B2, B3 × 7, ladders L1 and L2 back-to-back |
| **S2** | −0.12 | B4 (5 × 2), L3 about 2 min after B4, B6b 4 + 4 |
| **S3** | −0.12 | Re-clamp, then L4; L5 at +5 min; L6 back-to-back with L5 (the §22.5.3 repeat test) |

### 22.5.2 Results

| Test | −0.10 (S1a) | **−0.12 (S1b–S3)** | Gate | Verdict |
|---|---|---|---|---|
| B1 ZEA shift | ✅ | 1.98° | ≤ 8° | ✅ |
| B2 R_eff | 0.22009 Ω | 0.21883 Ω | 0.215–0.230 | ✅ |
| B3 drag at 64 rad/s, first run | 0.562 A | **0.442 A** (−21%) | — | — |
| **B3 drag_c, first run** (fit on top 3 speeds) | 0.414 A | **0.291 A** (−30%) | ≤ 0.40 A | ✅ |
| B3 drag_c, all runs pooled | — | 0.293 ± 0.021 A (7 runs) | — | — |
| B3 drift across session | +0.083 A over 4 runs | +0.022 A over 7 runs | — | ~7× less per run |
| B3 viscous | ~0.0025 A/(rad/s) | ~0.0024 | — | unchanged |
| B3 direction asymmetry | — | 3.8% | < 20% | ✅ |
| **B4 breakaway** | not run (B4 on the final configuration only) | **mean 0.288 A**, median 0.31, SD 0.12, range 0.085–0.49 | < 0.65 A | ✅ |
| Ladder k_beltline | 56.3 kN/m | 57.3 / 59.8 / 55.7 (L1–L3) · **51.4 / 50.4 / 52.7 (L4–L6, after re-clamp)** | unchanged | ✅ vs pulley; **−11% from the re-clamp** (§22.5.4) |
| Ladder swing at 0.6 A | 50 counts | 65 / 77 / 98 / 106 / 101 / 112 (L1–L6) | — | friction + clamp, see §22.5.3 (R15) |
| **B6b ring** | — | **f_d 64.3 ± 0.4 Hz** (8 captures, S2 clamp), ζ 0.32 (+) / 0.43 (−), **f_n ≈ 69.5 Hz** | J01 vs J02 within 15% | ⏸ D15 — J02 board failed |
| **B6a G** (friction-corrected, stable ladders L4–L6) | — | **31 (I_f 0.20) – 48 (I_f 0.29) counts** = 0.07–0.12° output. Upper end better supported (§22.5.3) | ≤ 20 | ⚠ Gate exception 2026-09-28 → **closed 2026-09-30** by recipe B (§22.6.3) |

J01 is now the lowest-drag belt-on joint measured (J02 accepted at 0.340 A, §22.4.2). The
1-minute rest between S1b runs 2 and 3 changed nothing (0.4532 → 0.4526 A), but one minute
is too short to count as a rest test. **The drift-vs-tension link is supported, not proven.**

**Prediction scorecard** (all stated before the runs):

| Prediction | Measured | Verdict |
|---|---|---|
| drag_c on −0.10 = 0.36–0.45 A (low confidence) | 0.414 | ✅ in band, in the 0.40–0.45 "marginal" branch, which triggered the −0.12 print |
| drag_c on −0.12 = 0.33–0.39 A | 0.291 | ❌ **Finding.** The pulley effect was ~3× what the belt-path estimate implied (Δr −0.02 mm × ~3.5 rad wrap ≈ −0.07 mm of path). **Drag is much more sensitive to belt-path geometry than that model** |
| B4 breakaway ≈ 0.5 A | 0.288 | ❌ **Finding.** Belt-on, the mean static breakaway equals the dynamic drag. There is no extra average sticking force to overcome from rest |
| Swing at 0.6 A > 70 counts ⇒ too loose, reject | — | ❌ **Rule retracted (R15)**, see §22.5.3 |
| Repeat ladder (S3) intercept within ±8 of L3's −4.0 ⇒ not progressive; up ≥ 15 ⇒ growing lost motion | L4–L6 mean **−6.3** (Δ −2.3, downward) | ✅ **Not progressive.** The wear hypothesis is rejected |
| Geometric backlash cost of −0.02 mm/side: groove +0.04 mm ≈ **+27 counts** | J01 − J02 at **matched** I_f ≈ **+56 counts** | ⚠ **Right direction and order, ~2× the size.** Confounded (§22.5.5) |

### 22.5.3 Backlash — six ladders; the drift is closed

| Ladder | Session, timing | Intercept (counts) | Slope (cnt/A) | k (kN/m) | Swing at 0.6 A | G at I_f 0.20 | **G at I_f 0.29** |
|---|---|---|---|---|---|---|---|
| — | S1a, −0.10 pulley | −57.3 | — | 56.3 | 50 | — | — |
| L1 | S1b | −37.0 | 167.8 | 57.3 | 65 | −3 | 12 |
| L2 | S1b, back-to-back with L1 | −16.6 | 160.8 | 59.8 | 77 | 16 | 30 |
| L3 | S2, ~2 min after B4 | −4.0 | 172.8 | 55.7 | 98 | 31 | 46 |
| **L4** | S3, first run after re-clamp | **−7.2** | 187.1 | 51.4 | 106 | 30 | 47 |
| **L5** | S3, +5 min | **−13.2** | 191.0 | 50.4 | 101 | 25 | 42 |
| **L6** | S3, back-to-back with L5 | **+1.6** | 182.5 | 52.7 | 112 | 38 | 55 |
| **L4–L6 mean** | | **−6.3 ± 7.4 (sd)** | 186.9 | 51.5 | | **31** | **48** |

G = intercept + slope × I_f. **The recorded J01 value is the L4–L6 mean, G = 31–48 counts**
across the I_f bracket.

**Which I_f.** The two columns bracket it:
- **0.29 A** = the B4 mean: whole-joint static breakaway, output bearing included.
- **0.20 A** = the session analysis's *estimate* of motor-side friction alone, on the grounds
  that the output bearing doesn't turn while the output is clamped.

⚠ **Conflict, stated rather than resolved.** No measurement in the archive supports 0.20 A.
The only measured motor-side static friction is J01's **belt-off** breakaway, **0.2923 ± 0.0184 A**
(motor alone, no belt: CONSTANTS §8.2), and the belt-on plant can only add to that. On the
archive's evidence **0.29 A is the better-supported value, not an upper bound**, so G ≈ 48
counts (0.12°) is the more likely end. The ±20% friction plant scatter (§8.2) keeps this from
being decisive. The gate exception is recorded at the full 31–48 range.

**1. The drift stopped (the repeat test, pre-stated gate).** L4–L6 average −6.3 against L3's
−4.0: a change of −2.3 counts, and downward. The intercepts climbed L1 → L3 and then
flattened. **Wear is rejected.** A new pulley bedding in on its belt fits the pattern (this is
a hypothesis; bedding-in was not separately tested). **Single-ladder noise is ±7 counts** (sd
of L4–L6, n = 3, one session): G cannot be resolved finer than that from one ladder.

**2. Output position matters less than the B4 spread implied.** L3 and L4–L6 ran at different
output positions, and their intercepts agree within 2–3 counts. The "±22 counts of G per
ladder from position-dependent friction" estimate (from B4's 0.085–0.49 A range) overstated
the effect, at least across these two positions. That estimate was the basis of the
3-position method, which is reversed (§22.4.4, R17).

**3. The bigger swing is lower friction, not looseness (explains L1).** Swing at a fixed
current rises when friction falls, because more of the current goes into stretching the belt.
The drag_c drop of 0.12 A at ~168 counts/A gives **+20 counts**. So 50 + 20 ≈ 70 counts, which
matches the measured 65 / 77. The old intercept moves to −57.3 + 20 = **−37**, and L1 measured
exactly −37.0. **L1 is fully explained by lower friction.** A reject rule on raw swing at a
fixed current confounds friction with lost motion. That rule is R15. From L3 onward the swing
also carries the clamp (§22.5.4), so raw swing is not a backlash measure on this plant at all.

**How the L1 → L3 rise was resolved.** L3 − L1 was a near-uniform +33 to +43 counts at every
current, and two hypotheses fitted it. The S3 repeat separated them:

| Hypothesis (2026-09-27) | Result |
|---|---|
| Lost motion growing (belt or printed-tooth wear) | ❌ **Rejected**: L4–L6 are not above L3 |
| Clamp slop differed between sessions | ❌ **Not the intercept's cause**: a measured re-clamp (L3 → L4) moved the intercept only −3.2 counts. It moved the *slope* instead (§22.5.4). All J01 ladders used the same two-screw method |
| (added) Bedding-in of a new pulley on its belt | ✅ **Consistent**: rise over the first three ladders, then flat. Not independently tested |

L2's + side crept through its run (36 → 43 counts), which also fits bedding-in.

**Size:** 31–48 counts = 45–70 µm of belt = **0.07–0.12° at the output** (34.4 mm pulley radius),
roughly 0.2–0.3 mm at the foot. J02 was accepted at −6 to +13 counts (at a higher I_f, see
§22.5.5).

### 22.5.4 The clamp moves stiffness, not backlash

| | L1–L3 (before re-clamp) | L4–L6 (after) | Change |
|---|---|---|---|
| Slope, mean (cnt/A) | 167.1 | 186.9 | **+12%** (L3 → L4 alone: +8.3%) |
| k_beltline, mean (kN/m) | 57.6 | 51.5 | **−11%** |
| Intercept | −4.0 (L3) | −6.3 | −2.3 counts: no change beyond the ±7 noise |

The ladder measures the clamp's own compliance **in series** with the belt, so a looser or
differently seated clamp reads as a higher slope and lower k. The intercept, which is the
backlash deliverable, is insensitive to it. (The session summary quoted "~8%"; that is the
L3 → L4 step. Mean-to-mean it is 12%.)

**Consequences:**

- **k is recorded with its clamp.** A ~10% re-clamp effect on one joint, same method, is too
  large to leave untagged. J01's k is **50–60 kN/m, clamp-dependent**, not a single number.
- **Every re-clamp is a new variable.** This is the measured reason the 3-position ladder
  method was reversed: each position change needs the output unscrewed.
- **Cross-joint k and B6b comparisons carry the clamp.** Because clamps are per-assembly
  (§22.5.7), expect ~±10% in k from the clamp alone. That is ~±5% in ring frequency (f ∝ √k),
  about a third of D15's 15% gate. The ring was **not** re-run after the re-clamp, so J01's
  64.3 Hz is at the S2 clamp.
- §22.3's "< 80 Hz → the clamp may be the spring" rule stays superseded (it was built on the
  retracted 100–230 Hz band). The fixture test that passed was on the glued plant. **Its
  add-stiffness test has never been run on a screwed clamp**, and the 11% shift says the screw
  clamp is not fully out of the loop for the ladder.

### 22.5.5 Geometry and the drag–backlash trade-off — WITHDRAWN (R21)

*Replaced 2026-09-30.* This section attributed recipe A's extra backlash to the −0.12
compensation (+27 counts predicted, ~56 measured cross-joint at matched I_f) and derived a
trade-off line of +14 / ~+28 counts of G per −0.01 mm. **Recipe B, at the same −0.12, measured
G ≈ 6 counts** (§22.6.3). The extra backlash was recipe A's toolpath, not the compensation value,
so the backlash half of the trade-off line is withdrawn. The drag half (≈ −0.06 A per −0.01 mm,
J01-P10 → J01-P12) stands as a same-recipe comparison: **both were 0.6-profile prints**
(⟨V24⟩ answered 2026-09-30, §22.6.6). It has not been measured on the 0.4 profile.

### 22.5.6 Decision — −0.12 locked with a gate exception — CLOSED 2026-09-30

*Replaced.* The owner locked −0.12 on 2026-09-28 with a visible B6a gate exception (G 31–48
counts against ≤ 20), pending the control-side test D17. **The exception closed on 2026-09-30:**
recipe B at −0.12 passed B6a at G = 5.8 counts (§22.6.3), and D17 is demoted (§22.6.7). The
decision to keep −0.12 stands and is now unconditional.

### 22.5.7 Clamp standard → §22.6.7

### 22.5.8 B11 record → §22.6.8 (the row was updated for recipe B on 2026-09-30)

### 22.5.9 Breakaway structure (B4)

| Component | Formula, per rotor position | Value |
|---|---|---|
| Symmetric (friction proper) | (I₊ + I₋)/2 | 0.29 ± 0.07 A |
| Direction-biased | (I₊ − I₋)/2 | up to **±0.18 A** |

- A direction-biased component that varies with angle is a torque that helps one direction and
  resists the other. That looks like **cogging or position-dependent preload, not friction**.
  **Moderate confidence:** 10 readings ~37° apart cannot resolve a cogging period of a few
  degrees (D16).
- Breakaway correlates with the jump distance at release, **r = 0.85**: stick-slip, stored
  energy releasing all at once.
- **Sim consequence (proposed, not adopted):** one friction constant will be wrong. Candidate:
  Coulomb randomised over 0.2–0.4 A plus a position-dependent ripple of ±0.18 A. The ripple is
  about ±8% of standing-load current (at the §22 B3 scale), too large to leave out.

### 22.5.10 Ring (B6b)

- f_d 64.3 ± 0.4 Hz over 8 captures, all at the S2 clamp and from one boot. This sits inside
  §22.3's 59–72 Hz band.
- The ladder slope (k_beltline ≈ 57 kN/m → k_motor = k·r₁² ≈ 0.83 N·m/rad) predicts ~32 Hz with
  J_rotor. The ring runs at about 2× that frequency, **≈ 4.6× in stiffness**: the small-amplitude
  ring is stiffer than the large-excursion ladder. §22.4.7 found the same direction on J02 (5–12×).
- f_n/3 ≈ 23 Hz. **Not carried into CONTROL.md §10** — D1 stays open (see D1 for why).

### 22.5.11 Fleet consequence → §22.6.7

### 22.5.12 Retractions and reversals

| # | Claim | Status | Reason |
|---|---|---|---|
| R15 | "Swing at 0.6 A > 70 counts → too loose, reject the pulley" | ❌ Retracted | Confounds friction with lost motion. Lower friction alone added +20 counts (§22.5.3) |
| R16 | "Breakaway is always higher than phase 5's drag_c" (firmware comments, `autocalib.h`, `joint_cal.h`) | 🔁 Belt-off only | Belt-on J01: mean breakaway 0.288 = drag_c 0.291. Comments and the phase-5 print corrected 2026-09-28 |
| R17 | "B6a G = mean over ladders at 3 output positions" (§22.4.4 method change, morning of 2026-09-28) | ↩️ **Reversed** the same day | Each position change needs the output unscrewed, and a re-clamp moved the slope 8–12% (§22.5.4): it adds a variable. The premise (±22 counts per ladder from position) was not borne out: L3 and L4–L6 at different positions agree within 2–3 counts |
| R18 | "Position-dependent friction explains most of the L1–L3 spread" | ❌ Withdrawn | The spread was bedding-in plus friction; position was not visible in the intercepts (§22.5.3) |
| R19 | "The extra backlash matches the geometric prediction (gap 25–40 vs 27 counts)" | 🔁 Reframed | That compared G at different I_f. At matched I_f the gap is ~56 counts, ~2× the prediction (§22.5.5) |
| R20 | "I_f = 0.29 A is an upper bound; motor-side friction ≈ 0.20 A" | ⚠ Unsupported | 0.20 A has no measurement behind it; the only measured motor-side static friction (belt-off breakaway) is 0.2923 A (§22.5.3) |

### 22.5.13 What stays open → §22.6.10

⟨V16⟩ (recipe-A raw rows) is **closed by owner decision**: only the latest acceptance (recipe
B) is archived, to avoid redundancy. Every recipe-A number in this section therefore stays
transcribed-from-summary, with no raw backing. ⟨V18⟩ and ⟨V20⟩ are answered in §22.6.

---

## 22.6 J01 — pulley recipe B (0.4 mm nozzle profile): closure (2026-09-30)

**What opened it.** After a CrealityPrint update the owner found that the filament presets
and the custom pulley process had been living under the **0.6 mm nozzle** machine profile,
although every setting in them (line widths, flow ratio, …) was tuned for the 0.4 mm nozzle
actually fitted. All presets were copied onto the 0.4 profile and the same −0.12 + precise-wall
pulley was reprinted under otherwise identical conditions, as a clean A/B.
Caliper before fitting: tip OD within 0.05 mm of recipe A (inside scatter), boss a snug fit.

**What closed it.**

1. **Recipe B passes all three acceptance tests** against recipe A (§22.6.3–§22.6.5).
2. **B6a passes outright: G = 5.8 counts** (3 ladders, 2 boots) against recipe A's 31–48.
   **The 2026-09-28 gate exception is CLOSED**, and D17 is demoted.
3. **Recipe B is the pulley recipe of record**, identified by the sliced G-code's md5 (⟨V22⟩).
4. **B11 complete on recipe B:** `breakaway_A` 0.295 A, and B3 run in both orders the same
   day: drag_c **fwd 0.317 / rev 0.333 A**, pooled across orders (§22.6.6). Every J01 belt-on
   field now describes the plant that exists.
5. **§22.5's attribution of the extra backlash to the −0.12 compensation is withdrawn (R21).**
   The same compensation on a correct toolpath gives ~6 counts.
6. **Clean single-variable A/B** (owner-confirmed): same plate assembly, belt and idlers as
   2026-09-28; only the pulley changed. **Every pulley before J01-P12B was sliced on the 0.6
   profile**, including J02's SCB (§22.6.6).

> ⚠ **Provenance.** Raw logs are archived in `docs/cal/pulley acceptance/`: the ladder
> (3 runs, 2 boots), breakaway (20 `M4,` rows), ring (8 bursts, `cap=` 1–4 and 6–9) and
> dynamic-drag (V, 1, 3, 5 in both orders, plus the belt-on phase-4 attempt) files. The drag
> fits are the firmware's own; pooling was done here.
> Here, the ladder G values and the breakaway statistics were **recomputed from those rows**,
> and they match the session analysis. The ring fits (damped sine, amplitude regression) are
> the **session analysis's offline fits**, not refit here.
>
> **The binary flashed for these runs is older than the current source.** The logs show
> `[SWING] ladder 1/2/3 A` and `belt=OFF`, and no `!! long pre-slide creep` warning on the
> `travel −240` reading. The source has printed the currents since 2026-09-28, carries the
> belt-on J01 row, and warns above 200 counts. Measurements are unaffected: the ladder,
> M4 ramp and kick logic are the same. **The build that produced them is not identified (⟨V23⟩).**

### 22.6.1 Recipe of record — B

**Output pulley 108T GT2, recipe B, accepted 2026-09-30 on J01.**

| Item | Value |
|---|---|
| Slicer | CrealityPrint V7.3.0.6149 |
| Printer profile | `Creality K2 Pro 0.4 nozzle` |
| Process | `0.20mm Standard @Creality K2 Pro 0.4 nozzle FOR PULLEY` |
| Filament | `BambuLab PLA Tough+ @Creality K2 Pro 0.4 nozzle` |
| Geometry settings | `nozzle_diameter = 0.4`, `xy_contour_compensation = −0.12`, `xy_hole_compensation = 0`, `precise_outer_wall = 1`, `precise_z_height = 1`, `wall_generator = arachne`, `wall_loops = 4`, `layer_height = 0.2` |
| Line widths | outer 0.40 / inner 0.48 / infill 0.42 / top 0.48 |
| Arachne thresholds | `min_bead_width = 85%` (→ 0.34 mm), `min_feature_size = 25%` (→ 0.10 mm) |
| Print rule | Solo per plate |
| Boss | CAD **15.26**, printed **14.95** (calipers, owner 2026-09-30); snug on the bearing. Effective −0.155 mm/side on this convex cylinder (compare §22.4.3's −0.175 for nominal −0.10) |
| Tip OD | Within 0.05 mm of recipe A (value not recorded) |
| **Identity** | **md5 of the complete sliced `.gcode` ⟨V22⟩**: `certutil -hashfile <file>.gcode MD5` |

**Superseded: recipe A.** Same process settings, sliced under the `Creality K2 Pro 0.6 nozzle`
machine profile, used through 2026-09-28. Differences found in the session analysis (from
G-code excerpts, not archived):

- **Arachne thresholds are percentages of `nozzle_diameter`.** Under A, `min_bead_width`
  resolved to 0.51 mm and the slicer generated inner-wall beads up to **0.76 mm** wide, from a
  physical 0.4 mm nozzle. So copying the process settings does **not** reproduce A's toolpath.
- Retraction 1.5 vs 0.8 mm; exhaust fan 0 vs 60%; overhang optimisation off vs on;
  `internal_bridge_speed` 70 mm/s vs 70%.

> **Rule: a pulley recipe is identified by the md5 of its sliced G-code, not by its settings.**
> A settings list can match while the toolpath differs, which is exactly what happened here.
> Archive the md5 with every recipe change. Re-slicing with a different machine profile is a
> new recipe and needs a new acceptance.

### 22.6.2 Plant and session

| Item | Value |
|---|---|
| Configuration | Idlers fitted at (1.72, ±10.00), top plate on, **no top screw**, leg off |
| Pulley | **J01-P12B** (recipe B). Recipe A's J01-P12 removed |
| Plate assembly and belt | **The same as 2026-09-28** (belt 104.85, same idlers). Only the pulley changed: owner-confirmed 2026-09-30, ⟨V21⟩ closed. The 104.7-belt / second-plate plan was not used |
| Output clamp | Ladder and ring: **2 × M3×10 countersunk into the top plate, perpendicular** (fleet standard, §22.6.7). Breakaway: output free |
| Kick | TORQUE(V) 0.2 → 0.8 V, the same as the 2026-09-28 ring (B6b row, §22) |

| Boot | UT89X | Banner | Δ |
|---|---|---|---|
| 1 (ladder L1, ring, breakaway) | 12.32 V | 12.34 V | +0.02 ✅ |
| 2 (ladders L2, L3) | 12.29 V | 12.31 V | +0.02 ✅ |

### 22.6.3 B6a swing ladder — PASS

| Ladder | Boot | Intercept (cnt) | Slope (cnt/A) | k_beltline (kN/m) | Max \|repeat\| | **G at I_f 0.295** | Largest I_f with G ≤ 20 |
|---|---|---|---|---|---|---|---|
| L1 | 1 | −39.7 | 152.4 | 63.1 | 4 | **5.3** | 0.392 A |
| L2 | 2 (+~20 min) | −40.1 | 148.3 | 64.9 | 2 | **3.7** | 0.405 A |
| L3 | 2 | −34.5 | 145.8 | 66.0 | 2 | **8.5** | 0.374 A |
| **Mean** | | −38.1 | 148.8 | 64.7 | | **5.8 (sd 2.5) = 8.5 µm = 0.014° output** | |

- Firmware fits, read from the archived log. G = intercept + slope · I_f is computed here, with
  I_f = this session's B4 mean (0.295 A, §22.6.4), per §22.4.4. **≤ 20 gate: PASS on every
  ladder.** Across the I_f 95% CI (0.24–0.35 A) the worst ladder gives 16.5 counts, still inside.
- **Fresh-mesh check.** Recipe A's G climbed ~34 counts over its first three ladders as it
  bedded in (§22.5.3). Recipe B shows no climb across a reboot and ~20 min of extra motion:
  L1 → L3 differ by +3 counts, inside the ±7 single-ladder noise. What remains unchecked is a
  **later session** (A's largest rise came across sessions), which D2 covers.
- **vs recipe A at matched I_f 0.295:** A's stable ladders (L4–L6) give −6.3 + 186.9 × 0.295
  = **48.8 counts**. B gives **5.8**. **Δ ≈ −43 counts = −63 µm of belt = −0.105° at the output.**
- **Stiffness:** k 64.7 kN/m against A's 51.5 (L4–L6) and 57.6 (L1–L3). The clamp alone moved
  A's k by 11% (§22.5.4), and B was clamped fresh, so **k is not attributed to the pulley.**
  The slope also fell 152.4 → 145.8 (k +4.5%, ~1.5σ) over the three ladders. Noted, not gating.
- **k convention (corrected 2026-10-01, R26):** the k values above are the firmware's
  pre-2026-10-01 print, Kt 0.026912 (`calKt`) × **reported** amps, which is true k × `i_scale`.
  **True belt-line k = k / 0.9621: 65.6 / 67.5 / 68.6, mean 67.2 kN/m.** Every k in §22 is in
  the same convention, so comparisons and ratios within §22 stand. The ladder now prints both
  (`autocalib.h`, through `irepToTorqueOut()`). *(The "62.4 / 64.1 / 65.2 kN/m at CONSTANTS'
  0.0266" line that stood here is deleted: 0.0266 is the pre-M1 Kt.)*

G sensitivity to I_f (L1): −27.5 (0.08 A) · −9.3 (0.20) · 4.5 (0.29) · 21.2 (0.40) · 32.6 (0.475).

### 22.6.4 B4 breakaway — PASS (no difference resolvable)

10 positions × 2 directions, raw 266–15504 (about one motor revolution), output free.
Recomputed here from the 20 archived `M4,` rows:

| | Mean | SEM | Range |
|---|---|---|---|
| All (n = 20) | **0.295 A** | 0.027 | 0.080–0.475 A |
| + direction | 0.314 A | 0.039 | |
| − direction | 0.277 A | 0.038 | |

- Direction difference 0.037 ± 0.049 A: not significant.
- **Position dominates the variance:** pair means range 0.12–0.42 A (sd 0.095).
- **One reading carries the long-creep flag:** raw 8411, −, 0.475 A, travel −240 counts
  (> 200 = biased high, §22 B4). The flashed binary did not print the warning (⟨V23⟩).
  Excluding it, the mean is **0.286 A**. `breakaway_A` stores the full-set 0.295, with this
  noted; the two differ by less than one SEM.
- **Comparison, corrected (R22):** the session analysis compared this 0.295 against 0.291,
  but 0.291 is recipe A's **phase-5 `drag_c`** (dynamic), not its breakaway. The matched pair is
  **breakaway 0.295 (B) vs 0.288 (A): Δ = +0.007 A.** The ±0.03 A equivalence band pre-set for
  this test is **not resolvable** at n = 20 (95% CI ≈ ±0.055 A). The result is "no difference
  seen", not "equivalence proven".
- Output torque at 0.295 A reported = **0.0743 N·m true** (0.295 × `calKtCmd()` 0.027972 × 9,
  i.e. `irepToTorqueOut(0.295)`; η excluded by the contract's definition). *Corrected
  2026-10-01 (R26): the 0.071 here used the pre-M1 Kt 0.0266 and no `i_scale`.*

**Method change for fleet A/Bs (adopted into §22 B4 and CALIBRATION M4):** take breakaway at
**fixed raw positions** and compare **pairwise**. With position sd ≈ 0.095 A, independent
position sets swamp any pulley-level difference.

### 22.6.5 B6b ring — PASS

TORQUE(V) 0.2 → 0.8 V, 13.86 kHz capture, 4 + 4 bursts. Damped-sine fits over 60 ms from the
first overshoot (session analysis).

| | Value |
|---|---|
| f_d, all 8 | **65.0 Hz** (SEM 1.0) |
| f_d, + kicks | 67.6 Hz (A₀ ≈ 18 counts) |
| f_d, − kicks | 62.4 Hz (A₀ ≈ 28 counts) |
| f_n | 69.8 Hz |
| ζ | 0.36 (+0.31 / −0.41) |
| Amplitude slope | −0.47 ± 0.08 Hz/count (R² 0.87), consistent with §22.3's −0.522 |
| f_d corrected to A₀ = 23 counts | 64.9 Hz |
| **vs recipe A (64.3 ± 0.4 Hz)** | **Δ = +0.7 Hz, PASS (≤ 1 Hz)** |

- The ± split is amplitude softening, not a direction effect: the − kicks rang larger.
- Burst 1 (the first motion after reassembly) rang ~2 Hz below the later + kicks.
- Recipe A's ring amplitudes were never recorded, so the amplitude-corrected comparison assumes
  a similar A₀. That assumption is supported by the same kick being used, not measured.
- `cap=5` was not dumped. The 8 archived bursts are caps 1–4 and 6–9 (⟨V25⟩, cosmetic).

### 22.6.6 B3 drag on recipe B, and the two owner answers

**Session (same day, output free):**
- `V`: 0.62° elec from stored ✅.
- Phase 1: 4000/4000, perr 0.
- Phase 3: R 0.22404 Ω (+0.26% on the stored 0.22346), U0 0.01462, drift 0 at all 9 points.
- Banner 12.30 vs UT89X 12.28 V ✅.
- The log still prints the old phase-5 text ("STATIC breakaway threshold is larger … run M4 by
  hand"), so this is **the same pre-2026-09-28 binary** (⟨V23⟩).

| Run | Order | Forward: Coulomb A + viscous mA/(rad/s), rms | Reverse: Coulomb A + viscous, rms | Second minus first |
|---|---|---|---|---|
| 1 | Forward first | 0.2265 + 4.009, 7.7 mA | 0.3576 + 2.597, 9.0 mA | **+0.131 A** |
| 2 | Reverse first | 0.4072 + 2.466, **13.9 mA** | 0.3084 + 2.730, 9.7 mA | **+0.099 A** |
| **Pooled over orders** | | **0.3169 + 3.24** | **0.3330 + 2.66** | asymmetry 5.0% |

- **The order effect is larger than the direction effect.** In both runs the direction run
  **second** reads 0.10–0.13 A higher. The raw points agree, so it isn't a fit artefact. At
  ~22 rad/s the first direction reads 0.315 / 0.357 A and the second 0.424 / 0.472 A. The
  effect shrinks with speed.
- **It is not simple warm-up drift.** Warm-up would rise monotonically, but run 2's first half
  (rev 0.308) reads below run 1's second half (rev 0.358). Hypothesis, untested: the second
  direction starts straight after 102 rad/s the other way (belt re-seating on the opposite
  flank, or the grease film), while the first starts from rest. J02 showed the same kind of
  order dependence at a smaller size (0.04–0.08 A, no consistent winner, §22.4.8). Deferred as
  **D18**.
- **Stored in `joint_cal.h`: each direction pooled across both orders**, which cancels the order
  effect: drag_c fwd 0.3169 / rev 0.3330 A, drag_v fwd 3.24e-3 / rev 2.66e-3. Mean
  **0.325 A**, inside J01's 0.30–0.40 band. **Convention change:** recipe A's row stored the
  first run only (0.291). Pooled is used because the robot runs warm and reverses constantly.
- **vs recipe A:**
  - First run: B **0.292 A** (firmware 5-point fits) or 0.271 (top-3-speed refit here) vs A's
    0.291. Recipe A's exact fit convention was never recorded.
  - Pooled: B 0.325 vs A's 7-run pool 0.293 ± 0.021, i.e. +0.032, ~1.5 sd, with n = 2 runs on B.
  - **Verdict: no drag change attributable to the recipe is resolvable.** Every difference sits
    inside the ±20% plant spread (CONSTANTS §8.2).
- **Breakaway vs drag, belt-on (R16):** breakaway 0.295 against drag_c 0.292 first-run /
  0.325 pooled. Breakaway is *not* larger belt-on, now on both recipes.
- **Ke** 0.018064 / 0.018070 is +0.7% on the belt-off 0.017941. That is the familiar
  drag contamination (§22.1 Finding 3), so it is not carried.
- **Fit quality:** the rms is 7.7–13.9 mA. Run 2's forward fit exceeds J02's 10 mA prediction,
  the same pattern as D10.

**Phase 4 belt-on: a physical limit, not a code bug.** The owner reported that phase 4 "wouldn't
start, the rotor failed to lock (small twitch)". The code has no refuse-to-start path: phase 4
is a 600 ms lock plus 32 fast steps, **under a second in total**, so the twitch *was* the run. It
completed (reps 32/32) and returned **WARN: drift −169 counts** (limit 40), with L 44.7 µH.
- **Mechanism:** the step train cycles the magnetic self-lock between ~0.8 A and ~3.2 A. With
  the belt on, ~0.3 A of friction plus belt wind-up let the rotor sit off-axis at the weak hold
  and get pulled in at the strong one, so it ratchets.
- **Why phase 3 is immune:** it sweeps strongest-first, and then friction holds the rotor still
  (drift 0 in the same session).
- **L 44.7 µH is +2.1% on the belt-off 43.77**, the same +2.1% seen belt-on on 2026-08-12 (§22).
- **Phase 4 is BELT-OFF ONLY** (R25). L is a motor constant and stays carried from belt-off.
  B2 was always phases 1 and 3 only. The firmware header, the `y` menu and the phase-4 comment
  are corrected.

**⟨V21⟩ answered: same plate assembly and belt.** The A → B comparison is single-variable, so the
G drop (48.8 → 5.8 counts at matched I_f) is the print recipe.

**⟨V24⟩ answered: every pulley before J01-P12B was sliced on the 0.6 profile.** That covers
A, A2, B, C, SC1, **SCB (J02's fitted pulley)**, FV1, FV2, STD, J01-P10 and J01-P12. Consequences:
- **The backlash cost is a recipe × compensation interaction, not compensation alone.** On the
  0.6 profile, −0.10 gave low G (J02 SCB −6 to +13; J01-P10 near zero, estimated) and −0.12
  gave 31–48. On the 0.4 profile, −0.12 gives ~6. So the wide Arachne beads cost backlash only
  at the larger compensation (R21, refined).
- **The drag trade-off stands as a same-recipe comparison:** J01-P10 → P12 (both 0.6 profile),
  −0.02 mm/side → −30% drag_c. It has not been re-measured on the 0.4 profile.
- **Every print-geometry number in §22.4.3 is a 0.6-profile number:** the effective offsets
  (−0.175 / −0.115 mm/side), the tip-OD fingerprint (−0.20 to −0.26), M-B 69.60, and D6's
  acceptance criteria. Recipe B's boss reads −0.155 mm/side at nominal −0.12. **Re-baseline the
  fingerprint on a recipe-B print before using it as a fleet check (D5).**
- **J02 still carries a 0.6-profile pulley (SCB).** Its §22.4 acceptance describes that pulley.
  D12's "+0.35 boss prints 15.10" prediction was derived from 0.6-profile behaviour and is
  **void** for recipe B. When J02's board is replaced, reprint on recipe B and re-accept before
  D15.

### 22.6.7 Consequences

- **B6a gate exception: CLOSED.** J01 passes ≤ 20 counts on recipe B. The gate stays the
  J02-derived ≤ 20 counts (§22.4.4), which recipe B meets with ~14 counts to spare.
- **D17 (control-side backlash test): DEMOTED.** Promote if any fleet joint fails G ≤ 20, or
  before the B6a gate is re-derived from controller tolerance.
- **Trade-off line: backlash half WITHDRAWN (R21).** The drag half (≈ −0.06 A per −0.01 mm)
  stands for the 0.6 profile only (J01-P10 and P12 were both on it); it is unmeasured on recipe B.
- **Fleet pulley bins (§22.5.11's interpretation, restated).** The idler holes are fixed
  fleet-wide at (1.72, ±10.00), so belt-length variation is absorbed by pulley compensation.
  Recipe B shows that −0.12 need not cost backlash, which removes the main argument *against*
  more negative bins. It does not settle the bins: that still needs the 20-belt histogram (2
  d.p. protocol, §22.3) and one short-belt joint (D4), all on recipe B.
- **Clamp standard (moved here from §22.5.7).** The shared jig proposed on 2026-09-28 is rejected:
  the clamp holes belong to each top-plate assembly, so no jig can register to them. **What is
  standardised is the screws: 2 × M3×10 countersunk into the top plate, perpendicular to each
  other** (owner 2026-09-30, ⟨V20⟩ closed). Cross-joint k and ring comparisons still carry a
  per-assembly clamp term of ~±10% in k (§22.5.4).

### 22.6.8 B11 record — J01 belt-on row (recipe B)

Updated in `joint_cal.h` 2026-09-30. `-e J01` flashes it. The belt-off baseline row stays
verbatim at index 13.

| Field | Value | Source / status |
|---|---|---|
| date | `"2026-09-30"` | recipe-B acceptance |
| belt | `"10mm-9:1"` | |
| zea, dir, R_eff, U0, Ke, L, vbus_scale, i_scale | unchanged | carried (belt-independent; belt-on Ke is never carried; belt-on L is not measurable, R25). 09-30 tripwires: V 0.62° elec, R 0.22404 |
| drag_c fwd, rev | **0.3169, 0.3330 A** | **recipe B**, B3 2026-09-30, each direction pooled over both run orders (§22.6.6) |
| drag_v fwd, rev | **3.24e-3, 2.66e-3 A/(rad/s)** | **recipe B**, same |
| **breakaway_A** | **0.295 A** | **recipe B, B4 n = 20** (0.286 excluding the one long-creep reading) |

**Not fields:** recipe B md5 ⟨V22⟩; boss 14.95 (CAD 15.26); G 5.8 counts; k 64.7 kN/m archive convention = 67.2 true (at its
clamp); f_d 65.0 / f_n 69.8 Hz; clamp 2 × M3×10 countersunk.

### 22.6.9 Retractions

| # | Claim | Status | Reason |
|---|---|---|---|
| R21 | "Recipe A's extra backlash is the −0.12 compensation geometry", and the trade-off line of +14 / ~+28 counts per −0.01 mm (§22.5.5) | ❌ Withdrawn | Recipe B at the same −0.12: G 5.8 vs 48.8 at matched I_f. The backlash came from the 0.6-profile toolpath **at −0.12**: the same profile at −0.10 gave low G on both joints (§22.6.6) |
| R22 | "B drag_c 0.295 vs A 0.291: pass" (session analysis) | 🔁 Corrected | That compared B's **breakaway** with A's phase-5 **drag_c**. The matched pair is breakaway 0.295 vs 0.288. B's drag_c, measured afterwards: 0.292 first-run / 0.325 pooled (§22.6.6) |
| R25 | "Phases 1, 3, **4** are locked-rotor and belt-agnostic" (CHANGELOG F7, `autocalib.h` header and `y` menu) | ❌ Falsified for 4 | Belt-on, phase 4's step train ratchets the self-lock: J01 drift −169 counts, WARN. 1 and 3 stand (phase-3 drift 0). Phase 4 is belt-off only |
| R23 | "`outer_wall_line_width` differed between recipes A and B" (session analysis) | ❌ Retracted | Both configs are 0.4. The 0.42 layer-1 outer wall in B was Arachne output |
| R24 | "Copying the process settings onto the 0.4 profile reproduces the old pulley" (implicit in the A/B framing) | ❌ False | Arachne thresholds scale with `nozzle_diameter`, so the toolpath changed. Hence the md5 rule (§22.6.1) |
| R26 | §22.6.3 "at CONSTANTS' 0.0266, k = 62.4 / 64.1 / 65.2 kN/m"; §22.6.4 "output torque ≈ 0.071 N·m (× 0.0266)" | 🔁 Corrected 2026-10-01 | 0.0266 is A1's pre-M1 Kt, and CONSTANTS §8.1 was still carrying it as a stored value (now a pointer to `calKt()`). J01 Kt = 0.026912; per reported amp `calKtCmd()` = 0.027972 → **0.0743 N·m true**. The firmware's own k (64.7) was Kt × reported amps = true k × `i_scale`; **true k 67.2 kN/m** |
| R27 | B12 scoping: lost motion of 0.25 mrad at the output is "below the encoder's practical noise band" | ❌ Wrong reason | G = 5.8 is in **motor-encoder** counts and the ladder resolved it. A bare-output B12 cannot test lost motion because the belt is outside the loop dynamics (output pulley ≈ 1% of rotor inertia), not because of resolution |
| R28 | B12 scoping: "1.6 A ≈ 0.38 N·m at the output, saturating at ~9 mrad" | 🔁 Corrected | Stale Kt and no `i_scale`. 1.6 A_rep × 0.027972 × 9 = **0.403 N·m true**, so saturation is at **9.8 mrad** for kp = 41 N·m/rad |
| R29 | a4 design (§22.7.12): "eight start positions sample the washboard; judge the row on the mean", and the simulated spreads built on random phase | ❌ False premise | The rotor always rests in the same 14/turn detent phase (O4, §22.7.13). Every step started there, so the mean tests the detent-start response. Re-simulated from the measured starts, the model matches |
| R30 | a4 row 1 session analysis: O3's roll-back is "~70% spring in series with output friction, ~25% pre-sliding; cogging ruled out (9/9 against step direction)" | ❌ Withdrawn | The ruling-out assumed random start phase. With every start in the detent (O4), a washboard pulling back against every step is exactly what cogging predicts. Belt wind-up at the ladder stiffness is ≤ ~1 mrad (§22.7.13) |

### 22.6.10 What stays open

**Owner confirmations:**
- ⟨V22⟩ recipe B G-code md5.
- ⟨V23⟩ which build was flashed. Resolves itself once the current source is flashed.
- ⟨V25⟩ whether `cap=5` matters (cosmetic).
- *Closed:* ⟨V21⟩ same plate and belt; ⟨V24⟩ all earlier pulleys on the 0.6 profile.

**Bench:** none left from this detour. B3 on recipe B was done 2026-09-30.

**Deferred:**
- D18: the drag run-order effect.
- D2: a ladder repeat in a later session.
- D4: short-belt joint, on recipe B.
- D15: the J01-vs-J02 ring comparison, which needs a J02 board.
- D16: cogging vs preload in the breakaway spread.
- D17: now demoted.
- D1: the Tier-2 ceiling.

Status: README §15.

---

## 22.7 B12a — MIT law on a bare output (J01, in progress, 2026-10-01)

Plant: J01, recipe-B pulley, belt on, **output pulley bare and free**. Firmware: the B12a contract build (C1–C8 plus the 2026-10-01 (c) line-ending fix), `-e J01`. Banner `Kt_cmd=0.027972`, `torque boundary: 1 A_rep = 0.2517 N.m`. Envelope: τ_max 0.39 N·m (outer), 1.6 A_rep (inner, compile-time). Vbus 12.25 V (banner).

### 22.7.1 Status

| Step | What it proves | Result |
|---|---|---|
| a0 | Speed-filter choice | ✅ **`Tf_mit` = 1.0 ms** (the firmware default stands) |
| a1 | Clamp chain + validator, disarmed | ✅ **PASS 22/22 on J01.** (The `-e J03` uncalibrated-row run is dropped: the env guard refuses another joint's env on the J01 board, by design) |
| a2 | Signs, clamps on a moving motor, voltage headroom | ✅ **PASS on every sign/safety criterion; auto-stop at 20 s verified by the owner during a2.** ✅ **Loop rate −4.7% vs ≤ 5% (2026-10-02)**, after the MIT trim. Prefetch measured no effect (§22.7.7). The a2 pulse re-ran clean on the new build. ✅ **"+" = counter-clockwise viewed facing the output-pulley shaft end** (the face pointing outward on the robot; owner, 2026-10-02) — closed |
| a3 | kd is a damper; v_des drives; ripple in current mode (D19) | ✅ **PASS.** Part 1: slope −0.0998, R² 0.998 (§22.7.9). Part 2: all six runs inside 0.8–1.8 rad/s, mean of three = +1.123 / −1.095 against drag-map predictions of 1.120 / 1.096; Uq ≤ 0.32 V; clamp 0 (§22.7.11). **D19 PROMOTED:** a 14/turn torque of 44–48 mN·m output, conservative and speed-independent. Also found: **O2**, friction that depends on output angle |
| a4 | Step timing vs kp (three stiffnesses) | ✅ **PASS on its aim (units end to end), 2026-10-02.** Mean stop time kp 41 **+1.6%**, kp 16.8 **−9.8%** (gates ±10%); kp 6.6 **+21%, gate ±15% missed** (n = 4). Rest errors, holding, Uq ≤ 0.31 V and clamp 0 pass in every row. The miss is the pre-registered washboard branch: re-run from the measured start counts, with no fitted parameters, the model gives 17.7 / 28.7 / 57.7 ms vs measured 19.5 / 28.0 / 61.2. **O4 (new, established):** the joint always rests in the same 14/turn detent phase, so the eight-position design never sampled the washboard (R29). O3 written; O1 narrowed (§22.7.13) |
| a5 | Saturation on a moving motor (outer, then inner clamp) | ⏭ **NEXT** — procedure §22.7.14. The last B12a step |

### 22.7.2 a0 — speed filter (analysis 2026-10-01)

Six TORQUE(V) captures at ~15–17 rad/s motor (3 per direction), 13.4 kHz. Noise isolated two ways (residual above 250 Hz; quantisation simulation of the smoothed motion):

| Tf (ms) | Predicted q/(√12·Tf) | Simulated | **Measured** | Lag at 25 Hz |
|---|---|---|---|---|
| 0.5 | 0.22 | 0.200 | 0.226 | 4.5° |
| 0.7 | — | 0.148 | 0.167 | 6.3° |
| **1.0** | **0.11** | **0.105** | **0.120** | **8.9°** |
| 1.5 | — | 0.071 | 0.082 | 13.3° |
| 2.0 | 0.055 | 0.054 | 0.062 | 17.4° ❌ |

rad/s at the motor. At the largest planned kd (0.365 N·m·s/rad output, ζ 0.7 at kp 41): noise torque 0.120 / 9 × 0.365 = **4.9 mN·m = 70% of the 7 mN·m budget**; 0.7 ms would use 98%. **Decision: 1.0 ms.**

a0 also found a **position-locked speed ripple at 14.2 ± 0.3 per motor turn** (2 per electrical cycle), phase-locked to rotor angle, ~4.3–5.5 mN·m at the motor (~39–49 mN·m output), only ~30% of it visible as current ripple → **D19** (§22.4.14).

### 22.7.3 a1 — clamp chain, motor disarmed: PASS (J01)

`MIT CLAMP TEST PASS 22/22`, row J01, `Nm_per_A_rep=0.25174`. Hand-checked values: 0.10 N·m → **0.3972** A_rep (predicted 0.397); outer clamp ±1.0 → **±1.5492** (predicted 1.549); outer 5.0 → inner **±1.6000**; NaN / ±inf / τ_max ≤ 0 / NaN / env 0 / NaN → **0, REJECT**; validator rejects negative kp, negative and NaN kd, kp 1e6, p_des 10, v_des inf, τ_ff 2.

~~Open: the same test on `-e J03`~~ — **dropped 2026-10-02.** The firmware's env guard refuses to run a J03 build on the J01 board, which is the protection working. The uncalibrated-row behaviour stays covered by the REJECT rows above: Kt ≤ 0 takes the same `calKtCmd() > 0` branch. It will be exercised for real when J03 is assembled.

### 22.7.4 a2 — directions and safety regressions

Four τ_ff pulses of ±0.15 N·m, 60 ms, auto-revert to zero torque; kp = kd = 0; capture decim 2 (185 µs/sample, 185 ms).

| Cap | Pulse | Measured τ in pulse | v at revert | **Peak v** (output / motor) | Peak excursion | Net travel | Roll-back after stop | Peak \|Uq\| | Clamp |
|---|---|---|---|---|---|---|---|---|---|
| 1 | +0.15 | +0.14…+0.15 | +1.73 | **1.84 / 16.6** | +0.079 rad | +0.073 | 6.1 mrad | 0.44 V | 0 |
| 2 | −0.15 | −0.13…−0.15 | −3.09 | **3.15 / 28.3** | −0.198 | −0.165 | 32.2 mrad | 0.64 V | 0 |
| 3 | +0.15 | +0.14…+0.15 | +2.71 | **2.76 / 24.8** | +0.146 | +0.137 | 8.9 mrad | 0.58 V | 0 |
| 4 | −0.15 | −0.13…−0.15 | −2.68 | **2.82 / 25.4** | −0.161 | −0.138 | 22.9 mrad | 0.60 V | 0 |

Against the pass list:
- ✅ **Signs:** + pulse → τ, v, p all positive; − pulse all negative, on both repeats. The three signs agree, so the spring term is correct by construction. Iq reaches 90% of the 0.595 A_rep set-point in ~1.5 ms and drops to ~0 within ~1.5 ms of the revert.
- ✅ **Peak speed ≤ ~45 rad/s motor:** worst 28.3. ✅ **Uq < 2.0 V:** worst 0.64. ✅ **Clamp counters 0** (outer/inner/reject, every sample and every status line).
- ⚠ **Travel prediction missed low** (0.08–0.20 rad vs "0.2–0.4"). The prediction (R32) used the *lowest-friction* acceleration. Backing friction out of the measured pulse acceleration (J_out = 20.5×10⁻⁶ × 81, unmeasured belt-on): 0.097 / 0.064 / 0.075 / 0.074 N·m = **0.39 / 0.25 / 0.30 / 0.29 A_rep, mean 0.31 A** — against B3's drag_c 0.317 / 0.333 A. At that friction the predicted travel is ~0.14 rad, matching cap 3. Not a fault; a consistency check of the torque boundary and J (one equation, two unknowns — consistent, not proof).
- ❌ **Loop rate:** MIT armed **11,300–11,520 lps (≈ 11,420)** vs TORQUE(I) armed **12,425–12,471 (≈ 12,440)** = **−8.2%**, over the 5% criterion. Cost ≈ 7.2 µs per loop, several times the arithmetic in `mitService()` — the cause is not identified (flash wait-state/cache effects of the larger loop are a candidate). **Consequence for the predictions: transport delay rises 77 → 84 µs; a4's lag-inclusive predictions move < 0.5%.** Recorded as a deviation, not a blocker for a3; profile (DWT cycle counter around `mitService()`) before a4 if it is to be closed.
- ✅ **Auto-stop at 20 s in MIT mode:** verified by the owner during a2 (not in the pasted log). ⏳ **Still open:** the written physical "+" direction.

**Observation O1 — the joint rolls back after it stops, with zero torque commanded.** After every pulse the rotor coasts to a stop and then reverses, **opposite to its motion**, by 6.1 / 32.2 / 8.9 / 22.9 mrad output (larger after the − pulses), reaching up to +0.89 rad/s output in cap 2. Measured τ is ~0 throughout (|Iq| ≲ 0.01 A_rep; Uq just tracks back-EMF). **Mechanism not established.** The D19 washboard predicts roll-back opposite to motion, and its stored energy (~7×10⁻⁴ J) matches cap 2's kinetic energy — but its torque (~0.04–0.05 N·m output) is below the friction backed out above (~0.075), which it would have to beat. So ripple-alone does not close; belt elasticity has no load to store against on a bare output. Relevance: at kp = 0 the joint does not stay where it stops, which feeds a4's rest-error bound. a3 part 2 (constant driven speed) is the discriminating data.

### 22.7.5 a3 procedure (keys spelled out)

See §22.7.6 for what every key and number means. Session start: `f`, then `m;`. Every MIT line ends with Enter **or** `;`.

0. ~~Auto-stop check~~ — done in a2 (owner).
1. **Part 1 — damper by hand.** `m kd 0.1;` then `m b;` (no staged changes, so `m go` only records). `g`. **The damper is live from `g`**: A already holds kd 0.1 and arming keeps kp/kd, so the pulley is free only while it is *still* and resists any motion in proportion to speed (plus the ~0.08 N·m friction it always has). `m go 0 32;` changes nothing in the control — B equals A — it only starts the ~2.75 s recording. Type `m go 0 32;` and **start turning the output pulley the moment you press Enter**: ~1 s slowly, ~1.5 s faster, back and forth, never faster than ~½ output turn per second. Capture lasts ~2.75 s, and **the console is silent while it records** (status lines are suppressed during a capture), so it feels shorter than it is → `CAPTURE done` → `x` → `d`. Twice.
   **Pass:** measured `tau` vs v differentiated offline from `cnt`: straight line, slope −0.10 ± 10%; clamp 0; no buzz.
2. **Part 2 — driven speed.** `m kd 0.1;` → `m b vd 2;` (the reply must read `B = A + vd=2.0000 -> if m go now: … kd=0.1000 …`, with **no** `!!` line) → `g` (the pulley holds still: A has vd 0) → `m go 0 8;` (B takes over after 50 samples; ~0.68 s capture). **The motor keeps spinning after `CAPTURE done`** — B is now the live command — so press `x` promptly, then `d`. Three times (the staged vd 2 survives `g`, so each repeat is just `g` → `m go 0 8;` → `x` → `d`). Then `m b vd -2;` and three more. **Watch which way the pulley turns at vd +2 and write it down** — that is the "+" direction still open from a2.
   **Pass:** steady speed ≈ ±1.2 rad/s output (0.8–1.8); Uq < 2.0 V; clamp 0; no buzz. **Ripple:** speed wobble ≈ ±1–1.5 rad/s motor locked at 14/turn → promote D19; ≤ ±0.5 → close D19.

### 22.7.6 MIT command reference (as used in B12a)

| Typed | Meaning |
|---|---|
| `m …;` | Opens an MIT command line; the text up to Enter/`;` is the command |
| `m;` | Enter MIT mode (stopped), or print MIT status if already in it |
| **A** | The **live** command: the five numbers the law uses **every loop while armed**. Edited by `m kd 0.1;` etc. — the change takes effect immediately |
| **B** | The **staged change** (redesigned 2026-10-02): a list of fields to change, applied to A by `m go` at a known sample of a capture — **B = A + changes**, resolved when `m go` fires. It exists so a step (new pd, a ff pulse, a new vd) lands *inside* a recording with 50 samples of before-data, instead of whenever you happen to press Enter |
| `m kd 0.1;` | Set fields of the **live** command **A** (what the motor obeys now) |
| `m b;` | Clear the staged changes: B = A, so `m go` only records |
| `m b vd 2;` | Stage changes. **Each `m b` line replaces the previous one** (nothing stale carries over). Fields not named come from A when `m go` fires, so `m kd 0.1;` before or after `m b vd 2;` gives the same B. Applied **only** by `m go`; survives `g`. The reply prints what B would be now |
| `!! vd does nothing while kd = 0` | Printed after any A edit, `m b` or `m go` whose command multiplies a target by a zero gain (τ = kp(pd − p) + kd(vd − v) + ff). Same for pd with kp = 0 |
| fields | `pd` target angle (rad, output, relative to the arm point) · `vd` target speed (rad/s, output) · `kp` spring (N·m/rad) · `kd` damper (N·m·s/rad) · `ff` extra torque (N·m) |
| `m go R D;` | Start a capture; after 50 samples B replaces A. **R** = auto-revert to zero torque R ms after the switch (`0` = never). **D** = record every D-th control loop: at ~11.9 kHz (measured 2026-10-02), D 1 ≈ 84 ms, 2 ≈ 179 ms, 8 ≈ 0.68 s, 32 ≈ 2.74 s per 1000-sample capture |
| `g` / `x` / `d` | Arm (p = 0 here; A's pd/vd/ff cleared, kp/kd and the staged changes kept; prints A and what `m go` will apply) / stop / dump. Dump **after** `x`: it blocks the loop ~1 s. A pd/vd/ff typed while **stopped** never run — `g` clears them, and the reply says so |

### 22.7.7 The loop-rate deviation — measured, first diagnosis falsified (2026-10-01, rewritten 2026-10-02)

> **What changed (2026-10-02):** the 2026-10-01 version of this section diagnosed the loop as instruction-fetch bound and built flash prefetch as the fix. The prefetch build was benched (`prefetch=1 flash_ws=8` in CFG). **TORQUE(I) did not move, so prefetch is not the fix and "fetch-bound, fixed by prefetch" is withdrawn.** The a2 criterion now passes anyway, because of the MIT code trim made in the same build.

**Data** (lps from status lines; per-loop time = 1/lps):

| | Before (2026-10-01, prefetch=0) | After (2026-10-02, prefetch=1 + trim) | Change |
|---|---|---|---|
| TORQUE(I) armed | ~12,440 (80.4 µs) | 12,471–12,491 (80.1 µs) | **+0.3%** |
| TORQUE(I) disarmed | 66,300 (15.1 µs) | 65,528–65,607 (15.25 µs) | −1.1% |
| MIT armed | 11,420–11,600 (86.9 µs) | 11,867–11,956 (84.1 µs) | +3% |
| MIT disarmed | 54,790 (18.25 µs) | 54,830–54,993 (18.2 µs) | ~0 |
| **MIT vs TORQUE(I), armed** | **−8.2%** ❌ | **−4.7%** ✅ (gate ≤ 5%) | |
| MIT mean cost, armed / disarmed | 6.5 / 3.2 µs | **3.95 / 2.9 µs** | |
| `svc_us` (max per window) | — | 7.2–9.5 armed (one 13.2), 4.1–6.1 disarmed | |

**Reading it.**
- **Prefetch:** no effect. The TORQUE(I) changes (+0.3% / −1.1%) are inside the build-to-build spread that any code-layout change produces. The pre-registered prediction ("any rise confirms") was too loose to be falsifiable at this size. It is judged as no change.
- **The −2.5 µs armed gain** sits only in the armed MIT path, where the trim was made (`micros()` → DWT cycle counter; per-loop measured τ removed). Disarmed MIT, where the trim saved only the `micros()` call, moved ~0.
- **`svc_us` is a max**, so it includes interrupt preemption (the current-sense/PWM interrupts at 25 kHz land inside a 3–4 µs window every few loops). It reads ~2× the mean. **The mean cost is the lps difference.** The prediction "`svc_us` ≤ ~3 µs" was posed against the wrong statistic. Measured as a mean (3.95 µs armed) it is still ~30% over that figure.
- **a2 pulse re-run on this build** (`m b ff 0.15;` → `g` → `m go 60 2;`). Every prediction held: +ff gave +Iq and +v; Iq reached 90% of 0.596 A_rep at **1.60 ms** (a2: ~1.5 ms); clamp 0; |Uq| ≤ 0.50 V; the revert landed at **60.0 ms**.

**What is still unexplained.** Disarmed, `mitService()` runs ~40 instructions plus two float divides — about 150 cycles by count — yet costs **2.9 µs ≈ 500 cycles**. The ranked hypotheses:

| # | Hypothesis | Fits the data? | Discriminating test |
|---|---|---|---|
| H1 | **Misses on branch/call targets at 8 wait states.** Each taken branch to an uncached line pays the full ~9-cycle miss. Prefetch only hides *sequential* misses, which is why it did nothing. The loop's code far exceeds the ~1 KB ART cache. | Yes — cost scales with code touched, and prefetch is null | Set `FLASH_LATENCY_4`, which is in spec at 170 MHz in range-1 boost and which the variant already sets. Prediction if H1: TORQUE(I) armed lps rises ≥ 5% and the MIT disarmed cost falls to ≤ 2 µs. If neither moves, H1 is dead |
| H2 | The cost is outside `mitService()`: other `MODE_MIT` branches elsewhere in the loop | Weakly — `svc_us` min is not recorded | Accumulate a **mean** `svc` (Σcycles / calls) next to the max. Mean ≈ 2.9 µs → the cost is inside; ≈ 1 µs → it is outside |
| H3 | Code layout: adding MIT code shifts every function's alignment and ART hit pattern | Explains the −1.1% TORQUE(I) disarmed wobble | Not separable from H1 without the wait-state test |

**This dataset cannot separate H1 from H3.**

**Decision: accept, and stop here.** −4.7% passes the a2 gate. The loop-rate difference changes no B12a prediction by more than ~0.5%, and the loop is dt-aware throughout. The wait-state change is a clock-tree edit to a vendor variant, and a wrong value shows up as random hard faults, so it should not be done casually.

**Deferred with reason:** the flash wait states (8 → 4) and the H1/H2 tests. **Promoted when** the Tier-0 loop with CAN handling is measured below the rate its current loop needs, or when a loop-rate gap again fails a gate.

`FLASH_PREFETCH` stays `true`: it is harmless, and today's captures ran with it.

### 22.7.8 a3 — partial data from the status lines (2026-10-01)

The owner armed with kd 0.1, kp 0, and turned the pulley by hand (one direction) before taking a capture, and stopped because the pulley resisted. **The resistance is the damper working, not a bug:** A held kd 0.1 from `g` (§22.7.5 wording corrected). The status lines show it:

| v (output rad/s) | −0.62 | −1.56 | −2.22 | −2.36 | −2.91 | −3.22 | −3.33 |
|---|---|---|---|---|---|---|---|
| measured τ (N·m) | +0.065 | +0.149 | +0.223 | +0.240 | +0.288 | +0.317 | +0.318 |

Least-squares through the origin: **slope −0.098 N·m·s/rad** (target −0.100; individual ratios 0.095–0.104). The sign opposes motion; current reached 1.32 A_rep (0.32 N·m) with no clamp. Disarmed, the same hand-turning at up to 3.5 rad/s gave τ ≈ 0, showing the contrast. **Not the a3 pass measurement:** these are single status-line snapshots of filtered quantities, taken during changing speed in one direction. The captured, two-direction runs with offline velocity are the test — done, §22.7.9 (−0.0998, consistent with this −0.098).

### 22.7.9 a3 part 1 — damper by hand, captured: PASS (2026-10-02)

Two captures at decim 32 (1000 samples, **2.744 s and 2.715 s** by `t_us`, fs ≈ 365 Hz). kp 0, kd 0.1, B = A. Build: `prefetch=1 flash_ws=8`. The duration is correct. The owner's impression that the capture was short comes from the console going silent while it records.

Velocity is **differentiated offline from `p`** (central difference on the logged `t_us`, not the firmware's `v`). Fit over samples with |v| > 0.2 rad/s:

| Capture | Moving samples | v range (rad/s, output) | Slope through 0 | Free fit (slope, intercept) | Residual | R² |
|---|---|---|---|---|---|---|
| cap 2 | 416, negative only | −1.17 … 0 | −0.1000 | −0.1007, −0.5 mN·m | 2.9 mN·m | 0.984 |
| cap 3 | 234 positive / 299 negative | −1.43 … +1.52 | −0.0998 (+: −0.0999, −: −0.0998) | −0.0998, −0.1 mN·m | 3.2 mN·m | 0.999 |
| **pooled** | 949 | | **−0.0998** | intercept ~0 | 3.1 mN·m | **0.998** |

- **Pass** against −0.10 ± 10%: the slope is −0.0998 in both directions. Clamp 0 throughout. |Uq| ≤ 0.10 V. No buzz was reported.
- The firmware `v` agrees with the offline velocity to 0.025 rad/s RMS while moving. That is consistent with a0's 0.120 rad/s at the motor ÷ 9 = 0.013, plus differentiation noise.
- Measured τ tracks the commanded τ to 2.8 mN·m RMS, mean ~0. The current loop delivers the law.
- **What this does NOT prove:** measured τ is `irepToTorqueOut(Iq)`, the same Kt_cmd the command used. So the slope checks the **law, estimator, sign and current loop**, not the absolute torque. Absolute torque (Kt, η) needs an external reference (B12b or a scale), as it always did.
- Both captures start still (cap 2 for 1.28 s, cap 3 for 0.70 s) because turning began after Enter. For part 2 this does not matter, since the motor drives itself.

### 22.7.10 a3 part 2, first attempt — zero torque by construction; `m b` redesigned (2026-10-02)

**Observed:** the pulley did not move. The capture confirms that nothing was commanded: `tau_law` = `tau_cmd` = 0.0000 in all 1000 samples, before and after the switch at sample 50, clamp 0. The motor sat at rest (`cnt` 1455–1456, ±1 count of noise).

**Cause, read off the console (certain):** `MIT B: pd=0 vd=2.0000 kp=0.000 kd=0.0000 ff=0`. The law is τ = kd(vd − v); with **kd = 0, vd has nothing to act through**. B was a complete second command. `m b vd 2` edited whatever B last held, and after a reboot that is all zeros. The §22.7.5 procedure assumed B still held kd 0.1 from part 1's `m b;`, which is true only without a reboot in between. **The error was in the procedure, and the UI made it easy to make.** The law, estimator and clamp chain behaved correctly: zero in, zero out.

**Also in the log:** A showed `vd=2.0000` before `g`, because a vd typed while stopped is cleared at arm by design. Silent before; now a note.

**UI change (`open_test.cpp`; J01 121,112 B, 92.4%):**
- **B is a set of staged field changes, resolved against A when `m go` fires** (B = A + changes). `m b vd 2` now means "at go, set vd to 2". Fields not named come from A, so the order of `m kd` and `m b` does not matter.
- **Each `m b` line replaces the previous one.** A leftover from an earlier test, such as a2's ff, cannot ride along. `m b` alone clears the changes.
- `m go` re-validates the resolved B (A may have changed) and refuses if it is out of range.
- **`!!` warning** whenever a command multiplies a target by a zero gain (vd with kd = 0, pd with kp = 0), on A edits, `m b` and `m go`. It would have caught this attempt on the first line.
- A pd/vd/ff typed while stopped prints a note that `g` clears them.
- `g` prints A and what `m go` will apply. `m` status prints the staged changes and the B they resolve to now.
- Unchanged: the switch, revert, capture and dump formats. Every earlier procedure keeps its meaning: a2's `m b ff 0.15` and part 1's `m b`.

### 22.7.11 a3 part 2 — driven speed and D19 ripple: PASS; D19 promoted (2026-10-02)

**Runs.** kp 0, kd 0.1, B = A + vd ±2, decim 8 (1000 samples, ~0.69 s, ~1,440 Hz per capture). Three runs per direction; each run went on after its capture until `x`. The firmware build has the redesigned `m b` (§22.7.10); its reply showed the resolved B with kd 0.1 and no `!!` line. **"+" = counter-clockwise** as reported by the owner; the viewing face is still to note.

Velocity is differentiated offline from `p`. The steady window runs from 150 ms after the switch to the end of the capture.

| Run | v_des | Mean v (output rad/s) | sd | Peak-to-peak | Mean τ (N·m) | τ − τ_cmd RMS | Uq max | Clamp | Output window (rad, mod 2π) |
|---|---|---|---|---|---|---|---|---|---|
| 1 | +2 | **+1.290** | 0.127 | 0.61 | +0.071 | 3.3 mN·m | 0.31 V | 0 | 0.00 → 0.82 |
| 2 | +2 | **+1.242** | 0.147 | 0.75 | +0.076 | 3.2 | 0.31 | 0 | 4.56 → 5.34 |
| 3 | +2 | **+0.838** | 0.187 | 0.82 | +0.116 | 3.1 | 0.29 | 0 | 2.54 → 3.05 |
| 4 | −2 | **−1.315** | 0.135 | 0.60 | −0.068 | 3.2 | 0.32 | 0 | 1.14 → 0.31 |
| 5 | −2 | **−1.005** | 0.198 | 0.88 | −0.100 | 3.0 | 0.30 | 0 | 2.34 → 1.71 |
| 6 | −2 | **−0.966** | 0.177 | 0.79 | −0.103 | 3.1 | 0.30 | 0 | 4.35 → 3.72 |

**Pass, on every criterion.**
- **Speed:** all six runs fall inside 0.8–1.8 rad/s; run 3 at 0.838 is the closest to the edge.
- **Voltage and clamps:** Uq ≤ 0.32 V against the 2.0 V limit, and clamp 0 throughout.
- **Current loop:** measured τ tracks the commanded τ to about 3 mN·m RMS, and |Id| ≤ 0.05 A.
- **Buzz:** none was reported.

**The mean speed matches the drag map.** The prediction comes from the stored J01 drag map, v = 2 − τ_drag(v)/kd. With drag_c 0.317 / 0.333 A and drag_v 3.24 / 2.66 mA per motor rad/s, it gives **+1.120 / −1.096 rad/s**. The mean of three runs measured **+1.123 / −1.095**. The spread between runs is far larger than that agreement — see O2.

**D19 — the 14/turn ripple is real torque and survives current mode.** The ripple torque was separated from the measured torque using J·dv/dt = τ_meas + τ_ripple, with J_out = 20.2e-6 × 81. It was then fitted against **absolute rotor angle** (raw encoder counts), jointly over orders 1–30, three runs per direction pooled:

| Order (per motor turn) | + direction: speed amp (motor rad/s) | + ripple torque (mN·m output, phase) | − direction: speed amp | − ripple torque |
|---|---|---|---|---|
| **14** | **±1.57** | **44.4 @ −7°** | **±1.74** | **48.2 @ −11°** |
| 28 | 0.14 | 9.4 @ +77° | 0.13 | 9.5 @ +69° |
| 12 (pinion teeth) | 0.28 | 7.1 @ −109° | 0.43 | 9.7 @ −109° |
| 7 (electrical) | 0.31 | 5.2 @ +62° | 0.33 | 6.3 @ +95° |
| 1 | 0.48 | 4.7 | 0.80 | 9.7 |

- **Locked to rotor angle:** the order-14 phase per run is −5, −6, −15 (+ direction) and −14, −7, −6 (− direction). Six runs, at different absolute positions, agree within ±5°.
- **Conservative:** the phase is the **same in both directions**. Ripple caused by modulated friction would flip by 180° when the direction reverses. A detent-like ripple does not flip.
- **Not encoder error:** an encoder nonlinearity would produce an apparent ripple torque that scales as speed². Here the per-run amplitude is 43.9 / 44.9 / 45.3 at speeds of 1.29 / 1.24 / 0.84 rad/s, where speed² changes by 0.42×. The amplitude does not change. It also matches a0's 39–49 mN·m, measured in voltage mode at 15–17 rad/s motor.
- **Inertia check:** solving for the J that makes the order-14 torque identical in both directions gives **J_motor = 20.2e-6**, exactly the stored value (the imaginary residual is 10%). This is a consistency check rather than an independent J measurement, because it assumes the ripple is conservative. It does mean that a4's "J is wrong" branch is now unlikely.
- **Size:** about 4.9–5.4 mN·m at the motor, above the ≥ 3 mN·m promotion threshold. The speed wobble is ±1.6–1.7 rad/s at the motor, inside the predicted "≈ ±1–1.5, promote" band and at its top. **D19 is promoted.** Next step: characterise it per joint after B12a (amplitude and phase vs electrical angle), then evaluate a Tier-0 feed-forward. The contract-freeze question (does Tier 0 cancel it, and does τ then include it?) is now live.
- **What the current loop sees:** about 39% of the ripple shows in measured τ (17–19 mN·m). That part is the law's kd reacting to the speed ripple; it does not show the current loop rejecting the ripple.
- Orders 28 (second harmonic), 12 (pinion mesh) and 7 are ≤ 10 mN·m and not acted on.

**O2 — new observation: friction depends on output angle.** The six steady-state friction values (mean τ) fall into two groups with no overlap: 0.068–0.076 N·m and 0.100–0.116 N·m. Absolute output angle was rebuilt by chaining each run's stop `p` from the status lines (`p` is re-zeroed at every arm, and O1 roll-back is mrad). On that basis, **all three slow runs sit inside 1.7–4.35 rad and all three fast runs outside it**, in both directions. That is a once-per-output-turn friction swing of about ±0.022 N·m around 0.089, a ratio of 0.78–1.30 to the mean. It matches the "same afternoon, 1.24× and 0.83× the drag map" note in `joint_cal.h`, and it explains a2's travel miss. The drag map is effectively the average over output angle: the three-run means hit it to 0.003 rad/s.
- **Status:** strongly suggested, **not established**. There are six windows, the angles are reconstructed rather than indexed, and time drift is not excluded (although the fast/slow order is not monotonic in time).
- **Decisive test:** one constant-speed run covering at least 1.5 output turns with a marked output index. At v_des 2 that is about 8 s at decim 32, so it needs either two chained captures or a longer capture.
- **Deferred with reason:** a4 does not need it. The friction range is inside a4's simulated band (0.082–0.12 N·m), and friction does not change the stop time. **Promoted when** a friction feed-forward is proposed, or when a4/b3 rest errors exceed their bounds.

**O1 (roll-back after a stop) — updated, not closed.** Part 2 shows the washboard is conservative, which the O1 hypothesis needs. Its torque (44–48 mN·m) is still below the lowest friction now measured (68 mN·m), so ripple alone still does not explain the roll-back. **→ Updated again in §22.7.13 (a4, O4):** friction near standstill is ~0.02–0.05 N·m, below the washboard, so the friction objection falls; O1 is narrowed, not closed.

**Not used:** the step response from switch to steady speed (T63 = 7.6–34 ms). The ripple and the position-dependent friction scatter it too much to read J from it. The order-14 consistency above is the better J evidence.

### 22.7.12 a4 procedure — spring + damper, three stiffnesses (keys spelled out)

**Aim.** Show that each kp gives the predicted motion timing. This proves the units end to end, from p and kp to N·m, A_rep, Iq and back to p. The **time from the switch to the first stop** is set by spring and inertia. Friction does not change it (simulated: 51.0 vs 51.1 ms at 0.082 / 0.12 N·m). The 14/turn washboard does change it, and by an amount that depends on where the rotor starts, so each row is measured at 8 start positions and judged on the **mean**.

**Settings and pre-registered predictions** (ζ ≈ 0.3; every step commands kp × step ≈ 0.235 N·m = 60% of τ_max). Stop times come from the 2026-10-01 simulation, which includes the 1 ms filter, the current loop, one loop of delay and dry friction; two models agree within 3%. a3 confirms the inputs: J 20.2e-6, ripple 44–48 mN·m, friction 0.068–0.116 N·m.

| Row | kp | kd | Step | Predicted mean stop time | Simulated spread (washboard) | Stops at | Rest-error bound (friction max + ripple) ÷ kp | Decim (capture length) |
|---|---|---|---|---|---|---|---|---|
| 25 Hz | 41 | 0.157 | 5.7 mrad | **19.2 ms** | −4% to +12% | 70–90% of the step, no overshoot | ±4.0 mrad | 2 (~175 ms) |
| 16 Hz | 16.8 | 0.100 | 14 mrad | **31.0 ms** | −13% to +28% | 70–90% | ±9.8 mrad | 2 (~175 ms) |
| 10 Hz | 6.6 | 0.063 | 35 mrad | **50.6 ms** | −21% to +59% | 70–90% | ±25 mrad | 4 (~350 ms) |

**Why re-arm between steps.** Each step starts from rest at a fresh zero (arming sets p = 0 here). A down step is staged as `pd −step` from its own zero rather than "back to 0", so every step is exactly the specified size even though the previous one stopped short.

**Session start:** `f`, then `m;`. Every MIT line ends with Enter or `;`.

**Row 1 — kp 41 (first: the most reliable row).**
1. `m kp 41 kd 0.157;`. The reply shows A with kp 41.000 and kd 0.1570. pd is 0, so there is no `!!` and no note.
2. `m b pd 0.0057;`. The reply must read `B = A + pd=0.00570 -> if m go now: pd=0.00570 … kp=41.000 kd=0.1570 …` with **no `!!` line**.
3. **Up step, four times:**
   1. `g`. The joint holds where it is; it feels like a stiff spring if touched, so don't touch it.
   2. Wait about 1 s.
   3. `m go 0 2;`. After 50 samples, pd jumps by +5.7 mrad.
   4. When `CAPTURE done` appears, **watch about 2 s of status lines**: p should stay constant, `cl=0/0/0`, and there should be no buzz or ticking.
   5. `x`, then `d`.
   6. **Turn the output pulley by hand by roughly 1–3°.** Any amount is fine; the point is a new rotor position against the 2.9°-of-output washboard.
4. `m b pd -0.0057;`, then the **down step four times**, exactly as in step 3.

**Row 2 — kp 16.8:** `m kp 16.8 kd 0.1;`, then `m b pd 0.014;` with four up steps, then `m b pd -0.014;` with four down steps. Keystrokes as in Row 1, still with `m go 0 2;`.

**Row 3 — kp 6.6:** `m kp 6.6 kd 0.063;`, then `m b pd 0.035;` with four up steps, then `m b pd -0.035;` with four down steps, using **`m go 0 4;`**.

That is 24 captures. Each arm lasts about 5 s, well inside the 20 s auto-stop. Paste the whole session log, including the `m b` replies and the 2 s of status lines after each capture.

**Pass:**
- **Mean stop time** within ±10% of prediction for kp 41 and 16.8, and within ±15% for kp 6.6. Individual-step scatter is recorded but not gated.
- Every rest error is inside its row's bound.
- No sustained oscillation or dither while holding, in the capture or in the 2 s of status lines. This also checks the "one-count kick can't dither" prediction.
- Uq < 2.0 V and clamp counters at 0 throughout. Step torque is 0.235 N·m and the kd term opposes it, so nothing should clamp.

**How to read results:**

| Pattern | Meaning |
|---|---|
| All three means off by the **same factor** | Inertia is no longer the likely cause (a3 consistency, §22.7.11). Suspect a kp/torque unit at a boundary that cancels in a3 (kp × rad → N·m → A_rep) |
| Means off by **different factors** per row | Unit or gear-ratio error at a boundary, or a delay larger than modelled (it would hurt the fast row most) |
| kp 41 and 16.8 pass, kp 6.6 is off | The washboard dominates at low stiffness. Supports the D19 feed-forward |
| Scatter much smaller than simulated | The washboard does not act as simulated; re-check the D19 phase model |
| Overshoot past the step | Less friction than any a3 value at that position, or less damping than modelled; check the kd units |

### 22.7.13 a4 — results: PASS on its aim; row 3's timing gate missed, explained by O4 (2026-10-02)

**Runs.** 20 captures. Row 1: kp 41, kd 0.157, ±5.7 mrad, 4 up + 4 down. Row 2: kp 16.8, kd 0.1, ±14 mrad, 4 + 4. Row 3: kp 6.6, kd 0.063, ±35 mrad, 2 + 2 (shortened plan). Decim 2 / 2 / 4. "First stop" is the peak of the step-direction displacement inside 3× the predicted time, as used for every row. Offline script on the logged `t_us`. The release positions come from the `run=0` status lines after `x`.

| Row | Mean stop time (pred) | Δ | Gate | Stop at (pred 70–90%) | Rest error (bound) | Uq max | Clamp | Holding |
|---|---|---|---|---|---|---|---|---|
| 1 | **19.5 ms** (19.2) | **+1.6%** | ±10% ✅ | 81–115% (two overshoots, caps 7, 8) | ≤ 1.79 mrad (±4.0) ✅ | 0.29 V | 0 | constant ✅ |
| 2 | **28.0 ms** (31.0) | **−9.8%** | ±10% ✅ (SE ±1.8%, so marginal) | 68–83% | ≤ 5.01 mrad (±9.8) ✅ | 0.28 V | 0 | constant ✅ |
| 3 | **61.3 ms** (50.6) | **+21%** | ±15% ❌ (n = 4, SE ±13%) | 93–100% | ≤ 4.42 mrad (±25) ✅ | 0.31 V | 0 | cap 11 creeps 12 counts toward the target over ~6 s; no dither ✅ |

Per capture (stop time ms / stop % / rest error mrad / roll-back after the peak mrad / held τ N·m):
- Row 1 up: 19.5/85/1.42/0.58/+0.058 · 22.7/98/0.60/0.48/+0.024 · 16.6/82/1.76/0.74/+0.070 · 17.1/81/1.79/0.73/+0.072. Down: 20.5/99/0.59/0.55/−0.028 · 20.5/108/0.16/0.60/−0.007 · 21.2/115/−0.18/0.68/+0.009 · 18.0/95/0.82/0.53/−0.032.
- Row 2 up: 25.4/83/4.88/2.51/+0.082 · 27.5/68/5.01/0.60/+0.084 · 27.1/72/4.71/0.85/+0.079 · 28.4/73/4.63/0.85/+0.078. Down: 26.4/80/4.21/1.42/−0.070 · 30.1/77/3.90/0.72/−0.066 · 29.4/79/3.63/0.62/−0.061 · 29.4/79/3.60/0.60/−0.060.
- Row 3: up 65.6/99/0.91/0.55/+0.006 · 65.1/100/0.40/0.25/+0.002; down 39.7/93/4.42/1.93/−0.029 · 74.6/96/1.36/0.0/−0.009.
- Row 2 caps 5, 6, 7 started from the same count (± 2): **30.1 / 29.4 / 29.4 ms**. Same start gives the same response to ±0.4 ms, so the scatter elsewhere is position, not noise. The pulley was not turned between caps 2→3→4 or 4→5→6→7 (start counts within 10); this costs nothing, see O4.

**Against the reading table (§22.7.12):** "kp 41 and 16.8 pass, kp 6.6 is off → the washboard dominates at low stiffness" is the branch that occurred. The units hypothesis is excluded on the shape of the misses: a kp, torque or gear-ratio error scales every row's stop time by the same factor (ω ∝ √kp). The rows are off by +2%, −10% and +21%, in the order of step size in washboard periods (0.11, 0.28 and 0.70 of the 1170-count period).

**O4 (new, ESTABLISHED) — the joint always comes to rest in the same 14/turn detent phase.** Rest positions, taken modulo 16384/14 = 1170.3 motor counts:
- **Hand-turned starts:** all seven row-1 re-positionings (thousands of counts apart) landed at **268–292**. Row 2 hand-turned starts landed at 284 and 287, and row 3 at 260, 303 and 317. One exception: row 3 cap 10 at 659.
- **After `x` (zero torque):** every up step released back to **282–296**, returning 0.95–1.10 of its displacement (200–255 counts in row 2). Every down step released to **128–176**.
- **Reading:** a detent centred at ≈ 218 counts, with a stick band of about ±70 counts. a3's order-14 fit alone puts the detent bottom at 263, which is 45 counts (14° of order-14 phase) from 218.
- **Strength:** under uniform phase, a 24-count window catching seven independent settles has probability ~10⁻¹². The detent lock is established, not suggested.

**What O4 changes.**
1. **a4's eight-position design did not do what it was meant to.** Each step was supposed to sample a random washboard phase, and the simulated spreads (−21% to +59% at kp 6.6) assumed that. In practice every step started in the detent. The row means therefore test the detent-start response, which was never simulated (R29).
2. **Post-hoc check, no fitted parameters.** The 2026-10-01 model was re-run with each capture's measured start count. It uses J 20.2e-6, kp/kd, the 1 ms filter, a 0.6 ms current lag, a3's orders 14 + 28 and a3's friction (`sat_sim.py`, `percap.py` in the session scratch). Means at F = 0.068: **row 1 17.7 (meas 19.5), row 2 28.7 (28.0), row 3 57.7 (61.2) ms.** Per capture, the row-2 down steps come out at 31.8/31.3/31.2 (meas 30.1/29.4/29.4), and **row 3 cap 10's early stop from its off-detent start is reproduced: 40.3 vs 39.7 ms.** The timing match also holds at F = 0.09: 17.6 / 28.4 / 52.5 ms. So it does not depend on the friction choice. **The plant model in hand explains the row-3 miss; the miss was in the predictions' sampling assumption.**
3. **The stop fraction does depend on friction,** and it favours the low end. At F 0.068 the means are 86 / 77 / 91% (meas 95 / 76 / 97); at F 0.09 they are 74 / 65 / 72%. During these slow moves (≤ 1.4 rad/s output) friction acts at or below a3's lowest value.
4. **Direction asymmetry, and row 1's two overshoots:** every start sits on the detent's upper edge (≈ 285). An up step climbs the washboard at once, so it stops shorter. A down step first slides into the bottom with the washboard's help: row 1 down 95–115% vs up 81–98%. Row 2 rest error up 4.6–5.0 vs down 3.6–4.2 mrad. Held torque up 0.078–0.084 vs down 0.060–0.070.
5. **Friction near standstill is small.** On release, only the washboard (≤ ~0.055 N·m with orders 14 + 28) is available to move the rotor, and it moved 200–255 counts. A stick band of ±70 counts around the detent means the rotor stops where the washboard torque falls to ~0.017 N·m. With a3's amplitude, the hold-then-release pairs bracket **static friction ≈ 0.03–0.05 N·m** at these positions. Sliding at very low speed is **~0.02 N·m**, against **0.068–0.116 N·m** at 0.8–1.3 rad/s in a3 and 0.08 for the drag map's Coulomb term. This is consistent with B4's breakaway from detents (mean 0.074 = friction plus the rising washboard), and with B4's own warning about rotors left mid-creep. **Hypothesis, not established:** the numbers depend on the washboard's shape at rest, which a3 measured only in motion (±20% amplitude, ±50 counts phase).
6. **Robot consequence:** at standstill the washboard is as large as friction. A joint commanded within roughly ±70 counts (3 mrad output) of a detent edge, with kp·error below ~0.05 N·m, stalls on the washboard, not on friction. At zero torque the joint rolls into the detent. That is the D19 feed-forward's job, and the reason a quasi-static measurement now belongs in D19's characterisation (D19 row).

**O3 — roll-back after the peak, while the motor still pushes forward.** Seen in 19 of 20 captures: row 1 0.48–0.74 mrad, row 2 0.60–2.51, row 3 0.25–1.93 (cap 11 none).
- **What changed:** after row 1, the session analysis attributed it ~70% to a spring in series with output friction (belt or tooth flex) and ~25% to pre-sliding, and called cogging "ruled out, 9/9 against step direction". **That ruling-out is withdrawn (R30).** It assumed random start phase. Because of O4, every step starts in the detent, so a washboard that pulls back against the step every time is exactly what cogging predicts.
- **Belt wind-up:** the recipe-B ladder stiffness (true 67.2 kN/m, about 80 N·m/rad output-referred) gives at most ~1 mrad (25 counts) at 0.084 N·m. That could be part of row 1's 0.5–0.7 mrad, but it cannot produce the 200–255-count returns on release.
- **Ranking:** washboard + low near-standstill friction (H1) high, because one mechanism covers O3, the release returns and O4; belt wind-up a contributor of ≤ 1 mrad; pre-sliding low. **This data cannot separate H1's share of O3 from ≤ 1 mrad of belt wind-up.** B12b's clamped output separates them.
- **Deferred with reason:** no B12a decision depends on it. **Promoted when** the D19 at-rest sweep gives the washboard shape (H1 then predicts O3 per capture), or when B12b b1 runs.

**O1 (roll-back at zero torque after the a2 pulses) — narrowed, not closed.** O4 supplies the mechanism that O1 lacked: friction near standstill is below the washboard. Roll-backs up to half a period (585 counts, 25 mrad output) fit a detent. a2 cap 2's 32.2 mrad exceeds that, so something more (orders 1, 7, 12, which are not 1170-periodic, or coasting) is involved. Same promotion condition as O3.

**Also closed today:** the "+" viewing face. **"+" = counter-clockwise viewed facing the output-pulley shaft end,** the face that points outward when mounted on the robot (owner, 2026-10-02).

**Verdict: a4 PASS on its aim.** The aim was the units end to end, p and kp → N·m → A_rep → Iq → p: shown by rows 1–2, and by the per-capture model agreement across all three rows.
- **Pre-registered gates:** rest errors, holding, Uq and clamps pass in every row. Row 3's mean stop-time gate is recorded as **missed** (+21% vs ±15%); with n = 4 its own SE is ±13%.
- **Prediction failures:** the stop fractions (row 1 up to 115%, row 3 93–100%) and the "8 positions sample the washboard" premise are recorded as findings: O4, R29.

### 22.7.14 a5 procedure — saturation on a moving motor (keys spelled out)

**Aim.** Make the clamps bind while the motor moves and show the response stays well-behaved. The law has no integrator, so nothing should wind up. Run 1 makes the **outer** clamp (τ_max 0.39 N·m) bind. Run 2 raises τ_max on purpose so the **inner** clamp (1.6 A_rep = 0.4028 N·m) binds instead. This is the last B12a step.

**Setting.** kp 41, kd 0.157 (row 1's gains). Saturation error e_s = 0.39 / 41 = 9.5 mrad; **step = 3 × e_s = 28.5 mrad** (1.6° of output, visible). The law asks for 1.17 N·m at the switch.

**Pre-registered predictions.** These come from the a4-validated model (§22.7.13), started from the detent (counts 3805 up / 3656 down). The ranges span F 0.068–0.09, with the washboard on.

| Quantity | Run 1 (outer) | Run 2 (inner) |
|---|---|---|
| `tau_cmd` while clamped | **0.390** (Iq 1.549 A_rep) | **0.4028** (Iq 1.600 A_rep) |
| Measured `tau`, mean over the plateau | 0.390 ± 0.005 | 0.403 ± 0.005, i.e. **+13 mN·m vs run 1** |
| Clamp code in the capture | 1 for the first **12–13 ms**; a second short burst on the braking side at 21–27 ms is possible at the low-friction end | 2 for the first ~10–13 ms |
| `cl=` after the capture (loops) | outer **130–350**, inner 0, reject 0 | outer 0, inner **110–300**, reject 0, plus the `!!` mark |
| Peak travel | **122–128% of the step** (34.8–36.5 mrad), against 81–115% for a4's linear steps. Overshoot **grows** under saturation, because braking is clamped too; the old "no overshoot growth" criterion was wrong | same ± 2% |
| Peak speed | **2.25–2.4 rad/s output** (20–22 rad/s motor) | same |
| Last count change | ~45 ms after the switch | same |
| Rest | 101–106% of the step, rest error within ±4.0 mrad (row 1's bound) | same |
| Uq | < 1.0 V | same |

**Session start:** `f`, then `m;`. Every MIT line ends with Enter or `;`. **No hand-turning between steps:** O4 shows every start lands in the same detent anyway.

**Run 1 — outer clamp.**
1. `m kp 41 kd 0.157;`
2. `m b pd 0.0285;`. The reply must read `B = A + pd=0.02850 -> if m go now: pd=0.02850 … kp=41.000 kd=0.1570 …` with **no `!!` line**.
3. **Up step, twice:** `g` → wait ~1 s → `m go 0 2;` → at `CAPTURE done`, watch ~2 s of status lines (p constant; `cl=` showing a non-zero first number and `/0/0`; no buzz) → `x` → `d`.
4. `m b pd -0.0285;`, then the **down step twice**, exactly as in 3.

**Run 2 — inner clamp** (τ_max raised; `g` resets it to 0.39 every arm, so the order matters).
1. `m b pd 0.0285;`
2. `g`
3. `m tmax 0.6;`. The reply must read `MIT A: … tf_ms=1.000 tmax=0.600` **and** be followed by the line `!! tmax is above the 1.6 A_rep envelope: the INNER clamp binds first (a5 run 2 only)`. If that line is missing, do **not** go on: `x` and paste. The capture header's `tau_max=` field also records 0.600, which is the check after the fact.
4. `m go 0 2;` → watch ~2 s of status lines (`cl=0/…/0` with the middle number non-zero) → `x` → `d`.
5. `m b pd -0.0285;` → `g` → `m tmax 0.6;` (check the `!!` line again) → `m go 0 2;` → `x` → `d`.

Six captures, each arm about 5 s. Paste the whole log, including the `m b` and `m tmax` replies and the status lines.

**Pass (gates):**
- Run 1: clamp code 1 only, and `cl` inner = reject = 0. Run 2: clamp code 2 only, `cl` outer = 0, and the `!!` line present.
- `tau_cmd` never exceeds 0.3900 in run 1 or 0.4028 in run 2.
- Uq < 2.0 V, peak speed < 45 rad/s motor, and the swings after the peak decay. Status lines hold constant after the capture (no limit cycle).
- Rest error within ±4.0 mrad.

**Predictions that are findings if missed** (not gates): peak travel, clamp duration and the run-2 plateau difference.

**How to read results:**

| Pattern | Meaning |
|---|---|
| Peak travel well below 122% | Braking stronger than modelled: more friction at speed, or the kd term not clamped where expected (check `tau_law` vs `tau_cmd`) |
| Run 2 plateau not ~13 mN·m above run 1 | The inner clamp is not where 1.6 × `calKtCmd()` × 9 says it is: a conversion-vs-envelope finding |
| Clamp code 2 in run 1 | The inner clamp sits below 0.39 N·m: the same finding |
| Oscillation that grows or persists | Saturation interacting with the loop delay; stop and paste. The model predicts none |
