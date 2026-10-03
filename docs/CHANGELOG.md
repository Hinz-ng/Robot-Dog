# Changelog — dated sessions, newest first

Every entry declares what changed and why, so nothing regresses silently.
Superseded values are listed at the foot of each entry rather than deleted, so a
number that reappears in an old note can be traced to its retraction.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 0. Changelog — 2026-10-03 b1 PASS; B12 CLOSED

- **b1 PASS** (BELT_DRIVE §22.7.16): 10 captures on the clamped output, clamp 0, all rings decay, no buzz. Overshoot 35 → 24 →
  17 → 9.5% (ζ 0.31 → 0.60) for kd 0 → 0.157 → 0.25 → 0.365 at kp 41; repeats within 1%.
- **Gate not decidable:** f_d(kp 41)/f_d(kp 0). The kp 0 repeats differ by 9%, rings are 2–9 counts. Static stiffness answers the
  underlying question (+54 vs +41 commanded). Gate design error recorded.
- **Misses, explained:** rest deflection 2× predicted. The belt is a softening spring: secant 142–182 vs small-signal 280–335
  N·m/rad. ζ(kp 41) −0.10, not −0.02: the prediction omitted the √(K/(K+kp)) scaling of structural damping. kd buys ~0.75 ζ per
  N·m·s/rad, above prediction for rungs 2–3.
- **Findings:** series compliance (output stiffness = kp·K/(kp+K), 80–89% of commanded at kp 41) → sim models the belt as a
  series spring; kd quantisation blip = kd × 0.039 N·m (6 / 10 / 14 mN·m measured); O3 closed as explained.
- **Sim/RL numbers:** belt mode 55–72 Hz, K_belt 140–335 N·m/rad output, ζ 0.20–0.45; demonstrated gain box kp ≤ 41,
  kd ≤ 0.365.
- **Deferred:** pushing kp/kd to a stability limit. Promote when gait/RL needs more; do it on the leg (clamped limit not
  conservative).
- **B12 CLOSED.** Next: B10 skip threshold → 7l homing → J03 → leg. Docs only; no src change.

---

## 0. Changelog — 2026-10-02 (e) a5 PASS; B12a CLOSED

- **a5 run 1 PASS** (outer clamp, 4 steps): code 1 only, `cl` inner/reject 0, τ_cmd ≤ 0.3900, Uq ≤ 0.72 V, rest error ≤ 1.4 mrad,
  no limit cycle. **Run 2** (inner clamp): `m go 2` typed for `m go 0 2` → 2 ms pulse, then the revert zeroed A's gains. The inner
  clamp still fired cleanly (code 2 only, `cl=0/22/0`, τ_cmd 0.4028); cap 6 void (kp 0). Not rerun: it gates nothing further.
- **Findings, not pursued:** peak travel 115–120% vs 122–128% predicted; plateau torque −5% (Iq sags as the back-EMF ramps);
  **measured Iq overshoots the clamped command by up to 6%** (1.69 vs 1.600 A_rep) → future envelopes sit ≥ 6% below the
  demonstrated-safe current. O4 narrowed: releases also rest outside the 218 ± 70 band.
- **Corrections (external review):** the a4 post-hoc rerun is "within 10%", not "explains" (−9 / +2.5 / −6%); a4 row 3 is
  "miss, explained", not a pass.
- **B12a CLOSED.** Next: B12b **b1** (clamped output, gain envelope). b2/b3 inertia bar deferred to the leg.
- CLAUDE.md: new section "Everything earns its place" (streamlining rule). Docs only otherwise; no src change.

---

## 0. Changelog — 2026-10-02 (d) a4 PASS on its aim; O4 detent rest; R29/R30; a5 procedure

- **a4 PASS on its aim** (units end to end; BELT_DRIVE §22.7.13). Mean stop time against the linear-model prediction: kp 41
  **+1.6%**, kp 16.8 **−9.8%** (gates ±10%), kp 6.6 **+21%** (gate ±15% **missed**, n = 4). Rest errors, holding, Uq ≤ 0.31 V
  and clamp 0 pass in every row. The three rows are off by different factors, in step-size order, so units are not the cause.
- **O4 (new, established):** the rotor always rests in the same 14/turn detent phase, ~218 counts mod 1170, with a ±70-count
  stick band. All 7 hand-turned starts in row 1 were at 268–292. Releases go to 282–296 after up steps and 128–176 after down
  steps. Re-simulated from the measured start counts with no fitted parameters, the model gives 17.7 / 28.7 / 57.7 ms against
  measured 19.5 / 28.0 / 61.2. Three caps in row 2 from the same start agreed to ±0.4 ms.
- **R29:** a4's premise "eight start positions sample the washboard" was false (O4). **R30:** the session claim that "cogging is
  ruled out (9/9)" for O3's roll-back is withdrawn, because it assumed a random start phase. Belt wind-up is ≤ ~1 mrad.
- **O1 narrowed:** friction near standstill is ~0.02–0.05 N·m (hypothesis). That is below the washboard, so the old objection
  falls. Not closed.
- **D19 scope:** an at-rest quasi-static torque–angle sweep is added (washboard shape at rest, plus static friction from the
  hysteresis).
- "+" viewing face closed: CCW viewed facing the output-pulley shaft end (outward on the robot).
- **a5 procedure written** (§22.7.14): kp 41 / kd 0.157, step ±28.5 mrad. Run 1 uses the outer clamp (0.39); run 2 sets
  `m tmax 0.6` so the inner 1.6 A_rep clamp (0.4028 N·m) binds. The old criterion "no overshoot growth under saturation" is
  replaced: overshoot is predicted to grow (122–128%), because braking is clamped too.
- Docs only. No src change.

---

## 0. Changelog — 2026-10-02 (c) a3 PASS; D19 promoted; O2 logged; a4 procedure

- **a3 part 2 PASS** (BELT_DRIVE §22.7.11). Six runs at v_des ±2 with kd 0.1: all inside 0.8–1.8 rad/s output; Uq ≤ 0.32 V;
  clamp 0; τ − τ_cmd ≈ 3 mN·m RMS. Mean of three runs +1.123 / −1.095 rad/s against drag-map predictions of +1.120 / −1.096.
- **D19 PROMOTED.** The 14/turn ripple torque is 44.4 / 48.2 mN·m output (4.9 / 5.4 mN·m motor). It is locked to absolute rotor
  angle (six runs within ±5°), has the same phase in both directions (conservative, not friction modulation), and does not change
  with speed from 0.84 to 1.32 rad/s (so not encoder error). Characterise per joint after B12a; Tier-0 feed-forward to be evaluated.
- **J consistency:** the J that makes the order-14 torque direction-independent is 20.2e-6, the stored value. Noted on
  `J_ROTOR_KGM2` in `fleet_config.h` (comment only).
- **O2 (new, deferred):** friction swings 0.068–0.116 N·m with output angle (slow windows all inside 1.7–4.35 rad). This explains
  the drag map's ±20% session scatter and a2's travel miss. Not established; the test is named in §22.7.11.
- "+" = counter-clockwise (owner); the viewing face is still to note. a4 procedure written (§22.7.12), using the new `m b` semantics.

---

## 0. Changelog — 2026-10-02 (b) `m b` redesigned: B = A + staged changes; zero-gain warnings

- **a3 part 2, first attempt, commanded zero torque.** B was `vd 2, kd 0`: `m b vd 2` edited a B that a reboot had
  zeroed, and τ = kd(vd − v) = 0. The capture shows τ_law = 0 in all 1000 samples. **The fault was in the procedure (it
  assumed B kept kd 0.1 from part 1) and the UI made it easy;** the law did what it was told. BELT_DRIVE §22.7.10.
- `open_test.cpp`: B is now **staged field changes** (`mit_stage` + mask), resolved against A when `m go` fires and
  re-validated there. Each `m b` line replaces the previous one; `m b` alone clears. New `!!` warning for vd with kd = 0 /
  pd with kp = 0, on A edits, `m b` and `m go`. A note when pd/vd/ff are set while stopped (`g` clears them). `g` and `m`
  print what `m go` will apply. Switch, revert, capture and dump are unchanged; a2's and part 1's procedures keep their meaning.
- J01 121,112 B (92.4%, +1.2 KB), RAM 24,508 B. Builds on J01/J02/J03; **not bench-run**.
- §22.7.5 part 2 procedure rewritten (`m kd 0.1;` first; the motor keeps spinning after the capture → `x`; note the "+"
  direction). §22.7.6 decim timings updated to the measured ~11.9 kHz.

---

## 0. Changelog — 2026-10-02 Prefetch diagnosis falsified; a2 loop rate passes; a3 part 1 PASS

- **Measurement beats conclusion:** with `prefetch=1`, TORQUE(I) armed lps moved +0.3% (12,440 → 12,481) and disarmed
  −1.1%. That is no effect. **(e)'s "fetch-bound, fixed by prefetch" is withdrawn.** BELT_DRIVE §22.7.7 is rewritten with the
  data table, the ranked hypotheses (branch-target misses at 8 WS / cost outside `mitService` / layout) and their tests.
- **a2 loop-rate criterion now PASSES:** MIT armed −4.7% vs TORQUE(I) (was −8.2%). The gain sits in the armed MIT path only,
  so it is attributed to the MIT trim (`micros()` → DWT, per-loop τ removed), not prefetch. MIT mean cost is 3.95 µs armed
  and 2.9 µs disarmed (from lps). `svc_us` is a max that includes interrupt preemption.
- The a2 pulse re-ran on the new build: Iq 90% at 1.60 ms, clamp 0, |Uq| ≤ 0.50 V, revert at 60.0 ms.
- `FLASH_PREFETCH` stays `true` (harmless; today's captures used it). Comments are corrected in `fleet_config.h`, `actuator_hw.h`
  and `open_test.cpp`; no code change. **Deferred:** flash wait states 8 → 4. Promoted when the Tier-0 loop with CAN is
  measured below its need, or a loop-rate gap fails a gate again.
- **a3 part 1 PASS** (§22.7.9): slope −0.0998 pooled, R² 0.998, both directions, clamp 0, |Uq| ≤ 0.10 V. Velocity is
  differentiated offline from `p`. This checks the law, estimator, sign and current loop, not absolute torque (same Kt both sides).
  The capture length (2.74 s) is correct; the console is silent while it records.
- **Dropped:** the a1 `m test` on `-e J03`. The env guard refuses another joint's build on the J01 board, by design.

---

## 0. Changelog — 2026-10-01 (e) Loop-rate deviation traced to flash prefetch; MIT timing on the cycle counter

- **Cause of the a2 loop-rate deviation:** the core runs at 170 MHz with `FLASH_LATENCY_8` and prefetch off, and the ART I-cache
  is ~1 KB. So the loop is instruction-fetch bound, and cost scales with code bytes. MIT disarmed cost 3.2 µs on an
  estimator of a few dozen instructions; armed it cost 7 µs.
- **`FLASH_PREFETCH = true`** (`fleet_config.h`), enabled first in `actuatorInitHw()`. The CFG banner prints `prefetch=` and
  `flash_ws=` from `FLASH->ACR`. **This changes the loop rate in every mode.** The 8 wait states are unchanged
  (4 is in spec; that is a separate decision).
- `mit_law.h`: `mitEstUpdate()` now takes `dt` from the caller; `mitEstPrime()` no longer takes a timestamp.
- `open_test.cpp`: MIT timing (dt, switch, revert) moved to `DWT->CYCCNT`. The per-loop measured-τ computation is removed
  (unused by the law). New `svc_us=` status field.
- J01 119,880 B (91.5%). Builds on J01/J02/J03, **not bench-run**. Predictions are in BELT_DRIVE §22.7.7.
- a2 auto-stop: verified by the owner. a3: the hand-turn resistance was the damper (live from `g`), and the slope from the
  status lines is −0.098. The §22.7.5 wording is corrected, and A/B is explained in §22.7.6.

---

## 0. Changelog — 2026-10-01 (d) B12a a0–a2 recorded (BELT_DRIVE §22.7); D19 logged

- **a0:** Tf_mit = 1.0 ms (measured noise 0.120 rad/s motor = 4.9 mN·m at kd 0.365, 70% of budget; 8.9° lag at 25 Hz).
- **a1 PASS 22/22 on J01** (0.10 N·m → 0.3972 A_rep; 0.39 → 1.5492; inner 1.6000; every bad input → 0). The J03 run is not logged.
- **a2:** all sign, clamp (0 hits), Uq (≤ 0.64 V) and speed (≤ 28.3 rad/s motor) criteria pass on 4 pulses.
  - **Loop rate −8.2%** (11,420 vs 12,440 lps), over the 5% gate. Prediction impact < 0.5%; cause not identified; profile before a4.
  - Travel prediction missed low: the prediction used the lowest-friction acceleration. Friction backed out of the pulses
    is mean 0.31 A_rep vs drag_c 0.32/0.33.
  - **O1:** the joint rolls back 6–32 mrad after stopping with zero torque, mechanism open.
  - Auto-stop and the "+" direction are not logged.
- **D19** (14/turn position-locked ripple) added to the BELT_DRIVE deferred table.

---

## 0. Changelog — 2026-10-01 (c) MIT line fix: commands from a monitor with no line ending; B survives arming

- **Bug (a1):** `m test` echoed and did nothing. The line waited for CR/LF, and the monitor sent none. A line now ends on CR, LF, `;`, **or** a whole-line burst (every char within 25 ms of the `m`, ≥ 2 non-space chars) followed by 250 ms idle. A plain idle rule was rejected: a typed `m kp 4…` pause would run kp = 4, and a bare `m` would hand the rest of the line to the key switch (`4` = AUTOCALIB phase 4). A 2 s hint replaces the silence. The status line is held while a line is open; it had been printing through the echo.
- **Arming no longer clears the staged B.** It only applies on `m go`, and clearing it forced every B to be typed after `g`, inside the 20 s auto-stop.
- J01 119,620 B (91.3%).

---

## 0. Changelog — 2026-10-01 (b) B12a contract code (C1–C8); current and Uq limits deliberately NOT raised

- **New `src/mit_types.h`** (C1): `MitCmd {p_des_rad, v_des_rads, kp_Nm_per_rad, kd_Nms_per_rad, tau_ff_Nm}`,
  `MitState {p_rad, v_rads, tau_Nm}`; output-side SI, no includes (Tier 1 can include it). Records the τ
  definition, the sign convention and the two open freeze items (homing, mirrored-leg sign).
- **New `src/mit_law.h`** (C3/C4): pure estimator, law and validator, for Tier-0 reuse.
  - **The estimator reads RAW COUNTS and applies `sensor_direction` once.** `motor.shaft_angle` already carries
    it (SimpleFOC `FOCMotor.cpp:64`), so the plan's "× dir" on shaft_angle would have flipped the sign twice.
    That is invisible on J01 and J02 (both dir = +1) and wrong on the first dir = −1 joint.
  - `Tf_mit` is its own first-order filter; `VEL_TF` is untouched. The default is 1 ms until a0 picks it.
  - `mitCmdValid()` rejects (never repairs) negative or non-finite gains and out-of-range fields.
- **`open_test.cpp`:**
  - **C2:** `MIT_TAU_MAX_Nm` 0.39 (settable to ≤ 1.0, reset on every arm) and `MIT_ENV_A_rep` 1.6 (const, the inner clamp).
  - **C4:** `MODE_MIT`. A new `isCurrentMode()` predicate gives MIT the sense-chain refusal **and** the sense-mismatch
    guard. Arming zeroes p, clears pd/vd/ff and keeps kp/kd.
  - **C5:** `m` line commands, with the line handler ahead of the `-`/`5` chord tracker; timed A→B switch after
    50 pre-trigger samples, optional auto-revert.
  - **C6:** MIT capture columns, RAM-neutral (slot reuse plus dump-time reconstruction).
  - **C7:** status fields. **C8:** `m test`.
  - `autocalib.h`'s mode restore handles `MODE_MIT`.
- **Flash:** J01 110,344 → **119,264 B (91.0%)**, RAM +232 B. The first build overflowed: `strtof()` linked newlib's
  strtod/gdtoa (~10 KB), and `strtok()` linked `__assert_func` → `fiprintf` → stdio (~3.5 KB). Both were replaced by
  small local code, and the binary now carries neither.
- **Decided: the current cap and `VOLT_LIMIT` are NOT raised for B12a.**
  - With a bare output the belt carries almost nothing at any current, so raising the cap would be physically safe in
    B12a. But it buys little (a4 times the first half-cycle, which Coulomb friction does not move). It also creates a
    second envelope that must be switched back before B12b loads the belt, and forgetting that switch is the error the
    inner clamp exists to stop. Raise it through B10.
  - `VOLT_LIMIT` is also the runaway backstop if current sensing collapses (≈ VOLT_LIMIT / R ≈ 9 A today). Every
    planned B12a motion stays below ~70 rad/s at the motor, under the ~81–91 rad/s ceiling, and Uq is logged.
- **Plan corrections** (relayed in the session, not yet written into BELT_DRIVE): a3's v_des 0.5 cannot move the
  joint (kd·v_des = 0.05 N·m < drag 0.082); a4 predictions must include filter and current-loop lag (9.9 / 16.1 /
  26.0 Hz at Tf 1 ms); the a0 noise rule needs one kd (ζ 0.7 → 0.365, not 0.26).

---

## 0. Changelog — 2026-10-01 Units decided (option A); the torque boundary exists; stage 3b accepted; stale Kt retracted

- **Units: option A, with three additions** (record in `fleet_config.h`; CONSTANTS §8.1c; README §15 7k).
  - Reported amps (`_A_rep`) stay the firmware's current unit. Physical torque, **τ = N × Kt × I_true
    at the output, η excluded**, meets them only at `joint_cal.h`'s `torqueOutToIrep()` /
    `irepToTorqueOut()`, **in both directions** (commands and feedback).
  - **Clamp chain** `tauOutCmdToIq()`: non-finite / bad limit / uncalibrated → 0; clamp τ_max (N·m);
    convert; clamp the demonstrated envelope (A_rep) **last**. Returns which clamp bound, and an
    inner-clamp hit is a conversion-vs-envelope finding. It is a command limit and adds no e-stop path.
  - **Unit in the identifier:** `CURR_MAX/LIMIT/STEP`, `KICK_BASE/KICK_A`, `AC_IMAX_ABORT`,
    `AC_M4_STEP/ABORT/WARN_A`, `AC_SW_I` → `…_A_rep`; `TORQUE_MAX/STEP` → `…_V` (they are volts in
    TORQUE(V) mode); the sense guard's literal 0.6 / 3.0 A became `GUARD_PAD_A_rep` / `GUARD_FLOOR_A_rep`.
    **Byte-identical** on J01 / J02 / J03 (`595176e9` / `7179c07e` / `fd75f109`).
  - **A leak fixed in existing code:** the ladder's `k_beltline` turned reported amps into force with
    plain `calKt()`, so it read **3.9% low**. It now goes through `irepToTorqueOut()` and prints
    both the true k and the archive-convention k.
  - **Banner:** the `cmds now scaled` label claimed a correction nothing applied. It is reworded, and a
    new line prints `torque boundary: 1 A_rep = 0.2517 N.m output` on J01.
  - J01 flash 110,168 → 110,344 B, RAM unchanged. New md5 J01 `2371ab6a`, J02 `fbf2da3e`, J03 `efe26c0f`.
- **Stage 3b bench-accepted on J01** (README §15). The UT89X reading and the auto-stop line still have to
  be backfilled into the log (7m).
- **R26–R28** (BELT_DRIVE §22.6.9): the stale Kt 0.0266 is gone from CONSTANTS §8.1 and HARDWARE (now
  `calKt()` per row). §22.6.4's torque is now 0.0743 N·m true. §22.6.3's k values are labelled as the archive
  convention (true = ×1.0394, mean 67.2 kN/m). B12 scoping: wrong reason for lost motion (R27), and the
  saturation torque (0.403 N·m, 9.8 mrad at kp 41, R28).
- **B12 split into B12a (bare output) and B12b (defined load).** Homing (7l) is open before the contract freezes.
- **Flagged, not changed:** the F/I table in `fleet_config.h` uses the pre-M1 Kt 0.026621 (+1.1% per true
  A, +5.1% per reported A). It is a design figure carrying an unmeasured η, and M14 replaces it.

**Superseded values:** Kt 0.0266 as a stored constant (CONSTANTS §8.1, HARDWARE) · 0.071 N·m breakaway torque ·
the k values 62.4 / 64.1 / 65.2 kN/m · 0.38 N·m and 9 mrad at the 1.6 A cap.

---

## 0. Changelog — 2026-09-30 (b) Recipe-B drag measured; B11 complete; phase 4 is belt-off only; both owner questions answered

- **B3 on recipe B** (BELT_DRIVE.md §22.6.6; raw log archived in `docs/cal/pulley acceptance/`):
  - Phase 5 in both orders. **Stored per direction, pooled over orders:** drag_c fwd **0.3169** /
    rev **0.3330 A**, drag_v fwd 3.24e-3 / rev 2.66e-3. The mean 0.325 is in J01's band.
  - **`joint_cal.h` J01 drag fields replaced.** Every belt-on field now describes recipe B.
  - **Convention change:** recipe A's row stored the first run only (0.291). Recipe B's first
    run is 0.292, so there is no drag change attributable to the recipe.
- **New finding: a run-order effect.** The direction run second reads 0.10–0.13 A higher, in
  both runs. It is not monotonic warm-up. Pooling cancels it. Deferred as **D18**.
- **Phase 4 belt-on (R25) is a physical limit, not a bug.** The "twitch" was the whole phase,
  which is under a second. It completed with **drift −169 counts** (WARN) because belt friction
  and wind-up make the 0.8 ↔ 3.2 A step train ratchet the self-lock. Phase 3 is immune
  (strongest-first; drift 0). The **"1, 3, 4 are belt-agnostic"** claim (F7) is falsified for 4.
  - Firmware header comment, `y`-menu print and the phase-4 comment are corrected.
  - L stays carried from belt-off. The belt-on L of 44.7 µH is +2.1%, as on 2026-08-12.
- **⟨V21⟩ closed:** the same plate and belt, so the A/B was single-variable and the G drop is the
  recipe.
- **⟨V24⟩ closed:** every pulley before J01-P12B, **including J02's SCB**, was on the 0.6
  profile. Consequences:
  - R21 is refined: the backlash is the 0.6 profile **at −0.12**, since −0.10 on the same
    profile gave low G on both joints.
  - The P10 → P12 drag trade-off stands, for the 0.6 profile only.
  - All §22.4.3 print fingerprints are 0.6-profile numbers; re-baseline them on recipe B (D5).
  - D12's +0.35 boss prediction is **void**.
  - J02 needs a recipe-B reprint and re-acceptance before D15.
- **Still open:** ⟨V22⟩ G-code md5. ⟨V23⟩ is still the old binary; flash the current source (README 7i).

### Superseded values

- J01 drag_c ~~0.291 / 0.291 A (recipe A, first run, split unrecorded)~~ → 0.3169 / 0.3330 A (recipe B, pooled)
- J01 drag_v ~~2.4e-3 / 2.4e-3~~ → 3.24e-3 / 2.66e-3
- ~~"phases 1, 3, 4 are belt-agnostic"~~ → 1 and 3 only (R25)
- D12 ~~"boss +0.35 prints 15.10"~~ → void (0.6-profile basis)

---

## 0. Changelog — 2026-09-30 Pulley recipe B adopted; J01 re-accepted; the gate exception closes; the pulley detour is over

### ✅ Recipe B is the pulley of record (BELT_DRIVE.md §22.6)

After a CrealityPrint update, the pulley presets turned out to have been living under the
**0.6 mm nozzle machine profile** with 0.4-tuned settings ("recipe A", every pulley through
2026-09-28). They were copied onto the 0.4 profile and J01's −0.12 + precise-wall pulley was
reprinted ("recipe B"). Arachne thresholds are percentages of `nozzle_diameter`, so A had
generated inner beads up to 0.76 mm wide from a 0.4 mm nozzle.

- **New rule:** a recipe is identified by the **md5 of its sliced G-code**, not by its settings (R24).

| Test | Recipe B | Recipe A | Result |
|---|---|---|---|
| **B6a G** (3 ladders, 2 boots, I_f 0.295) | **5.8 counts** | 48.8 (at matched I_f) | ✅ ≤ 20 gate passed; **gate exception CLOSED** |
| B6b f_d | 65.0 Hz (f_n 69.8, ζ 0.36) | 64.3 | ✅ Δ +0.7 ≤ 1 Hz |
| B4 breakaway | 0.295 A (n = 20) | 0.288 | ✅ Δ +0.007; ±0.03 band not resolvable at n = 20 |

- **`joint_cal.h`:** J01 `breakaway_A` 0.288 → **0.295**, date → 2026-09-30, and the comments
  rewritten for recipe B. **The drag fields are still recipe A:** B3 was not run on B, and it
  is now the one bench item left (README 7h).
- **D17 demoted.** Promote if a fleet joint fails G ≤ 20.
- **Owner answers:** clamp = **2 × M3×10 countersunk** (⟨V20⟩ closed); recipe-B boss CAD 15.26,
  printed **14.95** (⟨V18⟩ closed); ⟨V16⟩ closed by decision (only the latest acceptance is
  archived, in `docs/cal/pulley acceptance/`).
- **Method:** A/B breakaway at **fixed raw positions, compared pairwise** (position sd 0.095 A).
- **§22.5 demoted** to superseded history. Its decision, B11, clamp and fleet subsections are
  replaced by one-line pointers to §22.6.

### 🔴 Corrections

- **R21:** "recipe A's extra backlash is the −0.12 compensation geometry", and the
  +14 / ~+28 counts per −0.01 mm trade-off line, are **withdrawn**. The same −0.12 on a correct
  toolpath reads 6 counts.
- **R22:** the session analysis compared B's **breakaway** (0.295) with A's phase-5 **drag_c**
  (0.291). The matched pair is 0.295 vs 0.288 breakaway. B's drag_c is unmeasured.
- **R23:** "outer wall line width differed A vs B" is retracted. Both are 0.4 (Arachne output
  explains the 0.42).
- **Old binary on the bench:** the 09-30 logs show `ladder 1/2/3 A`, `belt=OFF` and no creep
  warning on a travel −240 reading, so the flashed build predates the 2026-09-28 source
  (⟨V23⟩). The measurements are unaffected. Flash the current build (README 7i).

### ⚠ Open conflict (⟨V21⟩)

The owner's plan for this test was a **different plate assembly and the 104.7 belt**. The
session analysis says the 09-28 plate and belt were used. The logs don't say. If it was the
different assembly, the G drop is not attributable to the recipe alone (a shorter belt is
tighter). The 09-28 J01 assembly (recipe A) would then still exist un-re-accepted, and this run
would also be D4's short-belt point.

### Superseded values

- J01 `breakaway_A` ~~0.288~~ → 0.295 (recipe B)
- J01 G ~~31–48 counts (gate exception)~~ → 5.8 counts (pass)
- J01 k ~~50–60 kN/m~~ → 64.7 kN/m (recipe B, at its clamp; clamp-dependent)
- Trade-off line ~~+14 / ~+28 counts G per −0.01 mm~~ → withdrawn (R21)

---

## 0. Changelog — 2026-09-28 (b) J01 −0.12 LOCKED with a B6a gate exception; B11 written; the 3-position ladder method reversed

### ✅ The pulley detour is closed (BELT_DRIVE.md §22.5)

- **Repeat ladders L4–L6** (after a re-clamp): intercept mean **−6.3** vs L3's −4.0, against a
  pre-stated ±8 gate. **Not progressive; wear rejected**; bedding-in fits.
  Single-ladder noise **±7 counts**.
- **The clamp moves stiffness, not backlash:** the re-clamp moved the slope +8% (L3→L4) / +12%
  (means) and k 57.6 → 51.5 kN/m (−11%), with the intercept within noise. k is recorded with
  its clamp.
- **Backlash G = 31–48 counts (0.07–0.12° output)**, above the ≤ 20 gate. **Owner chose to lock
  −0.12 with a visible gate exception** pending the control-side test **D17**; fallback −0.11.
- **B11:** `joint_cal.h` J01 is now the belt-on row (drag_c 0.291, drag_v 2.4e-3, breakaway
  0.288; electrical fields carried, belt-on Ke never). The belt-off baseline row is kept
  verbatim at index 13. Per-direction drag split and row date await owner confirmation.
- **Fleet decisions (owner):**
  - idler holes fixed at **(1.72, ±10.00)** for every joint, with belt length absorbed by
    pulley compensation (D4, first half answered);
  - clamp = **two perpendicular screws into the top plate**, standardised screws. The shared
    jig is rejected because the holes are per-assembly.

### ↩️ Reversed: the 3-position ladder method (R17)

This morning's §22.4.4 change is undone. Each position needs the output unscrewed, and a
re-clamp was measured to move the slope. The premise failed too: L3 and L4–L6 at different
positions agree within 2–3 counts. The rule is back to ladders at one clamp setting, with the
±7-count noise and the I_f choice now stated.

### 🔴 Two corrections to the session analysis, found while recording it

- **Geometry vs backlash compared G at different I_f.** J02's G was taken at 0.30–0.40 A and
  J01's at 0.20–0.29. At matched I_f the J01−J02 gap is **~56 counts, ~2× the 27-count
  geometric prediction**, not "matching" it (R19). The trade-off line now carries both rates
  (+14 predicted / ~+28 cross-joint counts per −0.01 mm).
- **"I_f 0.20 A motor-side" has no measurement behind it (R20).** J01's belt-off breakaway (motor
  alone) is 0.2923 A, as high as the belt-on B4 mean. So 0.29 A is the better-supported I_f
  and **G ≈ 48 counts is the more likely end**. The exception is recorded at the full range.

### Resolved doc conflict: J_total / M6a mode is `t`, not `c`

CALIBRATION §20.2 said `c` mode. CONSTANTS §8.2 shows the `J_rotor` run was a 0.20 → 0.80 V
step (the `KICKV` levels) solved against a "commanded 0.80 V". BELT_DRIVE B7's `t` procedure
was right, and CALIBRATION M6a/M6b are corrected.

### Firmware (comments and prints only)

- `autocalib.h` phase-5 closing print said breakaway is "larger" and to "run M4 by hand".
  It is now B/b, and "~equal" belt-on.
- `autocalib.h` header comment: the retired "ramps through zero" wording is replaced.
- `joint_cal.h` `breakaway_A` field comment: "Expect > drag_c" is now belt-off only (R16).

### Superseded values

- B6a method ~~3-position mean~~ → one clamp setting (R17)
- J01 k ~~56–60 kN/m~~ → 50–60 kN/m, clamp-dependent
- J01 G ~~12 / 30 / 46 (single ladders, L1–L3)~~ → 31–48 (L4–L6 mean, after bedding-in)
- ⟨V17⟩ idler hole → (1.72, ±10.00); ⟨V19⟩ clamp → two perpendicular screws

---

## 0. Changelog — 2026-09-28 J01 pulley −0.12 recorded; ladder G becomes a 3-position mean (REVERSED later the same day — see (b) above); stale procedures and prints corrected

### New data — BELT_DRIVE.md §22.5 (transcribed from session summaries, raw rows not yet archived)

J01, belt 104.85, slicer −0.10 → **−0.12 + precise wall**, top screw off:

- **drag_c 0.414 → 0.291 A (−30%)** with stiffness unchanged (ladder 56.3 → 55.7–59.8 kN/m).
  Session drift fell about 7× per run. The prediction was 0.33–0.39 A, so it failed: drag is
  ~3× more sensitive to belt path than the path model implied.
- **B4 breakaway mean 0.288 A = drag_c**, range 0.085–0.49. There is a direction-biased
  component of up to ±0.18 A (cogging or preload, moderate confidence).
- **B6b f_d 64.3 ± 0.4 Hz**, f_n ≈ 69.5, 8 captures. The J01-vs-J02 15% comparison is blocked
  because the J02 board failed (D15).
- **Backlash undecided:** G = 12 / 30 / 46 counts over three single-position ladders.

### 🔁 Method change — B6a G (BELT_DRIVE.md §22.4.4)

**G = mean over ladders at 3 output positions, with I_f = mean B4 breakaway.** Thresholds
unchanged (≤ 20 / 20–40 / > 40). Reason: position-dependent breakaway makes a single ladder's
G uncertain by ±22 counts, wider than the pass band. **J02's single-position PASS is
caveated, not withdrawn.** New setup rule: log the clamp method and output position with
every ladder.

### Procedures corrected (they disagreed with the firmware or with each other)

| Where | Was | Now |
|---|---|---|
| BELT_DRIVE §22 B6b | `c` mode, `k` only, predicted 100–230 Hz | `t` mode, 4 × `k` + 4 × `K`, `x` after `CAPTURE done`; measured 59–72 Hz |
| BELT_DRIVE §22 B6a | manual 0.05 A stepping in `c` mode | `w` ladder, 3-position G |
| BELT_DRIVE §22.3 ring box | "`x` immediately"; cycle count labelled `f_n` | `x` after `CAPTURE done`; it is `f_d` |
| BELT_DRIVE §22.3 | "neither test is a firmware command" | the swing test is `w` |
| BELT_DRIVE §22.3 B6b pass table | 100–230 Hz | superseded, no band |
| BELT_DRIVE §22.4.4 | I_f "measured by M4 by hand" | `B` / `b` firmware ramp |
| CALIBRATION §20.2 M4 | warns at < 1 s and > 0.4 A | warns on travel > 200 counts and > 0.60 A, `NO MOTION` at 0.80 A; settle into a detent first |
| FIRMWARE §11 | no `K`, `e`, `E`, `w`, `-``5`; `cnt` wraps at 4095 | added; wraps at 16383 |

### Firmware (serial prints and comments only; no behaviour change)

- `w` banner printed **"ladder 1/2/3 A"** since 2026-09-19 while running 0.6/1.0/1.4/1.6 A. It
  now prints the currents from `AC_SW_I`, so the banner cannot drift from the array again.
- The `y` menu didn't list `w`, and said phase 5 needs the belt off (B3 runs it belt-on). Both fixed.
- The M4 comment said breakaway is "always higher" than drag_c. That is true belt-off only (R16).

### Superseded values

- B6b expected band ~~100–230 Hz~~ → 59–72 Hz (already superseded in §22.3 2026-09-05; the §22 row and §22.3 pass table had not been updated)
- B4 belt-on prediction ~~≈ 0.5 A~~ → 0.288 A measured (J01)
- ~~"swing at 0.6 A > 70 → reject"~~ (R15)

---

## 0. Changelog — 2026-09-05 h1 localised to the output side; the swing confirms the corrected arithmetic; the ring test is retired

### ✅ h1 is NOT the belt — and the null is trustworthy because h2 is a positive control

Belt swap, three captures each on 105.0 and 104.7. **h1 (1/output-rev) did not move: −5.1%,
p = 0.42. h2 (2/output-rev) moved +490%, p = 0.0002.** The same captures, fit and window that
saw nothing at h1 detected a **5.9× change at h2 (d = 10.3)** — so "no change" and "the test
cannot see change" are distinguishable here, which is what makes this null worth anything. A
belt-borne contribution to h1 larger than **~0.013 A (~20% of the observed h1)** is excluded.
h1 also survived a full teardown and reassembly.

**h1 is fixed to the output side — pulley eccentricity or output-shaft bearing.** Separating
those two now needs the pulley reprint, because the pulley is bonded.

**h2 is belt-borne and becomes a fleet QC screen**: 0.032 A = 1.5% of standing load = 0.14 N at
the foot. If a joint later shows an unexplained 2/rev, **swap the belt first.**

⚠ **The +6.4% mean-drag difference is NOT evidence the shorter belt ran tighter** — its
p = 0.044 was computed against within-set scatter, not the ±20% reassembly variance the swap
actually incurred. **Read the mean as unchanged.**

### ✅ Swing test: 835 counts, and the corrected take-up arithmetic is confirmed

Reproducible to **±1 count** over four reads. Corrected path (230.96 mm) predicts ~980 → **−15%**;
the retracted path (229.98 mm) predicts ~1570 → **−47%. The approximate formula is excluded by
nearly 2×.** The residual 15% is the unverified "232.00 nominal", the one soft input left.

Inverting makes the drivetrain an absolute length gauge: **belt 105.0 = 231.97–232.07 mm.**
Caliper jig and encoder now agree on a length scale. **The stronger cross-check — the
410-count swing *difference* between the two belts, which depends on neither assumption — has
not been run yet.**

**Tooth skip: `I_skip` > 2.0 A on this plant.** At `CURR_MAX` (13.9 N of belt force) it did not
skip; position held to one count at both limits. An earlier hand observation simply exceeded
2.0 A-equivalent. ⚠ Read the count **5 s** after the limit — ~10 counts of creep converge first.

### 🔴 Ring test: retired. There is no single stiffness number

Twelve captures, four sessions. **`f_d` is set by ring AMPLITUDE, not by the fixture:**
`f_d = 86.9 − 0.522·A`, **r = −0.982, amplitude explaining 96.5% of the variance** (sd 4.86 →
0.91 Hz once removed). The drive is a **softening spring**, consistent with tooth engagement at
3.1 teeth in mesh — a second line of evidence for the pinion-wrap concern.

- ✅ **Fixture check PASSES.** At matched amplitude, 3 → 7 bond spots is indistinguishable
  (residual spread 1.11 Hz vs residual sd 0.91). **The bond was never in the load path.**
  One side, 3 spots, is sufficient.
- 🔴 **"5–8% per session drift" is STRUCK** — measurement scatter plus the amplitude effect,
  not a physical process. No creep, no CA shrinkage.
- 🔴 **`EA ≈ 4 kN` is withdrawn as a standalone number.** The ring measures cord, teeth, hub and
  fixture in series and is dominated by a nonlinear element that is not the cord. **A length
  mismatch is taken up by cord stretch, which tooth compliance does not relieve — so the ring
  cannot decide the plate geometry. The A/B plate question is OPEN again**, not closed as the
  previous session had it.
- **Next instrument: the caliper two-weight test** (1.475 kg / 5 kg, steel-wire splay
  reference, randomised) → `EA_cord`, then print three plate pairs at `d−0.5 / d / d+0.5` and
  swing-test each.

**Architectural consequence, now measured rather than assumed:** drivetrain `f_n` is **59–72 Hz**,
so the **Tier-2 impedance ceiling is 20–29 Hz**, not the 33–77 Hz a 100–230 Hz belt would have
allowed — and it moves the wrong way, **softening exactly when excursions are largest.**
Recorded in `CONTROL.md` §10.

### 🔴 Methodological failure, catalogued

**Boot-to-boot scatter is ~8× within-boot scatter.** A repeat with nothing touched moved the
frequency **+3.96% at "p = 0.0042"**. Every p-value on the bond-count comparison used a
variance ~3× too small, and a physical narrative was built on top of it. **The repeat was
nearly free and should have come first.** The real driver was an untested covariate. §12.

### Firmware — two fixes for defects this session exposed

- **`K` now steps the kick NEGATIVE** (`k` positive, unchanged). Every ring capture in the
  campaign stepped positive because `k` overwrote `target` regardless of any prior jog; four
  captures were taken believing two were negative-going. A drive with 7% dead-time asymmetry
  and one-sided tooth engagement has no reason to be symmetric. `j` explicitly resets the sign.
- **Captures now carry a serial number.** `logStart()` stamps `cap=N` into the **dump header**
  and `logDump()` prints `!! RE-DUMP` on any repeat dump. A session archived four dumps of
  which only three were unique, and nothing in the CSV said so. A header field survives into
  the archive; a console-watching habit does not.

### Also checked and excluded (recorded so they are not re-run)

- **`k` → `x` timing:** `Uq` holds 0.800 for the whole window in all twelve captures; `Iq` drops
  below 1 A in two, at 68.5 / 71.1 ms — past the 8–50 ms fit window. Press `x` after
  `CAPTURE done`.
- **LiPo droop:** real but 7× too small. It did surface a free measurement — end-of-step `Iq`
  fell 4.8% as a *step*, backing out to **+5.1% on resistance ⇒ ≈13 °C of winding warm-up**.
  Back-to-back captures are not thermally identical.

### Superseded values

- ring-test expected band **100–230 Hz → measured 59–72 Hz** (amplitude-dependent)
- fixture requirement **6,700–33,800 → ≈2,300–4,900 N·m/rad**, and passed empirically
- Tier-2 bandwidth ceiling **33–77 Hz → 20–29 Hz**
- swing prediction ~790 (batch-mean belt) → **835 measured** on belt 105.0
- ~~`EA ≈ 4 kN`~~ · ~~"5–8% per session drift"~~ · ~~h1 cause not localised~~

---

## 0-0. Changelog — 2026-09-02 idler rebuild unblocked; the take-up budget halves; output-rate drag modulation found

**Bearings measured (OD 8.99 × width 4.96), conical M3 washers proven at final clamp torque,
plate revision unblocked.** Idler mounting settled as **fixed M3 holes in both plates** — the
idler runs in **double shear across two plates**, so any slot must be set identically on both or
the axis tilts and edge-loads the bearings, voiding the "coasts ≥ 1 s" test the rebuild exists
to pass. Belt-length tolerance moves to **selective assembly**. Two stacked bearings give
9.92 mm under a 10 mm belt; **the idler provides no tracking** — that stays a pulley-flange job.

### 🔴 The 2.00 mm take-up budget was an arithmetic error. It is 1.04 mm

It came from the **approximate** belt-length formula `L ≈ 2C + π(r₁+r₂) + (r₂−r₁)²/C`, a series
expansion valid for `(r₂−r₁)/C ≲ 0.3`. **This drive runs at 0.687**, 2.3× outside it. The exact
closed form gives a no-idler path of **230.96 mm**, confirmed against the CAD span to **3 µm**
(which independently re-confirms r₂ = 34.3775 mm, i.e. 108T and an exact 9:1).

The error is **0.4% of belt length but 48% of the budget** — and it propagated to seven places
across five files. All corrected; §12 has the general lesson. It also made the "230 mm belt"
option look merely risky when in fact **230 mm is shorter than the 230.96 mm path**.

### Belt batch variance measured — the "same batch" assumption is falsified

10 belts, common-mode-cancelling caliper jig: **σ(L) = 0.158 mm, range 0.60 mm.** Inside the
±0.5 mm catalog figure but worth **52–129 N** against an ~80 N working preload, i.e. **±32% to
±81%**. `EA` is the dominant unknown and one `f_n` measurement closes it. **Calibrate the idler
hole on the LONGEST belt** — too long fails silently (unseated teeth near zero torque), too
short fails gracefully (more drag). Protocol fixes recorded for the 20-belt batch: 2 d.p.,
randomised order, reference belt every 5.

### Output-rate drag modulation on J02 — real, and NOT localised

**0.065 ± 0.003 A at 1/output-rev, 18–20σ, antiphase with velocity, reproducing across three
captures to 4.6% cv.** = 3.0% of standing load = **0.29 N at the foot**; 22% of J02's belt-off
breakaway. **h2 is not significant, so the pulley is round.** h18 (2/motor-rev) is the largest
single harmonic at 0.086 A. ⚠ **This is NOT the §9a 1/rev** — that was motor-rate, belt-off,
pinion-off, and closed as a rubbing magnet.

**Cause not localised, and not localisable here:** output rate (9.000 motor revs) and belt rate
(9.667) are **7.4% apart**, needing ~12,800 samples to separate against a `LOG_N` of 1000. The
cheap discriminator is a **different belt**, and it **expires the moment the pulley is bonded**
for the tension fixture.

### Tension method: what replaces the pluck test

**Swing test** (seating) and **ring test** (stiffness), both assembled from existing keys — no
firmware change. ⚠ `a`/logStats does **not** print encoder counts; use the live `cnt=` field.
Ring test uses **voltage mode** for a 2.8× larger kick and an excitation independent of
current-loop tuning; **cycle-counting beats FFT** at these capture lengths (1–2% vs 5–12%).

**Fixture stiffness is a first-class requirement:** `k_fix ≥ 10 × k_belt` = **6,700–33,800
N·m/rad**. Torsional stiffness goes as the **square** of the reaction radius — a 3 mm shaft
clamp instead of the 34.4 mm rim is a **131× penalty**, contributing **442 counts** of swing
against a 790-count signal. **Bond one side, at maximum radius, 3–4 spots**, then run the
acceptance test: **<3% `f_n` shift = fixture out of the loop; >10% = still measuring the
fixture.**

### Falsifiable prediction, stated before the run

| Slack | Predicted swing at ±1.5 A |
|---|---|
| **1.04 mm (corrected)** | **~790 counts** |
| ~~2.02 mm (retracted)~~ | ~1450 |

Nearly 2:1 — the swing test settles the take-up arithmetic empirically.

### Superseded values

- no-idler belt path **229.98 → 230.96 mm**
- take-up budget **2.00 → 1.04 mm**, and "~2 mm of slack" in the §22.1 plant descriptions → **~1.04 mm** (7 sites, 5 files)
- idler end washers: 3 × 6 mm shim → **conical M3** (the prohibition on flat Ø7 DIN 125 is unchanged and is the reason)
- idler mounting: ~~slot one hole~~ → **fixed M3 holes, both plates**
- ~~"same batch → minimal variation"~~ → **range 0.60 mm, measured**

### Withdrawn proposals — do not re-raise

Slotted idler with jackscrew or grub screw · printed spring cartridge · M2 / M2.5 grub screws ·
tape on the idler OD · the 230 mm (115T) belt option.

### ⚠ Two items in the session summary that were NOT adopted, and why

- **§24.1 KV360 was already closed on 2026-08-18**, not on 2026-09-02. What is new is that a
  nameplate **photograph was raised and rejected** — the rule was exercised and held; that is
  recorded. But the summary re-offered *"`Kt` agrees to −0.06% / +0.03%, the tightest agreement
  recorded"* as the supporting evidence, and **that pair was withdrawn on 2026-08-18** (computed
  at a `vbus_scale` M1 later found 1.1% low; corrected to +1.02% / +1.11%). **It is not
  reinstated.** The KV360 verdict rests on the ~5.5-point margin, which is unaffected.
- **The J02 `drag_c` restatement was already in §22.2** with the same amps (0.41–0.50 A) and the
  same reasoning. No new content added; instead the *duplicate* percentage in the B3 row was
  replaced with a cross-reference, per the one-source-of-truth convention. ⚠ **Unreconciled:**
  the doc says 18.9–23.1% / 17.6%, the summary says 18–22% / 16.7%. **The amps agree exactly and
  the amps are the gate**, so the percentages are left as the doc had them rather than
  overwritten from a summary whose divisor is not stated.

---

## 0-0a. Changelog — 2026-08-21 → 08-30 the live-Vbus detour: CLOSED. Four hypotheses killed, the cause named, the fix abandoned, the scaffolding deleted

**The DMA path and the `analogRead` seed disagreed by 60 counts on the same pin. The cause
is an uncalibrated ADC.** `analogRead()` runs `HAL_ADCEx_Calibration_Start()` on every call;
SimpleFOC's `b_g431` init never calibrates. Seed = calibrated converter, DMA = uncalibrated.

**Read the milestone note at the foot before deciding this was worth it. It was not.**

> ### 🔴 2026-08-30 bench result — `ADC_PRECAL` did NOT calibrate anything, and the run was read as a success
>
> ```
> ADC precal CCR_found=0x0  pass1 CF1=0 CF2=0  CCR_set=0x1C0000  pass2 CF1=0 CF2=0
>            ok=0 [p1a=0 p1b=0 p2a=0 p2b=0]
> ```
>
> **All four `adcCalibrate()` calls FAILED, and `CALFACT` read 0 immediately after both
> passes.** The `CALFACT = 71 / 69` seen later in the `p` dump was **not written by this
> code** — it appeared somewhere between the precal print and the probe, i.e. inside
> `driver.init()` / `motor.init()` / `currentSense.init()`.
>
> **The reading that called this a confirmed fix was taken from a partial paste that did not
> contain the precal line**, and inferred `pass2 CF1 = 71` from the probe dump. The full log
> falsifies it. This is the catalogued pattern *"attributing a symptom to whatever changed
> most recently"* — and it nearly promoted a no-op to permanent firmware.
>
> **Leading hypothesis for how 71 appeared (≈70%): `ADCAL` was armed with no kernel clock and
> sat PENDING.** `ADCAL` is set by software and cleared only by hardware — there is no cancel.
> A `DeInit`'d `CCR` is in asynchronous mode (`CCR_found = 0x0` ✅ as predicted), and if
> `RCC->CCIPR.ADC12SEL` is `00` at that point there is **no ADC clock at all**, so `ADCAL`
> cannot complete — and then fires later, inside `currentSense.init()`, at whatever clock and
> whatever moment that init happens to reach. That also explains rev 1's 117 vs rev 2's 71
> without either calibration having succeeded: different `CCR` at the accidental firing.
> **Alternative (≈30%): the init chain calibrates on its own and always did**, in which case
> the founding `CALFACT = 0` observation needs re-examining.
>
> ⚠ **This is a hazard, not just a null result.** A pending `ADCAL` that fires *after*
> `currentSense.init()` has measured its current zero offsets would break the additive-offset
> cancellation the current path depends on — offsets measured on an uncalibrated converter,
> signal read from a calibrated one. **Phase 3 in the same run returned `R_eff` = 0.22227
> ±0.83% against the stored 0.22810 — −2.56%**, which fails the >2% stop criterion set for
> this very check (−1.56% against the 2026-08-20 M2 self-fit 0.22580). Cause not established:
> thermal state and the §8.1d current-range dependence are live alternatives.
>
> **Code response (2026-08-30):** `adcCalibrate()` now returns a reason code instead of a
> bool, **refuses to arm `ADCAL` when there is no kernel clock**, and the precal line prints
> `CCIPR` / `ADC12SEL` / `kerclk` and — decisively — **`ADCAL pending after precal`**.
> `ADC_PRECAL` is **NOT promoted.** Row V1 stays open with one boot left.

> ### ✅ 2026-08-30 (second boot) — V1 CLOSED. Root cause named, scaffolding deleted
>
> ```
> ADC precal CCR_found=0x0 CCIPR=0x0 ADC12SEL=0 kerclk=0
>   rc p1a=NO_KERNEL_CLOCK(refused) …   ADCAL pending: A1=0 A2=0  (clean)
> ADC1 CALFACT=0x0    DELTA ch1=-65.2  ch5=-52.7  ch11=-61.7
> ```
>
> **`ADC12SEL = 0` — there is no ADC kernel clock at the only point in boot where the ADC is
> idle.** `ADCAL` cannot run there, full stop. With the guard refusing to arm it, `CALFACT`
> stayed 0 and the deltas returned to the −65 / −53 / −62 baseline. **That confirms the 70%
> hypothesis outright: the previous run's `CALFACT = 71/69` was our own `ADCAL`, armed
> without a clock, sitting pending, and firing later inside `currentSense.init()`.**
>
> The pre-calibration approach is **dead at this call site by construction** — not tuning,
> not timing, no clock. Deleted, not disabled. **Do not re-attempt without reading this
> entry**: the trap is that arming `ADCAL` there *appears* to work, because something else
> later completes it and writes a plausible-looking factor.
>
> **What ships instead: nothing.** `VBUS_LIVE` stays `false`, the seed path stays
> authoritative (validated against a UT89X at 12.24 / 12.25 / 22.73 V to better than 0.16%),
> and `CALFACT = 0` on both instances every boot is recorded as a board fact (§3).

> ### 2026-08-30 — and the `R_eff` scare was a fitting-range artefact, not the ADC
>
> Phase 3 came back **0.22227 (`CALFACT`=71)** and **0.22185 (`CALFACT`=0)** — **agreeing to
> 0.19%**, inside their own 0.7–0.8% SE. **The converter calibration state does not move
> `R_eff`**, exactly as the differential-cancellation argument predicted. The ADC is
> exonerated; the −2.6% against the stored 0.22810 has a different cause.
>
> Refitting the same 9 points, dropping the top one at a time:
>
> | kept | I_max | R (run B) | R (run A) | U0 | resid rms |
> |---|---|---|---|---|---|
> | ≤ 0.68 V (n=9) | 2.98 A | **0.22185** | 0.22227 | 0.02305 | 3.59 mV |
> | ≤ 0.40 V (n=8) | 1.69 A | **0.22718** | 0.22725 | 0.01859 | 1.76 mV |
> | ≤ 0.35 V (n=7) | 1.46 A | 0.22873 | 0.23001 | 0.01758 | 1.52 mV |
> | ≤ 0.30 V (n=6) | 1.23 A | 0.23024 | 0.23131 | 0.01669 | 1.42 mV |
>
> Monotonic, reproducing across both runs, and **the residual rms halves the moment the top
> point comes out.** A quadratic fits at **1.09 mV rms against the line's 3.59**, and its
> `I²` coefficient reproduces to **2.7%** across two independent runs — a real term, not
> fitting noise. Local slope **0.2316 Ω at 0.5 A → 0.2097 at 3.0 A**.
>
> **The cause is a change we made to ourselves.** `AC_R_V[0]` went 0.46 → 0.68 V on
> 2026-08-20, taking the ladder top from ~1.96 A to ~2.98 A. Modelled on run B's own curve,
> that alone moves the chord slope **0.22617 → 0.22167, −2.0%** — most of the −2.6% gap, the
> rest inside the fit's 0.70% SE.
>
> **Both stored constants are vindicated, including the one nobody was worried about:** the
> quadratic's intercepts (0.01514 / 0.01434) bracket the stored `U0` = 0.014937. The linear
> fit's inflated `U0` = 0.023 is the R–U0 trade-off absorbing curvature — the exact trap the
> retraction note at `autocalib.h:178` warns about, sprung again three sessions later.
>
> ⚠ **This was already documented and I re-derived it.** `joint_cal.h`'s `R_eff` field note
> has said *"R_eff IS NOT A SINGLE NUMBER … the stored values are the 0.46 V ladder"* since
> 2026-08-20. **The search of the archive that would have closed this in one minute was not
> done** — the catalogued *"planning a measurement campaign without searching the archive
> first"*. What is new is the quantification (the quadratic, reproducing) and that J02's
> ladder-top effect is **−2.35%, larger than the −1.59% recorded there**; that note is
> updated in place rather than duplicated.
>
> **Actions taken:** `R_eff` stamped with its fit range `@ 0.28–1.96 A` in J02's row, with a
> line saying a phase 3 reading ~0.222 is **not** a fault and **not** a reason to edit the
> row. **No constant changed.** Mechanism of the curvature deliberately **not** chased —
> static nonlinearity vs a self-heating transient in the descending ladder's first and
> largest point both predict this shape, the test that separates them (repeat `AC_R_V[0]` at
> the end of the ladder) moves no constant today, and the fit spans 0.28–2.98 A against a
> 30 A design point where it cannot be extrapolated anyway.

> ### Deletion executed 2026-08-30 — row V1 closed
>
> Removed from `open_test.cpp`: `ADC_PRECAL`, `ADC_CCR_OPERATING`, `adcCalibrate()`,
> `adcCalRcName()`, the `AdcCalRc` enum, the precal call site, the `adc_precal=` banner
> field, the `CCR` cross-check in `adcClockDump()`, `seed_ch5` / `seed_ch11` /
> `seed_aux_valid` and their pre-init seed block, `seedVsDmaDump()`, `vbusLoadTest()` with
> its `P` binding and help line, and `adcSetSmp()`. **371 lines. Flash 109,336 → 105,380 B
> (−3.9 K, 83.4% → 80.4%); RAM −12 B. The `-Wall` sweep is now clean — `adcSetSmp` was the
> last unused symbol in the file.**
>
> **Kept, deliberately:** the `p` probe and its register dumps — that is the instrument that
> produced §3's sequence map, clock table and `CALFACT` facts, and it is read-only. And
> **`Vdma`** (telemetry field + the two `M2,` columns), because §8.3's promotion condition
> for live Vbus is *`Vdma` against a terminal meter at two bus voltages*, which this field
> makes free at the next M2 for record. It was on the deletion list; keeping it is a
> judgement call and a one-line removal if you disagree.

**What the 2026-08-30 run DID establish, independent of who wrote `CALFACT`:**

| Finding | Evidence |
|---|---|
| **`CALFACT` is additive, 1 unit = 1 ADC count** | Two determinations: 0 → 117 moved the channel mean **+116.5** (ratio 0.996); 117 → 71 moved it **−45.9** (ratio 0.998) |
| **`CALFACT` is CLOCK-DEPENDENT on this part** | 117 and 71 came from calibrations differing only in the `CCR` in force when they fired. **This is the fact that will save someone a session** |
| **The per-channel residual is the 1.6% coupling term, and PA0 is the least-coupled channel by luck** | Predicting from pre-cal Δ + 71: ch1 +5.6 (obs **+5.6**), ch11 +9.3 (obs **+8.3**), ch5 +19.8 (obs **+19.7**). Each channel's residual scales with how far its **predecessor rank** sits from it — PA0's predecessor happens to sit nearest it. **Reordering `SQR1` would remove that luck** (§3) |
| `HAL_ADC_DeInit` clears `ADC12_COMMON->CCR` | `CCR_found = 0x0` ✅ — predicted before the run |
| **`DMA = seed − offset + CALFACT`, offset a fixed per-channel constant** | Three channels, ~1 count. Nothing unexplained remains in the shape |
| Against the meter (UT89X 12.21 V) | seed **12.169 (−0.34%)**, `Vdma` **12.212–12.220 (+0.01…+0.08%)**. Both inside the meter's own tolerance — **these instruments cannot separate them.** Same situation as §24.14b's 0.32% ambiguity |

⚠ **Retracted from the 2026-08-21 analysis: "~3.5 A of positive-current headroom lost."** The
framing was wrong, not just the arithmetic. The zero point moved 2482 → 2556 because the
uncalibrated converter was reading 74 counts **low**; 2556 is the true bias point. Positive
headroom is `(4095 − 2556) × 0.02937` = **45.2 A, and always was.** Nothing was lost — what
changed is that the previously-believed 47.4 A was overstated. 50% margin at 30 A either way.
The §3 "~1.9 A of headroom" line derived from it is withdrawn with it.



### The premise that started it was wrong, and it was in the source and the docs

| Claim | Status | What the register dump showed |
|---|---|---|
| The current sense owns the **injected** group; the regular group is free for VBUS | ❌ **RETRACTED** | `JSQR = 0`, `JADSTART = 0` on **both** ADCs. There is no injected group. Sensing runs on the **regular** group |
| "Regular and injected coexist by design — no pausing, no blind interval" | ❌ **RETRACTED** | True of the silicon, false of this firmware. Was in `open_test.cpp`'s SEED-ONLY block **and** §12 |
| `analogRead()` fails because `HAL_ADC_Start` sees an injected-BUSY peripheral | ⚠️ **symptom right, mechanism wrong** | `ADSTART = 1` permanently. It sees the **regular** group already started |

**The correction matters more than the correction.** `analogRead()` does not fail *beside* the
current sense — it **contends for the sequence the current sense is using**. A HAL version that
stopped the ADC first would let it through, rewrite `SQR1`/`SMPR`, and destroy current sensing
with no error. Boxed in §12 as a prohibition, not a curiosity.

### What the sequence actually is (§3 now carries it)

`L = 5` on ADC1: ranks **12 (PB1), 3 (PA2)** at 2.5 cyc — the phase currents, first, immediately
after the trigger — then **11 (PB12), 5 (PB14), 1 (PA0)** at 47.5 cyc. ADC2 runs one conversion,
ch3 = PA6. Circular DMA, 16-bit, right-aligned, ADC1's buffer at `0x2000050A`, ADC2's at
`0x20000508`. **PA0 is rank 5 and was already in RAM the whole time** — live Vbus never needed
any ADC configuration.

### Four hypotheses eliminated, each by a measurement

| # | Hypothesis | Killed by |
|---|---|---|
| **H1** | Charge sharing from the preceding rank | Fixed bus, 2 min warm-up: **PB14 moved 60.7 counts, PA0 moved 1.0.** Coupling 0.017 against the 0.474 required — ~55σ. The required `f` was also **not self-consistent** across three datasets (0.929 / 0.047 / 0.474). A physical constant does not do that |
| **H6** | An `OFR` offset register programmed on ch1 | `OFR1..4 = 0` on both instances |
| **H4** | PA0's divider node loaded after init | `MODER = 3 (analog)`, `PUPDR = 0 (none)` on PA0, PB12, **and** PB14 — and the delta appears on all three channels, which no per-node loading explains |
| **H7** | Ground/rail shift scaling with board current | `Vdma` across the M2 ladder: **2 counts (17 mV) over a 3.5× current range.** Apparent `dV/dI_bus` = 67 mΩ, and that is a **ceiling** — genuine pack + lead IR plausibly accounts for all of it. Bounded at bench scale |

### H5 confirmed, and the confirmation is not subtle

`CALFACT = 0` on both ADCs after `currentSense.init()`. Calibrating before `driver.init()`:

| Instance | `CALFACT` | DMA before | after | shift |
|---|---|---|---|---|
| ADC1 (ch1 = PA0) | **117** | 1370.9 | 1487.2 | **+116.3** |
| ADC2 (ch3) | **113** | 2481 | 2595 | **+114** |

**Two instances, two independent factors, each reading moved by exactly its own `CALFACT`.**
Uniform across all three ADC1 channels, and the seed unchanged (1436.3 → 1436.2). The seed path
was right all along — which is what the three-point UT89X agreement had already said.

### ⚠ And the fix is mis-tuned by 1.97×, on a named mechanism

`+116.3` delivered where `+59.4` was needed. `CALFACT1 = 117` of a 7-bit 127 is **92% of range** —
a trim code at the rail is a bad calibration, not a big offset. **Inferred cause: the wrong ADC
clock.** `HAL_ADC_DeInit` (inside every `analogRead`) clears `ADC12_COMMON->CCR`, so at the precal
call site `CKMODE = 00` / `PRESC = 0000` → `f_adc` = the async source **undivided** = PLL'P' =
**170 MHz, ~2.8× the part maximum**, where the SAR comparator cannot settle. `currentSense.init()`
later sets `PRESC = /16` → 10.625 MHz. Rev 1 measured the offset at one clock and applied it at
another. **Rev 2 calibrates twice — as-found, then at `ADC_CCR_OPERATING` — and prints both, so
the hypothesis is measured rather than assumed.** `CCR_found = 0x1C0000` would kill it in one line.

🔴 **`ADC_PRECAL = true` is a DIAGNOSTIC BUILD.** A knowingly-wrong `CALFACT` is on both instances;
no run of record may use it until pass 2 is validated. The CFG banner now prints `adc_precal=`.
Banked: the +120-count zero shift on ch12/ch3 costs **~3.5 A of positive-current headroom** —
irrelevant at 3 A, not irrelevant at 30.

### Three side findings worth more than the detour

- **`vbus_scale = 0.008516` now has the linearity check M1 never had.** Seed vs UT89X at
  **12.25 / 22.73 / 12.24 V** — a 1.86× span — at **−0.11% / +0.04% / −0.16%**. Two-point slope
  0.008500, 0.19% from stored, inside meter error. **Stored value unchanged** (§20).
- **PB14's thermal sign contradicts §16.** One session, continuous, DMA path: **1367 → 1427,
  rising** — the *vendor's* sign. §16's own discriminating rule says a rise vindicates the vendor
  formula. The 1431 → 1267 pair it was rejected on was **cross-session and through `analogRead`**,
  which is the frozen-ADC shape §12 catalogues. Verdict unchanged, evidence inverted.
- **Rank-to-rank residual charge is real and small: ~1.6%**, measured on two *different* adjacent
  pairs (0.016 and 0.0165). Nowhere near H1's 47.4%, but it is the bound any future `SMP` argument
  must beat (§3).

### Superseded / retracted in this entry

| Was | Now |
|---|---|
| "Current sense uses the injected group" (`open_test.cpp` SEED-ONLY block, §12) | Regular group, `ADSTART = 1` permanently |
| "VBUS belongs on a register-level regular conversion" (§12) | PA0 is already rank 5 in the DMA buffer — no configuration at all |
| "Reading PB14 requires the register-level ADC work in §8.3" (§16) | It is a RAM read |
| "Both [live Vbus and PB14] are blocked by the same register-level work" (§8.3 deferral review) | Neither is. Both are already in the buffer |
| H1's "textbook residual signature", offered at ~85% on one dataset | Retracted — 1374 landing 47.4% between 1302 and 1439 was a coincidence in a two-number space |
| "The apparent gain is entirely absorbed by per-boot seed scatter" | Correct for the two-bus determination, wrong as a blanket dismissal. Three determinations agree to 0.1%; promoted to *provisional*, still not adoptable |
| `CNDTR` 3-point liveness gate (probe v2) | Wrong: `CNDTR` sits at `L` through the idle gap ~54% of the time, so two equal reads have a ~29% prior. It reported a false VOID. Replaced with a transition count over 128 samples |
| `vbusLoadTest` read as an H5/H7 discriminator | It compares no-load to no-load — `PHASE_ON` with `dc = 0` is static. It bounds *quiescent* only, and now says so in its own output |

### 🔴 Milestone note — this was drift, and the doc set is what said so

README §15 records **F1 closed 2026-08-14** (crystal fitted, 8.000 MHz) and the **bearings arrived
2026-08-15**, which closed the external-constraint window that made parallel work legitimate. This
thread ran **five bench iterations over six days on a path that unblocks no milestone**, against
**task 4 (the idler rebuild) as the binding constraint**. The stopping rule is set and written
into §15: if the rev-2 boot does not land, `ADC_PRECAL = false`, `VBUS_LIVE = false`, execute the
deletion set, and go to the belt chain. **Flash cost so far: 99,808 → 108,716 B (+8.7 K,
76.1% → 82.9%).** The harness now carries more diagnostic than the thing it diagnoses.

---

## 0-0b. Changelog — 2026-08-20 M2 closed on both boards, the phase-3 ladder fixed, and two pieces of reasoning retracted

**The current sense under-reads on both boards, and every force figure in the doc set went UP.**

| | `R` (self-fit) | `c` | **`i_scale`** | ±1σ | σ from 1.0 | |
|---|---|---|---|---|---|---|
| **J01** | 0.22332 | 0.34816 | **0.9621** | 1.18% | **3.2** | **DECISIVE** |
| **J02** | 0.22580 | 0.34953 | **0.9690** | 2.69% | 1.2 | ⚠ **PROVISIONAL** |

`I_true = I_reported / g`, so true current is **3.9% (J01) / 3.2% (J02) higher than reported**, and torque commanded at `i_scale = 1.0` was delivered ~3–4% **over**.

### The change that made M2 work: self-fit `R`

`g = 1.5·R/c` now takes `R` from **the ladder's own `U_delivered`-vs-`I` slope** — same session, same current range, same thermal state — not from a phase 3. Three reasons: a cross-session `R` re-introduces the drift the cold/hot bracket exists to bound; the ladder spans 0.66–2.97 A where `c` is actually determined, giving **SE(R) 0.27% against phase 3's 0.37% at best**; and **`R_eff` turns out to be current-range dependent** (§8.1d). It also removes the need for the phase-3 bypass clip — M2 now needs only the banner, the DCA ladder, and `R_b`.

**All three coefficients validate independently**, which is the real evidence the fit is clean rather than merely well-conditioned: `a` = 0.803 W fitted vs 0.688 W idle + ~0.12 W PWM ripple · `b` = +0.035 W/A vs 1.5·`U0` = 0.020 · `U0` self-fit 0.01350 V vs phase-3 cold 0.01686 V.

### Two retractions

| Claim | Why it fails |
|---|---|
| **"`R_b` is the thing gating this result"** | `R_b` enters `R` (via `U_delivered`) **and** `c` (via `P_bus`), so it largely cancels in the ratio. **±16% on `R_b` moves `g` by ±0.9%; the ±0.6% measurement moves it ±0.03%.** `V_s` is more inert still — a uniform voltage-scale error cancels exactly |
| **"The old thermal bracket is invalid because every hot current is higher, so `R` didn't rise"** | **The conclusion was right and the justification was wrong**, which is the more dangerous failure because it survives retraction. That inference assumes `U0` is fixed; `U0` moved **67%**, and the whole fitted `R` rise was the `R`–`U0` trade-off. The correct diagnostic is a **constrained refit with `U0` held**, never a raw current comparison |

### `AC_R_V[0]` 0.46 → 0.68 V, permanent and fleet-wide

Not a tuning preference — it fixed a diagnostic returning a physically impossible answer.

| | corr(`R`,`U0`) | SE(`R`) | `U0` significance | resid rms |
|---|---|---|---|---|
| old top 0.46 V | −0.89 / −0.90 | 0.68 / 0.69% | 9.1σ / **2.8σ WARN** | 2.19 / 2.23 mV |
| **new top 0.68 V** | −0.84 / −0.84 | **0.41 / 0.37%** | **13.0σ / 12.1σ** | **1.87 / 1.67 mV** |

With `U0` held, `dR` goes from **−1.78% (heating running backwards)** to **+0.71% = +1.8 K** on J01, and +0.43% = +1.1 K on J02. Two back-to-back ~8 J sweeps should do exactly that.

### `AC_M2_V[]` 5 → 8 points, dwell 20 s → 12 s

A 3-parameter quadratic on n = 5 has **2 degrees of freedom**; n = 8 has 5. That is the difference between J01's 0.66% SE(`c`) and J02's 2.37%. **Thermal load is unchanged by construction:** 10.84 W-equivalent × 20 s = 217 J old, 17.94 × 12 = **215 J** new. The 12 s cap also cuts the point-1 heating smear (drift −0.63% J01, −0.93% J02) that corrupted the `(I_reported, Ibus)` pairing.

> ⚠ **The session write-up said "7 points (add 0.62, 0.47, 0.32)". Five plus three is eight, and 4 dof needs 7 — the two halves disagree.** The three named voltages were taken: they are the concrete half, and the thermal arithmetic above independently confirms eight-at-twelve. Recorded in the code so it does not get "fixed" back.

### The M2 prediction table omitted the ammeter's burden — its own gate would have failed a correct run

On instrumentation B or C the DMM is in series, so the board never receives the commanded `Uq`: at point 1 the terminals sit at 11.79 V against a 12.22 V seed. **The old table predicted 3.12 A where J01 correctly measured 2.9679 A — −4.9%, against a ±3% gate.** The table is now burden-corrected (`U_del = Uq·(V_s − Ibus·R_b)/V_seed`, solved self-consistently) and reproduces the measured ladder to **0.6% on every current and 1.5% on every `Ibus`**.

### Seed repeatability: measured, and the standing explanation was wrong

**5 back-to-back power cycles of J02: four boots at 12.29 V, one at 12.34** (5.9 counts, 0.05 V). ⚠ **§24.14's "single unaveraged pre-init `analogRead()`" is retracted** — `open_test.cpp` already discards the first conversion and averages **64**, which suppresses white noise ~8×; a 5.9-count outlier on that mean would need a per-sample sd of ~47 counts. **It is a real per-boot offset, so the requested "add seed averaging" fix would have addressed nothing.** `V_seed` lands 1:1 on `g` — booting on that outlier would have biased J02 by **+1.23%**, with no symptom in the data. **Mitigation is procedural:** session-start banner-vs-meter check, |Δ| > 0.03 V → reboot, now in `CALIBRATION.md` §20.1.

### What moved, and what did not

| Moved ÷0.9621 (J01) / ÷0.9690 (J02) | Did not move |
|---|---|
| J01 breakaway **7.87 → 8.18 mN·m**, **1.271 → 1.322 N/leg**, 12.95 → **13.5%** | **Every reported ampere** — `drag_c`, `drag_v`, `breakaway_A`, and every current LIMIT |
| J02 breakaway **8.10 → 8.36 mN·m**, **1.307 → 1.349 N/leg**, 13.3 → **13.75%** | **`Ke`, `Kt`, and the §24.1 KV verdict** — `Ke` is the coefficient on ω, so it is `g`-independent |
| Cogging **1.42 → 1.48 mN·m** | **Current-loop gains and `τ_e`** — the loop closes on reported amps and was tuned on a measured step response. [`CONTROL.md`](CONTROL.md) §10 unaffected, again |
| Belt-on decomposition **3.44/12.2/29.2 → 3.58/12.7/30.4%** | `R_eff`, `U0`, `L`, `vbus_scale` — all deliberately untouched |

⚠ **`AC_IMAX_ABORT` and every current limit are in REPORTED amps** — the 6.0 A abort actually trips at ~6.24 A. Stated in `fleet_config.h`; it must be resolved before Tier-0 ships a torque limit, and the fix is one decision (derate the limits, **or** correct the gain at source and re-run phases 3–4 fleet-wide) — not half of each.

⚠ **`i_scale` has no consumer in the firmware.** Nothing converts a torque to a current today, so these constants change nothing at runtime except the boot banner. **Tier-0 is the first caller of `calKtCmd()`.**

### Not changed, with reasons

**`vbus_scale` — left at 0.008448 / 0.008516.** The banner read 12.29 against a UT89X 12.27: 0.02 V, 0.16%, against that meter's own ±0.064 V at 12.3 V. **The disagreement is a quarter of the resolving power of the instrument judging it.** Folded into M2's error budget instead.

**`R_eff` — not updated, and §8.1d records why.** Fitted over the extended range it reads 0.21795 (J01) / 0.21894 (J02); like-for-like the stored values sit 1.5–2.5% above the new session, ≈4–6 K of ambient — **a different day, not a calibration error.** Updating `R` without re-measuring `L` would silently corrupt `τ_e` and the loop-gain provenance. **What is new and real: `R_eff` depends on the current range that measured it, by 1.0–1.6%, differently per board** (~0.2173 Ω at 3.05 A vs ~0.2204 Ω at 1.06 A on J01).

**`i_scale` — not pooled.** Inverse-variance pooling gives 0.9632 ±1.10% and is tempting. Two samples can fail to reject commonality; they cannot establish it — and pooling is the exact move that hid a real 0.80% `vbus_scale` difference for two weeks.

**A known systematic, recorded because it points the right way:** `c` is weighted toward the high-current points where `R` is lower, while the self-fit `R` is a linear compromise across the range. **That biases `g` high — true `g` is likely slightly further below unity, not closer.**

**Superseded values:** `i_scale` 1.0 (both rows) · J01 breakaway 7.87 mN·m / 1.271 N / 12.95%, cogging 1.42 mN·m, other-terms 1.271 / 1.228 / 0.228 / 0.323 N · J02 8.10 mN·m / 1.307 N / 13.3% · belt-on 3.44 / 12.2 / 29.2% · `AC_M2_V` = {0.70, 0.55, 0.40, 0.25, 0.16}, `AC_M2_MAX_MS` 20000 · `AC_R_V[0]` 0.46 · the M2 no-burden prediction column (3.12 / 2.44 / 1.76 / 1.08 / 0.68 A) · "use R-bar = mean of cold and hot".

---

## 0-1. Changelog — 2026-08-18 M1 measured per board: both joints rescaled, four cross-checks retracted, ten placeholder rows zeroed

**`vbus_scale` had never been measured on either board with a checked meter, and both stored rows were wrong because of it.** M1 against a UT89X, both boards, same session:

| Board | Joint | Stored before | **Measured** | Factor applied to `R_eff`, `U0`, `Ke`, `L` |
|---|---|---|---|---|
| `B-SPI-01` | J01 | 0.008358 (own board, DT9205A) | **0.008448** | **×1.010768** |
| `B-ABZ-01` | J02 | 0.008358 (**inherited from J01**) | **0.008516** | **×1.018904** |
| | | | **0.80% apart** | |

**The headline is not either number — it is that the two dividers differ by 0.80%.** This is the first time the fleet table has *measured* a per-board divider difference rather than warned about one, so *"M1 is mandatory per board"* is now a demonstrated fact.

**The original error was the meter, not the fit.** `0.008358 × 1.0111` (the DT9205A's measured DCV gain error) `= 0.0084507` against `0.0084479` measured — **0.03% apart**, no residual. That is an independent confirmation of the meter verdict by a completely different route, and it retracts an earlier attribution of the excess to the 2-point fit.

### What moved, and what provably did not

| Moved ×1.0108 / ×1.0189 | Did not move |
|---|---|
| `R_eff`, `U0`, `Ke`, `L` | **`τ_e = L/R`** — 195.87 / 202.74 µs. `L` and `R` scale together |
| `Kt`, and every torque and newton derived from it | **The current-loop gains.** [`CONTROL.md`](CONTROL.md) §10's boxed warning is unaffected; plant DC gain moved 1.1%, negligible against ζ = 0.60 |
| Breakaway **torque** 7.52 → **7.87** mN·m (J01), 7.95 → **8.10** (J02) | **Every ampere and radian**: `drag_c`, `drag_v`, `breakaway_A`, `zea`, `dir`, `i_scale`, INL, `T/T_loop` |
| Foot force 1.257 → **1.271 N** (12.95%), 1.283 → **1.307 N** (13.3%) | **The `U0` deadband** — `U0/R` is a ratio, so the factor cancels exactly and `dead_zone = 0.005` does not reopen |
| Belt-on three-state decomposition 3.4 / 12.1 / 28.9% → **3.44 / 12.2 / 29.2%** | Every ratio, and every percentage-of-a-percentage |

### Four things retracted, because the correction removed their basis

| Claim | Why it fails |
|---|---|
| **"`Kt` +0.38% vs KV360 confirms the VOLTAGE scale"** — it stood in §8.1a, both `joint_cal.h` row comments, `CALIBRATION.md` §20.1 and AUTOCALIB's printed report | **Circular.** `Ke` is computed *from* `vbus_scale`, so the comparison cannot referee the scale it came from. The tight +0.38% was the coincidence of a 1.1%-low divider; corrected, J01 reads **+1.45%** and J02 **+2.33%** |
| **"`R` moved 1.26% while `Ke` moved 0.06%, so the `R` difference is REAL"** | Assumed both boards shared one scale. A per-board difference moves `R` and `Ke` together *within* a board, so a **cross-board** comparison never tested it. Corrected: `Ke` **+0.87%**, `R_eff` **+2.08%** |
| **"Same banner as J01 on the same pack, so J02's divider is VERIFIED"** | The banner is a live measurement of the **pack**, not a constant |
| **"−0.06% / +0.03% — the tightest agreement this project has recorded"** (§24.1 KV360) | Computed at the low divider. Corrected: **+1.02% / +1.11%**. ⚠ **The KV360 verdict itself is unaffected** — KV380 moves to ~6.6% out, so the margin is intact |

> **Steelman, recorded rather than buried.** Two independently built motors agreeing on `Ke` to 0.06% is suspiciously good, and happens only if the dividers are identical — so the old agreement is weak evidence *against* the J01 correction. Direct measurement outranks a cross-joint coincidence, but it yields a **falsifiable prediction: the J01↔J02 `Ke` gap must come back at 0.87% when both ratios are re-measured in one sitting. If it comes back at 0.06%, the dividers are equal and the M1 reads are wrong.** README §24.14.

### The 0.32% that is bounded, not settled — new §24.14

J02 produced two banner/meter pairings against the same UT89X 12.33 V that imply **0.008489** and **0.008516**. `banner / V_true` is a fixed property of a board at a fixed scale and cannot be both 1.000 and 0.997, so one pairing was not simultaneous. **Decided in favour of one sitting at 12.33 V → 0.008516 stored.** The residual 0.32% is below the UT89X's own ±0.66% at that voltage on the 60 V range, so it is not resolvable with these instruments. **The open item is the banner's own repeatability**, which has never been measured: the seed is a *single unaveraged pre-init `analogRead()`* at 8.5 mV/count, and ±2 counts is ±17 mV before boot inrush is considered.

### Ten placeholder rows zeroed

**J03–J12 previously carried J01's `R_eff` / `U0` / `Ke` / `L` and one of three different `vbus_scale` values, under a comment calling them placeholders.** That is precisely the silent-inheritance failure the header warned about — and J02 is the proof it happens. **Every measurable field on those rows is now `0.0f` = NOT MEASURED** (`zea = -1`, `dir = 0` and `i_scale = 1.0f` keep their own documented unmeasured values). `vbus_scale = 0` disables live Vbus, which is byte-identical to a hardcoded divisor and already documented. **A placeholder that looks like a measurement is worse than no number.**

**Also changed:** AUTOCALIB's write-in gate tightened **1% → 0.5%** (J01's own error was 1.08% and would have passed a 1% gate by 0.02 points); `printJointCal()` now warns on `vbus_scale = 0` on a *built* joint; the `I_align` banner prints `?` instead of `inf` when `R_eff` is unmeasured; J01's row comment carried **7.52 and 7.78 mN·m two lines apart** and neither matched its own `Kt` — one figure now, 7.87.

**Deliberately NOT rescaled:** the **A1** row and §8/§8.1/§8.2's A1 tables. Those are the historical record of a superseded assembly with a rubbing magnet; rewriting them would falsify the record. They sit on `B-ABZ-01` at the 0.008358 belief, so **every voltage-derived A1 number reads 1.89% low** — noted in place, on both. `J_ROTOR_KGM2` also stands: if `J` came from `Kt·I/α` it moves ~1–2% against a stated **±12%**, one tenth of the uncertainty already carried. **Flagged, not chased.**

**Superseded values:** `vbus_scale` 0.008358 (both boards) · J01 `R_eff` 0.22108, `U0` 0.01026, `Ke` 0.017750, `L` 43.31 µH, `Kt` 0.026626 · J02 `R_eff` 0.22387, `U0` 0.01466, `Ke` 0.017761, `L` 45.39 µH, `Kt` 0.026641 · J01 breakaway 7.52 / 7.78 mN·m, 1.257 N, 12.8% · J02 7.95 mN·m, 1.283 N, 13.1% · cogging 1.4 mN·m · belt-on 3.4 / 12.1 / 28.9%.

---

## 0-2. Changelog — 2026-08-15 (session close) three robustness tests passed, the CAN work ends, and the doc set turns out to be gitignored

**The CAN ladder is closed and the bearings have arrived, so this entry does two jobs:
it records the last three results, and it ends the parallel work.** Tests A, C and D all
passed; Test B was dropped with its reason; the drop rate came in at zero. From here the
belt is the critical path and CAN is drift (§15).

### The reading that decides everything below: `buserr` is not evidence of a jammer

**The naive interpretation is backwards**, and getting it right is what separates Test A
from Test C — two results that look identical in the symptom and are opposite in meaning.

| | |
|---|---|
| **The symptom, in both tests** | The surviving node's `buserr` climbs at ~7.7 kHz |
| **What it actually means** | *Nobody answered.* Losing the only ACK partner — reset, loopback, unplugged, any reason — produces one ACK error per attempt |
| **The discriminator** | **`TEC`.** ACK errors on an error-passive transmitter are **exempt** and park it at **128, frozen**. Bit and form errors take **+8 with no exemption** and reach `BUS_OFF` in ~32 errors ≈ **4 ms** |
| **The rule** | **`TEC` frozen at 128 = silence. `BUS_OFF` = somebody is driving the bus** |

### Test A — ✅ PASSED. A loopback node babbles, and §23.9's run 1 is explained

| | |
|---|---|
| Result | ESC1 put into mode `e` on a live bus → **the ESP32 went `BUS_OFF`** |
| Why that is decisive | Bus-off is unreachable from ACK errors, so the ESC1 was transmitting into frames it could not see |
| Timing | ESP32 bus-off at `recv = 9560`, ESC1 `txok = 9553` — **7 frames = 35 ms apart**. Bus-off needs ~32 bit errors ≈ 4 ms, so 35 ms is the right order and slightly long, as expected if not every loopback frame collides |
| Consequence | §23.9's run 1 goes from a **60% hypothesis to a measured failure mode**, and the secondary marginal-termination hypothesis is dead |
| What it earns | The **error-active babbler** class §23.4b could only argue now exists on this bench, produced deliberately. **Tier-0 constraint 2 rests on a measurement** |

### Test C — ✅ PASSED. **A rebooting joint cannot disturb the other eleven**

The one with the largest consequence of the four, and it came back clean. ESC1 `RST`
shorted to GND for ~9 s, three times, with the ESP32 as the instrument.

**Six signatures matched the S1 negative control** — where the partner was definitively
silent because it was unpowered:

| Signature | S1: partner unpowered | Test C: partner in reset |
|---|---|---|
| `TEC` | 128, frozen | 128, frozen |
| State | `RUNNING`, never `BUS_OFF` | `RUNNING`, never `BUS_OFF` |
| `qtx` | 9 | 9 |
| `sent` | frozen | frozen at 11422 |
| `buserr` rate | **7683 /s** | **7874 /s** |
| Recovery | automatic, `TEC` → 0 | `TEC` 128 → 97 → 0 |

**The 2.5% rate difference is 3.2 µs of frame period — about 3 bit times**, i.e. stuff-bit
variation from a different payload. And **130.2 µs is not a fitted number**: it is the
figure already recorded as measured in §23.4b, so the S1 column cross-checks against a
capture taken two days earlier.

**The decisive argument is the bus-off that never happened.** ~70,000 errors over ~9 s. A
single bit or form error among them adds 8 to `TEC` with no exemption, so the counter would
have passed 255 within milliseconds. It parked at exactly 128. **Nobody was jamming.**

**Mechanism confirmed:** PC11 floats during reset → internal pull-up → **Standby** →
transmitter disabled. The clone's fail-safe is real and measured.

> ### ⚠ The pre-registered pass criterion for this test was impossible, and that is the
> ### finding worth keeping
>
> §23.10 as written gave **"`buserr = 0` through the hold"** as the clean outcome and
> **"`buserr` climbs, or it goes error-passive"** as the 🔴 failure. **Both wrong, same
> reason:** with the partner in reset the ESP32 has no ACK source, so both happen in the
> clean case too. **A criterion no outcome could satisfy would have read a pass as a
> failure.** Recorded, not silently fixed. The error was writing the prediction in terms of
> the symptom that was easiest to see rather than the one that discriminates.

**Two secondary results from the same capture:**

| | |
|---|---|
| **The firmware ordering window is clean** | `buserr` froze at 297801 and never moved again — including through the window where `startFDCAN()` drove PC11 LOW *before* configuring PB9 as AF9. **PB9-before-PC11 downgrades from Tier-0 requirement to recommended practice** (§23.3, §23.6). Keep the ordering — two instructions against an undocumented pull-up in an unmarked clone part — but stop recording it as a known hazard |
| **Scope caveat** | This was **MCU reset with the board powered.** A joint **power-cycling** while others run is a different case (VCC ramping on the transceiver). Same fail-safe expected, **untested**. Promoting condition: the first power distribution that can cycle one joint independently |

### Test D — ✅ PASSED, from logs already taken — with the headline number corrected

**The decrement law is exact:** `TEC` = 11 after 117 successful frames from 128, and
128 − 117 = 11 to the frame. Two independently-counted integers closing, nothing fitted.

> ### ⚠ Correction: **640 ms, not 585 ms**
>
> The session report headlined *"`TEC` 128 → 0 in 117 frames = 585 ms"*, which contradicts
> its own supporting arithmetic in the same sentence — if `TEC` is 11 at 117 frames it is
> not yet 0.
>
> | | Frames | At 200 Hz |
> |---|---|---|
> | Observed, `TEC = 11` | 117 | 585 ms |
> | **Full recovery, `TEC = 0`** | **128** | **640 ms** |
>
> **640 ms is the constant**, and it is exactly what §23.9 predicted. 585 ms is the length
> of the observation window. Recorded because a recovery constant that is 9% optimistic is
> the kind of number that gets designed against later.

**Why it matters:** a joint that loses and regains its partner is fully back inside **one
100 Hz policy cycle**, so Tier-1 needs no rejoin logic — the hardware handles it.

### Test B — DROPPED, and **S1d closes anyway, PASSED**

**Two reasons, and the second is stronger than the one the session gave.**

| | |
|---|---|
| 1. Moot under the frozen design | It tests a node retransmitting endlessly at 7.7 kHz while a partner joins. With `DAR = 1` no node ever retransmits, so that traffic cannot occur on the control path |
| 2. **It already ran, as Test C** | During Test C the ESP32 *was* error-passive at `TEC = 128` **and retransmitting at 7874/s** — the throttling node, present and active — and the ESC1 booted straight into it. **Both clean within ~640 ms** |

**That is §23.4b's own discriminating test, executed, in the harder configuration.**
Suspend transmission does its job; an error-passive node cannot lock out a joiner. **S1d
is closed PASSED**, §23.4b's box is confirmed by measurement, and the session report's
"one loose connector takes down all twelve legs" stays retracted. Single-shot continues to
stand on the stale-frame argument, which never depended on this.

### Drop rate — 0 in ~10,000 frames, and `DAR` confirmed in both directions

The Tx Event FIFO method that replaced the retracted cross-node one was run.

| | |
|---|---|
| Partner absent | `drop` climbing at **200 Hz** — every frame discarded, none retried |
| After each rejoin | `drop` **frozen**, at 2248 then 4137 |
| Over ~10,000 healthy frames | **0 new drops.** Upper bound **~0.03%** (rule of three) |

**Both directions matter.** A counter that never moved would not prove `DAR` was set — it
could equally mean the counter was broken. It moved when it should and stopped when it
should. The ~15-drop expectation was not contradicted, merely not reached at this sample
size, and it is not worth a dedicated run to tighten: the frozen architecture is a polled
round-robin master, so this prices a hazard the robot will not have.

**An unplanned cross-check:** the second drop window is 4137 − 2248 = **1889 frames =
9.4 s** on the ESC1's clock, against Test C's ~70,000 errors at 7874/s = **9.0 s** on the
ESP32's. Two independently-clocked instruments timing the same reset hold, agreeing to 4%.
*(Window assignment inferred from duration — corroboration, not a primary measurement.)*

### §23.6 — the Tier-0 constraint list is now four, plus an ordered init checklist

| # | Constraint | Earned at |
|---|---|---|
| 1 | Control frames are **single-shot** (`DAR = 1`) | §23.4b, confirmed acting §23.10 |
| 2 | 🔴 **Loopback is a power-on self-test only, with the bus quiescent** | §23.9 run 1, **measured** at Test A |
| 3 | **Drive PC11 LOW before FDCAN goes to NORMAL** — the `S` pin is MCU-owned on this clone, and forgetting it is a nearly silent failure | §23.3 |
| 4 | **Enable HSE and poll `HSERDY` with a timeout and a loud failure** before touching FDCAN — a clock that never came ready presents as a bit-rate mismatch three hours later | §23.5 |

**Added: the ordered FDCAN init checklist** (§23.6) — HSE + `HSERDY` → `FDCANSEL = 0` →
FDCAN clock → PB9/PA11 AF9 → PC11 LOW → `NBTP` NBRP 1 / TSEG1 6 / TSEG2 1 / SJW 1 →
`DAR = 1` → NORMAL. **Every value in it is measured.** Tier-0 should be written from the
checklist rather than re-derived; a fresh derivation would reach some of these and quietly
miss the rest.

### §23.2 — the termination rework is rewritten and deliberately **deferred**

**Changed from "10 of 12" to "all 12."** Leaving two boards terminated makes two of twelve
special, and *which* two depends on a bus topology that has not been designed.

| | Bus end 1 | Bus end 2 |
|---|---|---|
| Bench, after rework | SN65HVD230 breakout, **jumper ON** | discrete 120 Ω in the test lead |
| Robot, 13 nodes | master's transceiver board | discrete 120 Ω at the far end |

**Nothing is removed from the breakout** — its 120 Ω is jumper-selectable, so it is already
adjustable without an iron. Every ESC1 loses its resistor and none of them ever terminates,
which is what makes twelve boards interchangeable.

**⏸ NOT DONE, and that is the correct call.** Three to four hours of irreversible soldering,
not on the critical path, with the two-node bench bus working correctly as it stands.
**Promoting condition: the first bus with more than two nodes** — physical, and a hard gate,
because a third node on an unreworked bus does not degrade, it fails. **Optional 30-minute
de-risk: do J03 alone**, the one board not part of a characterised joint.

Recorded with it: the removal procedure, the **unpowered** acceptance rule, and a per-board
acceptance test with **both halves required** — unpowered open circuit *and* a functional
probe, because "open" only says the resistor left, not that the pads survived.

### Code — the bring-up sketches are in version control

**`tools/can_bringup/`** now mirrors both instruments: `esc1_s1c/` (rev 2) and
`esp32_node/` (rev 3), with their `platformio.ini` files so the toolchain pin travels with
the source. They lived in throwaway projects outside version control, which stopped being
acceptable when they became the tools that **re-validate every board during the rework.**

> ### ⚠ The third sketch is missing
> **The S1b GPIO-only ESC1 probe** — the one that measured the 125 ns transceiver loop
> delay and confirmed `S`-pin polarity — **is not in either project.** `../CAN Bringup`
> holds one `main.cpp` and has no git history. Its *results* are all in §23.3, so nothing
> measured is lost; what is lost is the ability to re-run it — **and §23.2's per-board
> acceptance test names it as the functional half.** That gap has to be closed before the
> rework, not during it. VS Code Local History is the one place left unchecked.

The ESC1 sketch header was **rev 1 while the file was rev 2** — it had already gained mode
`n`, `lastid`, EP/EW/BO decode and the Tx Event FIFO counter. Corrected, with a pointer to
the mirror.

### 🔴🔴 `.gitignore` contains `docs*` — the doc set has never been in version control

**§23.10 (~130 lines) disappeared between the start and the middle of this session.** No
edit of ours removed it; a `diff` against the session backup showed the deleted section was
the **only** difference, and it was restored from that backup. **Second occurrence** —
§23.4b (~150 lines) went the same way on 2026-08-14.

**Investigating why git had not caught either one found the real problem.**

| | |
|---|---|
| `.gitignore` line 7 | **`docs*`** |
| Tracked files in this repo | **11** — `README.md`, `platformio.ini`, four in `src/`, `.gitignore`, `.vscode/extensions.json`, three stubs |
| Ignored | **every file in `docs/`** — `CAN_BRINGUP.md`, `CHANGELOG.md`, **`CONSTANTS.md`**, `BELT_DRIVE.md`, `CONTROL.md`, `SENSING.md`, `HARDWARE.md`, `FAILURE_MODES.md`, `FIRMWARE.md`, `ROBOT_DESIGN.md`, `CALIBRATION.md` |
| When | The pattern predates the 2026-08-13 doc split and **silently swallowed it** |

**This is bigger than the two section losses.** §8 — the master table every other number in
the doc set defers to — has no history, so a changed constant leaves no trace anywhere.

> ### 🔴 And it defeats a mitigation that has been on the risk list for three sessions
>
> README risk #2 and task 3 both say: *"the belt-off baselines expire on first belt
> fitment… archive them to `docs/cal/` … **get them into git before touching a
> tensioner**."* **`docs*` means copying a CSV into `docs/cal/` does not put it in git.**
> The archive step would appear to succeed and protect nothing, and the baselines are
> unrecoverable after B0.
>
> **Not fixed here — it changes what the repo contains, which is a human decision.** New
> **§24.13** records the options. *(`docs/cal/README.md` is tracked despite the pattern, so
> it is already being worked around by hand.)* **Settle it before task 3.**

⚠ Both section losses were whole **trailing** sections, which `diff` makes obvious. **A
loss in the middle of a table would not be.**

### Belt — the bearings arrived, so parallel work ends

**One flag, then dropped** (`CLAUDE.md` milestone discipline): from 2026-08-13 the binding
constraint was external and the CAN ladder was legitimate parallel work. **It is not
external any more.** Rows 4 → 5 → 6 → 7 are the critical path.

Three new findings recorded into §22 ahead of the run:

| | |
|---|---|
| 🔴 **B10 cannot be motor-driven** | At `current_limit = 2 A` the actuator makes **0.48 N·m at the output** (2 A × `Kt` 0.026625 × 9, ignoring `DRIVETRAIN_ETA`, so the optimistic figure) against a **3.4–5.7 N·m** skip threshold — a **7–12× shortfall**. Replaced by a static lever at the output pulley plus a spring gauge. **The reframe:** the motor being unable to skip the belt is not an obstacle, it is a result — no commanded torque can damage the belt |
| 🔴 **Difference J02 against J02's own baseline, never J01's** | The inter-joint drag spread is ~25% and real; subtracting J01's baseline would attribute a board difference to the belt |
| **Two build-spec items at the same teardown** | Shim the top-plate clearance — **wear, not friction**: 0.0196 A is negligible, the debris it makes is not. And determine whether the M4 screw axially preloads the pinion bearing, then define a shim stack or screw depth. **Preload set by feel = twelve joints with twelve different drags** |

### Explicitly not doing, with the reason recorded so it is not reopened

| Item | Why |
|---|---|
| Analyzer bit-time measurement | S2's ±0.97% bound and the analyzer's ±0.83% are a **tie**, on a question that needs ±25% |
| S1e frame width | Deferred to the **real** control frame. 116 bits is this payload, not the robot's. Promoting condition: **message spec frozen** |
| Test B | Tests a configuration single-shot forbids — and it already ran as Test C |
| Termination rework | Deferred, promoting condition recorded above |
| Tightening the drop rate | Prices a hazard a polled master will not have |

### Superseded by this entry

| Was | Now |
|---|---|
| §23.10 as four **pending** tests with pre-registered predictions | Results. A, C, D passed; B dropped |
| §23.10 Test C's pass criterion, **"`buserr = 0` through the hold"** | Impossible as written. **`TEC` frozen at 128** is the criterion |
| "`TEC` 128 → 0 in 117 frames = **585 ms**" | **640 ms** — 585 ms is the observation window, not the recovery time |
| §23.2 "**10 of 12** boards need the resistor removed" | **All 12.** Termination lives in the harness |
| S1d "folded into Test B, to be run deliberately" | **Closed PASSED**, answered by Test C |
| §23.9 run 1's leading hypothesis at **~60%** | **Confirmed** by Test A. The secondary termination hypothesis is dead |
| PB9-before-PC11 ordering as a **Tier-0 requirement** | **Recommended practice.** The window was measured and it was clean |
| The Tx Event FIFO run as an **optional** open task | Run. **0 drops in ~10,000 frames** |
| "**Two** constraints the spec inherits" | **Four**, plus an ordered init checklist |
| B10 as "ramp torque until it skips" | The motor cannot get there. Static lever + spring gauge |
| ESC1 sketch header **rev 1** | **rev 2**, which is what the file already was |

---

## 0a-2. Changelog — 2026-08-14 (S2) the CAN ladder is complete, and HSI16 gets measured

**S2 passed on every pre-registered criterion.** Transport is proven end to end. The
session also produced a measured HSI16 error, a termination artefact worth a rule, and a
bus-off event that turned out to be the most useful thing in it.

### S2 — ✅ PASSED (§23.9)

| § | Change |
|---|---|
| **23.9** | **`rxbad = 0` across 21,800+ frames, `buserr = 0`, `arblost = 0`, ESP32 `RUNNING` with TEC = REC = 0 and `lastid = 0x200`.** The ESC1-alone phase parked at **TEC 128 / `LEC=ACK` / `EP=1` and held flat for 1,200+ frames — `DAR = 1` demonstrating itself**, against the ESP32's 7,683/s hammering in §23.4b. The unplug at the end fired the error-passive watch: negative control passed |
| **23.9** | ⚠ **`EW = 1` was not predicted, and it is correct.** `EW` sets at TEC ≥ 96 and stays set past 128, so **`EP=1` + `EW=1` + `BO=0` is the error-passive signature.** Predicting `EP` alone was incomplete — logged rather than quietly absorbed |
| **23.9** | **The failure-branch table earned its place on first use.** Run 1's `BO=1` was diagnosed in one step by its entry *"not a missing partner — that parks at 128"* |
| **23.9** | ⏸ Two predictions **not** settled: the TEC 128 → 0 recovery (~640 ms) was never captured because the run was already in steady state, and the drop rate needs a different method. **Tests D and the Tx Event FIFO run** respectively |

### HSI16 measured at +0.141%, which closes the crystal (§23.7, §23.9)

| § | Change |
|---|---|
| **NEW 23.9** | **The ESP32's `recv` runs ahead of its own `sent`, and the gap grows — +3 at `sent=2601`, +30 at `sent=21801`.** Both nodes transmit at 200 Hz off their own `millis()`, so **(30−3)/(21801−2601) = +0.1406% is pure timebase drift.** STM32 `millis()` is HSI16 (±1%); the ESP32-S3's is a 40 MHz crystal (±20 ppm). **A reusable constant for anything else clocked off HSI16 on this board** |
| **23.7, 23.9** | **Applying it corrects §23.7's `TSCV` number**: the "5,000 ms" window was really 4,993.0 ms, so **7.9826 MHz → 7.9939 MHz, −0.217% → −0.077%.** The correction moves it toward 8.000 **by a factor of 2.8**, which is what removing a real systematic error looks like |
| **23.7** | ✅ **The independence box in §23.7 is RESOLVED, and not the way it expected.** It said the pending check was an analyzer capture. **S2 supplied it instead:** in NORMAL mode the reference is the partner's crystal, not HSI16, so 21,801 error-free frames bound f_HSE at **±0.97% (7.92–8.08 MHz)** with no HSI16 in the chain. `LEC = 0` *is* a frequency bound in NORMAL mode for exactly the reason it was worthless in loopback — the clock it is checked against belongs to someone else |
| **23.9** | **The analyzer capture is therefore NOT NEEDED and S2b is closed unrun.** Its 10-bit cursor span gives ±0.42% one-sample, **±0.83% two-cursor — essentially a tie with S2's ±0.97%** — on a question that needs ±25%, the nearest candidate being 12 MHz. **HSE = 8.000 MHz, closed** |

### Termination closed, and an acceptance rule that would otherwise have been unusable

| § | Change |
|---|---|
| **NEW 23.9** | **59.5 Ω unpowered against 60.2 Ω predicted (−1.2%, inside meter tolerance). The bus is correctly terminated.** The confusing powered readings are fully explained: **53 Ω with 2 terminators implies a 441 Ω extra parallel path, 37 Ω with 3 implies 477 Ω — the same fixed element to 8%**, present powered and absent unpowered. That is a **powered transceiver's recessive bias network** pulling both lines toward VCC/2 |
| **23.2** | 🔴 **ACCEPTANCE RULE FOR THE 12-BOARD REWORK: MEASURE UNPOWERED, target ~60 Ω.** With 12 powered ESC1s those bias paths parallel to **~38 Ω**, which would swamp a 60 Ω reading completely — **a correctly reworked bus and a badly broken one would read the same.** This is the rule that makes the rework's acceptance test mean anything |
| **23.2** | The **37 Ω** three-terminator reading also stands alone as the **measured** over-termination datapoint — below the 45 Ω minimum transceiver load. §23.2's argument was arithmetic until now |

### Run 1 — a bus-off that is a finding, not a fault

| § | Change |
|---|---|
| **NEW 23.9** | **The ESP32 reached `BUS_OFF` with `buserr = 34,710`, `sent = 9`, `qtx = 9`** — nine queued, none completed. §23.4b established a lone node **cannot** bus-off from ACK errors, so these were **bit or form errors**; the ESC1 meanwhile showed only `LEC=ACK`, so its own frames were clean. Something was corrupting the ESP32's frames specifically |
| **NEW 23.9** | **Leading hypothesis (~60%): the ESC1 was still in mode `e` from the mux test.** External loopback **disregards the Rx pin**, so the node cannot lose arbitration and cannot detect bit errors — **it transmits blind every 5 ms over whatever else is on the wire.** The ESP32 sends recessive in arbitration, reads back dominant, raises a bit error, and reaches bus-off in ~32 frames. `r` recovered it and both ran clean **on identical wiring**, which excludes wiring, polarity and transceiver faults |
| **NEW 23.9** | **A rebuttal recorded so it is not re-made:** the ESC1 log showing `CCCR = 0x1040, TEST = 0` does **not** refute this — **that log begins when `n` was pressed**, and says nothing about the earlier window |
| **23.6, 23.9** | 🔴 **NEW TIER-0 REQUIREMENT: a node in external loopback is a babbling node that does not arbitrate. Loopback is a power-on self-test only, with the bus quiescent, and the node must be in NORMAL mode before any other node is enabled.** This is the **error-active babbler** class §23.4b named as the real one-kills-many mechanism — the one single-shot does *not* fix. **§23.4b could only argue it; run 1 is it happening on the bench** |

### The drop-rate method was confounded — retracted and replaced

| § | Change |
|---|---|
| **23.9** | ⚠ **RETRACTED: comparing ESC1 Δ`sent` against ESP32 Δ`recv`.** **S2's own data exposed two confounds, each larger than the signal:** status windows free-run so "the same 5 minutes" misaligns by up to ~1 s = **±200 frames**, and the measured 0.141% timebase drift is **±84 frames** — against an expected **~15**. The irony is that the drift confound is itself one of S2's best results |
| **23.9** | **Replaced with a single-node method** using the **Tx Event FIFO**, which records only *completed* transmissions. With `DAR = 1` an arbitration loss produces no event, so **`sent − tx_ok` IS the drop count**, on one board against one clock |
| **23.9** | **Priority: low, for an architectural reason.** A polled round-robin master has one initiator at a time and zero contention, so **this number prices a hazard the robot will not have.** Worth taking only to get the figure on record before the message spec freezes |

### S1e deferred properly, and the message spec is now the largest open item

| § | Change |
|---|---|
| **23.7** | **S1e DEFERRED with a physical promoting condition — "the CAN message spec is frozen", not a date.** Stuff-bit count is data-dependent, so measuring an ascending-counter `0x200` frame says nothing about a `{p_des, v_des, kp, kd, τ_ff}` frame. §23.7 was already saying "116 bits is this payload, not the robot's" while still listing the measurement as pending; **the two were inconsistent and the deferral resolves it** |
| **23.6** | **The message spec inherits a third earned constraint** — the loopback rule — alongside single-shot and polled discipline. ⚠ **It is design work, so it gets a fixed scope: frame IDs, field packing and scaling, and the poll schedule. Freeze it, then measure it** |

### NEW §23.10 — tests A–D, the questions the ladder never asked

| § | Change |
|---|---|
| **NEW 23.10** | **The ladder proved the bus works; these four ask what it does when something goes wrong.** ~30 min, no rewiring. Written as executable procedure — steps, outcome tables, and a stated numeric prediction each. **Run A first**, since it is the only one that can leave a node bus-off |
| **NEW 23.10** | **A — does a loopback node babble?** Promotes run 1 from a 60% hypothesis to a measurement. Predicted signature: ESP32 `buserr` climbing within a second while **the ESC1's own counters stay clean** — that asymmetry is what distinguishes it from a wiring fault |
| **NEW 23.10** | **B — boot-order robustness. This IS S1d**, open since §23.4b, now run deliberately rather than tripped over. Predicted recovery inside ~1 s, i.e. 128 frames at 200 Hz = **640 ms — the same constant Test D measures from the other side** |
| **NEW 23.10** | 🔴 **C — what a rebooting joint does to the bus. The largest consequence of the four, and never tested.** During reset PC11 floats → Standby → transmitter disabled (fail-safe, protective), but **PB9 floating relies on the transceiver's internal TXD pull-up to idle recessive; if it settles low the bus is jammed for as long as reset is held.** There is also a firmware window: `startFDCAN()` enables the transceiver *before* configuring PB9 as AF9. **On twelve joints this is the difference between one leg dropping out and all twelve** |
| **NEW 23.10** | **D — the recovery constant nobody captured.** Predicted TEC 128 → 0 in ~640 ms, with **`EW` clearing at 96 and `EP` at 128 on the way down** — a second check on the `EW` reading above |

### Firmware — ESC1 S1c sketch

| Where | Change |
|---|---|
| Tx Event FIFO | `FDCAN_STORE_TX_EVENTS` on the periodic TX, drained **every** `loop()` pass, with `txok=` and `drop=` in the status line. **`muxTest()` deliberately keeps `NO_TX_EVENTS`** — it floods 17k frames while blocking `loop()`, so a 3-deep FIFO would overflow and make `tx_ok` a lie |
| ⚠ **Two API errors in the proposed diff** | **`HAL_FDCAN_GetTxEventFifoFillLevel()` does not exist** in this HAL — only the Rx equivalent ships — and **`HAL_FDCAN_GetTxEvent()` takes two arguments, not three**; there is no `FDCAN_TX_EVENT_FIFO` selector. The diff **did not compile**. Fill level now read from `FDCAN1->TXEFS & FDCAN_TXEFS_EFFL_Msk`, matching how this file already reads PSR/ECR/NBTP/TSCV |
| Depth check | The "3 deep" claim **verified against the HAL**, not assumed: `SRAMCAN_TEF_NBR = 3` |
| Build | J01 SUCCESS, flash 17.9% → 18.1%. Double-backslash sweep clean on all three projects |

**Superseded by this entry:** the cross-node drop-rate method (§23.9) — **retracted, confounded**; "measure the analyzer bit time at S2b" — **not needed, S2 bounds it tighter than required**; the raw `TSCV` figure 7.9826 MHz — **superseded by 7.9939 MHz** once HSI16's +0.141% is removed; powered termination readings as an acceptance test — **must be unpowered**.

---

## 0a-1. Changelog — 2026-08-14 (bench) S1b + S1c passed, the crystal identified, and two claims retracted

**The CAN ladder is one rung from complete.** S1b, S1c and the crystal all closed in one
sitting; only S2 remains. The documentation had fallen four sessions behind the bench,
which is what this entry clears.

### The clock question — CLOSED (§23.5, NEW §23.7, NEW §23.8)

| § | Change |
|---|---|
| **23.5** | ✅ **CLOSED. HSE is fitted and it is 8.000 MHz.** `HSERDY` sets, `FDCANSEL = 00 = HSE` is used as-is. **Fact 2 — the `fdcan_ker_ck` maximum — never had to be answered**: it only mattered on the no-crystal branches, which a fitted crystal deletes. The 70%-confidence 80 MHz recall was never load-bearing |
| **23.5** | **The whole SYSCLK coupling is retired without ever firing.** The kernel-clock source table, the 340 MHz VCO problem, the SYSCLK-160 branch and the 500 kbit/s fallback were the map of what to do if no crystal existed. One does. **`T_DELAY_PER_LOOP`, the PWM frequency and the loop rate are untouched, and the 1 Mbit freeze never had to be re-opened** |
| **NEW 23.7** | **Crystal identified, not measured — the right framing.** You need to know which *standard* value it is (8/12/16/20/24/25), and the closest pair is 4.2% apart, so ±1% is ample. **4,989,144 FDCAN timestamp ticks in 5,000 ms → 7.9826 MHz → 8 MHz at −0.217%, with the runner-up 232× further away.** `HSE_ASSUMED_HZ` was already right; no constant changed |
| **NEW 23.7** | **A 75%-confidence guess retired by its own failure signature.** The method risked `TSCV` counting only during frames. `measureHSE()` calls `delay(5)`, so the bus goes idle after ~3 frames drain — a traffic-gated counter would have read ≈0 and tripped the sketch's guard. It read **99.78% of wall-clock**. `TSCV` free-runs, empirically, and is now a validated instrument |
| **NEW 23.8** | **8 MHz forces 8 tq at NBRP 1 — there is no other option**, and 8 tq is the CAN minimum, so the margins were checked rather than assumed. Propagation **875 ns vs 253 ns required = 3.46×**. Oscillator tolerance **0.4854% budget vs ±50 ppm crystal = 97×** |
| **NEW 23.8** | ⭐ **The tolerance row retroactively proves HSE was mandatory, not stylistic. HSI16 is ±1% — 2.06× over the 0.4854% budget.** A PCLK1/PLLQ kernel clock would have given **a bus that works on a cool bench and fails intermittently when warm.** That was general reasoning while the question was open; it is now arithmetic against a measured requirement |

### Two retractions, and the mux test that replaced one

| § | Change |
|---|---|
| **23.5** | ⚠ **RETRACTED: "external loopback proves the clock, the bit timing and the AF9 pin mux."** The last third is false — the M_CAN spec performs an **internal feedback from Tx to Rx and disregards the actual Rx pin** in *both* loopback modes. **`e` is expected to be byte-identical to `i` even with a broken mux and no transceiver.** The claim had reached the doc *and* the sketch header, and would have licensed skipping to S2 on a false pass |
| **NEW 23.7** | **What actually proved the mux: `TEST.RX`, which monitors the real pin.** Result **52.05% dominant.** The sketch predicted "20–50%" — a guess with no arithmetic behind it, and **the measurement landed outside the band it was told to expect.** The grounded prediction is ~50%, counted field by field: 58 dominant of 116 bits. **Landing within 2 points of the frame's own duty cycle is a far stronger pass than clearing a 1% floor**, and it is why the verdict is three-way — a stuck-dominant buffer reads ~100% |
| **23.3** | ⚠ **"Point the analyzer at PB9" was never executable. PB9 and PA11 are package pins with no pads.** The only CAN-carrying pads are CANH/CANL, and **neither decodes on the FX2 at 5 V**: CANH sits above V_IH in both states, CANL's dominant lands in the undefined zone. **No divider fixes it** — CANH needs `k > 0.515` *and* `k < 0.323` |

### S1b recorded at last, and the pin-5 substitute vindicated (§23.3)

| § | Change |
|---|---|
| **23.3** | ✅ **S1b PASSED, results recorded** — they were referenced from four places without ever being written down. **`S` pin LOW = Normal, HIGH = Standby, FLOATING = Standby, all measured.** PC11 reaches pin 8. **Transceiver loop delay 125 ns**, which is what makes §23.8's propagation row a measurement rather than an estimate |
| **23.3** | **The CANH/CANL bias reads 2.48 V ≈ 0.5 × `VCC_5V` — exactly the substitute mode-check specified when pin 5 turned out to be VIO rather than VREF.** The replacement works. Provenance of the 2.48 V is flagged rather than assumed: it is reported alongside threshold analysis, not in a pre-flight table |
| **23.3** | PB9/PA11 confirmed at S1b **as GPIO** — bit-banged, which is precisely why it did **not** cover the AF9 path, and why the `x` test was needed |

### Documentation loss found and repaired

| Where | Change |
|---|---|
| **23.4b** | ⚠ **The entire S1 section had gone missing in a restructure of `CAN_BRINGUP.md`** — results table, bus-off retraction, arbitration proof and single-shot rationale. **Restored verbatim from the session copy.** These are the only record of ~200k frames of two-node traffic, and the ESP32 sketch header plus §§23.7–23.9 all cite the section by number |
| **23.2** | ⚠ **The retracted bus-off claim had reappeared in the physical-layer primer** in the same restructure — *"after roughly 32 failures (TEC > 255) shuts itself off."* Corrected in place, boxed rather than deleted. **Second time this claim has come back; it is persuasive and wrong** |
| **NEW 24.11, 24.12** | Both retractions logged in the integrity register, so they are visible from the front page rather than only where they were made |
| Header | `CAN_BRINGUP.md`'s status banner **removed** rather than updated, to match the new convention that status lives only in the §15 task board. The header keeps only durable warnings — the two retractions and the invalidated PB9 instruction |

### Frame length, and what it does to the load figures

| § | Change |
|---|---|
| **23.7** | **S1e substantially answered from two directions that agree.** Field-by-field count gives **116 bits**; the mux-test flood (17,318 frames in 2 s = 8,659/s) implies **115.5 bits** at 100% occupancy. Both sit at the bottom of the 113–125 range §23.4b bracketed |
| **23.7** | Load figures firmed: S1 two-node bus **4.64%**, and 12 joints × 2 frames at 100 Hz **27.8%**. §23.5's ~130 bits was worst-case stuffing and **stays as the conservative design figure** — the real control frame carries the actuator contract, not ascending test bytes |
| **23.7** | ⚠ **"Confirmed three ways" overstates it — two ways, plus one pending.** `LEC = 0` is not a frequency confirmation at all: in loopback TX and RX share one clock, so it passes at *any* crystal value. And `TSCV` and the flood rate both compare against `millis()`, so both inherit the same HSI16 reference. **Changes no conclusion** — the discrimination needed is 50% and the shared reference is good to 1% — but the genuinely independent check is S2's analyzer measurement, still pending |

### S2 pre-registered (NEW §23.9), and the message spec unblocked (§23.6)

| § | Change |
|---|---|
| **NEW 23.9** | **Full S2 procedure with predictions stated in numbers before the run.** Bus load 4.64%; ESC1 TEC 0→128 in ~80 ms then **parking without hammering, because `DAR = 1`**; recovery 128→0 in ~640 ms; ESC1 frames measuring **10.00 µs per 10 bits on D1** — the prize, because it closes the crystal ID with no `millis()` and no HSI16 in the chain |
| **NEW 23.9** | **The first empirical price tag on single-shot.** With `DAR = 1` a frame that loses arbitration is dropped, not retried, and `0x100` beats `0x200` every time. From S1's 29 losses in ~113,000 frames, **ESP32 `recv` should lag ESC1 `sent` by ~0.026% — one frame per ~19.5 s.** Measure as *deltas* over 5 minutes; rule out `rxmiss` first or the lag is queue overflow |
| **NEW 23.9** | ⚠ **The stated reason for the ESC1-first startup order is partly wrong; the order is still right.** The claim that a hammering error-passive ESP32 would starve the ESC1 **does not follow** — suspend transmission forces 8 recessive bits before each retry, the same correction already boxed in §23.4b. Keep the order for a better reason: it is the only chance to watch `DAR = 1` park cleanly. **And if you power up in the "wrong" order, that run IS S1d — do not restart it** |
| **23.6** | ✅ **CAN message spec + RL vector UNBLOCKED** — they waited on the bit-rate decision, which is now closed. Inherits two earned constraints: **single-shot control frames**, and **prefer a polled/round-robin discipline**, because free-running nodes pay ~0.026% to arbitration and a polled bus pays none. **Decide the discipline before the frame IDs** — a priority-ordered ID map assumes contention exists |

### Firmware — ESC1 S1c sketch (`CAN Bringup`, separate project)

| Where | Change |
|---|---|
| `printStatus()` | **The S2 diff.** `LEC` decoded to a name (`STUFF`/`FORM`/`ACK`/`BIT0`/`BIT1`/`CRC`) instead of a bare numeral, plus **`EP` / `EW` / `BO`** from PSR and **`lastid`**. `LEC` is the field that discriminates almost every S2 failure branch, and a numeral is not a diagnosis. PSR is snapshotted once — it self-sets to 7 on read, so each line reports errors *since the previous line* |
| `last_rx_id` | Added and set in the RX drain, so `lastid` shows which node was last heard from |
| Header, banner, `startFDCAN()` | **Found beyond the report:** three places still told the operator to watch PB9 on the analyzer, which has no pad. All corrected, with the CANH/CANL threshold reason recorded so nobody re-derives it |
| `muxTest()` | **Found beyond the report:** the printed expectation still read "20–50%", which the real 52.05% result **falls outside**. Corrected to ~50% with the frame-duty-cycle arithmetic in the comment |
| Build | J01 SUCCESS, flash 17.9%. Double-backslash sweep clean on all three projects |

**Superseded by this entry:** "external loopback proves the AF9 pin mux" (§23.5 and the
sketch header) — **retracted**; "a lone node goes bus-off after ~32 frames" (§23.2, for
the second time) — **retracted**; "point the analyzer at PB9" (§23.5 and three places in
the sketch) — **not executable, PB9 has no pad**; "expected 20–50% dominant" (`muxTest`) —
**the band was wrong, not the result**.

---

## 0a0. Changelog — 2026-08-14 (schematic) the ESC1 clone's own schematic and vendor listing, extracted

Eleven images from the seller's product page: a schematic sheet, board photographs, a
specification table and MCSDK/SimpleFOC screenshots. Source rank 2 for the schematic,
rank 4 for everything the vendor asserts. Nothing here overrules a measurement; two items
**correct** documentation that was wrong, and two **resolve** open unknowns.

### Corrections to existing documentation

| § | Change |
|---|---|
| **2, 7** | 🔴 **THE SHUNTS ARE 3 mΩ, NOT 20 mΩ.** The vendor's board photo shows three parts silkscreened **`R003`**. §2 had claimed *"20 mΩ (`R020`), amp gain scaled to compensate — therefore use the genuine constants."* **The conclusion was right and the justification was invented.** The firmware constant `LowsideCurrentSense(0.003f, -64.0f/7.0f, …)` matches the physical part. **This is worse than a wrong number:** anyone who checked the 20 mΩ claim, found 3 mΩ and followed the stated logic would have *changed a correct constant* |
| **2** | **Consequence for `i_scale`: the larger unknown is closed.** Sense scale is `R_shunt × G_amp`; `R_shunt` is now confirmed to match the assumed value, so a 3.3× mismatch is excluded by direct observation. Only the amplifier gain remains unverified. **M2 is NOT escalated** — and the vendor's `RSHUNT 0.010` / `AMPLIFICATION_GAIN 5.18` is explicitly dismissed, because the shunt value in that same header is falsified by the vendor's own photograph, which makes the whole header a copied template |
| **2** | The listing's *"perfectly compatible with B-G431B-ESC1 — all its examples work directly"* is **marked false.** Pad-level pinout is compatible; the EG2124A gate-driver contract is not, and that was the entire M0 saga |
| **3** | `Temp_ADC` = **PB14** and `SpeedBT_ADC` = **PB12** promoted from *"probable"* to **confirmed** against the schematic |
| **16** | ⚠ **The vendor's PB14 transfer function has the wrong sign for our data.** They publish `V0` = 1.4 V at 25 °C with **+0.019 V/°C**; our two readings went **1431 → 1267 counts as the board warmed**, the wrong direction, and both imply ~5–12 °C on their own formula. **Not adopted.** Either the clone inverts the divider leg or the readings are invalid — and a frozen-ADC artefact has already been caught once on this peripheral (§12). Discriminator: one deliberate 1.5 A / two-minute warm-up watched continuously |
| **16** | ⚠ **The vendor rates the board BELOW the design operating point:** continuous **< 10 A**, instantaneous **< 40 A**, and *"very small, mainly for learning, not recommended for high-current drive."* Against **30 A peak per motor** that is 75% of the absolute instantaneous claim and 3× the continuous one, on twelve boards. The FETs are not the limit — the card is. **Named as a risk to instrument (M10 + FET temperature after a realistic duty cycle), not to reason further about** |

### Unknowns resolved or narrowed

| § | Change |
|---|---|
| **23.3** | ✅ **THE CAN TRANSCEIVER IS `SIT1042QTK/3`** — a TJA1042T/3-class part, not a TCAN330. On **`VCC_5V`**, and the `/3` suffix means **3.3 V-compatible logic pins**, so PB9/PA11 drive it directly. Pinout 1 TXD · 2 GND · 3 VCC · 4 RXD · 5 VREF · 6 CANL · 7 CANH · 8 S |
| **23.3** | 🔴 **`CAN_SHD` → pin 8 `S` (standby control) → `PC11`.** LOW = Normal, HIGH = Standby, with a fail-safe internal pull-up — so **a floating, unconfigured `S` sits in Standby, where the transmitter is disabled and RXD does not mirror TXD.** That is exactly the S0/S1b pass criterion, so an ESC1 that never drives PC11 presents as a dead transceiver and sends you probing wiring and FDCAN registers instead. **Tier-0 must drive PC11 low before enabling FDCAN.** The polarity is solid; the internal pull-up is ~75% datasheet recall and is on the S1c critical path |
| **23.3** | **`VREF` (pin 5) is a free mode indicator** — ≈ 0.5 × VCC in Normal, inactive in Standby. A static DC check that distinguishes the two before any frame is sent |
| **23.3** | **S1b reduced from ~30 min to ~10.** Identity and supply rail came from the schematic; the rung is now only "does PC11 reach pin 8, and are PB9/PA11 routed as drawn". **The cheaper source should have been read first** |
| **23.5** | 🟡 **Fact 1 narrowed, and its test replaced.** The schematic marks the **LSE pins (3, 4) explicitly no-connect** while the **HSE pins (5, 6) carry routed nets** — on a sheet that crosses out unused pins, that asymmetry says HSE is used. No crystal is identifiable in the photos, so it is evidence, not proof. **The decisive test is `HSEON` → poll `HSERDY` with a timeout, then an LED-and-stopwatch 1 Hz check to separate 8 from 24 MHz** — ten minutes, no partner, no cable. ⚠ **Do not use `MCO` to measure it: `MCO` is PA8 and PA8 is `TIM1_CH1`, a motor phase** |
| **3** | **`PB10` / `48V_EN` both divider ranges now tabulated:** **34.23 V** full scale as our firmware leaves it (measured, 0.07%) vs **65.3 V** driven high (vendor factor 0.050562). Ratio **1.907**, so the high range adds a resistor rather than switching a leg cleanly. The vendor's *"pull PB10 high above 24 V"* is an **MCSDK-configuration requirement** (their 8–28 V disable-PWM window), not a hardware one — the shipped range covers 6S at 25.2 V with 26% headroom. The real rule is the **ADC ceiling at 34.2 V** |
| **3** | ⚠ **New silent-failure hazard:** the 34.23 V calibration is valid only with PB10 in its reset default, which nothing currently writes. **Any future firmware that drives PB10 moves `vbus_scale` by ~1.9× with no error and no symptom**, and `R_eff`, `U0` and `Ke` all scale linearly with it — every `joint_cal.h` constant would silently be wrong by a factor of two. If Tier-0 configures port B wholesale, PB10 needs an explicit leave-alone comment at the init site |
| **NEW 2a** | **Vendor documentation section.** Driver-board spec (7.2–48 V, 60 V component rating, **50 kHz max PWM**, 26 × 51 × 1.6 mm 6-layer), the dock (**3 A connector limit** — *"above 3 A solder motor wires directly"*, against a 30 A design point; onboard ST-Link with SWD + VCP + mass storage), the MCSDK power-stage header with each value adjudicated, the board pad map, and the vendor's SimpleFOC example gains marked **not applicable** (`P = 1.0, I = 200` against our measured `0.1 / 335`) |
| **NEW 2a** | ✅ **The vendor confirms the silkscreen-modes finding in writing:** *"ABZ(ABI), HALL, I2C — the three interfaces multiplex the same IO."* §3 had established this from a bench failure; it is now corroborated by the manufacturer |
| **NEW 2a** | **The 6S electrolytic gate is not cleared, but the prior improves.** §19 lists the unreadable capacitor markings as an uncleared gate; the vendor claims 48 V board rating with 60 V components, at rank 4. **Step 2 of the staged commissioning still has to be run** |
| **3** | ⚠ **The vendor's own recommended I2C wiring is a boot trap.** Their AS5600 example calls `I2Cone.begin(PB7, PB8, 800000)` — **PB8 is BOOT0**, and I2C needs pull-ups, so a pull-up on BOOT0 holds the MCU in the ROM bootloader on every power cycle. **We are unaffected** (no AS5600; PB8 is a driven SPI `SCK` with no pull-up), but it is the fourth distinct PB8 trap in this project and the vendor is actively recommending it |
| **3** | **Complete 20-row pin map added**, with every row's confidence marked. `CAN_TX` = PB9 and `CAN_RX` = PA11 re-confirmed; `H1/A` = PB6, `H2/B` = PB7, `H3/Z` = PB8, `GPIO_BEMF` = PB5, `BUTTON` = PC10, `PWM` = PA15, `STATUS` = PC6. The fan-out direction is fixed by a silicon fact — PB5 has no FDCAN function, so `CAN_TX` cannot be there — and three bench anchors (PB3/PB4 UART, PA14 SWCLK) validate the tracing |
| **2** | Parts confirmed visually: **EG2124A** gate driver, six MOSFETs, `STM32G431CBU6` (UFQFPN48, date code CHN 2145), three `R003` shunts, 120 Ω R22 with TP4/TP12 |
| **NEW 24.7–24.10** | ⚠ **Board revision conflict: §1 says "Matches FOC V2.0"; the listing photographs "火柴 FOC V1.0".** The schematic is now the sole source for `CAN_SHD`, `Temp_ADC`, the divider factor and the HSE routing, so **read the silkscreen before trusting it** — and request the V2.0 sheet if that is what is on the bench |

**Superseded and deleted:** *"Shunts: 20 mΩ (`R020`), amp gain scaled to compensate"* · the
claim that the clone's shunt value differs from the genuine board's · *"PB14 = board
thermistor (**probable**)"* · *"PB12 = unpopulated speed-pot input"* as a guess · the
transceiver as *"unmarked, possibly TCAN330"* with unknown rail and unknown mode polarity ·
settling the HSE question *"by looking at the board near MCU pins 5 and 6"* · the vendor's
`RSHUNT 0.010` and `AMPLIFICATION_GAIN 5.18` as descriptions of this board.

---

## 0a. Changelog — 2026-08-13 doc set split; belt-on diagnosed to the idlers; 1/rev closed; CAN ladder opened

**Structural: the single 1,568-line README became a doc set.** Section numbers are
unchanged, so every existing `§8.2`-style reference still resolves; the routing table is
in `README.md`. The split was done by **moving verbatim blocks with a script and
verifying reassembly**, not by regenerating — 1,558 body lines checked identical.

### Belt-on characterisation — J01, 2026-08-12 (NEW §22.1)

| § | Change |
|---|---|
| **NEW 22.1** | **Three plant states measured in a one-variable-at-a-time chain.** `drag_c`: motor alone **0.0783 A** → + belt, no idlers **0.2752 A** → + sliding brass posts **0.6605 A**. Per leg: 0.336 / 1.182 / 2.838 N = **3.4% / 12.1% / 28.9%** of a standing load |
| **NEW 22.1** | **The idlers are 0.385 A — 58% of belt-on drag.** The mesh, output bearing and belt bending together are **0.197 A**, now measured rather than assumed. My capstan estimate was 0.87–2.19 A: **mechanism right, magnitude 2–6× high** |
| **NEW 22.1** | **Friction-biased alignment CONFIRMED BY REMOVAL.** ZEA +2.88° elec with the faulty idlers, **+0.15° without**, against stored 6.0542 rad. ⚠ **This resolves the provenance dispute boxed in the previous §0 entry** — the belt-on session it said "does not exist in this repository" is this one, and the 2.88° figure is now measured. The `acHaveCommutation()` change is retrospectively justified by data as well as by argument |
| **NEW 22.1** | **`T/T_loop` IS DRAG-CONTAMINATED: phase 6 is BELT-OFF ONLY.** 0.958 (0.078 A) → 1.042 (0.275 A) → 1.291 (0.661 A), roughly **+0.57 per amp, linearly**, at **5.6σ** above the fleet band with essentially zero ZEA error — so **not** an alignment artefact. The parity arithmetic is fine (odd part scales with speed, 1.193 vs 1.184); the *interpretation* as pure transport delay fails. Hypothesis, unverified: phase 6 divides by `Ke·ω` and high drag moves voltage into `R·Iq` |
| **NEW 22.1** | **`Ke` also tracks drag** — 0.017750 → 0.017952 → 0.018089. The `ω_e·L·Id` cross-coupling term (21.2 / 32.9 mV) explains part of it and leaves **+0.89% unexplained**, and the correction makes the residual rms *worse*. **The belt-off value is the one to keep** |
| **NEW 22.1** | **Everything electrical held across the belt change**: `R_eff` −0.47%, `L` +2.1%, parity 0/4000, `fitpts` 18/48. Offline `L` refit reproduces the firmware to **0.02% on τ**. That plant-independence is what made the whole decomposition interpretable |
| **7** | **The prediction that belt tension would grow the 1/rev INL term is NOT SUPPORTED.** 1/rev fell 3.3%, 2/rev 2.9%, and the even-part correlation between the two runs is **r = 0.9986 across 32 bins**. M7 demoted from mandatory to confirmation — **not retired**, because this run was at 2 mm slack with no idlers |
| **8.2** | **The forward/reverse asymmetry prediction failed informatively.** Predicted ~10× larger belt-on; measured Coulomb 8.4% → **0.3%** (no idlers) → 32.9% (posts), viscous 24.8% → **9.8%** → 167.5%. **The idler-free belt-on state is the most symmetric drag measurement in this project's history.** Removing the idlers removed the direction dependence entirely — the thread-walking mechanism confirmed by removal. Leaves a smaller, stranger question: why is belt-off *less* symmetric than belt-on-no-idlers? |
| **NEW 22.2** | **The idler rebuild specification.** Five named defects; **rolling vs not rolling is 25–100×, diameter is 2× on a term worth 2%.** 5 mm OD is 0.65× the 12T pitch diameter, ~half the practical back-side floor; 9 mm buys **3–6× belt life**. **The screw axis must move outward 2.0 mm — this is a plate revision, not a parts swap**, because the take-up budget is 2.00 mm and a fixed screw would demand +2.65 mm of stretch = 110–570 N against a ~150 N rating |
| **NEW 22.3** | **The pluck tension method is RETIRED** — the idlers subdivide every span, and span ambiguity alone spans a 5× tension range. **B6a/B6b replace it as fleet tension QC on every joint**, which withdraws the earlier plan to give J02 a reduced run |
| **22** | **B9 merged into B6a and moved up.** Its old position, five steps after the work it duplicated, was an invitation to skip the half of the tension trilemma that is hardest to diagnose. `f_n` prediction corrected **40–55 Hz → 100–230 Hz** (an earlier `k_belt` estimate was ~5× low) |
| **22** | ⚠ **Neither belt-on row goes into `joint_cal.h`.** Both describe plants that will not exist on the robot. Archive with plant-state headers |

### A3 — the 1/rev disturbance closed, 2026-08-13 (NEW §9a)

| § | Change |
|---|---|
| **NEW 9a** | **The 1/rev disturbance is not motor-internal, and the motor swap is CANCELLED.** J02, belt off, pinion off: 1/rev **0.0262 A max vs A1's 0.185 A**; `corr(Iq, vel)` **−0.78 → −0.042**; folded velocity ripple **8% → 0.3%**. Referred to the foot: **0.042–0.113 N = 0.4–1.1% of standing load** |
| 1, 8.2, 8.3 | The 1/rev item **retired from three places at once**, and A1's 0.185 A reclassified as rubbing-magnet contamination — joining `drag_c` = 1.05 A and the 4.5 N transparency figure. **Third time that one fault has explained a symptom it was not invented for** |
| **9a** | **7/rev now dominates** (0.031–0.039 A) — current-sense channel *offset*, a measurement artefact, not mechanical |
| **9a** | **Free fleet cross-check:** J02 reached **70.34 rad/s at Uq = 1.30 V against J01's 71.02 rad/s — 1% agreement.** Fleet consistency measured rather than assumed, at zero cost |
| **9a** | **Method note:** the plan had been to fold the *archived* J01 CSV. It holds **INL angle bins, not a current fold** — the quantity was never in it. **Check what an archive contains before planning a session around it** |
| **9a** | **A3b remains open and expires at B0**: one pinion-seated capture to separate bore runout. 10 min |
| **8.2** | Belt-off drag baseline re-fitted cleanly: fwd `0.0749 + 9.333e-4·ω`, rev `0.0816 + 7.272e-4·ω`, **fit rms 0.70 / 0.74 mA** — the 28% viscous asymmetry is real, not scatter. Independent check: odd-part mean **3.214°** elec vs `7·ω·T` predicting **3.21°** |

### Firmware / control corrections

| Change | Why |
|---|---|
| **§10 — the current-loop gains DO NOT satisfy the design rule, and they are still correct** | Configured `P/I` = **298.5 µs**; with measured `L` = 43.31 µH the rule wants **195.9 µs**. The 1.52× mismatch is the fingerprint of `65 µH / 0.218 Ω` = 298 µs — **the gains were tuned against a value now retired as a fit artefact.** The loop is *proportional*-dominated, not the integral-dominated failure fixed in July, and it measures 412 Hz / 9.3% / ζ 0.60 at ~80% of the transport-delay ceiling. **⚠ AUTOCALIB prints `suggested P=0.1089 I=555.6`. Do not paste it** — raising `I` by 1.66× walks back toward 83% overshoot. Boxed as do-not-fix so a future session cannot "correct" working gains |
| `fleet_config.h` | **`T_DELAY_PER_LOOP = 0.958` must be labelled belt-off**, and phase 6 should warn when `CAL.belt != OFF` — the same defect class as the phase-7 drag-map label fixed in §0b2 F7 |
| **A5 downgraded** | The `MULTIMETER: ___` field does not track pack SoC (the seed is re-measured every boot) and does not resolve the DT9205A-vs-RC3563 0.98% disagreement. **Its only value is retroactive rescaling** if `vbus_scale` is ever corrected — insurance on 90 s of AUTOCALIB, not a live measurement. Cost 30 s. Not raising it again |

### CAN transport — NEW §23

| § | Change |
|---|---|
| **1, 3, NEW 23.2** | ⚠ **ESC1 termination is a SINGLE HARD-WIRED ~121 Ω resistor, not firmware-controlled.** Measured unpowered, where a MOSFET-switched terminator reads open. **This contradicts project memory and bench evidence wins.** Bench (2 nodes) is 60.7 Ω — correct **by accident**. Robot (12 ESC1 + master) is **9.2 Ω against a 45 Ω floor: bus dead.** 🔴 **10 of 12 boards need the resistor physically removed**, found for the price of one measurement instead of at multi-node integration where it presents as "works with two nodes, not four" |
| **3, NEW 23.3** | `CAN_TX` = **PB9**, `CAN_RX` = **PA11** (AF9). ⚠ **RETRACTS the BOOT0 / `CAN_RX` collision hypothesis** — nothing in the CAN path touches PB8 |
| **NEW 23.1** | ⚠ **RETRACTED: "endless retransmission is the correct Stage-0 result."** A lone node in NORMAL mode goes **bus-off after ~32 unacknowledged frames** and falls silent, which reads exactly like a dead transceiver. **Stage 0 must run NO_ACK / self-test mode** — which requires the **raw ESP-IDF TWAI driver**, not the `ESP32-TWAI-CAN` wrapper. And **`TEC = 0` is not a pass indicator in NO_ACK mode**: ground truth is the analyzer decode plus RXD mirroring TXD |
| **NEW 23** | **Five-rung ladder, one new unknown per rung:** S0 (ESP32 + known-good transceiver + analyzer) · S1 (second ESP32, real ACK) · **S1b** (ESC1, GPIO only, no FDCAN — transceiver identity, rail, mode polarity) · **S1c** (FDCAN external loopback — clock, bit timing, AF9 mux, no partner) · S2 (the cable). S1b replaces six continuity checks that were specified wrongly: **the IC pads are inaccessible**, and everything needed is obtainable functionally |
| **NEW 23.4** | **S0 pre-flight PASSED:** 3V3 rail in band, TXD and RXD at ~3.3 V, CANH and CANL both 1.6–1.7 V, **CANH − CANL exactly 0 V**, termination 120 Ω single-node. That last reading proves the transceiver exists, is powered, and its driver is not jammed. TWAI confirmed transmitting at **200 frames/s** |
| **NEW 23.4** | **Instrument bug worth its own line: PulseView was left at 20 kHz instead of 24 MHz.** Against a 1 µs bit time that is 0.02 samples per bit — structurally incapable of showing a frame, and it looks like a dead bus. **Read the sample rate before concluding anything about the hardware.** ESP32 banner also printed before USB-CDC enumerated; fixed with `while (!Serial)` **plus a `b` key to reprint**, since the wait alone would hang a headless boot |
| **NEW 23.5** | ⚠ **THE FDCAN CLOCK IS COUPLED TO THE FOC LOOP RATE, AND NOBODY COSTED THAT WHEN 1 Mbit WAS FROZEN.** `CAN bit rate → FDCAN kernel clock → SYSCLK → PWM frequency → loop rate → transport delay`. The 170 MHz SYSCLK comes from a 340 MHz VCO, and **340 has no divisor landing on a clean 1 MHz sub-multiple at or below 80 MHz** — so if an 80 MHz cap is real *and* no HSE crystal is fitted, exact 1.000 Mbit needs **SYSCLK at 160 MHz**, moving PWM and loop rate by 6%. `T_DELAY_PER_LOOP` survives *because it is a ratio*; every absolute microsecond figure derived from it moves |
| **NEW 23.5** | **Two facts gate all STM32 CAN code and neither may be guessed:** is an HSE crystal fitted near MCU pins 5/6, and what is the maximum `fdcan_ker_ck` on the G431 (**believed 80 MHz at ~70% confidence — read RM0440**). Fallback is 500 kbit/s where 42.5 MHz factorises cleanly, at 62% bus load against 31% — **that would re-open a frozen decision and must be logged as such** |
| **NEW 23.5** | **Standing rule: never change the APB1 prescaler to fix the FDCAN clock.** It moves TIM2/3/4 and this project has a bit-banged encoder whose timing is already characterised |
| **NEW 23.6** | The **CAN message spec** and the **RL observation/action vector** are both unwritten Tier-0 boundary objects and should be frozen together — but not before the bit-rate decision |

### Housekeeping

| Change | |
|---|---|
| **NEW 24 — documentation integrity register** | Six live conflicts surfaced by the audit, each with the decision that closes it: **KV360 vs KV380** on the nameplate; **3.0 vs 4.0 kg** target mass (which moves every "% of standing load" by 33%); **5S frozen vs 6S live**; the **0.017952 vs 0.018035** `Ke` fit disagreement; plus `i_scale` and `DRIVETRAIN_ETA` promoted to front-page visibility |
| **Step-name prefixes disambiguated** | Three ladders were all using `S`-numbers. Now: **M** = manual calibration, **B** = belt build, **S** = CAN only, **A** = loose audit tasks. The July S0–S8 ladder is retired. Mapping table for the belt-on session's S1–S7 in `README.md` |
| Corrections applied where values had gone stale | `J_rotor` **±2.0 → ±2.4 × 10⁻⁶ (±10% → ±12%)** in §15/§8.3/§17 · `T/T_loop` **0.961 ± 0.014 n=5 → 0.958 ± 0.015 n=7** in §15 · §1 encoder row was still describing ABZ/4096 as current · §1 "moving to 10 mm" · §5's superseded `T ≈ 143 µs` · two duplicated headings in §15 |
| Not fixed, deliberately | §12 carries the *"a fit can measure one parameter superbly and another not at all"* learning **twice**, in two lengths. Left in place: the versions cite different datasets and merging them would lose the 2026-08-01 sweep's specifics. Flagged in §24 |

**Superseded and deleted:** *"endless retransmission is the correct Stage-0 result"* · *"TCAN330 onboard with firmware-controlled termination"* · the BOOT0/`CAN_RX` collision hypothesis · the 1/rev disturbance as **0.185 A / 0.79 N** on live hardware · *"belt side-load moves the magnet off-axis; the 1/rev term will grow"* as an established expectation · the pluck tension method · the plan to give J02 a reduced belt-on run · the claim that no belt-on session exists to support the 2.88° ZEA shift · `f_n` = 40–55 Hz · the post-rebuild drag prediction of 0.15–0.35 A (revised to 0.30–0.40 A: removing the idlers also removed the tension).

### Retired roadmap items, moved here from §15

**Superseded — was:**
1. ~~C1 — electrical re-characterisation of the SPARE~~ (30 min, belt off). Locked-rotor `Uq`-vs-`Iq` sweep at 8–10 currents 0.1→2.0 A for `R_eff` and `U0`; free-spin `Ke` at 4–5 voltages; `Kt` cross-check against `60/(2π·KV)`; current-step `L`. **The two-point values (0.1977 Ω / 0.0493 V) cannot separate R from U0** — same ill-conditioning as before.
2. **C2a — belt-off drag baseline** (10 min). Motor-only friction. Subtracting it from the belt-on map later **isolates belt drag**, which no single measurement can do.
3. **Build the 10 mm belt / 9:1 actuator on the SPARE.** Belt and pinion in hand; no longer blocked.
4. **C2b — belt-on drag map**, then **J_rotor** and **J_total**, then **force-per-amp** once the 80/100 leg exists.
5. **M1 proper — impedance controller**, with the friction feedforward (and U₀ feedforward if the bus exceeds ~15 V) designed in from the start rather than bolted on.
6. **6S staged commissioning** (30 min, separate session). §19.
7. Re-measure INL after the belt is fitted — belt tension changes DISP.

---

## 0a1. Changelog — 2026-08-12 the Jacobian behind G = 87.7, and phases 3-6 stop forcing a re-alignment

| § | Change |
|---|---|
| **`fleet_config.h`** | **`G = 87.7` resolved: it is the STROKE MEAN, not a value at any height.** The α-average of \|dh/dα\| over α = 40–70° is 102.641 mm → G = **87.68**, matching the stored 87.70 to **0.02%**. The h-weighted average gives 87.39 and does *not* match, which is what identifies it as specifically an α-average |
| **8.2 / 15** | **The "needs hip pivot separation from CAD" blocker was never real.** The five-bar is **coaxial-hip**, so the separation is zero by construction. `project_context.md:159` confirms it independently and was written first: it records `h → L1·cosα + L2` as `L2 → ∞` (exactly this formula's limit) and a stroke collapsing to `0.424·L1` = `L1(cos40 − cos70)`, which also pins the 40–70° range |
| **`fleet_config.h`** | **The Jacobian is NON-MONOTONIC** — it peaks at α = **63.44°** and turns back, so force per amp has a **minimum mid-stroke and rises at both ends**. Not the monotonic collapse-toward-extension a jump controller would assume. Full α/h/G/F-per-amp table now in the header |
| **`fleet_config.h`** | **G swings 22.7%** (83.18–102.08), so every force figure quoted at `Ḡ` does too: peak thrust 513 → **487–597 N**, force per amp 4.30 → **4.07–5.00 N/A**, added mass 1.243 → **1.118–1.684 kg**. Two new constants, `G_FOOT_AT_STROKE_TOP` and `G_FOOT_AT_STROKE_MIN`, plus the leg geometry |
| **17** | **This sharpens the 478 mm apex question through the MASS channel, not the force one.** Reflected inertia goes as `G²`, so added mass is **worst at the top of the stroke** — exactly where a jump leaves the ground. It does not settle the 4.0-vs-5.24 kg question |
| **15** | **Stroke check passes independently:** the closed form gives h = 93.31–147.05 mm = **53.74 mm** against the recorded 53.7 mm envelope |
| **15** | **Force-per-amp endpoints validated to 0.4%** (model 4.145 / 5.000 vs recorded 4.13 / 4.98). **The middle value 4.32 "at α = 55°" does not reproduce** — the model gives 4.186, and 4.32 needs α = 50.9°. Most likely the **stroke mean (4.296) that acquired an angle label** |

**Firmware — phases 3–6 no longer force a fresh alignment**

| Change | Why |
|---|---|
| **New `acHaveCommutation()`** | Separates the two gate meanings that `acNeed()` was conflating: **DATA** dependencies (phase 4 needs phase 3's `R`) from **COMMUTATION** dependencies (a valid ZEA, from *either* a stored row via `f`/`V` **or** a fresh phase 2). `acM4Breakaway()` already reasoned this way inline; this generalises it |
| **Phase 3: `acNeed(2)` → `acHaveCommutation()`** | Phase 3 is `velocity_openloop` at `target = 0` — the field angle is commanded directly and **the ZEA is not used anywhere in it**. Verified in the source, not assumed. Phases 4–6 inherit their gates from here, so the whole downstream chain was paying for an alignment none of it needed |
| **Phase 5: explicit `acHaveCommutation()` added** | Phase 5 *does* commutate (`MODE_TORQUE` through the FOC path), and it used to inherit its ZEA requirement transitively via phase 3. Now that phase 3 no longer implies one, the requirement is stated where it is actually true |
| **M2 assist: same change** | Its stated reason was comparability with the phase-3 sweep. Phase 3 now accepts a stored ZEA, so leaving M2 on `acNeed(2)` would have made it the last thing on the bench still forcing a re-alignment |

**Why this matters physically.** Alignment settles where the alignment torque balances **friction**, so the ZEA it returns is biased by the plant's friction state. The lowest-friction state is **belt-off**, so the belt-off ZEA is the one worth keeping — and making a re-alignment the price of reaching phase 5 silently replaces a good ZEA with a worse one.

> **⚠ RESOLVED 2026-08-12 — the session now exists, and the numbers check out. See §22.1.** The measured ZEA shift with the faulty idlers fitted is **+2.88° elec** against stored 6.0542 rad, and **+0.15° with them removed**, which is the friction-biased-alignment mechanism confirmed by removal. The box below is kept verbatim because the *reasoning* in it was correct at the time — a change should not rest on evidence that cannot be located — and because the resolution is a better illustration of the rule than the objection was. **The parity separation cross-check named at the end has still not been run.**
>
> **⚠ The change is right; the evidence offered for it was not.** *(As written 2026-08-12, before the belt-on run.)* The report justifying this cited a J01 **belt-on** session — "aligned 2.88° elec above belt-off, parity separation independently said 2.93°, re-aligning inflated `Id` 0.60 → 1.08 A and pushed fitted `Ke` 1.9% off". **No such session exists in this repository.** Every `JointCal` row reads `belt = "OFF"`, and **B0 — "fit the belt" — is still open task #4**. The only `1.08` in the README is §20's *predicted* M2 `I_reported` at point 4, which is unrelated. The numbers were **not** written into the code comments. The change stands on what is verifiable instead: phase 3's open-loop path provably ignores the ZEA, and friction-biased alignment is ordinary SimpleFOC behaviour. **When the belt does go on, that 2.88°/2.93° agreement is worth actually measuring** — it would be a good test.

---

## 0a2. Changelog — 2026-08-08 J02 characterised: a two-joint fleet, and a wrong-joint carry blocked

| § | Change |
|---|---|
| **`joint_cal.h`** | **J02 row added** — the rebuilt A1 hardware (rubbing magnet remounted, MT6816 SPI fitted). All six phases PASS. `board_sn` / `motor_sn` still `___`: **label the physical parts first** |
| **`joint_cal.h`** | **A1 marked HISTORICAL — the assembly no longer exists.** Kept at index 12 so no `JOINT_ID` shifts |
| **8.2** | **First real fleet transparency number: two independently built assemblies both at ~13% belt-off.** J01 12.8%, J02 13.1%, means differing by **0.19σ**. 3.6× better than A1's contaminated 46% |
| **8.2** | **But J02's scatter is 1.8× larger** (cv 36.8% vs 20.9%). *"Feels rougher by hand"* was the correct read: **worn grease raises the variance, not the level** |
| **8.2** | **The ±20–37% spread is created by HANDLING THE SHAFT.** Three untouched consecutive readings: sd 0.0029 A, **cv 1.2% — thirty times tighter**. So repeats at one position are **pseudo-replication**; a "7.2σ direction asymmetry" computed from such a block was counting pseudo-replicates. Done over six independent positions it is **0.43σ, not significant** |
| **8.2** | `J_rotor`: J02 gives 18.7e-6, **7.9% below J01 — and that gap is entirely the drag correction.** Rerunning J02's impulse integral with J01's drag map gives 20.10e-6, J01's own value to 0.5%. Fleet value stands; tolerance **±10% → ±12%**. **Limiting error is the drag map (25–32% of the impulse), not the fit** |
| **8.2** | **Free coast-down cross-check RETIRED.** Three attempts on two joints all started *after* the disable — the `L` keypress lands hundreds of ms behind `x`. Not viable with keyboard timing |
| **8.2** | **A fourth, independent probe of `U0`**: the M6a endpoint speeds only reconcile with the measured drag map if `U0` ≈ 0.003–0.008 V, not 0.0147. **Treat `U0` as 0.010 ± 0.005 V** |
| **8.1** | `T/T_loop` → **0.958 ± 0.015 over n=7, three assemblies.** The mean moved 0.3% and **the sd did not grow** when a third build was added |
| 15 | **`L` = 65 µH closed as a fit artefact.** Same motor and board now reads 45.39 µH on the dithered fit — 4.8% from J01. `L` is near-fleet at ~44 µH |
| 15 | **`R_eff` +1.26% while `Ke` +0.06%** — both scale with the voltage belief, so a scale error would move them together. It is a **real** board difference. First time the fleet table could separate those |

**Firmware**

| Change | Why |
|---|---|
| **`ac_zea_mismatch` blocks the phase-7 carry** | Phase 7 carries `vbus_scale`, `i_scale`, `breakaway_A`, `id` and the serials from the flashed row. That is right for a re-run and **exactly wrong when the row is another joint's** — J02's first report carried **J01's `breakaway_A` = 0.2920 with no warning**, thirty lines after the `V` check had failed. The report had every piece of information needed and said nothing. Now: a banner in the header, and each carried field emitted as `___` / `0.0f` with `*** CARRY BLOCKED ***` |
| **Two detectors, not one** | Set by `acVerifyZea()` **and** by `acP2()` — a fresh alignment *is* the same comparison, so it costs nothing and closes the hole where the operator never presses `V`. Shared `acZeaDeltaDeg()` so the two cannot disagree about what "different joint" means. WARN (8–15°) deliberately does **not** block: that is "check the mount", not "wrong joint" |
| **M4 settle instruction** | A rotor left mid-creep is still elastically loaded and breaks away far too easily in the *opposite* direction. J02 produced **0.0350 A that way — a quarter of the truth** — after 583 counts of unexplained forward motion. Reading dropped |
| `fleet_config.h` | `T_DELAY_PER_LOOP` 0.961 → **0.958**, sd 0.015, n=7 · `J_ROTOR_KGM2` tolerance **±12%** with J02's determinations and the "drag map is the limiting error" note |
| `joint_cal.h` | J01's stored force figures corrected to **include η** (1.257 N / 12.8%, was 1.37 N / 13.9%). The old numbers omitted η while A1's 4.5 N included it, so the comparison was against inconsistent units |

**One correction to the report that prompted this pass, and one to my own previous work.**
- The **direction-asymmetry figure `C` = −0.0144 ± 0.0332 (0.43σ) keeps the pair whose reverse reading was dropped from `S`.** Dropping it consistently gives **−0.0363 ± 0.0306 = 1.19σ** — still not significant, so the conclusion holds, but 0.43σ is the optimistic version and the row comment now says so.
- **I reintroduced the same raw-string escaping bug I had just fixed**, this time as `\\"` inside the phase-7 row emitter. It failed the build rather than shipping silently, unlike the `\n` case. Both came from using Python raw strings for replacement text containing C escapes.

---

## 0b. Changelog — 2026-08-08 code-review round 3: one self-inflicted regression, two dead guards, one circular constant

| # | Fix |
|---|---|
| **1** | **`autocalib.h:457` printed a literal `\n`.** A raw-string editing artefact from the previous pass turned `F("\n  -- key received")` into `F("\\n …")`, which in C is a backslash followed by `n`. Every keypress printed `\n  -- key received` on the console. Only occurrence in the tree; the two neighbouring `ABORT` prints were correct |
| **2a** | **M4's `el < 1000` guard was dead code.** The ramp is deterministic — `el = (i/AC_M4_STEP_A)·AC_M4_DWELL_MS = i × 30000 ms/A` — so `el < 1000 ms` is exactly `i < 0.0333 A`. At J01's 0.2923 A the ramp takes **8.8 s**; the test could never fire. Replaced with a **`travel` > 200 cnt** check, which is the quantity that actually carries pre-slide-creep information |
| **2b** | **M4's `> 0.40 A` "FAULT" warning was crying wolf.** Against J01's mean 0.2923 / sd 0.0611 it sits at **+1.76σ** and fires on ~4% of *healthy* readings — it **did** fire, on a 0.4050 A reading, and that false alarm cost a teardown detour. Raised to **0.60 A (+5.04σ)**. ⚠ **This forced `AC_M4_ABORT_A` 0.60 → 0.80**: the ramp loop exits at `i >= AC_M4_ABORT_A`, so a warning set *at* the abort value is unreachable. Thermally free (0.21 W) |
| **3** | **`DRIVETRAIN_ETA` = 0.92 is BACK-SOLVED and CIRCULAR**, and the repo settles it: the 4.5 N row's own "How measured" column reads `F = 2·G·η·Kt·I` — it was **computed** with a formula already containing η. Kept (the exact round trip makes it consistent with every existing force figure) but labelled, given a **do-not-double-count convention** against `drag_c`/`drag_v`, and **announced in the boot banner beside `i_scale`** so a built joint says out loud that two unmeasured multiplicative force factors are outstanding. M14 replaces it with a lumped `G·η·Kt` |
| **4** | **ALL A1 CONSTANTS WERE MEASURED WITH A RUBBING ENCODER MAGNET.** The lost-count fault is diagnosed as magnet contact. `drag_c` = 1.05 A is belt friction **plus a rub**, so **"4.5 N/leg = 46% of standing load"** and the "Coulomb friction dominates transparency" learning are both **suspect**, and **1.05 A must not be used as the M5 belt-drag prior**. The belt is probably *better* than assumed — 0.98 A of implied belt drag was always uncomfortably large for a 10 mm GT2 at 9:1 |
| **6** | **478 mm apex given a trigger:** *before the jump/gait controller's energy budget is written, or before it is quoted outside this repo.* Not "before the first jump" — the controller consumes the number earlier than the hardware does |
| 15 | Task board rewritten; the **45-minute** extraction (`mt6816.h` / `actuator_hw.h` / `safety.h`) scoped and scheduled **after J02, before Tier-0**, with the reasons the 2-hour refactor is declined |

**One correction to the review that prompted this pass.** It called `DRIVETRAIN_ETA` *"doubly suspect — derived from a contaminated number by a possibly circular route."* The two halves cannot both apply: η was back-solved as `4.5/(2·G·Kt·1.05)` while 4.5 was computed as `2·G·η·Kt·1.05`, so **the 1.05 A cancels exactly and the contamination cannot reach η.** It is not doubly suspect — it is *purely* circular, which is worse, because a contaminated number at least contains information.

Also: the proposed `if (i > 0.60f)` warning would never have fired, because `AC_M4_ABORT_A` was itself 0.60. The threshold and the abort limit are coupled, and that coupling is now documented at both constants.

---

## 0b1. Changelog — 2026-08-08 belt-off characterisation CLOSED: J_rotor, breakaway, cogging reopened

| § | Change |
|---|---|
| **8.2, `fleet_config.h`** | **`J_rotor` = 20.2 ± 2.0 × 10⁻⁶ kg·m² — CLOSED, three methods.** M6a driven step (20.2) plus two free coast-downs (16.31 / 24.29, mean 20.30). A driven and a free-decay measurement landing on the same number is why this is ±10% |
| **17** | **The prior estimate of 2.1 × 10⁻⁵ was only 4.0% high**, so every reflected-inertia figure in §17 stands, corrected down by 4%: **0.155 kg/motor, 0.311 kg/leg, 1.243 kg robot, 5.24 kg effective.** Rotor KE at takeoff 4.20 J against ~13.5 J body |
| **17** | ⚠️ **OPEN, and it is a documentation question, not a measurement one:** does the 478 mm apex use 4.0 kg or 5.24 kg? §17 says reflected inertia is 24% of effective mass *and* quotes 478 mm without stating a mass. If 4.0 kg, apex → **352 mm** on §17's own `h = F·s/(m·g) − s` model (365 mm on a naive `h ∝ 1/m`). The derivation is not in this repo — read the spreadsheet |
| **8.2, `joint_cal.h`** | **`breakaway_A` = 0.2923 ± 0.0184 A** (SE 6.3%, **sd 20.9%**) = 7.52 mN·m → **0.68 N/motor, 1.37 N/leg, 13.9% of standing load. 3.3× better than A1's 46% belt-on** |
| **8.2** | **The ±20% scatter is the PLANT, not the method.** M4's static sd/mean (20.9%) and the two coast-downs' implied friction (1.24× and 0.83× the drag map) agree — two different physics, same afternoon. **Grease redistribution**: a rub or a bent shaft would be *reproducible* |
| **8.2** | **COGGING REOPENED. 1.4 mN·m, 13× the stored 0.004 A figure.** The free-spin position-fold was **structurally blind, not noisy**: at 84/rev and 100 rad/s cogging sits at 1337 Hz, where rotor inertia limits the speed ripple to 8 × 10⁻³ rad/s, the 412 Hz current loop cannot track it, and decim-8 logging aliases it outright. M4's stationary fwd/rev pair has none of that. **Conclusion unchanged — still do not build a cogging table** (2.5% of standing load) |
| **`fleet_config.h`** | **`G_FOOT_PER_MOTOR_NM` = 87.7 /m, with the ×2 stated once, in one place.** The "161 /m" a naive back-calculation produces was never a second value of G — it is 87.7 with the per-leg factor of two folded in |
| **`fleet_config.h`** | Added `FRICTION_TRIAL_SPREAD`, `DRIVETRAIN_ETA`, and `footForcePerMotor()` / `footForcePerLeg()` so the factor of two is spent once rather than rediscovered per conversion |
| 15 | **Belt-off characterisation declared DONE**, with a hard stop rule and the three remaining risks — none of which is a precision question |
| 20.2 | M6a closed |

**Two corrections to the review that prompted this pass**, both verified against this file:
- **The 9% residual in the A1 cross-check is drivetrain efficiency, not leg height.** §8.2's formula is already `F = 2·G·η·Kt·I` — the ×2 was never missing from the README. 4.5 / 4.904 = **η ≈ 0.92**, an ordinary belt-drive number. Attributing it to leg height would wrongly imply G is uncertain.
- **Reflected inertia was not missing from the design.** §17 already documented 1.29 kg / 24% of effective mass, from an estimated `J_rotor`. This measurement **confirms that estimate to 4%** — it is not "a 24% error in the headline jump number". The only genuinely open item is the narrower one now boxed in §17: which mass the apex arithmetic used.

---

## 0b2. Changelog — 2026-08-08 review round 2: one safety-path bug and six clarity defects

| # | Fix |
|---|---|
| **F1** | **`ac_key` could clear a genuine guard trip.** The three abort conditions in `acService()` are independent `if`s in the same call, so a keypress arriving in the *same iteration* as an overcurrent set both `ac_key` and `ac_abort` — and the M2 ladder's advance cleared `ac_abort` on `ac_key` alone, resuming after a real fault. **Low probability, safety path.** Added `ac_guard`, set only by the overcurrent/overspeed branches and never cleared |
| **F2** | **A normal M2 advance printed `!! ABORT key pressed`** — five times in a five-point run, while working correctly. That is the message that makes someone stop and redo a good measurement. Now a neutral `-- key received`; each caller reports what it actually did |
| **F3** | **The F4 comment still asserted what F5 retracted**, nine lines apart, with the wrong one unmarked: *"phase relative to the loop is effectively random… 20 µs bins give ~17"*. It is not random, and binning alone gave **7**. Marked **PARTLY SUPERSEDED**, keeping only the true part (time binning is the mechanism; the **dither** supplies the coverage) |
| **F4** | **`printJointCal()`'s label named the wrong quantity.** `calKtCmd() = Kt/i_scale`, so the command is *multiplied* by `i_scale` — at 1.05 the command goes **+5%**, but it printed **−4.76%** under the label "torque cmds corrected by". Now prints **both**, each correctly named |
| **F5** | **The `i_scale` field comment told you to do the opposite of the `calKtCmd()` note.** Field comment: *"invalidates phases 3-4"*. `calKtCmd()` note: *"must not divide `R_eff`, `L` or the gains by it"*. Both true statements, opposite instructions, and the field comment is read first. Rewritten to separate the two cases: **storing** a measured `i_scale` invalidates nothing; correcting the gain **at source** moves the reported-amp unit and does require re-running 3–4 |
| **F6** | **M4's output printed `c0` twice** and put `" A at raw="` *after* the count, so it read as though the position were in amps; `ramp` and `travel` sat outside the delimited prefix. Now `M4,<dir>,<amps>,<raw>,<ramp_s>,<travel_cnt>` alone on its line, sentence separately. **Same fix applied to M2's line**, which had the same defect |
| **F7** | **"belt OFF" was hardcoded into things that do not need it** — and that starts mattering now that M2 is deferred past the belt build. Scoped: free shaft is needed by **2, 5, 6 only**; **1, 3, 4 and the M2 ladder are locked-rotor and belt-agnostic**; **M4 is run in both states deliberately**, so its banner now reports `CAL.belt` instead of demanding one. Phase 7's `DRAG MAP (belt OFF)` label now prints the actual belt field — an untagged belt-off label on belt-on data is how a baseline gets overwritten by its successor |
| — | `acExit()`'s *"keep_align = true only for phase 2"* comment was already stale (3, 4, 5 all pass true). Rewritten to say what the flag actually means |
| — | M2 now settles for `AC_R_SETTLE_MS` **before** latching the drift baseline, so the `drift` column means creep during the window rather than the settling transient of the voltage change |

**Task order revised** (§15): ordered by **what expires**, not by what is interesting. M4 and `J_rotor` are belt-off baselines and cannot be recovered after B0; **M2 is locked-rotor and belt-agnostic, so it moves behind the belt** — deferred, with a promotion condition, and with §8.2 now carrying an explicit note that every force figure in this document rides on an unmeasured `i_scale`.

---

## 0b3. Changelog — 2026-08-08 review round 1: manual-test firmware + the phase-voltage ceiling

| § | Change |
|---|---|
| **NEW 8.3** | **`DRIVER_VOLT_LIMIT` = 6.0 caps phase voltage at 3.46 V — 28% of a 12.46 V bus.** Nothing measured so far was clipped (highest `Uq` reached: 2.00 V), so **every J01 constant stands** — but the rig tops out near **190 rad/s unloaded**. Deferred with a physical trigger: **before the first commanded velocity above 150 rad/s** |
| **8.3** | **Correction to my own note:** raising it invalidates **`U0` and the low-side sensing window only**. `R_eff` and `Ke` are **immune** — `Ua = Ta·driver_vl` while `Ta ∝ Uq/driver_vl`, so the factor cancels and only the differential voltage drives a floating star |
| **NEW `N`** | **`acM2Assist()`** — open-loop self-lock walked down a 5-point voltage ladder, holding indefinitely so an external meter can settle. One keypress per point |
| **NEW `B`/`b`** | **`acM4Breakaway()`** — firmware-timed breakaway ramp, forward/reverse. Reports current, rotor position and elapsed ramp time, and self-warns on both failure modes |
| **20.1** | **M2 method replaced again, and my earlier version retracted.** A two-point difference does **not** remove the losses proportional to `I` (switching ~0.15 W, dead-time ~0.05 W at 3.1 A) — ~5% of the differenced signal, which would have reported **`g` ≈ 0.95 on a perfectly calibrated board**. Now **5 points, quadratic fit, `g = 1.5·R̄/c`** |
| 20.2 | **M4 phrasing retired.** *"Ramp `Iq` through zero at ±0.5 rad/s"* implied something was already turning. Nothing turns; 0.5 rad/s is only the threshold that counts as motion |
| **20.2** | **M6a promoted ahead of the belt.** `J_rotor` belt-off was blocked on "needs a drag map" — the belt-off map already exists. Belt-off is also the *better* measurement (pure rotor inertia). Fleet constant → `fleet_config.h`. `J_total` belt-on split out as M6b |
| **21** | **`i_scale` given its single consumer, `calKtCmd()`.** It is applied **only** to the torque→current conversion. `R_eff`, `L` and the current-loop gains must **not** be divided by it — they were measured in reported-amp units and are already self-consistent |
| `[env:A1]` | **Now fails at BUILD time** via `-D ENCODER_ABZ` + `#error`. A comment was not enough for a binary that boots, prints a plausible banner and reads garbage |
| `AC_L_DITHER_US` | **2 → 3.** 2 × 32 reps = 64 µs against a 75 µs loop period left an 11 µs hole — the reason the `LSB` `n` column still peaked every third bin. 3 × 32 = 96 µs covers it. J01's fit is unaffected; this is for J02+ |
| **NEW 8.2** | **Forward/reverse drag asymmetry** documented: 28% viscous, real, unexplained, **0.04 N at the foot**. Discriminator scheduled at M5 (swap the direction order). The offered `Id` "cross-check" is **declined as circular** — see the box |
| **NEW 22** | **The belt-on routine, B0–B13**, with its two hidden failure modes |
| 15 | Task board rewritten |

**Verified against the pinned library, not asserted:** `BLDCMotor.cpp:409` in SimpleFOC **2.3.1** does contain `voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit)` in the `torque`/`voltage` path — so `TORQUE_MAX = 2.6` above `VOLT_LIMIT = 2.0` **is** unreachable, as previously stated. Also: `'F'` (force-align) and `'G'` (go) were already bound, so the manual-assist routines took `N`/`B`/`b`; and `platformio.ini` has **no** `default_envs` line to remove.

---

## 0c1. Changelog — 2026-08-07 session 3 (AUTOCALIB rev2 re-run) + `JointCal` schema v2

| § | Change |
|---|---|
| **8.1a** | **A2/J01 re-measured, all six phases PASS.** `R_eff` 0.22108 Ω, `Ke` 0.017750, **`L` 43.31 µH from an 18-point dithered fit** |
| **8.1a** | **`L` = 44.43 / 44.8 / 45.1 µH RETIRED.** Those were 5- and 7-point fits on a *phase-locked* sample grid, biased **+4% in τ**. The F5 dither closed it; the new τ = 195.9 µs was reproduced by an independent offline refit to 195.92 µs |
| **8.1a** | **`U0` does not reproduce, and the fitted SE says it does.** 0.00367 / 0.00559 / **0.01026** V across three sessions against a per-fit SE of ~0.0016 → the real uncertainty is **±0.003 V, ~2× the fit's own SE**. A "6.5σ PASS" on this parameter is overconfident |
| **8.3** | **`U0` feedforward promotion condition REWRITTEN**: it is *"a board measures `U0` > 0.03 V"*, **not** *"the bus exceeds 15 V"*. On B-SPI-01 the 5S projection is 0.078 A ≈ 3% of standing leg load, not 12.9%. `dead_zone = 0.005` stays closed for this board |
| **8.1** | **`T_delay` restated as the fleet ratio `T/T_loop = 0.961 ± 0.014`** (5 points, 2 assemblies, 2 loop rates). Loss at 270 rad/s is **0.95%**, not 3.6%. The angle-lag sweep is **closed, not pending** |
| **10** | **`Ke`'s 0.57% session spread is the BUS, not the motor.** Ke ratio 0.99434 vs Vbus ratio 0.99521 → 0.09% residual. Widens the live-Vbus promotion condition to include cross-session constant comparison |
| **21** | **`JointCal` schema v2.** `Kt` **removed** (derived by `calKt()`); `i_scale`, `breakaway_A` and a **4-field direction-split drag map** added |
| **NEW** | **`src/fleet_config.h`** — pole pairs, encoder resolution, `Kt/Ke`, dq magnitude, PWM frequency, `dead_zone`, `T/T_loop`, gear ratio. Everything that is the same on all twelve joints, in one place |
| 20.1 | **M2 method replaced**: locked-rotor **bus power balance**, not a series ammeter in a phase lead |
| 1, 1a, 10, 16 | Retracted `R_eff` = 0.1977 Ω / "9.3% inter-assembly difference" / "cannot lose counts" purged from the places they still lived |

**Superseded:** `L` = 44.43 / 44.8 / 45.1 µH; `T = 57.0 µs` as a portable number (it is `0.961 × T_loop`); `U0`'s per-fit SE as its real uncertainty; the `\|I\|`/`Iq` free-spin gate; `dead_zone` reopening on bus voltage.

---

## 0c2. Changelog — 2026-08-07 AUTOCALIB + per-joint storage

| § | Change |
|---|---|
| **1a** | **RETRACTED: the 9.3% `R_eff` difference between assemblies.** 9-point autocalib gives A2 = 0.22184 Ω (**+1.8%** vs A1's 0.218) and Ke = 0.017851 (**+0.9%**). The 0.1977 came from a 2-point fit — exactly determined, no residual, undetectably wrong. **Third occurrence of that failure mode.** |
| **8** | **MASTER TABLE relabelled: it is MOTOR 1 / ASSEMBLY A1 ONLY.** Every constant now carries an assembly tag |
| **8.1** | **`T_delay` model corrected: T ≈ ONE FULL LOOP PERIOD.** 3 points, 2 loop rates, T/T_loop = 0.956 / 0.953 / 0.978. The old ½·PWM + ½·loop model is 15–29% low |
| 8.1 | A2 constants added from AUTOCALIB rev1 |
| **NEW 20** | **MANUAL CALIBRATION — permanent section.** What AUTOCALIB does *not* measure, and how to do each by hand |
| **NEW 21** | **PER-JOINT CALIBRATION STORAGE.** `joint_cal.h` + per-joint PlatformIO envs + the `V` verify command |
| 8.3 | Deferral decisions on live Vbus and the PB14 thermistor **re-examined and re-affirmed**, with revised promotion conditions |
| 12 | Four new entries |
| 15 | Roadmap: AUTOCALIB shipped; **the 10 mm build is next and unblocked** |

**Superseded:** the `\|I\|`/`Iq` gate as a free-spin check (regime-invalid — §12); `U0 = 0.028 V` as universal (it's per-board *and* scales with Vbus); the claim that per-unit `R` measurement is empirically required (it isn't — ZEA and direction are).

---

## 0d1. Changelog — 2026-08-06 SPI migration + angle-lag CLOSED

Full file replacement. Every change declared.

| § | Change |
|---|---|
| **NEW 2** | **Two assemblies now exist.** ORIGINAL (ABZ, open fault, preserved) and SPARE (MT6816 SPI, accepted, **now the reference actuator**). Read this section before using any number in this file |
| 1 | MT6816 SPI rows: pins, bit-bang timing, parity rate, stored ZEA |
| 3 | Board silkscreen pin table; **bit-bang-vs-peripheral decision CLOSED with numbers**; third instance of the pruned-pin-map trap (UART on PB6/PB7) |
| 5 | **Near-zero magnet air gap identified** as the likely ABZ root cause; "whine = bearing" **retracted** |
| 6 | ZEA is now a **stored constant**, reproducible across power cycles to 0.03° elec |
| 7 | **INL measured: 0.93° mech pk-pk, 1/rev dominant** |
| **8.1** | **MASTER TABLE SPLIT into per-assembly and shared columns.** R_eff/U0/Ke/Kt/L are per-assembly and do NOT transfer |
| **8.3** | **ANGLE LAG CLOSED.** T = 57 µs by parity separation, matches theory to 7%, 0.58% torque loss at 270 rad/s. **No compensation to be written** |
| 10 | **Per-unit calibration policy for 12 joints** — automate it, don't repeat it by hand |
| 12 | Six new entries |
| 15 | Roadmap rewritten: task 5 closed, task 4 downgraded, C1 promoted |

**Superseded and deleted:** the ABZ angle-lag numbers (T = 85.7 µs, δ₀ = 2.17°) — contaminated by count loss and by combining two alignments; `δ₀` as a fitted parameter (it is the ZEA residual, now +0.12° elec); the claim that `Ud`/`Id` sweeps need low-speed repeats (one 1000-sample capture spans 9.5 revolutions).

---

## 0d2. Changelog — 2026-08-01 bus-sense / M1-d session

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
| 8.3 | ~~Angle lag: preliminary T ≈ 143 µs~~ — **SUPERSEDED 2026-08-06, see §0 / §8.3. Final value 57.0 µs** |
| 8.3 | New deferred: live Vbus sampling; U₀ feedforward with a bus-voltage trigger | |
| **11** | Telemetry line rewritten; **new session-header protocol** | `Vb`, `Vb_src` added; multimeter reading now recorded per session |
| 11 | **Deleted** "`Ud` is NOT currently printed" | Stale — it is printed in three places |
| **12** | Five new failure modes, **one retraction** | All cost time this session |
| 15 | **M1-d closed.** M1-c and the angle-lag sweep are next | |
| 17 | Peak bus current corrected to **230 A, independent of pack voltage**; sag table added; battery spec | Arithmetic error found in the earlier estimate |
| **19** | **New section: battery specification** | Sag, not energy, is the binding constraint |

**Superseded and deleted:** the claim that `Vb` rose 1.1% under load (it never updated at all — see §12); the `U₀ = 0.0194 V` intercept from this session's sweep (ill-conditioned — §8.1); `dead_zone` as permanently closed (it re-opens above ~15 V bus — §8.3).

---

## 0d3. Changelog — 2026-07-29 characterisation session

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