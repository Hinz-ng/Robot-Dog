# M0/M1 Actuator — Quadruped QDD Bring-Up

Single-actuator bench platform for a ~4 kg dynamic quadruped (ODRI Solo / Mini Cheetah
class). Hardware, firmware contract, measured plant characteristics, and the failure
modes that cost the most time.

**Status lives in exactly one place — the task board in §15 below.** No other file in
this set carries a status line, so there is nothing to drift out of sync.

| Before you… | Read |
|---|---|
| flash anything | §4 firmware contract · §11 session workflow |
| believe any number | §12 failure-mode catalogue |
| quote any constant | **§8 — the master table. Everything else defers to it** |
| touch a tensioner | §22 — the belt-off baselines expire and cannot be recovered |
| write STM32 CAN code | §23.7 — the clock is settled (HSE 8 MHz) and §23.8 fixes the bit timing at 8 tq; **HSI16 would violate the tolerance budget by 2×** |

---

## Where everything lives

This was one 1,568-line file until 2026-08-13. It is now a doc set with the **original
section numbers preserved**, so every `§8.2`-style cross-reference in old notes, code
comments and chat logs still resolves. Find the section number, then the file.

| § | Topic | File |
|---|---|---|
| **0 – 0d3** | Dated changelogs, newest first | [`docs/CHANGELOG.md`](docs/CHANGELOG.md) |
| **1, 1a** | Hardware · the two assemblies | [`docs/HARDWARE.md`](docs/HARDWARE.md) |
| **2, 3** | Clone vs genuine · board pin truths (incl. CAN pins) | [`docs/HARDWARE.md`](docs/HARDWARE.md) |
| **4** | Firmware contract · known-good `platformio.ini` | [`docs/FIRMWARE.md`](docs/FIRMWARE.md) |
| **5, 6, 7** | Encoder · alignment/ZEA · current sense and INL | [`docs/SENSING.md`](docs/SENSING.md) |
| **8, 8.1–8.4** | ⭐ **MASTER TABLE** — measured constants, deferred items | [`docs/CONSTANTS.md`](docs/CONSTANTS.md) |
| **9, 9a** | Drivetrain health · the 1/rev localisation | [`docs/CONSTANTS.md`](docs/CONSTANTS.md) |
| **10** | Control — current loop, velocity loop, per-unit policy | [`docs/CONTROL.md`](docs/CONTROL.md) |
| **11, 18** | Session workflow, telemetry · sketch known gaps | [`docs/FIRMWARE.md`](docs/FIRMWARE.md) |
| **12, 13** | ⚠ Failure-mode catalogue · diagnostic ladder | [`docs/FAILURE_MODES.md`](docs/FAILURE_MODES.md) |
| **14, 16** | ST tooling and clone hardware · thermal ground rules | [`docs/HARDWARE.md`](docs/HARDWARE.md) |
| **15** | Status and roadmap · task board | **this file, below** |
| **17, 19** | Robot-level design decisions · battery specification | [`docs/ROBOT_DESIGN.md`](docs/ROBOT_DESIGN.md) |
| **20, 21** | Manual calibration (M1–M15) · per-joint storage | [`docs/CALIBRATION.md`](docs/CALIBRATION.md) |
| **22, 22.1–22.3** | Belt build B0–B13 · measured belt-on states · idler spec | [`docs/BELT_DRIVE.md`](docs/BELT_DRIVE.md) |
| **23, 23.1–23.10** | CAN transport bring-up S0–S2 · FDCAN clock · **the crystal (23.7)** · **bit timing (23.8)** · **S2 results (23.9)** · **robustness results A–D (23.10)** · **the four Tier-0 constraints + FDCAN init checklist (23.6)** | [`docs/CAN_BRINGUP.md`](docs/CAN_BRINGUP.md) |
| **24** | Documentation integrity register — unresolved conflicts | **this file, below** |
| — | **CAN bring-up instruments** — mirrors of the two diagnostic sketches, kept because they re-validate every board during the §23.2 rework | [`tools/can_bringup/`](tools/can_bringup/) |

Method, conventions and the assistant-side failure patterns live in
`project_context.md`; the short version is `CLAUDE.md`. **Neither restates a measured
number** — a second source of truth for constants drifts.

### Writing conventions for this doc set

Set 2026-08-14 after the split produced status lines in two places at once.

| Rule | Why |
|---|---|
| **Status appears only in the §15 task board.** No file header carries "Status:", "next up", "blocked on X" or "as of this session" | Two copies of a status is one copy that is wrong, and the wrong one is always the one someone reads |
| **File headers carry only durable content** — what the file covers, the § routing, and warnings that stay true (e.g. *"these baselines cannot be recovered after B0"*) | A header is the most-read and least-updated part of a file |
| **Date findings, don't tense them.** "Measured 2026-08-12" not "recently measured"; "the run that produced X" not "the current run" | A dated statement is still true in a year. A tensed one silently rots |
| **One source of truth per fact — cross-reference, never restate** | The `J_rotor` tolerance was quoted in three places and corrected in one |
| **Omit anything with low decision value**, however true | Volume competes with the things that matter for attention |

Where a section must reference the work queue, it points at §15 rather than repeating it.

### Step-name prefixes, so three ladders stop colliding

| Prefix | Ladder | Where |
|---|---|---|
| **M1 – M15** | Manual calibration, per board / per assembly / per operating point | §20 |
| **B0 – B13** | Belt-on build and characterisation | §22 |
| **S0 – S2** (incl. S1b–S1e) and **Tests A–D** | CAN transport bring-up | §23 |
| **A3, A3b, A5–A7** | Loose audit and archive tasks from the 2026-08-13 session | task board below |
| **V1** | Live-Vbus / DMA-offset detour — **one row, one decision, closes on the next boot** | task board below · evidence in §0 |
| ~~S0 – S8~~ | July 2026 characterisation ladder — **retired, all closed** | — |

> The belt-on session of 2026-08-12 numbered its steps S1–S7. **Those are B1–B7.**
> Mapping: S1→B1 · S2→B2 · S3→B3 · S4→§9a · S5→B4 · S6a/S6b→B6a/B6b · S7→B7.

---

## 15. Status & Roadmap

**Closed**
- **M0** — toolchain, spin under own firmware, e-stop, thermal sanity, current-sense calibration, two working clone drivers.
- **Encoder** — MT6816 4-wire SPI, 16384 cnt/rev, permanent magnet mount, min-window velocity estimator. (TIM4 hardware quadrature at 4096 CPR on the retired A1 build.)
- **Closed-loop torque (voltage) + velocity (voltage mode)** — characterised, disturbance rejection verified.
- **M1-a — actuator electrical model** — Kt, Ke, R_eff, L, τ_e, U₀ all measured. §8.
- **M1-b — `dead_zone`** — final at 0.005; deadband below the sense noise floor.
- **M1-e — current loop** — 412 Hz, 9.3% overshoot, zero steady-state error, at 80% of the transport-delay ceiling.
- **M1-d — stall clamp (2026-08-01)** — locked rotor, `tgt = 2.00 A` → `Iq` = 1.98–2.03 (mean 2.000), `|I|` mean 2.446 vs 2.449 predicted (0.1%), `Uq` = 0.4604 vs 0.464 predicted (0.8%), `Id` ≈ 0. **`current_limit` binds.** Locked-rotor `ratio` = **1.224**, `Iq_pp` = 0.062 A. `VOLT_LIMIT = 3.5` is authorised for the angle-lag sweep on this evidence.
- **Bus-voltage sense** — PA0 identified and calibrated to 0.07%; divisor is a measurement. Seed-only; live sampling deferred (§8.3).
- **Modulation** — SVPWM verified two independent ways.
- **Instrumentation** — burst logger v2 (per-sample `dt`, `a` stats, capture-condition recording), 921600 telemetry.

- **MT6816 SPI migration (SPARE, 2026-08-06)** — bit-banged 4-wire mode 3, PB5=CSN / PB6=MOSI / PB7=MISO / PB8=SCK, HVPP→3V3, UART unmoved. 2000/2000 reads clean; 2 parity errors in ~600 k reads (0.0003%), each costing one stale cycle. Locked-rotor `ratio` = 1.224 at both 1 A and 2 A. Armed loop 12,490 Hz vs 12,384 predicted. **20 s at ±2.0 V / ±110 rad/s with no drift and no stall — the exact condition that stalled the ABZ rig twice.**
- **✅ M1 angle lag — CLOSED, and re-confirmed at a second loop rate.** `T/T_loop = 0.958 ± 0.015` over 7 points / 3 assemblies / 2 loop rates. **0.95% torque loss at 270 rad/s at 13.3 kHz. No compensation to be written, and the sweep does not need running.** §8.3.
- **Sensor INL characterised** — 0.93° mech pk-pk, 1/rev dominant. §7.

**BELT-OFF CHARACTERISATION OF J01 IS DONE.** Not nearly done — done.

| | |
|---|---|
| `zea`, `dir` | ✅ closed, reproduced across three sessions |
| `R_eff`, `U0`, `L`, `Ke`, `Kt` | ✅ closed |
| Current loop | ✅ closed at 412 Hz, 80% of the transport-delay ceiling |
| `T_delay`, INL | ✅ closed — and the angle-lag sweep that was pending got **cancelled** |
| Drag map, both directions | ✅ closed, cross-validated by two coast-downs |
| Breakaway | ✅ 0.2923 ± 0.0184 A |
| **`J_rotor`** | ✅ **20.2 ± 2.4 × 10⁻⁶ (±12%), three methods, two joints** |
| Cogging | ⚠️ 1.4 mN·m, 13× the stored value. Small, but no longer "negligible, closed" |
| `i_scale` (M2) | ⏸ deferred, three written promotion conditions |

**There is no belt-off measurement left worth taking, with one exception.** The next thing that changes a decision is the belt going on — properly, on rebuilt idlers.

> ### Hard stop rule
> **No further belt-off measurement on J01 unless it would change whether the belt goes on.** Nothing currently qualifies. Friction precision, `J` precision and cogging are all at ±10–20% on quantities that are 1–14% of the loads the robot will actually command; further refinement has essentially zero decision value.
>
> **The one exception, added 2026-08-13: A3b.** A pinion-seated 1/rev capture is the only belt-off measurement that still carries information, because it is the last moment pinion and belt can be separated. Ten minutes. It qualifies under the rule above not because it changes whether the belt goes on, but because **it becomes impossible afterwards** — a different and rarer justification, and the only one that should be accepted here.

### Task board — updated 2026-08-15, ordered by dependency

> ### 🔴 The bearings have arrived — parallel work is over
>
> **From 2026-08-13 the binding constraint was external — bearings in transit — which made
> the CAN ladder legitimate parallel work. It is not external any more.** The belt chain
> (rows 4 → 5 → 6 → 7) is the critical path and everything below it is drift until B11.
>
> **The CAN ladder is closed and it should stay closed:** S0 → S2 both physical layers,
> both clock trees, four Tier-0 constraints, a measured failure mode and a measured
> fail-safe (§23–§23.10). What remains there is one **design** item that needs no
> hardware (the message spec) and one **irreversible** item that is gated on a bus that
> does not exist yet (the 12-board rework). Neither is a reason to go back to the bench.

| # | Task | Time | Blocked by | Status |
|---|---|---|---|---|
| — | **J02 characterised** — M1, AUTOCALIB 1–7, M4 (n=18), M6a | — | — | ✅ done. Row pasted; **`board_sn` / `motor_sn` still `___`** |
| — | **A3 — 1/rev localisation, level 0** | — | — | ✅ **done. 0.042–0.113 N, not motor-internal. Motor swap cancelled** (§9a) |
| — | **A6 / A7 — README corrections** | — | — | ✅ done in this pass: §9a written, §10 gain note boxed, §1/§3 termination corrected |
| — | **Belt-on diagnostic runs, with and without idlers** | — | — | ✅ done 2026-08-12 (§22.1). **Diagnostic only — neither row ships** |
| **A3b** | **L1 capture: one `L` run at Uq = 1.30 V with the pinion seated** | 10 min | — | 🔴 **EXPIRES AT B0.** The last chance to separate pinion bore runout from the motor |
| **1** | **Label the physical parts**, then fill `board_sn` / `motor_sn` in J02's row | 5 min | — | 🔴 the header rule is that `.id` matches what is written on the board, and this hardware still carries A1-era markings. **M1 has since identified the board as `B-ABZ-01` by its divider (0.008516), so the row's `"___"` fields are now the only thing left unlabelled** |
| **M1b** | **One sitting, two boards: banner vs UT89X back to back on J01 and J02 — plus 5× power-cycle banner repeatability on J02** | 10 min | — | 🔴 **§24.14.** Closes the 0.32% that J02's stored `vbus_scale` currently rides on, and establishes the banner's own noise floor, which has never been measured. Also runs the falsifiable prediction that tests the whole 2026-08-18 rescale: **the J01↔J02 `Ke` gap must come back at 0.87%, not 0.06%** |
| ~~**2**~~ | ~~Photograph the motor label — KV360 or KV380?~~ | — | — | ✅ **CLOSED 2026-08-18 — KV360, by bench arithmetic, not a photograph** (§24.1). Two independent `Kt` determinations agree to −0.06% / +0.03%. **Do not reopen on a nameplate photo** |
| **2b** | **Read the ESC1 silkscreen version** — `火柴 FOC V1.0` or V2.0? | 2 min | — | 🔴 **§24.7.** The listing schematic is a V1.0 page and it is now the sole source for `CAN_SHD` = PC11, `Temp_ADC` = PB14 and the HSE routing |
| ~~**2c**~~ | ~~Confirm the SIT1042 `S`-pin polarity and internal pull-up in the datasheet~~ | — | — | ✅ **CLOSED 2026-08-15 — by measurement, which is better than the datasheet** (§23.3). LOW = Normal, HIGH = Standby, **FLOATING = Standby**, and Test C proved the fail-safe holds under a real reset on a live bus. **A rebooting joint is a local event** |
| 3 | Archive both belt-on captures + J02's to `docs/cal/`, **each with a plant-state header** | 15 min | **§24.13 first** | 🔴 unrecoverable after B0, and the belt-on rows are actively misleading without the header. 🔴 **`.gitignore` contains `docs*` — copying a CSV into `docs/cal/` does NOT put it in git** (§24.13). This step currently protects nothing while looking like it worked |
| **4** | **Idler rebuild: bearings + plate revision + slot one hole** (§22.2) | 2 h | ~~bearings in transit~~ **arrived 2026-08-15** | 🔴 **THE BINDING TASK, and now genuinely unblocked.** Measure the bearings with calipers before committing the plate. **3×6 mm shim washers, not the standard ⌀7 M3** — a 7 mm washer bridges the outer race and locks the roller. Screw axis moves out 2.0 mm, which is what makes this a plate revision. **Acceptance: the roller coasts ≥1 s when flicked, and the pen mark rotates without walking axially.** ⚠ Settle the **two build-spec items at the same teardown** (top-plate clearance shim; does the M4 screw preload the pinion bearing?) — §22.2 |
| 5 | **Design + print the output-pulley clamp** | 1 h | — | 🟢 **prerequisite for B6a/B6b**, and far easier now than with legs on |
| 6 | **CAD check: is the belt tensionable with the leg links OFF?** | 5 min | — | 🟢 five minutes that protects four measurements (§22) |
| **7** | **Belt on. §22, B0–B13** | ~2 h | tasks 4, 5, 6 | ⏸ **The chain the bearings were holding up.** Prediction on the table: `drag_c` **0.30–0.40 A** (J01; **J02 projects 0.41–0.50 A**) = 13.8–18.5% of standing load (§22.2). ⚠ **Gate on the amps.** The percentage has been restated twice for calibration corrections while the amp target never moved. ⚠ **B10's method changed — the motor cannot skip the belt.** 0.48 N·m at the output at `current_limit = 2 A` against a 3.4–5.7 N·m skip threshold, a 7–12× shortfall: static lever + spring gauge instead (§22). **B11 ships `drag_c` to `joint_cal.h`** — the first belt-on constant to become fleet-authoritative |
| ~~**S0**~~ | ~~CAN: ESP32 alone, NO_ACK, analyzer~~ | — | — | ✅ **CLOSED 2026-08-13** (§23.4) |
| ~~**S1**~~ | ~~CAN: ESP32 ↔ ESP32~~ | — | — | ✅ **CLOSED 2026-08-14** (§23.4b). TEC/REC 0 over ~200k frames, `rxbad` 0 both nodes, arbitration proven by `arblost` 29/0. **Also retracted the bus-off claim** |
| ~~**S1b**~~ | ~~CAN: ESC1 transceiver probe~~ | — | — | ✅ **CLOSED 2026-08-14** (§23.3). `S`-pin LOW = Normal **measured**, floating = Standby; **transceiver loop delay 125 ns**; CANH/CANL bias 2.48 V |
| ~~**F1**~~ | ~~Two facts for S1c: HSE crystal present? Max `fdcan_ker_ck`?~~ | — | — | ✅ **CLOSED 2026-08-14** (§23.5, §23.7). **Crystal fitted, 8.000 MHz.** Fact 2 turned out **moot** — it only mattered on the no-crystal branches |
| ~~**S1c**~~ | ~~CAN: FDCAN external loopback~~ | — | — | ✅ **CLOSED 2026-08-14** (§23.7). Loopback + **the `x` mux test at 52.05% dominant**, which is what actually proved AF9 — loopback alone could not |
| ~~**S2**~~ | ~~CAN: ESP32 ↔ ESC1 on real wire~~ | — | — | ✅ **CLOSED 2026-08-14** (§23.9). `rxbad=0` over 21,800+ frames, `buserr=0`, `arblost=0`, and the ESC1 parked at TEC 128 **without hammering** — `DAR=1` proving itself. Last rung of the ladder |
| ~~**S2b**~~ | ~~Measure ESC1 bit time on the analyzer~~ | — | — | ✅ **NOT NEEDED** (§23.9). S2 itself bounds f_HSE at **±0.97% against the ESP32's crystal**, with no HSI16 in the chain — and the analyzer's own two-cursor quantisation is ±0.83%, essentially a tie, on a question needing ±25%. **Crystal closed at 8.000 MHz** |
| ~~**S1d**~~ | ~~Does an error-passive node starve a joining partner?~~ | — | — | ✅ **CLOSED 2026-08-15, PASSED** (§23.4b, §23.10). Answered by Test C's recovery leg, in the *harder* configuration: the throttling node was genuinely retransmitting at 7874/s and the joiner was clean in **~640 ms**. **Suspend transmission works; the starvation claim stays retracted** |
| ~~**A–D**~~ | ~~Four robustness tests: loopback babbler · boot order · joint reboot · TEC recovery~~ | — | — | ✅ **CLOSED 2026-08-15** (§23.10). **A ✅ PASSED** — a loopback node babbles, partner driven to `BUS_OFF`, run 1 explained. **C ✅ PASSED — a rebooting joint cannot disturb the other eleven**, six signatures matching the known-silent control. **D ✅ PASSED — `TEC` recovery 640 ms**, decrement exactly 1/frame. **B dropped**, see S1d |
| ~~⚪~~ | ~~Tx Event FIFO drop-rate run~~ | — | — | ✅ **CLOSED 2026-08-15** (§23.10). **0 drops in ~10,000 frames**, upper bound ~0.03%. `DAR=1` confirmed in both directions — discarding at 200 Hz with the partner absent, frozen when healthy |
| ~~**C1**~~ | ~~Get the bring-up sketches into version control~~ | — | — | ✅ **DONE 2026-08-15** — mirrored to [`tools/can_bringup/`](tools/can_bringup/). ⚠ **The S1b GPIO probe is not among them** and is not in either project; §23.2's per-board acceptance test names it, so that gap has to be closed before the rework |
| 🔴 | **12-board rework: the 120 Ω resistor comes off ALL 12 ESC1s** | 3–4 h | **first bus with >2 nodes** | ⏸ **NOT DONE, deliberately deferred** — §23.2 carries the full procedure and acceptance test. Revised from "10 of 12": termination lives in the **harness**, so no joint is ever special and the topology stays a screwdriver decision. **Acceptance UNPOWERED, ~60 Ω** — a powered transceiver adds a ~450 Ω bias path and 12 of them parallel to ~38 Ω, swamping the reading. Optional 30-min de-risk: **do J03 alone** — the one board not part of a characterised joint |
| ~~⏸~~ | ~~**Bit-rate / SYSCLK coupling decision**~~ | — | — | ✅ **CLOSED 2026-08-14 — the coupling never fired.** A fitted 8 MHz crystal means FDCAN takes HSE directly, so SYSCLK, the PWM frequency, the loop rate and `T_DELAY_PER_LOOP` are **all untouched**. The 1 Mbit freeze stands and did not have to be re-opened (§23.5) |
| ⏸ | Extract `mt6816.h` / `actuator_hw.h` / `safety.h` — 45 min, not the 2-hour refactor | 45 min | after B11 | ⏸ before the first line of Tier-0 |
| ~~⏸~~ | ~~M2 / `i_scale`~~ | — | — | ✅ **CLOSED 2026-08-20, both boards** (§8.1c). **J01 `i_scale` = 0.9621 ±1.2%, 3.2σ — decisive.** J02 = 0.9690 ±2.7%, 1.2σ — **provisional**. The self-fit formulation (`g = 1.5·R_M2/c` from the ladder's own `R`) removed the cross-session dependency and the bypass clip. **`R_b` and `V_s` proven inert** (±16% on `R_b` → ±0.03% on `g`) — an earlier claim that `R_b` gated this is retracted |
| **M2a** | **`AC_M2_V[]` widened 5 → 8 points, dwell 20 s → 12 s** | done | — | ✅ **APPLIED 2026-08-20.** Takes the quadratic from 2 dof to 5, which is what separates J01's decisive 0.66% SE(`c`) from J02's 2.37%. J03 onward inherits it. Thermal load is unchanged by construction (217 J → 215 J) |
| **M2b** | **Re-run M2 on J02** | 30 min | **only if something needs sub-1% torque accuracy** | ⏸ **DEFERRED WITH A CONDITION, and it is not a temporal one.** A repeat moves the stored number by ~1% and changes no action today. The 8-point ladder and the 12 s cap fix two of the three identified causes for free on the next run |
| ~~⏸~~ | ~~Seed averaging in the pre-init read~~ | — | — | ✅ **ALREADY IMPLEMENTED — the request was based on a wrong premise** (§24.14). `open_test.cpp` discards the first conversion and averages 64. The 0.05 V per-boot outlier is a **real offset**, not sample noise, so no amount of averaging addresses it; the fix is the session-start meter check |
| **V1** | **Delete the DMA-offset diagnostic scaffolding** — `seed_ch5` / `seed_ch11` / `seedVsDmaDump` / `vbusLoadTest` / `adcSetSmp` in `open_test.cpp`, the `Vdma` telemetry field, the trailing `Vdma` columns on the `M2,` row in `autocalib.h`, and `adcCalibrate` / `ADC_PRECAL` / `ADC_CCR_OPERATING` **unless those become the permanent fix** | 15 min | **ONE BOOT: the rev-2 two-pass calibration either lands or it does not** | 🔴 **THE CAUSE IS NAMED (§0) — this is now a decision, not a wait.** `CALFACT = 0`; the DMA converter has never been calibrated. Rev 1 confirmed it (each instance moved by exactly its own `CALFACT`) and **over-corrected 1.97×** by calibrating at the undivided 170 MHz clock. **Rev 2 is flashed and self-discriminating: `CCR_found` + two `CALFACT` passes in one boot.** ✅ **land** (`CF` 117 → ~60, Δ ≤ 8 counts) → `adcCalibrate` + the `CCR` write become ~20 permanent lines, the rest of this row executes, live Vbus unblocks **with per-board verification**. ❌ **miss** → `ADC_PRECAL = false`, `VBUS_LIVE = false`, execute the whole row. **Either way this row closes on the next boot.** 🔴 **`adc_precal=1` is a diagnostic build — no run of record may use it** (§4). Flash: 99,808 → 108,716 B, **+8.7 K, 76.1% → 82.9%** |
| ⏸ | The 478 mm apex mass question | 20 min | — | ⏸ **before the controller energy budget** (§17) |
| ⏸ | **CAN message spec + RL observation/action vector** | — | ~~bit-rate decision~~ · **after B11** | ⏸ **The largest open CAN item and it needs no hardware** (§23.6) — which is exactly why it waits: the belt is the critical path and this competes for the same attention rather than for the bench. Inherits **four measured Tier-0 constraints**: single-shot control frames · loopback power-on-only with a quiescent bus · PC11 LOW before FDCAN · HSE + `HSERDY` poll with loud failure. §23.6 also carries the **ordered FDCAN init checklist** — write Tier-0 from it rather than re-deriving. ⚠ **Give it a fixed scope or it expands: frame IDs, field packing and scaling for `{p_des, v_des, kp, kd, τ_ff}` → `{p, v, τ}`, and the poll schedule** |
| ⏸ | **S1e — measure the REAL control frame width** | 20 min | **message spec frozen** | ⏸ **deferred with a physical condition, not a temporal one.** 116 bits is the *test* payload; stuff-bit count is data-dependent, so measuring an ascending-counter `0x200` frame says nothing about the actuator contract (§23.7) |

> **One flag, then dropped.** M2 has been deferred across four sessions while
> instrument research continued around it. The instrument question is closed —
> **the remaining cost is one bench measurement**, and it is 30 minutes that does not
> expire. Nothing further to research.

### The two-joint fleet picture

> ### 🔴 Both rows were RESCALED on 2026-08-18 by M1 — read this before comparing anything
> **`vbus_scale` is per board, and it had never been measured on either board with a checked meter.** M1 against a UT89X:
>
> | Board | Joint | `vbus_scale` | Rescale applied to `R_eff`, `U0`, `Ke`, `L` |
> |---|---|---|---|
> | `B-SPI-01` | J01 | 0.008448 (was 0.008358) | **×1.010768** |
> | `B-ABZ-01` | J02 | **0.008516** (had *inherited* 0.008358) | **×1.018904** |
> | | | **0.80% apart, MEASURED** | |
>
> Two errors, one root cause — a single board's divider treated as fleet. J01's original figure was **1.1% low because of the meter, not the fit**: the DT9205A used for it has a ~1.11% DCV gain error, and `0.008358 × 1.0111` lands **0.03%** from the re-measurement. J02 then inherited that same wrong number *and* was on a different board. **`joint_cal.h` carries the full provenance; §8.1a and §8.1b in [`CONSTANTS.md`](docs/CONSTANTS.md) carry the corrected tables.**

| | J01 | J02 | Spread |
|---|---|---|---|
| `vbus_scale` | 0.008448 | 0.008516 | **+0.80% — measured, per board** |
| **`i_scale`** (M2 2026-08-20) | **0.9621** ±1.18% | **0.9690** ±2.69% ⚠ provisional | **0.72%** — inside J02's own uncertainty |
| `R_eff` | 0.22346 Ω | 0.22810 Ω | **+2.08%** (was read as +1.26%) |
| `Ke` | 0.017941 | 0.018097 | **+0.87%** (was read as +0.06%) |
| `L` | 43.77 µH | 46.25 µH | +5.7% (was +4.8%) |
| `U0` | 0.01037 V | 0.014937 V | +44% — the weak parameter, as always |
| `τ_e = L/R` | 195.87 µs | 202.74 µs | **ratios — the rescale did not move them at all** |
| Breakaway | 0.2923 A | 0.2983 A | **0.19σ** — amps, untouched |
| `T/T_loop` | 0.945, 0.974 | 0.937, 0.961 | fleet 0.958 ± 0.015, n=7 — untouched |

> ### ⚠ RETRACTED 2026-08-18 — the `R`-vs-`Ke` argument that used to sit here
> It read: *"`R_eff` moved 1.26% while `Ke` moved 0.06%, and both scale with the firmware's voltage belief. A scale error would have moved them together. It did not — so the `R` difference is real."*
>
> **That assumed the two boards shared one scale. They do not.** A per-board scale difference moves `R` and `Ke` together *within* a board, so a **cross-board** comparison never tested what it claimed. "The first time the fleet table has been able to separate those two explanations" goes with it — **what actually separated them was M1.** The `R` difference may still be real (FET `R_ds(on)`, shunt, solder joints, wire length), and keeping `R_eff` per-unit is still right; there is simply no longer an argument here that it is.
>
> **Steelman, kept because it is the one piece of evidence *against* the correction:** two independently built motors agreeing on `Ke` to 0.06% is suspiciously good, and happens only if the dividers are identical. Direct measurement outranks a cross-joint coincidence — but it makes a **falsifiable prediction. Re-measure both banner/meter ratios in ONE sitting: if the corrections are real, the `Ke` gap comes back at 0.87%. If it comes back at 0.06%, the dividers are equal and the M1 reads are wrong.** That test is §24.14.

**The `L` = 65 µH scare is closed.** Same motor, same board: 65 µH under the old 5-point phase-locked fit, **46.25 µH** under the 18-point dithered one. It was a fit artefact, not a different motor, and `L` is a near-fleet quantity at ~45 µH. Do not quote it better than ±2% — an offline refit of the `LSB` block gives τ = 206.3 µs against the firmware's 202.8, the fit band edge moving by one bin. **τ is a ratio and is immune to the rescale**; the µH figure is not.

### Why the 45-minute extraction is not the 2-hour refactor

`open_test.cpp` is a **bench harness, and it will be replaced rather than shipped** — `fleet_config.h` says so itself ("a test harness owns them; they are not shipped to the robot"). §9's no-logic-in-main rule was written for the robot firmware. Three reasons to leave it until after J02:

1. **The rig currently produces correct measurements**, and J02's characterisation runs on it unchanged. *One variable at a time* applies to code as much as to hardware — restructuring a working instrument mid-campaign is the standard way to lose a week to a bug that reads as a hardware fault.
2. **RAM is at 73.9% with ~8.5 kB of headroom.** Splitting into translation units shifts static allocation in ways that would have to be re-verified.
3. **The Tier-0 deliverable is not a refactored `open_test.cpp`.** It is new firmware implementing `{p_des, v_des, kp, kd, τ_ff}` over CAN — and the parts that *do* ship, `joint_cal.h` and `fleet_config.h`, are already extracted. AUTOCALIB stays in the harness permanently; it is needed for all twelve joints and every re-characterisation after.

**What is worth extracting, and only this:** `mt6816.h` (SPI bit-bang, parity, `No_Mag`), `actuator_hw.h` (driver/sense/motor construction, the hard-won **init order**, boot Vbus read, `runInitFOC()`), and `safety.h` (`stopMotor()`, guard thresholds, the single disable path). The third is the one that matters — §9 says *"single safety path; new e-stop call sites need design review"*, and if the harness and Tier-0 each grow their own copy, that rule is broken before Tier-0 compiles. **Timing: after J02 is characterised, before the first line of Tier-0.** Not now, because J02 runs on known-good code; not later, because by then the duplication exists and the extraction becomes a merge.

### The three real risks — none of them is a precision question

1. **The performance envelope may be stale.** §17's apex figure carries an unresolved mass question — **documentation, not measurement, and still larger than any precision question left on the bench.** ~~and every force in §8.2 carries an unmeasured `i_scale`~~ ✅ **the `i_scale` half is closed** (M2, 2026-08-20): the forces moved **up** 3–4% and now carry ±1.2% (J01) / ±2.7% (J02) instead of an unquantified ±(0–6)%. **The remaining multiplicative unknown on every force is `DRIVETRAIN_ETA`, which is still circular** — §24.6, M14.
2. **The belt-off baselines expire on first belt fitment.** Drag map, breakaway, `J_rotor`, INL — all captured, none archived. **Get them into git before touching a tensioner** — and 🔴 **read §24.13 first: `.gitignore` has `docs*`, so `docs/cal/` is not in git.** This risk has been recorded for three sessions with a mitigation that would not have worked.
3. **`open_test.cpp` is 1179 lines holding all logic and all wiring**, against §9 of the working context. Fine as a bench harness; a problem at Tier 1. Promotion condition already written (§8.3).

**Deferred, with promotion conditions** — see §8.3 for the full table.
`DRIVER_VOLT_LIMIT` 6.0 → ~V_bus, **before the first commanded velocity above 150 rad/s** · `open_test.cpp` structure, **before CAN / Tier-1 integration** · ~~M2 / `i_scale`~~ **done 2026-08-20** · **applying `i_scale` in the torque path — `calKtCmd()` exists and has NO caller; Tier-0 is the first, and the current limits are all in reported amps until then** (see the block in `fleet_config.h`) · live Vbus, **≥10 A bench currents or cross-session constant comparison — and now also gated on the DMA path's uncalibrated-converter offset, row V1 above (§0)** · ABZ fault on A1, un-blocking, whenever.

**Downgraded**
- ~~M1-c velocity harness~~ → **optional.** It existed as an instrument for the angle-lag sweep, which is now closed. Voltage-torque mode self-regulates to steady speeds and is sufficient for drag mapping. Build it only if a later task needs commanded speed. If built: `P ≈ 0.2, I ≈ 1.5` (§10).

**Deferred, un-blocking**
- **ABZ fault diagnosis on the ORIGINAL assembly** (15 min whenever). ZEA-delta test, n=4 alignments per side, at −2.07 V for 20 s. `< 13 counts` closes the near-zero-air-gap hypothesis; `> 20 counts` reopens it. The assembly is preserved un-modified for exactly this.

**Deferred with reasons** — see §8.3 for the full table with promotion conditions.

---

---

## 24. Documentation integrity register

Conflicts found while auditing the doc set on 2026-08-13. **Each needs a human decision
or a two-minute observation — none should be silently resolved**, because in every case
two sources of the doc set disagree and picking one by inference is how a wrong number
becomes authoritative.

| # | Conflict | Why it matters | Resolution |
|---|---|---|---|
| **24.1** | ~~Motor nameplate: KV360 or KV380?~~ | `claude.md` / project memory recorded **KV380**; §1, §8.1 and §8.1a all compute against **KV360** | ✅ **RESOLVED 2026-08-18 — KV360**, and it stays resolved. ⚠ **AMENDED the same day: the evidence is weaker than it was written.** The two teardown `Kt` determinations were computed at a `vbus_scale` that M1 then found 1.1% low, so they land at **+1.02%** and **+1.11%** against KV360's `Kt = 0.026526`, not −0.06% and +0.03% — **"the tightest agreement this project has recorded" is withdrawn.** KV380 moves from ~5.5% to **~6.6%** out by the same factor, so the ~5.5-point margin is intact and the verdict does not turn on the correction. **Second, a `Kt`-vs-nameplate comparison can never confirm `vbus_scale` — `Ke` is computed from it, so that use was circular and is retracted everywhere it appeared.** It discriminates a nameplate error; it cannot see a 1% divider error. **Do not reopen on a photograph** (§2a in `HARDWARE.md`) |
| **24.2** | **Target mass: 3.0 kg or 4.0 kg?** The working instructions say **3 kg**; §17 freezes **4.0 kg** and §19 sizes the pack as "≤700 g = 17.5% of 4 kg" | **Every "% of standing load" figure in the doc set divides by 9.81 N = 4.0 kg.** At 3.0 kg the divisor is 7.36 N and every transparency percentage rises by 33% — breakaway 13.5% → 18.0%, belt-on 12.7% → 16.9%. The *ratios* between friction terms are unaffected; the pass/fail bands in §22 are not | **Decide the number and state it once.** §17 is the place. 5 min, no bench |
| **24.3** | **Pack: 5S frozen or 6S live?** `claude.md` freezes **5S**; §17 and §19 treat **6S as re-opened** with a staged commissioning plan | §19's apex table spans 66–639 mm across pack choices, so this is the largest single lever on the headline capability figure that is still open | **Either re-freeze 5S and mark §19's 6S rows as an unexercised option, or unfreeze in `claude.md` with the bench evidence named** (HG5511D 60 V, a 22.5 V run, a 34.2 V divider) |
| **24.4** | **`Ke` for the no-idler run: 0.017952 or 0.018035?** Firmware fit vs offline refit of the same capture | 0.46% apart when they should agree. Neither is carried into `joint_cal.h`, so nothing depends on it — but an unexplained disagreement between the firmware's fit and an offline refit of its own data is a tripwire worth pulling now rather than during a run that matters | **Re-fit the archived CSV.** 5 min, no bench (§22.1) |
| **24.7** | **ESC1 board revision: V1.0 or V2.0?** §1 records "Matches FOC **V2.0**"; the vendor listing photographs a board silkscreened "火柴 FOC **V1.0**" | The listing's schematic page is now the only source for `CAN_SHD` = PC11, `Temp_ADC` = PB14, the 48 V divider factor and the HSE routing. If the physical board is V2.0, all four are sourced from a document that may not describe it | **Read the silkscreen** (2 min). If V2.0, request the V2.0 schematic before S1c |
| **24.8** | **PB14 thermistor: the vendor's transfer function has the wrong sign for our data.** Vendor: `V0` = 1.4 V at 25 °C, **+0.019 V/°C**. Ours: 1431 → 1267 counts *as the board warmed* | Both readings imply ~5–12 °C on the vendor's formula, and they move the wrong way. Either the clone inverts the divider leg, or the two readings are invalid — and a frozen-ADC artefact has already been caught on this peripheral once | **One deliberate warm-up run**: 1.5 A for two minutes, watching PB14 continuously (§16). Do not adopt the formula until then |
| **24.9** | **The vendor rates the board below the design operating point:** continuous < 10 A, instantaneous < 40 A, *"not recommended for high-current drive"* — against 30 A peak per motor | 30 A is 75% of the absolute instantaneous claim and 3× the continuous one, on twelve boards. A ~35 ms jump pulse is plausibly inside "instantaneous"; a sprint gait is not obviously inside "continuous" | **Instrument, do not reason.** M10 hot-vs-cold `R_eff` plus FET temperature after a realistic duty cycle, once the belt is on (§16) |
| **24.10** | ~~Shunt value: 20 mΩ or 3 mΩ?~~ | — | ✅ **RESOLVED 2026-08-14: 3 mΩ (`R003`), confirmed in the vendor's board photo.** The firmware constant was right and its stated justification was invented. A 3.3× shunt mismatch is excluded, so **M2 is not escalated** (§2) |
| **24.11** | ~~"External loopback proves the pin mux and the transceiver path"~~ | The M_CAN spec disregards the Rx pin in **both** loopback modes, so `e` matches `i` even with a broken mux. It had reached §23.5 **and** the sketch header, and would have licensed skipping straight to S2 on a false pass | ✅ **RESOLVED 2026-08-14.** Retracted in §23.5, replaced by the `TEST.RX` mux test — **52.05% dominant against ~50% predicted from the frame's own duty cycle** (§23.7) |
| **24.12** | ~~"A lone node goes bus-off after ~32 unacknowledged frames"~~ | Retracted at S1 on bench evidence (§23.4b), **but it reappeared in §23.2's primer during a later restructure of that file, and §23.4b itself went missing** | ✅ **RESOLVED 2026-08-14** — §23.4b restored from the session copy, §23.2 corrected in place. ⚠ **Second time this claim has come back.** It is persuasive and wrong; corrections to it are boxed, never deleted |
| **24.13** | 🔴🔴 **`.gitignore` line 7 is `docs*`. The entire doc set has never been in version control.** Found 2026-08-15 while investigating the second silent section loss. **11 files are tracked** — `README.md`, `platformio.ini`, four in `src/`, and stubs. Every one of `CAN_BRINGUP.md`, `CHANGELOG.md`, `CONSTANTS.md`, `BELT_DRIVE.md`, `CONTROL.md`, `SENSING.md`, `HARDWARE.md`, `FAILURE_MODES.md`, `FIRMWARE.md`, `ROBOT_DESIGN.md`, `CALIBRATION.md` is **ignored**. The pattern predates the 2026-08-13 doc split and silently swallowed it | **This is the root cause of both section losses being unrecoverable, and it is larger than that.** ① §8 — the master table every other number defers to — has no history, so a changed constant leaves no trace. ② **🔴 It defeats task 3 and risk #2 below.** *"Archive the belt-off captures to `docs/cal/` … get them into git before touching a tensioner"* — **`docs*` means putting a file in `docs/cal/` does not get it into git.** The archive step would appear to succeed and protect nothing, and the baselines are unrecoverable after B0 | **A human decision, not a silent fix — it changes what the repo contains.** The options are: remove `docs*` and commit the doc set (`docs/cal/README.md` is already tracked, so the pattern is being worked around by hand already); or narrow it to whatever it was actually written for; or `git add -f` the doc set and the archives. **Do this before task 3, not after** |
| **24.14** | 🟡 **The banner's per-boot repeatability — measured 2026-08-20, and the explanation for it was wrong** | **5 back-to-back power cycles of J02: four boots at 12.29 V, one at 12.34** — 5.9 counts, 0.05 V. ⚠ **The "single unaveraged pre-init `analogRead()`" explanation this register carried is RETRACTED.** `open_test.cpp` already discards the first conversion and averages **64**; white noise is suppressed ~8× and a 5.9-count outlier on that mean would need a per-sample sd of ~47 counts. **It is a real per-boot offset** — pack recovery between power cycles is the leading candidate — and averaging harder cannot touch it. **Why it matters:** `V_seed` lands 1:1 on M2's `g`; booting on that outlier would have biased J02's `g` by **+1.23%**, larger than J01's whole error budget, with no symptom in the data | ✅ **Mitigated procedurally, not in firmware** — the fix for a real offset is not a longer average. **Session-start gate, now written into [`CALIBRATION.md`](docs/CALIBRATION.md) §20.1: banner vs meter, \|Δ\| > 0.03 V → reboot before proceeding.** 🟡 **Still open:** the 0.32% `vbus_scale` ambiguity below, which this data does not settle — it bounds the *scatter*, not the *offset* |
| **24.14b** | 🟡 **The two J02 banner/meter pairings are not simultaneous, and 0.32% of `vbus_scale` rides on which one is right** | `banner / V_true` is a **fixed property of a board at a fixed scale.** J02 produced two pairings against the same UT89X reading of 12.33 V: banner 12.14 at scale 0.008358 → implies **0.008489**; banner 12.29 at scale 0.008489 → implies **0.008516**. Those imply ratios of 1.000 and 0.997 — it cannot be both, so **one pairing was not taken in the same sitting as its meter reading.** **Decided 2026-08-18 in favour of one sitting at 12.33 V, so `joint_cal.h` stores 0.008516** and every J02 constant is scaled ×1.018904 accordingly. The residual 0.32% sits **below the UT89X's own ±0.66% at 12.33 V on the 60 V range**, so these instruments cannot resolve it — it is bounded, not settled | **Ten minutes, one sitting, one pack, no charging in between.** Boot J01 → read banner and UT89X terminals back to back. Power down, swap to J02 → boot → read both back to back. Two ratios. **Take the 5× power-cycle repeatability at the same time** (J02, nothing else touched, record the banner each boot) — the seed is a *single unaveraged pre-init `analogRead()`* at 8.5 mV/count, ±2 counts of sample noise is ±17 mV on its own, and boot inrush can latch it low. **Until that number exists there is no noise floor, and every banner comparison including this one is uninterpretable.** The same sitting also runs §8.1b's falsifiable prediction: the J01↔J02 `Ke` gap must come back at **0.87%**, not 0.06%. ⚠ **2026-08-20 partially closed this and it is now LOWER priority, not higher:** the banner read 12.29 against a UT89X 12.27 — **0.02 V, 0.16%, against that meter's own ±0.064 V at 12.3 V.** The disagreement is a *quarter* of the resolving power of the instrument judging it, so **`vbus_scale` stays at 0.008516 and chasing this would move `R_eff`, `U0`, `Ke` and `L` by less than their own uncertainties on evidence the meter cannot supply.** It is folded into M2's error budget instead, where it is worth 0.16% of `g` |
| **24.5** | ~~`i_scale` is still 1.0 on every joint~~ | ~~Every N and N/A in the doc set carries an unquantified ±(0–6)% common-mode factor~~ | ✅ **RESOLVED 2026-08-20 — M2 ran on both boards.** `i_scale` **0.9621 ±1.2%** (J01, 3.2σ, decisive) and **0.9690 ±2.7%** (J02, 1.2σ, ⚠ **provisional** — fails the project's own ±2% standard for M2). **The sense UNDER-reads, so every force in the doc set went UP by 3–4%, not down**, and the unquantified ±(0–6)% band is replaced by ±1.2% / ±2.7%. Two boards 0.72% apart, both ~3.5% low → points at the `LowsideCurrentSense` constant, **not** per-board shunt tolerance — **but NOT pooled into a fleet constant** (§8.1c: two samples can fail to reject commonality, they cannot establish it, and pooling is the exact move that hid the 0.80% `vbus_scale` difference). ⚠ **`i_scale` has no consumer in the firmware yet** — Tier-0 is the first |
| **24.6** | **`DRIVETRAIN_ETA = 0.92` is circular** | Second unmeasured multiplicative factor on every force figure, announced in the boot banner. Unchanged; listed for the same reason as 24.5 | M14, load cell on an assembled leg |

### Defects fixed in this pass, declared

| Where | Defect | Fix |
|---|---|---|
| §15 | Two headings duplicated verbatim (`### Why the refactor is *not* item 7`, `### The three real risks`) — copy artefacts | Deduped; the surviving heading is the more specific of each pair |
| §12 | The learning *"a fit can measure one parameter superbly and another not at all"* appeared twice, in two lengths | Left in place for now — **flagged, not fixed**, because the two versions cite different datasets (a generic one and the 2026-08-01 sweep) and merging them would lose the second's specifics |
| §15, §8.3, §17 | `J_rotor` still quoted as **±2.0 × 10⁻⁶ / ±10%** after the 0a changelog widened it to **±2.4 / ±12%** | Corrected in all three |
| §15 | `T/T_loop` quoted as **0.961 ± 0.014, n=5** after 0a moved it to **0.958 ± 0.015, n=7** | Corrected |
| §1 | Encoder row described **ABZ / 4096 CPR** as though current; both live joints are SPI at 16384 | Corrected, with the ABZ figure kept as history |
| §1 | *"Possible angle latency — §8 deferred list"* — closed since 2026-08-06 | Marked closed |
| §5 | *"Preliminary T ≈ 143 µs"* still read as a current figure | Marked superseded |
| §1 | *"6 mm belt is the force ceiling — moving to 10 mm"* | 10 mm is fitted; row updated with belt length, centre distance and take-up budget |
| §10 | Design rule stated as `L/R = 300 µs`, which is the **retired** `L = 65 µH` | Boxed with the real mismatch and a do-not-fix warning |
| §7 | *"must be re-measured after the belt is fitted, because belt tension changes DISP"* | The measurement contradicted the prediction; demoted to a confirmation with the evidence |
| §22 | B9 (preload dead-zone check) sat five steps after the work it duplicated | Merged into **B6a** and moved up — its position was an invitation to skip it |
| §2, §7 | *"Shunts: 20 mΩ (`R020`), amp gain scaled to compensate"* — **the value was wrong and the conclusion it justified was right** | Corrected to 3 mΩ (`R003`) with the hazard spelled out: a wrong justification invites a future session to change a correct constant |
| §2 | The vendor's *"all B-G431B-ESC1 examples work directly"* claim went unchallenged | Marked false, with the M0 gate-driver saga named as the counter-example |
| §3 | Four analogue pins identified only by probe sweep, two of them as *"probable"* | `Temp_ADC` = PB14 and `SpeedBT_ADC` = PB12 **confirmed** against the schematic; a complete 20-row pin map added, every unanchored row flagged |
| §3 | `PB10` documented as *"do not touch it"* with no stated consequence | Both divider ranges tabulated (34.23 V measured / 65.3 V vendor), plus a **new silent-failure hazard**: any future firmware that drives PB10 moves `vbus_scale` by 1.9× with no symptom |
| §23.3 | Transceiver *"unmarked, not confirmed as TCAN330"*; mode polarity unknown | **`SIT1042QTK/3` on 5 V, `S` on PC11, LOW = Normal.** S1b reduced from 30 min to ~10 |
| §23.3 | **Pin 5 corrected 2026-08-14: it is VIO, not VREF** — VREF doesn't exist on this part (that's a TJA1040/1050 feature). The planned "meter pin 5 for mode" check was dead on two counts — wrong pin, and the IC is unreachable with a probe anyway | Substituted the **CANH/CANL bias** (~2.5 V Normal / ~0 V Standby at TP4/TP12, no IC access) as the mode-indicator check. "3.3 V-compatible logic" downgraded from stated fact to **conditional on VIO's net**, unchecked — flagged as a 30-second schematic read for S1b |
| §23.5 | Fact 1 was to be settled *"by looking at the board near pins 5 and 6"* | Replaced with an `HSERDY` poll plus an LED-and-stopwatch frequency check, and a note on why `MCO` on PA8 must not be used |