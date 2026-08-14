# M0/M1 Actuator — Quadruped QDD Bring-Up

Single-actuator bench platform for a ~4 kg dynamic quadruped (ODRI Solo / Mini Cheetah
class). Hardware, firmware contract, measured plant characteristics, and the failure
modes that cost the most time.

**Two joints (J01, J02) are fully characterised belt-off. The belt is fitted but the
idlers are faulty and being replaced.** Current binding work: the idler rebuild (§22.2),
with CAN transport bring-up (§23) as legitimate parallel work while bearings ship.

| Before you… | Read |
|---|---|
| flash anything | §4 firmware contract · §11 session workflow |
| believe any number | §12 failure-mode catalogue |
| quote any constant | **§8 — the master table. Everything else defers to it** |
| touch a tensioner | §22 — the belt-off baselines expire and cannot be recovered |
| write STM32 CAN code | §23.5 — the FDCAN clock decision is not settled |

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
| **23, 23.1–23.6** | CAN transport bring-up S0–S2 · FDCAN clock | [`docs/CAN_BRINGUP.md`](docs/CAN_BRINGUP.md) |
| **24** | Documentation integrity register — unresolved conflicts | **this file, below** |

Method, conventions and the assistant-side failure patterns live in
`project_context.md`; the short version is `CLAUDE.md`. **Neither restates a measured
number** — a second source of truth for constants drifts.

### Step-name prefixes, so three ladders stop colliding

| Prefix | Ladder | Where |
|---|---|---|
| **M1 – M15** | Manual calibration, per board / per assembly / per operating point | §20 |
| **B0 – B13** | Belt-on build and characterisation | §22 |
| **S0 – S2** (incl. S1b, S1c) | CAN transport bring-up | §23 |
| **A3, A3b, A5–A7** | Loose audit and archive tasks from the 2026-08-13 session | task board below |
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

### Task board — 2026-08-13, ordered by dependency

**The binding constraint is external (bearings in transit), so parallel work is
legitimate right now.** That stops being true the moment they arrive.

| # | Task | Time | Blocked by | Status |
|---|---|---|---|---|
| — | **J02 characterised** — M1, AUTOCALIB 1–7, M4 (n=18), M6a | — | — | ✅ done. Row pasted; **`board_sn` / `motor_sn` still `___`** |
| — | **A3 — 1/rev localisation, level 0** | — | — | ✅ **done. 0.042–0.113 N, not motor-internal. Motor swap cancelled** (§9a) |
| — | **A6 / A7 — README corrections** | — | — | ✅ done in this pass: §9a written, §10 gain note boxed, §1/§3 termination corrected |
| — | **Belt-on diagnostic runs, with and without idlers** | — | — | ✅ done 2026-08-12 (§22.1). **Diagnostic only — neither row ships** |
| **A3b** | **L1 capture: one `L` run at Uq = 1.30 V with the pinion seated** | 10 min | — | 🔴 **EXPIRES AT B0.** The last chance to separate pinion bore runout from the motor |
| **1** | **Label the physical parts**, then fill `board_sn` / `motor_sn` in J02's row | 5 min | — | 🔴 the header rule is that `.id` matches what is written on the board, and this hardware still carries A1-era markings |
| **2** | **Photograph the motor label** — KV360 or KV380? | 2 min | — | 🔴 **§24.** One `Kt` cross-check is either +0.38% or +5.9% depending on the answer |
| **2b** | **Read the ESC1 silkscreen version** — `火柴 FOC V1.0` or V2.0? | 2 min | — | 🔴 **§24.7.** The listing schematic is a V1.0 page and it is now the sole source for `CAN_SHD` = PC11, `Temp_ADC` = PB14 and the HSE routing |
| **2c** | **Confirm the SIT1042 `S`-pin polarity and internal pull-up** in the datasheet | 5 min | — | 🔴 **on the S1c critical path.** LOW = Normal is solid; the fail-safe pull-up is ~75% recall (§23.3) |
| 3 | Archive both belt-on captures + J02's to `docs/cal/`, **each with a plant-state header** | 15 min | — | 🔴 unrecoverable after B0, and the belt-on rows are actively misleading without the header |
| **4** | **Idler rebuild: bearings + plate revision + slot one hole** (§22.2) | 2 h | bearings in transit | 🔴 **THE BINDING TASK.** Measure the bearings with calipers before committing the plate |
| 5 | **Design + print the output-pulley clamp** | 1 h | — | 🟢 **prerequisite for B6a/B6b**, and far easier now than with legs on |
| 6 | **CAD check: is the belt tensionable with the leg links OFF?** | 5 min | — | 🟢 five minutes that protects four measurements (§22) |
| **7** | **Belt on. §22, B0–B13** | ~2 h | tasks 4, 5, 6 | ⏸ |
| **S0** | **CAN: ESP32 alone, NO_ACK, analyzer. Measure 10 bits = 10.00 µs** | 45 min | — | 🟢 **NEXT BENCH TASK.** Pre-flight already passed; sample rate now correct |
| **S1** | CAN: ESP32 ↔ ESP32, TEC/REC = 0 over 60 s | 30 min | S0 | 🟢 |
| **S1b** | CAN: ESC1 transceiver probe — rail, mode polarity, PB9/PA11 | 30 min | S0 | 🟢 |
| **F1** | **Two facts for S1c: HSE crystal present? Max `fdcan_ker_ck`?** | 20 min | — | 🟡 **method improved 2026-08-14: poll `HSERDY` in firmware, do not look at the board** (§23.5). Schematic now says the HSE pins are routed while LSE is no-connect, so a crystal is likely. Still gates all STM32 CAN work |
| S1c | CAN: FDCAN external loopback | — | S1b + F1 | ⏸ |
| S2 | CAN: ESP32 ↔ ESC1 echo | — | S1, S1c | ⏸ |
| 🔴 | **12-board rework: one 120 Ω resistor out on 10 of 12 boards** | — | before any >2-node bus | 🔴 §23.2. Plus the 30-second CANH→GND / CANL→GND check to find out whether it is one resistor or two |
| ⏸ | **Bit-rate / SYSCLK coupling decision** | — | F1 | ⏸ **may re-open the 1 Mbit freeze.** Only on evidence, explicitly logged (§23.5) |
| ⏸ | Extract `mt6816.h` / `actuator_hw.h` / `safety.h` — 45 min, not the 2-hour refactor | 45 min | after B11 | ⏸ before the first line of Tier-0 |
| ⏸ | M2 / `i_scale` | 30 min | — | ⏸ instrument path resolved at zero cost (ZTW890D on DCA + RC3563 as voltmeter). **Confirm ZTW890D ownership first.** Locked-rotor and belt-agnostic, so it does not expire at B0 |
| ⏸ | The 478 mm apex mass question | 20 min | — | ⏸ **before the controller energy budget** (§17) |
| ⏸ | CAN message spec + RL observation/action vector | — | bit-rate decision | ⏸ freeze together (§23.6) |

> **One flag, then dropped.** M2 has been deferred across four sessions while
> instrument research continued around it. The instrument question is closed —
> **the remaining cost is one bench measurement**, and it is 30 minutes that does not
> expire. Nothing further to research.

### The two-joint fleet picture

| | J01 | J02 | Spread |
|---|---|---|---|
| `R_eff` | 0.22108 Ω | 0.22387 Ω | **+1.26%** — a real board difference, see below |
| `Ke` | 0.017750 | 0.017761 | **+0.06%** |
| `L` | 43.31 µH | 45.39 µH | +4.8% |
| `U0` | 0.01026 V | 0.01466 V | +43% — the weak parameter, as always |
| Breakaway | 0.2923 A | 0.2983 A | **0.19σ** |
| `T/T_loop` | 0.945, 0.974 | 0.937, 0.961 | fleet 0.958 ± 0.015, n=7 |

**`R_eff` moved 1.26% while `Ke` moved 0.06%, and both scale with the firmware's voltage belief.** A scale error would have moved them *together*. It did not — so the `R` difference is a **real** board/assembly difference (FET `R_ds(on)`, shunt, solder joints, wire length), not an artefact. **This is the first time the fleet table has been able to separate those two explanations**, and it is the argument for keeping `R_eff` per-unit.

**The `L` = 65 µH scare is closed.** Same motor, same board: 65 µH under the old 5-point phase-locked fit, **45.39 µH** under the 18-point dithered one. It was a fit artefact, not a different motor, and `L` is a near-fleet quantity at ~44 µH. Do not quote it better than ±2% — an offline refit of the `LSB` block gives τ = 206.3 µs against the firmware's 202.8, the fit band edge moving by one bin.

### Why the 45-minute extraction is not the 2-hour refactor

`open_test.cpp` is a **bench harness, and it will be replaced rather than shipped** — `fleet_config.h` says so itself ("a test harness owns them; they are not shipped to the robot"). §9's no-logic-in-main rule was written for the robot firmware. Three reasons to leave it until after J02:

1. **The rig currently produces correct measurements**, and J02's characterisation runs on it unchanged. *One variable at a time* applies to code as much as to hardware — restructuring a working instrument mid-campaign is the standard way to lose a week to a bug that reads as a hardware fault.
2. **RAM is at 73.9% with ~8.5 kB of headroom.** Splitting into translation units shifts static allocation in ways that would have to be re-verified.
3. **The Tier-0 deliverable is not a refactored `open_test.cpp`.** It is new firmware implementing `{p_des, v_des, kp, kd, τ_ff}` over CAN — and the parts that *do* ship, `joint_cal.h` and `fleet_config.h`, are already extracted. AUTOCALIB stays in the harness permanently; it is needed for all twelve joints and every re-characterisation after.

**What is worth extracting, and only this:** `mt6816.h` (SPI bit-bang, parity, `No_Mag`), `actuator_hw.h` (driver/sense/motor construction, the hard-won **init order**, boot Vbus read, `runInitFOC()`), and `safety.h` (`stopMotor()`, guard thresholds, the single disable path). The third is the one that matters — §9 says *"single safety path; new e-stop call sites need design review"*, and if the harness and Tier-0 each grow their own copy, that rule is broken before Tier-0 compiles. **Timing: after J02 is characterised, before the first line of Tier-0.** Not now, because J02 runs on known-good code; not later, because by then the duplication exists and the extraction becomes a merge.

### The three real risks — none of them is a precision question

1. **The performance envelope may be stale.** §17's apex figure and every force in §8.2 carry, respectively, an unresolved mass question and an unmeasured `i_scale`. **Both are documentation, not measurement**, and both are larger than any precision question left on the bench.
2. **The belt-off baselines expire on first belt fitment.** Drag map, breakaway, `J_rotor`, INL — all captured, none archived. **Get them into git before touching a tensioner.**
3. **`open_test.cpp` is 1179 lines holding all logic and all wiring**, against §9 of the working context. Fine as a bench harness; a problem at Tier 1. Promotion condition already written (§8.3).

**Deferred, with promotion conditions** — see §8.3 for the full table.
`DRIVER_VOLT_LIMIT` 6.0 → ~V_bus, **before the first commanded velocity above 150 rad/s** · `open_test.cpp` structure, **before CAN / Tier-1 integration** · M2 / `i_scale`, **before M14, before the sim actuator model, and before any force number is called validated** · live Vbus, **≥10 A bench currents or cross-session constant comparison** · ABZ fault on A1, un-blocking, whenever.

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
| **24.1** | **Motor nameplate: KV360 or KV380?** `claude.md` / project memory record **KV380**; §1, §8.1 and §8.1a all compute against **KV360** | `Kt = 60/(2π·KV)` is **0.026526 at KV360 (+0.38% vs measured)** or **0.025132 at KV380 (+5.9%)**. `Kt` is *measured* as 1.5·`Ke`, so nothing downstream breaks — but the "+0.38% agreement" is quoted as a cross-check and is only true at 360 | **Photograph the label.** 2 min |
| **24.2** | **Target mass: 3.0 kg or 4.0 kg?** The working instructions say **3 kg**; §17 freezes **4.0 kg** and §19 sizes the pack as "≤700 g = 17.5% of 4 kg" | **Every "% of standing load" figure in the doc set divides by 9.81 N = 4.0 kg.** At 3.0 kg the divisor is 7.36 N and every transparency percentage rises by 33% — breakaway 12.8% → 17.1%, belt-on 12.1% → 16.1%. The *ratios* between friction terms are unaffected; the pass/fail bands in §22 are not | **Decide the number and state it once.** §17 is the place. 5 min, no bench |
| **24.3** | **Pack: 5S frozen or 6S live?** `claude.md` freezes **5S**; §17 and §19 treat **6S as re-opened** with a staged commissioning plan | §19's apex table spans 66–639 mm across pack choices, so this is the largest single lever on the headline capability figure that is still open | **Either re-freeze 5S and mark §19's 6S rows as an unexercised option, or unfreeze in `claude.md` with the bench evidence named** (HG5511D 60 V, a 22.5 V run, a 34.2 V divider) |
| **24.4** | **`Ke` for the no-idler run: 0.017952 or 0.018035?** Firmware fit vs offline refit of the same capture | 0.46% apart when they should agree. Neither is carried into `joint_cal.h`, so nothing depends on it — but an unexplained disagreement between the firmware's fit and an offline refit of its own data is a tripwire worth pulling now rather than during a run that matters | **Re-fit the archived CSV.** 5 min, no bench (§22.1) |
| **24.7** | **ESC1 board revision: V1.0 or V2.0?** §1 records "Matches FOC **V2.0**"; the vendor listing photographs a board silkscreened "火柴 FOC **V1.0**" | The listing's schematic page is now the only source for `CAN_SHD` = PC11, `Temp_ADC` = PB14, the 48 V divider factor and the HSE routing. If the physical board is V2.0, all four are sourced from a document that may not describe it | **Read the silkscreen** (2 min). If V2.0, request the V2.0 schematic before S1c |
| **24.8** | **PB14 thermistor: the vendor's transfer function has the wrong sign for our data.** Vendor: `V0` = 1.4 V at 25 °C, **+0.019 V/°C**. Ours: 1431 → 1267 counts *as the board warmed* | Both readings imply ~5–12 °C on the vendor's formula, and they move the wrong way. Either the clone inverts the divider leg, or the two readings are invalid — and a frozen-ADC artefact has already been caught on this peripheral once | **One deliberate warm-up run**: 1.5 A for two minutes, watching PB14 continuously (§16). Do not adopt the formula until then |
| **24.9** | **The vendor rates the board below the design operating point:** continuous < 10 A, instantaneous < 40 A, *"not recommended for high-current drive"* — against 30 A peak per motor | 30 A is 75% of the absolute instantaneous claim and 3× the continuous one, on twelve boards. A ~35 ms jump pulse is plausibly inside "instantaneous"; a sprint gait is not obviously inside "continuous" | **Instrument, do not reason.** M10 hot-vs-cold `R_eff` plus FET temperature after a realistic duty cycle, once the belt is on (§16) |
| **24.10** | ~~Shunt value: 20 mΩ or 3 mΩ?~~ | — | ✅ **RESOLVED 2026-08-14: 3 mΩ (`R003`), confirmed in the vendor's board photo.** The firmware constant was right and its stated justification was invented. A 3.3× shunt mismatch is excluded, so **M2 is not escalated** (§2) |
| **24.5** | **`i_scale` is still 1.0 on every joint** | Every N and N/A in the doc set carries an unquantified ±(0–6)% common-mode factor. Documented at §8.2, unchanged — listed here so it is visible from the front page rather than only in the master table | M2. **30 min, does not expire** |
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
| §23.3 | Transceiver *"unmarked, not confirmed as TCAN330"*; mode polarity unknown | **`SIT1042QTK/3` on 5 V with 3.3 V logic, `S` on PC11, LOW = Normal.** S1b reduced from 30 min to ~10 |
| §23.5 | Fact 1 was to be settled *"by looking at the board near pins 5 and 6"* | Replaced with an `HSERDY` poll plus an LED-and-stopwatch frequency check, and a note on why `MCO` on PA8 must not be used |