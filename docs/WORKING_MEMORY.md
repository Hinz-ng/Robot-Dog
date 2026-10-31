# Working memory — Robot Dog

Updated: 2026-10-31. Shared short- and medium-term working context for Codex, Work and Chat.
Read this before planning or resuming work. This is a mutable snapshot, not a bench log.

## Read and update

- Active work, assembly/firmware state, blockers, unresolved decisions and deferred items live here.
- At session end, replace changed state and record evidence paths/date/build; remove completed
  tasks. Keep dated results/procedures in their subject files and milestone records in
  [CHANGELOG.md](CHANGELOG.md). Do not grow a session diary here.
- Read [CONSTANTS.md](CONSTANTS.md) before quoting numbers. When accepted evidence changes a
  documented constant, update that file and its provenance; update owning firmware config only
  when explicitly in scope. Link to the value here rather than copying it.
- Find any technical detail through [TECHNICAL_REFERENCE.md](TECHNICAL_REFERENCE.md). Review
  the conflicts below before applying affected guidance; an imported status is not a present-rig check.

## Snapshot and provenance conflicts

Binding milestone: single-actuator platform; the imported next gate is CAN-T0 on J01 + J03,
then the hanging leg. The work order below is carried from the local docs, not revalidated today.

| Source | Recorded state | Use |
|---|---|---|
| Local checkout inspected 2026-10-31 | `main`, `d78a55b`; `src/fleet_config.h` configures MT6816, 14 bits; `joint_cal.h` dates J01 belt-on row 2026-09-30 and J03 belt-off row 2026-10-08 | Software snapshot; does not identify flashed firmware or present assembly |
| Imported technical README, status through 2026-10-10 | J01 belt off since 2026-10-10; CAN-T0 N0–N4 accepted; J03 belt-off baseline complete | Historical work-status snapshot |
| `docs/cal/Belt-On Baseline Procedure.md` | MT6835 banner and post-assembly checks; updated reference docs target MT6835 / 16-bit; historical records retain original resolution | Forward standard confirmed: MT6835 / 16-bit; source is can-t0 21cec6c, now merged into main. Confirm physical plant before applying gates |

The GitHub-facing root README is an overview, not the technical reference or live work board.
Historical instructions in handoffs/exports do not override current AGENTS.md or the user.

### Imported assembly state (CONSTANTS §8; not a present-rig check)

| Name | Motor / board | Status |
|---|---|---|
| A1 | `M-ABZ-01` on `B-ABZ-01`, ABZ encoder | historical: rubbing encoder magnet contaminated every friction number (`drag_c` 1.05 A, "4.5 N / 46%"). Not used; voltage-derived numbers 1.89% low |
| **J01** (= A2) | `B-SPI-01`, MT6816 SPI | reference actuator, belt on (recipe B) |
| **J02** (= rebuilt A1) | `M-ABZ-01` on `B-ABZ-01`, MT6816 SPI | belt on (SCB pulley). ESC board failed after P8; its MT6816 board found dead 2026-10-08 (MISO stuck low) and replaced, so its stored `zea`/`dir` belong to the old encoder |
| J03 | board_3 + motor_2 | belt-off baseline complete (2026-10-10) |

## 15. Work board

| # | Task | Blocked by | Status |
|---|---|---|---|
| **CAN-T0** | MIT contract over CAN, ESP32 ↔ J01 + J03 | confirm branch/config and assembly | N5/N6 pending in the imported 2026-10-10 board; dated N0–N4 results: [CAN_BRINGUP §23.11](CAN_BRINGUP.md#2311-can-t0-acceptance--2026-10-0410) |
| 7l | Output position: homing or a known boot pose (MT6835 is absolute over one motor turn = 40° of output) | — | before the contract freezes and before the leg. Then set per-joint P travel limits (now ±2π) |
| B10 | Tooth-skip threshold (static lever + spring gauge) | — | before any command above 1.6 A and before the leg; raises the envelope (≥ 6% below demonstrated-safe current) |
| — | CAN message spec + RL observation/action vector: frame IDs, packing and scaling, poll schedule | — | freeze together (§23.6) |
| S1e | Measure the real control frame width | message spec frozen | |
| 7d | J02: replacement ESC board (and encoder board, replaced) → recipe-B pulley → re-acceptance → B4 + B6b → J01-vs-J02 15% comparison | hardware | J02 ESC board failed after P8 (D12, D15) |
| 7e | Belt-length histogram (20 belts, §22.3 protocol) → pulley-compensation bins | — | desk work; plus one short-belt joint on recipe B (D4) |
| 7j | Record the recipe-B G-code md5 (`certutil -hashfile <file>.gcode MD5`) | — | 2 min |
| 7m | Backfill the stage-3b acceptance log with the UT89X reading and auto-stop line | — | 2 min |
| 1 | Label J02's physical parts, fill `board_sn` / `motor_sn` | — | 5 min |
| 2b | Read the ESC1 silkscreen version (V1.0 or V2.0) — §24.7 | — | 2 min |
| M1b | One sitting: banner vs UT89X on J01 and J02 back to back; the J01↔J02 `Ke` gap should come back at 0.87% (CONSTANTS §8.1b) | J02 board | low priority |
| M2b | Re-run M2 on J02 | — | only if something needs sub-1% torque accuracy |
| — | Remove R22 from the ten non-endpoint joints (§23.2); retain two endpoints | before adding a third enabled terminator | |
| — | Apex mass/sag question | — | Before the controller energy budget; see ROBOT_DESIGN §17 |


## 24. Unresolved documentation decisions

Docs-only audit: 2026-10-31, all 16 Markdown files under `docs/`, including the three calibration
guides. The comparisons below do not establish present hardware state. Keep disputed originals
until the user confirms the intended truth or inspected evidence resolves them; update the
owner and dependent claims together, then remove the resolved row.

### Accepted decisions and source applicability

User decisions, 2026-10-31: MT6835 for all forward work (MT6816 legacy); 6S frozen by
availability; ±0.03 V voltage-scale check; retain R22 on two endpoint joints and remove the
other ten; regulator-feed inference is high-confidence and the practical thermal conclusion
is accepted. Owners: SENSING, ROBOT_DESIGN, CALIBRATION, CAN_BRINGUP and HARDWARE.

Code inspected read-only: pre-merge main at `d78a55b` was legacy MT6816. Merged `can-t0` at
`21cec6c` contains MT6835, 16-bit reads, chip-tagged alignment and fixed delay 91.5 µs.
can-t0 was merged into main on 2026-11-01. Main now contains the MT6835 implementation; no hardware was operated.

- Keys verified in both branches: f loads usable calibration, otherwise aligns; F always
  aligns; V measures then restores. MT6835 usability also checks zea_enc.
- acP6 computes only, requires completed phases 4/5 (and link phase 1 on can-t0). The supported
  sequence is belt-off because phase 4 is belt-off. No standalone belt-on 5 → 6 procedure.
- M2: pad voltage in both power and delivered-voltage fits; no second burden subtraction.
  Store a valid positive gain with uncertainty; no arbitrary 0.95–0.98 gate or automatic
  sense-constructor correction. Existing calibration rows were not rewritten.
- Motor/board/assembly ownership is defined in CALIBRATION; per-row storage and shared PI
  tuning are separate policies. L remains a per-unit estimate.
- Current-loop PI gains remain measured settings. The 500 Hz ceiling claim is removed;
  1/(3T) is arithmetic, not a validated full-loop bandwidth limit. acP7's console still prints
  the heuristic as a ceiling; correct that firmware diagnostic only in a separately scoped
  code edit, preserving the measured response.
- CAN first usable communication and full TEC recovery are distinguished; no inference of
  automatic re-arming. Ring f_n/3 arithmetic corrected to about 20–24 Hz.
- J02 reference belt-force scale corrected to 7.334 N/A_rep using Kt/i_scale; original dated
  tables remain historical. Conditional backlash estimates must name friction equivalence;
  long-shaft J01 cannot inherit the short-pulley G verdict.

User approved documentation, branch merge and Sources refresh on 2026-11-01.


### Evidence gaps and design gates (not contradictory facts)

| ID | Item | Resolution / promotion condition |
|---|---|---|
| 24.6 | Efficiency η = 0.92 is explicitly circular in [CONSTANTS §8.2](CONSTANTS.md#82-mechanical--drivetrain); force calculations use it as an assumption | M14, load cell on an assembled leg; distinguish modeled force from measured force. |
| 24.7 | [HARDWARE §1/2a](HARDWARE.md#2a-vendor-documentation-火柴--hfoc-listing-rank-4--narrows-unknowns-never-overrules-a-measurement) leaves actual ESC1 silkscreen unconfirmed; vendor schematic/listing is V1.0 | Read the physical revision; obtain its schematic if different. The old imported claim that §1 says V2.0 is stale. |
| 24.8 | [HARDWARE §16](HARDWARE.md#16-thermal-ground-rules): vendor PB14 +0.019 V/°C, earlier cross-session analog readings fell, continuous DMA reading rose | One continuous 1.5 A / 2 min warm-up on the DMA path if thermal sensing is promoted; do not compare unlike read paths as one calibration. |
| 24.9 | Vendor continuous < 10 A / instantaneous < 40 A versus modeled 30 A peak per motor ([HARDWARE §16](HARDWARE.md#16-thermal-ground-rules)) | Duty cycle / duration and measured copper/FET temperature gate viability; peak versus continuous ratings are not themselves a contradiction. |
| — | Jump apex / sag / effective mass are already labeled unresolved in [ROBOT_DESIGN §17](ROBOT_DESIGN.md#17-robot-level-design-decisions-frozen-2026-07-29); pack resistances are estimates in §19 | Reconcile the model assumptions before controller energy sizing. No measured apex is established. |
| — | D19 asks per-joint ripple characterization; fleet-QC policy favors defect gates; belt-on E2 compares J01/J03 within 15% while the older routine specifies J01/J02 | Confirm which assembly comparison changes the next decision, and whether ripple is fleet characterization or a per-joint pass/fail screen. |

Not a docs-internal conflict: target mass is consistently 4.0 kg in this doc set; the imported
3-vs-4 kg instruction claim is outside this audit. Dated 0.93° versus 1.029° INL, recipe A versus
B, and changing loop rates belong to different sessions/configurations; do not average them or
delete the original evidence just to make values match.

### Documentation structure follow-up

File roles complement each other, but BELT_DRIVE mixes the executable routine with long dated
acceptance tables, and several files use bare cross-file section numbers. The router now bypasses
that search cost with direct links. Further splitting is deferred: it would add Sources files
without resolving the disputed applicability. Keep raw logs and complete procedures intact.

## 22.8 Deferred belt/control work

Imported from BELT_DRIVE; gates retain their original plant/config context. In particular,
legacy count-based gates are not transferable to a new encoder or pulley. Reconcile their
physical units and plant before use; D19's per-joint characterization request also needs review
against the fleet-QC rule before promotion.

| # | Item | Promote when |
|---|---|---|
| **D19** | **Active.** 14/turn ripple (44–48 mN·m output): characterise per joint (amplitude and phase vs electrical angle) including an **at-rest quasi-static torque–angle sweep** (p_des ramp at kp ~41 over ≥ 1170 counts, both directions → washboard shape and static friction); evaluate a Tier-0 feed-forward. Contract-freeze question: does Tier 0 cancel it, and does τ include it? | Per joint at bring-up |
| D1 | Tier-2 impedance ceiling from the ring | Before any CONTROL §10 ceiling update; needs a second joint and > 1 boot |
| D2 | B6a ladder repeat in a later session (recipe B G = 5.8 holds?) | Next J01 session. If G > 40: mid-batch belt or compensation change (holes are fixed) |
| D3 | Skip threshold on the tensioned plant (B10) | Before any command above 1.6 A (11 N belt force) and before leg attachment |
| D4 | Fleet compensation bins: confirm the 20-belt batch maximum ≤ 105.0; one joint on a short belt (≤ 104.7) on recipe B, drag_c in band and G ≤ 20 | 20-belt batch measurement / joint 3 |
| D5 | Fleet pulley repeatability: two prints of the same md5 (tip OD, M-B); re-baseline the fingerprint on recipe B | Before printing any fleet pulley |
| D6 | Fleet CAD route (§22.4.10) | Only if the slicer-setting coupling is to be removed |
| D7 | Belt-difference swing, 105.0 vs 104.7, predicted 410 counts | Before selective assembly relies on the caliper jig |
| D8 | Cord EA two-weight test (1.475 / 5 kg with a steel-wire splay reference) | Only if tension in newtons becomes load-bearing |
| D9 | J02 B4 breakaway on its tensioned plant | With J02's replacement board |
| D10 | Reverse drag fit rms 10–14 mA | Inspect the drag map for per-idler or per-output-rev terms |
| D11 | "1/rev disturbance gone" (hand feel only) | Confirm h1 in the next drag map |
| D12 | J02 reprint on recipe B (boss from recipe-B behaviour) and re-acceptance | When J02's board is replaced; before D15 |
| D13 | Chalk witness test for tooth contact | Only if a future pulley fails seating |
| D14 | Stepped vs ramped loading, no-idler ratchet | Only if the no-idler swing is used as a QC step again |
| D15 | B6b J01-vs-J02 15% comparison (4 + 4 in `t` mode, plus B4) | When J02 has a working board and recipe-B pulley |
| D16 | Cogging vs position-dependent preload in B4's ±0.18 A | If the sim friction model needs the ripple's period/phase (dense angle sweep) |
| D17 | Control-side backlash tolerance test (kp steps in position hold, watch for limit cycle, ~10 min) | If any fleet joint fails G ≤ 29 µm belt; J01 long-shaft (G unreadable): first hanging-leg block, no oscillation ≥ 1 mrad p-p output lasting > 1 s at any kp step up to 41 |
| D18 | Drag run-order effect (+0.10–0.13 A second direction) | If a drag feed-forward needs better than ±20%. Test: one direction twice with a rest vs back-to-back |
| O1/O2 | Roll-back at zero torque; friction vs output angle | When a friction feed-forward is proposed, or rest errors exceed bounds |
| — | Flash wait states 8 → 4 (BELT_DRIVE §22.7.7). Hypothesis: branch-target misses at 8 wait states; prefetch hides only sequential misses. Test `FLASH_LATENCY_4` (in spec at 170 MHz); predicted TORQUE(I) armed lps ≥ +5% | Tier-0 loop with CAN measured below its need, or a loop-rate gap fails a gate |
| — | kp/kd to a stability limit | Gait/RL needs kp > 41 or kd > 0.365; run it **on the leg** |
| — | b2/b3 inertia bar | Leg rings or misses the √(1 + J_L/J_m) bandwidth prediction |
| — | Back-EMF feed-forward in the current loop (−5% plateau sag) | b1 or the leg shows a torque shortfall in fast moves |
| — | Shim the top-plate clearance (wear debris, not drag) | Next teardown |
| — | Does the M4 screw axially preload the pinion bearing? Define a shim stack or screw depth | Next teardown; decides whether pinion drag is fleet or per-unit |
| — | Recipe B G-code md5 | Next pulley print |

## 8.3 Deferred electrical work

| Item | Size | Promotes when |
|---|---|---|
| J_total | — | M6b on a built actuator |
| Force per amp | Jacobian calculation: [CONSTANTS §8.2](CONSTANTS.md#82-mechanical--drivetrain) | needs the real leg (M14) to validate η |
| Sense gain mismatch (14/rev) | ~9% | if impedance force ripple above ~1 N matters |
| Genuine-board acceptance run | — | completes the EG2124A evidence table |
| **`DRIVER_VOLT_LIMIT` 6.0 → ~V_bus** | phase voltage capped at **3.46 V** (Uq_max = 0.5774 × 6.0); rig tops out near 190 rad/s unloaded | **before the first commanded velocity above 150 rad/s.** Raising it invalidates only `U0` and the low-side current-sense window (`R_eff` and `Ke` cancel). Afterwards re-run phase 3 and check phase 1 and the phase-5 \|I\| ratio |
| Live Vbus (`VBUS_LIVE`) | robot sag 3–7 V; at 10 A, 2.4% on R/U0/Ke | PA0 is already ADC1 rank 5 in the DMA buffer, but that converter is uncalibrated (`CALFACT = 0`, reads ~60 counts low). Promote when bench currents exceed ~10 A, the first multi-actuator bus, or constants must be compared across sessions. **Acceptance: `Vdma` vs a terminal meter at two voltages ≥ 5 V apart** (not seed vs DMA) |
| `CALFACT` noise floor | never measured | before comparing two `CALFACT` values. Cost: 5 power cycles |
| PB14 thermistor | — | when a joint runs unattended. Hot/cold `R_eff` is M10 (phase 3 before/after 60 s of load) |
| U₀ feed-forward | J01: 0.078 A ≈ 3% of standing load at 5S | when a board measures `U0` > 0.03 V. `dead_zone = 0.005` stays |

## 18. Firmware work / known gaps

Imported from FIRMWARE; inspect the selected source/build before treating a gap as still open.

- `logStats()` skips the first 25% of the buffer — right for steady sweeps, misleading after a `k`.
- `VBUS_LIVE = false` (§8.3).
- Bus voltage is not in the burst log. If added (`uint16_t vbus_x100`), drop `LOG_N` 1000 → 900 and
  read the free-RAM figure first (a static buffer colliding with the stack HardFaults). Only worth
  it once sampling is live.
- The single disable path is shareable (`stopMotor()` in `safety.h`) but not single: bare
  `motor.disable()` sites remain in `autocalib.h`. Closing it is a behaviour change needing design
  review.

## Other promotion gates

| Item | Evidence / promotion condition |
|---|---|
| 6S commissioning clearance | Imported ROBOT_DESIGN §19 snapshot: FETs 60 V (datasheet); regulator, 3.3 V rail, bulk capacitance (ran at 22.5 V); ADC range (34.2 V). Not cleared: electrolytic capacitor markings unreadable; switching at 25.2 V under load. Procedure remains in that file. 6S is selected; complete commissioning independently of that choice. |
| Idle-current recheck | At milestone close, ST-LINK USB plugged and pad voltage measured: use the dated reference and ≤ 61 mA at 12.28 V gate in HARDWARE §16a. |
| Logic-supply thermal model | Revisit when the electronics bay is sealed/unventilated or a board resets hot; measure the 78L05 input and K36 node on the robot pack (HARDWARE §16a). |
| 478 mm apex | Resolve spreadsheet mass and sag assumptions before using the controller energy budget (ROBOT_DESIGN §17). |
| Encoder reference docs | Forward docs target MT6835 / 16-bit, verified against can-t0 21cec6c. Source integration into main completed 2026-11-01. |
| CAN joint power cycling | Test VCC ramping on the transceiver, rather than reset only, when power distribution can cycle one joint independently (CAN_BRINGUP §23.10). |

## Documentation refresh

2026-11-01: reviewed documentation committed; can-t0 merged into main. J01 and T0_J01 builds
and documentation link checks passed; host test runner stalled without results. No hardware
was operated. The closing response and external upload manifest record Sources verification;
future documentation-changing sessions must refresh changed Sources and verify their content.
docs/cal remains excluded from durable uploads.
