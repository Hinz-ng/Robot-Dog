# active-memory handoff: Quadruped actuator — MT6835 migration, 16-bit read, J03 bring-up, belt-on baselines

**Handoff #1** · 2026-10-24 · Lineage: #1 (2026-10-24): N4 closed; MT6835 migration (J01, J03); 16-bit read; fixed-time control delay; phase-5 first-point abort; final assembly done, belt-on baselines next

## 0. Instructions for Claude (read first)

You are continuing work from a previous chat. That chat is gone; this file is the complete context and the source of truth.

1. Read this whole file before replying.
2. Follow sections 3 (Style), 4 (Hard rules) and 5 (Corrections) in every reply, for the rest of this chat. They override your defaults.
3. Use the values in section 8 exactly. Never round, re-estimate, or "correct" them.
4. Do not suggest anything listed in section 7 (Changed / rejected) again unless the user brings it up.
5. Code word: None active. If the user types /amcodeword, start every reply with the phrase they choose (default "Yes Boss!").
6. Your first reply: at most 5 lines covering the goal, the current state, and the next step (section 11). Mention any files from section 13 that were not attached. Ask the questions in section 12 if there are any. End with "Ready to continue with <next step>?" Then wait for the user's go.

## 1. Mission
- **Goal:** single-actuator bench platform for a ~4 kg, 12-DOF quadruped (sprint / jump / backflip). Binding milestone: MIT contract over CAN on two joints (CAN-T0: J01 + J03), then the hanging leg.
- **Done looks like (current stage):** J01 and J03 pass post-assembly checks and belt-on baselines (B1–B6b); belt-on rows stored in `src/joint_cal.h`; then N5/N6 close CAN-T0.
- **Context:** repo `C:\Users\ACER\Documents\PlatformIO\Projects\Quadruped Actuator`, branch `can-t0`. `CLAUDE.md` (repo root, gitignored, loaded automatically) holds the binding working rules; `docs/README.md` §15 holds status; `project_context.md` (gitignored) holds method and failure catalogue.

## 2. About the user (as relevant to this work)
- Git user "Hinz" (hinzng2005@gmail.com). Builds and runs the bench hardware: STM32G431 ESC boards (SimpleFOC 2.3.1), MT6835 encoders, ESP32-S3 master, PlatformIO on Windows, UT89X meter, bench pack ~12 V.
- Runs bench procedures from Claude's step lists and pastes raw console logs back as `.txt` attachments.
- Also consults a web Claude thread; pastes its review as `context.txt` and asks Claude Code to act on it.

## 3. Style & communication
- **Language:** English.
- **Tone:** direct, quantitative over qualitative, no padding. Short answers for short questions.
- **Reply length:** as needed for complete procedures; otherwise lean.
- **Formatting:** headings, tables, numbered procedures; predictions stated in numbers with pass lines; file refs as markdown links. No emojis in docs.
- **Working style (CLAUDE.md response shapes):** Debugging = top hypothesis with confidence → single most decisive test → what each outcome means. Design = constraints → trade-off table → recommendation → next step. Code = scope → code → integration notes → one hardware verification step. Tuning = one parameter at a time.
- **Bench procedures:** every step, key, wire, meter range and expected reading; predictions before the test; gates as value + one line of reason; say plainly when a dataset cannot separate hypotheses; when a measurement contradicts an earlier claim, retract it explicitly.
- **Avoid:** docs narrative; reminders to commit/archive in docs; characterisation that changes no decision; bringing up "step 5" (magnet-removal test).

## 4. Hard rules (word for word)
1. "One logical change per commit, don't push."
2. "Streamlining rule applies: everything earns its place." (CLAUDE.md: a test/calibration/analysis/doc update must (a) change a decision, (b) set a safety limit, (c) catch a catastrophic error class, or (d) set a sim/RL randomisation range, once, at fleet level.)
3. "do not bring it up again unless there is a good reason" (step 5, the magnet-removal test).
4. "defer documentation until after the change is done and tested."
5. "This session should aim to leave no loose ends up until the current point in the migration, report explicity what you did at the end." (close-out batches)
6. Standing rule after any edit touching a string literal — run and expect no output:
   `python -c "import pathlib; [print(p, i+1) for p in pathlib.Path('src').rglob('*.[ch]*') for i, l in enumerate(p.read_text(encoding='utf-8').splitlines()) if chr(92)*2 in l]"`
7. Docs: "data and procedure, nothing else"; "Record every physical quantity in physical units"; "Gates are a value plus one line of reason."; "Superseded means deleted."; "One home per fact."; "Docs are batched at milestone close."; "Retire closed items loudly — delete from the open list in the same edit that records the result."
8. "A number derived by rearranging your own settings is not a measurement."
9. "Single safety path. New e-stop call sites need design review."
10. Commit messages end with: `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`

## 5. Corrections log
| # | Claude did | The user wanted |
|---|---|---|
| 1 | 16-bit plan: implement C2–C5 only when both promotion parts were met | "correction - implement the whole plan (up to C5), regardless of the decision rule." |
| 2 | 16-bit plan draft: step 0 / C1 sequencing | C1 now regardless; step-0 window ≥ 30 s via an `E` transition count; a blip = τ step ≥ 5 mN·m with p within ±1 count |
| 3 | Link-health bench check written as pulling the MISO wire (FIRMWARE §11 steps 2–5 still say this) | No wire pulled: a console command that skips chip-select (forces all-zero reads; nothing cut) |
| 4 | Copied MT6816 SPI bit timing to the MT6835 (99.65% CRC errors once inlined) | Timing re-derived from the new part's datasheet (now a CLAUDE.md failure pattern) |
| 5 | Python edit scripts with C escapes broke string literals; bash heredocs with apostrophes failed | Edit C literals with the Edit tool; write Python edit scripts with the Write tool |
| 6 | Scratchpad cleanup deleted zig and the host tests | Host tests live in the repo (`test/host`); reinstall zig into the scratchpad when missing |

## 6. Decisions
| Decision | Why |
|---|---|
| Encoder MT6835 on every joint; MT6816 path deleted | migration closed 2026-10-13 |
| `ENC_BITS` 16 (since 2026-10-19; last 14-bit build `b63eca3`) | user's call; kd flicker 14 → 3.4–3.6 mN·m at kd 0.365 |
| Link health: trip on 4 invalid reads in a row; arm after 1 s with no invalid read | catches a 99.65%-failing link the old 200 ms guard missed |
| J03 `zea` 1.0739, `dir` +1 (belt off, 2026-10-24) | clean run; phase 6 ZEA residual 0.634° elec |
| `T_DELAY_S` 91.5 ± 7.5 µs (diagnostic only); sim latency range 84–99 µs | J01 84–93 µs + J03 95.6/98.5 µs, flat with loop rate |
| New gate: phase 6 ZEA residual ≤ 2° elec | a jammed bearing passed phase 2 78° elec off |
| One-turn gate 0.6981 ± 0.005 rad | hand alignment scatters ±0.003 rad |
| Phase-5 first-point abort: vel < 0.5 × (U − U0)/Ke_nameplate at 0.5 V, per direction | catches jam / bad ZEA / blocked output at ~2 A instead of 6 A |
| Belt-on baselines: skip B5 (INL), B7 (J_total), belt-on phases 4 and 6 | INL not stored and measured belt-off this week; phase 4 ratchets and phase 6 T is invalid belt-on |
| B10′ deferred | user's call; hard gate before loaded stance or raising the 1.6 A envelope; repeat one `w` ladder after it |
| Task 1 (fault injection + N4d) deferred | user's call; hard gate before the first armed run with links on |
| AUTOCALIB: paste only `zea`, `dir`, `zea_enc` after an encoder change | other fields are gates, not re-stored |

## 7. Changed / rejected
- One-turn gate ±0.002 → ±0.005 rad (tighter than hand alignment can resolve).
- Delay/INL bands 84–90 µs / 0.85–0.87° → 84–99 µs / J01 0.85–0.90°, J03 0.935° (more runs).
- `T_DELAY_PER_LOOP` 0.958 (MT6816, per loop) → `T_DELAY_S` 91.5 µs fixed time.
- Tier-0 disarmed lps prediction 41–44 k → measured ≈ 40.8 k.
- J03 belt-on `zea` 2.4326 → discarded (bearing jammed by PLA debris; 77.8° elec off).
- "J03 R_eff −7.2% = supply sag" → retracted (clean run R −0.11%; cause unresolved, did not recur).
- ❌ AUTOCALIB refusal override (phase 1 refusal is the link-safety interlock; phase 6 only computes on phase 5's data).
- ❌ Submerging the motor in water to clean grit (bearing rust/grease loss, magnet corrosion). Use air, tape, replace bearing if gritty.
- ❌ Pulling MISO for the link check → chip-select-skip console command.
- ❌ 21-bit read (finer than the CAN position step; counter wraps fast).
- ❌ MT6835 self-calibration now: deferred until INL > 1.5° mech pk-pk or leg tracking shows a 1×/2× rev component.
- ❌ Setting the fixed delay from J01 alone (superseded: J03 belt-off phase 6 became available).

## 8. Data & facts (exact)

**Resolution (16 bits):** 1 count = 2π/65536 rad = 0.0055° motor = 0.0107 mrad output = 0.366 µm belt; 1 pinion tooth = 5461.3 counts = 2 mm belt. Counts before 2026-10-19 are 14-bit (4×). Gear 9:1. One motor turn = 0.6981 rad output (40°).

**joint_cal.h rows (current):**
| Field | J01 (index 0, belt-on row, old recipe-B assembly) | J03 |
|---|---|---|
| zea, dir | 5.9879, +1 (MT6835 2026-10-13) | 1.0739, +1 (2026-10-24, belt off) |
| R_eff, U0 | 0.22346, 0.01037 | 0.23068, 0.01919 |
| Ke, L | 0.017941, 43.77e-6 | 0.017900, 45.75e-6 |
| vbus_scale, i_scale | 0.008448, 0.9621 | 0.008357, 0.9797 (PROVISIONAL) |
| drag_c fwd/rev | 0.3169 / 0.3330 | 0.0935 / 0.1058 (belt OFF) |
| drag_v fwd/rev | 3.24e-3 / 2.66e-3 | 0.001143 / 0.000713 |
| breakaway_A | 0.295 | 0.2545 (bare motor) |
| belt | "10mm-9:1" | "OFF" |
- J01 belt-off baseline row: index 13 (JOINT_ID 14), zea 6.0542 (MT6816), drag_c 0.0750 / 0.0816.
- UIDs: J01 `460030 34354B0F 30373336`; J03 `{ 0x52003E, 0x34354B0C, 0x33383735 }`.

**Recent measurements:**
- J01 16-bit (2026-10-19): Ke −0.74%, R_eff −0.34%, INL 0.897°, T 89.3/93.1 µs, V 2.10°, Tier 0 lps 40.8 k, one turn −0.6988.
- J03 clean (2026-10-24): R 0.23042, Ke 0.017889, L 43.3 µH, drag fwd 0.0893 + 0.001156·w, rev 0.1001 + 0.000786·w, INL 0.935° mech, ZEA residual 0.634° elec, T 95.6/98.5 µs at 12548 Hz, first point 25.53 rad/s at 0.1216 A, V 2.73°, Tier 0 lps 40.4–40.7 k, one CW turn −0.6984 rad.
- J01 after encoder refit (gap matched to J03): E 0 CRC in 1.23 M reads, span 65535 on the turn, V 2.42° → zea 5.9879 stands.

**Envelope:** inner 1.60 A_rep, outer 0.390 N·m; validator kp ≤ 41.0, kd ≤ 0.365. Tooth-skip threshold 3.4–5.7 N·m. kd blip = kd × 0.0098 N·m.

**Baseline gates (Claude-set, disclosed):**
| Step | Pass |
|---|---|
| V (B1) | ≤ 8° elec, direction matches |
| R_eff (B2, `1` then `3`) | J01 0.2168–0.2302 Ω; J03 0.2238–0.2376 Ω |
| Phase 5 first point | predicted ≥ 20 rad/s; abort floor ≈ 13.5 rad/s |
| drag_c pooled both orders (B3) | 0.25–0.45 A each direction; predicted J01 ≈ 0.32/0.33, J03 ≈ 0.33/0.35 |
| Breakaway (B4), 10 positions × `B`/`b` | each: no NO MOTION (0.80 A), travel ≤ 76.7 mrad, ≤ 0.60 A; mean 0.20–0.45 A |
| Backlash G (B6a, `w` ×2) | ≤ 29 µm belt pass; 29–59 marginal; > 59 slack (J01 prev 8.5 µm) |
| Ring (B6b, 4 × `k` + 4 × `K`) | J01 vs J03 within 15% (J01 prev f_d 65.0 Hz) → no-tensioner design holds, MIT gain box carries to J03 |

**Commits this chat (branch `can-t0`, none pushed):** `f03cc74` ENC_BITS 16 · `473ab9b` 16-bit docs · `502b0f3` 16-bit accepted · `36481d2` J03 belt-on zea (superseded) · `c0a324f` J03 belt-off zea · `8a3c187` T_DELAY_S · `0f18ce1` J03 docs + ZEA-residual gate · `28415eb` J03 closed, J01 refit · `86e7df5` phase-5 first-point abort.

## 9. People, terms & names
- **People:** Hinz (user); "web Claude" (separate review thread, source of `context.txt`).
- **Terms:** Tier 0 = per-joint STM32 firmware (`-e T0_J01`, `-e T0_J03`); harness = bench firmware (`-e J01`, `-e J03`); AUTOCALIB phases 1 LINK, 2 ALIGN, 3 R/U0, 4 L, 5 FREE-SPIN, 6 T/INL, 7 report; ZEA = zero electrical angle; G = backlash; B1–B13 belt steps (BELT_DRIVE §22); M1–M15 manual cal; N0–N6 CAN-T0 ladder; "step 5" = magnet-removal test; task 1 = fault injection + N4d; 7l = homing CAD check.
- **Harness keys:** `e`/`E` encoder test/monitor · `V` verify ZEA · `Y`, `1`–`7` AUTOCALIB · `-` then `5` within 0.8 s = phase 5 reverse-first · `B`/`b` breakaway · `w` swing ladder · `t g k K x d` ring capture · `m` MIT · `N` M2 ladder.
- **Tools:** pio = `~/.platformio/penv/Scripts/pio.exe` (not on PATH); host tests `python test/host/run.py` with `pip install --target <scratchpad>/zig ziglang` on PYTHONPATH; `docs/cal/*` is gitignored → `git add -f`; `CLAUDE.md`, `project_context.md` gitignored (edit locally, never committed). Env A1 fails to build on purpose.

## 10. Work state
| Item | Status | Location | Notes |
|---|---|---|---|
| MT6835 migration, 16-bit read | final | `src/mt6835.h`, `src/fleet_config.h` | — |
| J03 encoder acceptance | final | `src/joint_cal.h` row J03 | — |
| Phase-5 first-point abort | built, not yet bench-run | `src/autocalib.h` | first belt-on B3 run must not false-trip |
| Final assembly J01 + J03 | done (user) | — | — |
| Post-assembly checks + baselines | not started | procedure in section 11 | — |
| Belt-on rows (B11) | not started | `src/joint_cal.h` | J01 index 0 replaced; J03 gets a belt-on row, its belt-off values move to a baseline row (like J01 index 13) |
| Encoder link check steps 2–5 (task 1) | deferred | FIRMWARE §11 | doc still describes a MISO pull |
| README ENC row | open: J01 link check steps 2–5 only | `docs/README.md` §15 | — |

## 11. Next steps
1. **Next action:** user runs post-assembly checks on J01, then J03 (harness), and sends the logs:
   - Power off: hand-turn output a full revolution each way (smooth, no catch); idler rollers coast ≥ 1 s, no axial walk; pen marks across pinion/shaft and output pulley/hub.
   - Banner `MT6835 … crc_err=0 status=0 (weak=0 uv=0 ovs=0)`; `Vbus=` vs meter at pads ±0.03 V; `e` → ENC PASS; `V` ≤ 8° elec (> 8° → stop, send the line).
2. Baselines, J01 then J03, same sitting:
   - B2+B3 (output free, one boot): `Y`, `1`, `3`; `5`; `-` then `5`. Ignore belt-on Ke.
   - B4 (output free): 10 positions (5 bell marks + midpoints): turn, let go, wait 2 s, `B`; wait 2 s, `b`.
   - B6a (output clamped: 2 × M3×10 countersunk into the top plate, perpendicular): `V`, `w` ×2.
   - B6b (clamped, same boot): `t`, `g`, `k`, wait `CAPTURE done`, `x`, `d`; 4 × `k` + 4 × `K`.
   - User sends per joint: banner, `e`, `V`, R line, both phase-5 runs, 20 breakaway lines, both SW ladders, 8 ring dumps (joint + boot noted).
3. Claude: pool drag over both orders, friction-correct G (§22.4.4), fit rings, J01-vs-J03 comparison, store B11 rows (one commit per joint), docs batch.
4. N5 + N6 → CAN-T0 closes, §23.11 doc batch.
5. Before the first armed run with links on: task 1 (chip-select-skip fault injection + N4d). Before loaded stance / raising 1.6 A: B10′ then one `w` ladder.
6. Desk (parallel): 7l homing CAD check; ZERO jig; five-bar kinematics in the master; mounting signs; confirm J03's boot pulse matches J01's then commit `skip_align`.

## 12. Open questions ⚠️
- Track `CLAUDE.md` in git? (Currently gitignored, exists on one disk; user has not decided.)
- Task 1: env `T0_J01_FAULT` (and `T0_J01_HANG`) already build — does it already implement the chip-select skip, or must the console command be written? FIRMWARE §11 steps 2–5 must be rewritten to match either way.
- N4d: README records hang/HardFault → IWDG reset passing on 2026-10-10; web Claude listed it as unreported. Re-run it with task 1, or accept the 2026-10-10 record?
- Were the J03 logs saved to `docs/cal/J03/` (`J03_2026-10-24_autocalib (bare motor, MT6835).csv`, `… (belt on, jammed bearing).csv`) and J03's belt teeth inspected? Not confirmed.

## 13. Re-attach checklist
- [ ] Nothing required: rules, status and data are in the repo (`CLAUDE.md`, `docs/`, `src/joint_cal.h`) and this file.
- [ ] Optional: the latest web-Claude `context.txt`, if the user wants its task board re-checked.

---
<sub>Audit: 13/13 sections · 10 rules · 6 corrections · 60+ data points · secrets removed: none found · generated by active-memory</sub>
