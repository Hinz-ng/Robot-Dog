# AGENTS.md

Quadruped actuator project. Read `docs/WORKING_MEMORY.md` first for changing work state,
`docs/TECHNICAL_REFERENCE.md` to locate any detail in its owning file/section, and `docs/CONSTANTS.md` before quoting
numbers. `project_context.md` holds the method and past failure modes.

## What this is

~4 kg, 12 DOF quadruped for sprint / jump / backflip. Active milestone and joint state
live only in `docs/WORKING_MEMORY.md`.

Three tiers: **Tier 0** per-joint ESC firmware (STM32G431 + SimpleFOC 2.3.1), owns
commutation, current control, joint safety envelope, presents a frozen CAN contract.
**Tier 1** ESP32-S3 master: CAN, IMU, safety arbitration. **Tier 2** gait / RL / logging.

Actuator contract: `{p_des, v_des, kp, kd, tau_ff}` -> `{p, v, tau}`. There is **no
cascaded velocity PID** in it; `kd*(v_des - v)` is a term in a torque computation.

New joints use a new motor by default. Assign project identity `motor_<joint number>`
(J05 -> `motor_5`, J06 -> `motor_6`), unless the user reports a replacement/reuse.
These are project labels, not manufacturer serials; preserve historical reuse records.

## Source of truth, in order

1. **Bench evidence** — measurements, captures, scope traces, debugger output.
2. **Code and datasheets actually in hand.**
3. **Project documentation: `docs/CONSTANTS.md` for numbers; `docs/WORKING_MEMORY.md` for work state.**
4. **General knowledge and web search — lowest.** Clone boards and this toolchain have
   repeatedly punished "should work" reasoning.

When these conflict, **say so explicitly** rather than silently picking one. When a
measurement contradicts an earlier conclusion, the measurement wins and the conclusion
gets rewritten with a note saying what changed.

**A number derived by rearranging your own settings is not a measurement.** Before
trusting any recorded constant, ask what independent quantity was observed to produce it.

## Frozen — reopen only with new bench data, and say what data changed the picture

SimpleFOC 2.3.1 exact; `ststm32@17.6.0`. Three-tier architecture, CAN contract as the
Tier-0 boundary. Five-bar leg, 80 mm proximal / 100 mm distal. Belt drive 9:1, 10 mm GT2.
6S pack (availability decision, 2026-10-31). MT6835; MT6816 is legacy. Servo ab/ad. ESP32-S3 master + telemetry-bridge ESP32. `dead_zone = 0.005`.

## Three homes for a constant

| | |
|---|---|
| `fleet_config.h` | Identical on every joint by construction. Pole pairs, encoder chip and bits, dq convention, PWM/dead-zone, `T_DELAY_S` (MT6835; `T_DELAY_PER_LOOP` is legacy), `J_ROTOR_KGM2`, `GEAR_RATIO`, `G_FOOT_PER_MOTOR_NM` |
| `joint_cal.h` | Per-unit, measured on one physical assembly. `zea`, `dir`, `zea_enc` (the chip they were measured through), `R_eff`, `U0`, `Ke`, `L`, `vbus_scale`, `i_scale`, drag x4, `breakaway_A` |
| `open_test.cpp` | This bench harness's tuning and safety envelope. Control gains, limits, capture settings. Not shipped to the robot |

Belongs in `fleet_config.h` only if "would this differ between two correctly built
joints?" is NO. "Yes, by ~2%" means per-unit.

## Code conventions

- Constants in config headers; no magic numbers in logic.
- No logic in `main.cpp` — wiring only. Exactly one file defines `setup()`/`loop()`.
- Layer boundaries one-directional; no upward includes.
- Flag unit risks (rad/deg, mm/m, V/A) at every module boundary.
- Count thresholds are written in physical units and converted to counts through
  `fleet_config.h` (`encCountsFromMotorMrad`, `encCountsFromBeltUm`).
- **Single safety path.** New e-stop call sites need design review.
- One logical change per commit.
- Prefer full file replacement over a diff when the change is substantial; copy the old
  file and apply changes rather than regenerating, to prevent silent regressions.
- Comment the **reason**, especially where a value was set by measurement.
- **Sweep comments for staleness whenever the value above them changes.**

### Standing rule after any edit touching a string literal

```
python -c "import pathlib; [print(p, i+1) for p in pathlib.Path('src').rglob('*.[ch]*') for i, l in enumerate(p.read_text(encoding='utf-8').splitlines()) if chr(92)*2 in l]"
```

Prints every line holding a doubled backslash; expect none. (`grep -rn '\\\\'` matches every
single escape in git-bash, so the doubled one is lost in the list.) Two escaping regressions
shipped from Python replacement text containing C escapes: `\\n` at `autocalib.h:457`
reached the console silently, and `\\"` in the phase-7 row emitter failed the build. Two
more were caught at build on 2026-10-13. Edit C string literals with a literal-text editor.

## Bench conventions

- **One variable per test.** Violations have produced uninterpretable results.
- **State predictions before the test, in numbers.** A prediction that survives is
  confirmation; one that fails is a finding. A test with no prediction is data collection.
- Check `m=` in telemetry before interpreting any run — a refused mode change is nearly silent.
- Note which config is flashed. The boot `CFG` banner exists for this.
- Record the armed state and mode **with** the capture, not beside it.
- **Log a real per-sample timestamp.** `t = k*dt_mean` assumes uniform sampling that the
  guard ADC read, the LED blink and `handleSerial` all violate.
- **Lock the rotor for current-loop tests.**
- **Fold against integrated position, not time,** for anything mechanical.
- Keep the pen mark across magnet / mount / shaft.
- Runs boot disabled; torque modes arm at zero; auto-stop and overspeed guards stay in.

## Response shapes

Explain engineering in plain language: lead with the decision and physical effect,
then the reason and the next action. Define necessary jargon on first use. For bench
work, separate the concept, chosen settings, exact commands and expected observations.
Default to fuller engineering explanations with useful detail. State the goal/physical
idea and the deliverable, then the decisions, procedure and expected observations.
Explain what each command/button and argument does, its units, which joints it affects,
and the required state; never omit setup, stop/recovery or recording steps for brevity.
For implementation reports, name and link the exact code files/modules added or changed,
explain their responsibilities and integration, and separately list documentation changes.
Report checks actually run, physical tests still pending, and measured facts versus
chosen settings/assumptions. Do not make the user open a file to learn the essential
next procedure. Use tables or numbered steps when they help; avoid repetition and fluff.

- **Debugging** — top hypothesis with confidence -> the single most decisive test -> what
  each outcome would mean. One test per iteration, not a battery.
- **Design** — constraints -> trade-off table -> recommendation -> concrete next step.
- **Code** — goal -> exact files and responsibilities -> integration -> documentation
  changes -> actual software checks and remaining hardware verification.
- **Tuning** — one parameter at a time, with the physical meaning and expected change.

Quantitative over qualitative. No padding. Short answers for short questions. When
analysis stops changing the decision, say so and name the measurement.

## Failure patterns — if one is about to happen, stop

- Reasoning from a datasheet or general model instead of the rig's own measurements.
- Retracting a correct claim on the strength of a circular one. **Check the provenance of
  the number you are deferring to.**
- Using a small-angle form without checking the terms in it are the same size.
- Planning a measurement campaign without searching the archive first.
- Building from a stale uploaded file — **state which base version you built from.**
- Changing a parameter without asking what else it does.
- Attributing a symptom to whatever changed most recently.
- Confident single-cause diagnosis when the evidence supports several. Rank hypotheses
  with the discriminating test named for each, and say plainly when a dataset **cannot**
  separate two of them.
- Letting a subsystem refinement block the milestone.
- Carrying a value over from the part it was set for to a replacement part (bit timing
  copied from the MT6816 gave 99.65% CRC errors on the MT6835). Re-derive it from the new
  part's datasheet.

## Everything earns its place

- Explain each task's purpose before its steps: what uncertainty it resolves, what
  concrete artifact it produces, and which next decision uses it. Distinguish a
  physical measurement from a mathematical calculation or software check.
- Prefer the smallest useful implementation and measurement set. Reuse accepted
  evidence; do not repeat unchanged e-stop/fault checks or expand verification just
  for reassurance. Re-test affected behavior when code, wiring, assembly or new
  evidence changes its assumptions. Keep unperformed checks explicit.
- Run focused checks for the change, then stop when the decision is established.
  Defer speculative features and dense sampling without a named consumer. Respect
  user-requested scope (including a full ROM survey); efficiency means adaptive
  boundary sampling, not silently narrowing that scope.

A test, calibration, analysis or doc update must do at least one of:

- (a) change a decision;
- (b) set a safety limit;
- (c) catch a catastrophic error class (signs, units, clamps);
- (d) set a sim/RL randomisation range, **once, at fleet level**.

If you cannot name which one before starting, don't do it. Not sloppy: lean.

- **Per joint: pass/fail QC gates, not characterisation.** Measure to catch a defect, not
  for precision; the brass-idler drag (+0.385 A) was caught by a gate. Characterise a
  property once, on J01, and only if it sets a sim range.
- **Stop when the pre-registered pass/fail is decided.** A side observation gets one line
  plus a promotion condition, not a subsection. O1–O4 should each have been one line.
- **Docs are batched at milestone close.** Per session, keep the raw log and update
  `docs/WORKING_MEMORY.md`; record milestone outcomes in `docs/CHANGELOG.md`.
- Commit only when the user explicitly asks; never push unless asked.
- **Plans lead with the decision** and contain only what gates the next step. No extra
  simulations or predictions that don't feed a gate.

Small effects are expected at this rig's resolution (1 encoder count = 0.011 mrad output,
~0.4 mN·m at kp 41). The question is never "is there an effect" but "does it change a decision".

## Writing docs — data and procedure, nothing else

A doc holds **data** (value, ±, date, plant/config) and **procedure** (what to do, in
order). Anything else is cut, or cut down to one line. Model: `docs/cal/BELT_OFF_BASELINE.md`.

- **Procedures stay complete.** Every step, key, wire, meter range and expected reading. Never
  drop a step to save words.
- **Record every physical quantity in physical units** (rad / mrad, µm of belt, rad/s, N·m, A).
  Encoder counts appear only as raw-log columns or with the conversion stated alongside, and
  gates are written in physical units, so a sensor or resolution change is a one-line edit.
  At 16 bits (since 2026-10-19): 1 count = 2π/65536 rad = 0.0055° at the motor = 0.0107 mrad at
  the output = 0.366 µm of belt; 1 pinion tooth = 5461.3 counts = 2 mm of belt. Counts quoted from
  before 2026-10-19 are 14-bit (4× each of these).
- **Gates are a value plus one line of reason.** For example: `V ≤ 8° elec; > 15° = wrong row
  or slipped magnet`. Put the derivation in one place and link to it.
- **A result is the number, not the story.** Leave out how it was reached, which
  predictions survived, and which hypotheses were ruled out on the way.
- **Superseded means deleted.** No strikethrough, no "RETRACTED" paragraphs, no closed rows
  kept "for history". Git and a one-line CHANGELOG entry are the history. (The "note saying
  what changed" from *Source of truth* is that one line.)
- **One home per fact.** If it is stated elsewhere, link to it.
- **Cut on sight:** reminders to commit, archive or log; session narrative; "why we
  stopped doing X"; editorial to-do lists inside docs; essay headings ("— and why this
  matters"); bold on every other phrase; general knowledge (primers, textbook background).
- **A warning stays** only if it prevents a failure that has really happened. Keep it to one
  line that names the failure. The full catalogue lives in `project_context.md` §7 and
  `FAILURE_MODES.md`.
- Prefer tables and lists for comparable data and ordered procedures. Remove duplication;
  add detail where it clarifies the physical concept, command meaning or complete procedure.

## Milestone discipline

Binding milestone is the single-actuator platform. Parallel work is legitimate when the
binding constraint is external (prints curing, parts shipping) and drift when it is not.
Two patterns to flag **once** and then drop: purchasing distraction, and the analysis loop.

Gains, calibrations and drag maps are **per-plant** — mounting the leg invalidates them.
`Kt`, `Ke`, `R`, `L`, `J_rotor` survive.

Keep a "deferred with reason" list, not an open-questions list. Every entry names the
condition that promotes it back, **preferably physical rather than temporal.**

**Retire closed items loudly** — delete from the open list in the same edit that records
the result.

## ChatGPT project sources

- Keep every Markdown file under `docs/`, recursively excluding `docs/cal/`, in the
  ChatGPT **Robot Dog** project's Sources as durable reference material. The repository
  owns the current text; uploaded copies are snapshots for chats without local access.
- At the end of a session that changes these docs, and at milestone close, refresh the
  changed sources and add newly eligible files. Verify their names and uploaded content;
  retain one current copy per file and stay within the project's file limit.
- Keep these sources between sessions. Replace a source only as part of a verified
  refresh; list obsolete or empty-source deletion candidates before removing them.
- Compare versions before replacement. If an uploaded source contains newer evidence
  than its local counterpart, flag the conflict and preserve both until resolved.
- If Sources access is unavailable, report the pending files and do not claim they were
  uploaded. Instructions alone do not schedule or perform background synchronization.
- ChatGPT chats should consult these sources first, state the source version when
  freshness matters, and flag missing references into the excluded `docs/cal/` folder.

## Documentation ownership

- Use `docs/TECHNICAL_REFERENCE.md` as the central router for every detail lookup; follow its
  direct section links rather than reading the entire doc set or guessing a legacy section.
  When a topic moves or a file is added, update the route and check its links. Read the
  unresolved-conflict register in `docs/WORKING_MEMORY.md` before using affected guidance;
  preserve both claims and their provenance until the user/evidence resolves them.
- Codex, Work and Chat read `docs/WORKING_MEMORY.md` when starting/resuming work. Update it
  at session end with changed state, evidence paths/date/build, next gate and blockers;
  remove closed tasks. Keep volatile boards, assembly state and deferred work there only.
- Always consult `docs/CONSTANTS.md` before quoting a numerical constant. When accepted
  evidence changes a value, update its dated plant/config/units/uncertainty there and link
  from other docs. Keep configuration ownership in the three source files above; a doc edit
  does not authorize changing firmware. Flag source/measurement conflicts instead of choosing silently.
- Keep stable design, dated measurements and complete procedures in their subject files.
  `docs/TECHNICAL_REFERENCE.md` is an index; root `README.md` is the GitHub overview;
  `docs/CHANGELOG.md` records milestones, not current tasks or a session diary.

## End-of-session report

At the end of each session, give the user a concise, self-contained summary: what was done
and why; the relevant changed facts/values and their plant/config/provenance; files changed
with links; checks actually run and their results; unresolved conflicts, limitations and
pending work. State commit and Sources-refresh status. Separate measured facts from assumptions
and unperformed checks, and make any user decision needed explicit. Include enough detail for
the user to judge whether the work matches the request; do not rely on collapsed progress updates.
