# Quadruped Actuator — technical reference router

Use this page to find any project detail. Follow the question below to the owning section;
read its plant, date, units and firmware context before applying it. Section numbers are legacy
identifiers, not a reading order. This page owns navigation, not measurements or work status.

## Start or resume

1. Read [WORKING_MEMORY](WORKING_MEMORY.md) for assembly/build state, next gate and blockers.
2. Check [unresolved documentation conflicts](WORKING_MEMORY.md#24-unresolved-documentation-decisions)
   before using an affected command, gate or design assumption. No date alone settles a conflict.
3. Use the routes below. Consult [CONSTANTS](CONSTANTS.md) before quoting a reference value;
   follow its provenance to the subject evidence. Independent bench evidence and inspected
   source outrank a documentation summary when they disagree; flag the disagreement.

## Find a detail

| Question / search terms | Go directly to | Companion when needed |
|---|---|---|
| What is next, blocked, deferred or not yet decided? | [WORKING_MEMORY: work board](WORKING_MEMORY.md#15-work-board) | [Deferred belt/control](WORKING_MEMORY.md#228-deferred-beltcontrol-work), [electrical](WORKING_MEMORY.md#83-deferred-electrical-work), [firmware gaps](WORKING_MEMORY.md#18-firmware-work--known-gaps) |
| Which joint, encoder and build does this apply to? | [WORKING_MEMORY: provenance](WORKING_MEMORY.md#snapshot-and-provenance-conflicts) | [Conflicting instructions and gates](WORKING_MEMORY.md#24-unresolved-documentation-decisions) |
| What value, unit, uncertainty or calibration should I use? | [CONSTANTS: calibration reference](CONSTANTS.md#8-calibration-reference) | [Mechanical reference](CONSTANTS.md#82-mechanical--drivetrain), [bench settings](CONSTANTS.md#85-bench-tuning-reference) |
| What evidence localizes friction, cogging or output-rate drag ripple? | [CONSTANTS: drivetrain health](CONSTANTS.md#9-drivetrain-health) | [Current-sense harmonics](SENSING.md#7-current-sense-calibration-do-not-fix-again), [MIT ripple results](BELT_DRIVE.md#22711-a3-part-2--driven-speed-pass-d19-promoted) |
| What hardware, board revision, pins or supply rails are involved? | [HARDWARE: components](HARDWARE.md#1-hardware) | [Clone differences](HARDWARE.md#2-clone-vs-genuine-measured), [pin map and ADC](HARDWARE.md#3-board-pin-truths) |
| What heats up; how do I check idle load or winding heating? | [HARDWARE: thermal](HARDWARE.md#16-thermal-ground-rules) | [Logic supply and idle current](HARDWARE.md#16a-logic-supply-and-idle-current-j01-2026-10-10) |
| How do I build, flash, connect the console or start a session? | [FIRMWARE: build contract](FIRMWARE.md#4-firmware-contract) | [Power-up workflow and keys](FIRMWARE.md#11-session-workflow-and-telemetry); confirm the physical joint first |
| What do MIT commands, telemetry and capture columns mean? | [FIRMWARE: MIT console](FIRMWARE.md#mit-mode--typed-lines-not-keys) | [Telemetry](FIRMWARE.md#telemetry-line), [burst logger](FIRMWARE.md#burst-logger), [MIT test evidence](BELT_DRIVE.md#227-b12--mit-law-j01--closed-2026-10-03) |
| How do encoder wiring, ZEA, alignment, current sense or INL work? | [SENSING: encoder](SENSING.md#5-encoder--mt6835-4-wire-spi) | [Alignment](SENSING.md#6-alignment-zea--a-stored-constant), [current sense and INL](SENSING.md#7-current-sense-calibration-do-not-fix-again) |
| How do I baseline a bare motor / new joint? | [Belt-off command sheet](cal/BELT_OFF_BASELINE.md) | [Calibration methods](CALIBRATION.md#20-manual-calibration--what-autocalib-does-not-measure); command sheet targets MT6835; use the merged MT6835 firmware base |
| How do M1/M2/M4, voltage/current scale or calibration storage work? | [CALIBRATION: manual methods](CALIBRATION.md#20-manual-calibration--what-autocalib-does-not-measure) | [Row storage and invalidation](CALIBRATION.md#21-per-joint-calibration-storage), [reported vs true amps / torque](CONSTANTS.md#81c--i_scale-m2-2026-08-20) |
| Where are raw logs, required headers and pre-correction data rules? | [Calibration record index](cal/cal_README.md) | Follow the joint/plant paths there; raw logs remain unedited |
| How do I assemble and accept the belt drive, B0–B13? | [BELT_DRIVE: routine and QC](BELT_DRIVE.md#22-belt-on-routine) | [Belt-on command sheet](cal/Belt-On%20Baseline%20Procedure.md) targets MT6835; confirm the physical pulley/plant and gates first |
| How do I print the pulley or build idlers and set belt geometry? | [BELT_DRIVE: recipe B](BELT_DRIVE.md#2261-recipe-of-record--b) | [Idler specification](BELT_DRIVE.md#222-idler-specification), [belt length/path/tension](BELT_DRIVE.md#223-belt-geometry-and-tension) |
| How are backlash, breakaway, stiffness, ring tests or tooth skip assessed? | [BELT_DRIVE: swing readout](BELT_DRIVE.md#2244-swing-ladder--the-b6a-readout) | [Recipe-B dated results](BELT_DRIVE.md#226-j01-recipe-b--pulley-of-record-2026-09-30), [B12 gain envelope](BELT_DRIVE.md#22716-b1--clamped-output-gain-envelope-pass-b12-closed); B10 is in the routine |
| What current-loop gains, bandwidth or impedance limits are supported? | [CONTROL: measured response and limits](CONTROL.md#10-control) | [Settings](CONSTANTS.md#85-bench-tuning-reference), [per-unit policy](CONTROL.md#per-unit-calibration-policy), [belt-mode evidence](BELT_DRIVE.md#22716-b1--clamped-output-gain-envelope-pass-b12-closed) |
| Why does the rig fail, and which diagnostic comes first? | [FAILURE_MODES: catalogue](FAILURE_MODES.md#12-failure-mode-catalogue) | [Diagnostic ladder](FAILURE_MODES.md#13-diagnostic-ladder), then the relevant subsystem above |
| How do I wire CAN, terminate it, initialize FDCAN or interpret errors? | [CAN_BRINGUP: Tier-0 checklist](CAN_BRINGUP.md#236-tier-0-can-constraints-and-init-checklist) | [Termination](CAN_BRINGUP.md#232-termination-and-the-12-board-rework), [pin facts](CAN_BRINGUP.md#233-esc1-can-pin-facts), [wiring and error branches](CAN_BRINGUP.md#239-s2--esp32--esc1-on-real-wire) |
| Which CAN transport / CAN-T0 tests passed, and on what configuration? | [CAN_BRINGUP: transport results](CAN_BRINGUP.md#23-ladder--results) | [Robustness](CAN_BRINGUP.md#2310-robustness-tests-ad-2026-08-15), [CAN-T0 acceptance](CAN_BRINGUP.md#2311-can-t0-acceptance--2026-10-0410); pending work stays in WORKING_MEMORY |
| What leg geometry, mass, force, inertia or jump model is assumed? | [ROBOT_DESIGN: robot decisions](ROBOT_DESIGN.md#17-robot-level-design-decisions-frozen-2026-07-29) | [Mechanical constants and efficiency caveat](CONSTANTS.md#82-mechanical--drivetrain) |
| What pack, sag model, wiring or 6S commissioning procedure applies? | [ROBOT_DESIGN: battery](ROBOT_DESIGN.md#19-battery-specification) | [Resistance measurement](ROBOT_DESIGN.md#measurement-protocol-rc3563-four-wire-1-khz), [staged commissioning](ROBOT_DESIGN.md#6s-staged-commissioning); clearance is tracked in WORKING_MEMORY |
| What milestone changed a decision or accepted value? | [CHANGELOG](CHANGELOG.md) | Follow the dated entry to the subject evidence; historical values are not current defaults |

## File ownership

| Home | Owns | Keep elsewhere |
|---|---|---|
| This router | Questions, destinations, file boundaries | Values, task boards and duplicated procedures |
| [WORKING_MEMORY](WORKING_MEMORY.md) | Changing work state, next gates, blockers and unresolved conflicts | Settled evidence and milestone history |
| [CONSTANTS](CONSTANTS.md) | Accepted reusable numerical reference, units, uncertainty, plant/config and provenance | Full experimental tables and procedure-specific settings |
| HARDWARE / FIRMWARE / SENSING / CONTROL / CALIBRATION / BELT_DRIVE / CAN_BRINGUP / ROBOT_DESIGN | Their subsystem's design, complete procedures and dated evidence | Cross-project work status; copies of another file's reference table |
| [FAILURE_MODES](FAILURE_MODES.md) | Observed failure classes and diagnostic order | Open task queues |
| [cal/](cal/cal_README.md) | Executable calibration sheets and original records | A second master constant table |
| [CHANGELOG](CHANGELOG.md) | Milestone outcomes and accepted changes, linked to evidence | Current tasks and per-session diary |

When changing a fact, update its owner and any dependent claim; link instead of copying.
Keep measurement tables with their original plant/date, even when a newer reference supersedes
them. Add or repair the route when a topic moves. Avoid unqualified cross-file `§` references:
link the file and heading. Unresolved claims stay flagged until their evidence is reconciled.

**Legacy prefixes:** M = manual calibration; B = belt routine; S / A–D = CAN transport tests;
N = CAN-T0 acceptance; D / O = deferred items / observations. These are separate ladders.

**ChatGPT Sources:** `docs/cal/` is excluded from the durable uploads. If a linked guide or log
is unavailable to a chat, state the missing path and request the relevant content before
reconstructing a procedure or claiming to have checked its evidence.
