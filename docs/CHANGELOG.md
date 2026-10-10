# Changelog — dated sessions, newest first

One entry per session: what closed, which values changed (old → new), where the detail lives.
Detail and data live in the section docs; this file is the index of changes.

*Hub: [`README.md`](README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 2026-10-10 (bench) — J01 idle current; logic-supply heat closed

- J01 idle 59.6 mA (3S) / 39.3 mA (6S), ST-LINK USB unplugged +4.8 mA → board is constant-power; J01 not degraded
  (+1.7 mA at 12.28 V). Hot corner = 78L05 on the 9.5 V gate rail, ~0.22 W, design property; no heatsink (HARDWARE §16a).

## 2026-10-10 — docs

- Engineering hub moved `README.md` → `docs/README.md` (section numbers unchanged); root `README.md` is now the public overview.
- `.gitignore` `cal*` → `/docs/cal/*` (README and BELT_OFF_BASELINE excepted): raw cal data stays local, `docs/CALIBRATION.md` now tracked.

## 2026-10-03 — b1 PASS; B12 CLOSED

- b1 (BELT_DRIVE §22.7.16): clamped output, kp 41, overshoot 35 → 9.5% (ζ 0.31 → 0.60) for kd 0 → 0.365. Output stiffness
  = kp·K/(kp+K), 80–89% of commanded (belt = series spring, softening: secant 142–182 vs small-signal 280–335 N·m/rad).
- Sim/RL ranges: belt mode 55–72 Hz, K_belt 140–335 N·m/rad output, ζ 0.20–0.45; demonstrated gain box kp ≤ 41, kd ≤ 0.365.
- kd quantisation blip = kd × 0.039 N·m. O3 closed. Deferred: kp/kd stability limit (do it on the leg).

## 2026-10-02 (e) — a5 PASS; B12a CLOSED

- a5 (§22.7.15): saturation on a moving motor clean, both clamps. Measured Iq overshoots the clamped command by up to 6%
  → future envelopes sit ≥ 6% below demonstrated-safe current.

## 2026-10-02 (d) — a4 PASS on its aim

- a4 (§22.7.13): stop time vs linear model kp 41 +1.6%, kp 16.8 −9.8%, kp 6.6 +21% (gate ±15% missed, explained by O4).
- O4: rotor rests in one 14/turn detent phase (~218 counts mod 1170, ±70 stick band). R29/R30 withdrawn on that basis.
- "+" direction = CCW viewed facing the output-pulley shaft end.

## 2026-10-02 (c) — a3 PASS; D19 promoted

- a3 part 2 (§22.7.11): driven speed +1.123 / −1.095 rad/s vs drag-map prediction +1.120 / −1.096.
- D19: 14/turn ripple 44.4 / 48.2 mN·m output, locked to absolute rotor angle, conservative, speed-independent.
- O2 logged: friction 0.068–0.116 N·m with output angle (explains ±20% drag-map scatter). J_rotor 20.2e-6 consistent.

## 2026-10-02 (b) — `m b` redesigned

- `open_test.cpp`: B is now staged changes on top of A, resolved at `m go`; `!!` warning on vd with kd = 0 / pd with kp = 0.
  Cause: a3 part 2's first attempt commanded zero torque (§22.7.10).

## 2026-10-02 — prefetch diagnosis withdrawn; a2 loop rate passes; a3 part 1 PASS

- `prefetch=1` had no effect on loop rate (+0.3%); the gain came from the MIT trim (`micros()` → DWT). MIT armed −4.7% vs
  TORQUE(I) (gate 5%). `FLASH_PREFETCH` stays true. Deferred: flash wait states 8 → 4.
- a3 part 1 (§22.7.9): damper slope −0.0998, R² 0.998.

## 2026-10-01 (e) — MIT timing on the cycle counter

- `FLASH_PREFETCH = true`; banner prints `prefetch=` / `flash_ws=`. MIT timing on `DWT->CYCCNT`; `mitEstUpdate()` takes `dt`.

## 2026-10-01 (d) — B12a a0–a2 recorded

- a0: Tf_mit = 1.0 ms. a1 PASS 22/22 (J01). a2 PASS except loop rate −8.2% (fixed 10-02). O1 (roll-back after stop) logged.

## 2026-10-01 (c) — MIT line fix

- A line now ends on CR, LF, `;`, or a whole-line burst followed by 250 ms idle. Arming no longer clears staged B.

## 2026-10-01 (b) — B12a contract code (C1–C8)

- New `src/mit_types.h` (output-side SI, no includes) and `src/mit_law.h` (estimator, law, validator).
  Estimator reads raw counts and applies `sensor_direction` once (`shaft_angle` already carries it).
- `open_test.cpp`: `MODE_MIT`, `m` commands, MIT capture/status, `m test`. `MIT_TAU_MAX_Nm` 0.39, `MIT_ENV_A_rep` 1.6.
- `strtof`/`strtok` replaced (they linked ~13.5 KB of newlib). Current cap and `VOLT_LIMIT` deliberately not raised for B12a.

## 2026-10-01 — units decided (option A); torque boundary

- τ = N × Kt × I_true at the output, η excluded; converts only at `torqueOutToIrep()` / `irepToTorqueOut()`.
  Clamp chain `tauOutCmdToIq()` clamps τ_max then the A_rep envelope last. Unit suffixes `_A_rep` / `_V` on all limits.
- Fixed: ladder `k_beltline` read 3.9% low (used plain `calKt()`). Stage 3b bench-accepted on J01.
- Superseded: Kt 0.0266 as a stored constant (now `calKt()` per row); k 62.4/64.1/65.2 → ×1.0394 (true k mean 67.2 kN/m).

## 2026-09-30 (b) — recipe-B drag; B11 complete; phase 4 belt-off only

- J01 drag_c 0.291 → **0.3169 / 0.3330 A** (fwd/rev, pooled over run order); drag_v 2.4e-3 → 3.24e-3 / 2.66e-3.
- Run-order effect: second direction reads 0.10–0.13 A higher (D18). Phase 4 belt-on ratchets the self-lock (R25):
  phases 1 and 3 only are belt-agnostic. Every pulley before J01-P12B, incl. J02's, was on the 0.6 profile.

## 2026-09-30 — pulley recipe B adopted

- Recipe A had been sliced on the 0.6 mm nozzle profile. Recipe B (0.4 profile): G 48.8 → **5.8 counts** (pass, gate
  exception closed), f_d 65.0 Hz, breakaway 0.288 → **0.295 A**. A recipe is identified by the md5 of its G-code (R24).
- R21 (backlash from −0.12 geometry) withdrawn. Clamp = 2 × M3×10 countersunk. §22.5 superseded by §22.6.

## 2026-09-28 (b) — J01 −0.12 locked; B11 written

- Clamp moves stiffness (k 57.6 → 51.5 kN/m), not backlash. Idler holes fixed at (1.72, ±10.00) for every joint.
- 3-position ladder method reversed (R17): ladders at one clamp setting, ±7-count noise.
- M6a mode is `t`, not `c` (CALIBRATION corrected).

## 2026-09-28 — J01 pulley −0.12 recorded

- drag_c 0.414 → 0.291 A at unchanged stiffness; B6b f_d 64.3 Hz. Procedures corrected to match firmware
  (B6b in `t` mode, 4 × `k` + 4 × `K`; B6a = `w` ladder; M4 warns on travel > 200 counts and > 0.60 A).
- `w` banner now prints the currents from `AC_SW_I`.

## 2026-09-05 — h1 localised to the output side; ring test retired

- Belt swap: h1 unchanged (−5.1%, p 0.42), h2 +490% (p 0.0002) → h1 is pulley eccentricity or output bearing; h2 is
  belt-borne (fleet screen: unexplained 2/rev → swap the belt first).
- Swing 835 counts confirms the exact belt-path arithmetic (230.96 mm). I_skip > 2.0 A on this plant.
- Ring f_d = 86.9 − 0.522·A (amplitude-dependent, softening spring): no single stiffness number. f_n 59–72 Hz →
  Tier-2 impedance ceiling 20–29 Hz. `EA ≈ 4 kN` withdrawn.
- Firmware: `K` kicks negative; captures carry `cap=N`, re-dumps print `!! RE-DUMP`.
- Boot-to-boot scatter ~8× within-boot (§12).

## 2026-09-02 — idler rebuild unblocked; take-up budget 2.00 → 1.04 mm

- Exact belt-length closed form: no-idler path 229.98 → **230.96 mm**, take-up budget 2.00 → **1.04 mm** (the approximate
  formula was used at (r₂−r₁)/C = 0.687).
- Belt variance n = 10: σ 0.158 mm, range 0.60 mm. Bearings OD 8.99 × 4.96; conical M3 washers. Idler: fixed M3 holes in
  both plates. Withdrawn, do not re-raise: slotted idler, spring cartridge, M2/M2.5 grub screws, tape on idler OD, 230 mm belt.
- J02 output-rate drag modulation 0.065 A at 1/output-rev (§9b).

## 2026-08-21 → 08-30 — live-Vbus detour CLOSED

- DMA vs seed disagreed by ~60 counts because SimpleFOC never calibrates the ADC (`CALFACT = 0`); `analogRead()` does.
  Pre-calibration at boot impossible: `ADC12SEL = 0`, no kernel clock. Scaffolding deleted (371 lines, −3.9 KB).
- `VBUS_LIVE` stays false; seed validated vs UT89X at 12.24 / 12.25 / 22.73 V to < 0.16%.
- Board facts (§3): `CALFACT` is additive (1 unit = 1 count) and clock-dependent; current sense runs on the regular group
  (no injected group); PA0 is ADC1 rank 5 already in the DMA buffer.
- R_eff reads ~2% low on the 0.68 V ladder top: curvature, not the ADC (§8.1d). Positive current headroom is 45.2 A.

## 2026-08-20 — M2 closed on both boards

- `i_scale` J01 **0.9621 ±1.18%**, J02 **0.9690 ±2.69%** (provisional). True current 3–4% above reported.
- M2 uses self-fit R from the ladder. `AC_R_V[0]` 0.46 → 0.68 V; `AC_M2_V[]` 5 → 8 points, dwell 20 → 12 s.
  M2 prediction table now burden-corrected.
- Seed boot outlier is a real per-boot offset → session-start banner-vs-meter check (±0.03 V, else reboot).
- Breakaway J01 7.87 → 8.18 mN·m, J02 8.10 → 8.36 mN·m; cogging 1.42 → 1.48 mN·m.

## 2026-08-18 — M1 measured per board

- `vbus_scale` 0.008358 → J01 **0.008448**, J02 **0.008516** (0.80% apart; original error was the DT9205A meter, +1.11%).
  R_eff, U0, Ke, L rescaled ×1.0108 / ×1.0189. τ_e, current-loop gains, every ampere unchanged.
- Kt vs KV360 is circular (Ke derives from vbus_scale); corrected agreement +1.02% / +1.11%, KV360 verdict stands.
- J03–J12 placeholder rows zeroed (`0.0f` = not measured). AUTOCALIB write-in gate 1% → 0.5%.

## 2026-08-15 — CAN tests A, C, D passed; CAN work ends

- Results in CAN_BRINGUP §23.10. `TEC` frozen at 128 = silence; `BUS_OFF` = someone driving the bus.
- Tier-0 constraints now four plus an ordered init checklist (§23.6). Termination rework: all 12 boards, deferred until
  the first bus with > 2 nodes.
- `tools/can_bringup/` mirrors the sketches. `.gitignore` `docs*` found (doc set was untracked) and fixed.
- B10 cannot be motor-driven (0.48 vs 3.4–5.7 N·m); static lever + spring gauge.

## 2026-08-14 (S2) — CAN ladder complete

- S2 PASS: 21,800+ frames, 0 errors. Termination 59.5 Ω unpowered (measure unpowered: powered bias networks swamp it).
- HSI16 +0.141% → crystal 7.9939 MHz. Loopback babbles: loopback is a power-on self-test only.

## 2026-08-14 (bench) — S1b + S1c passed; crystal identified

- HSE fitted, 8.000 MHz. 8 tq at NBRP 1 is forced. `TEST.RX` mux test 52.05% dominant. S1b: `S` LOW = Normal.
- Retracted: "external loopback proves the AF9 mux"; "a lone node goes bus-off after ~32 frames".
- PB9/PA11 have no pads; CANH/CANL do not decode on the FX2 at 5 V.

## 2026-08-14 (schematic) — vendor schematic and listing extracted

- Shunts are 3 mΩ (`R003`); firmware constant was right. Transceiver SIT1042QTK/3, `S` pin = PC11. PB14 = Temp_ADC,
  PB12 = SpeedBT_ADC confirmed. PB10 high moves `vbus_scale` ~1.9× — leave it alone. HARDWARE §2a added.

## 2026-08-13 — doc set split; belt-on diagnosed to the idlers; 1/rev closed

- Single README split into the doc set (section numbers kept).
- Belt-on J01: drag_c motor 0.0783 → + belt 0.2752 → + brass posts 0.6605 A; idlers = 0.385 A. Phase 6 and Ke are
  drag-contaminated → belt-off only.
- 1/rev on J02 belt-off 0.0262 A (A1's 0.185 A was the rubbing magnet); motor swap cancelled (§9a).
- ESC1 termination is a hard-wired ~121 Ω. CAN pins PB9/PA11. Current-loop gains: do-not-fix box (§10).

## 2026-08-12 — leg Jacobian; phases 3–6 no longer force re-alignment

- G = 87.7 is the stroke mean (α-average 40–70°); G swings 83.18–102.08, peaking at α = 63.44°.
- `acHaveCommutation()`: phase 3 accepts a stored ZEA; phase 5 requires commutation explicitly.

## 2026-08-08 — J02 characterised

- J02 row added; A1 historical. Belt-off transparency ~13% on both joints. T/T_loop 0.958 ± 0.015 (n = 7).
- `ac_zea_mismatch` blocks phase-7 carry when the flashed row is another joint's. M4 settle-into-detent instruction.

## 2026-08-08 — review round 3

- Fixed `\n` printed literally at `autocalib.h:457`. M4 travel check replaces the dead `el < 1000` guard; warning
  0.40 → 0.60 A, abort 0.60 → 0.80 A. `DRIVETRAIN_ETA = 0.92` labelled circular. A1 constants contaminated by magnet rub.

## 2026-08-08 — belt-off characterisation closed

- J_rotor 20.2 ± 2.4 × 10⁻⁶ kg·m² (three methods). J01 breakaway_A 0.2923 A. Cogging 1.4 mN·m (no table).
  `G_FOOT_PER_MOTOR_NM` = 87.7 /m.

## 2026-08-08 — review round 2

- `ac_guard` added (a keypress could clear a real guard trip). M4/M2 output lines delimited. Phase 7 prints the actual
  belt field. Free shaft needed by phases 2, 5, 6 only.

## 2026-08-08 — review round 1

- `DRIVER_VOLT_LIMIT` 6.0 caps phase voltage at 3.46 V (§8.3). New `N` (M2 assist) and `B`/`b` (M4). `[env:A1]` fails at
  build. `AC_L_DITHER_US` 2 → 3. Belt-on routine B0–B13 written.

## 2026-08-07 — AUTOCALIB rev2 re-run; JointCal schema v2

- J01 R_eff 0.22108 Ω, Ke 0.017750, L 43.31 µH (dithered fit; L 44.4–45.1 retired). U0 real uncertainty ±0.003 V.
- Schema v2: Kt removed (derived), `i_scale`, `breakaway_A`, 4-field drag map added. `src/fleet_config.h` created.

## 2026-08-07 — AUTOCALIB + per-joint storage

- 9.3% R_eff inter-assembly difference retracted (+1.8%). T ≈ one loop period. §20 and §21 created.

## 2026-08-06 — SPI migration; angle lag closed

- MT6816 SPI reference actuator; ZEA a stored constant. INL 0.93° mech pk-pk. Angle lag closed, no compensation.

## 2026-08-01 — bus sense / M1-d

- PA0 bus sense calibrated. HG5511D FETs 60 V / 40 A. Peak bus current 230 A. Battery spec §19 created.

## 2026-07-29 — characterisation session

- Kt 0.0266, Ke 0.0177 (Kt ≠ Ke in SimpleFOC's convention). `monitor_speed` 921600; SVPWM. Current loop 412 Hz / 9.3%.
  Robot-level design §17 created.
