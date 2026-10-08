# Belt-off joint baseline: command sheet (J03 and every later joint)

**What this produces:** a complete, flashable `joint_cal.h` row for a new board + motor pairing, the board's Tier-0 ID, and the raw records in `docs/cal/Jnn/`.
**Time:** about 1.5 h. **Firmware:** the bench harness (`-e Jnn`), plus one Tier-0 boot at the end.
**Plant:** the **bare motor**, clamped so the stator cannot turn. No belt, **not in the assembly plate** (no pinion bearings), leg links nowhere near it.

Written for J03 (new board + J02's motor). For any later joint, replace `J03`/`3` with its number, and use the reference values of whichever motor it carries.

> **Why some numbers are stored and others are only checked.** The **board** is new, so its numbers are measured and stored: `vbus_scale`, `i_scale`, `U0`, `R_eff`, and the alignment `zea`/`dir`. The **motor** is J02's, already characterised, so its numbers (`Ke`, `L`, drag, breakaway) are **gates**: J03 must reproduce J02's values. A gate miss means a wiring, mounting or calibration error, not "a different motor".
>
> **The strongest single check in this sheet is `Ke`.** `Ke` belongs to the motor, but the firmware measures it through the board's voltage scale. If J03's `Ke` lands within 1% of J02's, M1 was done right. If not, M1 is wrong; stop and redo it.

---

## What you need

| Item | Notes |
|---|---|
| **UT89X** (the cross-checked meter) | The only meter trusted for M1 / M2. Record which meter you used in the header |
| **Bench pack, ~12 V** | Every step except M1's second point runs on this. All reference values were measured near 12 V (`U0` scales with bus voltage) |
| **Second supply ≥ 5 V higher**, 17–28 V | For M1's second point only: 5S pack or bench PSU. **Stay below 30 V** (plausibility window); the ADC clips at 34 V |
| M2 current measurement | One of options A/B/C in [`CALIBRATION.md`](../CALIBRATION.md) §20.1 (the M2 box). Option A (PSU with V and A display) is best |
| ST-Link + console | `pio device monitor -e J03 -p COMx` (921600). This sheet only needs **single-key** presses, which work in the monitor |
| Pen, tape | 5 marks on the motor bell at ~72° spacing + a fixed pointer, for M4 |
| Motor clamp | Vise or bracket on the motor **body**. Phase 5 spins the shaft at up to ~1,000 rpm |

**Magnet check before you start (M8):** gap 0.5–1.0 mm, never zero, centred.

## Keys used (bench harness, single keypresses)

| Key | Does | Motor moves? |
|---|---|---|
| `p` | Bus-voltage probe: prints `seed Vb=` to 4 decimals | no (refuses if enabled) |
| `e` | Encoder self-test, 2000 reads | no |
| `Y` | AUTOCALIB status: what has passed, what is next | no |
| `1` | Phase 1 LINK: sensor and current-sense link | no |
| `2` | Phase 2 ALIGN: `zea` + direction | small twitch |
| `3` | Phase 3 R/U0: locked rotor, ~6 s | holds still |
| `4` | Phase 4 L: locked step train. **Belt-off only** | holds still |
| `5` | Phase 5 SPIN: free spin both directions → `Ke`, drag | **spins fast** |
| `6` | Phase 6 T/INL: computation only | no |
| `7` | Phase 7 REPORT: prints the pasteable row + raw blocks | no |
| `V` | Verify the stored `zea` (one forced alignment) | small twitch |
| `N` | M2 bus-power ladder (8 locked points) | holds still |
| `B` / `b` | M4 breakaway ramp, forward / reverse | creeps then moves |
| `x` | Stop | — |
| any key | Aborts a running phase | — |

**One phase per key.** Wait for its `PASS`/`WARN`/`FAIL` line before pressing the next. A phase refuses to run if its prerequisite hasn't passed in this boot.

---

## Session header: copy into your log first

```
JOINT J03   date ____   board label ________   motor label ________ (J02's motor)
plant: bare motor, clamped, belt OFF, no plate bearings, pinion fitted? ___
meter: UT89X   pack ____   magnet gap ____ mm
```

---

## Step 0: prepare the row and labels (5 min, desk)

1. **Label the board** physically, e.g. `B-SPI-02`. The string you write in `board_sn` must match the label. The motor is J02's (`M-ABZ-01`, the old A1/J02 motor); confirm against its physical label.
2. In `src/joint_cal.h`, in the **J03** row only, set **`vbus_scale` to `0.0085f`** with the comment `// PROVISIONAL for M1 -- replace`.
   - Why: with `0.0f` the firmware ignores the bus reading, and M1 needs one.
   - **Do not skip this and compute M1 from the `p` probe's DMA buffer instead.** On J03 (2026-10-08) that path's slope was **0.66 % low**: its error is not offset-only. Only the seed (`seed Vb=`) is valid for M1.
   - `0.0085` is between J01 (0.008448) and J02 (0.008516); M1 replaces it.
3. Flash the harness: `pio run -e J03 -t upload`.

---

## Step 1: M1, the board's voltage scale (15 min)

**Point 1 (bench pack, ~12 V)**
1. Power the board from the pack. Open the monitor.
2. Meter on the **board's power pads** (`7V-48V` and `GND`), not the pack. Write down **V_meter1** to 0.01 V.
3. Press **`p`**. Write down **`seed Vb=`** (4 decimals) as **Vb1**.

**Point 2 (second supply, ≥ 5 V higher)**
4. Power down. Connect the second supply. Power up. The voltage is read **once at boot**, so a power cycle is required.
5. Meter at the board pads → **V_meter2**. Press **`p`** → **Vb2**.

**Compute** (or send the four numbers to Claude Code):

```
c1 = Vb1 / 0.0085        c2 = Vb2 / 0.0085          (seed in ADC counts)
vbus_scale = (V_meter2 - V_meter1) / (c2 - c1)       (the slope; discard the intercept)
check: s1 = V_meter1/c1 and s2 = V_meter2/c2 must agree within 0.3 %
```

6. Put `vbus_scale` in the J03 row, replacing the provisional value. Comment: `M1 <date>, UT89X, <V1> / <V2> V, slope`.
7. Flash `-e J03` again. Check the boot banner `CFG ... Vbus=` against the meter **at both voltages** (power-cycle between them).

| Gate | Pass | If not |
|---|---|---|
| s1 vs s2 | within **0.3 %** (J02's two-point check: 0.19 %) | Meter or contact problem. Re-measure; do not average a disagreement away |
| Banner vs meter after the fix | within **±0.03 V** at both voltages | Recompute. A boot offset of 0.05 V happens (README §24.14): **reboot and re-read before blaming the scale** |

**Go back to the bench pack (~12 V) now and stay there for the rest of the sheet.**

---

## Step 2: encoder link (1 min)

1. Press **`e`**.
2. Press **`E`** (live monitor) and **turn the shaft by hand through a full revolution**. Press any key to stop.

| Gate | Pass |
|---|---|
| Parity errors | **0 / 2000** |
| `no_mag` | **0** |
| **`raw` while turning** | **sweeps the whole 0–16383 range and follows the shaft.** Never stuck at one value |
| Read time | ≈ 6.65 µs (informative) |

> **⚠ Parity and `no_mag` alone are NOT a pass** (found on J03, 2026-10-08). If the chip never drives its data line (unpowered, HVPP not tied, CSN/SCK open), every read is all zeros. All zeros has even parity and a zero `no_mag` bit, so `e` reports a clean link while `raw` sits at **0** forever. Only the turning check catches it.

If it fails: on the encoder board, with power on and the firmware running, check:
- **VDD = 3.3 V** and **HVPP = 3.3 V** (both steady).
- **CSN ≈ 1.8–1.9 V** on a meter. The firmware reads continuously (low ~44 % of each loop), so a meter shows the average. Flat 3.3 V or 0 V means it isn't toggling.
- **SCK ≈ 2.5–3.0 V** average (2.89 V measured on J03's working line). Flat 3.3 V or 0 V means no clock is arriving.
- **If all four look right and `raw` is still stuck at 0, swap the encoder board.** On J03 (2026-10-08) the chip's MISO output was dead (J02 incident). The swap separates the encoder from the driver board in ~10 min, and the meter can't.

Then, with power off, check continuity PB5→CSN, `HA/A`(PB6)→MOSI, `HB/B`(PB7)→MISO, `HC/Z`(PB8)→SCK, and that MISO is not shorted to GND. Then the magnet gap/centring.

---

## Step 3: AUTOCALIB, bare motor (10 min)

**Session-start gate:** banner `Vbus=` vs meter within 0.03 V. If not, **reboot** first.

Press, waiting for each result line:

1. **`Y`**: status. Shows nothing passed yet.
2. **`1`**: LINK. Expect PASS.
3. **`2`**: ALIGN. Small twitch. Expect PASS with `zea=` and `dir=`. **Let go of the shaft.**
4. **`3`**: R/U0, about 6 s, rotor held still. Expect PASS with `R_eff` and `U0`.
5. **`4`**: L, step train. Expect PASS with `L` and τ.
6. **`5`**: SPIN, **both directions, fast**. Hands clear, motor clamped. Expect PASS with `Ke`, `Kt` and drag fwd/rev.
7. **`6`**: T/INL. Expect `T/T_loop` and the INL summary.
8. **`7`**: REPORT. Prints the **pasteable row** and the `BIN,` / `LSB,` / `DRAG,` blocks.

**Gates** (J02 = the same motor, so its row is the reference):

| Quantity | Stored or gate | J02 reference | **Pass band** | A miss means |
|---|---|---|---|---|
| `zea`, `dir` | stored | — | phase 2 PASS | — |
| `R_eff` | stored (board) | 0.2219 Ω on today's ladder (the 0.22810 in J02's row is an older, narrower ladder: chord effect, not a fault) | **0.211 – 0.233 Ω** (±5 %) | Phase-wire or solder-joint resistance, or a bad FET |
| `U0` | stored (board) | 0.0149 V | **0.005 – 0.030 V** (weak parameter, wide band) | — |
| **`Ke`** | **gate** (motor) | **0.018097** | **0.017916 – 0.018278** (±1 %) | **`vbus_scale` is wrong → redo M1.** Do not continue |
| `L` | gate (motor) | 46.25 µH | **41.6 – 50.9 µH** (±10 %) | Phase wiring, or current-sense gain far off |
| `drag_c` fwd / rev (dynamic Coulomb, phase 5) | gate (motor) | 0.106 / 0.107 A | **0.075 – 0.139 A** each (±30 %, session scatter is ±20 %) | A rubbing magnet or a dry/damaged motor bearing |
| `T/T_loop` | fleet check | 0.958 ± 0.015 | **0.913 – 1.003** | Loop-rate or sensor-timing problem; tell Claude Code |

**Archive:** save the whole phase-7 output (row + `BIN,` + `LSB,` + `DRAG,` blocks + the CFG banner), **verbatim**, as `docs/cal/J03/J03_<date>_autocalib (bare motor).csv`.

---

## Step 4: store the row and verify alignment (10 min)

1. Replace the **J03** row in `src/joint_cal.h` with the phase-7 row. Then fill in:
   - `board_sn` = the board label;
   - `motor_sn` = the motor label;
   - date;
   - belt `"OFF"`.

   `vbus_scale` is **carried** in the report from the flashed row: check it is your M1 value, not 0.0085. `i_scale` (1.0) and `breakaway_A` (0) are filled in by Steps 5–6.
2. Flash: `pio run -e J03 -t upload`. The boot banner should print `JOINT J03 board=<label> ...` with your numbers.
3. Press **`V`**.

| Gate | Pass | Warn | Fail |
|---|---|---|---|
| `V` | **≤ 8° elec**, direction matches | 8–15° (check the magnet mount) | > 15° or direction mismatch: wrong row or the magnet slipped |

4. **Commit:** `git add src/joint_cal.h` and commit with message `J03 belt-off row (AUTOCALIB, M1)`.

---

## Step 5: M2, the board's current-sense scale → `i_scale` (25 min)

**Procedure, instrumentation and fit are in [`CALIBRATION.md`](../CALIBRATION.md) §20.1, the M2 box.** Read that box once before starting; this is the key sequence.

1. Power up. **Gate: banner `Vbus=` vs meter ≤ 0.03 V**, otherwise reboot. This gate is worth 1.2 % of the result.
2. If you use options B/C, measure the ammeter's burden **`R_b`, 4-wire, unpowered** (1.547 Ω on the UT89X 600 mA range).
3. Press **`1`**, **`2`** (PASS), then **`3`** → write down **cold `R_eff`**.
4. Press **`N`**: the 8-point ladder. At **each** point:
   - let the meter settle;
   - write **Vbus** and **Ibus to 0.1 mA** next to that `M2,` line (point 1 too);
   - press any key. Each point auto-advances after 12 s.
5. Press **`3`** immediately → **hot `R_eff`**.
6. **Send the log + meter readings to Claude Code for the fit** (`g = 1.5·R/c`, with R fitted from the same run). `i_scale = g`.

| Gate (from §20.1) | Pass |
|---|---|
| `I_reported` vs the §20.1 table (burden column for B/C) | within **3 %** |
| Ibus span, point 1 → point 8 | **230 – 280 mA** |
| Hot `R_eff` vs cold | within **2 %** |
| `drift=` per point | 0. A point where the rotor moved is invalid |
| **`g`** | expected **0.95 – 0.98** (J01 0.9621, J02 0.9690) |

7. Put `i_scale` in the J03 row with its ± and date. Reflash `-e J03`. The banner line `i_scale=` should show it.

> **Fleet-value question, decided at milestone close, not here.** The proposal is: if J03 lands within ±1.5 % of 0.966, later G431 clone boards use a fleet value and skip M2. `joint_cal.h` warns against pooling on **two** boards (that move hid the 0.80 % `vbus_scale` difference). A third agreeing board is real evidence, but the decision is the owner's. Record J03's `g` and its ±; don't change any other row.

---

## Step 6: M4 belt-off breakaway, bare motor (10 min)

**Not the same as phase-5 drag.** Phase 5 measured **moving** friction (~0.1 A). M4 measures the current needed to **start** moving from rest, which includes cogging and stiction (~0.3 A). The references below are motor-alone, belt-off: **J01 0.2923 A, J02 (this motor) 0.2983 A**.

**Setup**
1. Alignment must be valid in this boot: press **`V`** (or `f`).
2. Put **5 marks on the motor bell at ~72°** (one revolution) and a fixed pointer. These are positions 1–5. **The raw positions printed by the firmware are the record**; the marks are just there to come back to.

**Forward, then reverse, at the same 5 positions**
3. Turn the bell to mark 1. **Let go and wait 2 s** so it settles into a cogging detent. A rotor left mid-creep reads far too low.
4. Press **`B`**. The current ramps 0.005 A every 150 ms (about 9 s to 0.3 A). The firmware prints `M4,+1,<amps>,<raw>,<ramp_s>,<travel_cnt>` when the rotor moves.
5. Repeat 3–4 at marks 2, 3, 4, 5.
6. Repeat at marks 1–5 with **`b`** (reverse).

| Gate | Pass | If not |
|---|---|---|
| Every reading | a value is printed, no `NO MOTION` (0.80 A) | Bearing preload or the magnet touching the sensor (M8). Stop |
| `travel` per reading | ≤ 200 counts | That reading is biased high: re-settle and redo that one |
| Single readings | ≤ 0.60 A (the firmware warns above) | Redo once; if repeatable at that position, note it (a tight spot) |
| **Mean of the 10** | **0.20 – 0.40 A** | Below: check the reading method (settle 2 s). Above: dry/damaged bearing or a rubbing magnet |
| sd of the 10 | ≤ 0.12 A (J02: 0.11, "rough"; J01: 0.06) | High scatter with a normal mean = uneven grease, not a fault |

> **⚠ Correction to the session plan:** it gave M4's pass as "mean ≤ ~0.10 A". That is the **dynamic** drag level (phase 5), not breakaway. **Measured** motor-alone breakaway is 0.29–0.30 A on both built joints (CONSTANTS §8.2), so ≤ 0.10 A would fail every healthy motor. This sheet gates breakaway against breakaway.

7. Put the mean into **`breakaway_A`** with n, sd and the date in the comment. Save the 10 `M4,` rows verbatim as `docs/cal/J03/J03_<date>_M4 (bare motor).csv`. These raw positions are what later checks repeat, pairwise.

---

## Step 7: Tier-0 board ID and boot check (10 min)

1. `pio run -e T0_J03 -t upload`, then `pio device monitor -e T0_J03 -p COMx`.
2. From the boot text, copy two lines:
   - **`cs gains after align: ...`**. Pass: **−36.458333 on all three**, meaning the start-up current-sense check changed nothing. This is the per-board evidence for skipping that check (and its phase pulses) at leg boot.
   - **`{ "J03", { 0x..., 0x..., 0x... } },`**, the board ID row. Paste it into `JOINT_UID[]` (index 2, the `J03` entry) in `src/joint_cal.h`.
3. Flash the harness back: `pio run -e J03 -t upload` (the next phases use it).

| Gate | Pass |
|---|---|
| `can_proto self-test` | PASS |
| `CAN: HSE ready ... NBTP=0x500 ... DAR=1` | present (no bus needed) |
| `cs gains after align` | −36.458333 ×3 |

---

## Step 8: commit and archive (5 min)

`docs/cal/` is ignored by `.gitignore` (`cal*`), so its files need `-f`:

```
git add src/joint_cal.h
git add -f "docs/cal/J03/"
git commit -m "J03 belt-off baseline: M1, AUTOCALIB, M2, M4, Tier-0 UID"
```

---

## Results sheet: fill in and send

| Step | Quantity | Measured | Pass band | ✓ |
|---|---|---|---|---|
| 1 | V_meter1 / Vb1 / V_meter2 / Vb2 | | | |
| 1 | s1 vs s2 | | ≤ 0.3 % | |
| 1 | **vbus_scale** | | — | |
| 1 | banner vs meter (both V) | | ≤ 0.03 V | |
| 2 | parity / no_mag | | 0 / 0 | |
| 3 | zea / dir | | PASS | |
| 3 | R_eff | | 0.211–0.233 Ω | |
| 3 | U0 | | 0.005–0.030 V | |
| 3 | **Ke** | | 0.017916–0.018278 | |
| 3 | L | | 41.6–50.9 µH | |
| 3 | drag_c fwd / rev | | 0.075–0.139 A | |
| 3 | T/T_loop | | 0.913–1.003 | |
| 4 | V (° elec) | | ≤ 8° | |
| 5 | cold / hot R_eff | | ≤ 2 % apart | |
| 5 | Ibus span | | 230–280 mA | |
| 5 | **i_scale (g ± )** | | 0.95–0.98 | |
| 6 | breakaway mean / sd (n = 10) | | 0.20–0.40 / ≤ 0.12 A | |
| 7 | cs gains | | −36.458333 ×3 | |
| 7 | UID row | | recorded | |

**Not in this sheet, on purpose:** hot/cold R under load (M10), the 14-per-turn ripple, the |I|/Iq ratio check, J_rotor. Each is either a fleet constant already measured on two joints or not needed for a decision now (CLAUDE.md, "Everything earns its place").
