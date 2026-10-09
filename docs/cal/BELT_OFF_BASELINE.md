# Belt-off joint baseline: command sheet

Produces a joint's `joint_cal.h` row, its Tier-0 board ID, and raw records in `docs/cal/Jnn/`. About 1.5 h.
**Plant:** bare motor, clamped by its body, belt off, not in the plate.
**Firmware:** bench harness `-e Jnn`, plus one Tier-0 boot at the end.
Written with J03's numbers; for another joint, swap the number and use its motor's reference row.

**Board numbers are stored** (`vbus_scale`, `i_scale`, `R_eff`, `U0`, `zea`/`dir`). **Motor numbers are gates** against the motor's existing row (`Ke`, `L`, drag, breakaway).

## Setup

- **UT89X** meter. **Bench pack ~12 V** for everything. **Second supply 17–28 V** for M1 point 2 only.
- **M2 only:** a second voltmeter (RC3563 or any DMM), because the UT89X is the ammeter there. See Step 4.
- Motor clamped (phase 5 spins ~1,000 rpm). Magnet gap 0.5–1.0 mm, centred.
- 5 pen marks at ~72° on the motor bell, plus a fixed pointer (for M4).
- Console: `pio device monitor -e J03 -p COMx`. Every input is a **single key**.

| Key | Does |
|---|---|
| `p` | Bus-voltage probe (`seed Vb=`, 4 dp) |
| `e` / `E` | Encoder self-test / live monitor |
| `Y`, `1`…`7` | AUTOCALIB status, phases 1–7 |
| `V` | Verify stored `zea` |
| `N` | M2 ladder |
| `B` / `b` | M4 breakaway, fwd / rev |
| `x` | Stop. Any key aborts a running phase |

Log header: `J03  date  board label  motor label  pack V  meter UT89X  magnet gap`.

---

## Step 0: prepare (desk)

1. Label the board. `board_sn` / `motor_sn` must match the physical labels.
2. In the J03 row, set **`vbus_scale = 0.0085f`** (provisional). With `0.0f` the seed is ignored and M1 has nothing to read. **Don't derive M1 from the `p` DMA buffer instead:** on J03 it was 0.66 % off.
3. `pio run -e J03 -t upload`.

## Step 1: M1, voltage scale (15 min)

1. **~12 V:** meter at the board's `7V-48V`/`GND` pads → **Vm1**. Press `p` → **Vb1**.
2. Power down, switch to the second supply, power up (the seed is read once at boot). Meter → **Vm2**, `p` → **Vb2**.
3. Compute (or send the four numbers to Claude Code):
   ```
   c = Vb / 0.0085                vbus_scale = (Vm2 - Vm1) / (c2 - c1)
   check: Vm1/c1 and Vm2/c2 agree within 0.3 %
   ```
4. Store it, reflash, and check the banner `Vbus=` against the meter at **both** voltages: **±0.03 V**. If one is off, reboot and re-read before blaming the scale.

Back to ~12 V for the rest.

## Step 2: encoder (2 min)

Press `e`: parity **0/2000**, `no_mag` **0**. Then `E` and **turn one full revolution**: `perr=0`, and some window shows `span=16383`. (`HIT` flags are expected while turning.)

> ⚠ **`e` alone is not a pass.** A chip that never drives MISO returns all-zero frames, which pass parity and `no_mag` while `raw` sits at 0 (J03, 2026-10-08).

**If `raw` is stuck:**
1. Power on, firmware running: VDD and HVPP **3.3 V**, CSN **≈1.85 V**, SCK **≈2.5–3.0 V**. Those two are meter averages of lines that toggle every loop; flat 0 or 3.3 V = not toggling.
2. Power off: continuity PB5→CSN, `HA/A`→MOSI, `HB/B`→MISO, `HC/Z`→SCK, and MISO not shorted to GND.
3. All fine → **swap the encoder board.** J03's original had a dead MISO output.

## Step 3: AUTOCALIB (10 min)

Gate first: banner `Vbus=` vs meter **±0.03 V**, else reboot.

Press **`Y 1 2 3 4 5 6 7`**, one at a time, waiting for each result line. `2` twitches; `5` spins both ways, so keep hands clear.

| Quantity | Reference (motor_2 / J02) | Pass |
|---|---|---|
| `zea`, `dir` | — | phase 2 PASS |
| `R_eff` (board) | 0.222 Ω, current ladder | 0.211–0.233 Ω |
| `U0` (board) | 0.015 V | 0.005–0.030 V |
| **`Ke`** | 0.018097 | ±1 %: 0.017916–0.018278 |
| `L` | 46.25 µH | 41.6–50.9 µH |
| `drag_c` fwd / rev | 0.106 / 0.107 A | 0.075–0.139 A each |
| `T/T_loop` | 0.958 | 0.913–1.003 |

**`Ke` is the M1 cross-check.** A miss with the banner off → redo M1. A small miss with M1 and the banner both confirmed → the reference is suspect, not the board (J03: −1.09 %, J02's scale carries a 0.32 % ambiguity). Store the new board's own `Ke` and note why.

Save the full phase-7 output **verbatim** as `docs/cal/J03/J03_<date>_autocalib (bare motor).csv`.

> **Then¹:** paste the phase-7 row over the J03 row; fill `board_sn`, `motor_sn`, date and `"OFF"`; check `vbus_scale` is the M1 value. Flash `-e J03`, press **`V`**: **≤ 8° elec** and direction matches. 8–15° = check the magnet; > 15° = wrong row or magnet slipped. Commit `joint_cal.h`.

## Step 4: M2, current scale → `i_scale` (25 min)

Theory, prediction table and fit: [`CALIBRATION.md`](../CALIBRATION.md) §20.1, M2 box.

### Bench setup

Wire this **before powering up**, and leave it in place for all of Step 4. The cold `R_eff`, the ladder and the hot `R_eff` must all see the same circuit.

| | |
|---|---|
| **Ammeter** | UT89X on **DC mA, 600 mA range**, in series with the **positive** supply lead: pack + → UT89X **mA** jack; UT89X **COM** → board `7V-48V` pad. Pack − goes straight to board `GND`. **Never the 10 A range:** 10 mA resolution is 4 % of the span |
| **Voltmeter** | A **second** meter (the RC3563 works). The UT89X can't do both, because moving its lead to the V jack opens the supply. **Gate (step 1): probes on the board pads**, since the banner reads the pads and the ammeter drops volts before them. **Ladder (`N`): probes on the pack terminals.** That is `V_s`; the fit subtracts the ammeter drop itself (`V_term = V_s − Ibus·R_b`). `V_s` barely matters (0.1 V → 0.0004 on `g`) |
| **`R_b`** | The ammeter's own resistance: **1.547 Ω** (UT89X, 600 mA range). Re-measure only if the meter or leads change: 4-wire, **unpowered** |
| **Plant** | Motor clamped, belt off, hands off the shaft. The rotor is held magnetically and must not creep (`drift=`) |

**Reading:** Ibus falls point by point, **~337 → ~81 mA** (§20.1 burden column). At each point let the reading settle, then write **Ibus to 0.1 mA** (e.g. `336.8`) and **V_s** beside that point's `M2,` line, then press any key. The firmware auto-advances after 12 s, so read within that window.

1. Gate (voltmeter on the **pads**): banner vs meter **±0.03 V**, else reboot. Then move the voltmeter to the **pack**.
2. `1`, `2`, `3` → **cold `R_eff`**.
3. `N`: at each of 8 points, record **V_s** (pack) and **Ibus to 0.1 mA**, then any key (auto-advances at 12 s).
4. `3` again at once → **hot `R_eff`**.
5. Send the log to Claude Code for the fit (`g = 1.5·R/c`). Store `i_scale = g` with its ±, then reflash.

| Gate | Pass |
|---|---|
| `I_reported` vs the §20.1 table | ±3 % |
| Ibus span, pt 1 → 8 | 230–280 mA |
| hot vs cold `R_eff` | ≤ 2 % |
| `drift=` | 0 every point |
| `g` | 0.95–0.98 (J01 0.9621, J02 0.9690) |

Fleet `i_scale` for later boards is decided at milestone close, not here. Record `g ±` only.

## Step 5: M4, breakaway → `breakaway_A` (10 min)

Static breakaway (~0.3 A), not phase-5 dynamic drag (~0.1 A). References, motor-alone: J01 0.2923, motor_2 0.2983 A.

1. Press `V` (valid alignment).
2. At each mark 1–5: turn to the mark, **let go, wait 2 s** (settle into a detent), press **`B`**. It ramps ~0.033 A/s and prints `M4,+1,<A>,<raw>,<s>,<travel>`.
3. Repeat marks 1–5 with **`b`**.

| Gate | Pass |
|---|---|
| each reading | no `NO MOTION` (0.80 A); `travel` ≤ 200 (else redo it); ≤ 0.60 A |
| **mean of 10** | **0.20–0.40 A** |
| sd | ≤ 0.12 A |

Store the mean (with n, sd and date) as `breakaway_A`. Save the 10 `M4,` rows as `docs/cal/J03/J03_<date>_M4 (bare motor).csv`. Their raw positions are what later checks repeat, pairwise.

## Step 6: Tier-0 ID, then commit (10 min)

1. `pio run -e T0_J03 -t upload`, then open the monitor on `T0_J03`. From the boot text:
   - `can_proto self-test: PASS`, and `CAN: HSE ready ... NBTP=0x500 ... DAR=1`.
   - **`cs gains after align`: −36.458333 ×3.** This is the evidence for skipping the boot pulses at leg boot.
   - The **`{ "J03", { 0x…, 0x…, 0x… } },`** line → paste into `JOINT_UID[]` (J03 entry).
2. Flash the harness back: `pio run -e J03 -t upload`.
3. Commit. `docs/cal/` is git-ignored (`cal*`), so it needs `-f`:
   ```
   git add src/joint_cal.h
   git add -f "docs/cal/J03/"
   git commit -m "J03 belt-off baseline: M1, AUTOCALIB, M2, M4, Tier-0 UID"
   ```

---

## Results (fill in and send)

| | Measured | Pass |
|---|---|---|
| Vm1 / Vb1 / Vm2 / Vb2 → `vbus_scale` | | ratios ≤ 0.3 %, banner ±0.03 V |
| encoder parity / no_mag / full sweep | | 0 / 0 / yes |
| R_eff · U0 · **Ke** · L | | see Step 3 |
| drag_c fwd / rev · T/T_loop | | see Step 3 |
| `V` | | ≤ 8° elec |
| M2: cold/hot R · Ibus span · **g ±** | | ≤ 2 % · 230–280 mA · 0.95–0.98 |
| M4: mean / sd (n = 10) | | 0.20–0.40 / ≤ 0.12 A |
| cs gains · UID row | | −36.458333 ×3 · recorded |

¹ The former "Step 4" (store the row, `V`) is a follow-on to Step 3, not a separate step.
