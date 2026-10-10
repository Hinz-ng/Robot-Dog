# `docs/cal/` — raw calibration records

Raw serial output, verbatim, so a fit can be redone offline against exactly what the firmware
emitted. Summary constants do not go here; they go in the `JointCal` row in `src/joint_cal.h`.
Nothing in the firmware reads this directory.

**Naming:** `docs/cal/Jnn/Jnn_<what> (<plant>).csv`, e.g. `J03/J03_autocalib (bare motor).csv`.
New-joint procedure: [`BELT_OFF_BASELINE.md`](BELT_OFF_BASELINE.md).

**What goes in:** the full phase-7 output (including the `BIN,` and `LSB,` blocks and `DRAG,` lines),
`M2,` and `M4,` rows, capture dumps — unedited, with a header line giving joint, date, plant state,
pack voltage and meter reading.

| Block | Columns | Use |
|---|---|---|
| `BIN,` | `bin, fwd_deg, rev_deg, even, odd` | 32-bin parity separation: odd → `T_delay`; even → INL + ZEA residual (1/rev vs 2/rev fitted offline) |
| `LSB,` | `t_us_centre, I_mean, n, frac` | inductance step trace binned by time; the only independent check on the firmware's `L` fit |

⚠ `cal*` in `.gitignore` matches this directory: new files need `git add -f`.

### Files captured before the M1/M2 corrections

J01 and J02 files from before 2026-08-18 carry `vbus_scale=0.008358` in their banners and emit
uncorrected "PASTE THIS ROW" blocks. **Do not paste from them.** To reuse a value:

| | Volts-derived (`R_eff`, `U0`, `Ke`, `L`, `Kt`) | Newtons / torque |
|---|---|---|
| J01 / `B-SPI-01` | × 1.010768 | × 1.0505 |
| J02 / `B-ABZ-01` | × 1.018904 | × 1.0515 |

Amperes, radians, τ_e = L/R and other ratios are unchanged. `BIN,`/`LSB,` shapes are unaffected.

### Contents

| Path | What |
|---|---|
| `J01/` | belt-off AUTOCALIB, M2, M4, hot/cold `R_eff`, belt-on no-idler drag (2026-08-12) |
| `J02/` | belt-off AUTOCALIB, M2, M4, hot/cold `R_eff`, belt-on drag tests (top on / off, idlers off) |
| `J03/` | belt-off AUTOCALIB, M2, M4 (2026-10-08/09) |
| `pulley acceptance/` | J01 recipe-B acceptance, 2026-09-30: swing ladder ×3, B4 breakaway (20 `M4,` rows), B6b ring (caps 1–4, 6–9), B3 dynamic drag both orders. Produced by a pre-2026-09-28 binary (banner prints `ladder 1/2/3 A`, `belt=OFF`) — read banners as build provenance, not plant state. Recipe-A raw rows were not kept |
| `M6a J_rotor.csv`, `coast-down_runs1&2.csv` | `J_rotor` determinations (2026-08-08) |
