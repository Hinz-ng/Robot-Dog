# `docs/cal/` — raw AUTOCALIB records

One file per joint per session: **`<JOINT>_<YYYY-MM-DD>.csv`**, e.g. `J01_2026-08-07.csv`.

**What goes in:** the `BIN,` and `LSB,` blocks from the phase-7 report, **verbatim**, plus the `DRAG,` lines. Do not reformat, do not round, do not drop the header comment lines — the point of this directory is that a fit can be redone offline months later against exactly the bytes the firmware emitted.

**What does not go in:** the summary constants. Those belong in the `JointCal` row in `src/joint_cal.h`, where they are one reviewable git diff with a date on it.

## Why these two blocks specifically

| Block | Columns | What it is for |
|---|---|---|
| `BIN,` | `bin, fwd_deg, rev_deg, even, odd` | The 32-bin parity separation. The **odd** part gives `T_delay`; the **even** part is the INL profile plus the ZEA residual. The 1/rev vs 2/rev harmonic split — eccentricity vs channel-gain mismatch — can only be fitted **offline**, and it is too large to store in the struct. |
| `LSB,` | `t_us_centre, I_mean, n, frac` | The inductance step trace, binned by time. This is what made the τ bias visible: the `n` column peaking every third bin is the un-dithered loop-phase grid showing through. **An independent refit of this block is the only check on the firmware's own `L` fit** — on `J01_2026-08-07` it reproduced τ to 195.92 vs 195.9 µs. |

> ## 🔴 2026-08-18 — EVERY CSV IN HERE PREDATES THE M1 CORRECTION. DO NOT PASTE FROM ONE.
>
> These captures are **raw and deliberately unedited**, which means their `CFG` banners still read `vbus_scale=0.008358` and their phase-7 **"PASTE THIS ROW"** blocks still emit the pre-correction constants. M1 measured the dividers per board — `B-SPI-01` **0.008448**, `B-ABZ-01` **0.008516** — so:
>
> | To reuse a value from a file in here | Multiply by |
> |---|---|
> | any `J01` / `B-SPI-01` capture (`R_eff`, `U0`, `Ke`, `L`, `Kt`, any newton) | **1.010768** |
> | any `J02` / `B-ABZ-01` capture | **1.018904** |
> | any **ampere**, **radian**, `τ_e = L/R`, or ratio | **1** — these never pass through the voltage belief |
>
> **And since 2026-08-20 there is a SECOND factor on forces only.** M2 measured the current sense under-reading, so any **torque or newton** taken from a file in here needs `i_scale` as well:
>
> | | M1 (`vbus_scale`) | M2 (`i_scale`) | **net on a newton** |
> |---|---|---|---|
> | J01 / `B-SPI-01` | ×1.010768 | ÷0.9621 | **×1.0505** |
> | J02 / `B-ABZ-01` | ×1.018904 | ÷0.9690 | **×1.0515** |
>
> The amperes still do not move under either. **That invariance is the only reason two independent corrections could be applied to this archive without re-running anything** — which is also the argument for gating bench decisions on amps rather than on percentages of standing load.
>
> **`src/joint_cal.h` is the corrected source of truth.** The `BIN,` and `LSB,` blocks — the reason this directory exists — are unaffected as *shapes*: harmonic amplitudes in degrees and the τ fit are both immune. Offline refits stay valid; only absolute volts-derived constants move.

## Status

| File | Present | Note |
|---|---|---|
| `J01_2026-08-07.csv` | ❌ **not captured** | The session-3 rev2 run happened, and its *derived* results are in `README.md` §8.1a and in the `J01` row. The raw serial blocks were not saved to the repo, and **they cannot be reconstructed from the summary.** Re-dump them on the next phase-7 run before the belt goes on — after that the belt-off INL and drag baselines are gone for good. |

Nothing in the firmware reads this directory. It is a record, not an input.
