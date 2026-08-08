# `docs/cal/` — raw AUTOCALIB records

One file per joint per session: **`<JOINT>_<YYYY-MM-DD>.csv`**, e.g. `J01_2026-08-07.csv`.

**What goes in:** the `BIN,` and `LSB,` blocks from the phase-7 report, **verbatim**, plus the `DRAG,` lines. Do not reformat, do not round, do not drop the header comment lines — the point of this directory is that a fit can be redone offline months later against exactly the bytes the firmware emitted.

**What does not go in:** the summary constants. Those belong in the `JointCal` row in `src/joint_cal.h`, where they are one reviewable git diff with a date on it.

## Why these two blocks specifically

| Block | Columns | What it is for |
|---|---|---|
| `BIN,` | `bin, fwd_deg, rev_deg, even, odd` | The 32-bin parity separation. The **odd** part gives `T_delay`; the **even** part is the INL profile plus the ZEA residual. The 1/rev vs 2/rev harmonic split — eccentricity vs channel-gain mismatch — can only be fitted **offline**, and it is too large to store in the struct. |
| `LSB,` | `t_us_centre, I_mean, n, frac` | The inductance step trace, binned by time. This is what made the τ bias visible: the `n` column peaking every third bin is the un-dithered loop-phase grid showing through. **An independent refit of this block is the only check on the firmware's own `L` fit** — on `J01_2026-08-07` it reproduced τ to 195.92 vs 195.9 µs. |

## Status

| File | Present | Note |
|---|---|---|
| `J01_2026-08-07.csv` | ❌ **not captured** | The session-3 rev2 run happened, and its *derived* results are in `README.md` §8.1a and in the `J01` row. The raw serial blocks were not saved to the repo, and **they cannot be reconstructed from the summary.** Re-dump them on the next phase-7 run before the belt goes on — after that the belt-off INL and drag baselines are gone for good. |

Nothing in the firmware reads this directory. It is a record, not an input.
