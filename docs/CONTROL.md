# Control — tuned and measured

§10 current loop, velocity loop, and the per-unit calibration policy for twelve
joints.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 10. Control — Tuned and Measured

### Current loop (`foc_current`) — CLOSED

| | value |
|---|---|
| `PID_current_q/d.P` | **0.1** |
| `PID_current_q/d.I` | **335** |
| `LPF_current_q/d.Tf` | **0.00025** |
| `PID_current_q/d.limit` | `voltage_limit` |
| `current_limit` | 2.0 A |

**Design rule.** For pole-zero cancellation the gain ratio must equal the electrical time constant:

```
CURQ_P / CURQ_I  =  L / R  =  tau_e        bandwidth omega_c = CURQ_I / R = CURQ_P / L
```

The old `900` gave a ratio of 111 µs against a believed `τ_e` of 300 µs — 2.7× too small, i.e. integral-dominated. That single number was the cause of the 83% overshoot, and fixing it was worth more than every filter change combined.

> ### ⚠ THESE GAINS DO NOT SATISFY THE DESIGN RULE, AND THEY ARE STILL CORRECT. DO NOT "FIX" THEM.
> **Recorded 2026-08-13.** The gains were chosen when `L` was believed to be ~65 µH, which is now retired as a phase-locked fit artefact (§8.1a).
>
> | | Value |
> |---|---|
> | Configured ratio `CURQ_P/CURQ_I` | 0.1 / 335 = **298.5 µs** |
> | `L/R` the rule wants, with measured `L` = 43.77 µH, `R` = 0.22346 Ω | **195.87 µs** |
> | Mismatch | **1.52×, in the direction of too much P** |
> | 65 µH / 0.218 Ω | 298 µs — *exactly the configured ratio.* This is the fingerprint of where the numbers came from |
>
> So the loop is **proportional-dominated**, not the integral-dominated failure that was fixed in July. And it measures **412 Hz, 9.3% overshoot, ζ = 0.60, zero steady-state error, at ~80% of the transport-delay ceiling.** Raising `CURQ_I` by 1.66× toward the rule would move back toward the 83%-overshoot regime.
>
> ⚠ **The M1 rescale of 2026-08-18 does not reach this box.** `L` and `R` both moved ×1.010768, so `L/R` is **195.87 µs against the previous 195.90** — a ratio, immune by construction. **The current-loop gains are untouched and must not be retuned for it.** The only thing that moved is plant DC gain, by 1.1%, negligible against ζ = 0.60. (`L` = 43.31 → 43.77 µH and `R` = 0.22108 → 0.22346 Ω in the row above are the same rescale; the 65 µH / 0.218 Ω fingerprint line below is left at its original values because it is a record of where the old numbers came from.)
>
> **The gains stand on the measured step response, not on the rule.** A rule is a starting point for a plant you have not measured; a measured step response outranks it. This is the same source-of-truth ordering that closed `Ke = Kt` and `L = 65 µH`.
>
> ⚠ **AUTOCALIB prints `suggested P=0.1089 I=555.6 for 400 Hz`. Do not paste it.** It is the design rule applied to the corrected `L`, and it is exactly the change this box exists to prevent. The routine has always been specified to print a suggestion and never apply one (§20.3, M12) — that separation is load-bearing here.

**Measured step response, 0.5 → 1.5 A:**

| config | overshoot | rise 10–90% | bandwidth | notes |
|---|---|---|---|---|
| `I=900, TF=0.005` | 83% | — | 80 Hz | filter-dominated |
| `I=900, TF=0.0005` | 83% | ~2.0 ms | 203 Hz | previous baseline |
| `I=335, TF=0.0005` | 15.6% | 1022 µs | 337 Hz | gain fix alone |
| **`I=335, TF=0.00025`** | **9.3%** | **850 µs** | **412 Hz** | **adopted.** ζ = 0.60, zero steady-state error, Iq sd 0.0094 A (0.6%), locked rotor |
| `I=900, TF=0.0002` | — | — | — | historical runaway with the *old* gains |

**Why to stop here.** Lowering `CUR_TF` further buys damping, not bandwidth — the modelled optimum is at 250 µs and bandwidth *declines* below it. And a transport delay of ~80–100 µs caps a well-damped loop at roughly `1/(3T)` ≈ 500 Hz. **412 Hz is ~80% of the hard ceiling.** More would need a faster control loop, not different gains.

**Why 412 Hz is enough.** Required current bandwidth ≈ 5× the natural frequency of the foot against commanded stiffness, with ~1.32 kg effective mass at the foot:

| Foot stiffness | Natural freq | Needed |
|---|---|---|
| 5 N/mm | 9.8 Hz | 49 Hz |
| 20 N/mm | 19.6 Hz | 98 Hz |
| 50 N/mm | 30.9 Hz | 155 Hz |

Legged robots run 5–30 N/mm. **You have 2–8× margin.**

> ### 🔴 The binding bandwidth limit is the BELT, not the current loop — measured 2026-09-05
>
> The drivetrain resonance is **59–72 Hz**, not the 100–230 Hz that was assumed when the
> margin above was written. **Tier-2 impedance bandwidth ceiling ≈ `f_n`/3 = 20–29 Hz.**
>
> ⚠ **It is not a single number, and it moves the wrong way.** The drive is a **softening
> spring** — measured `f_d = 86.9 − 0.522 × A`, r = −0.982 against ring amplitude, with
> amplitude explaining 96.5% of the variance (§22.3):
>
> | Excursion | `f_d` | Tier-2 ceiling |
> |---|---|---|
> | Small, near equilibrium (~29 ct) | 72 Hz | **~24 Hz** |
> | Large (~54 ct) | 59 Hz | **~20 Hz** |
>
> **Small motions see the stiff end; impact transients see the soft end.** For an impedance
> controller that is exactly backwards — **the drive gets softer precisely when the most is
> being asked of it.** Consistent with tooth engagement at 3.1 teeth in mesh rather than cord
> stretch, which is linear.
>
> **Consequence for Tier 2:** the RL policy's action rate and any commanded stiffness must be
> sized against the **20 Hz** end, not the 29 Hz end, and not against the current loop's
> 412 Hz. **First time this has been measured rather than assumed.**

### Velocity loop — voltage-mode numbers, NOT yet retuned on current

| gain | value |
|---|---|
| `PID_velocity.P` | **0.2 A/(rad/s)** — was 0.45 V/(rad/s) in voltage mode |
| `PID_velocity.I` | **1.5** — was 2.0 |
| `PID_velocity.D` | **0** — never use D: quantised encoder + filtered velocity = noise amplifier |
| `LPF_velocity.Tf` | 0.02 |

- P_crit ≈ 1.0. Tracks 2–5 rad/s to ±0.02; steps settle <300 ms with 8–12% overshoot.
- Disturbance rejection verified 2–10 rad/s.
- **2026-08-01 note:** on the ORIGINAL assembly with the belt on, a P sweep at 2.0–2.8 was entirely inside `current_limit` saturation — friction (1.05 A) eats half the 2.0 A budget and the 1/rev disturbance times P exceeds the rest. Size velocity P against the disturbance, not against P_crit: `P ≤ headroom / (ripple × filter attenuation)` → **P ≈ 0.2, I ≈ 1.5**, with I doing the work of supplying the friction current at zero error.

### Per-unit calibration policy for 12 joints (set 2026-08-06)

~~The two bench assemblies differ by **9.3% in `R_eff`**~~ — **retracted, the real spread is +1.8%** (§1a). What must be measured per joint is still answered with data rather than assumed, but the answer changed: **`zea` and `dir` are the mandatory ones**, not `R_eff`.

| Constant | Scope | Per-unit? | Cost of sharing |
|---|---|---|---|
| **ZEA** | assembly | **MANDATORY** | up to 180° elec — motor will not run |
| **sensor_direction** | assembly | **MANDATORY** | inverts torque |
| `R_eff` | motor + board | **yes** | ±10% → current-loop gain error only |
| `U0` | board | **yes** | ±50% → sub-1 N of transparency |
| `Ke`, `Kt` | motor | recommended | ±5% → ±5% stiffness error in impedance control |
| `L` | motor | no | ±10% → current-loop gain only |
| `J_rotor` | geometry | **no** | ±1% |
| INL | assembly | measure once | sets the angle-error floor |
| Drag map / friction | assembly | **yes** | dominates transparency — a 46% term |

**Policy: automate it, do not repeat it by hand.** One on-board `AUTO` routine per joint: encoder self-test → N alignments → median ZEA + direction detect → locked-rotor `R_eff`/`U0` sweep → free-spin `Ke` → print a paste-ready constants block. Target ~5 min per joint, ~1 hour for twelve.

**This is standard practice, not a workaround.** Every industrial servo drive ships a motor-identification routine (measures R, L, aligns the encoder at commissioning); robots with absolute encoders store a per-joint offset in drive flash at the factory; quasi-direct-drive research controllers do exactly this per unit. The routine must run **before final assembly**, while each joint can still move freely.
- **These gains are in VOLTS. On the current loop the PID output is AMPS.** Rescale by 1/R for the same DC gain — but that does *not* preserve stability margin, so sweep P_crit fresh.
- **Scope this deliberately.** The Tier-0 contract is `τ = kp(q_d−q) + kd(v_d−v) + τ_ff` → current loop. **There is no cascaded velocity PID in the shipping architecture.** Velocity mode is a test harness; tune it to "usable instrument" and stop. Test with **steps, not ramps**.

---