# Control — tuned and measured

§10 current loop, bandwidth limits, velocity loop (harness only), per-unit calibration policy.

*Hub: [`README.md`](README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 10. Control

### Current loop (`foc_current`) — closed

| | Value |
|---|---|
| `PID_current_q/d.P` | **0.1** |
| `PID_current_q/d.I` | **335** |
| `LPF_current_q/d.Tf` | **0.00025** |
| `PID_current_q/d.limit` | `voltage_limit` |
| `current_limit` | 2.0 A |

Measured step 0.5 → 1.5 A, locked rotor: **412 Hz, 9.3% overshoot, rise 850 µs, ζ 0.60, zero
steady-state error**, Iq sd 0.0094 A. That is ~80% of the transport-delay ceiling (~1/(3T) ≈ 500 Hz);
more bandwidth needs a faster loop, not different gains. Lowering `Tf` further buys damping, not
bandwidth.

| Config | Overshoot | Bandwidth |
|---|---|---|
| `I=900, Tf=0.0005` | 83% | 203 Hz |
| `I=335, Tf=0.0005` | 15.6% | 337 Hz |
| **`I=335, Tf=0.00025`** | **9.3%** | **412 Hz** (adopted) |

> ⚠ **Do not retune these gains to the design rule.** Pole-zero cancellation wants `P/I = L/R` =
> 195.87 µs; the configured ratio is 298.5 µs (tuned when L was believed to be 65 µH). The loop is
> proportional-dominated and the measured step response is what the gains stand on. Raising `I`
> toward the rule moves back toward the 83% regime. **AUTOCALIB prints `suggested P=0.1089
> I=555.6` — do not paste it.**

### Bandwidth limits

**Current loop:** required ≈ 5 × the foot's natural frequency (~1.32 kg effective mass): 5 / 20 /
50 N/mm → 49 / 98 / 155 Hz. 412 Hz gives 2–8× margin.

**The binding limit is the belt.** Drivetrain mode 55–72 Hz, softening with amplitude
(f_d = 86.9 − 0.522·A on the ring; BELT_DRIVE §22.3, §22.7.16). **Tier-2 impedance ceiling
≈ f_n/3 = 20–24 Hz.** Size the policy action rate and commanded stiffness against **20 Hz**: the
drive is softest at large excursions, exactly when the most is asked of it. The belt is also a
series spring: output stiffness = kp·K/(kp+K) (80–89% of commanded at kp 41).

### Velocity loop — bench harness only

The Tier-0 contract has no cascaded velocity PID; velocity mode (`v`) is a test instrument.
It closes on the current loop (`PID_velocity.limit = CURR_LIMIT_A_rep`).

| Gain | Value |
|---|---|
| `PID_velocity.P` | **0.2 A/(rad/s)** |
| `PID_velocity.I` | **1.5** |
| `PID_velocity.D` | **0** (quantised encoder + filtered velocity = noise amplifier) |
| `LPF_velocity.Tf` | 0.02 |

Voltage-mode history: P_crit ≈ 1.0; tracked 2–5 rad/s to ±0.02; steps settled < 300 ms with
8–12% overshoot. Size P against disturbance headroom (`P ≤ headroom / (ripple × filter
attenuation)`), not P_crit. Not re-swept on the current loop; test with steps, not ramps.

### Per-unit calibration policy

| Constant | Scope | Per unit? | Cost of sharing |
|---|---|---|---|
| **`zea`** | assembly | **mandatory** | up to 180° elec — will not run |
| **`dir`** | assembly | **mandatory** | inverts torque |
| `R_eff` | motor + board | yes | current-loop gain only |
| `U0` | board | yes | sub-1 N of transparency |
| `Ke`, `Kt` | motor | yes | ±5% stiffness error in impedance control |
| `L` | motor | no | current-loop gain only |
| `J_rotor` | geometry | no | ±1% |
| INL | assembly | measure once | angle-error floor |
| Drag map, breakaway | assembly | yes | dominates transparency |

Implemented as AUTOCALIB plus the hand steps in [`cal/BELT_OFF_BASELINE.md`](cal/BELT_OFF_BASELINE.md),
run before final assembly while each joint can move freely.
