# Control — tuned and measured

§10 current loop, bandwidth limits, velocity loop (harness only), per-unit calibration policy.

*Hub: [`TECHNICAL_REFERENCE.md`](TECHNICAL_REFERENCE.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 10. Control

### Current loop (`foc_current`) — closed

Settings: [CONSTANTS.md §8.5](CONSTANTS.md#85-bench-tuning-reference).

Measured step 0.5 → 1.5 A, locked rotor: **412 Hz, 9.3% overshoot, rise 850 µs, ζ 0.60, zero
steady-state error**, Iq sd 0.0094 A (legacy bench measurement).
The former 500 Hz / 80%-of-ceiling claim was unsupported: legacy T ≈ 71 µs gives
1/(3T) ≈ 4.7 kHz; MT6835's 91.5 µs gives ≈ 3.64 kHz. Neither is an established closed-loop
ceiling: filtering, PWM/sample delay, controller phase and selected margin matter. At 412 Hz,
a pure 91.5 µs delay contributes about 13.6° phase lag. Retain measured gains and revalidate
after plant/build changes; higher bandwidth is not guaranteed by this arithmetic.

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

Settings: [CONSTANTS.md §8.5](CONSTANTS.md#85-bench-tuning-reference).

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
| `L` | motor | store per-unit estimate; QC against motor reference | shared gains still require response validation |
| `J_rotor` | geometry | no | ±1% |
| INL | assembly | measure once | angle-error floor |
| Drag map, breakaway | assembly | yes | dominates transparency |

Implemented as AUTOCALIB plus the hand steps in [`cal/BELT_OFF_BASELINE.md`](cal/BELT_OFF_BASELINE.md),
run before final assembly while each joint can move freely.
