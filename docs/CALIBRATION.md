# Manual calibration and per-joint storage

§20 what AUTOCALIB does not measure, and how to do each by hand · §21 per-joint storage
(`joint_cal.h`, per-joint environments, the `V` verify command).

New joint, step by step: [`cal/BELT_OFF_BASELINE.md`](cal/BELT_OFF_BASELINE.md). This file holds
the method behind those steps.

*Hub: [`README.md`](README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 20. Manual calibration — what AUTOCALIB does not measure

AUTOCALIB (`Y`, phases 1–7) measures `zea`, `dir`, `R_eff`, `U0`, `Ke`, `L`, the drag map,
`T/T_loop` and INL in ~70 s of motor time. Phase 6 is belt-off only; phase 4 is belt-off only.

### 20.1 Per board, once — needs an external reference

| # | What | How |
|---|---|---|
| **M1** | **`vbus_scale`** | Two points at clearly different bus voltages, meter at the board's `7V-48V`/`GND` pads. `scale = (V₂−V₁)/(c₂−c₁)`. **Run before AUTOCALIB** (`R_eff`, `U0`, `Ke` are measured through it). Use a cross-checked meter and name it in the row comment (the DT9205A's 1.1% gain error once made J01's scale 1.1% low). The `Kt`-vs-KV comparison cannot check this scale |
| **M2** | **Absolute current-sense scale → `i_scale`** | Locked-rotor bus power balance (below). Not a phase-lead ammeter (the DC split between phases depends on the unknown stop angle) |
| M3 | Bus-cap and FET voltage ratings | read the markings |

#### M2 — method

A two-point difference does not work: switching (~0.15 W) and dead-time (~0.05 W) losses scale
with I and would report g ≈ 0.95 on a perfect board. Fit eight points:

```
P_bus = a + b*I + c*I^2      a = housekeeping, b = switching + dead time
U_del = U0 + R*I             <- this run's own slope (self-fit R)
g     = 1.5 * R / c
U_del = Uq_cmd * (V_s - Ibus*R_b) / V_seed      V_seed = the banner value
```

- **Use the run's own R** (same session, current range, thermal state), never a phase-3 `R_eff`.
- **`R_b`** = ammeter burden, 4-wire and **unpowered**: 1.547 Ω on the UT89X 600 mA range. It
  largely cancels (±16% on R_b → ±0.9% on g).
- **Belt-agnostic:** the rotor is magnetically locked; it must not creep (`drift=` per point).
- **Do not use a 10 A range** (10 mA resolution is 4% of the ~250 mA span).

**Instrumentation:**

| | How | Accuracy on g |
|---|---|---|
| A | Bench PSU with V and A readout, nothing in the circuit | ~±1.5% |
| B | Pack + two meters: ammeter (600 mA range) in series with the **positive** lead; voltmeter on the **board pads** | ~±2% |
| C | Pack + one meter: current sweep, then volts at the top and bottom point only | ~±2.5% |

**Procedure** (leg off, hands off the shaft, motor clamped):
1. Banner `Vbus=` vs meter at the pads: **> 0.03 V → power-cycle and re-check.** `V_seed` lands
   1:1 on g, and the seed has a real per-boot offset of up to 0.05 V. Record the reading even
   when it passes.
2. Measure `R_b` (unpowered) if the meter or leads changed.
3. `1`, `2` (alignment must PASS).
4. `3` → **cold `R_eff`**.
5. `N` → 8 points. At each, let the meter settle and write **Vbus** and **Ibus to 0.1 mA** next to
   the `M2,` line, then any key. Auto-advances at 12 s.
6. `3` immediately → **hot `R_eff`**.

**Predicted at g = 1.00** (R 0.22346 Ω, U0 0.01037 V, V_seed 12.22 V, R_b 1.547 Ω). Use the
burden column with B and C; the no-burden column with A:

| Point | Uq (V) | I no burden (A) | **I_reported with ammeter (A)** | **Ibus (A)** |
|---|---|---|---|---|
| 1 | 0.70 | 3.09 | **2.95** | 0.337 |
| 2 | 0.62 | 2.73 | **2.63** | 0.280 |
| 3 | 0.55 | 2.41 | **2.34** | 0.236 |
| 4 | 0.47 | 2.06 | **2.01** | 0.191 |
| 5 | 0.40 | 1.74 | **1.71** | 0.157 |
| 6 | 0.32 | 1.39 | **1.36** | 0.124 |
| 7 | 0.25 | 1.07 | **1.06** | 0.102 |
| 8 | 0.16 | 0.67 | **0.66** | 0.081 |

Absolute Ibus assumes ~1 W housekeeping; the **span (230–280 mA)** is what matters.

| Gate | Pass | If not |
|---|---|---|
| Banner vs meter at start | ±0.03 V | reboot |
| `I_reported` vs table | ±3% | lock not holding, or wrong joint flashed. A softer pack can read −5% at point 8; not a failure if `drift=` holds |
| Ibus span, point 1 → 8 | 230–280 mA | wiring, meter range, or the rotor moved |
| Hot vs cold `R_eff` | ≤ 2% | thermal drift; shorten dwells and redo |
| `dR` with `U0` held | +0.4 to +1.0% | refit with U0 constrained before reading it (R and U0 correlate −0.84) |
| `drift=` | ±3 counts | creep shows as tens |

**Coefficient checks** (a well-conditioned fit is not necessarily a clean one):

| | Check against | J01 2026-08-20 |
|---|---|---|
| `a` | idle current (ST-LINK USB plugged; unplugged adds 4.8 mA, HARDWARE §16a) × V_seed + ~0.10–0.12 W PWM ripple loss | 0.803 W vs 0.688 + ~0.12 |
| `b` | ≈ 1.5·U0 W/A | +0.035 vs 0.020 |
| U0 self-fit | phase-3 cold U0, ±0.003 V | 0.01350 vs 0.01686 |

| g | Action |
|---|---|
| 0.98–1.02 | `i_scale = 1.0`; record the measured g and SE in the row comment |
| within ±2σ of 1.0 | store it, marked **provisional** |
| 0.94–0.98 or 1.02–1.06 | store g as `i_scale` (the torque boundary corrects commands and feedback). Do not touch `R_eff`, `L`, `Ke` or the loop gains |
| outside ±6% | fix at source in the `LowsideCurrentSense` constructor, then re-run phases 3 and 4 |
| c ≤ 0 or SE(c) > 10% | fit failed; check the Ibus span and rotor motion |

Limit ±2% (dominated by the meter's DCA gain, 0.8%; a repeat buys ~0.1%). τ_actual = τ_des / g, so
an uncorrected 5%-high sense means 5% low torque with nothing flagging it.

#### M3b — electrical tripwire on a locked plant

When the output cannot turn (phase 3 and `V` unavailable): during a swing test the rotor is
stalled at each current limit. Fit the `(Iq, Uq)` pairs at vel ≈ 0 in the `|I|` domain, **both
directions combined** (the directional split is ~7% from dead-time asymmetry). J02 2026-09-05:
R 0.21536 Ω vs stored range 0.21556–0.21683 (0.09%).

### 20.2 Per assembly, after every mechanical rebuild

| # | What | How |
|---|---|---|
| **M4** | **Breakaway → `breakaway_A`** | `V` (or `f`, or phase 2) first. Belt off, pulley bare, leg **never** attached. At each of 5 positions ~72° apart: turn to the mark, **let go, wait 2 s** (settle into a detent), press `B`; then the same 5 with `b`. Ramp +0.005 A every 150 ms (~0.033 A/s); reports the current at \|ω\| > 0.5 rad/s as `M4,<dir>,<amps>,<raw>,<ramp_s>,<travel_cnt>`. Warns on `travel` > 200 counts (biased high, redo) and > 0.60 A; `NO MOTION` at 0.80 A is a fault (bearing preload or magnet skimming the sensor). Static breakaway is not phase 5's dynamic drag (J01: 0.2923 vs 0.0783 A belt-off). **A/B comparisons:** same raw positions, compared pairwise |
| M5 | Belt-on drag | Phase 5 belt on, both run orders (BELT_DRIVE B3) |
| M6a | `J_rotor`, belt off | `t` → `g` → `k` → ~1 s → `x` → `d` (Uq step 0.20 → 0.80 V). α from raw `cnt` over the first ~50 ms; `J = (Kt·Iq − τ_drag)/α` with the belt-off drag map. **Done: 20.2 ± 2.4 × 10⁻⁶ kg·m², fleet constant** |
| M6b | `J_total`, belt on | same, output unclamped (BELT_DRIVE B7); friction 40–60% of the impulse, ±20% |
| M7 | INL after belt fitting | phases 5 + 6 (read INL only) |
| **M8** | **Magnet gap and centring** | 0.5–1.0 mm, **never zero**, centred. Zero gap caused A1's count loss; centring sets the 1/rev INL |
| M9 | Gear ratio, teeth, lever arm | calipers and a tooth count |

### 20.3 Per operating point

| # | What | Trigger | How |
|---|---|---|---|
| **M10** | Hot vs cold `R_eff` | any time | phase 3 cold → load 60 s → phase 3. ΔR/R > 15% = peak force sags in a jump |
| M11 | `U0` | bus voltage change | dead time is a fraction of the PWM period, so U0 scales with V_bus |
| M12 | Current-loop gains | after R or L changes | `Kp = L·ω_bw`, `Ki = R·ω_bw`. AUTOCALIB prints a suggestion; **do not paste it** (§10) |
| M14 | Force per amp | leg exists | load cell; `F = 2·G·η·Kt·I` |

---

## 21. Per-joint calibration storage

**`joint_cal.h`** holds one hand-entered row per physical assembly; **`platformio.ini`** has one
environment per joint. Pick an environment; never edit a constant before a flash:

```
pio run -e J01 -t upload
```

`joint_cal.h` `#error`s if `JOINT_ID` is undefined. Rows are hand-entered (not auto-saved to
flash) so a human reviews every constant, each change is a dated git diff, and no flash write can
fail halfway.

**Row fields:** `zea`, `dir`, `R_eff`, `U0`, `Ke`, `L`, `vbus_scale`, `i_scale`, `drag_c_fwd/rev`,
`drag_v_fwd/rev` (positive magnitudes; the consumer applies sign(ω)), `breakaway_A`, serials, date,
belt state. `Kt` is derived (`calKt()` = 1.5·Ke), never stored. AUTOCALIB phase 7 emits a pasteable
row and **carries** `vbus_scale`, `i_scale` and `breakaway_A` from the flashed row (blocked, with
`*** CARRY BLOCKED ***`, if the flashed row is another joint's).

Which file a constant belongs in: the *Three homes* table in `CLAUDE.md`. `vbus_scale` is per
board (J01 0.008448, J02 0.008516, J03 0.008357). Unbuilt rows carry `0.0f` = not measured.

**Not stored per joint:** `T/T_loop` and `J_rotor` (fleet), current-loop gains (R and Ke spread
~2%), the INL 32-bin profile (goes to `docs/cal/*.csv`; the row comment carries pk-pk, 1/rev,
2/rev), force-per-amp (a leg property, M14).

### What invalidates what

| Scope | Constants | Invalidated by |
|---|---|---|
| Motor | `Ke`, `L`, cogging | rewinding, magnet replacement |
| Board | `vbus_scale`, `i_scale`, `U0` | board swap |
| The pairing | **`zea`**, **`dir`**, `R_eff`, INL, drag, `breakaway_A` | any motor ↔ board swap, or touching the magnet mount |

### Safe default for unfilled rows

`zea = -1.0f`, `dir = 0`: `runInitFOC()` falls back to a full alignment, and the banner prints
`!! UNCALIBRATED joint`.

### The `V` command — press it after every flash

`V` does one forced alignment, compares it to the stored `zea` (wrap-safe), then reinstalls the
stored value. A slipped magnet or a wrong-joint flash is otherwise silent.

| Result | Meaning |
|---|---|
| ≤ 8° elec | OK |
| > 8° | WARN — check the magnet mount |
| > 15°, or direction mismatch | FAIL — wrong joint's constants, or the magnet slipped |
