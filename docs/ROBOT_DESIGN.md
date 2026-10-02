# Robot-level design and battery specification

§17 robot-level design decisions · §19 battery specification.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 17. Robot-Level Design Decisions (frozen 2026-07-29, battery re-opened 2026-08-01)

Frozen on measured constants, not estimates. Re-open only with new bench data.

**Re-opened, with the data that did it:** the battery row. HG5511D datasheet (60 V), a bench run at 22.5 V, and the measured 34.2 V divider span together removed both objections to 6S. Everything else in this table stands.

| Parameter | Value | Reasoning |
|---|---|---|
| Target mass | **4.0 kg**, 12 DOF | Mass increase is nearly free for jumping — the limit is voltage, not torque |
| **Proximal link** | **80 mm** | L₁ sets stroke |
| **Distal link** | **100 mm** | **Longer distal links *reduce* stroke** (h → L₁cos α + L₂ asymptotically). Keep L₂/L₁ ≈ 1.25 |
| **Reduction** | **9:1 — unchanged** | Jump optimum is ~10:1; 9:1 within 2%. Optimal under both modulation assumptions |
| **Battery** | **5S or 6S — RE-OPENED 2026-08-01** | The old "6S buys 3% and needs a PB10 change" line was wrong on both counts. **No PB10 change is needed** (divider spans 34.2 V, §3) and the gain is not 3% — it is large, because the `R·I` toll is subtracted *before* anything buys speed. See §19 |
| **Belt** | **10 mm GT2** | Tooth load `Kt·I/r_pinion` = 208 N at 30 A on 6 mm — over the limit. Independent of gear ratio |
| Workspace | α ∈ [40°, 70°] | 40° of snap-through margin; reflected inertia worsens toward full extension |

**Resulting envelope:** 53.7 mm usable stroke, 93–147 mm working hip height, 513 N peak thrust (13.1× BW), **~478 mm jump apex**, backflip with ~2.8× angular margin (now a control problem, not a hardware one).

**Reflected inertia — now measured, and the estimate held.** `J_rotor` = **20.2 × 10⁻⁶ kg·m²** against the 2.1 × 10⁻⁵ this section had assumed: the estimate was **4.0% high**. Every figure below is the old one corrected down by 4%.

| | Estimated (2.1e-5) | **Measured (20.2e-6)** |
|---|---|---|
| Reflected mass per motor | 0.161 kg | **0.155 kg** |
| — per leg (×2 motors) | 0.323 kg | **0.311 kg** |
| — across the robot (×4 legs) | 1.292 kg | **1.243 kg** |
| Effective jumping mass (4.0 kg body) | 5.29 kg | **5.24 kg** |
| Reflected share of effective mass | 24.4% | **23.7%** |
| Reflected inertia at one joint | — | **1.64 × 10⁻³ kg·m²** (`J·N²`) |
| Rotor KE at takeoff (8 sagittal motors, 228 rad/s) | — | **4.20 J**, against ~13.5 J of body KE |

Scales as `J_rotor·(N/J̄)²`, and it is *worse* toward full extension where J̄ collapses.

**Accepted weakness:** swing clearance ≈ 34 mm, landing absorption over 54 mm. **This is a flat-ground sprinter and jumper, not a terrain robot.** An explicit choice.

> ### ⚠ OPEN: does the 478 mm apex figure use 4.0 kg or 5.24 kg?
>
> **This section states two things that may not be consistent with each other**, and the apex derivation is not in this repository so it cannot be settled by reading:
> 1. reflected inertia is **24% of effective mass** and "the design rests on it" — which implies an effective mass of ~5.24 kg;
> 2. the envelope quotes a **478 mm apex** without saying which mass it used.
>
> If (2) used 5.24 kg, everything is consistent and **478 mm stands** — the measurement then simply confirms it to 4%. If (2) used the 4.0 kg body mass alone, then on this section's own stroke-limited model `h ≈ F_max·s/(m·g) − s` with s = 53.7 mm:
>
> | | Apex |
> |---|---|
> | Quoted | 478 mm |
> | Corrected, using `h + s ∝ 1/m` (this section's model) | **352 mm** |
> | Corrected, using naive `h ∝ 1/m` | 365 mm |
>
> **Settle it by re-opening the apex spreadsheet and reading which mass it used.** Twenty minutes, no bench. Note that this is *not* the "measure `J_rotor` before buying a battery" item — that one is **closed**, and it closed in the estimate's favour.
>
> **TRIGGER: before the jump/gait controller's energy budget is written, or before 478 mm is quoted anywhere outside this repository.** Not "before the first jump" — **the controller design consumes this number earlier than the hardware does**, and 478 vs 352 mm is a 26% capability overstatement that will propagate into every energy and timing assumption built on it.
>
> Two further caveats stack on whichever number survives: the 478 mm figure **contains no sag term** (see below), and every force in this document carries an **unmeasured `i_scale`** (§8.2). Neither is new; both compound.

~~**Not measured yet, and the design rests on it:** `J_rotor` is estimated at 2.1×10⁻⁵ kg·m².~~ **CLOSED 2026-08-08 — measured at 20.2 ± 2.4 × 10⁻⁶ (±12%), three methods, two joints** (M6a driven step, plus two free coast-downs at 16.31 and 24.29 with a mean of 20.30). A driven measurement and a free-decay measurement landing on the same number is why the tolerance is as tight as it is; it was widened from ±10% to **±12%** when J02's determination came in, because the limiting error is the drag map rather than the fit (§8.2). It lives in `src/fleet_config.h` as `J_ROTOR_KGM2`, not in `JointCal` — it is geometric, ~1% between units.

### Peak bus current — corrected 2026-08-01

The earlier 150 A estimate was arithmetic error. At full modulation the bus voltage **cancels out**:

```
I_bus/motor = 1.5·Uq·Iq/(V_bus·η)   with Uq = V_bus/√3
            = 1.5·Iq/(√3·0.95) = 0.911·Iq
```

At `Iq` = 30 A: **27.3 A per motor, ×8 sagittal actuators = 219 A, +11 A servos/electronics = 230 A**, independent of pack voltage. Peak electrical power ≈ 3.9 kW, of which 2.35 kW is copper loss — **actuator efficiency at peak is ~40%.**

**The 478 mm apex figure above contains NO sag term.** A no-sag reconstruction gives 653 mm at 21.0 V and 358 mm at 18.5 V; 478 sits between them and assumes a terminal voltage that no realistic pack holds at 230 A. Treat 478 mm as an upper bound and use §19 for what a real pack delivers.

**Mass exchange rate, qualifying the "mass is nearly free" note above:** once force-limited at 30 A by belt tooth load, `h ≈ F_max·s/(m·g) − s`, so apex goes as **1/m**. 300 g of extra battery costs ~40 mm of apex; going from 45 mΩ to 25 mΩ of pack resistance buys ~190 mm. **The heavier, stiffer pack wins ~5:1. Spend the mass.**

---

## 19. Battery Specification (opened 2026-08-01)

**The binding constraint is sag, not energy.** One jump costs ~85 J = 0.024 Wh against a ~33–40 Wh pack — three orders of magnitude of margin. Heating is also a non-issue: 394 W dissipated inside the pack for 35 ms raises it 0.05 °C. **The pack is sized entirely by the voltage it retains at 230 A.**

### Why voltage leverages jump height so hard

```
Voltage needed = R_eff·Iq (a fixed toll, buys zero speed) + Ke·ω (buys speed)
               = 0.218 × 30 = 6.57 V                      + 0.0177·ω
Available      = V_bus/√3   (SVPWM ceiling)
```

The toll comes off the top, so sag eats the *remainder*, not the total — and apex goes as speed squared. **A 29% voltage loss becomes a 62% speed loss and an 86% apex loss.**

| Pack | Pulse R inc. wiring | Bus at 230 A | Ceiling | Foot speed at 30 A | **Apex** |
|---|---|---|---|---|---|
| 5S 2200 mAh 35C | ~53 mΩ | 8.8 V | 5.09 V | **cannot reach 30 A** | — |
| 5S 1800 mAh 110C ×1 | ~28.5 mΩ | 14.5 V | 8.34 V | 1.14 m/s | 66 mm |
| **5S 1800 110C ×2 parallel** | ~16.2 mΩ | 17.3 V | 9.97 V | 2.19 m/s | **244 mm** |
| **6S 1800 110C ×1** | ~33.4 mΩ | 17.5 V | 10.11 V | 2.28 m/s | **266 mm** |
| **6S 1800 110C ×2 parallel** | ~18.7 mΩ | 20.9 V | 12.07 V | 3.54 m/s | **639 mm** |

Note row 1: that pack's ceiling falls **below the 6.57 V toll**, so it cannot make 30 A flow at any speed. Fine for standing, walking, trotting; cannot jump at design force. **Every resistance figure above is an estimate — the RC3563 replaces them with measurements.**

### Target spec

| Parameter | Target | Rationale |
|---|---|---|
| Cells | **6S if commissioning passes, else 5S** | §3 shows no board change is needed either way |
| **Internal resistance** | **≤20 mΩ total pack** | *The* spec. Sets apex directly. Ask the seller (*内阻多少毫欧?*); if they cannot answer, it is not a high-C pack |
| Capacity | 1800–3000 mAh | <1800 → runtime impractical; >3000 → mass penalty beats the sag benefit |
| Advertised C | ≥100C @1800 mAh | Assume ~50% derate on advertised claims |
| Connector | **XT90 or direct 10 AWG solder** | XT60 is ~60 A cont / ~120 A burst — inadequate at 230 A |
| Trunk wire | 10 AWG minimum | Per-ESC branches at 30 A are fine on 16 AWG |
| Mass | ≤700 g (17.5% of 4 kg) | Exchange rate favours mass — §17 |

**Parallel pairs are the best value.** Doubling electrode area halves cell resistance, doubles capacity, and costs two cheap packs instead of one exotic one. Match voltage within ~0.1 V/cell before connecting (use a parallel board) and keep the packs the same model and age.

**Do not try to fix sag with capacitors.** 200 A for 35 ms at 1 V droop needs `C = I·Δt/ΔV` = **7 F**. Bulk caps handle switching ripple only.

### Measurement protocol (RC3563, four-wire, 1 kHz)

1. Charge to ~3.8 V/cell, rest 30 min, room temperature. Resistance depends on charge state and temperature — roughly doubles at 0 °C.
2. **Measure each cell individually through the balance lead, then sum.** A pack with one bad cell reads only slightly high overall, but that cell is what limits you and what fails first.
3. Add ~4 mΩ for wiring and connector.
4. **Multiply by ~1.4** to get the resistance that matters for a 35 ms pulse. The 1 kHz AC method sees only the fast ohmic part; charge-transfer and diffusion add more on the jump timescale. Factor is approximate and chemistry-dependent.
5. Re-measure every few months. Rising internal resistance is how a LiPo announces its death, long before capacity drops.

| Total pulse resistance | Verdict |
|---|---|
| ≤20 mΩ | Excellent — supports the full design jump |
| 20–30 mΩ | Good — roughly half design apex |
| 30–45 mΩ | Marginal — fine for trotting, weak on jumps |
| >45 mΩ | Cannot support the design operating point |

### 6S staged commissioning

Gates cleared: **FETs 60 V** (datasheet); **regulator, 3.3 V rail, bulk capacitance** (board demonstrably ran at 22.5 V); **ADC range** (34.2 V full scale). Not cleared: **electrolytic capacitor markings unreadable**, and **switching at 25.2 V under load untested**.

Abort at the first sign of anything warm that should not be:

1. Power only, no motor, 22.5 V, 10 min. Feel the electrolytics — they should be at ambient.
2. Power only, full 25.2 V, 10 min. **This is the capacitor test.**
3. Motor connected, open-loop, `VOLT_LIMIT = 1.0`, 30 s. First switching at 6S.
4. TORQUE(I) at 1.0 A, 60 s. **`|I|/Iq` must stay at 1.22–1.23.**
5. Only then run anything longer. **Re-measure U₀ at the new bus (§8.3).**

Expect switching overshoot around 1.3–1.5× the bus: 38 V worst case against 60 V breakdown = 63%. Acceptable margin, not a measurement.

---