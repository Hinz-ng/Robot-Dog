# Robot-level design and battery specification

§17 robot-level design decisions · §19 battery specification.

*Hub: [`README.md`](README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 17. Robot-level design decisions (frozen 2026-07-29)

Frozen on measured constants. Re-open only with new bench data. Pack choice: §24.3.

| Parameter | Value | Reasoning |
|---|---|---|
| Target mass | **4.0 kg**, 12 DOF (3 vs 4 kg: §24.2) | mass is nearly free for jumping; the limit is voltage |
| Proximal link | **80 mm** | L₁ sets stroke |
| Distal link | **100 mm** | longer distal links reduce stroke (h → L₁cos α + L₂); keep L₂/L₁ ≈ 1.25 |
| Reduction | **9:1** | jump optimum ~10:1; 9:1 within 2% |
| Battery | 5S or 6S (§19) | no board change either way (divider spans 34.2 V) |
| Belt | **10 mm GT2** | tooth load `Kt·I/r_pinion` = 208 N at 30 A exceeds a 6 mm belt; independent of ratio |
| Workspace | α ∈ [40°, 70°] | snap-through margin; reflected inertia worsens toward full extension |

**Envelope:** 53.7 mm usable stroke, 93–147 mm working hip height, 513 N peak thrust (13.1× BW;
487–597 N across the stroke since G varies 83–102 /m), backflip with ~2.8× angular margin.
**Jump apex ~478 mm is an upper bound:** it contains no pack-sag term (a no-sag reconstruction gives
653 mm at 21.0 V and 358 mm at 18.5 V; use §19 for real packs), and it is not stated whether it used
the 4.0 kg body or the 5.24 kg effective mass. If 4.0 kg, the stroke-limited model `h ≈ F·s/(m·g) − s`
gives **352 mm**. **Settle from the apex spreadsheet before the controller energy budget is written
or 478 mm is quoted outside this repo.**

**Reflected inertia** (`J_rotor` 20.2 × 10⁻⁶ kg·m²):

| | Value |
|---|---|
| Reflected mass per motor | 0.155 kg |
| per leg (2 motors) | 0.311 kg |
| across the robot | 1.243 kg |
| Effective jumping mass (4.0 kg body) | 5.24 kg (reflected share 23.7%) |
| Reflected inertia at one joint (`J·N²`) | 1.64 × 10⁻³ kg·m² |
| Rotor KE at takeoff (8 motors, 228 rad/s) | 4.20 J vs ~13.5 J body KE |

Scales as `J_rotor·(N/J̄)²`; worst at full extension. Leg geometry and the stroke-mean Jacobian
(G = 87.7 /m): `fleet_config.h`.

**Accepted weakness:** swing clearance ≈ 34 mm, landing absorption over 54 mm. A flat-ground sprinter
and jumper, not a terrain robot.

### Peak bus current

At full modulation the bus voltage cancels: `I_bus/motor = 1.5·Uq·Iq/(V_bus·η)` with `Uq = V_bus/√3`
→ 0.911·Iq. At Iq 30 A: 27.3 A per motor × 8 = 219 A, + 11 A servos/electronics = **230 A**,
independent of pack voltage. Peak electrical power ≈ 3.9 kW, 2.35 kW of it copper loss (~40%
actuator efficiency at peak).

**Mass exchange rate:** force-limited at 30 A, apex goes as 1/m. 300 g of extra battery costs
~40 mm of apex; 45 → 25 mΩ of pack resistance buys ~190 mm. **The heavier, stiffer pack wins ~5:1.**

---

## 19. Battery specification

**The binding constraint is sag, not energy.** One jump costs ~85 J (0.024 Wh) of a 33–40 Wh pack;
pack heating per jump is 0.05 °C. The pack is sized by the voltage it holds at 230 A.

```
Voltage needed = R_eff*Iq (fixed toll, buys no speed) + Ke*w (buys speed)
               = 0.218 * 30 = 6.57 V                  + 0.0177*w
Available      = V_bus / sqrt(3)   (SVPWM ceiling)
```

Sag comes out of the remainder after the toll, and apex goes as speed²: a 29% voltage loss → 62%
speed loss → 86% apex loss.

| Pack | Pulse R incl. wiring | Bus at 230 A | Ceiling | Foot speed at 30 A | Apex |
|---|---|---|---|---|---|
| 5S 2200 mAh 35C | ~53 mΩ | 8.8 V | 5.09 V | cannot reach 30 A | — |
| 5S 1800 110C ×1 | ~28.5 mΩ | 14.5 V | 8.34 V | 1.14 m/s | 66 mm |
| 5S 1800 110C ×2 parallel | ~16.2 mΩ | 17.3 V | 9.97 V | 2.19 m/s | 244 mm |
| 6S 1800 110C ×1 | ~33.4 mΩ | 17.5 V | 10.11 V | 2.28 m/s | 266 mm |
| 6S 1800 110C ×2 parallel | ~18.7 mΩ | 20.9 V | 12.07 V | 3.54 m/s | 639 mm |

Resistances are estimates until measured (protocol below).

### Target spec

| Parameter | Target | Rationale |
|---|---|---|
| Cells | 6S if commissioning passes, else 5S | |
| **Internal resistance** | **≤ 20 mΩ total pack** | sets apex directly. Ask the seller (*内阻多少毫欧?*) |
| Capacity | 1800–3000 mAh | below: runtime; above: mass beats sag benefit |
| Advertised C | ≥ 100C at 1800 mAh | assume ~50% derate |
| Connector | XT90 or direct 10 AWG solder | XT60 (~60 A cont) is inadequate at 230 A |
| Trunk wire | 10 AWG minimum | 16 AWG fine per ESC at 30 A |
| Mass | ≤ 700 g | |

Parallel pairs are the best value (half resistance, double capacity). Match voltage within ~0.1 V/cell
before connecting, same model and age. Capacitors cannot fix sag (200 A for 35 ms at 1 V needs 7 F).

### Measurement protocol (RC3563, four-wire, 1 kHz)

1. Charge to ~3.8 V/cell, rest 30 min, room temperature (resistance roughly doubles at 0 °C).
2. Measure **each cell** through the balance lead, then sum (one bad cell limits the pack).
3. Add ~4 mΩ for wiring and connector.
4. Multiply by ~1.4 for the 35 ms pulse resistance (1 kHz sees only the ohmic part).
5. Re-measure every few months; rising resistance is how a LiPo announces its end.

| Total pulse resistance | Verdict |
|---|---|
| ≤ 20 mΩ | full design jump |
| 20–30 mΩ | roughly half design apex |
| 30–45 mΩ | trotting fine, weak jumps |
| > 45 mΩ | cannot support the design point |

### 6S staged commissioning

Cleared: FETs 60 V (datasheet); regulator, 3.3 V rail, bulk capacitance (ran at 22.5 V); ADC range
(34.2 V). Not cleared: electrolytic capacitor markings unreadable; switching at 25.2 V under load.

Abort at the first sign of anything warm that should not be:
1. Power only, no motor, 22.5 V, 10 min. Electrolytics at ambient.
2. Power only, 25.2 V, 10 min. **The capacitor test.**
3. Motor connected, open-loop, `VOLT_LIMIT = 1.0`, 30 s.
4. TORQUE(I) at 1.0 A, 60 s. `|I|/Iq` must stay 1.22–1.23.
5. Then longer runs. Re-measure U0 at the new bus (M11).

Switching overshoot ~1.3–1.5× the bus: 38 V worst case vs 60 V breakdown (63%).
