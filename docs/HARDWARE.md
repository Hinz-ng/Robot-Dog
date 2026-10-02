# Hardware, board truths, tooling, thermal

§1 hardware · §1a the two assemblies · §2 clone vs genuine · §3 board pin truths ·
§14 ST tooling · §16 thermal ground rules.

CAN transport hardware (termination, `PB9`/`PA11`, transceivers) is in
[`CAN_BRINGUP.md`](CAN_BRINGUP.md) §23.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 1. Hardware

| Component | Part | Notes |
|---|---|---|
| Motor | TYI 4006, **KV360 (⚠ see the nameplate note below)**, 12N14P (7 pole pairs) | **Kt = 1.5 × Ke, derived per joint (`calKt()`): J01 0.026912, J02 0.027145 N·m per true A of Iq; Ke J01 0.017941, J02 0.018097 V/(rad/s).** Measured — see [`CONSTANTS.md`](CONSTANTS.md) §8.1. *(Was 0.0266 / 0.0177, A1's pre-M1 values — R26, 2026-10-01.)* Nameplate 18 A/60 s assumes **propeller airflow** — see §16. |
| Driver | B-G431B-ESC1 **clone** ("火柴"/Matches FOC V2.0) ×2 | Not a 1:1 copy — see §2. ~89 RMB. Seller lists 48 V capability. |
| MOSFETs | **HG5511D** ×6 | **60 V V(BR)DSS, 40 A cont (25 °C) / 25 A (100 °C), 120 A pulsed, R_DS(on) 11 mΩ @ V_GS = 10 V, R_θJA 35 °C/W.** Datasheet 2026-08-01. **Clears the FET gate for 6S** — 25.2 V bus with 1.5× switching overshoot = 38 V = 63% of breakdown. |
| **Bus voltage sense** | **PA0**, resistive divider | **PER BOARD — `B-SPI-01` 0.008448 · `B-ABZ-01` 0.008516 (M1 2026-08-18, UT89X), 0.80% apart.** Full scale **34.60 / 34.87 V**, 8.45 / 8.52 mV/count. ⚠ Supersedes the single **0.008358** this row carried, which came from a 2-point sweep (11.30 V → 1351, 22.50 V → 2694; ratios 1.991 vs 1.994) that was **1.1% low because of the METER** — a DT9205A with ~1.11% DCV gain error — **not the fit**, which back-predicted its own points to 0.07%. **6S at 25.2 V = 73% of range → no divider change needed.** Seed-only — see §12. |
| Reference driver | Genuine B-G431B-ESC1 | Acceptance test still not run. Has genuine onboard ST-Link. |
| Encoder | **MT6816** (AMR) on XJX-135 breakout | *Not* MT6701 — chip marking is ground truth. **Both live joints run 4-wire SPI: 14-bit, 16384 cnt/rev (0.022°).** The ABZ mode used on the retired A1 build was 1024 PPR ×4 = 4096 CPR (0.088°). ~~Possible angle latency~~ — **CLOSED, encoder excluded: 1 µs typ / 3 µs max propagation, ~3% of the observed budget (§8.3).** |
| Encoder interface | **A1 (retired): TIM4 hardware quadrature** (§5) · **J01 / J02: MT6816 4-wire SPI, bit-banged** | Zero interrupts, zero CPU. ~~cannot lose counts~~ — **RETRACTED, see §12:** TIM4 faithfully counts noise, and A1 lost ~60 counts under 8–11 A. Only the SPI path can *detect* corruption (parity). |
| Magnet | N35 **D6 × 3 mm** diametric, jig-centred, CA-bonded | **CLOSED.** Permanent mount; pen mark intact across many sessions. **No ferromagnetic material in/behind the field path.** History in §12. |
| Reduction | 9:1 GT2 belt, 12T alu pinion (r_pitch **3.82 mm**) → 108T printed pulley | **10 mm GT2 fitted 2026-08-12.** Belt **232 mm** pitch length; exact no-idler path at C = 44.5 mm is **230.96 mm**, so the take-up budget is **1.04 mm** (corrected 2026-09-02 — the 229.98 / 2.00 mm pair came from an approximate formula used outside its validity range, §22.2). Two back-side idlers. See §22. |
| **Idlers** | **2 × brass standoff, ~5 mm OD, on M3 threaded screws — FAULTY, being replaced** | **They slide instead of rolling and contribute 0.385 A = 58% of belt-on drag** (§22.1). 5 mm OD is 0.65× the 12T pitch diameter, roughly half the practical floor for a back-side idler. Rebuild spec in §22.2; awaiting 3 × 9 mm bearings. |
| Programmer | Clone ST-Link V2.1 | PlatformIO/OpenOCD ✓. **Never attempt ST FW upgrade — brick risk.** |
| Power | 3S LiPo (~11.3 V) on the bench; **5S frozen for the robot, 6S re-opened as an option** | Divider spans 34.2 V, so 5S *and* 6S both fit with no board change. **Board demonstrably powered and ran at 22.5 V** (2026-08-01, unloaded). Remaining untested gate: switching at 25.2 V under load — see §19. ⚠ `claude.md` freezes **5S** while §17/§19 treat 6S as live; see §24. |

**✅ CLOSED 2026-08-13 — the 1/rev disturbance is not motor-internal, and the motor swap is cancelled.** The discriminating test was run (A3, §9a): belt off, pinion off, folded against unwrapped `cnt`. On J02 the 1/rev amplitude is **0.0262 A at most against A1's 0.185 A**, `corr(Iq, vel)` fell from −0.78 to −0.042, and the folded velocity ripple fell from 8% to 0.3%. Referred to the foot that is **0.042–0.113 N, i.e. 0.4–1.1% of a standing leg load** — below the current-mode torque-ripple floor. **A1's 0.185 A was almost certainly the rubbing encoder magnet, not a bearing.** Full numbers in §9a.

*Historical: the note here previously read "open hardware issue — bearing whine, now quantified", with a motor swap costed at 60 RMB. Both the whine attribution (§5) and the 1/rev attribution turned out to be the same rubbing magnet. Third time that fault has explained a symptom it was not invented for.*

> **Still open, and it expires at B0: A3b.** One `L` capture at Uq = 1.30 V with the **pinion seated** isolates pinion bore runout from the motor. 10 minutes. After the belt goes on it can never be separated again.

## 1a. TWO ASSEMBLIES — read this before using any number below

| | **ORIGINAL** | **SPARE — the reference actuator** |
|---|---|---|
| Encoder | ABZ → TIM4 quadrature, 4096 cnt/rev | **MT6816 4-wire SPI, 16384 cnt/rev** |
| Status | **Fault DIAGNOSED (rubbing encoder magnet) and the assembly REBUILT AS J02.** A1 no longer exists as hardware. Every A1-era number carries an unknown mechanical friction term and is superseded by J02's row | **Accepted 2026-08-06.** All new work happens here |
| `R_eff` | 0.218 Ω (5 determinations) | **0.22346 Ω** (9-pt fit, §8.1a) |
| `U0` | 0.028 V | **0.01037 V** — but it does not reproduce, §8.1a |
| `vbus_scale` **(M1 2026-08-18)** | `B-ABZ-01` **0.008516** | `B-SPI-01` **0.008448** — **0.80% apart, measured.** ⚠ The ORIGINAL column's electrical numbers were all taken at 0.008358 and are therefore **1.89% low** on their own board. Not rescaled — historical record |
| ZEA | session-relative, sd 5.68° elec | **stored 6.0542 rad**, sd 3.32°, reproducible |
| Direction | CCW | **CW** |
| Armed loop | 13.5 kHz | **13.3 kHz** |

**The assemblies differ by +1.8% in `R_eff` and +0.9% in `Ke`** — small. ~~−9.3%~~ is **retracted**: it came from a 2-point fit on the SPARE (0.1977 Ω), which is exactly determined and therefore has no residual to expose a bad point. §12.

**What does *not* transfer at all is `zea` and `sensor_direction`** — completely different, and a wrong one either refuses to run or inverts torque. **Do not use an ORIGINAL constant on the SPARE or vice versa.** Every table below is labelled.

---

## 2. Clone vs. genuine (measured)

| | Genuine | This clone |
|---|---|---|
| Gate driver | 3× L6387D (no interlock, no internal dead-time) | **1× EG2124A** — interlock + internal dead-time + pulse filter, HIN/LIN active-high, VCC measured 9.5 V at the K36 node. **Confirmed in the vendor's own board photo** |
| MOSFETs | 6× STL180N6F7 | 6× HG5511D |
| **Shunts** | 3 mΩ | **3 mΩ — three of them, silkscreened `R003`, three-shunt low-side.** ⚠ **CORRECTED 2026-08-14 — see the box below** |
| **CAN transceiver** | — | **SIT1042QTK/3** (TJA1042T/3-class), on **VCC_5V**, `/3` = 3.3 V-compatible logic pins. §23.3 |
| **Termination** | — | **one R22 = 120 Ω ±1% 0603 hard-wired across CANH–CANL**, with test points TP4/TP12. §23.2 |
| MCU | STM32G431CB | **STM32G431CBU6** in UFQFPN48 (schematic symbol `STM32G431_49P` = 48 pins + exposed pad). Genuine ST silicon, date code CHN 2145, flashes normally |
| PCB | 4-layer | **6-layer immersion gold, 26 × 51 × 1.6 mm, single-sided placement, matte black** |

"IO-compatible" means external pads only. The firmware↔gate-driver contract differs — that was the root cause of the entire M0 saga.

> **The vendor's compatibility claim is false in exactly the way that cost the most.** The listing says *"perfectly compatible with B-G431B-ESC1 resources — all B-G431B-ESC1 examples can be used directly."* **They cannot.** SimpleFOC *latest* emits 6-PWM that violates the EG2124A input contract and produces a board that initialises, reports success, increments its angle and never moves (§4). The pad-level pinout is compatible; the gate-driver contract is not. Treat every "compatible" claim in this listing as *pinout* compatibility only.

> ### ⚠ CORRECTED 2026-08-14: the shunts are 3 mΩ, and the reasoning behind the right answer was wrong
>
> §2 and §7 previously said the clone uses **20 mΩ (`R020`) shunts with the amplifier gain scaled down to compensate**, and concluded *"therefore use the genuine constants."*
>
> **The vendor's board photo shows three shunts marked `R003` = 3 mΩ**, which is the *same* value as the genuine board. So the firmware constant is right:
>
> ```cpp
> LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
> //                  ^^^^^ matches the physical part, confirmed visually
> ```
>
> **The conclusion was correct and its justification was invented.** That is worse than a wrong number, because a wrong justification invites a future session to "fix" it: anyone who checked the 20 mΩ claim, found 3 mΩ, and followed the stated logic ("gain is scaled to compensate") would have *changed* a correct constant.
>
> **What this does to `i_scale`:** it removes the larger of the two unknowns. Current-sense scale is `R_shunt × G_amp`, and `R_shunt` is now confirmed to match the assumed value, so a 3.3× shunt mismatch is excluded by direct observation. Only the amplifier gain remains unverified, and **M2 stays at its existing priority — it is not escalated.**
>
> ⚠ **Do not be alarmed by the vendor's `RSHUNT 0.010` / `AMPLIFICATION_GAIN 5.18`** (§2a). That header's shunt value is falsified by the vendor's own photograph of the vendor's own board, so the header is a copied MCSDK template and is not a description of this hardware — including its gain figure. Recorded here so the discrepancy does not get rediscovered and treated as a finding.

---

## 2a. Vendor documentation — the 火柴 / HFOC listing (extracted 2026-08-14)

Everything in this section comes from the seller's product listing: a schematic page,
board photographs, a specification table and MCSDK/SimpleFOC screenshots. **Source rank
4 (vendor claim) except where noted**, so it is used to *narrow* unknowns and to give
bench tests something to aim at — never to overrule a measurement. Where the listing and
the bench disagree, the bench wins and the disagreement is recorded rather than resolved
silently.

> ### ⚠ VERSION MISMATCH — resolve before trusting the schematic
> The board in these photographs is silkscreened **`火柴 FOC V1.0`**. §1 of this doc set
> records the hardware as **"Matches FOC V2.0"**. **These may not be the same board.**
>
> This matters because the schematic page is now the sole source for `CAN_SHD = PC11`,
> `Temp_ADC = PB14` and the HSE routing — all of which feed S1b, S1c and the thermistor
> question. **Read the silkscreen on your own board and write the version in §1.** Two
> minutes. If it says V2.0, ask the seller for the V2.0 schematic before S1c.

### Driver-board specification, as claimed

| Parameter | Value | Note |
|---|---|---|
| MCU | STM32G431 | ✅ confirmed on the bench: `STM32G431CBU6` |
| Voltage range | **7.2 – 48 V** | ⚠ *"components are rated 60 V; exceeding 60 V may permanently damage the board"* — independently consistent with the measured HG5511D 60 V V(BR)DSS (§1) |
| **Max current** | **continuous < 10 A · instantaneous < 40 A** | *"must be used with good heat dissipation. This board is very small, mainly for learning, and is not recommended for high-current drive."* **See §16 — this collides with the design point** |
| **Max PWM frequency** | **50 kHz** | We run 25 kHz, so 2× headroom. Relevant if the FDCAN clock decision moves SYSCLK (§23.5) |
| Control interfaces | UART, CAN-FD, PWM, potentiometer, button | |
| **Sensor interfaces** | **ABZ(ABI), HALL, I2C — *"the three interfaces multiplex the same IO"*** | ✅ **This is the vendor confirming §3's silkscreen-modes-are-presets finding.** The pads are one set of IO with three alternate functions, not three independent ports |
| PCB | 6-layer immersion gold, 26 × 51 × 1.6 mm, single-sided placement | Useful for robot layout: 12 of these have to mount somewhere |
| Software | MCSDK, SimpleFOC, sensorless flux-linkage observer | The flux observer is irrelevant here — this rig has an absolute encoder |

### The companion dock — relevant even if unused

| Parameter | Value | Consequence |
|---|---|---|
| **Connector current limit** | **continuous < 3 A** | ⚠ *"above 3 A, solder the motor wires directly to the driver board."* **The design point is 30 A peak.** If any bench run has gone through the dock's screw terminals above 3 A, that is a connector thermal risk, not a rating to trust |
| Onboard ST-Link | SWD + virtual COM port + virtual mass storage | If this dock is in use it supersedes the clone ST-Link and its VREF quirk (§14). **Not confirmed present on this bench** |
| Connectors | 3.81 mm screw DC power 2-pin · 3.81 mm motor 3-pin · 2.54 mm sensor 5-pin · USB | |
| Enclosure / PCB | 3D-printed white shell, 2-layer PCB, same 60 V component note | |

### Vendor MCSDK power-stage parameters, and what to do with them

From `power_stage_parameters.h` in two of the listing's screenshots:

| `#define` | Value | Verdict |
|---|---|---|
| `VBUS_PARTITIONING_FACTOR` | 0.050561797752809 | **This is the PB10-HIGH (48 V) range** → full scale 3.3/0.0505618 = **65.3 V**. Our measured low range is 34.23 V (§1), a ratio of 1.907 — so the two ranges are *not* a clean factor of two |
| `NOMINAL_BUS_VOLTAGE_V` | 36 | Their example, not ours |
| `THREE_SHUNT` | — | ✅ confirmed, three `R003` parts |
| `RSHUNT` | 0.010 | ❌ **falsified by the vendor's own photo (`R003` = 3 mΩ).** A copied template |
| `AMPLIFICATION_GAIN` | 5.18 | ⚠ **unusable — it shares a header with a falsified shunt value.** See the §2 box |
| `V0_V` / `T0_C` / `dV_dT` / `T_MAX` | 1.4 V / 25 °C / **+0.019 V/°C** / 70 °C | ⚠ **contradicts our two bench readings — see §16** |

> **Their SimpleFOC example gains are not applicable.** The listing's `main.cpp` shows
> `PID_current_q.P = 1.0`, `.I = 200`, `LPF_current_q.Tf = 0.001`, `PID_velocity.P = 0.2`,
> `.I = 10`. Ours are `P = 0.1`, `I = 335`, `Tf = 0.00025`, measured against a step
> response on this plant (§10). A vendor default is a starting point for an unmeasured
> motor, and §10 is boxed against exactly this kind of substitution.

### Board pad map, read from the listing photographs

Silkscreen is on the **back** of the driver board. Verify against your own board before
wiring anything — this is a V1.0 photo (see the version box above).

| Edge | Pads, in order |
|---|---|
| Signal long edge | `GND · 3V · SWD · SCK · PWM · BUTTON · POTEN · TXD · RXD · GND · 5V · CANL · CANH` |
| Sensor short edge | `5V · GND · HC/Z · HB/B · HA/A` |
| Power | `7V~48V`, `GND` |
| Motor | `A/U · B/V · C/W` |

Also visible on the board: the potentiometer trimmer, the start button, the EG2124A, six
MOSFETs in a row, three `R003` shunts, and a shielded inductor marked `220` beside a
small regulator (**inferred** to be the 22 µH buck inductor for the 5 V rail — not
confirmed, and nothing depends on it).

> **The 6S electrolytic-capacitor gate is not cleared by this listing, but the risk is
> lower than "unknown".** §19 lists *"electrolytic capacitor markings unreadable"* as an
> uncleared gate. The vendor now claims the whole board is rated to 48 V with 60 V
> components, which bears directly on it — at **source rank 4**. A seller's rating does
> not clear a bench gate: **step 2 of the staged commissioning (full 25.2 V, 10 minutes,
> feel the electrolytics) still has to be run.** What changes is the prior, not the gate.

---

## 3. Board Pin Truths

- **`LED_BUILTIN` = PC6** (STATUS). **Not PB8** — a bare PB8 blink fails while `LED_BUILTIN` works.
- **PB8 = BOOT0.** High at reset → MCU enters the ROM bootloader and firmware never runs. **Never wire the encoder index (or anything that can idle high) to PB8.** We run without index.
- **The only exposed hardware UART is USART2 on PB3/PB4.** PB6/PB7 have no UART on this board — `HardwareSerial(PB7, PB6)` hangs the MCU. USB-CDC is impossible (PA12 is a phase pin).
- **Arduino-API hardware SPI on PB3/PB4/PB5 does not work on this variant.** `SPI.begin()` returns but the first `transfer16()` never does. Bit-banged MODE3 SPI on the same pins *does* work, proving chip + wiring were fine. Cause: the vendor-pruned `PeripheralPins_B_G431B_ESC1.c` pin map; a lookup miss lands in `Error_Handler()`, which is an infinite loop. **Do not re-fight this.**
- Same pruning killed `STM32HWEncoder` — it hangs inside `init()`. Solved by configuring TIM4 directly (§5).
- Silkscreen pad labels are *mode presets* (ABZ / SPI / I²C), not simultaneous functions.
- **Analogue pins identified by measurement** (four-channel probe, 3S then 6S):

| Pin | 11.30 V | 22.50 V | Identity |
|---|---|---|---|
| **PA0** | 1351 | 2694 | **VBUS divider.** Tracks the bus 1:1. ✅ Schematic net `VBUS_ADC` |
| PA1 | 155 | 155 | Current-sense op-amp input. ✅ Schematic net `Curr_fd1_OPAmp−` |
| PB14 | 1431 | **1267** | ~~Board thermistor (probable)~~ → ✅ **CONFIRMED: schematic net `Temp_ADC`.** But the *readings* still move the wrong way — see §16 |
| PB12 | 2126 | 2127 | ✅ **CONFIRMED: schematic net `SpeedBT_ADC`** — the potentiometer input. Parked at mid-rail because the trimmer is unused here |

### Complete pin map, from the clone schematic (2026-08-14)

Read off the vendor's `STM32G431_49P` schematic page. **Source rank 2 (a schematic in
hand), and every row marked ✅ is independently anchored by bench evidence**, which is
what makes the unanchored rows trustworthy: the anchors prove the wire-tracing is right.

| Net on the schematic | MCU pin | Confidence | Anchor / note |
|---|---|---|---|
| `CAN_TX` | **PB9** | ✅ certain | FDCAN1_TX AF9 is a silicon fact; PB5 has no FDCAN function, which is what fixes the fan-out direction |
| `CAN_RX` | **PA11** | ✅ certain | FDCAN1_RX AF9. **Retracts the BOOT0 collision hypothesis** — nothing in the CAN path touches PB8 |
| `CAN_SHD` | **PC11** | **high** | Position in a group anchored at both ends by `TXD2`=PB3, `RXD2`=PB4 and `SWCLK`=PA14. **Confirm with a GPIO toggle at S1b** — §23.3 |
| `H1/A` | **PB6** | ✅ certain | Our own ABZ wiring worked on PB6 = TIM4_CH1 |
| `H2/B` | **PB7** | ✅ certain | Our ABZ wiring, and the vendor's I2C example uses PB7 as SDA |
| `H3/Z` | **PB8** | ✅ certain | = **BOOT0.** The vendor's I2C example uses PB8 as SCL — see the trap below |
| `GPIO_BEMF` | **PB5** | high | Same fan-out. We use PB5 as the bit-banged MT6816 `CSN` and it works, so whatever BEMF network is on it does not load a GPIO output meaningfully |
| `TXD2` | **PB3** | ✅ certain | Bench: board TXD → ST-Link VCP RX |
| `RXD2` | **PB4** | ✅ certain | Bench: board RXD → VCP TX |
| `BUTTON` | **PC10** | high | Same group as `CAN_SHD` |
| `PWM` (input) | **PA15** | high | Same group |
| `SWCLK` / `SWDIO` | **PA14** / **PA13** | ✅ certain | Silicon |
| `STATUS` | **PC6** | ✅ certain | Bench: `LED_BUILTIN` = PC6 works, a bare PB8 blink does not |
| `Temp_ADC` | **PB14** | ✅ certain | Schematic + the probe sweep |
| `SpeedBT_ADC` | **PB12** | ✅ certain | Schematic + the probe sweep |
| `VBUS_ADC` | **PA0** | ✅ certain | Schematic + a 2-point calibration |
| `48V_EN` | **PB10** | ✅ certain | Schematic label, and the vendor's CubeMX screenshot configures `EN_48V` as push-pull output with pull-up, set HIGH |
| `TIM1_CH1/2/3` | PA8 / PA9 / PA10 | ✅ certain | Phase high sides |
| `TIM1_CH1N/2N/3N` | PC13 / PA12 / PB15 | ✅ certain | Phase low sides |
| `Curr_fd1/2/3_OPAmp±`, `BEMF1/2/3`, `OPAMPn_INT_GAIN` | PA1–PA7, PC4, PB0–PB2 | — | Current sense and back-EMF networks. The `INT_GAIN` nets are why the amplifier gain is a *board* property and not purely a firmware one — see the `i_scale` box in §2 |
| `32Ki` / `32Ko` (LSE) | pins 3 / 4 | ✅ certain | **Explicitly marked no-connect.** No 32.768 kHz crystal |
| `osci` / `osco` (HSE) | pins 5 / 6 | — | **Routed to nets that continue off the visible page — NOT marked no-connect, unlike the LSE pins.** This is the F1 blocker; see §23.5 for the decisive test |

> ### ⚠ The vendor's own recommended I2C wiring is a boot trap
> The listing suggests configuring the ABZ pads as I2C for an AS5600, and its example does
> exactly that: `I2Cone.begin(PB7, PB8, 800000)` — **PB7 = SDA, PB8 = SCL.**
>
> **PB8 is BOOT0.** I2C requires pull-ups on both lines, and a pull-up on BOOT0 holds it
> high at reset, which puts the MCU into the ROM bootloader so the firmware never runs
> (§3, §12). The vendor's suggested configuration therefore produces a board that
> programs fine and then appears dead on every subsequent power cycle.
>
> **We are not affected** — no AS5600, and PB8 is driven as the bit-banged SPI `SCK` with
> no pull-up. Recorded because it is the fourth distinct way this project has been bitten
> by PB8, and because it is a trap the vendor is actively recommending.

- **`PB10` = `48V_EN` switches the bus divider RANGE. Do not touch it — and now we know exactly what touching it would cost.** It is a *measurement* setting, not a power setting: it gates nothing and does not limit what the board can run on. `VBUS_PARTITIONING_FACTOR` is an **ST MCSDK** symbol that does not exist in this Arduino/SimpleFOC firmware — `VBUS_SCALE` does its job and is measured rather than derived.

| `PB10` | Partitioning factor | Bus full scale | Source |
|---|---|---|---|
| **as our firmware leaves it (undriven)** | **0.0964** | **34.23 V** | ✅ measured, 2 points vs a multimeter, back-predicts both to 0.07% |
| driven HIGH | 0.050562 | **65.3 V** | vendor listing (rank 4), derived full scale |

  Ratio 1.907 — **not a clean factor of two**, so the high range adds a resistor rather than switching one leg cleanly. The vendor instructs *"above 24 V, pull PB10 high and change the factor to 0.05056."* **That instruction is an MCSDK-configuration requirement, not a hardware one:** MCSDK ships an 8–28 V disable-PWM window, so their reading has to stay inside it. Our firmware has no such window, and the shipped range covers **6S at 25.2 V with 26% headroom.**

  **The real hardware rule is the ADC ceiling:** the low range reaches 3.3 V at the pin at a 34.23 V bus, so above that the reading clips and the pin's clamp diodes start conducting. **Stay on the low range up to ~34 V; above that, drive PB10 high and re-calibrate — expect roughly 1.9× the V/count.**

  > ⚠ **Silent-failure hazard, and it is a new one.** The 34.23 V calibration is only valid *with PB10 in the state our firmware leaves it.* Nothing in the firmware writes PB10 today, so it sits at its reset default. **Any future firmware that drives PB10 — or any port-wide GPIO init that catches it — moves `vbus_scale` by ~1.9× with no error and no symptom**, and `R_eff`, `U0` and `Ke` all scale linearly with `vbus_scale` (§8.1). Every constant in `joint_cal.h` would silently become wrong by a factor of two. **If Tier-0 firmware ever configures port B wholesale, PB10 needs an explicit "leave alone" comment at the init site.**
- **The ADC1 regular sequence is a shared resource, and its ORDER is load-bearing.** Read
  off the registers by the `p` probe on J02, 2026-08-21. SimpleFOC's `b_g431` low-side path
  puts everything on the **regular** group — there is no injected group on this board
  (§12).

| Rank | Channel | Pin | Net | `SMP` | What it is |
|---|---|---|---|---|---|
| **1** | `ADC1_IN12` | **PB1** | `OPAMP3_OUT` | 0 → 2.5 cyc | phase current |
| **2** | `ADC1_IN3` | **PA2** | `OPAMP1_OUT` | 0 → 2.5 cyc | phase current |
| 3 | `ADC1_IN11` | PB12 | `SpeedBT_ADC` | 4 → 47.5 cyc | pot, unpopulated |
| 4 | `ADC1_IN5` | PB14 | `Temp_ADC` | 4 → 47.5 cyc | board thermistor |
| 5 | `ADC1_IN1` | **PA0** | `VBUS_ADC` | 4 → 47.5 cyc | bus divider |

  ADC2 runs a single conversion — `ADC2_IN3` = **PA6** = `OPAMP2_OUT`, the third phase.
  Both instances are hardware-triggered every PWM period into **circular DMA, 16-bit
  transfers, right-aligned 12-bit data**: ADC1's 5-element buffer at `0x2000050A`, ADC2's
  1-element buffer at `0x20000508`. `ADSTART = 1` permanently on both.

  **Clock and calibration state, read off the running board 2026-08-21:**

| Register | Value | Meaning |
|---|---|---|
| `RCC->CCIPR.ADC12SEL` | 1 | Kernel source = **PLL "P"**, 170 MHz |
| `ADC12_COMMON->CCR` | `0x1C0000` | `CKMODE` = 00 (asynchronous), `PRESC` = 0b0111 (**÷16**) → **f_adc = 10.625 MHz** |
| whole ADC1 sequence | 210 cycles | **19.76 µs of a 40 µs PWM period — 50%** |
| `ADC1/2->CALFACT` | **`0`** | ⚠ **the converter is NEVER CALIBRATED by SimpleFOC** — see below |
| `OFR1..4`, `DIFSEL`, `CFGR2` | `0` | no offset registers, no differential channels, no oversampling, no `GCOMP` |

  > ⚠ **`CALFACT = 0` is a board truth with a 60-count consequence.** The Arduino core runs
  > `HAL_ADCEx_Calibration_Start()` inside **every** `analogRead()`; SimpleFOC's `b_g431`
  > init brings the converter up and never calibrates it. So any value read through the DMA
  > buffer comes off an **uncalibrated** converter and sits ~60 counts low, while a pre-init
  > `analogRead()` seed comes off a calibrated one. §0 has the confirmation and §12 the
  > consequence for VBUS.
  >
  > **It has never shown up in current data, and the reason is worth knowing:**
  > `currentSense.init()` measures its own zero offset *on the same uncalibrated converter*
  > and subtracts it, so the current path is differential and immune. The VBUS path is
  > absolute. That asymmetry is why this surfaced on PA0 and nowhere else.
  >
  > ~~⚠ **But the offset costs headroom.** 60 counts of zero shift on the phase channels is
  > **~1.9 A**…~~ **WITHDRAWN 2026-08-30 — the framing was wrong, not just the number.** The
  > phase zero point sits at 2556 on a *calibrated* converter; the uncalibrated 2482 was
  > reading 74 counts **low**. Positive headroom is `(4095 − 2556) × 0.02937` = **45.2 A, and
  > always was.** Nothing was ever lost; the previously-believed 47.4 A was overstated. 50%
  > margin at the 30 A design point either way (§0).
  >
  > **`HAL_ADC_DeInit` clears `CCR`** — confirmed on the bench 2026-08-30, `CCR_found = 0x0`.
  > Anything that calibrates between an `analogRead()` and `currentSense.init()` is doing so
  > in **asynchronous mode with `PRESC = 0`**, not at the 10.625 MHz the conversions run at.
  >
  > ⚠ **And in that window there may be no ADC clock at all.** `CKMODE = 00` takes the
  > asynchronous source named by `RCC->CCIPR.ADC12SEL`, and `ADC12SEL = 00` means **no clock
  > selected**. `ADCAL` is armed by software and **cleared only by hardware — there is no
  > cancel** — so arming it there does not fail, it sits **pending** and fires later, inside
  > whatever init next brings the clock up. That is a calibration whose clock *and* timing
  > are both accidental. **Measured 2026-08-30: `RCC->CCIPR.ADC12SEL = 0` at that point in
  > boot — there is genuinely no clock.** Calibrating the ADC earlier was attempted and
  > abandoned on exactly this; the code is deleted (§0). **`CALFACT = 0` on both instances
  > every boot is the normal, expected state of this board** — it is not a fault to fix.

  **`CALFACT` — measured behaviour on this part, 2026-08-30:**

| Property | Value | How it was determined |
|---|---|---|
| **Additive, 1 unit = 1 ADC count** | ratio **0.996** and **0.998** | Two determinations: `CALFACT` 0 → 117 moved the channel mean **+116.5**; 117 → 71 moved it **−45.9** |
| **Clock-dependent** | **117** at the as-found clock, **71** at 10.625 MHz | The two calibrations differed only in the `CCR` in force when they ran |
| Range | 7 bits, 0–127 | A code near the rail (117 = 92%) indicates a **bad** calibration, not a large offset |
| **Repeatability** | ⚠ **UNKNOWN — no noise floor has ever been measured** | See §8.3. A future "71 vs 65" comparison is uninterpretable until 5 power cycles have been logged |

  > ### ⚠ Silent-failure hazard — the current sample's position is set by the WHOLE sequence
  > Ranks 1–2 are the phase currents at the **shortest sample time the peripheral offers,
  > first in the sequence, immediately after the trigger.** Ranks 3–5 sit behind them and
  > cannot affect them. That is the only reason the current sample lands where the low-side
  > shunt reading is valid.
  >
  > **Anyone who reorders `SQR1`, or raises `SMP` on channel 3 or channel 12, moves the
  > current sample later inside the PWM window.** Nothing errors. Nothing looks wrong.
  > Every current-derived constant taken afterwards — `Kt`, `R_eff`, `U0`, the drag map —
  > is quietly wrong. **Same family as the `PB10` hazard above.**
  >
  > Three consequences that are not obvious from the table:
  >
  > - **Rank-to-rank residual charge is real, and it is ~1.6% — measured twice, on two
  >   different rank pairs.** Rank 4 (PB14, ~1230) sits ~13 counts above the affine offset
  >   model in both 2026-08-21 sessions, and the rank ahead of it is PB12 at ~2050:
  >   13 / (2050 − 1230) = **0.016**. Independently, the thermal run moved rank 4 by 60.7
  >   counts at a fixed bus while rank 5 (PA0) moved 1.0: **0.0165**. Two datasets, two
  >   methods, two different adjacent pairs, same coefficient. **This is not the
  >   charge-sharing explanation of the 62-count DMA offset** — that needed 47.4% and is
  >   dead (§12) — but it is non-zero, and it is the number any future argument about
  >   raising `SMP` on channel 1 has to beat, because a *shorter* sample time makes it
  >   larger. ⚠ **And PA0 is the least-coupled channel in the sequence purely by luck** —
  >   each channel's residual scales with how far its *predecessor rank* sits from it in
  >   value, and PA0's predecessor (PB14) happens to sit nearest. Measured 2026-08-30, in
  >   counts above the additive model: **ch1 +0.0, ch11 +3.6, ch5 +13.0**, ordering and scale
  >   both matching. **Reordering `SQR1` would remove that luck** and put the coupling term
  >   straight onto the bus reading.
  >
  > - **The two DMA buffers are adjacent allocations**, 2 bytes apart. Extending ADC1's
  >   sequence to 6 ranks would have the DMA write past the end of its own buffer into
  >   ADC2's. Adding a channel is **not available** without relocating SimpleFOC's buffer.
  > - **`SMPR1` writes require `ADSTART = 0`, and `ADSTART` is permanently 1.** Stopping the
  >   ADC without also resetting the DMA leaves `CNDTR` mid-count while the sequence
  >   restarts at rank 1 — **the phase currents then land in the wrong slots, permanently,
  >   with no error.** Any change needs `ADSTP` → disable DMA → write `SMPR1` → reload
  >   `CNDTR = 5` → re-enable DMA → `ADSTART`, done once at boot with everything disarmed
  >   and verified through the locked-rotor `|I|/Iq = 1.2247` gate.
- **`driver.pwm_frequency` reads back `-12345`** = SimpleFOC's `NOT_SET` sentinel. The platform default (25 kHz on STM32) is in force; the member is simply never written back. **Do not "fix" it by assigning a value** — you would be changing a variable to one you only believe is already active.

### CAN pins and termination — measured 2026-08-13

| Fact | Value | How known |
|---|---|---|
| `CAN_TX` | **PB9** (AF9) | Clone schematic |
| `CAN_RX` | **PA11** (AF9) | Clone schematic. ⚠ **This retracts a BOOT0 collision hypothesis** — `CAN_RX` does *not* touch PB8, so the BOOT0 trap is not in the CAN path |
| Onboard termination | **a single hard-wired ~121 Ω across CANH–CANL** | Measured **unpowered**. A firmware-switched terminator reads open with no power, so 121 Ω unpowered means a physical resistor is permanently fitted |
| Transceiver identity | **unmarked; not confirmed as TCAN330** | Unknown until S1b (§23) |

> ⚠ **`CAN_TERM` as a firmware-controlled terminator does not exist on this clone.** Project memory said "TCAN330 onboard, firmware-controlled termination"; bench evidence says otherwise and wins. **Consequence: 10 of 12 boards need their termination resistor physically removed** before any bus with more than two nodes. Arithmetic and the follow-up measurement are in §23.

### ~~Nameplate conflict — KV360 vs KV380~~ RESOLVED 2026-08-18 — KV360

Project memory recorded the motor label as **KV380**; every `Kt` cross-check in this doc set uses **KV360**. **Closed to KV360.** A seller listing plus two independent bench `Kt` determinations outrank a photographed silkscreen — a nameplate print is a vendor claim (tier 3 on the source-of-truth order in `CLAUDE.md`), and it lost to arithmetic that had already been done:

| Assumed KV | `Kt = 60/(2π·KV)` | vs measured |
|---|---|---|
| **360** | 0.026526 | **+1.0% to +1.1%** at the M1-corrected scale (see the amendment below) — was −0.06% / +0.03% |
| 380 | 0.025132 | off by **~6.6%**, both runs — ruled out, and the margin only widened |

`Kt` is *measured* as 1.5·`Ke`, so nothing downstream ever depended on KV — this was only ever a sanity check, and it still passes at KV360. **Do not reopen this on a photograph; a nameplate misprint is the standing explanation for the KV380 figure in memory.**

> #### ⚠ AMENDED 2026-08-18 — the verdict stands, the evidence is weaker than it was written
> **The conclusion does not change: KV360, by a margin of ~5.5 percentage points over KV380.** Two things in the paragraph above do.
>
> **① The agreement figures were computed at a `vbus_scale` that was 1.1% low.** M1 corrected `B-SPI-01` to 0.008448, and every `Kt` on that board moved ×1.010768: the two teardown determinations land at **≈0.026796 (+1.02%) and ≈0.026820 (+1.11%)**, not −0.06% and +0.03%. **"The tightest agreement this project has recorded" is withdrawn** — it was the coincidence of a low divider, and the KV380 figure moves from ~5.5% to ~6.6% by the same factor.
>
> **② A `Kt`-vs-nameplate comparison can never confirm `vbus_scale`, only the KV.** `Ke` is computed *from* `vbus_scale`, so using this agreement to validate the voltage scale is circular — that use is retracted everywhere it appeared (§8.1a in [`CONSTANTS.md`](CONSTANTS.md), the `joint_cal.h` row comments, and AUTOCALIB's printed report). It discriminates a 5.5% nameplate error comfortably; it cannot see a 1% divider error at all.

---

## 14. ST Tooling & Clone Hardware

- Clone ST-Link: PlatformIO/OpenOCD ✓; CubeProgrammer ✗; FW upgrade ✗ (**abort — brick risk**); Motor Pilot over the clone VCP not achievable.
- MCSDK path on genuine hardware: 6.3.1 + Motor Pilot 1.2.11.

---

## 16. Thermal Ground Rules

- The binding constraint is **motor copper**: `P_cu ≈ 1.5·R_ph·I²` with R_ph ≈ 0.218 Ω **on the ORIGINAL assembly** (**0.22346 Ω on the SPARE/J01**, M1-corrected 2026-08-18 — the ~0.198 figure was the retracted 2-point fit, §1a). 1.5 A ≈ 0.7 W; 4.5 A ≈ 6.6 W; 8.7 A ≈ 25 W; 19 A ≈ 118 W.
- Vendor 18 A/60 s assumes propeller airflow. At stall there is no airflow → derate hard; >5 A sustained stall is instrumented-only.
- **Open-loop mode applies `voltage_limit` directly with no current limit.** At `VOLT_LIMIT = 2.0` that is 8.6 A / 25 W with no throttle. Keep open-loop runs short or drop the limit to 1.0 for them.
- Switching ripple contributes real RMS heating even at zero average current.
- Bus current ≠ phase current: the inverter is a buck converter.
- **Hot-vs-cold R_eff has not been measured.** Copper is +0.39%/K. If `R_hot/R_cold > 1.15`, peak force sags during a jump sequence. 10-minute test whenever convenient.
- **Hot-vs-cold `R_eff` is now measurable WITHOUT a temperature sensor.** AUTOCALIB phase 3 takes 6 s: run it cold, load the motor for 60 s, run it again immediately. `ΔR/R > 15%` means peak force sags during a jump sequence. This retires the thermistor as a blocker — see §8.3 and §20.3 M10.
> ### ⚠ THE VENDOR RATES THIS BOARD BELOW THE DESIGN OPERATING POINT
>
> Added 2026-08-14 from the listing (§2a). This is the first time the board itself, rather than the FETs, has been the binding thermal constraint on paper.
>
> | | Vendor rating | Design point (§17) | Ratio |
> |---|---|---|---|
> | Continuous | **< 10 A** | sprinting is closer to continuous than to pulsed | — |
> | Instantaneous | **< 40 A** | **30 A peak per motor** | **75% of the absolute limit** |
> | FET capability (measured) | 40 A cont / 25 A at 100 °C / 120 A pulsed | 30 A | the FETs are *not* the limit |
>
> And the vendor's own words: *"this board is very small, mainly used for learning, and is **not recommended for high-current drive**."*
>
> **What this does and does not change.** It does not change a measured number — nothing in §8 is affected, and the FET gate for 6S still passes on the HG5511D datasheet. What it changes is the risk register for the *robot*: twelve of these boards, at 30 A peak, on a 26 × 51 mm 6-layer card the manufacturer says is for learning. The 40 A figure is a vendor claim (rank 4) with no stated test condition, no duration and no ambient.
>
> **The honest position:** a ~35 ms jump pulse at 30 A is plausibly inside "instantaneous"; a sprint gait is not obviously inside "continuous < 10 A". **This is not a reason to change hardware and it is not something to reason further about — it is a reason to instrument.** The measurement that settles it is M10 (hot-vs-cold `R_eff`, now 6 s per run) plus a fingertip on the FETs after a realistic duty cycle, taken once the belt is on. Logged in §24 so it is not rediscovered as a surprise at fleet scale.

- **The PB14 thermistor transfer function the vendor publishes disagrees with our own two readings.** `Temp_ADC = PB14` is now **confirmed** by the schematic (§3), which retires the "probable". But the vendor's MCSDK header gives `V = V0 + dV/dT·(T − T0)` with **`V0` = 1.4 V at 25 °C and `dV/dT` = +0.019 V/°C** (`T_MAX` 70 °C) — a channel whose voltage *rises* with temperature. Our two bench readings went **1431 → 1267 counts (1.153 → 1.021 V) as the board warmed**: the wrong direction, and both values imply ~5–12 °C on the vendor's own formula. **Do not adopt the transfer function.** Either the clone's NTC sits in the opposite divider leg from the config it ships, or those two readings are invalid — and they are prime suspects, because they were taken in separate sessions and this project has already caught one frozen-ADC artefact on exactly this peripheral (§12). **The discriminating test is a deliberate one-session warm-up: hold 1.5 A for two minutes and watch PB14 continuously.** A monotonic rise vindicates the vendor's formula; a monotonic fall means the polarity is inverted and the formula needs its sign flipped before use; no movement means the channel is not being sampled at all. Until then PB14 remains an unconverted raw count, which is all §16 ever needed it to be.

  > ### ⚠ 2026-08-21 — a second dataset has PB14 RISING, which is the vendor's sign
  >
  > The discriminating test named above has now been run in the only form currently
  > available: fixed bus, board powered and left to self-heat, PB14 watched through the
  > **DMA path** (rank 4 of ADC1's sequence, §3) rather than `analogRead`. Result:
  > **1367 → 1427 counts over ~2 minutes — a monotonic RISE.** By §16's own stated rule,
  > a monotonic rise vindicates the vendor's positive coefficient.
  >
  > **The "do not adopt" verdict stands, but what it rests on has inverted.** The evidence
  > against the vendor was the 1431 → 1267 pair; that pair is now the *disputed* half:
  >
  > - It was taken **across two sessions**, which is the frozen-ADC artefact shape §12
  >   catalogues on this exact peripheral — two different boots compared as one run.
  > - It was taken through **`analogRead`**, the path §12 now forbids after
  >   `currentSense.init()`. Whether those readings were live at all is unverified.
  > - The new run is one session, seconds-resolved, on a path that is demonstrably live.
  >
  > **What is still missing before adoption:** this was board self-heating, not the
  > specified 1.5 A load, so it is a smaller and slower excursion than the test called for,
  > and it is one dataset. Run the 1.5 A version and the question closes either way.
  > **Anyone re-opening this should treat the cross-session pair as the suspect number.**
  >
  > ⚠ **This also matters for something that is not the thermistor.** PB14 moving 60 counts
  > in 2 minutes at a *fixed bus* is the measurement that killed the charge-sharing
  > hypothesis for the VBUS DMA offset — PA0 moved 1.0 count over the same swing (§12).
  > A thermally live PB14 is a useful instrument precisely because it moves.

- **PB14 is a board thermistor channel** and is already wired — 1431 → 1267 counts as the board warmed between two sessions. It makes the hot-vs-cold test nearly free. **Caveat: it measures the BOARD, not the winding.** The binding constraint is motor copper, so treat PB14 as an indicator, not a substitute for measuring `R_eff` warm. Confirm the identification first: run at 1.5 A for two minutes and watch it — ~~fall further~~ **see the 2026-08-21 box above; the one continuous run available has it rising.** ~~**Reading it requires the register-level ADC work in §8.3**~~ — **corrected 2026-08-21: reading it requires no ADC work at all.** PB14 is already rank 4 of ADC1's regular sequence and already lands in the DMA buffer every PWM period (§3), so it is a RAM read. `analogRead()` is still unusable — and now forbidden — after `currentSense.init()` (§12), but that was never the only way to reach the channel.

---