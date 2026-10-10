# Hardware, board truths, tooling, thermal

§1 hardware · §1a assemblies · §2 clone vs genuine · §2a vendor documentation · §3 board pin
truths · §14 ST tooling · §16 thermal · §16a logic supply and idle current. CAN hardware: [`CAN_BRINGUP.md`](CAN_BRINGUP.md) §23.

*Hub: [`README.md`](README.md). Master table: [`CONSTANTS.md`](CONSTANTS.md) §8.*

---

## 1. Hardware

| Component | Part | Notes |
|---|---|---|
| Motor | TYI 4006, **KV360**, 12N14P (7 pole pairs) | Kt = 1.5 × Ke per joint (`calKt()`): J01 0.026912, J02 0.027145 N·m/A_true (§8.1). Nameplate 18 A/60 s assumes propeller airflow (§16). A photographed nameplate reads KV380; the bench rules it out (KV380 is ~6.6% off measured Kt, KV360 +1.0–1.1%). Do not reopen on a photo |
| Driver | B-G431B-ESC1 **clone** ("火柴" FOC; silkscreen version unconfirmed, §24.7) | not a 1:1 copy (§2). ~89 RMB |
| MOSFETs | **HG5511D** ×6 | 60 V V(BR)DSS, 40 A cont (25 °C) / 25 A (100 °C), 120 A pulsed, R_DS(on) 11 mΩ at 10 V, R_θJA 35 °C/W. 6S at 25.2 V with 1.5× overshoot = 63% of breakdown |
| Bus voltage sense | **PA0**, resistive divider | **per board:** `B-SPI-01` 0.008448, `B-ABZ-01` 0.008516 (M1, UT89X). Full scale 34.60 / 34.87 V. 6S at 25.2 V = 73% of range. Seed-only (§12) |
| Reference driver | genuine B-G431B-ESC1 | acceptance test not run |
| Encoder | **MT6816** (AMR) on XJX-135 breakout | 4-wire SPI, 14-bit, 16384 cnt/rev (0.022°), bit-banged. Propagation 1 µs typ / 3 µs max |
| Magnet | N35 D6 × 3 mm diametric, jig-centred, CA-bonded | no ferromagnetic material in or behind the field path. Gap 0.5–1.0 mm |
| Reduction | 9:1 GT2 10 mm belt, 12T alu pinion (r_pitch 3.82 mm) → 108T printed pulley | belt 232 mm, C = 44.5 mm, take-up 1.04 mm (§22) |
| Idlers | 2 × 3×9×5 ZZ bearings per side, fixed holes (1.72, ±10.00) | spec §22.2 |
| Programmer | clone ST-Link V2.1 | PlatformIO/OpenOCD ✓. **Never attempt an ST firmware upgrade — brick risk** |
| Power | 3S LiPo (~11.3 V) bench; 5S frozen for the robot (6S: §24.3) | divider spans 34.2 V, so 5S and 6S fit unchanged. Board ran at 22.5 V unloaded (2026-08-01); 25.2 V under load untested (§19) |

## 1a. Assemblies

Joint-by-joint status and constants: [`CONSTANTS.md`](CONSTANTS.md) §8. `zea` and
`sensor_direction` never transfer between joints; a wrong one refuses to run or inverts torque.

---

## 2. Clone vs genuine (measured)

| | Genuine | This clone |
|---|---|---|
| Gate driver | 3× L6387D (no interlock, no internal dead-time) | **1× EG2124A**: interlock, internal dead-time, pulse filter, HIN/LIN active-high, VCC 9.5 V at K36 |
| MOSFETs | 6× STL180N6F7 | 6× HG5511D |
| Shunts | 3 mΩ | **3 mΩ ×3 (`R003`), three-shunt low-side** — firmware `LowsideCurrentSense(0.003f, -64.0f/7.0f, …)` is correct |
| CAN transceiver | — | SIT1042QTK/3 on VCC_5V (§23.3) |
| Termination | — | R22 120 Ω ±1% 0603 hard-wired across CANH–CANL (§23.2) |
| MCU | STM32G431CB | STM32G431CBU6, UFQFPN48, genuine ST silicon |
| PCB | 4-layer | 6-layer immersion gold, 26 × 51 × 1.6 mm, single-sided |

"Compatible" means pad-level pinout only. The firmware ↔ gate-driver contract differs: SimpleFOC
*latest* emits 6-PWM that violates the EG2124A input contract (board initialises, reports success,
never moves; §4). The vendor's "all B-G431B-ESC1 examples work directly" is false.

---

## 2a. Vendor documentation (火柴 / HFOC listing, rank 4 — narrows unknowns, never overrules a measurement)

The listing photographs a board silkscreened **V1.0**; the schematic is the sole source for
`CAN_SHD` = PC11, `Temp_ADC` = PB14 and the HSE routing. Read your board's silkscreen (§24.7).

| Parameter | Claimed | Note |
|---|---|---|
| Voltage range | 7.2–48 V, components rated 60 V | consistent with HG5511D 60 V |
| **Max current** | **continuous < 10 A, instantaneous < 40 A**, "not recommended for high-current drive" | below the 30 A design point (§16) |
| Max PWM | 50 kHz | we run 25 kHz |
| Sensor interfaces | ABZ, HALL, I2C multiplex the same IO | silkscreen pad labels are mode presets |
| Interfaces | UART, CAN-FD, PWM, potentiometer, button | |

**Companion dock (not confirmed in use):** connector limit **< 3 A continuous** ("above 3 A solder
motor wires directly"); onboard ST-Link with SWD + VCP + mass storage.

**Vendor MCSDK `power_stage_parameters.h`:**

| `#define` | Value | Verdict |
|---|---|---|
| `VBUS_PARTITIONING_FACTOR` | 0.050562 | the PB10-HIGH range (65.3 V full scale) |
| `RSHUNT` | 0.010 | ❌ wrong (photo shows `R003`); the header is a copied template |
| `AMPLIFICATION_GAIN` | 5.18 | unusable (same template) |
| `V0_V` / `dV_dT` / `T_MAX` | 1.4 V at 25 °C / +0.019 V/°C / 70 °C | not adopted (§16) |

Vendor SimpleFOC example gains (`P = 1.0, I = 200`) do not apply; ours are measured (§10).

**Pad map** (silkscreen on the back): signal edge `GND · 3V · SWD · SCK · PWM · BUTTON · POTEN ·
TXD · RXD · GND · 5V · CANL · CANH`; sensor edge `5V · GND · HC/Z · HB/B · HA/A`; power `7V~48V`,
`GND`; motor `A/U · B/V · C/W`.

The 6S electrolytic-capacitor gate (§19) is not cleared by the vendor's rating; run step 2 of the
staged commissioning.

---

## 3. Board pin truths

- **`LED_BUILTIN` = PC6** (STATUS), not PB8.
- **PB8 = BOOT0.** High at reset → ROM bootloader, firmware never runs. Never wire anything that
  can idle high (encoder index, an I2C pull-up) to PB8. The vendor's AS5600 example
  (`I2Cone.begin(PB7, PB8, …)`) is exactly this trap.
- **The only exposed hardware UART is USART2 on PB3/PB4.** `HardwareSerial(PB7, PB6)` hangs the MCU.
  USB-CDC is impossible (PA12 is a phase pin).
- **Arduino-API hardware SPI and `STM32HWEncoder` hang on this variant** (pruned
  `PeripheralPins_B_G431B_ESC1.c`; a lookup miss lands in `Error_Handler()`). Bit-banged SPI works.
  Do not re-fight this.
- **`driver.pwm_frequency` reads back −12345** (`NOT_SET`); the 25 kHz platform default is in
  force. Do not assign a value to "fix" it.

### Pin map (clone schematic, 2026-08-14)

| Net | MCU pin | Confidence / anchor |
|---|---|---|
| `CAN_TX` / `CAN_RX` | PB9 / PA11 (AF9) | certain; AF9 proven at S1c |
| `CAN_SHD` (transceiver `S`) | PC11 | measured at S1b |
| `H1/A`, `H2/B`, `H3/Z` | PB6, PB7, PB8 (= BOOT0) | certain |
| MT6816 SPI | PB5 = CSN, PB6 = MOSI, PB7 = MISO, PB8 = SCK | bench |
| `GPIO_BEMF` | PB5 | high; does not load PB5 as CSN |
| `TXD2` / `RXD2` | PB3 / PB4 | bench |
| `BUTTON` / `PWM` input | PC10 / PA15 | high |
| `SWCLK` / `SWDIO` | PA14 / PA13 | silicon |
| `STATUS` LED | PC6 | bench |
| `VBUS_ADC` | PA0 | schematic + calibration |
| `Temp_ADC` | PB14 | schematic + probe |
| `SpeedBT_ADC` (pot) | PB12 | schematic + probe (parks mid-rail, unused) |
| `48V_EN` | PB10 | schematic |
| TIM1_CH1/2/3 (high side) | PA8 / PA9 / PA10 | certain. **MCO is PA8 — never use MCO** |
| TIM1_CH1N/2N/3N (low side) | PC13 / PA12 / PB15 | certain |
| Current sense / BEMF / `OPAMPn_INT_GAIN` | PA1–PA7, PC4, PB0–PB2 | amplifier gain is a board property |
| LSE (pins 3/4) | no-connect | — |
| HSE (pins 5/6) | 8.000 MHz crystal fitted | §23.7 |

### PB10 = `48V_EN` switches the bus divider range — leave it alone

| PB10 | Factor | Bus full scale |
|---|---|---|
| **as firmware leaves it (undriven)** | 0.0964 | **34.23 V** (measured) |
| driven HIGH | 0.050562 | 65.3 V (vendor) |

Ratio 1.907. The vendor's "above 24 V pull PB10 high" is an MCSDK requirement, not a hardware one;
the low range covers 6S with 26% headroom. Stay on the low range up to ~34 V.

> ⚠ **Silent failure:** any firmware that drives PB10 (including a port-wide GPIO init) moves
> `vbus_scale` ~1.9× with no error, and `R_eff`, `U0`, `Ke` scale with it. If Tier-0 configures
> port B wholesale, PB10 needs an explicit leave-alone comment at the init site.

### ADC1 regular sequence (read off the registers, J02, 2026-08-21)

SimpleFOC's `b_g431` low-side path uses the **regular** group only (no injected group).

| Rank | Channel | Pin | Net | `SMP` |
|---|---|---|---|---|
| 1 | `ADC1_IN12` | PB1 | `OPAMP3_OUT` (phase current) | 2.5 cyc |
| 2 | `ADC1_IN3` | PA2 | `OPAMP1_OUT` (phase current) | 2.5 cyc |
| 3 | `ADC1_IN11` | PB12 | `SpeedBT_ADC` | 47.5 cyc |
| 4 | `ADC1_IN5` | PB14 | `Temp_ADC` | 47.5 cyc |
| 5 | `ADC1_IN1` | PA0 | `VBUS_ADC` | 47.5 cyc |

ADC2: one conversion, `ADC2_IN3` = PA6 = `OPAMP2_OUT`. Both hardware-triggered every PWM period
into circular DMA (16-bit, right-aligned): ADC1 buffer `0x2000050A` (5 elements), ADC2 `0x20000508`.
`ADSTART = 1` permanently.

| Register | Value | Meaning |
|---|---|---|
| `RCC->CCIPR.ADC12SEL` | 1 | PLL "P", 170 MHz |
| `ADC12_COMMON->CCR` | `0x1C0000` | async, ÷16 → f_adc 10.625 MHz |
| Whole ADC1 sequence | 210 cycles | 19.76 µs of the 40 µs PWM period |
| `ADC1/2->CALFACT` | **0** | **never calibrated by SimpleFOC — normal for this board** |
| `OFR1..4`, `DIFSEL`, `CFGR2` | 0 | |

- **`CALFACT = 0`:** `analogRead()` calibrates on every call; SimpleFOC never does. The DMA path
  reads ~60 counts low on absolute channels (VBUS); the current path is immune (its zero offset
  is measured on the same converter). Positive current headroom is 45.2 A.
- **The ADC cannot be calibrated before `currentSense.init()`:** `ADC12SEL = 0` there (no kernel
  clock). `ADCAL` armed without a clock sits pending and fires later inside init, writing a
  plausible wrong factor. `HAL_ADC_DeInit` clears `CCR`.
- **`CALFACT` behaviour:** additive, 1 unit = 1 count; clock-dependent (117 at the as-found clock,
  71 at 10.625 MHz); 7-bit; repeatability never measured (§8.3).

> ⚠ **Silent failure: the sequence order is load-bearing.** Ranks 1–2 are the phase currents at the
> shortest sample time, first after the trigger. Reordering `SQR1` or raising `SMP` on channels 3
> or 12 moves the current sample later in the PWM window — no error, and every current-derived
> constant afterwards is wrong.
> - Rank-to-rank residual charge is ~1.6% (measured twice). PA0 is least coupled only because its
>   predecessor (PB14) sits nearest in value.
> - The two DMA buffers are 2 bytes apart: a 6th rank would overwrite ADC2's buffer.
> - `SMPR1` writes need `ADSTART = 0`. Stopping the ADC without resetting DMA leaves `CNDTR`
>   mid-count and puts the phase currents in the wrong slots permanently. Any change: `ADSTP` →
>   disable DMA → write `SMPR1` → reload `CNDTR = 5` → re-enable DMA → `ADSTART`, at boot, disarmed,
>   verified by the locked-rotor `|I|/Iq = 1.2247` gate.

**Analogue pins by probe sweep** (counts at 11.30 / 22.50 V): PA0 1351 / 2694 (VBUS) · PA1 155 /
155 (op-amp input) · PB14 1431 / 1267 (Temp_ADC; see §16) · PB12 2126 / 2127 (pot).

---

## 14. ST tooling and clone hardware

- Clone ST-Link: PlatformIO/OpenOCD ✓; CubeProgrammer ✗; firmware upgrade ✗ (**brick risk**);
  Motor Pilot over the clone VCP not achievable.
- MCSDK path on genuine hardware: 6.3.1 + Motor Pilot 1.2.11.

---

## 16. Thermal ground rules

- The binding constraint is **motor copper**: `P_cu ≈ 1.5·R_ph·I²`, R_ph ≈ 0.223 Ω. 1.5 A ≈ 0.7 W;
  4.5 A ≈ 6.6 W; 8.7 A ≈ 25 W; 19 A ≈ 118 W.
- Vendor 18 A/60 s assumes propeller airflow. At stall there is none: derate hard; > 5 A sustained
  stall is instrumented-only.
- **Open-loop mode applies `voltage_limit` with no current limit.** At `VOLT_LIMIT = 2.0` that is
  8.6 A / 25 W. Keep open-loop runs short or drop the limit to 1.0.
- Switching ripple heats even at zero average current. Bus current ≠ phase current (the inverter
  is a buck converter).
- **Hot vs cold `R_eff` (M10):** phase 3 cold → load 60 s → phase 3 again (6 s per run).
  ΔR/R > 15% means peak force sags during a jump sequence. Copper +0.39%/K.
- **Board rating below the design point** (§2a, §24.9): < 10 A continuous / < 40 A instantaneous
  vs 30 A peak per motor. The FETs are not the limit. Instrument it: M10 plus FET temperature after
  a realistic duty cycle.
- **PB14 thermistor:** measures the board, not the winding. Vendor formula not adopted. A single
  continuous DMA-path session (self-heating, fixed bus) read **1367 → 1427 counts, rising** (the
  vendor's sign); the earlier falling pair was cross-session through `analogRead` and is suspect.
  Close it with one continuous run at 1.5 A for 2 min watching PB14 (§24.8). Read it from the DMA
  buffer (rank 4); `analogRead()` is forbidden after `currentSense.init()` (§12).

### 16a. Logic supply and idle current (J01, 2026-10-10)

**Power tree.** Markings read from photos; only the 9.5 V gate rail is measured (3S, 2026-07-22, §2).

```
7V~48V ─ buck ("CHGDVB" 8-pin + 22 µH "220") ─ gate rail 9.5 V (K36 node)
           ├─ EG2124A VCC (3× K36 = bootstrap diodes)
           └─ 78L05 (SOT-89) ─ 5 V: SIT1042, 5V pads          [fed from gate rail: inferred]
                 └─ "L352" (SOT-23-5, no inductor = LDO) ─ 3.3 V: G431, LEDs, 3V pads   [inferred]
```

The 78L05 is not on the bus: at 6S the whole board draws 39.3 mA, below the ~45–55 mA a 5 V / 3.3 V
tree of these parts needs (datasheet typicals, not measured). Settled by one reading: 78L05 input
pin on 6S, probe one pin at a time.

**Idle current**, motor disabled, UT89X 600 mA in series. V_pads = V_pack − I × 2.30 Ω (J03 M2 fit).

| Pack (open circuit) | ST-LINK USB | I_idle | V_pads | P_in |
|---|---|---|---|---|
| 3S 11.89 V | plugged | 59.6 mA | 11.75 V | 0.700 W |
| 3S 11.89 V | unplugged | 64.4 mA | 11.74 V | — |
| 6S 22.70 V | plugged | 39.3 mA | 22.61 V | 0.889 W |

- 6S/3S current ratio 0.66 (constant power 0.52, linear 1.00) → the board is a mostly
  constant-power load: I ∝ V^−0.64, −3.1 mA/V at 12 V. Normalise: `I_ref = I · (V_ref / V_pads)^−0.64`.
  Valid while the buck regulates. The 9.5 V rail on 3S suggests it does there; the K36 node on 6S
  confirms it. If it does not, the 3S slope is ~0 and J01's delta below is +3.3 mA (still ≤ 40 mW).
- **Unplugging the ST-LINK USB adds 4.8 mA** (the board powers part of the ST-LINK). Record the USB
  state and the pad voltage with every idle reading.

**Normalised to 12.28 V** (J01's August reading as measured; its banner said 12.22):

| Board | Date | mA at 12.28 V | ST-LINK USB |
|---|---|---|---|
| J01 | 2026-08-20 | 56.3 | not recorded |
| J02 | 2026-08-20 | 56.8 | not recorded |
| J03 | 2026-10-09 | 54.0 | not recorded |
| J01 | 2026-10-10 | **58.0 — J01 reference** | plugged |
| J01 | 2026-10-10 | 62.6 | unplugged |

J01 vs August: +1.7 mA if August was plugged, +6.3 mA if not; ≤ 75 mW either way, so it cannot cause
the hot corner. Gate at milestone close (USB plugged, pads measured): **≤ 61 mA at 12.28 V; > 61 = a
new idle load, look for a warm part.**

**Hot corner = the 78L05, by design.** ≈ (9.5 − 5) V × ~50 mA ≈ 0.22 W in SOT-89 (θ_JA ~125–225 K/W,
typical, not measured) → +28–50 K. The L352 dissipates ≈ 0.07 W. The rail is regulated, so the
dissipation does not depend on pack voltage, and it is the same on every board. **No heatsink:** a
SOT-89 sheds heat through its tab into the copper, and bare aluminium beside fine-pitch pads is a
short risk. Promote when the electronics bay is sealed or has no airflow, or a board resets when
hot: first measure the 78L05 input pin and the K36 node on the robot pack.
