# CAN transport bring-up (§23)

Classic CAN at 1 Mbit/s, ESP32-S3 master ↔ ESC1 clones. The ladder is complete (S0–S2 and
robustness tests A, C, D passed 2026-08-13 → 15). This file holds the measured facts, the Tier-0
init checklist, and the deferred 12-board termination rework.

*Hub: [`README.md`](../README.md). Sketches: [`tools/can_bringup/`](../tools/can_bringup/).*

---

## 23. Ladder — results

| Stage | Adds | Result |
|---|---|---|
| S0 | ESP32-S3 + SN65HVD230 + logic analyzer, NO_ACK mode | ✅ 2026-08-13 (§23.4) |
| S1 | second ESP32, NORMAL mode | ✅ 2026-08-14, ~200k frames, 0 bad (§23.4b) |
| S1b | ESC1, GPIO only | ✅ 2026-08-14: `S` pin polarity, PC11 → pin 8, loop delay 125 ns (§23.3) |
| S1c | ESC1 FDCAN loopback + mux test | ✅ 2026-08-14: HSE 8 MHz, AF9 mux on both pins (§23.7) |
| S2 | ESP32 ↔ ESC1 on real wire | ✅ 2026-08-14: 21,800+ frames, 0 errors (§23.9) |
| A, C, D | loopback babble, reboot isolation, TEC recovery | ✅ 2026-08-15 (§23.10). B dropped (answered by C) |
| S1e | width of the real control frame | deferred until the CAN message spec is frozen |

---

## 23.1 Bus behaviour measured on this rig

- **A lone node does not go bus-off.** With no ACK it climbs TEC 0 → 128 (16 frames × 8), then
  **freezes at 128** (error-passive ACK-error exemption) and retransmits forever (7,683/s,
  130.2 µs per attempt). Bus-off needs bit or form errors.
- **`TEC` frozen at 128 = nobody answering. `BUS_OFF` = somebody is driving the bus.** A climbing
  `buserr` alone means only "no ACK".
- ESP32 `twai_state_t` has no error-passive value: detect it as TEC ≥ 128.
- Stage-0 (single node) needs NO_ACK/self-test mode, which needs the raw ESP-IDF TWAI driver (not
  the `ESP32-TWAI-CAN` wrapper). In NO_ACK mode `TEC = 0` is not a pass; ground truth is the
  analyzer decode plus RXD mirroring TXD.
- Mixed 5 V (ESC1, recessive ~2.48 V) and 3.3 V (SN65HVD230, ~1.65 V) transceivers on one bus
  work (S2).

---

## 23.2 Termination and the 12-board rework

**Each ESC1 has a hard-wired 120 Ω (R22, 0603, TP4/TP12) across CANH–CANL** — measured 121 Ω
unpowered; no firmware-switched terminator exists.

| Bus | Parallel resistance | Spec | Verdict |
|---|---|---|---|
| Bench: 1 ESC1 + 1 SN65HVD230 | **59.5 Ω** measured unpowered | 60 Ω | ✅ |
| Robot: 12 ESC1 + master | 9.2 Ω | ≥ 45 Ω | 🔴 bus dead |

**Rule: termination lives in the harness, never on a joint.** Every ESC1 loses its resistor;
none ever terminates.

| | Bus end 1 | Bus end 2 |
|---|---|---|
| Bench, after rework | SN65HVD230 breakout, jumper ON | discrete 120 Ω in the test lead |
| Robot, 13 nodes | master's transceiver board | discrete 120 Ω at the far end |

**Measure termination UNPOWERED.** A powered transceiver's bias network adds a fixed ~450 Ω
parallel path (2 terminators: 53 Ω powered vs 59.5 Ω unpowered; 3 terminators: 37 Ω powered).
With 12 powered ESC1s those paths are ~38 Ω, which hides any termination error.

### Rework procedure — ⏸ deferred until the first bus with more than two nodes

A third node on an unreworked bus fails outright. ~3–4 h, irreversible. Optional de-risk: do J03 alone first.

0. Before the first board, unpowered: measure CANH→GND and CANL→GND. Both open (> 1 MΩ) = single
   120 Ω, remove one resistor. Both ~60 Ω = split termination, remove two.
1. Locate the resistor by photograph, confirm by probing before heating anything.
2. Practise on scrap first.
3. Leaded solder, flux, chisel tip: add solder to both terminals, bridge them into one blob, slide
   the part off.
4. Fallback: crush the body with flush cutters and clean the pads.

**Per-board acceptance (both required):**

| Check | Pass |
|---|---|
| Unpowered resistance CANH–CANL | open (> 1 MΩ) |
| Functional, with the S1b probe (`x`, `r`, `t`) | board still talks |

**Whole bus after rework, everything unpowered: ~60 Ω.** The S1b GPIO probe sketch is lost:
before the rework, recreate it or use `esc1_s1c`'s `x` (mux) probe as the functional check.

---

## 23.3 ESC1 CAN pin facts

| Fact | Value | Source |
|---|---|---|
| `CAN_TX` / `CAN_RX` | **PB9 / PA11**, AF9 | schematic; AF9 proven at S1c |
| Transceiver | **U8 `SIT1042QTK/3`** (TJA1042T/3 class), on `VCC_5V`. 1 TXD · 2 GND · 3 VCC · 4 RXD · 5 VIO · 6 CANL · 7 CANH · 8 S | schematic |
| Mode pin | **`S` (pin 8) ← PC11.** LOW = Normal, HIGH = Standby, **floating = Standby** (internal pull-up) | measured S1b |
| Transceiver loop delay | **125 ns** | measured S1b |
| CANH/CANL recessive bias | 2.48 V (≈ 0.5 × VCC_5V) in Normal, ~0 V in Standby | S1b |
| Termination | R22 120 Ω, TP4/TP12 | §23.2 |
| VIO (pin 5) net | not checked on the schematic; weeks of use with RXD on PA11 without damage | — |

- **Tier-0 must drive PC11 LOW before FDCAN goes to NORMAL.** Forgetting it leaves the
  transmitter disabled with no error reported.
- A floating PC11 (MCU in reset) puts the transceiver in Standby, which is why a rebooting joint
  does not disturb the bus (Test C).
- **PB9 and PA11 have no pads.** The only CAN pads are CANH/CANL, and neither threshold-decodes
  on the FX2 at 5 V (CANH 2.48 / 3.88 V: always above V_IH; CANL dominant 1.12 V: undefined zone).
  No resistor divider fixes it. **Tap the ESP32 breakout's RXD instead.**
- Exposed pads: `GND · 3V · SWD · SCK · PWM · BUTTON · POTEN · TXD · RXD · GND · 5V · CANL ·
  CANH · PB5 · RST`; other edge `5V · GND · HC/Z · HB/B · HA/A · 7V-48V` and the three phases.

---

## 23.4 S0 — ESP32 + SN65HVD230 + analyzer

### Pre-flight with a meter

Press `p` to pause TX (TXD parks recessive). Referenced to breakout GND:

| Terminal | Expect | If wrong |
|---|---|---|
| Termination (power off) | 120 Ω | 59 Ω: an extra resistor is fitted |
| 3V3 | 3.25–3.35 V | transceiver unpowered |
| TXD | ~3.3 V | ~0 V: GPIO5 → TXD wire missing |
| RXD | ~3.3 V | 0 V: receiver stuck dominant, or GPIO4 wire open |
| CANH, CANL | 1.6–1.7 V each | |
| CANH − CANL | < 50 mV | standing differential = driver stuck dominant |

Measured 2026-08-13: all in range, differential exactly 0 V, TX at 200 frames/s.

### Analyzer — CY7C68013A (FX2LP), fx2lafw / PulseView

| Setting | Value |
|---|---|
| Channels | all 8 |
| **Sample rate** | **24 MSa/s** (12 if PulseView errors). **Check it on the toolbar before reading anything** — 20 kHz shows a dead bus |
| Samples | 1 M (41.6 ms, ~8 frames at 200 Hz) |
| Decoder | CAN on the **TXD/RXD logic line** (not CANH/CANL), bitrate 1000000 |
| Channel mapping | silkscreen N is probably D(N−1); confirm from which traces are active |

Bit time: cursors on two edges 10 bit-times apart → expect 10.00 µs (ESP32 read 10.007 µs).

ESP32 sketch: raw ESP-IDF TWAI driver; `b` reprints the banner (USB-CDC enumerates late); `p`
pauses TX.

---

## 23.4b S1 — ESP32 ↔ ESP32, NORMAL mode

One `main.cpp`, envs `nodeA` / `nodeB` / `solo`. **Child envs must inherit the parent
`build_flags`** (a child `build_flags =` replaces it, dropping `ARDUINO_USB_CDC_ON_BOOT`).

| Check | Node A (0x100) | Node B (0x101) |
|---|---|---|
| State, TEC, REC | `RUNNING`, 0, 0 | `RUNNING`, 0, 0 |
| `rxbad` | 0 of ~91,000 | 0 of ~113,000 |
| `arblost` | 0 | 29 (0x100 wins every collision: arbitration works) |
| `txfail` | 0 | 0 |

Node B unpowered: TEC 0 → 128 then frozen, state `RUNNING`, `buserr` 7,683/s; recovered
automatically on reconnect.

**Arbitration-loss rate between two free-running nodes: 0.0257%** (29 in ~113,000).

### Control frames are single-shot

ESP32: single-shot flag. G431: `FDCAN_CCCR.DAR = 1`. A control frame that missed its 10 ms slot is
stale, and applying a stale `{p_des, v_des, kp, kd, τ_ff}` is worse than dropping it. `txfail`
then becomes a per-joint health signal.

An error-passive node does not lock out a joiner (suspend transmission; measured in Test C), so
single-shot is not needed for bus starvation. The real one-kills-many hazard is an error-**active**
node raising bit/form errors (marginal wiring, or a node in loopback — Test A); wiring integrity
and termination address it.

### Frame length and bus load

S1c test payload: 116 bits (field count) / 115.5 bits (flood rate). Two-node S1 bus 4.64%;
12 joints × 2 frames at 100 Hz → **27.8%** at 116 bits, **31%** at the conservative 130-bit
design figure. The real control frame will stuff differently (S1e, deferred to the message spec).

---

## 23.5 FDCAN kernel clock — HSE 8.000 MHz

`HSERDY` sets; `RCC_CCIPR.FDCANSEL = 00 = HSE` is used as-is. Frequency identified at S1c and
bounded at S2 (§23.7, §23.9). The bit rate is independent of SYSCLK, so PWM, loop rate and
`T_DELAY_PER_LOOP` are untouched.

- **Do not change the APB1 prescaler to adjust the FDCAN clock** (it moves TIM2/3/4). Use
  `FDCANSEL` or the PLL.
- Do not measure HSE on `MCO`: MCO is PA8 = `TIM1_CH1`, a motor phase.
- **Loopback disregards the Rx pin** (both internal and external modes feed Rx internally), so
  loopback cannot prove the pin mux or the transceiver. `TEST.RX` monitors the real pin (§23.7).

---

## 23.6 Tier-0 CAN constraints and init checklist

**Constraints:**
1. **Control frames single-shot** (`DAR = 1`). Confirmed acting: frames discarded at 200 Hz with
   the partner absent; 0 drops in ~10,000 healthy frames.
2. **Loopback is a power-on self-test only, with the bus quiescent.** A node in loopback cannot
   arbitrate or see bit errors and babbles over the bus (Test A: partner went `BUS_OFF`). Be in
   NORMAL before any other node is enabled.
3. **Drive PC11 LOW before NORMAL** (§23.3).
4. **Enable HSE and poll `HSERDY` with a timeout; on timeout refuse to arm.**

**Preference:** polled / round-robin master. It removes arbitration contention (the 0.026%
single-shot drop rate). Decide it before the frame IDs.

**Still to design (unblocked):** the CAN message spec (frame IDs, packing and scaling of
`{p_des, v_des, kp, kd, τ_ff}` → `{p, v, τ}`, poll schedule) and the RL observation/action vector,
frozen together.

### Init checklist — every value measured

| # | Step | Source |
|---|---|---|
| 1 | Enable HSE, poll `HSERDY` with a timeout, fail loudly | §23.5 |
| 2 | `RCC_CCIPR.FDCANSEL = 0` (HSE) | §23.5 |
| 3 | Enable the FDCAN peripheral clock | — |
| 4 | PB9 (TX), PA11 (RX) → AF9 | §23.3 |
| 5 | PC11 LOW → transceiver Normal | §23.3 |
| 6 | `NBTP`: NBRP 1, TSEG1 6, TSEG2 1, SJW 1 → 8 tq, 87.5% sample point (`NBTP = 0x500`) | §23.8 |
| 7 | `CCCR.DAR = 1` | constraint 1 |
| 8 | NORMAL mode | constraint 2 |

Step 4 before step 5 is recommended, not required (Test C measured the window clean).

---

## 23.7 S1c — FDCAN loopback, mux test, crystal

Sketch `tools/can_bringup/esc1_s1c/`. Clock brought up by hand; every register read back.

**Internal loopback (`i`):** `HSE ready`; `NBTP = 0x500`; `CCCR = 0x10E0` (MON, TEST, DAR set);
`TEST = 0x90` (LBCK, RX recessive); `recv = sent − 1` over 9,400 frames; `rxbad`, TEC, REC, LEC 0.

**Crystal:** `TSCV` (FDCAN timestamp, one count per bit time, free-running) counted 4,989,144 in
5,000 ms of `millis()` → 7.9826 MHz; corrected for the measured HSI16 error (+0.141%, §23.9) →
**7.9939 MHz**. Nearest standard value 8 MHz; next candidate (12 MHz) is +50% away.
`HSE_ASSUMED_HZ = 8000000`.

**Mux test (`x`):** transmit in external loopback while sampling `TEST.RX` (the real Rx pin).
**52.05% dominant** vs ~50% predicted from the frame's bit content (58 dominant of 116). Stuck
dominant reads ~100%, dead mux ~0%. AF9 confirmed on both pins, loop closed through the
transceiver.

**Flood:** 17,318 frames in 2 s = 8,659/s → 1.004 Mbit/s at 116 bits.

---

## 23.8 Bit timing at 8 MHz

8 MHz ÷ 1 Mbit = 8 tq at NBRP 1 (the CAN minimum; the only option).

| Constraint | Requirement | Actual | Margin |
|---|---|---|---|
| Propagation: sample point > 2 × (loop + bus delay) | ≥ 253 ns (2 × (125 + 1.5)) | 875 ns | 3.46× |
| Oscillator tolerance `min(PS1,PS2) / (2(13·NBT − PS2))` | df ≤ 0.4854% | ±50 ppm crystal | 97× |
| Oscillator tolerance `SJW / (20·NBT)` | ≤ 0.625% | | 125× |
| Sample point | CiA | 87.5% | — |

HSI16 (±1%) exceeds the 0.4854% budget by 2×: an HSI-derived kernel clock would fail when warm.
HSE is mandatory.

---

## 23.9 S2 — ESP32 ↔ ESC1 on real wire

### Wiring (power off)

| Connection | |
|---|---|
| ESC1 CANH → breakout CANH, ESC1 CANL → breakout CANL | twisted pair, 20–50 cm |
| ESC1 GND → ESP32 GND | mandatory |
| Meter CANH–CANL, **unpowered** | 60–61 Ω, else stop |
| Analyzer | ESP32 RXD (GPIO4) → 1 kΩ → D1, GND tied, CAN decoder 1000000 |
| Motor | disconnected |

### Order

1. ESC1: flash, press `n`. It reaches TEC 128, `EP=1`, `LEC=ACK` within ~80 ms and parks (no
   hammering, `DAR = 1`).
2. ESP32: `pio run -e nodeA -t upload`, press `b`, confirm banner `NORMAL`, `ID = 0x100`.

Make sure the ESC1 is **not** in loopback (`e`) when the ESP32 starts (Test A).

### Result

| Check | Result |
|---|---|
| ESC1 alone | TEC 128, `LEC=ACK`, `EP=1`, `EW=1`, `BO=0`, flat for 1,200+ frames |
| ESP32 after partner | `RUNNING`, TEC = REC = 0, `lastid = 0x200` |
| `rxbad` / `buserr` / `arblost` | 0 / 0 / 0 over 21,800+ frames |
| ESC1 TEC recovery | 128 → 0 in 128 frames = **640 ms** at 200 Hz |
| Error-passive watch on unplug | fired |

**Error-passive signature: `EP=1` + `EW=1` + `BO=0`** (`EW` sets at TEC ≥ 96 and stays set).

**HSI16 = +0.141%** (STM32 `millis()` vs the ESP32's ±20 ppm crystal: `recv − sent` grew +3 →
+30 between `sent` 2601 and 21801). Reusable for anything clocked off HSI16 on this board.

**HSE bound from S2:** 21,801 error-free frames against the ESP32's crystal bound f_HSE to ±0.97%
(7.92–8.08 MHz). HSE = 8.000 MHz, closed. The analyzer bit-time capture (±0.83%) is not needed.

### Failure branches — `LEC` discriminates

| Symptom | Diagnosis |
|---|---|
| ESC1 `LEC=ACK`, `EP=1`, ESP32 silent | partner not running or not on the bus; check ESP32 banner says `NORMAL` |
| `LEC=STUFF` or `FORM`, both nodes erroring | bit-rate mismatch (crystal) |
| `LEC=CRC`, sporadic | signal integrity: termination, cable length, ground tie |
| `LEC=BIT0` | sent dominant, read recessive: driver not reaching the bus |
| `LEC=BIT1` | sent recessive, read dominant: another node driving, or a short |
| `BO=1` on either node | bit/form errors past TEC 255 — someone is driving the bus (e.g. a node in loopback). A missing partner parks at 128 |
| `rxmiss` climbing, all else clean | printing stall, not a bus fault |

---

## 23.10 Robustness tests A–D (2026-08-15)

| Test | Question | Result |
|---|---|---|
| **A** | Does a node in loopback babble over a live bus? | **Yes.** ESC1 in `e` on a live bus → ESP32 `BUS_OFF` (7 frames = 35 ms after the switch). Basis of constraint 2 |
| **B** | Does an error-passive node throttle a joiner? (S1d) | Dropped: answered by C (the ESP32 was error-passive and retransmitting at 7874/s when the ESC1 rejoined; both clean within ~640 ms) |
| **C** | Does a rebooting joint disturb the bus? | **No.** ESC1 `RST` held to GND ~9 s, three times: ESP32 TEC parked at 128, never `BUS_OFF`, `buserr` 7,874/s (same signature as an unpowered partner). PC11 floats → Standby → transmitter off |
| **D** | TEC recovery from 128 | Exactly −1 per successful frame: **640 ms** at 200 Hz. Back inside one 100 Hz policy cycle; no Tier-1 rejoin logic needed |

**Drop rate (Tx Event FIFO, `drop = sent − txok`, single node):** partner absent → `drop` climbs at
200 Hz (frames discarded, not retried); after rejoin → frozen; **0 new drops in ~10,000 frames**
(upper bound ~0.03%). The Tx Event FIFO is 3 deep on the G4: drain it every `loop()`.

**Untested:** a joint **power-cycling** (VCC ramping on the transceiver) rather than resetting.
Promote when the power distribution can cycle one joint independently.
