# CAN transport bring-up (§23)

The staged ladder from a lone ESP32 to a two-node ESP32↔ESC1 bus, the physical-layer
facts measured on this hardware, and the FDCAN kernel-clock decision — settled
2026-08-14 at HSE 8.000 MHz, with the bit-timing margins that follow from it.

> ⚠ **Two claims in this document have been retracted, both boxed rather than deleted.**
> 1. **A lone CAN node does NOT go bus-off from missing acknowledgement** — it parks at
>    error-passive, TEC = 128, and retransmits forever (**§23.4b**).
> 2. **External loopback does NOT prove the AF9 pin mux or the transceiver path** — both
>    loopback modes feed the receiver internally and disregard the Rx pin, so `e` is
>    expected to match `i` even with a broken mux (**§23.5**; proven instead in §23.7).
>
> A third correction is not a retraction but an invalidated instruction: **PB9 and PA11
> have no pads on this board, and CANH/CANL cannot be threshold-decoded at 5 V**, so
> "point the analyzer at PB9" was never executable (**§23.3**).

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 23. Why a ladder, and what each rung adds

Twelve actuators, one master, in an environment full of PWM switching noise. The
control path is Classic CAN at **1 Mbit/s**, which is a frozen decision (§3 of
`claude.md`) — but see §23.5, because it is coupled to the FOC loop rate in a way
that was not costed when it was frozen.

**One new unknown per rung**, so a failure has exactly one candidate cause.

| Stage | Adds | Proves | Blocked by |
|---|---|---|---|
| **S0** | ESP32-S3 + SN65HVD230 + logic analyzer | The analyzer setup, the ESP32 pins, one known-good transceiver | nothing |
| **S1** | a second ESP32 | Real ACK, real bit timing, the frame handling — **zero STM32 involved** | S0 |
| **S1b** | the ESC1 clone, **GPIO only, no FDCAN** | **Reduced 2026-08-14** — identity and rail now come from the schematic (§23.3). Confirms only that PC11 reaches the `S` pin and that PB9 / PA11 are routed as drawn | S0 |
| **S1c** | STM32 FDCAN in **external loopback** | ✅ **PASSED (§23.7).** FDCAN kernel clock, bit timing, message RAM — **no partner, no cable.** ⚠ **NOT** the AF9 mux: loopback disregards the Rx pin, so that needed the separate `x` test | S1b + the clock decision |
| **S2** | the wire between them | Termination and wiring only | S1, S1c |

**S1c is the rung that matters most.** It isolates the FDCAN kernel clock — the single
most likely failure — from the wiring, which is the thing you would otherwise be
tempted to blame. A misconfigured kernel clock produces symptoms **identical** to a
broken wire, with every register write appearing to succeed.

> ⚠ **Naming:** `S`-numbers belong to this ladder only. The belt-on steps are
> `B`-numbered (§22) and manual calibration is `M`-numbered (§20). A July
> characterisation ladder also used S0–S8; those names are retired, all of it is closed.

---

## 23.1 Ten-line physical-layer primer

- **Two wires, CANH and CANL, carry one differential signal.** Two states only:
  **dominant** (CANH high, CANL low, differential ≈ +2 V) and **recessive** (both at
  the same mid-rail voltage, differential ≈ 0 V).
- **Dominant always wins.** Any node pulling dominant overrides every node sending
  recessive. That is how arbitration and acknowledgement work with no clock line.
- **The MCU never touches CANH/CANL.** It talks to the transceiver over two ordinary
  logic pins: **TXD** (MCU → bus) and **RXD** (bus → MCU). On both, **low = dominant.**
  **These are the lines to point the analyzer at** — clean 0/3.3 V logic.
- **There is no clock wire.** Every node generates the same bit time from its own
  oscillator and resynchronises on recessive→dominant edges. Total tolerance is
  roughly **±1.5%**. This is why the FDCAN kernel clock is the whole ballgame.
- **Every frame ends with an ACK slot**, sent recessive by the transmitter and pulled
  dominant by any node that received it correctly.
- **Termination is two 120 Ω resistors, one at each physical end**, giving 60 Ω. More
  than two and the transceiver cannot pull the bus dominant.
- **Bit stuffing:** after 5 identical bits the transmitter inserts one opposite bit.
  These appear in the decode and are not data.
- **Mixed 3.3 V and 5 V transceivers on one bus is normal and fine.** Only the
  recessive common-mode bias differs (1.65 V vs 2.5 V) and both receivers accept either.
- **Topology:** a single trunk, node stubs as short as possible (<30 cm at 1 Mbit/s).
  No star wiring — every stub end is an unterminated reflection source.

> ### ⚠ RETRACTED: "endless retransmission is the correct Stage-0 result"
> That was wrong, and it would have wasted a session chasing a healthy board.
> **A lone node gets no ACK, treats it as an error, and retransmits the same frame
> endlessly** — measured at 7,683 attempts/s, ~88% of a 1 Mbit bus.
>
> > ⚠ **This paragraph used to end "and after roughly 32 failures (TEC > 255) shuts
> > itself off — bus-off." That is RETRACTED and was measured false at S1 (§23.4b).**
> > TEC climbs 0 → 128 in 16 frames and **freezes there forever**: fault confinement does
> > not increment TEC once an error-passive transmitter sees an ACK error with no dominant
> > bit during its passive error flag. **Bus-off from missing ACK alone is impossible.**
> > The claim reappeared here in a later rewrite of this file; it is corrected in place
> > rather than deleted, because it is persuasive and keeps coming back.
>
> **Stage 0 must run in NO_ACK / self-test mode**, which suppresses the ACK
> requirement. Two consequences that follow from it:
> - **Use the raw ESP-IDF TWAI driver, not the `ESP32-TWAI-CAN` wrapper** — the wrapper
>   does not expose the mode.
> - **`TEC = 0` is not a meaningful pass indicator in NO_ACK mode.** It is zero because
>   errors are suppressed, not because the bus is healthy. **Ground truth is the
>   analyzer decode plus RXD mirroring TXD.** The S0 pass criterion was revised to
>   drop TEC.

---

## 23.2 Termination on this hardware — and the 12-board rework it implies

**Measured 2026-08-13, board unpowered: 121 Ω across CANH–CANL on the ESC1 clone.**

A firmware-switched terminator (MOSFET + resistor) reads **open** when unpowered.
121 Ω unpowered means a **physical resistor is permanently fitted.**

> **This contradicts project memory** — "TCAN330 onboard, firmware-controlled
> termination via `CAN_TERM`". **Bench evidence wins.** There is no firmware-controlled
> terminator on this clone, and the elegant "two end boards enable it, everyone else
> disables it, all in one flash-once binary" plan does not exist.

| Bus | Parallel resistance | Transceiver load spec | Verdict |
|---|---|---|---|
| Bench: 1 ESC1 + 1 SN65HVD230 | 121 ∥ 122 = **60.7 Ω** | 60 Ω nominal | ✅ **correct by accident** |
| Robot: 12 ESC1 + 1 master | 121/12 ∥ 122 = **9.2 Ω** | 45 Ω minimum | 🔴 **bus dead.** The dominant differential collapses below the receiver threshold |

🔴 **The termination resistor comes off every board — all 12, not 10 of 12.** Found for the
price of one resistance measurement, instead of at multi-node integration, where it would
have presented as "the bus works with two nodes and not with four" — a symptom that invites
weeks of software debugging.

### Termination lives in the harness, never on a joint

**Revised 2026-08-15.** The obvious plan is to leave two boards terminated and strip ten.
Don't: it makes two of the twelve special, and *which* two depends on a bus topology that
has not been designed. Move a joint and you are back with an iron.

| | Bus end 1 | Bus end 2 |
|---|---|---|
| **Bench, 2 nodes, after rework** | SN65HVD230 breakout, **jumper ON** | discrete 120 Ω in the test lead |
| **Robot, 13 nodes** | master's transceiver board, at one physical end | discrete 120 Ω at the far end |

**The SN65HVD230 breakout keeps its resistor.** Its 120 Ω is **jumper-selectable**, so it is
already adjustable without touching an iron — which makes it the natural terminator wherever
the master sits. Nothing to remove there.

**Every ESC1 gets its resistor removed and none of them ever terminates.** That is what
makes twelve boards interchangeable, and it decouples the rework from a topology decision
that can then be made — and changed — with a screwdriver.

> ### 🔴 MEASURE UNPOWERED — the acceptance test is uninterpretable otherwise
> **Established at S2 (§23.9), and it cost two confusing readings to find.** A *powered*
> transceiver's recessive bias network sits across the bus as a fixed **~450 Ω** parallel
> path: the same bus read **53 Ω powered and 59.5 Ω unpowered** against 60.2 Ω predicted.
> **With 12 powered ESC1s those paths parallel to ~38 Ω**, which would swamp a 60 Ω
> reading completely — a correctly reworked bus and a badly broken one would look alike.
>
> **Target after rework, everything unpowered: ~60 Ω.**
>
> S2 also supplied the **measured** version of this section's arithmetic: three
> terminators powered read **37 Ω**, below the 45 Ω minimum transceiver load. The
> over-termination argument below is no longer only a calculation.

**One 30-second follow-up, unpowered, still to do:** measure CANH→GND and CANL→GND.

| Reading | Meaning | What to remove per board |
|---|---|---|
| Both open (>1 MΩ) | A single 120 Ω across the pair | one resistor |
| Both ~60 Ω | Split termination, 2 × 60 Ω with a cap to GND | two resistors |

### The rework — ⏸ **NOT DONE.** Procedure recorded now, executed later

> **Promoting condition: the first bus with more than two nodes.** Physical, not temporal.
> Three to four hours of **irreversible** soldering across twelve boards, not on the
> critical path, and the two-node bench bus works correctly as it stands (§23.9: 59.5 Ω
> unpowered). Deferring it is the milestone-discipline call, not procrastination — but it
> is a hard gate, because a third node on an unreworked bus does not degrade, it fails.

**Optional cheap de-risk, ~30 min:** do **J03 alone**. It is the one board that is not part
of a characterised joint, so it validates both the removal method and the acceptance test
at no risk to J01 or J02. The other eleven can wait for the promoting condition.

| Step | |
|---|---|
| 1 | **Locate by photograph, then confirm with three probes** before heating anything. The resistor is identified by measurement, not by looking like the right part |
| 2 | **Practice on scrap first.** This is a 12× irreversible operation; the first one should not be on a board that matters |
| 3 | **Leaded solder, flux, chisel tip.** Add leaded solder to both terminals, bridge them into one blob, and slide the part off. Do not try to lift one end at a time |
| 4 | **Fallback: flush-cutter crush.** If the iron is losing, crush the body and clean the pads. Ugly and completely acceptable — the part is going in the bin either way |

**Per-board acceptance test, both halves required:**

| Check | Pass |
|---|---|
| **Unpowered** resistance across CANH–CANL | **open** (>1 MΩ) — see the unpowered rule above; a powered reading proves nothing |
| **Functional**, with the S1b probe (`x`, `r`, `t`) | the board still talks. Removal is mechanical work next to a live trace, and "open" only says the resistor left, not that the pads survived |

**Whole-bus acceptance after rework, everything unpowered: ~60 Ω**, from the two harness
terminators alone.

---

## 23.3 ESC1 CAN pin facts

| Fact | Value | Provenance |
|---|---|---|
| `CAN_TX` | **PB9**, AF9 | Clone schematic |
| `CAN_RX` | **PA11**, AF9 | Clone schematic |
| **Transceiver** | **U8 = `SIT1042QTK/3`** — a TJA1042T/3-class part, *not* a TCAN330 | ✅ **Clone schematic, 2026-08-14.** SOIC-8: 1 TXD · 2 GND · 3 VCC · 4 RXD · 5 VIO · 6 CANL · 7 CANH · 8 S |
| **Supply rail** | **`VCC_5V`** | ✅ Schematic. The `/3` suffix means pin 5 is **VIO**, an I/O reference *input* — **not VREF**, a mode-indicator *output*. VREF is a TJA1040/TJA1050 feature; it does not exist on this part. *(Datasheet recall, ~80% confidence — verify in the SIT1042 datasheet, same pass as the `S`-pin polarity below.)* |
| **Mode pin** | **`CAN_SHD` → pin 8 = `S` (standby control)** | ✅ Schematic. Routed to **PC11** (§3, high confidence) |
| Termination | R22 = 120 Ω ±1% 0603 across CANH–CANL, test points TP4 / TP12 | ✅ Schematic — see §23.2 |

> ### 🔴 THE MODE PIN IS NOT OPTIONAL, AND FLOATING IS THE WRONG STATE
>
> On the TJA1042 family, pin 8 `S` selects **Normal** mode when driven **LOW** and
> **Standby** when **HIGH**, and the pin carries an internal pull-up for fail-safe — so
> **an unconfigured, floating `S` puts the transceiver in Standby.** *(Datasheet recall,
> ~75% confidence on the internal pull-up specifically; the LOW = Normal polarity is
> solid. Verify against the SIT1042 datasheet before S1c — it is a 30-second lookup and
> it is on the critical path.)*
>
> **In Standby the transmitter is disabled and RXD does not mirror TXD** — which is
> precisely the S0/S1b pass criterion. So a board that never drives PC11 presents as
> *"the ESC1 transceiver is dead"*, and the natural next move is to start probing wiring
> and FDCAN registers, neither of which is the fault.
>
> **This is the failure mode §23.3 was already warning about in the abstract** — *"a
> transceiver that boots into standby because nobody set its mode pin is a bus that works
> on the bench and dies on the robot"* — and it is now a specific pin with a specific
> polarity rather than a general worry.
>
> **Two consequences:**
> 1. **Tier-0 firmware must drive PC11 LOW before enabling FDCAN**, and that line needs a
>    comment saying why. It is one GPIO write and it is invisible when missing.
>    *(Now constraint 3 of four in §23.6, with the ordered init checklist.)*
> 2. **Pin 5 is VIO, not VREF — corrected 2026-08-14.** The original plan ("meter pin 5,
>    read ~2.5 V for Normal") checked a pin that cannot produce that signal on this part,
>    and was moot anyway: **the IC is unreachable with a probe** (confirmed below). The
>    working substitute uses pads already on the bench — **CANH/CANL themselves.** This
>    family biases CANH/CANL to ≈ 0.5 × VCC in Normal mode and to ground in Standby; at
>    `VCC_5V` that is **~2.5 V vs ~0 V**, read at test points TP4/TP12 — the same points
>    already used for the §23.2 termination measurement. Larger swing, no IC access
>    needed. *(~85% confidence; the polarity toggle on `S` confirms it independently.)*
>
> **One more consequence of VIO, not yet checked:** "digital pins are 3.3 V compatible"
> is conditional on VIO being tied to the 3.3 V rail, not intrinsic to the part — a clone
> that tied VIO to 5 V would put a 5 V RXD on PA11. **Empirically retired**, in the sense
> that the board has run for weeks with the transceiver powered and RXD idling recessive
> with no damage, but that is not the same as having checked it. **30-second addition to
> the S1b schematic pass: read VIO's net.**

### What this does to S1b

S1b was scoped to answer three unknowns by probing. **Two are now answered from the
schematic**, so the rung shrinks to a confirmation:

| S1b question | Status |
|---|---|
| Transceiver identity | ✅ **answered** — `SIT1042QTK/3` |
| Supply rail | ✅ **answered** — 5 V, with 3.3 V-compatible logic |
| Mode pin and polarity | **narrowed** — `S` on PC11, LOW = Normal. Confirm the polarity in the datasheet, then confirm PC11 actually reaches pin 8 by toggling it and watching the **CANH/CANL bias** (§23.3's corrected consequence 2 — not VREF, which this part doesn't have) |
| PB9 / PA11 routing | unchanged — confirm by toggling each as a GPIO |
| VIO net | **new, 2026-08-14** — read which rail VIO (pin 5) is tied to on the schematic. "3.3 V compatible" depends on it; weeks of undamaged operation is evidence, not a check |

**Revised S1b: drive PC11 low, toggle PB9, watch RXD on PA11 and the CANH/CANL bias at
TP4/TP12** (~2.5 V Normal / ~0 V Standby at `VCC_5V` — no IC access needed, unlike the
pin-5 plan it replaces). No FDCAN, no clock, no cable. Still worth running — a schematic
is rank 2 and a working GPIO toggle is rank 1 — but it is now ~10 minutes rather than 30.

> ### ⚠ RETRACTED: the BOOT0 / `CAN_RX` collision hypothesis
> The concern was that `CAN_RX` might land on **PB8 = BOOT0** (§3), which would make
> a recessive idle-high bus hold the MCU in the ROM bootloader. **The schematic shows
> `CAN_RX` on PA11.** There is no collision. Recorded because the failure mode was
> plausible and the retraction is the useful artefact — PB8 remains a live trap for
> everything *else*.

**A transceiver that boots into standby because nobody set its mode pin is a bus that
works on the bench and dies on the robot.** That is what S1b exists to prevent, and it
does it with GPIO reads only — no FDCAN, so a failure cannot be a clock problem.

### S1b — ✅ PASSED 2026-08-14

| Question | Result |
|---|---|
| `S`-pin polarity | ✅ **MEASURED: LOW = Normal, HIGH = Standby, FLOATING = Standby.** The datasheet-recall polarity was right and the fail-safe pull-up is confirmed by behaviour |
| PC11 reaches pin 8 | ✅ confirmed by driving it and watching the transceiver change state |
| PB9 / PA11 routing | ✅ confirmed **as GPIO** — bit-banged, which is why it does **not** cover the AF9 alternate-function path. That gap is what §23.7's `x` test closes |
| **Transceiver loop delay** | ✅ **125 ns measured.** Load-bearing: it sets the propagation-segment requirement in §23.8 |
| CANH/CANL recessive bias | **2.48 V** — i.e. ≈ 0.5 × `VCC_5V`, exactly the substitute mode-check specified above. **The pin-5 problem is fully retired: the check that replaced it works** |

> **Provenance note on the 2.48 V.** It is reported alongside the FX2 threshold analysis
> rather than in a pre-flight table, so it may be a meter reading or a datasheet-nominal
> figure. **If it was metered with PC11 driven LOW it independently confirms Normal
> mode**; if not, the `S`-pin toggle already does. Flagged rather than assumed, because
> "0.5 × VCC" and "2.48 V" are the same claim from two very different sources.

The `FLOATING = Standby` row above turned out to be the most load-bearing line in the table.

> ### ✅ The fail-safe now has a consequence, measured 2026-08-15
>
> S1b established the behaviour on a bench pin. **Test C (§23.10) established what it is
> worth:** the board was held in reset — every GPIO floating, including PC11 — **on a live
> bus, three times**, and the partner never saw a single bit or form error. The
> transceiver's internal pull-up took it to Standby, the transmitter stayed disabled, and
> **a rebooting joint is a local event rather than a bus-wide one.**
>
> The same run retired a firmware-ordering worry: PB9 configured as AF9 **before** PC11 is
> driven LOW is **recommended, not required** — the enable window was measured and it was
> clean. See §23.6's checklist, step 4. **Untested scope:** a joint *power-cycling* rather
> than resetting, where VCC ramps on the transceiver itself.

### ⚠ PB9 and PA11 have no pads — and CANH/CANL do not decode

**Corrected 2026-08-14, and it invalidates a bench instruction given several times.**
The exposed pads are `GND · 3V · SWD · SCK · PWM · BUTTON · POTEN · TXD · RXD · GND ·
5V · CANL · CANH · PB5 · RST` on one edge, plus `5V · GND · HC/Z · HB/B · HA/A ·
7V-48V` and the three phases on the other. **PB9 and PA11 are package pins only.**
"Point the analyzer at PB9" was never executable.

The only CAN-carrying pads are CANH and CANL, and **neither decodes on the FX2** at
`VCC_5V`:

| Line | Recessive | Dominant | Against FX2 V_IL 0.8 / V_IH 2.0 |
|---|---|---|---|
| CANH | 2.48 V | 3.88 V | **both above V_IH — never changes state** |
| CANL | 2.48 V | 1.12 V | dominant lands **inside the undefined zone** |

**No resistor divider fixes this**, and the reason is worth stating because the fix
looks obvious: a divider scales both levels by the same factor, and the two required
inequalities are contradictory in each case — CANH needs `k > 0.515` and `k < 0.323`;
CANL needs `k > 0.806` and `k < 0.714`. A level shifter with an offset, or a comparator,
would work; a divider cannot. **This is the 5 V restatement of §23.4's finding 3**,
which found the same thing on the 3.3 V side for a different reason (recessive sitting
*inside* the undefined zone rather than above it).

⇒ **The analyzer's first real look at ESC1 traffic is S2**, tapping the ESP32
breakout's RXD — clean 3.3 V logic, and the same tap used in S0.

*The six unpowered continuity checks originally specified for S1b were dropped: the IC
pads are inaccessible on this board. Everything needed is obtainable functionally
through S1b instead — and in the event most of it came from the schematic anyway,
which is the cheaper source that should have been read first.*

---

## 23.4 S0 — results and method — ✅ PASSED 2026-08-13

### Pre-flight with a meter, 3 minutes — ✅ PASSED

Done **before** the analyzer, so that an analyzer failure afterwards has exactly one
possible cause. Press `p` to pause TX first, so GPIO5 parks recessive (high) and every
reading below is a static DC level. Referenced to breakout GND:

| Terminal | Expect | Measured | If wrong |
|---|---|---|---|
| Termination (power off) | **120 Ω** | ✅ 120 Ω | 59 Ω means the external resistor is still in — pull it |
| 3V3 | 3.25–3.35 V | ✅ | Transceiver unpowered — fix before anything else |
| TXD | ~3.3 V | ✅ | ~0 V or drifting → **wire 1 (GPIO5 → TXD) is missing.** This is the check that settles it |
| RXD | ~3.3 V | ✅ | 0 V → receiver output stuck dominant, or the wire to GPIO4 is open |
| CANH | 1.6–1.7 V | ✅ | |
| CANL | 1.6–1.7 V | ✅ | |
| **CANH − CANL** | **< 50 mV** | ✅ **exactly 0 V** | A standing differential means the driver is stuck dominant |

**CANH and CANL both at Vcc/2 with near-zero differential is the transceiver actively
biasing a recessive bus.** That single reading proves the part exists, is powered, and
its driver is not jammed — three things otherwise guessed at.

**Also confirmed:** the TWAI driver is running and transmitting at **200 frames/s**,
banner reports `NO_ACK`, GPIO 5/4, 1 Mbit.

### Analyzer setup — CY7C68013A (FX2LP) under fx2lafw / PulseView, Windows

| Setting | Value | Why |
|---|---|---|
| Channels | **all 8 enabled** | Costs nothing — fx2lafw sends 1 byte per sample regardless of channel count |
| **Sample rate** | **24 MSa/s** (fall back to 12 if PulseView errors or the decode is garbled) | 41.7 ns per sample |
| Samples | 1 M → a **41.6 ms** window, ~8 frames at 200 Hz | |
| Trigger | none available on this dongle | Capture and scroll |
| Decoder | CAN, `can_rx` assigned to the **TXD** channel, bitrate **1000000** | TXD is the MCU-side logic line, not the differential pair |

**Channel mapping:** the dongle is silkscreened 1–8; sigrok names them D0–D7, so
silkscreen N is almost certainly D(N−1). **Do not assume it — the two traces with
activity identify themselves.** Write the mapping in here once and stop thinking
about it.

| Observation | Meaning | Next |
|---|---|---|
| Decodes ID 0x100, DLC 8, data bytes counting up | ✅ Data link layer confirmed | measure the bit time |
| Decoder flags no-acknowledge on the ACK slot | ✅ **Expected** — nothing out there to ACK | continue |
| TXD flat despite pre-flight reading 3.3 V | Channel mapping wrong, or the tap lead is open | swap channels, recheck the tap |
| Decode garbage | Bitrate not exactly 1000000, or dropped samples | fix bitrate, then drop to 12 MSa/s |

> **Bug that cost real time: the PulseView sample rate had been left at 20 kHz instead
> of 24 MHz.** Against a 1 µs bit time that is 0.02 samples per bit — the capture was
> structurally incapable of showing a frame, and it looks like a dead bus. **Read the
> sample rate off the toolbar before concluding anything about the hardware.** Same
> family as every other instrument-configuration failure in §12: the instrument
> reported success and returned nothing.

### The measurement S0 exists to take

Zoom into the 11-bit identifier field, put cursors on two edges **10 bit-times apart**,
read Δt.

**Expect 10.00 µs.** At 41.7 ns per sample that is 0.4% resolution over 10 bits —
comfortably enough to catch any clock error that matters, since CAN's total tolerance
is about ±1.5%. **Record the number: it becomes the independent check on the STM32's
computed bit rate at S1c.**

### Firmware notes

- **ESP32-S3 bring-up sketch uses the raw ESP-IDF TWAI driver** in NO_ACK mode, with a
  boot banner, heartbeat TX, non-blocking RX drain, and periodic
  `twai_get_status_info()` reporting.
- **Banner timing bug, fixed:** the banner printed before USB-CDC had enumerated, so it
  was never seen. Fixed with a `while (!Serial)` wait **plus a `b` key to reprint on
  demand** — the wait alone would hang a headless boot.
- **`p` pauses TX** so the pre-flight DC readings are static.

---

## 23.4b S1 — ESP32 ↔ ESP32, NORMAL mode — ✅ PASSED 2026-08-14

> ### ⚠ Restored 2026-08-14 — this section had gone missing
> A restructure of this file dropped §23.4b along with the S1 results table, the bus-off
> retraction and the single-shot rationale. Restored verbatim from the session copy.
> **These measurements exist nowhere else** — this is the only record of ~200k frames of
> two-node traffic — and the ESP32 sketch header, §23.2 and §§23.7–23.9 all cite this
> section by number. The retracted bus-off claim had also reappeared in §23.2's primer
> in the same restructure and has been corrected there again.

> **Numbering note.** The session report filed this as "§23.2". §23.2 is the termination
> section and is unrelated; S0's record is §23.4, so S1 lands here as **§23.4b**. The
> sub-letter follows the ladder's own convention (S1b, S1c, B6a/B6b).

**Build:** one `main.cpp`, three `platformio.ini` envs (`nodeA` / `nodeB` / `solo`).
`STAGE` and `NODE_TX_ID` arrive as `-D` flags. **The parent `build_flags` inheritance is
mandatory in the child envs** or `ARDUINO_USB_CDC_ON_BOOT` is silently dropped and
serial goes dead — a child `build_flags =` *replaces* the parent's, it does not extend it.

### Result over ~91k / ~113k frames

| Check | Node A (0x100) | Node B (0x101) | Verdict |
|---|---|---|---|
| State, TEC, REC | `RUNNING`, 0, 0 | `RUNNING`, 0, 0 | ✅ |
| `lastid` | 0x101 | 0x100 | ✅ bidirectional |
| `rxbad` | 0 of ~91,000 | 0 of ~113,000 | ✅ payload intact end to end |
| `rxmiss` | 4, **static** | 0 | ✅ startup artefact, benign because it stopped |
| `txfail` | 0 | 0 | ✅ |
| `arblost` | 0 | **29** | ✅ **positive result — see below** |
| `buserr` | 0 during S1 | 217,486, **static** | ✅ cumulative counter, stale from before A was flashed |

**`arblost = 29` on B and 0 on A is the arbitration proof, not a fault.** Both nodes
transmit every 5 ms from independent crystals, so their phases drift past each other and
they occasionally start a frame in the same bit time. CAN resolves it by identifier —
**0x100 beats 0x101**, so A always wins and B backs off and retransmits. Twenty-nine
collisions, zero lost frames, `rxbad = 0` on both. The lower ID winning every time is
what makes this a measurement of arbitration rather than of noise.

### Negative control: node B unpowered — and the retraction it forced

| Observation | Value |
|---|---|
| TEC | 0 → **128** (16 frames × 8), then **FROZEN** |
| State | stayed **`RUNNING`** — `twai_state_t` has no error-passive value |
| `buserr` rate | **7,683/s** → 130.2 µs per attempt |
| `qtx` | 9 (8-deep queue + 1 in flight), `sent` frozen at 127,991 |
| On reconnect | TEC → 0, transmission resumed **automatically**. No `r` required |

> ### ⚠ CORRECTED CLAIM — a lone node does **not** go bus-off
> Both the earlier notes and the `main.cpp` rev 2 changelog said a lone node in NORMAL
> mode climbs TEC by 8 per unacknowledged frame and reaches bus-off (TEC > 255) in about
> 32 attempts, within milliseconds. **That is wrong.**
>
> CAN fault confinement has an explicit exemption: **when a transmitter is already
> error-passive and detects an acknowledgement error, and sees no dominant bit while
> sending its passive error flag, TEC is not incremented.** So a lone node climbs
> 0 → 128, becomes error-passive, and **stops there permanently.** Bus-off needs
> TEC > 255, which requires a *different error class* — bit errors, form errors, a
> babbling node — not silence.
>
> The log is a clean demonstration: TEC stopped at exactly 128 = 16 × 8 and never moved
> again. **The bus-off message correctly did not print, because bus-off correctly did not
> occur.** The `r` keypress landed after recovery had already happened and was harmless —
> the recovery call returns an invalid-state error when the node is not bus-off.
>
> Note this is the rare case where general CAN knowledge and the rig agree exactly, so
> the source-of-truth ordering never had to be exercised: the exemption predicts a hard
> stop at 128, and the bench produced a hard stop at 128.
>
> ⇒ **TEC ≥ 128 is the only way to detect error-passive.** The state field will never
> show it. `main.cpp` rev 3 adds that check; rev 2 had none because it expected bus-off.

### The 130.2 µs is measured; the 114 µs frame time is not

The session report offered "114 µs frame + 17 bits of passive error flag, delimiter and
intermission = 131 µs, **matches the measured 130.2 µs to 1%**." **That agreement is
circular.** 114 µs appears nowhere in this doc set, and subtracting an assumed 17-bit
overhead from the measured 130.2 µs is the only place it can have come from — adding the
17 bits back then reproduces the number it was derived from.

What *is* measured is **130.2 µs per failed attempt**, from the bus-error counter rate.
The decomposition is model-dependent and not settled:

| Model | Bits before stuffing | Implied stuffing |
|---|---|---|
| Whole frame sent, then error flag + delimiter + intermission | 108 + 17 = 125 | ~5 stuff bits |
| Aborted at the ACK slot (no ACK delimiter, no end-of-frame), then the same 17 | 100 + 17 = 117 | ~13 stuff bits |

Both sit inside the 0–19 stuff-bit range for a DLC-8 standard frame, so **130.2 µs does
not discriminate between them and confirms no particular frame length.**

**This matters beyond bookkeeping**, because §23.5 and §23.6 both compute bus load from
an assumed **~130 bits** per frame — that is where "31% at 1 Mbit" and "62% at 500 kbit"
come from. S1 implies something nearer 113–125 bits for *this* payload, so those figures
are **conservative, not wrong**, and they stay as they are: the real control frame
carries the actuator contract, not ascending test bytes, and will stuff differently.

**Cheapest way to close it: one analyzer capture.** The FX2LP rig in §23.4 is already
configured — capture one frame on TXD and read the frame width directly. Five minutes, and
every bus-load figure in §23.5/§23.6 stops being an assumption.

> **DEFERRED (S1e), and the promoting condition is physical rather than temporal:
> the CAN message spec is frozen (§23.6).** Measuring the width of a frame carrying
> ascending test bytes answers the question for a payload the robot will never send. The
> capture is worth taking once — of the *real* control frame.

Provisional load figures, flagged as resting on that unmeasured frame length: **~4.6% for
the two-node S1 bus**, and **~27% extrapolated to 12 joints × 2 frames at 100 Hz**,
against §23.5's conservative 31%. Either way **1 Mbit has comfortable headroom** — which
is the only conclusion the frozen decision actually needs.

### Architectural requirement: the control path must be single-shot

**Adopt it.** ESP32: set the single-shot flag on the message. STM32G431: the **`DAR`**
bit in `FDCAN_CCCR` (disable automatic retransmission).

**The reason that holds on its own:** a control frame that missed its slot is worthless.
At a 100 Hz policy rate the next one is 10 ms away, and acting on a stale
`{p_des, v_des, kp, kd, τ_ff}` is **worse than acting on no frame at all**, because the
actuator would apply it. Retransmission buys nothing on this path and costs latency and
bandwidth. `txfail` then becomes a clean per-joint health signal instead of a silent one.

> ### ⚠ The bus-starvation argument for it does **not** hold, and is not recorded as if it did
> The session report justified single-shot with: *"one ESC1 loses its ACK partner — a
> connector backs out — and it hammers the bus at 7.7 kHz, ~88% occupancy, starving the
> other eleven joints. One loose connector takes down all twelve legs."* Three problems:
>
> 1. **ACK is sent by every receiving node, not by a partner.** On a 12-joint bus a
>    transmitter goes unacknowledged only if it is alone on its bus segment.
> 2. **If the connector backs out, that node is off the bus.** Its retransmissions are
>    then on a segment with nobody on it, consuming no bandwidth the other eleven want.
>    The stated trigger and the stated consequence cannot both be true at once.
> 3. **CAN already defends against exactly this.** *Suspend transmission*: an
>    error-passive node must send 8 recessive bits after intermission before starting
>    another transmission, precisely so it cannot out-compete error-active nodes. The 88%
>    figure was measured with **nobody else asking for the bus**; extrapolating it to a
>    populated bus drops the one rule that governs the populated case.
>
> **The real "one node takes down twelve" mechanism is the opposite error class.** A node
> with a *marginal* connection — not a clean open — raises **bit and form** errors while
> still error-**active**, and an active error flag is **6 dominant bits that destroy
> whatever frame is on the bus.** That one genuinely does take down all twelve, and it
> *is* the path to bus-off. **Single-shot does not fix it.** Wiring integrity and
> termination do — which is what the §23.2 12-board rework is for.
>
> **Provenance:** points 1–3 were general CAN knowledge when written — the lowest tier in
> `CLAUDE.md`'s ordering — and were **not** measured on this rig. **They are now.**
>
> > ### ✅ MEASURED 2026-08-15 — this box is confirmed, and S1d closes
> >
> > The discriminating test specified here was *leave node A alone and unacknowledged until
> > `TEC` parks at 128, then bring node B up and watch B's frame rate.* **That is exactly
> > what Test C ran** (§23.10): the ESP32 sat error-passive at `TEC = 128` retransmitting at
> > 7874/s, and the ESC1 booted straight into it. **Both were clean within ~640 ms** — B was
> > not throttled at all.
> >
> > **Suspend transmission does its job**, the starvation justification stays retracted, and
> > **S1d is closed.** Note the configuration was the *harder* one: the throttling node was
> > genuinely retransmitting, not single-shot. Single-shot continues to stand on the
> > stale-frame argument, which never depended on this either way.

### Still unresolved, low priority — do not spend time on it

**Why TEC stayed 0 in S0 (NO_ACK) with GPIO5 physically unconnected** (§23.4, finding 2).
Leading theory: the SJA1000-derived no-ACK/self-test mode loops TX→RX internally, so bit
monitoring passes regardless of the pin. **S1 has now proven the NORMAL-mode counters
work correctly**, which is the property everything downstream actually depends on.

---

## 23.5 The FDCAN kernel clock — ✅ CLOSED 2026-08-14. HSE is fitted and it is 8.000 MHz

> **Both facts are in. Outcome 1 applies and nothing else in this document had to
> move.** `HSERDY` sets, so a crystal is fitted and oscillating; `FDCANSEL = 00 = HSE`
> is used as-is; the frequency is **8.000 MHz**, identified three ways in §23.7.
> **Fact 2 — the `fdcan_ker_ck` maximum — never had to be answered at all**, because
> 8 MHz is an order of magnitude below any candidate cap. The measurement and the
> bit-timing consequences are in **§23.7** and **§23.8**.
>
> **Everything below the horizontal rule is retained as history, not as open work.**
> The kernel-clock source table, the 340 MHz VCO coupling, the SYSCLK-160 branch and
> the 500 kbit/s fallback are all **retired**: they were the map of what to do if no
> crystal existed. One did. `T_DELAY_PER_LOOP`, the PWM frequency and the loop rate
> are **untouched** — the coupling never fired.

**The two facts, as they stood before the measurement:**

| # | Fact needed | How to get it | Status |
|---|---|---|---|
| ~~**1**~~ | ~~**Is an HSE crystal fitted, and at what frequency?**~~ `RCC_CCIPR.FDCANSEL` resets to `00 = HSE`, so with no crystal FDCAN gets **no clock at all** on reset defaults | Poll `HSERDY` in firmware. Not by eye | ✅ **CLOSED 2026-08-14. Yes, and 8.000 MHz** (§23.7). The schematic's asymmetry — HSE pins wired, LSE pins crossed out — called it correctly, and the firmware poll settled it in seconds |
| ~~**2**~~ | ~~**What is the maximum permitted `fdcan_ker_ck` on the G431?**~~ | RM0440, RCC / FDCAN chapter | ✅ **MOOT, never answered.** It only mattered for the PCLK1 / PLLQ branches, which a fitted crystal deletes. **8 MHz is far below any candidate cap.** The 70%-confidence 80 MHz recall was never load-bearing and is not worth resolving now |

> ### Fact 1 — narrowed by the schematic, and there is a far better test than looking
>
> **What the clone schematic shows (§3):** pins 3 and 4 (`32Ki` / `32Ko`, the LSE) carry
> explicit **no-connect crosses**. Pins 5 and 6 (`osci` / `osco`, the HSE) carry **wires
> that continue off the visible page and are not crossed out.** On a schematic that marks
> its unused pins, that asymmetry is meaningful: **the HSE pins are used.**
>
> It is evidence, not proof — the nets could reach an unpopulated footprint, and **no
> crystal package is identifiable in the vendor's board photographs** at the available
> resolution. So the prior moves up substantially without closing.
>
> **The decisive test is software, not a magnifier.** Squinting at a 3225 package under a
> phone camera is the wrong instrument for a question the MCU can answer itself:
>
> ```
> 1. Set RCC_CR.HSEON. Poll RCC_CR.HSERDY with a ~100 ms timeout.
>       HSERDY sets    -> a crystal or oscillator IS fitted and oscillating.
>       times out      -> it is not. Outcome 3 in the table below, and the
>                         SYSCLK question becomes real.
> 2. If it sets, get the frequency: run SYSCLK from HSE with an assumed value,
>    then blink the STATUS LED (PC6) at a computed 1 Hz and time 60 flashes
>    against a stopwatch. An 8 vs 24 MHz mistake shows up as a 3x error --
>    unmissable, and it needs no analyzer.
> ```
>
> **Why not MCO:** the obvious move is to output HSE on `MCO` and measure it with the
> logic analyzer. **`MCO` is PA8 on this part, and PA8 is `TIM1_CH1` — a motor phase.**
> Do not reconfigure a phase pin to make a measurement; that is an instrument that can
> destroy what it measures. The LED-and-stopwatch route is crude, definitive to a factor,
> and safe.
>
> **This test costs ten minutes, needs no partner, no transceiver and no cable, and it
> unblocks the entire clock decision below.** If `HSERDY` sets, outcome 1 applies and
> nothing else in this document has to move.

| Kernel clock source | Frequency | Clean 1.000 Mbit? | Notes |
|---|---|---|---|
| PCLK1 | 170 MHz | ✅ NBRP = 10 → 17 tq (13 + 3 + SJW 1, 82.4% sample point) | Only if the 80 MHz cap is not real |
| PLLQ (VCO 340, /2) | 85 MHz | ✅ NBRP = 5 → 17 tq | 6% over an 80 MHz cap |
| PLLQ (VCO 340, /4) | 42.5 MHz | ❌ non-integer — closest is +1.19% error | Eats nearly the whole ±1.5% budget |
| PLLQ (VCO 340, /6) | 56.67 MHz | ❌ same problem | |
| **HSE, if a crystal exists** | 8 or 24 MHz | ✅ 8 tq or 24 tq, both exact, 87.5% sample point | **cleanest outcome by far** |

### The hidden coupling, and why it is worse than a lookup

The 170 MHz SYSCLK comes from a **340 MHz VCO**, and 340 has **no divisor that lands on
a clean sub-multiple of 1 MHz at or below 80 MHz.** So if the 80 MHz cap is real *and*
there is no crystal, reaching exactly 1.000 Mbit requires moving **SYSCLK to 160 MHz**
(VCO 320, PLLQ /4 = 80 MHz, NBRP 5 → 16 tq).

**That changes the PWM frequency and the FOC loop rate by 6%, which changes the
transport delay.** One constraint worth knowing before that branch is taken: the vendor
rates the board's **maximum PWM frequency at 50 kHz** (§2a) and we run 25 kHz, so a 6%
move has ample headroom on that axis — the cost is entirely in the measured constants,
not in the hardware.

```
CAN bit rate -> FDCAN kernel clock -> SYSCLK -> PWM frequency -> loop rate
             -> transport delay
```

**This is a frozen-decision-adjacent coupling that nobody costed when 1 Mbit was
chosen.** `T_DELAY_PER_LOOP` = 0.958 survives because it is a **ratio** — that is
exactly the property it was restated as a ratio for (§8.1) — but **every absolute
microsecond figure derived from it moves.**

| Outcome | Action |
|---|---|
| **1. Crystal present** | Use HSE. Nothing else changes. Best case |
| **2. No crystal, 170 MHz kernel clock permitted** | PCLK1 with NBRP 10. Nothing else changes |
| **3. No crystal, the 80 MHz cap is real** | Either **fit a crystal** (a few RMB, one solder job, and it improves bit-timing margin too), or accept **SYSCLK 160 MHz**, or **drop the bus to 500 kbit/s** where 42.5 MHz factorises cleanly (NBRP 5 → 17 tq) |

**500 kbit/s would re-open a frozen decision** and should only happen on this evidence,
explicitly logged. Load check: 12 joints × 2 frames × ~130 bits at 500 kbit = 6.2 ms
per cycle → **62% bus load at a 100 Hz policy rate.** Workable but tight; 1 Mbit gives
31%.

> ### ⚠ Standing rule for the eventual integrated firmware
> **Do not change the APB1 prescaler to fix the FDCAN clock.** It moves TIM2/3/4 and
> everything else on APB1 — and this project has a **bit-banged encoder whose timing is
> already characterised and a TIM4 path in its history.** Fix the FDCAN clock at
> `FDCANSEL` or the PLL, never at the bus prescaler.

**The S1c sketch was written this way and it worked** — every register read back and
printed, so the HAL's arithmetic is checked rather than trusted (§23.7).

> ### ⚠ RETRACTED: "external loopback proves the AF9 pin mux"
> The paragraph that stood here said external loopback *"proves the clock, the bit
> timing and the AF9 pin mux with no partner and no wiring involved."* **The last third
> is false.** The M_CAN specification the STM32 FDCAN implements is explicit: in
> external loopback the controller **performs an internal feedback from its Tx output to
> its Rx input, and the actual value of the Rx pin is disregarded.** Internal loopback
> disconnects the Rx pin as well.
>
> **So both loopback modes feed the receiver internally.** The only difference between
> `i` and `e` is whether the Tx pin is driven — and on this board there is no pad to
> observe it on. **`e` is expected to be byte-identical to `i` whether or not the mux
> works and whether or not the transceiver exists.**
>
> This is the kind of plausible-but-false claim that gets copied into a later session's
> reasoning, which is why it is boxed rather than deleted. **The mux is proven instead by
> `TEST.RX`, which monitors the real pin — see §23.7.** And "after that, S2 is just the
> cable" was optimistic for the same reason: S2 is the first test that exercises ACK,
> arbitration and the transceiver loop delay at all.

---

## 23.6 What still has to be designed, not just brought up

Transport is not the contract. Two artefacts remain unwritten and both are
Tier-0 boundary objects, so they should be frozen together:

| Artefact | Why it is coupled to the bus |
|---|---|
| **The CAN message spec** — frame IDs, packing and scaling for `{p_des, v_des, kp, kd, τ_ff}` → `{p, v, τ}` | Bit-rate and load figures assume ~130 bits worst-case (116 measured for the S1c test payload, §23.7) and 2 frames per joint per cycle. A different packing changes the load arithmetic |
| **The RL observation / action vector** | It has to be expressible in whatever the CAN contract carries, at the rate the bus supports. Freezing the message spec without it risks a policy that cannot be fed |

**✅ UNBLOCKED 2026-08-14.** These waited on the bit-rate decision, and **§23.5 is now
closed: 1 Mbit stands, HSE 8 MHz, no coupling to SYSCLK.** Neither was ever blocked by
the bearings or the analyzer either, which makes this the largest piece of legitimate
parallel work now available.

> ### Four constraints Tier-0 inherits, every one of them earned on the bench
>
> 1. **Control frames are single-shot** (§23.4b) — ESP32 single-shot flag, G431
>    `FDCAN_CCCR.DAR`, which the S1c build already sets. A frame that missed its 10 ms
>    slot is stale, and acting on a stale setpoint is worse than dropping it. **Confirmed
>    acting, in both directions, at §23.10:** frames discarded at 200 Hz with the partner
>    absent, and **0 drops in ~10,000 healthy frames.**
> 2. **🔴 Loopback is a power-on self-test only, with the bus quiescent.** A node in
>    external loopback **disregards its Rx pin**, so it cannot arbitrate and cannot detect
>    bit errors — **it babbles over everyone else.** Tier-0 must be in NORMAL mode before
>    any other node is enabled. This is the **error-active babbler** class §23.4b named as
>    the real one-kills-many mechanism, and **Test A (§23.10) produced it deliberately:
>    the partner went `BUS_OFF`.** Measured, not argued.
> 3. **Drive PC11 LOW before FDCAN goes to NORMAL.** On this clone the transceiver's `S`
>    pin is owned by the MCU, not strapped (§23.3). Firmware that forgets it leaves the
>    transceiver in **Standby with the transmitter disabled** — and the failure is nearly
>    silent: the node configures, queues, and reports no fault while nothing reaches the
>    wire. *(The same default is what makes a rebooting joint harmless — §23.10 Test C.)*
> 4. **Enable HSE and poll `HSERDY` with a timeout and a loud failure, before touching
>    FDCAN.** The kernel clock is the thing that goes wrong (§23.5), and a clock that never
>    came ready does not announce itself — it presents as a bit-rate mismatch three hours
>    later. **Loud** means the joint refuses to arm, not a line in a log.
>
> **And one design preference, which is not a constraint:** prefer a **polled /
> round-robin** bus discipline. Single-shot has a price — with two free-running nodes the
> arbitration-loss rate is ~0.026% (§23.4b), one dropped frame per ~20 s per node. A master
> polling in turn takes arbitration contention to zero and that price with it. Decide it
> *before* the frame IDs, because a priority-ordered ID map assumes contention exists.

### The Tier-0 FDCAN init checklist — **every value below is measured**

**Write Tier-0's FDCAN bring-up from this list rather than re-deriving it.** The bring-up
sketch reached these by measurement over four sessions; a fresh derivation would reach some
of them and quietly miss the rest. Steps, not code, so it survives a HAL version change.

| # | Step | Where the value came from |
|---|---|---|
| 1 | Enable **HSE**, poll **`HSERDY`** with a timeout, **fail loudly** on timeout | §23.5 — HSE is fitted and is the kernel clock |
| 2 | `RCC_CCIPR.FDCANSEL = 0` → **HSE** as FDCAN kernel clock | §23.5 — deliberately *not* PLLQ, so the bit rate is decoupled from any future SYSCLK change |
| 3 | Enable the **FDCAN peripheral clock** | Ordinary RCC housekeeping, listed so it cannot be skipped |
| 4 | Configure **PB9 (TX) and PA11 (RX) as AF9** | §23.3. **Do this before step 5** — recommended, see below |
| 5 | Drive **PC11 LOW** → transceiver Normal | §23.3, constraint 3 above |
| 6 | **`NBTP`: NBRP 1, TSEG1 6, TSEG2 1, SJW 1** → 8 tq, **87.5%** sample point | §23.8, forced by the measured **8.000 MHz** crystal |
| 7 | **`CCCR.DAR = 1`** — single-shot | Constraint 1 above |
| 8 | Enter **NORMAL** mode — never loopback with the bus live | Constraint 2 above |

**Step 4 before step 5 is recommended, not required.** The concern was that enabling the
transceiver while PB9 is still a floating input could put a dominant on the bus. **Test C
(§23.10) measured that window and it was clean — zero errors.** Keep the ordering anyway:
it costs two instructions and the alternative is betting twelve boards on an undocumented
internal pull-up in an unmarked clone part. But it is **no longer a known hazard**, and it
should not be recorded as one.

---

## 23.7 S1c — FDCAN loopback, the mux test, and the crystal — ✅ PASSED 2026-08-14

Standalone sketch, separate PlatformIO project (`CAN Bringup`), clock brought up by hand
because the clock is the thing that goes wrong. Every register read back and printed, so
the HAL's arithmetic is **checked** rather than trusted.

### S1c-i — internal loopback

| Evidence | Reading |
|---|---|
| `HSE ready` | Crystal oscillating; `HSEON` set by the sketch, `HSERDY` polled |
| `NBTP = 0x500` | NBRP 1, TSEG1 6, TSEG2 1, SJW 1 → **8 tq, 87.5% sample point.** The HAL wrote exactly what was asked |
| `CCCR = 0x10E0` | MON=1, TEST=1, INIT=0, CCE=0 → internal loopback, running |
| `TEST = 0x90`, LBCK=1 | Loopback engaged |
| `recv` = `sent` − 1 over 9,400 frames | The −1 is the in-flight frame at sampling time, not a loss |
| `rxbad = 0`, `TEC = REC = 0`, `LEC = 0` | No errors of any kind |

**Two confirmations nobody designed in.** `CCCR` bit 6 = `DAR` = 1, so **automatic
retransmission is already disabled** — the single-shot decision from §23.4b is live in
this build rather than pending. And `TEST` bit 7 = `RX` = 1 is the peripheral's readback
of the actual FDCAN_RX pin: **PA11 reads recessive**, corroborating S1b from inside.

⇒ **HSE reaches FDCAN, the peripheral configures, message RAM and FIFO work.**

### The crystal: identification, not metrology

The right framing, and it lowers the bar enormously. You do not need to *measure* the
crystal, only to **identify which standard value it is** — 8, 12, 16, 20, 24 or 25 MHz.
The closest pair is 24 vs 25, 4.2% apart, so **±1% is ample.** Once identified, the real
accuracy is the crystal's own ±50 ppm, not the measurement's. And a misidentification
cannot propagate silently: S2 would fail immediately with climbing error counters.

**Method: the FDCAN timestamp counter** (`TSCV`) increments once per CAN bit time off
the same HSE that clocks the peripheral. Count it against `millis()`, which runs off
HSI16 at spec ±1%. No analyzer, no wiring, ~2 minutes.

| Quantity | Value |
|---|---|
| Measured | **4,989,144 bit times in 5,000 ms** → 997.829 kbit/s → **7.9826 MHz** |
| Nearest standard candidate | **8 MHz**, −0.217% away |
| Next-nearest | 12 MHz, **+50.3%** away |
| Reference uncertainty | HSI16 via `millis()`, ±1% |

**The −0.217% sits inside the reference's own error, and the runner-up is 232× further
away than the winner. Identification is certain.** `HSE_ASSUMED_HZ = 8000000` was
already correct, so no constant changed.

**A result nobody designed for, which retired a 75%-confidence guess.** The method
carried a risk that `TSCV` might count only during frames rather than free-running. It
would have shown up unmistakably: `measureHSE()` calls `delay(5)`, which blocks `loop()`,
so after at most three queued frames drain the bus goes idle — a traffic-gated counter
would have read ≈0 and tripped the sketch's own guard. It read **997.8 kbit/s, i.e.
99.78% of wall-clock at one count per microsecond.** `TSCV` free-runs, empirically, and
is now a validated instrument for future clock work.

### The mux test `x` — what loopback could not prove

After `i` and `e`, exactly two things remained open on the ESC1 side: **the AF9 mux on
PB9, and the AF9 mux on PA11.** Everything else — transceiver present, powered, in
Normal mode, both pads reaching it, 125 ns loop delay — was measured at S1b with
**bit-banged GPIO, which bypasses the alternate-function path entirely.**

`FDCAN_TEST` bit 7 (`RX`) monitors the **actual** Rx pin, not the internal feedback.
Sampling it while transmitting in external loopback closes the loop through real
hardware: a dominant reading requires PB9's mux → transceiver → bus → receiver →
PA11's mux → peripheral.

**Result: 52.05% dominant.** The sketch's stated expectation was "20–50%", which was a
guess with no arithmetic behind it — **the measurement landed outside the band it was
told to expect, and the band was what was wrong.** The grounded prediction is ~50%:

| Frame field | Bits | Dominant |
|---|---|---|
| SOF | 1 | 1 |
| ID `0x200` = `01000000000` | 11 | 10 |
| RTR + IDE + r0 | 3 | 3 |
| DLC `1000` | 4 | 3 |
| Data (ascending counters, uniform) | 64 | ~32 |
| CRC | 15 | ~7.5 |
| CRC delim + ACK slot + ACK delim | 3 | 0 |
| EOF + IFS | 10 | 0 |
| Stuff bits (~5) | ~5 | ~1.5 |
| **Total** | **~116** | **~58 = 50.0%** |

Measured 52.05% against 50.0% predicted — a 2-point gap, inside the stuff-bit and
CRC-density uncertainty. **This matters more than clearing a threshold.** A stuck-dominant
input buffer reads ~100% and a dead mux ~0%; landing within two points of the frame's own
duty cycle is a far stronger pass than clearing a 1% floor, and it is why the sketch's
verdict is three-way rather than "any dominant at all".

⇒ **AF9 confirmed on both pins, and the TX → transceiver → bus → RX loop is closed
through real hardware.**

### A second crystal figure from data collected for something else

During the flood, `sent` went 1,201 → 18,519: **17,318 frames in 2 s = 8,659 frames/s.**
At ~116 bits that is **1.004 Mbit/s**, within 0.4% of nominal and consistent only with an
8 MHz kernel clock. It also shows the bus was ~100% occupied (17,318 × 116 µs = 2.009 s),
which is what makes the 52% duty figure interpretable at all.

> ### ⚠ "Confirmed three ways" overstates the independence — two ways, plus one pending
> The session report called the crystal confirmed by `TSCV`, by the flood rate, and by
> `LEC` never leaving 0. **The third is not a confirmation of anything about frequency.**
> The sketch's own header says it: in loopback, TX and RX share one clock, so **`LEC = 0`
> passes at *any* crystal value.** That is the whole reason mode `i` cannot settle the
> frequency.
>
> And the first two are **not fully independent** — both compare an HSE-derived counter
> against `millis()`, so both inherit the same HSI16 reference. What the pair genuinely
> cross-checks is whether `TSCV` means what we think it means, which is worth having.
>
> **It changes no conclusion.** The discrimination needed is 50%; the shared reference is
> good to 1%. Identification stands.
>
> ### ✅ RESOLVED at S2 — and the analyzer was not needed
> **S2 supplied the independent check this box called pending, from a direction neither
> of us planned.** In NORMAL mode the reference is the **partner's crystal (±20 ppm)**,
> not HSI16, so 21,801 error-free frames bound the ESC1's bit rate at **±0.97%** with no
> HSI16 in the chain. And the ESP32/ESC1 frame-count drift **measures** HSI16 at
> **+0.141%**, which corrects the `TSCV` number from 7.9826 to **7.9939 MHz (−0.077%)**.
> Both in §23.9. **HSE = 8.000 MHz, closed.**
>
> Note that `LEC = 0` *does* become a frequency bound in NORMAL mode, for the same reason
> it was worthless in loopback: the clock it is checked against is somebody else's.

### What this does to the frame-length question (§23.4b, S1e)

**Substantially answered, from two directions that agree.** The field-by-field count
gives **116 bits**, and the flood implies 1e6 / 8,659 = **115.5 bits** at 100% occupancy.
Both land inside the 113–125 range §23.4b bracketed, and they pin it near the bottom.

| Figure | Was | Now |
|---|---|---|
| S1 two-node bus load | ~4.6%, frame length assumed | **4.64%** at 116 bits |
| 12 joints × 2 frames @ 100 Hz | 27% assumed / 31% conservative | **27.8%** at 116 bits |

**§23.5's ~130 bits was worst-case stuffing** and stays valid as a conservative design
figure. The real control frame carries the actuator contract, not ascending test bytes,
so it will stuff differently — **116 bits is this payload, not the robot's.**

> ### S1e DEFERRED, and the promoting condition is physical, not temporal
> **Condition: the CAN message spec is frozen (§23.6).** Measuring the width of an
> ascending-counter frame with ID `0x200` says nothing about a
> `{p_des, v_des, kp, kd, τ_ff}` frame, because **stuff-bit count is data-dependent** and
> the packing does not exist yet. The 20 minutes costs the same later and is informative
> only then. **Deferring it is the point of the paragraph above**, which was already
> saying "116 bits is this payload, not the robot's" while still listing the measurement
> as pending — the two were inconsistent and the deferral resolves it.

---

## 23.8 Bit timing at 8 MHz — the crystal forces the configuration

**8 MHz ÷ 1 Mbit = 8 tq per bit with NBRP = 1. There is no other option**, and 8 tq is
the minimum the CAN specification permits. That is worth checking rather than assuming,
because "the only possible configuration" and "a configuration with adequate margin" are
different claims.

| Constraint | Requirement | Actual | Margin |
|---|---|---|---|
| **Propagation:** sample point > 2 × (transceiver loop + bus delay) | ≥ **253 ns** = 2 × (125 ns measured at S1b + 1.5 ns for 30 cm) | **875 ns** | **3.46×** |
| **Oscillator tolerance,** `min(PhaseSeg1, PhaseSeg2) / (2 × (13·NBT − PhaseSeg2))` | df ≤ **0.4854%** | crystal ±50 ppm = **0.005%** | **97×** |
| Oscillator tolerance, `SJW / (2 × 10 × NBT)` | df ≤ 0.625% | 0.005% | 125×, non-binding |
| Sample point | CiA convention | 87.5% | conventional |

With TSEG2 = 1, `min(PhaseSeg1, PhaseSeg2)` = 1, so the binding numerator is 1 and the
first tolerance row evaluates to 1 / (2 × (13 × 8 − 1)) = 1/206.

> ### The middle row proves the HSE decision was mandatory, not stylistic
> **HSI16 is specified at ±1% over temperature — more than double the 0.4854% budget**,
> by a factor of 2.06. A PCLK1- or PLLQ-derived kernel clock traces back to HSI16 on this
> board, and would have produced **a bus that works on a cool bench and fails
> intermittently when warm.**
>
> That was argued from general reasoning while the clock question was open. **It is now
> arithmetic against a measured tolerance requirement** — and it retroactively converts
> §23.5's "HSE is the cleanest outcome by far" from a preference into a constraint.
> A robot leg is also not a 0–85 °C environment by assumption, and HSI16's spread widens
> outside that range, which only strengthens it.

**A margin that is not yet tested.** The propagation row uses the S1b loop delay, but
**the transceiver loop delay is never exercised in loopback** — the internal feedback
bypasses the transceiver entirely. The 253 ns requirement gets its first real test at
**S2**, and that is the rung where a propagation problem would first be able to appear.

---

## 23.9 S2 — ESP32 ↔ ESC1 on real wire — ✅ PASSED 2026-08-14. **The CAN ladder is complete**

Predictions were written into this section **before** the run. They are scored against
outcomes below: a prediction that survived is confirmation, one that failed is a finding,
and one whose *method* failed is the most useful of the three.

### Scorecard

| Predicted | Outcome |
|---|---|
| ESC1 alone parks error-passive without hammering — TEC → 128, `LEC=ACK`, `EP=1` | ✅ **exactly**, and held flat for 1,200+ frames. `DAR = 1` demonstrating itself on real hardware |
| ESP32 after the partner appears: `RUNNING`, TEC = REC = 0, `lastid = 0x200` | ✅ all three |
| `rxbad = 0` | ✅ **0 across 21,800+ frames** |
| No bit/form errors | ✅ `buserr = 0` for the whole ESC1-first run |
| ESP32 `arblost = 0` — `0x100` wins every collision | ✅ 0 |
| Error-passive watch fires on partner loss | ✅ fired on the ESC1 unplug — the negative control |
| **`EW = 1` was NOT predicted** | ⚠ **and it is correct.** `EW` sets at TEC ≥ 96 and stays set past 128, so **`EP=1` + `EW=1` + `BO=0` is the error-passive signature.** Predicting `EP` alone was incomplete |
| ESC1 TEC 128 → 0 in ~640 ms once the partner starts | ✅ **confirmed 2026-08-15, from logs already taken.** Decrement is exactly 1 per successful frame: TEC = 11 at 117 frames, so 0 at 128 frames = **640 ms**. §23.10 Test D |
| ESC1 frames measure 10.00 µs per 10 bits on the analyzer | ⏸ **deferred, and now argued unnecessary** — see the box below |
| ESP32 `recv` lags ESC1 `sent` by ~0.026% | ❌ **method retracted — the measurement is confounded.** Replaced by the single-node Tx Event FIFO method below, which **ran and returned 0 drops in ~10,000 frames** (§23.10) |

**The §23.9 failure-branch table earned its place on its first outing.** Run 1 put the
ESP32 into `BUS_OFF`, and the table's entry — *"`BO=1`: bit or form errors past TEC 255.
**Not** a missing partner — that parks at 128"* — is what turned an alarming symptom
into a specific diagnosis in one step. See "Run 1" below.

### Wiring — power off first

| Connection | |
|---|---|
| ESC1 **CANH** → breakout **CANH** | twisted pair, 20–50 cm |
| ESC1 **CANL** → breakout **CANL** | |
| ESC1 **GND** → ESP32 **GND** | **mandatory** |
| **Meter CANH–CANL** | **expect 60–61 Ω** (121 ∥ 120). Anything else, stop |
| Motor | still disconnected |
| Analyzer | ESP32 **RXD** (GPIO4 tap) → 1 kΩ → CH2 = **D1**, GND tied. CAN decoder, 1000000 |

**The mixed 5 V / 3.3 V transceiver pairing is fine, and this is its first outing.** The
ESC1 biases recessive to ~2.48 V and the SN65HVD230 to ~1.65 V, so the shared bus idles
somewhere between. Both parts have differential thresholds (dominant > 0.9 V, recessive
< 0.5 V) and common-mode ranges (−7…+12 V and wider) that swallow that offset entirely —
the point §23.1 already records. **Note the actual idle common mode when you meter it:
it is a number worth having, and it is the first direct check on that claim.**

The analyzer must tap the ESP32's RXD, not the bus — see §23.3 for why CANH/CANL cannot
be threshold-decoded at 5 V. That line carries the **whole** bus, both nodes' frames and
the ACK bits.

### Order of operations — ESC1 first

1. **ESC1:** flash, press `n`.
2. **Expect it to reach TEC = 128, EP = 1, LEC = ACK within ~80 ms and park.** Correct:
   nobody is acknowledging. With `DAR = 1` it does **not** hammer.
3. **ESP32:** `pio run -e nodeA -t upload`. Press `b`, confirm the banner reads `NORMAL`
   and `ID = 0x100`.

> **The stated reason for this order is partly wrong, and the order is still right.**
> The report justified it as: if the ESP32 starts first it goes error-passive with
> retransmission enabled, floods at 7.7 kHz, and since `0x100` beats `0x200` the ESC1
> would struggle for a slot. **The starvation half does not follow** — an error-passive
> node must send 8 recessive bits of *suspend transmission* before each retry, precisely
> so it cannot lock out other nodes. This is the same correction already boxed in §23.4b.
>
> **Keep the order anyway, for a better reason:** starting the ESC1 alone is the only
> chance to watch `DAR = 1` park cleanly at TEC 128 with no partner, which is the
> §23.4b evidence worth capturing.
>
> **And if you do power up in the "wrong" order, do not restart — that run is S1d.**
> S1d asks exactly this question: does an error-passive node throttle a partner that
> joins later? If the ESC1 reaches full rate promptly with the ESP32 hammering, suspend
> transmission works and S1d closes for free.

### Predictions, as pre-registered before the run

| Quantity | Predicted | Basis |
|---|---|---|
| Bus load | **4.64%** | 2 × 200 frames/s × 116 µs (§23.7) |
| ESC1 `TEC` 0 → 128 | **~80 ms** | 16 frames × 8, at 200 Hz with `DAR = 1` |
| ESC1 `TEC` recovery once the ESP32 starts | **128 → 0 in ~640 ms** | TEC decrements 1 per successful TX, 128 frames at 200 Hz |
| ESC1 steady state | `recv` ~200/s, `rxbad = 0`, `TEC = REC = 0`, `LEC = none`, `EP = EW = BO = 0`, `lastid = 0x100` | |
| ESP32 steady state | `RUNNING`, `TEC = REC = 0`, `recv` ~200/s, `lastid = 0x200`, `rxbad = 0` | |
| Analyzer on D1 | Both IDs decode, **ACK slot dominant**, no warnings row | |
| **ESC1 frames on D1** | **10.00 µs per 10 bits** | **the real prize** — closes the crystal ID on the same instrument that read 10.007 µs for the ESP32 in S0, with no `millis()` and no HSI16 in the chain (§23.7) |

### The one prediction that puts a price on single-shot

The ESC1 runs `DAR = 1`, so a frame that loses arbitration is **dropped, not retried** —
and `0x100` beats `0x200` every time. From S1, node B logged **29 arbitration losses in
~113,000 frames = 0.0257%.** So:

> **ESP32 `recv` should lag ESC1 `sent` by roughly 0.026%** — about **one frame per
> 19.5 s** at 200 Hz.

`sent` counts frames *queued*, not frames *transmitted*, so a dropped frame does not
decrement it — which is exactly what makes this comparison able to see the loss.

> ### ⚠ RETRACTED: the cross-node method cannot see this signal
> The plan was to compare Δ`sent` on the ESC1 against Δ`recv` on the ESP32 over 5
> minutes. **Two confounds each swamp the effect, and S2's own data exposed both.**
>
> | Source | Magnitude over a 60,000-frame run | vs the ~15-frame signal |
> |---|---|---|
> | **Status-window misalignment** — each node's 1 s status timer free-runs, so "the same 5 minutes" carries up to ~1 s of boundary mismatch | **±200 frames** | **13× larger** |
> | **Timebase drift** — the two `millis()` differ by 0.1406% (measured, below) | **±84 frames** | **5× larger** |
>
> The irony is that the drift confound is *itself* one of S2's best results — it is what
> retires HSI16 from the crystal measurement. It is only a confound here.

**The fix is a single-node measurement**, immune to both because it never compares two
boards. The **Tx Event FIFO** records only transmissions that *completed*:

```cpp
tx.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;   // periodic TX only
static uint32_t tx_ok = 0;                       // completed, vs tx_count queued
```

Drained every pass of `loop()` — **the FIFO is only 3 deep on the G4**, so a slower drain
loses events and undercounts. With `DAR = 1` a frame that loses arbitration is discarded
and generates **no** Tx event, so **`sent − tx_ok` *is* the drop count**, measured on one
board against one clock.

Expected ~0.026%, i.e. **~15 drops in a 5-minute 60,000-frame run.**

> ### ✅ RUN 2026-08-15 — and the expectation was wrong in the safe direction
>
> **0 drops in ~10,000 healthy frames**, upper bound ~0.03% by the rule of three — so the
> prediction of ~15 was not contradicted, merely not reached at this sample size. The
> counter did move when it should: with the partner absent it climbed at 200 Hz, which is
> what confirms `DAR = 1` is discarding rather than the counter being dead. Full result in
> §23.10.
>
> **Priority was low, and the reason was architectural** — the frozen architecture is a
> **polled round-robin master**, contention zero, so this prices a hazard the robot will
> not have. It was taken anyway because it fell out of the same session's logs for free,
> and having the empirical figure before the message spec freezes (§23.6) is worth
> something. It is not worth a dedicated 5-minute run to tighten.


### HSI16 measured at +0.141%, which retires it from the crystal number

The ESP32's `recv` runs **ahead** of its own `sent`, and the gap grows: **+3 at
`sent = 2601`, +30 at `sent = 21801`.** Both nodes transmit at 200 Hz off their own
`millis()`, so the gap is pure timebase drift — the ESC1 is emitting frames slightly
faster than the ESP32.

> (30 − 3) / (21801 − 2601) = **+0.1406%.** The STM32's SysTick runs fast relative to the
> ESP32's clock.

The STM32's `millis()` derives from **HSI16 (spec ±1%)**; the ESP32-S3's from a **40 MHz
crystal (±20 ppm)**. So +0.141% is HSI16's trim error, comfortably in spec — and it is a
**reusable constant for anything else on this board clocked off HSI16.**

Now apply it to §23.7's `TSCV` measurement, which counted CAN bit times against that same
fast `millis()`. Its "5,000 ms" window was really **4,993.0 ms**:

| | Value |
|---|---|
| Raw | 7.9826 MHz, −0.217% from 8.000 |
| **Corrected for the measured HSI16 error** | **7.9939 MHz, −0.077%** |

**The correction moves it toward 8.000 by a factor of 2.8** — which is the behaviour a
real systematic error should show when you remove it, and weak evidence against the
alternative that the residual was noise.

> ### This resolves the box in §23.7 — the pending independent check arrived, without the analyzer
> §23.7 flagged that `TSCV` and the flood rate both compare against `millis()` and so
> share the HSI16 reference, and that `LEC = 0` **in loopback** proves nothing about
> frequency because TX and RX share one clock. **Both caveats are now discharged by S2.**
>
> **In NORMAL mode the reference is the partner's crystal, not HSI16.** Error-free
> exchange therefore bounds the ESC1's bit rate against something with no HSI16 in it:
>
> | Method | Bound on f_HSE | Reference |
> |---|---|---|
> | **S2: 21,801 frames, `LEC = none`, `buserr = 0`** | **±0.97% → 7.92–8.08 MHz** | ESP32 crystal, ±20 ppm |
> | Analyzer, 10-bit cursor span at 24 MSa/s | ±0.42% one-sample, **±0.83% two-cursor** | crystal-independent |
> | **Discrimination actually required** | **±25%** — nearest candidate is 12 MHz | |
>
> The S2 bound follows from §23.8's own arithmetic: per-node tolerance 0.4854%, so a
> two-node mismatch above **2 × 0.4854% = 0.97%** produces form errors in the 13-bit
> unstuffed EOF/IFS region. Zero such errors over 21,801 frames bounds it below that, and
> since the ESP32 is at ±20 ppm the whole budget belongs to the ESC1. *(A hand-check —
> one tq of drift accumulated over 13 bits = 1/(13 × 8) = 0.96% — agrees.)*
>
> ⇒ **HSE = 8.000 MHz. CLOSED.** And note what the table says about the analyzer: once
> two-cursor quantisation is counted it is **±0.83% against S2's ±0.97% — essentially a
> tie**, on a question that needs 26× less precision than either already delivers.

### Termination — ✅ CLOSED at 59.5 Ω, and the powered readings were an artefact

The powered bus read **53 Ω** where 121 ∥ 120 = **60.2 Ω** was predicted. That is not a
fault; it is a measurement taken in the wrong state.

| Configuration | Predicted | Measured | Implied extra parallel path |
|---|---|---|---|
| 3 terminators, powered | 40.1 Ω | 37 Ω | **477 Ω** |
| 2 terminators, powered | 60.2 Ω | 53 Ω | **441 Ω** |
| **2 terminators, unpowered** | **60.2 Ω** | **59.5 Ω** | **none** (−1.2%, inside meter tolerance) |

**The two implied values agree to 8%** — a single fixed ~440–480 Ω element present in
both powered readings and absent unpowered. That is the signature of a **powered
transceiver's recessive bias network**, pulling CANH and CANL toward VCC/2 through
internal resistors. Corroborating: S1 measured a two-node ESP32 bus at 60 Ω, and the only
thing that changed since is that a **powered** ESC1 joined.

*The ~450 Ω is lower than a typical datasheet bias network, so the exact mechanism is
worth a second look if anyone ever needs it. **The operational rule does not depend on
it** — the element is real, fixed, and vanishes when unpowered, which is all the rule
requires.*

> ### 🔴 Acceptance rule for the 12-board rework: MEASURE UNPOWERED
> With 12 powered ESC1s those bias paths sit at **~38 Ω in parallel**, which would swamp
> a 60 Ω termination reading entirely and make the acceptance test **uninterpretable** —
> a correctly reworked bus and a badly broken one would read about the same. **Target
> after rework, everything unpowered: ~60 Ω.**

**The 37 Ω also stands alone as the empirical over-termination datapoint** — below the
45 Ω minimum transceiver load, and the measured version of the argument §23.2 makes from
arithmetic alone.

### Run 1 — the ESP32 reached BUS_OFF, and that is a real finding

> ### ✅ CONFIRMED 2026-08-15 — the hypothesis below is no longer a hypothesis
>
> **Test A (§23.10) reproduced this deliberately.** Putting the ESC1 into mode `e` on a
> live bus drove the ESP32 to `BUS_OFF`, and the frame counters place it at the transition:
> bus-off at `recv = 9560` against ESC1 `txok = 9553`, **7 frames = 35 ms apart.** The
> ~60% below becomes a measurement, and the secondary termination hypothesis is dead.
>
> The reasoning is left standing as written, because the value of the record is that the
> right answer was reached from counters before it was reproduced.

Before the clean run, an attempt left the ESP32 in `BUS_OFF` with **`buserr = 34,710`**
and **`sent = 9`, `qtx = 9`** — nine queued, not one completed. §23.4b established that a
lone node **cannot** reach bus-off from ACK errors, so the ESP32 saw genuine **bit or form
errors**. Meanwhile the ESC1 showed only `LEC=ACK` and no `buserr`, so *its* frames went
out cleanly. Something was corrupting the ESP32's frames specifically.

**Leading hypothesis (~60%): the ESC1 was still in mode `e` when the ESP32 booted.**
Mode `e` was the last thing running from the mux test, and **external loopback disregards
the Rx pin** — so the node cannot lose arbitration and cannot detect bit errors. **It
transmits blind, every 5 ms, over whatever else is on the wire.** The ESP32 in NORMAL mode
sends recessive during arbitration, reads back the ESC1's dominant, raises a **bit
error** — the one class that reaches bus-off — and gets there in ~32 frames.

Everything else follows: a bus-off node neither transmits nor ACKs, so when `n` was
pressed the ESC1 was effectively alone → TEC = 128, `LEC=ACK`. Pressing `r` recovered the
ESP32 and both worked immediately, **on identical wiring** — which excludes wiring,
polarity and transceiver faults.

> **A rebuttal that does not hold, recorded so it is not re-made.** It was argued that the
> ESC1 log shows `CCCR = 0x1040, TEST = 0, LBCK = 0`, i.e. not loopback. **The ESC1 log
> begins at the moment `n` was pressed.** It says nothing about the earlier window in
> which the ESP32 accumulated 34,710 bus errors, and the ESC1's state immediately before
> `n` was mode `e`, left running from the mux test.

**Secondary hypothesis (~25%), now weaker: marginal termination.** The unpowered 59.5 Ω
and the fully explained powered readings leave little room for it, and node B was off for
these runs.

> ### 🔴 Tier-0 requirement, whichever hypothesis survives
> **A node in external loopback is a babbling node that does not arbitrate.** Never enter
> loopback on a live bus. **Tier-0 must treat loopback as a power-on self-test only, with
> the bus quiescent, and must be in NORMAL mode before any other node is enabled.**
>
> This is a concrete instance of the **error-active babbler** class identified in §23.4b
> as the real one-kills-many mechanism — the one that single-shot does *not* fix. §23.4b
> could only argue it. **Run 1 is it happening on the bench by accident, and Test A
> (§23.10) is it happening on purpose.** The requirement rests on a measurement, and it is
> now constraint 2 of the four in §23.6.

### Failure branches — `LEC` discriminates almost everything

| Symptom | Diagnosis |
|---|---|
| ESC1 `LEC=ACK`, `EP=1`, ESP32 silent | Partner not running or not on the bus. Check the ESP32 banner says `NORMAL` |
| ESC1 `LEC=STUFF` or `FORM`, both nodes erroring | **Bit-rate mismatch — the crystal is not 8 MHz after all.** Go measure a 10-bit span on D1 |
| ESC1 `LEC=CRC`, sporadic | Signal integrity: termination, cable length, ground tie |
| ESC1 `LEC=BIT0` | Sent dominant, read back recessive → driver not reaching the bus |
| ESC1 `LEC=BIT1` | Sent recessive, read back dominant → another node driving, or a short |
| `BO=1` on either node | Bit or form errors past TEC 255. **Not** a missing partner — that parks at 128 (§23.4b) |
| All pass but ESP32 `rxmiss` climbing | Printing stall, not a bus fault (§23.4, finding 1) |


---

## 23.10 Tests A–D — the robustness questions — ✅ **A, C, D PASSED 2026-08-15** · B dropped

The ladder proved the bus **works**. These four asked what it does when something goes
**wrong**, which is the part a 12-joint robot actually depends on. Three ran and all three
passed. **Test C carried the largest consequence of the four and it came back clean.**

| Test | Question | Result |
|---|---|---|
| **A** | Does a node in loopback babble over a live bus? | ✅ **PASSED — yes it does.** §23.9's run 1 is explained, and the Tier-0 loopback rule is now measured rather than argued |
| **B** | Does an error-passive node throttle a partner that joins later? *(= S1d)* | **DROPPED as a separate run.** The question was answered by **Test C's recovery leg**, and single-shot forbids the configuration on the control path anyway. **S1d closes — see below** |
| **C** | Does a rebooting joint disturb the other eleven? | ✅ **PASSED — no.** The clone's Standby fail-safe is real and measured. **One joint resetting is a local event** |
| **D** | How fast does `TEC` walk back from 128? | ✅ **PASSED**, found in logs already taken. The decrement law is exact — but the headline number needed correcting, below |

---

### ⚠ The confound running through all three: `buserr` climbing is **not** evidence of a jammer

**Get this right first, because the naive reading is backwards and it decides A and C in
opposite directions from a symptom that looks identical.**

Whenever a node loses its only ACK partner — for *any* reason: reset, loopback, unplugged
connector — it logs **one ACK error per transmission attempt**, and because it retransmits,
that is ~7.7 kHz of `buserr`. A climbing `buserr` therefore says only **"nobody answered"**.
It says nothing at all about whether anyone is driving the bus.

**`TEC` is the discriminator, and it is a clean one:**

| Error class | `TEC` behaviour | Endpoint |
|---|---|---|
| **ACK** error, transmitter already error-passive | **Not incremented** — the fault-confinement exemption (§23.4b) | Parks at **128 and freezes**, state stays `RUNNING` |
| **Bit** or **form** error | **+8 every time, no exemption** | `TEC > 255` → **`BUS_OFF`** in ~32 errors ≈ **4 ms** at this rate |

> **The rule that decided both tests: `TEC` frozen at 128 means silence. `BUS_OFF` means
> somebody is driving the bus.** Test A reached bus-off. Test C parked at 128.

---

### Test A — ✅ PASSED. A loopback node does babble, and run 1 is explained

**Result.** With the ESC1 put into external loopback (`e`) on a live bus, **the ESP32 went
`BUS_OFF`.** Under the rule above that is only reachable from bit or form errors, so the
ESC1 was transmitting into frames it could not see.

**The frame counts pin it to the transition:** ESP32 bus-off at `recv = 9560`, ESC1
`txok = 9553` — **7 frames apart, 35 ms at 200 Hz.** Bus-off needs ~32 bit errors ≈ 4 ms,
so a 35 ms delay is the right order and slightly long, which is what you would expect if
not every loopback frame lands on top of an ESP32 frame. The timing is quantitatively
consistent with the hypothesis, not merely coincident with it.

**§23.9's run 1 is therefore explained**, and promoted from a 60% hypothesis to a measured
failure mode. The rebuttal it had to survive is recorded there so it is not re-made: the
ESC1 log began at `n`, so `CCCR = 0x1040` describes the window *after* the transition and
says nothing about the one before it.

**What this earns.** The **error-active babbler** class that §23.4b named as the real
one-kills-many mechanism now exists on this bench, deliberately produced, with the
counters to show it. The Tier-0 loopback constraint (§23.6) rests on a measurement.

---

### Test B — DROPPED as a run, and **S1d closes anyway**

**Two independent reasons, and the second is the stronger one.**

1. **It tests a configuration single-shot forbids.** Test B's premise is a node
   retransmitting endlessly at 7.7 kHz while a partner tries to join. With `DAR = 1` on the
   control path no node ever retransmits, so that traffic pattern cannot occur on the
   shipped bus. Measured for the ESC1 at the same session: with the partner absent, frames
   were **discarded, not retried** (`drop` climbing at 200 Hz, not 7.7 kHz).
2. **The informative half already ran, as Test C.** During Test C the ESP32 was error-passive
   at `TEC = 128` and retransmitting at 7874/s — the throttling node, present and active —
   and the ESC1 booted straight into that bus. **Both were clean within ~640 ms.** That is
   §23.4b's discriminating test, executed, with the *harder* variant of the configuration
   (the throttler was genuinely retransmitting, not single-shot).

> ### ✅ S1d — CLOSED, PASSED
> **Suspend transmission does its job.** An error-passive node does **not** lock out a
> partner that joins later. §23.4b's box — which argued the bus-starvation justification
> for single-shot does not hold — is **confirmed by measurement**, and the session report's
> "one loose connector takes down all twelve legs" stays retracted. Single-shot continues
> to stand on the stale-frame argument, which never depended on this.

---

### Test C — ✅ PASSED. **A rebooting joint cannot disturb the other eleven**

**Result.** ESC1 `RST` shorted to GND and held ~9 s, three times, with the ESP32 watched
throughout. **The ESP32 never left `RUNNING`.** `TEC` climbed to 128 and froze.

The decisive comparison is against the S1 negative control, where the partner was
definitively silent because it was **unpowered** — the known-silent reference:

| Signature | S1: partner unpowered | Test C: partner held in reset |
|---|---|---|
| `TEC` | 128, frozen | 128, frozen |
| State | `RUNNING`, never `BUS_OFF` | `RUNNING`, never `BUS_OFF` |
| `qtx` | 9 (8 queued + 1 in flight) | 9 |
| `sent` | frozen | frozen at 11422 |
| **`buserr` rate** | **7683 /s** | **7874 /s** |
| Recovery when the partner returned | automatic, `TEC` → 0 | `TEC` 128 → 97 → 0 |

**Six signatures, all matching.** The 2.5% rate difference is **3.2 µs of frame period**
(130.2 µs vs 127.0 µs) — about **3 bit times**, i.e. stuff-bit variation from a different
`tx_count` payload. And 130.2 µs is not a fitted number: it is the figure already recorded
as measured in §23.4b, so the S1 column of this table cross-checks against a capture taken
two days earlier.

**The decisive argument is the bus-off that never happened.** Over ~9 s the ESP32 logged
~70,000 errors. If **any** of them had been bit or form errors — the signature of the ESC1
driving the bus — `TEC` would have passed 255 within milliseconds. It parked at exactly 128
and stayed there, which happens **only** for ACK errors under the error-passive exemption.
**Nobody was jamming; nobody was answering.**

**Prediction confirmed.** `PC11` floats during reset → the transceiver's internal pull-up
selects **Standby** → transmitter disabled. The clone's fail-safe is real, measured, and it
means a joint rebooting on a 12-node bus is a **local** event.

> ### ⚠ RETRACTED: this section's own pass criterion was impossible
> The pre-registered outcome table gave **"ESP32 stays `RUNNING`, `buserr = 0` through the
> hold"** as the clean result, and **"`buserr` climbs, or it goes error-passive"** as the
> 🔴 failure. **Both are wrong, for the same reason.** With the ESC1 in reset the ESP32 has
> no ACK partner, so `buserr` *must* climb and it *must* go error-passive — in the clean
> case and the jamming case alike. A criterion no outcome could satisfy would have read a
> pass as a failure.
>
> **The correct criterion is the `TEC` rule at the top of this section**, and it was the
> bench that supplied it. Recorded rather than silently fixed: the error was writing a
> prediction in terms of the symptom that was easiest to see instead of the one that
> discriminates.

#### The third outcome is ruled out too — the firmware ordering window is empirically clean

`buserr` froze at **297801** and never moved again, **including through the window where
`n` was pressed and `startFDCAN()` drove PC11 LOW before configuring PB9 as AF9.** Zero
errors during the transceiver-enable window.

**So the ordering concern downgrades from Tier-0 requirement to recommended practice**
(§23.3). Still configure PB9 before PC11 — betting twelve boards on an undocumented
internal TXD pull-up in an unmarked clone part is not a trade worth making for two
instructions of ordering — but it is **no longer a known hazard.**

> ### Scope caveat — what this did **not** test
> This was **MCU reset with the board powered.** A joint **power-cycling** while the others
> run is a different case: VCC ramping on the transceiver itself, not just a floating mode
> pin. The same fail-safe should apply and there is no reason to expect otherwise, but it
> is **untested**. Not worth a bench trip now. **Promoting condition: the first power
> distribution capable of cycling one joint independently of the bus.**

---

### Test D — ✅ PASSED, from logs already taken — with the headline number corrected

**The decrement law is exact.** `TEC` falls by **exactly 1 per successful transmission**:
the log shows **`TEC = 11` after 117 successful frames** from 128, and 128 − 117 = 11 to
the frame. Nothing is fitted here; two independently-counted integers close.

> ### ⚠ Correction to the reported figure: **640 ms, not 585 ms**
> The session report headlined this as *"`TEC` 128 → 0 in 117 frames = 585 ms"*. **That
> conflicts with its own supporting arithmetic in the same sentence.** If `TEC` is 11 at
> 117 frames, it is not yet 0; reaching 0 takes **128** frames.
>
> | | Frames | At 200 Hz |
> |---|---|---|
> | Observed, `TEC = 11` | 117 | 585 ms |
> | **Full recovery, `TEC = 0`** | **128** | **640 ms** |
>
> **640 ms is the constant**, and it is exactly what §23.9 predicted. The 585 ms figure is
> the length of the observation window, not the recovery time. Recorded because a recovery
> constant that is 9% optimistic is the kind of number that gets designed against later.

**Prediction confirmed**, and it was worth having: this is the first Tier-0 timing constant
the bus produced. A joint that loses and regains its partner is fully back inside **one
policy cycle at 100 Hz**, so no Tier-1 rejoin logic is needed — the hardware handles it.

---

### Drop rate — **0 in ~10,000 frames**, and `DAR` confirmed in both directions

The Tx Event FIFO method proposed in §23.9 was run, and it settles the last open number
from S2. `drop` = `sent` − `txok`, counted on one node, immune to the cross-node window
and timebase confounds that forced the earlier method's retraction.

| Observation | Reading |
|---|---|
| `drop` while the partner was absent | **climbing at 200 Hz** — every frame discarded, none retried. `DAR = 1` confirmed *acting* |
| `drop` after each rejoin | **frozen**, at 2248 and then 4137 | 
| New drops over ~10,000 healthy frames | **0** |
| 95% upper bound (rule of three, 3/N) | **~0.03%** |

**Both directions matter.** A `drop` counter that never moved would not prove `DAR` was
set — it could equally mean the counter was broken. It moved when it should and stopped
when it should.

**A bonus cross-check nobody planned.** The second drop window spans 4137 − 2248 = **1889
frames = 9.4 s** on the ESC1's clock, against Test C's ~70,000 ESP32 errors at 7874/s =
**9.0 s** on the ESP32's. Two independently-clocked instruments timing the same reset hold
and agreeing to 4%. *(Assignment of that window to the Test C hold is inferred from the
duration, not logged directly — treat it as corroboration, not as a primary measurement.)*
