# Failure-mode catalogue and diagnostic ladder

§12 every failure mode that cost at least one session · §13 the diagnostic ladder.

**Read this before believing any number.**

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 12. Failure-Mode Catalogue

Each of these cost at least one session.

**Toolchain / build**
- Unpinned platform silently upgrades the core.
- `^2.3.1` → dead motor (EG2124A contract).
- Two files with `setup()`/`loop()` → clean build, wrong firmware.
- Missing `lib_archive = false` → linker drops 6-PWM.
- **Library defaults are silent decisions.** `foc_modulation` defaulted to `SinePWM` for the whole project, costing 13.4% of the voltage ceiling with nothing in any log to say so. Echo every load-bearing default in the boot banner.

**Board / pin map**
- Vendor-pruned pin map → `Error_Handler()` infinite loop, not an error return.
- PB8 = BOOT0.
- **`NOT_SET` reads as `-12345`, not as an error.** `pwm_frequency` printing `-12345` means "never assigned", and the platform default is silently in force.

**Sensor / alignment**
- ZEA hardcoded across boots → rotor locks at full current.
- `Skip … calib` = stored value reported back, not a measurement.
- `initFOC()` with the driver disabled → "Failed to notice movement".
- Software quadrature loses counts above ~100 k edges/s → silent ZEA corruption.
- **Magnet slipping in its mount** — CLOSED, kept as history. Presented as escalating current at constant speed ending in a stall that realignment "fixed". CA was what failed. **Keep the pen mark.**
- Steel screw through a diametric magnet corrupts the field entirely.

**Control / measurement**
- **`motor.current` is NOT updated in voltage mode.** 2.3.1's `loopFOC()` returns from the `voltage` branch without touching it, so `Iq`/`Id` in TORQUE(V) and OPENLOOP are whatever `initFOC` or the last current-mode run left behind. **This would have silently invalidated every Uq-vs-Iq measurement in the campaign** — a frozen number that reads exactly like data. Fixed by refreshing `motor.current` explicitly. `|I|` was always live, which is why it hid for so long.
- **A blocking telemetry print freezes commutation.** During the print block `loopFOC()` does not run, so `setPhaseVoltage()` stops updating while the PWM timer keeps its last duty cycles — the voltage vector is **frozen in space while the rotor keeps turning.** At 96 rad/s and a 5.7 ms print, the rotor sweeps 61% of an electrical revolution and the back-EMF comes into anti-phase with the frozen vector: `(Uq + E)/R = (2.00 + 1.70)/0.218 = 17 A`. Observed `|I|` up to 15.5 A while `Iq` read 1.0 A. **Mitigated 7× by 921600 baud; the proper fix is a chunked non-blocking emit.** Also the most likely cause of the unexplained mid-run load step in the Ke sweep.
- **A capture taken with the motor disarmed looks perfectly healthy.** `Uq = 2.0` and `vel = 2.0` are both stale-but-plausible; only `|I| = 0.053` gave it away. Fixed by recording `m=` and `run=` at `logStart` and printing them with every dump and stats line.
- **A measurement filter inside a loop is part of the loop dynamics** — but it is a *damping* lever, not a bandwidth lever. Below the optimum, lowering it makes the loop slower.
- **Gain ratio beats filter tuning.** Three sessions were spent on `CUR_TF` while `CURQ_P/CURQ_I` was 2.7× off the plant. Measure `L`, compute the ratio, then tune the filter.
- Dead time is a **voltage dead zone**, not just a safety margin — but at `dead_zone = 0.005` it is now **below the sense noise floor** and the question is closed.
- **`Id` cannot detect commutation drift in current mode.** The d-axis loop regulates the *apparent* Id to zero in its own rotated frame and succeeds even when misaligned. **The symptom disappears; the `cos(δ)` torque loss remains.** In *voltage* mode Id is an honest open-loop diagnostic — the original lesson applies to current mode only.
- **`current_limit` does not bind in voltage torque mode.**
- Torque mode has **no speed limit**.
- Stale state latches. **Always ask who updates a value and when.**
- Raw `|I|` is meaningless when the drive saturates.
- A ramp test hides marginal stability; a **step** test exposes it.
- A guard on a noisy signal needs debouncing.

**ADC sharing / silent stale values (2026-08-01)**
- **Arduino `analogRead()` returns 0 on ANY pin of the ADC that `LowsideCurrentSense` owns, from the moment `currentSense.init()` runs.** Not a contention-between-two-calls problem — a *single isolated* call fails. Proven by `vraw=0` printed from the telemetry block while the pre-init seed read in `setup()` works every boot. ~~Cause: the current sense arms the ADC for TIM1-triggered **injected** conversions~~ → **mechanism corrected 2026-08-21, see the box below** — it is the **regular** group. `HAL_ADC_Start` returns BUSY either way. **The PWM timer runs whether or not the motor is enabled**, so this fails even in disabled OPENLOOP. Same family as the pruned pin map: the Arduino layer fails silently instead of erroring.
- **RETRACTED — the "+1.1% `Vbus` rise under load" was never real.** Because the loop sampler never updated once, every `Vb` ever displayed was that boot's `setup()` seed. Two *different boots* (11.30 and 11.42) were compared as if they were one run, and a sample-and-hold residue mechanism was invented to explain the difference. **"Zero drift over 20 s" was the tell and was read as a pass — a frozen value looks exactly like a perfectly stable measurement.** Ask who *writes* a value and when, not what it reads.
- **A dummy `analogRead()` added to flush that imaginary residue broke nothing further, because nothing was working.** Lesson: a fix for a 1.1% artefact was shipped against a 100% failure that was already present and invisible.
- **The remedy is a flag only one branch writes.** `vbus_valid` starts `true` in `setup()` and is only ever cleared by the sampler, so a single telemetry field answers "is the sampler running?" — it resolved in one glance what three turns of hypothesising could not. Design guards this way deliberately.
- **A diagnostic that mutates the state it reports makes its own "before" reading unreliable.** The temporary `z` probe set `vbus_valid = false` as its test action, so a second press showed a misleading `before:` line. Prefer a passive telemetry field over a state-disturbing probe.
- ~~**Correct sharing mechanism, for when this is implemented properly:** STM32 ADCs run **regular** and **injected** groups concurrently on one peripheral. Injected preempts, regular resumes. VBUS belongs on a register-level regular conversion.~~ **RETRACTED 2026-08-21 — true of the silicon, false of this firmware.** See the box below. The reasoning was correct for a chip and never checked against the peripheral, which is the §12 "reasoning from a datasheet instead of the rig" pattern applied to a register map.

> ### ⚠ After `currentSense.init()`, `analogRead()` is FORBIDDEN on this board
>
> Measured 2026-08-21 by the `p` probe on J02, register dump:
> **`JSQR = 0` and `JADSTART = 0` on BOTH ADC instances — there is no injected group at
> all. `ADSTART = 1` on both.** SimpleFOC's `b_g431` low-side path runs the current sense
> on the **regular** group with circular DMA (§3 has the sequence).
>
> **It fails safe today, and only by accident.** `analogRead()` returns 0 because
> `HAL_ADC_Start` finds the regular group already started. But it is not reading a
> peripheral that is busy elsewhere — **it is contending for the very sequence the current
> sense is using.** A core or HAL version that stops the ADC first would let the call
> through, and it would rewrite `SQR1` and `SMPR` to its own single-channel conversion.
> Result: no error, no exception, no log line — the phase currents land in the wrong slots
> or stop converting, `|I|`/`Iq` go to garbage, and **every current-derived constant taken
> after that point is wrong.**
>
> The rule is not "analogRead doesn't work here". The rule is **any ADC access after
> `currentSense.init()` must be a pure read of state the current sense already produces**
> — which, for VBUS, it does: PA0 is rank 5 of that same sequence and already lands in the
> DMA buffer. No configuration, no pausing, no blind interval in the current loop.
> Pausing would be an instrument that disturbs what it measures (§6 of the working-context
> doc), and at 30 A a blind interval is not cosmetic.
>
> ⚠ **The buffer slot reads 60 counts low, and the cause is named: the converter feeding it
> has never been calibrated.** `analogRead()` runs `HAL_ADCEx_Calibration_Start()` on every
> call; SimpleFOC's `b_g431` init does not, so `CALFACT = 0` on both instances. Seed = a
> calibrated converter, DMA = an uncalibrated one. Confirmed the way a mechanism should be:
> calibrating before `driver.init()` moved each instance's reading by **exactly its own
> `CALFACT`** (ADC1 117 → +116.3, ADC2 113 → +114), uniformly across all three channels,
> with the seed unchanged. H1, H4, H6 and H7 are all dead — §0 has the evidence for each.
>
> **`buf[4]` is still not consumable, for a different and smaller reason:** the calibration
> as first written was taken at the wrong ADC clock and over-corrects by 1.97×. See §0.
> **The prohibition above is unaffected by any of this** — it is about who owns the regular
> group, not about what the numbers say.

**Sensor architecture (2026-08-06)**
- **An incremental counter cannot detect its own error.** ABZ lost ~60 counts (1.5% of a revolution) under 8–11 A; the sketch comment claimed "cannot lose counts", which was true of the TIM4 decoder and false of the whole chain — TIM4 faithfully counts noise. **The comment encoded a belief that made the failure mode invisible for months.**
- **Absolute + parity changes the failure class, not the noise.** SPI still sees the same EMC environment, but a corrupted frame fails parity, is rejected, and the angle recovers in one cycle instead of drifting permanently. Measured: 2 events in 600 k reads.
- **`analogRead()` returns 0 on any pin of the ADC that `LowsideCurrentSense` owns**, from the moment `currentSense.init()` runs. A single isolated call fails — not a two-call contention issue. Same silent-failure family as the pruned pin map.
- **Third instance of the pruned-pin-map trap:** UART on PB6/PB7 fails for the same reason `STM32HWEncoder` and Arduino hardware SPI did. **Rule: on this board, any peripheral on a non-default pin needs register-level setup or bit-banging.**
- **Freed CPU with no other work to run is worth nothing.** DMA does not shorten a transfer — SCK frequency does. On a single blocking control loop the distinction between "CPU busy" and "CPU idle waiting" is not a distinction.
- **A timing comment written from a model is a prediction, not data.** The bit-bang NOP table was 6× optimistic because GCC unrolls small loops and not large ones. `us_per_read` in the self-test settled it in one run. **Build the measurement into the feature.**

**Measurement design (2026-08-07)**
- **A gate is only valid in the regime it was characterised in.** The `|I|`/`Iq` check reads 1.27–1.36 free-spinning at `Iq` = 0.10–0.18 A, because `|I|` is always positive so `mean(|I|) > |mean(I)|` under ripple, and the bias grows as the DC current shrinks (+8.1% at 0.100 A → +3.9% at 0.179 A). §11 already documented the effect; AUTOCALIB applied the tight gate anyway and produced a **false FAIL**. Fix: accumulate in the **squared** domain — `|I|² = 1.5·(Iq²+Id²)` instantaneously, so one sqrt at the end carries no bias. The tight 1.22–1.23 check is **locked-rotor only**.
- **Averaging by sample index throws away timing jitter that is actually useful.** The L step train's phase relative to the control loop is random across repeats — free equivalent-time sampling. Index-averaging smeared it and left 5 fit points; binning the same data by **time** gives ~17.
- **Third occurrence of the two-point trap.** `R_eff` = 0.1977 from 2 points versus **0.22184 from 9** (rms 1.76 mV, SE 0.54%). Two points are exactly determined — no residual, no way to detect a bad point. **And it became load-bearing:** it was the sole evidence for a "9.3% inter-assembly difference" that is now retracted. The true spread is +1.8%.
- **A constant that scales everything and cannot be self-checked must be demanded, not assumed.** `VBUS_SCALE` puts its full error onto `R_eff`, `U0` and `Ke`. No internal reference exists, so the report now prints a blank line for a multimeter reading instead of staying silent about it.

**Interpretation**
- 300 ms telemetry aliases everything above ~1.7 Hz.
- **Forward-only data cannot separate an even term from an odd one.** Three sessions of angle-lag work produced T = 85.7 µs with a 28.7 µs unexplained excess. Running the same measurement in **both directions** and splitting into even and odd parts gave T = 57.0 µs with a 3.6 µs excess, plus the INL profile and the ZEA residual, from two captures. **When two mechanisms have the same shape in one dataset, change the experiment, not the model.**
- **A test's noise floor must be measured before its result is interpreted.** Six null-condition ZEA repeats took two minutes and turned "the test is undoable" into "average four".
- **A fit can measure one parameter superbly and another not at all.** A 14-point locked-rotor sweep pinned `R_eff` to ±1.55% while its intercept `U0` sat 4.6σ from zero with a 21.8% standard error. Report the well-conditioned parameter; say plainly that the other is unresolved.
- **Check phase coverage before specifying repeats.** A `L` capture at 109 rad/s spans **9.5 revolutions** and 106 samples/rev — repeats add no phase diversity. At 21 rad/s the same capture spans 1.2 revolutions and the 1/rev disturbance lands *in* the answer. **Compute revolutions-per-capture, don't assume it.**
- **A fit can measure one parameter superbly and another not at all.** The 2026-08-01 14-point locked-rotor sweep pins `R_eff` to ±1.55% (0.10σ from the table) while its intercept `U₀` lands 4.6σ from zero with a 21.8% standard error; subsetting the points swings `U₀` from 0.019 to 0.042 while `R` moves the other way. **Report the well-conditioned parameter and say plainly that the other is not resolved** — do not overwrite a dedicated measurement with a byproduct.
- **Aliased telemetry can still be right for the wrong reason.** The 1/rev disturbance was correctly guessed from 2.11-samples-per-revolution telemetry — right at Nyquist. The guess only became a finding after a proper position fold. Don't promote a marginal reading to a conclusion.
- Free-shaft runs hit the **voltage ceiling** and look like runaway. At 96 rad/s: `E = 0.0177 × 96 = 1.70 V`, `+R·I +U₀ = 1.95 V ≈ VOLT_LIMIT`. Compute back-EMF first.
- **A frequency-domain FFT smears when the speed varies.** Fold against integrated position instead; ±10% speed variation destroyed the spectral peaks that the position fold resolved cleanly.
- **`|I|/Iq = 1.225` is a special case.** Use `|I| = 1.2247·√(Id²+Iq²)`.
- **Check a closed-form approximation against its validity range before using it.** The take-up budget stood at **2.00 mm for weeks** because it came from the *approximate* belt-length formula `L ≈ 2C + π(r₁+r₂) + (r₂−r₁)²/C`, a series expansion valid for `(r₂−r₁)/C ≲ 0.3`. **This drive runs at 0.687** — 2.3× outside it. The error was 0.97 mm: **0.4% of belt length, but 48% of the quantity being budgeted.** An approximation error is invisible precisely because the number it produces looks reasonable. It also made a bad option (a 230 mm belt) look merely risky rather than impossible. **State the validity range next to the formula, or use the exact one.**
- **A significance test is only as good as the error term, and within-boot scatter is the WRONG one.** Twelve ring captures across four sessions: a 3 → 5 → 7 bond-spot trend was reported as significant, with a physical story (creep, CA shrinkage, tooth unseating) built on top. **A repeat under identical conditions — nothing touched, 20 minutes later, new boot — moved the frequency +3.96% at "p = 0.0042".** Boot-to-boot scatter is **~8× within-boot scatter**, so every p-value in the comparison was computed against a variance ~3× too small. **The repeat was nearly free and should have been the first thing run.** *Reproduce before you explain.* The real driver turned out to be a covariate nobody tested — ring amplitude, r = −0.982, explaining 96.5% of the variance — and bond count merely correlated with it. **Before attributing a trend to the variable you changed, plot it against the variables you did not.**
- **A re-dump looks exactly like a fresh capture.** A ring session archived four dumps of which **only three were unique**: a `k` press was refused (wrong mode), no capture armed, and the next `d` re-dumped the previous buffer byte-for-byte. Nothing in the CSV said so. **Fixed in firmware rather than by habit** — `logStart()` now stamps a `cap=` serial into the dump header and `logDump()` prints `!! RE-DUMP` on any second dump of the same capture. A console-watching habit does not survive into the archived file; a header field does.
- **Two disturbance sources 7% apart in frequency are ONE source in a short capture.** Output-pulley rate (9.000 motor revs) and belt rate (9.667) cannot be separated in a 1.18-cycle window — the fit will happily assign all the energy to whichever one you ask it for, and report it with a confident σ. Resolving them needs ~14 cycles of the slower one = 12,800 samples against a 1000-sample buffer. **Compute the frequency separation and the achievable resolution BEFORE attributing a harmonic to a part.** Same family as the phase-coverage note and the 2.11-samples/rev guess above.

---

## 13. Diagnostic Ladder

1. Battery-side DC ammeter vs commanded voltage — does current scale?
2. Drag A/B: hand-spin powered vs unpowered.
3. Phase-pad → GND DC average.
4. Gate-driver VCC at the K36 common node.
5. **Read the actual IC markings.**
6. Firmware A/B against the pinned recipe, one variable at a time, `.pio` deleted.
7. Sensor buses: **raw/bit-bang transaction test before driver classes.**
8. Make faults visible: step-marker blinks, HardFault strobe.
9. **Position-resolved current map** for any "it feels rough" question. Fold against integrated `cnt`, not time.
10. **Burst capture** for anything dynamic.
11. **Check the mode and armed state of the capture itself** before interpreting it.

Meta-lessons: instrument-first beats hypothesis iteration; verify resolved versions, not requested ones; chip markings over listings; one variable per test; check `m=` first; **echo every load-bearing default at boot.**

---