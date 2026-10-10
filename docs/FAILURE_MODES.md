# Failure-mode catalogue and diagnostic ladder

§12 failure modes that each cost at least one session · §13 diagnostic ladder.
**Read this before believing any number.**

*Hub: [`README.md`](../README.md).*

---

## 12. Failure-mode catalogue

**Toolchain / build**
- Unpinned platform silently upgrades the core. Pin `ststm32@17.6.0`.
- SimpleFOC `^2.3.1` → dead motor (EG2124A contract). Pin exactly 2.3.1.
- Two files with `setup()`/`loop()` → clean build, wrong firmware.
- Missing `lib_archive = false` → linker drops 6-PWM.
- Library defaults are silent decisions (`foc_modulation` defaulted to SinePWM, −13.4% voltage
  ceiling). Echo every load-bearing default in the boot banner.
- Python raw strings used as replacement text containing C escapes produced `\\n` and `\\"`
  regressions twice. After any edit touching a string literal: `grep -rn '\\\\' src/`.

**Board / pin map**
- Vendor-pruned pin map → `Error_Handler()` infinite loop, not an error return. Any peripheral on a
  non-default pin needs register-level setup or bit-banging (hit by HW SPI, `STM32HWEncoder`, UART
  on PB6/PB7).
- PB8 = BOOT0 (§3).
- `NOT_SET` reads as −12345, not an error (`pwm_frequency`).

**Sensor / alignment**
- ZEA hardcoded on an incremental encoder → rotor locks at full current.
- `Skip … calib` = stored value reported back, not a measurement.
- `initFOC()` with the driver disabled → "Failed to notice movement".
- Software quadrature loses counts above ~100 k edges/s → silent ZEA corruption.
- An incremental counter cannot detect its own error (TIM4 faithfully counted noise). Absolute +
  parity changes the failure class: a bad frame is rejected and recovers in one cycle.
- **All-zero frames pass parity and `No_Mag`** (dead MISO). Check that `raw` moves when turned.
- Magnet slipping in its mount: escalating current at constant speed, then a stall that realignment
  "fixes". Keep the pen mark. A steel screw through a diametric magnet corrupts the field.
- Magnet touching the package (zero gap): count loss, a "bearing" whine, a 1/rev disturbance.

**Control / measurement**
- **`motor.current` is not updated in voltage mode** (2.3.1 returns early). Iq/Id in TORQUE(V) and
  OPENLOOP are stale unless refreshed explicitly.
- **A blocking telemetry print freezes commutation:** the voltage vector stays put while the rotor
  turns (observed |I| 15.5 A with Iq reading 1.0 A). Mitigated by 921600 baud; proper fix is a
  chunked non-blocking emit.
- A capture taken disarmed looks healthy. `m=` and `run=` are recorded at `logStart`; check them first.
- A measurement filter in a loop is a damping lever, not a bandwidth lever.
- Gain ratio beats filter tuning: measure L, compute P/I, then tune the filter.
- `Id` cannot detect commutation drift in current mode (the d-loop zeroes the apparent Id); the
  cos(δ) torque loss remains. Use voltage mode to diagnose alignment.
- `current_limit` does not bind in voltage torque mode. Torque mode has no speed limit.
- Stale state latches: ask who writes a value and when. A frozen value looks like a perfectly
  stable measurement ("zero drift over 20 s" was a dead sampler).
- Raw `|I|` is meaningless when the drive saturates.
- A ramp hides marginal stability; a step exposes it.
- A guard on a noisy signal needs debouncing.
- Free-shaft runs hit the voltage ceiling and look like runaway. Compute back-EMF first.

**ADC (§3)**
- **After `currentSense.init()`, `analogRead()` is forbidden.** It returns 0 today only because the
  regular group is already started; a HAL that stopped the ADC first would rewrite `SQR1`/`SMPR`
  and silently corrupt the phase currents. Any ADC access after init must be a pure read of the
  DMA buffer (PA0 = rank 5, PB14 = rank 4).
- The DMA buffer reads ~60 counts low on absolute channels (`CALFACT = 0`, never calibrated).
- Build guards from a flag only one branch writes (e.g. `vbus_valid`, cleared only by the sampler).
- A diagnostic that mutates the state it reports makes its own "before" reading unreliable. Prefer
  passive telemetry fields.

**Measurement design**
- **A gate is valid only in the regime it was characterised in.** `|I|/Iq` free-spinning at low
  current reads 1.27–1.36 (mean of |I| is biased under ripple). Accumulate in the squared domain;
  the tight 1.22–1.23 check is locked-rotor only.
- Bin repeated step data by time, not sample index (phase jitter gives free equivalent-time samples).
- **Two-point fits are exactly determined** — no residual, no way to see a bad point (R_eff 0.1977
  from 2 points vs 0.22184 from 9). Use ≥ 5 points.
- A constant that scales everything and cannot be self-checked must be measured against an
  external reference (`vbus_scale` → M1).
- A timing comment written from a model is a prediction. Build the measurement into the feature
  (`us_per_read`).
- DMA or freed CPU buys nothing on a single blocking loop; SCK frequency does.

**Interpretation**
- 300 ms telemetry aliases everything above ~1.7 Hz.
- Forward-only data cannot separate even terms (ZEA, INL) from odd ones (delay). Run both
  directions and split by parity.
- Measure a test's noise floor before interpreting it (six null repeats, two minutes).
- **A fit can pin one parameter and not another** (R_eff ±1.55% while U0 had 21.8% SE). Report the
  well-conditioned one; say the other is unresolved; do not overwrite a dedicated measurement with
  a by-product.
- Check revolutions per capture before specifying repeats (a 109 rad/s capture spans 9.5 revs).
- Fold against integrated position, not time, when speed varies.
- `|I|/Iq = 1.225` is a special case; use `|I| = 1.2247·√(Id² + Iq²)`.
- Check a closed-form approximation against its validity range (the approximate belt-length
  formula at (r₂−r₁)/C = 0.687 gave a take-up budget 2× too large).
- **Within-boot scatter is the wrong error term.** Boot-to-boot scatter was ~8× larger; a "p =
  0.004" trend vanished on a free repeat. Reproduce before explaining, and plot against the
  variables you did not change (ring amplitude explained 96.5% of the variance).
- A re-dump looks like a fresh capture. Dumps carry `cap=N`; `!! RE-DUMP` flags a repeat.
- Two disturbance sources 7% apart in frequency are one source in a short capture. Compute the
  required resolution before attributing a harmonic to a part.
- Search the archive before planning a measurement, and check what an archived file actually
  contains before planning around it.

---

## 13. Diagnostic ladder

1. Battery-side DC ammeter vs commanded voltage — does current scale?
2. Drag A/B: hand-spin powered vs unpowered.
3. Phase pad → GND DC average.
4. Gate-driver VCC at the K36 common node.
5. **Read the actual IC markings.**
6. Firmware A/B against the pinned recipe, one variable at a time, `.pio` deleted.
7. Sensor buses: raw/bit-bang transaction test before driver classes.
8. Make faults visible: step-marker blinks, HardFault strobe.
9. Position-resolved current map for any "it feels rough" question, folded against integrated `cnt`.
10. Burst capture for anything dynamic.
11. Check the mode and armed state of the capture before interpreting it.
