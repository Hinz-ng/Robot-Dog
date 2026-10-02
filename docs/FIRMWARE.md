# Firmware contract, session workflow, telemetry

§4 firmware contract and the known-good `platformio.ini` · §11 session workflow
and telemetry · §18 sketch known gaps.

*Part of the M0/M1 actuator doc set. Hub and section routing table: [`README.md`](../README.md). §8 in [`CONSTANTS.md`](CONSTANTS.md) is the master table — every number elsewhere defers to it.*

---

## 4. Firmware Contract

**M0 symptom:** every init reports SUCCESS, angle increments, motor never moves, battery flat at ~40 mA regardless of commanded voltage → all six FETs off.
**Root cause:** SimpleFOC *latest* emits 6-PWM that violates the EG2124A input contract. **2.3.1 works.** The genuine board's L6387 tolerates both, which is why no upstream bug exists.

### Known-good `platformio.ini`

```ini
[env:disco_b_g431b_esc1]
platform = ststm32@17.6.0          ; -> core 2.8.1. PIN THIS.
board = disco_b_g431b_esc1
framework = arduino
monitor_speed = 921600             ; raised from 115200 — see §12 print-block freeze
lib_archive = false
build_flags =
    -DHAL_OPAMP_MODULE_ENABLED
    -DSIMPLEFOC_STM32_DEBUG
lib_deps =
    askuric/Simple FOC @ 2.3.1     ; EXACT. No caret.
    SPI
    Wire
```

Load-bearing rules:

- **Pin the platform.** Unpinned resolves to the latest core (19.6.0 / 2.12.0) and has silently broken working sketches. Verify `17.6.0 / 2.8.1` in the build log every time.
- **`Simple FOC @ 2.3.1` exact.** `^2.3.1` resolves to latest = dead motor.
- **`lib_archive = false`** or the linker drops the STM32 6-PWM implementation.
- After any lib/platform change: **delete `.pio`**, rebuild, and **read the resolved versions** in the Dependency Graph. Verify what resolved, not what you requested.
- **Exactly one file in `src/` may define `setup()`/`loop()`.** A stray `main.cpp` produces a clean build, a verified flash, and the wrong firmware running. Check the compile list.
- **Wiring and sketch must describe the same configuration.** Encoder on the serial pins + a console sketch produced phantom serial bytes decoded as `g` → motor self-starting.
- **`motor.foc_modulation = SpaceVectorPWM` must be set explicitly.** 2.3.1 defaults to `SinePWM`, which caps the linear range at `V_bus/2` instead of `V_bus/√3` — **13.4% of your voltage, silently.** Verified on the bench: at bench modulation depth the phase currents are identical (SVPWM changes the ceiling, not the gain), and `|I|/Iq = 1.225` survives because the zero-sequence component SVPWM adds is common-mode.
- ~~SimpleFOCDrivers, if used, must be 2.3.1-era (1.0.5 compiles). Nothing currently needs it.~~ **REMOVED from `platformio.ini` 2026-09-10.** Upstream **withdrew 1.0.5 from the PlatformIO registry** (only 1.0.7/1.0.8/1.0.9 remain), so the pin was unsatisfiable on any *clean* build, on any machine, for all twelve joints — it only kept working because a cached copy sat in `.pio`. **This rule's own "delete `.pio`" step is what surfaced it**, which is the rule working as intended. Dropped rather than re-pinned because nothing in `src/` includes it. ⚠ **If it is ever needed back, take the upstream git tag `v1.0.5` (which still exists) — not a registry version bump**, or you silently change library version on a frozen toolchain.

### Console
USART2: `HardwareSerial SerialUART(PB4, PB3)` — board TXD(PB3) → ST-Link VCP RX, RXD(PB4) → VCP TX, common ground, **921600**.
OpenOCD's "target voltage may be too low" is a clone ST-Link VREF quirk. Harmless.

---

## 11. Session Workflow & Telemetry

Every power-up:

1. Power on. Motor boots **DISABLED**, mode = OPENLOOP, `foc_ready = false`.
2. **Read the `CFG` banner** — `modulation / dead_zone / pwm_Hz / Vbus / v_align / Uq_max / Uq_ceil / Ilim / spi_nops`. A measurement is only comparable to others taken under the same values.
   - `pwm_Hz` now prints **25000** rather than `-12345`: the frequency is assigned explicitly instead of being left at `NOT_SET`. Same hardware behaviour, honest banner.
   - **`Uq_max` is what `move()` clamps `voltage.q` to** (`motor.voltage_limit`). **`Uq_ceil` is what the modulator can physically synthesise**, `min(driver.voltage_limit, V_bus)/√3` = 3.46 V here. If `Uq_max` ever exceeds `Uq_ceil` you have a limit that does not exist and an integrator that will wind up against it.
   - **The boot line above it also prints `vbus_scale`, `i_scale`, both drag pairs and `breakaway_A`**, and warns if a *built* joint still carries `i_scale = 1.0` or `breakaway_A = 0`.
2a. **Put the multimeter on the pack terminals and write the reading in the session header, next to the banner `Vbus`.** The seed is a per-boot scale factor on every voltage the firmware reports, and it drifts 1.3% boot to boot (11.28–11.43 V observed vs 11.26–11.30 V true). Recording it costs 30 s and lets any session be rescaled in post-processing. Observed worst case 0.53%, which propagates to 0.5% on `R_eff` — inside the existing scatter, so this is bookkeeping, not a blocker.
3. Press **`f`** → confirm a *real* alignment (twitch visible, no `Skip offset calib`).
4. `v` / `t` / `c` → `g`.
5. **Check `m=` before interpreting anything.**

Commands: `g` go · `x`/`s` stop · `+`/`-` target · `o` open-loop · `t` torque(V) · `c` torque(I) · `v` velocity · `f` align · `F` force align · `e` encoder self-test (2000 reads) · `E` encoder monitor, continuous (harness wiggle test) · `l`/`L` burst capture fast/slow · `k` step+capture, **positive** · **`K` step+capture, negative** · **`j` zero-based step** · `d` dump · **`a` stats** · `q` toggle print interval · `?` help.

*Corrected 2026-09-28:* the list above lacked `K`, `e` and `E`, all present in `handleSerial()`. The lower-case `v` is velocity mode; upper-case `V` is the ZEA check (below).

Reference: **`p`** — VBUS/ADC register dump, read-only, motor disabled. Prints the ADC kernel clock, both instances' sequence and offset registers, `CALFACT`, the GPIO analog state of PA0/PB12/PB14, and the live DMA buffer. **This is the instrument that produced §3's sequence map**; it is not a test and it writes nothing. (`P`, the gate-drive load test, was deleted with row V1 on 2026-08-30.)

> ### ~~V1 bench procedure — the ADC pre-calibration boot~~ — RETIRED 2026-08-30
>
> **Row V1 is closed and the code this drove is deleted** (§0). Kept as a one-line
> pointer because the trap is worth not re-walking: at the only point in boot where the
> ADC is idle there is **no ADC kernel clock** (`RCC->CCIPR.ADC12SEL = 0`), so `ADCAL`
> cannot run — and arming it there does not fail, it sits **pending** and fires later
> inside `currentSense.init()`, writing a plausible-looking wrong factor. That is what
> made a no-op look like a confirmed fix for a session. **`CALFACT = 0` on both
> instances, every boot, is the normal state of this board** (§3).

AUTOCALIB: `Y` menu · `1`–`6` phases · `7` report · `0` reset · **`V` verify stored ZEA** · **`-` then `5` within 0.8 s** = phase 5 reverse-first (the swapped-order B3 repeat; the run header prints `ORDER SWAPPED`). Any key aborts a running phase.
Manual-assist (not part of the 1–7 chain): **`N`** M2 bus-power ladder · **`B`**/**`b`** M4/B4 breakaway ramp, forward/reverse, output free · **`w`** swing ladder (B6a), 0.6/1.0/1.4/1.6 A, **output locked**, ~80 s, stops on tooth skip (BELT_DRIVE.md §22.4.4).

> `N`, `B` and `b` were chosen because **`F` is force-align and `G` is go** — binding either would have shadowed an existing command silently.

**MIT mode (B12a, added 2026-10-01) — typed LINES, not keys.** `m` or `M` opens a line (`MIT> `), **Enter, LF or `;` runs it** — and so does a whole line that arrives as one burst followed by 250 ms of silence, which is what monitors with no line ending send (fixed 2026-10-01: `m test` from such a monitor used to sit unexecuted). A line typed key by key still needs Enter or `;`, and one idle 2 s prints a hint. ESC cancels, 15 s idle abandons it. The status line is held while a line is open. Inside a line no character is a command — typed as single keys, `m kp 41` would run AUTOCALIB phase 4 and `pd -0.005` would arm the `-`/`5` phase-5 swap chord, so the line handler runs before the chord tracker.

| Line | Does |
|---|---|
| `m` | Enter MIT mode (stopped) / print MIT status: live A, staged B, `Tf_mit`, τ_max, envelope, p, v, clamp counters |
| `m kp 41 kd 0.157` | Set LIVE fields, any number of pairs: `pd` (rad) `vd` (rad/s) `kp` (N·m/rad) `kd` (N·m·s/rad) `ff` (N·m), plus `tf <ms>` and `tmax <N·m>`. **Output side.** The whole line is validated before anything is applied (kp 0–200, kd 0–2, \|pd\| ≤ 6.28, \|vd\| ≤ 5, \|ff\| ≤ 1, tf 0.05–50 ms, tmax ≤ 1.0). Numbers are plain decimal (`-0.005`, `1e-3`); no `nan`/`inf` |
| `m b vd 2` / `m b` | **Stage changes** for the next `m go`: B = A + the named fields, resolved when `m go` fires (unnamed fields come from A then). Each `m b` line **replaces** the previous changes; `m b` alone clears them (B = A, `m go` only records). Redesigned 2026-10-02 — B used to be a full second command, and `m b vd 2` on a fresh boot staged kd 0 = zero torque (BELT_DRIVE §22.7.10) |
| `m go [rev_ms] [decim]` | Resolve and re-validate B, then start a capture; B takes effect after **50 pre-trigger samples**; optional auto-revert to zero torque (kp, kd, vd, ff = 0) `rev_ms` after the switch. Running and MIT mode only |
| `m zero` | LIVE → zero torque now (pd = current p) |
| `m test` | **B12a a1:** the clamp chain and the validator against a fixed table, **motor disarmed**. Auto-checks clamp code, sign, bounds and round trip; on an uncalibrated row every torque case must REJECT (not bench-run on J01: the env guard refuses another joint's build, by design) |

Arming (`g`) in MIT mode re-zeroes p (session-relative), clears A's pd/vd/ff, **keeps kp/kd and the staged changes** (zero error × any kp = zero torque; B only ever applies on an explicit `m go`, so it can be staged before `g` instead of inside the 20 s auto-stop window) and resets τ_max to 0.39. It prints A and what `m go` will apply. A pd/vd/ff typed while stopped are therefore never run, and the reply says so. **`!!` lines** flag a target multiplied by a zero gain (vd with kd = 0, pd with kp = 0) on every A edit, `m b` and `m go`. The inner clamp (1.6 A_rep, D3) is compile-time and not settable. `+`/`-`, `k`, `K` and `j` are refused in MIT mode — the law owns `target`.

Guards: boots disabled, 20 s auto-stop, 150 rad/s overspeed, torque modes arm at 0, PID reset on arm, debounced sense-mismatch trip.

### Telemetry line
```
m=<mode> run=<0/1> tgt= cnt= vel= Iq= Id= |I|= Uq= Ud= Vb= Vb_src= lps= pr_us=
```
- **`Ud` IS printed** — in the telemetry line, in `logStats()`, and in the burst CSV. (The old note claiming otherwise was stale and has been deleted.) It is the angle-lag channel — §8.3.
- **`Vb` is the boot-time seed, NOT live.** `Vb_src=seed` says so on every line. It does not track sag. See §12.
- **`Uq` is the saturation check.** Tuning while `Uq` is pinned is tuning a clamp. At 3S, `VOLT_LIMIT = 2.0` saturates above ~90 rad/s: `Uq = R·Iq + U₀ + Ke·ω` = 2.08 V at 1.5 A and 98 rad/s. **Any free-spin run above ~90 rad/s at 2.0 V is saturated and its `Iq` is meaningless.**
- **`ratio=` is only a valid calibration gate when `Uq` is unsaturated AND speed is low.** `|I|` is a square root of squares, so averaging an always-positive rippling quantity biases the mean *upward*. Measured: 1.255 at 97 rad/s (`Iq_pp` = 0.533 A) vs **1.224 at locked rotor** (`Iq_pp` = 0.062 A), same calibration. Gate it at locked rotor.
- **`motor.shaft_velocity` is written only inside `motor.move()`**, skipped while stopped → freezes. Compute fresh when stopped.
- **CFG banner (2026-10-01) ends with `prefetch= flash_ws=`**, read back from `FLASH->ACR`. The fetch path can change the loop rate in every mode (`fleet_config.h`, `FLASH_PREFETCH`); pre-2026-10-01 builds ran `prefetch=0 flash_ws=8`. Prefetch itself measured **no** loop-rate effect (2026-10-02, BELT_DRIVE §22.7.7).
- **MIT mode appends** `p= pd= v= tau= cl=o/i/r svc_us=` (`svc_us` = longest `mitService()` in the status window — a **max**, inflated ~2× by interrupt preemption; the mean cost is the lps difference vs TORQUE(I)) — output frame; `tau` is **measured** (`irepToTorqueOut(Iq)`); `cl` counts loops each clamp bound since arm (outer / inner / reject). **`!!` after `cl` = the inner clamp bound at least once** — at the default τ_max that is a conversion-vs-envelope finding, not a saturation. `vel=` stays SimpleFOC's 20 ms-filtered motor speed; the law uses `v=`.

### Burst logger
RAM ring buffer, 1000 samples × **18 B = 18.0 kB**. *(Corrected 2026-09-10 — this said 16 B. `LogSample` carries **nine** `int16`/`uint16` fields, not eight: `dt_us` was added later, when `logDump()` stopped reconstructing `t = k · dt_mean` from an assumed-uniform clock. **§18's arithmetic is unaffected** — adding a tenth field (`vbus_x100`, 20 B) at `LOG_N` 900 still lands on 18.0 kB; only its stated 16 B baseline was wrong. This buffer is 55% of the G431's 32 kB and is why RAM sits at 73.9%.)*
- `l` = decim 1 (~65 ms) — current-loop steps.
- `L` = decim 8 (~520 ms) — judder, resonance, position folds.
- `k` = pre-load to base, settle 300 ms, step while capturing. Works in **TORQUE(V) and TORQUE(I)**.
- `a` = mean of the last capture, one line per sweep point. Echoes `m=`, `run=`, `dz=` so a measurement can never be separated from its conditions.
- **Per-sample `dt_us` is stored**, not assumed. `logDump` reports min/max jitter; use the `t_us` column for any fit.
- Periodic printing is suppressed during capture.
- **MIT captures (B12a) use a different column set**, same 18 B per sample: `i,t_us,p,p_des,v,tau_law,tau_cmd,tau,sp,Iq,Id,Uq,rawI,cnt,clamp`. Three slots are reused (`vel` → v_mit ×1000, `Ud` → τ_law ×1e4, `cnt` bits 14–15 → clamp code); **p, p_des, tau_cmd and tau are reconstructed at dump time** through the boundary pair, so they cost no RAM. The header records A, B, the switch and revert sample indices, `Tf_mit`, τ_max and `dir`. `a` rescales the reused slots. **For a3's τ-vs-v check, plot measured `tau` against v differentiated offline from `cnt`** — `tau_law` vs `v` is true by construction.
- **Step between two nonzero currents.** Stepping from 0 puts the dead-zone traverse inside the measurement.
- **`vel` is filtered at 20 ms — never fit inertia from it. Use `cnt`** (uint16 holding the 14-bit MT6816 raw angle, **wraps at 16383**; unwrap before differentiating). *Corrected 2026-09-28 — "wraps at 4095" was the retired ABZ/4096-CPR figure.*

---

## 18. Sketch — Known Gaps

Reviewed 2026-07-29. No functional bugs found. Outstanding items:

- `logStats()` skips the first 25% of the buffer — correct for steady-state sweeps, misleading after a `k` step.
- `VBUS_LIVE = false`. `Vb` is a boot seed and does not track. §12 has the reason; §8.3 has the promotion condition. ⚠ **The reason changed on 2026-08-21** — it is no longer "reading PA0 needs register-level work" (it does not; PA0 is rank 5 of the DMA'd sequence, §3). It is that the DMA path's converter is uncalibrated and reads ~60 counts low. §0.
- ⚠ **`Vdma` in the telemetry line and the two trailing `Vdma` columns on the `M2,` row are DIAGNOSTIC.** `Vdma` is the DMA buffer's own bus reading — it sits **~60 counts (0.51 V) below `Vb`** because SimpleFOC never calibrates the converter (§3), it has no filter and no plausibility window, and it is **not** `driver.voltage_power_supply`. **`Vb` is the number the firmware actually uses.** `Vdma` is retained only because §8.3's promotion condition for live Vbus is `Vdma` against a terminal meter, which this makes free at the next M2 for record.
- Bus voltage is not in the burst log. Add `uint16_t vbus_x100` (drop `LOG_N` 1000 → 900 to hold 18.0 kB, and **read the free-RAM figure in the build output first** — a static buffer that collides with the stack gives a HardFault, not a compile error). Only worth doing once sampling is live.

---