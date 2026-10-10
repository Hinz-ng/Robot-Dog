# Firmware contract, session workflow, telemetry

§4 firmware contract and the known-good `platformio.ini` · §11 session workflow, commands and
telemetry · §18 sketch known gaps.

*Hub: [`README.md`](README.md).*

---

## 4. Firmware contract

**M0 symptom:** every init reports SUCCESS, angle increments, motor never moves, battery flat at
~40 mA → all six FETs off. **Cause:** SimpleFOC *latest* emits 6-PWM that violates the EG2124A
input contract. **2.3.1 works.**

### Known-good `platformio.ini`

```ini
[env:disco_b_g431b_esc1]
platform = ststm32@17.6.0          ; -> core 2.8.1. PIN THIS.
board = disco_b_g431b_esc1
framework = arduino
monitor_speed = 921600             ; 115200 freezes commutation during prints (§12)
lib_archive = false
build_flags =
    -DHAL_OPAMP_MODULE_ENABLED
    -DSIMPLEFOC_STM32_DEBUG
lib_deps =
    askuric/Simple FOC @ 2.3.1     ; EXACT. No caret.
    SPI
    Wire
```

- **Pin the platform**; verify `17.6.0 / 2.8.1` in the build log every time.
- **`Simple FOC @ 2.3.1` exact.** `^2.3.1` resolves to latest = dead motor.
- **`lib_archive = false`** or the linker drops the STM32 6-PWM implementation.
- After any lib/platform change: delete `.pio`, rebuild, read the resolved versions in the
  dependency graph.
- **Exactly one file in `src/` defines `setup()`/`loop()`.** Check the compile list.
- **`motor.foc_modulation = SpaceVectorPWM` explicitly** (2.3.1 defaults to SinePWM: ceiling V_bus/2
  instead of V_bus/√3, −13.4%).
- Wiring and sketch must describe the same configuration (an encoder on the serial pins decoded
  as `g` and self-started the motor).
- SimpleFOCDrivers is not used. If ever needed: upstream git tag `v1.0.5` (1.0.5 is gone from the
  registry), not a registry version bump.

### Console

USART2: `HardwareSerial SerialUART(PB4, PB3)` — board TXD (PB3) → ST-Link VCP RX, RXD (PB4) → VCP TX,
common ground, **921600**. OpenOCD's "target voltage may be too low" is a clone ST-Link quirk.

---

## 11. Session workflow and telemetry

Every power-up:

1. Power on. Motor boots **disabled**, mode OPENLOOP, `foc_ready = false`.
2. **Read the `CFG` banner:** `modulation / dead_zone / pwm_Hz / Vbus / v_align / Uq_max / Uq_ceil /
   Ilim / spi_nops / prefetch / flash_ws`, plus the joint row (`vbus_scale`, `i_scale`, drag,
   `breakaway_A`). Measurements are comparable only under the same values.
   - `Uq_max` = what `move()` clamps `voltage.q` to; `Uq_ceil` = what the modulator can synthesise
     (`min(driver.voltage_limit, V_bus)/√3` = 3.46 V). `Uq_max` > `Uq_ceil` is a limit that does not
     exist and an integrator that will wind up.
3. **Meter at the board's `7V-48V`/`GND` pads vs banner `Vbus`:** |Δ| > 0.03 V → power-cycle and
   re-check. Write the reading in the session header even when it passes.
4. `V` (verify stored ZEA), or `f` for a fresh alignment (twitch visible, no `Skip offset calib`).
5. `v` / `t` / `c` → `g`.
6. **Check `m=` before interpreting anything.**

**Keys:** `g` go · `x`/`s` stop · `+`/`-` target · `o` open-loop · `t` torque(V) · `c` torque(I) ·
`v` velocity · `f` align · `F` force fresh alignment · `V` verify stored ZEA · `e` encoder self-test
(20000 reads; FAILs on all-zero frames) · `E` encoder monitor, continuous · `l`/`L` burst capture
fast/slow · `k` step + capture positive · `K` negative · `j` zero-based step · `d` dump · `a` stats ·
`q` print interval · `p` VBUS/ADC register dump (read-only, motor disabled) · `?` help.

**AUTOCALIB:** `Y` menu · `1`–`6` phases · `7` report · `0` reset · `-` then `5` within 0.8 s =
phase 5 reverse-first (header prints `ORDER SWAPPED`). Any key aborts a running phase.
**Manual-assist:** `N` M2 bus-power ladder · `B`/`b` M4/B4 breakaway ramp, output free · `w` swing
ladder (B6a), 0.6/1.0/1.4/1.6 A, output locked, ~80 s, stops on tooth skip (BELT_DRIVE §22.4.4).

**Guards:** boots disabled, 20 s auto-stop, 150 rad/s overspeed, torque modes arm at 0, PID reset on
arm, debounced sense-mismatch trip.

### MIT mode — typed lines, not keys

`m` or `M` opens a line (`MIT> `). **Enter, LF or `;` runs it**, as does a whole line arriving as one
burst followed by 250 ms of silence (monitors with no line ending). ESC cancels; 15 s idle
abandons. Inside a line no character is a command. Semantics of A (live) and B (staged changes):
BELT_DRIVE §22.7.6.

| Line | Does |
|---|---|
| `m` | enter MIT mode (stopped) / print status: A, staged B, `Tf_mit`, τ_max, envelope, p, v, clamp counters |
| `m kp 41 kd 0.157` | set live fields: `pd` (rad) `vd` (rad/s) `kp` (N·m/rad) `kd` (N·m·s/rad) `ff` (N·m), plus `tf <ms>`, `tmax <N·m>`. Output side. Whole line validated first (kp 0–200, kd 0–2, \|pd\| ≤ 6.28, \|vd\| ≤ 5, \|ff\| ≤ 1, tf 0.05–50 ms, tmax ≤ 1.0). Plain decimals only |
| `m b vd 2` / `m b` | stage changes for the next `m go` (B = A + named fields, resolved at `m go`). Each `m b` replaces the previous; `m b` alone clears |
| `m go [rev_ms] [decim]` | resolve and re-validate B, start a capture, switch after 50 pre-trigger samples; optional auto-revert to zero torque `rev_ms` after the switch |
| `m zero` | live → zero torque now (pd = current p) |
| `m test` | clamp chain and validator against a fixed table, motor disarmed |

Arming (`g`) in MIT mode re-zeroes p, clears A's pd/vd/ff, keeps kp/kd and staged changes, resets
τ_max to 0.39, and prints A and what `m go` will apply. `!!` lines flag a target multiplied by a zero
gain. The inner clamp (1.6 A_rep) is compile-time. `+`/`-`, `k`, `K`, `j` are refused in MIT mode.

### Telemetry line

```
m=<mode> run=<0/1> tgt= cnt= vel= Iq= Id= |I|= Uq= Ud= Vb= Vb_src= lps= pr_us=
```

- **`Vb` is the boot seed, not live** (`Vb_src=seed`).
- **`Uq` is the saturation check.** At 3S with `VOLT_LIMIT = 2.0`, free spin above ~90 rad/s is
  saturated and its `Iq` is meaningless.
- **`ratio=` is a valid gate only at locked rotor** (1.224); spinning it reads high (1.255 at
  97 rad/s) because |I| is always positive.
- `motor.shaft_velocity` is written only inside `motor.move()` and freezes while stopped.
- MIT mode appends `p= pd= v= tau= cl=o/i/r svc_us=`: output frame; `tau` is measured
  (`irepToTorqueOut(Iq)`); `cl` counts loops each clamp bound since arm (outer/inner/reject); `!!`
  after `cl` = the inner clamp bound at least once; `svc_us` is a max (≈ 2× the mean); the law uses
  `v=`, not the 20 ms-filtered `vel=`.
- `Vdma` (telemetry and the `M2,` row) is the DMA buffer's bus reading, ~60 counts (0.51 V) below
  `Vb`, diagnostic only. It exists for the live-Vbus acceptance test (§8.3).

### Burst logger

RAM ring buffer, 1000 samples × 18 B = 18.0 kB (55% of RAM).
- `l` = decim 1 (~65 ms) — current-loop steps. `L` = decim 8 (~520 ms) — judder, resonance, folds.
- `k` = pre-load to base, settle 300 ms, step while capturing (TORQUE(V) and TORQUE(I)).
- `a` = mean of the last capture, echoing `m=`, `run=`, `dz=`.
- **Per-sample `dt_us` is stored**; fit against the `t_us` column. Periodic printing stops during a
  capture. Each dump carries `cap=N`; `!! RE-DUMP` flags a repeated dump.
- **MIT captures** use columns `i,t_us,p,p_des,v,tau_law,tau_cmd,tau,sp,Iq,Id,Uq,rawI,cnt,clamp`
  (p, p_des, tau_cmd, tau reconstructed at dump time). For τ-vs-v checks, plot measured `tau`
  against v differentiated offline from `cnt` (`tau_law` vs `v` is true by construction).
- **Step between two nonzero currents** (stepping from 0 puts the dead zone inside the measurement).
- **`vel` is filtered at 20 ms — never fit inertia from it. Use `cnt`** (14-bit raw angle, wraps at
  16383; unwrap before differentiating).

---

## 18. Sketch — known gaps

- `logStats()` skips the first 25% of the buffer — right for steady sweeps, misleading after a `k`.
- `VBUS_LIVE = false` (§8.3).
- Bus voltage is not in the burst log. If added (`uint16_t vbus_x100`), drop `LOG_N` 1000 → 900 and
  read the free-RAM figure first (a static buffer colliding with the stack HardFaults). Only worth
  it once sampling is live.
- The single disable path is shareable (`stopMotor()` in `safety.h`) but not single: bare
  `motor.disable()` sites remain in `autocalib.h`. Closing it is a behaviour change needing design
  review.
