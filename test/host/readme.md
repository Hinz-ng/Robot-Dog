# Host tests

The real `src/` headers compiled for the PC against mocks. No board, no current.

```
python test/host/run.py            # all
python test/host/run.py joint_cal  # name filter
python test/host/run.py leg        # leg policy, parser, fake bus, Tier-0 cutoffs
python test/host/test_leg_kinematics_tool.py  # offline calculator integration
python test/host/test_web_controls.py        # compile both pages, JS syntax + button routing
```

Compiler: `$CXX`, else `g++` / `clang++`, else `python -m ziglang c++` (`pip install ziglang`).

| Test | Covers |
|---|---|
| `test_web_controls.py`, `test_web_controls.js` | Both embedded page profiles: paired commands, disabled individual actions in LEG, normal node selection, log command scope and JS syntax |
| `test_leg_kinematics.cpp` | Independent geometry fixture, FK/IK round trips/wraps, asymmetric links, analytic/numerical Jacobian, branch/singularity/input guards and missing-reference/stale/faulted read-only snapshots |
| `test_leg_kinematics_tool.py` | Same-header PC calculator: degree/radian/mm conversions, FK/IK/sample, required branch and rejection exits; no device access |
| `test_log_control.cpp` | Append after pause, preserve full buffer, refuse clear while recording/armed, reject unknown commands without erasing |
| `test_mt6835.cpp` | `mt6835.h` against a bit-level MT6835 model: command words, 48-clock frame, ENC_BITS truncation, CRC (reference bitwise form), every single-bit error, random-corruption escape rate (≈ 0.39%), all-zero and stuck-high frames, power-up, link health (`linkTripped`, `linkArmable`) at 99.65 / 50 / 20 / 5 % error rates |
| `test_mit_law.cpp` | the MIT estimator across the raw-count and 2³² accumulator wraps, both directions and both `dir` signs: p continuous, v unaffected |
| `test_joint_cal.cpp` | `calZeaUsable()` and the banner, per JOINT_ID: J01 / J03 usable, J04 uncalibrated, row 14 (zea through the MT6816) ignored |
| `test_leg_bench.cpp` | Pure paired session, reference, signs, quantized bounds, slow moves, invalid/stale state and coupled faults |
| `test_leg_console.cpp` | Strict leg command grammar, finite numbers and raw-command policy |
| `test_leg_runtime.cpp` | Real poll adapter with fake node/bus replies: partial ARM, partner shutdown, control-gap stop and phone deadline despite stalled HTTP |
| `test_leg_t0_policy.cpp` | J01/J03 local reference/bounds/gains; no DAMP refresh; master/Tier-0 config consistency when enabled |

Run after any edit to `mt6835.h`, `mit_law.h`, `joint_cal.h` or `fleet_config.h`. A case whose
expectation changes with the code (a row converted, a threshold moved) is edited
in `run.py` in the same commit.

Offline measured-reference converter: `python test/host/test_leg_reference.py` checks reference offsets, units/configuration signs, rotated/translated coordinates and invalid measurements.

## G473 port checks

`python test/host/run.py "G473 commissioning"` checks J04's reviewed MT6835 row,
torque conversion/clamp, independent unbuilt J05-J12 defaults and unrecorded UIDs.
After building both G473 targets, `python tools/check_g473_build.py` checks their
ELF/backend/memory metadata and pre-port G431 harness binary equivalence.
The [public technical reference](../../docs/technical.md) describes the configuration
boundaries. Detailed commissioning procedures and original bench records are maintained
separately from the public repository.
