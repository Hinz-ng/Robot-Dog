# Host tests

The real `src/` headers compiled for the PC against mocks. No board, no current.

```
python test/host/run.py            # all
python test/host/run.py joint_cal  # name filter
```

Compiler: `$CXX`, else `g++` / `clang++`, else `python -m ziglang c++` (`pip install ziglang`).

| Test | Covers |
|---|---|
| `test_mt6835.cpp` | `mt6835.h` against a bit-level MT6835 model: command words, 48-clock frame, ENC_BITS truncation, CRC (reference bitwise form), every single-bit error, random-corruption escape rate (≈ 0.39%), all-zero and stuck-high frames, power-up, link health (`linkTripped`, `linkArmable`) at 99.65 / 50 / 20 / 5 % error rates |
| `test_joint_cal.cpp` | `calZeaUsable()` and the banner, per JOINT_ID: J01 usable, J03 / J04 uncalibrated, row 14 (zea through the MT6816) ignored |

Run after any edit to `mt6835.h`, `joint_cal.h` or `fleet_config.h`. A case whose
expectation changes with the code (a row converted, a threshold moved) is edited
in `run.py` in the same commit.
