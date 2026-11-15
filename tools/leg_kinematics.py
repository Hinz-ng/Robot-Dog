"""Read-only geometry calculator. Compiles the firmware's pure C++ math for the PC.

No device, serial port, network, firmware configuration or source file is modified.
Build artifacts go under .pio/kinematics-tools. See docs/KINEMATICS.md for units.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    fk = commands.add_parser("fk", help="absolute upper-link angles -> foot pin")
    fk.add_argument("--theta-deg", nargs=2, type=float, required=True, metavar=("J01", "J03"))
    ik = commands.add_parser("ik", help="foot pin -> absolute angles near a seed")
    ik.add_argument("--foot-mm", nargs=2, type=float, required=True, metavar=("X", "Z"))
    ik.add_argument("--seed-deg", nargs=2, type=float, required=True, metavar=("J01", "J03"))
    sample = commands.add_parser("sample", help="measured reference angles + session displacements -> foot pin")
    sample.add_argument("--ref-deg", nargs=2, type=float, required=True, metavar=("J01", "J03"))
    sample.add_argument("--q-rad", nargs=2, type=float, required=True, metavar=("J01", "J03"))
    for p in (fk, ik, sample):
        p.add_argument("--branch", nargs=3, type=int, choices=(-1, 1), required=True,
                       metavar=("FOOT", "ELBOW1", "ELBOW3"), help="fixed geometric cross-product signs; no guessed default")
    args = parser.parse_args()
    runner = ROOT / "test/host/run.py"
    spec = importlib.util.spec_from_file_location("host_compiler", runner)
    module = importlib.util.module_from_spec(spec)
    # Avoid a __pycache__ write beside the test source merely to discover a compiler.
    exec(compile(runner.read_text(encoding="utf-8"), str(runner), "exec"), module.__dict__)
    cxx = module.compiler()
    cache = ROOT / ".pio/kinematics-tools"
    cache.mkdir(parents=True, exist_ok=True)
    exe = cache / ("leg_kinematics.exe" if os.name == "nt" else "leg_kinematics")
    source = ROOT / "tools/leg_kinematics_cli.cpp"
    # Compile each invocation: compiler/config changes cannot leave a stale calculator.
    env = os.environ.copy()
    tmp = cache / "tmp"
    tmp.mkdir(exist_ok=True)
    env["TEMP"] = env["TMP"] = str(tmp)
    env.setdefault("ZIG_GLOBAL_CACHE_DIR", str(cache / "zig-cache"))
    command = cxx + ["-std=c++11", "-O2", str(source), "-o", str(exe)]
    built = subprocess.run(command, env=env, capture_output=True, text=True)
    if built.returncode:
        print(built.stderr, file=sys.stderr)
        return built.returncode
    if args.mode == "fk":
        values = args.theta_deg
    elif args.mode == "ik":
        values = args.foot_mm + args.seed_deg
    else:
        values = args.ref_deg + args.q_rad
    return subprocess.run([str(exe), args.mode, *map(str, values + args.branch)], env=env).returncode


if __name__ == "__main__":
    sys.exit(main())
