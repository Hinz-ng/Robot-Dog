"""Host tests: compile src/ headers for the PC against mocks and run them.

    python test/host/run.py [name-filter]

Compiler: $CXX if set, else g++ / clang++ on PATH, else `python -m ziglang c++`
(pip install ziglang -- no system compiler needed). Exit code 0 = all pass.
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.normpath(os.path.join(HERE, "..", "..", "src"))
PROTO = os.path.normpath(os.path.join(HERE, "..", "..", "can_protocol"))
MOCK = os.path.join(HERE, "mock")

# (name, source, extra -D flags)
CASES = [
    ("G473 commissioning J04", "test_g473_commissioning.cpp", ["-DJOINT_ID=4", "-DQUAD_G473"]),
    ("G473 commissioning J05", "test_g473_commissioning.cpp", ["-DJOINT_ID=5", "-DQUAD_G473"]),
    ("G473 commissioning J06", "test_g473_commissioning.cpp", ["-DJOINT_ID=6", "-DQUAD_G473"]),
    ("G473 commissioning J07", "test_g473_commissioning.cpp", ["-DJOINT_ID=7", "-DQUAD_G473"]),
    ("G473 commissioning J08", "test_g473_commissioning.cpp", ["-DJOINT_ID=8", "-DQUAD_G473"]),
    ("G473 commissioning J09", "test_g473_commissioning.cpp", ["-DJOINT_ID=9", "-DQUAD_G473"]),
    ("G473 commissioning J10", "test_g473_commissioning.cpp", ["-DJOINT_ID=10", "-DQUAD_G473"]),
    ("G473 commissioning J11", "test_g473_commissioning.cpp", ["-DJOINT_ID=11", "-DQUAD_G473"]),
    ("G473 commissioning J12", "test_g473_commissioning.cpp", ["-DJOINT_ID=12", "-DQUAD_G473"]),
    ("leg kinematics + read-only validity", "test_leg_kinematics.cpp", []),
    ("log append/pause/clear", "test_log_control.cpp", []),
    ("leg_bench policy + session", "test_leg_bench.cpp", []),
    ("leg_bench console grammar", "test_leg_console.cpp", []),
    ("leg_bench runtime fake bus", "test_leg_runtime.cpp", []),
    ("leg_t0 policy J01", "test_leg_t0_policy.cpp", ["-DJOINT_ID=1"]),
    ("leg_t0 policy J03", "test_leg_t0_policy.cpp", ["-DJOINT_ID=3"]),
    ("mt6835 driver + link health", "test_mt6835.cpp", []),
    ("mit_law estimator wrap-safe", "test_mit_law.cpp", []),
    # joint_cal.h encoder-chip guard: (JOINT_ID, calZeaUsable expected, banner text)
    ("joint_cal J01 (MT6835 zea)",  "test_joint_cal.cpp", ["-DJOINT_ID=1", "-DEXPECT_USABLE=1", "-DEXPECT_TEXT=\"press 'V'\""]),
    ("joint_cal J03 (MT6835 zea)",  "test_joint_cal.cpp", ["-DJOINT_ID=3", "-DEXPECT_USABLE=1", "-DEXPECT_TEXT=\"press 'V'\""]),
    ("joint_cal J04 (unbuilt)",     "test_joint_cal.cpp", ["-DJOINT_ID=4", "-DEXPECT_USABLE=0", "-DEXPECT_TEXT=\"UNCALIBRATED\""]),
    # Row 14 = J01 belt-off baseline: zea measured through the MT6816 -> ignored.
    ("joint_cal row 14 (stale chip)", "test_joint_cal.cpp", ["-DJOINT_ID=14", "-DEXPECT_USABLE=0", "-DEXPECT_TEXT=\"IGNORED\""]),
]


def compiler():
    if os.environ.get("CXX"):
        return [os.environ["CXX"]]
    for c in ("g++", "clang++"):
        if shutil.which(c):
            return [c]
    try:
        import ziglang  # noqa: F401
        return [sys.executable, "-m", "ziglang", "c++"]
    except ImportError:
        sys.exit("no host C++ compiler: set CXX, install g++/clang++, or pip install ziglang")


def main():
    cxx = compiler()
    flt = sys.argv[1] if len(sys.argv) > 1 else ""
    failed = 0
    with tempfile.TemporaryDirectory() as tmp:
        for i, (name, src, defs) in enumerate(CASES):
            if flt not in name:
                continue
            exe = os.path.join(tmp, "t%d.exe" % i)
            cmd = cxx + ["-std=gnu++11", "-O2", "-w", "-I", MOCK, "-I", SRC, "-I", PROTO] + defs + \
                  [os.path.join(HERE, src), "-o", exe]
            b = subprocess.run(cmd, capture_output=True, text=True)
            if b.returncode:
                print("BUILD FAIL  %s\n%s" % (name, b.stderr[-3000:]))
                failed += 1
                continue
            r = subprocess.run([exe], capture_output=True, text=True)
            tail = (r.stdout.strip().splitlines() or ["(no output)"])[-1]
            print("%s  %-30s %s" % ("PASS" if r.returncode == 0 else "FAIL", name, tail))
            if r.returncode:
                print(r.stdout[-3000:])
                failed += 1
    print("ALL PASS" if not failed else "%d FAILED" % failed)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
