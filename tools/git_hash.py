# PlatformIO pre-script: stamp the build with the git commit it came from.
#
# Used by the Tier-0 envs (platformio.ini [tier0]) and the ESP32 master
# (master/platformio.ini). NOT by the bench harness envs: a hash that changes
# every commit would break the harness's byte-identical md5 gate.
#
# Defines:
#   GIT_HASH   first 8 hex digits of HEAD, as a uint32 (0 if git is unavailable)
#   GIT_DIRTY  1 if tracked files differ from HEAD, else 0
#
# Why it exists: two firmwares now exist per board (harness and Tier 0), so a
# stale or wrong binary is easy to flash. Tier 0 reports these in its STATUS
# frame and the master prints them on IDENT (CAN-T0 plan, owner addition).
Import("env")  # noqa: F821 -- provided by PlatformIO/SCons
import subprocess


def _git(*args):
    try:
        return subprocess.check_output(
            ["git"] + list(args),
            cwd=env.subst("$PROJECT_DIR"),  # noqa: F821
            stderr=subprocess.DEVNULL,
        ).decode().strip()
    except Exception:
        return ""


_h = _git("rev-parse", "--short=8", "HEAD")
_hash = int(_h, 16) if _h else 0
_dirty = 1 if _git("status", "--porcelain", "--untracked-files=no") else 0

env.Append(CPPDEFINES=[  # noqa: F821
    ("GIT_HASH", "0x%08Xu" % _hash),
    ("GIT_DIRTY", str(_dirty)),
])
print("git_hash.py: GIT_HASH=0x%08X GIT_DIRTY=%d" % (_hash, _dirty))
