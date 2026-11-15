"""Inspect built G473 artifacts; no hardware access. Run after PIO metadata export."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PIO = Path.home() / ".platformio"
NM = PIO / "packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm.exe"
META = ROOT / ".pio/g473-validation/metadata.json"


def require(ok, message):
    if not ok:
        raise AssertionError(message)


def inspect(name, meta):
    build = ROOT / ".pio/build" / name
    defines = meta["defines"]
    joint_id = int(name.rsplit("J", 1)[1])
    for expected in ("STM32G473xx", "QUAD_G473", "ARDUINO_B_G431B_ESC1", f"JOINT_ID={joint_id}"):
        require(expected in defines, f"{name}: missing {expected}")
    require("STM32G431xx" not in defines, "wrong device macro")
    require('VARIANT_H="variant_G473_FOC.h"' in defines, "wrong variant")
    elf = build / "firmware.elf"
    nm = subprocess.check_output([str(NM), "-C", "-S", str(elf)], text=True)
    require(re.search(r"^20018000 \w _estack$", nm, re.M), "stack must end at 96 KiB")
    for symbol, size in (("adcBuffer1", 10), ("adcBuffer2", 2)):
        match = re.search(rf"^([0-9a-f]+) ([0-9a-f]+) B {symbol}$", nm, re.M)
        require(match is not None, f"missing {symbol}")
        address, actual_size = (int(s, 16) for s in match.groups())
        require(actual_size == size and 0x20000000 <= address <= 0x20018000 - size,
                f"{symbol} outside ordinary SRAM or wrong length")
    for irq in ("DMA1_Channel1_IRQHandler", "DMA1_Channel2_IRQHandler"):
        require(re.search(rf"^\w+ \w+ T {irq}$", nm, re.M), f"{irq} not strong")
    # These interrupt slots belong to the G473 vector, not the G431 vector.
    for irq in ("ADC3_IRQHandler", "ADC4_IRQHandler", "ADC5_IRQHandler"):
        require(irq in nm, f"missing G473 vector slot {irq}")
    require(nm.count(" T _configureADCLowSide(") == 1, "wrong backend count")
    for obj in build.glob("lib*/Simple FOC/current_sense/hardware_specific/stm32/stm32g4/*.o"):
        symbols = subprocess.check_output([str(NM), "--defined-only", str(obj)], text=True)
        # Arduino.h emits this weak time shim even when the backend is excluded.
        symbols = "\n".join(line for line in symbols.splitlines()
                            if not line.endswith(" W _gettimeofday"))
        require(not re.search(r"^\w+ [TtDdBbRrWwVv] ", symbols, re.M),
                f"generic G4 backend unexpectedly active: {obj}")
    binary = (build / "firmware.bin").read_bytes()
    require(len(binary) <= 512 * 1024, "flash overflow")
    # The provisional warning is compiled out once a dated calibration is stored.
    # Device and role identification must remain present in either state.
    role = b"TIER0" if name.startswith("T0_") else b"HARNESS"
    require(b"STM32G473CEU6" in binary and b"BOARD G473_FOC" in binary
            and role in binary, "missing board/role banner")
    print(f"PASS {name}: {len(binary)} binary bytes; SHA256 {hashlib.sha256(binary).hexdigest()}")


if __name__ == "__main__":
    try:
        metadata = json.loads(META.read_text(encoding="utf-8-sig"))
        targets = sys.argv[1:] or ["G473_J04", "T0_G473_J04"]
        for target in targets:
            inspect(target, metadata[target])
        before = ROOT / ".pio/g473-validation/J01-before.bin"
        require(before.read_bytes() == (ROOT / ".pio/build/J01/firmware.bin").read_bytes(),
                "G431 J01 binary changed")
        print("PASS G431 J01 binary byte-identical to pre-port build")
    except (AssertionError, OSError, KeyError, subprocess.CalledProcessError) as error:
        print(f"FAIL: {error}")
        sys.exit(1)
