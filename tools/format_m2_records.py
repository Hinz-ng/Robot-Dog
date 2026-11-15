"""Verify two-file M2 records, or extract archived originals on explicit request.

Default operation is read-only and never regenerates loose files in docs/cal.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import re

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def records(folder):
    bundle = json.loads((folder / "RAW_DATA.json").read_text(encoding="utf-8"))
    assert bundle["schema"] == "m2-evidence-v1"
    result = {}
    for item in bundle["records"]:
        name = item["name"]
        assert Path(name).name == name and name not in result, name
        payload = item["content_utf8"].encode("utf-8")
        assert hashlib.sha256(payload).hexdigest() == item["sha256"], name
        result[name] = payload
    return result


def csv_rows(items, name):
    return list(csv.DictReader(io.StringIO(items[name].decode("utf-8-sig"))))


def verify(folder, items):
    joint = folder.parent.name
    assert sorted(p.name for p in folder.iterdir()) == ["RAW_DATA.json", "README.md"]
    if joint in ("J01", "J02"):
        rows = csv_rows(items, f"{joint}_M2_points.csv")
        capture = items[f"{joint}_M2.csv"].decode("utf-8-sig")
        r, c, g = {"J01": (.22332, .34816, .9621),
                   "J02": (.22580, .34953, .9690)}[joint]
        assert abs(1.5*r/c-g) < .00005
    elif joint == "J04":
        rows = csv_rows(items, "J04_M2_rerun_points.csv")
        capture = items["J04_M2_rerun_console.txt"].decode("utf-8-sig")
        current = np.array([float(row["I_rep_mean_A"]) for row in rows])
        volts = np.array([float(row["V_pad_recorded_V"]) for row in rows])
        command = np.array([float(row["U_command_V"]) for row in rows])
        bus = np.array([float(row["I_bus_recorded_max_mA"])/1000 for row in rows])
        voltage_fit = np.polyfit(current, command*volts/11.2952, 1)
        power_fit = np.polyfit(current, volts*bus, 2)
        saved = csv_rows(items, "J04_M2_rerun_fit.csv")[0]
        assert np.isclose(voltage_fit[0], float(saved["R_self_ohm"]), atol=1e-10)
        assert np.isclose(power_fit[0], float(saved["c_W_per_Arep2"]), atol=1e-10)
        assert abs(1.5*voltage_fit[0]/power_fit[0]-.9702266386614543) < 1e-9
    elif joint == "J05":
        rows = csv_rows(items, "J05_M2_points.csv")
        capture = items["J05_M2.txt"].decode("utf-8-sig")
        current = np.array([float(row["I_rep_mean_A"]) for row in rows])
        volts = np.array([float(row["V_pad_V"]) for row in rows])
        command = np.array([float(row["U_command_V"]) for row in rows])
        bus = np.array([float(row["Ibus_max_mA"])/1000 for row in rows])
        voltage_fit = np.polyfit(current, command*volts/11.26, 1)
        power_fit = np.polyfit(current, volts*bus, 2)
        saved = json.loads(items["J05_M2_fit.json"])
        assert np.isclose(voltage_fit[0], saved["delivered_voltage_fit"]["R_ohm"], atol=1e-10)
        assert np.isclose(power_fit[0], saved["power_fit"]["coefficients"][2], atol=1e-10)
        assert abs(1.5*voltage_fit[0]/power_fit[0]-saved["conditional_g"]) < 1e-9
    else:
        raise ValueError(f"No registered fit verification for {joint}")
    raw_rows = [line.split(",") for line in capture.splitlines() if line.startswith("M2,")]
    assert len(raw_rows) == len(rows) == (5 if joint in ("J01", "J02") else 8)
    for raw, row in zip(raw_rows, rows):
        assert float(raw[1]) == float(row["U_command_V"])
        assert float(raw[2]) == float(row["I_rep_mean_A"])
    print(f"PASS {joint}: two files, {len(items)} hashes, {len(rows)} raw/normalized point joins, fit arithmetic")


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--joint", choices=("J01", "J02", "J04", "J05"))
    parser.add_argument("--extract-to", type=Path,
                        help="Restore original records below this directory, one subfolder per joint")
    args = parser.parse_args()
    folders = sorted((ROOT / "docs/cal").glob("J*/M2"))
    if args.joint:
        folders = [folder for folder in folders if folder.parent.name == args.joint]
    for folder in folders:
        items = records(folder)
        verify(folder, items)
        if args.extract_to:
            destination = args.extract_to.resolve() / folder.parent.name
            # Never restore legacy loose files into the live M2 folders.
            assert not destination.is_relative_to((ROOT / "docs/cal").resolve())
            destination.mkdir(parents=True, exist_ok=True)
            for name, content in items.items():
                target = destination / name
                if target.exists():
                    assert target.read_bytes() == content, f"Refusing to overwrite {target}"
                else:
                    target.write_bytes(content)
            print(f"Extracted {len(items)} records to {destination}")


if __name__ == "__main__":
    main()
