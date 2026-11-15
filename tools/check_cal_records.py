"""Read-only verification of consolidated AUTOCALIB, M4 and current-step evidence."""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import re

import numpy as np

CAL = Path(__file__).resolve().parents[1] / "docs/cal"


def archive(folder):
    obj = json.loads((folder / "RAW_DATA.json").read_text(encoding="utf-8"))
    items = {}
    for record in obj["records"]:
        content = record["content_utf8"].encode("utf-8")
        assert hashlib.sha256(content).hexdigest() == record["sha256"], record["name"]
        assert record["name"] not in items
        items[record["name"]] = content
    assert sorted(p.name for p in folder.iterdir()) == ["RAW_DATA.json", "README.md"]
    return items


def rows(raw):
    return list(csv.DictReader(io.StringIO(raw.decode("utf-8-sig"))))


def autocalib(folder):
    items = archive(folder)
    sources = [v.decode("utf-8-sig") for k, v in items.items() if "autocalib" in k.lower() and k.endswith((".csv", ".txt"))]
    assert len(sources) == 1
    text = sources[0]
    reports = text.count("AUTOCALIB REPORT")
    assert reports >= 1
    bins = re.findall(r"^BIN,", text, re.M)
    lsb = re.findall(r"^LSB,", text, re.M)
    assert len(bins) == 32*reports and len(lsb) > 0
    checked = 0
    for section in text.split("[3] R/U0")[1:]:
        points = np.array(re.findall(r"^\s+U=([\d.]+) I=([\d.]+) drift=", section.split("[4]")[0], re.M), float)
        printed = re.search(r"R=([\d.]+) \+-", section)
        if len(points) and printed:
            fit = np.polyfit(points[:, 1], points[:, 0], 1)
            assert abs(fit[0]-float(printed[1])) < 3e-5
            checked += 1
    if "refit_results.json" in items:
        saved = json.loads(items["refit_results.json"])
        assert np.isclose(saved["R_U0"]["slope"], fit[0], atol=1e-9)
        assert np.isclose(saved["R_U0"]["intercept"], fit[1], atol=1e-9)
    print(f"PASS {folder.parent.name} AUTOCALIB: {len(items)} hashes; {reports} reports, {len(bins)} BIN and {len(lsb)} LSB records; {checked} R refits")


def current_steps(folder):
    items = archive(folder)
    original = rows(items["J04_current_step_summary.csv"])
    repeat = rows(items["J04_current_step_negative_repeat_summary.csv"])[0]
    cases = [("J04_current_step_cap1.csv", original[0], "settling_to_5pct_ms", "last10ms_Iq_mean_A_rep"),
             ("J04_current_step_cap2.csv", original[1], "settling_to_5pct_ms", "last10ms_Iq_mean_A_rep"),
             ("J04_current_step_negative_repeat.csv", repeat, "settle_ms", "final_mean_A")]
    for name, summary, settle_key, mean_key in cases:
        points = rows(items[name])
        assert len(points) == 1000
        t, q, sp = (np.array([float(r[key]) for r in points]) for key in ("t_us", "Iq", "sp"))
        assert np.all(np.diff(t) > 0)
        edge = np.flatnonzero(sp != sp[0])[0]
        outside = np.flatnonzero((np.arange(len(t)) >= edge) & (abs(q-sp[-1]) > .075))
        last = outside[-1]+1
        assert last < len(t)
        settling = (t[last]-t[edge])/1000
        mean = np.mean(q[t >= t[-1]-10000])
        assert abs(settling-float(summary[settle_key])) < 1e-9
        assert abs(mean-float(summary[mean_key])) < 1e-9
    print("PASS J04 CURRENT_STEP: all archive hashes, 3000 timestamped samples, three independently recomputed outcomes")


def m4(folder):
    data = rows((folder / "RAW_DATA.csv").read_bytes())
    joint = folder.parent.name
    assert len(data) == {"J01": 11, "J02": 18, "J04": 12, "J05": 12}[joint]
    cpr = 65536 if joint in ("J04", "J05") else 16384
    for record in data:
        assert record["direction"] in ("+", "-")
        assert float(record["breakaway_A_rep"]) > 0
        assert abs(float(record["travel_motor_mrad"])-int(record["travel_count"])*2*np.pi/cpr*1000) < 1e-6
    if joint in ("J04", "J05"):
        primary = [r for r in data if r["block"] == "primary"]
        repeat = [r for r in data if r["block"] == "repeat_P1"]
        assert len(primary) == 10 and len(repeat) == 2
        assert [r["direction"] for r in primary] == ["+", "-"] * 5
        assert [int(r["point"]) for r in primary] == [p for p in range(1, 6) for _ in range(2)]
        amps = np.array([float(r["breakaway_A_rep"]) for r in primary])
        assert .20 <= amps.mean() <= .40 and amps.std(ddof=1) <= .12
        assert max(amps) <= .60 and all(abs(float(r["travel_motor_mrad"])) <= 76.7 for r in primary)
        assert all(not r["recorded_warning"] for r in primary)
        expected = .3280 if joint == "J04" else .3380
        assert abs(amps.mean() - expected) < 1e-9
        if joint == "J05":
            assert abs(float(repeat[1]["travel_motor_mrad"])) > 76.7
            assert "biased HIGH" in repeat[1]["recorded_warning"]
    assert sorted(p.name for p in folder.iterdir()) == ["RAW_DATA.csv", "README.md"]
    print(f"PASS {folder.parent.name} M4: {len(data)} complete observations, direction/block/warnings retained, physical-unit conversion checked")


def belt_on(folder):
    items = archive(folder)
    source = next(v.decode("utf-8-sig") for name, v in items.items() if name.endswith((".txt", ".csv")))
    sweeps = 0
    for block in source.split("[5] FREE-SPIN")[1:]:
        points = np.array(re.findall(r"^\s+U=[-\d.]+ vel=([-\d.]+) Iq=([-\d.]+)", block, re.M), float)
        assert points.shape == (10, 2)
        for direction, sign in (("fwd", 1), ("rev", -1)):
            selected = points[points[:, 0]*sign > 0]
            slope, intercept = np.polyfit(abs(selected[:, 0]), abs(selected[:, 1]), 1)
            printed = re.search(rf"drag {direction}\s+Iq = ([\d.]+) \+ ([\d.]+)", block)
            if printed:
                # Console inputs round speed to .01 rad/s and current to .0001 A.
                # Propagate those bounds through OLS rather than demanding that
                # a refit of rounded points equal a fit of unrounded samples.
                design = np.column_stack((abs(selected[:, 0]), np.ones(len(selected))))
                weights = np.linalg.pinv(design)
                bounds = np.sum(abs(weights), axis=1) * (.00005+abs(slope)*.005)
                assert abs(intercept-float(printed[1])) < bounds[1]+.00005
                assert abs(slope-float(printed[2])) < bounds[0]+.0000005
        sweeps += 1
    captures = len(re.findall(r"^# cap=", source, re.M))
    assert captures == len(re.findall(r"^# END", source, re.M))
    print(f"PASS {folder.parent.parent.name} {folder.name}: hashes, {sweeps} two-direction drag refits, {captures} complete capture blocks")


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--joint", choices=[f"J{i:02}" for i in range(1, 6)])
    args = parser.parse_args()
    for joint in sorted(CAL.glob("J[0-9][0-9]")):
        if args.joint and joint.name != args.joint:
            continue
        for name, check in (("AUTOCALIB", autocalib), ("M4", m4), ("CURRENT_STEP", current_steps)):
            if (joint / name).exists():
                check(joint / name)
        for folder in sorted((joint / "BELT_ON").glob("*")):
            if folder.is_dir():
                belt_on(folder)


if __name__ == "__main__":
    main()
