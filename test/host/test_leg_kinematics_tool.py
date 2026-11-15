"""PC calculator integration: real compiler/header, unit conversions, rejection exits."""
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]


class CalculatorTest(unittest.TestCase):
    def call(self, *args):
        return subprocess.run([sys.executable, str(ROOT / "tools/leg_kinematics.py"), *args],
                              cwd=ROOT, capture_output=True, text=True)

    def test_fk_independent_symmetric_geometry_and_units(self):
        r = self.call("fk", "--theta-deg", "-60", "-120", "--branch", "1", "-1", "1")
        self.assertEqual(r.returncode, 0, r.stderr)
        d = json.loads(r.stdout)
        self.assertTrue(d["geometry_only"])
        expected_z = -80 * math.sqrt(3) / 2 - math.sqrt(100**2 - 61.5**2)
        self.assertAlmostEqual(d["foot_mm"][0], 0, places=3)
        self.assertAlmostEqual(d["foot_mm"][1], expected_z, places=3)
        self.assertAlmostEqual(d["theta_rad"][0], -math.pi / 3, places=6)

    def test_sample_displacement_sign_and_ik_roundtrip(self):
        r = self.call("sample", "--ref-deg", "-60", "-120", "--q-rad", ".01", "0", "--branch", "1", "-1", "1")
        self.assertEqual(r.returncode, 0, r.stderr)
        d = json.loads(r.stdout)
        self.assertAlmostEqual(d["theta_rad"][0], -math.pi / 3 + .01, places=6)
        back = self.call("ik", "--foot-mm", *map(str, d["foot_mm"]), "--seed-deg", "-60", "-120", "--branch", "1", "-1", "1")
        self.assertEqual(back.returncode, 0, back.stderr)
        self.assertAlmostEqual(json.loads(back.stdout)["theta_rad"][0], d["theta_rad"][0], places=5)

    def test_unreachable_nonfinite_and_branch_rejections(self):
        cases = [
            ("ik", "--foot-mm", "1000", "0", "--seed-deg", "-60", "-120", "--branch", "1", "-1", "1"),
            ("fk", "--theta-deg", "nan", "-120", "--branch", "1", "-1", "1"),
            ("fk", "--theta-deg", "-60", "-120", "--branch", "1", "1", "1"),
        ]
        for args in cases:
            with self.subTest(args=args):
                r = self.call(*args)
                self.assertEqual(r.returncode, 2, r.stderr)
                self.assertNotIn("foot_mm", r.stdout)

    def test_no_default_branch(self):
        r = self.call("fk", "--theta-deg", "-60", "-120")
        self.assertEqual(r.returncode, 2)
        self.assertIn("--branch", r.stderr)


if __name__ == "__main__":
    unittest.main()
