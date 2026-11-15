"""Independent coordinate fixture: signs, units, offset and frame invariance."""
import importlib.util
from pathlib import Path
import math
import unittest

path = Path(__file__).resolve().parents[2] / 'tools/leg_reference.py'
spec = importlib.util.spec_from_file_location('reference_tool', path)
module = importlib.util.module_from_spec(spec)
exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)


class ReferenceTest(unittest.TestCase):
    points = [(20, 0), (-20, 0), (60, -60), (-60, -60), (0, -140)]

    def test_known_geometry_offset_and_branch(self):
        r = module.reference(*self.points, [.1, -.2])
        self.assertEqual(r['branch'], [1, -1, 1])
        self.assertAlmostEqual(r['spacing_mm'], 40)
        self.assertEqual(r['lower_mm'], [100, 100])
        self.assertAlmostEqual(r['reference_deg'][0], math.degrees(math.atan2(-60, 40)-.1))
        self.assertAlmostEqual(r['reference_deg'][1], math.degrees(math.atan2(-60, -40)+.2))

    def test_rotated_translated_grid(self):
        moved = [(-z+120, x-37) for x, z in self.points]
        a = module.reference(*self.points, [0, 0])
        b = module.reference(*moved, [0, 0])
        self.assertEqual(a, b)

    def test_bad_measurements(self):
        for foot in [(float('nan'), 0), (60, -60), (0, -60)]:
            with self.assertRaises(ValueError):
                module.reference(*self.points[:4], foot, [0, 0])
        with self.assertRaises(ValueError):
            module.reference(self.points[0], self.points[0], *self.points[2:], [0, 0])


if __name__ == '__main__':
    unittest.main()
