"""Offline reference from measured pivot centres in a common x/z millimetre frame.

Axes: x from J03 toward J01, z toward housing (CCW positive). No device access.
All centres and q must describe the same stationary pose and zero session.
"""
import argparse
import json
import math


def reference(h1, h3, k1, k3, foot, q):
    if not all(math.isfinite(v) for pair in (h1, h3, k1, k3, foot, q) for v in pair):
        raise ValueError("all measurements must be finite")
    def sub(a, b):
        return (a[0]-b[0], a[1]-b[1])
    def cross(a, b):
        return a[0]*b[1]-a[1]*b[0]
    def norm(a):
        return math.hypot(*a)
    axis = sub(h1, h3)
    spacing = norm(axis)
    if spacing <= 0:
        raise ValueError("driven centres must be distinct")
    upper = [sub(k1, h1), sub(k3, h3)]
    lower = [sub(foot, k1), sub(foot, k3)]
    pairs = [(sub(k3, k1), lower[0]), (upper[0], lower[0]), (upper[1], lower[1])]
    signs = []
    for a, b in pairs:
        scale = norm(a)*norm(b)
        if scale == 0 or abs(cross(a, b))/scale < math.sin(math.radians(15)):
            raise ValueError("measurement pose too close to straight/folded/collinear; use an interior pose")
        signs.append(1 if cross(a, b) > 0 else -1)
    # Rotation relative to measured hip line also permits a rotated paper grid.
    theta = [math.atan2(cross(axis, u), axis[0]*u[0]+axis[1]*u[1]) for u in upper]
    return {"measurement_only": True,
            "reference_deg": [math.degrees(t-d) for t, d in zip(theta, q)],
            "pose_theta_deg": list(map(math.degrees, theta)),
            "branch": signs, "spacing_mm": spacing,
            "upper_mm": list(map(norm, upper)), "lower_mm": list(map(norm, lower))}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("h1", "h3", "k1", "k3", "foot"):
        p.add_argument("--"+name, nargs=2, type=float, required=True, metavar=("X_MM", "Z_MM"))
    p.add_argument("--q-rad", nargs=2, type=float, required=True, metavar=("J01", "J03"))
    a = p.parse_args()
    try:
        result = reference(a.h1, a.h3, a.k1, a.k3, a.foot, a.q_rad)
    except ValueError as e:
        p.error(str(e))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
