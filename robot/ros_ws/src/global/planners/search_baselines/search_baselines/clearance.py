"""Known-obstacle clearance for lane legs — the 3D lawnmower's altitude surface.

WHY. A lane goal is a flat (x, y) at `flight_altitude_m`. Over a suburb at
12 m that point is inside a tree canopy every few lanes, `disparity_expansion`
marks the canopy solid, droan_gl can never get within `goal_tolerance_m`,
and the sweep sits there until the stall timer skips the leg — thirty
seconds lost AND the ground under the canopy never swept. A coverage path is
planned over a KNOWN map by definition, so the lawnmower is allowed to know
where the obstacles are: the layout generator wrote them (house, tree, car
boxes in the scene's annotation JSON, the same file the GCS draws), and each
leg is flown at `max(cruise, tallest_thing_along_the_leg + clearance)`. Over
open ground the drone stays at cruise, where the detector works; over a
canopy it pops up and comes back down on the way to the next goal.

FRAMES. The annotation boxes are in the Isaac WORLD frame. The planner's
lanes are in this robot's MAP frame (takeoff-anchored); `shifted()` moves
the boxes by the same `enu(fix) - odom` vector `_place_search_area` applies
to the search polygon, so boxes and lanes disagree by nothing.

Pure numpy, no ROS, so `tests/test_lawnmower_baseline.py` can drive it.
"""

import json
import math

import numpy as np

# Classes that are OBSTACLES to a drone at cruise height. People are in the
# same file and are deliberately not here: they are what the search is for.
OBSTACLE_CLASSES = ('house', 'building', 'tree', 'car', 'truck', 'prop')


def load_boxes(path, classes=OBSTACLE_CLASSES):
    """`[(cx, cy, hx, hy, top_z, bottom_z, class), ...]` from an annotation
    JSON (`[{"class", "bbox_world": {"center_xyz_m", "size_xyz_m"}}]`).
    Unknown classes and malformed entries are dropped, not raised on."""
    with open(path, encoding='utf-8') as fh:
        doc = json.load(fh)
    items = doc if isinstance(doc, list) else (doc.get('boxes') or [])
    want = {str(c).lower() for c in classes}
    out = []
    for b in items:
        try:
            cls = str(b.get('class', '')).lower()
            if cls not in want:
                continue
            bw = b.get('bbox_world') or {}
            c, s = bw['center_xyz_m'], bw['size_xyz_m']
            out.append((float(c[0]), float(c[1]),
                        abs(float(s[0])) / 2.0, abs(float(s[1])) / 2.0,
                        float(c[2]) + abs(float(s[2])) / 2.0,
                        float(c[2]) - abs(float(s[2])) / 2.0, cls))
        except (KeyError, TypeError, ValueError, IndexError, AttributeError):
            continue
    return out


class KnownObstacles:
    """Axis-aligned boxes with a top height, queried along a 2D segment."""

    def __init__(self, boxes, inflate_m=0.0):
        boxes = list(boxes or ())
        arr = np.array([[b[0], b[1], b[2], b[3], b[4], b[5]] for b in boxes],
                       dtype=float).reshape(-1, 6)
        self.classes = [str(b[6]) for b in boxes]
        self._raw = arr
        self.inflate = float(inflate_m)

    def __len__(self):
        return int(self._raw.shape[0])

    @property
    def top(self):
        return self._raw[:, 4]

    def shifted(self, dx, dy, dz=0.0):
        """The same boxes in a frame translated by (-dx, -dy, -dz): world
        boxes -> map boxes for a map whose origin sits at world (dx, dy, dz)."""
        other = KnownObstacles([], self.inflate)
        arr = self._raw.copy()
        arr[:, 0] -= float(dx)
        arr[:, 1] -= float(dy)
        arr[:, 4] -= float(dz)
        arr[:, 5] -= float(dz)
        other._raw = arr
        other.classes = list(self.classes)
        return other

    def hits_along(self, a, b):
        """Boolean mask: which INFLATED boxes the segment a->b passes through
        (2D). Slab test in the segment's own parameter, vectorised."""
        n = len(self)
        if n == 0:
            return np.zeros(0, dtype=bool)
        a = np.asarray(a, dtype=float).reshape(2)
        b = np.asarray(b, dtype=float).reshape(2)
        cx, cy = self._raw[:, 0], self._raw[:, 1]
        hx, hy = self._raw[:, 2] + self.inflate, self._raw[:, 3] + self.inflate
        lo = np.stack([cx - hx, cy - hy], axis=1)
        hi = np.stack([cx + hx, cy + hy], axis=1)
        d = b - a
        t0 = np.zeros(n)
        t1 = np.ones(n)
        ok = np.ones(n, dtype=bool)
        for i in range(2):
            if abs(d[i]) < 1e-12:
                ok &= (a[i] >= lo[:, i]) & (a[i] <= hi[:, i])
                continue
            ta = (lo[:, i] - a[i]) / d[i]
            tb = (hi[:, i] - a[i]) / d[i]
            tmin, tmax = np.minimum(ta, tb), np.maximum(ta, tb)
            t0 = np.maximum(t0, tmin)
            t1 = np.minimum(t1, tmax)
        return ok & (t0 <= t1)

    def top_along(self, a, b):
        """`(top_z, index)` of the tallest inflated box the segment a->b
        crosses, or `(None, -1)` when it crosses none."""
        m = self.hits_along(a, b)
        if not m.any():
            return None, -1
        idx = np.flatnonzero(m)
        k = int(idx[np.argmax(self._raw[idx, 4])])
        return float(self._raw[k, 4]), k

    def top_at(self, p):
        return self.top_along(p, p)


def leg_z(top, cruise, clearance_m, lo=0.0, hi=0.0, lift_m=0.0):
    """Altitude for one leg: cruise, raised to `top + clearance` when
    something stands under it, plus any stall lift, clamped to the flight
    band (0 on either bound = unconstrained, as `_clamp_alt`)."""
    z = float(cruise)
    if top is not None:
        z = max(z, float(top) + float(clearance_m))
    z += float(lift_m)
    if lo > 0.0:
        z = max(z, float(lo))
    if hi > 0.0:
        z = min(z, float(hi))
    return z


def _selftest():
    ob = KnownObstacles([(10.0, 0.0, 5.0, 5.0, 9.0, 0.0, 'house'),
                         (50.0, 50.0, 6.0, 6.0, 18.0, 0.0, 'tree')], inflate_m=1.0)
    assert len(ob) == 2
    assert ob.top_along((0.0, 0.0), (30.0, 0.0)) == (9.0, 0)      # through the house
    assert ob.top_along((0.0, 20.0), (30.0, 20.0)) == (None, -1)  # clear
    assert ob.top_along((4.5, -20.0), (4.5, 20.0))[1] == 0        # inflated edge
    assert ob.top_along((3.5, -20.0), (3.5, 20.0))[1] == -1       # just outside
    assert ob.top_along((48.0, 48.0), (49.0, 49.0)) == (18.0, 1)  # inside a box
    assert ob.top_at((50.0, 50.0)) == (18.0, 1)
    sh = ob.shifted(10.0, 0.0, 1.0)
    assert sh.top_along((-10.0, 0.0), (5.0, 0.0)) == (8.0, 0)
    assert ob.top_along((-10.0, 0.0), (5.0, 0.0)) == (9.0, 0)     # original untouched
    assert leg_z(None, 12.0, 3.0) == 12.0
    assert leg_z(9.0, 12.0, 3.0) == 12.0                            # 9+3 == cruise
    assert leg_z(18.0, 12.0, 3.0) == 21.0
    assert leg_z(18.0, 12.0, 3.0, hi=20.0) == 20.0
    assert leg_z(None, 12.0, 3.0, lo=2.0, hi=25.0, lift_m=15.0) == 25.0
    empty = KnownObstacles([])
    assert empty.top_along((0, 0), (1, 1)) == (None, -1)
    assert math.isclose(leg_z(None, 12.0, 3.0, lift_m=5.0), 17.0)
    return True


if __name__ == '__main__':
    _selftest()
    print('clearance: selftest ok')
