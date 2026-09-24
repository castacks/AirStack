#!/usr/bin/env python3
"""spawn_utils.py — generate randomized, collision-free drone spawn configs.

Used by the Pegasus launch scripts to place N drones inside a user-specified
spawn area (a rectangle or arbitrary convex polygon, in world meters) such
that every drone is:

  * inside the polygon (so it can't spawn through a wall / outside the map),
  * at least `min_dist` metres from every other drone (takeoff clearance),
  * given a random yaw (orientation randomized per the experiment design).

The spawn area is passed as a list of (x, y) corners. Two corners are treated
as opposite corners of an axis-aligned rectangle; three or more corners are
treated as the vertices of a convex polygon (in order).

`generate_spawn_configs` returns the same DRONE_CONFIGS dict shape the launch
scripts already consume:

    {"domain_id", "x_m", "y_m", "z_m", "orient" [x,y,z,w], "lidar_min_range"}

Placement uses rejection sampling. If the area is too small to fit N drones at
`min_dist` spacing, it raises RuntimeError rather than silently overlapping
them — a loud failure is far better than two drones spawning on top of each
other.
"""

import math
import random


def _rect_from_two_corners(poly):
    """Expand [(x0,y0),(x1,y1)] (opposite corners) into 4 ordered corners."""
    (x0, y0), (x1, y1) = poly
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def normalize_polygon(poly):
    """Accept 2 corners (axis-aligned rect) or >=3 corners (polygon);
    return a list of >=3 (x, y) vertices."""
    poly = [(float(x), float(y)) for x, y in poly]
    if len(poly) == 2:
        return _rect_from_two_corners(poly)
    if len(poly) < 3:
        raise ValueError(f"spawn polygon needs 2 (rect) or >=3 corners, got {poly}")
    return poly


def point_in_polygon(p, poly):
    """Ray-casting point-in-polygon (works for convex and concave polys).
    Points exactly on an edge may test either way — fine for spawn sampling."""
    x, y = p
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
           (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def polygon_centroid(poly):
    """Area-weighted centroid of a polygon (for defaulting the overhead
    camera / search center to the middle of the spawn area)."""
    poly = normalize_polygon(poly)
    a = 0.0
    cx = 0.0
    cy = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cross = x0 * y1 - x1 * y0
        a += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    if abs(a) < 1e-9:                       # degenerate — fall back to mean
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        return (sum(xs) / n, sum(ys) / n)
    a *= 0.5
    return (cx / (6 * a), cy / (6 * a))


def _yaw_to_quat_xyzw(yaw):
    """Yaw (rotation about +Z) → quaternion [x, y, z, w], matching the
    init_orient convention the spawn nodes expect."""
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def generate_spawn_configs(poly, n, z_m, lidar_min_range,
                           min_dist=3.0, margin=0.0,
                           randomize_yaw=True, seed=None, max_tries=20000):
    """Generate `n` DRONE_CONFIGS placed inside `poly`, >= `min_dist` apart.

    poly:        2 opposite corners (rect) or >=3 polygon vertices, world metres.
    z_m:         spawn height above floor (metres).
    margin:      shrink the sampling bounding box by this many metres on every
                 side (keep drones off the very edge of the area).
    randomize_yaw: random heading per drone; False → all face +X (yaw 0).
    seed:        int/str for reproducible layouts; None → fresh layout each run.

    Raises RuntimeError if `n` drones can't be placed within `max_tries`.
    """
    poly = normalize_polygon(poly)
    rng = random.Random(seed)

    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    xmin, xmax = min(xs) + margin, max(xs) - margin
    ymin, ymax = min(ys) + margin, max(ys) - margin
    if xmin >= xmax or ymin >= ymax:
        raise RuntimeError(
            f"spawn area collapsed after margin={margin}m (bbox x[{xmin},{xmax}] "
            f"y[{ymin},{ymax}]) — reduce margin or enlarge the spawn polygon")

    pts = []
    tries = 0
    while len(pts) < n and tries < max_tries:
        tries += 1
        p = (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
        if not point_in_polygon(p, poly):
            continue
        if any(math.hypot(p[0] - q[0], p[1] - q[1]) < min_dist for q in pts):
            continue
        pts.append(p)

    if len(pts) < n:
        raise RuntimeError(
            f"could not place {n} drones >= {min_dist}m apart inside {poly} "
            f"(placed {len(pts)} in {max_tries} tries) — the spawn area is too "
            f"small for that many drones at this spacing")

    configs = []
    for i, (x, y) in enumerate(pts, start=1):
        yaw = rng.uniform(-math.pi, math.pi) if randomize_yaw else 0.0
        configs.append({
            "domain_id": i,
            "x_m": round(x, 3),
            "y_m": round(y, 3),
            "z_m": z_m,
            "orient": _yaw_to_quat_xyzw(yaw),
            "lidar_min_range": lidar_min_range,
        })
    return configs
