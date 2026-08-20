#!/usr/bin/env python
"""
osm_building_stats.py — what shape is a suburban commercial building, really?

`osm_measurements.md` records the buildings query as an OPEN ITEM: it hit a
ConnectionError after Overpass rate-limited an earlier run, so the generator's
building footprints were never OSM-calibrated. This closes that for the
commercial case, which is the one where guessing goes wrong fastest — a
suburban shop is not a scaled-up house, it is a box with a deep plan and a wide
frontage, and the aspect ratio is the whole difference.

Aggregates only. No per-object data is written, which keeps this clear of
ODbL's share-alike (that attaches to redistributing a Derivative Database;
summary statistics are facts).

    (c) OpenStreetMap contributors — ODbL.

Run:

    uv run --no-project --with shapely scene_gen/tools/osm_building_stats.py
    uv run --no-project --with shapely scene_gen/tools/osm_building_stats.py \
        --place naperville_il --dist 1600

Two Overpass requests per place, not osmnx's eight — the heavy polygon queries
are exactly what earns the rate limit that then starves everything after them.

WHAT IT MEASURES, AND WHY EACH ONE
----------------------------------
    footprint area      how big a unit actually is
    short / long side   from the MINIMUM ROTATED RECTANGLE, not the axis-aligned
                        bbox: suburban buildings sit at whatever angle the
                        parking lot does, and an AABB of a rotated box
                        overstates both sides (median 1.7x on the suburb lots
                        this generator already plats)
    aspect  long:short   the number that says "box, not L"
    levels              building:levels, which is what sets storey count
"""

import argparse
import json
import math
import urllib.error
import urllib.parse
import urllib.request

# Five US subdivisions, the same sample the street measurements cite — recorded
# here as coordinates because the earlier run only ever named them in prose,
# which made it impossible to reproduce.
SUBURBS = {
    "levittown_ny":  (40.7259, -73.5143),   # the archetypal post-war tract
    "naperville_il": (41.7508, -88.1535),
    "plano_tx":      (33.0198, -96.6989),
    "mesa_az":       (33.4152, -111.8315),
    "cary_nc":       (35.7915, -78.7811),
}

ENDPOINT = "https://overpass-api.de/api/interpreter"

# What counts as suburban commercial. Split so the report can say which kind
# each measurement came from — a supermarket and a kiosk are both "retail" and
# are nothing alike in plan.
BUILDING_VALUES = ("retail|commercial|supermarket|office|industrial|warehouse|"
                   "kiosk|service|shop|hotel|civic|public")


def _bbox(center, dist_m):
    lat, lon = center
    dlat = dist_m / 111_320.0
    dlon = dist_m / (111_320.0 * max(0.2, math.cos(math.radians(lat))))
    return lat - dlat, lon - dlon, lat + dlat, lon + dlon


def _local_xy(center):
    """Equirectangular metres about *center*. Fine over a 3 km box."""
    lat0, lon0 = center
    k = math.cos(math.radians(lat0))
    return lambda lat, lon: ((lon - lon0) * 111_320.0 * k,
                             (lat - lat0) * 111_320.0)


def _overpass(query, timeout=180, retries=3):
    data = urllib.parse.urlencode({"data": query}).encode()
    last = None
    for attempt in range(retries):
        try:
            req = urllib.request.Request(
                ENDPOINT, data=data,
                headers={"User-Agent": "airstack-scene-gen/1.0 (research)"})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode())
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError) as e:
            last = e
            if attempt < retries - 1:
                import time
                time.sleep(12 * (attempt + 1))   # rate limit: back off, don't hammer
    raise RuntimeError(f"overpass failed after {retries}: {last}")


def _min_rot_rect(poly):
    """(short, long) of the minimum-area rectangle, in metres."""
    from shapely.geometry import Polygon
    g = Polygon(poly)
    if not g.is_valid:
        g = g.buffer(0)
    if g.is_empty or g.area <= 0:
        return None
    r = g.minimum_rotated_rectangle
    xs, ys = r.exterior.coords.xy
    sides = []
    for i in range(4):
        dx, dy = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
        sides.append(math.hypot(dx, dy))
    a, b = sorted(sides[:2] if len(sides) >= 2 else [0, 0])
    return (a, b) if a and b else None


def _q(vals, unit="", nd=1):
    if not vals:
        return "n/a"
    s = sorted(vals)
    n = len(s)

    def p(f):
        return s[min(n - 1, int(f * n))]
    return (f"{p(0.5):.{nd}f}{unit} (IQR {p(0.25):.{nd}f}-{p(0.75):.{nd}f}, "
            f"n={n})")


def measure(name, center, dist):
    south, west, north, east = _bbox(center, dist)
    bb = f"{south},{west},{north},{east}"
    q = (f'[out:json][timeout:180];'
         f'(way["building"~"^({BUILDING_VALUES})$"]({bb});'
         f' way["building"]["shop"]({bb});'
         f' way["building"]["amenity"~"^(fast_food|restaurant|cafe|bank|'
         f'pharmacy|fuel|car_wash|veterinary|clinic)$"]({bb});'
         f');out geom tags;')
    data = _overpass(q)
    to_xy = _local_xy(center)

    rows = []
    for el in data.get("elements", []):
        geom = el.get("geometry") or []
        if len(geom) < 4:
            continue
        pts = [to_xy(g["lat"], g["lon"]) for g in geom]
        mr = _min_rot_rect(pts)
        if not mr:
            continue
        short, long_ = mr
        if not (25.0 <= short * long_ <= 30_000.0):   # drop sheds and malls
            continue
        t = el.get("tags", {})
        kind = (t.get("shop") or t.get("amenity") or t.get("building") or "?")
        lv = t.get("building:levels")
        try:
            lv = int(float(lv)) if lv is not None else None
        except ValueError:
            lv = None
        rows.append({"short": short, "long": long_, "area": short * long_,
                     "aspect": long_ / max(1e-6, short), "kind": kind,
                     "levels": lv})
    return rows


def report(name, rows):
    print(f"\n=== {name}: {len(rows)} commercial footprints ===")
    if not rows:
        print("  (none)")
        return
    print(f"  footprint area   {_q([r['area'] for r in rows], ' m2', 0)}")
    print(f"  short side       {_q([r['short'] for r in rows], ' m')}")
    print(f"  long side        {_q([r['long'] for r in rows], ' m')}")
    print(f"  aspect long:short{_q([r['aspect'] for r in rows], 'x', 2)}")
    lv = [r["levels"] for r in rows if r["levels"]]
    print(f"  building:levels  {_q(lv, '', 1)}"
          f"   ({100 * len(lv) // max(1, len(rows))}% tagged)")
    from collections import Counter
    c = Counter(r["kind"] for r in rows)
    top = ", ".join(f"{k}={v}" for k, v in c.most_common(10))
    print(f"  kinds            {top}")
    # the question the generator actually asks
    boxy = sum(1 for r in rows if r["aspect"] < 2.5)
    print(f"  aspect < 2.5     {100 * boxy // len(rows)}%  <- how many are 'a box'")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--place", default="all",
                    help=f"one of: {', '.join(SUBURBS)}, or all")
    ap.add_argument("--dist", type=int, default=1600,
                    help="half-width of the sample box in metres")
    args = ap.parse_args()

    places = SUBURBS if args.place == "all" else {args.place: SUBURBS[args.place]}
    allrows = []
    for name, center in places.items():
        try:
            rows = measure(name, center, args.dist)
        except Exception as e:
            print(f"\n=== {name}: FAILED — {e}")
            continue
        report(name, rows)
        allrows += rows
    if len(places) > 1:
        report("ALL SUBURBS POOLED", allrows)


if __name__ == "__main__":
    main()
