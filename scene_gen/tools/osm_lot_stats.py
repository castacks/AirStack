#!/usr/bin/env python
"""
osm_lot_stats.py — how much land does one house get, and how much more of it in
a large-lot district?

`suburb_parcel.DENSITY` scales lot width, front setback and lot depth per
district: tight 0.78 / normal 1.00 / loose 1.45 / **estate 2.20**. The first
three are defensible against the street measurements already recorded in
`osm_measurements.md`; the estate row was guessed outright. This measures it,
by running one set of statistics over two contrasting samples and printing the
ratio between them:

    A  ordinary post-war subdivision   — the five places in
                                         `osm_building_stats.SUBURBS`
    B  large-lot / estate residential  — five US towns whose single-family
                                         zoning is written in acres (below)

The B/A ratio of each figure IS the multiplier an `estate` class should carry.

LOTS, NOT BUILDINGS. `osm_building_stats.py` measures footprints, which OSM
records well. This measures the LAND under and around them, which OSM records
patchily in the US — most residential landuse in OSM is a neighbourhood blob,
not a cadastral parcel. So read the coverage block the report prints before
copying any absolute figure. The ratio is the sturdier half: every bias here
(streets counted inside landuse polygons, centreline-not-kerb setbacks,
under-tagged dwellings) lands on both samples in the same direction.

Aggregates only. No per-object data is written, which keeps this clear of
ODbL's share-alike (that attaches to redistributing a Derivative Database;
summary statistics are facts).

    (c) OpenStreetMap contributors — ODbL.

Run:

    uv run --no-project --with shapely scene_gen/tools/osm_lot_stats.py
    uv run --no-project --with shapely scene_gen/tools/osm_lot_stats.py \
        --sample estate --dist 1200

Two Overpass requests per place, for the same reason `osm_building_stats` keeps
it to two: the heavy polygon queries are exactly what earns the rate limit that
then starves everything after them. The bbox, projection, rectangle and
quantile helpers are imported from that module rather than restated; the
request function is NOT, because this one has to rotate mirrors (see
`ENDPOINTS`).

WHAT IT MEASURES, AND WHY EACH ONE
----------------------------------
    per-dwelling land       (residential landuse area) / (dwellings inside it),
                            in m2 AND acres, because US zoning is written in
                            acres. Includes the streets inside the polygon, so
                            it overstates the deeded lot by roughly the street
                            share — equally in both samples.
    parcel area/frontage/   minimum-rotated-rectangle short/long of landuse
    depth                   polygons that hold exactly ONE dwelling, i.e. the
                            rare places where OSM has real per-parcel geometry
    dwellings per ha / km2  the density figure the generator actually needs
    nearest-neighbour       distance between dwelling centroids. This is the
                            one measure that needs no landuse polygons at all,
                            so it is the fallback when parcel coverage is nil —
                            and along a street it is essentially lot frontage.
    dwellings per km        of public road centreline — NOT a result, a
                            COMPLETENESS SCREEN. Read it first. See the comment
                            at the print site.
    footprint area          for the coverage ratio (house : land) that zoning
                            writes as a maximum
    setback                 building footprint to nearest PUBLIC road
                            centreline. highway=service is excluded on purpose:
                            an estate driveway runs to the garage door and
                            would report a 2 m setback for a 40 m one.
"""

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from osm_building_stats import (  # noqa: E402
    SUBURBS, _bbox, _local_xy, _min_rot_rect, _q,
)

ACRE_M2 = 4046.8564224
HA_M2 = 10_000.0

# ---------------------------------------------------------------------------
# sample B — large-lot / estate residential
# ---------------------------------------------------------------------------
# Chosen to be genuinely low-density by ORDINANCE rather than by reputation, so
# the contrast is "half-acre-plus minimum lot" and not "wealthy area" — and
# then SCREENED, because the first cut of this list was chosen on zoning alone
# and two of the five turned out to be measuring OSM's mapping effort instead
# (the rejects are recorded below). Coordinates are kept here for the same reason `SUBURBS`
# records its own: the earlier street work named its places only in prose and
# could not be reproduced.
#
# Each entry's measured dwellings-per-km-of-road is given, since that is the
# screen every one of them had to pass.
ESTATES = {
    # Nassau County 2-acre zoning. THE CONTROL: same county, same OSM building
    # import and same mapper population as Levittown in sample A, 10 km away
    # and platted at an eighth of the land per dwelling (0.21 ac vs 1.75).
    # Whatever bias the import carries, this pair carries it on both sides
    # and it cancels in the ratio.                        11.3 dwellings/km
    "old_westbury_ny":     (40.7880, -73.5990),
    # Montgomery County RE-2, two-acre residential estate, over most of the
    # Potomac / Falls Road area. Carries the land-per-dwelling figure: 483 of
    # the estate sample's 671 ha of usable landuse.        21.6 dwellings/km
    "potomac_md":          (39.0230, -77.2050),
    # 1-acre-plus minimum, Midwest, platted early around Cranbrook: curvilinear
    # roads on big wooded parcels, which is a different geometry from a modern
    # estate cul-de-sac. 24 usable tracts, the most of any one
    # place here.                                          12.3 dwellings/km
    "bloomfield_hills_mi": (42.5800, -83.2450),
    # 1-2 acre minimum, and MassGIS's statewide footprint import means the
    # buildings are close to complete — which is exactly what the sparse
    # candidates lacked.                                   13.3 dwellings/km
    "weston_ma":           (42.3620, -71.3020),
    # RA-1 backcountry, 1-acre minimum. Contributes spacing, footprint and
    # setback only: OSM has ZERO landuse=residential polygons in this box, so
    # it adds nothing to land-per-dwelling.                14.9 dwellings/km
    "greenwich_ct":        (41.0700, -73.6300),
}

# Candidates that were measured and then dropped, kept here so the next person
# does not re-run them. Both failure modes are worth recognising.
#
#   FAILURE 1, the box is bigger than the town. atherton_ca (37.4590,
#   -122.1990) came back at 37.8 dwellings/km, 1156 m2 per dwelling and 23.0 m
#   spacing — statistically indistinguishable from sample A. Atherton is 13
#   km2, so a 5.76 km2 box centred on it is mostly its dense neighbours. A
#   one-acre town needs a box that fits inside it, or a polygon boundary rather
#   than a bbox.
#
#   FAILURE 2, the buildings are not mapped. saddle_river_nj (41.0290,
#   -74.1000) returned 55 buildings in 5.76 km2 = 1.1 dwellings/km, and
#   paradise_valley_az (33.5340, -111.9500) 4.4/km. Two-acre zoning cannot
#   produce that; the neighbouring places return 11-22. Left in, they would
#   have reported 6.5-acre lots and 36 m spacing with no warning at all.
#   lake_forest_il and barrington_hills_il failed the same way on the count
#   probe and were never measured.
#
# NOTE ON REGIONAL SPREAD, because the surviving five are Northeast-heavy
# (NY, CT, MA, MD) plus one Midwest. That is not a choice; it is what passed.
# The two Western/Sun Belt candidates failed the screens above, so this sample
# describes large-lot platting in the eastern half of the US and the West is
# simply unmeasured here.

ENDPOINTS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.openstreetmap.fr/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
)
# CHECK ANY MIRROR YOU ADD AGAINST A KNOWN BOX BEFORE TRUSTING IT. Several of
# the ones the wiki lists are REGIONAL EXTRACTS that answer an out-of-area
# query with a valid, cheerful, empty result rather than an error —
# overpass.osm.ch returns `ways: 0` for Long Island and 200 OK with it, which
# would have shown up here as "this suburb has no buildings". kumi and
# private.coffee are the same host behind two names, so they fail together and
# only count as one mirror. overpass.osm.jp did not answer at all.


# NO SPACES IN THE USER-AGENT. openstreetmap.fr sits behind a filter that 403s
# any UA containing a space or a bracket, which is most of the polite forms the
# OSM wiki suggests -- "airstack-scene-gen/1.0 (research)" is refused and the
# same string minus the bracketed part is served. curl gets through because
# "curl/8.5.0" has no space either. This cost an hour: the 403 arrives in 0.3 s,
# so the mirror looks alive, answers curl by hand, and only fails under the
# tool.
UA = {"User-Agent": "airstack-scene-gen/1.0", "Accept": "*/*"}


def _overpass(query, timeout=180, retries=3, endpoints=None):
    """POST *query*, rotating mirrors and backing off. Same shape as the one in
    `osm_building_stats`, plus the rotation."""
    eps = list(endpoints or ENDPOINTS)
    data = urllib.parse.urlencode({"data": query}).encode()
    last = None
    for attempt in range(retries * len(eps)):
        ep = eps[attempt % len(eps)]
        try:
            req = urllib.request.Request(ep, data=data, headers=UA)
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode())
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError,
                json.JSONDecodeError) as e:
            last = f"{ep}: {e}"
            if attempt % len(eps) == len(eps) - 1:
                time.sleep(12 * (1 + attempt // len(eps)))   # all down: wait
    raise RuntimeError(f"overpass failed on every mirror; last {last}")

# Public roads only. Service ways (driveways, alleys, parking aisles) are the
# single biggest way to get a setback wrong, and they get worse exactly where
# this measurement matters most — see the module docstring.
ROAD_CLASSES = ("residential|living_street|unclassified|tertiary|tertiary_link|"
                "secondary|secondary_link|primary|primary_link|road")

# Tag values that positively confirm a single-family dwelling.
STRICT = {"house", "detached", "residential", "semidetached_house",
          "bungalow", "villa", "static_caravan"}
# Multi-dwelling or non-dwelling: never counted as one house.
MULTI = {"apartments", "terrace", "dormitory", "hotel", "commercial", "retail",
         "industrial", "warehouse", "office", "school", "church", "chapel",
         "civic", "public", "hospital", "supermarket", "kindergarten",
         "government", "sports_hall", "stadium", "train_station"}
# Dropped in the query itself, to keep the payload down.
NOT_A_BUILDING = ("garage|garages|carport|shed|hut|roof|greenhouse|barn|"
                  "stable|farm_auxiliary|silo|tank|container|cabin|boathouse|"
                  "shelter|canopy|bunker|ruins|construction")

# A dwelling footprint band. The floor drops detached garages and pool houses
# that were tagged building=yes; the ceiling drops schools and clubhouses that
# escaped MULTI. Both are deliberately generous — an estate house really can
# have a 700 m2 footprint, and clipping that would manufacture the very
# convergence this tool is trying to test.
MIN_DWELLING_M2 = 55.0
MAX_DWELLING_M2 = 1500.0

# A landuse=residential polygon holding exactly one dwelling and no bigger than
# this is treated as a real cadastral parcel. 10 acres is far above any of these
# towns' minimum lot sizes, so the cap only excludes farm and estate-grounds
# polygons that happen to hold a single house.
PARCEL_MAX_M2 = 10.0 * ACRE_M2
PARCEL_MIN_M2 = 150.0

# A landuse polygon smaller than this is too small for area/count to mean
# anything -- a 3-house sliver gives a ratio driven by where the mapper drew
# the edge, not by the plat.
TRACT_MIN_M2 = 1.0 * HA_M2


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
def _poly(geom, to_xy):
    """OSM way geometry -> shapely Polygon in local metres, or None."""
    from shapely.geometry import MultiPolygon, Polygon
    if not geom or len(geom) < 4:
        return None
    g = Polygon([to_xy(p["lat"], p["lon"]) for p in geom])
    if not g.is_valid:
        g = g.buffer(0)
    if isinstance(g, MultiPolygon):
        g = max(g.geoms, key=lambda p: p.area) if g.geoms else None
    if g is None or g.is_empty or g.area <= 0.0:
        return None
    return g


def _rect(g):
    """(short, long) of the minimum rotated rectangle of a shapely polygon."""
    return _min_rot_rect(list(g.exterior.coords))


def _fetch(center, dist, timeout, endpoints=None):
    south, west, north, east = _bbox(center, dist)
    bb = f"{south},{west},{north},{east}"
    land = _overpass(
        f'[out:json][timeout:{timeout}];'
        f'(way["landuse"="residential"]({bb});'
        f' way["highway"~"^({ROAD_CLASSES})$"]({bb});'
        f');out geom tags;', timeout=timeout, endpoints=endpoints)
    bld = _overpass(
        f'[out:json][timeout:{timeout}];'
        f'way["building"]["building"!~"^({NOT_A_BUILDING})$"]({bb});'
        f'out geom tags;', timeout=timeout, endpoints=endpoints)
    return land, bld


def measure(center, dist, timeout=240, endpoints=None):
    from shapely.geometry import LineString, box as _box
    from shapely.ops import unary_union
    from shapely.strtree import STRtree

    land, bld = _fetch(center, dist, timeout, endpoints)
    to_xy = _local_xy(center)
    # THE SAMPLE BOX AS GEOMETRY, and it is load-bearing. Overpass returns a
    # whole way if any part of it is in the bbox, so an unclipped
    # landuse=residential blob can be several times the box — Levittown's 1.96
    # km2 box came back with 8.8 km2 of landuse. The dwellings inside it are
    # clipped to the box and the polygon is not, so land-per-dwelling came out
    # 4.5x too high until this clip went in.
    box = _box(-dist, -dist, dist, dist)
    box_m2 = box.area

    landuse, roads = [], []
    for el in land.get("elements", []):
        t = el.get("tags", {})
        if t.get("landuse") == "residential":
            g = _poly(el.get("geometry"), to_xy)
            if g is not None:
                landuse.append(g)
        elif t.get("highway"):
            geom = el.get("geometry") or []
            if len(geom) >= 2:
                roads.append(LineString([to_xy(p["lat"], p["lon"])
                                         for p in geom]))

    n_buildings, dwell = 0, []
    for el in bld.get("elements", []):
        g = _poly(el.get("geometry"), to_xy)
        if g is None:
            continue
        n_buildings += 1
        bv = (el.get("tags", {}) or {}).get("building", "yes")
        if bv in MULTI:
            continue
        a = g.area
        if not (MIN_DWELLING_M2 <= a <= MAX_DWELLING_M2):
            continue
        mr = _rect(g)
        if not mr:
            continue
        dwell.append({"g": g, "c": g.centroid, "area": a,
                      "short": mr[0], "long": mr[1], "strict": bv in STRICT})

    st = {
        "box_km2": box_m2 / 1e6,
        "n_landuse": len(landuse),
        "landuse_m2": sum(g.area for g in landuse),
        "n_roads": len(roads),
        "road_km": sum(l.length for l in roads) / 1000.0,
        "n_buildings": n_buildings,
        "n_dwell": len(dwell),
        "n_strict": sum(1 for d in dwell if d["strict"]),
        "fp_area": [d["area"] for d in dwell],
        "fp_short": [d["short"] for d in dwell],
        "fp_long": [d["long"] for d in dwell],
        "per_dwelling": [], "union_m2": 0.0, "union_dwell": 0,
        "n_tracts": 0, "n_parcel_polys": 0, "n_orphan_polys": 0,
        "parcel_area": [], "parcel_short": [], "parcel_long": [],
        "nn": [], "setback": [],
    }

    # --- dwellings inside each residential landuse polygon ------------------
    if landuse and dwell:
        tree = STRtree([d["c"] for d in dwell])
        clipped = []
        for g in landuse:
            c = g.intersection(box)
            if c.is_empty or c.area <= 0.0:
                continue
            clipped.append(c)
            n = sum(1 for i in tree.query(c) if c.contains(dwell[i]["c"]))
            whole = c.area >= 0.98 * g.area       # not a chopped-off fragment
            if n >= 3 and c.area >= TRACT_MIN_M2:
                st["n_tracts"] += 1
                st["per_dwelling"].append(c.area / n)
            elif n == 1 and whole and PARCEL_MIN_M2 <= c.area <= PARCEL_MAX_M2:
                st["n_parcel_polys"] += 1
                mr = _rect(c) if c.geom_type == "Polygon" else None
                if mr:
                    st["parcel_area"].append(c.area)
                    st["parcel_short"].append(mr[0])
                    st["parcel_long"].append(mr[1])
            elif n == 0:
                st["n_orphan_polys"] += 1

        # POOLED figure over the UNION, not the sum: residential landuse
        # polygons overlap in OSM (a subdivision inside a neighbourhood inside
        # a place), and summing areas and counts double-counted 2522 dwellings
        # out of 1816 real ones in the first Levittown run.
        u = unary_union(clipped)
        if not u.is_empty:
            st["union_m2"] = u.area
            st["union_dwell"] = sum(1 for i in tree.query(u)
                                    if u.contains(dwell[i]["c"]))

    # --- nearest-neighbour house spacing ------------------------------------
    if len(dwell) >= 3:
        try:
            tree = STRtree([d["c"] for d in dwell])
            for d in dwell:
                idx = tree.query_nearest(d["c"], exclusive=True)
                best = None
                for i in (idx if hasattr(idx, "__len__") else [idx]):
                    v = d["c"].distance(dwell[int(i)]["c"])
                    if v > 0.0 and (best is None or v < best):
                        best = v
                if best is not None and best < 500.0:
                    st["nn"].append(best)
        except Exception as e:                       # noqa: BLE001
            st["nn_error"] = repr(e)

    # --- setback to the nearest public road centreline -----------------------
    if roads and dwell:
        try:
            tree = STRtree(roads)
            for d in dwell:
                idx = tree.query_nearest(d["g"])
                best = None
                for i in (idx if hasattr(idx, "__len__") else [idx]):
                    v = roads[int(i)].distance(d["g"])
                    if best is None or v < best:
                        best = v
                if best is not None and best < 300.0:
                    st["setback"].append(best)
        except Exception as e:                       # noqa: BLE001
            st["setback_error"] = repr(e)

    return st


def pool(sts):
    out = {}
    for st in sts:
        for k, v in st.items():
            if isinstance(v, list):
                out.setdefault(k, []).extend(v)
            elif isinstance(v, (int, float)):
                out[k] = out.get(k, 0) + v
    return out


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------
def _med(vals):
    if not vals:
        return None
    s = sorted(vals)
    return s[len(s) // 2]


def report(name, st):
    print(f"\n=== {name} — {st['box_km2']:.2f} km2 of sample box ===")

    lu_share = 100.0 * st["union_m2"] / max(1e-9, st["box_km2"] * 1e6)
    strict_pct = 100 * st["n_strict"] // max(1, st["n_dwell"])
    print("  coverage")
    print(f"    landuse=residential   {st['n_landuse']} polys; clipped union "
          f"{st['union_m2'] / HA_M2:.0f} ha = {lu_share:.0f}% of box, holding "
          f"{st['union_dwell']} of the {st['n_dwell']} dwellings")
    print(f"    public road centreline{st['road_km']:8.1f} km"
          f"  ({st['n_roads']} ways)")
    print(f"    buildings returned    {st['n_buildings']}")
    print(f"    -> dwellings          {st['n_dwell']}"
          f"   ({strict_pct}% carry a single-family building=* tag; the rest "
          f"are building=yes passing the {MIN_DWELLING_M2:.0f}-"
          f"{MAX_DWELLING_M2:.0f} m2 filter)")
    print(f"    landuse polys with 0 dwellings inside: {st['n_orphan_polys']}"
          f"   (unmapped buildings, or non-dwelling landuse)")
    # THE COMPLETENESS TELL, and it is the first line to read. Every other
    # figure assumes the buildings in the box are most of the buildings on the
    # ground, and where they are not, the failure is SILENT and it inflates
    # nearest-neighbour spacing — which is the headline number here.
    #
    # This figure does fall as lots widen; that is half the point of measuring
    # it. But it falls only in PROPORTION to frontage, so tripling the lot
    # width at most thirds it. Levittown returns ~50/km, so a two-acre town
    # should still return something in the teens. Saddle River NJ returned
    # 1.1/km — 46x down, which no zoning ordinance can do — and was dropped
    # from the estate sample for that reason. Below ~8, read it as "buildings
    # not mapped", not as "low density".
    per_km = st["n_dwell"] / max(1e-9, st["road_km"])
    flag = "" if per_km >= 8.0 else "   <-- SPARSE: buildings likely unmapped"
    print(f"    dwellings / km of road{per_km:8.1f}{flag}")

    print("  land per dwelling  [landuse polygon area / dwellings inside it]")
    if st["per_dwelling"]:
        print(f"    median            {_q(st['per_dwelling'], ' m2', 0)}"
              f"   over {st['n_tracts']} polygons")
        print(f"                      "
              f"{_q([a / ACRE_M2 for a in st['per_dwelling']], ' ac', 3)}")
        pooled = st["union_m2"] / max(1, st["union_dwell"])
        print(f"    pooled            {pooled:.0f} m2 = {pooled / ACRE_M2:.3f} ac"
              f"   ({st['union_m2'] / HA_M2:.0f} ha / {st['union_dwell']} dwellings)")
    else:
        print("    NOT OBTAINABLE — no landuse=residential polygon in this box "
              "held 3+ mapped dwellings")

    print("  individual parcels  [landuse polygon holding exactly 1 dwelling]")
    if st["parcel_area"]:
        print(f"    parcel area       {_q(st['parcel_area'], ' m2', 0)}")
        print(f"                      "
              f"{_q([a / ACRE_M2 for a in st['parcel_area']], ' ac', 3)}")
        print(f"    frontage (short)  {_q(st['parcel_short'], ' m')}")
        print(f"    depth (long)      {_q(st['parcel_long'], ' m')}")
    else:
        print(f"    NOT OBTAINABLE — {st['n_parcel_polys']} single-dwelling "
              "landuse polygons found; OSM has no cadastre here")

    print("  density")
    if st["union_dwell"]:
        per_ha = st["union_dwell"] / (st["union_m2"] / HA_M2)
        print(f"    dwellings / ha    {per_ha:.2f}"
              f"   (within residential landuse, n={st['union_dwell']})")
        print(f"    dwellings / km2   {per_ha * 100.0:.0f}")
    else:
        print("    NOT OBTAINABLE from landuse")
    gross = st["n_dwell"] / max(1e-9, st["box_km2"])
    print(f"    gross over box    {gross:.0f} / km2 = {gross / 100.0:.2f} / ha"
          f"   (includes roads, parks, everything)")

    print("  spacing")
    if st["nn"]:
        print(f"    nearest house     {_q(st['nn'], ' m')}"
              "   <- no landuse needed; ~ lot frontage along a street")
    else:
        print(f"    NOT OBTAINABLE ({st.get('nn_error', 'too few dwellings')})")

    print("  building footprint")
    print(f"    area              {_q(st['fp_area'], ' m2', 0)}")
    print(f"    short side        {_q(st['fp_short'], ' m')}")
    print(f"    long side         {_q(st['fp_long'], ' m')}")
    fp, land = _med(st["fp_area"]), _med(st["per_dwelling"])
    if fp and land:
        print(f"    coverage ratio    {100.0 * fp / land:.1f}% of the land "
              "under roof  (median/median)")

    print("  setback  [footprint to nearest PUBLIC road centreline; "
          "highway=service excluded]")
    if st["setback"]:
        print(f"    to centreline     {_q(st['setback'], ' m')}")
        m = _med(st["setback"])
        print("    NOTE: the generator's setback runs from the LOT FRONT LINE, "
              "which sits half a right-of-way inside the centreline.")
        print(f"    less half-ROW:    {m - 7.6:.1f} m at a 50 ft ROW, "
              f"{m - 9.1:.1f} m at 60 ft")
    else:
        print(f"    NOT OBTAINABLE ({st.get('setback_error', 'no roads')})")


RATIO_ROWS = [
    ("land per dwelling", lambda s: _med(s["per_dwelling"])),
    ("land per dwelling (pooled)",
     lambda s: (s["union_m2"] / s["union_dwell"]) if s["union_dwell"] else None),
    ("dwellings / ha",
     lambda s: (s["union_dwell"] / (s["union_m2"] / HA_M2))
     if s["union_m2"] else None),
    ("nearest-house spacing", lambda s: _med(s["nn"])),
    ("parcel frontage", lambda s: _med(s["parcel_short"])),
    ("parcel depth", lambda s: _med(s["parcel_long"])),
    ("parcel area", lambda s: _med(s["parcel_area"])),
    ("footprint area", lambda s: _med(s["fp_area"])),
    ("footprint long side", lambda s: _med(s["fp_long"])),
    ("setback to centreline", lambda s: _med(s["setback"])),
]


def ratios(a, b):
    print("\n" + "=" * 72)
    print("B (estate) / A (ordinary suburb) — this ratio IS the multiplier")
    print("=" * 72)
    print(f"  {'measure':<28}{'A suburb':>12}{'B estate':>12}{'B/A':>9}")
    for label, f in RATIO_ROWS:
        va, vb = f(a), f(b)
        if va is None or vb is None or va == 0:
            print(f"  {label:<28}{'n/a':>12}{'n/a':>12}{'—':>9}")
            continue
        print(f"  {label:<28}{va:>12.2f}{vb:>12.2f}{vb / va:>9.2f}x")

    # The generator scales a LINEAR frontage, so a land-area ratio has to be
    # square-rooted before it can be compared with `lot`. Both are printed
    # because they answer different questions: area is what zoning writes,
    # width is what the plat draws.
    la, lb = _med(a["per_dwelling"]), _med(b["per_dwelling"])
    if la and lb:
        print(f"\n  land ratio {lb / la:.2f}x  ->  sqrt = {math.sqrt(lb / la):.2f}x "
              "if the extra land were split evenly between width and depth")
    # The setback ratio is SENSITIVE to the half-ROW offset, because
    # subtracting a constant from both sides of a ratio moves it. Print the
    # sweep rather than one number, so nobody copies a false precision.
    sa, sb = _med(a["setback"]), _med(b["setback"])
    if sa and sb:
        print(f"  setback ratio {sb / sa:.2f}x measured to the centreline. "
              "Net of half a right-of-way:")
        for row_ft, row_m in ((40, 6.1), (50, 7.6), (60, 9.1), (66, 10.1)):
            na, nb = sa - row_m, sb - row_m
            if na > 0.5 and nb > 0.5:
                print(f"    {row_ft} ft ROW  A {na:5.1f} m   B {nb:5.1f} m   "
                      f"{nb / na:.2f}x")


# What `suburb_parcel.DENSITY["estate"]` currently carries. Copied rather than
# imported on purpose: this tool must not drag the generator's import graph in,
# and a stale copy here is harmless — it is only ever printed next to the
# measurement so the two can be compared by eye.
CURRENT_ESTATE = {"lot": 2.20, "setback": 1.60, "depth": 1.35, "w": 0.06}


def recommend(a, b):
    """Turn the ratios into the four numbers an `estate` class actually needs."""
    print("\n" + "=" * 72)
    print("WHAT AN `estate` CLASS SHOULD CARRY")
    print("=" * 72)

    nn_a, nn_b = _med(a["nn"]), _med(b["nn"])
    la, lb = _med(a["per_dwelling"]), _med(b["per_dwelling"])
    sa, sb = _med(a["setback"]), _med(b["setback"])

    # WIDTH comes from nearest-neighbour spacing, not from land area. Along a
    # street the nearest other house is the one next door, so this measures
    # frontage directly and needs no landuse polygon — which matters, because
    # landuse coverage is the weakest link in the estate sample.
    width = (nn_b / nn_a) if (nn_a and nn_b) else None

    # DEPTH is the residual. Land = width x depth, so once width is pinned by
    # spacing, whatever is left of the land ratio has to be depth. This is the
    # honest way round: measuring estate lot depth directly needs per-parcel
    # geometry, which US OSM mostly does not have.
    land = (lb / la) if (la and lb) else None
    depth = (land / width) if (land and width) else None

    # SETBACK, net of half a right-of-way, because the measurement is to the
    # centreline and the generator's is to the lot front line. 50 ft is the
    # common post-war residential ROW; the ratio's sensitivity to that choice
    # is printed above, and it is not small.
    row = 7.6
    setb = ((sb - row) / (sa - row)) if (sa and sb and sa > row and sb > row) else None

    # WEIGHT. "What share of a low-density district is estate-sized" is a
    # distribution question, not a median question: the estate sample is not
    # uniformly estate, and the ordinary sample is not uniformly ordinary.
    # Threshold at twice the ordinary median spacing and count both ways.
    share_a = share_b = None
    if nn_a:
        cut = 2.0 * nn_a
        if a["nn"]:
            share_a = sum(1 for v in a["nn"] if v >= cut) / len(a["nn"])
        if b["nn"]:
            share_b = sum(1 for v in b["nn"] if v >= cut) / len(b["nn"])

    rows = [("lot (width)", width, CURRENT_ESTATE["lot"]),
            ("setback", setb, CURRENT_ESTATE["setback"]),
            ("depth", depth, CURRENT_ESTATE["depth"]),
            ("weight in a low-density mix", share_b, CURRENT_ESTATE["w"])]
    print(f"  {'multiplier vs normal':<30}{'measured':>11}{'current':>10}"
          f"{'current is':>14}")
    for label, got, cur in rows:
        if got is None:
            print(f"  {label:<30}{'n/a':>11}{cur:>10.2f}{'—':>14}")
            continue
        verdict = ("about right" if 0.85 <= cur / got <= 1.18
                   else ("TOO HIGH" if cur > got else "TOO LOW"))
        print(f"  {label:<30}{got:>11.2f}{cur:>10.2f}{verdict:>14}")

    if share_a is not None:
        print(f"\n  {100 * share_a:.1f}% of ORDINARY-suburb dwellings already sit "
              f"at >= 2x the ordinary median spacing")
    if share_b is not None:
        print(f"  {100 * share_b:.1f}% of ESTATE-town dwellings do")
    print("\n  Reminder: these are RATIOS, so they transfer even though the "
          "generator's\n  `normal` lot is already wider in metres than a real "
          "one (its house assets\n  are oversized, which suburb_net.yaml "
          "documents).")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sample", default="both",
                    choices=("both", "suburb", "estate"))
    ap.add_argument("--place", default=None,
                    help="one place name from either dict")
    ap.add_argument("--dist", type=int, default=1200,
                    help="half-width of the sample box in metres")
    ap.add_argument("--timeout", type=int, default=240)
    ap.add_argument("--endpoint", action="append", default=None,
                    help="override the mirror list; repeatable")
    ap.add_argument("--sleep", type=float, default=6.0,
                    help="seconds between places, to stay under the rate limit")
    args = ap.parse_args()

    groups = []
    if args.place:
        src = SUBURBS if args.place in SUBURBS else ESTATES
        groups.append((args.place, {args.place: src[args.place]}))
    else:
        if args.sample in ("both", "suburb"):
            groups.append(("A  ORDINARY SUBURB", SUBURBS))
        if args.sample in ("both", "estate"):
            groups.append(("B  LARGE-LOT / ESTATE", ESTATES))

    pooled = {}
    for gname, places in groups:
        print(f"\n{'#' * 72}\n# {gname}\n{'#' * 72}")
        sts = []
        for name, center in places.items():
            try:
                st = measure(center, args.dist, args.timeout,
                             args.endpoint)
            except Exception as e:                   # noqa: BLE001
                print(f"\n=== {name}: FAILED — {e}")
                continue
            report(name, st)
            sts.append(st)
            if args.sleep:
                time.sleep(args.sleep)
        if len(sts) > 1:
            p = pool(sts)
            p["box_km2"] = sum(s["box_km2"] for s in sts)
            report(f"{gname} — POOLED", p)
            pooled[gname[0]] = p

    if "A" in pooled and "B" in pooled:
        ratios(pooled["A"], pooled["B"])
        recommend(pooled["A"], pooled["B"])


if __name__ == "__main__":
    main()
