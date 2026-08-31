#!/usr/bin/env python
"""catch_clip_probe — verify the plan-clipped catch/heap-floor plates on the
GAC/downtowncity fire path (`urban_fire._plate`, wired into `r_expose_interior`
and `r_fire_collapse`) never poke past the building's own footprint the way
the old `W x D` box plates did (user review, fire_dtc2, 2026-08-30, dtc
Building_11 F1: "the catch ... looks like it's coming outside the side walls
... it's irregular-shape façades + roof; you can't treat it like a cuboid").

Runs `gac_fire.burn_gac` end to end on a bare USD stage (no Kit, no Flow, no
physics — same as `tools/gac_burn_probe.py`), finds every authored `catch_*`
plate (`r_expose_interior`'s own catch floor AND `r_fire_collapse`'s heap
floor both register into `ctx["fit"]["slabs"]` under that same name prefix —
see `urban_fire._plate`'s docstring), and for each one prints:

  * storey, the plate's own polygon vertex count and area, and that area as
    a fraction of the mass's full `W x D` bbox area (a box-fallback plate,
    or a polygon plate on a rectangular building, both read close to the
    `(W - 2*WALL_INSET) x (D - 2*WALL_INSET)` fraction; a setback storey's
    polygon plate reads smaller)
  * HULL EXCURSION: the worst signed distance any plate vertex sits outside
    the CONVEX HULL of the merged mesh's own vertices in that storey's
    z-band (`gac_fire.prepare`'s own `lo_m`/`hi_m` window, mirrored here as
    LO_M/HI_M) -- independently computed with `scipy.spatial.ConvexHull`,
    not the hand-rolled monotone-chain hull `gac_fire.prepare` uses, so a
    bug shared between the two would not hide here. <= 0 (within a 0.05 m
    tolerance) means every plate vertex is inside or on the real hull.
  * NOTCH GAP: the worst distance from a plate-edge MIDPOINT to the nearest
    actual mesh vertex in the same z-band. A convex hull straight-lines
    across a concave notch, so a plate can still be built from the (already
    inset) hull polygon and clear the vertex-in-hull test above while one
    stretch of its perimeter hangs over a notch with no real wall under it
    -- this is the check for exactly that failure mode.

Also samples the `fireheap*` debris chips r_fire_collapse drops after
authoring the heap floor and reports how many, of a sample, sit with their
world AABB bottom above the plate's own top face and their xy centre inside
the plate's own polygon (or the box's, on the kit-frozen path) -- "the heap
still sits on the plate".

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/catch_clip_probe.py dtc:Building_11 F1"

A bare name means GreatAmericanCity, unchanged, exactly like `gac_burn_probe.py`.
"""
import math
import os
import random
import sys
import traceback

import numpy as np

SCENE_GEN = os.environ.get("CATCHCLIP_SCENE_GEN", "/isaac-sim/AirStack/scene_gen")
sys.dont_write_bytecode = True
sys.path.insert(0, SCENE_GEN)
from pxr import Usd, UsdGeom                                       # noqa: E402
from disaster import fracture, gac_fire as gf, urban_fire as uf    # noqa: E402
from detail import gac_storey_slice as gss                         # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "SM_Building_09"
level = sys.argv[2] if len(sys.argv) > 2 else "F6"

# THE SAME z-BAND `gac_fire.prepare`'s `_storey_footprints` measures with.
# That function is a closure local to `prepare` (not a module-level, importable
# constant -- see the edit-scope note in `gac_fire.prepare`'s own docstring),
# so this is kept in sync by hand; both default to (0.3, 2.5) as of this probe.
LO_M, HI_M = 0.3, 2.5
TOL_M = 0.05

print("[catch_clip] scene_gen root: {0}".format(SCENE_GEN))
print("[catch_clip] urban_fire module: {0}".format(uf.__file__))
print("[catch_clip] gac_fire module: {0}".format(gf.__file__))
fracture.ensure_vtk(verbose=False)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Scope.Define(st, "/W/bench")
cell = "/W/bench/g0"
UsdGeom.Xform.Define(st, cell)
mats = uf.materials(st, "/W/bench")
rng = random.Random(7)
nrng = np.random.default_rng(7)
try:
    ctx = gf.burn_gac(st, cell, name, level, rng, nrng, mats, "g0",
                      flow_root=None, mat_cache={}, ssf=1.0, verbose=True)
except Exception:
    traceback.print_exc()
    sys.exit(1)

for n in ctx["notes"]:
    print("   note:", n[:220])

m = ctx["gac"]["mass"]
footprints = ctx["fire"].get("footprints") or {}
print("\n[catch_clip] {0} storey footprint(s) measured by gac_fire.prepare "
      "(of {1} storeys)".format(len(footprints), len(m["levels"])))

# GROUND TRUTH, RE-READ OFF THE PLACED SOURCE. Nothing in the burn removes or
# deactivates `<cell>/src` -- only individual glazing GeomSubsets are ever
# SetActive(False) (`damage_windows`) -- so the merged mesh is still exactly
# what `gac_fire.prepare` measured against.
src = cell + "/src"
mesh = gss.read_mesh(st, src, verbose=False)
if mesh is None:
    print("[catch_clip] FATAL: could not re-read the merged mesh off {0}"
          .format(src))
    sys.exit(1)
kind, asset = gf.split_kind(name)
pack = gf.PACKS[kind]
mesh_bldg = gf.mesh_without_props(mesh, pack["bbox_exclude"])
_MP, _MT = mesh_bldg["P"], mesh_bldg["tris"]
Pb = _MP[np.unique(_MT)]


def band_xy(z):
    band = Pb[(Pb[:, 2] >= z + LO_M) & (Pb[:, 2] <= z + HI_M)]
    return band[:, :2]


def band_edges(z):
    """Every triangle EDGE (as an xy line segment), for every triangle with
    at least one vertex in the z-band. VERTICES ALONE UNDER-SAMPLE A LARGE
    FLAT PANEL: a CAD wall is a handful of big quads, so a point at the
    middle of a 10 m panel can sit >10 m from the nearest raw vertex while
    resting exactly on the panel's own surface in plan. An edge is a full
    line segment along that panel's boundary, so distance-to-nearest-edge
    stays small anywhere along a real wall, sparse tessellation or not."""
    zt = _MP[_MT][:, :, 2]
    in_band = ((zt >= z + LO_M) & (zt <= z + HI_M)).any(axis=1)
    sel = _MT[in_band]
    if not len(sel):
        return np.zeros((0, 2, 2))
    segs = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        pa = _MP[sel[:, a]][:, :2]
        pb = _MP[sel[:, b]][:, :2]
        segs.append(np.stack([pa, pb], axis=1))
    return np.concatenate(segs, axis=0)


def point_seg_dist(pt, segs):
    if len(segs) == 0:
        return float("inf")
    p = np.asarray(pt, dtype=float)
    a, b = segs[:, 0, :], segs[:, 1, :]
    ab = b - a
    ap = p - a
    denom = np.einsum("ij,ij->i", ab, ab)
    denom = np.where(denom < 1e-12, 1e-12, denom)
    t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0.0, 1.0)
    proj = a + ab * t[:, None]
    d = np.hypot(proj[:, 0] - p[0], proj[:, 1] - p[1])
    return float(d.min())


def convex_hull_xy(xy):
    """CCW hull vertices, via `scipy.spatial.ConvexHull` -- deliberately NOT
    `gac_fire.prepare`'s own monotone-chain hull, so this check is not just
    re-running the same algorithm against itself."""
    from scipy.spatial import ConvexHull
    if len(xy) < 3:
        return [tuple(p) for p in xy]
    try:
        h = ConvexHull(xy)
    except Exception:
        return [tuple(p) for p in xy]
    return [tuple(xy[i]) for i in h.vertices]     # CCW, per scipy's own docs


def poly_area(poly):
    n = len(poly)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        s += x0 * y1 - x1 * y0
    return 0.5 * abs(s)


MIN_EDGE_M = 0.10   # see `point_edge_excursion`'s own note


def point_edge_excursion(qx, qy, hull):
    """Signed distance of (qx, qy) outside the CCW convex polygon `hull` --
    the max, over every edge, of the point's distance beyond that edge's
    outward side. <= 0 means inside (or on) the hull.

    EDGES SHORTER THAN `MIN_EDGE_M` ARE SKIPPED. `scipy.spatial.ConvexHull`
    (used for the GROUND-TRUTH hull this is checked against, `convex_hull_xy`
    below) can keep a pair of near-duplicate raw mesh vertices a millimetre
    apart as two distinct hull vertices where the hand-rolled hull in
    `gac_fire.prepare` would round-and-merge them -- the tiny "edge" between
    them then has a direction dominated by mesh-tessellation noise, and
    treating it as an infinite half-plane (the correct test for a well-formed
    convex polygon) can cut off a chunk of the TRUE hull that has nothing to
    do with that noise (measured on SM_Building_09: a real ~0.15-0.3 m corner
    chamfer's tiny edges pulled the reported margin from the expected >=
    0.35 m down to 0.083 m with no actual violation -- confirmed separately
    with an even-odd point-in-polygon test). Skipping short edges only ever
    WIDENS the accepted region (a removed edge's constraint is redundant
    with its longer neighbours everywhere that matters), so this cannot hide
    a real excursion, only a noise-edge false alarm."""
    n = len(hull)
    worst = -1e9
    for i in range(n):
        x0, y0 = hull[i]
        x1, y1 = hull[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        el = math.hypot(dx, dy)
        if el < MIN_EDGE_M:
            continue
        nx, ny = dy / el, -dx / el          # outward normal, CCW polygon
        d = (qx - x0) * nx + (qy - y0) * ny
        worst = max(worst, d)
    return worst


def hull_excursion(poly, hull):
    return max(point_edge_excursion(qx, qy, hull) for qx, qy in poly)


def notch_gap(poly, segs):
    """Worst distance from a plate-EDGE MIDPOINT to the nearest actual mesh
    EDGE (line segment, `band_edges`) in the same z-band -- large means that
    stretch of the plate's own perimeter is bridging a concave notch with no
    real wall under it (a convex hull straight-lines across a notch; the
    vertex-in-hull check above cannot see this because the plate's OWN
    vertices sit on the hull, which is exactly the shape doing the
    bridging)."""
    n = len(poly)
    worst = 0.0
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        worst = max(worst, point_seg_dist(((x0 + x1) / 2.0, (y0 + y1) / 2.0),
                                          segs))
    return worst


def plate_top_xy(path):
    """(top-face xy polygon, world-frame plate top z, plate bottom z) off the
    authored mesh. `urban_fire._plate` always authors `pts = top + bot` (n
    top-face corners in polygon order, then n bottom-face corners); `qf._box`
    authors its own fixed 8 (4 bottom, THEN 4 top -- the opposite half order,
    for the box-fallback case). Either way the two halves are EQUAL LENGTH
    and each is a closed loop in ITS OWN AUTHORED ORDER -- which must be
    preserved (NOT re-sorted/deduped by coordinate: that scrambles the
    winding and silently corrupts both the shoelace area and every
    edge-midpoint this probe computes downstream -- caught empirically on
    dtc Building_11 F1, where a sort-based version of this function reported
    a 304 m2 / 11 m-notch-gap plate that was actually a fine, ~929 m2,
    no-notch one; see the excursion table in the final report)."""
    pr = st.GetPrimAtPath(path)
    if not pr or not pr.IsValid():
        return None, None, None
    me = UsdGeom.Mesh(pr)
    pts = me.GetPointsAttr().Get()
    if not pts or len(pts) < 6 or len(pts) % 2 != 0:
        return None, None, None
    xf = UsdGeom.XformCache()
    world = xf.GetLocalToWorldTransform(pr)
    wp = [world.Transform(p) for p in pts]
    n = len(wp) // 2
    first, second = wp[:n], wp[n:]
    z_first = sum(float(p[2]) for p in first) / n
    z_second = sum(float(p[2]) for p in second) / n
    if z_first >= z_second:
        top_half, z_top, z_bot = first, z_first, z_second
    else:
        top_half, z_top, z_bot = second, z_second, z_first
    poly = [(float(p[0]), float(p[1])) for p in top_half]
    return poly, z_top, z_bot


plates = []
for (mtag, storey), path in sorted(
        (ctx.get("fit", {}).get("slabs") or {}).items(),
        key=lambda kv: kv[0][1]):
    if not path or not path.rsplit("/", 1)[-1].startswith("catch_"):
        continue
    plates.append((storey, path))

print("\n[catch_clip] {0} catch/heap plate(s) found under ctx['fit']['slabs']"
      .format(len(plates)))
W_area = m["W"] * m["D"]
print("{0:>7} {1:>6} {2:>10} {3:>9} {4:>13} {5:>11}"
     .format("storey", "n_vtx", "area_m2", "area/WD", "hull_excurs",
             "notch_gap"))
worst_hull_exc, worst_notch = -1e9, 0.0
for storey, path in plates:
    poly, z_top, z_bot = plate_top_xy(path)
    if poly is None or len(poly) < 3:
        print("  storey {0}: could not read plate geometry at {1}"
              .format(storey, path))
        continue
    z = m["levels"][storey]
    band_pts = band_xy(z)
    if len(band_pts) < 3:
        print("{0:>7} {1:>6} {2:>10.1f} {3:>9} {4:>13} {5:>11}  (too few mesh "
              "points in the z-band to check against)"
              .format(storey, len(poly), poly_area(poly), "-", "-", "-"))
        continue
    hull = convex_hull_xy(band_pts)
    exc = hull_excursion(poly, hull)
    gap = notch_gap(poly, band_edges(z))
    worst_hull_exc = max(worst_hull_exc, exc)
    worst_notch = max(worst_notch, gap)
    area = poly_area(poly)
    print("{0:>7} {1:>6} {2:>10.1f} {3:>9.2f} {4:>13.3f} {5:>11.3f}"
         .format(storey, len(poly), area,
                 area / W_area if W_area else 0.0, exc, gap))
    used_footprint = footprints.get(storey) is not None
    print("        (from measured footprint: {0})".format(used_footprint))

if plates:
    print("\n[catch_clip] worst hull excursion across all plates: {0:.3f} m "
         "({1})".format(worst_hull_exc,
                        "PASS" if worst_hull_exc <= TOL_M else "FAIL"))
    print("[catch_clip] worst notch-bridging gap across all plates: {0:.3f} m"
         .format(worst_notch))
else:
    print("\n[catch_clip] no catch_/heap-floor plate was authored for this "
         "(name, level) -- nothing to check")

# THE HEAP MUST STILL SIT ON THE PLATE. `r_fire_collapse` sets
# `ctx["collapse_s0"]` to its own `s0`; the heap floor it authors (if any)
# sits at storey `bs = max(0, s0 - 1)` -- find THAT plate specifically
# (`plates` also holds `r_expose_interior`'s unrelated catch floor(s)),
# then sample `fireheap*` chips and check each one's xy centre against the
# plate's own polygon (a chip already passed the box-based off-plate cull
# `r_fire_collapse` still runs unchanged -- see that function's own
# `abs(lx) > W_/2 - 0.15` clip -- so a violation here means the polygon
# plate is SMALLER than that box in the notch/setback sense: a chip the old
# box-based clip correctly kept has no floor under it any more).
heap_plate = None
s0 = ctx.get("collapse_s0")
if s0 is not None:
    bs = max(0, int(s0) - 1)
    for storey, path in plates:
        if storey == bs:
            poly, z_top, z_bot = plate_top_xy(path)
            if poly is not None:
                heap_plate = (storey, poly, z_top)
            break
if heap_plate is not None:
    hp_storey, hp_poly, hp_ztop = heap_plate
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    chips = []
    parent = st.GetPrimAtPath(ctx["parent"])
    for p in (parent.GetChildren() if parent and parent.IsValid() else []):
        if p.GetName().startswith("fireheap") and p.IsActive():
            chips.append(p)
    sample = chips[:40]
    n_checked, n_below_floor, n_outside = 0, 0, 0
    for p in sample:
        box = bc.ComputeWorldBound(p).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        n_checked += 1
        lo_pt, hi_pt = box.GetMin(), box.GetMax()
        cx = (lo_pt[0] + hi_pt[0]) / 2.0
        cy = (lo_pt[1] + hi_pt[1]) / 2.0
        z_bottom = lo_pt[2]
        if z_bottom < hp_ztop - 1.0:      # fell noticeably below the plate top
            n_below_floor += 1
        if point_edge_excursion(cx, cy, hp_poly) > TOL_M:
            n_outside += 1
    print("\n[catch_clip] heap floor at storey {0}: sampled {1}/{2} "
         "'fireheap*' chip(s); {3} sit noticeably below the plate top, "
         "{4} sit outside the plate's own xy polygon (both should be 0)"
         .format(hp_storey, n_checked, len(chips), n_below_floor, n_outside))
else:
    print("\n[catch_clip] no heap-floor plate found (no fire collapse on "
         "this level, or nothing needed a floor authored under it)")
