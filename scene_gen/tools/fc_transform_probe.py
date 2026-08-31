#!/usr/bin/env python3
"""fc_transform_probe — prove the CITY-cell holder transform, offline.

    docker exec isaac-sim bash -c \\
      "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
       /isaac-sim/AirStack/scene_gen/tools/fc_transform_probe.py"

BARE USD. No `SimulationApp`, no Kit, no GPU — safe beside a running sim
(memory: "Nucleus USD without Kit"). It builds a tiny in-memory stage,
references ONE existing bake under exactly the holder
`urban_fire_city_launch_script.py` authors, and measures what composed.

WHY THIS EXISTS — THE TRANSFORM TRAP (`urban_fire_city_plan.md` sec 4a).
`quake.assemble`'s "keep the placement's transform, swap the reference" is
WRONG for a fire bake. `scene_generator.apply_placements` authors
`translate = (x_m - rotated_centroid_offset) * ssf`,
`rotateXYZ(roll, pitch, yaw)` (roll 90 deg for a Y-up asset) and
`scale = p["scale"]` (0.01 for a GAC asset) — all of that is about the
INTACT asset's own units, up-axis and pivot. A bake is none of those things:
it is already in METRES, Z-up, plan-centred on its own origin with its base
at z = 0. Inheriting the cell's transform would scale it by 0.01 and lay it
on its side. So the launcher authors a FRESH holder instead —
`translate = (x, y, z) * ssf`, `rotateXYZ = (0, 0, yaw_deg)`, `scale = 1`,
the bake referenced onto a CHILD of it — and hides the intact prim.

This probe asserts, on a real bake:

  1. THE TRANSFORM LAW. The composed world bbox centre equals
     `R(yaw) . own_centre + (x, y)` and the composed height is unchanged —
     i.e. the holder does exactly what the launcher's arithmetic assumes,
     with no hidden scale or up-axis correction anywhere in the arc.
  2. THE SHELL LANDS ON THE CELL. The plan (sec 4a) states this as "the
     composed bbox centre lands at (x, y) +- 1 m". MEASURED, ON A REAL
     BAKE, THAT IS THE WRONG INVARIANT and it fails by design:
     `gac_SM_Building_02_F3_s69` has its `masses["main"]` centre at
     (0.000, 0.000) — the SHELL is plan-centred to floating-point — while
     its bbox centre is +1.636 m in x, because the bbox also contains the
     DEBRIS the F3 fire dropped out of its east elevation
     (`fire["sides"] == ["E"]`; bbox x runs -13.99 .. +17.26 against a
     27.98 m shell, so 3.27 m of spall lies east of the east face).
     Re-centring on the bbox would shove the building 1.6 m WEST off its
     own footprint and drag the rubble back onto the plot. So the
     authoritative check is the SHELL: the sidecar's mass centre, rotated
     and translated, must land on the cell. The bbox-centre offset is
     reported beside it as the debris spill, and only warns.
  3. THE FOOTPRINT SWAP. At yaw 90 deg the composed footprint is the bake's
     own W x D with W and D exchanged (to the millimetre) — the check that
     the rotation is about Z and about the holder's own origin, not about
     some inherited pivot.

Usage:
    fc_transform_probe.py [BAKE.usd] [--x 100] [--y 50] [--yaw 90]
                          [--tol 1.0]
With no BAKE it searches `/isaac-sim/.cache/fire_bakes*` (newest small `.usd`
first — a 3 MB bake opens in a second, a 40 MB one does not) and says which
directories it looked in.

Exit 0 clean, 1 on any failed assertion, 2 when no bake could be found.
"""

import argparse
import glob
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom

#: where a row bake and a city bake land (`fire_bake.DEFAULT_OUT_DIR` and
#: `fire_city_bake.sh`'s `$FB_OUT/city_<seed>/`), container paths.
SEARCH_GLOBS = ("/isaac-sim/.cache/fire_bakes/*.usd",
                "/isaac-sim/.cache/fire_bakes/city_*/*.usd",
                "/isaac-sim/.cache/fire_bakes_*/*.usd")


def find_bake():
    """The smallest bake on disk (fastest to open), or `None`."""
    found = []
    for pat in SEARCH_GLOBS:
        for p in glob.glob(pat):
            try:
                found.append((os.path.getsize(p), p))
            except OSError:
                pass
    if not found:
        return None
    found.sort()
    return found[0][1]


def world_bbox(stage, path):
    """`[x0, y0, z0, x1, y1, z1]` or `None`. `useExtentsHint=False` — an
    extents hint is authored data and can be stale (`fix-floating-debris`)."""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return [float(lo[0]), float(lo[1]), float(lo[2]),
            float(hi[0]), float(hi[1]), float(hi[2])]


def read_sidecar(usd):
    """The bake's `.json` beside it, or `None` — plain `json.load`, never
    `fire_bake.read_sidecar`, so this probe keeps its only import `pxr`."""
    path = os.path.splitext(usd)[0] + ".json"
    if not os.path.exists(path):
        return None
    try:
        import json
        with open(path) as fh:
            return json.load(fh)
    except Exception as exc:
        print("[fc_probe] sidecar {0} unreadable ({1})".format(path, exc))
        return None


def measure_alone(usd):
    """The bake's own bbox, opened standalone with no holder at all."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    kid = stage.DefinePrim(Sdf.Path("/World/alone"))
    if not kid.GetReferences().AddReference(usd):
        raise RuntimeError("could not reference {0}".format(usd))
    kid.Load()
    return world_bbox(stage, "/World/alone")


def compose_under_holder(usd, x, y, z, yaw_deg, stem="probe"):
    """EXACTLY the launcher's idiom: a fresh Xform holder carrying
    translate / rotateXYZ(0,0,yaw) / scale 1, with the bake referenced onto
    a CHILD of it (never onto the Xform itself — the same "reference onto a
    child" rule `gac_fire.place_source`, `kit_bake.load_kit` and the row
    assembler all follow, so a referenced asset's own transform can never
    compose with, or clobber, the one the caller authored)."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/fire"))
    holder = "/World/fire/{0}".format(stem)
    xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    kid = stage.DefinePrim(Sdf.Path(holder + "/bake"))
    if not kid.GetReferences().AddReference(usd):
        raise RuntimeError("could not reference {0}".format(usd))
    kid.Load()
    return stage, holder


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("bake", nargs="?", default=None)
    ap.add_argument("--x", type=float, default=100.0)
    ap.add_argument("--y", type=float, default=50.0)
    ap.add_argument("--z", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=90.0)
    ap.add_argument("--tol", type=float, default=1.0,
                    help="metres of slack on the centre check (default 1)")
    args = ap.parse_args(argv)

    usd = args.bake or find_bake()
    if not usd or not os.path.exists(usd):
        print("[fc_probe] no bake found; looked in:")
        for pat in SEARCH_GLOBS:
            print("    " + pat)
        return 2
    print("[fc_probe] bake: {0}  ({1:.1f} MB)".format(
        usd, os.path.getsize(usd) / 1e6))

    own = measure_alone(usd)
    if not own:
        print("[fc_probe] FAIL: the bake composed an EMPTY bbox on its own")
        return 1
    ow, od, oh = own[3] - own[0], own[4] - own[1], own[5] - own[2]
    ocx, ocy = 0.5 * (own[0] + own[3]), 0.5 * (own[1] + own[4])
    print("[fc_probe] alone:    W x D x H = {0:.3f} x {1:.3f} x {2:.3f} m, "
          "centre ({3:+.3f}, {4:+.3f}), z {5:.3f}..{6:.3f}".format(
              ow, od, oh, ocx, ocy, own[2], own[5]))

    stage, holder = compose_under_holder(usd, args.x, args.y, args.z, args.yaw)
    got = world_bbox(stage, holder)
    if not got:
        print("[fc_probe] FAIL: the holder composed an EMPTY bbox — the "
              "reference did not compose")
        return 1
    gw, gd, gh = got[3] - got[0], got[4] - got[1], got[5] - got[2]
    gcx, gcy = 0.5 * (got[0] + got[3]), 0.5 * (got[1] + got[4])
    print("[fc_probe] composed: W x D x H = {0:.3f} x {1:.3f} x {2:.3f} m, "
          "centre ({3:+.3f}, {4:+.3f}), z {5:.3f}..{6:.3f}".format(
              gw, gd, gh, gcx, gcy, got[2], got[5]))

    bad = []
    th = math.radians(args.yaw)
    ca, sa = math.cos(th), math.sin(th)

    # 1) THE TRANSFORM LAW: composed centre == R(yaw) . own_centre + (x, y)
    ex = ocx * ca - ocy * sa + args.x
    ey = ocx * sa + ocy * ca + args.y
    d_law = math.hypot(gcx - ex, gcy - ey)
    d_z = abs(got[2] - (own[2] + args.z))
    ok = d_law <= 0.05 and d_z <= 0.01
    print("[fc_probe] {0} transform law: expected centre ({1:+.3f}, {2:+.3f}), "
          "got ({3:+.3f}, {4:+.3f}), error {5:.4f} m; base z {6:.4f} vs "
          "{7:.4f} (error {8:.4f} m)".format(
              "PASS" if ok else "FAIL", ex, ey, gcx, gcy, d_law,
              got[2], own[2] + args.z, d_z))
    if not ok:
        bad.append("the composed pose is not R(yaw).own + (x, y, z) — "
                   "something in the arc (a scale, an up-axis roll, an "
                   "inherited pivot) is not what the launcher assumes")

    # 2) THE SHELL LANDS ON THE CELL — measured on the sidecar's own mass
    #    centre (the STRUCTURE), never on the bbox (which also contains the
    #    debris the fire dropped into the street). See the module docstring.
    doc = read_sidecar(usd)
    mcx = mcy = None
    if doc:
        masses = doc.get("masses") or {}
        key = ((doc.get("fire") or {}).get("mass") or "main")
        m = masses.get(key) or (list(masses.values())[0] if masses else None)
        if m is not None:
            mcx, mcy = float(m.get("cx", 0.0)), float(m.get("cy", 0.0))
    if mcx is None:
        d_cell = math.hypot(gcx - args.x, gcy - args.y)
        ok = d_cell <= args.tol
        print("[fc_probe] {0} cell centre (NO SIDECAR — falling back to the "
              "bbox, which includes debris): ({1:+.3f}, {2:+.3f}) vs "
              "({3:+.1f}, {4:+.1f}) — {5:.3f} m (tol {6:.1f})".format(
                  "PASS" if ok else "FAIL", gcx, gcy, args.x, args.y,
                  d_cell, args.tol))
        if not ok:
            bad.append("with no sidecar there is no shell centre to check "
                       "against, and the bbox centre is {0:.2f} m off the "
                       "cell".format(d_cell))
    else:
        sx = mcx * ca - mcy * sa + args.x
        sy = mcx * sa + mcy * ca + args.y
        d_cell = math.hypot(sx - args.x, sy - args.y)
        ok = d_cell <= args.tol
        print("[fc_probe] {0} shell on the cell: sidecar mass centre "
              "({1:+.4f}, {2:+.4f}) m -> composed ({3:+.3f}, {4:+.3f}) vs "
              "cell ({5:+.1f}, {6:+.1f}) — {7:.4f} m (tol {8:.1f})".format(
                  "PASS" if ok else "FAIL", mcx, mcy, sx, sy,
                  args.x, args.y, d_cell, args.tol))
        if not ok:
            bad.append("the bake's SHELL is {0:.2f} m off its own origin, so "
                       "the holder does not land it on the cell — correct by "
                       "the sidecar (plan sec 4a's `trim_bbox` note) before "
                       "trusting the cell position".format(d_cell))
        # the debris spill, reported not asserted
        spill = math.hypot(ocx - mcx, ocy - mcy)
        print("[fc_probe] INFO debris spill: the bbox centre is {0:+.3f} m "
              "east/{1:+.3f} m north of the shell centre ({2:.2f} m, "
              "{3:.0f}% of the {4:.1f} m footprint) — the rubble the fire "
              "dropped, and the reason the bbox centre is NOT the thing to "
              "seat on the cell".format(
                  ocx - mcx, ocy - mcy, spill,
                  100.0 * spill / max(1e-6, max(ow, od)), max(ow, od)))
        if spill > 0.5 * max(ow, od):
            print("[fc_probe] WARNING: that spill is over half the footprint "
                  "— check the bake for airborne debris before assembling it")

    # 3) THE FOOTPRINT SWAP at yaw 90: W and D exchange, H is untouched
    swapped = abs(((args.yaw / 90.0) % 2.0) - 1.0) < 1e-6      # 90 or 270 deg
    exp_w, exp_d = (od, ow) if swapped else (ow, od)
    e_w, e_d, e_h = abs(gw - exp_w), abs(gd - exp_d), abs(gh - oh)
    ok = max(e_w, e_d, e_h) <= 0.01
    print("[fc_probe] {0} footprint: expected {1:.3f} x {2:.3f} x {3:.3f} m "
          "({4}), got {5:.3f} x {6:.3f} x {7:.3f} (err {8:.4f} / {9:.4f} / "
          "{10:.4f} m)".format(
              "PASS" if ok else "FAIL", exp_w, exp_d, oh,
              "W/D SWAPPED by the yaw" if swapped else "unswapped",
              gw, gd, gh, e_w, e_d, e_h))
    if not ok:
        bad.append("the footprint did not rotate as a pure yaw about Z")

    print()
    if bad:
        print("FC TRANSFORM PROBE *** PROBLEM ***")
        for b in bad:
            print("  - " + b)
        return 1
    print("FC TRANSFORM PROBE OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
