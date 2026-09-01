#!/usr/bin/env python
"""tornado_street_seat_probe -- the C1 street cell on a BARE Usd stage, with
every posed prop's world min-z measured from MESH POINTS.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \\
        bash scene_gen/tools/usd_python.sh \\
        scene_gen/tools/tornado_street_seat_probe.py [SEED]"

No Kit, no `SimulationApp`, no viewport -- safe beside a running sim. It
rebuilds `urban_tornado_bench_launch_script.build_c1_cell`'s placements
verbatim (same pools, same resolver, same rng stream), runs
`tornado_street.plan_street` + `apply_street`, and prints a per-prop table:

    path  category  action/pose  world-min-z BEFORE  AFTER  seat gap

`ground z` for C1 is 0 (every bench holder sits at z=0). A NEGATIVE gap is
a sunk prop, a POSITIVE gap is a floater. The whole point of the points
path is that `UsdGeom.BBoxCache` LIES about a rotated prim (the
fix-floating-debris lesson), so both numbers are printed side by side: the
bbox column is the datum `tornado.toss_prim` uses internally, the points
column is the truth.

Env:
    UTS_SEED       int, default 7 (the bench's own UTB_SEED)
    UTS_MIN_TIPPED int, default 2 (the bench's own floor)
    UTS_ASSET_CONFIG  preset name, default downtown_tornado_bench_500
"""
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SCENE_GEN)

from pxr import Gf, Sdf, Usd, UsdGeom                            # noqa: E402

import scene_generator as sg                                     # noqa: E402
from suburb_scene import AssetPools, _raw_pool                    # noqa: E402
from compile_disaster import load_scene_config                   # noqa: E402
from disaster import tornado as tn                                # noqa: E402
from disaster import tornado_street as ts                         # noqa: E402

SEED = int(os.environ.get("UTS_SEED") or 7)
MIN_TIPPED = int(os.environ.get("UTS_MIN_TIPPED") or 2)
MIN_MOVED = int(os.environ.get("UTS_MIN_MOVED") or 0)
TREE_POOL = os.environ.get("UTS_TREE_POOL") or "trees"
ASSET_CONFIG = os.environ.get("UTS_ASSET_CONFIG") or \
    "downtown_tornado_bench_500"

PARENT = "/World/tornado_bench"
GRID_PITCH = 60.0
CELL_ORDER = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4", "B5",
              "C1", "C2"]
C_X, C_Y = -30.0, -70.0
LENGTH_M = 100.0
INTENSITY = 0.85


def _seed_for(cell_id):
    return SEED * 1000003 + CELL_ORDER.index(cell_id)


def _synth_wind_cfg():
    cfg = dict(tn.DEFAULTS)
    cfg.update({"origin_m": [0.0, 60.0], "heading_deg": 35.0,
                "width_m": 300.0, "wobble_m": 0.0, "edge_noise_m": 0.0,
                "along_min": 1.0, "width_min": 1.0})
    return cfg


WIND_CFG = _synth_wind_cfg()
WIND = tn.wind_at(WIND_CFG, 0.0, 0.0)


# ---------------------------------------------------------------------------
# measurement
# ---------------------------------------------------------------------------

def _iter_gprims(stage, root):
    prim = stage.GetPrimAtPath(Sdf.Path(root))
    if not prim or not prim.IsValid():
        return
    for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(
            Usd.PrimIsActive & Usd.PrimIsDefined)):
        yield p


def points_min_z(stage, root, xcache=None):
    """World-space minimum z over every POINT of every mesh/curve/points
    prim under *root*, in STAGE units. `None` if nothing carries points."""
    xcache = xcache or UsdGeom.XformCache(Usd.TimeCode.Default())
    best = None
    for p in _iter_gprims(stage, root):
        pts_attr = None
        if p.IsA(UsdGeom.Mesh) or p.IsA(UsdGeom.Points) or \
                p.IsA(UsdGeom.BasisCurves) or p.IsA(UsdGeom.NurbsCurves):
            pts_attr = UsdGeom.PointBased(p).GetPointsAttr()
        if pts_attr is None or not pts_attr.HasAuthoredValue():
            continue
        pts = pts_attr.Get()
        if not pts:
            continue
        m = xcache.GetLocalToWorldTransform(p)
        for v in pts:
            z = (float(v[0]) * m[0][2] + float(v[1]) * m[1][2] +
                 float(v[2]) * m[2][2] + m[3][2])
            if best is None or z < best:
                best = z
    # PointInstancers (tree crowns) -- prototype bound per instance is the
    # cheapest honest floor; report it too so a crown that dips under grade
    # is not invisible to this probe.
    for p in _iter_gprims(stage, root):
        if not p.IsA(UsdGeom.PointInstancer):
            continue
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_,
                                UsdGeom.Tokens.render])
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        if not r.IsEmpty():
            z = float(r.GetMin()[2])
            if best is None or z < best:
                best = z
    return best


def bbox_min_z(stage, root):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    prim = stage.GetPrimAtPath(Sdf.Path(root))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    return None if r.IsEmpty() else float(r.GetMin()[2])


# ---------------------------------------------------------------------------
# the cell (a verbatim copy of build_c1_cell's placement half)
# ---------------------------------------------------------------------------

def _prop_placement(pools, resolver, usd, x, y, yaw, category="prop"):
    sc = pools.scale_of(usd)
    au = pools.axis_of(usd)
    fp = resolver.get(usd, category, scale=sc, axis_up=au)
    return {"usd": usd, "x_m": x, "y_m": y, "z_m": float(fp.get("base", 0.0)),
            "yaw_deg": float(yaw) + pools.yaw_of(usd),
            "roll_deg": pools.roll_of(usd), "pitch_deg": 0.0, "scale": sc,
            "category": category, "axis_up": au}


def main():
    print("[seat] preset {0}, seed {1}, min_tipped {2}".format(
        ASSET_CONFIG, SEED, MIN_TIPPED))
    config = load_scene_config(ASSET_CONFIG)
    resolver = sg._make_resolver(config)
    pools = AssetPools(config)

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    ssf = 1.0
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Xform.Define(stage, Sdf.Path(PARENT))

    holder = PARENT + "/C1"
    xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(C_X * ssf, C_Y * ssf, 0.0))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    xf.AddScaleOp().Set(Gf.Vec3f(ssf, ssf, ssf))
    cell = holder + "/cell"
    UsdGeom.Xform.Define(stage, Sdf.Path(cell))

    rng = random.Random(_seed_for("C1"))
    lights = pools.load(_raw_pool(config, "streetlights"))
    bins_ = pools.load(_raw_pool(config, "trash_cans"))
    signals = pools.load(_raw_pool(config, "traffic_lights"))
    trees = pools.load(_raw_pool(config, TREE_POOL))
    cars = pools.load(_raw_pool(config, "cars"))
    for nm, pool in (("streetlights", lights), ("trash_cans", bins_),
                     ("traffic_lights", signals), (TREE_POOL, trees),
                     ("cars", cars)):
        print("[seat] pool {0:<15} {1} entr(y/ies)".format(nm, len(pool)))
        for u in pool[:6]:
            print("           {0}".format(os.path.basename(str(u))))

    placements = []
    for i, sx in enumerate((-0.35, -0.12, 0.12, 0.35)):
        if not lights:
            break
        x = sx * LENGTH_M
        side = 5.0 if i % 2 == 0 else -5.0
        placements.append(_prop_placement(
            pools, resolver, lights[i % len(lights)], x, side,
            rng.uniform(0.0, 360.0), category="streetlight"))
    if signals:
        placements.append(_prop_placement(
            pools, resolver, signals[0], -0.42 * LENGTH_M, 6.0,
            rng.uniform(0.0, 360.0), category="traffic_light"))
    for i, sx in enumerate((-0.25, 0.05, 0.30)):
        if not bins_:
            break
        placements.append(_prop_placement(
            pools, resolver, bins_[i % len(bins_)], sx * LENGTH_M,
            -5.5 if i % 2 else 5.5, rng.uniform(0.0, 360.0),
            category="trash_can"))
    for i, sx in enumerate((-0.30, 0.0, 0.30)):
        if not trees:
            break
        placements.append(_prop_placement(
            pools, resolver, trees[i % len(trees)], sx * LENGTH_M,
            7.5 if i % 2 == 0 else -7.5, rng.uniform(0.0, 360.0),
            category="street_tree"))
    car_placements = []
    if cars:
        for i, (sx, sy) in enumerate(((-0.30, 3.0), (-0.05, -3.5),
                                      (0.20, 3.5))):
            usd_c = cars[i % len(cars)]
            p = _prop_placement(pools, resolver, usd_c, sx * LENGTH_M, sy,
                                rng.uniform(0.0, 360.0), category="car")
            placements.append(p)
            car_placements.append(p)

    sg.apply_placements(stage, placements, cell + "/furniture", ssf,
                        resolver=resolver, instance_categories=set())

    before = {}
    for p in placements:
        path = p["prim_path"]
        before[path] = (points_min_z(stage, path), bbox_min_z(stage, path))

    # ---- the DATUM dump: what apply_placements actually authored --------
    print("")
    print("[datum] per-placement: fp(base/cz), authored ops, local bbox")
    seen = set()
    for p in placements:
        path = p["prim_path"]
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
        opsdesc = ", ".join("{0}={1}".format(o.GetOpName().split(":")[-1],
                                             o.Get()) for o in ops)
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_,
                                UsdGeom.Tokens.render])
        r = bc.ComputeUntransformedBound(prim).ComputeAlignedRange()
        loc = ("empty" if r.IsEmpty() else
               "min=({0:.3f},{1:.3f},{2:.3f}) max=({3:.3f},{4:.3f},{5:.3f})"
               .format(r.GetMin()[0], r.GetMin()[1], r.GetMin()[2],
                       r.GetMax()[0], r.GetMax()[1], r.GetMax()[2]))
        bcd = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                [UsdGeom.Tokens.default_])
        rd = bcd.ComputeUntransformedBound(prim).ComputeAlignedRange()
        locd = ("empty" if rd.IsEmpty() else
                "min_z={0:.3f} max_z={1:.3f}".format(rd.GetMin()[2],
                                                     rd.GetMax()[2]))
        key = (p["usd"], p["category"])
        fp = resolver.get(p["usd"], p["category"],
                          scale=pools.scale_of(p["usd"]),
                          axis_up=pools.axis_of(p["usd"]))
        print("  {0:<24} {1}".format(path.rsplit("/", 1)[-1][:24],
                                     os.path.basename(str(p["usd"]))))
        print("      scale={0} axis_up={1} yaw_off={2} roll={3}".format(
            pools.scale_of(p["usd"]), pools.axis_of(p["usd"]),
            pools.yaw_of(p["usd"]), p["roll_deg"]))
        print("      fp base={0:.4f} cx={1:.4f} cy={2:.4f} cz={3:.4f} "
              "sx={4:.3f} sy={5:.3f} sz={6:.3f}".format(
                  fp["base"], fp["cx"], fp["cy"], fp["cz"], fp["sx"],
                  fp["sy"], fp["sz"]))
        print("      ops: {0}".format(opsdesc))
        print("      local(default+render) {0}".format(loc))
        print("      local(default ONLY)   {0}".format(locd))
        if key not in seen:
            seen.add(key)
    print("")

    def _intensity_fn(_x, _y):
        return INTENSITY

    def _wind_at_fn(_x, _y):
        return WIND

    actions = ts.plan_street(placements, _intensity_fn, _wind_at_fn, rng,
                             min_tipped=MIN_TIPPED, min_moved=MIN_MOVED)
    for a in actions:
        print("[plan] {0}".format(
            {k: (round(v, 3) if isinstance(v, float) else v)
             for k, v in a.items()}))
    counts = ts.apply_street(stage, actions, min_tipped=MIN_TIPPED,
                             min_moved=MIN_MOVED, verbose=True)
    print("")
    print("[post] authored ops AFTER apply_street")
    for p in placements:
        prim = stage.GetPrimAtPath(Sdf.Path(p["prim_path"]))
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
        print("  {0:<24} {1}".format(
            p["prim_path"].rsplit("/", 1)[-1][:24],
            ", ".join("{0}={1}".format(o.GetOpName().split(":")[-1], o.Get())
                      for o in ops)))
    print("")

    by_path = {a.get("path"): a for a in actions}
    print("")
    print("{0:<26} {1:<13} {2:<20} {3:>8} {4:>8} {5:>8}  {6:<22} {7}".format(
        "prim", "category", "action/pose", "pts_pre", "pts_post", "GAP_m",
        "posed WxDxH (m)", "fall bearing"))
    print("-" * 132)
    n_bad = 0
    for p in placements:
        path = p["prim_path"]
        a = by_path.get(path)
        act = "-" if a is None else str(a.get("action"))
        if a is not None and a.get("action") == "car_pose":
            act = "car:" + str(a.get("pose")) + (
                "+thrown" if a.get("thrown") else "") + (
                "+forced" if a.get("forced") else "")
        elif a is not None and a.get("action") == "level":
            act = "tree:" + str(a.get("level")) + "/" + str(a.get("geom"))
        pre_pts, _pre_bb = before[path]
        post_pts = points_min_z(stage, path)
        post_bb = bbox_min_z(stage, path)
        gap = None if post_pts is None else post_pts / ssf
        if gap is not None and abs(gap) > 0.02:
            n_bad += 1
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        dims, bearing = "-", "-"
        if prim and prim.IsValid() and prim.IsActive():
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_,
                                    UsdGeom.Tokens.render])
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if not r.IsEmpty():
                sz = r.GetSize()
                dims = "{0:.1f}x{1:.1f}x{2:.1f}".format(
                    sz[0] / ssf, sz[1] / ssf, sz[2] / ssf)
                cx = 0.5 * (r.GetMin()[0] + r.GetMax()[0]) / ssf
                cy = 0.5 * (r.GetMin()[1] + r.GetMax()[1]) / ssf
                ox = p["x_m"] + C_X
                oy = p["y_m"] + C_Y
                if a is not None and a.get("action") == "level" and \
                        a.get("geom"):
                    b = math.degrees(math.atan2(cy - oy, cx - ox)) % 360.0
                    bearing = "{0:.0f} (want {1:.0f})".format(
                        b, a["azimuth_deg"])
        print("{0:<26} {1:<13} {2:<20} {3:>8} {4:>8} {5:>8}  {6:<22} {7}"
              .format(
                  path.rsplit("/", 1)[-1][:26], p["category"], act[:20],
                  "n/a" if pre_pts is None else "{0:.3f}".format(pre_pts / ssf),
                  "n/a" if post_pts is None else "{0:.3f}".format(post_pts / ssf),
                  "n/a" if gap is None else "{0:+.3f}".format(gap),
                  dims, bearing))
    print("-" * 132)
    print("[seat] {0} prop(s) off grade by > 2 cm".format(n_bad))
    tipped = sum(1 for a in actions if a.get("kind") == "car"
                 and a.get("toppled"))
    rolled = sum(1 for a in actions if a.get("kind") == "car"
                 and not a.get("toppled"))
    print("[seat] cars: {0} tipped ({1} thrown, {2} forced), {3} shoved"
          .format(tipped, counts.get("n_thrown", 0),
                  counts.get("n_forced", 0), rolled))
    return 0


if __name__ == "__main__":
    sys.exit(main())
