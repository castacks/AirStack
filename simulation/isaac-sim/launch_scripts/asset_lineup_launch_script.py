#!/usr/bin/env python
"""
ASSET LINEUP — every building we have, side by side, grouped by pack.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/lineup \
    ISAAC_SIM_SCRIPT_NAME=asset_lineup_launch_script.py airstack up isaac-sim

Not a city. A showroom: each pack stands on its own tinted ground pad with a
clear gap between them, buildings in name order so a given one is easy to
find, all facing the same way so they can be compared. Click any building in
the GUI and its prim path names the pack and the asset:

    /World/lineup/GreatAmericanCity/SM_Building_17
    /World/lineup/CitySample/CHC_A

ZONES
  GreatAmericanCity  31 whole buildings, ready to place, 36k-823k tris each
  CitySample         18 families composed out of the façade kit by
                     `detail/citysample_building` — the pack ships the parts
                     but not the buildings; see that module's docstring

PLACED BY BOUNDING-BOX CENTRE, NOT BY THE ASSET'S OWN ORIGIN. Those are not
the same point and on this stock they are far apart — `SM_Building_31`'s
origin is 70 m from its footprint centre — so translating to (x, y, 0) leaves
a building metres from the slot it was given and, where `z0` is not zero,
off the ground. `_plans/gac_buildings.json` carries `cx, cy, z0` for exactly
this.

Env:
    LINEUP_ROW_M   target row width, m (default 420)
    LINEUP_PACKS   comma list to show (default "gac,cs")
    CS_MAX_POINTS  per-module budget for the CitySample kit (default 60000)
    SNAP_DIR       captures, under /isaac-sim/.nvidia-omniverse/logs/
"""

import json
import math
import os
import sys
import time

from isaacsim import SimulationApp


def _env(n, d=""):
    v = os.environ.get(n)
    return d if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")

import omni.kit.app                                            # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade        # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

import scene_generator as sg                                   # noqa: E402
from detail import citysample_building as cs                   # noqa: E402
from disaster import damage                                    # noqa: E402

ROOT = "/World/lineup"
NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
GAC_DIR = "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/"
SCALE_GAC = 0.01

ROW_M = float(_env("LINEUP_ROW_M", "420"))
PACKS = [q.strip() for q in _env("LINEUP_PACKS", "gac,cs").split(",") if q.strip()]
CS_MAX_POINTS = int(_env("CS_MAX_POINTS", str(cs.MAX_POINTS)))
CS_INSTANCE = _env("CS_INSTANCE", "1") not in ("0", "false")
SNAP_DIR = _env("SNAP_DIR", "")

GAP = 18.0            # between neighbours in a row — wide enough to read
                      # each building as its own object, which is the point
ROW_GAP = 30.0        # between rows
ZONE_GAP = 170.0      # between one pack's pad and the next
PAD_M = 26.0          # pad margin round a zone


def _rows(items, row_m):
    """Name-ordered items into rows no wider than `row_m`."""
    rows, cur, run = [], [], 0.0
    for it in items:
        sep = 0.0 if not cur else GAP
        if cur and run + sep + it["W"] > row_m:
            rows.append((cur, run))
            cur, run, sep = [], 0.0, 0.0
        cur.append((it, run + sep))
        run += sep + it["W"]
    if cur:
        rows.append((cur, run))
    return rows


def layout(items, row_m, y_top):
    """Place `items` in rows going DOWN from `y_top`. Returns (placed, y_bot)."""
    placed, y = [], y_top
    for row, run in _rows(items, row_m):
        dmax = max(it["D"] for it, _ in row)
        x0 = -run / 2.0
        for it, off in row:
            placed.append(dict(it, x=x0 + off + it["W"] / 2.0,
                               y=y - dmax / 2.0))
        y -= dmax + ROW_GAP
    return placed, y + ROW_GAP


def pad(stage, path, x0, y0, x1, y1, mat, z=0.02):
    g = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    g.CreatePointsAttr([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y0, z),
                        Gf.Vec3f(x1, y1, z), Gf.Vec3f(x0, y1, z)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateExtentAttr([Gf.Vec3f(x0, y0, z), Gf.Vec3f(x1, y1, z)])
    UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(mat)


def place_gac(stage, path, e):
    holder = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    # a typeless child takes the reference: these assets' default prim is a
    # Mesh, and referencing onto a declared Xform leaves the local type
    # winning — the prim then holds mesh attributes, is not a Mesh, and draws
    # nothing
    kid = stage.DefinePrim(Sdf.Path(path + "/asset"))
    kid.GetReferences().AddReference(sg._join_asset_root(e["usd"], NUC))
    stage.Load(Sdf.Path(path))
    xf = UsdGeom.Xformable(holder)
    xf.ClearXformOpOrder()
    # ops apply as T . R . S, and cx/cy/z0 are already in metres, so the
    # translate that lands the CENTRE on (x, y) and the base on z=0 is direct
    xf.AddTranslateOp().Set(Gf.Vec3d(float(e["x"] - e["cx"]),
                                     float(e["y"] - e["cy"]),
                                     float(-e["z0"])))
    xf.AddScaleOp().Set(Gf.Vec3f(SCALE_GAC, SCALE_GAC, SCALE_GAC))
    return holder.GetPrim()


def main():
    t0 = time.time()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    w = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(w.GetPrim())
    UsdGeom.Scope.Define(stage, Sdf.Path(ROOT))

    gac = sorted(json.load(open(os.path.join(_SG, "_plans",
                                             "gac_buildings.json"))),
                 key=lambda r: r["name"])
    kit = cs.load_kit()
    zones = []
    if "gac" in PACKS:
        zones.append(("GreatAmericanCity", "gac",
                      [dict(r, usd=GAC_DIR + r["name"] + ".usd",
                            style=r["name"]) for r in gac],
                      (0.058, 0.050, 0.040)))
    if "cs" in PACKS:
        items = []
        for fam, var in cs.families(kit):
            W, D = cs.footprint(kit, fam, var)
            spec = cs.plan_building(kit, fam, var, 70.0,
                                    max_points=CS_MAX_POINTS)
            if spec:
                items.append({"style": fam + "_" + var, "fam": fam,
                              "var": var, "spec": spec, "W": W, "D": D,
                              "H": spec["H"]})
        zones.append(("CitySample", "cs", sorted(items,
                                                 key=lambda q: q["style"]),
                      (0.036, 0.042, 0.056)))

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1250.0)
    dome.CreateColorAttr(Gf.Vec3f(0.80, 0.84, 0.93))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(3800.0)
    key.CreateAngleAttr(0.8)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.93))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 0.0, 26.0))
    ground = damage._pbr(stage, "/World/Looks/ground", (0.020, 0.021, 0.022),
                         0.95)
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 2400.0
    g.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                        Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateExtentAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, e, 0)])
    UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(ground)
    roof_mat = damage._pbr(stage, "/World/Looks/roofslab",
                           (0.048, 0.047, 0.045), 0.90)

    y_top, bands, n_miss, tall = 0.0, [], 0, 0.0
    for zname, kind, items, rgb in zones:
        placed, y_bot = layout(items, ROW_M, y_top)
        pmat = damage._pbr(stage, "/World/Looks/pad_" + kind, rgb, 0.94)
        x0 = min(p["x"] - p["W"] / 2.0 for p in placed) - PAD_M
        x1 = max(p["x"] + p["W"] / 2.0 for p in placed) + PAD_M
        pad(stage, "{0}/{1}_pad".format(ROOT, kind),
            x0, y_bot - PAD_M, x1, y_top + PAD_M, pmat)
        print("\n=== {0}: {1} buildings   x {2:.0f}..{3:.0f}   y {4:.0f}..{5:.0f}"
              .format(zname, len(placed), x0, x1, y_bot - PAD_M,
                      y_top + PAD_M), flush=True)
        for p in placed:
            path = "{0}/{1}/{2}".format(ROOT, zname, p["style"])
            if kind == "gac":
                pr = place_gac(stage, path, p)
                nres = sum(1 for q in Usd.PrimRange(pr) if q.IsA(UsdGeom.Mesh))
                n_miss += 0 if nres else 1
                note = "{0:5.0f}k tris{1}".format(
                    p["tris"] / 1000.0, "" if nres else "  << NOT LOADED")
                H = p["H"]
            else:
                r = cs.build(stage, path, kit, p["spec"], p["x"], p["y"],
                             p["W"], p["D"], yaw=0.0, tag="m",
                             material=roof_mat, instance=CS_INSTANCE)
                # LOAD THE PAYLOADS AND THEN MEASURE WHAT IS ACTUALLY THERE.
                # `build` returns the height it AUTHORED, which is arithmetic
                # over the catalogue and true whether or not a single module
                # reached the stage — the same trap the GreatAmericanCity
                # references fell into. The bounding box is the only witness.
                for _ in range(2):
                    omni.kit.app.get_app().update()
                bb = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    [UsdGeom.Tokens.default_]).ComputeWorldBound(
                        stage.GetPrimAtPath(path)).ComputeAlignedRange()
                mh = 0.0 if bb.IsEmpty() else bb.GetMax()[2]
                H = r["H"]
                ok = mh > r["H"] * 0.5
                n_miss += 0 if ok else 1
                note = "{0:2d} lvl {1:4d} prims  measured {2:5.1f} m{3}".format(
                    r["levels"], r["prims"], mh,
                    "" if ok else "  << NOT DRAWN")
            tall = max(tall, H)
            print("  {0:<22} {1:6.1f} x {2:5.1f} x {3:6.1f} m   at "
                  "({4:7.0f},{5:7.0f})   {6}"
                  .format(p["style"], p["W"], p["D"], H, p["x"], p["y"], note),
                  flush=True)
        bands.append((zname, x0, x1, y_bot - PAD_M, y_top + PAD_M))
        y_top = y_bot - PAD_M - ZONE_GAP
        omni.kit.app.get_app().update()

    for _ in range(90):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 74)
    print("ASSET LINEUP — {0} buildings over {1} pack(s)"
          .format(sum(len(z[2]) for z in zones), len(zones)))
    for zname, x0, x1, ylo, yhi in bands:
        print("  {0:<20} {1:5.0f} x {2:5.0f} m pad".format(
            zname, x1 - x0, yhi - ylo))
    if n_miss:
        print("  !! {0} building(s) FAILED TO LOAD".format(n_miss))
    print("  tallest {0:.0f} m   built in {1:.0f} s"
          .format(tall, time.time() - t0))
    print("=" * 74 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            ax0 = min(b[1] for b in bands)
            ax1 = max(b[2] for b in bands)
            ay0 = min(b[3] for b in bands)
            ay1 = max(b[4] for b in bands)
            cx, cy = 0.5 * (ax0 + ax1), 0.5 * (ay0 + ay1)
            allspan = max(ax1 - ax0, ay1 - ay0)
            sn.place_camera(stage, (cx, cy, tall + allspan / 1.05),
                            (cx, cy, 0.0))
            sn.snapshot(os.path.join(SNAP_DIR, "all_top.png"))
            for zname, x0, x1, ylo, yhi in bands:
                zx, zy = 0.5 * (x0 + x1), 0.5 * (ylo + yhi)
                zs = max(x1 - x0, yhi - ylo)
                sn.place_camera(stage, (zx, zy, tall + zs / 1.15), (zx, zy, 0.0))
                sn.snapshot(os.path.join(SNAP_DIR, zname + "_top.png"))
                # an oblique from the south, far enough back for the tallest
                sn.place_camera(stage, (zx, ylo - zs * 0.75, zs * 0.42 + tall * 0.45),
                                (zx, zy, tall * 0.28))
                sn.snapshot(os.path.join(SNAP_DIR, zname + "_view.png"))
                # and a low walk-past, which is where a façade gives itself up
                sn.place_camera(stage, (x0 + (x1 - x0) * 0.18, ylo - 95.0, 22.0),
                                (zx, zy, 45.0))
                sn.snapshot(os.path.join(SNAP_DIR, zname + "_close.png"))
            print("[lineup] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[lineup] snapshots FAILED: {0}".format(exc))

    print("ASSET LINEUP DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
