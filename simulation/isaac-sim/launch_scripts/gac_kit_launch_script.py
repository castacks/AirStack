#!/usr/bin/env python
"""
GAC buildings cut into a KIT, laid out for inspection. No damage.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/gac_kit \
    ISAAC_SIM_SCRIPT_NAME=gac_kit_launch_script.py airstack up isaac-sim

`detail/gac_slice.py` cuts a merged GreatAmericanCity mesh on its own measured
storey/bay grid into the same grammar `detail/urban_building.py` assembles the
ModernCityEnvironment01 kit with: a roof, and per storey a run of wall pieces
between two corner pieces. This scene exists to LOOK at that cut before any
damage is put on top of it — a bad slice and a good slice produce identical
logs, and the only way to tell them apart is to see the pieces.

Three views, side by side, one column per building:

  ASSEMBLED   the pieces back in place. Should be indistinguishable from the
              original asset — if it is not, the cut lost or moved geometry.
  EXPLODED    every piece pushed out along its own elevation and up by its
              storey. This is where a wrong storey period shows up, as pieces
              that straddle two floors instead of sitting on one.
  CATALOGUE   the distinct pieces laid flat on the ground in a grid, which is
              what a kit-bash kit actually looks like on disk.

THE MATERIALS ARE SLICED TOO, which is not cosmetic. Out of the box a piece
binds to a Material prim living inside the merged source's subtree, so the
pieces only render while the thing they were cut out of is still on the stage;
deactivating it turns every piece WHITE with its geometry and UVs perfectly
intact. `gac_slice.rehome_materials` gives the kit its own `Looks` scope,
referencing each material's own `Materials/M_*_Inst.usd` file — measured with
`tools/gac_mat_probe.py` — so a piece is self-contained.

Env:
    GK_BUILDINGS  comma list (default SM_Building_01,SM_Building_04,SM_Building_24)
    GK_MODE       assembled | exploded | catalogue | all   (default all)
    GK_EXPLODE_M  outward separation, m (default 2.5)
    GK_EXPLODE_Z  extra rise per storey, m (default 1.6)
    GK_COL_M      spacing between buildings, m (default 90)
    GK_HIDE_SRC   1 (default) hides the merged original — the proof that the
                  pieces stand on their own. 0 leaves it for an A/B.
    SNAP_DIR / KEEP_OPEN
"""

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
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                  # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import gac_slice as gsl                            # noqa: E402

PARENT = "/World/kit"
GAC_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
           "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
BUILDINGS = [v.strip() for v in _env(
    "GK_BUILDINGS", "SM_Building_01,SM_Building_04,SM_Building_24"
).split(",") if v.strip()]
MODE = _env("GK_MODE", "all").lower()
EX_M = float(_env("GK_EXPLODE_M", "2.5"))
EX_Z = float(_env("GK_EXPLODE_Z", "1.6"))
COL_M = float(_env("GK_COL_M", "90"))
HIDE_SRC = _env("GK_HIDE_SRC", "1") not in ("0", "false", "no")
SNAP_DIR = _env("SNAP_DIR")
# how far apart the three views stand, in metres of Y
VIEW_Y = {"assembled": -170.0, "exploded": 0.0, "catalogue": 190.0}


def ground_and_light(stage, w, d):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1200.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.85, 0.92))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2600.0)
    key.CreateAngleAttr(0.9)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.93))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-46.0, 0.0, 34.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    hw, hd = w * 0.6, d * 0.6
    g.CreatePointsAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, -hd, 0),
                        Gf.Vec3f(hw, hd, 0), Gf.Vec3f(-hw, hd, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.32, 0.31)])
    g.CreateExtentAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, hd, 0)])


def place_source(stage, cell, usd, scale=0.01):
    """Reference the merged asset, centred in plan with its base at z=0."""
    holder = cell + "/src"
    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    kid.GetReferences().AddReference(usd)
    stage.Load(Sdf.Path(holder))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    c = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(cell)).ExtractTranslation()
    tr.Set(Gf.Vec3d(-(0.5 * (mn[0] + mx[0]) - c[0]),
                    -(0.5 * (mn[1] + mx[1]) - c[1]), -(mn[2] - c[2])))
    return holder


def build_one(stage, name, cell, view):
    """Slice `name` into `cell`, re-home its materials, return the placements."""
    src = place_source(stage, cell, GAC_DIR + name + ".usd", 0.01)
    if not src:
        print("      {0}: nothing composed".format(name))
        return None, None
    wins, bbox = gsl.window_centres(stage, src)
    g = gsl.measure_grid(wins, bbox, name=name)
    if g.get("confidence", 0.0) < gsl.MIN_CONFIDENCE:
        print("      {0}: grid not recovered (confidence {1:.2f}) — NOT sliced"
              .format(name, g.get("confidence", 0.0)))
        return None, g
    pls = gsl.slice_building(stage, src, cell + "/pieces", g, "gac_" + name)
    # THE MATERIALS, WITHOUT WHICH THIS IS NOT A KIT
    mats = gsl.rehome_materials(stage, getattr(gsl.slice_building,
                                               "last_materials", {}),
                                cell + "/Looks")
    from pxr import UsdShade
    n_rebound = 0
    for p in pls:
        prim = stage.GetPrimAtPath(p["prim_path"])
        if not prim or not prim.IsValid():
            continue
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            cur = UsdShade.MaterialBindingAPI(
                sub.GetPrim()).ComputeBoundMaterial()[0]
            key = str(cur.GetPrim().GetPath()) if cur and cur.GetPrim().IsValid() else ""
            new = mats.get(key)
            if new is not None:
                UsdShade.MaterialBindingAPI(sub.GetPrim()).Bind(new)
                n_rebound += 1
    print("      {0}: {1} subset(s) re-bound to the kit's own Looks"
          .format(name, n_rebound))
    if HIDE_SRC:
        # HIDDEN, NOT DEACTIVATED — the materials were referenced out, but the
        # source is also what the assembled view is compared against, and a
        # deactivated prim cannot be toggled back on in the viewport.
        UsdGeom.Imageable(stage.GetPrimAtPath(src)).MakeInvisible()
    if view == "exploded":
        gsl.explode(stage, pls, out_m=EX_M, up_m=EX_Z)
    elif view == "catalogue":
        catalogue(stage, pls, g)
    return pls, g


def catalogue(stage, pls, g):
    """Lay the pieces flat on the ground in a grid, grouped by role.

    What a kit-bash kit looks like on disk: every distinct piece standing on
    its own, in rows by role, so a malformed piece cannot hide inside the
    silhouette of the building it came from.
    """
    order = {"corner": 0, "wall": 1, "parapet": 2, "parapet_corner": 3,
             "roof": 4, "core": 5}
    rows = {}
    for p in pls:
        rows.setdefault(p.get("_role", "wall"), []).append(p)
    pitch = max(6.0, 1.3 * (g["storey_h"] + 1.0))
    for role, items in sorted(rows.items(), key=lambda kv: order.get(kv[0], 9)):
        ri = order.get(role, 9)
        items.sort(key=lambda q: (q.get("_side", ""), q.get("_storey", 0),
                                  q.get("_bay", 0)))
        ncol = max(1, int(math.ceil(math.sqrt(len(items)))))
        for k, p in enumerate(items):
            prim = stage.GetPrimAtPath(p["prim_path"])
            if not prim or not prim.IsValid():
                continue
            gx = (k % ncol) * pitch - 0.5 * ncol * pitch
            gy = (k // ncol) * pitch + ri * 3.0 * pitch
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            # each piece to its own slot, standing on the ground
            xf.AddTranslateOp().Set(Gf.Vec3d(
                gx - p["x_m"], gy - p["y_m"], -p["z_m"]))


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    _, ssf = get_stage_meters_per_unit(stage)
    t0 = time.time()
    problems = gsl.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))

    views = (["assembled", "exploded", "catalogue"] if MODE == "all"
             else [MODE])
    ground_and_light(stage, len(BUILDINGS) * COL_M + 400.0, 900.0)
    grids, made = {}, 0
    for view in views:
        y = VIEW_Y.get(view, 0.0)
        print("\n[gac_kit] === {0} ===".format(view))
        for ci, name in enumerate(BUILDINGS):
            x = (ci - (len(BUILDINGS) - 1) / 2.0) * COL_M
            cell = "{0}/{1}_{2}".format(PARENT, view, ci)
            cx = UsdGeom.Xform.Define(stage, Sdf.Path(cell))
            cx.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
            tb = time.time()
            try:
                pls, g = build_one(stage, name, cell, view)
                if pls:
                    made += 1
                    grids[name] = g
                    print("      {0:<18} {1} piece(s)  ({2:.0f} s)".format(
                        name, len(pls), time.time() - tb))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("      FAILED {0}: {1}".format(name, exc))
            for _ in range(2):
                omni.kit.app.get_app().update()

    for _ in range(10):
        omni.kit.app.get_app().update()
    print("\n" + "=" * 74)
    print("GAC KIT   {0} building(s) x {1} view(s) = {2} built".format(
        len(BUILDINGS), len(views), made))
    for nm, g in sorted(grids.items()):
        print("  {0:<18} {1:.1f} x {2:.1f} x {3:.1f} m   storey {4:.2f} m x{5}"
              "   confidence {6:.2f}".format(
                  nm, g["W"], g["D"], g["H"], g["storey_h"],
                  len(g["storeys"]), g["confidence"]))
    print("  views: " + ", ".join(views) +
          "   (merged original {0})".format("hidden" if HIDE_SRC else "shown"))
    print("  {0:.0f} s".format(time.time() - t0))
    print("=" * 74 + "\n")

    app = omni.kit.app.get_app()
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            for _ in range(120):
                app.update()
            # one framed shot per (view, building), sized off that building
            for view in views:
                y = VIEW_Y.get(view, 0.0)
                for ci, name in enumerate(BUILDINGS):
                    g = grids.get(name)
                    if not g:
                        continue
                    x = (ci - (len(BUILDINGS) - 1) / 2.0) * COL_M
                    span = max(g["W"], g["D"], g["H"]) * (2.2 if view == "catalogue"
                                                          else 1.5)
                    sn.place_camera(
                        stage,
                        ((x - span * 0.75) * ssf, (y - span * 0.75) * ssf,
                         span * 0.62 * ssf),
                        (x * ssf, y * ssf, g["H"] * 0.45 * ssf))
                    sn.snapshot(os.path.join(
                        SNAP_DIR, "{0}_{1}.png".format(view, name)))
            print("[gac_kit] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[gac_kit] snapshots FAILED: {0}".format(exc))

    if _env("KEEP_OPEN") == "1" or not _HEADLESS:
        while simulation_app.is_running():
            app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
