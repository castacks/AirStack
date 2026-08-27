#!/usr/bin/env python
"""
Earthquake building bench — ONE kit style, a row of damage recipes, on flat
ground, fractured and settled live, captured from above and obliquely.

    EQ_STYLE=commercial EQ_RECIPES=DG2,DG3,DG4,DG5,tilt_sink \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/eq_bench \
    ISAAC_SIM_SCRIPT_NAME=eq_building_bench_launch_script.py airstack up isaac-sim

Each column of the row is the SAME building (same style, same façade seed)
taken apart by one entry of `EQ_RECIPES`, so the row is a controlled
comparison of the recipes and nothing else. An entry is either a GRADE
(`DG1`..`DG5`, looked up in `quake_flow.LADDER` for the style's construction
type) or a single RECIPE name (`out_of_plane`, `soft_storey`, `pancake`,
`tilt_sink`, ...) run on its own with its defaults, or `pristine`.

This is the earthquake counterpart of `house_damage_test_launch_script.py`
and it is deliberately SLOW: every building is fractured and settled in
process. The archetype bake is where that cost goes once the look is right.

Env:
    EQ_STYLE        urban_building style (default commercial)
    EQ_RECIPES      comma list (default pristine,DG2,DG3,DG4,DG5)
    EQ_SEED         rng seed (default 7)
    EQ_SPACING      metres between columns (default = 2.6 x footprint, min 50)
    SETTLE_STEPS    physics step ceiling (default 900)
    KEEP_PHYSICS    1 leaves bodies live instead of baking
    SNAP_DIR        viewport captures, MUST be under /isaac-sim/.nvidia-omniverse/logs/
    EQ_FIT_ALL      1 fits out every storey (default: only what a recipe opens)
"""

import math
import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                  # noqa: E402
from scene_prep import get_stage_meters_per_unit              # noqa: E402
from detail import urban_building as ub                       # noqa: E402
from disaster import fracture, settle, quake_flow as qf        # noqa: E402

PARENT = "/World/stage/generated"
STYLES = [q.strip() for q in os.environ.get("EQ_STYLE", "commercial").split(",") if q.strip()]
STYLE = STYLES[0]
RECIPES = [r.strip() for r in os.environ.get(
    "EQ_RECIPES", "pristine,DG2,DG3,DG4,DG5").split(",") if r.strip()]
SEED = int(os.environ.get("EQ_SEED", "7"))
SETTLE_STEPS = int(os.environ.get("SETTLE_STEPS", "2200"))
KEEP_PHYSICS = os.environ.get("KEEP_PHYSICS", "0") not in ("0", "", "false")
SNAP_DIR = os.environ.get("SNAP_DIR", "").strip()
FIT_ALL = os.environ.get("EQ_FIT_ALL", "0") not in ("0", "", "false")


def build_ground_and_light(stage):
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 600.0
    ground.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                             Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.36, 0.36, 0.35)])
    ground.CreateExtentAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, e, 0.0)])
    # Pavement-grey ground, because this is a downtown and the pile has to
    # read against asphalt, not lawn.
    try:
        mp = stage.DefinePrim(Sdf.Path("/World/Looks/pavement"))
        mp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/megascans/Road_Asphalt.usda", ""))
        mp.Load()
        m = UsdShade.Material.Get(stage, "/World/Looks/pavement")
        if m:
            UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(m)
    except Exception as exc:
        print("[eq_bench] ground material unavailable: {0}".format(exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.82, 0.9))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2800.0)
    key.CreateAngleAttr(0.8)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-48.0, 0.0, 35.0))


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
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    build_ground_and_light(stage)
    _, ssf = get_stage_meters_per_unit(stage)
    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=True)
    t0 = time.time()

    problems = ub.check(verbose=False) + qf.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))
    # ONE ROW PER STYLE (EQ_STYLE=commercial,office), one column per recipe.
    dims = {st: ub.footprint(ub.STYLES[st]) + (ub.height(ub.STYLES[st]),) for st in STYLES}
    big = max(max(w, d) for w, d, _h in dims.values())
    spacing = float(os.environ.get("EQ_SPACING", "0") or 0) or max(50.0, 2.6 * big)
    x0 = -0.5 * spacing * (len(RECIPES) - 1)
    y0 = -0.5 * spacing * (len(STYLES) - 1)
    spec = ub.STYLES[STYLE]
    W, D, H = dims[STYLE]

    # 1) build every column pristine, identically per row
    cols = []
    k = 0
    for r, st in enumerate(STYLES):
        for i, rec in enumerate(RECIPES):
            x, y = x0 + i * spacing, y0 + r * spacing
            parent = "{0}/b{1}".format(PARENT, k)
            UsdGeom.Scope.Define(stage, Sdf.Path(parent))
            pls = ub.build_building(st, x, y, 0.0, random.Random(SEED))
            sg.apply_placements(stage, pls, parent, ssf)
            cols.append(dict(i=k, col=i, rec=rec, x=x, y=y, parent=parent, pls=pls,
                             style=st, W=dims[st][0], D=dims[st][1], H=dims[st][2]))
            k += 1
    n_glass = ub.apply_glass_tint(stage, [p for c in cols for p in c["pls"]])
    for _ in range(5):
        omni.kit.app.get_app().update()
    for st in STYLES:
        print("[eq_bench] {0}: {1} x {2} m, {3} m tall, type {4}".format(
            st, dims[st][0], dims[st][1], dims[st][2],
            qf.FAMILY_TYPE.get(ub.STYLES[st].get("family"))))
    print("[eq_bench] {0} buildings, glass tint on {1}".format(len(cols), n_glass))

    # 2) wreck each column
    mats = qf.materials(stage, PARENT)
    cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    for c in cols:
        rec = c["rec"]
        if rec == "pristine":
            static += [p["prim_path"] for p in c["pls"] if p.get("prim_path")]
            c["notes"] = []
            continue
        recipes = rec if rec.startswith("DG") else [(rec, {})]
        rng = random.Random(SEED + 101 * c["i"])
        nrng = np.random.default_rng(SEED + 101 * c["i"])
        tb = time.time()
        res = qf.wreck_building(stage, c["parent"], c["style"], c["pls"], c["x"], c["y"],
                                0.0, recipes, rng, nrng, mats, "b{0}".format(c["i"]),
                                fit_storeys=None, mat_cache=cache)
        loose += res["loose"]
        static += res["static_extra"]
        vel.update(res["velocity"])
        c["notes"] = res["notes"]
        print("[eq_bench] {6:<12} {1:<16} {2:4d} loose, {3:4d} static, "
              "{4:4d} authored  ({5:.0f} s)".format(
                  c["i"], rec, len(res["loose"]), len(res["static_extra"]),
                  len(res["authored"]), time.time() - tb, c["style"]))
        for n in res["notes"]:
            print("[eq_bench]     " + n)
    for _ in range(10):
        omni.kit.app.get_app().update()

    # 3) settle — gravity only, per-body outward velocities, masonry density
    if loose:
        settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.12,
                   rng=random.Random(SEED), bake_result=not KEEP_PHYSICS,
                   velocity_map=vel, density=1900.0, max_speed=6.0)
    for _ in range(10):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 72)
    print("EARTHQUAKE BUILDING BENCH  {0}".format(", ".join(STYLES)))
    for c in cols:
        print("  x={0:+7.1f} y={1:+7.1f}  {2:<12} {3}".format(c["x"], c["y"], c["style"], c["rec"]))
    print("  {0} loose bodies, {1:.0f} s".format(len(loose), time.time() - t0))
    print("=" * 72 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            span = spacing * (len(RECIPES) - 1) + 2.5 * big
            _snaps.overview(stage, (0.0, 0.0), span, os.path.join(SNAP_DIR, "row.png"), ssf)
            _snaps.place_camera(stage, (-0.35 * span, -0.9 * span, 0.45 * span + H),
                                (0.0, 0.0, H * 0.3))
            _snaps.snapshot(os.path.join(SNAP_DIR, "row_obl.png"))
            # ONE CAMERA PRIM PER BUILDING PER ANGLE, kept on the stage under
            # /World/ReviewCams so a person at the GUI can flip between them
            # after the captures. top, four compass obliques, one street view.
            import omni.kit.viewport.utility as vp
            for c in cols:
                name = "{0}_{1}_{2}".format(c["col"], c["style"], c["rec"])
                x, y = c["x"], c["y"]
                W, D, H = c["W"], c["D"], c["H"]
                d = 1.5 * max(W, D, H * 0.8)
                views = {
                    "top": ((x, y, d * 1.4 + H), (x, y, 0.0)),
                    "sw": ((x - d * 0.75, y - d * 0.95, 0.55 * d + 0.4 * H), (x, y, H * 0.3)),
                    "se": ((x + d * 0.75, y - d * 0.95, 0.55 * d + 0.4 * H), (x, y, H * 0.3)),
                    "ne": ((x + d * 0.75, y + d * 0.95, 0.55 * d + 0.4 * H), (x, y, H * 0.3)),
                    "nw": ((x - d * 0.75, y + d * 0.95, 0.55 * d + 0.4 * H), (x, y, H * 0.3)),
                    "street": ((x - 0.3 * d, y - 0.9 * d - D / 2.0, 2.2), (x, y, H * 0.35)),
                }
                for vname, (eye, tgt) in views.items():
                    cpath = "/World/ReviewCams/b{0}_{1}".format(c["i"], vname)
                    cam = UsdGeom.Camera.Define(stage, Sdf.Path(cpath))
                    cam.GetHorizontalApertureAttr().Set(20.955)
                    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.5, 20000.0))
                    cam.GetFocalLengthAttr().Set(18.0)
                    xf = UsdGeom.Xformable(cam)
                    xf.ClearXformOpOrder()
                    xf.AddTranslateOp().Set(Gf.Vec3d(*eye))
                    xf.AddRotateXYZOp().Set(_snaps._look_at(eye, tgt))
                    vp.get_active_viewport().camera_path = cpath
                    _snaps.snapshot(os.path.join(SNAP_DIR, "{0}_{1}.png".format(name, vname)))
            vp.get_active_viewport().camera_path = _snaps.CAM
            print("[eq_bench] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[eq_bench] snapshots FAILED: {0}".format(exc))

    timeline.play()
    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
