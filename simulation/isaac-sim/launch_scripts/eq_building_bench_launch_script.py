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
    EQ_MILD_TILT    "<deg>,<sink_m>": give every `pristine` column the CITY
                    assembly's mild lean + ground response (agent C)
    EQ_YAW          build every column at this yaw (default 0) — the only way
                    to see a recipe the way the CITY runs it (agent C)
    EQ_NEIGHBOUR    "<style>,<gap_m>,<side>": a second PRISTINE kit building
                    that clear of every column on that side (S/E/N/W), static
                    for the settle, so the PAIR recipes (`lean_on`,
                    `collapse_onto`, `pounding`) have something to fall into.
                    The review cameras then frame the pair, not the column.
                        EQ_STYLE=apartment_tall EQ_RECIPES=lean_on \
                        EQ_NEIGHBOUR=walkup,6.0,E
"""

import math
import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

# ISAAC_SIM_HEADLESS=true renders off-screen; the captures still work.
simulation_app = SimulationApp(launch_config={"headless": os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() in ("1", "true", "yes")})

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
# --- AGENT C: EQ_MILD_TILT="<deg>,<sink_m>" ---------------------------------
# Applies the CITY assembly's mild lean (quake._tilt_prim's matrix + ground
# response) to every `pristine` column, so a bench row can show the city path
# and the baked TILT recipe next to each other. Empty = off.
_MT = os.environ.get("EQ_MILD_TILT", "").strip()
MILD_TILT = None
if _MT:
    _f = [q.strip() for q in _MT.split(",") if q.strip()]
    MILD_TILT = (float(_f[0]), float(_f[1]) if len(_f) > 1 else 0.8)
# EQ_YAW builds every column at that yaw instead of 0. The bench and the bake
# have only ever built at yaw 0, which is why a pivot that was rotated into
# world and then rotated again by `_to_world` survived in three recipes: it is
# the identity at yaw 0 and 41 % of a city plat is at yaw 90 or 270.
YAW = float(os.environ.get("EQ_YAW", "0") or 0)
# --- end AGENT C ------------------------------------------------------------


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

    # --- AGENT D: EQ_NEIGHBOUR=<style>,<gap_m>,<side> -----------------------
    # A second, PRISTINE kit building `gap_m` clear of EVERY column building on
    # `side` (S/E/N/W), placed before the recipes run and registered as static
    # for the settle, so a pair recipe (`lean_on`, `collapse_onto`,
    # `pounding`) has something to fall into. The recipes see it through
    # `ctx["neighbours"]` (`quake_flow.d_set_neighbours`).
    NB = os.environ.get("EQ_NEIGHBOUR", "").strip()
    nb_style, nb_gap, nb_side = None, 0.4, "E"
    nbW = nbD = nbH = 0.0
    if NB:
        _f = [q.strip() for q in NB.split(",")]
        nb_style = _f[0] or None
        if len(_f) > 1 and _f[1]:
            nb_gap = float(_f[1])
        if len(_f) > 2 and _f[2]:
            nb_side = _f[2].upper()[:1]
        if nb_style not in ub.STYLES:
            raise RuntimeError("EQ_NEIGHBOUR: unknown style {0}".format(nb_style))
        nbW, nbD = ub.footprint(ub.STYLES[nb_style])
        nbH = ub.height(ub.STYLES[nb_style])
        # a PAIR is wider than one building: keep each column clear of the
        # next column's neighbour
        spacing = max(spacing, 1.35 * (big + nb_gap + max(nbW, nbD)) + 14.0)
        print("[eq_bench] neighbour {0}: {1} x {2} m, {3} m tall, {4:.2f} m gap "
              "on the {5} side; column spacing {6:.0f} m".format(
                  nb_style, nbW, nbD, nbH, nb_gap, nb_side, spacing))
    # --- end AGENT D -------------------------------------------------------

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
            pls = ub.build_building(st, x, y, YAW, random.Random(SEED))   # EQ_YAW (agent C)
            sg.apply_placements(stage, pls, parent, ssf)
            cols.append(dict(i=k, col=i, rec=rec, x=x, y=y, parent=parent, pls=pls,
                             style=st, W=dims[st][0], D=dims[st][1], H=dims[st][2]))
            k += 1
    # --- AGENT D: place one neighbour per column and register the pair ------
    nb_all = []
    if nb_style:
        _NRM = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}
        _ox, _oy = _NRM[nb_side]
        for c in cols:
            # the two facing half-depths plus the clear gap between the walls
            _B = c["D"] if nb_side in ("S", "N") else c["W"]
            _Bn = nbD if nb_side in ("S", "N") else nbW
            _off = _B / 2.0 + nb_gap + _Bn / 2.0
            nx, ny = c["x"] + _ox * _off, c["y"] + _oy * _off
            nparent = "{0}/nb{1}".format(PARENT, c["i"])
            UsdGeom.Scope.Define(stage, Sdf.Path(nparent))
            npls = ub.build_building(nb_style, nx, ny, 0.0, random.Random(SEED + 17))
            sg.apply_placements(stage, npls, nparent, ssf)
            c["nb_pls"] = npls
            c["nb_xy"] = (nx, ny, nbW, nbD, nbH)
            qf.d_set_neighbours("b{0}".format(c["i"]), [
                qf.d_neighbour(nb_style, npls, nx, ny, 0.0, nb_side, nb_gap,
                               parent=nparent, tag="nb{0}".format(c["i"]))])
            nb_all += npls
        ub.apply_glass_tint(stage, nb_all)
        print("[eq_bench] {0} neighbour building(s) placed on the {1} side".format(
            len(cols), nb_side))
    # --- end AGENT D -------------------------------------------------------

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
            # --- AGENT C: EQ_MILD_TILT=<deg>,<sink_m> -----------------------
            # The CITY path on a `pristine` column: `quake._tilt_prim`'s matrix
            # and ground response, applied to the kit pieces instead of to a
            # referenced archetype, so the reviewer can compare the city's mild
            # lean with the baked TILT recipe side by side in one row. This is
            # the only place the assembly-side code is exercised on the bench.
            if MILD_TILT:
                from disaster import quake as qk           # no SimulationApp
                mrng = random.Random(SEED + 101 * c["i"])
                mm = qk._c_mass({"x_m": c["x"], "y_m": c["y"], "z_m": 0.0,
                                 "yaw_deg": YAW},
                                {"W": c["W"], "D": c["D"], "H": c["H"]}, ssf)
                M, gg = qf._c_tilt_matrix(mm, mrng.choice(["S", "E", "N", "W"]),
                                          MILD_TILT[0], MILD_TILT[1],
                                          max_drop_m=qf.C_MAX_DROP_M)
                qf._transform_prims(
                    stage, [p["prim_path"] for p in c["pls"] if p.get("prim_path")], M)
                qk._c_tilt_ground(stage, c["parent"], mm, M, mrng, geom=gg,
                                  mats=mats, tag="mild{0}".format(c["i"]),
                                  scope=c["parent"] + "/quake_tilt")
                c["notes"] = ["mild tilt (city path): {0:.1f} deg toward {1}, "
                              "sunk {2:.2f} m (low -{3:.2f} m, high +{4:.2f} m)".format(
                                  MILD_TILT[0], gg["low"], gg["sink"], gg["drop"],
                                  gg["rise"])]
                for _n in c["notes"]:
                    print("[eq_bench]     " + _n)
            # --- end AGENT C ------------------------------------------------
            continue
        # a LEVEL (DG1..DG5 and the three foundation levels, looked up in
        # `quake_flow.LADDER` for this style's construction type) or a bare
        # recipe name. `SETTLE`/`TILT`/`OV` used to fall through to
        # `RECIPES["TILT"]` and raise KeyError.
        recipes = rec if (rec.startswith("DG") or rec in qf.FOUNDATION) else [(rec, {})]
        rng = random.Random(SEED + 101 * c["i"])
        nrng = np.random.default_rng(SEED + 101 * c["i"])
        tb = time.time()
        res = qf.wreck_building(stage, c["parent"], c["style"], c["pls"], c["x"], c["y"],
                                YAW, recipes, rng, nrng, mats, "b{0}".format(c["i"]),
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

    # --- AGENT D: the neighbours are static for the settle, and the review
    # cameras frame the PAIR rather than the column building ----------------
    if nb_style:
        _lo = set(loose)
        for c in cols:
            static += [p["prim_path"] for p in c.get("nb_pls", [])
                       if p.get("prim_path") and p["prim_path"] not in _lo]
            nx, ny, _w, _d, _h = c["nb_xy"]
            ax0 = min(c["x"] - c["W"] / 2.0, nx - _w / 2.0)
            ax1 = max(c["x"] + c["W"] / 2.0, nx + _w / 2.0)
            ay0 = min(c["y"] - c["D"] / 2.0, ny - _d / 2.0)
            ay1 = max(c["y"] + c["D"] / 2.0, ny + _d / 2.0)
            c["x"], c["y"] = (ax0 + ax1) / 2.0, (ay0 + ay1) / 2.0
            c["W"], c["D"] = ax1 - ax0, ay1 - ay0
            c["H"] = max(c["H"], _h)
    # --- end AGENT D -------------------------------------------------------

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

    print("EQ BENCH DONE")
    # Headless (ISAAC_SIM_HEADLESS=true) there is nobody to review the stage:
    # exit once the captures are on disk unless KEEP_OPEN=1. The GUI path
    # keeps the app open for the review cameras, as before.
    _keep = (os.environ.get("KEEP_OPEN", "").strip() == "1"
             or os.environ.get("ISAAC_SIM_HEADLESS", "false").strip().lower() not in ("1", "true", "yes"))
    app = omni.kit.app.get_app()
    if _keep:
        timeline.play()
        while simulation_app.is_running():
            app.update()
        timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
