#!/usr/bin/env python
"""
All-cars bench — every car in the pool, on empty ground, one person seated in
each. Nothing else is built: no fire, no blockage, no plat.

    ISAAC_SIM_SCRIPT_NAME=all_cars_bench_launch_script.py \
    SCENE_CONFIG=suburb_wildfire airstack up isaac-sim

WHY THIS EXISTS (and why it is not `car_occupants_launch_script.py`)
-------------------------------------------------------------------
`car_occupants_launch_script.py` is the TUNING bench: three cars, hand-tuned
seat offsets, a burning blockage around them. It answers "is this one car's
seat right, to the centimetre". This one answers a different question — "we
just added a pile of cars to the pool; stand a person in EVERY one and let me
look" — so it reads the whole `usds.cars` pool from the live config (new cars
appear here automatically) and lays them on a bare grid.

THE SEAT PER CAR comes from `disaster.people.car_seats`: the measured rows in
`_CAR_SEATS` for the three cars somebody sat on the tuning bench, and a derived
fallback (one driver, a tenth of the length forward) for every car nobody has
measured yet. A fallback figure that looks wrong is the SIGNAL — it means that
car has earned a row in `_CAR_SEATS`.

CAN YOU SEE THE OCCUPANT? Not in most of these. Every window in this library
renders OPAQUE (the renderer forces fractional cutout opacity to 1.0), so an
occupant is visible only where the glass can be REMOVED. Three cars in the pool
have separable glass (`130`, the GMC van, and the Fairlady via one named mesh);
the eleven standalone cars added on 2026-08-27 are each a single merged mesh
with the windows painted into the texture — nothing to remove, occupant
invisible from outside. So this bench takes TWO shots of every car:

    _ext   the car as it parks, glass opened where it can be. This is what a
           drone would actually see — and for a painted-glass car it is a car
           with nobody visible in it, which is the honest answer.
    _cut   the car body deactivated, the seated figure alone. This is the only
           way to judge the SEATING (pose, height, facing) on a car whose body
           you cannot see through, and the reason the bench is worth running on
           the new cars at all.

Grid, banner, and both shots per car; edit nothing to add a car — add it to the
pool and rerun.
"""

import math
import os
import sys

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"],
})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: F401

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import scene_generator as sg                                    # noqa: E402
import suburb_scene as ss                                       # noqa: E402
from scene_prep import get_stage_meters_per_unit                # noqa: E402
from compile_disaster import load_scene_config                  # noqa: E402
from disaster import people as ppl                              # noqa: E402
from detail import vehicles as veh                              # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
SNAP_DIR = os.environ.get("SNAP_DIR", "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)

# Grid geometry. Cars all point +X (heading 0); columns run along +X, rows
# stack along +Y. 16 m across and 14 m up clears the longest car in the pool
# (the 9.5 m bus) with room for the label post and the side camera.
COLS = int(os.environ.get("COLS", "5"))
DX_M = 16.0
DY_M = 14.0

GRASS_MAT = "airstack://scene_gen/assets/materials/Grass_Cut.usda"
ASPHALT_MAT = ("airstack://scene_gen/assets/materials/megascans/"
               "Road_Asphalt.usda")


def _mat(stage, key, url):
    path = "/World/ground/materials/" + key
    prim = stage.DefinePrim(Sdf.Path(path))
    prim.GetReferences().AddReference(sg._join_asset_root(url, ""))
    prim.Load()
    return path


def build_ground(stage, ssf, ncols, nrows):
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground"))
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground/materials"))
    grass = _mat(stage, "grass", GRASS_MAT)
    asphalt = _mat(stage, "asphalt", ASPHALT_MAT)
    x0, x1 = -DX_M, DX_M * ncols
    y0, y1 = -DY_M, DY_M * nrows
    sg._make_plane_mesh(stage, "/World/ground/lawn",
                        x0 - 8.0, y0 - 8.0, x1 + 8.0, y1 + 8.0,
                        -0.01, 3.0, ssf, display_color=(0.24, 0.36, 0.17),
                        mat_prim_path=grass)
    # An asphalt pad under each column lane, so a car reads as parked rather
    # than floating on a lawn.
    for c in range(ncols):
        cx = c * DX_M
        sg._make_plane_mesh(stage, "/World/ground/pad_{0}".format(c),
                            cx - 4.5, y0 - 4.0, cx + 4.5, y1 + 4.0,
                            0.02, 3.0, ssf, display_color=(0.21, 0.23, 0.26),
                            mat_prim_path=asphalt)

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.76, 0.80, 0.87))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2200.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-48.0, 0.0, 28.0))


def _label_post(stage, label, x, y):
    post = UsdGeom.Cube.Define(
        stage, Sdf.Path("{0}/post_{1}".format(PARENT, label)))
    post.CreateSizeAttr(1.0)
    px = UsdGeom.Xformable(post)
    px.ClearXformOpOrder()
    px.AddTranslateOp().Set(Gf.Vec3d(x * ssf, (y - 6.5) * ssf, 0.9 * ssf))
    px.AddScaleOp().Set(Gf.Vec3f(0.3 * ssf, 0.3 * ssf, 1.8 * ssf))
    post.CreateDisplayColorAttr([Gf.Vec3f(0.86, 0.24, 0.20)])


def main():
    global ssf
    omni.timeline.get_timeline_interface().stop()
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))

    _mpu, ssf = get_stage_meters_per_unit(stage)
    config = load_scene_config(SCENE_CONFIG)

    pools = ss.AssetPools(config)
    resolver = sg._make_resolver(config)
    raw_c = ss._raw_pool(config, "cars")
    raw_h = ss._raw_pool(config, "humans")
    cars_usd = pools.load(raw_c)
    glassy = frozenset(pools.load_tagged(raw_c, "glass_separable"))
    # Tag every car so the banner can say what a placement is FOR even where
    # nobody sits in it (livery, wreck, parked_only).
    tags = {}
    for t in ("residential", "commercial", "livery", "parked_only", "wreck",
              "rv", "glass_separable"):
        for p in pools.load_tagged(raw_c, t):
            tags.setdefault(p, set()).add(t)
    humans = pools.load_tagged(raw_h, "rigged") or pools.load(raw_h)
    ctx = {"asset_pools": pools, "resolver": resolver, "glassy": glassy}

    ncols = max(1, COLS)
    nrows = int(math.ceil(len(cars_usd) / float(ncols)))
    build_ground(stage, ssf, ncols, nrows)

    cars, people, rows = [], [], []
    for i, usd in enumerate(cars_usd):
        col, row = i % ncols, i // ncols
        x, y = col * DX_M, row * DY_M
        base = os.path.basename(usd)
        label = "{0:02d}_{1}".format(i, os.path.splitext(base)[0])[:28]

        car = ppl._car_placement(ctx, usd, x, y, 0.0, "bench")
        cars.append(car)

        fp = resolver.get(usd, "car", scale=pools.scale_of(usd),
                          axis_up=pools.axis_of(usd))
        ln = float(fp.get("sx", 4.5))
        wd = float(fp.get("sy", 1.9))
        roof = float(fp.get("sz", 1.5))

        seats = ppl.car_seats(usd, ln, wd, roof)
        n_here = 0
        for occ in seats:
            seat = max(0.05, roof / 3.0 + float(occ.get("dz", 0.0)))
            fwd, lat = float(occ.get("fwd", 0.0)), float(occ.get("lat", 0.0))
            hx, hy = x + fwd, y + lat            # heading 0 -> +X forward
            hu = humans[(len(people)) % len(humans)]
            people.append(ppl._human_placement(
                ctx, hu, hx, hy, seat, float(occ.get("dyaw", 0.0)),
                occ.get("pose", "seated_car_arms_down")))
            n_here += 1

        _label_post(stage, label, x, y)
        rows.append({
            "i": i, "label": label, "base": base, "x": x, "y": y,
            "ln": ln, "wd": wd, "roof": roof, "n_occ": n_here,
            "src": seats[0].get("source", "?"),
            "tags": sorted(tags.get(usd, [])),
            "can_open": veh.can_open_cabin(usd), "removed": 0,
        })

    # CARS, THEN GLASS, THEN PEOPLE — a passenger authored before the windows
    # come off is a passenger you cannot see.
    sg.apply_placements(stage, cars, PARENT + "/cars", ssf, resolver=resolver,
                        instance_categories=set())
    for q, rec in zip(cars, rows):
        pp = q.get("prim_path")
        if not pp:
            continue
        # Canonical per-asset recipe first (strip for the glassy cars, the
        # named mesh for the Fairlady, nothing for painted glass). If it took
        # nothing off, PROBE with a raw strip in case this asset grew real
        # glass the CABIN_RULES table has not been told about.
        n = veh.open_cabin(stage, pp, q["usd"])
        if n == 0:
            n = veh.strip_glass(stage, pp)
        rec["removed"] = n
    sg.apply_placements(stage, people, PARENT + "/people", ssf,
                        resolver=resolver, instance_categories=set())

    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    app = omni.kit.app.get_app()
    for _ in range(30):
        app.update()

    print("\n" + "=" * 82)
    print("ALL-CARS BENCH — {0} cars, {1} people, {2}x{3} grid"
          .format(len(cars), len(people), ncols, nrows))
    print("  {0:<30} {1:>5} {2:>5} {3:>5} {4:>4} {5:>8} {6:>7}  {7}".format(
        "label", "L", "W", "roof", "occ", "seat", "glass", "tags"))
    for r in rows:
        vis = ("{0} off".format(r["removed"]) if r["removed"]
               else ("OPAQUE" if not r["can_open"] else "none"))
        print("  {0:<30} {1:5.2f} {2:5.2f} {3:5.2f} {4:>4} {5:>8} {6:>7}  {7}"
              .format(r["label"], r["ln"], r["wd"], r["roof"], r["n_occ"],
                      r["src"], vis, ",".join(r["tags"])))
    print("\n  _ext = as parked (glass opened where possible)")
    print("  _cut = body hidden, seated figure alone (the only view of an "
          "occupant in a painted-glass car)")
    print("=" * 82 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)

            def _oblique(cx, cy, ln, roof, tgt_h):
                d = max(4.5, ln * 1.35)
                _snaps.place_camera(
                    stage,
                    ((cx - d * 0.5) * ssf, (cy - d) * ssf,
                     max(roof * 1.1, 2.0) * ssf),
                    (cx * ssf, cy * ssf, tgt_h * ssf), focal_mm=30.0)

            for r in rows:
                cx, cy, ln, roof = r["x"], r["y"], r["ln"], r["roof"]
                # Exterior: the car as it parks.
                _oblique(cx, cy, ln, roof, roof * 0.55)
                _snaps.snapshot(os.path.join(
                    SNAP_DIR, "{0}_ext.png".format(r["label"])))
                # Cutaway: hide the car body, shoot the seated figure alone.
                pp = cars[r["i"]].get("prim_path")
                pr = stage.GetPrimAtPath(pp) if pp else None
                toggled = bool(pr and pr.IsValid() and pr.SetActive(False))
                for _ in range(4):
                    app.update()
                _oblique(cx, cy, max(ln, 4.0), roof, roof * 0.45)
                _snaps.snapshot(os.path.join(
                    SNAP_DIR, "{0}_cut.png".format(r["label"])))
                if toggled:
                    pr.SetActive(True)
                    for _ in range(2):
                        app.update()

            # One plumb overview of the whole grid.
            gx = (ncols - 1) * DX_M / 2.0
            gy = (nrows - 1) * DY_M / 2.0
            span = max(ncols * DX_M, nrows * DY_M) * 1.2
            _snaps.overview(stage, (gx, gy), span,
                            os.path.join(SNAP_DIR, "grid.png"), ssf)
            print("[bench] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[bench] snapshots FAILED: {0}".format(exc))

    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
