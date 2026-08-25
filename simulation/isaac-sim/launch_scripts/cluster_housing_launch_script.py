#!/usr/bin/env python
"""
Cluster-housing bench — attached rows around a shared parking court.

    ISAAC_SIM_SCRIPT_NAME=cluster_housing_launch_script.py \
    SCENE_CONFIG=suburb_wildfire airstack up isaac-sim

WHAT THIS IS, AND WHY IT IS NOT THE SUBURB
------------------------------------------
The plat this project generates is DETACHED suburbia: every dwelling gets its
own lot, its own driveway, a side yard on both flanks and a back garden. That
is one American morphology and it is not the only one. This bench is the other
common one — garden apartments / attached townhouses — where the dwellings are
packed onto a fraction of the ground and everything a detached lot keeps
private is instead SHARED: no back garden, no garage, no driveway per door,
and one parking court in the middle that every unit walks to.

It matters for this dataset rather than being an aesthetic choice. The two
fabrics fail differently in a fire and they are searched differently
afterwards:

  * Attached rows have NO defensible space between units. A detached plat
    loses houses in a scatter; a row loses the whole run, because the gap
    that would have stopped it is 2.5 m of shared wall instead of 12 m of
    lawn.
  * The parking court concentrates the vehicles. In a detached plat the cars
    are spread one per driveway; here they are all in one place, which is
    both the obvious refuge (bare asphalt, nothing on it to burn) and the
    obvious trap if the single access drive is blocked.
  * Everyone's route out is the same route. One drive serves the cluster, so
    a blockage on it strands every unit at once.

So this is a layout preview: the fabric only, on empty ground, with nothing
burnt and nobody placed. Judge the SHAPE — the spacing between units, whether
the court reads as shared rather than as somebody's drive, and whether the
walk from a front door to a car is one a person would actually take.

WHAT IS BUILT
-------------
    two rows of `terrace` units facing each other across the court, at
    `UNIT_GAP_M` between party walls (2.5 m is a real townhouse separation —
    close enough to read as attached, far enough that the kit's wall geometry
    does not interpenetrate)

    a shared court, double-loaded, with painted bays and a kerb

    ONE access drive at the west end, which is the whole point of the
    morphology and the thing that makes it dangerous

    footways from the court to every front door, and a landscaped strip
    behind each row where a detached plat would have put gardens

Knobs are at the top. Nothing here is procedural — it is a hand-composed
block, because what is being judged is one arrangement rather than a
distribution over arrangements.
"""

import math
import os
import random
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
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

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

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
SEED = int(os.environ.get("CLUSTER_SEED", "5"))
ARCH_DIR = os.environ.get(
    "ARCH_DIR", "/isaac-sim/AirStack/scene_gen/assets/archetypes")
SNAP_DIR = os.environ.get("SNAP_DIR", "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)

GRASS_MAT = "airstack://scene_gen/assets/materials/Grass_Cut.usda"
ROUGH_MAT = "airstack://scene_gen/assets/materials/Grass_Countryside.usda"
ASPHALT_MAT = ("airstack://scene_gen/assets/materials/megascans/"
               "Road_Asphalt.usda")

# ---------------------------------------------------------------------------
# THE LAYOUT. Every dimension here is a real one; the comments say which.
# ---------------------------------------------------------------------------
# `terrace` measures 12.9 x 22.6 m with its WIDTH on X and its DEPTH on Y, so
# a row runs along X and the units front along Y toward the court.
UNIT_STYLE = "terrace"
UNIT_W_M, UNIT_D_M = 12.9, 22.6
# Between party walls. Real attached townhouses share a wall outright; this
# kit's units are modelled as free-standing shells with their own end walls,
# so butting them together interpenetrates the geometry. 2.5 m is the closest
# that stays clean and still reads as a terrace rather than as detached.
UNIT_GAP_M = 2.5
UNITS_PER_ROW = 5

# The court. 2.7 x 5.5 m bays and a 7.0 m aisle are the same figures the park's
# refuge lot uses (`suburb_park.PARKING`), because they are the regulation ones
# and not a property of either scene.
BAY_W_M, BAY_D_M, AISLE_M = 2.7, 5.5, 7.0
COURT_PAD_M = 3.0          # kerb strip between the bays and the footway
# Distance from the court edge to the units' front walls: a footway, a planted
# strip and a doorstep. Under about 6 m the units read as fronting directly
# onto parked cars, which is what a badly-designed one does.
FRONT_SETBACK_M = 7.5
DRIVE_W_M = 8.0            # the single access drive


def _mat(stage, key, url):
    path = "/World/ground/materials/" + key
    prim = stage.DefinePrim(Sdf.Path(path))
    prim.GetReferences().AddReference(sg._join_asset_root(url, ""))
    prim.Load()
    return path


def _slab(stage, path, x0, y0, x1, y1, z, ssf, mat, col):
    sg._make_plane_mesh(stage, path, x0, y0, x1, y1, z, 3.0, ssf,
                        display_color=col, mat_prim_path=(mat or ""))


def _box(stage, path, cx, cy, w, d, h, ssf, col=(0.9, 0.9, 0.88), z0=0.0):
    hx, hy = w / 2.0, d / 2.0
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    pts = [Gf.Vec3f((cx + dx) * ssf, (cy + dy) * ssf, z0 * ssf)
           for (dx, dy) in corners]
    pts += [Gf.Vec3f((cx + dx) * ssf, (cy + dy) * ssf, (z0 + h) * ssf)
            for (dx, dy) in corners]
    faces = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
             (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    counts, idx = [], []
    for f in faces:
        counts.append(4)
        idx += list(f)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    m.CreateDisplayColorAttr([Gf.Vec3f(*col)])
    return m.GetPrim()


def build_ground_and_light(stage, ssf, span_x, span_y):
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground"))
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground/materials"))
    rough = _mat(stage, "rough", ROUGH_MAT)
    grass = _mat(stage, "grass", GRASS_MAT)
    asphalt = _mat(stage, "asphalt", ASPHALT_MAT)
    _slab(stage, "/World/ground/base", -span_x, -span_y, span_x, span_y,
          -0.02, ssf, rough, (0.21, 0.31, 0.15))
    _slab(stage, "/World/ground/lawn", -span_x * 0.8, -span_y * 0.8,
          span_x * 0.8, span_y * 0.8, 0.0, ssf, grass, (0.24, 0.36, 0.17))

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.76, 0.80, 0.87))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2300.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 35.0))
    return {"grass": grass, "rough": rough, "asphalt": asphalt}


def build_court(stage, ssf, mats, w, d):
    """The shared court: asphalt, two bay rows either side of one aisle."""
    _slab(stage, f"{PARENT}/court", -w / 2, -d / 2, w / 2, d / 2, 0.03, ssf,
          mats["asphalt"], (0.21, 0.23, 0.26))
    bays = []
    n = int((w - 2 * COURT_PAD_M) // BAY_W_M)
    x0 = -(n * BAY_W_M) / 2.0
    for row, sign in enumerate((-1.0, 1.0)):
        # Bays back onto the court's long edges, noses to the central aisle.
        y_out = sign * (d / 2.0)
        y_in = sign * (d / 2.0 - BAY_D_M)
        for i in range(n):
            bx = x0 + (i + 0.5) * BAY_W_M
            # The divider between bays, painted on the asphalt.
            _box(stage, f"{PARENT}/bay_{row}_{i}", x0 + i * BAY_W_M,
                 (y_out + y_in) / 2.0, 0.12, BAY_D_M, 0.02, ssf,
                 col=(0.93, 0.93, 0.90), z0=0.03)
            bays.append((bx, (y_out + y_in) / 2.0, 90.0 if sign > 0 else 270.0))
        # The end line the bays butt against.
        _box(stage, f"{PARENT}/bayend_{row}", 0.0, y_in, n * BAY_W_M, 0.12,
             0.02, ssf, col=(0.93, 0.93, 0.90), z0=0.03)
    return bays


def build_units(stage, ssf, arch, court_w, court_d):
    """Two facing rows, packed at `UNIT_GAP_M`, fronting the court."""
    key = "house_{0}_pristine".format(UNIT_STYLE)
    usd = (arch or {}).get(key)
    if not usd:
        print("[cluster] MISSING archetype {0}".format(key))
        return []
    pitch = UNIT_W_M + UNIT_GAP_M
    x0 = -(UNITS_PER_ROW - 1) * pitch / 2.0
    # Front wall stands `FRONT_SETBACK_M` off the court; the anchor is the
    # unit's centre, so add half its depth.
    y_off = court_d / 2.0 + FRONT_SETBACK_M + UNIT_D_M / 2.0
    placed = []
    # THE FRONTS FACE THE COURT, which is the whole idea of the morphology —
    # a row whose doors open onto the back field is just two terraces with a
    # car park behind them. The `terrace` kit unit carries its porch on the
    # -Y end of its own frame, so the SOUTH row (court to its north) needs
    # 180 and the north row needs 0. Read off the render: the porch and the
    # glazed doors should be the wall you see from the parking.
    for row, (sign, yaw) in enumerate(((-1.0, 180.0), (1.0, 0.0))):
        for i in range(UNITS_PER_ROW):
            x = x0 + i * pitch
            y = sign * y_off
            path = "{0}/unit_{1}_{2}".format(PARENT, row, i)
            prim = stage.DefinePrim(Sdf.Path(path), "Xform")
            if not prim.GetReferences().AddReference(usd):
                continue
            prim.Load()
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
            xf.AddRotateZOp().Set(float(yaw))
            placed.append((x, y, yaw, sign))
    return placed


def build_paving(stage, ssf, mats, units, court_w, court_d):
    """Footways from the court to every door, and the single access drive."""
    n = 0
    for (x, y, yaw, sign) in units:
        # A path from the court kerb to the front wall, on the unit's centre.
        y_court = sign * (court_d / 2.0)
        y_door = y - sign * (UNIT_D_M / 2.0)
        y0, y1 = sorted((y_court, y_door))
        _slab(stage, f"{PARENT}/walk_{n}", x - 1.1, y0, x + 1.1, y1, 0.025,
              ssf, mats["asphalt"], (0.62, 0.60, 0.57))
        n += 1
    # THE ONE WAY IN. A cluster like this is served by a single drive, which
    # is what concentrates both the parking and the risk.
    _slab(stage, f"{PARENT}/drive", -court_w / 2 - 34.0, -DRIVE_W_M / 2,
          -court_w / 2, DRIVE_W_M / 2, 0.03, ssf, mats["asphalt"],
          (0.21, 0.23, 0.26))
    # A landscaped strip behind each row, where a detached plat would have
    # put private gardens. Communal, and deliberately shallow.
    # `s`/`n`, not the signed number: a USD prim name cannot contain '-', so
    # f"green_{int(-1.0)}" is an illegal path and Sdf.Path rejects it.
    for tag, sign in (("s", -1.0), ("n", 1.0)):
        y_back = sign * (court_d / 2.0 + FRONT_SETBACK_M + UNIT_D_M + 5.0)
        _slab(stage, f"{PARENT}/green_{tag}",
              -(UNITS_PER_ROW * (UNIT_W_M + UNIT_GAP_M)) / 2.0,
              min(y_back, sign * (court_d / 2.0 + FRONT_SETBACK_M + UNIT_D_M)),
              (UNITS_PER_ROW * (UNIT_W_M + UNIT_GAP_M)) / 2.0,
              max(y_back, sign * (court_d / 2.0 + FRONT_SETBACK_M + UNIT_D_M)),
              0.005, ssf, mats["grass"], (0.26, 0.38, 0.18))
    return n


def main():
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
    rng = random.Random(SEED)

    court_w = UNITS_PER_ROW * (UNIT_W_M + UNIT_GAP_M) - UNIT_GAP_M
    court_d = 2.0 * BAY_D_M + AISLE_M
    span_x = court_w * 1.6
    span_y = (court_d / 2.0 + FRONT_SETBACK_M + UNIT_D_M) * 2.2

    mats = build_ground_and_light(stage, ssf, span_x, span_y)
    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in os.listdir(ARCH_DIR) if f.endswith(".usd")} \
        if os.path.isdir(ARCH_DIR) else {}

    bays = build_court(stage, ssf, mats, court_w, court_d)
    units = build_units(stage, ssf, arch, court_w, court_d)
    n_walk = build_paving(stage, ssf, mats, units, court_w, court_d)

    # Cars in the court, and two people walking to them, purely for scale —
    # a court reads as oversized until there is something car-shaped in it.
    pools = ss.AssetPools(config)
    resolver = sg._make_resolver(config)
    raw_c = ss._raw_pool(config, "cars")
    raw_h = ss._raw_pool(config, "humans")
    cars_res = pools.load_tagged(raw_c, "residential") or pools.load(raw_c)
    humans = pools.load_tagged(raw_h, "rigged") or pools.load(raw_h)
    glassy = frozenset(pools.load_tagged(raw_c, "glass_separable"))
    ctx = {"asset_pools": pools, "resolver": resolver, "glassy": glassy}

    car_pl = []
    if cars_res and bays:
        rng.shuffle(bays)
        for i, (bx, by, byaw) in enumerate(bays[:max(4, len(bays) // 2)]):
            usd = cars_res[rng.randrange(len(cars_res))]
            car_pl.append(ppl._car_placement(ctx, usd, bx, by, byaw, "court"))
        sg.apply_placements(stage, car_pl, PARENT + "/cars", ssf,
                            resolver=resolver, instance_categories=set())
    ppl_pl = []
    if humans:
        for (hx, hy, hyaw) in ((-6.0, 0.0, 20.0), (7.5, -2.0, 200.0),
                               (0.0, court_d / 2.0 + 3.0, 270.0)):
            usd = humans[rng.randrange(len(humans))]
            ppl_pl.append(ppl._human_placement(ctx, usd, hx, hy, 0.03, hyaw,
                                               "walk"))
        sg.apply_placements(stage, ppl_pl, PARENT + "/people", ssf,
                            resolver=resolver, instance_categories=set())

    app = omni.kit.app.get_app()
    for _ in range(30):
        app.update()

    print("\n" + "=" * 76)
    print("CLUSTER HOUSING — attached rows around a shared court")
    print("  units        {0} x {1} rows of `{2}` at {3:.1f} m between walls"
          .format(UNITS_PER_ROW, 2, UNIT_STYLE, UNIT_GAP_M))
    print("  court        {0:.1f} x {1:.1f} m, {2} bays, {3} car(s) parked"
          .format(court_w, court_d, len(bays), len(car_pl)))
    print("  frontage     {0:.1f} m court-edge to front wall, {1} footway(s)"
          .format(FRONT_SETBACK_M, n_walk))
    print("  access       ONE drive, {0:.1f} m wide, at the west end"
          .format(DRIVE_W_M))
    print("  NO private garden, garage or driveway — that is the morphology.")
    print("=" * 76 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            _snaps.overview(stage, (0.0, 0.0), span_y * 1.5,
                            os.path.join(SNAP_DIR, "plan.png"), ssf)
            _snaps.place_camera(stage, (-70.0 * ssf, -78.0 * ssf, 44.0 * ssf),
                                (0.0, 0.0, 3.0 * ssf), focal_mm=22.0)
            _snaps.snapshot(os.path.join(SNAP_DIR, "aerial.png"))
            _snaps.place_camera(stage, (-34.0 * ssf, -26.0 * ssf, 6.0 * ssf),
                                (10.0 * ssf, 6.0 * ssf, 3.0 * ssf),
                                focal_mm=24.0)
            _snaps.snapshot(os.path.join(SNAP_DIR, "eye.png"))
            print("[cluster] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            print("[cluster] snapshots FAILED: {0}".format(exc))

    omni.timeline.get_timeline_interface().play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
