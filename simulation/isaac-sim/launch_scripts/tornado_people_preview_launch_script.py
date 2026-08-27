#!/usr/bin/env python
"""
The tornado PEOPLE bench — every placement type as its own set-piece, side by
side on open ground, with the camera aimed at each one.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/people_bench \
    ISAAC_SIM_SCRIPT_NAME=tornado_people_preview_launch_script.py \
    airstack up isaac-sim

WHY A BENCH AND NOT THE FULL SCENE
----------------------------------
Every one of these placements is a claim about what a drone can SEE, and one
of them — `trapped_partial` — is a claim that cannot be checked any other way.
A figure that is meant to be mostly buried and partly visible is either right
or worthless, and the difference is a few centimetres of sink and where a
handful of boards landed. In the assembled 500 m scene that figure is one of
ninety, somewhere in a corridor, at whatever altitude the overview camera
happens to be at. Here it is the subject of its own photograph.

This is the same argument `burn_ground_preview_launch_script.py` makes for the
ground scar, and it was made after four approaches had each been diagnosed
slowly inside a twenty-minute full-block build.

IT RUNS THE REAL PLANNER
------------------------
Each unit builds a synthetic `ctx` — one wreck, some cars, a few road points —
and calls `tornado_people.plan_people` with the shares zeroed except for the
scenario under test. So what is photographed is the code the assembly runs,
not a second implementation that agrees with it today and drifts tomorrow. A
bench that reimplements the thing it is checking proves nothing.

THE UNITS

    A  on_the_rubble      survivors standing and walking on a levelled house,
                          one of them signalling
    B  trapped_partial    the hard one: mostly under debris, head and torso
                          showing, with boards authored across the lower body
    C  neighbour_dig      3-6 working one collapsed house, all facing in
    D  vehicles           a car on its side, one on its roof, one nosed in,
                          and occupants standing at an upright one
    E  thrown + assisted  a prone figure downtrack, and a supported trio
    F  street             walkers on a carriageway strip

Env knobs:

    ARCH_DIR   archetype library (default `scene_gen/assets/archetypes_tornado`)
    PEOPLE_SEED  rng seed (default 5)
    SNAP_DIR   viewport PNGs; MUST be under the mounted log directory
    UNITS      comma-separated subset, e.g. `UNITS=B,D` (default all)
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

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
import suburb_scene as ss                                      # noqa: E402
from suburb_scene import AssetPools, _raw_pool                 # noqa: E402
from compile_disaster import load_scene_config                 # noqa: E402
from disaster import planks                                    # noqa: E402
from disaster import tornado as tn                             # noqa: E402
from disaster import tornado_people as tpp                     # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_tornado")
SEED = int(os.environ.get("PEOPLE_SEED", "5"))
ARCH_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
SNAP_DIR = os.environ.get("SNAP_DIR", "")
UNITS = set(u.strip().upper() for u in
            os.environ.get("UNITS", "A,B,C,D,E,F").split(",") if u.strip())

# Units on a grid, far enough apart that one unit's debris never lands in
# another's photograph. 60 m: the widest set-piece is the vehicle row at ~24 m
# and a thrown figure reaches 40 m from its wreck.
GRID_M = 60.0
_UNIT_XY = {"A": (0.0, 0.0), "B": (GRID_M, 0.0), "C": (2 * GRID_M, 0.0),
            "D": (0.0, GRID_M), "E": (GRID_M, GRID_M),
            "F": (2 * GRID_M, GRID_M)}


def build_ground_and_light(stage, ssf):
    """Open ground, a dome and a key light. Mud, because that is what the
    corridor floor is and a figure's contrast against it is half of what this
    bench exists to judge."""
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/bench_ground"))
    e = 260.0
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e * 3, -e, 0),
                            Gf.Vec3f(e * 3, e * 2, 0), Gf.Vec3f(-e, e * 2, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.34, 0.28, 0.17)])
    try:
        mat = tn_mud_material(stage)
        if mat:
            UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(mat)
    except Exception as exc:
        print("[bench] mud material unavailable: {0}".format(exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1000.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2400.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-48.0, 0.0, 35.0))
    del ssf


def tn_mud_material(stage):
    """The scour surface as a plain opaque material — no cutout, no bands.

    The assembled scene lays `Soil_Mud` as a TRANSLUCENT overlay in opacity
    bands, which needs the fractional-cutout Kit flag and a coverage field.
    Neither is wanted on a bench whose subject is the people: here it is just
    the floor, bound opaque, so nothing about the ground can be blamed for a
    figure that reads badly.
    """
    return planks.wood_material(stage, PARENT + "/BenchLooks/mud",
                                tile_m=6.0, tint=(0.52, 0.42, 0.28),
                                roughness=0.95)


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0):
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    return True


def _only(scenario, total, **over):
    """A `tornado_people` config with every share zeroed but one.

    This is what makes the bench exercise the real planner: rather than
    calling a private `_neighbour_dig` directly, each unit asks
    `plan_people` for a population that happens to be entirely one scenario.
    Same entry point, same ordering, same spacing rules as the assembly.
    """
    cfg = tpp.resolve_cfg({})
    cfg = dict(cfg)
    cfg["total"] = int(total)
    sc = {k: dict(v) for k, v in cfg["scenarios"].items()}
    for k in sc:
        sc[k]["share"] = 1.0 if k == scenario else 0.0
    if over:
        sc[scenario].update(over)
    cfg["scenarios"] = sc
    return cfg


def main():
    omni.timeline.get_timeline_interface().stop()
    ctx_usd = omni.usd.get_context()
    ctx_usd.new_stage()
    stage = ctx_usd.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    _, ssf = get_stage_meters_per_unit(stage)
    build_ground_and_light(stage, ssf)

    config = load_scene_config(SCENE_CONFIG)
    resolver = sg._make_resolver(config)
    pools = AssetPools(config)
    raw_h = _raw_pool(config, "humans")
    rigged = pools.load_tagged(raw_h, "rigged")
    if not rigged:
        posed = pools.load_tagged(raw_h, "posed_standing")
        rigged = [u for u in pools.load(raw_h) if u not in posed]
    raw_c = _raw_pool(config, "cars")
    car_usds = pools.load_tagged(raw_c, "residential") or pools.load(raw_c)
    print("[bench] {0} rigged human asset(s), {1} car asset(s)"
          .format(len(rigged), len(car_usds)))

    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in (os.listdir(ARCH_DIR) if os.path.isdir(ARCH_DIR) else [])
            if f.endswith(".usd")}
    if not arch:
        print("[bench] !! ARCH_DIR {0} has no archetypes — the wrecks will be "
              "missing and only the people will show".format(ARCH_DIR))

    base_ctx = {"humans": rigged, "resolver": resolver, "asset_pools": pools,
                "throw_deg": 58.0}
    rng = random.Random(SEED)
    all_humans, all_debris, all_records = [], [], []
    subjects = {}

    def wreck_at(x, y, level, style="cottage", fp=14.0):
        """Reference a wrecked-house archetype and return its planner dict."""
        key = "house_{0}_{1}".format(style, level)
        usd = arch.get(key)
        if usd:
            _ref(stage, "{0}/wreck_{1:.0f}_{2:.0f}".format(PARENT, x, y), usd,
                 x, y, rng.uniform(0.0, 360.0), ssf)
        return {"x": x, "y": y, "fp": fp, "intensity": 0.85, "level": level}

    def run(unit, scenario, ctx_extra, total, at=None, **over):
        """Plan one unit with the real planner and collect its output.

        `at` is where the CAMERA goes. It defaults to the unit's own grid cell,
        but a scenario that deliberately places away from that cell has to say
        so — the assisted trio stands on a road stub 20 m off, and a camera
        left on the cell centre photographs empty mud beside it.
        """
        if unit not in UNITS:
            return
        cfg = _only(scenario, total, **over)
        c = dict(base_ctx)
        c.update(ctx_extra)
        h, d, r = tpp.plan_people(cfg, c, random.Random(SEED + ord(unit)))
        all_humans.extend(h)
        all_debris.extend(d)
        for rec in r:
            rec["unit"] = unit
        all_records.extend(r)
        # AIM AT WHAT WAS ACTUALLY PLACED. Better still than the declared
        # point: the centroid of the figures this unit produced, so the frame
        # is centred on the subject even when the planner scattered it.
        if h:
            subjects["%s_%s" % (unit, scenario)] = (
                sum(float(q["x_m"]) for q in h) / len(h),
                sum(float(q["y_m"]) for q in h) / len(h))
        elif at is not None:
            subjects["%s_%s" % (unit, scenario)] = at
        print("[bench] unit {0}  {1:<16} {2} figure(s), {3} plank spec(s)"
              .format(unit, scenario, len(h), len(d)))

    # ---- A: survivors on the rubble of a levelled house -------------------
    ax, ay = _UNIT_XY["A"]
    if "A" in UNITS:
        w = wreck_at(ax, ay, "leveled", "wide_house", 16.0)
        run("A", "on_the_rubble", {"wrecks": [w], "intact": []}, 4,
            per_wreck=[4, 4])

    # ---- B: the hard one — partially buried --------------------------------
    bx, by = _UNIT_XY["B"]
    if "B" in UNITS:
        w = wreck_at(bx, by, "partial_collapse", "two_storey", 15.0)
        # THREE OF THEM, at different sink fractions across the allowed range,
        # so one photograph shows the shallow, middle and deep cases together
        # and the band can be judged rather than guessed at from one sample.
        run("B", "trapped_partial", {"wrecks": [w] * 3, "intact": []}, 3,
            sink_frac=[0.28, 0.58], planks_over=[5, 9])

    # ---- C: the dig cluster ------------------------------------------------
    cx, cy = _UNIT_XY["C"]
    if "C" in UNITS:
        w = wreck_at(cx, cy, "leveled", "ranch", 18.0)
        run("C", "neighbour_dig",
            {"wrecks": [w], "intact": [(cx + 26.0, cy + 8.0)]}, 6,
            group_size=[6, 6])

    # ---- D: vehicles, one of each pose ------------------------------------
    dx, dy = _UNIT_XY["D"]
    cars_ctx = []
    if "D" in UNITS and car_usds:
        # roll, pitch, label — the three tipped poses plus one merely shoved,
        # which is the majority case and the control.
        # Three tipped poses plus TWO merely shoved. The shoved pair are the
        # control and the majority case — Paulikas' rates put roughly two
        # thirds of displaced vehicles upright even on the centreline — and
        # `in_vehicle` needs an upright car to stand its occupants beside.
        poses = ((92.0, 0.0, "on its side"), (178.0, 0.0, "on its roof"),
                 (14.0, 58.0, "nosed in"), (4.0, 0.0, "shoved"),
                 (-3.0, 0.0, "shoved"))
        for i, (roll, pitch, label) in enumerate(poses):
            x = dx - 13.0 + i * 6.5
            usd = car_usds[i % len(car_usds)]
            q = [{"usd": usd, "x_m": x, "y_m": dy, "z_m": 0.0,
                  "yaw_deg": rng.uniform(0.0, 360.0), "roll_deg": 0.0,
                  "pitch_deg": 0.0, "scale": pools.scale_of(usd),
                  "category": "car",
                  "axis_up": pools.axis_of(usd)}]
            sg.apply_placements(stage, q, "{0}/car_{1}".format(PARENT, i),
                                ssf, resolver=resolver,
                                instance_categories=set())
            path = q[0].get("prim_path")
            if path:
                # THE SAME `toss_prim` THE ASSEMBLY USES — it measures the
                # prop's own bounding box and seats it, which is the thing
                # this unit exists to confirm. A car that floats or sinks
                # here floats or sinks in the scene.
                tn.toss_prim(stage, path, 0.0, 0.0, roll,
                             rng.uniform(-30.0, 30.0), pitch_deg=pitch)
            print("[bench]   car {0}: {1}".format(i, label))
            cars_ctx.append({"x": x, "y": dy,
                             "toppled": abs(roll) > 30.0 or abs(pitch) > 30.0})
        run("D", "in_vehicle", {"cars": cars_ctx}, 3, per_car=[1, 2])

    # ---- E: thrown, and the assisted trio ----------------------------------
    ex, ey = _UNIT_XY["E"]
    if "E" in UNITS:
        w = wreck_at(ex, ey, "swept", "cottage", 12.0)
        run("E", "thrown", {"wrecks": [w] * 2, "intact": []}, 2,
            max_count=2, range_m=[12.0, 26.0])
        run("E", "assisted",
            {"road_pts": [(ex + 16.0, ey - 12.0, 38.0)]}, 3,
            at=(ex + 16.0, ey - 12.0))

    # ---- F: the street -----------------------------------------------------
    fx, fy = _UNIT_XY["F"]
    if "F" in UNITS:
        strip = UsdGeom.Mesh.Define(stage, Sdf.Path(PARENT + "/road_strip"))
        hw, hl = 4.0, 22.0
        pts = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
        strip.CreatePointsAttr([Gf.Vec3f((fx + px) * ssf, (fy + py) * ssf,
                                         0.03 * ssf) for (px, py) in pts])
        strip.CreateFaceVertexCountsAttr([4])
        strip.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        strip.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
        strip.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.18, 0.19)])
        rp = [(fx + t, fy + rng.uniform(-2.0, 2.0), 0.0)
              for t in range(-18, 19, 6)]
        run("F", "street", {"road_pts": rp}, 6, groups=[1, 1])

    # ---- author everything the planner produced ----------------------------
    n_people = 0
    if all_humans:
        sg.apply_placements(stage, all_humans, PARENT + "/people", ssf,
                            resolver=resolver, instance_categories=set())
        n_people = len(all_humans)
    n_boards = 0
    if all_debris:
        # THE BOARDS THAT MAKE `trapped_partial` PARTIAL. Authored through the
        # same `planks` path the scene's debris field uses, so they take the
        # same sawn-timber material and read as the same material.
        specs = []
        for d in all_debris:
            specs.append({"x": d["x"], "y": d["y"], "z": d["z"],
                          "l": d["len"], "w": d["wide"],
                          "t": random.Random(SEED).uniform(0.02, 0.05),
                          "yaw": d["yaw"],
                          "pitch": rng.uniform(-8.0, 8.0),
                          "roll": rng.uniform(-10.0, 10.0),
                          "class": "sheathing" if d["wide"] > 0.4 else "board"})
        pmats = planks.materials(stage, PARENT)
        planks.build(stage, PARENT + "/trap_debris", specs, pmats, ssf)
        n_boards = len(specs)

    for _ in range(20):
        omni.kit.app.get_app().update()

    summ = tpp.summarise(all_records)
    print("\n" + "=" * 72)
    print("TORNADO PEOPLE BENCH   units: {0}".format(",".join(sorted(UNITS))))
    print("  people      {0} authored".format(n_people))
    print("  by scenario {0}".format(
        ", ".join("%s=%d" % kv for kv in sorted(summ["by_scenario"].items()))))
    print("  visibility  {0}".format(
        ", ".join("%s=%d" % kv for kv in sorted(summ["by_visibility"].items()))))
    print("  boards      {0} laid over trapped figures".format(n_boards))
    print("  LOOK FOR:")
    print("    A  do figures stand ON the debris, not through or above it?")
    print("    B  is the head AND torso of each trapped figure visible from")
    print("       straight above, with the lower body genuinely hidden?")
    print("    C  does the ring read as purposeful — everyone facing in?")
    print("    D  is each car SEATED — no float, no sink — in all four poses?")
    print("    E  is the prone figure legible against the mud at altitude?")
    print("    F  do the walkers read as walking rather than as standing?")
    print("=" * 72 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots",
                                                 os.path.normpath(_sp))
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            # TIGHTER THAN THE SCENE'S DEFAULTS. The subject here is a figure
            # about 1.8 m tall, not a 500 m plate: `views_around`'s 60 m
            # top-down would put a person at a handful of pixels, which is
            # precisely the condition this bench exists to avoid judging from.
            _snaps.views_around(stage, subjects, SNAP_DIR, ssf,
                                top_h=26.0, obl_dist=20.0, obl_h=11.0)
            print("[bench] snapshots -> {0} ({1} subject(s))"
                  .format(SNAP_DIR, len(subjects)))
        except Exception as _exc:
            print("[bench] snapshots FAILED: {0}".format(_exc))

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
