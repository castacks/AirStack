#!/usr/bin/env python
"""
Assemble a 500 x 500 m suburb with a TORNADO TRACK cut across it — by
reference, with no live fracture and no settle.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    SCENE_CONFIG=suburb_tornado \
    ISAAC_SIM_SCRIPT_NAME=suburb_tornado_launch_script.py \
    airstack up isaac-sim

Bake the archetype library FIRST with `bake_tornado_archetypes_launch_script.py`
into the same `ARCH_DIR`. Nothing here is fractured or settled: the expensive
work is done once per (style, level) and this script references it.

WHAT THIS SCENE IS
------------------
The wind counterpart of `suburb_assemble_launch_script.py`, against the same
generator and the same assembly pattern. `tornado.jpeg` in the repo root is
the reference photograph, and everything below is one of the features in it:

  1. A CORRIDOR of structural damage, graded from roofs-stripped at the edge
     to swept slabs on the centreline, with intact green suburb either side.
  2. Everything BLOWN ONE WAY. Wrecked houses trail plank debris downtrack;
     fallen trees point the same way; the corridor between the lots is
     covered in boards that came from somewhere else.
  3. BROWN SCOURED GROUND, darkest on the centreline and fading out to the
     path edge, where the turf was peeled off.
  4. PALE debris, not dark. A tornado exposes the inside of buildings —
     framing, sheathing, the unpainted back of everything — and that
     photographs light against grass. A burn scar photographs dark. Getting
     that backwards is the fastest way to build the wrong disaster.
  5. Roads, driveways, pools and slabs COMING THROUGH INTACT. They are in the
     ground or full of water; the overlay sits under the asphalt on the z
     ladder so the streets stay legible straight through the track, which is
     exactly what the aerial shows.

NO PEOPLE, AND NO FIRE. Survivors are a later pass — nothing here calls
`disaster.people`. Nothing here calls `disaster.fire`, `damage.soot_materials`,
`damage.char_materials` or `damage.scorched_material` either; see
`disaster/wind_flow.py` for why every one of those is a fire pass with no
no-fire mode.

THE KIT FLAG IS LOAD-BEARING, TWICE
-----------------------------------
The scour overlay is a translucent OmniPBR whose opacity is a FRACTIONAL
CUTOUT, and RTX Real-Time discards fractional cutout unless
`/rtx/raytracing/fractionalCutoutOpacity` is on. It has to be a command-line
flag (below) AND re-asserted after the Pegasus environment stage is loaded,
because loading a stage with authored render settings resets the property to
its default. The symptom of missing either is "I don't see the ground at all".
Full account in the `build-wildfire-scenes` skill.

Env knobs:

    ARCH_DIR      archetype library (default `scene_gen/assets/archetypes_tornado`)
    SCENE_CONFIG  preset (default `suburb_tornado`)
    TOR_SEED      layout seed override for the damage draws (default 11)
    TOR_PLANKS    boards per wrecked house (default 140); 0 disables the field
    TOR_TRACK_PER100  boards per 100 m2 of corridor (default 1.6)
    TOR_GROUND    0 disables the mud overlay
    MUD_*         overlay tuning — see `disaster.tornado.knobs_from_env`
    SNAP_DIR      write viewport PNGs here (must be under the mounted log dir)
"""

import math
import os
import random
import sys
import time

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
from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import add_sky, get_stage_meters_per_unit      # noqa: E402
from scene_generator import resolve_sky                        # noqa: E402
import suburb_scene as ss                                      # noqa: E402
from suburb_scene import generate_suburb_on_stage              # noqa: E402
from compile_disaster import load_scene_config                 # noqa: E402
from disaster import ground, planks, wind_flow                 # noqa: E402
from disaster import tornado as tn                             # noqa: E402
from detail import modular_house as mh                         # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_tornado")
SEED = int(os.environ.get("TOR_SEED", "11"))
ARCH_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
N_PLANKS = int(os.environ.get("TOR_PLANKS", "140"))
TRACK_PER100 = float(os.environ.get("TOR_TRACK_PER100", "1.6"))
DO_GROUND = os.environ.get("TOR_GROUND", "1").strip().lower() not in (
    "0", "false", "no")
SNAP_DIR = os.environ.get("SNAP_DIR", "")
if SNAP_DIR:
    os.makedirs(SNAP_DIR, exist_ok=True)

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}

# WHICH LEVELS ARE YAWED TO THE TRACK RATHER THAN TO THE STREET.
#
# Every archetype is baked with its debris thrown toward local +X, so the yaw
# a reference is given decides which way its wreckage points. For a house that
# still reads as a house, the street wins — a building facing the wrong way is
# the most obvious defect a suburb can have, and its debris field is small
# enough that its direction is not what you are looking at. For a pile, the
# track wins, because nobody can tell which way a pile was facing and where
# its material went is the entire read.
_TRACK_YAWED = ("leveled", "swept")
_TREE_TRACK_YAWED = ("leaning", "fallen")


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0, instance=True):
    """Reference `usd` at a pose. Same helper as the wildfire assembly."""
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    # INSTANCE IT. The same archetype is referenced many times across the
    # plate, and un-instanced trees were the 186M-point wall that OOM'd Isaac
    # on the wildfire plat. Transform ops on the instance ROOT are still
    # legal; only authoring INSIDE the referenced content is forbidden, which
    # is why a recoloured row home has to opt out.
    if instance:
        prim.SetInstanceable(True)
    return True


def _toss(stage, prim_path, dx, dy, roll_deg, yaw_jitter_deg, lift_m=0.0):
    """Move and roll an already-placed prop — a car, a bin, a play structure.

    THE ONE THING A TORNADO DOES THAT A FIRE DOES NOT, applied to the things
    the assembly has already built. A burnt car is an ordinary car standing in
    a black landscape; a car in a track has been PUSHED, and often rolled onto
    its side. That is cheap to author here and impossible to bake into an
    archetype, because these are per-placement props rather than per-type.

    Rebuilds the op order rather than appending, for the same reason
    `vegetation.tip_tree` does: a prim placed by `apply_placements` carries
    translate + rotateZ + maybe scale, and appending a rotate to the end of
    that list applies it in the wrong frame — the prop orbits the origin
    instead of rolling in place.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    xf = UsdGeom.Xformable(prim)
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    xf.SetXformOpOrder([])
    xf.AddTranslateOp().Set(Gf.Vec3d(float(t[0]) + dx, float(t[1]) + dy,
                                     float(t[2]) + lift_m))
    xf.AddRotateZOp().Set(float(vals.get("rotateZ") or 0.0)
                          + float(yaw_jitter_deg))
    if abs(roll_deg) > 0.01:
        xf.AddRotateXOp().Set(float(roll_deg))
    sc = vals.get("scale")
    if sc is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(float(sc[0]), float(sc[1]), float(sc[2])))
    return True


def wait_for_stage(stage, timeout_s=20.0):
    app = omni.kit.app.get_app()
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if stage and stage.GetPrimAtPath("/World").IsValid():
            return True
        app.update()
    return False


def main():
    omni.timeline.get_timeline_interface().stop()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    pg.load_environment(ENV_URL)
    stage = omni.usd.get_context().get_stage()
    wait_for_stage(stage)
    for name in ("GroundPlane", "Environment"):
        p = stage.GetPrimAtPath("/World/" + name)
        if p and p.IsValid():
            p.SetActive(False)

    config = load_scene_config(SCENE_CONFIG)
    _, ssf = get_stage_meters_per_unit(stage)

    # 1) LAYOUT + CHEAP DETAIL, houses/trees returned as instances -----------
    t0 = time.time()
    binfo = {}
    placements = generate_suburb_on_stage(stage, config, parent_path=PARENT,
                                          scene_scale_factor=ssf,
                                          info_out=binfo, assembly=True)
    add_sky(stage, resolve_sky(config))
    houses = binfo.get("house_instances", [])
    trees = binfo.get("tree_instances", [])
    print("[tornado] layout in {0:.0f}s: {1} house + {2} tree instance(s)"
          .format(time.time() - t0, len(houses), len(trees)))

    if not os.path.isdir(ARCH_DIR):
        print("[tornado] ARCH_DIR {0} does not exist — run "
              "bake_tornado_archetypes_launch_script.py first".format(ARCH_DIR))
    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in (os.listdir(ARCH_DIR) if os.path.isdir(ARCH_DIR) else [])
            if f.endswith(".usd")}

    # 2) THE TRACK -----------------------------------------------------------
    tcfg = tn.resolve_cfg(config)
    region = tuple(binfo.get("region") or (-250, -250, 250, 250))
    span = max(region[2] - region[0], region[3] - region[1])
    # ONE rng for the field, a DIFFERENT one for the per-object draws. The
    # field's noise is numpy and the ladders are stdlib; sharing a generator
    # between them would make a re-tune of one silently re-roll the other.
    inten = tn.intensity_field(tcfg, region, np.random.default_rng(SEED + 23))
    throw_deg = float(tcfg["heading_deg"]) + float(tcfg["curl_deg"])
    drng = random.Random(SEED + 5)
    summ = tn.summarise(tcfg, region, np.random.default_rng(SEED + 23))
    print("[tornado] track {0:.0f} m wide toward {1:.0f} deg through "
          "({2:.0f}, {3:.0f}); {4:.1%} of the plate in the path, debris "
          "toward {5:.0f} deg".format(
              tcfg["width_m"], tcfg["heading_deg"], tcfg["origin_m"][0],
              tcfg["origin_m"][1], summ["in_path_frac"], throw_deg))

    # 3) REFERENCE A HOUSE ARCHETYPE PER INSTANCE ----------------------------
    fp_by_style = {e["style"]: max(e["w"], e["d"])
                   for e in ss.modular_catalogue(config)}
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/inst"))
    n_h = miss_h = 0
    htally = {}
    pal_jobs = []
    wrecks = []          # (x, y, footprint, intensity) for the plank field
    for i, h in enumerate(houses):
        it = float(inten(h["x"], h["y"]))
        level = tn.house_level_for_intensity(it, drng)
        htally[level] = htally.get(level, 0) + 1
        key = "house_{0}_{1}".format(h["style"], level)
        usd = arch.get(key) or arch.get("house_{0}_pristine".format(h["style"]))
        if not usd:
            miss_h += 1
            continue
        yaw = (throw_deg + drng.uniform(-14.0, 14.0)
               if level in _TRACK_YAWED else h["yaw"])
        # Row homes are recoloured per unit, and USD forbids authoring inside
        # an instance — so a terrace opts out of instancing. A few dozen
        # prims against a plate's worth; the trees are what the budget is for.
        _pal = h.get("palette")
        _recolour = bool(h.get("row")) and bool(_pal)
        _hp = "{0}/inst/h_{1}".format(PARENT, i)
        if _ref(stage, _hp, usd, h["x"], h["y"], yaw, ssf,
                instance=not _recolour):
            n_h += 1
            if _recolour:
                pal_jobs.append({"prim_path": _hp, "palette": _pal,
                                 "category": "house"})
            if level != "pristine":
                wrecks.append((h["x"], h["y"],
                               fp_by_style.get(h["style"], 12.0), it, level))
    if pal_jobs:
        try:
            n_pal = mh.apply_palette(stage, pal_jobs, PARENT)
            print("[tornado] row homes: {0} subset(s) recoloured across "
                  "{1} unit(s)".format(n_pal, len(pal_jobs)))
        except Exception as _exc:
            print("[tornado] row-home palette FAILED: {0}".format(_exc))

    # 4) REFERENCE A TREE ARCHETYPE PER INSTANCE -----------------------------
    n_t = miss_t = 0
    ttally = {}
    trng = random.Random(SEED + 71)
    for i, t in enumerate(trees):
        it = float(inten(t["x"], t["y"]))
        sp = t["species"]
        level = tn.tree_level_for_intensity(it, trng)
        ttally[level] = ttally.get(level, 0) + 1
        if level == "pristine":
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale, yaw = 0.01, t["yaw"]
        else:
            usd = arch.get("tree_{0}_{1}".format(sp, level))
            scale = 1.0
            # A FALLEN TRUNK POINTS SOMEWHERE, and that is the cheapest
            # directional cue in the scene. The archetype lies toward +X, so
            # the yaw IS the fall bearing. A wide jitter, because trees go
            # over in the direction their own rooting and neighbours allow as
            # much as in the direction the wind was going — a stand all
            # pointing within five degrees of each other reads as a logging
            # operation, not as windthrow.
            yaw = (throw_deg + trng.uniform(-38.0, 38.0)
                   if level in _TREE_TRACK_YAWED else t["yaw"])
        if not usd:
            miss_t += 1
            continue
        if _ref(stage, "{0}/inst/t_{1}".format(PARENT, i), usd,
                t["x"], t["y"], yaw, ssf, scale=scale):
            n_t += 1

    # 5) STREET FURNITURE, FENCES AND CARS IN THE CORRIDOR -------------------
    #
    # `damage.INCOMBUSTIBLE` IS THE WRONG LIST HERE and it is worth saying so
    # explicitly: it exists to say what does not BURN, and it contains
    # `streetlight` and `sign` because those come through a wildfire visibly
    # untouched. A tornado shears them off. Reusing the fire list would leave
    # a corridor of levelled houses with every street sign standing perfectly
    # upright in it — see `wind_flow.IMMOVABLE` for the list that is right.
    BLOWN = ("fence", "trash_can", "bin", "mailbox", "sign", "streetlight",
             "bench", "chair", "picnic_table", "table", "planter",
             "play_structure", "swing", "seesaw", "bus_stop", "cafe_set",
             "bike_rack", "goal", "basket", "hoop")
    frng = random.Random(SEED + 13)
    n_gone = n_car = 0
    for q in placements:
        path = q.get("prim_path")
        cat = str(q.get("category", ""))
        if not path:
            continue
        it = float(inten(float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0))))
        if it <= 0.10:
            continue
        if wind_flow.is_immovable(cat):
            continue
        if any(k in cat for k in BLOWN):
            # GONE, not damaged. There is no "leaning fence" archetype and
            # there does not need to be one: what a track leaves of a timber
            # fence is a line of post stubs and a lot of boards somewhere
            # else, and the boards are already being authored by the plank
            # field. Probability tracks intensity so the corridor edge keeps
            # partial runs, which is what makes the edge read.
            if frng.random() < min(0.97, 0.25 + 1.05 * it):
                pr = stage.GetPrimAtPath(path)
                if pr and pr.IsValid() and pr.SetActive(False):
                    n_gone += 1
        elif "car" in cat and it > 0.22:
            # PUSHED AND ROLLED. Vehicles strewn along a track are one of the
            # most legible features in the reference photograph, and they are
            # also the only prop in this scene that a viewer knows the correct
            # resting pose of — which is exactly why moving them is worth the
            # code.
            d = float(tcfg["throw_m"]) * (it ** 1.6) * frng.uniform(0.2, 0.9)
            a = math.radians(throw_deg + frng.uniform(-40.0, 40.0))
            roll = (frng.choice((-1.0, 1.0)) * frng.uniform(62.0, 118.0)
                    if frng.random() < min(0.75, it) else
                    frng.uniform(-9.0, 9.0))
            if _toss(stage, path, math.cos(a) * d * ssf,
                     math.sin(a) * d * ssf, roll,
                     frng.uniform(-70.0, 70.0),
                     lift_m=(0.7 * ssf if abs(roll) > 30.0 else 0.0)):
                n_car += 1
    print("[tornado] corridor: {0} fence/furniture prim(s) blown away, "
          "{1} car(s) pushed or rolled".format(n_gone, n_car))

    # 6) THE PLANK FIELD ------------------------------------------------------
    #
    # AUTHORED, NOT SIMULATED, and it is what carries the whole directional
    # read. See `disaster/planks.py` for why boards are boxes rather than
    # fracture output, and why they are merged into one mesh per stock class.
    n_boards = 0
    if N_PLANKS > 0:
        prng = random.Random(SEED + 77)
        specs = []
        for (hx, hy, fp, it, _lv) in wrecks:
            specs += planks.scatter_from_wreck(
                hx, hy, fp, it, throw_deg, float(tcfg["throw_m"]), prng,
                n_pieces=N_PLANKS)
        n_house_boards = len(specs)
        if TRACK_PER100 > 0.0:
            specs += planks.scatter_over_region(
                region, inten, throw_deg, prng, per_100m2=TRACK_PER100,
                cell_m=10.0)
        pmats = planks.materials(stage, PARENT)
        made = planks.build(stage, PARENT + "/debris", specs, pmats, ssf)
        n_boards = len(specs)
        print("[tornado] plank field: {0} board(s) ({1} off {2} wreck(s), "
              "{3} across the corridor) in {4} mesh(es)".format(
                  n_boards, n_house_boards, len(wrecks),
                  n_boards - n_house_boards, len(made)))

    # 7) THE GROUND SCOUR -----------------------------------------------------
    made_g = []
    if DO_GROUND:
        zs = float(binfo.get("z_scale") or ss.ground_z_scale(config, region))
        # BETWEEN GRASS AND ASPHALT on the z ladder, exactly where the burn
        # scar sits and for the same reason: roads, drives and walks then come
        # through the track UNSCOURED, which is correct (they are pavement)
        # and is what makes the streets legible straight across the corridor
        # in the reference photograph.
        mud_z = (ss._Z_GRASS + 0.5 * (ss._Z_ASPHALT - ss._Z_GRASS)) * zs
        kn = tn.knobs_from_env(span)
        cov = tn.scour_coverage(tcfg, region,
                                np.random.default_rng(SEED + 31),
                                intensity=inten, gamma=kn["gamma"],
                                islands=kn["islands"])
        made_g = ground.build_overlay(
            stage, cov, region, ssf, mud_z, material_parent=PARENT,
            root="/World/scourGround", cell_m=kn["cell_m"], bands=kn["bands"],
            tile_m=kn["tile_m"], op_range=kn["op_range"],
            texture=tn.MUD_TEXTURE,
            # You cannot scour water. A pool is a hole in the lawn with water
            # below grade and an overlay cell drawn over it is a film floating
            # on the surface — the same exclusion the burn scar makes.
            skip=ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0))

    # 8) RE-ASSERT THE CUTOUT FLAG, AFTER THE STAGE LOAD ---------------------
    #
    # THE COMMAND-LINE FLAG IS NOT ENOUGH IN A SCENE THAT LOADS A STAGE. Kit
    # maps the flag onto a render-settings USD property at ~12 s; the Pegasus
    # environment stage is then loaded, and loading a stage with authored
    # render settings RESETS that property to its default (OFF) — so the
    # overlay comes out fully transparent with the flag plainly correct on the
    # command line. Set THIS late it does push onto the live property.
    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(30):
        omni.kit.app.get_app().update()

    rw, rh = region[2] - region[0], region[3] - region[1]
    print("\n" + "=" * 72)
    print("TORNADO TRACK ASSEMBLED  {0:.0f} x {1:.0f} m  (by reference)"
          .format(rw, rh))
    print("  total       {0:.0f} s".format(time.time() - t0))
    print("  track       {0:.0f} m wide toward {1:.0f} deg, {2:.1%} of the "
          "plate".format(tcfg["width_m"], tcfg["heading_deg"],
                         summ["in_path_frac"]))
    print("  houses      {0} referenced ({1} missing); {2}".format(
        n_h, miss_h, ", ".join("%s=%d" % kv for kv in sorted(htally.items()))))
    print("  trees       {0} referenced ({1} missing); {2}".format(
        n_t, miss_t, ", ".join("%s=%d" % kv for kv in sorted(ttally.items()))))
    print("  debris      {0} board(s); {1} prop(s) blown away, {2} car(s) "
          "moved".format(n_boards, n_gone, n_car))
    print("  scour       {0} band(s) of Soil_Mud".format(len(made_g)))
    if miss_h or miss_t:
        print("  !! MISSING ARCHETYPES — bake them with "
              "bake_tornado_archetypes_launch_script.py into {0}"
              .format(ARCH_DIR))
    if DO_GROUND and not made_g:
        print("  !! NO SCOUR BANDS. Either the track misses the plate or the "
              "coverage is zero everywhere — check tools/tornado_png.py")
    print("=" * 72 + "\n")

    # SNAPSHOTS. Nobody is at the GUI on a scripted run and a scene this size
    # can only be judged by looking at it, so the launcher photographs its own
    # result. `SNAP_DIR=` turns it on and MUST sit under the mounted log
    # directory or the PNGs are written somewhere the host cannot read.
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots",
                                                 os.path.normpath(_sp))
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            _snaps.overview(stage, (0.5 * (region[0] + region[2]),
                                    0.5 * (region[1] + region[3])),
                            max(rw, rh),
                            os.path.join(SNAP_DIR, "overview.png"), ssf)
            # ALONG THE TRACK, NOT AT A LIST OF LANDMARKS. What has to be
            # judged here is a gradient across a corridor, so the subjects are
            # points sampled along the centreline plus one on each flank — a
            # top-down and an oblique of each shows the ladder in order.
            _tt, (ux, uy), (vx, vy) = tn.frame(tcfg)
            ox, oy = tcfg["origin_m"]
            _pts = {}
            for k, s in enumerate((-0.28, 0.0, 0.28)):
                a = s * max(rw, rh)
                _pts["core_%d" % k] = (ox + ux * a, oy + uy * a)
            half = 0.5 * float(tcfg["width_m"])
            _pts["edge_left"] = (ox + vx * half * 0.95, oy + vy * half * 0.95)
            _pts["edge_right"] = (ox - vx * half * 0.95, oy - vy * half * 0.95)
            if wrecks:
                w0 = max(wrecks, key=lambda w: w[3])
                _pts["worst_wreck"] = (w0[0], w0[1])
            _snaps.views_around(stage, _pts, SNAP_DIR, ssf)
            print("[tornado] snapshots -> {0} ({1} subject(s))"
                  .format(SNAP_DIR, len(_pts)))
        except Exception as _exc:
            print("[tornado] snapshots FAILED: {0}".format(_exc))

    app = omni.kit.app.get_app()
    omni.timeline.get_timeline_interface().play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
