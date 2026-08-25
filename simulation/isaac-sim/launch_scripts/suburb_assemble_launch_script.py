#!/usr/bin/env python
"""
Assemble the 1600 x 1200 burnt plat by REFERENCE — no live fracture or settle.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes \
    SCENE_CONFIG=suburb_wildfire \
    ISAAC_SIM_SCRIPT_NAME=suburb_assemble_launch_script.py airstack up isaac-sim

`generate_suburb_on_stage(assembly=True)` builds the CHEAP layer live — streets,
ground, driveways, walks, fences, props, ground scar — and hands back each
house's (style, pose) and each tree's (species, pose) instead of building their
geometry. This script then references a pre-baked damage archetype
(`bake_archetypes_launch_script.py`) for every house and tree at the damage
level the fire gives it. The whole plat's collapse is thus O(reference), not
O(fracture+settle) — the scaling path the mini bake proved at 255x.

SURVIVORS. `disaster.people` plans ~60 live people into the six situations
search-and-rescue after-action reports actually find them in (see that
module's docstring for the Camp Fire / Lahaina numbers the shares come from),
plus the cars of a gridlocked egress queue and whatever blocked it. THE ORDER
IS LOAD-BEARING: the queue's cars go in BEFORE step 4b so the burnable/scorch
pass chars them like every other car on the plat, and the PEOPLE go in after
it, because a survivor is not scorched. Ground truth lands in
``PEOPLE_JSON`` (default ``$ARCH_DIR/humans_<seed>.json``).
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

enable_extension("omni.flowusd")
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
import scene_generator as sg                                  # noqa: E402
from scene_prep import add_sky, get_stage_meters_per_unit     # noqa: E402
from scene_generator import resolve_sky                       # noqa: E402
import suburb_scene as ss                                     # noqa: E402
from suburb_scene import generate_suburb_on_stage             # noqa: E402
from compile_disaster import load_scene_config                # noqa: E402
from disaster import bake, damage, fire, ground               # noqa: E402
from detail import modular_house as mh                        # noqa: E402
from disaster import people as ppl                            # noqa: E402
from disaster import vegetation as veg                        # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
SEED = int(os.environ.get("MINI_SEED", "11"))
ARCH_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes"))
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
PEOPLE_JSON = os.environ.get(
    "PEOPLE_JSON", os.path.join(ARCH_DIR, "humans_{0}.json".format(SEED)))
# PEOPLE_POLES=1 authors the magenta group markers (see build_people_poles).
POLES = os.environ.get("PEOPLE_POLES", "").strip().lower() in ("1", "true", "yes")
# Where the launcher photographs itself. Empty = no captures.
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


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0, instance=True):
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    # INSTANCE IT. The same archetype (or green species usd) is referenced
    # many times across the plat; without instancing, N copies cost N x the
    # geometry and the trees alone OOM'd Isaac at ~186M points. The transform
    # ops above sit on the instance ROOT, which instancing still allows —
    # only editing INSIDE the referenced content is forbidden, which the
    # assembly never does.
    # NOT ALWAYS INSTANCEABLE. Instancing is what keeps 9k trees inside VRAM,
    # and it is also a hard bar on authoring INSIDE the referenced content —
    # which is exactly what a palette rebind does. A caller that means to
    # recolour a building has to opt out, and can afford to: it is a few dozen
    # prims against nine thousand.
    if instance:
        prim.SetInstanceable(True)
    return True


# A locator pole per survivor GROUP. Sixty people over 1600 x 1200 m are
# individually invisible until you are almost on top of them, so finding them
# to judge them costs more than judging them does. One pole per group (not per
# person — a group is the thing you fly to), 25 m so it clears the houses and
# every tree, and magenta because nothing else in a burn scar is.
#
# EVERY POLE HANGS OFF ONE SCOPE so the whole set switches off with a single
# prim. This is a LOOKING aid and does not belong in a capture: it is authored
# only when PEOPLE_POLES is set, and it carries no semantic label, so a run
# that forgets to disable it still produces no annotation for it.
POLE_H_M = 25.0
POLE_R_M = 0.35
POLE_SCOPE = "/_people_poles"


ROW_POLE_SCOPE = "/_rowhome_poles"


def build_row_poles(stage, clusters, ssf):
    """One CYAN pole at each row-home court. Returns the count.

    Same argument as the survivor poles and a different colour so the two
    cannot be confused: 62 row units on a 1600 x 1200 m plat are perfectly
    present and completely unfindable, and "I don't see any row homes" is what
    that looks like from the cockpit. Its own scope, so it switches off
    independently of the people markers.
    """
    root = PARENT + ROW_POLE_SCOPE
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    n = 0
    for i, c in enumerate(clusters or ()):
        pk = (c or {}).get("parking") or {}
        ctr = pk.get("centre") or c.get("centre")
        if not ctr:
            continue
        cyl = UsdGeom.Cylinder.Define(
            stage, Sdf.Path("{0}/row_{1}".format(root, i)))
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(POLE_H_M * ssf)
        cyl.CreateRadiusAttr(POLE_R_M * ssf)
        cyl.AddTranslateOp().Set(Gf.Vec3d(float(ctr[0]) * ssf,
                                          float(ctr[1]) * ssf,
                                          POLE_H_M * 0.5 * ssf))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(0.0, 0.95, 1.0)])
        n += 1
        print("[assemble]   row-home court {0} at ({1:.0f}, {2:.0f}), "
              "{3} bay(s)".format(i, ctr[0], ctr[1], len(pk.get("bays") or [])))
    return n


def build_people_poles(stage, recs, ssf):
    """One magenta pole at each group's centroid. Returns the pole count."""
    groups = {}
    for r in recs:
        groups.setdefault((r["scenario"], int(r.get("group", 0))), []).append(r)
    root = PARENT + POLE_SCOPE
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    n = 0
    for (scenario, gi), members in sorted(groups.items()):
        cx = sum(float(m["x"]) for m in members) / len(members)
        cy = sum(float(m["y"]) for m in members) / len(members)
        path = "{0}/pole_{1}_{2}".format(root, _sanitize(scenario), gi)
        cyl = UsdGeom.Cylinder.Define(stage, Sdf.Path(path))
        cyl.CreateAxisAttr("Z")
        cyl.CreateHeightAttr(POLE_H_M * ssf)
        cyl.CreateRadiusAttr(POLE_R_M * ssf)
        # Cylinders are authored centred on their origin.
        cyl.AddTranslateOp().Set(
            Gf.Vec3d(cx * ssf, cy * ssf, POLE_H_M * 0.5 * ssf))
        cyl.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.0, 0.85)])
        # An unlit-looking marker reads at any time of day and in smoke.
        UsdGeom.Gprim(cyl).CreateDisplayOpacityAttr([1.0])
        n += 1
        print("[assemble]   pole {0:<16} group {1}  {2} person(s)  "
              "({3:.0f}, {4:.0f})".format(scenario, gi, len(members), cx, cy))
    return n


def _sanitize(name):
    return "".join(c if c.isalnum() else "_" for c in str(name))


# `Burnt_Forest_Floor` IS DELIBERATELY NOT USED ON TREES OR LOGS any more.
# It is a photographed GROUND surface; wrapped round a trunk or a log it
# reads as ground standing up. `veg.char_bole` records the same finding on
# a standing bole and rejected it there too. Generated log debris takes
# `veg.bark_material` (real bark, dark-tinted for a burnt scene); the ground
# scar still uses the surface, which is what it is for.


def _load_burnt_wood(stage):
    """The material for generated log debris: BARK, not burnt ground.

    Was a reference to `Burnt_Forest_Floor`, chosen because a flat
    colour "has no normal or ORM map, so a cylinder lit by one sun
    reads as painted pipe". The reasoning was right and the surface was
    wrong: it is a photographed GROUND texture, and `veg.char_bole`
    records the same experiment on a standing trunk and rejected it —
    ground wrapped round a log reads as ground. `veg.bark_material` is
    real oak bark at 4K WITH a normal map, tinted dark here because
    this is a burnt scene; a charred log is charred BARK.
    """
    return veg.bark_material(stage, PARENT + "/BurnLooks/log_bark",
                             tile_m=1.7, tint=(0.30, 0.26, 0.23))


def _tube(stage, path, p0, p1, r0, r1, ssf, sides=8):
    """One log or limb of the blockage — see `veg.log_mesh`.

    The private implementation this replaces authored the barrel quads
    and NOTHING ELSE, so every piece was an open tube you could see
    straight down the inside of, and it was a mathematically exact
    cylinder — the one shape nothing in a forest has. Together those
    are why a fallen log read as a hollow pipe rather than as a trunk
    that broke. `log_mesh` caps both ends and jitters the girth.
    """
    return veg.log_mesh(stage, path, p0, p1, r0, r1, ssf,
                        sides=max(7, int(sides) + 1),
                        rng=random.Random(abs(hash(path)) % 99991))


def _place_debris(stage, spec, ssf, i, mat):
    """The strewn field that makes a blockage impassable.

    `people._blocker_debris` decides WHERE (it is a planner and touches no
    stage); this authors it. Bound to the photographed charred surface rather
    than a flat dark PBR — a plain colour has no normal or ORM map, so a
    cylinder lit by one sun reads as painted pipe.
    """
    n = 0
    for j, d in enumerate(spec.get("debris") or ()):
        prim = _tube(stage, "{0}/inst/blockdeb_{1}_{2}".format(PARENT, i, j),
                     d["p0"], d["p1"], d["r0"], d["r1"], ssf,
                     sides=8 if d["kind"] == "log" else 6)
        if mat is not None:
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
        n += 1
    return n


def _place_blocker(stage, spec, ssf, i):
    """Put a blockage across the carriageway. See `people._add_blocker`.

    TWO KINDS AND TWO MECHANISMS. A fallen tree is a baked archetype and lands
    the same way every other tree does — one reference, instanceable. A
    toppled streetlight is a prim that is ALREADY ON THE STAGE, so it is
    re-authored rather than added: `ClearXformOpOrder` + fresh ops on the
    instance ROOT, which USD allows (only editing INSIDE an instance is
    forbidden, and the transform ops live outside it — the same rule `_ref`
    relies on).
    """
    if spec["kind"] == "fallen_tree":
        return _ref(stage, "{0}/inst/blocker_{1}".format(PARENT, i),
                    spec["usd"], spec["x"], spec["y"], spec["yaw_deg"], ssf)
    if spec["kind"] != "streetlight" or not spec.get("prim_path"):
        return False
    prim = stage.GetPrimAtPath(spec["prim_path"])
    if not (prim and prim.IsValid()):
        return False
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return False
    sc = float(spec.get("scale", 1.0))
    xf.ClearXformOpOrder()
    sg._set_xform_ops(xf, prim,
                      translate=(spec["x"] * ssf, spec["y"] * ssf,
                                 spec["z"] * ssf),
                      rotate=(spec["roll_deg"], 0.0, spec["yaw_deg"]),
                      scale=(sc, sc, sc))
    return True


def wait_for_stage(stage, timeout_s=20.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        wp = stage.GetPrimAtPath("/World")
        if wp.IsValid() and [c for c in wp.GetChildren()
                             if c.GetName() != "PhysicsScene"]:
            return True
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

    # 1) LAYOUT + CHEAP DETAIL, houses/trees returned as instances
    t0 = time.time()
    binfo = {}
    placements = generate_suburb_on_stage(stage, config, parent_path=PARENT,
                                          scene_scale_factor=ssf,
                                          info_out=binfo, assembly=True)
    add_sky(stage, resolve_sky(config))
    houses = binfo.get("house_instances", [])
    trees = binfo.get("tree_instances", [])
    print("[assemble] layout in {0:.0f}s: {1} house + {2} tree instance(s)"
          .format(time.time() - t0, len(houses), len(trees)))

    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in os.listdir(ARCH_DIR) if f.endswith(".usd")}

    # 2) FIRE FIELD (same model as the mini)
    fcfg = dict(fire.DEFAULTS)
    fcfg.update((config.get("disaster") or {}).get("fire") or {})
    ox, oy = fcfg["origin_m"]
    th = math.radians(float(fcfg["heading_deg"]))
    ct, stt = math.cos(th), math.sin(th)
    head, flank, back = (float(fcfg["head_mps"]), float(fcfg["flank_mps"]),
                         float(fcfg["back_mps"]))

    def arrival(x, y):
        dx, dy = x - ox, y - oy
        return fire._ignition_time(dx * ct + dy * stt, -dx * stt + dy * ct,
                                   head, flank, back)

    arr = [arrival(h["x"], h["y"]) for h in houses]
    fin = sorted(t for t in arr if math.isfinite(t))
    # SMALLER BURN by default. `max(fin)` runs the front until the LAST house
    # is reached, burning almost the whole plat; a quantile stops it earlier so
    # a clear unburnt fraction remains. MINI_BURN_FRAC is the share of houses
    # inside the burn; MINI_ELAPSED still overrides outright.
    burn_frac = float(os.environ.get("MINI_BURN_FRAC", "0.45"))
    elapsed = float(os.environ.get("MINI_ELAPSED", "0")) or (
        fin[int(min(0.999, max(0.0, burn_frac)) * (len(fin) - 1))]
        if fin else 300.0)
    span = max(1.0, elapsed - (min(fin) if fin else 0.0))
    phases = dict(ignition_s=0.03 * span, flame_s=0.35 * span,
                  smoulder_s=0.25 * span, ash_after_s=0.45 * span)

    def age(x, y):
        t = arrival(x, y)
        return -1.0 if not math.isfinite(t) else elapsed - t

    # 3) REFERENCE A HOUSE ARCHETYPE PER INSTANCE
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/inst"))
    n_h = miss_h = 0
    htally = {}
    hlevels = []
    pal_jobs = []
    for i, h in enumerate(houses):
        d = age(h["x"], h["y"])
        level = "pristine" if d < 0 else damage.level_for_age(d, **phases)[0]
        # CAPTURED, because it is computed here and nowhere else. The people
        # pass needs it to tell a house that sheltered nobody (`roof_collapsed`
        # / `burned_out` / `rubble`) from one that came through — that is how
        # the pool refuges and the exposed-interior figures are chosen.
        hlevels.append(level)
        htally[level] = htally.get(level, 0) + 1
        key = "house_{0}_{1}".format(h["style"], level)
        usd = arch.get(key) or arch.get("house_{0}_pristine".format(h["style"]))
        if not usd:
            miss_h += 1
            continue
        # ROW HOMES ARE RECOLOURED, so they cannot be instanced. A terrace
        # whose units are all the same colour reads as one long building
        # rather than as eight houses, and the whole point of the morphology
        # is that they are eight houses — `row_housing` draws a palette per
        # unit with the neighbour's excluded, and this is where that draw
        # stops being advisory. Detached houses keep their style's palette,
        # which the archetype was baked with, and stay instanced.
        _pal = h.get("palette")
        _recolour = bool(h.get("row")) and bool(_pal)
        _hp = "{0}/inst/h_{1}".format(PARENT, i)
        if _ref(stage, _hp, usd, h["x"], h["y"], h["yaw"], ssf,
                instance=not _recolour):
            n_h += 1
            if _recolour:
                pal_jobs.append({"prim_path": _hp, "palette": _pal,
                                 "category": "house"})

    if pal_jobs:
        try:
            n_pal = mh.apply_palette(stage, pal_jobs, PARENT)
            print("[assemble] row homes: {0} subset(s) recoloured across "
                  "{1} unit(s)".format(n_pal, len(pal_jobs)))
        except Exception as _exc:
            print("[assemble] row-home palette FAILED: {0}".format(_exc))

    # 4) REFERENCE A TREE ARCHETYPE PER INSTANCE (green species usd if pristine)
    n_t = miss_t = 0
    ttally = {}
    tree_prims = []
    trng = random.Random(SEED + 71)
    for i, t in enumerate(trees):
        d = age(t["x"], t["y"])
        sp = t["species"]
        if d < 0:
            level = "pristine"
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale = 0.01
        else:
            # SAME SCALED PHASES AS THE HOUSES. With the default (fixed) phases
            # every tree in a 1600 m burn is past `ash_after` and comes out
            # `snag` — one look for the whole plat. The scaled phases spread
            # the ladder across the real arrival range (scorched at the edge ->
            # torched -> snag in the core); `stand_outcome` adds the fallen/
            # stump minority.
            level = veg.level_for_age(d, **phases)[0]
            if level != "pristine":
                level = veg.stand_outcome(level, trng)
            if level == "pristine":
                usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
                scale = 0.01
            else:
                usd = arch.get("tree_{0}_{1}".format(sp, level))
                scale = 1.0
        ttally[level] = ttally.get(level, 0) + 1
        if not usd:
            miss_t += 1
            continue
        _tpath = "{0}/inst/t_{1}".format(PARENT, i)
        if _ref(stage, _tpath, usd, t["x"], t["y"], t["yaw"], ssf,
                scale=scale):
            n_t += 1
            tree_prims.append((_tpath, float(t["x"]), float(t["y"])))

    # 4a) SURVIVORS, PLANNED. Nothing is authored here except the EVACUATION
    # QUEUE and its blockage: those are cars and they belong to the fire, so
    # they have to exist before 4b scorches everything the front reached. The
    # people themselves are held back until after that pass — see 4c.
    resolver = sg._make_resolver(config)
    apools = ss.AssetPools(config)
    pcfg = ppl.resolve_cfg(config)
    pctx = ppl.build_ctx(config, binfo, placements, resolver, apools,
                         age, elapsed, span, arch=arch, levels=hlevels)
    p_cars, p_blockers, p_humans, p_recs = ppl.plan_people(
        pcfg, pctx, random.Random(int(pcfg.get("seed", 91)) + SEED))
    n_blocked = n_deb = 0
    _deb_mat = None
    try:
        _deb_mat = _load_burnt_wood(stage)
    except Exception as _exc:
        print("[assemble] blockage material unavailable: {0}".format(_exc))
    for _i, _spec in enumerate(p_blockers):
        n_blocked += 1 if _place_blocker(stage, _spec, ssf, _i) else 0
        n_deb += _place_debris(stage, _spec, ssf, _i, _deb_mat)
    if n_deb:
        print("[assemble] blockage: {0} strewn piece(s) on the carriageway"
              .format(n_deb))
    # FIRE ON THE BLOCKAGE. A pile of charred timber across a road is
    # ambiguous from the air — it could be a woodpile, it could be a shadow.
    # A plume over it is not, and it is the one cue that says the road is
    # closed NOW rather than at some point in the past. Cheap, too: four
    # emitters on the whole plat, against the ~99 the full burn would want.
    n_flow = 0
    if p_blockers:
        try:
            fire.setup_flow_stack(stage, density_cell_size_m=0.16,
                                  max_blocks=16384, scene_scale_factor=ssf,
                                  root="/World/flow_blockage")
            for _i, _spec in enumerate(p_blockers):
                # NOT EVERY BLOCKAGE BURNS. A tree across a road is a blockage
                # whether or not it is alight, and a plat where every one of
                # them is on fire reads as staged. `people._add_blocker` draws
                # the flag; absent (an older plan) means burn, which is the
                # previous behaviour.
                if not _spec.get("fire", True):
                    continue
                _bx = float(_spec.get("road_x", _spec.get("x", 0.0)))
                _by = float(_spec.get("road_y", _spec.get("y", 0.0)))
                # AHEAD OF THE BLOCKAGE, NOT ON THE QUEUE. The cars back up
                # BEHIND the blockage, so an emitter centred on it puts flame
                # among them — the user's note was that the cars sit "way too
                # close to the fire". The fire belongs on the far side, in the
                # timber that fell and the ground it fell on, which is also
                # where a fire that brought the tree down would be. `out_bear`
                # is the direction the queue is trying to travel, so +bearing
                # is past the blockage and away from the cars.
                _ob = math.radians(float(_spec.get("out_bear_deg", 0.0)))
                _fx, _fy = math.cos(_ob), math.sin(_ob)
                for _k, (_st, _along, _hz) in enumerate(
                        (("flame", 7.0, 1.1),
                         ("smoke", 15.0, 0.9))):
                    _dx, _dy = _fx * _along, _fy * _along
                    _pr = fire._flow_create(
                        stage, "/World/flow_blockage/em_{0}_{1}".format(_i, _k),
                        "FlowEmitterBox")
                    if not _pr or not _pr.IsValid():
                        continue
                    fire._set(_pr, "layer", Sdf.ValueTypeNames.Int,
                              fire.FLOW_LAYER)
                    fire._set(_pr, "position", Sdf.ValueTypeNames.Float3,
                              Gf.Vec3f((_bx + _dx) * ssf, (_by + _dy) * ssf,
                                       _hz * ssf))
                    fire._set(_pr, "halfSize", Sdf.ValueTypeNames.Float3,
                              Gf.Vec3f(3.2 * ssf, 2.6 * ssf, 1.3 * ssf))
                    fire.set_emission(_pr, _st, scale=1.0)
                    n_flow += 1
            print("[assemble] blockage: {0} Flow emitter(s) burning"
                  .format(n_flow))
        except Exception as _exc:
            print("[assemble] blockage fire FAILED: {0}".format(_exc))
    if p_cars:
        # UN-INSTANCED, like every other car on the plat: a car has to be
        # authorable to be scorched, and to have its GLASS removed — which is
        # the only way the camera ever sees the people sitting in one, because
        # this renderer forces fractional opacity to 1.0.
        sg.apply_placements(stage, p_cars, PARENT + "/people_cars", ssf,
                            resolver=resolver, instance_categories=set())
        from detail import vehicles as _veh
        # Every queue / cul-de-sac car, by the same per-asset rule the parked
        # cars take — an occupant nobody can see is an occupant nobody can
        # label. See `vehicles.CABIN_RULES`.
        _n_mesh = sum(_veh.open_cabin(stage, q["prim_path"], q.get("usd", ""))
                      for q in p_cars if q.get("prim_path"))
        print("[assemble] people: {0} car(s) placed ({1} glass mesh(es) "
              "stripped), {2} blocker(s)".format(len(p_cars), _n_mesh,
                                                 n_blocked))
        # NOT added to `placements`: 4b no longer scorches cars (see BURNABLE).

    # 4b) FENCES + BURNABLE FURNITURE. A timber fence is a line of dry fuel on
    # the ground and one of the first things a wildfire takes: runs VANISH deep
    # in the burn and stand SCORCHED at its edge. `apply_ground`/planting built
    # them intact and un-instanced (see the preset), so damage them here. No
    # fracture — there is no settle in the assembly, so consumed-or-scorched is
    # the whole vocabulary (which is most of the read from the air anyway).
    def coverage_at(x, y):
        d = age(x, y)
        if d < 0.0:
            return 0.0
        return min(1.0, 0.45 + 0.55 * min(1.0, d / max(1e-6, elapsed)))

    # FENCES are line fuel that VANISHES deep in the burn — consumed. They
    # take one shared dark OmniPBR when they survive, NOT per-subset soot:
    # there are ~7k of them and compositing a texture each blew VRAM to 17 GB.
    # EVERYTHING ELSE the fire reached — cars, benches, bins, play structures,
    # park goals/baskets — is SCORCHED IN PLACE with `soot_materials` (its own
    # texture + soot, like the house walls) and never consumed, so nothing
    # "disappears" and a burnt car reads as a charred car, not flat ash. That
    # list is only a few hundred, so the soot pass is affordable.
    char = damage._pbr(stage, PARENT + "/BurnLooks/fence_char",
                       (0.05, 0.045, 0.04), 0.9)
    # `car` IS DELIBERATELY NOT IN THIS LIST. Compositing soot onto a vehicle
    # looked wrong on sight: these car materials are glossy, tightly-mapped
    # paint over a small UV layout, and a soot wash built for a brick wall
    # comes out as grey blotches rather than as scorching. A car in a burn
    # scar is also not usually charred — the ones that burn are consumed
    # outright and the rest are ordinary cars standing in a black landscape,
    # which is exactly what an abandoned evacuation queue looks like. Nothing
    # is lost from the read and a whole class of bad-looking props goes away.
    BURNABLE = ("fence", "bench", "chair", "picnic_table", "table",
                "trash_can", "play_structure", "planter", "bin", "cafe_set",
                "swing", "seesaw", "bike_rack", "mailbox", "bus_stop",
                "park_feature", "goal", "basket", "hoop")
    brng = random.Random(SEED + 5)
    n_gone = n_char = 0
    scorch_props = []
    for q in placements:
        path = q.get("prim_path")
        cat = str(q.get("category", ""))
        if not path or not any(k in cat for k in BURNABLE):
            continue                                   # only combustibles
        if damage.is_incombustible(cat):
            continue
        d = age(float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0)))
        if d <= 0.0:
            continue                                   # front never reached it
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if "fence" in cat:                             # line fuel
            if d / max(1e-6, span) > 0.55 and brng.random() < 0.90:
                if prim.SetActive(False):
                    n_gone += 1
            else:
                for m in Usd.PrimRange(prim):
                    if m.IsA(UsdGeom.Mesh):
                        UsdShade.MaterialBindingAPI(m).Bind(char)
                n_char += 1
        else:                                          # cars, park props, …
            scorch_props.append(q)
    n_soot = damage.soot_materials(stage, scorch_props, PARENT,
                                   random.Random(SEED), coverage_at=coverage_at)
    print("[assemble] burnable: {0} fence(s) consumed, {1} fence(s) charred, "
          "{2} prop subset(s) scorched (cars/park/furniture)"
          .format(n_gone, n_char, n_soot))

    # PARK GROUND SURFACES scorched. The court slabs, pitch, line markings and
    # paths are drawn geometry (not placements), so they miss the soot pass
    # above; re-bind the ones the front reached to the dark char material.
    gnd_prim = stage.GetPrimAtPath(PARENT + "/ground")
    _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_park = 0
    if gnd_prim and gnd_prim.IsValid():
        for prim in gnd_prim.GetChildren():
            _nm = prim.GetName()
            if not _nm.startswith("park_"):
                continue
            # ASPHALT DOES NOT CHAR. The refuge lot and its apron are laid in
            # the same material as the streets, and the streets are handled
            # separately below (re-bound to Damaged_Asphalt, not to soot) for
            # exactly that reason. Charring the lot black would also delete the
            # feature the survivors are standing on: a parking lot reads as a
            # refuge BECAUSE it is bare pavement, which is the property that
            # made 14 of the Camp Fire's 31 temporary refuge areas car parks.
            if _nm.startswith("park_parking"):
                continue
            r = _bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            if age(c[0] / ssf, c[1] / ssf) < 0.0:
                continue
            for m in Usd.PrimRange(prim):
                if m.IsA(UsdGeom.Mesh):
                    UsdShade.MaterialBindingAPI(m).Bind(char)
            n_park += 1
    print("[assemble] park ground: {0} surface(s) scorched".format(n_park))

    # 5) GROUND SCAR (built on the fly, cheap)
    region = tuple(binfo.get("region") or (-800, -600, 800, 600))
    zs = float(binfo.get("z_scale") or ss.ground_z_scale(config, region))
    burn_z = (ss._Z_GRASS + 0.5 * (ss._Z_ASPHALT - ss._Z_GRASS)) * zs
    kn = ground.knobs_from_env(max(region[2] - region[0], region[3] - region[1]))
    cov = ground.feathered_coverage(
        arrival, elapsed, (ox, oy), region, np.random.default_rng(SEED + 23),
        edge_m=kn["edge_m"], finger_m=kn["finger_m"], islands=kn["islands"])
    made = ground.build_overlay(
        stage, cov, region, ssf, burn_z, material_parent=PARENT,
        cell_m=kn["cell_m"], bands=kn["bands"], tile_m=kn["tile_m"],
        op_range=kn["op_range"],
        skip=ground.skip_rects(binfo.get("pool_rects") or (), pad=0.0))

    # FIRE-DAMAGED PAVING -> Damaged_Asphalt. `apply_ground` builds the road
    # and drive ribbons before any fire field exists, so re-bind here: every
    # road/turnaround/asphalt-drive whose centre the front reached takes the
    # cracked, heat-damaged asphalt. Brick drives (Brick_Wall_Worn) are left
    # alone — they are not asphalt, and worn brick already reads as damaged.
    dmg_url = sg._join_asset_root(
        "airstack://scene_gen/assets/materials/megascans/Damaged_Asphalt.usda", "")
    dmg_path = PARENT + "/ground/materials/damaged_asphalt"
    dprim = stage.DefinePrim(Sdf.Path(dmg_path))
    dprim.GetReferences().AddReference(dmg_url)
    dprim.Load()
    dmat = UsdShade.Material(stage.GetPrimAtPath(dmg_path))
    gnd = stage.GetPrimAtPath(PARENT + "/ground")
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_dmg = 0
    if gnd and gnd.IsValid() and dmat:
        for prim in gnd.GetChildren():
            nm = prim.GetName()
            if not nm.startswith(("road_", "bulb_", "drive_")):
                continue
            bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if bound and "Brick" in bound.GetPrim().GetPath().pathString:
                continue                       # worn-brick drive stays brick
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            if age(c[0] / ssf, c[1] / ssf) >= 0.0:   # front reached it
                UsdShade.MaterialBindingAPI(prim).Bind(dmat)
                n_dmg += 1
    print("[assemble] {0} road/drive ribbon(s) re-bound to Damaged_Asphalt "
          "in the burn".format(n_dmg))

    # 4c) THE PEOPLE, AFTER THE SCORCH PASS. `"human"` deliberately matches
    # nothing in BURNABLE above, and holding them back until here is the
    # belt-and-braces version of that: a survivor is not scorched. Authored
    # through `apply_placements` because that is what binds each `pose` onto
    # the character's own UsdSkel rig (`scene_generator._HUMAN_POSES`), and
    # NOT instanced, because the pose animation is authored inside each prim.
    # 4b-2) CLEAR A GLADE ROUND EACH OPEN-GROUND GROUP. `_open_ground` already
    # requires 15 m from any structure or tree TRUNK, but a trunk keep-out is
    # not a clearing: these crowns are 10-25 m across, so a group can satisfy
    # the rule and still sit under a closed canopy where a drone sees leaves.
    # The scenario IS "people on open ground", so the ground is opened —
    # deactivating a handful of tree references is cheaper and more certain
    # than fighting the planting passes for a gap, and a burnt stand with a
    # hole in it is exactly what a fire leaves anyway.
    n_glade = 0
    _glade_r = float((pcfg.get("scenarios", {}).get("open_ground") or {})
                     .get("glade_r_m", 16.0))
    if _glade_r > 0.0 and tree_prims:
        _seeds = [(r["x"], r["y"]) for r in p_recs
                  if r.get("scenario") == "open_ground"]
        if _seeds:
            _r2 = _glade_r * _glade_r
            for _tp, _tx, _ty in tree_prims:
                for (_sx, _sy) in _seeds:
                    if (_tx - _sx) ** 2 + (_ty - _sy) ** 2 <= _r2:
                        _pr = stage.GetPrimAtPath(_tp)
                        if _pr and _pr.IsValid() and _pr.SetActive(False):
                            n_glade += 1
                        break
            print("[assemble] clearings: {0} tree(s) removed within {1:.0f} m "
                  "of {2} open-ground group(s)".format(
                      n_glade, _glade_r, len(_seeds)))

    n_people = 0
    if p_humans:
        sg.apply_placements(stage, p_humans, PARENT + "/people", ssf,
                            resolver=resolver, instance_categories=set())
        n_people = len(p_humans)
        ppl.write_records(PEOPLE_JSON, p_recs, meta={
            "seed": SEED, "scene_config": SCENE_CONFIG,
            "elapsed_s": round(elapsed, 1), "span_s": round(span, 1),
            "burn_frac": burn_frac,
            "fire_origin_m": [ox, oy],
            "fire_heading_deg": float(fcfg["heading_deg"]),
            "blockers": p_blockers,
        })
        print("[assemble] people: {0} authored, ground truth -> {1}"
              .format(n_people, PEOPLE_JSON))
        if POLES:
            n_poles = build_people_poles(stage, p_recs, ssf)
            n_rowp = build_row_poles(stage, binfo.get("clusters"), ssf)
            if n_rowp:
                print("[assemble] row-home poles: {0} under {1}{2} (cyan)"
                      .format(n_rowp, PARENT, ROW_POLE_SCOPE))
            print("[assemble] people poles: {0} under {1}{2} "
                  "(deactivate that one prim to hide them all)"
                  .format(n_poles, PARENT, POLE_SCOPE))

    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(30):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 72)
    print("ASSEMBLED 1600 x 1200 (by reference, no live fracture/settle)")
    print("  total       {0:.0f} s".format(time.time() - t0))
    print("  houses      {0} referenced ({1} missing); {2}".format(
        n_h, miss_h, ", ".join("%s=%d" % kv for kv in sorted(htally.items()))))
    print("  trees       {0} referenced ({1} missing); {2}".format(
        n_t, miss_t, ", ".join("%s=%d" % kv for kv in sorted(ttally.items()))))
    print("  ground scar {0} band(s)".format(len(made)))
    _ptally = {}
    for _r in p_recs:
        _ptally[_r["scenario"]] = _ptally.get(_r["scenario"], 0) + 1
    print("  people      {0} alive, {1} car(s), {2} blocker(s) -> {3}".format(
        sum(1 for _r in p_recs if _r.get("alive")), len(p_cars),
        len(p_blockers), PEOPLE_JSON))
    for _name in ppl.SCENARIOS:
        print("    {0:<18} {1:>3}".format(_name, _ptally.get(_name, 0)))
    print("=" * 72 + "\n")

    # SNAPSHOTS. Nobody is at the GUI on a scripted run, and a scene this size
    # can only be judged by looking at it — so the launcher photographs its own
    # result: an overview of the plate, then a top-down and an oblique of each
    # thing this build is supposed to have got right. `SNAP_DIR=` turns it on.
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots",
                                                 os.path.normpath(_sp))
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            _pts = {}
            _park = (binfo.get("park") or {})
            _lot = (_park.get("parking") or {})
            if _lot.get("centre"):
                _pts["refuge_lot"] = tuple(_lot["centre"])
            for _i, _b in enumerate(p_blockers[:2]):
                if _b.get("x") is not None:
                    _pts["blocker_%d" % _i] = (_b["x"], _b["y"])
            _byscen = {}
            for _r in p_recs:
                _byscen.setdefault(_r["scenario"], _r)
            for _name in ("pools", "open_ground", "at_home",
                          "exposed_interior"):
                _r = _byscen.get(_name)
                if _r:
                    _pts[_name] = (_r["x"], _r["y"])
            _reg = binfo.get("region")
            if _reg:
                _snaps.overview(stage,
                                (0.5 * (_reg[0] + _reg[2]),
                                 0.5 * (_reg[1] + _reg[3])),
                                max(_reg[2] - _reg[0], _reg[3] - _reg[1]),
                                os.path.join(SNAP_DIR, "overview.png"), ssf)
            _snaps.views_around(stage, _pts, SNAP_DIR, ssf)
            print("[assemble] snapshots -> {0} ({1} subject(s))"
                  .format(SNAP_DIR, len(_pts)))
        except Exception as _exc:
            print("[assemble] snapshots FAILED: {0}".format(_exc))

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
