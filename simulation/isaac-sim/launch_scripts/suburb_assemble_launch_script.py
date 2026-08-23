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
import scene_generator as sg                                  # noqa: E402
from scene_prep import add_sky, get_stage_meters_per_unit     # noqa: E402
from scene_generator import resolve_sky                       # noqa: E402
import suburb_scene as ss                                     # noqa: E402
from suburb_scene import generate_suburb_on_stage             # noqa: E402
from compile_disaster import load_scene_config                # noqa: E402
from disaster import bake, damage, fire, ground               # noqa: E402
from disaster import vegetation as veg                        # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_wildfire")
SEED = int(os.environ.get("MINI_SEED", "11"))
ARCH_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes"))
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0):
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
    prim.SetInstanceable(True)
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
    generate_suburb_on_stage(stage, config, parent_path=PARENT,
                             scene_scale_factor=ssf, info_out=binfo,
                             assembly=True)
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
    fin = [t for t in arr if math.isfinite(t)]
    elapsed = float(os.environ.get("MINI_ELAPSED", "0")) or (
        max(fin) if fin else 300.0)
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
    for i, h in enumerate(houses):
        d = age(h["x"], h["y"])
        level = "pristine" if d < 0 else damage.level_for_age(d, **phases)[0]
        htally[level] = htally.get(level, 0) + 1
        key = "house_{0}_{1}".format(h["style"], level)
        usd = arch.get(key) or arch.get("house_{0}_pristine".format(h["style"]))
        if not usd:
            miss_h += 1
            continue
        if _ref(stage, "{0}/inst/h_{1}".format(PARENT, i), usd,
                h["x"], h["y"], h["yaw"], ssf):
            n_h += 1

    # 4) REFERENCE A TREE ARCHETYPE PER INSTANCE (green species usd if pristine)
    n_t = miss_t = 0
    ttally = {}
    for i, t in enumerate(trees):
        d = age(t["x"], t["y"])
        sp = t["species"]
        if d < 0:
            level = "pristine"
            usd = sg._join_asset_root(TREE_SPECIES.get(sp, ""), "")
            scale = 0.01
        else:
            level = veg.level_for_age(d)[0]
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
        if _ref(stage, "{0}/inst/t_{1}".format(PARENT, i), usd,
                t["x"], t["y"], t["yaw"], ssf, scale=scale):
            n_t += 1

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
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
