#!/usr/bin/env python
"""
Preview launcher for the ModularNeighborhood house kit
(scene_gen/detail/modular_house.py).

Spawns every style in the catalogue — 6 residential, 8 suburban commercial —
assembled piece by piece from
`/Library/Stages/Muyang/ModularNeighborhood/Assets/` rather than placed as
whole-house models. Nothing else: no drone, no sensors, no ROS 2 bridge, so it
boots fast and is only about whether the kit assembles correctly.

    ISAAC_SIM_SCRIPT_NAME=modular_house_preview_launch_script.py \
    airstack up isaac-sim

This is a CATALOGUE, not a neighbourhood: every style on a grid plus a strip of
the pack's loose props, so the range of shapes, materials and roof forms can be
read off in one view. Making it look like a coherent place is the suburb
generator's job.

Environment:
    HOUSE_SEED      RNG seed for the fenestration (default 7)
    HOUSE_DRESS     0 to skip driveways/fences/pools and see the shells alone

RELOAD WITHOUT RESTARTING
-------------------------
Open the Kit Script Editor (Window -> Script Editor) and run:

    import os, sys, importlib, random
    SCENE_GEN = "/isaac-sim/AirStack/scene_gen"
    ISAAC_DIR = "/isaac-sim/AirStack/simulation/isaac-sim"
    sys.path[:0] = [SCENE_GEN, os.path.join(ISAAC_DIR, "utils")]
    import scene_generator, scene_prep
    from detail import modular_house
    for m in (scene_generator, modular_house):
        importlib.reload(m)
    import omni.usd, omni.timeline
    from pxr import Sdf
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    stage.RemovePrim(Sdf.Path("/World/stage/generated"))    # clear the old build
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    pl = modular_house.build_catalogue(random.Random(7))
    scene_generator.apply_placements(stage, pl, "/World/stage/generated", ssf)
    modular_house.apply_palette(stage, pl, "/World/stage/generated")
    omni.timeline.get_timeline_interface().play()
"""

import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

# The reload loop above depends on the Script Editor every run.
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import (scale_stage_prim, add_sky, get_stage_meters_per_unit)
import scene_generator as sg
from detail import modular_house

# ----- CONFIGURATION -----
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SEED = int(os.environ.get("HOUSE_SEED", "7"))
SPACING_M = float(os.environ.get("HOUSE_SPACING", "30"))
DRESS = os.environ.get("HOUSE_DRESS", "1") not in ("0", "false", "no")
PARENT = "/World/stage/generated"
# -------------------------

_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    """Deactivate the base environment's GroundPlane and Environment xforms.

    Same reasoning as `scene_launch_script`: `RemovePrim` cannot delete across a
    reference — it returns False rather than raising — so these are deactivated,
    which does compose and is reversible in the viewport.
    """
    n = 0
    for root_path in ("/", "/World", "/World/stage"):
        root = (stage.GetPseudoRoot() if root_path == "/"
                else stage.GetPrimAtPath(root_path))
        if not root or not root.IsValid():
            continue
        for child in root.GetChildren():
            if child.GetName() not in _ENV_CLUTTER or not child.IsActive():
                continue
            if child.SetActive(False):
                n += 1
            else:
                UsdGeom.Imageable(child).MakeInvisible()
    print(f"[modular_house] env clutter: {n} prim(s) deactivated")


def add_colliders_skip_empty(prim):
    """`scene_prep.add_colliders`, skipping point-less stub meshes.

    The kit carries material-only USDs whose meshes have no points; PhysX logs
    one error per empty mesh per instance, which buries real problems.
    """
    applied = skipped = 0
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Gprim):
            continue
        if p.IsA(UsdGeom.Mesh):
            pts = p.GetAttribute("points")
            if not pts or not pts.HasAuthoredValue() or not (pts.Get() or []):
                skipped += 1
                continue
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(p)
            applied += 1
    print(f"[modular_house] colliders: {applied} applied, {skipped} empty skipped")
    return applied, skipped


def report_missing(stage, placements):
    """Name any placement that resolved to nothing.

    The kit has 293 USDs of which a good number are MATERIALS, not geometry —
    `Fence_01.usd` is a material and `Fence_01_0.usd` is the fence. Referencing
    the wrong one places a prim that loads clean and draws nothing, so it is
    worth saying out loud rather than wondering where the fences went.
    """
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    empty = {}
    for p in placements:
        path = p.get("prim_path")
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if bc.ComputeWorldBound(prim).ComputeAlignedRange().IsEmpty():
            empty[p["usd"]] = empty.get(p["usd"], 0) + 1
    if empty:
        print(f"[modular_house] WARN {sum(empty.values())} placements drew nothing:")
        for usd, n in sorted(empty.items(), key=lambda kv: -kv[1]):
            print(f"[modular_house]   {n:4d} x {usd}")
    else:
        print("[modular_house] every placement drew geometry")
    return empty


def wait_for_stage(stage, timeout_s: float = 10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren()
                           if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


class ModularHousePreviewApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        _remove_env_clutter(stage)
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            for _ in range(10):
                omni.kit.app.get_app().update()

        rng = random.Random(SEED)
        # Fails with a sentence if a style's roof does not cover its
        # footprint or a style has no front door — cheaper than
        # finding either in a render.
        modular_house.check()
        placements = modular_house.build_catalogue(rng, dress=DRESS)
        print("[modular_house] " +
              modular_house.summarise(placements).replace("\n", "\n[modular_house] "))

        _, ssf = get_stage_meters_per_unit(stage)
        sg.apply_placements(stage, placements, PARENT, ssf)
        # After placement, because it needs each prim_path. The kit ships ONE
        # wall texture, so without this every building is the same cream.
        n_mat = modular_house.apply_palette(stage, placements, PARENT)
        print(f'[modular_house] palette: {n_mat} subsets rebound')

        # PHYSICS COLLIDERS ARE OFF, deliberately.
        #
        # `add_colliders_skip_empty` applies UsdPhysics.CollisionAPI to EVERY
        # gprim under the generated root — 76,668 of them on a full suburb —
        # and PhysX then cooks collision for all of them inside the FIRST
        # `app.update()`. That one frame ran long enough that the window stopped
        # responding, X invalidated it (`BadWindow`), and Kit tore down a render
        # thread through pybind11 without the GIL, which aborts the process.
        #
        # Nothing here needs physics yet: this is a capture scene, and neither
        # rendering nor LiDAR reads CollisionAPI. Re-enable when something has
        # to fly into the geometry — and filter by category first, because a
        # drone needs to collide with building shells, trees and poles, not with
        # hedges, bins, chairs, paving slabs or window frames.
        #
        # gen = stage.GetPrimAtPath(PARENT); if gen.IsValid():
        #     add_colliders_skip_empty(gen)
        for _ in range(10):
            omni.kit.app.get_app().update()

        report_missing(stage, placements)
        add_sky(stage, "")

        print("\n" + "=" * 70)
        print("MODULAR HOUSE PREVIEW READY")
        print(f"  seed {SEED}   spacing {SPACING_M} m   dressing {'on' if DRESS else 'off'}")
        for s in modular_house.ORDER:
            print(f"    {s:<12} {modular_house.STYLES[s]['note']}")
        print("  reload: see the snippet in this script's docstring")
        print("=" * 70 + "\n")
        self.timeline.play()

    def run(self):
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    ModularHousePreviewApp().run()


if __name__ == "__main__":
    main()
