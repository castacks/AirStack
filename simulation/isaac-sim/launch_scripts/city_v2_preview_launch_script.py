#!/usr/bin/env python
"""
Preview launcher for the detailed city (scene_gen/generate_city_v2.py).

Deliberately a near-copy of scene_preview_launch_script.py — same base
environment, same stage prep, same lighting — so that running the two side by
side isolates the thing under test: the city itself. The only differences are
which generator entry point is called and which config it defaults to.

    SCENE_CONFIG=urban_v2 \
    ISAAC_SIM_SCRIPT_NAME=city_v2_preview_launch_script.py \
    airstack up isaac-sim

Compare against the unchanged original with:

    SCENE_CONFIG=none \
    ISAAC_SIM_SCRIPT_NAME=scene_preview_launch_script.py \
    airstack up isaac-sim

No drone, no sensors, no ROS 2 bridge, so startup is fast and the scene can be
regenerated in place after editing the YAML — see RELOAD below.

RELOAD WITHOUT RESTARTING
--------------------------
Open the Kit Script Editor (Window -> Script Editor) and run:

    import os, sys, importlib
    SCENE_GEN = "/isaac-sim/AirStack/scene_gen"
    ISAAC_DIR = "/isaac-sim/AirStack/simulation/isaac-sim"
    sys.path[:0] = [SCENE_GEN, os.path.join(ISAAC_DIR, "utils")]
    import scene_generator, city_detail, city_layout, districts, road_markings
    import generate_city_v2, scene_prep
    for m in (scene_generator, city_detail, city_layout, districts,
              road_markings, generate_city_v2):
        importlib.reload(m)          # pick up edits to the new passes too
    import omni.usd, omni.timeline
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    # add_colliders_skip_empty, not scene_prep.add_colliders — the stock one
    # floods the log with PhysX errors for point-less stub meshes.
    import city_v2_preview_launch_script as v2
    generate_city_v2.reload_city_v2_on_stage(
        stage, os.path.join(SCENE_GEN, "config", "presets", "urban_v2.yaml"),
        scene_scale_factor=ssf, add_colliders_fn=v2.add_colliders_skip_empty)
    omni.timeline.get_timeline_interface().play()
"""

import os
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
from scene_prep import (scale_stage_prim, add_colliders, add_sky,
                        get_stage_meters_per_unit, settle_rigid_props)
from scene_generator import resolve_sky
from generate_city_v2 import generate_city_v2_on_stage
from compile_disaster import load_scene_config

# ----- CONFIGURATION -----
ENV_URL     = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or os.path.join(
    _SCENE_GEN_DIR, "config", "presets", "urban_v2.yaml")
# -------------------------


_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    """Deactivate the GroundPlane and Environment xforms the base environment
    brings in. The generator lays its own ground, so these cause z-fighting and
    an unwanted visual backdrop — `Environment/Geometry` is the grid mesh that
    reads as a blue square under the spawn point.

    `default_environment.usd` has defaultPrim `/World`, which Pegasus references
    at `/World/stage`, so everything here composes from a referenced layer.
    `RemovePrim` cannot delete across a reference: it returns False rather than
    raising, so a try/except around it never fires and the prim stays visible.
    Deactivation does compose, and is reversible in the viewport.

    `SphereLight` is a sibling of these, not a child, so lighting survives.
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
                carb.log_info(f"[city_v2] deactivated {child.GetPath()}")
            else:
                UsdGeom.Imageable(child).MakeInvisible()
                carb.log_info(f"[city_v2] hid {child.GetPath()}")
    print(f"[city_v2] env clutter: {n} prim(s) deactivated")


def _disable_sky_sun(stage):
    """Switch off the DistantLight the sky rig hangs under its axis chain.

    `add_sky` builds /World/Environment/sky/AxisNorth/.../DistantLight — a hard
    directional sun on top of the dome. It runs AFTER `_remove_env_clutter`, so
    the clutter pass cannot see it; this has to come after `add_sky` instead.
    The dome itself is left alone, since that is what lights the scene.
    """
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
            carb.log_info(f"[city_v2] disabled sun {prim.GetPath()}")
    print(f"[city_v2] sky sun: {n} DistantLight(s) disabled")
    return n


def add_colliders_skip_empty(prim):
    """`scene_prep.add_colliders`, but skipping meshes that have no points.

    The stock version applies UsdPhysics.CollisionAPI to every gprim it finds.
    Some referenced props carry empty stub Meshes — `Planter_03`, pulled in by
    the Tower pack's `Bench_01`, is the one that shows up here — and PhysX then
    logs "Provided mesh geom with a PhysicsCollisionAPI does not have points,
    collision will not be created" once per empty mesh **per instance**, which
    at city density is thousands of error lines that bury real problems.

    Skipping them changes nothing physically: a mesh with no points has no
    geometry to collide with either way. Returns (applied, skipped).
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
    print(f"[city_v2] colliders: {applied} applied, {skipped} empty meshes "
          f"skipped")
    return applied, skipped


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


class CityV2PreviewApp:

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
            add_colliders_skip_empty(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        config = load_scene_config(SCENE_CONFIG)

        _, ssf = get_stage_meters_per_unit(stage)
        placements = generate_city_v2_on_stage(
            stage, config, parent_path="/World/stage/generated",
            scene_scale_factor=ssf)

        generated_prim = stage.GetPrimAtPath("/World/stage/generated")
        if generated_prim.IsValid():
            add_colliders_skip_empty(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        settle_rigid_props(
            stage,
            [p["prim_path"] for p in placements
             if p.get("settle") and p.get("prim_path")],
            ground_path="/World/stage/generated/ground",
        )

        add_sky(stage, resolve_sky(config))
        _disable_sky_sun(stage)

        print("\n" + "=" * 70)
        print("CITY V2 PREVIEW READY")
        print(f"  config: {SCENE_CONFIG}")
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
    CityV2PreviewApp().run()


if __name__ == "__main__":
    main()
