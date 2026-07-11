#!/usr/bin/env python
"""
Minimal Isaac Sim launcher for iterating on the procedural city generator.

Loads a flat base environment, generates the city, then sits in a render loop.
No drone, no sensors, no ROS 2 bridge — so startup is much faster than the
full generated_scene_launch_script.py and you can reload after editing the YAML
without restarting Isaac Sim.

RELOAD WITHOUT RESTARTING
--------------------------
While the sim is running, open the Kit Script Editor (Window → Script Editor)
and run:

    import os, sys
    sys.path.insert(0, "/path/to/AirStack/simulation/isaac-sim/utils")
    import importlib, scene_generator, scene_prep
    importlib.reload(scene_generator)           # picks up code edits too
    import omni.usd, omni.timeline
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    scene_generator.reload_scene_on_stage(
        stage,
        "/path/to/AirStack/simulation/isaac-sim/config/scene_generator_config.yaml",
        scene_scale_factor=ssf,
        add_colliders_fn=scene_prep.add_colliders,
    )
    omni.timeline.get_timeline_interface().play()

The reload snippet is also printed to stdout when this script starts so you can
copy-paste it from the terminal into the Script Editor.
"""

import os
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension
# Auto-enable Script Editor since this workflow's reload loop (see module
# docstring and reload_scene.py) depends on it every run.
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Sdf, UsdGeom
from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light, get_stage_meters_per_unit
from scene_generator import generate_scene_on_stage

# ----- CONFIGURATION -----
ENV_URL     = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SCENE_CONFIG = os.path.join(_ISAAC_SIM_DIR, "config", "scene_generator_config.yaml")
# -------------------------


_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    """Remove or hide GroundPlane and Environment xforms added by the base
    environment load. The scene generator lays its own ground, so these cause
    z-fighting and an unwanted visual backdrop.

    Tries RemovePrim first; falls back to MakeInvisible for prims that live in
    a referenced layer and can't be deleted from the session layer.
    """
    for root_path in ("/World", "/World/stage"):
        root = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            continue
        for child in root.GetChildren():
            if child.GetName() not in _ENV_CLUTTER:
                continue
            path = child.GetPath()
            try:
                stage.RemovePrim(path)
                carb.log_info(f"[scene_preview] Removed {path}")
            except Exception:
                UsdGeom.Imageable(child).MakeInvisible()
                carb.log_info(f"[scene_preview] Hid {path} (in referenced layer, can't delete)")


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


class ScenePreviewApp:

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
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        _, ssf = get_stage_meters_per_unit(stage)
        generate_scene_on_stage(stage, SCENE_CONFIG,
                                parent_path="/World/stage/generated",
                                scene_scale_factor=ssf)

        generated_prim = stage.GetPrimAtPath("/World/stage/generated")
        if generated_prim.IsValid():
            add_colliders(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        add_dome_light(stage)

        _reload_script = os.path.join(_ISAAC_SIM_DIR, "reload_scene.py")
        print("\n" + "=" * 70)
        print("SCENE PREVIEW READY — to reload, open Window → Script Editor and run:")
        print(f'  exec(open({repr(_reload_script)}).read())')
        print("Edit config/scene_generator_config.yaml or reload_scene.py between runs.")
        print("=" * 70 + "\n")

        self.timeline.play()

    def run(self):
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    ScenePreviewApp().run()


if __name__ == "__main__":
    main()
