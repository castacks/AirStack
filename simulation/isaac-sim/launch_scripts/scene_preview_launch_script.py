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
    sys.path.insert(0, "/path/to/AirStack/scene_gen")
    sys.path.insert(0, "/path/to/AirStack/simulation/isaac-sim/utils")
    import importlib, scene_generator, scene_prep
    importlib.reload(scene_generator)           # picks up code edits too
    import omni.usd, omni.timeline
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    scene_generator.reload_scene_on_stage(
        stage,
        "/path/to/AirStack/scene_gen/config/low_level/compiled/earthquake.yaml",
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
# The scene generator lives at the repo root in scene_gen/ (it is sim-agnostic
# and has its own configs and asset pipeline); scene_prep stays here because it
# is Isaac Sim stage tooling the plain Pegasus launch scripts share.
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)
from scene_prep import (scale_stage_prim, add_colliders, add_sky,
                        get_stage_meters_per_unit, settle_rigid_props)
from scene_generator import generate_scene_on_stage, resolve_sky
# Accepts a config at either level: a high-level disaster spec is
# compiled in memory, a low-level scene config is used as is.
from compile_disaster import load_scene_config

# ----- CONFIGURATION -----
ENV_URL     = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
# The SCENE_CONFIG env var wins when set, so `airstack up` can target a scene
# without editing this file — it takes any form load_scene_config() accepts
# (bare name, high-level spec, or compiled low-level config).
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or os.path.join(
    _SCENE_GEN_DIR, "config", "presets", "earthquake.yaml")
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

        config = load_scene_config(SCENE_CONFIG)

        _, ssf = get_stage_meters_per_unit(stage)
        placements = generate_scene_on_stage(stage, config,
                                             parent_path="/World/stage/generated",
                                             scene_scale_factor=ssf)

        generated_prim = stage.GetPrimAtPath("/World/stage/generated")
        if generated_prim.IsValid():
            add_colliders(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        # Physics-settle toppled/strewn props so they rest naturally.
        settle_rigid_props(
            stage,
            [p["prim_path"] for p in placements
             if p.get("settle") and p.get("prim_path")],
            ground_path="/World/stage/generated/ground",
        )

        add_sky(stage, resolve_sky(config))

        _reload_script = os.path.join(_SCENE_GEN_DIR, "reload_scene.py")
        print("\n" + "=" * 70)
        print("SCENE PREVIEW READY — to reload, open Window → Script Editor and run:")
        print(f'  exec(open({repr(_reload_script)}).read())')
        print("Edit the scene config or reload_scene.py between runs.")
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
