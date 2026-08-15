#!/usr/bin/env python
"""
Single-drone PX4 launcher that procedurally generates the scene at runtime.

Same shape as example_one_px4_pegasus_launch_script.py, but instead of relying
on a hand-authored environment for clutter it:
 - Loads a (usually flat / terrain-only) base environment
 - Scatters an asset library across configured zones via scene_gen/scene_generator
 - Adds colliders + dome light, then spawns the drone

SCENE_CONFIG points at a scene config under scene_gen/config/. Pick a different
compiled disaster (or recompile one from its high-level spec with
scene_gen/compile_disaster.py) to change what gets generated — no code changes
needed. For repeatable batch runs, pre-bake instead with:
    python scene_gen/scene_generator.py --config <yaml> --output assets/scenes/foo.usd
and load the result like any other environment.
"""

import os
import sys
import time

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports.
simulation_app = SimulationApp(launch_config={"headless": False})

import omni.kit.app
import omni.timeline
import omni.usd

from pxr import Sdf, UsdGeom

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

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
                        get_stage_meters_per_unit, settle_rigid_props,
                        settle_selection)
from scene_generator import generate_scene_on_stage, resolve_sky
# Accepts a config at either level: a high-level disaster spec is
# compiled in memory, a low-level scene config is used as is.
from compile_disaster import load_scene_config


# --------------------- CONFIGURATION ---------------------
# Base environment to scatter assets onto. A flat/terrain-only env is the usual
# choice; the generator supplies the buildings/props.
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]

# Scale applied to /World/stage. 0.01 converts cm→m for Nucleus assets;
# 1.0 if the environment is already in meters.
# STAGE_SCALE = 0.01
STAGE_SCALE = 1.00

# Scene spec consumed by the generator. Relative to scene_gen/.
# The SCENE_CONFIG env var wins when set, so `airstack up` can target a scene
# without editing this file — it takes any form load_scene_config() accepts
# (bare name, high-level spec, or compiled low-level config).
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or os.path.join(
    _SCENE_GEN_DIR, "config",
    "presets", "tornado.yaml")

# Raycast each generated asset down onto terrain colliders for its Z. Needs the
# physics scene stepped first; leave False for flat ground at the configured z.
SNAP_TO_GROUND = False

DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"
# ---------------------------------------------------------


# Enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.graph.core",
    "omni.graph.action",
    "omni.graph.action_nodes",
    "isaacsim.core.nodes",
    "omni.graph.ui",
    "omni.graph.visualization.nodes",
    "omni.graph.scriptnode",
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.graph.ui_nodes",
    "pegasus.simulator",
    "omni.kit.window.script_editor",
]:
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled_immediate(ext, True)


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
                carb.log_info(f"[scene_gen] Removed {path}")
            except Exception:
                UsdGeom.Imageable(child).MakeInvisible()
                carb.log_info(f"[scene_gen] Hid {path} (in referenced layer, can't delete)")


def wait_for_stage(stage, timeout_s: float = 10.0):
    """Pump the Kit app loop until /World has content (scene fully loaded)."""
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


class PegasusApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Keep the timeline stopped throughout setup so OmniGraph's
        # OnPlaybackTick never fires mid-build.
        self.timeline.stop()

        # Load base environment
        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        # Remove or hide the GroundPlane and Environment xforms that Isaac Sim /
        # Pegasus adds when loading the base environment. The scene generator
        # supplies its own ground tiles, so these would cause z-fighting and an
        # unwanted visible skybox/backdrop at ground level.
        _remove_env_clutter(stage)

        # ----- Scene preparation -----
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        # ----- Procedural scene generation -----
        # Place metric coordinates into stage-space using the stage's unit scale.
        config = load_scene_config(SCENE_CONFIG)
        _, scene_scale_factor = get_stage_meters_per_unit(stage)
        placements = generate_scene_on_stage(
            stage,
            config,
            parent_path="/World/stage/generated",
            scene_scale_factor=scene_scale_factor,
            snap_to_ground=SNAP_TO_GROUND,
        )

        # Apply colliders to the freshly placed assets so the drone can sense them.
        generated_prim = stage.GetPrimAtPath("/World/stage/generated")
        if generated_prim.IsValid():
            add_colliders(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        # Drop toppled/strewn props (flipped cars, downed poles) to a natural
        # resting pose under physics, then freeze them. Must run before the
        # drone graph is spawned since it briefly plays the timeline.
        settle_rigid_props(
            stage,
            settle_selection(placements),
            ground_path="/World/stage/generated/ground",
        )

        # Sky + ambient light: borrowed stage prims or HDRI dome per config `sky:`.
        add_sky(stage, resolve_sky(config))

        # ----- Spawn drone OmniGraph -----
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/base_link",
            robot_name="robot_1",
            vehicle_id=1,   # MAVLink port = 14540 + vehicle_id
            domain_id=1,    # ROS 2 domain ID — match vehicle_id by convention
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )

        add_rtx_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            lidar_config="ouster_os1",
            lidar_topic_name="point_cloud_raw",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            min_range=0.75,
        )

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

    def run(self):

        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            world = World.instance()
            if world is not None and hasattr(world, '_scene'):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        carb.log_warn("Closing simulation.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
