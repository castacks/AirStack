#!/usr/bin/env python3

import os

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment (Must start this before importing omni modules)
simulation_app = SimulationApp(
    {"headless": os.getenv("ISAAC_SIM_HEADLESS", "false").lower() == "true"})

# Set local Nucleus as asset root before importing Pegasus (which resolves it at import time)
carb.settings.get_settings().set(
    "/persistent/isaac/asset_root/default",
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
)

import json
import os
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd
import omni.client

# pymavlink waits on its socket with select.select(), which cannot handle
# file descriptors >= 1024. DDS-over-TCP (fastdds LARGE_DATA) opens a TCP
# connection per remote participant, pushing this process's fd numbers past
# that, and every MAVLink read then throws "filedescriptor out of range in
# select()". Replace the wait with poll(), which has no fd-number limit.
# Must run before Pegasus creates its MAVLink connections.
import select as _select
import time as _time
from pymavlink import mavutil as _mavutil

def _mavfile_select_poll(self, timeout):
    if self.fd is None:
        _time.sleep(min(timeout, 0.5))
        return True
    poller = _select.poll()
    poller.register(self.fd, _select.POLLIN)
    try:
        return len(poller.poll(timeout * 1000)) > 0
    except OSError:
        return False

_mavutil.mavfile.select = _mavfile_select_poll

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

# gps_utils lives in the same directory as this script
_LAUNCH_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_SCRIPTS_DIR)
from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN
from spawn_utils import generate_spawn_configs, polygon_centroid

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import (
    scale_stage_prim, add_colliders, add_dome_light, get_stage_meters_per_unit,
    reference_root_prims_under_world, dedupe_physics_scenes,
    add_orthographic_camera, add_overhead_camera_publisher,
)


# --------------------- CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"

#env/stage path and scale
_LOCAL_SCENES_DIR = os.path.normpath(os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "assets", "scenes"))
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/Shipyard.usd"
#ENV_URL = f"file://{_LOCAL_SCENES_DIR}/ModernCityDowntown.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/scenes/urban/allegheny_county_fire_academy/fire_academy.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/RetroNeighborhood/RetroNeighborhood.stage.usd"
ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/AbandonedFactory/AbandonedFactory.stage.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/ConstructionSite/ConstructionSite.stage.usd"

# Env override (set by the mission runner per iteration; falls back to the
# hardcoded ENV_URL above when running standalone). The override may be a full
# URL (omniverse://… or file://…) OR just the path after the nucleus server,
# e.g. "Library/Stages/RetroNeighborhood/RetroNeighborhood.stage.usd" — in
# which case the omniverse://{NUCLEUS_SERVER}/ prefix is prepended.
_env_url_override = os.environ.get("ENV_URL")
if _env_url_override:
    ENV_URL = (_env_url_override if "://" in _env_url_override
               else f"omniverse://{NUCLEUS_SERVER}/{_env_url_override.lstrip('/')}")
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/MilitaryBase_t_x1100_y200_z0_o_x0_y0_z90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/AbandonedCity.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/downtown_edited_v3_818.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/copy-rayfronts-planner/environments_start_pos/SnowyVillage_t_x-152_y-80_z-2_o_x0_y0_z_90.scene.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/edit_v1_shipyard.usd"
#ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Library/Stages/Dmytro/ModernCityDowntown.stage.usd"

# Per-environment override (mission runner sets STAGE_SCALE per iteration);
# falls back to 0.01 for standalone runs.
STAGE_SCALE = float(os.environ.get("STAGE_SCALE") or 0.01)

DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

# Lighting
ADD_DOME_LIGHT = False
DOME_LIGHT_PATH = "/World/DomeLight"
DOME_LIGHT_INTENSITY = 3500.0
DOME_LIGHT_EXPOSURE = -5.0

# GPS world anchor: what world (0, 0, 0) maps to in real GPS coordinates.
# Matches the Lisbon default in px4_config.yaml — change here to relocate the sim world.
WORLD_GPS_ORIGIN = DEFAULT_WORLD_ORIGIN

# Drone spawn configs.
# x_m = East offset from world origin (meters)
# y_m = North offset from world origin (meters)
# z_m = Up offset / height above floor (meters)
# orient = initial quaternion [x, y, z, w]
# spawn location for /Assets/Fire_Academy_Digital_Twin/fire_academy.usd:
# {"domain_id": 1, "x_m": 20.0, "y_m": -7.0, ...}
# {"domain_id": 2, "x_m": 17.0, "y_m":  1.5, ...}

# Per-environment override (mission runner sets SPAWN_HEIGHT_M per iteration);
# falls back to 0.3 for standalone runs.
SPAWN_HEIGHT_ABOVE_FLOOR_M = float(os.environ.get("SPAWN_HEIGHT_M") or 0.3)
# DRONE_CONFIGS = [
#     {"domain_id": 1, "x_m": 32.0, "y_m": 12.6, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
#     {"domain_id": 2, "x_m": 28.0, "y_m": 14.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
#     {"domain_id": 3, "x_m": 32.0, "y_m": 19.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0}
#     ]


LIDAR_MIN_RANGE_M = 0.75

# Hardcoded fallback layout, used when SPAWN_POLY is not set (standalone runs).
_DEFAULT_DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": 3.0, "y_m": 3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": LIDAR_MIN_RANGE_M},
    {"domain_id": 2, "x_m": 0.0, "y_m": 0.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": LIDAR_MIN_RANGE_M},
    {"domain_id": 3, "x_m": -3.0, "y_m": -3.0, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, 0.0, 1.0], "lidar_min_range": LIDAR_MIN_RANGE_M},
    ]

# Spawn-area override: when SPAWN_POLY is set (the mission runner sets it per
# iteration), generate a randomized, collision-free layout inside it instead of
# using the fallback above. SPAWN_POLY is JSON — 2 opposite corners for an
# axis-aligned rectangle ("[[11,6.5],[-13,-6.5]]") or >=3 corners for a convex
# polygon. NUM_ROBOTS sets the count; SPAWN_SEED (optional) makes it
# reproducible; SPAWN_MIN_DIST_M / SPAWN_MARGIN_M tune spacing and edge inset.
_SPAWN_POLY = os.environ.get("SPAWN_POLY")
if _SPAWN_POLY:
    _poly = json.loads(_SPAWN_POLY)
    # Seed: honour SPAWN_SEED when set (reproducible layout), otherwise derive a
    # fresh seed from the wall clock + PID so every container bring-up gets a
    # different spawn layout. The effective seed is logged below so any run can
    # still be reproduced by setting SPAWN_SEED to it.
    _spawn_seed = os.environ.get("SPAWN_SEED") or f"{time.time_ns()}-{os.getpid()}"
    DRONE_CONFIGS = generate_spawn_configs(
        _poly,
        n=int(os.environ.get("NUM_ROBOTS", "3")),
        z_m=SPAWN_HEIGHT_ABOVE_FLOOR_M,
        lidar_min_range=LIDAR_MIN_RANGE_M,
        min_dist=float(os.environ.get("SPAWN_MIN_DIST_M") or 3.0),
        margin=float(os.environ.get("SPAWN_MARGIN_M") or 0.0),
        seed=_spawn_seed,
    )
    _spawn_center = polygon_centroid(_poly)
else:
    DRONE_CONFIGS = _DEFAULT_DRONE_CONFIGS
    _spawn_center = (0.0, 0.0)
    _spawn_seed = None

# Logged so each iteration's chosen environment + layout (+ the seed that
# produced it) is captured in the isaac-sim container logs, which mission_runner
# snapshots per iteration.
print(f"[spawn] ENV_URL={ENV_URL}", flush=True)
print(f"[spawn] SPAWN_SEED={_spawn_seed}", flush=True)
print(f"[spawn] DRONE_CONFIGS={json.dumps(DRONE_CONFIGS)}", flush=True)

# Top-down "map" camera. Captures one aerial of the static scene that the
# GCS visualizer turns into a textured ground in Foxglove's 3D panel. The
# camera centers on (OVERHEAD_CENTER_X_M, OVERHEAD_CENTER_Y_M) in world
# meters — leave both 0.0 for the legacy origin-centered behavior.
OVERHEAD_ALTITUDE_M    = 165.0
OVERHEAD_COVERAGE_M    = 300  # per-map knob: world meters per side.
# Default the overhead camera to the centre of the spawn area so the aerial
# texture frames the drones in whichever environment is active (the three test
# scenes sit in completely different coordinate regions). Override explicitly
# with OVERHEAD_CENTER_X_M / OVERHEAD_CENTER_Y_M.
OVERHEAD_CENTER_X_M    = float(os.environ.get("OVERHEAD_CENTER_X_M") or _spawn_center[0])
OVERHEAD_CENTER_Y_M    = float(os.environ.get("OVERHEAD_CENTER_Y_M") or _spawn_center[1])
OVERHEAD_PX_PER_METER  = 10.0     # Source-image density. Bump for sharper texture.
OVERHEAD_TOPIC         = "/sim/overhead/image"
OVERHEAD_SPEC_TOPIC    = "/sim/overhead/spec"
OVERHEAD_CENTER_X_TOPIC = "/sim/overhead/center_x"
OVERHEAD_CENTER_Y_TOPIC = "/sim/overhead/center_y"
OVERHEAD_FRAME_ID      = "map"
OVERHEAD_DOMAIN_ID     = 0
# ---------------------------------------------------------


ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        # Try immediate enable if available (more robust across Kit versions), fall back otherwise
        try:
            ext_manager.set_extension_enabled_immediate(ext, True)
        except Exception:
            ext_manager.set_extension_enabled(ext, True)


def nucleus_stat(url: str) -> bool:
    result, info = omni.client.stat(url)
    return result == omni.client.Result.OK


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
        # Write GPS origins immediately so robot containers can read them
        # before this container finishes its heavy USD loading.
        set_gps_origins(DRONE_CONFIGS, world_origin=WORLD_GPS_ORIGIN)

        omni.client.initialize()
        nucleus_stat(f"omniverse://{NUCLEUS_SERVER}")
        nucleus_stat(ENV_URL)

        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.timeline.stop()

        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        dedupe_physics_scenes(stage)

        # ----- Scene preparation -----
        # Bring in sky/sun/environment prims that sit at root level in the
        # source USD next to the defaultPrim that pg.load_environment already
        # loaded into /World/stage. reference_root_prims_under_world skips
        # the defaultPrim, so this can't duplicate geometry.
        reference_root_prims_under_world(stage, ENV_URL)

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        if ADD_DOME_LIGHT:
            add_dome_light(stage, DOME_LIGHT_PATH, DOME_LIGHT_INTENSITY, DOME_LIGHT_EXPOSURE)

        # Units
        mpu, s = get_stage_meters_per_unit(stage)

        # Top-down orthographic camera over (0, 0). Publishes one JPEG aerial
        # of the static scene at low rate; the GCS visualizer republishes it
        # as a textured ground for Foxglove's 3D panel.
        cam_path = add_orthographic_camera(
            stage,
            prim_path="/World/MapCamera",
            altitude_m=OVERHEAD_ALTITUDE_M,
            coverage_m=OVERHEAD_COVERAGE_M,
            scene_scale_factor=s,
            center_x_m=OVERHEAD_CENTER_X_M,
            center_y_m=OVERHEAD_CENTER_Y_M,
        )
        add_overhead_camera_publisher(
            parent_graph_path="/World/MapCameraGraph",
            camera_prim_path=cam_path,
            topic=OVERHEAD_TOPIC,
            spec_topic=OVERHEAD_SPEC_TOPIC,
            center_x_topic=OVERHEAD_CENTER_X_TOPIC,
            center_y_topic=OVERHEAD_CENTER_Y_TOPIC,
            frame_id=OVERHEAD_FRAME_ID,
            coverage_m=OVERHEAD_COVERAGE_M,
            center_x_m=OVERHEAD_CENTER_X_M,
            center_y_m=OVERHEAD_CENTER_Y_M,
            pixels_per_meter=OVERHEAD_PX_PER_METER,
            domain_id=OVERHEAD_DOMAIN_ID,
        )

        # Spawn all drones
        for cfg in DRONE_CONFIGS:
            i = cfg["domain_id"]
            pos = [cfg["x_m"] * s, cfg["y_m"] * s, cfg["z_m"] * s]

            graph_handle = spawn_px4_multirotor_node(
                pegasus_node_name=f"PX4Multirotor_{i}",
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                vehicle_id=i,
                domain_id=i,
                usd_file=DRONE_USD,
                init_pos=pos,
                init_orient=cfg["orient"],
            )

            add_zed_stereo_camera_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                camera_name="ZEDCamera",
                camera_offset=[0.21, 0.0, 0.05],
                camera_rotation_offset=[0.0, 0.0, 0.0],
            )

            add_rtx_lidar_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                lidar_config="ouster_os1",
                lidar_topic_name="point_cloud_raw",
                lidar_offset=[0.0, 0.0, 0.025],
                lidar_rotation_offset=[0.0, 0.0, 0.0],
                min_range=cfg["lidar_min_range"],
            )

        # Toggle /World/stage off and on to clear a post-scale visual artifact
        # that lingers until the prim is deactivated/reactivated.
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            stage_prim.SetActive(False)
            app = omni.kit.app.get_app()
            wait_end = time.time() + 8.0
            while time.time() < wait_end:
                app.update()
            stage_prim.SetActive(True)

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"
        self.stop_sim = False

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        while simulation_app.is_running() and not self.stop_sim:
            world = World.instance()
            if world is not None and hasattr(world, '_scene'):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()