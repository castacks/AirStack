#!/usr/bin/env python3

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment (Must start this before importing omni modules)
simulation_app = SimulationApp({"headless": False})

# Set local Nucleus as asset root before importing Pegasus (which resolves it at import time)
carb.settings.get_settings().set(
    "/persistent/isaac/asset_root/default",
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
)

import os
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd
import omni.client

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph

# gps_utils lives in the same directory as this script
_LAUNCH_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_SCRIPTS_DIR)
from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
import scene_prep
from scene_prep import (
    scale_stage_prim, add_colliders, add_dome_light, get_stage_meters_per_unit,
    reference_root_prims_under_world,
    add_orthographic_camera, add_overhead_camera_publisher,
)


# --------------------- CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"

#env/stage path and scale
ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/scenes/urban/allegheny_county_fire_academy/fire_academy.scene.usd"
#f"omniverse://{NUCLEUS_SERVER}/Library/Assets/FireAcademyFaro/fire_academy_faro.usd"
#f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/RayFronts-Planner/FireAcademy.scene.usd"
#f"omniverse://{NUCLEUS_SERVER}/Library/Assets/Fire_Academy_Digital_Twin/fire_academy.usd"
STAGE_SCALE = 0.01

DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

# Lighting
ADD_DOME_LIGHT = False
DOME_LIGHT_PATH = "/World/DomeLight"
DOME_LIGHT_INTENSITY = 3500.0
DOME_LIGHT_EXPOSURE = -3.0

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
SPAWN_HEIGHT_ABOVE_FLOOR_M = 0.03
DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": 27.0, "y_m": 7.6, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
    {"domain_id": 2, "x_m": 23.0, "y_m": 9.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 4.0},
]

# Top-down "map" camera over (0, 0). Captures one aerial of the static scene
# that the GCS visualizer turns into a textured ground in Foxglove's 3D panel.
OVERHEAD_ALTITUDE_M    = 150.0
OVERHEAD_COVERAGE_M    = 200.0   # per-map knob: world meters per side.
OVERHEAD_PX_PER_METER  = 4.0     # Source-image density. Bump for sharper texture.
OVERHEAD_TOPIC         = "/sim/overhead/image"
OVERHEAD_SPEC_TOPIC    = "/sim/overhead/spec"
OVERHEAD_FRAME_ID      = "map"
OVERHEAD_DOMAIN_ID     = 0
# ---------------------------------------------------------


ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.physx.forcefields",
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "airlab.pegasus",                   # Airlab extension Pegasus core extension
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

        # Load environment (places geometry at /World/stage)
        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        # ----- Scene preparation -----
        # Bring in sky/sun/environment prims that sit outside /World in the source USD
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
        )
        add_overhead_camera_publisher(
            parent_graph_path="/World/MapCameraGraph",
            camera_prim_path=cam_path,
            topic=OVERHEAD_TOPIC,
            spec_topic=OVERHEAD_SPEC_TOPIC,
            frame_id=OVERHEAD_FRAME_ID,
            coverage_m=OVERHEAD_COVERAGE_M,
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

            add_ouster_lidar_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=f"/World/drone{i}/base_link",
                robot_name=f"robot_{i}",
                lidar_name="OS1_REV6_128_10hz___512_resolution",
                lidar_offset=[0.0, 0.0, 0.025],
                lidar_rotation_offset=[0.0, 0.0, 0.0],
                lidar_min_range=cfg["lidar_min_range"],
            )

        # Give physics time to register the spawned drone prims before reset.
        # Complex scenes (e.g. fire academy) need more ticks than simpler ones.
        for _ in range(30):
            omni.kit.app.get_app().update()

        # Reset so physics/articulations are ready
        self.world.reset()

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"
        self.stop_sim = False

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        # Main loop
        while simulation_app.is_running() and not self.stop_sim:
            try:
                self.world.step(render=True)
            except Exception as e:
                carb.log_error(f"Error during simulation step: {e}")
                break

        # Cleanup
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
