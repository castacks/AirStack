#!/usr/bin/env python3
"""
Multi-drone launcher for the Fire Academy digital-twin scene (Nucleus-hosted).

Demonstrates a scene-import scenario on top of ``pegasus_app.PegasusApp``:
 - Nucleus asset root override (must happen before Pegasus imports)
 - Referencing root-level sky/sun prims from the source USD
 - Physics-scene dedupe for imported scenes
 - A top-down orthographic "map" camera published for the GCS visualizer
 - Explicit per-drone spawn poses (world meters, scaled to stage units)
"""

import os
import sys

import carb

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Must be created before any omni/pegasus imports.
simulation_app = create_simulation_app()

# Set local Nucleus as asset root before importing Pegasus (which resolves it at import time)
carb.settings.get_settings().set(
    "/persistent/isaac/asset_root/default",
    "omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
)

import omni.client  # noqa: E402

from pegasus_app import PegasusApp  # noqa: E402
from gps_utils import DEFAULT_WORLD_ORIGIN  # noqa: E402
from scene_prep import (  # noqa: E402
    dedupe_physics_scenes,
    reference_root_prims_under_world,
    add_orthographic_camera,
    add_overhead_camera_publisher,
)


# --------------------- CONFIGURATION ---------------------
NUCLEUS_SERVER = "airlab-nucleus.andrew.cmu.edu"

# env/stage path and scale
ENV_URL = f"omniverse://{NUCLEUS_SERVER}/Projects/AirStack/scenes/urban/allegheny_county_fire_academy/fire_academy.scene.usd"

STAGE_SCALE = 0.01

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
    {"domain_id": 1, "x_m": 32.0, "y_m": 12.6, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 0.75},
    {"domain_id": 2, "x_m": 28.0, "y_m": 14.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 0.75},
    {"domain_id": 3, "x_m": 32.0, "y_m": 19.8, "z_m": SPAWN_HEIGHT_ABOVE_FLOOR_M, "orient": [0.0, 0.0, -0.937, 0.35], "lidar_min_range": 0.75}
]

# NOTE: this scene historically uses a ZED offset that differs from the
# canonical [0.2, 0.0, -0.05] used by every other script AND from the URDF
# (iris_with_sensors.pegasus.robot.urdf). Preserved as-is pending a decision;
# see notebook entry / RFC #380 vehicle configs which will make this a single
# source of truth.
ZED_CAMERA_OFFSET = [0.21, 0.0, 0.05]

# Top-down "map" camera. Captures one aerial of the static scene that the
# GCS visualizer turns into a textured ground in Foxglove's 3D panel. The
# camera centers on (OVERHEAD_CENTER_X_M, OVERHEAD_CENTER_Y_M) in world
# meters — leave both 0.0 for the legacy origin-centered behavior.
OVERHEAD_ALTITUDE_M    = 165.0
OVERHEAD_COVERAGE_M    = 225  # per-map knob: world meters per side.
OVERHEAD_CENTER_X_M    = 0.0  # -152     # world-X of camera center / texture center.
OVERHEAD_CENTER_Y_M    = 0.0  # -80      # world-Y of camera center / texture center.
OVERHEAD_PX_PER_METER  = 10.0  # Source-image density. Bump for sharper texture.
OVERHEAD_TOPIC         = "/sim/overhead/image"
OVERHEAD_SPEC_TOPIC    = "/sim/overhead/spec"
OVERHEAD_CENTER_X_TOPIC = "/sim/overhead/center_x"
OVERHEAD_CENTER_Y_TOPIC = "/sim/overhead/center_y"
OVERHEAD_FRAME_ID      = "map"
OVERHEAD_DOMAIN_ID     = 0
# ---------------------------------------------------------


def nucleus_stat(url: str) -> bool:
    result, info = omni.client.stat(url)
    return result == omni.client.Result.OK


class SceneImportApp(PegasusApp):

    def pre_scene_prep(self, stage):
        dedupe_physics_scenes(stage)

        # Bring in sky/sun/environment prims that sit at root level in the
        # source USD next to the defaultPrim that pg.load_environment already
        # loaded into /World/stage. reference_root_prims_under_world skips
        # the defaultPrim, so this can't duplicate geometry.
        reference_root_prims_under_world(stage, ENV_URL)

    def post_scene_prep(self, stage):
        # Top-down orthographic camera over (0, 0). Publishes one JPEG aerial
        # of the static scene at low rate; the GCS visualizer republishes it
        # as a textured ground for Foxglove's 3D panel.
        cam_path = add_orthographic_camera(
            stage,
            prim_path="/World/MapCamera",
            altitude_m=OVERHEAD_ALTITUDE_M,
            coverage_m=OVERHEAD_COVERAGE_M,
            scene_scale_factor=self._overhead_scale(stage),
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

    @staticmethod
    def _overhead_scale(stage):
        from scene_prep import get_stage_meters_per_unit

        _, s = get_stage_meters_per_unit(stage)
        return s


def main():
    omni.client.initialize()
    nucleus_stat(f"omniverse://{NUCLEUS_SERVER}")
    nucleus_stat(ENV_URL)

    SceneImportApp(
        env_url=ENV_URL,
        stage_scale=STAGE_SCALE,
        drone_configs=DRONE_CONFIGS,
        camera_offset=ZED_CAMERA_OFFSET,
        enable_lidar=True,
        dome_light=False,
        # GPS origins are written immediately so robot containers can read them
        # before this container finishes its heavy USD loading.
        world_gps_origin=WORLD_GPS_ORIGIN,
        # Spawn poses are authored in world meters; this scene's stage is cm.
        scale_spawn_positions=True,
    ).run()


if __name__ == "__main__":
    main()
