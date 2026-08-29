#!/usr/bin/env python
"""
Multi-drone PX4 Pegasus launcher, parametrized by NUM_ROBOTS.

Env:
 - NUM_ROBOTS (default 1): how many drones to spawn
 - ENABLE_LIDAR (default false): attach an Ouster lidar to each drone (``add_rtx_lidar_subgraph``;
   publishes ``point_cloud_raw`` under ``/robot_<i>/sensors/ouster/`` via OmniGraph). The
   single-drone example script always enables LiDAR; AirStack pytest ``isaacsim`` liveliness
   sets ``ENABLE_LIDAR=true`` so behavior matches.
 - PLAY_SIM_ON_START (default true): autoplay timeline
 - ISAAC_SIM_SCENE / ISAAC_SIM_STAGE_SCALE (set by `airstack up --scene`):
   scene to load — a Pegasus catalog key or USD URL (default: Default Environment)
 - ISAAC_SIM_HEADLESS / ISAAC_SIM_LIVESTREAM: see pegasus_app.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Must be created before any omni/pegasus imports.
simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import (  # noqa: E402
    PegasusApp,
    resolve_scene_from_env,
    resolve_spawn_center_from_env,
    row_spawn_configs,
)

NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "1"))
ENABLE_LIDAR = os.environ.get("ENABLE_LIDAR", "false").lower() == "true"


def main():
    print(f"[example_multi] Spawning {NUM_ROBOTS} drone(s), lidar={'on' if ENABLE_LIDAR else 'off'}")
    env_url, stage_scale = resolve_scene_from_env(SIMULATION_ENVIRONMENTS)
    print(f"[example_multi] Scene: {env_url} (stage_scale={stage_scale})")
    PegasusApp(
        env_url=env_url,
        stage_scale=stage_scale,
        # Spread drones along X, centered on ISAAC_SIM_SPAWN_XY (default origin)
        drone_configs=row_spawn_configs(
            NUM_ROBOTS, center_xy=resolve_spawn_center_from_env()
        ),
        enable_lidar=ENABLE_LIDAR,
    ).run()


if __name__ == "__main__":
    main()
