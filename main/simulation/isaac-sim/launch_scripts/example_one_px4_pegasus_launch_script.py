#!/usr/bin/env python
"""
Example single-drone PX4 launcher with scene preparation.

Demonstrates:
 - Loading a Pegasus world with an environment
 - Scaling the environment prim and adding collision geometry
 - Adding a dome light
 - Spawning a PX4 multirotor with ZED camera and RTX lidar
 - Optionally saving the prepared scene as a self-contained USD
   (pass ``save_scene_to=`` below)

Env (see pegasus_app.py): ISAAC_SIM_LIVESTREAM, ISAAC_SIM_HEADLESS,
PLAY_SIM_ON_START.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Must be created before any omni/pegasus imports.
simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp  # noqa: E402


def main():
    PegasusApp(
        # Environment to load. Swap this URL/key for any other scene.
        env_url=SIMULATION_ENVIRONMENTS["Default Environment"],
        # 0.01 converts cm→m for Nucleus assets; 1.0 if already in meters.
        stage_scale=1.0,
        drone_configs=[
            {
                "domain_id": 1,  # MAVLink port = 14540 + vehicle_id (= domain_id)
                "x_m": 0.0, "y_m": 0.0, "z_m": 0.07,
                # Single-drone scenes keep the historical prim/node names.
                "prim": "/World/base_link",
                "node_name": "PX4Multirotor",
            }
        ],
        enable_lidar=True,
    ).run()


if __name__ == "__main__":
    main()
