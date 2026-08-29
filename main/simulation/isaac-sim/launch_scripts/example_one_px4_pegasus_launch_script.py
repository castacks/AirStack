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
PLAY_SIM_ON_START, and ISAAC_SIM_SCENE / ISAAC_SIM_STAGE_SCALE (set by
`airstack up --scene <shortname>` — a Pegasus catalog key or USD URL;
default: Default Environment).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Must be created before any omni/pegasus imports.
simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp, resolve_scene_from_env  # noqa: E402


def main():
    # Scene from `airstack up --scene` (or set ISAAC_SIM_SCENE to any
    # catalog key / USD URL); stage_scale converts cm-authored stages to m.
    env_url, stage_scale = resolve_scene_from_env(SIMULATION_ENVIRONMENTS)
    print(f"[example_one] Scene: {env_url} (stage_scale={stage_scale})")
    PegasusApp(
        env_url=env_url,
        stage_scale=stage_scale,
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
