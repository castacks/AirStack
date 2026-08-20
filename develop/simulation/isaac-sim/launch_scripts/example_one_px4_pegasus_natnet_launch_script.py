#!/usr/bin/env python
"""
Single-drone PX4 Pegasus launcher with OptiTrack NatNet mocap streaming.

Same scene prep and sensor stack as ``example_one_px4_pegasus_launch_script.py``, plus
a Motive-compatible NatNet server that always streams:

 - ``Drone`` (id 1) from the Pegasus ``body`` prim under ``/World/base_link``
 - ``Target`` (id 100) from a static ``/World/target`` prim

Pair with robot-side ``LAUNCH_NATNET=true`` and a matching ``natnet_config.yaml``
profile. To consume the target on the robot, add a Target body to the profile
(see the commented scaffolding in ``natnet_config.yaml``).

Override rigid-body names with ``NATNET_BODY_NAME`` / ``NATNET_TARGET_NAME``.
"""

import os
import sys

import carb

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Must be created before any omni/pegasus imports.
simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp  # noqa: E402
from gps_utils import DEFAULT_WORLD_ORIGIN  # noqa: E402

# Register the emulator extension with Kit before importing from it.
# See docs/simulation/isaac_sim/natnet_emulator.md.
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("optitrack.natnet.emulator")

from optitrack.natnet.emulator.isaac import (  # noqa: E402
    DEFAULT_TARGET_PATH,
    DEFAULT_TARGET_POSITION,
    DEFAULT_TARGET_STREAMING_ID,
    author_static_target,
    author_drone_natnet_interface,
)

# --------------------- CONFIGURATION ---------------------
# What world (0, 0, 0) maps to in GPS coordinates. Must match the GCS origin and
# the robot's natnet_ros2 mavros_gp_origin.yaml.
WORLD_GPS_ORIGIN = DEFAULT_WORLD_ORIGIN

# Rigid body this scene streams. Must match a body entry in the robot's profile in
# natnet_ros2/config/natnet_config.yaml — a mismatch fails silently.
# See docs/simulation/isaac_sim/natnet_emulator.md.
NATNET_BODY_NAME = os.environ.get("NATNET_BODY_NAME", "Drone")
NATNET_BODY_ID = 1
NATNET_TARGET_NAME = os.environ.get("NATNET_TARGET_NAME", "Target")

_NATNET_SERVER_KWARGS = {
    "pose_noise_enabled": True,
    "pose_noise_std_meters": 0.0005,
    "pose_noise_rotation_deg": 0.05,
}
# ---------------------------------------------------------


class NatNetPegasusApp(PegasusApp):

    def post_spawn(self, stage):
        """Author the NatNet interface prim (drone + static target).

        Runs before the timeline starts; the emulator extension builds the server
        from this prim on Play.
        """
        try:
            author_static_target(stage, DEFAULT_TARGET_PATH, DEFAULT_TARGET_POSITION)
            bodies = [
                (NATNET_BODY_NAME, NATNET_BODY_ID, "/World/base_link/body"),
                (NATNET_TARGET_NAME, DEFAULT_TARGET_STREAMING_ID, DEFAULT_TARGET_PATH),
            ]
            author_drone_natnet_interface(stage, bodies, **_NATNET_SERVER_KWARGS)
            carb.log_warn(
                f"[natnet] Interface authored: '{NATNET_BODY_NAME}' (-> /World/base_link/body), "
                f"'{NATNET_TARGET_NAME}' (-> {DEFAULT_TARGET_PATH})."
            )
        except Exception as exc:  # noqa: BLE001 - never let NatNet kill the sim
            carb.log_error(f"[natnet] Failed to author interface: {exc}")


def main():
    NatNetPegasusApp(
        env_url=SIMULATION_ENVIRONMENTS["Default Environment"],
        stage_scale=1.0,
        drone_configs=[
            {
                "domain_id": 1,
                "x_m": 0.0, "y_m": 0.0, "z_m": 0.07,
                "prim": "/World/base_link",
                "node_name": "PX4Multirotor",
            }
        ],
        enable_lidar=True,
        # Written before the PX4 SITL subprocess starts.
        world_gps_origin=WORLD_GPS_ORIGIN,
    ).run()


if __name__ == "__main__":
    main()
