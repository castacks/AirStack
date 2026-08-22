#!/usr/bin/env python
"""
Multi-drone PX4 Pegasus launcher with OptiTrack NatNet mocap streaming.

Same scene prep and sensor stack as ``example_multi_px4_pegasus_launch_script.py``,
plus a Motive-compatible NatNet server that always streams one rigid body per drone
and a shared static ``Target`` (id 100) at ``/World/target``.

Body naming:
 - ``NUM_ROBOTS=1``: drone body ``Drone`` (id 1)
 - ``NUM_ROBOTS>1``: ``Drone1``, ``Drone2``, … (ids 1..N)

Intended multi-robot profile pairing (see ``natnet_config.yaml`` commented scaffolding):
 - ``robot_1``: tracks its drone + the shared ``Target``
 - ``robot_2``: tracks its drone + the shared ``Target``
 - ``robot_3``: tracks its drone only (no Target in profile)

Set ``NUM_ROBOTS=3`` on both sim and robot stacks; each container picks its profile
via ``ROBOT_NAME``.

Env:
 - ``NUM_ROBOTS`` (default 1)
 - ``ENABLE_LIDAR`` (default false)
 - ``PLAY_SIM_ON_START`` (default true)
 - ``ISAAC_SIM_HEADLESS`` / ``ISAAC_SIM_LIVESTREAM``: see pegasus_app.py
"""

import os
import sys

import carb

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pegasus_app import create_simulation_app

# Must be created before any omni/pegasus imports.
simulation_app = create_simulation_app()

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS  # noqa: E402
from pegasus_app import PegasusApp, row_spawn_configs  # noqa: E402

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
NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "1"))
ENABLE_LIDAR = os.environ.get("ENABLE_LIDAR", "false").lower() == "true"
# Base name for the streamed drone bodies; drone i is streamed with id i. Must match
# the body entries in the per-robot profiles in natnet_config.yaml.
# See docs/simulation/isaac_sim/natnet_emulator.md.
NATNET_BODY_NAME = "Drone"
NATNET_TARGET_NAME = "Target"

_NATNET_SERVER_KWARGS = {
    "pose_noise_enabled": True,
    "pose_noise_std_meters": 0.0005,
    "pose_noise_rotation_deg": 0.05,
}
# ---------------------------------------------------------


def _drone_body_name(index: int) -> str:
    """Single agent uses bare ``Drone``; multi uses ``Drone1``, ``Drone2``, …"""
    return NATNET_BODY_NAME if NUM_ROBOTS == 1 else f"{NATNET_BODY_NAME}{index}"


class NatNetPegasusApp(PegasusApp):

    def post_spawn(self, stage):
        """Author NatNet bodies: one per drone plus one shared static target.

        Runs before the timeline starts; the emulator extension builds the server
        from this prim on Play.
        """
        try:
            author_static_target(stage, DEFAULT_TARGET_PATH, DEFAULT_TARGET_POSITION)
            bodies = [
                (_drone_body_name(i), i, f"/World/drone{i}/base_link/body")
                for i in range(1, NUM_ROBOTS + 1)
            ]
            bodies.append((NATNET_TARGET_NAME, DEFAULT_TARGET_STREAMING_ID, DEFAULT_TARGET_PATH))

            author_drone_natnet_interface(stage, bodies, **_NATNET_SERVER_KWARGS)
            carb.log_warn(
                f"[natnet] Interface authored with {NUM_ROBOTS} drone body(ies) "
                f"and shared target '{NATNET_TARGET_NAME}' (robot_1/robot_2 subscribe via "
                f"natnet_config; robot_3 omits Target)."
            )
        except Exception as exc:  # noqa: BLE001 - never let NatNet kill the sim
            carb.log_error(f"[natnet] Failed to author interface: {exc}")


def main():
    print(f"[example_multi_natnet] Spawning {NUM_ROBOTS} drone(s), lidar={'on' if ENABLE_LIDAR else 'off'}")
    NatNetPegasusApp(
        env_url=SIMULATION_ENVIRONMENTS["Default Environment"],
        stage_scale=1.0,
        drone_configs=row_spawn_configs(NUM_ROBOTS),
        enable_lidar=ENABLE_LIDAR,
    ).run()


if __name__ == "__main__":
    main()
