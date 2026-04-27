#!/usr/bin/env python
"""
Benchmark 3 — Full PX4 SITL + OmniGraph + RTX sensors, MAVLink back-and-forth flight.

Demonstrates:
 - Loading a Pegasus world with an environment
 - Scaling the environment prim and adding collision geometry
 - Adding a dome light
 - Spawning a PX4 multirotor with ZED camera and Ouster lidar
 - Autonomous back-and-forth flight via MAVLink OFFBOARD mode
 - Optionally saving the prepared scene as a self-contained USD

Compare real-time ratio with:
  • test_nonlinear_controller_no_px4.py   — bare Multirotor, no sensors
  • test_omnigraph_sensors_nonlinear.py   — sensors, no PX4 process
"""

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports
simulation_app = SimulationApp({"headless": False})

import os
import sys
import time
import math
import threading

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light, save_scene_as_contained_usd, spawn_falling_cubes, SimTimer


# ---------------------------------------------------------------------------
# MAVLink back-and-forth commander
# ---------------------------------------------------------------------------

class MavlinkBackAndForthCommander(threading.Thread):
    """Daemon thread that arms the drone and flies it back and forth via MAVLink OFFBOARD.

    Connects to PX4 SITL's GCS UDP port (14540 + vehicle_id).
    Pre-streams setpoints, switches to OFFBOARD, arms, climbs to CRUISE_ALT,
    then sends sinusoidal SET_POSITION_TARGET_LOCAL_NED messages.
    """

    CRUISE_ALT  = 3.0    # m (NED: negative is up → -3.0)
    HALF_RANGE  = 4.0    # m amplitude
    PERIOD_S    = 12.0   # s per back-and-forth cycle
    SETPOINT_HZ = 20     # Hz stream rate for OFFBOARD to stay active

    def __init__(self, vehicle_id: int = 1):
        super().__init__(daemon=True)
        self._stop_evt = threading.Event()
        self._vehicle_id = vehicle_id
        # GCS port: 14540 for vehicle 0, 14541 for vehicle 1, …
        self._port = f"udpin:localhost:{14540 + vehicle_id}"

    def stop(self):
        self._stop_evt.set()

    def _connect(self):
        try:
            from pymavlink import mavutil
        except ImportError:
            carb.log_error("pymavlink not installed — MAVLink commander disabled.")
            return None
        conn = mavutil.mavlink_connection(self._port)
        conn.wait_heartbeat(timeout=30)
        carb.log_warn(f"[commander] Connected to PX4 on {self._port}")
        return conn

    def _send_sp(self, conn, x, y, z_ned):
        """Send SET_POSITION_TARGET_LOCAL_NED (type-mask = position only)."""
        conn.mav.set_position_target_local_ned_send(
            0,                           # time_boot_ms (irrelevant in SITL)
            conn.target_system,
            conn.target_component,
            1,                           # MAV_FRAME_LOCAL_NED
            0b0000111111111000,          # type-mask: enable pos x/y/z
            x, y, z_ned,                 # position (NED)
            0, 0, 0,                     # velocity
            0, 0, 0,                     # acceleration
            0, 0,                        # yaw, yaw_rate
        )

    def run(self):
        conn = self._connect()
        if conn is None:
            return

        dt = 1.0 / self.SETPOINT_HZ

        # Pre-stream setpoints so OFFBOARD mode can be engaged
        carb.log_warn("[commander] Pre-streaming setpoints …")
        for _ in range(int(self.SETPOINT_HZ * 2)):
            if self._stop_evt.is_set():
                return
            self._send_sp(conn, 0.0, 0.0, -self.CRUISE_ALT)
            time.sleep(dt)

        # Switch to OFFBOARD
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            176,        # MAV_CMD_DO_SET_MODE
            0,
            1,          # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            6,          # PX4 OFFBOARD custom mode
            0, 0, 0, 0, 0,
        )
        time.sleep(0.5)

        # Arm
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            400,        # MAV_CMD_COMPONENT_ARM_DISARM
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        carb.log_warn("[commander] Armed — climbing …")

        t = 0.0
        while not self._stop_evt.is_set():
            if t < 5.0:
                # Climb phase
                self._send_sp(conn, 0.0, 0.0, -self.CRUISE_ALT)
            else:
                # Sinusoidal back-and-forth
                phase = 2.0 * math.pi * (t - 5.0) / self.PERIOD_S
                x = self.HALF_RANGE * math.sin(phase)
                self._send_sp(conn, x, 0.0, -self.CRUISE_ALT)
            time.sleep(dt)
            t += dt


# --------------------- CONFIGURATION ---------------------
# Environment to load. Swap this URL/key for any other scene.
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]

# Scale applied to /World/stage. 0.01 converts cm→m for Nucleus assets.
# Set to 1.0 if the environment is already in meters.
STAGE_SCALE = 1.0

# Set to a directory path to export a self-contained USD after scene prep.
# Set to None to skip saving.
SAVE_SCENE_TO = None  # e.g. os.path.expanduser("~/AirStack/my_scene/")

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
]:
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled_immediate(ext, True)


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

        # Keep the timeline stopped throughout setup so that OmniGraph's
        # OnPlaybackTick never fires.
        self.timeline.stop()

        # Load environment
        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        # Wait for the environment to finish loading before modifying it
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        # ----- Scene preparation -----

        # Scale /World/stage if the asset uses non-metric units (e.g. cm).
        # Remove or set STAGE_SCALE=1.0 if the environment is already in meters.
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)

            # Apply CollisionAPI to every mesh so physics works correctly
            add_colliders(stage_prim)

            # Let the app process the transform and collision changes
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        # Add a dome light for uniform scene illumination.
        # Pass intensity/exposure kwargs to override defaults defined in scene_prep.
        add_dome_light(stage)
        spawn_falling_cubes(stage)

        # Optionally save the prepared scene as a self-contained USD package.
        # The Collector copies all Nucleus-hosted textures and MDLs locally.
        if SAVE_SCENE_TO:
            import tempfile
            tmp_usd = os.path.join(tempfile.gettempdir(), "prepared_scene.usd")
            success, error = asyncio.get_event_loop().run_until_complete(
                omni.usd.get_context().export_as_stage_async(tmp_usd)
            )
            if success:
                os.makedirs(SAVE_SCENE_TO, exist_ok=True)
                save_scene_as_contained_usd(tmp_usd, SAVE_SCENE_TO)
                os.remove(tmp_usd)
            else:
                carb.log_error(f"Scene export failed: {error}")

        # ----- Spawn drone OmniGraph -----
        # This only creates the graph topology. The actual drone + PX4
        # backend are created by compute_base on the first Play tick.

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

        # Add a ZED stereo camera subgraph to the drone
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],       # X, Y, Z offset from base_link
            camera_rotation_offset=[0.0, 0.0, 0.0], # roll, pitch, yaw in degrees
        )

        # Add an Ouster lidar subgraph to the drone
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],         # X, Y, Z offset from base_link
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range=0.75,                    # avoid propeller hits
        )

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

        # Start the MAVLink back-and-forth commander in the background.
        # It will connect once PX4 SITL is up and the sim is playing.
        self.commander = MavlinkBackAndForthCommander(vehicle_id=1)
        self.commander.start()

        self.timer = SimTimer("px4-full", interval_s=5.0)

    def run(self):

        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            # File → Save re-opens the stage, which invalidates the World.
            # Fall back to app.update() until the extension re-creates it.
            world = World.instance()
            if world is not None and hasattr(world, '_scene'):
                world.step(render=True)
                self.timer.tick(self.world.current_time)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        carb.log_warn("Closing simulation.")
        self.commander.stop()
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
