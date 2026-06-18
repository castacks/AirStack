#!/usr/bin/env python
"""
SVG ground control: multi-drone PX4 Pegasus launcher, single ROS domain.

Adapted from example_multi_px4_pegasus_launch_script.py with one key
difference: ALL drones share one ROS_DOMAIN_ID (default 0) and are isolated
by namespace (drone_1 ... drone_N) instead of by domain. This lets the
central svg_ground_control swarm commander (and its CBF safety filter) see
every drone's state from a single node.

PX4 SITL instance i keeps the standard per-instance MAVLink ports
(offboard 14540+i, onboard 14580+i, system id 1+i), matching
svg_ground_control/scripts/launch_sim_interfaces.sh.

Env:
 - NUM_ROBOTS (default 3): how many drones to spawn
 - SVG_DOMAIN_ID (default 0): the shared ROS domain
 - ENABLE_LIDAR (default false): attach an Ouster lidar to each drone
 - PLAY_SIM_ON_START (default true): autoplay timeline
 - DRONE_MODES (default all "sim"): comma-separated 'sim'|'real' per drone,
   length NUM_ROBOTS. 'sim' spawns a PX4 SITL body as usual; 'real' spawns a
   visual-only AVATAR (no SITL) that is teleported every step to that drone's
   /{name}/odometry_conversion/odometry — so a real (mocap/hardware) drone
   appears in the Isaac viewport at its live pose. Run with a GUI viewport
   (ISAAC_SIM_HEADLESS=false) and ROS_DOMAIN_ID matching the drones to see it.
"""

import asyncio
import os
import sys
import time

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports
_headless = os.environ.get("ISAAC_SIM_HEADLESS", "false").lower() == "true"
simulation_app = SimulationApp({"headless": _headless})

import omni.kit.app
import omni.timeline
import omni.usd

from pxr import Gf, UsdGeom

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light


# --------------------- CONFIGURATION ---------------------
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.0
DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "3"))
SVG_DOMAIN_ID = int(os.environ.get("SVG_DOMAIN_ID", "0"))
ENABLE_LIDAR = os.environ.get("ENABLE_LIDAR", "false").lower() == "true"


def _parse_drone_modes():
    """Per-drone 'sim'|'real' from DRONE_MODES (default all 'sim')."""
    raw = [m.strip() for m in os.environ.get("DRONE_MODES", "").split(",") if m.strip()]
    if not raw:
        return ["sim"] * NUM_ROBOTS
    if len(raw) != NUM_ROBOTS:
        raise ValueError(
            f"DRONE_MODES has {len(raw)} entries but NUM_ROBOTS={NUM_ROBOTS}")
    for m in raw:
        if m not in ("sim", "real"):
            raise ValueError(f"DRONE_MODES entries must be sim|real, got '{m}'")
    return raw


DRONE_MODES = _parse_drone_modes()

# The PX4 SITL drones get their ROS domain via the OmniGraph ROS2Context
# (domain_id=SVG_DOMAIN_ID), but the in-process rclpy node that drives the
# real-drone avatars reads ROS_DOMAIN_ID from the environment. Align them so
# the avatar bridge subscribes to the drones' odometry on the right domain.
os.environ["ROS_DOMAIN_ID"] = str(SVG_DOMAIN_ID)
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


def spawn_drone(index: int):
    """Spawn drone_<index> with vehicle_id=index on the shared ROS domain."""
    robot_name = f"drone_{index}"
    drone_prim = f"/World/drone{index}/base_link"
    # Spread drones along X, centered near origin
    init_x = 2.0 * (index - 1) - 2.0 * (NUM_ROBOTS - 1) / 2.0

    graph_handle = spawn_px4_multirotor_node(
        pegasus_node_name=f"PX4Multirotor_{index}",
        drone_prim=drone_prim,
        robot_name=robot_name,
        vehicle_id=index,
        domain_id=SVG_DOMAIN_ID,
        usd_file=DRONE_USD,
        init_pos=[init_x, 0.0, 0.07],
        init_orient=[0.0, 0.0, 0.0, 1.0],
    )

    if ENABLE_LIDAR:
        add_rtx_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim=drone_prim,
            robot_name=robot_name,
            lidar_config="ouster_os1",
            lidar_topic_name="point_cloud_raw",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            min_range=0.75,
        )


def spawn_avatar(index: int, stage):
    """Spawn a visual-only avatar for a REAL drone (no PX4 SITL, no physics).

    References the same iris USD as the SITL bodies so it looks like a drone,
    but is just an Xform we teleport every step (see AvatarBridge) to the real
    drone's odometry. Returns (robot_name, translate_op, orient_op) so the
    bridge can update the transform in place. The per-step transform override
    dominates any embedded physics, so the avatar simply tracks the pose.
    """
    robot_name = f"drone_{index}"
    avatar_prim = f"/World/avatars/drone{index}"
    prim = stage.DefinePrim(avatar_prim, "Xform")
    prim.GetReferences().AddReference(os.path.expanduser(DRONE_USD))
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    orient_op = xform.AddOrientOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.07))
    orient_op.Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    xform.SetXformOpOrder([translate_op, orient_op])
    print(f"[svg_multi_drone] drone_{index} = REAL -> visual avatar, "
          f"tracking /{robot_name}/odometry_conversion/odometry")
    return robot_name, translate_op, orient_op


class AvatarBridge:
    """rclpy node that drives the real-drone avatars from their odometry.

    One subscription per real drone; ``update()`` (called each sim step) pumps
    rclpy once and writes the latest pose into each avatar's USD transform ops.
    Lives in the Isaac process — rclpy is available here (the Pegasus
    ros2_backend uses it too).
    """

    def __init__(self, avatars):
        # avatars: list of (robot_name, translate_op, orient_op)
        import rclpy
        from nav_msgs.msg import Odometry

        self._rclpy = rclpy
        try:
            rclpy.init()
        except Exception:  # already initialised elsewhere in the process
            pass
        self.node = rclpy.create_node("svg_avatar_bridge")
        self.ops = {name: (t, o) for name, t, o in avatars}
        self.latest = {}
        for name in self.ops:
            topic = f"/{name}/odometry_conversion/odometry"
            self.node.create_subscription(
                Odometry, topic, lambda msg, n=name: self._on_odom(n, msg), 10)

    def _on_odom(self, name, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest[name] = ((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))

    def update(self):
        self._rclpy.spin_once(self.node, timeout_sec=0.0)
        for name, (translate_op, orient_op) in self.ops.items():
            pose = self.latest.get(name)
            if pose is None:
                continue
            (px, py, pz), (qw, qx, qy, qz) = pose
            translate_op.Set(Gf.Vec3d(px, py, pz))
            orient_op.Set(Gf.Quatf(qw, qx, qy, qz))

    def shutdown(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass


class PegasusApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Keep the timeline stopped throughout setup so OmniGraph doesn't tick early.
        self.timeline.stop()

        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        add_dome_light(stage)

        n_real = DRONE_MODES.count("real")
        print(f"[svg_multi_drone] Spawning {NUM_ROBOTS} drone(s) on ROS domain "
              f"{SVG_DOMAIN_ID}, modes={DRONE_MODES} "
              f"({NUM_ROBOTS - n_real} SITL + {n_real} avatar), "
              f"lidar={'on' if ENABLE_LIDAR else 'off'}")
        avatars = []
        for i in range(1, NUM_ROBOTS + 1):
            if DRONE_MODES[i - 1] == "real":
                avatars.append(spawn_avatar(i, stage))
            else:
                spawn_drone(i)

        # Bridge driving the real-drone avatars from their odometry. Created
        # only if there is at least one real drone, so pure-sim runs are
        # unchanged (and never import rclpy here).
        self.avatar_bridge = AvatarBridge(avatars) if avatars else None

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
            # Teleport real-drone avatars to their latest odometry every loop.
            if self.avatar_bridge is not None:
                self.avatar_bridge.update()

        carb.log_warn("Closing simulation.")
        if self.avatar_bridge is not None:
            self.avatar_bridge.shutdown()
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
