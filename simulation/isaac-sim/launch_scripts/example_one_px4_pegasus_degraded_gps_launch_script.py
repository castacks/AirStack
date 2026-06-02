#!/usr/bin/env python
"""
Single-drone PX4 Pegasus launch with realistic GPS degradation.

Extends example_one_px4_pegasus_launch_script.py with:
  - Urban canyon walls applying UsdPhysics.CollisionAPI for PhysX raycasting
  - GPSDegradationNode running in the Isaac Sim physics loop
  - Optional jamming zone (sphere) that forces DENIED state
  - Publishes /robot_1/sensors/gps_degraded and /robot_1/gps/degradation_state
"""

import carb
from isaacsim import SimulationApp

# SimulationApp MUST be created before any omni.* imports
simulation_app = SimulationApp({"headless": False})

import os
import sys
import time
import math

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import UsdPhysics, UsdGeom, Gf

from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

import rclpy

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")))
from gps_degradation_node import GPSDegradationNode
from gps_degradation_staging import GpsDegradationConfig

# ----------------------------- CONFIGURATION --------------------------------

ENV_URL    = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.0
DRONE_USD  = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

GPS_DEGRADATION_CONFIG = GpsDegradationConfig(
    scenario="urban",
    gps_update_every_n_steps=20,       # 5 Hz GPS from 100 Hz physics
    ou_correlation_time_s=300.0,
    ou_base_xy_noise_m_sqrts=0.08,
    ou_base_z_noise_m_sqrts=0.12,
    uere_base_m=0.3,
    cn0_zenith_dbhz=45.0,
    cn0_floor_dbhz=30.0,
)

# Jamming zone: sphere in world coordinates. Set radius=0 to disable.
JAMMING_CENTER_M = (50.0, 0.0, 10.0)   # (x, y, z)
JAMMING_RADIUS_M = 15.0

# ----------------------------------------------------------------------------


def _apply_collision(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid() and not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)


def _add_urban_canyon(stage):
    """Two tall walls flanking the take-off pad to create a GPS canyon."""
    walls = [
        ("/World/CanyonWall_L", ( 20.0,  0.0, 25.0),  (2.0, 30.0, 25.0)),
        ("/World/CanyonWall_R", (-20.0,  0.0, 25.0),  (2.0, 30.0, 25.0)),
    ]
    for path, pos, half_scale in walls:
        prim = stage.DefinePrim(path, "Cube")
        UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*pos))
        UsdGeom.XformCommonAPI(prim).SetScale(Gf.Vec3f(*half_scale))
        _apply_collision(stage, path)


def _wait_for_stage(stage, timeout_s: float = 10.0) -> bool:
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        wp = stage.GetPrimAtPath("/World")
        if wp.IsValid():
            children = [c for c in wp.GetChildren() if c.GetName() != "PhysicsScene"]
            if children:
                return True
        time.sleep(0.1)
    return False


class PegasusAppDegradedGPS:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.timeline.stop()

        # Enable required extensions
        ext_mgr = omni.kit.app.get_app().get_extension_manager()
        for ext in [
            "omni.graph.core", "omni.graph.action", "omni.graph.action_nodes",
            "isaacsim.core.nodes", "omni.graph.scriptnode", "pegasus.simulator",
        ]:
            if not ext_mgr.is_extension_enabled(ext):
                ext_mgr.set_extension_enabled_immediate(ext, True)

        self.pg.load_environment(ENV_URL)
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not _wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale/collision.")

        add_dome_light(stage)

        # Urban canyon walls with collision meshes for PhysX raycasting
        _add_urban_canyon(stage)

        # Spawn drone
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/base_link",
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=1,
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )
        add_rtx_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            lidar_config="ouster_os1",
            lidar_topic_name="point_cloud_raw",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            min_range=0.75,
        )

        # GPS degradation node
        if not rclpy.ok():
            rclpy.init()
        self._gps_node = GPSDegradationNode(
            vehicle_id=1,
            cfg=GPS_DEGRADATION_CONFIG,
        )

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

    def _check_jamming(self, world_pos) -> bool:
        if JAMMING_RADIUS_M <= 0.0:
            return False
        dx = world_pos[0] - JAMMING_CENTER_M[0]
        dy = world_pos[1] - JAMMING_CENTER_M[1]
        dz = world_pos[2] - JAMMING_CENTER_M[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) < JAMMING_RADIUS_M

    def _get_drone_world_pos(self):
        """Read drone prim transform from USD stage."""
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath("/World/base_link")
            if not prim.IsValid():
                return (0.0, 0.0, 0.0)
            xform = UsdGeom.Xformable(prim)
            mat = xform.ComputeLocalToWorldTransform(0)
            t = mat.ExtractTranslation()
            return (float(t[0]), float(t[1]), float(t[2]))
        except Exception:
            return (0.0, 0.0, 0.0)

    def run(self):
        if self.play_on_start:
            self.timeline.play()

        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            world = World.instance()
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()
                continue

            sim_time_s = float(world.current_time)
            world_pos = self._get_drone_world_pos()
            jamming = self._check_jamming(world_pos)

            self._gps_node.set_world_pos(*world_pos)
            self._gps_node.set_jamming(jamming)
            self._gps_node.physics_step(sim_time_s)
            rclpy.spin_once(self._gps_node, timeout_sec=0.0)

        carb.log_warn("Closing simulation.")
        self.timeline.stop()
        rclpy.shutdown()
        simulation_app.close()


def main():
    PegasusAppDegradedGPS().run()


if __name__ == "__main__":
    main()
