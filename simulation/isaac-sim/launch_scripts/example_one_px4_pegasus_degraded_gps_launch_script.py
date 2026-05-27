#!/usr/bin/env python
"""
GPS Degradation SITL Launcher for AirStack.

Full realistic GPS pipeline (always active, no fallback):
  Walker-delta(24:6:1) constellation → PhysX per-satellite raycasting
  → LOS/NLOS classification → C/N0 + lognormal fading filter
  → geometry matrix DOP → pseudorange WLS multipath bias
  → OU (Gauss-Markov) position drift scaled by HDOP/VDOP
  → hysteretic state machine → ramp-rate-limited EKF2 output

Flags:
  --tier2      Lower elevation mask 5° → 2°: admits more multipath-prone
               low-elevation satellites, stresses the signal model.
  --tier3      Activate a spatial jamming zone. DENIAL and NOISE supported;
               SPOOFING/MEACONING handled as DENIAL pending model extension.

Usage:
    isaacsim_python example_one_px4_pegasus_degraded_gps_launch_script.py
    isaacsim_python ... --tier2
    isaacsim_python ... --tier3 --tier3-mode DENIAL
"""

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports.
simulation_app = SimulationApp({"headless": False})

import math
import os
import sys
import time
import asyncio
import argparse

import omni.kit.app
import omni.timeline
import omni.usd
import rclpy

from omni.isaac.core.world import World
from pxr import UsdGeom, UsdPhysics, Gf

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light, save_scene_as_contained_usd

# GPS degradation — staging package and node
_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR = os.path.normpath(os.path.join(_LAUNCH_DIR, ".."))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)
sys.path.insert(0, _LAUNCH_DIR)

from gps_degradation_staging import GpsDegradationConfig
from gps_degradation_node import GPSDegradationNode


# --------------------- CONFIGURATION ---------------------
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.0
SAVE_SCENE_TO = None
DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

GPS_DEGRADATION_CONFIG = {
    # Receiver thermal noise floor — maps to uere_base_m in GpsDegradationConfig.
    # Atmospheric delay variability is handled by the OU process (not here).
    "base_h_accuracy": 0.3,       # metres, 1σ (C/A thermal noise, narrow correlator)

    # --tier2: admit satellites down to 2° elevation (default mask is 5°).
    # Lower-elevation sats have higher multipath probability; enabling this
    # exercises the full signal_model + pseudorange WLS for marginal geometry.
    "tier2_elevation_mask_deg": 2.0,

    # --tier3: spatial jamming zone (world-frame metres, X=East Y=North Z=Up)
    "jamming_enabled": False,
    "jamming_mode": "DENIAL",     # DENIAL | NOISE  (SPOOFING/MEACONING = DENIAL)
    "jamming_zone": {
        "center": [0.0, 0.0, 0.0],   # world-frame metres
        "radius": 50.0,              # metres
    },

    # Scene geometry
    "canyon_width_m":  30.0,
    "canyon_height_m": 20.0,
    "canyon_length_m": 200.0,
}
# ---------------------------------------------------------


def build_gps_config(args) -> GpsDegradationConfig:
    """Build GpsDegradationConfig from GPS_DEGRADATION_CONFIG + CLI args."""
    cfg = GpsDegradationConfig()
    cfg.scenario = "urban"
    cfg.uere_base_m = GPS_DEGRADATION_CONFIG["base_h_accuracy"]

    if args.tier2:
        # Admit satellites down to 2° — exercises more multipath-prone geometry.
        cfg.gps_elevation_mask_deg = GPS_DEGRADATION_CONFIG["tier2_elevation_mask_deg"]
        carb.log_info(f"Elevation mask lowered to {cfg.gps_elevation_mask_deg}° (tier2)")

    return cfg


def add_urban_canyon(stage, width_m: float = 30.0, height_m: float = 20.0, length_m: float = 200.0) -> None:
    """
    Add two parallel walls forming an urban canyon and apply PhysicsCollisionAPI
    so PhysX raycasts (satellite LOS checks) can hit them.

    Wall orientation: canyon runs along the X axis (East). Walls are offset ±Y.
    """
    carb.log_info(f"Adding urban canyon: {width_m}m wide × {height_m}m tall × {length_m}m long")

    for name, y_offset in [("WallNorth", width_m / 2.0), ("WallSouth", -width_m / 2.0)]:
        path = f"/World/{name}"
        prim = stage.DefinePrim(path, "Cube")

        xform = UsdGeom.XformCommonAPI(prim)
        xform.SetTranslate(Gf.Vec3d(0.0, y_offset, height_m / 2.0))
        xform.SetScale(Gf.Vec3f(length_m, 1.0, height_m))

        # Required for PhysX raycasting to detect this geometry.
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)

        carb.log_info(f"  {name}: Y={y_offset:.1f}m, CollisionAPI applied")


def enable_required_extensions() -> None:
    """Enable Pegasus and required OmniGraph extensions."""
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


def wait_for_stage(stage, timeout_s: float = 10.0) -> bool:
    """Pump Kit app loop until /World has non-physics content."""
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


def _in_jamming_zone(world_pos_xyz, zone_cfg: dict) -> bool:
    """Return True if world_pos_xyz is within the spherical jamming zone."""
    cx, cy, cz = zone_cfg["center"]
    radius = zone_cfg["radius"]
    dx = world_pos_xyz[0] - cx
    dy = world_pos_xyz[1] - cy
    dz = world_pos_xyz[2] - cz
    return (dx * dx + dy * dy + dz * dz) <= radius * radius


class PegasusAppWithDegradedGPS:
    """Pegasus app launcher with the full realistic GPS degradation pipeline."""

    def __init__(self, args):
        self.timeline = omni.timeline.get_timeline_interface()
        self.args = args
        self._physics_step_count: int = 0

        # Jamming zone configuration (None when tier3 not active)
        if args.tier3:
            GPS_DEGRADATION_CONFIG["jamming_enabled"] = True
            GPS_DEGRADATION_CONFIG["jamming_mode"] = args.tier3_mode
            carb.log_info(f"Tier 3 jamming ENABLED — mode: {args.tier3_mode}")
        self._jamming_cfg = (
            GPS_DEGRADATION_CONFIG["jamming_zone"]
            if GPS_DEGRADATION_CONFIG["jamming_enabled"] else None
        )

        # Pegasus world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.timeline.stop()

        # Load environment
        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("USD stage failed to load.")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        # Apply colliders to any imported scene geometry
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        add_dome_light(stage)

        # Urban canyon — walls get CollisionAPI applied inside add_urban_canyon().
        add_urban_canyon(
            stage,
            width_m=GPS_DEGRADATION_CONFIG["canyon_width_m"],
            height_m=GPS_DEGRADATION_CONFIG["canyon_height_m"],
            length_m=GPS_DEGRADATION_CONFIG["canyon_length_m"],
        )

        if SAVE_SCENE_TO:
            self._save_scene(SAVE_SCENE_TO)

        self._spawn_drone()

        # GPS degradation node — must be created after SimulationApp is live.
        rclpy.init()
        gps_cfg = build_gps_config(args)
        self.gps_node = GPSDegradationNode(vehicle_id=1, scenario="urban", cfg=gps_cfg)

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

        carb.log_info("GPS pipeline: Walker-delta(24:6:1) + PhysX raycasting active")

    def _spawn_drone(self) -> None:
        carb.log_info("Spawning PX4 multirotor...")

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

        carb.log_info("Drone spawned.")

    def _save_scene(self, save_path: str) -> None:
        import tempfile
        tmp_usd = os.path.join(tempfile.gettempdir(), "prepared_scene.usd")
        success, error = asyncio.get_event_loop().run_until_complete(
            omni.usd.get_context().export_as_stage_async(tmp_usd)
        )
        if success:
            os.makedirs(save_path, exist_ok=True)
            save_scene_as_contained_usd(tmp_usd, save_path)
            os.remove(tmp_usd)
        else:
            carb.log_error(f"Scene export failed: {error}")

    def _check_jamming(self) -> None:
        """Update jamming state based on drone world position."""
        if self._jamming_cfg is None:
            return
        pose = self.gps_node._last_pose
        if pose is None:
            return
        p = pose.pose.position
        in_zone = _in_jamming_zone((p.x, p.y, p.z), self._jamming_cfg)
        self.gps_node.set_jamming(in_zone)

    def run(self) -> None:
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()

        try:
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

                self._physics_step_count += 1
                sim_time_s = float(world.current_time)

                # Full GPS pipeline — raycasts valid here (physics thread, scene playing).
                self.gps_node.physics_step(sim_time_s)

                # Jamming zone override (reads last pose from subscriptions).
                self._check_jamming()

                # Drain incoming ROS 2 messages (subscriptions update caches).
                rclpy.spin_once(self.gps_node, timeout_sec=0.0)

        finally:
            carb.log_warn("Closing simulation.")
            self.gps_node.destroy_node()
            rclpy.shutdown()
            self.timeline.stop()
            simulation_app.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="AirStack GPS Degradation SITL launcher")
    parser.add_argument("--tier2", action="store_true",
                        help="Tier 2: lower elevation mask for more multipath-prone geometry")
    parser.add_argument("--tier3", action="store_true",
                        help="Tier 3: activate spatial jamming zone")
    parser.add_argument(
        "--tier3-mode",
        default="DENIAL",
        choices=["DENIAL", "NOISE", "SPOOFING", "MEACONING"],
        help="Jamming mode (SPOOFING/MEACONING currently treated as DENIAL)",
    )
    args = parser.parse_args()

    enable_required_extensions()

    carb.log_info("GPS Degradation SITL Launcher (Walker-delta constellation + PhysX raycasting)")
    carb.log_info(f"  Low elevation mask (--tier2):  {'ENABLED' if args.tier2 else 'disabled'}")
    carb.log_info(f"  Spatial jamming zone (--tier3): {'ENABLED' if args.tier3 else 'disabled'}")

    app = PegasusAppWithDegradedGPS(args)
    app.run()


if __name__ == "__main__":
    main()
