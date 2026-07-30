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
from scene_prep import scale_stage_prim, add_dome_light

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")))
from gps_degradation_node import GPSDegradationNode
from gps_degradation_staging import GpsDegradationConfig

# ----------------------------- CONFIGURATION --------------------------------

ENV_URL = "file:///isaac-sim/CCity_Building_Set_1/scene.usdc"
STAGE_SCALE = 0.01   # City USD is in cm (1 unit=1cm). scale=0.01 converts to metres.
                     # 2000-unit building → 20m world. Drone wingspan 0.55m → ratio ~36:1 (correct).
STAGE_ROTATE_X_DEG = 90.0
DRONE_USD  = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"
ADD_SYNTHETIC_URBAN_CANYON = False

GPS_DEGRADATION_CONFIG = GpsDegradationConfig(
    scenario="urban",
    gps_update_every_n_steps=20,    # 5 Hz GPS from 100 Hz physics
    # ou_* and uere_* intentionally use config.py defaults — see comments there.
    # tau=60s matches urban multipath correlation (RTCA DO-229: 25s; upper urban: 60s).
    # ou_base_xy=0.27 gives 1.48m SS-RMS at HDOP=1; 7.4m at HDOP=5 (correlated component).
    # uere=2.5m gives EPH 2.5m (open sky) → 12.5m (DENIED), matching literature.
    cn0_zenith_dbhz=45.0,
    cn0_floor_dbhz=30.0,
)

# Jamming zone: sphere in world coordinates. Set radius=0 to disable.
JAMMING_CENTER_M = (0.0, 0.0, 8.0)    # centre of canyon, mid-altitude
JAMMING_RADIUS_M = 0.0                 # keep 0 unless forcing DENIED

# ----------------------------------------------------------------------------


def _apply_collision(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid() and not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)


def _add_building_box_colliders(stage, stage_prim) -> int:
    """Create world-space box colliders from building bounding boxes.

    WHY NOT add_colliders (triangle mesh):
      PhysX silently drops triangle-mesh collision shapes when the parent prim
      carries a non-unit scale transform (our /World/stage has scale=0.01).
      The shapes are never cooked, so every satellite raycast misses.

    FIX — box colliders:
      A Cube prim with UsdPhysics.CollisionAPI supports arbitrary parent scales
      and cooks instantly. We compute each building mesh's WORLD-SPACE bounding
      box (which already includes the 0.01 scale + 90° rotation), then place a
      matching box at that location. Boxes are accurate enough for satellite
      line-of-sight raycasting.
    """
    from pxr import Usd

    # BBoxCache evaluates geometry in world space — includes the 0.01 scale.
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False
    )

    box_root_path = "/World/_GPSBoxColliders"
    if not stage.GetPrimAtPath(box_root_path).IsValid():
        stage.DefinePrim(box_root_path, "Xform")

    MIN_HEIGHT_M = 2.0   # skip ground / road meshes; only buildings > 2 m
    MAX_BOXES    = 250   # 250 boxes cook in <1 s; enough to cover a dense block
    count        = 0

    stage_path_str = str(stage_prim.GetPath())

    for prim in stage.Traverse():
        if count >= MAX_BOXES:
            break
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not str(prim.GetPath()).startswith(stage_path_str):
            continue

        try:
            bb  = cache.ComputeWorldBound(prim)
            rng = bb.GetRange()
            if rng.IsEmpty():
                continue

            mn, mx = rng.GetMin(), rng.GetMax()
            height = float(mx[2] - mn[2])
            if height < MIN_HEIGHT_M:
                continue

            cx = float((mn[0] + mx[0]) / 2)
            cy = float((mn[1] + mx[1]) / 2)
            cz = float((mn[2] + mx[2]) / 2)
            hx = float((mx[0] - mn[0]) / 2)  # half-extents
            hy = float((mx[1] - mn[1]) / 2)
            hz = float((mx[2] - mn[2]) / 2)

            box_path = f"{box_root_path}/b{count}"
            box_prim = stage.DefinePrim(box_path, "Cube")
            UsdGeom.XformCommonAPI(box_prim).SetTranslate(Gf.Vec3d(cx, cy, cz))
            UsdGeom.XformCommonAPI(box_prim).SetScale(Gf.Vec3f(hx, hy, hz))
            UsdPhysics.CollisionAPI.Apply(box_prim)
            # Make boxes invisible — collision only, no rendering
            UsdGeom.Imageable(box_prim).MakeInvisible()
            count += 1

        except Exception:
            continue

    return count


def _add_urban_canyon(stage):
    """Two tall walls flanking the take-off pad to create a GPS canyon.

    Walls at x=±8m create a 16m-wide canyon that blocks ~79% of the eastern
    and western sky hemisphere (arctan((50-8)/8) ≈ 79° elevation cutoff).
    With 4 GNSS constellations (~50 visible sats), this reduces n_los enough
    to reliably trigger DEGRADED and MARGINAL states during traversal.
    """
    walls = [
        ("/World/CanyonWall_L", ( 8.0,  0.0, 25.0),  (1.5, 40.0, 25.0)),
        ("/World/CanyonWall_R", (-8.0,  0.0, 25.0),  (1.5, 40.0, 25.0)),
    ]
    for path, pos, half_scale in walls:
        prim = stage.DefinePrim(path, "Cube")
        UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*pos))
        UsdGeom.XformCommonAPI(prim).SetScale(Gf.Vec3f(*half_scale))
        _apply_collision(stage, path)


def _wait_for_stage(stage, timeout_s: float = 30.0) -> bool:
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid() and stage_prim.IsLoaded():
            return True
        time.sleep(0.1)
    return False


def _count_meshes(prim) -> int:
    count = 1 if prim.IsA(UsdGeom.Mesh) else 0
    for child in prim.GetChildren():
        count += _count_meshes(child)
    return count


def _find_city_floor_z(stage) -> float:
    """Return the minimum world-space Z of all box colliders under /World/_GPSBoxColliders.

    These boxes were placed using world-space bounding boxes AFTER the 0.01 scale
    and 90-degree X rotation, so their minimum Z is the city's actual floor height
    in the simulation coordinate system.  Falls back to 0.0 if no boxes exist yet.
    """
    from pxr import Usd
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False
    )
    root = stage.GetPrimAtPath("/World/_GPSBoxColliders")
    if not root.IsValid():
        return 0.0

    min_z = float("inf")
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith("/World/_GPSBoxColliders"):
            continue
        try:
            rng = cache.ComputeWorldBound(prim).GetRange()
            if not rng.IsEmpty():
                min_z = min(min_z, float(rng.GetMin()[2]))
        except Exception:
            continue

    return max(min_z, 0.0) if min_z != float("inf") else 0.0


class PegasusAppDegradedGPS:
    def __init__(self):
        # Kill any leftover PX4 SITL process before starting. If the Python
        # script is restarted without killing the container, PX4 keeps running
        # with its old sim_time. The Pegasus frontend resets sim_time to 0,
        # causing a backward time-jump that permanently stalls PX4's EKF in
        # MAV_STATE_UNINIT — arming is then impossible until PX4 is restarted.
        import subprocess
        kill_result = subprocess.run(["pkill", "-9", "-f", "px4_sitl_default"],
                                     capture_output=True)
        if kill_result.returncode == 0:
            carb.log_warn("[gps_scene] Killed stale PX4 process — fresh start")
            time.sleep(2.0)   # let PX4 fully exit before Pegasus re-spawns it

        carb.log_warn(f"[gps_scene] Starting city GPS launch with environment: {ENV_URL}")
        if ENV_URL.startswith("file://"):
            env_path = ENV_URL.removeprefix("file://")
            if not os.path.isfile(env_path):
                raise FileNotFoundError(
                    f"[gps_scene] City USD is missing inside the container: {env_path}"
                )
            carb.log_warn(
                f"[gps_scene] Found city USD inside container: {env_path} "
                f"({os.path.getsize(env_path)} bytes)"
            )

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
            world = stage.GetPrimAtPath("/World")
            children = [str(child.GetPath()) for child in world.GetChildren()] if world.IsValid() else []
            raise RuntimeError(
                f"[gps_scene] Timed out waiting for /World/stage after loading {ENV_URL}. "
                f"/World children: {children}"
            )

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            mesh_count = _count_meshes(stage_prim)
            carb.log_warn(f"[gps_scene] Loaded /World/stage with {mesh_count} mesh prims")
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            UsdGeom.Xformable(stage_prim).AddRotateXOp().Set(STAGE_ROTATE_X_DEG)

            # Pump the app so the scale/rotation transform is committed before
            # we compute world-space bounding boxes for the box colliders.
            for _ in range(20):
                omni.kit.app.get_app().update()

            box_count = _add_building_box_colliders(stage, stage_prim)

            # Let PhysX cook the new box shapes before simulation starts.
            for _ in range(30):
                omni.kit.app.get_app().update()

            carb.log_warn(
                f"[gps_scene] Prepared {box_count} world-space box colliders "
                f"for PhysX raycasting (from {mesh_count} meshes, "
                f"scale={STAGE_SCALE}, rotate_x_deg={STAGE_ROTATE_X_DEG})"
            )
        else:
            raise RuntimeError("[gps_scene] /World/stage is missing after environment load")

        add_dome_light(stage)

        # Log city floor for reference only — do not use it to offset spawn Z.
        # The default ground plane is at Z=0; the drone spawns 0.5 m above it.
        city_floor_z = _find_city_floor_z(stage)
        carb.log_warn(f"[gps_scene] Detected city floor Z = {city_floor_z:.3f} m "
                      f"(for reference — drone always spawns at Z=0.5 m above ground plane)")

        self.world.scene.add_default_ground_plane()

        if ADD_SYNTHETIC_URBAN_CANYON:
            carb.log_warn("[gps_scene] Adding synthetic urban canyon walls")
            _add_urban_canyon(stage)
        else:
            carb.log_warn("[gps_scene] Synthetic urban canyon walls disabled")

        # Spawn drone above the city floor, level (identity quaternion = Z-up upright)
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/base_link",
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=1,
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.5],
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
        lidar_enabled = os.environ.get("ENABLE_LIDAR", "false").lower() == "true"
        if lidar_enabled:
            carb.log_warn("[gps_scene] ENABLE_LIDAR=true: adding RTX lidar graph")
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
        else:
            carb.log_warn("[gps_scene] ENABLE_LIDAR=false: skipping RTX lidar graph")

        # GPS degradation node
        if not rclpy.ok():
            rclpy.init()
        self._gps_node = GPSDegradationNode(
            vehicle_id=1,
            cfg=GPS_DEGRADATION_CONFIG,
            fallback_origin_lat=self.pg.latitude,
            fallback_origin_lon=self.pg.longitude,
            fallback_origin_alt=self.pg.altitude,
        )
        carb.log_warn(
            f"[gps_scene] GPS degradation ready: jamming_radius_m={JAMMING_RADIUS_M}, "
            "quality_topic=/robot_1/gps/degradation_state, "
            f"origin=({self.pg.latitude}, {self.pg.longitude}, {self.pg.altitude})"
        )

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"
        carb.log_warn(f"[gps_scene] PLAY_SIM_ON_START={self.play_on_start}")

    def _check_jamming(self, world_pos) -> bool:
        if JAMMING_RADIUS_M <= 0.0:
            return False
        dx = world_pos[0] - JAMMING_CENTER_M[0]
        dy = world_pos[1] - JAMMING_CENTER_M[1]
        dz = world_pos[2] - JAMMING_CENTER_M[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) < JAMMING_RADIUS_M

    def _get_drone_world_pos(self):
        """Read the live vehicle rigid-body transform from the USD stage."""
        try:
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath("/World/base_link/body")
            if not prim.IsValid():
                return (0.0, 0.0, 0.0)
            xform = UsdGeom.Xformable(prim)
            mat = xform.ComputeLocalToWorldTransform(0)
            t = mat.ExtractTranslation()
            return (float(t[0]), float(t[1]), float(t[2]))
        except Exception:
            return (0.0, 0.0, 0.0)

    def _log_scene_geometry_debug(self, stage):
        """One-shot: print building bounding boxes and drone position to verify scale."""
        carb.log_warn("[geo_debug] ── Scene geometry diagnostic ──────────────────")
        carb.log_warn(f"[geo_debug] STAGE_SCALE={STAGE_SCALE}  ROTATE_X={STAGE_ROTATE_X_DEG}°")

        # Drone world position
        drone_pos = self._get_drone_world_pos()
        carb.log_warn(f"[geo_debug] Drone world pos (stage units): {drone_pos}")

        # Sample up to 5 mesh prims and log their world bbox Z range
        count = 0
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            try:
                bbox = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(prim)
                rng = bbox.GetRange()
                carb.log_warn(
                    f"[geo_debug] Mesh {str(prim.GetPath())[:60]}  "
                    f"Z=[{rng.GetMin()[2]:.2f}, {rng.GetMax()[2]:.2f}]  "
                    f"XY=[{rng.GetMin()[0]:.1f}..{rng.GetMax()[0]:.1f}, "
                    f"{rng.GetMin()[1]:.1f}..{rng.GetMax()[1]:.1f}]"
                )
                count += 1
            except Exception as e:
                carb.log_warn(f"[geo_debug] bbox error for {prim.GetPath()}: {e}")
            if count >= 5:
                break
        carb.log_warn(f"[geo_debug] ── end (sampled {count} meshes) ─────────────")

    def run(self):
        if self.play_on_start:
            self.timeline.play()

        app = omni.kit.app.get_app()
        _geo_debug_logged = False

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

            # Log geometry debug once, 2s into sim, so physics has settled
            if not _geo_debug_logged and sim_time_s > 2.0:
                stage = omni.usd.get_context().get_stage()
                self._log_scene_geometry_debug(stage)
                _geo_debug_logged = True

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
