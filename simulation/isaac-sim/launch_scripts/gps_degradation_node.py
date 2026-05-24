#!/usr/bin/env python
"""
GPS Degradation Node — full realistic pipeline.

Uses GpsDegradationModel from gps_degradation_staging:
  Walker-delta(24:6:1) constellation → PhysX raycasting LOS/NLOS classification
  → elevation-dependent C/N0 + lognormal shadow fading → DOP (geometry matrix)
  → pseudorange WLS position error → hysteretic state machine
  → ramp-rate-limited output applied to HIL-level GPS fields.

IMPORTANT: this module must be imported and run inside the Isaac Sim Python
environment (ISAACSIM_PYTHON). physics_step() issues PhysX raycasts, which
require the physics scene to be active (timeline playing). Call physics_step()
from the Isaac Sim main loop — after world.step() — so raycasts execute in the
physics thread.

Inputs (ROS 2 subscriptions):
  /{ns}/sensors/gps         NavSatFix    — true (undegraded) Pegasus GPS
  /{ns}/sensors/gps_twist   TwistStamped — true GPS velocity
  /{ns}/state/pose          PoseStamped  — world-frame drone position
                                           AirStack convention: X=East, Y=North, Z=Up

Outputs (ROS 2 publications):
  /{ns}/sensors/gps_degraded        NavSatFix         — degraded GPS with position error + uncertainty
  /{ns}/sensors/gps_twist_degraded  TwistStamped      — degraded velocity (noise scales with state)
  /{ns}/gps/degradation_state       Float32MultiArray — pipeline internals (state, sats, HDOP, VDOP, eph, epv, deltas)
"""

import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TwistStamped
from geometry_msgs.msg import PoseStamped

import numpy as np

# ---------------------------------------------------------------------------
# Staging package path — one directory above launch_scripts/
# ---------------------------------------------------------------------------
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
_SIM_DIR = os.path.normpath(os.path.join(_LAUNCH_DIR, ".."))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

from gps_degradation_staging import (
    GpsDegradationModel,
    GpsDegradationConfig,
    GpsState,
)

# ---------------------------------------------------------------------------


class GPSDegradationNode(Node):
    """
    Full GPS degradation node backed by GpsDegradationModel.

    Must run inside ISAACSIM_PYTHON. The caller is responsible for:
      1. Creating SimulationApp before importing this module.
      2. Calling physics_step(sim_time_s) every Isaac Sim physics step
         (after world.step()) so PhysX raycasts execute in the physics thread.
      3. Calling rclpy.spin_once(node, timeout_sec=0.0) in the same loop
         to drain incoming ROS 2 messages.
    """

    def __init__(
        self,
        vehicle_id: int = 1,
        scenario: str = "urban",
        cfg: GpsDegradationConfig = None,
    ):
        super().__init__("gps_degradation_node")

        self._vehicle_id = vehicle_id
        self._step_count: int = 0
        self._jamming: bool = False

        if cfg is None:
            cfg = GpsDegradationConfig()
        cfg.scenario = scenario

        # Pass self as ros2_node so GpsDegradationPublisher (ros2_publisher.py)
        # auto-creates the /gps/degradation_state publisher.
        self._model = GpsDegradationModel(
            vehicle_id=vehicle_id,
            cfg=cfg,
            ros2_node=self,
        )
        self._cfg = cfg

        ns = f"/robot_{vehicle_id}"

        # --- Subscriptions (true Pegasus outputs) ---
        self.create_subscription(
            NavSatFix, f"{ns}/sensors/gps", self._gps_cb, 10
        )
        self.create_subscription(
            TwistStamped, f"{ns}/sensors/gps_twist", self._twist_cb, 10
        )
        self.create_subscription(
            PoseStamped, f"{ns}/state/pose", self._pose_cb, 10
        )

        # --- Degraded output publishers ---
        self._pub_gps = self.create_publisher(
            NavSatFix, f"{ns}/sensors/gps_degraded", 10
        )
        self._pub_twist = self.create_publisher(
            TwistStamped, f"{ns}/sensors/gps_twist_degraded", 10
        )

        # --- Incoming message cache ---
        self._last_gps: NavSatFix | None = None
        self._last_twist: TwistStamped | None = None
        self._last_pose: PoseStamped | None = None

        self.get_logger().info(
            f"GPSDegradationNode — vehicle_id={vehicle_id}, scenario={scenario}, "
            f"gps_rate={100 // cfg.gps_update_every_n_steps} Hz, "
            f"constellation=Walker-delta({cfg.gps_n_planes * cfg.gps_sats_per_plane}"
            f":{cfg.gps_n_planes}:1), "
            f"elevation_mask={cfg.gps_elevation_mask_deg}°"
        )

    # -----------------------------------------------------------------------
    # ROS 2 callbacks — store latest message, overwrite on each arrival
    # -----------------------------------------------------------------------

    def _gps_cb(self, msg: NavSatFix) -> None:
        self._last_gps = msg

    def _twist_cb(self, msg: TwistStamped) -> None:
        self._last_twist = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._last_pose = msg

    # -----------------------------------------------------------------------
    # Physics-thread entry point
    # -----------------------------------------------------------------------

    def physics_step(self, sim_time_s: float) -> None:
        """
        Call once per Isaac Sim physics step (nominally 100 Hz), AFTER world.step(),
        from the main simulation loop. PhysX raycasts are valid here.

        Silently skips until both a GPS message and a pose message have arrived.
        """
        self._step_count += 1

        if self._last_gps is None or self._last_pose is None:
            return

        lat_deg = self._last_gps.latitude
        lon_deg = self._last_gps.longitude
        alt_m = self._last_gps.altitude

        p = self._last_pose.pose.position
        world_pos = (p.x, p.y, p.z)

        out = self._model.step(
            sim_time_s=sim_time_s,
            step_count=self._step_count,
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            alt_m=alt_m,
            world_pos=world_pos,
            jamming=self._jamming,
        )

        self._publish_gps(out)
        self._publish_twist(out)

    # -----------------------------------------------------------------------
    # Jamming control (set from the launch script's main loop)
    # -----------------------------------------------------------------------

    def set_jamming(self, active: bool) -> None:
        if active != self._jamming:
            self._jamming = active
            self.get_logger().warn(
                f"GPS jamming {'ACTIVE' if active else 'cleared'}"
            )

    # -----------------------------------------------------------------------
    # Publishers
    # -----------------------------------------------------------------------

    def _publish_gps(self, out) -> None:
        if self._last_gps is None:
            return

        msg = NavSatFix()
        msg.header = self._last_gps.header

        # Apply position error deltas computed by WLS + state machine
        msg.latitude = self._last_gps.latitude + out.delta_lat_deg
        msg.longitude = self._last_gps.longitude + out.delta_lon_deg
        msg.altitude = self._last_gps.altitude + out.delta_alt_m

        # NavSatStatus
        if out.fix_type == 0:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = self._last_gps.status.service

        # Diagonal covariance from eph/epv (metres² on diagonal)
        eph2 = out.eph_m ** 2
        epv2 = out.epv_m ** 2
        msg.position_covariance = [
            eph2, 0.0, 0.0,
            0.0, eph2, 0.0,
            0.0, 0.0, epv2,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._pub_gps.publish(msg)

    def _publish_twist(self, out) -> None:
        if self._last_twist is None:
            return

        # Velocity noise scales with degradation relative to open-sky eph baseline.
        # Capped at 5× to prevent extreme values in DENIED state.
        noise_scale = min(out.eph_m / max(self._cfg.uere_base_m, 1e-6), 5.0)
        rng = self._model._rng

        msg = TwistStamped()
        msg.header = self._last_twist.header
        lx = self._last_twist.twist.linear
        msg.twist.linear.x = lx.x + float(rng.normal(0.0, 0.05 * noise_scale))
        msg.twist.linear.y = lx.y + float(rng.normal(0.0, 0.05 * noise_scale))
        msg.twist.linear.z = lx.z + float(rng.normal(0.0, 0.02 * noise_scale))
        msg.twist.angular = self._last_twist.twist.angular

        self._pub_twist.publish(msg)


def main(args=None):
    """
    Standalone entry point — only valid inside ISAACSIM_PYTHON.
    Prefer launching via outdoor_urban_canyon_launch.py which also drives
    physics_step() from the simulation loop.
    """
    rclpy.init(args=args)
    node = GPSDegradationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
