"""GPS degradation ROS 2 node — wraps GpsDegradationModel for Isaac Sim integration.

Usage in Isaac Sim main loop:
    while simulation_app.is_running():
        world.step(render=False)
        gps_node.set_world_pos(x, y, z)
        gps_node.physics_step(world.current_time)
        rclpy.spin_once(gps_node, timeout_sec=0.0)
"""
from __future__ import annotations
import os
import sys

_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
_SIM_DIR    = os.path.normpath(os.path.join(_LAUNCH_DIR, ".."))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

import builtin_interfaces.msg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped

from gps_degradation_staging import GpsDegradationModel, GpsDegradationConfig, GpsState


class GPSDegradationNode(Node):
    def __init__(
        self,
        vehicle_id: int = 1,
        scenario: str = "urban",
        cfg: GpsDegradationConfig = None,
    ):
        super().__init__(f"gps_degradation_{vehicle_id}")
        self._vehicle_id = vehicle_id

        if cfg is None:
            cfg = GpsDegradationConfig(scenario=scenario)

        # Pipeline — passes self as ros2_node so state publisher auto-creates
        self._model = GpsDegradationModel(cfg=cfg, ros2_node=self, robot_id=vehicle_id)

        self._pub_gps = self.create_publisher(
            NavSatFix,
            f"/robot_{vehicle_id}/sensors/gps_degraded",
            10,
        )
        self._pub_twist = self.create_publisher(
            TwistStamped,
            f"/robot_{vehicle_id}/sensors/gps_twist_degraded",
            10,
        )

        self._latest_gps: NavSatFix | None = None
        self._latest_twist: TwistStamped | None = None
        self._world_pos = (0.0, 0.0, 0.0)
        self._jamming = False
        self._step_count = 0

        self.create_subscription(
            NavSatFix,
            f"/robot_{vehicle_id}/sensors/gps",
            self._gps_cb,
            10,
        )
        self.create_subscription(
            TwistStamped,
            f"/robot_{vehicle_id}/sensors/gps_twist",
            self._twist_cb,
            10,
        )

        self.get_logger().info(
            f"GPS degradation node: vehicle={vehicle_id}  scenario={scenario}  "
            f"constellation={'TLE' if self._model._constellation.using_tle else 'Walker-delta'}"
        )

    # ------------------------------------------------------------------
    # Callbacks — just cache latest messages
    # ------------------------------------------------------------------

    def _gps_cb(self, msg: NavSatFix):
        self._latest_gps = msg

    def _twist_cb(self, msg: TwistStamped):
        self._latest_twist = msg

    # ------------------------------------------------------------------
    # Called from Isaac Sim main loop
    # ------------------------------------------------------------------

    def set_world_pos(self, x: float, y: float, z: float):
        """Update drone world position (meters, Isaac Sim Z-up coords)."""
        self._world_pos = (x, y, z)

    def set_jamming(self, active: bool):
        self._jamming = active

    def physics_step(self, sim_time_s: float):
        """Drive pipeline. MUST be called from Isaac Sim loop after world.step()."""
        self._step_count += 1
        if self._latest_gps is None:
            return

        gps = self._latest_gps
        output = self._model.step(
            sim_time_s=sim_time_s,
            step_count=self._step_count,
            lat_deg=gps.latitude,
            lon_deg=gps.longitude,
            alt_m=gps.altitude,
            world_pos=self._world_pos,
            jamming=self._jamming,
        )

        self._publish_gps(output, sim_time_s)
        self._publish_twist(output, sim_time_s)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_gps(self, output, sim_time_s: float):
        msg = NavSatFix()
        msg.header.stamp = _ros_stamp(sim_time_s)
        msg.header.frame_id = "gps"
        msg.status.service = NavSatStatus.SERVICE_GPS

        if output.state == GpsState.DENIED:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = msg.altitude = 0.0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.latitude  = output.lat_deg
            msg.longitude = output.lon_deg
            raw_alt = self._latest_gps.altitude if self._latest_gps else 0.0
            msg.altitude = raw_alt + output.delta_alt
            eph2 = output.eph_m ** 2
            epv2 = output.epv_m ** 2
            msg.position_covariance = [eph2, 0.0, 0.0, 0.0, eph2, 0.0, 0.0, 0.0, epv2]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._pub_gps.publish(msg)

    def _publish_twist(self, output, sim_time_s: float):
        if self._latest_twist is None:
            return
        msg = TwistStamped()
        msg.header.stamp = _ros_stamp(sim_time_s)
        msg.header.frame_id = "gps"
        msg.twist = self._latest_twist.twist  # velocity passthrough
        self._pub_twist.publish(msg)


def _ros_stamp(sim_time_s: float) -> builtin_interfaces.msg.Time:
    t = builtin_interfaces.msg.Time()
    t.sec = int(sim_time_s)
    t.nanosec = int((sim_time_s - int(sim_time_s)) * 1e9)
    return t
