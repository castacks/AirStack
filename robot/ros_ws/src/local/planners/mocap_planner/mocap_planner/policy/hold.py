import math

from nav_msgs.msg import Odometry
from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from geometry_msgs.msg import Point

from .base import PlannerPolicy


def _yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class HoldPositionPolicy(PlannerPolicy):
    """Latches to the first received pose and holds there indefinitely."""

    def __init__(self) -> None:
        self._hold_x: float | None = None
        self._hold_y: float | None = None
        self._hold_z: float | None = None
        self._hold_yaw: float | None = None

    def compute(self, odom: Odometry) -> TrajectoryXYZVYaw:
        if self._hold_x is None:
            p = odom.pose.pose.position
            self._hold_x = p.x
            self._hold_y = p.y
            self._hold_z = p.z
            self._hold_yaw = _yaw_from_odom(odom)

        wp = WaypointXYZVYaw()
        wp.position = Point(x=self._hold_x, y=self._hold_y, z=self._hold_z)
        wp.velocity = 0.0
        wp.yaw = self._hold_yaw

        traj = TrajectoryXYZVYaw()
        traj.waypoints = [wp]
        return traj
