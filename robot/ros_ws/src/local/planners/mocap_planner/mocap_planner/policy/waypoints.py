import math
from typing import List, Tuple

from nav_msgs.msg import Odometry
from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
from geometry_msgs.msg import Point

from .base import PlannerPolicy

Waypoint = Tuple[float, float, float, float]  # x, y, z, yaw


class WaypointFollowPolicy(PlannerPolicy):
    """
    Visits a fixed list of (x, y, z, yaw) waypoints in order.
    Advances to the next waypoint once within acceptance_radius.
    Holds at the final waypoint when the list is exhausted.
    """

    def __init__(
        self,
        waypoints: List[Waypoint],
        velocity: float,
        acceptance_radius: float,
    ) -> None:
        self._waypoints = waypoints
        self._velocity = velocity
        self._acceptance_radius = acceptance_radius
        self._idx = 0

    def compute(self, odom: Odometry) -> TrajectoryXYZVYaw:
        p = odom.pose.pose.position
        tx, ty, tz, tyaw = self._waypoints[self._idx]

        dist = math.sqrt((p.x - tx) ** 2 + (p.y - ty) ** 2 + (p.z - tz) ** 2)
        if dist < self._acceptance_radius and self._idx < len(self._waypoints) - 1:
            self._idx += 1
            tx, ty, tz, tyaw = self._waypoints[self._idx]

        at_final = self._idx == len(self._waypoints) - 1 and dist < self._acceptance_radius

        wp = WaypointXYZVYaw()
        wp.position = Point(x=tx, y=ty, z=tz)
        wp.velocity = 0.0 if at_final else self._velocity
        wp.yaw = tyaw

        traj = TrajectoryXYZVYaw()
        traj.waypoints = [wp]
        return traj
