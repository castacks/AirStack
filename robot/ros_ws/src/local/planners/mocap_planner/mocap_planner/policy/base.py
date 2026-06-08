from abc import ABC, abstractmethod

from nav_msgs.msg import Odometry
from airstack_msgs.msg import TrajectoryXYZVYaw


class PlannerPolicy(ABC):
    """
    Swap this out to plug in a real planner.
    Receives the latest odometry and returns the trajectory to execute.
    """

    @abstractmethod
    def compute(self, odom: Odometry) -> TrajectoryXYZVYaw:
        ...
