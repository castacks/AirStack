import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from airstack_msgs.msg import TrajectoryXYZVYaw

from .policy import HoldPositionPolicy, WaypointFollowPolicy, PlannerPolicy


class MocapPlannerNode(Node):

    def __init__(self) -> None:
        super().__init__('mocap_planner')

        self.declare_parameter('mode', 'hold')          # 'hold' | 'waypoints'
        self.declare_parameter('publish_rate', 10.0)    # Hz
        self.declare_parameter('velocity', 0.5)         # m/s for waypoint mode
        self.declare_parameter('acceptance_radius', 0.15)  # m
        # Flat list: x y z yaw  x y z yaw  ...
        self.declare_parameter('waypoints', [0.0, 0.0, 1.0, 0.0])

        mode = self.get_parameter('mode').value
        self._policy: PlannerPolicy = self._build_policy(mode)

        self._odom: Odometry | None = None
        self._odom_sub = self.create_subscription(
            Odometry, 'odometry', self._odom_cb, 10)
        self._traj_pub = self.create_publisher(
            TrajectoryXYZVYaw, 'trajectory_override', 1)

        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'mocap_planner started in {mode!r} mode')

    # ------------------------------------------------------------------
    # To plug in a real planner: replace _build_policy or set self._policy
    # after construction.  The rest of the node stays unchanged.
    # ------------------------------------------------------------------

    def _build_policy(self, mode: str) -> PlannerPolicy:
        if mode == 'hold':
            return HoldPositionPolicy()

        if mode == 'waypoints':
            flat = self.get_parameter('waypoints').value
            if len(flat) % 4 != 0:
                self.get_logger().error(
                    'waypoints param length must be a multiple of 4 (x y z yaw); '
                    'falling back to hold mode')
                return HoldPositionPolicy()
            wps = [(flat[i], flat[i+1], flat[i+2], flat[i+3])
                   for i in range(0, len(flat), 4)]
            return WaypointFollowPolicy(
                waypoints=wps,
                velocity=self.get_parameter('velocity').value,
                acceptance_radius=self.get_parameter('acceptance_radius').value,
            )

        self.get_logger().warn(f'Unknown mode {mode!r}; falling back to hold')
        return HoldPositionPolicy()

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _tick(self) -> None:
        if self._odom is None:
            return
        traj = self._policy.compute(self._odom)
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = self._odom.header.frame_id
        self._traj_pub.publish(traj)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        rclpy.spin(MocapPlannerNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
