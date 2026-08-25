"""Publish the TRUTHFUL `world` -> `map` transform.

`robot.launch.xml` publishes this static transform as IDENTITY. That is only
correct when the drone spawns at the world origin, which is exactly what the
default `DRONE_CONFIGS` do — so the error is invisible in the common case and
grows with the spawn offset:

    house bench, DRONE_XY=0,-45                  ->  45 m
    suburb_wildfire, spawn (302.4, -144.3)       -> 340 m

`map` is anchored at the robot's TAKEOFF POINT: PX4's local origin is set from
the GPS home position, which `gps_utils.set_gps_origins` derives from the spawn
pose. So odometry, TF, `/global_plan` and any occupancy grid published in `map`
all sit in a frame offset from world by the spawn point — while the simulator's
overhead ground texture (`/gcs/sim_ground`) and the GCS's own robot markers are
in WORLD ENU. Under an identity transform those two families are drawn on top of
each other and disagree by the spawn offset, which reads as "the drone is
floating off the map".

This node measures the offset instead of assuming it is zero: world ENU from the
GPS fix, minus the same instant's odometry in `map`.

    map_origin_world = enu(fix) - odom_in_map

Z is NOT taken from GPS. The fix's altitude is referenced to the world origin's
MSL, which does not line up with the sim's ground plane — measured 53.8 m for a
drone 1 m off the floor. The true z offset is just the spawn height above
ground, so it is a parameter and defaults to 0.

OPT-IN. `robot.launch.xml` still publishes identity unless MAP_ANCHOR_ENU is
true, because making this transform truthful changes the frame every recorded
mission was replayed in, and that is not a change to make silently.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tf2_ros import StaticTransformBroadcaster

try:
    from coordination_bringup.frame_utils import gps_to_enu
except ImportError:      # same flat-earth constants the rest of the stack uses
    def gps_to_enu(lat, lon, alt, origin_lat=38.736832, origin_lon=-9.137977,
                   origin_alt=90.0):
        return ((lon - origin_lon) * 111320.0 * math.cos(math.radians(origin_lat)),
                (lat - origin_lat) * 111320.0,
                alt - origin_alt)


class MapAnchorNode(Node):

    def __init__(self):
        super().__init__('map_anchor')
        self._world_frame = self.declare_parameter('world_frame', 'world').value
        self._map_frame = self.declare_parameter('map_frame', 'map').value
        # Height of the takeoff point above the sim's ground plane. See the
        # module docstring for why this is not read from the GPS fix.
        self._z_offset = float(
            self.declare_parameter('spawn_height_m', 0.0).value)
        # A single fix is noisy; average a few before committing to a transform
        # that is published ONCE and never revised.
        self._n_samples = int(self.declare_parameter('samples', 10).value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._fix = None
        self._odom = None
        self._samples = []
        self._done = False
        self._tf = StaticTransformBroadcaster(self)

        self.create_subscription(
            NavSatFix,
            self.declare_parameter(
                'navsat_topic', 'interface/mavros/global_position/global').value,
            self._on_fix, qos)
        self.create_subscription(
            Odometry,
            self.declare_parameter(
                'odom_topic', 'odometry_conversion/odometry').value,
            self._on_odom, qos)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'map_anchor: measuring {self._map_frame} origin in '
            f'{self._world_frame} from GPS vs odometry')

    def _on_fix(self, msg):
        self._fix = msg

    def _on_odom(self, msg):
        self._odom = msg

    def _tick(self):
        if self._done or self._fix is None or self._odom is None:
            return
        # status < 0 is STATUS_NO_FIX; an unfixed receiver reports a position
        # that is not merely imprecise but meaningless.
        if self._fix.status.status < 0:
            self.get_logger().info('map_anchor: waiting for a GPS fix',
                                   throttle_duration_sec=10.0)
            return
        ex, ey, _ = gps_to_enu(self._fix.latitude, self._fix.longitude,
                               self._fix.altitude)
        p = self._odom.pose.pose.position
        self._samples.append((ex - p.x, ey - p.y))
        if len(self._samples) < self._n_samples:
            return

        dx = sum(s[0] for s in self._samples) / len(self._samples)
        dy = sum(s[1] for s in self._samples) / len(self._samples)
        spread = max(max(abs(s[0] - dx), abs(s[1] - dy)) for s in self._samples)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._world_frame
        t.child_frame_id = self._map_frame
        t.transform.translation.x = float(dx)
        t.transform.translation.y = float(dy)
        t.transform.translation.z = float(self._z_offset)
        t.transform.rotation.w = 1.0
        self._tf.sendTransform(t)
        self._done = True
        self.get_logger().info(
            f'map_anchor: {self._map_frame} origin is at {self._world_frame} '
            f'({dx:.2f}, {dy:.2f}, {self._z_offset:.2f}) '
            f'[{len(self._samples)} samples, spread {spread:.2f} m]')


def main(args=None):
    rclpy.init(args=args)
    node = MapAnchorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
