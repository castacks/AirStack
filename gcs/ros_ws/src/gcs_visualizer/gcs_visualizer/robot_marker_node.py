import math
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# Lisbon — matches Pegasus configs.yaml and gps_utils.py DEFAULT_WORLD_ORIGIN
ORIGIN_LAT = 38.736832
ORIGIN_LON = -9.137977
ORIGIN_ALT = 90.0

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

GPS_SUFFIX = '/interface/mavros/global_position/global'
ODOM_SUFFIX = '/odometry_conversion/odometry'

ROBOT_COLORS = [
    (1.0, 0.2, 0.2),  # red
    (0.2, 1.0, 0.2),  # green
    (0.2, 0.2, 1.0),  # blue
]

# OBJ -> ROS axis correction quaternion (belly -Z, nose +X)
AXIS_CORRECTION = (-0.5, -0.5, 0.5, 0.5)  # x, y, z, w


def gps_to_enu(lat, lon, alt, alt_ground):
    """Convert GPS lat/lon/alt to ENU metres relative to Lisbon origin. z is relative to first fix."""
    x = (lon - ORIGIN_LON) * 111320.0 * math.cos(math.radians(ORIGIN_LAT))
    y = (lat - ORIGIN_LAT) * 111320.0
    z = alt - alt_ground
    return x, y, z


def multiply_quaternions(q1, q2):
    """Hamilton product q1 * q2 (apply q2 first, then q1). Both are (x,y,z,w) tuples."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


class RobotMarkerNode(Node):
    def __init__(self):
        super().__init__('robot_marker_node')

        self.declare_parameter('robot_name_prefix', 'robot')
        self._prefix = self.get_parameter('robot_name_prefix').value
        self._gps_pattern = re.compile(
            rf'^/({re.escape(self._prefix)}_\w+){re.escape(GPS_SUFFIX)}$'
        )
        self._odom_pattern = re.compile(
            rf'^/({re.escape(self._prefix)}_\w+){re.escape(ODOM_SUFFIX)}$'
        )

        self._gps_positions = {}    # robot_name -> (x, y, z) ENU metres
        self._orientations = {}     # robot_name -> (x, y, z, w) from odometry
        self._subscribed_gps = set()
        self._subscribed_odom = set()
        self._alt_ground = None     # altitude of first fix, used as z=0 reference

        self._pub = self.create_publisher(MarkerArray, '/gcs/robot_markers', 10)

        # Broadcast map as root TF frame so Foxglove 3D panel has a display frame
        self._static_tf = StaticTransformBroadcaster(self)
        self._dynamic_tf = TransformBroadcaster(self)
        map_tf = TransformStamped()
        map_tf.header.stamp = self.get_clock().now().to_msg()
        map_tf.header.frame_id = 'map'
        map_tf.child_frame_id = 'enu_origin'
        map_tf.transform.rotation.w = 1.0
        self._static_tf.sendTransform(map_tf)

        self.create_timer(5.0, self._discover_robots)
        self.create_timer(0.1, self._publish_markers)
        self._discover_robots()

    def _discover_robots(self):
        for topic, type_list in self.get_topic_names_and_types():
            if topic not in self._subscribed_gps:
                m = self._gps_pattern.match(topic)
                if m and 'sensor_msgs/msg/NavSatFix' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        NavSatFix, topic,
                        lambda msg, n=name: self._gps_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_gps.add(topic)
                    self.get_logger().info(f'Subscribed to GPS: {topic}')

            if topic not in self._subscribed_odom:
                m = self._odom_pattern.match(topic)
                if m and 'nav_msgs/msg/Odometry' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        Odometry, topic,
                        lambda msg, n=name: self._odom_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_odom.add(topic)
                    self.get_logger().info(f'Subscribed to odom: {topic}')

    def _gps_callback(self, msg: NavSatFix, robot_name: str):
        if msg.status.status < 0:
            return
        if self._alt_ground is None:
            self._alt_ground = msg.altitude
        self._gps_positions[robot_name] = gps_to_enu(msg.latitude, msg.longitude, msg.altitude, self._alt_ground)

    def _odom_callback(self, msg: Odometry, robot_name: str):
        o = msg.pose.pose.orientation
        self._orientations[robot_name] = (o.x, o.y, o.z, o.w)

    def _publish_markers(self):
        if not self._gps_positions:
            return

        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        for i, robot_name in enumerate(sorted(self._gps_positions.keys())):
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            x, y, z = self._gps_positions[robot_name]
            orientation = self._orientations.get(robot_name)  # (x,y,z,w) or None

            # --- Mesh marker ---
            mesh = Marker()
            mesh.header.frame_id = 'map'
            mesh.header.stamp = now
            mesh.ns = 'robot_meshes'
            mesh.id = i * 3
            mesh.type = Marker.MESH_RESOURCE
            mesh.action = Marker.ADD
            mesh.mesh_resource = 'package://robot_descriptions/iris/meshes/base_link_body_body.obj'
            mesh.pose.position.x = x
            mesh.pose.position.y = y
            mesh.pose.position.z = z
            if orientation:
                qx, qy, qz, qw = multiply_quaternions(orientation, AXIS_CORRECTION)
            else:
                qx, qy, qz, qw = AXIS_CORRECTION
            mesh.pose.orientation.x = qx
            mesh.pose.orientation.y = qy
            mesh.pose.orientation.z = qz
            mesh.pose.orientation.w = qw
            mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
            mesh.color.r = color[0]
            mesh.color.g = color[1]
            mesh.color.b = color[2]
            mesh.color.a = 0.9
            mesh.lifetime = lifetime
            array.markers.append(mesh)

            # --- Label marker ---
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'robot_labels'
            label.id = i * 3 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 1.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = robot_name
            label.lifetime = lifetime
            array.markers.append(label)

            # --- TF: map -> robot_N/robot_pose ---
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = 'map'
            tf.child_frame_id = f'{robot_name}/robot_pose'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            if orientation:
                tf.transform.rotation.x = orientation[0]
                tf.transform.rotation.y = orientation[1]
                tf.transform.rotation.z = orientation[2]
                tf.transform.rotation.w = orientation[3]
            else:
                tf.transform.rotation.w = 1.0
            self._dynamic_tf.sendTransform(tf)

        self._pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
