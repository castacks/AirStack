import copy
import math
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster

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

GPS_SUFFIX  = '/interface/mavros/global_position/global'
ODOM_SUFFIX = '/odometry_conversion/odometry'
TRAJ_SUFFIX = '/trajectory_controller/trajectory_vis'
PLAN_SUFFIX = '/global_plan'
VDB_SUFFIX  = '/vdb_mapping/vdb_map_visualization'

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


def rotate_vector(v, q):
    """Rotate vector v=(vx,vy,vz) by quaternion q=(x,y,z,w). Returns (x,y,z)."""
    vx, vy, vz = v
    qx, qy, qz, qw = q
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    return (
        vx + 2.0 * (qw * cx + qy * cz - qz * cy),
        vy + 2.0 * (qw * cy + qz * cx - qx * cz),
        vz + 2.0 * (qw * cz + qx * cy - qy * cx),
    )


class FoxgloveVisualizerNode(Node):
    def __init__(self):
        super().__init__('foxglove_visualizer_node')

        self.declare_parameter('robot_name_prefix', 'robot')
        self._prefix = self.get_parameter('robot_name_prefix').value
        self._gps_pattern  = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(GPS_SUFFIX)}$')
        self._odom_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(ODOM_SUFFIX)}$')
        self._traj_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(TRAJ_SUFFIX)}$')
        self._plan_pattern = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(PLAN_SUFFIX)}$')
        self._vdb_pattern  = re.compile(rf'^/({re.escape(self._prefix)}_\w+){re.escape(VDB_SUFFIX)}$')

        self._gps_positions    = {}   # robot_name -> (x, y, z) ENU metres current position
        self._gps_boot         = {}   # robot_name -> (x, y, z) ENU metres at first fix (odom origin)
        self._orientations     = {}   # robot_name -> (x, y, z, w) from odometry
        self._trajectories     = {}   # robot_name -> latest MarkerArray
        self._global_plans     = {}   # robot_name -> latest nav_msgs/Path
        self._vdb_markers      = {}   # robot_name -> latest VDB Marker
        self._subscribed_gps   = set()
        self._subscribed_odom  = set()
        self._subscribed_traj  = set()
        self._subscribed_plan  = set()
        self._subscribed_vdb   = set()
        self._alt_ground       = None

        self._pub = self.create_publisher(MarkerArray, '/gcs/robot_markers', 10)

        self._static_tf = StaticTransformBroadcaster(self)
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

            if topic not in self._subscribed_traj:
                m = self._traj_pattern.match(topic)
                if m and 'visualization_msgs/msg/MarkerArray' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        MarkerArray, topic,
                        lambda msg, n=name: self._traj_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_traj.add(topic)
                    self.get_logger().info(f'Subscribed to trajectory_vis: {topic}')

            if topic not in self._subscribed_plan:
                m = self._plan_pattern.match(topic)
                if m and 'nav_msgs/msg/Path' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        Path, topic,
                        lambda msg, n=name: self._plan_callback(msg, n),
                        10,
                    )
                    self._subscribed_plan.add(topic)
                    self.get_logger().info(f'Subscribed to global_plan: {topic}')

            if topic not in self._subscribed_vdb:
                m = self._vdb_pattern.match(topic)
                if m and 'visualization_msgs/msg/Marker' in type_list:
                    name = m.group(1)
                    self.create_subscription(
                        Marker, topic,
                        lambda msg, n=name: self._vdb_callback(msg, n),
                        SENSOR_QOS,
                    )
                    self._subscribed_vdb.add(topic)
                    self.get_logger().info(f'Subscribed to vdb_map_visualization: {topic}')

    def _gps_callback(self, msg: NavSatFix, robot_name: str):
        if msg.status.status < 0:
            return
        if self._alt_ground is None:
            self._alt_ground = msg.altitude
        pos = gps_to_enu(msg.latitude, msg.longitude, msg.altitude, self._alt_ground)
        self._gps_positions[robot_name] = pos
        if robot_name not in self._gps_boot:
            self._gps_boot[robot_name] = pos

    def _odom_callback(self, msg: Odometry, robot_name: str):
        o = msg.pose.pose.orientation
        self._orientations[robot_name] = (o.x, o.y, o.z, o.w)

    def _traj_callback(self, msg: MarkerArray, robot_name: str):
        self._trajectories[robot_name] = msg

    def _plan_callback(self, msg: Path, robot_name: str):
        self._global_plans[robot_name] = msg

    def _vdb_callback(self, msg: Marker, robot_name: str):
        self._vdb_markers[robot_name] = msg

    def _publish_markers(self):
        if not self._gps_positions:
            return

        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        for i, robot_name in enumerate(sorted(self._gps_positions.keys())):
            x, y, z = self._gps_positions[robot_name]
            orientation = self._orientations.get(robot_name)  # (x,y,z,w) or None

            # --- Mesh marker ---
            mesh = Marker()
            mesh.header.frame_id = 'map'
            mesh.header.stamp = now
            mesh.ns = 'robot_meshes'
            mesh.id = i * 10
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
            mesh.color.r = 0.6
            mesh.color.g = 0.6
            mesh.color.b = 0.6
            mesh.color.a = 1.0
            mesh.lifetime = lifetime
            array.markers.append(mesh)

            # --- Label marker ---
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'robot_labels'
            label.id = i * 10 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 1.0
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = robot_name
            label.lifetime = lifetime
            array.markers.append(label)

            # --- Axes markers (X=red, Y=green, Z=blue) ---
            axes = [
                ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                ((0.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
            ]
            q = orientation if orientation else (0.0, 0.0, 0.0, 1.0)
            axis_len = 0.6
            for j, (unit_vec, axis_color) in enumerate(axes):
                tip = rotate_vector(unit_vec, q)
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = now
                arrow.ns = f'{robot_name}_axes'
                arrow.id = i * 10 + 2 + j
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                start = Point(x=x, y=y, z=z)
                end = Point(x=x + tip[0] * axis_len,
                            y=y + tip[1] * axis_len,
                            z=z + tip[2] * axis_len)
                arrow.points = [start, end]
                arrow.scale.x = 0.04
                arrow.scale.y = 0.08
                arrow.scale.z = 0.0
                arrow.color.r = axis_color[0]
                arrow.color.g = axis_color[1]
                arrow.color.b = axis_color[2]
                arrow.color.a = 1.0
                arrow.lifetime = lifetime
                array.markers.append(arrow)

            # --- Trajectory markers (offset by boot GPS position = odom origin, skip if not ready) ---
            traj = self._trajectories.get(robot_name)
            boot = self._gps_boot.get(robot_name)
            if traj is not None and boot is not None:
                bx, by, bz = boot
                for k, orig in enumerate(traj.markers):
                    m = copy.deepcopy(orig)
                    m.header.frame_id = 'map'
                    m.header.stamp = now
                    m.ns = f'{robot_name}_traj'
                    m.id = i * 10000 + k
                    m.lifetime = lifetime
                    for pt in m.points:
                        pt.x += bx
                        pt.y += by
                        pt.z += bz
                    if m.color.a > 0:
                        m.color.r = 0.8
                        m.color.g = 0.5
                        m.color.b = 0.0
                    array.markers.append(m)

            # --- Global plan (offset from odom origin to ENU, same as trajectory) ---
            plan = self._global_plans.get(robot_name)
            if plan is not None and boot is not None:
                bx, by, bz = boot
                color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
                line = Marker()
                line.header.frame_id = 'map'
                line.header.stamp = now
                line.ns = f'{robot_name}_global_plan'
                line.id = i * 10000 + 9999
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.1
                line.color.r = color[0]
                line.color.g = color[1]
                line.color.b = color[2]
                line.color.a = 0.8
                line.lifetime = Duration(sec=2, nanosec=0)
                for pose_stamped in plan.poses:
                    p = pose_stamped.pose.position
                    line.points.append(Point(x=p.x + bx, y=p.y + by, z=p.z + bz))
                array.markers.append(line)

            # --- VDB map (offset from odom origin to ENU) ---
            vdb = self._vdb_markers.get(robot_name)
            if vdb is not None and boot is not None:
                bx, by, bz = boot
                m = copy.deepcopy(vdb)
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = f'{robot_name}_vdb'
                m.id = i * 10000 + 9998
                m.lifetime = Duration(sec=2, nanosec=0)
                for pt in m.points:
                    pt.x += bx
                    pt.y += by
                    pt.z += bz
                array.markers.append(m)

        self._pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
