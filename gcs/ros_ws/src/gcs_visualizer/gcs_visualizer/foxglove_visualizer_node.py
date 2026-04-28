"""
foxglove_visualizer_node.py — Standard GCS visualization node.

Handles data common to every project: robot mesh markers, name labels,
body-frame axes, local trajectory, global plan, and VDB occupancy map.
All markers are published to /gcs/robot_markers in the global ENU 'map' frame.
"""

import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster

from gcs_visualizer.gcs_utils import (
    gps_to_enu, multiply_quaternions, rotate_vector, transform_marker_array,
    ORIGIN_LAT, ORIGIN_LON,
)

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

# OBJ mesh axis correction quaternion (belly -Z, nose +X)
AXIS_CORRECTION = (-0.5, -0.5, 0.5, 0.5)


def _translate_marker(msg: Marker, bx: float, by: float, bz: float) -> Marker:
    """Return a new Marker with all points translated by (bx, by, bz) via numpy."""
    m = Marker()
    m.type = msg.type
    m.action = msg.action
    m.pose = msg.pose
    m.scale = msg.scale
    m.color = msg.color
    m.colors = list(msg.colors)
    m.header.frame_id = 'map'
    n = len(msg.points)
    if n > 0:
        xyz = np.array([(pt.x, pt.y, pt.z) for pt in msg.points], dtype=np.float64)
        xyz[:, 0] += bx
        xyz[:, 1] += by
        xyz[:, 2] += bz
        m.points = [Point(x=float(r[0]), y=float(r[1]), z=float(r[2])) for r in xyz]
    return m


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

        self._gps_positions  = {}
        self._gps_boot       = {}
        self._orientations   = {}
        self._trajectories   = {}
        self._global_plans   = {}
        self._vdb_markers    = {}
        self._vdb_global     = {}  # pre-translated global-frame VDB, keyed by robot_name
        # Per-robot NavSatFix re-publisher (frame_id rewritten to 'map' so
        # Foxglove's Map panel will accept it as a location source).
        self._location_pubs: dict = {}
        self._subscribed_gps  = set()
        self._subscribed_odom = set()
        self._subscribed_traj = set()
        self._subscribed_plan = set()

        # Make 'map' a known frame on the GCS domain so Foxglove resolves it.
        # The robot side already publishes the same identity static TF
        # (robot.launch.xml world_to_map_broadcaster).
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(t)

        # Publish a stationary "map origin" location at the configured
        # ORIGIN_LAT/LON so the Foxglove Map panel always has a fixed
        # reference (otherwise the panel auto-centers on the robot, making
        # the robot appear at the visible map's center). 1 Hz so any newly
        # opened Map layer picks it up quickly.
        self._origin_pub = self.create_publisher(
            NavSatFix, '/gcs/map_origin/location', 10)
        self.create_timer(1.0, self._publish_map_origin)
        self._subscribed_vdb  = set()
        self._alt_ground      = None

        self._pub = self.create_publisher(MarkerArray, '/gcs/robot_markers', 10)

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
            if robot_name in self._vdb_markers:
                bx, by, bz = pos
                self._vdb_global[robot_name] = _translate_marker(
                    self._vdb_markers[robot_name], bx, by, bz)

        # Re-publish on /gcs/<robot>/location with frame_id='map' so the
        # Foxglove Map panel will accept it as a location source. lat/lon
        # are passed through unchanged.
        if robot_name not in self._location_pubs:
            self._location_pubs[robot_name] = self.create_publisher(
                NavSatFix, f'/gcs/{robot_name}/location', 10)
        out = NavSatFix()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.status = msg.status
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.position_covariance = msg.position_covariance
        out.position_covariance_type = msg.position_covariance_type
        self._location_pubs[robot_name].publish(out)

    def _odom_callback(self, msg: Odometry, robot_name: str):
        o = msg.pose.pose.orientation
        self._orientations[robot_name] = (o.x, o.y, o.z, o.w)

    def _traj_callback(self, msg: MarkerArray, robot_name: str):
        self._trajectories[robot_name] = msg

    def _plan_callback(self, msg: Path, robot_name: str):
        self._global_plans[robot_name] = msg

    def _vdb_callback(self, msg: Marker, robot_name: str):
        self._vdb_markers[robot_name] = msg
        boot = self._gps_boot.get(robot_name)
        if boot is not None:
            bx, by, bz = boot
            self._vdb_global[robot_name] = _translate_marker(msg, bx, by, bz)

    def _publish_map_origin(self):
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.status.status = 0  # STATUS_FIX
        m.status.service = 1  # SERVICE_GPS
        m.latitude = ORIGIN_LAT
        m.longitude = ORIGIN_LON
        m.altitude = 0.0
        self._origin_pub.publish(m)

    def _publish_markers(self):
        if not self._gps_positions:
            return

        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=1, nanosec=0)

        for i, robot_name in enumerate(sorted(self._gps_positions.keys())):
            x, y, z = self._gps_positions[robot_name]
            orientation = self._orientations.get(robot_name)

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

            boot = self._gps_boot.get(robot_name)

            traj = self._trajectories.get(robot_name)
            if traj is not None and boot is not None:
                bx, by, bz = boot
                transformed = transform_marker_array(traj, bx, by, bz)
                for k, m in enumerate(transformed.markers):
                    m.ns = f'{robot_name}_traj'
                    m.id = i * 10000 + k
                    m.header.stamp = now
                    m.lifetime = lifetime
                    if m.color.a > 0:
                        m.color.r = 0.8
                        m.color.g = 0.5
                        m.color.b = 0.0
                    needs_points = m.type in (Marker.LINE_STRIP, Marker.LINE_LIST,
                                              Marker.POINTS, Marker.ARROW)
                    if needs_points and len(m.points) == 0:
                        continue
                    array.markers.append(m)

            plan = self._global_plans.get(robot_name)
            if plan is not None and boot is not None:
                bx, by, bz = boot
                poses = plan.poses
                n_poses = len(poses)
                if n_poses >= 2:
                    color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
                    xyz = np.array(
                        [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
                         for ps in poses], dtype=np.float64)
                    xyz[:, 0] += bx
                    xyz[:, 1] += by
                    xyz[:, 2] += bz
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
                    line.points = [Point(x=float(r[0]), y=float(r[1]), z=float(r[2]))
                                   for r in xyz]
                    array.markers.append(line)

            vdb = self._vdb_global.get(robot_name)
            if vdb is not None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = f'{robot_name}_vdb'
                m.id = i * 10000 + 9998
                m.type = vdb.type
                m.action = vdb.action
                m.pose = vdb.pose
                m.scale = vdb.scale
                m.color = vdb.color
                m.colors = vdb.colors
                m.points = vdb.points  # pre-translated, safe to share read-only
                m.lifetime = Duration(sec=2, nanosec=0)
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
