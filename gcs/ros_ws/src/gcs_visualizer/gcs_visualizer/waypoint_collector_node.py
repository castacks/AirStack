"""Waypoint collector: receives click points from Foxglove 3D panel,
manages a waypoint list, and publishes markers + path.

Commands arrive as JSON on /gcs/waypoints/command (std_msgs/String):
  {"action": "add", "x": 1.0, "y": 2.0, "z": 5.0}
  {"action": "delete", "index": 2}
  {"action": "move", "index": 1, "x": 3.0, "y": 4.0, "z": 5.0}
  {"action": "reorder", "from": 2, "to": 0}
  {"action": "clear"}
  {"action": "set_altitude", "z": 10.0}  (sets default altitude for clicks)

Publishes:
  /gcs/waypoints/markers  (MarkerArray — spheres, labels, connecting lines)
  /gcs/waypoints/list     (String — JSON array of waypoints for the panel)
  /gcs/waypoints/path     (nav_msgs/Path — for use by navigate task)
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

_RELIABLE = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)

_LATCHED = QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Marker colors
_STAR_COLOR = ColorRGBA(r=0.2, g=0.8, b=1.0, a=1.0)
_TEXT_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
_STAR_SCALE = 1.0


class WaypointCollectorNode(Node):
    def __init__(self):
        super().__init__('waypoint_collector')

        self._waypoints: list[dict] = []  # [{x, y, z}, ...]
        self._default_z = 10.0
        self._frame_id = 'map'
        # Click capture is gated by an Enable toggle from the editor panel so
        # the waypoint and polygon editors can share /clicked_point without
        # stealing each other's clicks.
        self._enabled = False

        # Subscribers
        self.create_subscription(
            PointStamped, '/clicked_point', self._on_click, 10)
        self.create_subscription(
            String, '/gcs/waypoints/command', self._on_command, _RELIABLE)

        # Publishers
        self._marker_pub = self.create_publisher(
            MarkerArray, '/gcs/waypoints/markers', _LATCHED)
        self._list_pub = self.create_publisher(
            String, '/gcs/waypoints/list', _LATCHED)
        self._path_pub = self.create_publisher(
            Path, '/gcs/waypoints/path', _LATCHED)

        # Publish initial empty state
        self._publish_all()
        self.get_logger().info('Waypoint collector ready')

    # ── Click handler ────────────────────────────────────────────────────
    def _on_click(self, msg: PointStamped):
        if not self._enabled:
            return
        wp = {
            'x': round(msg.point.x, 2),
            'y': round(msg.point.y, 2),
            'z': round(self._default_z, 2),
        }
        self._waypoints.append(wp)
        self.get_logger().info(
            f'Added waypoint {len(self._waypoints)-1}: '
            f'({wp["x"]}, {wp["y"]}, {wp["z"]})')
        self._publish_all()

    # ── Command handler ──────────────────────────────────────────────────
    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Bad command JSON: {msg.data}')
            return

        action = cmd.get('action', '')

        if action == 'add':
            self._waypoints.append({
                'x': round(float(cmd.get('x', 0)), 2),
                'y': round(float(cmd.get('y', 0)), 2),
                'z': round(float(cmd.get('z', self._default_z)), 2),
            })

        elif action == 'delete':
            idx = int(cmd.get('index', -1))
            if 0 <= idx < len(self._waypoints):
                self._waypoints.pop(idx)

        elif action == 'move':
            idx = int(cmd.get('index', -1))
            if 0 <= idx < len(self._waypoints):
                self._waypoints[idx] = {
                    'x': round(float(cmd.get('x', 0)), 2),
                    'y': round(float(cmd.get('y', 0)), 2),
                    'z': round(float(cmd.get('z', 0)), 2),
                }

        elif action == 'reorder':
            fi = int(cmd.get('from', -1))
            ti = int(cmd.get('to', -1))
            if 0 <= fi < len(self._waypoints) and 0 <= ti < len(self._waypoints):
                wp = self._waypoints.pop(fi)
                self._waypoints.insert(ti, wp)

        elif action == 'clear':
            self._waypoints.clear()

        elif action == 'set_altitude':
            self._default_z = float(cmd.get('z', self._default_z))
            self.get_logger().info(f'Default altitude set to {self._default_z}')
            self._publish_list()
            return

        elif action == 'set_enabled':
            self._enabled = bool(cmd.get('enabled', False))
            self.get_logger().info(
                f'Click capture {"ENABLED" if self._enabled else "disabled"}')
            self._publish_list()
            return

        else:
            self.get_logger().warn(f'Unknown command: {action}')
            return

        self._publish_all()

    # ── Publishing ───────────────────────────────────────────────────────
    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        self._publish_markers(now)
        self._publish_list()
        self._publish_path(now)

    def _publish_markers(self, stamp: Time):
        array = MarkerArray()

        # Delete all previous markers
        delete = Marker()
        delete.header.frame_id = self._frame_id
        delete.header.stamp = stamp
        delete.ns = 'waypoints'
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        for i, wp in enumerate(self._waypoints):
            pos = Point(x=wp['x'], y=wp['y'], z=wp['z'])

            # Star glyph (text marker, always faces camera)
            star = Marker()
            star.header.frame_id = self._frame_id
            star.header.stamp = stamp
            star.ns = 'waypoints'
            star.id = i * 2
            star.type = Marker.TEXT_VIEW_FACING
            star.action = Marker.ADD
            star.pose.position = pos
            star.pose.orientation.w = 1.0
            star.scale.z = _STAR_SCALE
            star.color = _STAR_COLOR
            star.text = '★'
            array.markers.append(star)

            # Index label
            label = Marker()
            label.header.frame_id = self._frame_id
            label.header.stamp = stamp
            label.ns = 'waypoints'
            label.id = i * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=wp['x'], y=wp['y'], z=wp['z'] + 0.5)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color = _TEXT_COLOR
            label.text = str(i)
            array.markers.append(label)

        self._marker_pub.publish(array)

    def _publish_list(self):
        self._list_pub.publish(String(data=json.dumps({
            'waypoints': self._waypoints,
            'default_z': self._default_z,
            'enabled': self._enabled,
        })))

    def _publish_path(self, stamp: Time):
        path = Path()
        path.header = Header(frame_id=self._frame_id, stamp=stamp)
        for wp in self._waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = Point(x=wp['x'], y=wp['y'], z=wp['z'])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
