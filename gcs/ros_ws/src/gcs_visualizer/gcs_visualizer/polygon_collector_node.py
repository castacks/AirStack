"""Polygon collector: receives click points from Foxglove 3D panel,
manages a polygon vertex list, and publishes markers + vertex list.

Commands arrive as JSON on /gcs/polygon/command (std_msgs/String):
  {"action": "add", "x": 1.0, "y": 2.0, "z": 0.0}
  {"action": "delete", "index": 2}
  {"action": "move", "index": 1, "x": 3.0, "y": 4.0, "z": 0.0}
  {"action": "reorder", "from": 2, "to": 0}
  {"action": "clear"}
  {"action": "set_altitude", "z": 0.0}  (sets default z for clicks)

Publishes:
  /gcs/polygon/markers  (MarkerArray — red vertex spheres + closed red LINE_STRIP)
  /gcs/polygon/list     (String — JSON {vertices: [{x,y,z},...], default_z: float})

The polygon is rendered as a closed loop (last vertex connects back to first)
so the user sees the area they're outlining.
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

_RELIABLE = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)

_LATCHED = QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)

_LINE_COLOR = ColorRGBA(r=1.0, g=0.15, b=0.15, a=1.0)
_VERTEX_COLOR = ColorRGBA(r=1.0, g=0.15, b=0.15, a=1.0)
_TEXT_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
_VERTEX_RADIUS = 0.35
_LINE_WIDTH = 0.15


class PolygonCollectorNode(Node):
    def __init__(self):
        super().__init__('polygon_collector')

        self._vertices: list[dict] = []  # [{x, y, z}, ...]
        self._default_z = 0.0
        self._frame_id = 'map'
        # Click capture is gated by an Enable toggle so the waypoint and
        # polygon editors can share /clicked_point without conflicting.
        self._enabled = False

        self.create_subscription(
            PointStamped, '/clicked_point', self._on_click, 10)
        self.create_subscription(
            String, '/gcs/polygon/command', self._on_command, _RELIABLE)

        self._marker_pub = self.create_publisher(
            MarkerArray, '/gcs/polygon/markers', _LATCHED)
        self._list_pub = self.create_publisher(
            String, '/gcs/polygon/list', _LATCHED)

        self._publish_all()
        self.get_logger().info('Polygon collector ready')

    def _on_click(self, msg: PointStamped):
        if not self._enabled:
            return
        v = {
            'x': round(msg.point.x, 2),
            'y': round(msg.point.y, 2),
            'z': round(self._default_z, 2),
        }
        self._vertices.append(v)
        self.get_logger().info(
            f'Added vertex {len(self._vertices)-1}: '
            f'({v["x"]}, {v["y"]}, {v["z"]})')
        self._publish_all()

    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Bad command JSON: {msg.data}')
            return

        action = cmd.get('action', '')

        if action == 'add':
            self._vertices.append({
                'x': round(float(cmd.get('x', 0)), 2),
                'y': round(float(cmd.get('y', 0)), 2),
                'z': round(float(cmd.get('z', self._default_z)), 2),
            })

        elif action == 'delete':
            idx = int(cmd.get('index', -1))
            if 0 <= idx < len(self._vertices):
                self._vertices.pop(idx)

        elif action == 'move':
            idx = int(cmd.get('index', -1))
            if 0 <= idx < len(self._vertices):
                self._vertices[idx] = {
                    'x': round(float(cmd.get('x', 0)), 2),
                    'y': round(float(cmd.get('y', 0)), 2),
                    'z': round(float(cmd.get('z', 0)), 2),
                }

        elif action == 'reorder':
            fi = int(cmd.get('from', -1))
            ti = int(cmd.get('to', -1))
            if 0 <= fi < len(self._vertices) and 0 <= ti < len(self._vertices):
                v = self._vertices.pop(fi)
                self._vertices.insert(ti, v)

        elif action == 'clear':
            self._vertices.clear()

        elif action == 'set_altitude':
            self._default_z = float(cmd.get('z', self._default_z))
            self.get_logger().info(f'Default z set to {self._default_z}')
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

    def _publish_all(self):
        now = self.get_clock().now().to_msg()
        self._publish_markers(now)
        self._publish_list()

    def _publish_markers(self, stamp: Time):
        array = MarkerArray()

        delete = Marker()
        delete.header.frame_id = self._frame_id
        delete.header.stamp = stamp
        delete.ns = 'polygon'
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        for i, v in enumerate(self._vertices):
            pos = Point(x=v['x'], y=v['y'], z=v['z'])

            sphere = Marker()
            sphere.header.frame_id = self._frame_id
            sphere.header.stamp = stamp
            sphere.ns = 'polygon'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = pos
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = _VERTEX_RADIUS
            sphere.scale.y = _VERTEX_RADIUS
            sphere.scale.z = _VERTEX_RADIUS
            sphere.color = _VERTEX_COLOR
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self._frame_id
            label.header.stamp = stamp
            label.ns = 'polygon'
            label.id = i * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=v['x'], y=v['y'], z=v['z'] + _VERTEX_RADIUS + 0.3)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.4
            label.color = _TEXT_COLOR
            label.text = str(i)
            array.markers.append(label)

        # Closed polygon outline (last vertex back to first).
        if len(self._vertices) >= 2:
            line = Marker()
            line.header.frame_id = self._frame_id
            line.header.stamp = stamp
            line.ns = 'polygon'
            line.id = len(self._vertices) * 2 + 100
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = _LINE_WIDTH
            line.color = _LINE_COLOR
            line.pose.orientation.w = 1.0
            for v in self._vertices:
                line.points.append(Point(x=v['x'], y=v['y'], z=v['z']))
            # Close the loop
            first = self._vertices[0]
            line.points.append(Point(x=first['x'], y=first['y'], z=first['z']))
            array.markers.append(line)

        self._marker_pub.publish(array)

    def _publish_list(self):
        self._list_pub.publish(String(data=json.dumps({
            'vertices': self._vertices,
            'default_z': self._default_z,
            'enabled': self._enabled,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = PolygonCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
