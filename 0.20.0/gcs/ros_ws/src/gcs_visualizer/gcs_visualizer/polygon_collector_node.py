"""Polygon collector: receives click points from Foxglove 3D panel,
manages a polygon vertex list and named saves, and publishes markers.

Commands arrive as JSON on /gcs/polygon/command (std_msgs/String):
  {"action": "add", "x": 1.0, "y": 2.0, "z": 0.0}
  {"action": "delete", "index": 2}
  {"action": "move", "index": 1, "x": 3.0, "y": 4.0, "z": 0.0}
  {"action": "reorder", "from": 2, "to": 0}
  {"action": "clear"}
  {"action": "set_altitude", "z": 0.0}  (sets default z for clicks)
  {"action": "add_save", "name": "north"}   (snapshot active vertices, in-memory)
  {"action": "save_save", "name": "north"}  (persist that save to disk)
  {"action": "load_save", "name": "north"}  (replace active list with save)
  {"action": "delete_save", "name": "north"}

Publishes:
  /gcs/polygon/markers       (MarkerArray — active editor only, red)
  /gcs/polygon/list          (String — JSON {vertices, default_z, enabled})
  /gcs/polygon/save_markers  (MarkerArray — each save rendered in its color)
  /gcs/polygon/saves         (String JSON — save metadata)

The active polygon is rendered as a closed loop so the user sees the area
they're outlining; same for each save in its own color.
"""

import copy
import json
import os

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
_SAVE_NUMBER_SCALE = 0.8

_SAVES_PATH = os.path.expanduser('~/.airstack/gcs_polygon_saves.json')

_SAVE_PALETTE = [
    (0.20, 0.80, 1.00),  # cyan
    (1.00, 0.30, 0.80),  # magenta
    (0.50, 1.00, 0.20),  # lime
    (1.00, 0.60, 0.10),  # orange
    (1.00, 0.50, 0.70),  # pink
    (1.00, 0.95, 0.10),  # yellow
    (0.40, 0.70, 1.00),  # sky
    (0.65, 0.35, 1.00),  # violet
    (1.00, 0.45, 0.45),  # coral
    (0.40, 1.00, 0.75),  # mint
]


def _next_palette_color(used_colors) -> tuple[float, float, float]:
    """Lowest palette index not currently in use; wrap if all 10 are taken."""
    used = {tuple(round(x, 6) for x in c) for c in used_colors}
    for entry in _SAVE_PALETTE:
        if tuple(round(x, 6) for x in entry) not in used:
            return entry
    return _SAVE_PALETTE[len(used) % len(_SAVE_PALETTE)]


def _load_saves_from_disk() -> dict:
    try:
        with open(_SAVES_PATH, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _atomic_write_saves(saves_to_persist: dict):
    os.makedirs(os.path.dirname(_SAVES_PATH), exist_ok=True)
    tmp = _SAVES_PATH + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(saves_to_persist, f, indent=2)
    os.replace(tmp, _SAVES_PATH)


class PolygonCollectorNode(Node):
    def __init__(self):
        super().__init__('polygon_collector')

        self._vertices: list[dict] = []  # [{x, y, z}, ...]
        self._default_z = 0.0
        self._frame_id = 'map'
        # Click capture is gated by an Enable toggle so the waypoint and
        # polygon editors can share /clicked_point without conflicting.
        self._enabled = False

        # Named saves (in-memory) and a mirror of what's been persisted.
        self._saves: dict = {}
        # Track namespaces last published so DELETEALL can wipe them on edit.
        self._published_save_names: set[str] = set()
        self._saved_on_disk = _load_saves_from_disk()
        for name, payload in self._saved_on_disk.items():
            color = payload.get('color')
            if color is None:
                color = list(_next_palette_color(
                    s['color'] for s in self._saves.values()))
            self._saves[name] = {
                'color': color,
                'vertices': payload.get('vertices', []),
            }

        self.create_subscription(
            PointStamped, '/clicked_point', self._on_click, 10)
        self.create_subscription(
            String, '/gcs/polygon/command', self._on_command, _RELIABLE)

        self._marker_pub = self.create_publisher(
            MarkerArray, '/gcs/polygon/markers', _LATCHED)
        self._list_pub = self.create_publisher(
            String, '/gcs/polygon/list', _LATCHED)
        self._save_marker_pub = self.create_publisher(
            MarkerArray, '/gcs/polygon/save_markers', _LATCHED)
        self._saves_meta_pub = self.create_publisher(
            String, '/gcs/polygon/saves', _LATCHED)

        self._publish_all()
        self._publish_saves()
        self.get_logger().info(
            f'Polygon collector ready ({len(self._saves)} saves loaded)')

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

        elif action == 'duplicate':
            idx = int(cmd.get('index', -1))
            if 0 <= idx < len(self._vertices):
                self._vertices.insert(idx + 1, dict(self._vertices[idx]))

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

        elif action == 'add_save':
            name = str(cmd.get('name', '')).strip()
            if not name:
                return
            color = (self._saves[name]['color']
                     if name in self._saves
                     else list(_next_palette_color(
                         s['color'] for s in self._saves.values())))
            self._saves[name] = {
                'color': color,
                'vertices': [dict(v) for v in self._vertices],
            }
            self.get_logger().info(
                f'Added save {name!r} with {len(self._vertices)} vertices')
            self._publish_saves()
            return

        elif action == 'save_save':
            name = str(cmd.get('name', '')).strip()
            if not name or name not in self._saves:
                return
            self._saved_on_disk[name] = copy.deepcopy(self._saves[name])
            try:
                _atomic_write_saves(self._saved_on_disk)
                self.get_logger().info(f'Saved save {name!r} to disk')
            except OSError as e:
                self.get_logger().warn(f'Failed to save save {name!r}: {e}')
                self._saved_on_disk.pop(name, None)
            self._publish_saves()
            return

        elif action == 'load_save':
            name = str(cmd.get('name', '')).strip()
            if name not in self._saves:
                return
            self._vertices = [dict(v) for v in self._saves[name]['vertices']]
            self.get_logger().info(
                f'Loaded save {name!r} into active editor '
                f'({len(self._vertices)} vertices)')

        elif action == 'delete_save':
            name = str(cmd.get('name', '')).strip()
            if name not in self._saves:
                return
            self._saves.pop(name, None)
            if name in self._saved_on_disk:
                self._saved_on_disk.pop(name)
                try:
                    _atomic_write_saves(self._saved_on_disk)
                except OSError as e:
                    self.get_logger().warn(
                        f'Failed to update saves file after delete: {e}')
            self.get_logger().info(f'Deleted save {name!r}')
            self._publish_saves()
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

    # ── Save publishing ──────────────────────────────────────────────────
    def _publish_saves(self):
        """Publish save metadata + save MarkerArray for all named saves."""
        now = self.get_clock().now().to_msg()

        meta = []
        for name, save in self._saves.items():
            saved = (name in self._saved_on_disk and
                     self._saved_on_disk[name].get('vertices') == save['vertices'])
            meta.append({
                'name': name,
                'color': save['color'],
                'count': len(save['vertices']),
                'saved': saved,
                'vertices': save['vertices'],
            })
        self._saves_meta_pub.publish(String(data=json.dumps({'saves': meta})))

        # DELETEALL only clears markers in its own namespace, so wipe both
        # currently-known and previously-published namespaces explicitly.
        array = MarkerArray()
        current_names = set(self._saves.keys())
        for ns_name in (self._published_save_names | current_names):
            wipe = Marker()
            wipe.header.frame_id = self._frame_id
            wipe.header.stamp = now
            wipe.ns = f'polygon_saves/{ns_name}'
            wipe.action = Marker.DELETEALL
            array.markers.append(wipe)

        next_id = 0
        for name, save in self._saves.items():
            r, g, b = save['color']
            color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            ns = f'polygon_saves/{name}'
            verts = save['vertices']
            if not verts:
                continue

            if len(verts) >= 2:
                line = Marker()
                line.header.frame_id = self._frame_id
                line.header.stamp = now
                line.ns = ns
                line.id = next_id
                next_id += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = _LINE_WIDTH
                line.color = color
                line.pose.orientation.w = 1.0
                for v in verts:
                    line.points.append(Point(x=v['x'], y=v['y'], z=v['z']))
                line.points.append(
                    Point(x=verts[0]['x'], y=verts[0]['y'], z=verts[0]['z']))
                array.markers.append(line)

            # Name label floating above the first vertex so the user can
            # tell which polygon belongs to which save at a glance.
            first = verts[0]
            label = Marker()
            label.header.frame_id = self._frame_id
            label.header.stamp = now
            label.ns = ns
            label.id = next_id
            next_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=first['x'], y=first['y'], z=first['z'] + 0.6)
            label.pose.orientation.w = 1.0
            label.scale.z = _SAVE_NUMBER_SCALE
            label.color = color
            label.text = name
            array.markers.append(label)

        self._save_marker_pub.publish(array)
        self._published_save_names = current_names


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
