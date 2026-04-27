"""Waypoint collector: receives click points from Foxglove 3D panel,
manages a waypoint list and named saves, and publishes markers + path.

Commands arrive as JSON on /gcs/waypoints/command (std_msgs/String):
  {"action": "add", "x": 1.0, "y": 2.0, "z": 5.0}
  {"action": "delete", "index": 2}
  {"action": "move", "index": 1, "x": 3.0, "y": 4.0, "z": 5.0}
  {"action": "reorder", "from": 2, "to": 0}
  {"action": "clear"}
  {"action": "set_altitude", "z": 10.0}  (sets default altitude for clicks)
  {"action": "add_save", "name": "robot_1"}    (snapshot active list, in-memory)
  {"action": "save_save", "name": "robot_1"}   (persist that save to disk)
  {"action": "load_save", "name": "robot_1"}   (replace active list with save)
  {"action": "delete_save", "name": "robot_1"} (remove save)

Publishes:
  /gcs/waypoints/markers       (MarkerArray — active editor only)
  /gcs/waypoints/list          (String — JSON of active waypoints for the panel)
  /gcs/waypoints/path          (nav_msgs/Path — for use by navigate task)
  /gcs/waypoints/save_markers  (MarkerArray — every save rendered in its color)
  /gcs/waypoints/saves         (String JSON — save metadata for editor + tasks)
"""

import copy
import json
import os

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

# Save persistence
_SAVES_PATH = os.path.expanduser('~/.airstack/gcs_waypoint_saves.json')

# Visually distinct palette for saves — auto-assigned by hashing the save name
# so the same name keeps the same color across sessions.
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
    """Pick the lowest palette index not currently in use; wrap if all are taken.

    `used_colors` is an iterable of [r, g, b] lists from existing saves.
    """
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

        # Named saves: in-memory by default; the user explicitly Saves to
        # persist. Disk holds only the saves the user has Saved.
        # Shape: {name: {'color': [r,g,b], 'vertices': [{x,y,z}, ...]}}
        self._saves: dict = {}
        # Track the namespaces we last published so we can DELETEALL stale
        # ones (Marker.DELETEALL only clears markers in its own namespace).
        self._published_save_names: set[str] = set()
        self._saved_on_disk = _load_saves_from_disk()
        # On startup, in-memory mirrors disk. If a save on disk has no color
        # (e.g. a hand-edited file), assign the next available palette entry.
        for name, payload in self._saved_on_disk.items():
            color = payload.get('color')
            if color is None:
                color = list(_next_palette_color(
                    s['color'] for s in self._saves.values()))
            self._saves[name] = {
                'color': color,
                'vertices': payload.get('vertices', []),
            }

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
        self._save_marker_pub = self.create_publisher(
            MarkerArray, '/gcs/waypoints/save_markers', _LATCHED)
        self._saves_meta_pub = self.create_publisher(
            String, '/gcs/waypoints/saves', _LATCHED)

        # Publish initial empty state (active + saves loaded from disk)
        self._publish_all()
        self._publish_saves()
        self.get_logger().info(
            f'Waypoint collector ready ({len(self._saves)} saves loaded)')

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
                'vertices': [dict(wp) for wp in self._waypoints],
            }
            self.get_logger().info(
                f'Added save {name!r} with {len(self._waypoints)} waypoints')
            self._publish_saves()
            return

        elif action == 'save_save':
            name = str(cmd.get('name', '')).strip()
            if not name or name not in self._saves:
                return
            # Deep-copy so further in-memory edits don't silently mutate the
            # on-disk mirror and break the `saved` comparison.
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
            self._waypoints = [dict(wp) for wp in self._saves[name]['vertices']]
            self.get_logger().info(
                f'Loaded save {name!r} into active editor '
                f'({len(self._waypoints)} waypoints)')

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

    # ── Save publishing ──────────────────────────────────────────────────
    def _publish_saves(self):
        """Publish both the save metadata JSON and the save MarkerArray."""
        now = self.get_clock().now().to_msg()

        # Metadata for editor + Robot Tasks panel
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

        # Marker array — all saves in one publish, each in its own namespace.
        # DELETEALL only clears markers in its own ns, so emit one DELETEALL
        # per previously- or currently-known save namespace before re-adding.
        array = MarkerArray()
        current_names = set(self._saves.keys())
        for ns_name in (self._published_save_names | current_names):
            wipe = Marker()
            wipe.header.frame_id = self._frame_id
            wipe.header.stamp = now
            wipe.ns = f'waypoint_saves/{ns_name}'
            wipe.action = Marker.DELETEALL
            array.markers.append(wipe)

        next_id = 0
        for name, save in self._saves.items():
            r, g, b = save['color']
            color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            ns = f'waypoint_saves/{name}'

            # Big colored index numbers — these ARE the waypoint markers.
            for i, wp in enumerate(save['vertices']):
                num = Marker()
                num.header.frame_id = self._frame_id
                num.header.stamp = now
                num.ns = ns
                num.id = next_id
                next_id += 1
                num.type = Marker.TEXT_VIEW_FACING
                num.action = Marker.ADD
                num.pose.position = Point(
                    x=wp['x'], y=wp['y'], z=wp['z'])
                num.pose.orientation.w = 1.0
                num.scale.z = _STAR_SCALE
                num.color = color
                num.text = str(i)
                array.markers.append(num)

            # Connecting line through the waypoints (route order)
            if len(save['vertices']) >= 2:
                line = Marker()
                line.header.frame_id = self._frame_id
                line.header.stamp = now
                line.ns = ns
                line.id = next_id
                next_id += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.1
                line.color = color
                line.pose.orientation.w = 1.0
                for wp in save['vertices']:
                    line.points.append(
                        Point(x=wp['x'], y=wp['y'], z=wp['z']))
                array.markers.append(line)

            # Save-name label floating above the first waypoint.
            if save['vertices']:
                first = save['vertices'][0]
                label = Marker()
                label.header.frame_id = self._frame_id
                label.header.stamp = now
                label.ns = ns
                label.id = next_id
                next_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position = Point(
                    x=first['x'], y=first['y'], z=first['z'] + 1.2)
                label.pose.orientation.w = 1.0
                label.scale.z = _STAR_SCALE
                label.color = color
                label.text = name
                array.markers.append(label)

        self._save_marker_pub.publish(array)
        self._published_save_names = current_names


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
