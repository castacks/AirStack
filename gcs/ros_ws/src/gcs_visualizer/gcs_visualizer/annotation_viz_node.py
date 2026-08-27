"""annotation_viz_node — publish ground-truth bbox annotations for a scene.

Loads `annotations/<scene_name>.json` from this package's share dir and
publishes a `visualization_msgs/MarkerArray` on `/gcs/annotations/bboxes`.
The JSON is in the Isaac Sim global frame, which matches the GCS-side
global ENU `map` frame, so no per-drone transform is applied.

Tuning parameters (all live, applied uniformly to every bbox):
    scene_name (str)                     — JSON file under annotations/
    tx, ty, tz (double, m)               — global translation of every center
    roll_deg, pitch_deg, yaw_deg (double) — orientation of every box (about own center)

Each entry may carry an optional `bbox_world.quat_xyzw = [x,y,z,w]`. When
present, it's composed under the global rotation:
    pose.orientation = global_rot * entry_quat

This lets the annotation_tuner bake the global rotation back into the JSON
on a per-entry basis, so future runs need no live tuning.

Enable `/gcs/annotations/bboxes` in Foxglove's 3D panel to see them.
"""

import colorsys
import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from gcs_visualizer.gcs_utils import multiply_quaternions


TOPIC = '/gcs/annotations/bboxes'
FRAME_ID = 'map'
# The GT is static, so publish it ONCE on a latched (TRANSIENT_LOCAL) topic
# rather than streaming it. Late subscribers (Foxglove, the mcap recorder) still
# receive the last sample on connect — same as /tf_static — so the bag stores one
# annotation message instead of thousands. Re-published only on a live edit
# (annotation_tuner param change) or scene change. Lifetime 0 = persist forever
# (no periodic refresh to keep it alive in RViz/Foxglove).
MARKER_LIFETIME_S = 0

_BOX_CORNERS = [
    (-1, -1, -1), (+1, -1, -1), (+1, +1, -1), (-1, +1, -1),
    (-1, -1, +1), (+1, -1, +1), (+1, +1, +1), (-1, +1, +1),
]
_BOX_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0),
              (4, 5), (5, 6), (6, 7), (7, 4),
              (0, 4), (1, 5), (2, 6), (3, 7)]


def _euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Intrinsic ZYX (yaw-pitch-roll) Euler angles in degrees → (x,y,z,w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class AnnotationViz(Node):
    TUNING_PARAMS = ('tx', 'ty', 'tz', 'roll_deg', 'pitch_deg', 'yaw_deg')

    def __init__(self):
        super().__init__('annotation_viz_node')

        # scene_name is wired from the RESULTS_SCENE env var via the launch file
        # ($(env RESULTS_SCENE RetroNeighborhood)); the mission runner sets it per
        # environment so multi-scene missions show each iteration's own GT.
        self.declare_parameter('scene_name', 'RetroNeighborhood')
        for name in self.TUNING_PARAMS:
            self.declare_parameter(name, 0.0)

        self.scene_name = self.get_parameter('scene_name').value
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        self.scene_path = ''
        self.annotations = []
        self._load_scene(self.scene_name)

        # Latched: publish once; late joiners get the last sample on subscribe.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(MarkerArray, TOPIC, latched_qos)
        self._publish()   # once; re-published only on param/scene change

        # WAIT FOR THE FILE. On a GENERATED scene the GT does not exist when
        # this node starts: isaac-sim writes `annotations/<scene>.json` partway
        # through scene generation, which finishes minutes after the GCS comes
        # up. Publishing once at startup therefore latched an EMPTY MarkerArray
        # for the whole run — a recorded bag shows the topic present with a
        # single 0-marker message, and a viewer that connects later gets that
        # empty sample. Poll until it appears, then publish for real and stop.
        self._retry_timer = None
        if not self.annotations:
            self._retry_left = int(self._p_int('annotation_retry_count', 120))
            self._retry_timer = self.create_timer(5.0, self._retry_load)
            self.get_logger().info(
                f'no annotations yet at {self.scene_path} — polling every 5 s '
                f'({self._retry_left} tries)')
        # AND WATCH IT AFTERWARDS. The GCS comes up before the scene is
        # built, so the file that exists at startup is the PREVIOUS run's GT;
        # the launcher rewrites it minutes later (houses, trees, cars, this
        # run's people). Loaded-once meant every run showed the run before.
        # Reload and republish whenever the file's mtime changes.
        self._file_mtime = self._mtime()
        self._watch_timer = self.create_timer(5.0, self._watch_file)

        self.add_on_set_parameters_callback(self._on_set_params)

    def _p_int(self, name, default):
        try:
            self.declare_parameter(name, default)
        except Exception:
            pass
        try:
            return int(self.get_parameter(name).value)
        except Exception:
            return default

    def _mtime(self):
        try:
            return os.path.getmtime(self.scene_path)
        except OSError:
            return None

    def _watch_file(self):
        """Reload + republish when the annotation file is rewritten."""
        m = self._mtime()
        if m is None or m == self._file_mtime:
            return
        self._file_mtime = m
        try:
            self._load_scene(self.scene_name)
        except Exception as exc:          # a half-written file: try next tick
            self.get_logger().warn(f'annotation reload failed: {exc}')
            self._file_mtime = None
            return
        self._publish()
        self.get_logger().info(
            f'annotation file changed — reloaded {len(self.annotations)} '
            f'boxes from {self.scene_path} and republished')

    def _retry_load(self):
        """Re-attempt the load until the scene writes its GT, then stop."""
        self._retry_left -= 1
        self._load_scene(self.scene_name)
        if self.annotations:
            self._publish()
            self.get_logger().info(
                f'annotations appeared: {len(self.annotations)} box(es) from '
                f'{self.scene_path}')
            self._retry_timer.cancel()
            self._retry_timer = None
        elif self._retry_left <= 0:
            self.get_logger().warn(
                f'giving up waiting for {self.scene_path}; '
                'the GT topic stays empty for this run')
            self._retry_timer.cancel()
            self._retry_timer = None

    # ------------------------------------------------------------------ JSON

    def _annotation_dirs(self):
        """Where to look for `<scene>.json`, most specific first.

        The package SHARE dir is a build-time COPY, so a scene whose ground
        truth is generated at run time — the procedural scenes write theirs from
        the layout generator, see `simulation/isaac-sim/utils/scene_annotations.py`
        — lands in the SOURCE tree and never reaches the copy without a rebuild.
        Both containers bind-mount the same repo, so preferring the source tree
        is what lets the simulator hand this node a fresh GT in the same run.

        ANNOTATIONS_DIR overrides both, for a GT that lives somewhere else
        entirely (a mission's results dir, a hand-tuned file).
        """
        dirs = []
        env_dir = os.environ.get('ANNOTATIONS_DIR', '').strip()
        if env_dir:
            dirs.append(env_dir)
        share = get_package_share_directory('gcs_visualizer')
        # <ws>/install/gcs_visualizer/share/gcs_visualizer -> <ws>/src/...
        src = os.path.normpath(os.path.join(
            share, '..', '..', '..', '..', 'src', 'gcs_visualizer', 'annotations'))
        dirs.append(src)
        dirs.append(os.path.join(share, 'annotations'))
        return dirs

    def _scene_path_for(self, scene_name):
        candidates = [os.path.join(d, f'{scene_name}.json')
                      for d in self._annotation_dirs()]
        for path in candidates:
            if os.path.isfile(path):
                return path
        # Nothing found: return the share path so the existing "missing file"
        # log names the place a hand-authored GT would normally live.
        return candidates[-1]

    def _load_scene(self, scene_name):
        path = self._scene_path_for(scene_name)
        if not os.path.exists(path):
            # Throttled: on a generated scene this is the EXPECTED state for
            # the first minutes while isaac-sim writes the GT, and the retry
            # timer would otherwise log it every 5 s for the whole wait.
            self.get_logger().warn(f'Annotation file not found: {path}',
                                   throttle_duration_sec=30.0)
            self.scene_name = scene_name
            self.scene_path = path
            self.annotations = []
            return
        with open(path, 'r') as f:
            data = json.load(f)
        out = []
        for item in data:
            bw = item['bbox_world']
            cx, cy, cz = bw['center_xyz_m']
            sx, sy, sz = bw['size_xyz_m']
            qx, qy, qz, qw = bw.get('quat_xyzw', [0.0, 0.0, 0.0, 1.0])
            out.append({'class': item['class'],
                        'center': [float(cx), float(cy), float(cz)],
                        'size': [float(sx), float(sy), float(sz)],
                        'quat': [float(qx), float(qy), float(qz), float(qw)]})
        self.scene_name = scene_name
        self.scene_path = path
        self.annotations = out
        self.get_logger().info(
            f'Loaded scene={scene_name} annotations={len(out)} from {path} '
            f'publishing to {TOPIC}')

    # ----------------------------------------------------------- parameters

    def _on_set_params(self, params):
        scene_changed = False
        next_state = {
            'tx': self.tx, 'ty': self.ty, 'tz': self.tz,
            'roll_deg': self.roll_deg, 'pitch_deg': self.pitch_deg, 'yaw_deg': self.yaw_deg,
            'scene_name': self.scene_name,
        }
        for p in params:
            if p.name in self.TUNING_PARAMS:
                try:
                    v = float(p.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name}: not a number')
                if not math.isfinite(v):
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name}: must be finite')
                next_state[p.name] = v
            elif p.name == 'scene_name':
                if not isinstance(p.value, str) or not p.value:
                    return SetParametersResult(successful=False,
                                               reason='scene_name: must be non-empty string')
                if p.value != self.scene_name:
                    scene_changed = True
                    next_state['scene_name'] = p.value

        self.tx = next_state['tx']
        self.ty = next_state['ty']
        self.tz = next_state['tz']
        self.roll_deg = next_state['roll_deg']
        self.pitch_deg = next_state['pitch_deg']
        self.yaw_deg = next_state['yaw_deg']
        if scene_changed:
            self._load_scene(next_state['scene_name'])
        self._publish()
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------- markers

    def _class_color(self, name):
        hue = (hash(name) & 0xFFFF) / 0xFFFF
        r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
        return float(r), float(g), float(b)

    def _publish(self):
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0
        global_q = _euler_deg_to_quat(self.roll_deg, self.pitch_deg, self.yaw_deg)

        for ann in self.annotations:
            cx, cy, cz = ann['center']
            sx, sy, sz = ann['size']
            r, g, b = self._class_color(ann['class'])
            qx, qy, qz, qw = multiply_quaternions(global_q, tuple(ann['quat']))
            px, py, pz = cx + self.tx, cy + self.ty, cz + self.tz

            box = Marker()
            box.header.frame_id = FRAME_ID
            box.header.stamp = now
            box.ns = ann['class']
            box.id = mid; mid += 1
            box.type = Marker.LINE_LIST
            box.action = Marker.ADD
            box.pose.position.x = float(px)
            box.pose.position.y = float(py)
            box.pose.position.z = float(pz)
            box.pose.orientation.x = float(qx)
            box.pose.orientation.y = float(qy)
            box.pose.orientation.z = float(qz)
            box.pose.orientation.w = float(qw)
            box.scale.x = 0.05
            box.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
            corners = [(ex * hx, ey * hy, ez * hz)
                       for ex, ey, ez in _BOX_CORNERS]
            for a, c in _BOX_EDGES:
                for cpt in (corners[a], corners[c]):
                    p = Point()
                    p.x, p.y, p.z = float(cpt[0]), float(cpt[1]), float(cpt[2])
                    box.points.append(p)
            box.lifetime.sec = MARKER_LIFETIME_S
            msg.markers.append(box)

            fill = Marker()
            fill.header.frame_id = FRAME_ID
            fill.header.stamp = now
            fill.ns = ann['class'] + '_fill'
            fill.id = mid; mid += 1
            fill.type = Marker.CUBE
            fill.action = Marker.ADD
            fill.pose = box.pose
            fill.scale.x = float(sx)
            fill.scale.y = float(sy)
            fill.scale.z = float(sz)
            fill.color = ColorRGBA(r=r, g=g, b=b, a=0.15)
            fill.lifetime.sec = MARKER_LIFETIME_S
            msg.markers.append(fill)

            label = Marker()
            label.header.frame_id = FRAME_ID
            label.header.stamp = now
            label.ns = ann['class'] + '_label'
            label.id = mid; mid += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(px)
            label.pose.position.y = float(py)
            label.pose.position.z = float(pz) + float(sz) / 2.0 + 0.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.8
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            label.text = ann['class']
            label.lifetime.sec = MARKER_LIFETIME_S
            msg.markers.append(label)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnnotationViz()
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
