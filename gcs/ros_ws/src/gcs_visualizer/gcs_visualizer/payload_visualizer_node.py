"""
payload_visualizer_node.py — GCS visualization for gossip payloads.

Reads PeerProfile payloads from /gossip/peers and republishes each to its own
topic (/gcs/payload/{robot_name}/{payload_name}) so Foxglove exposes full
visualization controls per payload independently.

Payloads arrive already transformed into global ENU by gossip_node — handlers
only need to apply the display z-offset and republish.

To add a new payload: add a handler method and register it in PAYLOAD_HANDLERS
keyed by the payload name (last segment of the topic in gossip_payloads.yaml).

Handler signature:
    def _handle_<name>(self, robot_name, msg, i, now):
        # i — stable robot index (for marker IDs: i * 100000 + offset)
"""

from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration

from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_bringup.peer_profile import PeerProfile
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point as GPoint
import json

from gcs_visualizer.gcs_utils import (
    transform_marker_array, transform_point_cloud2, point_cloud2_to_cube_marker,
    ROBOT_COLORS,
)

# Must match frame_utils.DEFAULT_ORIGIN_ALT — used to compute the z-offset between
# gossip_node's fixed ENU origin and the GCS display datum (first GPS altitude seen).
_GOSSIP_ORIGIN_ALT = 90.0

_GOSSIP_SEEN_SIZE = 50

GOSSIP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Match the publisher in foxglove_visualizer_node._ground_msl_pub. Latched
# delivery: a late-joining subscriber receives the most recent value on
# subscribe, so startup order between the two visualizer nodes doesn't matter.
GROUND_MSL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class PayloadVisualizerNode(Node):
    def __init__(self):
        super().__init__('payload_visualizer_node')

        self._gps_boot      = {}
        self._last_stamp    = {}
        self._alt_ground    = None
        self._payload_cache = {}
        self._pubs          = {}
        self._seen: OrderedDict = OrderedDict()

        self.create_subscription(
            PeerProfileMsg, '/gossip/peers',
            self._on_peer_profile, GOSSIP_QOS)

        # Source of truth for the ground reference altitude. Computed by
        # foxglove_visualizer_node from msl - odom_z so it isn't biased by a
        # robot that's already in the air at GCS startup.
        self.create_subscription(
            Float64, '/gcs/map_origin/ground_msl',
            self._on_ground_msl, GROUND_MSL_QOS)

        self.get_logger().info('PayloadVisualizerNode started')

    def _on_ground_msl(self, msg: Float64) -> None:
        new_val = float(msg.data)
        if self._alt_ground != new_val:
            self.get_logger().info(
                f'ground_msl received: {new_val:.2f} m '
                f'(display z-offset = {_GOSSIP_ORIGIN_ALT - new_val:.2f} m)')
        self._alt_ground = new_val

    def _pub_for(self, topic, msg_type):
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(msg_type, topic, 10)
        return self._pubs[topic]

    def _update_boot(self, robot_name: str, gps_fix) -> None:
        # Per-robot first-seen flag — independent of the ground reference,
        # which now comes from the latched /gcs/map_origin/ground_msl topic.
        if gps_fix.status.status < 0:
            return
        if robot_name not in self._gps_boot:
            self._gps_boot[robot_name] = True

    def _display_z_offset(self) -> float:
        """Correct for the altitude datum difference between gossip_node and foxglove_visualizer."""
        if self._alt_ground is None:
            return 0.0
        return _GOSSIP_ORIGIN_ALT - self._alt_ground

    def _on_peer_profile(self, msg: PeerProfileMsg) -> None:
        robot_name = msg.robot_name

        msg_id = (robot_name,
                  msg.gps_fix.header.stamp.sec,
                  msg.gps_fix.header.stamp.nanosec)
        if msg_id in self._seen:
            return
        self._seen[msg_id] = None
        if len(self._seen) > _GOSSIP_SEEN_SIZE:
            self._seen.popitem(last=False)

        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)
        if new_t < self._last_stamp.get(robot_name, 0.0):
            return
        self._last_stamp[robot_name] = new_t

        self._update_boot(robot_name, msg.gps_fix)

        if robot_name not in self._gps_boot:
            return

        profile = PeerProfile.from_ros_msg(msg)
        now = self.get_clock().now().to_msg()
        robot_index = self._robot_index(robot_name)

        for name, (type_str, handler) in self.PAYLOAD_HANDLERS.items():
            payload = profile.get_payload_by_name(name)
            if payload is not None:
                self._payload_cache[(robot_name, name)] = payload
            cached = self._payload_cache.get((robot_name, name))
            if cached is not None:
                handler(self, robot_name, cached, robot_index, now)

    def _robot_index(self, robot_name: str) -> int:
        """Stable integer index for a robot name (alphabetical order)."""
        known = sorted(self._gps_boot.keys())
        return known.index(robot_name) if robot_name in known else 0

    def _handle_filtered_rays(self, robot_name, msg, i, now):
        bz = self._display_z_offset()
        out_ma = transform_marker_array(msg, 0.0, 0.0, bz)
        for k, m in enumerate(out_ma.markers):
            m.header.stamp = now
            m.ns = f'{robot_name}_filtered_rays'
            m.id = i * 100000 + k
            m.lifetime = Duration(sec=2, nanosec=0)
        self._pub_for(f'/gcs/payload/{robot_name}/filtered_rays', MarkerArray).publish(out_ma)

    def _handle_raw_frontiers(self, robot_name, msg, i, now):
        color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        marker = point_cloud2_to_cube_marker(
            msg, 0.0, 0.0, self._display_z_offset(),
            ns=f'{robot_name}_raw_frontiers',
            marker_id=i * 100000,
            stamp=now,
            lifetime=Duration(sec=2, nanosec=0),
            fallback_color=(color[0], color[1], color[2], 1.0),
            scale=0.4,
        )
        if marker is None:
            return
        marker.type = Marker.SPHERE_LIST
        marker.colors = []
        out = MarkerArray()
        out.markers.append(marker)
        self._pub_for(f'/gcs/payload/{robot_name}/raw_frontiers', MarkerArray).publish(out)

    def _handle_navigation_mode(self, robot_name, msg, i, now):
        # Pure passthrough — no spatial transform needed for a String.
        self._pub_for(
            f'/gcs/payload/{robot_name}/navigation_mode', String).publish(msg)

    def _handle_kept_frontiers(self, robot_name, msg, i, now):
        """Post-filter own frontiers (altitude + polygon + zone). Renders as a
        sphere list so the shrinking explorable area is obvious in Foxglove.
        Color is the robot's accent color at full opacity."""
        color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        marker = point_cloud2_to_cube_marker(
            msg, 0.0, 0.0, self._display_z_offset(),
            ns=f'{robot_name}_kept_frontiers',
            marker_id=i * 100000 + 200,
            stamp=now,
            lifetime=Duration(sec=2, nanosec=0),
            fallback_color=(color[0], color[1], color[2], 1.0),
            scale=0.5,
        )
        if marker is None:
            return
        marker.type = Marker.SPHERE_LIST
        marker.colors = []
        out = MarkerArray()
        out.markers.append(marker)
        self._pub_for(
            f'/gcs/payload/{robot_name}/kept_frontiers', MarkerArray).publish(out)

    def _handle_completed_zones(self, robot_name, msg, i, now):
        """Coverage cones (payload v2: x,y,yaw,range) or disks (v1: x,y) gossiped
        by raven_nav. Renders each cone as a flat triangular wedge."""
        from sensor_msgs_py import point_cloud2 as pc2
        import math
        color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        field_names = {f.name for f in msg.fields}
        has_cone = {'yaw', 'range'}.issubset(field_names)
        req = ('x', 'y', 'yaw', 'range') if has_cone else ('x', 'y')
        try:
            pts = list(pc2.read_points(msg, field_names=req, skip_nans=True))
        except Exception:
            return
        out = MarkerArray()
        half_hfov = math.radians(90.0) * 0.5
        for j, p in enumerate(pts):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = f'{robot_name}_completed_zones'
            m.id = i * 100000 + j
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 0.18
            m.lifetime = Duration(sec=3, nanosec=0)
            m.pose.orientation.w = 1.0
            if has_cone:
                x, y, yaw, rng = float(p[0]), float(p[1]), float(p[2]), float(p[3])
                m.type = Marker.TRIANGLE_LIST
                m.action = Marker.ADD
                m.pose.position.x = 0.0
                m.pose.position.y = 0.0
                m.pose.position.z = 0.1
                m.scale.x = 1.0
                m.scale.y = 1.0
                m.scale.z = 1.0
                left_x  = x + rng * math.cos(yaw + half_hfov)
                left_y  = y + rng * math.sin(yaw + half_hfov)
                right_x = x + rng * math.cos(yaw - half_hfov)
                right_y = y + rng * math.sin(yaw - half_hfov)
                for px, py in ((x, y), (left_x, left_y), (right_x, right_y)):
                    pt = GPoint(); pt.x = px; pt.y = py; pt.z = 0.0
                    m.points.append(pt)
            else:
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = float(p[0])
                m.pose.position.y = float(p[1])
                m.pose.position.z = 0.1
                m.scale.x = 20.0
                m.scale.y = 20.0
                m.scale.z = 0.2
            out.markers.append(m)
        self._pub_for(
            f'/gcs/payload/{robot_name}/completed_zones', MarkerArray).publish(out)

    def _handle_confirmed_targets(self, robot_name, msg, i, now):
        """JSON-encoded list of confirmed-target AABBs from raven_nav. Each
        becomes a translucent box (or wireframe) so the operator sees the
        spatial extent of every found house/tower/etc. Status drives color:
        green=visited, yellow=observing, gray=other.

        Z note: gossip passes String payloads through untransformed (see
        gossip_node._transform_to_global — only PointCloud2 and MarkerArray
        get rewritten to global ENU). So the JSON's cx,cy,cz are still in
        the sender's local 'map' frame, where z=0 is the drone's odom
        origin (≈ ground). The GCS display map shares that z=0 ground
        reference, so we use cz directly — adding _display_z_offset would
        sink the boxes below ground when terrain MSL > 90 m (Lisbon, etc).
        """
        try:
            items = json.loads(msg.data) if msg.data else []
        except (ValueError, AttributeError):
            return
        if not isinstance(items, list):
            return
        out = MarkerArray()
        for j, it in enumerate(items):
            try:
                cx = float(it.get('cx', 0.0))
                cy = float(it.get('cy', 0.0))
                cz = float(it.get('cz', 0.0))
                sx = float(it.get('sx', 1.0))
                sy = float(it.get('sy', 1.0))
                sz = float(it.get('sz', 1.0))
            except (TypeError, ValueError):
                continue
            status = str(it.get('status', '')).lower()
            label = str(it.get('label', '?'))
            if status == 'visited':
                rgb = (0.1, 0.85, 0.2)
            elif status == 'observing':
                rgb = (0.95, 0.8, 0.15)
            else:
                rgb = (0.65, 0.65, 0.65)
            # Translucent solid box for the AABB extent.
            box = Marker()
            box.header.frame_id = 'map'
            box.header.stamp = now
            box.ns = f'{robot_name}_confirmed_targets'
            box.id = i * 100000 + j * 2
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = cx
            box.pose.position.y = cy
            box.pose.position.z = cz
            box.pose.orientation.w = 1.0
            box.scale.x = max(sx, 0.1)
            box.scale.y = max(sy, 0.1)
            box.scale.z = max(sz, 0.1)
            box.color.r = rgb[0]
            box.color.g = rgb[1]
            box.color.b = rgb[2]
            box.color.a = 0.25
            box.lifetime = Duration(sec=3, nanosec=0)
            out.markers.append(box)
            # Label above the box.
            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp = now
            txt.ns = f'{robot_name}_confirmed_targets_labels'
            txt.id = i * 100000 + j * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = cx
            txt.pose.position.y = cy
            txt.pose.position.z = cz + max(sz, 0.1) / 2.0 + 1.0
            txt.pose.orientation.w = 1.0
            txt.scale.z = 1.2
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = f'{label} [{status or "?"}]'
            txt.lifetime = Duration(sec=3, nanosec=0)
            out.markers.append(txt)
        self._pub_for(
            f'/gcs/payload/{robot_name}/confirmed_targets', MarkerArray).publish(out)

    def _handle_rgb_voxels(self, robot_name, msg, i, now):
        size = 0.5
        cubes = point_cloud2_to_cube_marker(
            msg, 0.0, 0.0, self._display_z_offset(),
            ns=f'{robot_name}_voxel_rgb',
            marker_id=i * 100000,
            stamp=now,
            lifetime=Duration(sec=2, nanosec=0),
            scale=size,
        )
        out = MarkerArray()
        if cubes is not None:
            out.markers.append(cubes)
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            edges = self._build_cube_edges(
                cubes.points, size,
                ns=f'{robot_name}_voxel_rgb_edges',
                marker_id=i * 100000 + 1,
                stamp=now,
                color=color,
                lifetime=Duration(sec=2, nanosec=0),
            )
            out.markers.append(edges)
        self._pub_for(f'/gcs/payload/{robot_name}/voxel_rgb', MarkerArray).publish(out)

    @staticmethod
    def _build_cube_edges(centers, size, ns, marker_id, stamp, color, lifetime):
        from geometry_msgs.msg import Point as GPoint
        from std_msgs.msg import ColorRGBA
        h = size / 2.0 * 1.03
        corners = [
            (-h, -h, -h), ( h, -h, -h), ( h,  h, -h), (-h,  h, -h),
            (-h, -h,  h), ( h, -h,  h), ( h,  h,  h), (-h,  h,  h),
        ]
        edge_idx = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.04
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        m.lifetime = lifetime
        pts = []
        for c in centers:
            cx, cy, cz = c.x, c.y, c.z
            for a, b in edge_idx:
                ax, ay, az = corners[a]
                bx, by, bz = corners[b]
                pts.append(GPoint(x=cx + ax, y=cy + ay, z=cz + az))
                pts.append(GPoint(x=cx + bx, y=cy + by, z=cz + bz))
        m.points = pts
        return m

    PAYLOAD_HANDLERS = {
        'filtered_rays':              ('visualization_msgs/msg/MarkerArray', _handle_filtered_rays),
        'raw_frontiers':              ('sensor_msgs/msg/PointCloud2',        _handle_raw_frontiers),
        'kept_frontiers':             ('sensor_msgs/msg/PointCloud2',        _handle_kept_frontiers),
        'completed_frontier_zones':   ('sensor_msgs/msg/PointCloud2',        _handle_completed_zones),
        'voxel_rgb':                  ('sensor_msgs/msg/PointCloud2',        _handle_rgb_voxels),
        'navigation_mode':            ('std_msgs/msg/String',                _handle_navigation_mode),
        'confirmed_targets':          ('std_msgs/msg/String',                _handle_confirmed_targets),
    }


def main(args=None):
    rclpy.init(args=args)
    node = PayloadVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
