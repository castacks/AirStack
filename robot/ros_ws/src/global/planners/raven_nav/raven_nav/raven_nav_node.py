import json
import os
import re
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from coordination_bringup.frame_utils import gps_to_enu
from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from airstack_msgs.msg import BidVector

from raven_nav.behavior_manager import BehaviorManager
from raven_nav.peer_state import PeerState
from raven_nav import bid_manager
from raven_nav.ray_groups import compute_ray_groups


GOSSIP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Latched: late-joining raven_nav still gets the most recent search_area.
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_NAV_MODE_TAG = {
    'Frontier-based': 'frontier',
    'Ray-based':      'ray',
    'Voxel-based':    'voxel',
}


class RavenNavNode(Node):
    def __init__(self):
        super().__init__('raven_nav')

        self._robot_name = os.getenv('ROBOT_NAME', 'robot')
        self._prefix = f'/{self._robot_name}'
        # rayfronts topics are under /robot_${ROS_DOMAIN_ID}/rayfronts/...
        ros_domain = os.getenv('ROS_DOMAIN_ID', '0')
        self._rf_prefix = f'/robot_{ros_domain}/rayfronts/msg_serv'

        # Numeric id for the tiebreak; lower id wins contested frontiers.
        try:
            self._my_id = int(ros_domain)
        except (TypeError, ValueError):
            m = re.search(r'(\d+)$', self._robot_name)
            self._my_id = int(m.group(1)) if m else 0

        # Ray/voxel data from rayfronts (converted to FLU)
        self._ray_origins = None
        self._ray_dirs = None
        self._ray_scores = None
        self._vox_xyz = None
        self._vox_scores = None

        self._frontiers = None
        self._cur_pose = None

        self._waypoint_locked = False
        self._target_waypoint = None
        self._target_waypoint2 = None
        self._behavior_mode = 'Frontier-based'
        self._last_completed = []

        # Set on first GPS fix.
        self._boot_enu: 'np.ndarray | None' = None
        self._alt_ground: 'float | None' = None
        self._peer_state = PeerState()

        # Polygon constraint in local 'map' frame. None = unconstrained.
        self._search_area_xy: 'np.ndarray | None' = None
        self._warned_polygon_degenerate = False

        # Coverage tracking: XY centers (Nx2) of frontier zones this robot
        # has cleared. A zone is added when the drone reaches within
        # _zone_visit_radius_m of a frontier waypoint while in frontier mode.
        # Gossiped to peers so the whole fleet stops chasing nearby frontiers.
        self._completed_zones: 'np.ndarray' = np.zeros((0, 2), dtype=np.float64)
        self._zone_visit_radius_m: float = float(self.declare_parameter(
            'zone_visit_radius_m', 5.0).value)
        # If a new candidate zone is within this of an existing zone, dedupe.
        self._zone_dedupe_radius_m: float = float(self.declare_parameter(
            'zone_dedupe_radius_m', 5.0).value)

        self._score_threshold = self.declare_parameter('score_threshold', 0.68).value
        query_labels_param = self.declare_parameter(
            'query_labels', ['red building', 'water tower', 'radio tower']).value
        self._query_labels = list(query_labels_param)
        # Subset of query_labels to navigate toward; empty = all.
        target_labels_param = self.declare_parameter('target_labels', ['']).value
        target_labels = [t for t in target_labels_param if t]
        self._target_objects = target_labels if target_labels else self._query_labels[:]

        self._min_altitude = self.declare_parameter('min_altitude_agl', 1.5).value
        self._max_altitude = self.declare_parameter('max_altitude_agl', 100.0).value
        self._voxel_score_threshold = float(self.declare_parameter(
            'voxel_score_threshold', 0.7).value)
        self._voxel_min_cluster_size = int(self.declare_parameter(
            'voxel_min_cluster_size', 30).value)

        timer_period = self.declare_parameter('timer_period', 0.5).value
        # Coordination debug lines are tagged "[coord]".
        self._debug_coord = self.declare_parameter('debug_coordination', True).value
        # Pretty-printed ray score + ray group tables to the logger.
        self._debug_ray_table = self.declare_parameter('debug_ray_table', True).value
        self._debug_table_max_rows = int(self.declare_parameter(
            'debug_table_max_rows', 30).value)
        self._debug_table_period = float(self.declare_parameter(
            'debug_table_period_sec', 5.0).value)
        # Labels parsed from rayfronts q{N}_<label> topics — authoritative
        # column order. Filled lazily on first ray callback.
        self._detected_query_labels: 'list[str] | None' = None

        self._path_pub = self.create_publisher(
            Path, f'{self._prefix}/global_plan', 10)
        self._filtered_rays_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/filtered_rays', 10)
        self._viewpoint_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/frontier_viewpoints', 10)
        # Shared with peers via gossip; receivers apply their own filters.
        self._raw_frontiers_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/raw_frontiers', 10)
        self._current_target_pub = self.create_publisher(
            String, f'{self._prefix}/current_target', 10)
        self._voxel_bbox_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/voxel_clusters', 10)
        self._completed_targets_pub = self.create_publisher(
            String, f'{self._prefix}/completed_targets', 10)
        self._committed_target_pub = self.create_publisher(
            String, f'{self._prefix}/committed_target', 10)
        self._nav_mode_pub = self.create_publisher(
            String, f'{self._prefix}/navigation_mode', 10)
        self._my_bids_pub = self.create_publisher(
            BidVector, f'{self._prefix}/bids', 10)
        # Pretty-printed debug tables. Use the view_ray_tables script to render.
        self._ray_table_pub = self.create_publisher(
            String, f'{self._prefix}/debug/ray_table', 10)
        self._groups_table_pub = self.create_publisher(
            String, f'{self._prefix}/debug/groups_table', 10)
        self._bids_table_pub = self.create_publisher(
            String, f'{self._prefix}/debug/bids_table', 10)
        self._voxel_table_pub = self.create_publisher(
            String, f'{self._prefix}/debug/voxel_table', 10)
        self._frontier_table_pub = self.create_publisher(
            String, f'{self._prefix}/debug/frontier_table', 10)
        self._shared_rays_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/shared_rays', 10)
        self._ray_groups_viz_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/ray_groups_viz', 10)
        # Coverage tracking: XY centers of frontier zones this robot has cleared.
        # Shared via gossip so peers stop chasing the same areas.
        self._completed_zones_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/raven_nav/completed_frontier_zones', 10)
        # Confirmed-target AABBs fed from voxel_behavior. JSON; gossip-shared
        # so peers can anchor ray triangulation onto these and dedupe.
        self._confirmed_targets_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/confirmed_targets', 10)
        # Per-robot merged discoveries list (local consumption by
        # semantic_search_task and GCS visualizer).
        self._discoveries_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/discoveries', 10)
        self._prev_ray_group_marker_count = 0
        self._assigned_target: 'str | None' = None
        self._committed_to_assigned = False
        self._solo_ticks = 0
        # Running-max bid was a single-target stability hack — dropped now
        # that bids are per-RayGroup and the auction is triangulation-aware.
        # Per-tick per-group bids drive the auction; stability comes from the
        # commit state machine instead.
        # Cached direction/origin (in local map frame) of the last ray group
        # we observed for the currently committed target. Used by frontier
        # behavior to bias toward the last-seen direction when ray-mode loses
        # sight of the target temporarily.
        self._committed_target_last_dir: 'np.ndarray | None' = None
        self._committed_target_last_origin: 'np.ndarray | None' = None
        # Drone is "committed" once it reaches within this distance of its
        # first ray waypoint. Until then, it broadcasts its raw distance bid
        # so a closer peer can still take over.
        self._commit_radius_m = self.declare_parameter(
            'commit_radius_m', 3.0).value

        self._publisher_dict = {
            'path': self._path_pub,
            'filtered_rays': self._filtered_rays_pub,
            'viewpoint': self._viewpoint_pub,
            'raw_frontiers': self._raw_frontiers_pub,
            'current_target': self._current_target_pub,
            'voxel_bbox': self._voxel_bbox_pub,
            'frontier_table': self._frontier_table_pub,
        }

        self._behavior_manager = BehaviorManager(
            get_clock=self.get_clock,
            publisher_dict=self._publisher_dict,
            score_threshold=self._score_threshold,
            min_altitude=self._min_altitude,
            max_altitude=self._max_altitude,
            voxel_score_threshold=self._voxel_score_threshold,
            voxel_min_cluster_size=self._voxel_min_cluster_size,
        )

        # rayfronts publishes _sim/all topics only when something subscribes.
        self.create_subscription(
            PointCloud2,
            f'{self._rf_prefix}/rays_sim/all',
            self._ray_all_cb, 10)
        self.create_subscription(
            PointCloud2,
            f'{self._rf_prefix}/voxels_sim/all',
            self._vox_all_cb, 10)

        self.create_subscription(
            PointCloud2,
            f'{self._rf_prefix}/frontiers',
            self._frontiers_cb, 10)
        self.create_subscription(
            Odometry,
            f'{self._prefix}/odometry',
            self._odometry_cb, 10)
        self.create_subscription(
            String,
            '/input_prompt',
            self._input_prompt_cb, 10)

        # mavros publishes raw/fix as BEST_EFFORT; matching it here is required.
        navsat_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            NavSatFix,
            f'{self._prefix}/interface/mavros/global_position/raw/fix',
            self._navsat_cb, navsat_qos)
        self.create_subscription(
            PeerProfileMsg, '/gossip/peers',
            self._on_peer_profile, GOSSIP_QOS)
        self.create_subscription(
            PolygonStamped,
            f'{self._prefix}/raven_nav/search_area',
            self._search_area_cb, LATCHED_QOS)

        self.create_timer(timer_period, self._timer_cb)

        self.get_logger().info(
            f'raven_nav started | robot={self._robot_name} (id={self._my_id}) | '
            f'timer={timer_period:.2f}s | '
            f'query_labels={self._query_labels} | '
            f'score_threshold={self._score_threshold} | '
            f'altitude=[{self._min_altitude}, {self._max_altitude}]')

    def _detect_rayfronts_labels(self) -> 'list[str] | None':
        """Parse q{N}_<label> topics under {rf_prefix}/rays_sim/ to derive the
        authoritative column ordering. Returns labels in q-index order, or None
        if no q-topics are visible (rayfronts may not be up yet)."""
        prefix = f'{self._rf_prefix}/rays_sim/'
        pat = re.compile(rf'^{re.escape(prefix)}q(\d+)_(.+)$')
        parsed = []
        for name, _ in self.get_topic_names_and_types():
            m = pat.match(name)
            if m:
                # q-topic uses underscores; query_labels param uses spaces.
                parsed.append((int(m.group(1)), m.group(2).replace('_', ' ')))
        if not parsed:
            return None
        parsed.sort()
        return [lbl for _, lbl in parsed]

    def _ray_all_cb(self, msg: PointCloud2):
        """Ray PointCloud2 fields: x,y,z,theta,phi,sim_0,sim_1,..."""
        Q = len(self._query_labels)
        if Q == 0:
            return
        msg_field_names = [f.name for f in msg.fields]
        sim_fields = sorted([f for f in msg_field_names if f.startswith('sim_')],
                            key=lambda s: int(s.split('_', 1)[1]))
        # Detect canonical column labels once. Warn loudly if the configured
        # query_labels don't match — the column→label mapping would be wrong.
        if self._detected_query_labels is None:
            detected = self._detect_rayfronts_labels()
            if detected is not None:
                self._detected_query_labels = detected
                if detected != self._query_labels:
                    self.get_logger().warn(
                        f'[ray_table] query_labels MISMATCH — using rayfronts '
                        f'topic order as truth.\n'
                        f'  configured: {self._query_labels}\n'
                        f'  rayfronts:  {detected}')
        if not sim_fields:
            self._ray_origins = None
            self._ray_dirs = None
            self._ray_scores = None
            return
        fields = ('x', 'y', 'z', 'theta', 'phi') + tuple(sim_fields)
        pts = list(point_cloud2.read_points(msg, field_names=fields, skip_nans=True))
        if not pts:
            self._ray_origins = None
            self._ray_dirs = None
            self._ray_scores = None
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        rdf_orig = arr[:, :3]
        theta = np.deg2rad(arr[:, 3])
        phi = np.deg2rad(arr[:, 4])
        sim_all = arr[:, 5:]
        dx = np.cos(theta) * np.sin(phi)
        dy = np.sin(theta) * np.sin(phi)
        dz = np.cos(phi)
        rdf_dirs = np.stack([dx, dy, dz], axis=1)
        # RDF → FLU
        flu_orig = np.stack([rdf_orig[:, 2], -rdf_orig[:, 0], -rdf_orig[:, 1]], axis=1)
        flu_dirs = np.stack([rdf_dirs[:, 2], -rdf_dirs[:, 0], -rdf_dirs[:, 1]], axis=1)
        self._ray_origins = flu_orig
        self._ray_dirs = flu_dirs
        self._ray_scores = sim_all
        self._publish_shared_rays(flu_orig, flu_dirs, sim_all, sim_fields, msg.header.stamp)

    def _publish_shared_rays(self, origins, dirs, scores, sim_field_names, stamp):
        """Republish rays in FLU/local frame for gossip distribution.

        Pre-filtered to only rays where some target column passes threshold —
        peers re-filter locally but starting from a smaller cloud cuts gossip
        bandwidth and the merged-ray count significantly.

        Fields: x, y, z, dx, dy, dz, sim_0, sim_1, ...
        gossip_node translates only x,y,z; dx,dy,dz pass through unchanged.
        """
        from sensor_msgs.msg import PointField
        if self._target_objects:
            label_indices = [self._query_labels.index(t)
                             for t in self._target_objects
                             if t in self._query_labels]
            if label_indices:
                relevant = scores[:, label_indices]
                keep = (relevant > self._score_threshold).any(axis=1)
                origins = origins[keep]
                dirs = dirs[keep]
                scores = scores[keep]
        n = len(origins)
        if n == 0:
            return
        K = scores.shape[1]
        out = PointCloud2()
        out.header.stamp = stamp
        out.header.frame_id = 'map'
        out.height = 1
        out.width = n
        out.is_bigendian = False
        out.is_dense = True
        names = ['x', 'y', 'z', 'dx', 'dy', 'dz'] + list(sim_field_names)
        out.fields = [
            PointField(name=nm, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, nm in enumerate(names)
        ]
        out.point_step = 4 * len(names)
        out.row_step = out.point_step * n
        flat = np.empty((n, len(names)), dtype=np.float32)
        flat[:, 0:3] = origins.astype(np.float32)
        flat[:, 3:6] = dirs.astype(np.float32)
        flat[:, 6:6 + K] = scores.astype(np.float32)
        out.data = flat.tobytes()
        self._shared_rays_pub.publish(out)

    def _add_completed_zone(self, xy: np.ndarray) -> None:
        """Record an XY center as a 'visited' frontier zone.

        Deduplicates: if the new point is within zone_dedupe_radius_m of any
        existing zone, skip — the zone-coverage filter treats the whole
        ZONE_RADIUS_M neighborhood as resolved already.
        """
        xy = np.asarray(xy, dtype=np.float64)[:2]
        if self._completed_zones.shape[0] > 0:
            d = np.linalg.norm(self._completed_zones - xy[None, :], axis=1)
            if float(d.min()) < self._zone_dedupe_radius_m:
                return
        self._completed_zones = np.vstack([self._completed_zones, xy[None, :]])

    def _publish_completed_zones(self) -> None:
        """Gossip our completed-zone bitmap as a PointCloud2 (x,y only).

        Peers union this with their own zones via peer_completed_zones in
        the receiver-side filter and novelty cost.
        """
        from sensor_msgs.msg import PointField
        n = int(self._completed_zones.shape[0])
        out = PointCloud2()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.height = 1
        out.width = n
        out.is_bigendian = False
        out.is_dense = True
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        ]
        out.point_step = 8
        out.row_step = out.point_step * n
        if n > 0:
            flat = self._completed_zones.astype(np.float32)
            out.data = flat.tobytes()
        self._completed_zones_pub.publish(out)

    def _vox_all_cb(self, msg: PointCloud2):
        """Voxel PointCloud2 fields: x,y,z,sim_0,sim_1,..."""
        Q = len(self._query_labels)
        if Q == 0:
            return
        msg_field_names = [f.name for f in msg.fields]
        sim_fields = sorted([f for f in msg_field_names if f.startswith('sim_')],
                            key=lambda s: int(s.split('_', 1)[1]))
        if not sim_fields:
            self._vox_xyz = None
            self._vox_scores = None
            return
        fields = ('x', 'y', 'z') + tuple(sim_fields)
        pts = list(point_cloud2.read_points(msg, field_names=fields, skip_nans=True))
        if not pts:
            self._vox_xyz = None
            self._vox_scores = None
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        rdf_xyz = arr[:, :3]
        sim_all = arr[:, 3:]
        # RDF → FLU
        flu_xyz = np.stack([rdf_xyz[:, 2], -rdf_xyz[:, 0], -rdf_xyz[:, 1]], axis=1)
        self._vox_xyz = flu_xyz
        self._vox_scores = sim_all

    def _frontiers_cb(self, msg: PointCloud2):
        pts = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                            skip_nans=True))
        self._frontiers = (
            np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
            if pts else None)

    def _odometry_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._cur_pose = np.array([p.x, p.y, p.z], dtype=np.float64)

    def _input_prompt_cb(self, msg: String):
        targets = [t.strip() for t in msg.data.split(',') if t.strip()]
        if targets:
            self._target_objects = targets
            if targets != self._query_labels:
                # Column count changed; clear cached arrays + visited state.
                self._query_labels = targets[:]
                self._vox_xyz = None
                self._vox_scores = None
                self._ray_origins = None
                self._ray_dirs = None
                self._ray_scores = None
                self._behavior_manager.voxel_behavior.reset()
                self._last_completed = []
            self.get_logger().info(f'target objects updated: {self._target_objects}')

    def _navsat_cb(self, msg: NavSatFix):
        """Capture boot ENU + ground altitude on first valid fix."""
        if self._boot_enu is not None:
            return
        if msg.status.status < 0:
            return
        self._alt_ground = msg.altitude
        self._boot_enu = np.array(
            gps_to_enu(msg.latitude, msg.longitude, msg.altitude),
            dtype=np.float64,
        )
        self.get_logger().info(
            f'boot GPS captured: alt_ground={self._alt_ground:.2f}m, '
            f'boot_enu=({self._boot_enu[0]:.2f}, {self._boot_enu[1]:.2f}, '
            f'{self._boot_enu[2]:.2f})')

    def _on_peer_profile(self, msg: PeerProfileMsg):
        # Skip own profile (gossip publishes self too) and pre-boot messages.
        if msg.robot_name == self._robot_name:
            return
        if self._boot_enu is None:
            if self._debug_coord:
                self.get_logger().info(
                    f'[coord] dropped peer profile from {msg.robot_name}: '
                    'own boot GPS not received yet',
                    throttle_duration_sec=5.0)
            return
        new_peer = msg.robot_name not in self._peer_state.peer_last_seen
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._peer_state.update(msg, self._boot_enu, self._alt_ground, now_sec)
        if new_peer and self._debug_coord:
            wp = self._peer_state.peer_waypoints.get(msg.robot_name)
            pos = self._peer_state.peer_positions.get(msg.robot_name)
            wp_s = (f'wp=({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})'
                    if wp is not None else 'wp=none')
            pos_s = (f'pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})'
                     if pos is not None else 'pos=none')
            self.get_logger().info(
                f'[coord] peer FIRST SEEN: {msg.robot_name} '
                f'(id={self._peer_state.peer_ids.get(msg.robot_name)}) {pos_s} {wp_s}')

    def _search_area_cb(self, msg: PolygonStamped):
        """Empty / <3 vertex polygon clears the constraint."""
        pts = msg.polygon.points
        if len(pts) < 3:
            self._search_area_xy = None
            if not self._warned_polygon_degenerate and len(pts) > 0:
                self.get_logger().warn(
                    f'search_area has {len(pts)} vertex/vertices (<3); '
                    'treating as unconstrained.')
                self._warned_polygon_degenerate = True
            return
        self._warned_polygon_degenerate = False
        self._search_area_xy = np.array(
            [[p.x, p.y] for p in pts], dtype=np.float64)
        self.get_logger().info(
            f'search_area updated: {self._search_area_xy.shape[0]} vertices.')

    def _merge_own_and_peer_rays(self):
        """Stack own + peer rays. Peer rays are already in local frame.

        Returns (origins, dirs, scores) or (None, None, None) if nothing exists.
        Peer rays whose score column count differs from ours are dropped — column
        ordering is only consistent across robots when query_labels match.
        """
        own_o = self._ray_origins
        own_d = self._ray_dirs
        own_s = self._ray_scores
        K = own_s.shape[1] if own_s is not None else None
        chunks_o, chunks_d, chunks_s = [], [], []
        if own_o is not None and len(own_o) > 0:
            chunks_o.append(own_o)
            chunks_d.append(own_d)
            chunks_s.append(own_s)
        for name, pr in self._peer_state.peer_rays.items():
            if pr.scores.size == 0:
                continue
            if K is None:
                K = pr.scores.shape[1]
            if pr.scores.shape[1] != K:
                continue
            chunks_o.append(pr.origins.astype(np.float32))
            chunks_d.append(pr.dirs.astype(np.float32))
            chunks_s.append(pr.scores.astype(np.float32))
        if not chunks_o:
            return None, None, None
        return (np.concatenate(chunks_o, axis=0),
                np.concatenate(chunks_d, axis=0),
                np.concatenate(chunks_s, axis=0))

    _RAY_GROUP_COLORS = [
        (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
        (1.0, 1.0, 0.0), (0.0, 1.0, 1.0), (1.0, 0.0, 1.0),
        (1.0, 0.5, 0.0), (0.5, 0.0, 1.0), (0.0, 0.5, 0.5),
        (0.5, 0.5, 0.5),
    ]

    def _publish_ray_groups_viz(self, groups):
        """Emit one ARROW per ray + one TEXT label per group, colored by group index."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        marker_id = 0
        for i, g in enumerate(groups):
            r, gr, b = self._RAY_GROUP_COLORS[i % len(self._RAY_GROUP_COLORS)]
            for k in range(g.num_rays):
                p0 = g.ray_origins[k]
                d = g.ray_dirs[k]
                d = d / (np.linalg.norm(d) + 1e-6)
                p1 = p0 + 2.0 * d
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = now
                arrow.ns = 'ray_groups'
                arrow.id = marker_id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.points = [
                    Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
                    Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])),
                ]
                arrow.scale.x = 0.3
                arrow.scale.y = 0.6
                arrow.scale.z = 0.4
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, gr, b, 0.7
                ma.markers.append(arrow)
                marker_id += 1
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'ray_groups_labels'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(g.avg_origin[0])
            text.pose.position.y = float(g.avg_origin[1])
            text.pose.position.z = float(g.avg_origin[2] + 1.5)
            text.pose.orientation.w = 1.0
            text.scale.z = 1.0
            text.color.r, text.color.g, text.color.b, text.color.a = r, gr, b, 1.0
            text.text = f'{g.label} (n={g.num_rays}, d={g.min_dist_to_robot:.1f}m)'
            ma.markers.append(text)
            marker_id += 1
        # Delete leftover markers from a previous (longer) frame.
        for j in range(marker_id, self._prev_ray_group_marker_count):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'ray_groups' if (j % 2 == 0) else 'ray_groups_labels'
            m.id = j
            m.action = Marker.DELETE
            ma.markers.append(m)
        self._prev_ray_group_marker_count = marker_id
        self._ray_groups_viz_pub.publish(ma)

    def _column_labels(self) -> 'list[str]':
        """Authoritative per-column label list: detected rayfronts labels if
        present, otherwise the configured query_labels. Padded with col_N if
        sim_K column count exceeds the label list."""
        return self._detected_query_labels or self._query_labels

    def _debug_print_ray_table(self):
        """Per-tick (throttled) softmax score table.
        Rows = own rays. Columns = sim_0..sim_(Q-1) in rayfronts order.
        Last two columns: argmax label + max score."""
        scores = self._ray_scores
        if scores is None or len(scores) == 0:
            self._ray_table_pub.publish(String(data='(no rays)'))
            return
        labels = self._column_labels()
        K = scores.shape[1]
        col_labels = [labels[i] if i < len(labels) else f'col_{i}' for i in range(K)]
        target_set = set(self._target_objects or [])
        # Mark target columns with [T], background with [B]; column header
        # itself includes the marker so it's obvious in the printed table.
        col_kinds = ['T' if lbl in target_set else 'B' for lbl in col_labels]
        col_headers = [f'[{k}]{lbl}' for k, lbl in zip(col_kinds, col_labels)]
        col_w = max(10, max(len(s) for s in col_headers))
        target_idxs = [j for j, k in enumerate(col_kinds) if k == 'T']
        n_show = min(len(scores), self._debug_table_max_rows)

        header = (['ray#'.rjust(5)]
                  + [s.rjust(col_w) for s in col_headers]
                  + ['argmax'.rjust(col_w + 3), 'max'.rjust(7), 'pass'.rjust(5)])
        lines = ['  '.join(header)]
        for i in range(n_show):
            row = scores[i]
            ax = int(np.argmax(row))
            ax_label = f'[{col_kinds[ax]}]{col_labels[ax]}'
            mx = float(row[ax])
            # New filter (matches ray_groups.py): argmax must land on a target
            # column AND that score must exceed threshold.
            passes = bool(target_idxs
                          and ax in target_idxs
                          and mx > self._score_threshold)
            cells = ([str(i).rjust(5)]
                     + [f'{v:.3f}'.rjust(col_w) for v in row]
                     + [ax_label.rjust(col_w + 3), f'{mx:.3f}'.rjust(7),
                        ('Y' if passes else 'n').rjust(5)])
            lines.append('  '.join(cells))
        if len(scores) > n_show:
            lines.append(f'  ... ({len(scores) - n_show} more rays, '
                         f'increase debug_table_max_rows to see all)')
        n_pass = int(sum(
            1 for i in range(len(scores))
            if int(np.argmax(scores[i])) in target_idxs
            and scores[i, int(np.argmax(scores[i]))] > self._score_threshold))
        lines.append(
            f'  threshold={self._score_threshold} | '
            f'targets={[col_labels[j] for j in target_idxs]} | '
            f'background={[col_labels[j] for j in range(K) if j not in target_idxs]} | '
            f'passing={n_pass}/{len(scores)}')
        # Per-target max ray: best evidence ray for each target column,
        # whether or not it actually passes the filter. Useful to see "is
        # this target even visible?" when filter rejects everything.
        max_lines = ['', 'per-target max ray:']
        origins = self._ray_origins
        dirs = self._ray_dirs
        if not target_idxs:
            max_lines.append('  (no targets configured)')
        else:
            tlabel_w = max(len(col_labels[j]) for j in target_idxs)
            for j in target_idxs:
                col = scores[:, j]
                k = int(np.argmax(col))
                row = scores[k]
                ax = int(np.argmax(row))
                ax_label = f'[{col_kinds[ax]}]{col_labels[ax]}'
                target_label = f'[T]{col_labels[j]}'
                pieces = [
                    f'  {target_label.ljust(tlabel_w + 3)}',
                    f'score={float(col[k]):.3f}',
                    f'argmax={ax_label}',
                ]
                if origins is not None and k < len(origins):
                    o = origins[k]
                    pieces.append(
                        f'origin=({o[0]:7.2f},{o[1]:7.2f},{o[2]:7.2f})')
                if dirs is not None and k < len(dirs):
                    d = dirs[k]
                    pieces.append(
                        f'dir=({d[0]:6.2f},{d[1]:6.2f},{d[2]:6.2f})')
                max_lines.append('  '.join(pieces))
        body = (f'{len(scores)} own rays\n'
                + '\n'.join(lines)
                + '\n' + '\n'.join(max_lines))
        self._ray_table_pub.publish(String(data=body))

    def _debug_print_groups_table(self, groups):
        """Filtered ray groups summary.
        Columns: group#, num_rays, avg_score, max_score, min_score,
                 label_idx (in rayfronts column order), label, min_dist."""
        labels = self._column_labels()
        if not groups:
            self._groups_table_pub.publish(String(data='(no groups passed filter)'))
            return
        header = ['group#', 'num_rays', 'avg', 'max', 'min', 'col_idx', 'label', 'min_dist']
        widths = [6, 8, 6, 6, 6, 7, 18, 8]
        lines = ['  '.join(h.rjust(w) for h, w in zip(header, widths))]
        for i, g in enumerate(groups):
            min_s = float(g.ray_scores.min()) if len(g.ray_scores) > 0 else 0.0
            try:
                col_idx = labels.index(g.label)
            except ValueError:
                col_idx = -1
            cells = [str(i), str(g.num_rays),
                     f'{g.avg_score:.3f}', f'{g.max_score:.3f}', f'{min_s:.3f}',
                     str(col_idx), g.label, f'{g.min_dist_to_robot:.2f}']
            lines.append('  '.join(c.rjust(w) for c, w in zip(cells, widths)))
        body = (f'{len(groups)} groups (threshold={self._score_threshold})\n'
                + '\n'.join(lines))
        self._groups_table_pub.publish(String(data=body))

    def _debug_print_voxel_table(self):
        """Per-target voxel summary so the user can tune voxel-mode thresholds.

        Columns: target | total | >0.5 | >0.7 | >0.85 | >0.95 | largest_cc
        ('largest_cc' = size of biggest connected high-conf component using
        the current voxel-mode threshold of 0.85.)"""
        vox_xyz = self._vox_xyz
        vox_scores = self._vox_scores
        if vox_xyz is None or vox_scores is None or len(vox_xyz) == 0:
            self._voxel_table_pub.publish(String(data='(no voxels yet)'))
            return
        labels = self._column_labels()
        K = vox_scores.shape[1]
        col_labels = [labels[i] if i < len(labels) else f'col_{i}' for i in range(K)]
        target_set = set(self._target_objects or [])
        target_idxs = [j for j, lbl in enumerate(col_labels) if lbl in target_set]
        if not target_idxs:
            self._voxel_table_pub.publish(String(data='(no targets configured)'))
            return

        n_total = int(len(vox_xyz))
        thresholds = [0.3, 0.5, 0.7, 0.85]
        # Mirror the live voxel-mode params so cc counts reflect what mode
        # selection is actually doing this tick.
        cc_threshold = self._voxel_score_threshold
        min_samples_default = self._voxel_min_cluster_size

        header_cells = (['target'.ljust(18), 'total'.rjust(7)]
                        + [f'>{t}'.rjust(7) for t in thresholds]
                        + ['cc_max'.rjust(7),
                           f'cc_n>={min_samples_default}'.rjust(9)])
        lines = ['  '.join(header_cells)]

        # Connected-component count per target above cc_threshold (mirrors
        # voxel_behavior.condition_check). Only do this when there's data.
        try:
            import scipy.ndimage  # noqa: F401
            have_scipy = True
        except ImportError:
            have_scipy = False

        for j in target_idxs:
            col = vox_scores[:, j]
            counts = [int((col > t).sum()) for t in thresholds]
            cc_max = 0
            cc_n_big = 0
            mask = col > cc_threshold
            if have_scipy and mask.any():
                import scipy.ndimage as sn
                pts = vox_xyz[mask]
                vox_size = 0.5
                pts_round = np.round(pts, 3)
                min_c = pts_round.min(axis=0)
                norm = ((pts_round - min_c) / vox_size).astype(int)
                max_c = norm.max(axis=0) + 1
                occ = np.zeros(tuple(max_c.tolist()), dtype=np.uint8)
                for x, y, z in norm:
                    occ[x, y, z] = 1
                lab, ncomp = sn.label(occ, structure=np.ones((3, 3, 3), dtype=np.uint8))
                comp_sizes = []
                for c in range(1, ncomp + 1):
                    comp_sizes.append(int((lab == c).sum()))
                if comp_sizes:
                    cc_max = max(comp_sizes)
                    cc_n_big = sum(1 for s in comp_sizes if s >= min_samples_default)
            cells = ([col_labels[j].ljust(18), str(n_total).rjust(7)]
                     + [str(c).rjust(7) for c in counts]
                     + [str(cc_max).rjust(7), str(cc_n_big).rjust(9)])
            lines.append('  '.join(cells))
        lines.append(
            f'  cc_threshold={cc_threshold}  min_cluster_size={min_samples_default}'
            f'  → voxel-mode fires when any target has '
            f'cc_n>={min_samples_default} ≥ 1'
        )
        body = f'{n_total} voxels  |  targets={[col_labels[j] for j in target_idxs]}\n' + '\n'.join(lines)
        self._voxel_table_pub.publish(String(data=body))

    def _timer_cb(self):
        if self._cur_pose is None:
            self.get_logger().warn('waiting for odometry...', throttle_duration_sec=5.0)
            self._nav_mode_pub.publish(String(data='idle'))
            return

        # Merge own rays with peer rays so the auction sees the same ray pool
        # all robots see. Peer rays already converted to local frame in PeerState.
        merged_origins, merged_dirs, merged_scores = self._merge_own_and_peer_rays()

        # Compute ray groups once: shared between the bid auction and
        # ray_behavior waypoint selection.
        ray_groups = compute_ray_groups(
            merged_origins, merged_dirs, merged_scores,
            self._query_labels, self._target_objects,
            self._score_threshold, self._cur_pose,
            min_altitude=self._min_altitude,
            max_altitude=self._max_altitude)
        self._behavior_manager.ray_behavior.ray_groups = ray_groups
        self._publish_ray_groups_viz(ray_groups)

        if self._debug_ray_table:
            self._debug_print_ray_table()
            self._debug_print_groups_table(ray_groups)
            self._debug_print_voxel_table()

        # New auction protocol — always broadcast best-seen raw bid;
        # commitment lives in a separate /committed_target string:
        #   1. See target  → record best bid (running max). Provisionally claim.
        #      Broadcast bid every tick (NOT per-tick distance).
        #   2. Receive competing peer raw bid (peer not yet committed):
        #        peer better → drop the claim
        #        peer worse  → commit (broadcast committed_target=label)
        #   3. Once committed: keep broadcasting raw bid + committed_target.
        #      Released by (a) target completion (own or any peer), or
        #      (b) double-commit collision: another peer also committed to
        #      this label AND has a better raw bid (closer) → I yield.
        # Per-group bids: one BidEntry per RayGroup. Triangulation in
        # bid_manager.assign() decides which (own, peer) pairs are actually
        # competing for the same target — different-target bids don't contest.
        per_tick_bids = bid_manager.compute_my_bids(ray_groups)

        # Aggregate completed: own + every peer we've heard from. Once a target
        # is in here it's permanently retired from this robot's claim pool.
        own_completed = set(self._behavior_manager.completed_queries)
        all_completed = set(own_completed)
        for peer_done in self._peer_state.peer_completed.values():
            all_completed |= peer_done

        # Drop bids for completed labels.
        my_bid_entries = [b for b in per_tick_bids
                          if b.label not in all_completed]

        # Same filter on peers — exclude bids whose label is in all_completed.
        peer_bid_entries = {
            name: [e for e in entries if e.label not in all_completed]
            for name, entries in self._peer_state.peer_bids.items()
        }

        # Drop the current commitment if its label was completed by anyone.
        if (self._assigned_target is not None
                and self._assigned_target in all_completed):
            if self._debug_coord:
                self.get_logger().info(
                    f'[coord] dropping {self._assigned_target}: '
                    f'completed by self or peer')
            self._assigned_target = None
            self._committed_to_assigned = False
            self._committed_target_last_dir = None
            self._committed_target_last_origin = None

        peer_commits = {
            n: t for n, t in self._peer_state.peer_committed_target.items() if t
        }
        peer_committed_labels = set(peer_commits.values())

        # Run the triangulation-aware auction. Returns (label, BidEntry) or
        # None. The returned BidEntry is the local group we won (so we can
        # cache its origin/direction for frontier fallback).
        polygon_xy = (self._search_area_xy
                      if isinstance(self._search_area_xy, np.ndarray)
                      and len(self._search_area_xy) >= 3 else None)
        won = bid_manager.assign(
            my_id=self._my_id,
            my_bids=my_bid_entries,
            peer_bids=peer_bid_entries,
            peer_ids=self._peer_state.peer_ids,
            polygon_xy=polygon_xy,
        )

        if self._assigned_target is None and won is not None:
            label, entry = won
            self._assigned_target = label
            self._committed_to_assigned = False
            self._committed_target_last_dir = entry.avg_dir.copy()
            self._committed_target_last_origin = entry.avg_origin.copy()
            if self._debug_coord:
                self.get_logger().info(
                    f'[coord] provisional claim on {label} '
                    f'(my_bid={entry.value:.2f}, '
                    f'rays={entry.num_rays}) — broadcasting until peer '
                    f'competition arrives')

        # Re-check competition for an already-assigned-but-not-yet-committed
        # target. If after triangulation we no longer win it, drop.
        if (self._assigned_target is not None
                and not self._committed_to_assigned):
            if won is None or won[0] != self._assigned_target:
                if self._debug_coord:
                    self.get_logger().info(
                        f'[coord] DROPPING {self._assigned_target} — '
                        f'lost contested auction')
                self._assigned_target = None
                self._committed_target_last_dir = None
                self._committed_target_last_origin = None
            else:
                self._committed_to_assigned = True
                _, entry = won
                if self._debug_coord:
                    self.get_logger().info(
                        f'[coord] LOCKED {self._assigned_target} '
                        f'(my_bid={entry.value:.2f}, rays={entry.num_rays})')

        # Double-commit collision: if I'm committed AND a peer is also
        # committed to the same target, the FARTHER (lower-value) one yields.
        # With per-group bids the comparison restricts to peer entries with
        # matching label that triangulate to the same target as my best group.
        if self._assigned_target is not None and self._committed_to_assigned:
            my_best = max(
                (e for e in my_bid_entries if e.label == self._assigned_target),
                key=lambda e: e.value, default=None,
            )
            if my_best is not None:
                from raven_nav.ray_targets import is_same_target as _ist
                yielded = False
                for pname, plabel in peer_commits.items():
                    if plabel != self._assigned_target:
                        continue
                    p_entries = peer_bid_entries.get(pname, [])
                    pid = self._peer_state.peer_ids.get(pname)
                    if pid is None:
                        continue
                    for pe in p_entries:
                        if pe.label != self._assigned_target:
                            continue
                        same, _, _ = _ist(
                            bid_manager._GroupView(my_best),
                            bid_manager._GroupView(pe),
                            polygon_xy,
                        )
                        if not same:
                            continue
                        peer_wins = (pe.value > my_best.value
                                     or (pe.value == my_best.value
                                         and pid < self._my_id))
                        if peer_wins:
                            if self._debug_coord:
                                self.get_logger().info(
                                    f'[coord] double-commit on '
                                    f'{self._assigned_target}: peer {pname} '
                                    f'(id={pid}) bid {pe.value:.2f} >= mine '
                                    f'{my_best.value:.2f} — yielding')
                            self._assigned_target = None
                            self._committed_to_assigned = False
                            self._committed_target_last_dir = None
                            self._committed_target_last_origin = None
                            yielded = True
                            break
                    if yielded:
                        break

        # Cache last-seen direction/origin for the committed target.
        if self._assigned_target is not None and ray_groups:
            for g in ray_groups:
                if g.label == self._assigned_target and g.num_rays > 0:
                    self._committed_target_last_dir = np.asarray(
                        g.avg_dir, dtype=np.float64).copy()
                    self._committed_target_last_origin = np.asarray(
                        g.avg_origin, dtype=np.float64).copy()
                    break

        self._behavior_manager.ray_behavior.assigned_target = self._assigned_target

        # Publish per-group BidVector — one row per ray-group bid entry.
        bv = BidVector()
        bv.robot_name = self._robot_name
        bv.labels = [e.label for e in my_bid_entries]
        bv.values = [float(e.value) for e in my_bid_entries]
        bv.origin_x = [float(e.avg_origin[0]) for e in my_bid_entries]
        bv.origin_y = [float(e.avg_origin[1]) for e in my_bid_entries]
        bv.origin_z = [float(e.avg_origin[2]) for e in my_bid_entries]
        bv.dir_x = [float(e.avg_dir[0]) for e in my_bid_entries]
        bv.dir_y = [float(e.avg_dir[1]) for e in my_bid_entries]
        bv.dir_z = [float(e.avg_dir[2]) for e in my_bid_entries]
        bv.num_rays = [int(e.num_rays) for e in my_bid_entries]
        bv.avg_score = [float(e.avg_score) for e in my_bid_entries]
        self._my_bids_pub.publish(bv)
        # Broadcast our currently committed target so peers can resolve
        # double-commit collisions by distance comparison.
        committed_str = (self._assigned_target
                         if (self._assigned_target is not None
                             and self._committed_to_assigned)
                         else '')
        self._committed_target_pub.publish(String(data=committed_str))

        # Pretty-printed bids panel for the view_ray_tables script.
        own_done = sorted(self._behavior_manager.completed_queries)
        peer_done_lines = []
        for pname in sorted(self._peer_state.peer_completed.keys()):
            pdone = sorted(self._peer_state.peer_completed[pname])
            if pdone:
                peer_done_lines.append(f'  {pname}: {pdone}')
        bids_lines = [f'assigned: {self._assigned_target} '
                      f'(committed={self._committed_to_assigned})',
                      f'completed (own): {own_done if own_done else "none"}']
        if peer_done_lines:
            bids_lines.append('completed (peers):')
            bids_lines.extend(peer_done_lines)
        else:
            bids_lines.append('completed (peers): none')
        bids_lines.extend([
            '',
            f'my_bids ({self._robot_name}, id={self._my_id}):',
        ])
        if my_bid_entries:
            for e in sorted(my_bid_entries,
                            key=lambda x: (x.label, -x.value)):
                bids_lines.append(
                    f'  {e.label:<14}  {e.value:>10.3f} '
                    f'(rays={e.num_rays:<3} '
                    f'dir=({e.avg_dir[0]:+.2f},{e.avg_dir[1]:+.2f}))')
        else:
            bids_lines.append('  (none)')
        bids_lines.append('')
        bids_lines.append(f'peer_bids ({len(self._peer_state.peer_bids)} peers):')
        if self._peer_state.peer_bids:
            for peer_name in sorted(self._peer_state.peer_bids.keys()):
                pb = self._peer_state.peer_bids[peer_name]
                pid = self._peer_state.peer_ids.get(peer_name, '?')
                bids_lines.append(f'  {peer_name} (id={pid}):')
                if pb:
                    for e in sorted(pb, key=lambda x: (x.label, -x.value)):
                        bids_lines.append(
                            f'    {e.label:<14}  {e.value:>10.3f} '
                            f'(rays={e.num_rays:<3} '
                            f'dir=({e.avg_dir[0]:+.2f},{e.avg_dir[1]:+.2f}))')
                else:
                    bids_lines.append('    (empty)')
        else:
            bids_lines.append('  (none — no peers heard yet)')
        self._bids_table_pub.publish(String(data='\n'.join(bids_lines)))
        if self._debug_coord:
            def _fmt_bid_list(entries):
                if not entries:
                    return '[]'
                return '[' + ', '.join(f'{e.label}:{e.value:.1f}'
                                       for e in entries) + ']'

            peer_str = ', '.join(
                f'{n}={_fmt_bid_list(b)}'
                + (f' commit={peer_commits[n]}' if n in peer_commits else '')
                for n, b in self._peer_state.peer_bids.items()
            ) or '(none)'

            self.get_logger().info(
                f'[coord] mode={self._behavior_mode} '
                f'assigned={self._assigned_target} '
                f'committed={self._committed_to_assigned} | '
                f'my_bids={_fmt_bid_list(my_bid_entries)} | '
                f'peer_bids={peer_str}',
                throttle_duration_sec=2.0)

        prev_mode = self._behavior_mode
        # Voxel-mode (fine approach + 3m arrival detection) only fires on the
        # CURRENTLY ASSIGNED target — otherwise an incidental fly-by of any
        # cluster would mark unrelated targets complete and break the auction.
        voxel_targets = [self._assigned_target] if self._assigned_target else []
        self._behavior_manager.mode_select(
            query_labels=self._query_labels,
            target_objects=voxel_targets,
            vox_xyz=self._vox_xyz,
            vox_scores=self._vox_scores,
        )
        self._behavior_mode = self._behavior_manager.behavior_mode

        if self._behavior_mode != prev_mode:
            self.get_logger().info(f'behavior mode: {prev_mode} -> {self._behavior_mode}')
            self._waypoint_locked = False
            self._target_waypoint = None
            self._target_waypoint2 = None

        # Record a completed-frontier zone when we arrive at a frontier
        # waypoint. We only do this in Frontier-based mode — voxel/ray
        # arrivals are tracked separately (completed_queries / visited
        # clusters) and don't represent "this patch of the polygon is now
        # observed".
        if (self._behavior_mode == 'Frontier-based'
                and self._target_waypoint is not None
                and self._cur_pose is not None):
            dist_to_wp = float(np.linalg.norm(
                self._cur_pose[:2] - self._target_waypoint[:2]))
            if dist_to_wp < self._zone_visit_radius_m:
                self._add_completed_zone(self._target_waypoint[:2])

        # Build the union of completed zones (own + peers) in our local frame.
        zone_chunks = [self._completed_zones]
        for pz in self._peer_state.peer_completed_zones.values():
            if pz is not None and pz.shape[0] > 0:
                zone_chunks.append(pz)
        completed_zones_xy = np.vstack(zone_chunks) if zone_chunks else None

        # Gossip our own zones (PointCloud2 with x,y fields).
        self._publish_completed_zones()

        self._waypoint_locked, self._target_waypoint, self._target_waypoint2 = \
            self._behavior_manager.behavior_execute(
                behavior_mode=self._behavior_mode,
                frontiers=self._frontiers,
                cur_pose_np=self._cur_pose,
                waypoint_locked=self._waypoint_locked,
                target_waypoint=self._target_waypoint,
                target_waypoint2=self._target_waypoint2,
                publisher_dict=self._publisher_dict,
                vox_xyz=self._vox_xyz,
                vox_scores=self._vox_scores,
                query_labels=self._query_labels,
                peer_state=self._peer_state,
                completed_zones_xy=completed_zones_xy,
                my_id=self._my_id,
                search_area_xy=self._search_area_xy,
                debug_logger=(self.get_logger() if self._debug_coord else None),
                assigned_target=self._assigned_target,
                committed_target_dir=self._committed_target_last_dir,
                committed_target_origin=self._committed_target_last_origin,
            )

        self._nav_mode_pub.publish(
            String(data=_NAV_MODE_TAG.get(self._behavior_mode, 'idle')))

        completed = sorted(self._behavior_manager.completed_queries)
        # Publish every tick (not just on change) so peers and any restarted
        # gossip_node always see the latest list of completed targets.
        self._completed_targets_pub.publish(String(data=json.dumps(completed)))
        self._last_completed = completed

        # Compact mode-tagged status line. Verbose ray/voxel/wp counts are in
        # the debug topics now; this line drives action feedback so keep it
        # short.
        ray_beh = self._behavior_manager.ray_behavior
        parts = [f'[{self._behavior_mode}]']
        if ray_beh.current_target:
            parts.append(f'target={ray_beh.current_target}')
        if completed:
            parts.append(f'completed={completed}')
        self.get_logger().info(' '.join(parts), throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = RavenNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
