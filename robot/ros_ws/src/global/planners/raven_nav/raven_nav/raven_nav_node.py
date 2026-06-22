import json
import os
import re
from types import SimpleNamespace
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from coordination_bringup.frame_utils import gps_to_enu, dir_to_quat
from coordination_msgs.msg import PeerProfile as PeerProfileMsg
from coordination_msgs.msg import CoverageGrid
from airstack_msgs.msg import BidVector

from raven_nav.behavior_manager import BehaviorManager
from raven_nav.behaviors.frontier_behavior import _points_in_polygon
from raven_nav.peer_state import PeerState
from raven_nav import bid_manager
from raven_nav.ray_groups import RayGroup, compute_ray_groups, same_ray_group
from raven_nav.track_confirmation import TemporalConfirmer
from raven_nav.ray_targets import build_targets, ray_aabb_hits, is_same_target
from raven_nav.discoveries import (
    ConfirmedTarget,
    build_discoveries,
    confirmed_targets_to_json,
    discoveries_to_json,
    merge_confirmed_targets,
    merge_house_boxes,
    merge_similar_targets,
)


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
    'Complete':       'complete',
}


def _polygon_area_xy(poly_xy: np.ndarray) -> float:
    """Shoelace area of a 2D polygon. Returns 0 for degenerate input."""
    if poly_xy is None or poly_xy.shape[0] < 3:
        return 0.0
    x = poly_xy[:, 0]
    y = poly_xy[:, 1]
    return 0.5 * float(np.abs(
        np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


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
        self._cur_yaw: float = 0.0

        self._waypoint_locked = False
        self._target_waypoint = None
        self._target_waypoint2 = None
        self._behavior_mode = 'Frontier-based'
        self._last_completed = []

        # Set on first GPS fix.
        self._boot_enu: 'np.ndarray | None' = None
        self._alt_ground: 'float | None' = None
        self._peer_state = PeerState()

        # Mission metrics + per-target event log (for the result dump).
        self._mission_start_ts: 'float | None' = None
        self._completion_reason: str = ''
        self._path_length_m: float = 0.0
        self._prev_odom_xyz: 'np.ndarray | None' = None
        self._num_odom_samples: int = 0
        self._target_events: list = []
        self._result_written: bool = False

        # Polygon constraint in local 'map' frame. None = unconstrained.
        self._search_area_xy: 'np.ndarray | None' = None
        self._warned_polygon_degenerate = False

        # 2D observed-cells grid. Cell size matches rayfronts vox_size.
        self._cell_size_m: float = float(self.declare_parameter(
            'coverage_cell_size_m', 0.5).value)
        self._max_raycast_range_m: float = float(self.declare_parameter(
            'coverage_raycast_range_m', 30.0).value)
        self._observed_cells: set = set()

        # Sticky once tripped — never re-arms within a single run.
        self._coverage_threshold: float = float(self.declare_parameter(
            'coverage_complete_threshold', 0.90).value)
        self._search_complete: bool = False
        self._last_coverage_frac: float = 0.0
        # Highest 10% milestone already logged (0..10); -1 means none yet.
        self._coverage_milestone: int = -1

        # Rate-limit raycast stamping — it's O(frontiers × ray_length / cell)
        # and at hover repaints the same starburst every tick.
        self._raycast_min_step_m: float = float(self.declare_parameter(
            'coverage_raycast_min_step_m', 5.0).value)
        self._last_raycast_xy: 'np.ndarray | None' = None

        self._score_threshold = self.declare_parameter('score_threshold', 0.7).value
        query_labels_param = self.declare_parameter(
            'query_labels', ['red building', 'water tower', 'radio tower']).value
        self._query_labels = list(query_labels_param)
        # Subset of query_labels to navigate toward; empty = all.
        target_labels_param = self.declare_parameter('target_labels', ['']).value
        target_labels = [t for t in target_labels_param if t]
        self._target_objects = target_labels if target_labels else self._query_labels[:]

        self._min_altitude = self.declare_parameter('min_altitude_agl', 1.5).value
        self._max_altitude = self.declare_parameter('max_altitude_agl', 100.0).value
        # Ray/frontier prefer higher targets: reward per metre of altitude above
        # min_altitude when scoring frontier viewpoints and ray groups.
        self._altitude_pref_weight = float(self.declare_parameter(
            'altitude_preference_weight', 2.0).value)
        self._voxel_score_threshold = float(self.declare_parameter(
            'voxel_score_threshold', 0.85).value)
        self._voxel_min_cluster_size = int(self.declare_parameter(
            'voxel_min_cluster_size', 35).value)
        # Temporal confirmation: persist across N ticks before a detection counts.
        self._voxel_confirm_hits = int(self.declare_parameter(
            'voxel_confirm_hits', 3).value)
        self._voxel_track_max_misses = int(self.declare_parameter(
            'voxel_track_max_misses', 4).value)
        # Engage voxel-mode on a cluster within this range even without a ray.
        self._voxel_proximity_engage_m = float(self.declare_parameter(
            'voxel_proximity_engage_m', 12.0).value)
        # Drop confirmed AABBs below this confidence (semantic*persistence);
        # 0 = report all. Raise (~0.3) to cut flickery low-persistence FPs.
        self._voxel_min_confidence = float(self.declare_parameter(
            'voxel_min_confidence', 0.0).value)
        # 1 = no temporal accumulation (ray groups rarely false-positive).
        self._ray_confirm_hits = int(self.declare_parameter(
            'ray_confirm_hits', 1).value)
        self._ray_track_max_misses = int(self.declare_parameter(
            'ray_track_max_misses', 4).value)

        # Where to dump this robot's result JSON (AABBs + event log + path
        # length, in global ENU). '' = don't dump. The mission sets a shared
        # bind-mounted path so the compile step can read every robot's file.
        self._results_dir = str(self.declare_parameter(
            'results_dir', '').value)
        # Re-dump cadence (s) so a fresh file exists when the task is cancelled.
        self._results_dump_period_s = float(self.declare_parameter(
            'results_dump_period_s', 3.0).value)
        self._last_results_dump_ts: float = 0.0

        # Run the frontier-only baseline: navigation ignores all semantic
        # cues (no Ray-/Voxel-based pursuit, no committed-target bias);
        # tracking still runs.
        self._frontier_only_baseline = bool(self.declare_parameter(
            'frontier_only_baseline', False).value)
        if self._frontier_only_baseline:
            self.get_logger().info(
                'frontier_only_baseline=true — navigation is pure frontier '
                'exploration (semantic tracking still on)')

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
            PointCloud2, f'{self._prefix}/shared_frontiers', 10)
        # Post-filter own frontiers (altitude + polygon + zone). Visualizing
        # this in Foxglove shows the explorable area shrinking in real time.
        self._kept_frontiers_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/filtered_frontiers', 10)
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
        self._discoveries_table_pub = self.create_publisher(
            String, f'{self._prefix}/debug/discoveries_table', 10)
        self._shared_rays_pub = self.create_publisher(
            PointCloud2, f'{self._prefix}/shared_rays', 10)
        self._ray_groups_viz_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/ray_groups_viz', 10)
        # Coverage tracking: XY centers of frontier zones this robot has cleared.
        # Shared via gossip so peers stop chasing the same areas.
        self._completed_zones_pub = self.create_publisher(
            CoverageGrid, f'{self._prefix}/raven_nav/explored_area_coverage', 10)
        # Gossip-shared so peers can anchor ray triangulation onto these and dedupe.
        self._confirmed_targets_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/confirmed_targets', 10)
        self._discoveries_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/discoveries', 10)
        # Serviced ray-leads (reached + rays cohered) — gossiped so a completed
        # lead is dropped team-wide and the task table stays bounded.
        self._served_leads_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/served_leads', 10)
        # Auction table (JSON, global ENU) — the unified table (visited BBs +
        # available rays/BBs with state + assignment). Drives the terminal viewer
        # and is gossiped so GCS can build the combined overlay from it.
        self._auction_table_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/auction_table', 10)
        # Accumulated ray-lead memory (JSON, global ENU bearings) — gossiped so
        # the persistent lead set is shared across the team.
        self._ray_leads_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/ray_leads', 10)
        self._prev_ray_group_marker_count = 0
        self._assigned_target: 'str | None' = None
        self._committed_to_assigned = False
        self._solo_ticks = 0
        # Last ray group observed for the committed target, in local map frame.
        # Frontier behavior biases toward this direction when ray-mode loses sight.
        self._committed_target_last_dir: 'np.ndarray | None' = None
        self._committed_target_last_origin: 'np.ndarray | None' = None
        # Ray-mode hysteresis: hold the committed bearing unless a rival ray group
        # beats it by commit_swap_improvement_frac AND it's been held commit_min_hold_s.
        self._committed_bearing_lock_ts: 'float | None' = None
        self._commit_swap_frac = float(self.declare_parameter(
            'commit_swap_improvement_frac', 0.25).value)
        self._commit_min_hold_s = float(self.declare_parameter(
            'commit_min_hold_s', 4.0).value)
        # Within this distance of the first ray waypoint, the drone is "committed"
        # and stops broadcasting raw distance bids that would let peers take over.
        self._commit_radius_m = self.declare_parameter(
            'commit_radius_m', 3.0).value

        # A ray's target sits further out than its origin: scale ray bid distance
        # by this (scene-tunable) so rays don't masquerade as close vs BBs. This is
        # the sole knob favoring BBs over rays now.
        self._ray_reach_factor = float(self.declare_parameter(
            'ray_reach_factor', 3.0).value)
        # Gentle behind-only heading penalty for ray/BB target selection (front/
        # side free) — keeps it nearest-first without reversing for side targets.
        self._target_behind_penalty_weight = float(self.declare_parameter(
            'target_behind_penalty_weight', 8.0).value)
        # Assignment hysteresis: a different target must beat the one we're
        # committed to by this much (m-equivalent) for the auction to switch.
        self._commit_switch_margin_m = float(self.declare_parameter(
            'commit_switch_margin_m', 8.0).value)
        # Centre of the BB this drone is currently committed to (None = none / ray).
        self._committed_bb_center: 'np.ndarray | None' = None
        # Persistent visited-target BBs (own + peer) — accumulated, never TTL'd.
        self._peer_visited_bbs: list = []
        # Observing-but-not-visited boxes (own + peer), kept past live-track
        # age-out so consensus can route a drone back to finish them.
        self._observed_bbs: dict = {}
        # CBAA consensus over the unified task table (ray-leads + confirmed BBs).
        # Every robot solves the same shared task set + agent positions -> same
        # conflict-free assignment.
        self._consensus = bid_manager.ConsensusAssigner()
        # Ray-leads this robot has serviced (reached + rays cohered); persistent,
        # accumulated own + peer, so a completed lead isn't re-added (bounds table).
        self._served_leads: list = []   # [(label, point(3))] in local frame
        # Persistent ray-lead memory: bearings accumulate (deduped) and stay until
        # their BB is observed/visited or a drone services them, so the task table
        # expands as detections arrive instead of being a per-tick snapshot.
        self._ray_leads: list = []   # [{'o','d','label','ts'}] in local frame

        self._publisher_dict = {
            'path': self._path_pub,
            'filtered_rays': self._filtered_rays_pub,
            'viewpoint': self._viewpoint_pub,
            'raw_frontiers': self._raw_frontiers_pub,
            'kept_frontiers': self._kept_frontiers_pub,
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
            voxel_confirm_hits=self._voxel_confirm_hits,
            voxel_track_max_misses=self._voxel_track_max_misses,
            voxel_proximity_engage_m=self._voxel_proximity_engage_m,
            voxel_min_confidence=self._voxel_min_confidence,
            altitude_preference_weight=self._altitude_pref_weight,
        )

        # Temporal gate on ray bearings (disabled when ray_confirm_hits <= 1).
        self._ray_confirmer = (
            TemporalConfirmer(confirm_hits=self._ray_confirm_hits,
                              max_misses=self._ray_track_max_misses)
            if self._ray_confirm_hits > 1 else None)

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
        self.create_subscription(
            Empty,
            f'{self._prefix}/raven_nav/clear_blacklist',
            self._clear_blacklist_cb, 10)

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
        cached_n = (len(self._detected_query_labels)
                    if self._detected_query_labels is not None else 0)
        if cached_n != len(sim_fields):
            detected = self._detect_rayfronts_labels()
            if detected is not None and detected != self._detected_query_labels:
                prev = self._detected_query_labels
                self._detected_query_labels = detected
                self.get_logger().info(
                    f'[ray_table] column labels refreshed: {prev} -> {detected}')
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

    def _xy_to_cells(self, xys: np.ndarray) -> np.ndarray:
        """(N,2) float XYs -> (N,2) int32 cell indices."""
        if xys.size == 0:
            return np.zeros((0, 2), dtype=np.int32)
        return np.floor(np.asarray(xys, dtype=np.float64)
                        / self._cell_size_m).astype(np.int32)

    def _stamp_cells_xy(self, xys: np.ndarray) -> None:
        """Mark every cell touching any point in `xys` (FLU local frame)."""
        cells = self._xy_to_cells(xys)
        if cells.shape[0] == 0:
            return
        self._observed_cells.update(map(tuple, cells.tolist()))

    def _stamp_raycast_cells(self, origin_xy: np.ndarray,
                             targets_xy: np.ndarray) -> None:
        """Stamp every cell on the 2D line from origin toward each target,
        stopping `pullback` metres SHORT of the frontier endpoint so the
        frontier cell itself stays unobserved. Otherwise we'd mark the frontier
        as explored, drop it on the next tick's cell filter, and falsely report
        the polygon as cleared."""
        if targets_xy is None or targets_xy.shape[0] == 0:
            return
        origin = np.asarray(origin_xy, dtype=np.float64).reshape(2)
        tgt = np.asarray(targets_xy, dtype=np.float64).reshape(-1, 2)
        delta = tgt - origin[None, :]
        dist = np.linalg.norm(delta, axis=1)
        nonzero = dist > 1e-6
        if not np.any(nonzero):
            return
        delta = delta[nonzero]
        dist = dist[nonzero]
        # Pull back from the frontier point
        pullback = 5
        clamp = np.minimum(dist, self._max_raycast_range_m) - pullback
        keep = clamp > 0.0
        if not np.any(keep):
            return
        delta = delta[keep]
        dist = dist[keep]
        clamp = clamp[keep]
        unit = delta / dist[:, None]
        end = origin[None, :] + unit * clamp[:, None]
        # Half-cell step ensures diagonal lines don't skip cells.
        step = max(self._cell_size_m * 0.5, 0.05)
        n_steps = int(np.ceil(float(clamp.max()) / step)) + 1
        ts = np.linspace(0.0, 1.0, n_steps)[None, :, None]
        starts = origin[None, None, :]
        ends = end[:, None, :]
        pts = starts + (ends - starts) * ts
        cells = self._xy_to_cells(pts.reshape(-1, 2))
        cells_unique = np.unique(cells, axis=0)
        self._observed_cells.update(map(tuple, cells_unique.tolist()))

    def _own_cell_centers_xy(self) -> np.ndarray:
        """Observed cells as (M,2) float64 cell-center XYs."""
        if not self._observed_cells:
            return np.zeros((0, 2), dtype=np.float64)
        arr = np.array(list(self._observed_cells), dtype=np.float64)
        return (arr + 0.5) * self._cell_size_m

    def _coverage_fraction(self, polygon_xy: np.ndarray,
                           observed_centers_xy: np.ndarray) -> float:
        """Fraction of search polygon area covered by observed cells.
        `observed_centers_xy` should be the merged own+peer cell centers so
        peers' coverage counts toward the gate too."""
        if polygon_xy is None or polygon_xy.shape[0] < 3:
            return 0.0
        poly_area = _polygon_area_xy(polygon_xy)
        if poly_area <= 0.0 or observed_centers_xy is None \
                or observed_centers_xy.shape[0] == 0:
            return 0.0
        in_poly = _points_in_polygon(observed_centers_xy[:, :2], polygon_xy)
        # Cells coming from peers may use a different grid origin, so
        # quantize merged centers onto our grid before counting unique cells.
        cells = np.floor(observed_centers_xy[in_poly]
                         / self._cell_size_m).astype(np.int64)
        if cells.shape[0] == 0:
            return 0.0
        unique = np.unique(cells, axis=0)
        covered_area = unique.shape[0] * (self._cell_size_m ** 2)
        return float(min(covered_area / poly_area, 1.0))

    def _own_confirmed_targets(self) -> list:
        """ConfirmedTarget AABBs from voxel_behavior's live clusters, plus the
        persistent visited instances so peers keep excluding them; 'visited' once
        the drone has passed within range, else 'observing'."""
        vb = self._behavior_manager.voxel_behavior
        out: list = []
        now_ts = float(self.get_clock().now().nanoseconds) * 1e-9
        for cid, bb in vb.target_voxel_clusters.items():
            label = vb.cluster_query_map.get(cid, '')
            if not label:
                continue
            status = ('visited'
                      if vb.is_visited(np.array(bb[:3], dtype=float),
                                       np.array(bb[3:6], dtype=float), label)
                      else 'observing')
            out.append(ConfirmedTarget(
                label=label,
                center=np.array(bb[:3], dtype=float),
                size=np.array(bb[3:6], dtype=float),
                status=status,
                confidence=float(vb.cluster_confidence.get(cid, 1.0)),
                ts=now_ts,
            ))
        live_centers = [np.asarray(c.center, dtype=float) for c in out]
        for label, vcenter, vsize in vb.visited_instances:
            vc = np.asarray(vcenter, dtype=float)
            if any(float(np.linalg.norm(vc - lc)) <= 3.0 for lc in live_centers):
                continue
            out.append(ConfirmedTarget(
                label=label, center=vc, size=np.asarray(vsize, dtype=float),
                status='visited', confidence=1.0, ts=now_ts))
        return out

    def _obs_key(self, label, center):
        c = np.round(np.asarray(center, dtype=float)[:3] / 2.0).astype(int)
        return (label, int(c[0]), int(c[1]), int(c[2]))

    def _bb_is_visited(self, label, center) -> bool:
        c = np.asarray(center, dtype=float)[:3]
        for lab, ex in self._peer_visited_bbs:
            if lab == label and float(np.linalg.norm(
                    np.asarray(ex, dtype=float)[:3] - c)) <= self._VISITED_DEDUP_M:
                return True
        return False

    _VISITED_DEDUP_M = 5.0

    def _add_visited(self, label, bb) -> None:
        """Add a visited target to the persistent visited set, deduped by centre."""
        c = np.asarray(bb, dtype=float)[:3]
        for lab, ex in self._peer_visited_bbs:
            if lab == label and float(np.linalg.norm(
                    np.asarray(ex, dtype=float)[:3] - c)) <= self._VISITED_DEDUP_M:
                return
        self._peer_visited_bbs.append((label, np.asarray(bb, dtype=float)))

    def _house_boxes(self):
        """One fused box per physical target: union the observing boxes with the
        accumulated visited fragments (visited is sticky on AABB-union), so a
        house that's been reached is a single visited box — not a big observing
        box with small visited boxes inside it."""
        srcs = list(self._observed_bbs.values())
        for lab, bb in self._peer_visited_bbs:
            b = np.asarray(bb, dtype=float)
            srcs.append(ConfirmedTarget(label=lab, center=b[:3],
                                        size=(b[3:6] if b.size >= 6 else np.full(3, 1.0)),
                                        status='visited', confidence=1.0, ts=0.0))
        return merge_house_boxes(srcs)

    def _fresh_peers(self, now) -> set:
        """Peers heard from within the TTL (stale peers' state is ignored)."""
        return {n for n, t in self._peer_state.peer_last_seen.items()
                if now - t <= self._PEER_TTL_S}

    def _peer_confirmed_targets_flat(self) -> list:
        """Flatten fresh peers' peer_confirmed_targets into a ConfirmedTarget list."""
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = self._fresh_peers(now)
        out: list = []
        for name, entries in self._peer_state.peer_confirmed_targets.items():
            if name not in fresh:
                continue
            for pct in entries:
                out.append(ConfirmedTarget(
                    label=pct.label,
                    center=np.asarray(pct.center, dtype=float),
                    size=np.asarray(pct.size, dtype=float),
                    status=pct.status,
                    confidence=pct.confidence,
                    ts=pct.ts,
                ))
        return out

    def _update_discoveries(self, ray_groups) -> None:
        """Build RayTargets + ConfirmedTargets → Discoveries, publish 2 topics.

        Publishes:
          /<robot>/raven_nav/confirmed_targets   (JSON for gossip)
          /<robot>/raven_nav/discoveries         (JSON for semantic_search_task
                                                  + GCS viz)
        """
        now_ts = float(self.get_clock().now().nanoseconds) * 1e-9

        # Combine own fragments locally (peak-aware) BEFORE transmit, so each
        # robot ships clean per-target boxes; the cross-robot step then fuses only
        # SIMILAR boxes (same target) and leaves dissimilar ones as distinct.
        own_cts = merge_confirmed_targets(self._own_confirmed_targets())
        peer_cts = self._peer_confirmed_targets_flat()
        merged_cts = merge_similar_targets(own_cts + peer_cts)

        # Visited BBs (own + peer) gate ray exclusion / commitment release;
        # consensus (own+peer) is the double-commit same-target key.
        def _bb(ct):
            return np.concatenate([np.asarray(ct.center, dtype=float),
                                   np.asarray(ct.size, dtype=float)])
        # Visited is monotonic: accumulate so a completed target stays excluded
        # even after the peer that visited it goes silent (TTL) or dies.
        for ct in (own_cts + peer_cts):
            if str(ct.status).lower() == 'visited':
                self._add_visited(ct.label, _bb(ct))
        for ct in merged_cts:
            key = self._obs_key(ct.label, ct.center)
            if str(ct.status).lower() == 'visited':
                self._observed_bbs.pop(key, None)
            else:
                self._observed_bbs[key] = ct
        for key in list(self._observed_bbs):
            oct = self._observed_bbs[key]
            if self._bb_is_visited(oct.label, oct.center):
                self._observed_bbs.pop(key, None)
        self._consensus_bbs = [(ct.label, _bb(ct)) for ct in merged_cts]
        self._behavior_manager.voxel_behavior.peer_visited_bbs = [
            bb for _, bb in self._peer_visited_bbs]

        # Peer-OBSERVING BBs (label, bb, that peer's position) — lets a far
        # committed drone yield a target a closer peer is sitting on.
        fresh = self._fresh_peers(now_ts)
        self._peer_observing_bbs = []
        for name, entries in self._peer_state.peer_confirmed_targets.items():
            if name not in fresh:
                continue
            ppos = self._peer_state.peer_positions.get(name)
            for pct in entries:
                if str(pct.status).lower() == 'observing':
                    self._peer_observing_bbs.append((pct.label, _bb(pct), ppos))

        # Gossip OWN targets in global ENU (xy + boot via _local_to_world) so
        # peers and the GCS share one frame; own_cts stay local for the merge.
        if self._boot_enu is not None:
            own_world = [
                ConfirmedTarget(label=ct.label,
                                center=self._local_to_world(ct.center),
                                size=ct.size, status=ct.status,
                                confidence=ct.confidence, ts=ct.ts)
                for ct in own_cts
            ]
        else:
            own_world = own_cts
        self._confirmed_targets_pub.publish(
            String(data=confirmed_targets_to_json(own_world)))

        # Ray groups that pierce a known BB get absorbed onto that BB.
        known_bbs = []
        for i, ct in enumerate(merged_cts):
            bb = np.concatenate([ct.center, ct.size])
            known_bbs.append((i, ct.label, bb))

        polygon_xy = (self._search_area_xy
                      if isinstance(self._search_area_xy, np.ndarray)
                      and len(self._search_area_xy) >= 3 else None)

        ray_targets = build_targets(
            own_groups=ray_groups,
            peer_groups=[],
            known_bbs=known_bbs,
            polygon_xy=polygon_xy,
            now_ts=now_ts,
        )

        peer_names = list(self._peer_state.peer_confirmed_targets.keys())

        discoveries = build_discoveries(
            ray_targets=ray_targets,
            confirmed_targets=merged_cts,
            contributing_robot=self._robot_name,
            peer_contributions=peer_names,
            now_ts=now_ts,
        )

        self._discoveries_pub.publish(
            String(data=discoveries_to_json(discoveries)))

        self._update_target_events(discoveries)

        self._publish_discoveries_table(discoveries)

    # BB/ray bearing are both estimates; pad so a ray aimed at a short target
    # doesn't graze, while staying a real 3D test (overhead rays still miss).
    _PEER_BID_PAD_M = 3.0
    # Drop a peer's commits/bids/BBs if not heard from within this long (sim s):
    # a crashed/landed peer must not block targets forever.
    _PEER_TTL_S = 8.0

    def _ray_on_peer_bbs(self, origin, direction, label, bbs) -> bool:
        """True if the ray passes through a label-compatible BB. Padded 3D
        ray-AABB so a bearing aimed at a short ground target doesn't graze."""
        if not bbs:
            return False
        o = np.asarray(origin, dtype=float)
        d = np.asarray(direction, dtype=float)
        bl = (label or '').lower()
        for lab, bb in bbs:
            ll = lab.lower()
            if not (bl in ll or ll in bl):
                continue
            padded = np.asarray(bb, dtype=float).copy()
            padded[3:6] = padded[3:6] + 2.0 * self._PEER_BID_PAD_M
            hit, _t = ray_aabb_hits(o, d, padded, t_min_ahead=0.0)
            if hit:
                return True
        return False

    # Project a bearing this far to compare target positions, and the proximity
    # that counts as the same physical target. _TASK_MATCH_M is the single radius
    # shared by the consensus task clustering and same-target deconfliction.
    _RAY_PROJECT_M = 20.0
    _TASK_MATCH_M = 10.0
    _PEER_TARGET_MATCH_M = _TASK_MATCH_M
    _TASK_KEY_GRID = 6.0
    # Persistent ray-lead memory: dedup radius + bearing-angle (matches the same
    # filtered ray across ticks; does NOT regroup — compute_ray_groups already
    # clustered at 45deg), and BB pad for the "points at a known BB" test.
    _RAY_LEAD_MATCH_M = 3.0
    _RAY_LEAD_DIR_COS = float(np.cos(np.deg2rad(12.0)))
    _RAY_LEAD_BB_PAD_M = 4.0

    def _lead_points_at_known_bb(self, o, d, label) -> bool:
        """The ray (o, d) pierces a same-label BB that's already observed
        (_observed_bbs) or visited (_peer_visited_bbs) -> the lead is resolved."""
        o = np.asarray(o, dtype=float)
        d = np.asarray(d, dtype=float)
        bl = (label or '').lower()
        cand = []
        for ct in self._observed_bbs.values():
            cl = (ct.label or '').lower()
            if bl in cl or cl in bl:
                cand.append((np.asarray(ct.center, float), np.asarray(ct.size, float)))
        for lab, bb in self._peer_visited_bbs:
            cl = (lab or '').lower()
            if bl in cl or cl in bl:
                b = np.asarray(bb, float)
                cand.append((b[:3], b[3:6]))
        for c, s in cand:
            bbarr = np.concatenate([c, s + self._RAY_LEAD_BB_PAD_M])
            if ray_aabb_hits(o, d, bbarr, t_min_ahead=0.0)[0]:
                return True
        return False

    def _lead_match(self, o, d, label, leads):
        """A stored lead matching bearing (o, d): same label, origin within
        _RAY_LEAD_MATCH_M, direction within the angle."""
        o2 = np.asarray(o, float)[:2]
        d2 = np.asarray(d, float)[:2]
        nd = np.linalg.norm(d2)
        d2 = d2 / nd if nd > 1e-9 else d2
        for L in leads:
            if L['label'] != label:
                continue
            if np.linalg.norm(o2 - np.asarray(L['o'], float)[:2]) > self._RAY_LEAD_MATCH_M:
                continue
            ld = np.asarray(L['d'], float)[:2]
            ln = np.linalg.norm(ld)
            if ln > 1e-9 and float(np.dot(d2, ld / ln)) >= self._RAY_LEAD_DIR_COS:
                return L
        return None

    # A lead is "reached" once a drone has flown >=_RAY_CLEAR_AHEAD_M along its
    # bearing and is within _RAY_CLEAR_PERP_M of the line — it (and parallel
    # neighbours sharing that corridor) drop off the table.
    _RAY_CLEAR_AHEAD_M = 8.0
    _RAY_CLEAR_PERP_M = 2.5

    def _lead_reached(self, o, d, agent_pos) -> bool:
        o = np.asarray(o, dtype=float)
        d = np.asarray(d, dtype=float)
        nd = np.linalg.norm(d)
        if nd < 1e-9:
            return False
        d = d / nd
        for p in agent_pos.values():
            v = np.asarray(p, dtype=float)[:3] - o
            t = float(np.dot(v, d))
            if t < self._RAY_CLEAR_AHEAD_M:
                continue
            if float(np.linalg.norm(v - t * d)) <= self._RAY_CLEAR_PERP_M:
                return True
        return False

    def _is_same_committed_lead(self, origin, direction) -> bool:
        """True if the currently-committed bearing is the same lead (origin within
        _RAY_LEAD_MATCH_M and bearing within _RAY_LEAD_DIR_COS). When True we keep
        holding it instead of re-seeding, so the bearing-block hysteresis owns it."""
        co = self._committed_target_last_origin
        cd = self._committed_target_last_dir
        if co is None or cd is None:
            return False
        if float(np.linalg.norm(np.asarray(co, float)[:2]
                                - np.asarray(origin, float)[:2])) > self._RAY_LEAD_MATCH_M:
            return False
        a = np.asarray(cd, float)[:2]
        b = np.asarray(direction, float)[:2]
        na = float(np.linalg.norm(a))
        nb = float(np.linalg.norm(b))
        if na < 1e-6 or nb < 1e-6:
            return False
        return float(np.dot(a / na, b / nb)) >= self._RAY_LEAD_DIR_COS

    def _ensure_followable_lead(self, my_task) -> None:
        """Guarantee the assigned ray-lead is in ray_behavior's groups so the
        drone can navigate to it even when this tick's rays (or its own rayfronts)
        don't contain it — e.g. a persistent lead or a peer's lead. Without this a
        ray-assigned drone would drop to frontier."""
        o = np.asarray(self._committed_target_last_origin, dtype=float)
        d = np.asarray(self._committed_target_last_dir, dtype=float)
        nd = float(np.linalg.norm(d[:2]))
        if nd < 1e-6:
            return
        rg = self._behavior_manager.ray_behavior.ray_groups
        d2 = d[:2] / nd
        for g in rg:
            if g.label != my_task.label:
                continue
            gd = np.asarray(g.avg_dir, dtype=float)[:2]
            gn = float(np.linalg.norm(gd))
            if gn > 1e-6 and float(np.dot(d2, gd / gn)) >= self._RAY_LEAD_DIR_COS:
                return   # a real matching group already exists
        synth = RayGroup(label=my_task.label, ray_origins=o.reshape(1, 3),
                         ray_dirs=d.reshape(1, 3), ray_scores=np.ones((1, 1)))
        synth._finalize(self._cur_pose if self._cur_pose is not None else o)
        rg.append(synth)

    def _accumulate_ray_leads(self, ray_groups, all_completed, now, agent_pos) -> None:
        """Fold the current ray groups into the persistent lead memory (dedup by
        bearing), then prune leads whose BB is observed/visited, that a drone has
        reached, or whose label completed. No TTL — leads persist until resolved."""
        targets = set(self._target_objects or [])
        for g in ray_groups:
            if g.num_rays <= 0 or g.label in all_completed:
                continue
            if targets and g.label not in targets:
                continue
            o = np.asarray(g.avg_origin, dtype=float)
            d = np.asarray(g.avg_dir, dtype=float)
            if self._lead_points_at_known_bb(o, d, g.label):
                continue
            m = self._lead_match(o, d, g.label, self._ray_leads)
            if m is not None:
                m['o'], m['d'], m['ts'] = o, d, now
            else:
                self._ray_leads.append(
                    {'o': o, 'd': d, 'label': g.label, 'ts': now})
        self._ray_leads = [
            L for L in self._ray_leads
            if L['label'] not in all_completed
            and not self._lead_points_at_known_bb(L['o'], L['d'], L['label'])
            and not self._lead_reached(L['o'], L['d'], agent_pos)]

    _LEAD_SERVICE_RADIUS_M = 6.0
    _LEAD_ALIGN_COS = 0.82   # ~35 deg: a neighboring ray group "same direction"

    def _build_consensus_tasks(self, ray_groups, all_completed, polygon_xy, now_ts,
                               agent_pos):
        """Unified shared task list: confirmed BBs + accumulated ray-leads. Each
        filtered ray is its own lead (like a frontier, separated by query + angle)
        — no triangulation into waypoints. Leads come from persistent memory, so
        the table expands as detections arrive and prunes when a lead's BB is
        observed/visited or a drone reaches it."""
        targets = set(self._target_objects or [])
        known_bbs = []
        items = []
        # 1. One fused box per house; the still-OBSERVING ones are tasks. Visited
        # houses are excluded here (they show as one visited box in the table).
        houses = self._house_boxes()
        cts = sorted((h for h in houses if str(h.status).lower() != 'visited'),
                     key=lambda ct: (str(ct.label), float(ct.center[0]),
                                     float(ct.center[1]), float(ct.center[2])))
        for i, ct in enumerate(cts):
            if targets and ct.label not in targets:
                continue
            if ct.label in all_completed:
                continue
            c = np.asarray(ct.center, dtype=float)
            if self._bb_is_visited(ct.label, c):
                continue
            size = np.asarray(ct.size, dtype=float)
            known_bbs.append((i, ct.label, np.concatenate([c, size])))
            items.append((ct.label, c, size, 'bb-observing'))
        # 2. Accumulate current rays into persistent memory, then emit each lead
        # directly as a ray task (origin + bearing). No triangulation: it only
        # confirms rays against a BB, which the BB-prune already handles.
        self._accumulate_ray_leads(ray_groups, all_completed, now_ts, agent_pos)
        combined = list(self._ray_leads)
        for name, pleads in self._peer_state.peer_ray_leads.items():
            if now_ts - self._peer_state.peer_last_seen.get(name, 0.0) > self._PEER_TTL_S:
                continue
            for pl in pleads:
                if targets and pl['label'] not in targets:
                    continue
                if pl['label'] in all_completed:
                    continue
                if self._lead_points_at_known_bb(pl['o'], pl['d'], pl['label']):
                    continue
                if self._lead_reached(pl['o'], pl['d'], agent_pos):
                    continue
                if self._lead_match(pl['o'], pl['d'], pl['label'], combined) is None:
                    combined.append(pl)
        for L in combined:
            o = np.asarray(L['o'], dtype=float)
            d = np.asarray(L['d'], dtype=float)
            items.append((L['label'], o, np.zeros(3), 'ray', o, d))
        return bid_manager.build_tasks(items, self._TASK_MATCH_M, self._TASK_KEY_GRID)

    def _lead_served(self, label, point) -> bool:
        """True if this lead was already serviced (own or peer), within match."""
        p = np.asarray(point, dtype=float)
        bl = (label or '').lower()
        served = self._served_leads + [
            s for lst in self._peer_state.peer_served_leads.values() for s in lst]
        for lab, c in served:
            ll = (lab or '').lower()
            if (bl in ll or ll in bl) and float(np.linalg.norm(
                    p[:2] - np.asarray(c, float)[:2])) <= self._TASK_MATCH_M:
                return True
        return False

    def _add_served_lead(self, label, point) -> None:
        p = np.asarray(point, dtype=float)[:3]
        for lab, c in self._served_leads:
            if lab == label and float(np.linalg.norm(
                    p[:2] - np.asarray(c, float)[:2])) <= self._VISITED_DEDUP_M:
                return
        self._served_leads.append((label, p))

    def _mark_lead_serviced_if_reached(self, my_task, ray_groups) -> None:
        """Retire only a DEAD ray-lead: the drone reached it but no same-label ray
        still points there (nothing was actually there). A live lead — rays still
        present — is left alone; it becomes a BB and is handled by the normal
        visited path. Serviced leads are gossiped so peers drop them too."""
        if my_task is None or my_task.status not in ('ray', 'ray-localized'):
            return
        if self._cur_pose is None:
            return
        pt = np.asarray(my_task.centroid, dtype=float)
        if float(np.linalg.norm(self._cur_pose[:2] - pt[:2])) > self._LEAD_SERVICE_RADIUS_M:
            return
        bl = (my_task.label or '').lower()
        rays_present = False
        for g in ray_groups:
            gl = (g.label or '').lower()
            if not (bl in gl or gl in bl) or g.num_rays <= 0:
                continue
            if float(np.linalg.norm(
                    np.asarray(g.avg_origin, float)[:2] - pt[:2])) <= self._TASK_MATCH_M \
                    or self._points_near(pt, self._ray_point(g.avg_origin, g.avg_dir)):
                rays_present = True
                break
        if not rays_present:
            self._add_served_lead(my_task.label, pt)

    def _agent_positions(self, fresh_peers):
        """{agent_id: xyz} for me + fresh peers with a known position."""
        pos = {self._my_id: np.asarray(self._cur_pose, dtype=float)}
        for name in fresh_peers:
            pid = self._peer_state.peer_ids.get(name)
            p = self._peer_state.peer_positions.get(name)
            if pid is not None and p is not None:
                pos[pid] = np.asarray(p, dtype=float)
        return pos

    def _ray_point(self, origin, direction) -> np.ndarray:
        d = np.asarray(direction, dtype=float)
        n = float(np.linalg.norm(d))
        o = np.asarray(origin, dtype=float)
        return o + (d / n) * self._RAY_PROJECT_M if n > 1e-6 else o

    def _points_near(self, a, b) -> bool:
        return float(np.linalg.norm(
            np.asarray(a, float)[:2] - np.asarray(b, float)[:2])) \
            <= self._PEER_TARGET_MATCH_M

    def _point_near_bbs(self, pt, label, bbs) -> bool:
        if not bbs:
            return False
        bl = (label or '').lower()
        for lab, bb in bbs:
            ll = lab.lower()
            if (bl in ll or ll in bl) and self._points_near(
                    pt, np.asarray(bb, float)[:3]):
                return True
        return False

    # A peer observing a target must be at least this much closer to it than us
    # before we yield our committed ray (margin avoids flip-flop near-equidistant).
    _OBSERVER_TAKEOVER_MARGIN_M = 5.0

    def _ray_yields_to_observer(self, origin, direction, label) -> bool:
        """True if my ray points at a BB a peer is OBSERVING and that peer is
        clearly closer to it — yield so the closer (voxel-mode) drone takes it.
        Applies even to my own committed ray."""
        bbs = getattr(self, '_peer_observing_bbs', None)
        if not bbs or self._cur_pose is None:
            return False
        o = np.asarray(origin, dtype=float)
        d = np.asarray(direction, dtype=float)
        my_pt = self._ray_point(origin, direction)
        bl = (label or '').lower()
        for lab, bb, ppos in bbs:
            if ppos is None:
                continue
            ll = lab.lower()
            if not (bl in ll or ll in bl):
                continue
            center = np.asarray(bb, dtype=float)[:3]
            padded = np.asarray(bb, dtype=float).copy()
            padded[3:6] = padded[3:6] + 2.0 * self._PEER_BID_PAD_M
            if not (ray_aabb_hits(o, d, padded, t_min_ahead=0.0)[0]
                    or self._points_near(my_pt, center)):
                continue
            peer_d = float(np.linalg.norm(np.asarray(ppos, float)[:2] - center[:2]))
            my_d = float(np.linalg.norm(self._cur_pose[:2] - center[:2]))
            if peer_d + self._OBSERVER_TAKEOVER_MARGIN_M < my_d:
                return True
        return False

    def _ray_off_limits(self, g, polygon_xy, peer_committed) -> bool:
        """Off-limits if the ray points at a target already visited (any drone),
        one a peer is committed to / heading toward, or one a peer is observing
        from clearly closer — geometric same-instance OR forward-point proximity
        (robust to triangulation flicker). Our own committed instance is never
        off-limits to us unless a closer peer is observing it."""
        if self._ray_yields_to_observer(g.avg_origin, g.avg_dir, g.label):
            return True
        my_pt = self._ray_point(g.avg_origin, g.avg_dir)
        visited = getattr(self, '_peer_visited_bbs', None)
        if self._ray_on_peer_bbs(g.avg_origin, g.avg_dir, g.label, visited) \
                or self._point_near_bbs(my_pt, g.label, visited):
            return True
        if (self._committed_to_assigned
                and self._assigned_target == g.label
                and self._committed_target_last_dir is not None
                and (self._same_instance(
                        g.avg_origin, g.avg_dir,
                        self._committed_target_last_origin,
                        self._committed_target_last_dir, g.label, polygon_xy)
                     or self._points_near(my_pt, self._ray_point(
                        self._committed_target_last_origin,
                        self._committed_target_last_dir)))):
            return False
        for (_n, pl, po, pd, _pid) in peer_committed:
            if pl != g.label:
                continue
            if self._same_instance(g.avg_origin, g.avg_dir, po, pd,
                                   g.label, polygon_xy) \
                    or self._points_near(my_pt, self._ray_point(po, pd)):
                return True
        return False

    def _rays_share_bb(self, o1, d1, o2, d2, label) -> bool:
        """True if a consensus BB (label-compatible) that BOTH rays pass through
        exists — a same-target signal independent of triangulation."""
        bbs = getattr(self, '_consensus_bbs', None)
        if not bbs:
            return False
        o1 = np.asarray(o1, float); d1 = np.asarray(d1, float)
        o2 = np.asarray(o2, float); d2 = np.asarray(d2, float)
        bl = (label or '').lower()
        for lab, bb in bbs:
            ll = lab.lower()
            if not (bl in ll or ll in bl):
                continue
            padded = np.asarray(bb, float).copy()
            padded[3:6] = padded[3:6] + 2.0 * self._PEER_BID_PAD_M
            if (ray_aabb_hits(o1, d1, padded, t_min_ahead=0.0)[0]
                    and ray_aabb_hits(o2, d2, padded, t_min_ahead=0.0)[0]):
                return True
        return False

    def _same_instance(self, o1, d1, o2, d2, label, polygon_xy) -> bool:
        """Two bearings point at the same physical target: triangulation OR a
        shared consensus BB (the latter catches wide-baseline triangulation
        misses once a BB exists)."""
        a = SimpleNamespace(label=label, avg_origin=np.asarray(o1, float),
                            avg_dir=np.asarray(d1, float))
        b = SimpleNamespace(label=label, avg_origin=np.asarray(o2, float),
                            avg_dir=np.asarray(d2, float))
        same, _, _ = is_same_target(a, b, polygon_xy)
        return bool(same) or self._rays_share_bb(o1, d1, o2, d2, label)

    def _ray_pin_cost(self, g, prev_o, prev_d) -> float:
        """Cost (lower=better) for following a ray as the committed instance:
        weighted toward the same direction as the previous ray and an origin
        moving along it; falls back to closeness with no prior bearing."""
        cost = float(g.avg_dist_to_robot)
        cost -= self._altitude_pref_weight * max(
            float(g.avg_origin[2]) - self._min_altitude, 0.0)
        if prev_o is None or prev_d is None:
            return cost
        pd = np.asarray(prev_d, float); pd = pd / (np.linalg.norm(pd) + 1e-6)
        gd = np.asarray(g.avg_dir, float); gd = gd / (np.linalg.norm(gd) + 1e-6)
        cost += 20.0 * (1.0 - float(np.dot(gd, pd)))
        rel = np.asarray(g.avg_origin, float) - np.asarray(prev_o, float)
        rn = float(np.linalg.norm(rel))
        if rn > 1.0:
            cost += 20.0 * (1.0 - float(np.dot(rel / rn, pd)))
        return cost

    def _match_committed_group(self, cands):
        """Candidate ray group matching the currently-held committed bearing
        (same_ray_group), or None."""
        if (self._committed_target_last_dir is None
                or self._committed_target_last_origin is None):
            return None
        prev = SimpleNamespace(
            label=self._assigned_target,
            avg_dir=np.asarray(self._committed_target_last_dir, dtype=float),
            avg_origin=np.asarray(self._committed_target_last_origin, dtype=float))
        for g in cands:
            if same_ray_group(prev, g):
                return g
        return None

    def _fresh_peer_committed_instances(self, now):
        """[(name, label, origin, dir, pid)] of peers whose committed instance is
        fresh (last seen within the TTL)."""
        out = []
        for name, inst in self._peer_state.peer_committed_instance.items():
            if now - self._peer_state.peer_last_seen.get(name, 0.0) > self._PEER_TTL_S:
                continue
            pid = self._peer_state.peer_ids.get(name)
            if pid is None:
                continue
            lbl, o, d = inst
            out.append((name, lbl, o, d, pid))
        return out

    # Match radius for treating a discovery as an already-logged instance, so a
    # drifting centroid (which flips _stable_id) isn't logged twice.
    _EVENT_MATCH_DIST_M = 8.0

    def _match_event(self, label: str, pos_enu: np.ndarray):
        """Existing event dict for this instance (same label, nearest centroid
        within _EVENT_MATCH_DIST_M), or None."""
        best = None
        best_d = self._EVENT_MATCH_DIST_M
        for ev in self._target_events:
            if ev['label'] != label:
                continue
            d = float(np.linalg.norm(np.asarray(ev['pos_enu']) - pos_enu))
            if d <= best_d:
                best_d = d
                best = ev
        return best

    def _update_target_events(self, discoveries) -> None:
        """Record first discovery/confirmation/visit time per target, once each.

        Milestones: discovered = any Discovery (ray bearing or AABB); confirmed
        = first AABB (either status); visited = first AABB with status
        'visited' (never set in the frontier baseline)."""
        if self._boot_enu is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        targets = set(self._target_objects or [])
        for d in discoveries:
            if targets and d.label not in targets:
                continue
            pos_enu = self._local_to_world(d.position)
            has_aabb = d.size is not None
            is_visited = has_aabb and str(d.status).lower() == 'visited'
            ev = self._match_event(d.label, pos_enu)
            if ev is None:
                ev = {
                    'label': d.label,
                    'instance_id': d.instance_id,
                    'pos_enu': pos_enu.tolist(),
                    'first_discovered_ts': now,
                    'first_confirmed_ts': None,
                    'first_visited_ts': None,
                }
                self._target_events.append(ev)
                self.get_logger().info(
                    f'[event] DISCOVERED {d.label} '
                    f'@ENU=({pos_enu[0]:.1f},{pos_enu[1]:.1f},{pos_enu[2]:.1f}) '
                    f't={now:.2f}')
            else:
                # Keep the freshest centroid (AABB centre beats a ray bearing).
                if has_aabb:
                    ev['pos_enu'] = pos_enu.tolist()
                    ev['instance_id'] = d.instance_id
            if has_aabb and ev['first_confirmed_ts'] is None:
                ev['first_confirmed_ts'] = now
                self.get_logger().info(
                    f'[event] CONFIRMED {d.label} (AABB, status={d.status}) '
                    f'@ENU=({pos_enu[0]:.1f},{pos_enu[1]:.1f},{pos_enu[2]:.1f}) '
                    f't={now:.2f}')
            if is_visited and ev['first_visited_ts'] is None:
                ev['first_visited_ts'] = now
                self.get_logger().info(
                    f'[event] VISITED {d.label} '
                    f'@ENU=({pos_enu[0]:.1f},{pos_enu[1]:.1f},{pos_enu[2]:.1f}) '
                    f't={now:.2f}')

    def _maybe_dump_results(self, force: bool = False) -> None:
        """Write the result JSON, throttled to the dump period unless forced."""
        if not self._results_dir:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not force:
            if self._results_dump_period_s <= 0.0:
                return
            if (now - self._last_results_dump_ts) < self._results_dump_period_s:
                return
        self._last_results_dump_ts = now
        self._write_results()

    def _local_to_world(self, p) -> np.ndarray:
        """Local 'map' point → annotation/world frame: add boot ENU in X/Y, but
        keep Z as AGL (ground-relative). boot_enu[2] = alt_ground - origin_alt
        reflects the scene-vs-datum altitude gap, which the ground-relative GT
        annotations don't have — adding it would lift detections ~tens of m."""
        b = self._boot_enu
        p = np.asarray(p, dtype=float)
        return np.array([p[0] + b[0], p[1] + b[1], p[2]], dtype=float)

    def _write_results(self) -> None:
        """Serialize own AABBs (world frame) + event log + path length."""
        if not self._results_dir or self._boot_enu is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        targets = []
        for ct in self._own_confirmed_targets():
            center_enu = self._local_to_world(ct.center).tolist()
            targets.append({
                'label': ct.label,
                'center_enu': center_enu,
                'size': np.asarray(ct.size, dtype=float).tolist(),
                'status': ct.status,
                'confidence': float(ct.confidence),
            })
        start = self._mission_start_ts
        events = []
        for ev in self._target_events:
            def _rel(t):
                return (t - start) if (t is not None and start is not None) else None
            events.append({
                'label': ev['label'],
                'instance_id': ev['instance_id'],
                'pos_enu': ev['pos_enu'],
                'first_discovered_ts': ev['first_discovered_ts'],
                'first_confirmed_ts': ev['first_confirmed_ts'],
                'first_visited_ts': ev['first_visited_ts'],
                'first_discovered_rel_s': _rel(ev['first_discovered_ts']),
                'first_confirmed_rel_s': _rel(ev['first_confirmed_ts']),
                'first_visited_rel_s': _rel(ev['first_visited_ts']),
            })
        out = {
            'robot': self._robot_name,
            'boot_enu': self._boot_enu.tolist(),
            'alt_ground': self._alt_ground,
            'mission_start_ts': start,
            'dump_ts': now,
            'mission_duration_s': (now - start) if start is not None else None,
            'completion_reason': self._completion_reason or 'in_progress',
            'coverage_fraction': self._last_coverage_frac,
            'coverage_threshold': self._coverage_threshold,
            'path_length_m': self._path_length_m,
            'num_odom_samples': self._num_odom_samples,
            'query_labels': list(self._query_labels),
            'target_labels': list(self._target_objects or []),
            'confirmed_targets_enu': targets,
            'target_events': events,
        }
        tmp = os.path.join(self._results_dir, f'.{self._robot_name}.json.tmp')
        final = os.path.join(self._results_dir, f'{self._robot_name}.json')
        try:
            os.makedirs(self._results_dir, exist_ok=True)
            with open(tmp, 'w') as f:
                json.dump(out, f, indent=2)
            os.replace(tmp, final)
        except OSError as e:
            self.get_logger().error(f'[results] write failed: {e}')

    def _publish_discoveries_table(self, discoveries) -> None:
        """Render the merged Discovery list as a compact text table.

        Groups by status (visited / observing / unconfirmed) so the operator
        can see at a glance which targets are completed, which are being
        pursued, and which are single-drone bearings still waiting for a
        second viewpoint to triangulate.
        """
        # Assign a stable per-label index to each instance so the operator
        # can distinguish "house#1" from "house#2" in the table. Sorted by
        # instance_id (hash), so the numbering is deterministic across ticks
        # and across drones — both robots will agree on which house is #1.
        per_label_idx: dict = {}
        for label in sorted({d.label for d in discoveries}):
            same_label = sorted(
                [d for d in discoveries if d.label == label],
                key=lambda x: x.instance_id)
            for i, d in enumerate(same_label, start=1):
                per_label_idx[d.instance_id] = i

        def _pretty(d) -> str:
            idx = per_label_idx.get(d.instance_id)
            return f'{d.label}#{idx}' if idx is not None else d.label

        # Discovery of the assigned label whose position is closest to the
        # forward-projected committed ray (origin + dir * 8m).
        committed_instance = None
        if (self._assigned_target is not None
                and self._committed_target_last_origin is not None
                and self._committed_target_last_dir is not None
                and discoveries):
            d_origin = np.asarray(self._committed_target_last_origin)
            d_dir = np.asarray(self._committed_target_last_dir)
            d_norm = float(np.linalg.norm(d_dir))
            if d_norm > 1e-6:
                proj = d_origin + (d_dir / d_norm) * 8.0
                same_label = [d for d in discoveries
                              if d.label == self._assigned_target]
                if same_label:
                    committed_instance = min(
                        same_label,
                        key=lambda d: float(
                            np.linalg.norm(np.asarray(d.position) - proj)))

        assigned_str = self._assigned_target or '-'
        if committed_instance is not None:
            assigned_str = _pretty(committed_instance)

        # Ray-only bearings have size=None; voxel-confirmed ones carry an AABB.
        aabb_total = sum(1 for d in discoveries if d.size is not None)

        lines = []
        lines.append(f'total={len(discoveries)}  '
                     f'AABBs={aabb_total}  '
                     f'assigned={assigned_str}  '
                     f'committed={self._committed_to_assigned}')

        # Each Discovery is one instance (deduped by AABB / position via
        # discoveries._should_merge and _stable_id), so per-label row counts
        # are the unique-instance counts for that class.
        if discoveries:
            by_label: dict = {}
            for d in discoveries:
                bucket = by_label.setdefault(
                    d.label, {'visited': 0, 'observing': 0,
                              'unconfirmed': 0, 'aabb': 0})
                bucket[d.status] = bucket.get(d.status, 0) + 1
                if d.size is not None:
                    bucket['aabb'] += 1
            lines.append('[per-label instance count]')
            for label in sorted(by_label):
                b = by_label[label]
                total = b['visited'] + b['observing'] + b['unconfirmed']
                lines.append(
                    f'  {label:<14} total={total}  '
                    f'AABBs={b["aabb"]}  '
                    f'visited={b["visited"]}  '
                    f'observing={b["observing"]}  '
                    f'unconfirmed={b["unconfirmed"]}')
        lines.append('')

        if not discoveries:
            lines.append('(no targets discovered yet)')
        else:
            by_status = {'visited': [], 'observing': [], 'unconfirmed': []}
            for d in discoveries:
                by_status.setdefault(d.status, []).append(d)
            section_order = [
                ('visited',    'completed'),
                ('observing',  'pursuing'),
                ('unconfirmed','unconfirmed bearings'),
            ]
            for key, header in section_order:
                lst = by_status.get(key, [])
                if not lst:
                    continue
                lines.append(f'[{header}] ({len(lst)})')
                lines.append('   instance         pos                size              '
                             'conf  contributors')
                # Sort by confidence desc. '*' marks the specific committed
                # instance so two drones on different houses each show their own.
                committed_id = (committed_instance.instance_id
                                if committed_instance is not None else None)
                for d in sorted(lst, key=lambda x: -x.confidence):
                    pos = f'({d.position[0]:>6.1f},{d.position[1]:>6.1f},'\
                          f'{d.position[2]:>5.1f})'
                    if d.size is not None:
                        sz = f'({d.size[0]:>4.1f},{d.size[1]:>4.1f},'\
                             f'{d.size[2]:>4.1f})'
                    else:
                        sz = '   (no AABB)   '
                    contrib = ','.join(d.contributing_robots)
                    pursued = '*' if d.instance_id == committed_id else ' '
                    lines.append(
                        f' {pursued} {_pretty(d):<16} {pos}  {sz}  '
                        f'{d.confidence:>4.2f}  {contrib}')
                lines.append('')

        self._discoveries_table_pub.publish(
            String(data='\n'.join(lines).rstrip()))

    def _publish_completed_zones(self) -> None:
        """Publish observed cells as a packed-bitmask CoverageGrid (local frame)."""
        out = CoverageGrid()
        out.resolution = float(self._cell_size_m)
        if not self._observed_cells:
            out.width = 0
            out.height = 0
            self._completed_zones_pub.publish(out)
            return
        arr = np.array(list(self._observed_cells), dtype=np.int64)
        min_c = arr.min(axis=0)
        max_c = arr.max(axis=0)
        out.width = int(max_c[0] - min_c[0] + 1)
        out.height = int(max_c[1] - min_c[1] + 1)
        out.origin_x = float(min_c[0]) * self._cell_size_m
        out.origin_y = float(min_c[1]) * self._cell_size_m
        occ = np.zeros((out.height, out.width), dtype=np.uint8)
        occ[arr[:, 1] - min_c[1], arr[:, 0] - min_c[0]] = 1
        out.data = np.packbits(occ.reshape(-1)).tobytes()
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
        """(N, 6) float32: [x, y, z, empty_cnt, unobserved_cnt, occupied_cnt].
        Counts are 0-filled when the message lacks them."""
        field_names = {f.name for f in msg.fields}
        has_cnts = ({'empty_cnt', 'unobserved_cnt', 'occupied_cnt'}
                    .issubset(field_names))
        cols = (('x', 'y', 'z', 'empty_cnt', 'unobserved_cnt', 'occupied_cnt')
                if has_cnts else ('x', 'y', 'z'))
        pts = list(point_cloud2.read_points(msg, field_names=cols, skip_nans=True))
        if not pts:
            self._frontiers = None
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        if has_cnts:
            self._frontiers = arr
        else:
            padded = np.zeros((arr.shape[0], 6), dtype=np.float32)
            padded[:, :3] = arr
            self._frontiers = padded

    def _odometry_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        cur = np.array([p.x, p.y, p.z], dtype=np.float64)
        # Cumulative path length with a 2 cm deadband: don't advance the anchor
        # on sub-deadband steps so hover jitter doesn't inflate the total, but
        # small genuine motion still accumulates once it crosses the threshold.
        if self._prev_odom_xyz is None:
            self._prev_odom_xyz = cur
            self._num_odom_samples += 1
        else:
            step = float(np.linalg.norm(cur - self._prev_odom_xyz))
            if step >= 0.02:
                self._path_length_m += step
                self._prev_odom_xyz = cur
                self._num_odom_samples += 1
        self._cur_pose = cur
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._cur_yaw = float(np.arctan2(siny_cosp, cosy_cosp))

    def _clear_blacklist_cb(self, _msg: Empty) -> None:
        n = self._behavior_manager.frontier_behavior.clear_blacklist()
        self.get_logger().warn(
            f'[escalation] clear_blacklist requested — dropped {n} blacklisted XY(s)')

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
        """Capture boot ENU + ground altitude on first valid fix.

        boot_enu must anchor the odom ORIGIN (spawn) to global ENU. raven is
        spawned per-search — after takeoff, and possibly after the drone has
        flown — so the drone is rarely at the odom origin when its first fix
        arrives. Using gps_to_enu(fix) directly would anchor wherever the drone
        currently is, offsetting every world-frame output (confirmed-target
        AABBs, event log, results) by the current odom position — pure XY drift,
        invisible only when the search starts at spawn. odom/map is ENU-aligned,
        so subtract _cur_pose to back the origin out; correct at any capture time.
        """
        if self._boot_enu is not None:
            return
        if msg.status.status < 0:
            return
        if self._cur_pose is None:
            return
        cur = np.asarray(self._cur_pose, dtype=np.float64)
        self._alt_ground = float(msg.altitude) - float(cur[2])
        self._boot_enu = np.array(
            gps_to_enu(msg.latitude, msg.longitude, msg.altitude),
            dtype=np.float64,
        ) - cur
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
                qx, qy, qz, qw = dir_to_quat(g.ray_dirs[k])
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = now
                arrow.ns = 'ray_groups'
                arrow.id = marker_id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = float(p0[0])
                arrow.pose.position.y = float(p0[1])
                arrow.pose.position.z = float(p0[2])
                arrow.pose.orientation.x = qx
                arrow.pose.orientation.y = qy
                arrow.pose.orientation.z = qz
                arrow.pose.orientation.w = qw
                arrow.scale.x = 2.0
                arrow.scale.y = 0.4
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

    def _publish_auction_table(self, tasks, assigned_map):
        """JSON snapshot of the unified table: visited BBs + available targets
        (rays + BBs) with state, size, and assigned robot id. Coordinates are
        global ENU (X/Y + boot, Z kept AGL — same convention as confirmed_targets)
        so GCS can build the overlay; the terminal viewer + GCS read this topic.
        Pre-GPS (no boot_enu) it stays in the local frame."""
        owner = {tk: aid for aid, tk in assigned_map.items()}
        to_world = (self._local_to_world if self._boot_enu is not None
                    else (lambda p: np.asarray(p, dtype=float)))
        frame = 'enu' if self._boot_enu is not None else 'local'
        visited = []
        for h in self._house_boxes():
            if str(h.status).lower() != 'visited':
                continue
            g = to_world(np.asarray(h.center, dtype=float))
            s = np.asarray(h.size, dtype=float)
            visited.append({'label': h.label, 'x': float(g[0]), 'y': float(g[1]),
                            'z': float(g[2]), 'sx': float(s[0]),
                            'sy': float(s[1]), 'sz': float(s[2])})
        available = []
        for t in tasks:
            c = to_world(np.asarray(t.centroid, dtype=float))
            s = np.asarray(t.size, dtype=float)
            entry = {'label': t.label, 'status': t.status,
                     'x': float(c[0]), 'y': float(c[1]), 'z': float(c[2]),
                     'sx': float(s[0]), 'sy': float(s[1]), 'sz': float(s[2]),
                     'assigned': owner.get(t.key)}
            if t.direction is not None:
                # Unit bearing (frame-invariant) so GCS draws a fixed-length
                # arrow from the ray point — no projected far endpoint.
                d = np.asarray(t.direction, dtype=float)
                entry['dx'], entry['dy'], entry['dz'] = (
                    float(d[0]), float(d[1]), float(d[2]))
            available.append(entry)
        self._auction_table_pub.publish(String(data=json.dumps(
            {'robot': self._robot_name, 'my_id': self._my_id, 'frame': frame,
             'visited_bbs': visited, 'available': available})))

    def _publish_ray_leads(self):
        """Gossip the accumulated ray-lead bearings (origin in global ENU; unit
        direction is frame-invariant) so the persistent memory is team-shared."""
        if self._boot_enu is None:
            return
        out = []
        for L in self._ray_leads:
            g = self._local_to_world(np.asarray(L['o'], dtype=float))
            d = np.asarray(L['d'], dtype=float)
            out.append({'label': L['label'],
                        'ox': float(g[0]), 'oy': float(g[1]), 'oz': float(g[2]),
                        'dx': float(d[0]), 'dy': float(d[1]), 'dz': float(d[2])})
        self._ray_leads_pub.publish(String(data=json.dumps(out)))

    def _publish_served_leads(self):
        """Gossip serviced ray-leads (global ENU) so peers drop them too."""
        if self._boot_enu is None:
            return
        out = []
        for lab, c in self._served_leads:
            g = np.asarray(c, dtype=float) + self._boot_enu
            out.append({'label': lab, 'x': float(g[0]),
                        'y': float(g[1]), 'z': float(g[2])})
        self._served_leads_pub.publish(String(data=json.dumps(out)))

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
            # Matches ray_groups.py: argmax must land on a target column AND
            # exceed threshold.
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

        Columns: target, map_total, per-threshold voxel counts, cc_max (size of
        the biggest connected high-conf component at the live
        voxel_score_threshold), and cc_n (instance count)."""
        vox_xyz = self._vox_xyz
        vox_scores = self._vox_scores
        if vox_xyz is None or vox_scores is None or len(vox_xyz) == 0:
            self._voxel_table_pub.publish(String(data='(no voxels yet)'))
            self.get_logger().info('[voxel-table] (no voxels yet)',
                                   throttle_duration_sec=10.0)
            return
        labels = self._column_labels()
        K = vox_scores.shape[1]
        col_labels = [labels[i] if i < len(labels) else f'col_{i}' for i in range(K)]
        target_set = set(self._target_objects or [])
        target_idxs = [j for j, lbl in enumerate(col_labels) if lbl in target_set]
        if not target_idxs:
            self._voxel_table_pub.publish(String(data='(no targets configured)'))
            self.get_logger().info('[voxel-table] (no targets configured)',
                                   throttle_duration_sec=10.0)
            return

        n_total = int(len(vox_xyz))
        thresholds = [0.3, 0.5, 0.7, 0.9]
        # Mirror the live voxel-mode params so cc counts reflect what mode
        # selection is actually doing this tick.
        cc_threshold = self._voxel_score_threshold
        min_samples_default = self._voxel_min_cluster_size

        # Column meanings:
        #   map_total   — total voxels in the whole map (NOT label-specific)
        #   >T columns  — count of voxels where this label's score exceeds T
        #   cc_max      — size of the single largest connected component
        #                 above cc_threshold (this is ONE cluster, not the
        #                 total above-threshold voxel count)
        #   cc_n>=N     — number of distinct clusters at least N voxels in
        #                 size — this is how many instances the system thinks
        #                 it sees of this label
        header_cells = (['target'.ljust(18), 'map_total'.rjust(9)]
                        + [f'>{t}'.rjust(7) for t in thresholds]
                        + ['cc_max'.rjust(7),
                           f'cc_n>={min_samples_default}'.rjust(9)])
        lines = ['  '.join(header_cells)]
        # Per-target cluster size list (filled in below). Surfaced after the
        # main table so the operator sees the size distribution at a glance.
        cluster_size_lines: list = []

        # Connected-component count per target above cc_threshold (mirrors
        # voxel_behavior.condition_check). Only do this when there's data.
        try:
            import scipy.ndimage
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
                    # Per-cluster breakdown: only the ones that pass the
                    # min_cluster_size gate (i.e., real instances, not noise).
                    big_sizes = sorted(
                        (s for s in comp_sizes if s >= min_samples_default),
                        reverse=True)
                    if big_sizes:
                        sizes_str = ', '.join(str(s) for s in big_sizes[:20])
                        if len(big_sizes) > 20:
                            sizes_str += f', ... ({len(big_sizes) - 20} more)'
                        cluster_size_lines.append(
                            f'  {col_labels[j]}: {len(big_sizes)} cluster(s) '
                            f'sizes=[{sizes_str}]')
            cells = ([col_labels[j].ljust(18), str(n_total).rjust(9)]
                     + [str(c).rjust(7) for c in counts]
                     + [str(cc_max).rjust(7), str(cc_n_big).rjust(9)])
            lines.append('  '.join(cells))
        lines.append(
            f'  cc_threshold={cc_threshold}  min_cluster_size={min_samples_default}'
            f'  → voxel-mode fires when the drone is committed to a ray AND '
            f'a cluster geometrically matches that ray'
        )
        if cluster_size_lines:
            lines.append('')
            lines.append('per-target cluster sizes (each cluster = one detected instance):')
            lines.extend(cluster_size_lines)
        body = (f'{n_total} voxels in map  |  '
                f'targets={[col_labels[j] for j in target_idxs]}\n'
                f'(map_total counts ALL voxels regardless of label; '
                f'cc_n column is the per-instance count)\n' + '\n'.join(lines))
        self._voxel_table_pub.publish(String(data=body))
        self.get_logger().info('[voxel-table]\n' + body,
                               throttle_duration_sec=10.0)

    def _timer_cb(self):
        if self._cur_pose is None:
            self.get_logger().warn('waiting for odometry...', throttle_duration_sec=5.0)
            self._nav_mode_pub.publish(String(data='idle'))
            return

        # Mission start = first tick with odometry; metrics are reported
        # relative to it.
        if self._mission_start_ts is None:
            self._mission_start_ts = self.get_clock().now().nanoseconds * 1e-9

        # Peer rays already converted to local frame in PeerState.
        merged_origins, merged_dirs, merged_scores = self._merge_own_and_peer_rays()

        # Shared between the bid auction and ray_behavior waypoint selection.
        ray_groups = compute_ray_groups(
            merged_origins, merged_dirs, merged_scores,
            self._query_labels, self._target_objects,
            self._score_threshold, self._cur_pose,
            min_altitude=self._min_altitude,
            max_altitude=self._max_altitude)
        if self._ray_confirmer is not None:
            ray_groups = self._ray_confirmer.update(
                ray_groups, match=same_ray_group)
        self._publish_ray_groups_viz(ray_groups)

        self._update_discoveries(ray_groups)

        if self._debug_ray_table:
            self._debug_print_ray_table()
            self._debug_print_groups_table(ray_groups)
            self._debug_print_voxel_table()

        own_completed = set(self._behavior_manager.completed_queries)
        all_completed = set(own_completed)
        for peer_done in self._peer_state.peer_completed.values():
            all_completed |= peer_done

        now = self.get_clock().now().nanoseconds * 1e-9
        fresh_peers = self._fresh_peers(now)
        polygon_xy = (self._search_area_xy
                      if isinstance(self._search_area_xy, np.ndarray)
                      and len(self._search_area_xy) >= 3 else None)
        peer_committed_instances = self._fresh_peer_committed_instances(now)

        # Hard-exclude rays at a target already visited (any drone) or one a peer
        # is committed to. Everything else competes by weight (distance + bearing
        # continuity), so off-limits targets are removed from bidding and nav.
        ray_groups = [g for g in ray_groups
                      if not self._ray_off_limits(
                          g, polygon_xy, peer_committed_instances)]
        self._behavior_manager.ray_behavior.ray_groups = ray_groups

        heading_xy = np.array([np.cos(self._cur_yaw), np.sin(self._cur_yaw)])
        # Exploration bids on RAYS only (unconfirmed detections). Confirmed-target
        # (BB) assignment is handled by the consensus below, not this bid auction.
        my_bid_entries = [b for b in bid_manager.compute_my_bids(ray_groups)
                          if b.label not in all_completed]
        bid_manager.finalize_bid_values(
            my_bid_entries, heading_xy,
            self._ray_reach_factor, self._target_behind_penalty_weight)
        peer_bid_entries = {
            name: [e for e in entries
                   if e.label not in all_completed and e.num_rays > 0]
            for name, entries in self._peer_state.peer_bids.items()
            if name in fresh_peers
        }

        # CBAA consensus over the unified task table (ray-leads + confirmed BBs):
        # every robot solves the same shared task set with the same agent
        # positions, so all reach the same conflict-free 1:1 assignment (no bid
        # race, no reactive yields). Winner retention keeps it stable tick-to-tick.
        agent_pos = self._agent_positions(fresh_peers)
        tasks = self._build_consensus_tasks(
            ray_groups, all_completed, polygon_xy, now, agent_pos)
        assigned_map = self._consensus.assign(
            tasks, agent_pos, self._commit_switch_margin_m, self._TASK_MATCH_M)
        my_task = self._consensus.my_task(assigned_map, self._my_id, tasks)

        won = None   # ray exploration is now a consensus task, not a side auction
        if my_task is not None:
            prev = self._assigned_target
            self._assigned_target = my_task.label
            self._committed_to_assigned = True
            if my_task.status == 'ray' and my_task.direction is not None:
                # Lone bearing: follow the ray (origin + dir), no BB point. Seed the
                # committed bearing only on a NEW lead; while continuing the same
                # one, leave it to the bearing-block hysteresis (no per-tick reset)
                # so the hold is strictly persistent.
                self._committed_bb_center = None
                base = np.asarray(my_task.origin if my_task.origin is not None
                                  else my_task.centroid, dtype=float)
                nd = np.asarray(my_task.direction, dtype=float)
                if not self._is_same_committed_lead(base, nd):
                    self._committed_target_last_origin = base.copy()
                    self._committed_target_last_dir = nd.copy()
                self._ensure_followable_lead(my_task)
            else:
                # Localized target (BB or triangulated ray): head to the point.
                self._committed_bb_center = np.asarray(my_task.centroid, float).copy()
            if prev != my_task.label:
                self._committed_bearing_lock_ts = now
                if self._debug_coord:
                    self.get_logger().info(
                        f'[coord] consensus assigned {my_task.label} '
                        f'[{my_task.status}] @ '
                        f'({my_task.centroid[0]:.0f},{my_task.centroid[1]:.0f})')
            self._mark_lead_serviced_if_reached(my_task, ray_groups)
        else:
            # Nothing assigned to me -> frontier exploration (downstream).
            self._assigned_target = None
            self._committed_to_assigned = False
            self._committed_bb_center = None
            self._committed_target_last_origin = None
            self._committed_target_last_dir = None

        self._publish_served_leads()
        self._publish_ray_leads()
        self._publish_auction_table(tasks, assigned_map)

        # Ray bids lost to a peer's same-target bid this tick (debug table only).
        from raven_nav.bid_manager import _GroupView
        from raven_nav.ray_targets import is_same_target as _is_same_target
        rays_avoided = 0
        avoided_breakdown: list = []
        for mine in my_bid_entries:
            best_peer = None
            best_peer_value = mine.value
            for peer_name, p_entries in peer_bid_entries.items():
                pid = self._peer_state.peer_ids.get(peer_name)
                if pid is None:
                    continue
                for p in p_entries:
                    if p.label != mine.label:
                        continue
                    same, _, _ = _is_same_target(
                        _GroupView(mine), _GroupView(p), polygon_xy)
                    if not same:
                        continue
                    if p.value > best_peer_value or (
                            p.value == best_peer_value and pid < self._my_id):
                        best_peer_value = p.value
                        best_peer = peer_name
            if best_peer is not None:
                rays_avoided += 1
                avoided_breakdown.append((mine.label, best_peer))

        # Committed instance bearing. BB: point drone -> BB centre so voxel keeps
        # pursuing this instance. Ray: follow the most-aligned ray group, with
        # hysteresis to avoid flip-flop between near-equal rays.
        if self._committed_bb_center is not None:
            rel = self._committed_bb_center - np.asarray(self._cur_pose, float)
            rn = float(np.linalg.norm(rel))
            if rn > 1e-6:
                self._committed_target_last_origin = np.asarray(
                    self._cur_pose, float).copy()
                self._committed_target_last_dir = rel / rn
        elif self._assigned_target is not None and ray_groups:
            cands = [g for g in ray_groups
                     if g.label == self._assigned_target and g.num_rays > 0]
            if cands:
                def _pin(gg):
                    return self._ray_pin_cost(
                        gg, self._committed_target_last_origin,
                        self._committed_target_last_dir)
                best = min(cands, key=_pin)
                chosen = best
                held = self._match_committed_group(cands)
                if (held is not None and held is not best
                        and self._committed_bearing_lock_ts is not None):
                    held_c = _pin(held)
                    margin = self._commit_swap_frac * (abs(held_c) + 1e-6)
                    held_for = now - self._committed_bearing_lock_ts
                    if not (_pin(best) < held_c - margin
                            and held_for >= self._commit_min_hold_s):
                        chosen = held
                if held is None or chosen is not held:
                    self._committed_bearing_lock_ts = now
                self._committed_target_last_dir = np.asarray(
                    chosen.avg_dir, dtype=np.float64).copy()
                self._committed_target_last_origin = np.asarray(
                    chosen.avg_origin, dtype=np.float64).copy()

        self._behavior_manager.ray_behavior.assigned_target = self._assigned_target

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
        # Broadcast our committed instance (label + bearing in global ENU) so
        # peers can dedup per-instance, not just by label. '' = uncommitted.
        committed_str = ''
        if (self._assigned_target is not None and self._committed_to_assigned
                and self._committed_target_last_origin is not None
                and self._boot_enu is not None):
            o = np.asarray(self._committed_target_last_origin, float) + self._boot_enu
            d = np.asarray(self._committed_target_last_dir, float)
            committed_str = json.dumps({
                'label': self._assigned_target,
                'ox': float(o[0]), 'oy': float(o[1]), 'oz': float(o[2]),
                'dx': float(d[0]), 'dy': float(d[1]), 'dz': float(d[2])})
        self._committed_target_pub.publish(String(data=committed_str))

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

        if rays_avoided > 0:
            by_peer: dict = {}
            for lbl, pname in avoided_breakdown:
                by_peer.setdefault(pname, []).append(lbl)
            parts = []
            for pname in sorted(by_peer):
                lbls = by_peer[pname]
                parts.append(f'{pname}:{len(lbls)} ({",".join(sorted(set(lbls)))})')
            bids_lines.append(
                f'rays_avoided={rays_avoided} (lost to peers — {"; ".join(parts)})')
        else:
            bids_lines.append('rays_avoided=0')
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

            _pcommit = self._peer_state.peer_committed_target
            peer_str = ', '.join(
                f'{n}={_fmt_bid_list(b)}'
                + (f' commit={_pcommit[n]}' if _pcommit.get(n) else '')
                for n, b in self._peer_state.peer_bids.items()
            ) or '(none)'

            self.get_logger().info(
                f'[coord] mode={self._behavior_mode} '
                f'assigned={self._assigned_target} '
                f'committed={self._committed_to_assigned} | '
                f'my_bids={_fmt_bid_list(my_bid_entries)} | '
                f'peer_bids={peer_str}',
                throttle_duration_sec=2.0)

        # Proximity-voxel deconfliction: peers' positions + committed points so a
        # drone defers a nearby cluster to a closer / committed peer.
        vb = self._behavior_manager.voxel_behavior
        vb.my_id = self._my_id
        vb.fresh_peer_points = [
            (self._peer_state.peer_ids.get(n), self._peer_state.peer_positions[n])
            for n in fresh_peers if n in self._peer_state.peer_positions]
        vb.peer_committed_points = [
            self._ray_point(po, pd)
            for (_n, _lbl, po, pd, _pid) in peer_committed_instances]

        prev_mode = self._behavior_mode
        # Cluster over EVERY configured target label so confirmed_targets /
        # AABBs publish from tick 1.
        voxel_targets = list(self._target_objects or [])
        if self._frontier_only_baseline:
            # Baseline = pure-frontier navigation, but we still want PASSIVE
            # detection: run the voxel clustering so target_voxel_clusters
            # populates and AABBs / confirmed_targets publish to Foxglove for
            # anything the drone happens to fly past. Pass no committed ray so
            # EVERY detected cluster is recorded (not just one tied to an
            # auction commitment), and ignore the return value — voxel-MODE
            # must never activate here, so we hard-force Frontier-based.
            self._behavior_manager.voxel_behavior.condition_check(
                self._vox_xyz, self._vox_scores, self._query_labels,
                voxel_targets, committed_origin=None, committed_dir=None)
            # Passive arrival: clusters within 3 m flip to 'visited'; never
            # touches the nav candidate pool, so baseline stays pure frontier.
            self._behavior_manager.voxel_behavior.mark_arrivals(
                self._cur_pose, 3.0)
            self._behavior_manager.behavior_mode = 'Frontier-based'
        else:
            self._behavior_manager.mode_select(
                query_labels=self._query_labels,
                target_objects=voxel_targets,
                vox_xyz=self._vox_xyz,
                vox_scores=self._vox_scores,
                committed_origin=self._committed_target_last_origin,
                committed_dir=self._committed_target_last_dir,
                cur_pose=self._cur_pose,
                committed_bb_center=self._committed_bb_center,
            )
            # Passive arrival status (same as baseline): confirmed clusters
            # within 3 m flip to 'visited'. Status-only; nav unaffected.
            self._behavior_manager.voxel_behavior.mark_arrivals(
                self._cur_pose, 3.0)
        self._behavior_mode = self._behavior_manager.behavior_mode

        if self._behavior_mode != prev_mode:
            self.get_logger().info(f'behavior mode: {prev_mode} -> {self._behavior_mode}')
            self._waypoint_locked = False
            self._target_waypoint = None
            self._target_waypoint2 = None

        if self._cur_pose is not None:
            self._stamp_cells_xy(self._cur_pose[:2].reshape(1, 2))
            if self._vox_xyz is not None and self._vox_xyz.shape[0] > 0:
                self._stamp_cells_xy(self._vox_xyz[:, :2])
            if self._frontiers is not None and self._frontiers.shape[0] > 0:
                # Skip raycast stamping unless the drone has moved at least
                # coverage_raycast_min_step_m since the last stamp. Hover
                # otherwise re-paints the same starburst every tick.
                cur_xy = self._cur_pose[:2]
                moved_enough = (
                    self._last_raycast_xy is None
                    or float(np.linalg.norm(cur_xy - self._last_raycast_xy))
                       >= self._raycast_min_step_m)
                if moved_enough:
                    # frontiers cols 0:3 are RDF; convert to FLU XY for raycast.
                    fr_rdf = self._frontiers[:, :3]
                    fr_flu_xy = np.stack(
                        [fr_rdf[:, 2], -fr_rdf[:, 0]], axis=1)
                    self._stamp_raycast_cells(cur_xy, fr_flu_xy)
                    self._last_raycast_xy = cur_xy.copy()

        own_centers = self._own_cell_centers_xy()
        zone_chunks = [own_centers] if own_centers.shape[0] > 0 else []
        for pz in self._peer_state.peer_completed_zones.values():
            if pz is not None and pz.shape[0] > 0:
                zone_chunks.append(pz)
        completed_zones_xy = np.vstack(zone_chunks) if zone_chunks else None

        self._publish_completed_zones()

        # Polygon-area completion gate. Uses merged own+peer cells so a swarm
        # can finish together. Once tripped, stays tripped for the run.
        if self._search_area_xy is not None and completed_zones_xy is not None:
            frac = self._coverage_fraction(
                self._search_area_xy, completed_zones_xy)
            self._last_coverage_frac = frac
            # Log every new 10% bucket crossed, once each.
            bucket = min(int(frac * 10), 10)
            while self._coverage_milestone < bucket:
                self._coverage_milestone += 1
                pct = self._coverage_milestone * 10
                if pct > 0:
                    self.get_logger().info(
                        f'[coverage] reached {pct}% of polygon '
                        f'(actual {frac * 100:.1f}%)')
            if not self._search_complete and frac >= self._coverage_threshold:
                self._search_complete = True
                self._completion_reason = 'coverage'
                self.get_logger().info(
                    f'search complete: coverage {frac * 100:.1f}% '
                    f'>= {self._coverage_threshold * 100:.1f}% '
                    f'of polygon area — hovering in place')
            elif self._debug_coord:
                self.get_logger().info(
                    f'[coverage] {frac * 100:.1f}% '
                    f'(threshold {self._coverage_threshold * 100:.1f}%)',
                    throttle_duration_sec=2.0)

        if self._search_complete:
            # Final dump on the coverage path, forced to capture the reason.
            self._maybe_dump_results(force=True)
            if self._cur_pose is not None:
                hover = Path()
                hover.header.stamp = self.get_clock().now().to_msg()
                hover.header.frame_id = 'map'
                ps = PoseStamped()
                ps.header = hover.header
                ps.pose.position.x = float(self._cur_pose[0])
                ps.pose.position.y = float(self._cur_pose[1])
                ps.pose.position.z = float(self._cur_pose[2])
                ps.pose.orientation.w = 1.0
                hover.poses.append(ps)
                self._path_pub.publish(hover)
            self._waypoint_locked = True
            self._target_waypoint = (self._cur_pose.copy()
                                     if self._cur_pose is not None else None)
            self._target_waypoint2 = self._target_waypoint
            self._nav_mode_pub.publish(
                String(data=_NAV_MODE_TAG['Complete']))
            completed = sorted(self._behavior_manager.completed_queries)
            self._completed_targets_pub.publish(
                String(data=json.dumps(completed)))
            self._last_completed = completed
            return

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
                cell_size_m=self._cell_size_m,
                my_id=self._my_id,
                search_area_xy=self._search_area_xy,
                debug_logger=(self.get_logger() if self._debug_coord else None),
                assigned_target=self._assigned_target,
                committed_target_dir=(
                    None if self._frontier_only_baseline
                    else self._committed_target_last_dir),
                committed_target_origin=(
                    None if self._frontier_only_baseline
                    else self._committed_target_last_origin),
                committed_bb_center=(
                    None if self._frontier_only_baseline
                    else self._committed_bb_center),
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

        # Keep the result file fresh so a cancel has current data to compile.
        self._maybe_dump_results()


def main(args=None):
    rclpy.init(args=args)
    node = RavenNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort final dump on shutdown (e.g. SIGTERM on task cancel).
        try:
            node._maybe_dump_results(force=True)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
