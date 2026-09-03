"""raven_nav_node — single-agent RAVEN over rayfronts topics.

This is the paper's planner, not a fleet: Voxel > Ray > LVLM-guided > Frontier
(`raven_nav/behavior_manager.py`), each behaviour a numpy port of the matching
file in `RayFronts_raven/rayfronts/behaviors/`. N robots are N independent
ravens; there is no auction, no consensus, no peer state, and the only
surviving baseline is `frontier_only_baseline`.

What this file owns: ROS wiring (parameters, topics, timer), the frame lift
from the robot's local `map` to global ENU, and the AirStack-facing reporting
that the behaviours know nothing about (discoveries, results dump, coverage
completion). The navigation logic itself is in `raven_nav/behaviors/`, which
imports no ROS at all.

Four documented deviations from the OG paper logic — see README.md:
  1. every waypoint's z is clamped into [min_altitude_agl, max_altitude_agl];
  2. `search_area` polygon: viewpoints / ray waypoints / clusters outside it
     are skipped;
  3. `frontier_only_baseline` is kept as a real benchmark arm;
  4. detection memory + coverage completion are kept for the benchmark.
"""
from __future__ import annotations

import json
import os
import re
import threading
import time
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, NavSatFix, PointCloud2
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import MarkerArray

from coordination_bringup.frame_utils import gps_to_enu
from coordination_msgs.msg import CoverageGrid

from raven_nav import params as P
from raven_nav import ros_io
from raven_nav.behavior_manager import BehaviorManager
from raven_nav.behaviors.common import TickContext
from raven_nav.behaviors.lvlm_behavior import build_prompt
from raven_nav.coverage import CoverageTracker
from raven_nav.detection_memory import DetectionMemory, TargetEventLog
from raven_nav.discoveries import (
    build_discoveries, confirmed_targets_to_json, discoveries_to_json,
    ConfirmedTarget,
)
from raven_nav.lvlm_client import AsyncVlmClient, VlmClient, encode_jpeg
from raven_nav.ray_targets import build_targets
from raven_nav.results import build_results_dict, write_results_atomic

# Latched: a late-joining raven still gets the most recent search_area, and a
# late-joining rayfronts still gets the guiding-query list.
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
# mavros publishes raw/fix BEST_EFFORT; matching it here is required.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
# FPV frames: latest only, best effort — a stale frame is worse than none.
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


class _GatedPublisher:
    """Wraps a publisher so publish() is a no-op while disabled — lets an
    operator stop the drone being commanded without stopping the planner
    (`ros2 param set /<robot>/raven_nav nav_output_enabled false`)."""

    def __init__(self, pub, is_enabled) -> None:
        self._pub = pub
        self._is_enabled = is_enabled

    def publish(self, msg) -> None:
        if self._is_enabled():
            self._pub.publish(msg)

    def __getattr__(self, name):
        return getattr(self._pub, name)


def sanitize_topic_label(s: str) -> str:
    """rayfronts' `messaging_services/ros.py::_sanitize_topic_name`, copied so
    we can match a `q{k}_{label}` topic name back to the label that made it."""
    if not isinstance(s, str) or not s:
        return ''
    out = []
    for c in s:
        out.append(c if (c.isalnum() or c == '_') else '_')
    name = ''.join(out)
    while '__' in name:
        name = name.replace('__', '_')
    return name.strip('_')


class RavenNavNode(Node):

    # How often (monotonic seconds) the sim_k -> label mapping is re-verified
    # against the live q{k}_{label} topic names even when the column COUNT is
    # unchanged — a mapper restart can permute labels without changing their
    # number (see _refresh_columns).
    LABEL_REVERIFY_S = 5.0

    def __init__(self, *, context=None, cli_args=None):
        """`context` / `cli_args` exist for tests and for anyone embedding the
        node: `main()` leaves both None and gets exactly the old behaviour
        (default context, global `--ros-args`). Pass a private
        `rclpy.Context()` and the node binds to it — and then its parameters
        must come from `cli_args`, because global arguments belong to the
        default context."""
        super().__init__('raven_nav', context=context, cli_args=cli_args,
                         use_global_arguments=(context is None))

        self._robot_name = os.getenv('ROBOT_NAME', 'robot')
        self._prefix = f'/{self._robot_name}'
        ros_domain = os.getenv('ROS_DOMAIN_ID', '0')
        self._rf_prefix = f'/robot_{ros_domain}/rayfronts/msg_serv'
        # RAYFRONTS_MODE=shared: the off-board server PINS every label that
        # arrives on new_text_query as permanent mission vocabulary
        # (GuidingQueryRegistry.pin), so guiding objects must reach it ONLY
        # via guiding_queries or they become undeletable and the OG
        # delete-when-dropped semantics are lost. per_robot: the legacy
        # server only listens on new_text_query (and has no delete API).
        self._rayfronts_shared = (
            os.getenv('RAYFRONTS_MODE', '').strip().lower() == 'shared')

        # ── parameters ──────────────────────────────────────────────────────
        self._p = {}
        for spec in P.PARAM_TABLE:
            if spec.auto:            # use_sim_time: rclpy declares it itself
                continue
            default = spec.default
            # The three env-backed knobs: the declared default comes from the
            # environment, so an explicit `-p` still overrides it.
            if spec.name == 'lvlm_enabled':
                default = P.resolve_lvlm_enabled(None)
            elif spec.name == 'lvlm_request_interval_s':
                default = P.resolve_lvlm_interval_s(
                    None, warn=self.get_logger().warn)
            elif spec.name == 'lvlm_ray_threshold':
                default = P.resolve_lvlm_ray_threshold(
                    None, warn=self.get_logger().warn)
            elif spec.name == 'voxel_max_extent_m':
                default = P.resolve_voxel_max_extent_m(
                    None, warn=self.get_logger().warn)
            elif spec.name == 'voxel_size_m':
                default = P.resolve_voxel_size_m(
                    None, warn=self.get_logger().warn)
            self._p[spec.name] = self.declare_parameter(
                spec.name, default).value
        ignored = [n for n in P.INERT_PARAMS
                   if self._p.get(n) != P.param(n).default]
        if ignored:
            self.get_logger().info(
                '[params] accepted but IGNORED by single-agent RAVEN '
                f'(coordination/baselines removed): {sorted(ignored)}')

        self._query_labels: List[str] = list(self._p['query_labels'])
        targets = [t for t in self._p['target_labels'] if t]
        self._target_objects: List[str] = targets or list(self._query_labels)
        self._min_altitude = float(self._p['min_altitude_agl'])
        self._max_altitude = float(self._p['max_altitude_agl'])
        self._results_dir = str(self._p['results_dir'])
        self._results_dump_period_s = float(self._p['results_dump_period_s'])
        self._coverage_threshold = float(self._p['coverage_complete_threshold'])
        self._debug_verbose = bool(self._p['debug_coordination'])
        self._debug_tables = bool(self._p['debug_ray_table'])
        self._debug_rows = int(self._p['debug_table_max_rows'])
        self._debug_period = float(self._p['debug_table_period_sec'])
        self._nav_output_enabled = bool(self._p['nav_output_enabled'])
        self._frontier_only = bool(self._p['frontier_only_baseline'])
        if self._frontier_only:
            self.get_logger().info(
                'frontier_only_baseline=true — navigation is pure frontier '
                'exploration (passive voxel detection still on)')

        # ── state ───────────────────────────────────────────────────────────
        self._ray_origins = self._ray_dirs = self._ray_scores = None
        self._vox_xyz = self._vox_scores = None
        self._frontiers = None
        self._cur_pose: Optional[np.ndarray] = None
        self._waypoint_locked = False
        self._target_waypoint = None
        self._target_waypoint2 = None
        self._behavior_mode = 'Frontier-based'
        self._search_area_xy: Optional[np.ndarray] = None
        self._warned_polygon_degenerate = False
        self._boot_enu: Optional[np.ndarray] = None
        self._alt_ground: Optional[float] = None
        self._mission_start_ts: Optional[float] = None
        self._completion_reason = ''
        self._path_length_m = 0.0
        self._prev_odom_xyz: Optional[np.ndarray] = None
        self._num_odom_samples = 0
        self._search_complete = False
        # MAP-READY hold (see _rf_status_cb / _map_unusable). The real hazard
        # when the shared mapper OOM-restarts is not that the process is
        # briefly gone — it is that the MAP is empty and stays empty while it
        # rebuilds, so flying on it gives garbage frontiers/detections. Status
        # arrival resumes at 1 Hz the instant the process restarts, well
        # before the map is usable, so we gate on the map itself: the status
        # JSON carries frames_total (resets on restart) and vox_count
        # (collapses, then climbs). Hold when the map has collapsed and until
        # vox_count recovers past a floor — this covers BOTH the down window
        # (no status at all) and the rebuild window.
        self._last_rf_status_ts = None
        self._rf_prev_frames = None
        self._rf_map_ready = None            # None until first status
        self._mapper_holding = False
        self._rf_hold_timeout_s = float(
            os.getenv('RAYFRONTS_HOLD_TIMEOUT_S', '') or 5.0)
        self._rf_resume_vox = int(
            os.getenv('RAYFRONTS_RESUME_VOX', '') or 300)
        # RESTART-ON-COMPLETE (user 2026-09-02 night: "after the entire area
        # is explored it'll reset and start over"). When on, reaching the
        # coverage threshold wipes the explored map + frontier viewpoint
        # memory and keeps exploring, instead of latching complete and
        # hovering. env RAVEN_COVERAGE_RESTART, default off = OG benchmark
        # behaviour (complete-and-hover) preserved for every other mission.
        self._coverage_restart = (
            os.getenv('RAVEN_COVERAGE_RESTART', '').strip().lower()
            in ('1', 'true', 'yes'))
        self._coverage_laps = 0
        self._last_results_dump_ts = 0.0
        self._last_table_ts = 0.0
        self._prev_ray_marker_count = 0
        self._prev_cluster_marker_count = 0
        self._detected_query_labels: Optional[List[str]] = None
        self._labels_verified_ts: float = 0.0
        self._registered_queries: set = set()
        self._latest_image = None

        self._coverage = CoverageTracker(
            cell_size_m=float(self._p['coverage_cell_size_m']),
            raycast_range_m=float(self._p['coverage_raycast_range_m']),
            raycast_min_step_m=float(self._p['coverage_raycast_min_step_m']))
        self._detections = DetectionMemory(
            min_confidence=float(self._p['voxel_min_confidence']))
        self._events = TargetEventLog()
        self._manager = BehaviorManager(
            score_threshold=float(self._p['score_threshold']),
            voxel_score_threshold=float(self._p['voxel_score_threshold']),
            voxel_min_cluster_size=int(self._p['voxel_min_cluster_size']),
            voxel_max_extent_m=float(self._p['voxel_max_extent_m'] or 0.0),
            voxel_size_m=float(self._p['voxel_size_m'] or 0.5),
            lvlm_enabled=bool(self._p['lvlm_enabled']),
            lvlm_request_interval_s=float(self._p['lvlm_request_interval_s']),
            lvlm_ray_threshold=float(self._p['lvlm_ray_threshold']),
            frontier_only=self._frontier_only)

        self._make_publishers()
        self._make_subscriptions()
        self._setup_lvlm()

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(float(self._p['timer_period']), self._timer_cb)

        self.get_logger().info(
            f'{P.LOG_STARTED} | robot={self._robot_name} | '
            f'timer={float(self._p["timer_period"]):.2f}s | '
            f'query_labels={self._query_labels} | '
            f'targets={self._target_objects} | '
            f'score_threshold={float(self._p["score_threshold"])} | '
            f'voxel_score_threshold={float(self._p["voxel_score_threshold"])} | '
            f'altitude=[{self._min_altitude}, {self._max_altitude}] | '
            f'lvlm={"on" if self._manager.lvlm_behavior.enabled else "off"}')

    # ── wiring ──────────────────────────────────────────────────────────────
    def _make_publishers(self) -> None:
        pfx = self._prefix
        self._path_pub = _GatedPublisher(
            self.create_publisher(Path, f'{pfx}/global_plan', 10),
            lambda: self._nav_output_enabled)
        self._nav_mode_pub = self.create_publisher(
            String, f'{pfx}/navigation_mode', 10)
        self._completed_pub = self.create_publisher(
            String, f'{pfx}/completed_targets', 10)
        self._discoveries_pub = self.create_publisher(
            String, f'{pfx}/raven_nav/discoveries', 10)
        self._confirmed_pub = self.create_publisher(
            String, f'{pfx}/raven_nav/confirmed_targets', 10)
        self._coverage_pub = self.create_publisher(
            CoverageGrid, f'{pfx}/raven_nav/explored_area_coverage', 10)
        self._viewpoint_pub = self.create_publisher(
            PointCloud2, f'{pfx}/frontier_viewpoints', 10)
        self._kept_frontiers_pub = self.create_publisher(
            PointCloud2, f'{pfx}/filtered_frontiers', 10)
        self._filtered_rays_pub = self.create_publisher(
            MarkerArray, f'{pfx}/filtered_rays', 10)
        self._voxel_bbox_pub = self.create_publisher(
            MarkerArray, f'{pfx}/voxel_clusters', 10)
        self._current_target_pub = self.create_publisher(
            String, f'{pfx}/current_target', 10)
        self._table_pubs = {
            name: self.create_publisher(String, f'{pfx}/debug/{name}', 10)
            for name in ('ray_table', 'groups_table', 'voxel_table',
                         'frontier_table', 'discoveries_table')}
        # LVLM-guided behaviour (new).
        self._lvlm_trigger_pub = self.create_publisher(
            Bool, f'{pfx}/raven_nav/lvlm_trigger', 10)
        self._guiding_objects_pub = self.create_publisher(
            String, f'{pfx}/raven_nav/guiding_objects', 10)
        self._lvlm_request_pub = self.create_publisher(
            String, f'{pfx}/raven_nav/lvlm_request', 10)
        # To rayfronts: one label per String (the legacy per-robot server) plus
        # the whole current guiding list as JSON (the shared server).
        self._text_query_pub = self.create_publisher(
            String, f'{self._rf_prefix}/new_text_query', 10)
        self._guiding_queries_pub = self.create_publisher(
            String, f'{self._rf_prefix}/guiding_queries', LATCHED_QOS)

    def _make_subscriptions(self) -> None:
        pfx = self._prefix
        self.create_subscription(PointCloud2, f'{self._rf_prefix}/rays_sim/all',
                                 self._ray_all_cb, 10)
        self.create_subscription(PointCloud2, f'{self._rf_prefix}/voxels_sim/all',
                                 self._vox_all_cb, 10)
        self.create_subscription(PointCloud2, f'{self._rf_prefix}/frontiers',
                                 self._frontiers_cb, 10)
        self.create_subscription(Odometry, f'{pfx}/odometry',
                                 self._odometry_cb, 10)
        self.create_subscription(String, '/input_prompt',
                                 self._input_prompt_cb, 10)
        self.create_subscription(Empty, f'{pfx}/raven_nav/clear_blacklist',
                                 self._clear_blacklist_cb, 10)
        self.create_subscription(NavSatFix,
                                 f'{pfx}/interface/mavros/global_position/raw/fix',
                                 self._navsat_cb, SENSOR_QOS)
        self.create_subscription(PolygonStamped, f'{pfx}/raven_nav/search_area',
                                 self._search_area_cb, LATCHED_QOS)
        self.create_subscription(String, f'{pfx}/raven_nav/lvlm_output',
                                 self._lvlm_output_cb, 10)
        # Shared-mapper LIVENESS: /robot_N/rayfronts/status ticks at 1 Hz
        # while the off-board mapper is up. When it OOM-restarts (~10 s down
        # + warmup) this goes stale; raven then HOLDS instead of flying on a
        # frozen/empty map and picking bad frontiers (user 2026-09-02 night).
        rf_domain = os.getenv('ROS_DOMAIN_ID', '0')
        self.create_subscription(
            String, f'/robot_{rf_domain}/rayfronts/status',
            self._rf_status_cb, 10)
        self._image_topic = P.resolve_image_topic(
            self._p['lvlm_image_topic'], self._robot_name)
        self.create_subscription(Image, self._image_topic,
                                 self._image_cb, IMAGE_QOS)

    def _setup_lvlm(self) -> None:
        """Preflight the VLM endpoint off the main thread; an unreachable
        endpoint disables the behaviour (it falls through to Frontier)."""
        self._vlm = None
        beh = self._manager.lvlm_behavior
        if not beh.enabled:
            return
        client = VlmClient(base_url=str(self._p['lvlm_vlm_url']),
                           model=str(self._p['lvlm_vlm_model']))
        self._vlm = AsyncVlmClient(client)

        def _preflight():
            try:
                models = client.preflight(timeout=10.0)
            except Exception as exc:                     # noqa: BLE001
                beh.enabled = False
                self.get_logger().warn(
                    f'[lvlm] VLM endpoint {client.base_url} unreachable '
                    f'({type(exc).__name__}: {exc}) — LVLM-guided behaviour '
                    'DISABLED for this run')
                return
            self.get_logger().info(
                f'[lvlm] VLM ready at {client.base_url} model={client.model} '
                f'(served: {models}) image={self._image_topic} '
                f'interval={beh.request_interval_s:g}s')

        threading.Thread(target=_preflight, daemon=True,
                         name='raven-vlm-preflight').start()

    # ── subscriptions ───────────────────────────────────────────────────────
    def _detect_rayfronts_labels(self) -> Optional[List[str]]:
        """Parse `{rf}/rays_sim/q{N}_<label>` topic names — the authoritative
        column order. Sanitised names are matched back to the labels we know
        (queries + guiding objects); anything unknown keeps its sanitised
        form with underscores turned back into spaces."""
        prefix = f'{self._rf_prefix}/rays_sim/'
        pat = re.compile(rf'^{re.escape(prefix)}q(\d+)_(.+)$')
        known = {sanitize_topic_label(l): l for l in
                 list(self._query_labels)
                 + list(self._manager.lvlm_behavior.guiding_objects)}
        parsed = []
        for name, _types in self.get_topic_names_and_types():
            m = pat.match(name)
            if m:
                sanitized = m.group(2)
                parsed.append((int(m.group(1)),
                               known.get(sanitized, sanitized.replace('_', ' '))))
        if not parsed:
            return None
        parsed.sort()
        return [lbl for _i, lbl in parsed]

    def _refresh_columns(self, n_sims: int) -> None:
        # RE-VERIFY EVEN WHEN THE COUNT MATCHES (user directive, live
        # 2026-09-02: "verify the sim_X order before assuming which is
        # person"). A mapper restart that re-registers the SAME number of
        # labels in a DIFFERENT order used to slip past the count-equality
        # early-return below, leaving this node confirming `road` voxels as
        # `person` off a stale sim_k -> label mapping. The topic-name scan is
        # cheap; every LABEL_REVERIFY_S we run it regardless.
        cached = len(self._detected_query_labels or [])
        now = time.monotonic()
        if (cached == n_sims
                and now - self._labels_verified_ts < self.LABEL_REVERIFY_S):
            return
        detected = self._detect_rayfronts_labels()
        self._labels_verified_ts = now
        if detected is None or detected == self._detected_query_labels:
            return
        prev = self._detected_query_labels
        self._detected_query_labels = detected
        self._query_labels = list(detected)
        self.get_logger().warning(
            f'[ray_table] column labels refreshed: {prev} -> {detected}')

    def _ray_all_cb(self, msg: PointCloud2):
        self._refresh_columns(len(ros_io.sim_field_names(msg)))
        o, d, s = ros_io.parse_ray_cloud(msg)
        self._ray_origins, self._ray_dirs, self._ray_scores = o, d, s

    def _vox_all_cb(self, msg: PointCloud2):
        self._refresh_columns(len(ros_io.sim_field_names(msg)))
        xyz, s = ros_io.parse_voxel_cloud(msg)
        self._vox_xyz, self._vox_scores = xyz, s

    def _frontiers_cb(self, msg: PointCloud2):
        self._frontiers = ros_io.parse_frontier_cloud(msg)

    def _odometry_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        cur = np.array([p.x, p.y, p.z], dtype=np.float64)
        if self._prev_odom_xyz is None:
            self._prev_odom_xyz = cur
            self._num_odom_samples += 1
        else:
            # 2 cm deadband so hover jitter does not inflate the path length.
            step = float(np.linalg.norm(cur - self._prev_odom_xyz))
            if step >= 0.02:
                self._path_length_m += step
                self._prev_odom_xyz = cur
                self._num_odom_samples += 1
        self._cur_pose = cur

    def _image_cb(self, msg: Image):
        self._latest_image = msg

    def _input_prompt_cb(self, msg: String):
        targets = [t.strip() for t in msg.data.split(',') if t.strip()]
        if not targets:
            return
        self._target_objects = targets
        self.get_logger().info(f'target objects updated: {self._target_objects}')

    def _clear_blacklist_cb(self, _msg: Empty) -> None:
        # The OG planner had no blacklist and neither does this one; the
        # subscription stays so the escalation hook keeps its topic.
        self.get_logger().warn(
            '[escalation] clear_blacklist requested — no-op: single-agent '
            'RAVEN keeps no frontier blacklist')

    def _navsat_cb(self, msg: NavSatFix):
        """Anchor the odom ORIGIN (not the current position) to global ENU.

        raven is spawned after takeoff, so the drone is rarely at the odom
        origin when its first fix lands; odom/map is ENU-aligned, so
        subtracting the current pose backs the origin out correctly at any
        capture time."""
        if self._boot_enu is not None or msg.status.status < 0 \
                or self._cur_pose is None:
            return
        cur = np.asarray(self._cur_pose, dtype=np.float64)
        self._alt_ground = float(msg.altitude) - float(cur[2])
        self._boot_enu = np.array(
            gps_to_enu(msg.latitude, msg.longitude, msg.altitude),
            dtype=np.float64) - cur
        self.get_logger().info(
            f'{P.LOG_BOOT_GPS}: alt_ground={self._alt_ground:.2f}m, '
            f'boot_enu=({self._boot_enu[0]:.2f}, {self._boot_enu[1]:.2f}, '
            f'{self._boot_enu[2]:.2f})')

    def _search_area_cb(self, msg: PolygonStamped):
        pts = msg.polygon.points
        if len(pts) < 3:
            self._search_area_xy = None
            if not self._warned_polygon_degenerate and len(pts) > 0:
                self.get_logger().warn(
                    f'{P.LOG_SEARCH_AREA} has {len(pts)} vertex/vertices (<3); '
                    'treating as unconstrained.')
                self._warned_polygon_degenerate = True
            return
        self._warned_polygon_degenerate = False
        self._search_area_xy = np.array([[p.x, p.y] for p in pts],
                                        dtype=np.float64)
        self.get_logger().info(
            f'{P.LOG_SEARCH_AREA} updated: '
            f'{self._search_area_xy.shape[0]} vertices.')

    def _lvlm_output_cb(self, msg: String):
        """An externally supplied LVLM answer (OG mapping_server_rosnode.py:
        504-506 `/lvlm_output`). Used for tests and manual guidance."""
        self.get_logger().info(f'[lvlm] external answer: {msg.data}')
        self._apply_lvlm_answer(msg.data)

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        for p in parameters:
            if p.name == 'nav_output_enabled':
                self._nav_output_enabled = bool(p.value)
                self.get_logger().warn(
                    f'[raven] nav_output_enabled -> {self._nav_output_enabled}')
        return SetParametersResult(successful=True)

    def _rf_status_cb(self, msg) -> None:
        """Track shared-mapper liveness AND map-readiness from the status
        JSON (frames_total, vox_count)."""
        self._last_rf_status_ts = self._now()
        try:
            s = json.loads(msg.data)
            frames = int(s.get('frames_total', 0))
            vox = int(s.get('vox_count', 0))
        except (ValueError, TypeError, AttributeError):
            return
        # A restart resets frames_total to a small value. Detect the drop and
        # mark the map not-ready until vox_count climbs back.
        if self._rf_prev_frames is not None and frames < self._rf_prev_frames:
            self._rf_map_ready = False
        self._rf_prev_frames = frames
        if self._rf_map_ready is None:
            self._rf_map_ready = vox >= self._rf_resume_vox
        elif not self._rf_map_ready and vox >= self._rf_resume_vox:
            self._rf_map_ready = True

    def _map_unusable(self) -> bool:
        """Hold when the shared map is not usable: either the mapper has gone
        fully silent (status stale) or it has restarted and the fresh map has
        not rebuilt past the resume floor yet. False before the first status
        (raven startup) and when disabled (timeout <= 0)."""
        if not self._rayfronts_shared or self._rf_hold_timeout_s <= 0.0:
            return False
        if self._last_rf_status_ts is None:
            return False
        if (self._now() - self._last_rf_status_ts) > self._rf_hold_timeout_s:
            return True                       # fully down: no status at all
        return self._rf_map_ready is False    # restarted, still rebuilding

    # ── frames + helpers ────────────────────────────────────────────────────
    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _local_to_world(self, p) -> np.ndarray:
        """Local `map` -> global ENU: add boot ENU in XY, keep z as AGL (the
        ground-truth annotations are ground-relative)."""
        p = np.asarray(p, dtype=float)
        if self._boot_enu is None:
            return np.array([p[0], p[1], p[2]], dtype=float)
        b = self._boot_enu
        return np.array([p[0] + b[0], p[1] + b[1], p[2]], dtype=float)

    def _tick_context(self) -> TickContext:
        # NOT `_context`: rclpy's Node.__init__ stores the ROS context on
        # `self._context`, and an instance attribute shadows a method.
        return TickContext(
            cur_pose=np.asarray(self._cur_pose, dtype=np.float64),
            now=self._now(),
            query_labels=list(self._query_labels),
            target_objects=list(self._target_objects),
            ray_origins=self._ray_origins, ray_dirs=self._ray_dirs,
            ray_scores=self._ray_scores,
            vox_xyz=self._vox_xyz, vox_scores=self._vox_scores,
            frontiers=self._frontiers,
            search_area_xy=self._search_area_xy,
            min_altitude=self._min_altitude, max_altitude=self._max_altitude,
            waypoint_locked=self._waypoint_locked,
            target_waypoint=self._target_waypoint,
            target_waypoint2=self._target_waypoint2,
            # Anti-revisit: FrontierBehavior penalises viewpoints whose cell
            # neighbourhood is already covered. This is the tracker's LIVE set,
            # deliberately aliased rather than copied — a 250 m plate at 0.5 m
            # is ~250k tuples and this runs at the tick rate. Two consequences,
            # both intended: (a) behaviours must treat it as read-only, and
            # (b) `_update_coverage` runs after `_tick_context()` on the same
            # tick, so by the time the behaviour scores, the set already
            # includes this tick's stamps — the drone's current position never
            # reads as novel.
            observed_cells=self._coverage.cells_set,
            coverage_cell_size_m=self._coverage.cell_size_m,
            # Visited-target boxes for the ray anti-revisit exclusion
            # (behaviors/ray_behavior.py). Own detections only.
            visited_bbs=[
                np.concatenate([np.asarray(ct.center, float),
                                np.asarray(ct.size, float)])
                for ct in self._detections.confirmed_targets()
                if str(ct.status).lower() == 'visited'])

    # ── LVLM plumbing ───────────────────────────────────────────────────────
    def _apply_lvlm_answer(self, answer: str) -> None:
        beh = self._manager.lvlm_behavior
        objects = beh.set_guiding_objects(answer)
        self._guiding_objects_pub.publish(String(data=json.dumps(objects)))
        # Whole list for the shared server (latched)...
        self._guiding_queries_pub.publish(String(data=json.dumps(objects)))
        # ...and one message per NEW label for the legacy per-robot server —
        # but NOT in shared mode: the shared server pins every
        # new_text_query label forever (see __init__), and guiding labels
        # must stay deletable.
        if not self._rayfronts_shared:
            for obj in objects:
                if obj in self._registered_queries:
                    continue
                self._registered_queries.add(obj)
                self._text_query_pub.publish(String(data=obj))
        if beh.guiding_changed:
            self.get_logger().info(f'[lvlm] guiding objects: {objects}')

    def _service_lvlm(self, ctx: TickContext) -> None:
        beh = self._manager.lvlm_behavior
        if beh.want_trigger:
            self._lvlm_trigger_pub.publish(Bool(data=True))
        if beh.want_request and self._vlm is not None and not self._vlm.busy:
            prompt = build_prompt(ctx.target_objects)
            jpeg = self._latest_jpeg()
            if jpeg is None:
                self.get_logger().warn(
                    f'[lvlm] no FPV frame on {self._image_topic} yet — '
                    'skipping this request', throttle_duration_sec=30.0)
            else:
                self._vlm.submit(prompt, jpeg)
        result = self._vlm.poll() if self._vlm is not None else None
        if result is None:
            return
        if result.ok:
            self.get_logger().info(
                f'[lvlm] answer in {result.latency_s:.1f}s: {result.answer!r}')
            self._apply_lvlm_answer(result.answer)
        else:
            self.get_logger().warn(f'[lvlm] request failed: {result.error}')
        self._lvlm_request_pub.publish(String(data=json.dumps({
            'ts': result.ts, 'prompt': result.prompt, 'model': result.model,
            'latency_s': round(result.latency_s, 3), 'ok': result.ok,
            'answer': result.answer, 'error': result.error,
            'objects': list(beh.guiding_objects)})))

    def _latest_jpeg(self) -> Optional[bytes]:
        msg = self._latest_image
        if msg is None:
            return None
        try:
            bgr = ros_io.image_to_bgr(msg)
            return encode_jpeg(bgr) if bgr is not None else None
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().warn(f'[lvlm] JPEG encode failed: {exc}',
                                   throttle_duration_sec=30.0)
            return None

    # ── reporting ───────────────────────────────────────────────────────────
    def _publish_detections(self, ctx: TickContext) -> None:
        now = ctx.now
        clusters = self._manager.voxel_behavior.clusters
        self._detections.update(clusters, now)
        self._detections.mark_reached(ctx.cur_pose)
        for c in self._manager.voxel_behavior.newly_visited:
            self._detections.mark_visited(c.label, c.center, c.size)
        self._manager.voxel_behavior.newly_visited = []
        confirmed = self._detections.confirmed_targets()

        world = [ConfirmedTarget(label=ct.label,
                                 center=self._local_to_world(ct.center),
                                 size=ct.size, status=ct.status,
                                 confidence=ct.confidence, ts=ct.ts)
                 for ct in confirmed]
        self._confirmed_pub.publish(String(data=confirmed_targets_to_json(world)))

        groups = self._manager.ray_behavior.ray_groups()
        known_bbs = [(i, ct.label,
                      np.concatenate([np.asarray(ct.center, dtype=float),
                                      np.asarray(ct.size, dtype=float)]))
                     for i, ct in enumerate(confirmed)]
        ray_targets = build_targets(own_groups=groups, peer_groups=[],
                                    known_bbs=known_bbs,
                                    polygon_xy=ctx.search_area_xy,
                                    now_ts=now)
        discoveries = build_discoveries(ray_targets=ray_targets,
                                        confirmed_targets=confirmed,
                                        contributing_robot=self._robot_name,
                                        peer_contributions=None, now_ts=now)
        self._discoveries_pub.publish(String(data=discoveries_to_json(discoveries)))

        if self._boot_enu is not None:
            for kind, ev in self._events.update(
                    discoveries, self._local_to_world, now,
                    self._target_objects):
                pos = ev['pos_enu']
                self.get_logger().info(
                    f'[event] {kind} {ev["label"]} @ENU=('
                    f'{pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) t={now:.2f}')
        self._publish_tables(ctx, discoveries)

    def _publish_tables(self, ctx: TickContext, discoveries) -> None:
        if not self._debug_tables:
            return
        now = ctx.now
        if (now - self._last_table_ts) < self._debug_period:
            return
        self._last_table_ts = now
        self._table_pubs['frontier_table'].publish(
            String(data=self._manager.frontier_behavior.frontier_table(ctx)))
        self._table_pubs['groups_table'].publish(
            String(data=self._manager.ray_behavior.group_table()))
        self._table_pubs['voxel_table'].publish(
            String(data=self._manager.voxel_behavior.voxel_table()))
        self._table_pubs['ray_table'].publish(String(data=self._ray_table(ctx)))
        lines = [f'discoveries={len(discoveries)}']
        for d in discoveries[:self._debug_rows]:
            p = self._local_to_world(d.position)
            lines.append(f'{d.status:<12} {d.label:<18} '
                         f'({p[0]:8.1f},{p[1]:8.1f},{p[2]:6.1f}) '
                         f'conf={d.confidence:.2f} id={d.instance_id}')
        self._table_pubs['discoveries_table'].publish(
            String(data='\n'.join(lines)))

    def _ray_table(self, ctx: TickContext) -> str:
        s = ctx.ray_scores
        if s is None or len(s) == 0:
            return 'no rays'
        cols = ctx.target_columns()
        header = ' '.join(f'{l[:10]:>10}' for l in ctx.query_labels)
        lines = [f'rays={len(s)} columns: {ctx.query_labels}',
                 f'{"#":>5} {"x":>8} {"y":>8} {"z":>7} {header}']
        rank = (np.argsort(-s[:, cols].max(axis=1)) if cols
                else np.arange(len(s)))
        for i in rank[:self._debug_rows]:
            o = ctx.ray_origins[int(i)]
            row = ' '.join(f'{v:10.3f}' for v in s[int(i)])
            lines.append(f'{int(i):>5} {o[0]:8.1f} {o[1]:8.1f} {o[2]:7.1f} {row}')
        return '\n'.join(lines)

    def _publish_viz(self, ctx: TickContext) -> None:
        stamp = self._stamp()
        fb = self._manager.frontier_behavior
        if fb.viewpoints.shape[0]:
            self._viewpoint_pub.publish(
                ros_io.make_xyz_cloud(stamp, fb.viewpoints))
        if fb.kept_frontiers.shape[0]:
            self._kept_frontiers_pub.publish(
                ros_io.make_xyz_cloud(stamp, fb.kept_frontiers))
        origins, dirs, gids = self._manager.ray_behavior.arrows()
        markers, n = ros_io.make_ray_markers(stamp, origins, dirs, gids,
                                             self._prev_ray_marker_count)
        self._prev_ray_marker_count = n
        self._filtered_rays_pub.publish(markers)
        boxes, n = ros_io.make_cluster_markers(
            stamp, self._manager.voxel_behavior.unvisited,
            self._prev_cluster_marker_count)
        self._prev_cluster_marker_count = n
        self._voxel_bbox_pub.publish(boxes)
        self._coverage_pub.publish(
            ros_io.make_coverage_grid(self._coverage.packed_grid()))
        target = self._manager.ray_behavior.current_target
        if target:
            self._current_target_pub.publish(String(data=target))

    def _maybe_dump_results(self, force: bool = False) -> None:
        if not self._results_dir or self._boot_enu is None:
            return
        now = self._now()
        if not force:
            if self._results_dump_period_s <= 0.0:
                return
            if (now - self._last_results_dump_ts) < self._results_dump_period_s:
                return
        self._last_results_dump_ts = now
        payload = build_results_dict(
            robot=self._robot_name, boot_enu=self._boot_enu,
            alt_ground=self._alt_ground,
            mission_start_ts=self._mission_start_ts, now=now,
            completion_reason=self._completion_reason,
            coverage_fraction=self._coverage.fraction,
            coverage_threshold=self._coverage_threshold,
            path_length_m=self._path_length_m,
            num_odom_samples=self._num_odom_samples,
            query_labels=self._query_labels,
            target_labels=self._target_objects,
            confirmed_targets=self._detections.confirmed_targets(),
            to_world=self._local_to_world,
            target_events=self._events.events)
        try:
            write_results_atomic(self._results_dir, self._robot_name, payload)
        except OSError as e:
            self.get_logger().error(f'[results] write failed: {e}')

    # ── coverage ────────────────────────────────────────────────────────────
    def _update_coverage(self, ctx: TickContext) -> None:
        self._coverage.stamp_points(ctx.cur_pose[:2].reshape(1, 2))
        if ctx.vox_xyz is not None and len(ctx.vox_xyz):
            self._coverage.stamp_points(ctx.vox_xyz[:, :2])
        if ctx.frontiers is not None and len(ctx.frontiers):
            fr = np.asarray(ctx.frontiers, dtype=np.float64)[:, :3]
            self._coverage.stamp_raycast(
                ctx.cur_pose[:2], np.stack([fr[:, 2], -fr[:, 0]], axis=1))
        if ctx.search_area_xy is None:
            return
        frac = self._coverage.coverage_fraction(ctx.search_area_xy)
        for pct in self._coverage.new_milestones(frac):
            self.get_logger().info(
                f'[coverage] reached {pct}% of polygon (actual {frac*100:.1f}%)')
        if frac >= self._coverage_threshold and not self._search_complete:
            if self._coverage_restart:
                self._coverage_laps += 1
                self.get_logger().info(
                    f'[coverage] {frac*100:.1f}% >= '
                    f'{self._coverage_threshold*100:.1f}% — RESETTING to '
                    f're-explore (lap {self._coverage_laps})')
                self._coverage.reset()
                self._manager.frontier_behavior.visited_viewpoints.clear()
            else:
                self._search_complete = True
                self._completion_reason = 'coverage'
                self.get_logger().info(
                    f'search complete: coverage {frac*100:.1f}% >= '
                    f'{self._coverage_threshold*100:.1f}% of polygon area — '
                    'hovering in place')
        elif self._debug_verbose:
            self.get_logger().info(
                f'[coverage] {frac*100:.1f}% '
                f'(threshold {self._coverage_threshold*100:.1f}%)',
                throttle_duration_sec=2.0)

    # ── tick ────────────────────────────────────────────────────────────────
    def _timer_cb(self):
        if self._cur_pose is None:
            self.get_logger().warn(P.LOG_WAITING_ODOM, throttle_duration_sec=5.0)
            self._nav_mode_pub.publish(String(data=P.NAV_MODE_IDLE))
            return
        if self._mission_start_ts is None:
            self._mission_start_ts = self._now()

        ctx = self._tick_context()
        # Perception first, and unconditionally: the frontier-only baseline
        # reports detections it never navigates to.
        self._manager.perceive(ctx)
        self._publish_detections(ctx)
        self._update_coverage(ctx)

        if self._map_unusable():
            # Freeze at the current pose until the shared map is usable again.
            if not self._mapper_holding:
                self._mapper_holding = True
                self.get_logger().warn(
                    'shared map unusable (mapper down or rebuilding) — '
                    'HOLDING position until vox_count >= %d'
                    % self._rf_resume_vox)
            hover = ctx.clamp(ctx.cur_pose)
            self._waypoint_locked = True
            self._target_waypoint = hover
            self._target_waypoint2 = hover
            self._path_pub.publish(ros_io.make_path(self._stamp(), [hover]))
            self._nav_mode_pub.publish(String(data=P.NAV_MODE_IDLE))
            return
        if self._mapper_holding:
            self._mapper_holding = False
            self._waypoint_locked = False
            self.get_logger().info('shared map rebuilt — resuming search')

        if self._search_complete:
            self._finish(ctx)
            return

        prev_mode = self._behavior_mode
        self._behavior_mode = self._manager.mode_select(ctx)
        if self._behavior_mode != prev_mode:
            # OG mapping_server_rosnode.py:508-511 `mode_switch_trigger`.
            self.get_logger().info(
                f'behavior mode: {prev_mode} -> {self._behavior_mode}')
            self._waypoint_locked = False
            self._target_waypoint = None
            self._target_waypoint2 = None
            ctx.waypoint_locked = False
            ctx.target_waypoint = None
            ctx.target_waypoint2 = None

        self._service_lvlm(ctx)

        out = self._manager.behavior_execute(self._behavior_mode, ctx)
        self._waypoint_locked = out.waypoint_locked
        self._target_waypoint = out.target_waypoint
        self._target_waypoint2 = out.target_waypoint2
        if out.path:
            self._path_pub.publish(ros_io.make_path(self._stamp(), out.path))
        elif out.note and self._debug_verbose:
            self.get_logger().info(f'[{self._behavior_mode}] {out.note}',
                                   throttle_duration_sec=5.0)

        self._publish_viz(ctx)
        self._nav_mode_pub.publish(String(
            data=P.NAV_MODE_TAG.get(self._behavior_mode, P.NAV_MODE_IDLE)))

        completed = self._detections.completed_labels()
        self._completed_pub.publish(String(data=json.dumps(completed)))

        self.get_logger().info(
            P.format_status_line(self._behavior_mode,
                                 self._manager.ray_behavior.current_target,
                                 completed),
            throttle_duration_sec=2.0)
        self._maybe_dump_results()

    def _finish(self, ctx: TickContext) -> None:
        """Coverage threshold tripped: hover, publish `complete`, dump."""
        self._maybe_dump_results(force=True)
        hover = ctx.clamp(ctx.cur_pose)
        self._path_pub.publish(ros_io.make_path(self._stamp(), [hover]))
        self._waypoint_locked = True
        self._target_waypoint = hover
        self._target_waypoint2 = hover
        self._nav_mode_pub.publish(String(data=P.NAV_MODE_TAG['Complete']))
        completed = self._detections.completed_labels()
        self._completed_pub.publish(String(data=json.dumps(completed)))


def main(args=None):
    rclpy.init(args=args)
    node = RavenNavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._maybe_dump_results(force=True)
        except Exception:                                 # noqa: BLE001
            pass
        node.destroy_node()
        # rclpy's OWN SIGINT handler has already shut the default context down
        # by the time spin() returns, so an unconditional second shutdown
        # raises `RCLError: rcl_shutdown already called on the given context`
        # — which used to escape main() as a traceback and a non-zero exit on
        # every clean Ctrl-C / `docker stop` / spawner teardown, i.e. a normal
        # end-of-mission looked like a planner crash to whoever reaped it.
        # Measured in the robot container after SIGINT: spin() raises
        # KeyboardInterrupt, rclpy.ok() is False, destroy_node() is fine,
        # rclpy.shutdown() throws. Found by the e2e chain smoke
        # (test/integration/test_e2e_shared_rayfronts.py).
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
