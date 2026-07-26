"""llm_nav: LLM-at-the-decision-level semantic search planner over RayFronts.

The LLM (Qwen3, local) is the decision maker; RayFronts is its perception.
RayFronts voxels/rays are converted to a text digest (top-k labels + scores
per tracked voxel instance and per ray group — see digest.py); the LLM
narrates what it is seeing and, when free, picks ONE instance (V#) or ray
lead (R#) to visit. That commitment is locked until completed (instance +
label: arrive within reach_m of the instance / ray endpoint, with a ray
commitment refining onto a matching instance that appears along its line).
While locked the LLM keeps digesting and narrating but cannot change goals.

Interfaces (robot = ROBOT_NAME, rf = /robot_{ROS_DOMAIN_ID}/rayfronts/msg_serv):
  sub  {rf}/voxels_sim/all     PointCloud2  x,y,z,sim_0..sim_{Q-1} (RDF)
  sub  {rf}/rays_sim/all       PointCloud2  x,y,z,theta,phi,sim_* (RDF)
  sub  {rf}/frontiers          PointCloud2  x,y,z[,counts] (RDF)
  sub  /{robot}/odometry       nav_msgs/Odometry (remapped to odometry_conversion)
  sub  /{robot}/raven_nav/search_area  PolygonStamped (latched, from task node)
  pub  {rf}/new_text_query     std_msgs/String (target + label bank)
  pub  /{robot}/global_plan    nav_msgs/Path (2 poses, frame 'map')
  pub  /{robot}/navigation_mode           std_msgs/String
  pub  /{robot}/llm_nav/llm_narration     std_msgs/String (what the LLM sees)
  pub  /{robot}/llm_nav/llm_decision      std_msgs/String (JSON decision)
  pub  /{robot}/llm_nav/digest            std_msgs/String (latest map digest)
  pub  /{robot}/raven_nav/discoveries     std_msgs/String (JSON, task-node compat)

Everything the LLM does — every prompt, response, decision, state transition,
and digest/waypoint function call — is logged as JSONL (see mission_log.py).
"""

import json
import math
import os
import re
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .digest import (DigestBuilder, aabb_surface_distance, compass_of,
                     nearest_point_in_polygon_xy, point_in_polygon_xy,
                     standoff_point)
from .llm_client import LLMClient
from .mission_log import MissionLog, log_call, set_active_log

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)


class Commitment:
    """The locked goal the drone must finish before the LLM may pick again."""

    def __init__(self, kind: str, ref_id: str, label: str,
                 waypoint: np.ndarray, ray_origin=None, ray_dir=None,
                 path_points=None):
        self.kind = kind                  # 'instance' | 'ray' | 'frontier' | 'survey'
        self.ref_id = ref_id              # V#, R#, compass sector, or 'survey'
        self.label = label
        self.waypoint = waypoint          # final goal point
        self.ray_origin = ray_origin      # set for kind='ray'
        self.ray_dir = ray_dir
        self.path_points = path_points    # survey circle waypoints
        self.extension = None             # wp + 2m past, raven-style segment
        self.passed_half = False          # survey: reached the far side
        self.ray_missing = 0              # digests with no matching ray evidence
        self.refined_instance_id = None   # set when a ray resolves to a V#
        self.t_commit = time.monotonic()
        self.best_dist = float('inf')
        self.t_best_dist = time.monotonic()

    def describe(self) -> str:
        if self.kind == 'instance':
            return f'flying to {self.ref_id} ({self.label})'
        if self.kind == 'survey':
            return 'surveying (climb + circle to scan the horizon)'
        if self.kind == 'frontier':
            return f'exploring unmapped area to the {self.ref_id}'
        if self.refined_instance_id:
            return (f'following ray lead {self.ref_id} ({self.label}), '
                    f'resolved to {self.refined_instance_id}')
        return f'following ray lead {self.ref_id} ({self.label})'


class LlmNavNode(Node):

    def __init__(self):
        super().__init__('llm_nav')
        self._robot_name = os.getenv('ROBOT_NAME', 'robot_1')
        self._prefix = f'/{self._robot_name}'
        domain = os.getenv('ROS_DOMAIN_ID', '1')
        self._rf_prefix = f'/robot_{domain}/rayfronts/msg_serv'

        # ── parameters ────────────────────────────────────────────────────────
        self._target = str(self.declare_parameter('target', 'house').value).strip()
        self._model_path = self.declare_parameter(
            'model_path', 'Qwen/Qwen3-1.7B').value
        enable_thinking = bool(self.declare_parameter('enable_thinking', False).value)
        max_new_tokens = int(self.declare_parameter('max_new_tokens', 384).value)
        quantization = str(self.declare_parameter('quantization', '4bit').value)
        self._min_altitude = float(self.declare_parameter('min_altitude_agl', 5.0).value)
        self._max_altitude = float(self.declare_parameter('max_altitude_agl', 20.0).value)
        self._digest_period = float(self.declare_parameter('digest_period_s', 5.0).value)
        self._narrate_period = float(self.declare_parameter('narrate_period_s', 12.0).value)
        floor_cos = float(self.declare_parameter('floor_cos', 0.12).value)
        min_instance_voxels = int(self.declare_parameter('min_instance_voxels', 5).value)
        max_instances_shown = int(self.declare_parameter('max_instances_shown', 15).value)
        max_ray_groups = int(self.declare_parameter('max_ray_groups', 8).value)
        ray_sector_deg = float(self.declare_parameter('ray_sector_deg', 30.0).value)
        assoc_radius = float(self.declare_parameter('assoc_radius_m', 8.0).value)
        assoc_max_range = float(self.declare_parameter('assoc_max_range_m', 40.0).value)
        self._ray_commit_dist = float(self.declare_parameter('ray_commit_dist_m', 15.0).value)
        self._standoff_m = float(self.declare_parameter('standoff_m', 5.0).value)
        self._reach_m = float(self.declare_parameter('reach_m', 3.0).value)
        self._progress_timeout = float(self.declare_parameter('progress_timeout_s', 90.0).value)
        self._progress_eps = float(self.declare_parameter('progress_epsilon_m', 2.0).value)
        self._blacklist_ttl = float(self.declare_parameter('blacklist_ttl_s', 120.0).value)
        self._survey_radius = float(self.declare_parameter('survey_radius_m', 8.0).value)
        self._survey_cooldown = float(self.declare_parameter('survey_cooldown_s', 120.0).value)
        self._llm_aliases = bool(self.declare_parameter('llm_aliases', True).value)
        self._llm_perception = bool(self.declare_parameter('llm_perception', True).value)
        self._retune_cooldown = float(self.declare_parameter('retune_cooldown_s', 60.0).value)
        label_bank = [str(l).strip() for l in self.declare_parameter(
            'label_bank', ['house', 'car', 'tree', 'road', 'grass']).value if str(l).strip()]
        surface_labels = [str(l).strip() for l in self.declare_parameter(
            'surface_labels', ['road', 'grass', 'sky']).value if str(l).strip()]
        # 'alias=canonical' entries: part labels cluster into their parent
        # object (one house = one instance, not roof/wall/house fragments).
        label_aliases = {}
        for entry in self.declare_parameter('label_aliases', ['roof=house']).value:
            if '=' in str(entry):
                alias, canonical = str(entry).split('=', 1)
                label_aliases[alias.strip()] = canonical.strip()
        results_dir = str(self.declare_parameter('results_dir', '').value) or '/tmp'

        # ── logging ───────────────────────────────────────────────────────────
        self._mlog = MissionLog(
            os.path.join(results_dir, f'llm_nav_{self._robot_name}_events.jsonl'),
            self._robot_name,
            sim_clock_fn=lambda: self.get_clock().now().nanoseconds / 1e9)
        set_active_log(self._mlog)

        # ── map + planner state (guarded by _map_lock) ────────────────────────
        self._map_lock = threading.Lock()
        self._vox_xyz = None
        self._vox_scores = None
        self._ray_origins = None
        self._ray_dirs = None
        self._ray_scores = None
        self._frontiers = None
        self._cur_pos = None
        self._heading_deg = 0.0
        self._search_poly: list = []
        self._commitment: 'Commitment | None' = None
        self._visited: list = []          # [{'instance_id','label','cx','cy','cz','confidence'}]
        self._blacklist: dict = {}        # instance id -> wall time until blocked
        self._blacklisted_dirs: list = [] # [np(3) unit dirs] from aborted ray commits
        self._detected_labels = None
        self._last_survey_end = 0.0       # wall time; drives the survey cooldown
        self._last_retune = 0.0           # wall time; drives the retune cooldown

        surface_lower = {s.lower() for s in surface_labels}
        object_labels = [l for l in label_bank if l.lower() not in surface_lower]
        self._label_bank = label_bank
        self._config_aliases = dict(label_aliases)
        self._all_queries = [self._target] + [
            l for l in label_bank if l.lower() != self._target.lower()]
        self._builder = DigestBuilder(
            target=self._target, object_labels=object_labels,
            surface_labels=surface_labels, floor_cos=floor_cos,
            min_instance_voxels=min_instance_voxels,
            max_instances_shown=max_instances_shown,
            max_ray_groups=max_ray_groups, ray_sector_deg=ray_sector_deg,
            assoc_radius_m=assoc_radius, assoc_max_range_m=assoc_max_range,
            label_aliases=label_aliases)

        self._stop = False
        self._prev_rf_sub_count = 0
        self._nav_mode = 'initializing'

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, f'{self._rf_prefix}/voxels_sim/all', self._vox_all_cb, 10)
        self.create_subscription(
            PointCloud2, f'{self._rf_prefix}/rays_sim/all', self._ray_all_cb, 10)
        self.create_subscription(
            PointCloud2, f'{self._rf_prefix}/frontiers', self._frontiers_cb, 10)
        # Sim odometry publishes BEST_EFFORT; a RELIABLE subscriber gets an
        # incompatible-QoS mismatch and silently receives nothing.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            Odometry, f'{self._prefix}/odometry', self._odom_cb, sensor_qos)
        # Task node publishes the search polygon here (latched); topic name kept
        # from raven so semantic_search_task needs no change.
        self.create_subscription(
            PolygonStamped, f'{self._prefix}/raven_nav/search_area',
            self._search_area_cb, LATCHED_QOS)

        self._text_query_pub = self.create_publisher(
            String, f'{self._rf_prefix}/new_text_query', 10)
        self._path_pub = self.create_publisher(
            Path, f'{self._prefix}/global_plan', 10)
        self._nav_mode_pub = self.create_publisher(
            String, f'{self._prefix}/navigation_mode', 10)
        self._narration_pub = self.create_publisher(
            String, f'{self._prefix}/llm_nav/llm_narration', 10)
        self._decision_pub = self.create_publisher(
            String, f'{self._prefix}/llm_nav/llm_decision', 10)
        self._digest_pub = self.create_publisher(
            String, f'{self._prefix}/llm_nav/digest', 10)
        # Topic name kept from raven so semantic_search_task's feedback/result
        # (found instances) works unchanged.
        self._discoveries_pub = self.create_publisher(
            String, f'{self._prefix}/raven_nav/discoveries', 10)
        # Tracked-instance AABBs + labels for Foxglove/RViz: all objects on
        # /instances, target-label-only on /target_instances (cleaner overlay).
        self._markers_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/llm_nav/instances', 10)
        self._target_markers_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/llm_nav/target_instances', 10)

        self._llm = LLMClient(self._model_path, self.get_logger(), self._mlog,
                              enable_thinking=enable_thinking,
                              max_new_tokens=max_new_tokens,
                              quantization=quantization)

        self.create_timer(0.5, self._tick)
        threading.Thread(target=self._llm.load, daemon=True).start()
        threading.Thread(target=self._worker_loop, daemon=True).start()

        self.get_logger().info(
            f'llm_nav started | robot={self._robot_name} | target={self._target} | '
            f'model={self._model_path} | bank={len(self._all_queries)} labels | '
            f'altitude=[{self._min_altitude}, {self._max_altitude}]')
        self._mlog.event('node_start', target=self._target,
                         model=self._model_path, queries=self._all_queries,
                         altitude=[self._min_altitude, self._max_altitude])

    # ── rayfronts input callbacks (RDF -> FLU, mirrors raven_nav) ─────────────

    def _detect_rayfronts_labels(self):
        """Authoritative sim_N column order from rayfronts' q{N}_<label> topics."""
        prefix = f'{self._rf_prefix}/rays_sim/'
        pat = re.compile(rf'^{re.escape(prefix)}q(\d+)_(.+)$')
        parsed = []
        for name, _ in self.get_topic_names_and_types():
            m = pat.match(name)
            if m:
                parsed.append((int(m.group(1)), m.group(2).replace('_', ' ')))
        if not parsed:
            return None
        parsed.sort()
        return [lbl for _, lbl in parsed]

    def _refresh_labels(self, n_sim_fields: int):
        cached = len(self._detected_labels) if self._detected_labels else 0
        if cached == n_sim_fields:
            return
        detected = self._detect_rayfronts_labels()
        if detected and detected != self._detected_labels:
            self._detected_labels = detected
            with self._map_lock:
                self._builder.set_labels(detected)
            self.get_logger().info(f'rayfronts labels detected: {detected}')
            self._mlog.event('labels_detected', labels=detected)

    @staticmethod
    def _sim_fields_of(msg) -> list:
        names = [f.name for f in msg.fields]
        return sorted([f for f in names if f.startswith('sim_')],
                      key=lambda s: int(s.split('_', 1)[1]))

    def _vox_all_cb(self, msg: PointCloud2):
        sim_fields = self._sim_fields_of(msg)
        if not sim_fields:
            return
        self._refresh_labels(len(sim_fields))
        fields = ('x', 'y', 'z') + tuple(sim_fields)
        pts = list(point_cloud2.read_points(msg, field_names=fields, skip_nans=True))
        if not pts:
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        rdf_xyz = arr[:, :3]
        flu_xyz = np.stack([rdf_xyz[:, 2], -rdf_xyz[:, 0], -rdf_xyz[:, 1]], axis=1)
        with self._map_lock:
            self._vox_xyz = flu_xyz
            self._vox_scores = arr[:, 3:]

    def _ray_all_cb(self, msg: PointCloud2):
        sim_fields = self._sim_fields_of(msg)
        if not sim_fields:
            return
        self._refresh_labels(len(sim_fields))
        fields = ('x', 'y', 'z', 'theta', 'phi') + tuple(sim_fields)
        pts = list(point_cloud2.read_points(msg, field_names=fields, skip_nans=True))
        if not pts:
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        rdf_orig = arr[:, :3]
        theta = np.deg2rad(arr[:, 3])
        phi = np.deg2rad(arr[:, 4])
        dx = np.cos(theta) * np.sin(phi)
        dy = np.sin(theta) * np.sin(phi)
        dz = np.cos(phi)
        rdf_dirs = np.stack([dx, dy, dz], axis=1)
        flu_orig = np.stack([rdf_orig[:, 2], -rdf_orig[:, 0], -rdf_orig[:, 1]], axis=1)
        flu_dirs = np.stack([rdf_dirs[:, 2], -rdf_dirs[:, 0], -rdf_dirs[:, 1]], axis=1)
        with self._map_lock:
            self._ray_origins = flu_orig
            self._ray_dirs = flu_dirs
            self._ray_scores = arr[:, 5:]

    def _frontiers_cb(self, msg: PointCloud2):
        pts = list(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            return
        arr = np.array([list(p) for p in pts], dtype=np.float32)
        flu = np.stack([arr[:, 2], -arr[:, 0], -arr[:, 1]], axis=1)
        with self._map_lock:
            self._frontiers = flu

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        with self._map_lock:
            self._cur_pos = np.array([p.x, p.y, p.z], dtype=np.float64)
            self._heading_deg = math.degrees(yaw)

    def _search_area_cb(self, msg: PolygonStamped):
        poly = [(p.x, p.y) for p in msg.polygon.points]
        with self._map_lock:
            self._search_poly = poly
        if len(poly) >= 3:
            self.get_logger().info(f'search_area: {len(poly)} vertices')
            self._mlog.event('search_area_set', n_vertices=len(poly))

    # ── 2 Hz tick: queries, path publishing, completion/watchdog ──────────────

    def _tick(self):
        if self._stop:
            return
        self._maybe_publish_queries()
        self._nav_mode_pub.publish(String(data=self._nav_mode))
        with self._map_lock:
            com = self._commitment
            cur = self._cur_pos
        if cur is not None:
            self._mark_passive_visits(cur)
        if com is None or cur is None:
            return
        self._check_ray_refinement(com, cur)
        if not self._validate_commitment(com):
            return
        self._publish_path(com, cur)
        self._check_completion(com, cur)
        self._check_watchdog(com, cur)

    def _validate_commitment(self, com: Commitment) -> bool:
        """Continuous (2 Hz) validation of a followed instance against the
        freshest evidence — not just the digest-rate miss counter:

        1. SUPPORT: count voxels inside the committed bbox whose argmax is
           still the target group (live voxel cache, fresher than digests).
           A false positive re-scores away as the drone closes in; when the
           supporting voxels collapse, stop flying at it immediately.
        2. CONTAINMENT: if the committed bbox is now swallowed by a much
           larger different-label instance (house-box inside a tree-box),
           the target reading was canopy misfire — reject.

        Rejected instances are blacklisted (TTL) so the next decision can't
        instantly re-pick them while the tracker catches up."""
        inst_id = (com.ref_id if com.kind == 'instance'
                   else com.refined_instance_id)
        if inst_id is None:
            return True
        with self._map_lock:
            inst = self._builder.instance(inst_id)
            vox_xyz = self._vox_xyz
            vox_scores = self._vox_scores
            cols = list(self._builder._canonical_groups.get(
                self._target.lower(), []))
            floor = self._builder.floor_cos
            min_vox = self._builder.min_instance_voxels
            others = [(o.label, o.bbox_min, o.bbox_max)
                      for o in self._builder._instances.values()
                      if o.id != inst_id and o.label != com.label]
        if inst is None:
            return True     # dissolution path in _check_completion handles it
        reason = None
        if vox_xyz is not None and cols:
            m = np.all((vox_xyz >= inst.bbox_min - 0.25)
                       & (vox_xyz <= inst.bbox_max + 0.25), axis=1)
            support = 0
            if m.any():
                sub = vox_scores[m]
                q = min(sub.shape[1], len(self._builder.labels))
                best = np.argmax(sub[:, :q], axis=1)
                best_s = sub[np.arange(len(sub)), best]
                support = int((np.isin(best, [c for c in cols if c < q])
                               & (best_s >= floor)).sum())
            if support < min_vox:
                reason = f'support collapsed ({support} target voxels left)'
        if reason is None:
            vol = float(np.prod(np.maximum(
                inst.bbox_max - inst.bbox_min, 0.1)))
            for lbl, bmin, bmax in others:
                if (np.all(inst.bbox_min >= bmin - 0.5)
                        and np.all(inst.bbox_max <= bmax + 0.5)
                        and float(np.prod(np.maximum(bmax - bmin, 0.1)))
                        >= 2.0 * vol):
                    reason = f'fully contained in a larger "{lbl}" instance'
                    break
        if reason is None:
            return True
        with self._map_lock:
            self._blacklist[inst_id] = time.time() + self._blacklist_ttl
            self._commitment = None
            self._nav_mode = 'deciding'
        self.get_logger().warn(
            f'[commit] REJECTED {inst_id} ({com.label}): {reason} — re-deciding')
        self._mlog.event('commitment_rejected', ref=com.ref_id,
                         instance=inst_id, label=com.label, reason=reason)
        return False

    def _mark_passive_visits(self, cur: np.ndarray):
        """'Within reach_m of a {target} = seen' applies to EVERY target
        instance the drone passes, not just the committed one — without this,
        a house flown past en route stays unvisited and gets 'revisited'
        (instantly completed) later."""
        target_l = self._target.lower()
        newly = []
        with self._map_lock:
            for iid, inst in self._builder._instances.items():
                if inst.visited or inst.label.lower() != target_l:
                    continue
                # Inherit visited across tracker identity breaks: memory is
                # SPATIAL — if a "new" instance sits on a location we already
                # visited, it is not a new target (its voxels still score
                # high forever; identity alone must not resurrect it).
                inherited = any(
                    np.all(np.array([v['cx'], v['cy'], v['cz']])
                           >= inst.bbox_min - 0.5)
                    and np.all(np.array([v['cx'], v['cy'], v['cz']])
                               <= inst.bbox_max + 0.5)
                    for v in self._visited)
                if inherited:
                    inst.visited = True
                    self.get_logger().info(
                        f'[visit] {iid} sits on an already-visited location '
                        f'— inherited visited')
                    self._mlog.event('instance_visited', instance=iid,
                                     label=self._target, inherited=True)
                    continue
                if aabb_surface_distance(
                        cur, inst.bbox_min, inst.bbox_max) <= self._reach_m:
                    inst.visited = True
                    conf = dict(inst.top_labels).get(inst.label, 0.0)
                    self._visited.append(dict(
                        instance_id=iid, label=inst.label,
                        cx=float(inst.centroid[0]), cy=float(inst.centroid[1]),
                        cz=float(inst.centroid[2]), confidence=float(conf)))
                    newly.append(iid)
            visited_snapshot = list(self._visited)
        for iid in newly:
            self.get_logger().info(
                f'[visit] passed within {self._reach_m:.0f}m of {iid} '
                f'({self._target}) — marked seen '
                f'({len(visited_snapshot)} visited)')
            self._mlog.event('instance_visited', instance=iid,
                             label=self._target, passive=True)
        if newly:
            self._discoveries_pub.publish(
                String(data=json.dumps(visited_snapshot)))

    def _maybe_publish_queries(self):
        """Register target + label bank with rayfronts whenever its query
        subscriber (re)appears — same DDS edge-trigger as semantic_search_task."""
        try:
            n = self.count_subscribers(f'{self._rf_prefix}/new_text_query')
        except Exception:
            return
        if n > 0 and self._prev_rf_sub_count == 0:
            def _send():
                time.sleep(0.3)  # DDS discovery race, see semantic_search_task
                for q in self._all_queries:
                    self._text_query_pub.publish(String(data=q))
                    time.sleep(0.1)
                self.get_logger().info(
                    f'Published {len(self._all_queries)} queries to rayfronts '
                    f'(target + label bank)')
                self._mlog.event('bank_published', queries=self._all_queries)
            threading.Thread(target=_send, daemon=True).start()
        self._prev_rf_sub_count = n

    @log_call
    def _publish_path(self, com: Commitment, cur: np.ndarray):
        # raven-style: [waypoint, waypoint + 2m along travel direction] — a
        # short segment AT the goal, never a line from the robot to the goal.
        # droan_gl *follows* whatever line it is given, so a long segment that
        # cuts through a building keeps pulling the drone back into the wall
        # it is avoiding (the observed stall/orbit). With only the stub at the
        # target, droan free-routes there with its own obstacle avoidance.
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        if com.path_points:                       # survey circle (short hops)
            pts = list(com.path_points)
        elif com.extension is not None:
            pts = (com.waypoint, com.extension)
        else:
            pts = (com.waypoint,)
        for pt in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = float(pt[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)

    def _update_ray_carrot(self, digest):
        """A ray commitment follows LIVE evidence, not a fixed endpoint.

        Rays are beyond-range evidence: as the drone flies, the map boundary
        (and the ray origins with it) advances, so the carrot waypoint slides
        forward with the freshest aligned ray group. The commitment ends by
        RESOLUTION (a matching instance appears -> refinement), DISSIPATION
        (the area got mapped and no target was there -> re-decide, no
        blacklist), or plain arrival as the fallback. ray_commit_dist_m is
        only the carrot length, not a terminal distance."""
        with self._map_lock:
            com = self._commitment
            cur = self._cur_pos
        if (com is None or com.kind != 'ray'
                or com.refined_instance_id is not None or cur is None):
            return
        best, best_cos = None, math.cos(math.radians(30.0))
        for g in digest.ray_groups:
            if g.label.lower() != com.label.lower() or g.visited_target:
                continue
            c = float(np.dot(g.mean_dir, com.ray_dir))
            if c > best_cos:
                best, best_cos = g, c
        if best is None:
            com.ray_missing += 1
            if com.ray_missing < 3:
                return
            # Evidence gone and nothing resolved: the pointed-at region got
            # mapped and held no target. Release WITHOUT blacklisting — any
            # instances found there are already normal candidates.
            with self._map_lock:
                self._commitment = None
                self._nav_mode = 'deciding'
            self.get_logger().info(
                f'[commit] ray {com.ref_id} ({com.label}) evidence dissipated '
                f'— area mapped, no target resolved; re-deciding')
            self._mlog.event('commitment_dissipated', ref=com.ref_id,
                             label=com.label)
            return
        com.ray_missing = 0
        new_wp = self._ray_waypoint(best, cur)
        advance = float((new_wp - com.waypoint) @ com.ray_dir)
        if advance > 1.0:
            with self._map_lock:
                com.ray_origin = best.mean_origin.copy()
                com.ray_dir = best.mean_dir.copy()
                com.waypoint = new_wp
                com.extension = self._extension_point(cur, new_wp)
                com.best_dist = float('inf')
                com.t_best_dist = time.monotonic()
            self._mlog.event('ray_carrot_advanced', ref=com.ref_id,
                             advance_m=round(advance, 1),
                             waypoint=[round(float(v), 1) for v in new_wp])

    def _check_ray_refinement(self, com: Commitment, cur: np.ndarray):
        """A ray commitment resolves onto an instance of the committed label
        that appears near the committed line — same commitment, better goal."""
        if com.kind != 'ray' or com.refined_instance_id is not None:
            return
        with self._map_lock:
            candidates = [
                (iid, inst) for iid, inst in self._builder._instances.items()
                if inst.label.lower() == com.label.lower() and not inst.visited]
        best_id, best_inst, best_d = None, None, self._builder.assoc_radius_m
        for iid, inst in candidates:
            v = inst.centroid - com.ray_origin
            t = float(v @ com.ray_dir)
            if t <= 0:
                continue
            d = float(np.linalg.norm(v - t * com.ray_dir))
            if d < best_d:
                best_id, best_inst, best_d = iid, inst, d
        if best_id is None:
            return
        wp = self._instance_waypoint(best_inst, cur)
        with self._map_lock:
            com.refined_instance_id = best_id
            com.waypoint = wp
            com.extension = self._extension_point(cur, wp)
            com.best_dist = float('inf')
            com.t_best_dist = time.monotonic()
        self.get_logger().info(
            f'[commit] ray {com.ref_id} resolved to instance {best_id} '
            f'({com.label}) — refining waypoint')
        self._mlog.event('commitment_refined', ray=com.ref_id,
                         instance=best_id, label=com.label,
                         waypoint=[round(float(v), 1) for v in wp])

    def _check_completion(self, com: Commitment, cur: np.ndarray):
        # Survey: the circle's last point is near its first, so require the
        # far side of the circle to have been reached before arming arrival —
        # otherwise the camera never sweeps the full horizon.
        if com.kind == 'survey' and com.path_points and not com.passed_half:
            mid = com.path_points[len(com.path_points) // 2]
            if float(np.linalg.norm(cur - mid)) <= self._reach_m + 1.0:
                com.passed_half = True
            else:
                return
        done = False
        inst_id = com.ref_id if com.kind == 'instance' else com.refined_instance_id
        if inst_id is not None:
            with self._map_lock:
                inst = self._builder.instance(inst_id)   # follows merge redirects
            # Dissolution release: instances re-cluster from LIVE voxel scores
            # every digest, so a false positive (e.g. 'house' argmax inside a
            # tree) loses its voxels as closer views re-score them. When the
            # committed target has no support for 2 consecutive digests (or is
            # gone), stop flying at its ghost — without this, the only escape
            # was the 90s watchdog, stuck beside a tree.
            if inst is None or inst.misses >= 2:
                with self._map_lock:
                    self._commitment = None
                    self._nav_mode = 'deciding'
                self.get_logger().info(
                    f'[commit] target {inst_id} dissolved (voxels re-scored '
                    f'away from "{com.label}") — re-deciding')
                self._mlog.event('commitment_dissolved', ref=com.ref_id,
                                 instance=inst_id, label=com.label,
                                 gone=inst is None)
                return
            if aabb_surface_distance(
                    cur, inst.bbox_min, inst.bbox_max) <= self._reach_m:
                done = True
        if not done and float(np.linalg.norm(cur - com.waypoint)) <= self._reach_m:
            done = True
        if not done:
            return
        self._complete_commitment(com, cur, inst_id)

    def _complete_commitment(self, com: Commitment, cur, inst_id):
        with self._map_lock:
            inst = self._builder.instance(inst_id) if inst_id else None
            if inst is not None:
                already_visited = inst.visited
                self._builder.mark_visited(inst_id)
                # Don't double-count an instance re-reached via a stale pick
                # or a merged fragment.
                if not already_visited:
                    conf = dict(inst.top_labels).get(inst.label, 0.0)
                    self._visited.append(dict(
                        instance_id=inst.id, label=inst.label,
                        cx=float(inst.centroid[0]), cy=float(inst.centroid[1]),
                        cz=float(inst.centroid[2]), confidence=float(conf)))
            self._commitment = None
            self._nav_mode = 'deciding'
            visited_snapshot = list(self._visited)
        if com.kind == 'survey':
            self._last_survey_end = time.time()
        self.get_logger().info(
            f'[commit] COMPLETE: {com.describe()} '
            f'({len(visited_snapshot)} visited so far)')
        self._mlog.event('commitment_complete', kind=com.kind, ref=com.ref_id,
                         label=com.label, instance=inst_id,
                         n_visited=len(visited_snapshot))
        if inst_id is not None:
            self._mlog.event('instance_visited', instance=inst_id, label=com.label)
        self._discoveries_pub.publish(
            String(data=json.dumps(visited_snapshot)))

    def _check_watchdog(self, com: Commitment, cur: np.ndarray):
        d = float(np.linalg.norm(cur - com.waypoint))
        now = time.monotonic()
        if d < com.best_dist - self._progress_eps:
            com.best_dist = d
            com.t_best_dist = now
            return
        if now - com.t_best_dist < self._progress_timeout:
            return
        # No progress — abort and blacklist so the next decision avoids it.
        with self._map_lock:
            inst_id = (com.ref_id if com.kind == 'instance'
                       else com.refined_instance_id)
            if inst_id is not None:
                self._blacklist[inst_id] = time.time() + self._blacklist_ttl
            elif com.ray_dir is not None:
                self._blacklisted_dirs.append(np.asarray(com.ray_dir).copy())
            if com.kind == 'survey':
                # A failed survey still burns the cooldown so we don't loop.
                self._last_survey_end = time.time()
            self._commitment = None
            self._nav_mode = 'deciding'
        self.get_logger().warn(
            f'[commit] TIMEOUT (no progress {self._progress_timeout:.0f}s, '
            f'{d:.1f}m away): {com.describe()} — blacklisted, re-deciding')
        self._mlog.event('commitment_timeout', kind=com.kind, ref=com.ref_id,
                         label=com.label, dist_left=round(d, 1),
                         blacklisted=inst_id or 'direction')

    # ── waypoint geometry ─────────────────────────────────────────────────────

    @log_call
    def _instance_waypoint(self, inst, cur: np.ndarray) -> np.ndarray:
        wp = standoff_point(cur, inst.centroid, inst.bbox_min, inst.bbox_max,
                            self._standoff_m)
        wp[2] = float(min(max(wp[2], self._min_altitude), self._max_altitude))
        return wp

    def _survey_allowed(self) -> bool:
        return time.time() - self._last_survey_end > self._survey_cooldown

    @log_call
    def _extension_point(self, cur: np.ndarray, wp: np.ndarray) -> np.ndarray:
        """2m past the waypoint along the commit-time travel direction —
        raven's segment recipe (frontier_behavior: wp + 2.0*dir)."""
        v = wp - cur
        d = float(np.linalg.norm(v))
        if d < 1e-6:
            return wp.copy()
        return wp + (v / d) * 2.0

    @log_call
    def _survey_circle(self, cur: np.ndarray) -> list:
        """Circle waypoints around the current position at max altitude.
        droan_gl yaws along the velocity direction, so flying the circle
        sweeps the camera through the full horizon and casts semantic rays
        at everything distant."""
        with self._map_lock:
            poly = list(self._search_poly)
        alt = self._max_altitude
        base = math.radians(self._heading_deg)
        pts = []
        for k in range(1, 9):                       # 45° steps, full loop
            a = base + k * math.pi / 4.0
            x = cur[0] + self._survey_radius * math.cos(a)
            y = cur[1] + self._survey_radius * math.sin(a)
            if len(poly) >= 3 and not point_in_polygon_xy(x, y, poly):
                x, y = nearest_point_in_polygon_xy(x, y, poly)
            pts.append(np.array([x, y, alt], dtype=np.float64))
        return pts

    @log_call
    def _frontier_waypoint(self, sector_target: np.ndarray) -> np.ndarray:
        wp = np.asarray(sector_target, dtype=np.float64).copy()
        with self._map_lock:
            poly = list(self._search_poly)
        if len(poly) >= 3 and not point_in_polygon_xy(wp[0], wp[1], poly):
            nx, ny = nearest_point_in_polygon_xy(wp[0], wp[1], poly)
            wp[0], wp[1] = nx, ny
        wp[2] = float(min(max(wp[2], self._min_altitude), self._max_altitude))
        return wp

    @log_call
    def _ray_waypoint(self, group, cur: np.ndarray) -> np.ndarray:
        wp = group.mean_origin + group.mean_dir * self._ray_commit_dist
        with self._map_lock:
            poly = list(self._search_poly)
        if len(poly) >= 3 and not point_in_polygon_xy(wp[0], wp[1], poly):
            nx, ny = nearest_point_in_polygon_xy(wp[0], wp[1], poly)
            wp = np.array([nx, ny, wp[2]], dtype=np.float64)
        wp = np.asarray(wp, dtype=np.float64)
        wp[2] = float(min(max(wp[2], self._min_altitude), self._max_altitude))
        return wp

    # ── LLM worker: digest -> narrate/decide -> commit ────────────────────────

    def _worker_loop(self):
        last_digest_t = 0.0
        last_narrate_t = 0.0
        digest = None
        aliases_done = False
        while rclpy.ok() and not self._stop:
            time.sleep(0.5)
            if not self._llm.ready:
                continue
            if not aliases_done:
                aliases_done = True
                self._bootstrap_aliases()
                self._bootstrap_perception()
            with self._map_lock:
                have_map = (self._builder.ready and self._vox_xyz is not None
                            and self._cur_pos is not None)
            if not have_map:
                continue
            now = time.monotonic()

            if now - last_digest_t >= self._digest_period:
                digest = self._refresh_digest()
                last_digest_t = now
                if digest is not None:
                    self._update_ray_carrot(digest)
            if digest is None:
                continue

            with self._map_lock:
                committed = self._commitment is not None

            if not committed:
                # Rebuild RIGHT BEFORE deciding: a completion may have landed
                # since the periodic build, and deciding from a stale digest
                # re-picks just-visited instances (seen in the first PoC run:
                # V17/V18/V22 each committed twice).
                digest = self._refresh_digest()
                last_digest_t = now
                if digest is None:
                    continue
                if (digest.selectable_instances or digest.selectable_rays
                        or digest.frontier_targets or self._survey_allowed()):
                    self._nav_mode = 'deciding'
                    self._decide_and_commit(digest)
                    # Force a digest rebuild next cycle so STATUS reflects
                    # the new commitment.
                    last_digest_t = 0.0
                    last_narrate_t = now
            elif now - last_narrate_t >= self._narrate_period:
                parsed = self._llm.narrate(digest.text, self._target)
                if parsed is not None:
                    seeing = parsed.get('seeing', '')
                    self._narration_pub.publish(String(data=seeing))
                    self.get_logger().info(f'[LLM seeing] {seeing}')
                    self._mlog.event('narrate', seeing=seeing,
                                     notes=parsed.get('notes', ''))
                last_narrate_t = now

    def _bootstrap_perception(self):
        """One-shot LLM call: clustering params fitted to the target. The
        config values are only the pre-LLM fallback — no fixed thresholds."""
        if not self._llm_perception:
            return
        with self._map_lock:
            current = dict(min_voxels=self._builder.min_instance_voxels,
                           score_floor=self._builder.floor_cos,
                           merge_gap_m=self._builder.merge_gap_m)
        params = self._llm.perception_params(self._target, current)
        if params is None:
            self.get_logger().warn(
                f'LLM perception-params call failed — keeping {current}')
            return
        with self._map_lock:
            self._builder.set_perception_params(
                params['min_voxels'], params['score_floor'],
                params['merge_gap_m'])
        self.get_logger().info(
            f'[perception] LLM set clustering for "{self._target}": '
            f'min_voxels={params["min_voxels"]} '
            f'score_floor={params["score_floor"]:.2f} '
            f'merge_gap={params["merge_gap_m"]:.1f}m — {params["reason"]}')
        self._mlog.event('perception_params_set', **params)

    def _clustering_stats(self) -> str:
        """Short text summary of the current instance set, fed back to the
        LLM when it retunes."""
        with self._map_lock:
            insts = list(self._builder._instances.values())
        if not insts:
            return 'no instances at all'
        by_label = {}
        for i in insts:
            size = i.bbox_max - i.bbox_min
            by_label.setdefault(i.label, []).append(
                f'{size[0]:.0f}x{size[1]:.0f}m/{i.n_voxels}vox')
        return '; '.join(f'{lbl}: {len(v)} instances ({", ".join(v[:6])})'
                         for lbl, v in by_label.items())

    def _bootstrap_aliases(self):
        """One-shot LLM call: which bank labels are PARTS of the target?
        Feeds the same alias mechanism as the config (config entries kept as
        the base, LLM additions merged in; config-only on failure) — so
        part-of-object grouping needs no per-target hand editing."""
        if not self._llm_aliases:
            return
        amap = self._llm.part_aliases(self._target, self._label_bank)
        if amap is None:
            self.get_logger().warn(
                'LLM part-alias call failed — keeping config aliases '
                f'{self._config_aliases}')
            return
        merged = dict(self._config_aliases)
        merged.update(amap)
        with self._map_lock:
            self._builder.set_label_aliases(merged)
        self.get_logger().info(f'[aliases] parts of "{self._target}": '
                               f'{sorted(amap)} (merged: {merged})')
        self._mlog.event('aliases_set', llm_parts=sorted(amap), merged=merged)

    def _refresh_digest(self):
        """Build + publish + log a fresh digest (single path for both the
        periodic rebuild and the mandatory pre-decision rebuild)."""
        digest = self._build_digest()
        if digest is not None:
            self._digest_pub.publish(String(data=digest.text))
            self._mlog.event('digest_built', text=digest.text,
                             **{k: len(v) for k, v in
                                digest.structured().items()})
            self._publish_instance_markers()
        return digest

    def _publish_instance_markers(self):
        """AABB + label markers for every tracked instance — what the LLM's
        candidate list looks like in space. Colors: yellow = committed goal,
        green = selectable target-label, white = other object, blue =
        visited, red = blacklisted."""
        now = time.time()
        with self._map_lock:
            insts = list(self._builder._instances.values())
            com = self._commitment
            blacklist = dict(self._blacklist)
        committed_id = None
        if com is not None:
            committed_id = (com.ref_id if com.kind == 'instance'
                            else com.refined_instance_id)
        target_l = self._target.lower()
        arr = MarkerArray()
        target_arr = MarkerArray()
        for a in (arr, target_arr):
            wipe = Marker()
            wipe.action = Marker.DELETEALL
            a.markers.append(wipe)
        stamp = self.get_clock().now().to_msg()
        for inst in insts:
            try:
                mid = int(inst.id[1:])
            except ValueError:
                mid = abs(hash(inst.id)) % 100000
            if inst.id == committed_id:
                rgba = (1.0, 0.85, 0.1, 0.5)
            elif blacklist.get(inst.id, 0.0) > now:
                rgba = (0.9, 0.2, 0.2, 0.35)
            elif inst.visited:
                rgba = (0.3, 0.5, 0.9, 0.3)
            elif inst.label.lower() == target_l:
                rgba = (0.2, 0.9, 0.3, 0.4)
            else:
                rgba = (0.8, 0.8, 0.8, 0.25)
            center = (inst.bbox_min + inst.bbox_max) / 2.0
            size = np.maximum(inst.bbox_max - inst.bbox_min, 0.5)
            box = Marker()
            box.header.frame_id = 'map'
            box.header.stamp = stamp
            box.ns = 'instances'
            box.id = mid
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = \
                (float(v) for v in center)
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = (float(v) for v in size)
            box.color.r, box.color.g, box.color.b, box.color.a = rgba
            arr.markers.append(box)
            txt = Marker()
            txt.header = box.header
            txt.ns = 'labels'
            txt.id = mid
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(center[0])
            txt.pose.position.y = float(center[1])
            txt.pose.position.z = float(inst.bbox_max[2]) + 1.0
            txt.pose.orientation.w = 1.0
            txt.scale.z = 1.2
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 0.9
            top = inst.top_labels[0] if inst.top_labels else (inst.label, 0.0)
            flags = ' [VISITED]' if inst.visited else ''
            txt.text = (f'{inst.id} {inst.label} '
                        f'{top[1] * 100:.0f} ({inst.n_voxels}vox){flags}')
            arr.markers.append(txt)
            if inst.label.lower() == target_l:
                target_arr.markers.append(box)
                target_arr.markers.append(txt)
        self._markers_pub.publish(arr)
        self._target_markers_pub.publish(target_arr)

    def _build_digest(self):
        with self._map_lock:
            vox_xyz = self._vox_xyz
            vox_scores = self._vox_scores
            ray_o, ray_d, ray_s = (self._ray_origins, self._ray_dirs,
                                   self._ray_scores)
            frontiers = self._frontiers
            cur = self._cur_pos
            heading = self._heading_deg
            com = self._commitment
            blacklist = dict(self._blacklist)
            bdirs = list(self._blacklisted_dirs)
        if vox_xyz is None or cur is None:
            return None
        try:
            with self._map_lock:
                self._builder.update_instances(vox_xyz, vox_scores)
                merges = list(self._builder.last_merges)
                visited_pts = [np.array([v['cx'], v['cy'], v['cz']])
                               for v in self._visited]
                status = (com.describe() + f', {np.linalg.norm(cur - com.waypoint):.0f}m to go'
                          ) if com is not None else 'choosing the next goal'
                digest = self._builder.build(
                    cur, heading, status, ray_o, ray_d, ray_s,
                    frontiers, blacklist, bdirs, visited_points=visited_pts)
            if merges:
                self.get_logger().info(f'[track] merged instances: {merges}')
                self._mlog.event('instances_merged', merges=merges)
            return digest
        except Exception:
            import traceback
            tb = traceback.format_exc()
            self.get_logger().error(f'digest build failed:\n{tb}')
            self._mlog.event('digest_error', traceback=tb)
            return None

    def _retune_allowed(self) -> bool:
        return time.time() - self._last_retune > self._retune_cooldown

    def _decide_and_commit(self, digest):
        parsed = self._llm.decide(
            digest.text, self._target,
            digest.selectable_instances, digest.selectable_rays,
            set(digest.frontier_targets), self._survey_allowed(),
            allow_retune=self._llm_perception and self._retune_allowed())
        fallback = False
        if parsed is None:
            # Deterministic fallback: strongest target-scoring candidate. Loud —
            # this means the LLM failed twice, which is a finding in itself.
            fallback = True
            parsed = self._fallback_decision(digest)
            if parsed is None:
                return
            self.get_logger().error(
                f'LLM decision failed twice — falling back to '
                f'{parsed["action"]["type"]} {parsed["action"]["id"]}')
        act = parsed['action']
        seeing = parsed.get('seeing', '')
        if seeing:
            self._narration_pub.publish(String(data=seeing))
            self.get_logger().info(f'[LLM seeing] {seeing}')
        self._decision_pub.publish(String(data=json.dumps(
            dict(**parsed, fallback=fallback))))

        if act['type'] == 'retune':
            # The LLM thinks the clustering is wrong (fragments/mega-blob):
            # let it re-set the perception params with the current stats,
            # then the worker loop rebuilds the digest and decides again.
            self._last_retune = time.time()
            current = dict(min_voxels=self._builder.min_instance_voxels,
                           score_floor=self._builder.floor_cos,
                           merge_gap_m=self._builder.merge_gap_m)
            params = self._llm.perception_params(
                self._target, current, stats=self._clustering_stats())
            if params is not None:
                with self._map_lock:
                    self._builder.set_perception_params(
                        params['min_voxels'], params['score_floor'],
                        params['merge_gap_m'])
                self.get_logger().info(
                    f'[perception] RETUNE: {current} -> {params}')
                self._mlog.event('perception_retuned', previous=current,
                                 **params)
            return

        with self._map_lock:
            cur = self._cur_pos
        if cur is None:
            return
        if act['type'] == 'goto_instance':
            with self._map_lock:
                inst = self._builder.instance(act['id'])
            if inst is None:
                self._mlog.event('commit_failed', reason='instance vanished',
                                 id=act['id'])
                return
            wp = self._instance_waypoint(inst, cur)
            com = Commitment('instance', act['id'], inst.label, wp)
        elif act['type'] == 'goto_ray':
            group = next((g for g in digest.ray_groups if g.id == act['id']), None)
            if group is None:
                self._mlog.event('commit_failed', reason='ray group vanished',
                                 id=act['id'])
                return
            wp = self._ray_waypoint(group, cur)
            com = Commitment('ray', act['id'], group.label, wp,
                             ray_origin=group.mean_origin.copy(),
                             ray_dir=group.mean_dir.copy())
        elif act['type'] == 'goto_frontier':
            tgt = digest.frontier_targets.get(act['id'])
            if tgt is None:
                self._mlog.event('commit_failed', reason='frontier sector gone',
                                 id=act['id'])
                return
            wp = self._frontier_waypoint(tgt)
            com = Commitment('frontier', act['id'], 'unexplored', wp)
        else:  # survey
            pts = self._survey_circle(cur)
            wp = pts[-1]
            com = Commitment('survey', 'survey', 'horizon scan', wp,
                             path_points=pts)
        com.extension = self._extension_point(cur, com.waypoint)
        with self._map_lock:
            self._commitment = com
            self._nav_mode = f'committed_{com.kind}'
        self.get_logger().info(
            f'[commit] {com.describe()} | waypoint '
            f'({wp[0]:.0f}, {wp[1]:.0f}, {wp[2]:.0f}) | '
            f'reason: {act.get("reason", "")}'
            + (' | FALLBACK' if fallback else ''))
        self._mlog.event('commitment_set', kind=com.kind, ref=com.ref_id,
                         label=com.label, fallback=fallback,
                         waypoint=[round(float(v), 1) for v in wp],
                         reason=act.get('reason', ''), seeing=seeing)

    def _fallback_decision(self, digest) -> 'dict | None':
        """Deterministic policy when the LLM fails twice. Never tours clutter:
        unvisited TARGET-labeled instance > target-labeled ray lead >
        survey > biggest frontier sector."""
        target_l = self._target.lower()

        def target_score(top_labels):
            return dict(top_labels).get(self._target, 0.0)
        insts = [i for i in digest.instances
                 if i.id in digest.selectable_instances
                 and i.label.lower() == target_l]
        if insts:
            best = max(insts, key=lambda i: target_score(i.top_labels))
            return dict(seeing='', action=dict(
                type='goto_instance', id=best.id,
                reason='fallback: strongest unvisited target instance'))
        rays = [g for g in digest.ray_groups
                if g.id in digest.selectable_rays
                and g.label.lower() == target_l]
        if rays:
            best = max(rays, key=lambda g: target_score(g.top_labels))
            return dict(seeing='', action=dict(
                type='goto_ray', id=best.id,
                reason='fallback: strongest target ray lead'))
        if self._survey_allowed():
            return dict(seeing='', action=dict(
                type='survey', id='',
                reason='fallback: no target candidates — scanning the horizon'))
        if digest.frontier_targets:
            # frontier_targets preserves count-descending order.
            sector = next(iter(digest.frontier_targets))
            return dict(seeing='', action=dict(
                type='goto_frontier', id=sector,
                reason='fallback: exploring the largest unmapped direction'))
        return None

    # ── shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self):
        self._stop = True
        self._mlog.event('node_shutdown', n_visited=len(self._visited))
        set_active_log(None)
        self._mlog.close()


def main(args=None):
    rclpy.init(args=args)
    node = LlmNavNode()

    import signal

    def _sigterm(_sig, _frm):
        # semantic_search_task kills us with SIGTERM on the process group;
        # flush the JSONL log before dying.
        node.shutdown()
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, _sigterm)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
