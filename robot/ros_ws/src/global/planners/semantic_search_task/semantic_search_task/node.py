import os
import queue
import re
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Point, Polygon, PolygonStamped, Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Empty
from action_msgs.srv import CancelGoal
from task_msgs.action import SemanticSearchTask, ExplorationTask

# ── ANSI / ROS log stripping ──────────────────────────────────────────────────

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')
_ROS_PREFIX_RE = re.compile(
    r'^\s*\[?(INFO|WARN|ERROR|DEBUG)\]?\s*\[\d+\.\d+\]\s*\[[^\]]+\]:\s*')


def _clean(line: str) -> str:
    line = _ANSI_RE.sub('', line)
    line = _ROS_PREFIX_RE.sub('', line)
    return line.strip()


# ── Output filters ────────────────────────────────────────────────────────────

def _filter_rayfronts(line: str) -> str | None:
    """Return a short human-readable string for key rayfronts events, else None."""
    raw = line
    line = _clean(line)
    low = line.lower()

    if 'error' in low or 'exception' in low or 'traceback' in low:
        return f'ERROR: {line}'
    if 'downloading' in low:
        return 'Downloading model weights...'
    if 'parsing model identifier' in low or 'loaded built-in' in low:
        return 'Loading model...'
    if 'subscribing to' in low or 'subscribed to' in low:
        return f'Subscribing: {line}'
    if 'received queries' in low:
        return f'Encoding queries: {line}'
    if 'mapping' in low and ('start' in low or 'running' in low or 'loop' in low):
        return 'Mapping started'
    if 'process started' in low or 'process has died' in low:
        return line
    # Skip noise: warnings, deprecations, progress bars, blank lines
    if not line or 'warning' in low or 'warn' in low or 'deprecated' in low:
        return None
    if '|' in raw and '█' in raw:   # tqdm progress bar
        return None
    return None


def _filter_raven(line: str) -> str | None:
    """Reformat raven's status line; skip everything else."""
    line = _clean(line)
    if not line:
        return None
    low = line.lower()
    if '[coord]' in line:
        return line
    if 'raven_nav started' in low:
        return 'raven_nav started'
    if 'waiting for odometry' in low:
        return 'Waiting for odometry...'
    if 'boot gps captured' in low or 'search_area' in low:
        return line
    if 'error' in low or 'exception' in low or 'traceback' in low:
        return f'ERROR: {line}'
    if re.search(r'\[(Frontier-based|Ray-based|Voxel-based)\]', line):
        return line
    return None


# ── Subprocess helpers ────────────────────────────────────────────────────────

def _pipe_to_queue(proc: subprocess.Popen, q: queue.Queue,
                   log_path: str | None = None) -> None:
    """Stream subprocess stdout into a queue, and optionally tee unfiltered to a file.

    The action feedback path runs every line through _filter_rayfronts /
    _filter_raven, which drops most output (tqdm progress bars, INFO logs that
    don't match the whitelist, etc.). The tee preserves everything for
    debugging — `tail -f /tmp/rayfronts_<robot>.log` shows raw stdout/stderr.
    """
    log_fh = None
    if log_path:
        try:
            log_fh = open(log_path, 'w', buffering=1)  # line-buffered
        except OSError:
            log_fh = None
    try:
        for line in iter(proc.stdout.readline, b''):
            decoded = line.decode('utf-8', errors='replace').rstrip()
            q.put(decoded)
            if log_fh is not None:
                try:
                    log_fh.write(decoded + '\n')
                except OSError:
                    pass
        q.put(None)
    finally:
        if log_fh is not None:
            try:
                log_fh.close()
            except OSError:
                pass


def _drain(q: queue.Queue) -> list:
    lines = []
    while True:
        try:
            item = q.get_nowait()
            if item is None:
                break
            lines.append(item)
        except queue.Empty:
            break
    return lines


def _sanitize(label: str) -> str:
    return re.sub(r'[^a-zA-Z0-9_]', '_', label)


# ── Node ──────────────────────────────────────────────────────────────────────

class SemanticSearchTaskNode(Node):
    def __init__(self):
        super().__init__('semantic_search_task')
        robot_name = os.getenv('ROBOT_NAME', 'robot_1')
        ros_domain = os.getenv('ROS_DOMAIN_ID', '0')
        self._robot_prefix = f'/{robot_name}'
        self._rf_prefix = f'/robot_{ros_domain}/rayfronts/msg_serv'

        self._cbg = ReentrantCallbackGroup()
        self._task_active = False
        self._cur_pos = None

        self._text_query_pub = self.create_publisher(
            String, f'{self._rf_prefix}/new_text_query', 10)

        # TRANSIENT_LOCAL so raven_nav (spawned after this node) gets the
        # most recent polygon on subscribe. Cleared at task end to avoid
        # carrying a stale polygon into the next task.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._search_area_pub = self.create_publisher(
            PolygonStamped, f'{self._robot_prefix}/raven_nav/search_area',
            latched_qos)
        self._clear_blacklist_pub = self.create_publisher(
            Empty, f'{self._robot_prefix}/raven_nav/clear_blacklist', 10)

        self.create_subscription(
            Odometry, f'{self._robot_prefix}/odometry',
            self._odom_cb, 10, callback_group=self._cbg)

        self._action_server = ActionServer(
            self, SemanticSearchTask, '~/semantic_search_task',
            execute_callback=self._execute,
            goal_callback=self._handle_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cbg)

        self._cleanup_existing()
        self.get_logger().info('semantic_search_task ready')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._cur_pos = [p.x, p.y, p.z]

    def _publish_search_area(self, polygon: Polygon) -> None:
        """Republish the polygon as a stamped, frame-tagged message for raven_nav.

        action_relay has already transformed the vertices into the robot's local
        'map' frame, so we just stamp and forward. An empty polygon clears the
        constraint downstream.
        """
        stamped = PolygonStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'map'
        stamped.polygon = polygon
        self._search_area_pub.publish(stamped)

    def _handle_goal(self, goal_request):
        if self._task_active:
            self.get_logger().warn('Rejecting goal: task already active')
            return GoalResponse.REJECT
        self._task_active = True
        return GoalResponse.ACCEPT

    def _cleanup_existing(self) -> None:
        """Kill any leftover rayfronts or raven processes and cancel any
        active navigation goals so a fresh ExplorationTask isn't rejected
        by droan (`task already active`) or delivered to a stale executor.

        random_walk_planner is intentionally NOT killed here: it runs as a
        long-lived task executor under global_bringup, and pkill'ing it
        would tear down the ExplorationTask action server every time this
        node initializes or starts a goal.
        """
        for pattern in ['rayfronts.mapping_server', 'raven_nav_node']:
            result = subprocess.run(
                ['pkill', '-SIGTERM', '-f', pattern], capture_output=True)
            if result.returncode == 0:
                self.get_logger().info(f'Killed existing {pattern} process(es)')
        time.sleep(2.0)
        for pattern in ['rayfronts.mapping_server', 'raven_nav_node']:
            subprocess.run(['pkill', '-SIGKILL', '-f', pattern], capture_output=True)

        robot_name = os.getenv('ROBOT_NAME', 'robot_1')
        self._cancel_active_navigation(robot_name)

    def _cancel_active_navigation(self, robot_name: str) -> None:
        """Cancel-all on ExplorationTask and NavigateTask servers.

        Uses the action's underlying _action/cancel_goal service with a
        zero goal_id, which the action spec defines as cancel-all.
        """
        for action in (f'/{robot_name}/tasks/exploration',
                       f'/{robot_name}/tasks/navigate'):
            srv_name = f'{action}/_action/cancel_goal'
            client = self.create_client(
                CancelGoal, srv_name, callback_group=self._cbg)
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f'No cancel service at {srv_name} (server not up — skipping)')
                self.destroy_client(client)
                continue
            req = CancelGoal.Request()
            future = client.call_async(req)
            try:
                deadline = time.time() + 2.0
                while not future.done() and time.time() < deadline:
                    time.sleep(0.05)
                resp = future.result() if future.done() else None
                n = len(resp.goals_canceling) if resp is not None else 0
                self.get_logger().info(
                    f'Cancelled {n} active goal(s) on {action}')
            except Exception as e:
                self.get_logger().warn(f'cancel-all on {action} failed: {e}')
            self.destroy_client(client)

    def _spawn(self, cmd: list, log_name: str | None = None) -> tuple:
        """Spawn a subprocess; if log_name is given, tee unfiltered stdout to
        /tmp/<log_name>_<robot>.log for debugging (filter still drives feedback).
        """
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True)
        q = queue.Queue()
        log_path = None
        if log_name:
            robot_name = os.getenv('ROBOT_NAME', 'robot')
            log_path = f'/tmp/{log_name}_{_sanitize(robot_name)}.log'
            self.get_logger().info(f'Teeing {log_name} stdout to {log_path}')
        threading.Thread(
            target=_pipe_to_queue,
            args=(proc, q, log_path),
            daemon=True,
        ).start()
        return proc, q

    def _kill(self, name: str, proc: subprocess.Popen) -> None:
        if proc is None:
            return
        self.get_logger().info(f'Stopping {name} (pid={proc.pid})')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    def _send_exploration_task(self, robot_name: str, parent_goal):
        """Send an unbounded ExplorationTask goal to random_walk_planner to
        activate droan_gl. Altitude and speed bounds are forwarded from the
        parent SemanticSearchTask goal so the operator's limits flow through
        to the actual flight executor. Returns (client, send_future) so the
        caller can cancel the upstream goal when the semantic_search task
        ends."""
        client = ActionClient(
            self,
            ExplorationTask,
            f'/{robot_name}/tasks/exploration',
            callback_group=self._cbg)
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('ExplorationTask server not available after 10s')
            return None, None
        goal = ExplorationTask.Goal()
        goal.min_altitude_agl = parent_goal.min_altitude_agl
        goal.max_altitude_agl = parent_goal.max_altitude_agl
        goal.min_flight_speed = parent_goal.min_flight_speed
        goal.max_flight_speed = parent_goal.max_flight_speed
        send_future = client.send_goal_async(goal)
        self.get_logger().info(
            f'ExplorationTask sent to random_walk_planner '
            f'(alt {goal.min_altitude_agl:.1f}-{goal.max_altitude_agl:.1f} m AGL, '
            f'speed {goal.min_flight_speed:.1f}-{goal.max_flight_speed:.1f} m/s)')
        return client, send_future

    def _cancel_exploration_task(self, send_future):
        """Best-effort cancel of an ExplorationTask started via
        _send_exploration_task. Safe to call with None / not-yet-accepted."""
        if send_future is None or not send_future.done():
            return
        try:
            handle = send_future.result()
        except Exception:
            return
        if handle is None or not getattr(handle, 'accepted', False):
            return
        try:
            handle.cancel_goal_async()
            self.get_logger().info(
                'Cancelled upstream ExplorationTask (random_walk_planner)')
        except Exception as e:
            self.get_logger().warn(f'Failed to cancel ExplorationTask: {e}')

    def _interruptible_sleep(self, goal_handle, secs: float) -> bool:
        """Sleep for secs, waking early if cancel requested. Returns True if cancelled."""
        steps = max(1, int(secs / 0.1))
        for _ in range(steps):
            if goal_handle.is_cancel_requested:
                return True
            time.sleep(0.1)
        return False

    def _execute(self, goal_handle):
        goal = goal_handle.request
        queries = [q.strip() for q in goal.query.split(',') if q.strip()]
        if not queries:
            self._task_active = False
            goal_handle.abort()
            result = SemanticSearchTask.Result()
            result.success = False
            result.message = 'Empty query'
            return result

        bg_raw = [bq.strip() for bq in goal.background_queries.split(',')
                  if bq.strip()]
        if not bg_raw:
            self._task_active = False
            goal_handle.abort()
            result = SemanticSearchTask.Result()
            result.success = False
            result.message = (
                'background_queries is required but was not provided. '
                'Softmax normalization needs contrast classes (e.g. '
                '"building,tree,ground") to produce meaningful scores.')
            return result

        # Softmax across queries needs N >= 2 to produce discriminative
        # scores; with N=1 every score is 1.0.
        bg = [bq for bq in bg_raw if bq not in queries]
        all_queries = queries + bg

        if len(all_queries) < 2:
            self._task_active = False
            goal_handle.abort()
            result = SemanticSearchTask.Result()
            result.success = False
            result.message = (
                f'Need at least 2 total queries for softmax normalization, '
                f'got {len(all_queries)}. Add more background_queries that '
                f'differ from the target query.')
            return result

        self.get_logger().info(
            f'SemanticSearchTask | targets={queries} all_queries={all_queries}')

        # action_relay has already transformed vertices into the robot's
        # local 'map' frame.
        self._publish_search_area(goal.search_area)
        n_pts = len(goal.search_area.points)
        if n_pts >= 3:
            self.get_logger().info(
                f'search_area: {n_pts} vertices forwarded to raven_nav')
        else:
            self.get_logger().info(
                f'search_area: {n_pts} vertices (unconstrained search)')

        rayfronts_proc = raven_proc = None
        exploration_send_future = None
        rayfronts_q = raven_q = queue.Queue()

        STUCK_TIMEOUT_S = 5.0
        STUCK_DISTANCE_M = 0.3
        ESCALATE_AFTER_RESTARTS = 10
        last_motion_pos = None
        last_motion_time = None
        exploration_restarts = 0
        last_restart_time: 'float | None' = None

        last_rf_status = 'Starting rayfronts...'
        last_rv_status = 'Starting raven...'

        try:
            self._cleanup_existing()

            robot_name = os.getenv('ROBOT_NAME', 'robot_1')

            # Wait for old rayfronts DDS subscription to fully unregister.
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = SemanticSearchTask.Result()
                    result.success = False
                    result.message = 'Cancelled'
                    return result
                if self.count_subscribers(f'{self._rf_prefix}/new_text_query') == 0:
                    break
                time.sleep(0.5)

            # Wait for rayfronts to accumulate points before starting raven —
            # otherwise multiple drones pick the same frontier.
            all_labels_yaml = str(all_queries).replace("'", '"')
            target_labels_yaml = str(queries).replace("'", '"')

            rayfronts_proc, rayfronts_q = self._spawn([
                'ros2', 'launch', 'perception_bringup', 'rayfronts.launch.xml',
            ], log_name='rayfronts')

            mapping_batches_seen = 0
            required_batches = 8
            self.get_logger().info(
                f'Waiting for {required_batches} rayfronts mapping batches')
            while mapping_batches_seen < required_batches and rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._kill('rayfronts', rayfronts_proc)
                    goal_handle.canceled()
                    result = SemanticSearchTask.Result()
                    result.success = False
                    result.message = 'Cancelled'
                    return result
                for raw in _drain(rayfronts_q):
                    low = _clean(raw).lower()
                    if 'ms/batch' in low:
                        mapping_batches_seen += 1
                    msg = _filter_rayfronts(raw)
                    if msg:
                        last_rf_status = msg
                fb = SemanticSearchTask.Feedback()
                fb.status = f'[rayfronts] {last_rf_status}'
                goal_handle.publish_feedback(fb)
                time.sleep(0.2)
            self.get_logger().info(
                f'rayfronts processed {mapping_batches_seen} batches — '
                f'starting raven')

            raven_proc, raven_q = self._spawn([
                'ros2', 'run', 'raven_nav', 'raven_nav_node',
                '--ros-args',
                '-p', f'query_labels:={all_labels_yaml}',
                '-p', f'target_labels:={target_labels_yaml}',
                '-p', f'min_altitude_agl:={goal.min_altitude_agl}',
                '-p', f'max_altitude_agl:={goal.max_altitude_agl}',
                '-r', (f'/{robot_name}/odometry:='
                       f'/{robot_name}/odometry_conversion/odometry'),
            ], log_name='raven')


            best_conf = 0.0
            rayfronts_ready = False
            prev_rf_sub_count = 0
            mapping_started = False
            random_walk_started = False
            raven_published_waypoint = False
            completed_targets: set = set()

            # Discoveries deduped by instance_id (stable across ticks via
            # raven's _stable_id hash) and filtered to requested labels.
            queries_lower = {q.lower() for q in queries}
            max_instances = int(getattr(goal, 'max_instances', 0) or 0)
            discoveries_by_id: dict = {}

            nav_mode_complete = False

            # BEST_EFFORT QoS to match raven's publisher (some robots publish
            # global_plan as BEST_EFFORT, others RELIABLE; BEST_EFFORT accepts both).
            from nav_msgs.msg import Path
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            global_plan_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
            def _global_plan_cb(msg):
                nonlocal raven_published_waypoint
                if msg.poses:
                    if not raven_published_waypoint:
                        self.get_logger().info(
                            f'First raven global_plan received '
                            f'({len(msg.poses)} poses)')
                    raven_published_waypoint = True
            self.create_subscription(
                Path, f'/{robot_name}/global_plan',
                _global_plan_cb, global_plan_qos, callback_group=self._cbg)

            def _completed_targets_cb(msg):
                nonlocal completed_targets
                import json
                try:
                    completed_targets = set(json.loads(msg.data))
                except Exception:
                    pass
            self.create_subscription(
                String, f'/{robot_name}/completed_targets',
                _completed_targets_cb, 10, callback_group=self._cbg)

            def _discoveries_cb(msg):
                import json
                try:
                    items = json.loads(msg.data)
                except Exception:
                    return
                if not isinstance(items, list):
                    return
                for d in items:
                    lbl = str(d.get('label', '')).lower()
                    if queries_lower and lbl not in queries_lower:
                        continue
                    inst_id = str(d.get('instance_id', ''))
                    if not inst_id:
                        continue
                    discoveries_by_id[inst_id] = d
            self.create_subscription(
                String, f'/{robot_name}/raven_nav/discoveries',
                _discoveries_cb, 10, callback_group=self._cbg)

            # Single source of truth for "polygon explored".
            def _nav_mode_cb(msg):
                nonlocal nav_mode_complete
                if (msg.data or '').strip() == 'complete':
                    nav_mode_complete = True
            self.create_subscription(
                String, f'/{robot_name}/navigation_mode',
                _nav_mode_cb, 10, callback_group=self._cbg)

            def _vox_all_cb(msg):
                nonlocal best_conf, mapping_started
                msg_field_names = [f.name for f in msg.fields]
                sim_fields = sorted([f for f in msg_field_names if f.startswith('sim_')])
                if not sim_fields:
                    return
                fields = ('x', 'y', 'z') + tuple(sim_fields)
                from sensor_msgs_py import point_cloud2 as pc2
                pts = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
                if pts:
                    import numpy as _np
                    arr = _np.array([list(p) for p in pts], dtype=_np.float32)
                    conf = float(arr[:, 3:].max())
                    if conf > best_conf:
                        best_conf = conf
                    mapping_started = True
            self.create_subscription(
                PointCloud2, f'{self._rf_prefix}/voxels_sim/all',
                _vox_all_cb, 10, callback_group=self._cbg)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    # Return whatever we've accumulated so the operator still
                    # sees partial discoveries even on cancel.
                    from geometry_msgs.msg import Pose, PoseArray
                    cp = PoseArray()
                    cl: list = []
                    cbc = 0.0
                    for d in discoveries_by_id.values():
                        p = Pose()
                        p.position.x = float(d.get('cx', 0.0))
                        p.position.y = float(d.get('cy', 0.0))
                        p.position.z = float(d.get('cz', 0.0))
                        p.orientation.w = 1.0
                        cp.poses.append(p)
                        cl.append(str(d.get('label', '')))
                        cbc = max(cbc, float(d.get('confidence', 0.0)))
                    goal_handle.canceled()
                    result = SemanticSearchTask.Result()
                    result.success = False
                    result.found_poses = cp
                    result.found_labels = cl
                    result.confidence = max(best_conf, cbc)
                    result.objects_found = len(discoveries_by_id)
                    result.message = (
                        f'Cancelled — partial: {len(discoveries_by_id)} '
                        f'instance(s) found')
                    return result

                for raw in _drain(rayfronts_q):
                    msg = _filter_rayfronts(raw)
                    if msg:
                        last_rf_status = msg

                for raw in _drain(raven_q):
                    msg = _filter_raven(raw)
                    if msg:
                        last_rv_status = msg

                # Wait for raven's first waypoint before starting random_walk —
                # otherwise the drone drifts before raven has any semantic targets.
                if raven_published_waypoint and not random_walk_started:
                    random_walk_started = True
                    self._cancel_active_navigation(robot_name)
                    self.get_logger().info(
                        'Raven published first waypoint — sending ExplorationTask '
                        'to existing random_walk_planner')
                    _, exploration_send_future = self._send_exploration_task(
                        robot_name, goal)
                    last_motion_pos = list(self._cur_pos) if self._cur_pos else None
                    last_motion_time = time.time()

                if (random_walk_started and self._cur_pos is not None
                        and last_motion_time is not None):
                    now = time.time()
                    if last_motion_pos is None:
                        last_motion_pos = list(self._cur_pos)
                        last_motion_time = now
                    else:
                        dx = self._cur_pos[0] - last_motion_pos[0]
                        dy = self._cur_pos[1] - last_motion_pos[1]
                        dz = self._cur_pos[2] - last_motion_pos[2]
                        if (dx * dx + dy * dy + dz * dz) ** 0.5 > STUCK_DISTANCE_M:
                            last_motion_pos = list(self._cur_pos)
                            last_motion_time = now
                        elif now - last_motion_time > STUCK_TIMEOUT_S:
                            self.get_logger().warn(
                                f'Robot has not moved >{STUCK_DISTANCE_M:.1f} m in '
                                f'{STUCK_TIMEOUT_S:.1f}s — cancelling and re-sending '
                                f'ExplorationTask')
                            self._cancel_exploration_task(exploration_send_future)
                            self._cancel_active_navigation(robot_name)
                            time.sleep(0.5)
                            _, exploration_send_future = self._send_exploration_task(
                                robot_name, goal)
                            exploration_restarts += 1
                            last_restart_time = time.time()
                            last_motion_pos = list(self._cur_pos)
                            last_motion_time = time.time()

                            if exploration_restarts >= ESCALATE_AFTER_RESTARTS:
                                self.get_logger().error(
                                    f'[escalation] {exploration_restarts} exploration '
                                    f'restarts — clearing raven blacklist + restarting '
                                    f'random_walk_planner')
                                self._clear_blacklist_pub.publish(Empty())
                                try:
                                    subprocess.run(
                                        ['pkill', '-f', 'random_walk_planner'],
                                        check=False, timeout=2.0)
                                except Exception as e:
                                    self.get_logger().warn(
                                        f'[escalation] pkill failed: {e}')
                                exploration_restarts = 0

                # Resend queries whenever rayfronts' subscriber appears (initial
                # load AND any restart mid-task). All queries (target + background)
                # so softmax stays meaningful.
                rf_sub_count = self.count_subscribers(f'{self._rf_prefix}/new_text_query')
                if rf_sub_count > 0 and prev_rf_sub_count == 0:
                    rayfronts_ready = True
                    # DDS discovery race: count_subscribers flips to >0 before
                    # the pub/sub match is fully wired — the first publish can
                    # silently drop. Sleep, then pace publishes so rayfronts
                    # processes each query before the next arrives.
                    time.sleep(0.3)
                    for q in all_queries:
                        self._text_query_pub.publish(String(data=q))
                        time.sleep(0.1)
                    last_rf_status = f'Queries sent: {", ".join(all_queries)}'
                    self.get_logger().info(
                        f'Queries sent to rayfronts: {all_queries}')
                if rf_sub_count == 0 and prev_rf_sub_count > 0:
                    rayfronts_ready = False
                    self.get_logger().info('rayfronts subscriber lost — will resend on reconnect')
                prev_rf_sub_count = rf_sub_count

                from geometry_msgs.msg import Pose, PoseArray
                cur_poses = PoseArray()
                cur_labels: list = []
                cur_best_conf = 0.0
                for d in discoveries_by_id.values():
                    p = Pose()
                    p.position.x = float(d.get('cx', 0.0))
                    p.position.y = float(d.get('cy', 0.0))
                    p.position.z = float(d.get('cz', 0.0))
                    p.orientation.w = 1.0
                    cur_poses.poses.append(p)
                    cur_labels.append(str(d.get('label', '')))
                    c = float(d.get('confidence', 0.0))
                    if c > cur_best_conf:
                        cur_best_conf = c

                # Gated on random_walk_started so we don't trip before raven is online.
                now_ts = time.time()
                polygon_done = random_walk_started and nav_mode_complete

                hit_max = (max_instances > 0
                           and len(discoveries_by_id) >= max_instances)
                if hit_max or polygon_done:
                    goal_handle.succeed()
                    result = SemanticSearchTask.Result()
                    result.success = True
                    result.found_poses = cur_poses
                    result.found_labels = cur_labels
                    result.confidence = max(best_conf, cur_best_conf)
                    result.objects_found = len(discoveries_by_id)
                    if hit_max:
                        result.message = (
                            f'Reached max_instances={max_instances}: '
                            f'{len(discoveries_by_id)} instance(s) found')
                    else:
                        result.message = (
                            f'Polygon explored — {len(discoveries_by_id)} '
                            f'instance(s) found: {", ".join(sorted(set(cur_labels)))}')
                    return result

                if not rayfronts_ready:
                    status = f'[rayfronts] {last_rf_status}'
                elif not mapping_started:
                    status = (f'[rayfronts] {last_rf_status}\n'
                              f'[raven] {last_rv_status}')
                else:
                    status = f'[raven] {last_rv_status}'

                # Transient "just restarted" banner for 3s after a restart,
                # plus a running count once the banner clears.
                if last_restart_time is not None and (
                        time.time() - last_restart_time) < 3.0:
                    status = (
                        f'[failsafe] Restarted ExplorationTask — drone stuck '
                        f'(<{STUCK_DISTANCE_M:.1f} m in {STUCK_TIMEOUT_S:.1f}s)\n'
                        + status)
                if exploration_restarts > 0:
                    status += f'\n[exploration restarts: {exploration_restarts}]'

                fb = SemanticSearchTask.Feedback()
                fb.status = status
                fb.objects_found_so_far = len(discoveries_by_id)
                fb.best_confidence_so_far = max(best_conf, cur_best_conf)
                fb.current_found_poses = cur_poses
                fb.current_found_labels = cur_labels
                goal_handle.publish_feedback(fb)

                if self._interruptible_sleep(goal_handle, 1.0):
                    continue   # cancel was requested, loop back to check it

        finally:
            # Stop the bringup-launched random_walk_planner from wandering.
            self._cancel_exploration_task(exploration_send_future)
            self._kill('rayfronts', rayfronts_proc)
            self._kill('raven', raven_proc)
            # Clear the latched polygon so the next task isn't constrained
            # by this one's search_area.
            self._publish_search_area(Polygon())
            self._task_active = False

        goal_handle.abort()
        result = SemanticSearchTask.Result()
        result.success = False
        result.message = 'Node shutdown'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSearchTaskNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
