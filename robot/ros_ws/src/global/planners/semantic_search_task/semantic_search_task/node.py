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
from task_msgs.action import SemanticSearchTask, NavigateTask

# ── ANSI / ROS log stripping ──────────────────────────────────────────────────

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')
_ROS_PREFIX_RE = re.compile(
    r'^\s*\[?(INFO|WARN|ERROR|DEBUG)\]?\s*\[\d+\.\d+\]\s*\[[^\]]+\]:\s*')

# Shared bind-mounted dir (./cache -> /root/.cache in every robot replica), so
# all robots' raven dumps land where any container can compile them.
RESULTS_DIR = '/root/.cache/raven_results'
RESULTS_COVERAGE_THRESHOLD = 0.80  # the "80%" of "15 min OR 80%"
RESULTS_SCENE = os.getenv('RESULTS_SCENE', 'RetroNeighborhood')  # GT annotations


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


def _point_in_polygon_xy(x: float, y: float, poly: list) -> bool:
    """Ray-cast point-in-polygon test in the XY plane. poly is a list of (x, y)
    vertices. Fewer than 3 vertices means 'unbounded' → always inside."""
    n = len(poly)
    if n < 3:
        return True
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi):
            inside = not inside
        j = i
    return inside


def _nearest_point_in_polygon_xy(x: float, y: float, poly: list,
                                 inset_m: float = 1.0) -> tuple:
    """Closest point on the polygon boundary to (x, y) (projection onto the
    nearest edge, mirroring random_walk's nearest_inside_point), nudged inset_m
    toward the centroid so the result lands clearly inside rather than on the
    edge. Returns (x, y) unchanged if poly has < 3 vertices."""
    n = len(poly)
    if n < 3:
        return x, y
    best = (x, y)
    best_d2 = float('inf')
    for i in range(n):
        ax, ay = poly[i - 1]
        bx, by = poly[i]
        ex, ey = bx - ax, by - ay
        seg2 = ex * ex + ey * ey
        t = 0.0 if seg2 < 1e-9 else ((x - ax) * ex + (y - ay) * ey) / seg2
        t = max(0.0, min(1.0, t))
        px, py = ax + t * ex, ay + t * ey
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (px, py)
    cx = sum(p[0] for p in poly) / n
    cy = sum(p[1] for p in poly) / n
    dx, dy = cx - best[0], cy - best[1]
    d = (dx * dx + dy * dy) ** 0.5
    if d > 1e-6:
        f = min(inset_m, d) / d
        best = (best[0] + dx * f, best[1] + dy * f)
    return best


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

    def _finalize_metrics(self, reason: str) -> str:
        """Compile all robots' raven dumps + score vs ground truth on exit.

        Runs on cancel and on 80%-coverage success. Returns a compact JSON
        summary to embed in the action result, or '' on failure."""
        import json as _json
        expect = 0
        try:
            expect = int(os.getenv('NUM_ROBOTS', '0') or 0)
        except ValueError:
            expect = 0
        try:
            subprocess.run(
                ['python3', '-m', 'raven_nav.compile_results',
                 '--results-dir', RESULTS_DIR,
                 '--expect', str(expect), '--wait-timeout', '30',
                 '--reason', reason],
                check=False, timeout=120, capture_output=True, text=True)
            compiled = os.path.join(RESULTS_DIR, 'compiled_results.json')
            # Score vs RESULTS_SCENE GT (set per environment), class-aware on the
            # live query so multi-class scenes score correctly.
            class_filter = getattr(self, '_target_query', None) or 'house'
            subprocess.run(
                ['python3', '-m', 'raven_nav.compare_to_groundtruth',
                 '--compiled', compiled, '--scene', RESULTS_SCENE,
                 '--class-filter', class_filter, '--class-aware'],
                check=False, timeout=120, capture_output=True, text=True)
            summary = {'reason': reason}
            with open(os.path.join(RESULTS_DIR, 'metrics.json')) as f:
                m = _json.load(f)
                summary['detections'] = m.get('detections')
                summary['path'] = {k: m['path'].get(k) for k in (
                    'num_robots', 'total_path_length_m', 'mean_path_length_m',
                    'max_path_length_m', 'path_imbalance', 'makespan_s')}
            gt_path = os.path.join(RESULTS_DIR, 'groundtruth_comparison.json')
            if os.path.exists(gt_path):
                with open(gt_path) as f:
                    gt = _json.load(f)
                keys = ('tp', 'fp', 'fn', 'precision', 'recall', 'f1',
                        'mean_iou_matched', 'mean_center_error_m')
                summary['groundtruth'] = {
                    'num_ground_truth': gt.get('num_ground_truth'),
                    'either': {k: gt['either'].get(k) for k in keys},
                    'visited': {k: gt['visited'].get(k) for k in keys},
                }
            self.get_logger().info(f'[finalize] metrics ({reason}): {summary}')
            return _json.dumps(summary)
        except Exception as e:
            self.get_logger().error(f'[finalize] failed: {e}')
            return ''

    def _cleanup_existing(self) -> None:
        """Kill any leftover rayfronts or raven processes and cancel any
        active navigation goals so a fresh NavigateTask isn't rejected by
        droan_gl (`task already active`) or delivered to a stale executor.

        droan_gl is not killed here — only on the stuck-escalation path
        (_reset_navigate_node); routine cleanup just cancels its active goal.
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
        """Cancel-all on the NavigateTask server (droan_gl).

        Uses the action's underlying _action/cancel_goal service with a
        zero goal_id, which the action spec defines as cancel-all.
        """
        for action in (f'/{robot_name}/tasks/navigate',):
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

    def _pick_rayfronts_gpu(self) -> str | None:
        """GPU index to pin rayfronts to: robot N → GPU N mod <count>, so
        with one GPU per robot + sim, GPU 0 stays with Isaac. Containers are
        privileged, so every robot sees every GPU and torch always defaults
        to cuda:0 otherwise. Returns None on single-GPU/no-GPU hosts."""
        try:
            out = subprocess.run(['nvidia-smi', '-L'], capture_output=True,
                                 text=True, timeout=10).stdout
            n_gpus = sum(1 for line in out.splitlines()
                         if line.startswith('GPU '))
        except Exception:
            return None
        if n_gpus <= 1:
            return None
        return str(int(os.getenv('ROS_DOMAIN_ID', '1')) % n_gpus)

    def _spawn(self, cmd: list, log_name: str | None = None,
               env: dict | None = None) -> tuple:
        """Spawn a subprocess; if log_name is given, tee unfiltered stdout to
        /tmp/<log_name>_<robot>.log for debugging (filter still drives feedback).
        """
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
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

    def _cancel_navigate_task(self, send_future):
        """Best-effort cancel of a NavigateTask started via
        _send_navigate_activator. Safe to call with None / not-yet-accepted."""
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
            self.get_logger().info('Cancelled NavigateTask (droan_gl)')
        except Exception as e:
            self.get_logger().warn(f'Failed to cancel NavigateTask: {e}')

    def _reset_navigate_node(self) -> None:
        """Hard-reset droan_gl (the NavigateTask action server) by killing it so
        launch respawns a fresh instance, clearing any wedged planner state when
        repeated soft re-activations fail to produce motion. Best-effort and
        in-container (droan_gl runs alongside this node under the 'full' role).
        The caller re-sends the activator, which waits for the respawned server."""
        try:
            subprocess.run(['pkill', '-f', 'droan_gl_node'],
                           check=False, timeout=2.0)
            self.get_logger().warn(
                '[escalation] killed droan_gl_node — launch will respawn it')
        except Exception as e:
            self.get_logger().warn(f'[escalation] droan_gl pkill failed: {e}')
        time.sleep(3.0)

    def _send_navigate_activator(self, robot_name: str):
        """Activate droan_gl with an empty-plan NavigateTask: it enters
        ADD_SEGMENT and steers by the /global_plan topic (raven's waypoint)
        until cancelled. Returns (client, send_future)."""
        client = ActionClient(
            self, NavigateTask, f'/{robot_name}/tasks/navigate',
            callback_group=self._cbg)
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateTask server (droan_gl) not available after 10s')
            return None, None
        goal = NavigateTask.Goal()
        goal.goal_tolerance_m = 1.0
        send_future = client.send_goal_async(goal)
        self.get_logger().info(
            'NavigateTask activator sent to droan_gl — following raven /global_plan')
        return client, send_future

    def _send_navigate_to(self, robot_name: str, x: float, y: float, z: float,
                          goal_tolerance_m: float = 1.5):
        """Send droan_gl a single-pose NavigateTask (a concrete plan it follows
        to completion) to fly to (x, y, z) in the robot's 'map' frame. Used to
        bring the drone inside the search polygon before exploring. Returns
        (client, send_future)."""
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        client = ActionClient(
            self, NavigateTask, f'/{robot_name}/tasks/navigate',
            callback_group=self._cbg)
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateTask server (droan_gl) not available after 10s')
            return None, None
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header = plan.header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        plan.poses.append(ps)
        goal = NavigateTask.Goal()
        goal.global_plan = plan
        goal.goal_tolerance_m = float(goal_tolerance_m)
        send_future = client.send_goal_async(goal)
        self.get_logger().info(
            f'NavigateTask (approach) sent to droan_gl → '
            f'({x:.1f}, {y:.1f}, {z:.1f})')
        return client, send_future

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
        # Remember the query so finalize scores GT against the searched classes.
        self._target_query = goal.query
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
        navigate_send_future = None
        rayfronts_q = raven_q = queue.Queue()

        STUCK_TIMEOUT_S = 5.0
        STUCK_DISTANCE_M = 0.3
        ESCALATE_AFTER_RESTARTS = 10
        last_motion_pos = None
        last_motion_time = None
        navigate_restarts = 0
        last_restart_time: 'float | None' = None

        # Search polygon (robot-local 'map' frame) for the out-of-bounds guard.
        # <3 vertices ⇒ unconstrained, so no approach is needed.
        search_poly = [(p.x, p.y) for p in goal.search_area.points]
        approached_bounds = len(search_poly) < 3

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

            rf_env = None
            gpu = self._pick_rayfronts_gpu()
            if gpu is not None:
                rf_env = {**os.environ, 'CUDA_VISIBLE_DEVICES': gpu}
                self.get_logger().info(f'rayfronts pinned to GPU {gpu}')
            rayfronts_proc, rayfronts_q = self._spawn([
                'ros2', 'launch', 'perception_bringup', 'rayfronts.launch.xml',
            ], log_name='rayfronts', env=rf_env)

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
                # Match the rest of the sim stack so published Path/marker
                # stamps line up with the controllers' sim clock.
                '-p', 'use_sim_time:=true',
                # Default full coordination; FRONTIER_ONLY_BASELINE=true → baseline.
                '-p', f'frontier_only_baseline:={os.getenv("FRONTIER_ONLY_BASELINE", "false").strip().lower()}',
                # Multi-robot target assignment: cbba | hungarian | greedy.
                '-p', f'assignment_strategy:={os.getenv("ASSIGNMENT_STRATEGY", "hungarian").strip().lower()}',
                # End at 80% coverage; osmo enforces the 15-min limit by cancel.
                '-p', f'coverage_complete_threshold:={RESULTS_COVERAGE_THRESHOLD}',
                '-p', f'results_dir:={RESULTS_DIR}',
                '-r', (f'/{robot_name}/odometry:='
                       f'/{robot_name}/odometry_conversion/odometry'),
            ], log_name='raven')


            best_conf = 0.0
            rayfronts_ready = False
            prev_rf_sub_count = 0
            mapping_started = False
            navigation_started = False
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
                    # Cancel = the 15-min limit (or a manual stop). Finalize
                    # metrics before returning so they ride out in the result.
                    metrics_json = self._finalize_metrics('cancelled')
                    goal_handle.canceled()
                    result = SemanticSearchTask.Result()
                    result.success = False
                    result.found_poses = cp
                    result.found_labels = cl
                    result.confidence = max(best_conf, cbc)
                    result.objects_found = len(discoveries_by_id)
                    result.message = (
                        f'Cancelled — {len(discoveries_by_id)} instance(s) '
                        f'found | metrics={metrics_json}')
                    return result

                for raw in _drain(rayfronts_q):
                    msg = _filter_rayfronts(raw)
                    if msg:
                        last_rf_status = msg

                for raw in _drain(raven_q):
                    msg = _filter_raven(raw)
                    if msg:
                        last_rv_status = msg

                # Out-of-bounds guard: raven only picks in-polygon frontiers, so
                # from outside it has nothing to steer toward. Fly to the nearest
                # in-bounds point first, then hand off. Runs once.
                if not approached_bounds and self._cur_pos is not None:
                    if _point_in_polygon_xy(self._cur_pos[0], self._cur_pos[1],
                                            search_poly):
                        approached_bounds = True
                    else:
                        tx, ty = _nearest_point_in_polygon_xy(
                            self._cur_pos[0], self._cur_pos[1], search_poly)
                        tz = self._cur_pos[2]
                        self.get_logger().info(
                            f'Drone outside search polygon — flying to nearest '
                            f'in-bounds point ({tx:.1f}, {ty:.1f}) before exploring')
                        self._cancel_active_navigation(robot_name)
                        approach_client, approach_future = self._send_navigate_to(
                            robot_name, tx, ty, tz)
                        deadline = time.time() + 60.0
                        while time.time() < deadline:
                            if goal_handle.is_cancel_requested:
                                break
                            if self._cur_pos is not None and _point_in_polygon_xy(
                                    self._cur_pos[0], self._cur_pos[1], search_poly):
                                self.get_logger().info('Drone reached search polygon')
                                break
                            fb = SemanticSearchTask.Feedback()
                            fb.status = 'Flying to search polygon (out of bounds)'
                            goal_handle.publish_feedback(fb)
                            time.sleep(0.5)
                        else:
                            self.get_logger().warn(
                                'Approach to polygon timed out — exploring from '
                                'current pose anyway')
                        self._cancel_navigate_task(approach_future)
                        self._cancel_active_navigation(robot_name)
                        approached_bounds = True

                # Wait for raven's first waypoint before activating droan_gl —
                # otherwise the drone drifts before raven has any semantic targets.
                if raven_published_waypoint and not navigation_started:
                    navigation_started = True
                    self._cancel_active_navigation(robot_name)
                    self.get_logger().info(
                        'Raven published first waypoint — activating droan_gl '
                        'to navigate to it')
                    _, navigate_send_future = self._send_navigate_activator(
                        robot_name)
                    last_motion_pos = list(self._cur_pos) if self._cur_pos else None
                    last_motion_time = time.time()

                if (navigation_started and self._cur_pos is not None
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
                                f'{STUCK_TIMEOUT_S:.1f}s — re-activating droan_gl')
                            self._cancel_navigate_task(navigate_send_future)
                            self._cancel_active_navigation(robot_name)
                            time.sleep(0.5)
                            _, navigate_send_future = self._send_navigate_activator(
                                robot_name)
                            navigate_restarts += 1
                            last_restart_time = time.time()
                            last_motion_pos = list(self._cur_pos)
                            last_motion_time = time.time()

                            if navigate_restarts >= ESCALATE_AFTER_RESTARTS:
                                self.get_logger().error(
                                    f'[escalation] {navigate_restarts} navigate '
                                    f'restarts — clearing raven blacklist + resetting '
                                    f'droan_gl (the navigate action node)')
                                self._clear_blacklist_pub.publish(Empty())
                                self._reset_navigate_node()
                                _, navigate_send_future = (
                                    self._send_navigate_activator(robot_name))
                                navigate_restarts = 0
                                last_motion_pos = list(self._cur_pos)
                                last_motion_time = time.time()

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

                # Gated on navigation_started so we don't trip before raven is online.
                now_ts = time.time()
                polygon_done = navigation_started and nav_mode_complete

                hit_max = (max_instances > 0
                           and len(discoveries_by_id) >= max_instances)
                if hit_max or polygon_done:
                    # 80% coverage reached — same finalize as the cancel path.
                    reason = 'max_instances' if hit_max else 'coverage'
                    metrics_json = self._finalize_metrics(reason)
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
                            f'{len(discoveries_by_id)} instance(s) found '
                            f'| metrics={metrics_json}')
                    else:
                        result.message = (
                            f'Polygon explored — {len(discoveries_by_id)} '
                            f'instance(s) found: '
                            f'{", ".join(sorted(set(cur_labels)))} '
                            f'| metrics={metrics_json}')
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
                        f'[failsafe] Re-activated droan_gl — drone stuck '
                        f'(<{STUCK_DISTANCE_M:.1f} m in {STUCK_TIMEOUT_S:.1f}s)\n'
                        + status)
                if navigate_restarts > 0:
                    status += f'\n[navigate restarts: {navigate_restarts}]'

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
            # Cancel droan_gl's NavigateTask so it stops following /global_plan.
            self._cancel_navigate_task(navigate_send_future)
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
