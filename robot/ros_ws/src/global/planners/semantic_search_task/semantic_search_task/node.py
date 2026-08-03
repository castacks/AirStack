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
from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw

# ── ANSI / ROS log stripping ──────────────────────────────────────────────────

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')
_ROS_PREFIX_RE = re.compile(
    r'^\s*\[?(INFO|WARN|ERROR|DEBUG)\]?\s*\[\d+\.\d+\]\s*\[[^\]]+\]:\s*')

# Shared bind-mounted dir (./cache -> /root/.cache in every robot replica), so
# all robots' raven dumps land where any container can compile them.
RESULTS_DIR = '/root/.cache/raven_results'
RESULTS_COVERAGE_THRESHOLD = 0.80
RESULTS_SCENE = os.getenv('RESULTS_SCENE', 'RetroNeighborhood')


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
    if '|' in raw and '█' in raw:
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


def _filter_lvlm(line: str) -> str | None:
    """Surface key lvlm_baseline events for action feedback; skip the rest."""
    line = _clean(line)
    if not line:
        return None
    low = line.lower()
    if 'error' in low or 'exception' in low or 'traceback' in low:
        return f'ERROR: {line}'
    if 'loading internvl3' in low:
        return 'Loading InternVL3-2B model...'
    if 'model loaded' in low:
        return 'InternVL3-2B model loaded'
    if 'control loop active' in low:
        return 'LVLM navigating'
    if 'planned action' in low or 'model response' in low:
        return line
    if 'target objects set' in low:
        return line
    if 'not enough data' in low:
        return 'Waiting for FPV / odometry...'
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
            log_fh = open(log_path, 'w', buffering=1)
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
        self._cur_yaw = 0.0
        # Last pose of raven's /global_plan; the stuck-recovery faces/creeps to it.
        self._latest_global_target = None

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
        # Same latched polygon for the LVLM baseline (waypoint clamp).
        self._lvlm_search_area_pub = self.create_publisher(
            PolygonStamped, f'{self._robot_prefix}/lvlm_baseline/search_area',
            latched_qos)
        self._clear_blacklist_pub = self.create_publisher(
            Empty, f'{self._robot_prefix}/raven_nav/clear_blacklist', 10)
        self._reset_stuck_pub = self.create_publisher(
            Empty, f'{self._robot_prefix}/droan/reset_stuck', 10)
        self._traj_override_pub = self.create_publisher(
            TrajectoryXYZVYaw,
            f'{self._robot_prefix}/trajectory_controller/trajectory_override', 10)

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
        q = msg.pose.pose.orientation
        self._cur_yaw = float(np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

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

    def _publish_lvlm_search_area(self, polygon: Polygon) -> None:
        """Latched search polygon (robot-local 'map' frame) for the LVLM baseline
        to clamp its waypoints into. An empty polygon clears the constraint."""
        stamped = PolygonStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'map'
        stamped.polygon = polygon
        self._lvlm_search_area_pub.publish(stamped)

    def _execute_lvlm(self, goal_handle, goal, queries):
        """FPV+LVLM baseline run: spawn lvlm_baseline_node (which drives the drone
        by sending NavigateTask goals to droan_gl) and forward the query +
        search_area. No rayfronts/raven, no detection scoring — the node owns
        navigation end to end. The loop just relays feedback and honors cancel."""
        robot_name = os.getenv('ROBOT_NAME', 'robot_1')
        lvlm_proc = None
        try:
            # Clear any leftover baseline node from a previous run.
            subprocess.run(['pkill', '-SIGTERM', '-f', 'lvlm_baseline_node'],
                           capture_output=True)
            time.sleep(1.0)
            subprocess.run(['pkill', '-SIGKILL', '-f', 'lvlm_baseline_node'],
                           capture_output=True)
            self._cancel_active_navigation(robot_name)

            # Forward the search polygon (already in the robot's local 'map' frame).
            self._publish_lvlm_search_area(goal.search_area)

            targets_yaml = str(queries).replace("'", '"')
            env = None
            gpu = self._pick_rayfronts_gpu()
            if gpu is not None:
                env = {**os.environ, 'CUDA_VISIBLE_DEVICES': gpu}
                self.get_logger().info(f'lvlm_baseline pinned to GPU {gpu}')

            self.get_logger().info(
                f'LVLM baseline | targets={queries} | '
                f'altitude=[{goal.min_altitude_agl}, {goal.max_altitude_agl}]')
            # Run the node through its isolated venv (transformers/bitsandbytes/
            # accelerate), reusing system rclpy/cv_bridge/torch via the inherited
            # ROS + workspace PYTHONPATH. Fall back to `ros2 run` where the venv
            # isn't built (dev/local).
            ros_args = [
                '--ros-args',
                '-p', f'target_objects:={targets_yaml}',
                '-p', f'min_altitude_agl:={goal.min_altitude_agl}',
                '-p', f'max_altitude_agl:={goal.max_altitude_agl}',
                '-p', 'use_sim_time:=true',
            ]
            venv_py = '/opt/lvlm-venv/bin/python'
            if os.path.exists(venv_py):
                lvlm_cmd = [venv_py, '-m', 'lvlm_baseline.lvlm_baseline_node'] + ros_args
            else:
                lvlm_cmd = ['ros2', 'run', 'lvlm_baseline', 'lvlm_baseline_node'] + ros_args
            lvlm_proc, lvlm_q = self._spawn(lvlm_cmd, log_name='lvlm', env=env)

            last_status = 'Starting LVLM baseline...'
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = SemanticSearchTask.Result()
                    result.success = False
                    result.message = 'Cancelled (LVLM baseline)'
                    return result
                for raw in _drain(lvlm_q):
                    msg = _filter_lvlm(raw)
                    if msg:
                        last_status = msg
                # The baseline node spins forever in a healthy run; an early exit
                # means it crashed (e.g. package not built, import/model error).
                # Surface it and abort instead of hanging on a dead process —
                # the tail of /tmp/lvlm_<robot>.log has the traceback.
                rc = lvlm_proc.poll()
                if rc is not None:
                    self.get_logger().error(
                        f'lvlm_baseline_node exited early (rc={rc}); '
                        f'last log: {last_status}')
                    goal_handle.abort()
                    result = SemanticSearchTask.Result()
                    result.success = False
                    result.message = (
                        f'lvlm_baseline_node exited early (rc={rc}) — '
                        f'last log: {last_status}')
                    return result
                fb = SemanticSearchTask.Feedback()
                fb.status = f'[lvlm] {last_status}'
                goal_handle.publish_feedback(fb)
                if self._interruptible_sleep(goal_handle, 1.0):
                    continue
        finally:
            self._cancel_active_navigation(robot_name)
            self._kill('lvlm', lvlm_proc)
            self._publish_lvlm_search_area(Polygon())
            self._task_active = False

        goal_handle.abort()
        result = SemanticSearchTask.Result()
        result.success = False
        result.message = 'Node shutdown'
        return result

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

        droan_gl is never killed — the stuck path recovers it in-place
        (blacklist → reset_stuck → 2 m nudge); cleanup just cancels its goal.

        In debug mode this cancel is what makes the drone hold at task start
        (clearing any stale auto-follow); debug then never re-commands it.
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
        """GPU index to pin rayfronts to.

        **GPU 0 is reserved for Isaac Sim**, so robot N takes GPU N whenever the
        host has one to spare — the 4-GPU / 3-robot sizing the OSMO mission
        workflow requests gives sim→0, robot_1→1, robot_2→2, robot_3→3.

        The previous rule was ``ROS_DOMAIN_ID % n_gpus``, which on a 3-GPU host
        sent robot_3 to GPU 0 and made it fight Isaac Sim for memory; rayfronts
        then died with a CUDA OOM loading its encoder, and because
        semantic_search_task waits for rayfronts batches with no timeout, that
        robot never got a navigator and hovered for the whole mission. With
        fewer GPUs than robots+1 the robots now share the non-zero GPUs instead.

        Containers are privileged, so every robot sees every GPU and torch
        defaults to cuda:0 unless pinned. Returns None on single/no-GPU hosts,
        where there is nothing to choose.
        """
        try:
            out = subprocess.run(['nvidia-smi', '-L'], capture_output=True,
                                 text=True, timeout=10).stdout
            n_gpus = sum(1 for line in out.splitlines()
                         if line.startswith('GPU '))
        except Exception:
            return None
        if n_gpus <= 1:
            return None
        robot_id = int(os.getenv('ROS_DOMAIN_ID', '1'))
        if robot_id < n_gpus:
            gpu = robot_id                      # one GPU each, 0 left to the sim
        else:
            gpu = 1 + (robot_id - 1) % (n_gpus - 1)   # share, still never 0
        self.get_logger().info(
            f'{n_gpus} GPU(s) visible; robot {robot_id} -> GPU {gpu} '
            f'(GPU 0 reserved for Isaac Sim)')
        return str(gpu)

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

    def _send_navigate_activator(self, robot_name: str, retries: int = 4):
        """Activate droan_gl with an empty-plan NavigateTask: it enters
        ADD_SEGMENT and steers by the /global_plan topic (raven's waypoint)
        until cancelled. Waits for the goal to be *accepted* and retries on
        rejection — droan rejects while a just-cancelled goal is still winding
        down (its execute loop notices cancel only at 1 Hz), so a too-quick
        re-send would silently drop the follow out of ADD_SEGMENT and the drone
        would stop navigating. Returns (client, send_future) with send_future
        accepted, or (client, None) if it never got accepted."""
        client = ActionClient(
            self, NavigateTask, f'/{robot_name}/tasks/navigate',
            callback_group=self._cbg)
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateTask server (droan_gl) not available after 10s')
            return None, None
        goal = NavigateTask.Goal()
        goal.goal_tolerance_m = 1.0
        for attempt in range(retries):
            send_future = client.send_goal_async(goal)
            deadline = time.time() + 3.0
            while not send_future.done() and time.time() < deadline:
                time.sleep(0.05)
            handle = send_future.result() if send_future.done() else None
            if handle is not None and getattr(handle, 'accepted', False):
                self.get_logger().info(
                    'NavigateTask activator accepted — following raven /global_plan')
                return client, send_future
            self.get_logger().warn(
                f'activator not accepted (attempt {attempt + 1}/{retries}) — '
                f'droan busy, retrying')
            time.sleep(1.2)
        self.get_logger().error(
            'Failed to activate droan_gl follow — rejected repeatedly')
        return client, None

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

    def _nudge_up_override(self, height_m: float = 1.0, hold_s: float = 3.0,
                           cancel_handle=None) -> None:
        """Climb height_m straight up via a trajectory_override, bypassing
        droan's planner. The controller keeps its mode (ADD_SEGMENT, held by the
        activator), so droan resumes merging raven-derived segments the moment it
        can plan forward from the higher pose — control returns to raven
        automatically. Re-published toward a fixed top over hold_s so a stray
        droan segment can't pull the climb off; holds current heading."""
        if self._cur_pos is None:
            return
        top_z = self._cur_pos[2] + height_m
        yaw = self._cur_yaw
        deadline = time.time() + hold_s
        while time.time() < deadline:
            if cancel_handle is not None and cancel_handle.is_cancel_requested:
                return
            if self._cur_pos is None:
                time.sleep(0.1)
                continue
            cx, cy, cz = self._cur_pos
            if cz >= top_z - 0.2:
                return
            traj = TrajectoryXYZVYaw()
            traj.header.frame_id = 'map'
            traj.header.stamp = self.get_clock().now().to_msg()
            steps = 5
            for i in range(steps + 1):
                wp = WaypointXYZVYaw()
                wp.position.x = float(cx)
                wp.position.y = float(cy)
                wp.position.z = float(cz + (top_z - cz) * i / steps)
                wp.velocity = 1.0
                wp.yaw = float(yaw)
                traj.waypoints.append(wp)
            self._traj_override_pub.publish(traj)
            time.sleep(0.5)

    def _recover_toward_target(self, creep_m: float = 1.5, hold_s: float = 3.0,
                               cancel_handle=None) -> None:
        """Yaw to face raven's target and creep a short step toward it via
        trajectory_override. Facing it fills in the unseen space ahead so droan
        resumes planning; the small move breaks the controller's zero-velocity
        stall. Holds altitude; climbs 1 m when no target / already over it."""
        if self._cur_pos is None:
            return
        target = self._latest_global_target
        cx, cy, cz = self._cur_pos
        if target is None:
            self._nudge_up_override(1.0, hold_s, cancel_handle)
            return
        dx, dy = float(target[0]) - cx, float(target[1]) - cy
        horiz = (dx * dx + dy * dy) ** 0.5
        if horiz < 0.5:
            # Already over the target in XY — climb instead.
            self._nudge_up_override(1.0, hold_s, cancel_handle)
            return
        ux, uy = dx / horiz, dy / horiz
        yaw = float(np.arctan2(dy, dx))
        step = min(creep_m, horiz)
        start_x, start_y = cx, cy
        deadline = time.time() + hold_s
        while time.time() < deadline:
            if cancel_handle is not None and cancel_handle.is_cancel_requested:
                return
            if self._cur_pos is None:
                time.sleep(0.1)
                continue
            cx, cy, cz = self._cur_pos
            moved = ((cx - start_x) ** 2 + (cy - start_y) ** 2) ** 0.5
            if moved >= step - 0.2:
                return
            tx, ty = start_x + ux * step, start_y + uy * step
            traj = TrajectoryXYZVYaw()
            traj.header.frame_id = 'map'
            traj.header.stamp = self.get_clock().now().to_msg()
            steps = 5
            for i in range(steps + 1):
                wp = WaypointXYZVYaw()
                wp.position.x = float(cx + (tx - cx) * i / steps)
                wp.position.y = float(cy + (ty - cy) * i / steps)
                wp.position.z = float(cz)
                wp.velocity = 1.0
                wp.yaw = yaw
                traj.waypoints.append(wp)
            self._traj_override_pub.publish(traj)
            time.sleep(0.5)

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
        # Debug/manual-fly mode: spin up rayfronts + raven and publish all their
        # visualization, but skip every drone-commanding path (droan follow,
        # out-of-bounds approach, stuck recovery) so the operator can hand-fly
        # the drone and watch what rayfronts detects. Settable per-goal
        # (goal.debug; getattr keeps this working pre-task_msgs-rebuild) or
        # per-node (SEMANTIC_SEARCH_DEBUG env var).
        # ─── TEMP DEBUG HARDCODE — manual-fly forced ON; REVERT WHEN DONE ───
        # Restore the two commented lines below (delete `debug = True`) to go
        # back to per-goal / env-var control.
        debug = False
        # debug = (bool(getattr(goal, 'debug', False))
        #          or os.getenv('SEMANTIC_SEARCH_DEBUG', '').strip().lower()
        #          in ('1', 'true', 'yes'))
        # ─── END TEMP DEBUG HARDCODE ────────────────────────────────────────
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

        # LVLM baseline: pure FPV+LVLM navigation via the NavigateTask action.
        # Selected by LVLM_BASELINE=true, mirroring FRONTIER_ONLY_BASELINE. Skips
        # rayfronts/raven detection (no background_queries / softmax needed).
        if os.getenv('LVLM_BASELINE', 'false').strip().lower() in ('1', 'true', 'yes'):
            return self._execute_lvlm(goal_handle, goal, queries)

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
        if debug:
            self.get_logger().warn(
                '[DEBUG] manual-fly mode — rayfronts + raven will start and '
                'publish visualization, but the drone will NOT be commanded. '
                'Fly it yourself (navigate / fixed-trajectory from GCS).')

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

        self._latest_global_target = None

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

            raven_args = [
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
                # VLFM_BASELINE=true → greedy VLFM baseline (mutually exclusive
                # with FRONTIER_ONLY_BASELINE; raven_nav lets frontier win).
                '-p', f'vlfm_baseline:={os.getenv("VLFM_BASELINE", "false").strip().lower()}',
                # End at 80% coverage; osmo enforces the 15-min limit by cancel.
                '-p', f'coverage_complete_threshold:={RESULTS_COVERAGE_THRESHOLD}',
                '-p', f'results_dir:={RESULTS_DIR}',
                '-r', (f'/{robot_name}/odometry:='
                       f'/{robot_name}/odometry_conversion/odometry'),
            ]
            # Optional raven_nav tuning from the mission goal; a sentinel (<0, or
            # <1 for the count) means "unset" -> raven_nav.yaml default is used.
            for pname, gval, sentinel in (
                ('score_threshold', goal.score_threshold, 0.0),
                ('voxel_score_threshold', goal.voxel_score_threshold, 0.0),
                ('voxel_min_confidence', goal.voxel_min_confidence, 0.0),
                ('voxel_min_cluster_size', goal.voxel_min_cluster_size, 1),
                ('bundle_len', int(getattr(goal, 'bundle_len', -1)), 1),
            ):
                if gval >= sentinel:
                    raven_args += ['-p', f'{pname}:={gval}']
            raven_proc, raven_q = self._spawn(raven_args, log_name='raven')


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
            # Written by _discoveries_cb on another executor thread (Reentrant
            # group + MultiThreadedExecutor) while _execute reads it. Guard so a
            # concurrent insert can't fault iteration ("dict changed size").
            discoveries_lock = threading.Lock()

            nav_mode_complete = False
            nav_mode = ''

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
                    p = msg.poses[-1].pose.position
                    self._latest_global_target = (p.x, p.y, p.z)
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
                    with discoveries_lock:
                        discoveries_by_id[inst_id] = d
            self.create_subscription(
                String, f'/{robot_name}/raven_nav/discoveries',
                _discoveries_cb, 10, callback_group=self._cbg)

            # Single source of truth for "polygon explored".
            def _nav_mode_cb(msg):
                nonlocal nav_mode_complete, nav_mode
                nav_mode = (msg.data or '').strip()
                if nav_mode == 'complete':
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
                    with discoveries_lock:
                        snapshot = list(discoveries_by_id.values())
                    for d in snapshot:
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
                    # Debug runs aren't real searches — skip the GT scoring.
                    metrics_json = '' if debug else self._finalize_metrics(
                        'cancelled')
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
                # in-bounds point first, then hand off. Runs once. Skipped in
                # debug — the operator positions the drone themselves.
                if not debug and not approached_bounds and self._cur_pos is not None:
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
                # Skipped in debug: never activate droan follow, so the drone is
                # left for the operator. navigation_started stays False, which
                if not debug and raven_published_waypoint and not navigation_started:
                    navigation_started = True
                    self._cancel_active_navigation(robot_name)
                    self.get_logger().info(
                        'Raven published first waypoint — activating droan_gl '
                        'to navigate to it')
                    _, navigate_send_future = self._send_navigate_activator(
                        robot_name)

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
                with discoveries_lock:
                    snapshot = list(discoveries_by_id.values())
                for d in snapshot:
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

                if debug:
                    status = ('[DEBUG] manual-fly — drone NOT commanded; '
                              'fly it yourself\n' + status)

                fb = SemanticSearchTask.Feedback()
                fb.status = status
                fb.objects_found_so_far = len(discoveries_by_id)
                fb.best_confidence_so_far = max(best_conf, cur_best_conf)
                fb.current_found_poses = cur_poses
                fb.current_found_labels = cur_labels
                goal_handle.publish_feedback(fb)

                if self._interruptible_sleep(goal_handle, 1.0):
                    continue

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
