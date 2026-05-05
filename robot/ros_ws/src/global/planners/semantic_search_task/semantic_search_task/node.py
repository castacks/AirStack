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
from std_msgs.msg import String
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
    # Pass through multi-robot coordination debug lines verbatim — these are
    # gated behind raven's debug_coordination param and tagged "[coord]" so
    # we don't have to re-parse the format.
    if '[coord]' in line:
        return line
    # Surface startup, waiting, and error messages
    if 'raven_nav started' in low:
        return 'raven_nav started'
    if 'waiting for odometry' in low:
        return 'Waiting for odometry...'
    if 'boot gps captured' in low or 'search_area' in low:
        return line
    if 'error' in low or 'exception' in low or 'traceback' in low:
        return f'ERROR: {line}'
    # The periodic status line looks like:
    # [Frontier-based] frontiers=N rays=N filtered=N voxels=N | target=X | completed=[] | wp=(...)
    m = re.search(r'\[(Frontier-based|Ray-based|Voxel-based)\]', line)
    if not m:
        return None

    mode = m.group(1)
    frontiers = re.search(r'frontiers=(\d+)', line)
    rays = re.search(r'rays=(\d+)', line)
    voxels = re.search(r'voxels=(\d+)', line)
    completed = re.search(r'completed=(\[[^\]]*\])', line)
    wp = re.search(r'wp=(\([^)]+\))', line)

    target = re.search(r'target=(.+?)\s*\|', line)
    filtered = re.search(r'filtered=(\d+)', line)

    parts = [f'[{mode}]']
    if target and target.group(1).strip() != 'None':
        parts.append(f'current_target="{target.group(1).strip()}"')
    if frontiers:
        parts.append(f'frontiers={frontiers.group(1)}')
    if rays:
        parts.append(f'rays={rays.group(1)}')
    if filtered:
        parts.append(f'filtered={filtered.group(1)}')
    if voxels:
        parts.append(f'voxels={voxels.group(1)}')
    if completed and completed.group(1) != '[]':
        parts.append(f'completed={completed.group(1)}')
    if wp and wp.group(1) != 'none':
        parts.append(f'wp={wp.group(1)}')
    return ' '.join(parts)


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

        # Latched polygon constraint for raven_nav. raven_nav joins after this
        # node spawns it, so TRANSIENT_LOCAL ensures it gets the most recent
        # polygon on subscribe. We publish at task start and clear at task end
        # so a stale polygon never carries over to the next task.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._search_area_pub = self.create_publisher(
            PolygonStamped, f'{self._robot_prefix}/raven_nav/search_area',
            latched_qos)

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

    def _send_exploration_task(self, robot_name: str):
        """Send an unbounded ExplorationTask goal to random_walk_planner to
        activate droan_gl. Returns (client, send_future) so the caller can
        cancel the upstream goal when the semantic_search task ends."""
        client = ActionClient(
            self,
            ExplorationTask,
            f'/{robot_name}/tasks/exploration',
            callback_group=self._cbg)
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('ExplorationTask server not available after 10s')
            return None, None
        goal = ExplorationTask.Goal()
        goal.min_altitude_agl = 2.0
        goal.max_altitude_agl = 15.0
        goal.min_flight_speed = 1.0
        goal.max_flight_speed = 3.0
        send_future = client.send_goal_async(goal)
        self.get_logger().info('ExplorationTask sent to random_walk_planner')
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

        # Build the full query list: target queries + background contrast queries.
        # Softmax normalization across queries requires N >= 2 to produce
        # meaningful discriminative scores; with N=1 every score is 1.0.
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

        # Push the search area to raven_nav before spawning it. action_relay has
        # already transformed vertices into the robot's local 'map' frame.
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

        # Track last meaningful line from each process for feedback
        last_rf_status = 'Starting rayfronts...'
        last_rv_status = 'Starting raven...'

        try:
            # Kill any leftover processes from a previous run
            self._cleanup_existing()

            robot_name = os.getenv('ROBOT_NAME', 'robot_1')

            # Wait for old rayfronts DDS subscription to fully unregister
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
            required_batches = 3
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

            # Subscribe to raven's global_plan to detect first waypoint.
            # Use BEST_EFFORT QoS to match raven's publisher (some robots in
            # the stack publish global_plan as BEST_EFFORT for low-latency,
            # others as RELIABLE; subscribing BEST_EFFORT accepts both).
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

            # Subscribe to raven's completed_targets
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

            # Subscribe to voxels_sim/all for best-confidence tracking
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
                    goal_handle.canceled()
                    result = SemanticSearchTask.Result()
                    result.success = False
                    result.message = 'Cancelled'
                    return result

                # Drain and filter rayfronts output
                for raw in _drain(rayfronts_q):
                    msg = _filter_rayfronts(raw)
                    if msg:
                        last_rf_status = msg

                # Drain and filter raven output
                for raw in _drain(raven_q):
                    msg = _filter_raven(raw)
                    if msg:
                        last_rv_status = msg

                # Start random_walk once raven publishes its first waypoint.
                # The drone must NOT start moving until raven is ready to
                # provide semantic targets — random_walk would otherwise drift
                # the robot around before raven has computed where to look.
                if raven_published_waypoint and not random_walk_started:
                    random_walk_started = True
                    self._cancel_active_navigation(robot_name)
                    self.get_logger().info(
                        'Raven published first waypoint — sending ExplorationTask '
                        'to existing random_walk_planner')
                    _, exploration_send_future = self._send_exploration_task(
                        robot_name)

                # Send queries to rayfronts whenever its subscriber appears (or reappears).
                # This handles the initial load AND any rayfronts restart mid-task.
                # Send ALL queries (target + background) so softmax is meaningful.
                rf_sub_count = self.count_subscribers(f'{self._rf_prefix}/new_text_query')
                if rf_sub_count > 0 and prev_rf_sub_count == 0:
                    rayfronts_ready = True
                    for q in all_queries:
                        self._text_query_pub.publish(String(data=q))
                    last_rf_status = f'Queries sent: {", ".join(all_queries)}'
                    self.get_logger().info(
                        f'Queries sent to rayfronts: {all_queries}')
                if rf_sub_count == 0 and prev_rf_sub_count > 0:
                    rayfronts_ready = False
                    self.get_logger().info('rayfronts subscriber lost — will resend on reconnect')
                prev_rf_sub_count = rf_sub_count

                # Success: raven visited a cluster for every query
                queries_set = set(q.lower() for q in queries)
                completed_lower = set(c.lower() for c in completed_targets)
                if queries_set and queries_set.issubset(completed_lower):
                    goal_handle.succeed()
                    result = SemanticSearchTask.Result()
                    result.success = True
                    result.message = (
                        f'All targets visited: {", ".join(sorted(queries_set))}')
                    result.confidence = best_conf
                    return result

                # Build clean feedback status
                pos_str = ''
                if self._cur_pos:
                    pos_str = (f'pos=({self._cur_pos[0]:.1f}, '
                               f'{self._cur_pos[1]:.1f}, '
                               f'{self._cur_pos[2]:.1f})')

                pending = [q for q in queries
                           if q.lower() not in set(c.lower() for c in completed_targets)]
                done = [q for q in queries
                        if q.lower() in set(c.lower() for c in completed_targets)]

                if not rayfronts_ready:
                    status = f'[rayfronts] {last_rf_status}'
                elif not mapping_started:
                    status = (f'[rayfronts] {last_rf_status}\n'
                              f'[raven] {last_rv_status}')
                else:
                    progress = f'{len(done)}/{len(queries)} targets visited'
                    done_str = f'done=[{", ".join(done)}]' if done else ''
                    pending_str = f'pending=[{", ".join(pending)}]' if pending else ''
                    status = (f'[raven] {last_rv_status}\n'
                              f'{progress}  {done_str}  {pending_str}  {pos_str}').strip()

                fb = SemanticSearchTask.Feedback()
                fb.status = status
                fb.best_confidence_so_far = best_conf
                if self._cur_pos:
                    fb.current_position = Point(
                        x=self._cur_pos[0],
                        y=self._cur_pos[1],
                        z=self._cur_pos[2])
                goal_handle.publish_feedback(fb)

                if self._interruptible_sleep(goal_handle, 1.0):
                    continue   # cancel was requested, loop back to check it

        finally:
            # Cancel the upstream ExplorationTask so the bringup-launched
            # random_walk_planner stops wandering when this task ends.
            self._cancel_exploration_task(exploration_send_future)
            self._kill('rayfronts', rayfronts_proc)
            self._kill('raven', raven_proc)
            # Clear the latched polygon so the next task starts unconstrained
            # by default unless it provides its own search_area.
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
