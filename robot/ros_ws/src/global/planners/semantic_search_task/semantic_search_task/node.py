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
from geometry_msgs.msg import Point, Polygon, Pose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
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
    # Surface startup, waiting, and error messages
    if 'raven_nav started' in low:
        return 'raven_nav started'
    if 'waiting for odometry' in low:
        return 'Waiting for odometry...'
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

def _pipe_to_queue(proc: subprocess.Popen, q: queue.Queue) -> None:
    for line in iter(proc.stdout.readline, b''):
        q.put(line.decode('utf-8', errors='replace').rstrip())
    q.put(None)


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

    def _handle_goal(self, goal_request):
        if self._task_active:
            self.get_logger().warn('Rejecting goal: task already active')
            return GoalResponse.REJECT
        self._task_active = True
        return GoalResponse.ACCEPT

    def _cleanup_existing(self) -> None:
        """Kill any leftover rayfronts or raven processes.

        random_walk_planner is intentionally NOT killed here: it now runs as
        a long-lived task executor under global_bringup, and pkill'ing it
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

    def _spawn(self, cmd: list) -> tuple:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True)
        q = queue.Queue()
        threading.Thread(target=_pipe_to_queue, args=(proc, q), daemon=True).start()
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

    def _spawn_random_walk(self, robot_name: str) -> subprocess.Popen:
        # Resolve the config file path via ros2 pkg
        prefix = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'random_walk_planner'],
            capture_output=True, text=True).stdout.strip()
        config = f'{prefix}/share/random_walk_planner/config/random_walk_config.yaml'
        return self._spawn([
            'ros2', 'run', 'random_walk_planner', 'random_walk_planner',
            '--ros-args',
            '--params-file', config,
            '-r', f'~/global_plan:=/{robot_name}/global_plan',
            '-r', f'~/exploration_task:=/{robot_name}/tasks/exploration',
            '-r', f'navigate_task:=/{robot_name}/tasks/navigate',
            '-r', f'odometry:=/{robot_name}/odometry_conversion/odometry',
            '-r', f'vdb_map_visualization:=/{robot_name}/vdb_mapping/vdb_map_visualization',
            '-r', f'~/global_plan_toggle:=/{robot_name}/behavior/global_plan_toggle',
        ])

    def _send_exploration_task(self, robot_name: str) -> None:
        """Send an unbounded ExplorationTask goal to random_walk_planner to activate droan_gl."""
        client = ActionClient(
            self,
            ExplorationTask,
            f'/{robot_name}/tasks/exploration',
            callback_group=self._cbg)
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('ExplorationTask server not available after 10s')
            return
        goal = ExplorationTask.Goal()
        goal.min_altitude_agl = 2.0
        goal.max_altitude_agl = 15.0
        goal.min_flight_speed = 1.0
        goal.max_flight_speed = 3.0
        client.send_goal_async(goal)
        self.get_logger().info('ExplorationTask sent to random_walk_planner')

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

        rayfronts_proc = raven_proc = random_walk_proc = None
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

            # Start raven first so it's subscribed to rays_sim/all and voxels_sim/all
            # before rayfronts runs its first query cycle (rayfronts lazy-publishes
            # those topics only when a subscriber exists).
            # query_labels = full set (for sim column mapping)
            # target_labels = only the user's targets (for behavior filtering)
            all_labels_yaml = str(all_queries).replace("'", '"')
            target_labels_yaml = str(queries).replace("'", '"')
            raven_proc, raven_q = self._spawn([
                'ros2', 'run', 'raven_nav', 'raven_nav_node',
                '--ros-args',
                '-p', f'query_labels:={all_labels_yaml}',
                '-p', f'target_labels:={target_labels_yaml}',
                '-p', f'min_altitude_agl:={goal.min_altitude_agl}',
                '-p', f'max_altitude_agl:={goal.max_altitude_agl}',
                '-r', (f'/{robot_name}/odometry:='
                       f'/{robot_name}/odometry_conversion/odometry'),
            ])

            rayfronts_proc, rayfronts_q = self._spawn([
                'ros2', 'launch', 'perception_bringup', 'rayfronts.launch.xml',
            ])


            best_conf = 0.0
            rayfronts_ready = False
            prev_rf_sub_count = 0
            mapping_started = False
            random_walk_started = False
            raven_published_waypoint = False
            completed_targets: set = set()

            # Subscribe to raven's global_plan to detect first waypoint
            from nav_msgs.msg import Path
            def _global_plan_cb(msg):
                nonlocal raven_published_waypoint
                if msg.poses:
                    raven_published_waypoint = True
            self.create_subscription(
                Path, f'/{robot_name}/global_plan',
                _global_plan_cb, 1, callback_group=self._cbg)

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

                # Start random_walk once raven publishes its first waypoint
                if raven_published_waypoint and not random_walk_started:
                    random_walk_started = True
                    random_walk_proc, _ = self._spawn_random_walk(robot_name)
                    self.get_logger().info(
                        'Raven published first waypoint — starting random_walk_planner')
                    time.sleep(3.0)
                    self._send_exploration_task(robot_name)

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
            self._kill('rayfronts', rayfronts_proc)
            self._kill('raven', raven_proc)
            self._kill('random_walk', random_walk_proc)
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
