"""Action relay: bridges ROS 2 actions from GCS (domain 0) to a robot (domain N).

foxglove_bridge drops nested message fields when calling action services via
callService, so this relay uses plain String topics with JSON payloads instead.

  Foxglove extension                     Robot (domain N)
    publishes String JSON                  Task executor (ActionServer)
    on .../goal                            unchanged
         |                                      ^
         v                                      |
  action_relay (this node, domain 0+N)          |
    subscribes .../goal (String)                |
    parses JSON -> typed Goal msg               |
    sends via ActionClient  --------------------+
    publishes .../relay_feedback (String JSON)
    publishes .../relay_result   (String JSON)

Usage:
    ros2 run action_relay action_relay_node \\
        --ros-args -p robot_name:=robot_1 -p robot_domain:=1
"""

import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Polygon, Point32, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String

from airstack_msgs.msg import FixedTrajectory
from diagnostic_msgs.msg import KeyValue
from task_msgs.action import (
    TakeoffTask, LandTask, NavigateTask,
    FixedTrajectoryTask, SemanticSearchTask,
    ExplorationTask,
)

# ── Map-frame ENU origin (must match gcs_visualizer/gcs_utils.py) ────────────
# Foxglove panels publish waypoints/polygons in this same global ENU frame, and
# the gcs_visualizer renders the robot at gps_to_enu(...) - boot. The robot's
# task executors expect coordinates relative to its own boot pose, so we
# subtract the robot's boot ENU position before forwarding.
ORIGIN_LAT = 38.736832
ORIGIN_LON = -9.137977


def gps_to_enu(lat, lon, alt, alt_ground):
    x = (lon - ORIGIN_LON) * 111320.0 * math.cos(math.radians(ORIGIN_LAT))
    y = (lat - ORIGIN_LAT) * 111320.0
    z = alt - alt_ground
    return x, y, z

# ── Registry ─────────────────────────────────────────────────────────────────
RELAYS = [
    ('takeoff',          TakeoffTask),
    ('land',             LandTask),
    ('navigate',         NavigateTask),
    ('fixed_trajectory', FixedTrajectoryTask),
    ('semantic_search',  SemanticSearchTask),
    ('exploration',      ExplorationTask),
]

_RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


# ── Goal builders ────────────────────────────────────────────────────────────
# Each converts a JSON dict into a typed Goal message.

def _map_to_robot(x, y, z, ts):
    """Subtract the robot's boot ENU position from a global-ENU point."""
    boot = ts.get('boot') if ts else None
    if boot is None:
        return x, y, z
    bx, by, bz = boot
    return x - bx, y - by, z - bz


def _build_takeoff_goal(d, ts):
    g = TakeoffTask.Goal()
    g.target_altitude_m = float(d.get('target_altitude_m', 0))
    g.velocity_m_s = float(d.get('velocity_m_s', 0))
    return g


def _build_land_goal(d, ts):
    g = LandTask.Goal()
    g.velocity_m_s = float(d.get('velocity_m_s', 0))
    return g


def _build_navigate_goal(d, ts):
    g = NavigateTask.Goal()
    g.goal_tolerance_m = float(d.get('goal_tolerance_m', 1.0))
    plan_data = d.get('global_plan', {})
    header = Header()
    hdr = plan_data.get('header', {})
    in_frame = str(hdr.get('frame_id', 'map'))
    # Waypoints from the Foxglove editor are in *global* ENU (gcs_visualizer's
    # shared map). The robot's own TF tree uses 'map' rooted at its takeoff
    # position, so we subtract the boot offset and keep the 'map' frame_id —
    # the on-robot planner (droan_gl, target_frame=map) expects this.
    if in_frame == 'map':
        if ts is None or ts.get('boot') is None:
            raise RuntimeError(
                'Cannot transform map-frame waypoints: robot GPS boot pose '
                'not yet received. Wait for first GPS fix and retry.')
    header.frame_id = in_frame
    stamp = hdr.get('stamp', {})
    header.stamp = Time(sec=int(stamp.get('sec', 0)),
                        nanosec=int(stamp.get('nanosec', 0)))
    path = Path(header=header)
    for pose_data in plan_data.get('poses', []):
        ps = PoseStamped()
        ps.header = header
        pos = pose_data.get('pose', {}).get('position', {})
        ori = pose_data.get('pose', {}).get('orientation', {})
        x = float(pos.get('x', 0))
        y = float(pos.get('y', 0))
        z = float(pos.get('z', 0))
        if in_frame == 'map':
            x, y, z = _map_to_robot(x, y, z, ts)
        ps.pose.position = Point(x=x, y=y, z=z)
        ps.pose.orientation.x = float(ori.get('x', 0))
        ps.pose.orientation.y = float(ori.get('y', 0))
        ps.pose.orientation.z = float(ori.get('z', 0))
        ps.pose.orientation.w = float(ori.get('w', 1))
        path.poses.append(ps)
    g.global_plan = path
    return g


def _build_fixed_trajectory_goal(d, ts):
    g = FixedTrajectoryTask.Goal()
    g.loop = bool(d.get('loop', False))
    spec_data = d.get('trajectory_spec', {})
    spec = FixedTrajectory()
    spec.type = str(spec_data.get('type', ''))
    for attr in spec_data.get('attributes', []):
        kv = KeyValue()
        kv.key = str(attr.get('key', ''))
        kv.value = str(attr.get('value', ''))
        spec.attributes.append(kv)
    g.trajectory_spec = spec
    return g


def _build_semantic_search_goal(d, ts):
    g = SemanticSearchTask.Goal()
    g.query = str(d.get('query', ''))
    g.background_queries = str(d.get('background_queries', ''))
    g.min_altitude_agl = float(d.get('min_altitude_agl', 3.0))
    g.max_altitude_agl = float(d.get('max_altitude_agl', 15.0))
    g.min_flight_speed = float(d.get('min_flight_speed', 0))
    g.max_flight_speed = float(d.get('max_flight_speed', 0))
    g.confidence_threshold = float(d.get('confidence_threshold', 0.95))
    area = d.get('search_area', {})
    g.search_area = _build_polygon_from_global(area.get('points', []), ts)
    return g


def _build_polygon_from_global(points, ts):
    """Transform editor-frame polygon points to the robot's local 'map' frame."""
    if points and (ts is None or ts.get('boot') is None):
        raise RuntimeError(
            'Cannot transform polygon vertices: robot GPS boot pose '
            'not yet received. Wait for first GPS fix and retry.')
    poly = Polygon()
    for pt in points:
        x, y, z = _map_to_robot(
            float(pt.get('x', 0)),
            float(pt.get('y', 0)),
            float(pt.get('z', 0)),
            ts,
        )
        poly.points.append(Point32(x=x, y=y, z=z))
    return poly


def _build_exploration_goal(d, ts):
    g = ExplorationTask.Goal()
    g.min_altitude_agl = float(d.get('min_altitude_agl', 0))
    g.max_altitude_agl = float(d.get('max_altitude_agl', 0))
    g.min_flight_speed = float(d.get('min_flight_speed', 0))
    g.max_flight_speed = float(d.get('max_flight_speed', 0))
    g.time_limit_sec = float(d.get('time_limit_sec', 0))
    bounds = d.get('search_bounds', {})
    g.search_bounds = _build_polygon_from_global(bounds.get('points', []), ts)
    return g


_GOAL_BUILDERS = {
    'takeoff':          _build_takeoff_goal,
    'land':             _build_land_goal,
    'navigate':         _build_navigate_goal,
    'fixed_trajectory': _build_fixed_trajectory_goal,
    'semantic_search':  _build_semantic_search_goal,
    'exploration':      _build_exploration_goal,
}


# ── Relay wiring ─────────────────────────────────────────────────────────────

def _make_relay(node0, nodeN, executorN, topic, suffix, action_type,
                robot_domain, transform_state):
    """Wire up goal subscriber (domain 0) + action client (domain N)."""

    client = ActionClient(nodeN, action_type, topic)
    cbg = ReentrantCallbackGroup()

    feedback_pub = node0.create_publisher(String, f'{topic}/relay_feedback', _RELIABLE_QOS)
    result_pub = node0.create_publisher(String, f'{topic}/relay_result', _RELIABLE_QOS)

    goal_builder = _GOAL_BUILDERS[suffix]
    active_goal = {'handle': None}
    cancel_event = threading.Event()

    # Tasks exempt from the "must be airborne" precondition.
    AIRBORNE_EXEMPT = {'takeoff', 'land'}
    MIN_AIRBORNE_Z_M = 5.0

    def on_goal_str(msg):
        """Receive JSON goal from Foxglove, parse, forward as action."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            node0.get_logger().error(f'[relay] {topic}: bad JSON: {e}')
            _publish_result(False, f'Invalid JSON: {e}')
            return

        # Airborne precondition: every task except takeoff/land requires the
        # drone to be at least MIN_AIRBORNE_Z_M above its takeoff altitude.
        if suffix not in AIRBORNE_EXEMPT:
            cur_z = transform_state.get('current_z')
            if cur_z is None:
                node0.get_logger().warn(
                    f'[relay] {topic}: rejected — no GPS altitude yet')
                _publish_result(
                    False,
                    'Robot altitude unknown (no GPS fix yet) — takeoff first.')
                return
            if cur_z < MIN_AIRBORNE_Z_M:
                node0.get_logger().warn(
                    f'[relay] {topic}: rejected — drone at {cur_z:.2f}m AGL, '
                    f'need >= {MIN_AIRBORNE_Z_M}m')
                _publish_result(
                    False,
                    f'Takeoff first — drone is at {cur_z:.2f}m AGL '
                    f'(need >= {MIN_AIRBORNE_Z_M:.0f}m).')
                return

        try:
            goal_msg = goal_builder(data, transform_state)
        except RuntimeError as e:
            node0.get_logger().warn(f'[relay] {topic}: {e}')
            _publish_result(False, str(e))
            return
        node0.get_logger().info(f'[relay] {topic}: goal: {goal_msg}')

        if not client.wait_for_server(timeout_sec=5.0):
            node0.get_logger().warn(f'[relay] {topic}: robot server not available')
            _publish_result(False, 'Robot action server not available')
            return

        send_future = client.send_goal_async(
            goal_msg,
            feedback_callback=_on_feedback,
        )

        while not send_future.done():
            executorN.spin_once(timeout_sec=0.05)

        robot_goal_handle = send_future.result()
        if not robot_goal_handle.accepted:
            node0.get_logger().warn(f'[relay] {topic}: robot rejected goal')
            _publish_result(False, 'Robot rejected goal')
            return

        active_goal['handle'] = robot_goal_handle
        cancel_event.clear()
        node0.get_logger().info(f'[relay] {topic}: goal accepted')
        feedback_pub.publish(String(data='Goal accepted by robot'))

        # Drive the result loop. We always end this block by publishing a
        # relay_result so the panel can move on, even if the robot's executor
        # gets stuck on cancel or result_future.result() blows up.
        try:
            result_future = robot_goal_handle.get_result_async()
            cancel_sent = False
            cancel_deadline = None
            CANCEL_TIMEOUT_SEC = 10.0

            while not result_future.done():
                if cancel_event.is_set():
                    if not cancel_sent:
                        # Issue cancel from the same thread that owns executorN
                        # to avoid racing the rcl ActionClient.
                        robot_goal_handle.cancel_goal_async()
                        cancel_sent = True
                        cancel_deadline = time.monotonic() + CANCEL_TIMEOUT_SEC
                        feedback_pub.publish(String(data='Cancel forwarded to robot'))
                    elif cancel_deadline is not None and time.monotonic() > cancel_deadline:
                        node0.get_logger().warn(
                            f'[relay] {topic}: cancel timed out after '
                            f'{CANCEL_TIMEOUT_SEC:.0f}s — forcing CANCELED result')
                        break
                executorN.spin_once(timeout_sec=0.05)

            if result_future.done():
                wrapped = result_future.result()
                robot_result = wrapped.result
                status = wrapped.status
                status_str = {4: 'SUCCEEDED', 5: 'CANCELED'}.get(status, 'ABORTED')
                success = getattr(robot_result, 'success', status == 4)
                message = getattr(robot_result, 'message', status_str)
            else:
                # cancel timed out — server didn't return a result in time
                status_str = 'CANCELED'
                success = False
                message = 'Canceled (server did not respond within timeout)'

            node0.get_logger().info(f'[relay] {topic}: {status_str}')
            _publish_result(success, message)
        except Exception as e:
            node0.get_logger().error(f'[relay] {topic}: result loop failed: {e}')
            try:
                _publish_result(False, f'Result handling failed: {e}')
            except Exception:
                pass
        finally:
            active_goal['handle'] = None
            cancel_event.clear()

    def on_cancel(msg):
        """Signal the goal-handler loop to forward a cancel to the robot.

        If there's no active goal at the relay (already finished, or the
        panel's state desynced from the relay's), republish a synthetic
        result so any stuck panel UI can flip back to idle. A panel that's
        already idle will ignore it (its `t.active` is false).
        """
        if active_goal.get('handle') is None:
            node0.get_logger().warn(f'[relay] {topic}: cancel requested but no active goal')
            _publish_result(
                False,
                'Cancel: no active goal at relay (panel state likely desynced)')
            return
        node0.get_logger().info(f'[relay] {topic}: cancel requested')
        cancel_event.set()

    def _on_feedback(fb_msg):
        fb = fb_msg.feedback
        fields = fb.get_fields_and_field_types()
        fb_dict = {}
        for name in fields:
            val = getattr(fb, name)
            if hasattr(val, 'get_fields_and_field_types'):
                sub = {}
                for sn in val.get_fields_and_field_types():
                    sub[sn] = getattr(val, sn)
                fb_dict[name] = sub
            else:
                fb_dict[name] = val
        feedback_pub.publish(String(data=json.dumps(fb_dict, default=str)))

    def _publish_result(success, message):
        result_pub.publish(String(
            data=json.dumps({'success': success, 'message': message})))

    node0.create_subscription(
        String, f'{topic}/goal', on_goal_str, _RELIABLE_QOS,
        callback_group=cbg,
    )
    node0.create_subscription(
        String, f'{topic}/cancel', on_cancel, _RELIABLE_QOS,
        callback_group=cbg,
    )

    if not hasattr(node0, '_relays'):
        node0._relays = []
    node0._relays.append((client, feedback_pub, result_pub))

    node0.get_logger().info(
        f'[relay] {topic}/goal -> client(domain {robot_domain})')


def main(args=None):
    ctx0 = rclpy.Context()
    ctx0.init(args=args)

    tmp = rclpy.create_node('_action_relay_params', context=ctx0)
    robot_name = tmp.declare_parameter('robot_name', 'robot_1').value
    robot_domain = tmp.declare_parameter('robot_domain', 1).value
    tmp.destroy_node()

    ctxN = rclpy.Context()
    ctxN.init(args=[], domain_id=int(robot_domain))

    node0 = rclpy.create_node('action_relay', context=ctx0)
    nodeN = rclpy.create_node('action_relay_client', context=ctxN)

    executorN = SingleThreadedExecutor(context=ctxN)
    executorN.add_node(nodeN)

    node0.get_logger().info(
        f'Action relay: robot={robot_name} domain={robot_domain} '
        f'relaying {len(RELAYS)} action(s)')

    # Track this robot's boot ENU position so we can convert global-frame
    # waypoints/polygons (from the Foxglove editors) to robot-local before
    # forwarding. First valid GPS fix is treated as boot, matching gcs_visualizer.
    # current_z is the most recent altitude AGL (relative to alt_ground), used
    # to gate non-takeoff tasks so the user can't accidentally send a navigate
    # command while the drone is on the ground.
    transform_state = {'boot': None, 'alt_ground': None, 'current_z': None}
    gps_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    def _on_gps(msg: NavSatFix):
        if msg.status.status < 0:
            return
        if transform_state['alt_ground'] is None:
            transform_state['alt_ground'] = msg.altitude
        if transform_state['boot'] is None:
            pos = gps_to_enu(msg.latitude, msg.longitude, msg.altitude,
                             transform_state['alt_ground'])
            transform_state['boot'] = pos
            node0.get_logger().info(
                f"[relay] {robot_name} boot ENU = "
                f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        transform_state['current_z'] = msg.altitude - transform_state['alt_ground']

    node0.create_subscription(
        NavSatFix,
        f'/{robot_name}/interface/mavros/global_position/global',
        _on_gps,
        gps_qos,
    )

    for suffix, action_type in RELAYS:
        topic = f'/{robot_name}/tasks/{suffix}'
        _make_relay(node0, nodeN, executorN, topic, suffix, action_type,
                    robot_domain, transform_state)

    executor0 = MultiThreadedExecutor(context=ctx0)
    executor0.add_node(node0)

    try:
        executor0.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node0.destroy_node()
        nodeN.destroy_node()
        ctx0.try_shutdown()
        ctxN.try_shutdown()
