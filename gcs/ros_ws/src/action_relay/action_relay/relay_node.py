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

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Polygon, Point32, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, String

from airstack_msgs.msg import FixedTrajectory
from diagnostic_msgs.msg import KeyValue
from task_msgs.action import (
    TakeoffTask, LandTask, NavigateTask,
    FixedTrajectoryTask, SemanticSearchTask,
)

# ── Registry ─────────────────────────────────────────────────────────────────
RELAYS = [
    ('takeoff',          TakeoffTask),
    ('land',             LandTask),
    ('navigate',         NavigateTask),
    ('fixed_trajectory', FixedTrajectoryTask),
    ('semantic_search',  SemanticSearchTask),
]

_RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


# ── Goal builders ────────────────────────────────────────────────────────────
# Each converts a JSON dict into a typed Goal message.

def _build_takeoff_goal(d):
    g = TakeoffTask.Goal()
    g.target_altitude_m = float(d.get('target_altitude_m', 0))
    g.velocity_m_s = float(d.get('velocity_m_s', 0))
    return g


def _build_land_goal(d):
    g = LandTask.Goal()
    g.velocity_m_s = float(d.get('velocity_m_s', 0))
    return g


def _build_navigate_goal(d):
    g = NavigateTask.Goal()
    g.goal_tolerance_m = float(d.get('goal_tolerance_m', 1.0))
    plan_data = d.get('global_plan', {})
    header = Header()
    hdr = plan_data.get('header', {})
    header.frame_id = str(hdr.get('frame_id', 'map'))
    stamp = hdr.get('stamp', {})
    header.stamp = Time(sec=int(stamp.get('sec', 0)),
                        nanosec=int(stamp.get('nanosec', 0)))
    path = Path(header=header)
    for pose_data in plan_data.get('poses', []):
        ps = PoseStamped()
        ps.header = header
        pos = pose_data.get('pose', {}).get('position', {})
        ori = pose_data.get('pose', {}).get('orientation', {})
        ps.pose.position = Point(
            x=float(pos.get('x', 0)), y=float(pos.get('y', 0)),
            z=float(pos.get('z', 0)))
        ps.pose.orientation.x = float(ori.get('x', 0))
        ps.pose.orientation.y = float(ori.get('y', 0))
        ps.pose.orientation.z = float(ori.get('z', 0))
        ps.pose.orientation.w = float(ori.get('w', 1))
        path.poses.append(ps)
    g.global_plan = path
    return g


def _build_fixed_trajectory_goal(d):
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


def _build_semantic_search_goal(d):
    g = SemanticSearchTask.Goal()
    g.query = str(d.get('query', ''))
    g.background_queries = str(d.get('background_queries', ''))
    g.min_altitude_agl = float(d.get('min_altitude_agl', 3.0))
    g.max_altitude_agl = float(d.get('max_altitude_agl', 15.0))
    g.min_flight_speed = float(d.get('min_flight_speed', 0))
    g.max_flight_speed = float(d.get('max_flight_speed', 0))
    g.confidence_threshold = float(d.get('confidence_threshold', 0.95))
    # search_area polygon
    area = d.get('search_area', {})
    poly = Polygon()
    for pt in area.get('points', []):
        poly.points.append(Point32(
            x=float(pt.get('x', 0)), y=float(pt.get('y', 0)),
            z=float(pt.get('z', 0))))
    g.search_area = poly
    return g


_GOAL_BUILDERS = {
    'takeoff':          _build_takeoff_goal,
    'land':             _build_land_goal,
    'navigate':         _build_navigate_goal,
    'fixed_trajectory': _build_fixed_trajectory_goal,
    'semantic_search':  _build_semantic_search_goal,
}


# ── Relay wiring ─────────────────────────────────────────────────────────────

def _make_relay(node0, nodeN, executorN, topic, suffix, action_type, robot_domain):
    """Wire up goal subscriber (domain 0) + action client (domain N)."""

    client = ActionClient(nodeN, action_type, topic)
    cbg = ReentrantCallbackGroup()

    feedback_pub = node0.create_publisher(String, f'{topic}/relay_feedback', _RELIABLE_QOS)
    result_pub = node0.create_publisher(String, f'{topic}/relay_result', _RELIABLE_QOS)

    goal_builder = _GOAL_BUILDERS[suffix]
    active_goal = {'handle': None}

    def on_goal_str(msg):
        """Receive JSON goal from Foxglove, parse, forward as action."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            node0.get_logger().error(f'[relay] {topic}: bad JSON: {e}')
            _publish_result(False, f'Invalid JSON: {e}')
            return

        goal_msg = goal_builder(data)
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
        node0.get_logger().info(f'[relay] {topic}: goal accepted')
        feedback_pub.publish(String(data='Goal accepted by robot'))

        result_future = robot_goal_handle.get_result_async()
        while not result_future.done():
            executorN.spin_once(timeout_sec=0.05)

        active_goal['handle'] = None
        wrapped = result_future.result()
        robot_result = wrapped.result
        status = wrapped.status
        status_str = {4: 'SUCCEEDED', 5: 'CANCELED'}.get(status, 'ABORTED')
        node0.get_logger().info(f'[relay] {topic}: {status_str}')

        success = getattr(robot_result, 'success', status == 4)
        message = getattr(robot_result, 'message', status_str)
        _publish_result(success, message)

    def on_cancel(msg):
        """Cancel the active goal on the robot."""
        gh = active_goal.get('handle')
        if gh is None:
            node0.get_logger().warn(f'[relay] {topic}: cancel requested but no active goal')
            return
        node0.get_logger().info(f'[relay] {topic}: forwarding cancel to robot')
        gh.cancel_goal_async()

    def _on_feedback(fb_msg):
        fb = fb_msg.feedback
        fields = fb.get_fields_and_field_types()
        fb_dict = {}
        for name in fields:
            val = getattr(fb, name)
            # Convert ROS message types to dicts for JSON
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

    for suffix, action_type in RELAYS:
        topic = f'/{robot_name}/tasks/{suffix}'
        _make_relay(node0, nodeN, executorN, topic, suffix, action_type, robot_domain)

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
