#!/usr/bin/env python3
"""Check policy_commander's goal-aligned frame transform without running the commander.

Two modes:

  --selftest   No ROS graph or mocap needed. Builds random drone/ball/goal states and
               checks policy_commander's observation and action transforms against an
               independent implementation written here from the frame's definition,
               plus the frame's invariances (rotating or shifting the whole scene must
               leave the observation unchanged and rotate the world action with it).
               Runs in the robot container or on the host (drone_soccer's gpu env);
               ROS modules are stubbed when they are missing.

  (default)    Live. Subscribes to the drone odometry and ball mocap that the commander
               reads and, every --period seconds, prints the goal-aligned observation
               labelled in plain words, its difference from the independent
               implementation, a check of the odometry velocity against the
               finite-differenced position, and (with a policy loaded) the action the
               policy would take and the waypoint it would send. Run it in the robot
               container with mocap and the drone interface up. Move the drone and
               ball by hand and watch the readout.

This script only subscribes. It creates no publishers, services or clients, so it
never commands the drone. It imports the commander's functions from this source
checkout (not the colcon install space), so it checks the code you would launch
after a rebuild.

Usage, inside the robot container (workspace sourced):
    python3 /root/AirStack/robot/ros_ws/src/svg_ground_control/scripts/check_goal_frame.py --selftest
    python3 .../check_goal_frame.py --goal 1.0 -2.0
    python3 .../check_goal_frame.py --goal-topic /policy_commander/goal
    python3 .../check_goal_frame.py --goal 0 0 --model none     # transform only, no policy
On the host:
    cd ~/Desktop/drone_soccer && PYTHONPATH=$PWD ~/miniforge3/envs/gpu/bin/python \\
        ~/Desktop/AirStack/robot/ros_ws/src/svg_ground_control/scripts/check_goal_frame.py --selftest
"""

from __future__ import annotations

import argparse
import math
import sys
import types
from collections import deque
from pathlib import Path

import numpy as np

PACKAGE_SRC = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = '/root/drone_soccer/drone_soccer/saved_policies/goal_aligned.zip'
# Commander defaults from policy_commander_drone3_goal_retarget.yaml.
DEFAULT_ACTION_LOW = (-3.0, -3.0, 0.5)
DEFAULT_ACTION_HIGH = (3.0, 3.0, 1.5)
DEFAULT_BOUNDS_MIN = (-4.7, -5.5, 0.5)
DEFAULT_BOUNDS_MAX = (5.0, 5.0, 1.5)
DEFAULT_GEOFENCE_BUFFER_M = 0.5
DEFAULT_MAX_GOAL_DISTANCE_M = 4.0

# Agreement needed between the commander's functions and the reference (float32 math).
TOLERANCE = 1e-4

OBS_LABELS = (
    'drone x (frame)', 'drone y (frame)', 'drone z (altitude)',
    'vel x', 'vel y', 'vel z',
    'body-z x', 'body-z y', 'body-z z',
    'ang vel x (body)', 'ang vel y (body)', 'ang vel z (body)',
    'ball vel x', 'ball vel y',
    'goal->ball dist',
)


def _install_ros_stubs() -> None:
    """Stand-ins for the ROS modules policy_commander imports, for hosts without ROS.

    Only the pure functions are used in --selftest, so each stubbed name just needs
    to exist.
    """
    class _StubModule(types.ModuleType):
        def __getattr__(self, name: str):
            if name.startswith('__'):
                raise AttributeError(name)
            stub = type(name, (), {'__init__': lambda self, *a, **k: None})
            setattr(self, name, stub)
            return stub

    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'geometry_msgs', 'geometry_msgs.msg',
                 'nav_msgs', 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg', 'std_srvs',
                 'std_srvs.srv', 'px4_msgs', 'px4_msgs.msg'):
        sys.modules.setdefault(name, _StubModule(name))


def _import_commander(allow_stubs: bool):
    sys.path.insert(0, str(PACKAGE_SRC))
    try:
        import rclpy  # noqa: F401
    except ImportError:
        if not allow_stubs:
            sys.exit('rclpy not found: run live mode inside the robot container '
                     '(or use --selftest, which works without ROS)')
        _install_ros_stubs()
        print('[note] rclpy not found; ROS modules stubbed (fine for --selftest)')
    import svg_ground_control
    from svg_ground_control import policy_commander
    source = Path(svg_ground_control.__file__).resolve().parent
    if source != PACKAGE_SRC / 'svg_ground_control':
        print(f'[warn] svg_ground_control imported from {source}, not this checkout')
    return policy_commander


# ---------------------------------------------------------------------------------
# Independent reference, written from the frame's definition (drone_soccer
# deploy/goal_frame.py and deploy/observation.py docstrings). It deliberately does
# not call drone_soccer or policy_commander.
#
# Goal-aligned frame: origin at the (capped) goal on the floor, x-axis from the goal
# toward the ball, y-axis x rotated +90 deg about world z (up), z-axis up.
# ---------------------------------------------------------------------------------

def ref_shown_goal(goal_xy: np.ndarray, ball_xy: np.ndarray, cap_m: float) -> np.ndarray:
    """The goal the policy sees: pulled toward the ball to at most cap_m away."""
    offset = goal_xy - ball_xy
    dist = math.hypot(*offset)
    scale = min(1.0, cap_m / max(dist, 1e-9))
    return ball_xy + offset * scale


def ref_axes(goal_xy: np.ndarray, ball_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Frame x and y axes in world coordinates."""
    d = ball_xy - goal_xy
    n = math.hypot(*d)
    ex = d / n if n > 1e-6 else np.array([1.0, 0.0])
    ey = np.array([-ex[1], ex[0]])
    return ex, ey


def ref_body_z(q_wxyz: np.ndarray) -> np.ndarray:
    """Third column of the body-to-world rotation (Hamilton, w-first)."""
    w, x, y, z = np.asarray(q_wxyz, dtype=float) / np.linalg.norm(q_wxyz)
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def ref_observation(state: dict, goal_xy: np.ndarray, cap_m: float) -> np.ndarray:
    ball_xy = np.asarray(state['ball_position'][:2], dtype=float)
    shown = ref_shown_goal(np.asarray(goal_xy, dtype=float), ball_xy, cap_m)
    ex, ey = ref_axes(shown, ball_xy)

    def rot(v):
        v = np.asarray(v, dtype=float)
        return np.concatenate([[v[:2] @ ex, v[:2] @ ey], v[2:]])

    p = np.asarray(state['drone_position'], dtype=float)
    return np.concatenate([
        rot([p[0] - shown[0], p[1] - shown[1], p[2]]),
        rot(state['drone_velocity']),
        rot(ref_body_z(state['drone_quat_wxyz'])),
        np.asarray(state['drone_angular_velocity_body'], dtype=float),
        rot(np.asarray(state['ball_velocity'], dtype=float)[:2]),
        [math.hypot(*(ball_xy - shown))],
    ])


def ref_action_to_world(frame_action: np.ndarray, ball_xy: np.ndarray,
                        goal_xy: np.ndarray) -> np.ndarray:
    """(dx, dy) along the frame axes -> world (dx, dy); z is a world altitude already."""
    ex, ey = ref_axes(np.asarray(goal_xy, dtype=float), np.asarray(ball_xy, dtype=float))
    a = np.asarray(frame_action, dtype=float)
    xy = a[0] * ex + a[1] * ey
    return np.array([xy[0], xy[1], a[2]])


# ---------------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------------

def heading_deg(v: np.ndarray) -> float:
    """ENU heading of an xy vector: 0 = east (+x), 90 = north (+y)."""
    return math.degrees(math.atan2(v[1], v[0])) % 360.0


def yaw_deg(q_wxyz: np.ndarray) -> float:
    w, x, y, z = q_wxyz
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360.0


def angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float('nan')
    return math.degrees(math.acos(np.clip(a @ b / (na * nb), -1.0, 1.0)))


def fmt(v, digits: int = 3) -> str:
    return '[' + ', '.join(f'{float(x):+.{digits}f}' for x in np.ravel(v)) + ']'


def rotz(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def load_policy(pc, model: str, action_low, action_high, policy_type: str):
    if model.lower() == 'none':
        return None
    policy, compat = pc.load_policy_checkpoint(model, action_low, action_high, policy_type)
    frame = pc.policy_frame_of(policy)
    print(f'[policy] {model} | frame={frame}'
          f'{" | NumPy-2 compat loader" if compat else ""}')
    if frame != 'goal_aligned':
        print('[warn] this checkpoint is a world-frame policy; the commander would '
              'not apply the goal-aligned transform to it')
    return policy


def frame_action_of(policy, obs: np.ndarray, action_low, action_high) -> np.ndarray:
    """The clipped action in the policy's frame, as PolicyCommander._policy_action does."""
    action, _ = policy.predict(obs, deterministic=True)
    return np.clip(np.asarray(action, dtype=np.float32), action_low, action_high)


# ---------------------------------------------------------------------------------
# --selftest
# ---------------------------------------------------------------------------------

def random_state(rng: np.random.Generator) -> tuple[dict, np.ndarray]:
    q = rng.standard_normal(4)
    q[0] = abs(q[0]) + 2.0  # mostly near level, like flight
    q /= np.linalg.norm(q)
    state = {
        'drone_position': np.r_[rng.uniform(-4, 4, 2), rng.uniform(0.5, 1.5)].astype(np.float32),
        'drone_quat_wxyz': q,
        'drone_velocity': rng.uniform(-2, 2, 3).astype(np.float32),
        'drone_angular_velocity_body': rng.uniform(-2, 2, 3).astype(np.float32),
        'ball_position': np.r_[rng.uniform(-4, 4, 2), rng.uniform(0.1, 0.4)].astype(np.float32),
        'ball_velocity': rng.uniform(-2, 2, 3).astype(np.float32),
    }
    # Goals up to ~8 m from the ball, so the 4 m cap is exercised too.
    goal = (state['ball_position'][:2] + rng.uniform(-6, 6, 2)).astype(np.float32)
    return state, goal


def transform_scene(state: dict, goal: np.ndarray, theta: float,
                    shift_xy: np.ndarray) -> tuple[dict, np.ndarray]:
    """Rotate the whole scene by theta about world z, then shift it in xy."""
    r = rotz(theta)
    shift = np.r_[shift_xy, 0.0]
    q_rot = np.array([math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2)])
    return {
        'drone_position': (r @ state['drone_position'] + shift).astype(np.float32),
        'drone_quat_wxyz': quat_mul(q_rot, state['drone_quat_wxyz']),
        'drone_velocity': (r @ state['drone_velocity']).astype(np.float32),
        'drone_angular_velocity_body': state['drone_angular_velocity_body'],
        'ball_position': (r @ state['ball_position'] + shift).astype(np.float32),
        'ball_velocity': (r @ state['ball_velocity']).astype(np.float32),
    }, (r[:2, :2] @ goal + shift_xy).astype(np.float32)


def selftest(args: argparse.Namespace) -> int:
    pc = _import_commander(allow_stubs=True)
    rng = np.random.default_rng(args.seed)
    cap = args.max_goal_distance
    low = np.asarray(args.action_low, dtype=np.float32)
    high = np.asarray(args.action_high, dtype=np.float32)

    policy = None
    if args.model.lower() != 'none' and Path(args.model).exists():
        policy = load_policy(pc, args.model, low, high, args.policy_type)
    elif args.model.lower() != 'none':
        print(f'[note] {args.model} not found; policy checks skipped '
              '(pass --model <path> or --model none)')

    worst = {'obs vs reference': 0.0, 'world action vs reference': 0.0,
             'obs rotation/shift invariance': 0.0, 'world action rotation equivariance': 0.0,
             'waypoint = drone + action (inside fence)': 0.0}
    cap_hits = 0
    for _ in range(args.samples):
        state, goal = random_state(rng)
        cap_hits += np.linalg.norm(goal - state['ball_position'][:2]) > cap

        obs = pc.build_policy_observation('goal_aligned', state, goal, cap)
        assert obs.shape == (15,), obs.shape
        worst['obs vs reference'] = max(
            worst['obs vs reference'],
            float(np.max(np.abs(obs - ref_observation(state, goal, cap)))))

        frame_action = (frame_action_of(policy, obs, low, high) if policy is not None
                        else rng.uniform(low, high).astype(np.float32))
        world = pc.policy_action_to_world(
            'goal_aligned', frame_action, state['ball_position'][:2], goal)
        worst['world action vs reference'] = max(
            worst['world action vs reference'],
            float(np.max(np.abs(world - ref_action_to_world(
                frame_action, state['ball_position'][:2], goal)))))

        # Waypoint: away from the fence it is exactly drone xy + action xy, z = action z.
        center_state = dict(state, drone_position=np.array(
            [0.0, 0.0, state['drone_position'][2]], dtype=np.float32))
        waypoint = pc.compute_bounded_waypoint(
            center_state['drone_position'], world, np.array(DEFAULT_BOUNDS_MIN),
            np.array(DEFAULT_BOUNDS_MAX), DEFAULT_GEOFENCE_BUFFER_M)
        expected = np.r_[world[:2], np.clip(world[2], DEFAULT_BOUNDS_MIN[2], DEFAULT_BOUNDS_MAX[2])]
        worst['waypoint = drone + action (inside fence)'] = max(
            worst['waypoint = drone + action (inside fence)'],
            float(np.max(np.abs(waypoint - expected))))

        theta = rng.uniform(-math.pi, math.pi)
        moved_state, moved_goal = transform_scene(state, goal, theta, rng.uniform(-1, 1, 2))
        moved_obs = pc.build_policy_observation('goal_aligned', moved_state, moved_goal, cap)
        worst['obs rotation/shift invariance'] = max(
            worst['obs rotation/shift invariance'], float(np.max(np.abs(moved_obs - obs))))

        # Same frame action in the moved scene -> world action rotated by theta.
        moved_world = pc.policy_action_to_world(
            'goal_aligned', frame_action, moved_state['ball_position'][:2], moved_goal)
        worst['world action rotation equivariance'] = max(
            worst['world action rotation equivariance'],
            float(np.max(np.abs(moved_world - rotz(theta) @ world))))

    # NED conversion of a known ENU point.
    ned = pc.enu_waypoint_to_ned(np.array([1.0, 2.0, 3.0]))
    ned_ok = np.allclose(ned, [2.0, 1.0, -3.0])

    # A hand-checkable case: goal at origin, ball 2 m east, drone 1 m further east
    # (behind the ball), 0.5 m north, level, moving north.
    hand_state = {
        'drone_position': np.array([3.0, 0.5, 1.0], dtype=np.float32),
        'drone_quat_wxyz': np.array([1.0, 0.0, 0.0, 0.0]),
        'drone_velocity': np.array([0.0, 1.0, 0.0], dtype=np.float32),
        'drone_angular_velocity_body': np.zeros(3, dtype=np.float32),
        'ball_position': np.array([2.0, 0.0, 0.2], dtype=np.float32),
        'ball_velocity': np.array([-1.0, 0.0, 0.0], dtype=np.float32),
    }
    hand_obs = pc.build_policy_observation('goal_aligned', hand_state, np.zeros(2), cap)
    hand_expected = np.array([3, 0.5, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 2], dtype=np.float32)
    # Rotate that scene 90 deg CCW (ball 2 m north of the goal): frame x now points
    # north, so a frame action of (-1, 0) (toward the goal) must be world (0, -1).
    hand_world = pc.policy_action_to_world(
        'goal_aligned', np.array([-1.0, 0.0, 1.0]), np.array([0.0, 2.0]), np.zeros(2))

    print(f'\nself-test: {args.samples} random states ({cap_hits} with the goal beyond '
          f'the {cap:g} m cap), {"policy actions" if policy is not None else "random actions"}')
    failed = False
    for name, err in worst.items():
        ok = err < TOLERANCE
        failed |= not ok
        print(f'  {"PASS" if ok else "FAIL"}  {name:42s} max |diff| {err:.2e}')
    for name, ok, detail in (
        ('ENU -> NED waypoint (1,2,3) -> (2,1,-3)', ned_ok, fmt(ned)),
        ('hand case: obs', np.allclose(hand_obs, hand_expected, atol=1e-5), fmt(hand_obs, 2)),
        ('hand case: ball north of goal, toward goal', np.allclose(hand_world, [0, -1, 1], atol=1e-5),
         fmt(hand_world, 2)),
    ):
        failed |= not ok
        print(f'  {"PASS" if ok else "FAIL"}  {name:42s} {detail}')
    print('\nRESULT:', 'FAIL' if failed else 'all transform checks passed')
    return 1 if failed else 0


# ---------------------------------------------------------------------------------
# Live mode
# ---------------------------------------------------------------------------------

def live(args: argparse.Namespace) -> int:
    pc = _import_commander(allow_stubs=False)
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    from svg_ground_control.ball_state import BallStateTracker

    low = np.asarray(args.action_low, dtype=np.float32)
    high = np.asarray(args.action_high, dtype=np.float32)
    policy = load_policy(pc, args.model, low, high, args.policy_type)

    class GoalFrameChecker(Node):
        def __init__(self) -> None:
            super().__init__('goal_frame_checker')
            self.goal = None if args.goal is None else np.asarray(args.goal, dtype=np.float32)
            self.odom = None
            self.odom_rx = None
            self.ball_rx = None
            self.ball_from_odom = False
            self.ball = BallStateTracker(velocity_alpha=0.4)
            # (stamp_s, position, twist linear, body-to-world rotation) per odom message
            self.history = deque(maxlen=400)
            self.frames_printed = False
            self.start = self._now()

            odom_topic = f'/{args.drone}/odometry_conversion/odometry'
            ball_odom_topic = f'/{args.ball}/mocap_odometry'
            ball_pose_topic = f'/{args.ball}/pose'
            self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
            self.create_subscription(Odometry, ball_odom_topic, self._ball_odom_cb, 10)
            self.create_subscription(PoseStamped, ball_pose_topic, self._ball_pose_cb, 10)
            if args.goal_topic:
                self.create_subscription(
                    Float32MultiArray, args.goal_topic, self._goal_cb, 10)
            self.create_timer(args.period, self._report)
            goal_src = args.goal_topic if args.goal_topic else f'fixed {fmt(self.goal, 2)}'
            print(f'[live] drone {odom_topic} | ball {ball_odom_topic} (or {ball_pose_topic}) '
                  f'| goal {goal_src}\n[live] subscribe-only: nothing is published')

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _odom_cb(self, msg) -> None:
            self.odom = msg
            self.odom_rx = self._now()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            self.history.append((
                stamp or self.odom_rx,
                np.array([p.x, p.y, p.z]),
                np.array([v.x, v.y, v.z]),
                pc.quaternion_to_rotation_matrix(q.w, q.x, q.y, q.z),
            ))
            if not self.frames_printed:
                self.frames_printed = True
                print(f'[live] drone odom frame_id={msg.header.frame_id!r} '
                      f'child_frame_id={msg.child_frame_id!r}')

        def _ball_odom_cb(self, msg) -> None:
            self.ball_from_odom = True
            self.ball.update_from_odometry(msg)
            self.ball_rx = self._now()

        def _ball_pose_cb(self, msg) -> None:
            if not self.ball_from_odom:
                self.ball.update_from_pose(msg)
                self.ball_rx = self._now()

        def _goal_cb(self, msg) -> None:
            self.goal = np.asarray(msg.data[:2], dtype=np.float32)

        def _velocity_check(self):
            """Odometry twist vs finite-differenced position over the last ~0.3 s.

            Returns (d(pos)/dt, mean twist as world, mean twist rotated body->world),
            each averaged over the same window so a turn does not look like a
            mismatch, or None while there is too little history.
            """
            if len(self.history) < 2:
                return None
            t1, p1 = self.history[-1][:2]
            window = [h for h in self.history if t1 - h[0] <= 0.3]
            t0, p0 = window[0][:2]
            if t1 - t0 < 0.15:
                return None
            fd = (p1 - p0) / (t1 - t0)
            as_world = np.mean([h[2] for h in window], axis=0)
            as_body = np.mean([h[3] @ h[2] for h in window], axis=0)
            return fd, as_world, as_body

        def _report(self) -> None:
            now = self._now()
            missing = []
            if self.odom is None or now - self.odom_rx > args.stale:
                missing.append('drone odometry')
            if self.ball.state is None or now - self.ball_rx > args.stale:
                missing.append('ball mocap')
            if self.goal is None:
                missing.append(f'goal ({args.goal_topic})')
            if missing:
                print(f'[live] waiting for: {", ".join(missing)}')
                return

            o = self.odom
            p, q, t = o.pose.pose.position, o.pose.pose.orientation, o.twist.twist
            ball = self.ball.state
            state = {  # exactly what PolicyCommander._read_state builds
                'drone_position': np.array([p.x, p.y, p.z], dtype=np.float32),
                'drone_quat_wxyz': np.array([q.w, q.x, q.y, q.z], dtype=np.float64),
                'drone_velocity': np.array([t.linear.x, t.linear.y, t.linear.z], dtype=np.float32),
                'drone_angular_velocity_body': np.array(
                    [t.angular.x, t.angular.y, t.angular.z], dtype=np.float32),
                'ball_position': np.asarray(ball.position),
                'ball_velocity': np.asarray(ball.velocity),
            }
            goal = self.goal
            cap = args.max_goal_distance
            obs = pc.build_policy_observation('goal_aligned', state, goal, cap)
            ref = ref_observation(state, goal, cap)
            obs_err = float(np.max(np.abs(obs - ref)))

            ball_xy = state['ball_position'][:2].astype(float)
            shown = ref_shown_goal(goal.astype(float), ball_xy, cap)
            ex, _ = ref_axes(shown, ball_xy)
            capped = not np.allclose(shown, goal, atol=1e-6)

            lines = [f'\n=== t={now - self.start:.1f}s ' + '=' * 60]
            lines.append(
                f'ENU   drone {fmt(state["drone_position"], 2)} yaw {yaw_deg(state["drone_quat_wxyz"]):5.1f} deg'
                f' (0=east, 90=north) | ball {fmt(state["ball_position"], 2)}')
            lines.append(
                f'      goal {fmt(goal, 2)}' + (f' -> shown (capped to {cap:g} m) {fmt(shown, 2)}' if capped else '')
                + f' | frame +x (goal->ball) heading {heading_deg(ex):5.1f} deg')
            for i in range(0, 15, 3):
                lines.append('obs   ' + '   '.join(
                    f'[{j:2d}] {OBS_LABELS[j]:16s} {obs[j]:+7.3f}' for j in range(i, min(i + 3, 15))))

            behind = obs[0] - obs[14]
            side = obs[1]
            lines.append(
                f'      -> drone is {abs(behind):.2f} m '
                f'{"BEHIND the ball (far side from goal)" if behind > 0 else "BETWEEN ball and goal (goal side)"}'
                f', {abs(side):.2f} m {"LEFT" if side > 0 else "RIGHT"} of the goal->ball line'
                f' (looking from goal to ball); ball {obs[14]:.2f} m from the shown goal')
            lines.append(
                f'      -> body-z {fmt(obs[6:9], 2)}: '
                + ('level' if obs[8] > 0.98 else f'tilted {math.degrees(math.acos(np.clip(obs[8], -1, 1))):.0f} deg'))
            lines.append(f'check obs vs independent reference: max |diff| {obs_err:.1e}  '
                         + ('OK' if obs_err < TOLERANCE else '*** MISMATCH ***'))

            # The commander feeds odom twist.linear to the policy as WORLD velocity.
            # Check that against the position track; a body-frame twist would only
            # agree after rotating it by the drone's attitude.
            check = self._velocity_check()
            if check is not None:
                fd, as_world, as_body = check
                speed = np.linalg.norm(fd[:2])
                if speed < 0.3:
                    verdict = 'move the drone (> 0.3 m/s) to check'
                else:
                    err_world = np.linalg.norm(as_world[:2] - fd[:2])
                    err_body = np.linalg.norm(as_body[:2] - fd[:2])
                    yaw = (yaw_deg(state['drone_quat_wxyz']) + 180.0) % 360.0 - 180.0
                    verdict = f'err if world {err_world:.2f}, if body {err_body:.2f} m/s -> '
                    if abs(yaw) < 30.0 and abs(err_world - err_body) < 0.15:
                        verdict += (f'cannot tell at yaw {yaw:+.0f} deg; '
                                    'yaw the drone 90 deg and check again')
                    elif err_world <= err_body and err_world < 0.25 + 0.2 * speed:
                        verdict += 'OK (twist is world-frame, as the commander assumes)'
                    elif err_body < err_world:
                        verdict += '*** twist looks BODY-frame; the commander treats it as world ***'
                    else:
                        verdict += '*** twist does not match the position track ***'
                lines.append(f'vel   odom twist {fmt(state["drone_velocity"], 2)} | '
                             f'd(pos)/dt {fmt(fd, 2)}: {verdict}')

            if policy is not None:
                frame_action = frame_action_of(policy, obs, low, high)
                world = pc.policy_action_to_world('goal_aligned', frame_action, ball_xy, goal)
                ref_world = ref_action_to_world(frame_action, ball_xy, goal)
                act_err = float(np.max(np.abs(world - ref_world)))
                waypoint = pc.compute_bounded_waypoint(
                    state['drone_position'], world, np.asarray(args.bounds_min),
                    np.asarray(args.bounds_max), args.geofence_buffer)
                ned = pc.enu_waypoint_to_ned(waypoint)
                to_goal = goal.astype(float) - ball_xy
                lines.append(
                    f'act   frame {fmt(frame_action, 2)} (x: toward ball/away from goal, y: left) '
                    f'-> world dx,dy,z {fmt(world, 2)}')
                lines.append(
                    f'      waypoint ENU {fmt(waypoint, 2)} | NED {fmt(ned, 2)} | move heading '
                    f'{heading_deg(world[:2]):5.1f} deg, {angle_between_deg(world[:2], to_goal):.0f} deg '
                    f'from the ball->goal direction')
                lines.append(f'check world action vs independent reference: max |diff| {act_err:.1e}  '
                             + ('OK' if act_err < TOLERANCE else '*** MISMATCH ***'))
                lines.append('      (not sent: this script publishes nothing)')
            print('\n'.join(lines), flush=True)

    rclpy.init()
    node = GoalFrameChecker()
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--selftest', action='store_true',
                        help='offline checks on random states; no ROS graph needed')
    goal = parser.add_mutually_exclusive_group()
    goal.add_argument('--goal', type=float, nargs=2, metavar=('X', 'Y'),
                      help='fixed ENU goal xy (m) for live mode (default 0 0)')
    goal.add_argument('--goal-topic', metavar='TOPIC',
                      help='read the goal from a Float32MultiArray topic '
                           '(e.g. /policy_commander/goal) instead of --goal')
    parser.add_argument('--model', default=DEFAULT_MODEL,
                        help='policy checkpoint for action checks, or "none" '
                             '(default: %(default)s)')
    parser.add_argument('--policy-type', default='standard', choices=('standard', 'residual'))
    parser.add_argument('--drone', default='drone_3')
    parser.add_argument('--ball', default='VolleyBall')
    parser.add_argument('--period', type=float, default=0.5, help='live print period (s)')
    parser.add_argument('--stale', type=float, default=0.5,
                        help='inputs older than this (s) count as missing')
    parser.add_argument('--max-goal-distance', type=float, default=DEFAULT_MAX_GOAL_DISTANCE_M)
    parser.add_argument('--action-low', type=float, nargs=3, default=DEFAULT_ACTION_LOW)
    parser.add_argument('--action-high', type=float, nargs=3, default=DEFAULT_ACTION_HIGH)
    parser.add_argument('--bounds-min', type=float, nargs=3, default=DEFAULT_BOUNDS_MIN)
    parser.add_argument('--bounds-max', type=float, nargs=3, default=DEFAULT_BOUNDS_MAX)
    parser.add_argument('--geofence-buffer', type=float, default=DEFAULT_GEOFENCE_BUFFER_M)
    parser.add_argument('--samples', type=int, default=2000, help='--selftest states')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    if not args.selftest and args.goal is None and args.goal_topic is None:
        args.goal = [0.0, 0.0]
    return selftest(args) if args.selftest else live(args)


if __name__ == '__main__':
    sys.exit(main())
