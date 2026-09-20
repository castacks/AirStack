"""Central multi-drone ground commander with a CBF collision safety filter.

One node commands the whole swarm through the AirStack robot_interface
abstraction (works unchanged over MAVROS in sim and px4_interface/uXRCE-DDS
on hardware — only the topic templates in the config YAML differ):

    state in:    {state_topic_template}            nav_msgs/Odometry (ENU)
    goals in:    /svg/{name}/goal_command   PoseStamped (position + optional
                 orientation; an all-zero quaternion = nose on +X)
                 /svg/{name}/goal_xyzt      Float64MultiArray [x, y, z, theta]
                 theta in DEGREES, 0 = +X, CLOCKWISE positive (seen from above)
    command out: {velocity_command_topic_template} geometry_msgs/TwistStamped (ENU)
                 real drones (real_command_mode: trajectory):
                 {real_trajectory_command_topic_template}
                     trajectory_msgs/MultiDOFJointTrajectory, one point =
                     position + velocity + acceleration (ENU)
    services:    {robot_command_service_template}  airstack_msgs/srv/RobotCommand

Nominal commands come from a *scenario* (hover, random_walk, random_goals,
head_on, antipodal, squeeze — see scenarios.py, ported from ~/drone_soccer).
Drones listed in ``teleop_drones`` are operator-driven instead (one teleop
topic per drone); an empty list means every drone follows the scenario.
Drones in ``external_drones`` are tracked for the safety filter but never
commanded (e.g. RC-flown). Their MEASURED velocity (scaled by
``cbf_external_velocity_gain``) is fed into the filter as a FIXED row, so the
commanded drones react to their true approach speed and absorb the full
evasion themselves.

Every commanded velocity passes through the velocity-CBF filter
(cbf_filter.filter_velocities, ported from drone_soccer/cbf.py). Drones listed
in ``cbf_exempt_drones`` are CBF-EXEMPT — the filter still sees them (everyone
else dodges them) but leaves their own command uncorrected, so they play the
moving obstacle. Exemption is independent of role: a policy-driven (auto) drone
or a teleop drone can be exempt. (The 'squeeze' scenario additionally
self-designates its intruder via squeeze_intruder_cbf_exempt; the two union.)

Per-drone sim/real routing: ``drone_modes`` (comma-separated 'sim'/'real',
one per drone) routes each drone's command topic + robot_command service to
either the MAVROS/sim interface (``/{name}/interface/...``) or the
px4_interface/uXRCE-DDS hardware interface (``/{name}/fmu/...``). The state
topic is identical for both. This lets one run mix real and simulated drones
(e.g. squeeze with real holders + a simulated intruder), all in one CBF.

Position hold and trajectories (trajectory.py, position_hold.py): every
commanded drone has a REFERENCE POINT — where it was told to be, integrated
from the velocity actually published (post-CBF, post-fence), seeded at the
drone whenever control changes hands and leashed so it never runs far from a
drone that is held back. Scenario drones fly an acceleration-limited profile
toward their goal evaluated at that reference; teleop sticks are a velocity
that moves it. Real drones (``real_command_mode: trajectory``) get the
reference, the velocity and the acceleration in ONE setpoint, so PX4 closes
the position loop onboard with feedforward — as tight as its own Position
mode — and holds position instead of drifting whenever the velocity is zero.
Velocity-only drones (sim/MAVROS) fly the stateless braking law at their own
position, plus the commander's P term on the reference for teleop.

Geofence: with ``fence_enabled`` and the box [``fence_min``, ``fence_max``],
``fence_behavior`` picks what happens:
    hold_all — any ACTIVE drone (any role) outside the box latches a breach:
               every drone freezes, the scenario stops, ``start`` is blocked
               until ``~/reset_fence``.
    keep_in  — nobody stops. Every commanded drone's velocity is clipped per
               axis so it cannot cross a wall and is pushed back inside if it
               is out (fence.keep_in_velocity); the reference point is
               clamped into the box. External drones cannot be steered, so
               they are only reported.

Visualization: every drone's WORLD position (offset-corrected, so real and
simulated drones share one frame) is published as a MarkerArray on
``/svg/viz/markers`` for RViz.

Lifecycle (std_srvs/Trigger services):
    ~/takeoff — arm + offboard + ascend everyone to the scenario's initial
                positions, then HOLD there
    ~/start   — begin the scenario (nominal policies go live)
    ~/hold    — pause: every drone holds its current position (panic button)
    ~/land    — descend all commanded drones, disarm on touchdown
    ~/reset_fence — clear a latched geofence breach
"""

import re
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Transform, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Float32, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from airstack_msgs.srv import RobotCommand

from svg_ground_control.cbf_filter import filter_velocities
from svg_ground_control.fence import (BEHAVIORS as FENCE_BEHAVIORS, clamp_to_box,
                                      keep_in_velocity, outside, violation_text)
from svg_ground_control.position_hold import advance_reference, tracking_velocity
from svg_ground_control.scenarios import Bounds, make_scenario
from svg_ground_control.trajectory import seek_velocity, stopping_distance


def heading_to_yaw(theta_deg: float) -> float:
    """Operator heading (degrees, 0 = +X, clockwise positive) -> ENU yaw (rad)."""
    return -np.radians(float(theta_deg))


def yaw_to_heading(yaw: float) -> float:
    """ENU yaw (rad) -> operator heading in degrees (0 = +X, clockwise)."""
    return float((-np.degrees(yaw) + 180.0) % 360.0 - 180.0)


class FlightState(Enum):
    IDLE = 0       # on the ground, not commanded
    ARMING = 1     # streaming zero setpoints, requesting offboard + arm
    ASCEND = 2     # climbing to the takeoff target
    ACTIVE = 3     # holding / following the scenario or teleop
    LANDING = 4    # descending; disarm at land_complete_altitude


# Seconds after entering ARMING at which each step fires.
ARMING_OFFBOARD_S = 1.0    # request offboard (REQUEST_CONTROL)
ARMING_ARM_S = 1.5         # arm
ARMING_DONE_S = 2.5        # transition to ASCEND


class DroneHandle:
    """Book-keeping for one drone."""

    def __init__(self, name: str, role: str):
        self.name = name
        self.role = role                  # 'auto' | 'teleop' | 'external'
        self.mode = 'sim'                 # 'sim' | 'real' (command routing)
        self.output = 'velocity'          # 'velocity' | 'trajectory' (see below)
        self.position_offset = np.zeros(3)  # local-frame -> world correction
        self.takeoff_target = None        # np (3,), set from the scenario
        self.hold_target = None           # np (3,), position to hold when not in mission
        self.state = FlightState.IDLE
        self.position = None              # np (3,) ENU, None until first odometry
        self.velocity = np.zeros(3)
        self.last_odom_time = None        # rclpy Time
        self.arming_start = None          # rclpy Time
        self.arming_steps_done = set()
        self.cmd_pub = None
        self.robot_command_client = None
        self.teleop_twist = np.zeros(3)
        self.teleop_yaw_rate = 0.0
        self.last_teleop_time = None
        # Reference point (world ENU): where the drone was told to be,
        # integrated from the published velocity (position_hold.advance_
        # reference). PX4's position setpoint on the trajectory output, the
        # point the go-to-goal profile is evaluated at, and the teleop hold
        # point. None = re-seed at the drone next tick (control hand-over).
        self.ref = None
        # Velocity the reference last moved with (what was published, or the
        # measured velocity after a seed/leash) — the profile re-attaches to it.
        self.applied = np.zeros(3)

    @property
    def commanded(self) -> bool:
        return self.role in ('auto', 'teleop')


class SwarmCommander(Node):

    def __init__(self, **kwargs):
        # **kwargs forwards e.g. parameter_overrides=[...] to rclpy.node.Node,
        # which lets tests construct the commander with specific params without
        # a launch file. main() and the launch file call it with no args.
        super().__init__('swarm_commander', **kwargs)

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('drone_names', ['drone_1', 'drone_2', 'drone_3'])
        # Comma-separated names of operator-driven drones (commands come from
        # teleop_command instead of the scenario). Empty string = every drone
        # follows the scenario. (A string, not a list: an empty YAML list has
        # no type and cannot override a string-array parameter default.)
        # NOTE: teleop drones are NOT automatically CBF-exempt — list them in
        # cbf_exempt_drones below if you want their commands left uncorrected.
        self.declare_parameter('teleop_drones', '')
        # Comma-separated names tracked for the safety filter but never
        # commanded (e.g. RC-flown).
        self.declare_parameter('external_drones', '')
        # Comma-separated names whose commands the CBF does NOT correct
        # (deliberate obstacles). Exempt drones are still seen by the filter,
        # so every other drone avoids them — the CBF simply restores their own
        # command after filtering. Independent of role: a policy-driven (auto)
        # drone or a teleop drone can be exempt. Empty = no drone exempt by
        # this list. (The 'squeeze' scenario additionally self-designates its
        # intruder via squeeze_intruder_cbf_exempt; the two are unioned.)
        self.declare_parameter('cbf_exempt_drones', '')

        # Scenario selection — see scenarios.py. NOTE: for 'squeeze' the
        # drone_names order matters: [holder, holder, intruder].
        self.declare_parameter('scenario', 'hover')
        # Nominal speed (m/s). Changeable in flight with
        #   ros2 param set /swarm_commander scenario_speed_mps 1.0
        # (applies to every scenario-driven drone; per-drone overrides via the
        # goal scenario's speed_command topic). Effective speed is also capped
        # by cbf_max_speed_mps and, near a goal, by the braking law (goal_accel_mps2, goal_settle_s).
        self.declare_parameter('scenario_speed_mps', 0.6)
        # Go-to-goal law (trajectory.py), all scenarios. The speed setting is
        # a cruise cap; the drone accelerates and brakes at goal_accel_mps2
        # and eases into the goal with time constant goal_settle_s, so it
        # only reaches the cap if the goal is farther than
        # v^2/(2 a) + v * settle. goal_lead_m: how far the reference point may
        # get ahead of the drone (tracking lag, wall, gust) before it is
        # pulled back and the profile restarts from the drone's own speed
        # (PX4: MPC_XY_ERR_MAX). Velocity-only drones (sim) use the
        # same law at their own position with the longer settle, which
        # absorbs PX4's ~0.7 s velocity-loop lag without a feedforward.
        self.declare_parameter('goal_accel_mps2', 3.0)
        self.declare_parameter('goal_settle_s', 0.3)
        self.declare_parameter('goal_lead_m', 2.0)
        self.declare_parameter('goal_velocity_only_settle_s', 1.0)
        self.declare_parameter('scenario_seed', 7)
        self.declare_parameter('arena_low', [-2.0, -2.0, 0.8])
        self.declare_parameter('arena_high', [2.0, 2.0, 2.0])
        # squeeze scenario geometry (ENU, meters), set both explicitly:
        # the two holder posts as flat [x1,y1,z1, x2,y2,z2] ...
        self.declare_parameter('squeeze_holder_positions',
                               [0.0, -0.69, 1.2, 0.0, 0.69, 1.2])
        # ... and the two waypoints the intruder shuttles between, flat
        # [ax,ay,az, bx,by,bz]; it starts at A and flies toward B first.
        self.declare_parameter('squeeze_intruder_waypoints',
                               [-1.5, 0.0, 1.2, 1.5, 0.0, 1.2])
        # The intruder is the deliberate obstacle: exempt from the CBF so it
        # presses through and the holders alone yield (filtering it makes
        # the filter push it backwards as it approaches the gap).
        self.declare_parameter('squeeze_intruder_cbf_exempt', True)
        # Used by the 'hover' scenario only: flat [x1,y1,z1, ...] per drone.
        self.declare_parameter('hover_positions',
                               [-1.5, 0.0, 1.2, 1.5, 0.0, 1.2, 0.0, -1.5, 1.2])

        # Per-drone position offset (flat [x1,y1,z1, ...]) ADDED to incoming
        # odometry to bring every drone into one shared world frame. Needed
        # in SIM: each PX4 SITL's local origin is its own spawn point, so
        # raw odometries live in different frames (set each drone's offset
        # to its spawn position). With mocap-anchored EKFs leave at zeros.
        self.declare_parameter('drone_position_offsets',
                               [0.0] * 9)

        self.declare_parameter('state_topic_template',
                               '/{name}/odometry_conversion/odometry')
        self.declare_parameter('velocity_command_topic_template',
                               '/{name}/interface/velocity_command')
        self.declare_parameter('robot_command_service_template',
                               '/{name}/interface/robot_command')
        self.declare_parameter('teleop_topic_template', '/svg/{name}/teleop_command')

        # ---- Hybrid sim/real routing ------------------------------------
        # Per-drone mode (comma-separated, one per drone): 'sim' routes
        # commands through the MAVROS/sim interface, 'real' through the
        # px4_interface/uXRCE-DDS hardware interface. Empty = every drone
        # uses the single velocity_command/robot_command templates above
        # (backward-compatible with the pure swarm_sim / swarm_real configs).
        self.declare_parameter('drone_modes', '')
        self.declare_parameter('default_drone_mode', 'sim')
        self.declare_parameter('sim_velocity_command_topic_template',
                               '/{name}/interface/velocity_command')
        self.declare_parameter('sim_robot_command_service_template',
                               '/{name}/interface/robot_command')
        self.declare_parameter('real_velocity_command_topic_template',
                               '/{name}/fmu/velocity_command')
        self.declare_parameter('real_robot_command_service_template',
                               '/{name}/fmu/robot_command')
        # What real drones are sent: 'trajectory' = position + velocity +
        # acceleration in one MultiDOFJointTrajectory (PX4 holds position and
        # tracks with feedforward, like its own Position mode — needs the
        # px4_interface with trajectory_command); 'velocity' = the old bare
        # TwistStamped velocity setpoint.
        self.declare_parameter('real_command_mode', 'trajectory')
        self.declare_parameter('real_trajectory_command_topic_template',
                               '/{name}/fmu/trajectory_command')

        # ---- Goal scenario live retargeting -----------------------------
        self.declare_parameter('goal_command_topic_template',
                               '/svg/{name}/goal_command')
        self.declare_parameter('speed_command_topic_template',
                               '/svg/{name}/speed_command')
        # [x, y, z, theta_deg]: one flat goal with heading (theta: 0 = +X,
        # clockwise positive). 3 values = heading +X.
        self.declare_parameter('goal_xyzt_topic_template', '/svg/{name}/goal_xyzt')

        # ---- Geofence (safety latch) ------------------------------------
        # If any airborne drone leaves [fence_min, fence_max] (world ENU, m),
        # latch a breach: every drone freezes at its current position, the
        # scenario stops, and start is blocked until ~/reset_fence.
        self.declare_parameter('fence_enabled', False)
        self.declare_parameter('fence_min', [-1000.0, -1000.0, -1000.0])
        self.declare_parameter('fence_max', [1000.0, 1000.0, 1000.0])
        # 'hold_all': breach -> everyone freezes (latch, ~/reset_fence).
        # 'keep_in' : nobody stops; commanded drones are held inside the box
        #             by a per-axis velocity barrier (see fence.py).
        self.declare_parameter('fence_behavior', 'hold_all')
        # keep_in only: outward speed allowed = gain * distance to the wall
        # (1/s); margin shrinks the box so the wall is met that early (m).
        self.declare_parameter('fence_keep_in_gain', 1.0)
        self.declare_parameter('fence_margin_m', 0.0)

        # ---- Visualization ----------------------------------------------
        self.declare_parameter('publish_viz', True)
        self.declare_parameter('viz_frame', 'map')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('teleop_timeout_s', 0.5)

        # Hold/ascend P-controller
        self.declare_parameter('hover_kp', 1.0)
        self.declare_parameter('arrival_threshold_m', 0.15)

        # Takeoff climb speed (a braking-law profile toward the takeoff
        # target, not a P-law: a 1 m target must not mean a 1 m/s step).
        self.declare_parameter('takeoff_speed_mps', 0.5)
        # Leash of the reference point while ASCENDING / LANDING / holding
        # outside a mission. PX4's altitude loop is stiff (MPC_Z_P = 5) and a
        # drone cannot follow anything during its takeoff thrust ramp
        # (~1.5 s), so the reference must stay close: bag C1_0920_203148 had
        # it 1.3 m above a drone still on the ground and the drone shot to
        # twice the hover height when the motors caught up.
        self.declare_parameter('hold_lead_m', 0.2)
        # Landing
        self.declare_parameter('land_speed_mps', 0.3)
        self.declare_parameter('land_complete_altitude_m', 0.15)

        # CBF safety filter
        self.declare_parameter('cbf_safety_radius_m', 0.55)
        self.declare_parameter('cbf_max_speed_mps', 1.2)
        self.declare_parameter('cbf_alpha', 2.5)
        self.declare_parameter('teleop_max_speed_mps', 1.2)
        # Position-mode teleop: leash of the teleop reference point (how far
        # it may run ahead of a drone that is held back by the CBF, fence or
        # a wall; 0 = no leash) and, for velocity-only drones, the
        # commander's P-gain on (reference - position). On the trajectory
        # output PX4 holds the reference itself (MPC_XY_P) and teleop_kp is
        # unused.
        self.declare_parameter('teleop_kp', 1.0)
        self.declare_parameter('teleop_lead_m', 0.5)
        # Gain on an EXTERNAL drone's measured velocity as seen by the CBF.
        # 1.0 = react to its true approach speed; > 1 pretends it is faster,
        # so commanded drones start yielding earlier and dodge harder.
        self.declare_parameter('cbf_external_velocity_gain', 1.0)

        def name_list(param: str) -> list:
            raw = str(self.get_parameter(param).value)
            return [n.strip() for n in raw.split(',') if n.strip()]

        names = list(self.get_parameter('drone_names').value)
        teleop_names = name_list('teleop_drones')
        external_names = name_list('external_drones')
        exempt_names = name_list('cbf_exempt_drones')
        for n in teleop_names + external_names + exempt_names:
            if n not in names:
                raise ValueError(f'"{n}" not in drone_names')
        # Drones whose commands the CBF leaves uncorrected (still obstacles for
        # everyone else). External drones are never commanded, so they cannot
        # be "exempt" in the command sense — reject that to catch config typos.
        for n in exempt_names:
            if n in external_names:
                raise ValueError(
                    f'"{n}" is in both external_drones and cbf_exempt_drones; '
                    'external drones are never commanded')
        self.cbf_exempt_names = set(exempt_names)

        offsets_flat = list(self.get_parameter('drone_position_offsets').value)
        if len(offsets_flat) != 3 * len(names):
            raise ValueError(
                f'drone_position_offsets needs {3 * len(names)} values '
                f'(3 per drone), got {len(offsets_flat)}')
        position_offsets = np.array(offsets_flat).reshape(-1, 3)

        # Per-drone sim/real modes.
        modes_list = name_list('drone_modes')
        if modes_list and len(modes_list) != len(names):
            raise ValueError(
                f'drone_modes has {len(modes_list)} entries for '
                f'{len(names)} drones')
        default_mode = str(self.get_parameter('default_drone_mode').value)
        drone_modes = modes_list if modes_list else [default_mode] * len(names)
        for m in drone_modes:
            if m not in ('sim', 'real'):
                raise ValueError(f'drone mode must be sim|real, got "{m}"')
        self._use_mode_templates = bool(modes_list)

        # Geofence.
        self.fence_enabled = bool(self.get_parameter('fence_enabled').value)
        self.fence_min = np.array(self.get_parameter('fence_min').value, dtype=float)
        self.fence_max = np.array(self.get_parameter('fence_max').value, dtype=float)
        self.fence_behavior = str(self.get_parameter('fence_behavior').value).strip()
        if self.fence_behavior not in FENCE_BEHAVIORS:
            raise ValueError(
                f"fence_behavior '{self.fence_behavior}' unknown; "
                f"use one of {', '.join(FENCE_BEHAVIORS)}")
        self.fence_keep_in_gain = float(self.get_parameter('fence_keep_in_gain').value)
        self.fence_margin = float(self.get_parameter('fence_margin_m').value)
        self.fence_breached = False

        self.state_timeout = float(self.get_parameter('state_timeout_s').value)
        self.teleop_timeout = float(self.get_parameter('teleop_timeout_s').value)
        self.hover_kp = float(self.get_parameter('hover_kp').value)
        self.arrival_threshold = float(self.get_parameter('arrival_threshold_m').value)
        self.land_speed = float(self.get_parameter('land_speed_mps').value)
        self.takeoff_speed = float(self.get_parameter('takeoff_speed_mps').value)
        self.hold_lead = float(self.get_parameter('hold_lead_m').value)
        self.land_complete_alt = float(
            self.get_parameter('land_complete_altitude_m').value)
        self.cbf_safety_radius = float(self.get_parameter('cbf_safety_radius_m').value)
        self.cbf_max_speed = float(self.get_parameter('cbf_max_speed_mps').value)
        self.cbf_alpha = float(self.get_parameter('cbf_alpha').value)
        self.teleop_max_speed = float(self.get_parameter('teleop_max_speed_mps').value)
        self.teleop_kp = float(self.get_parameter('teleop_kp').value)
        self.teleop_lead = float(self.get_parameter('teleop_lead_m').value)
        self.goal_lead = float(self.get_parameter('goal_lead_m').value)
        real_command_mode = str(self.get_parameter('real_command_mode').value).strip()
        if real_command_mode not in ('trajectory', 'velocity'):
            raise ValueError(
                f"real_command_mode '{real_command_mode}' unknown; "
                "use trajectory|velocity")
        self.cbf_external_velocity_gain = float(
            self.get_parameter('cbf_external_velocity_gain').value)

        # ---- Scenario -----------------------------------------------------
        scenario_name = str(self.get_parameter('scenario').value)
        scenario_kwargs = {
            'accel': float(self.get_parameter('goal_accel_mps2').value),
            'settle_s': float(self.get_parameter('goal_settle_s').value),
            'velocity_only_settle_s': float(
                self.get_parameter('goal_velocity_only_settle_s').value),
        }
        if scenario_name == 'hover':
            scenario_kwargs['hover_positions'] = np.array(
                self.get_parameter('hover_positions').value)
        elif scenario_name == 'goal':
            # Goals start at the takeoff layout; retargeted live via topics.
            scenario_kwargs['initial_goals'] = np.array(
                self.get_parameter('hover_positions').value)
        elif scenario_name == 'squeeze':
            scenario_kwargs['holder_positions'] = np.array(
                self.get_parameter('squeeze_holder_positions').value)
            scenario_kwargs['intruder_waypoints'] = np.array(
                self.get_parameter('squeeze_intruder_waypoints').value)
            scenario_kwargs['intruder_cbf_exempt'] = bool(
                self.get_parameter('squeeze_intruder_cbf_exempt').value)
        self.scenario = make_scenario(
            scenario_name,
            num_drones=len(names),
            nominal_speed=float(self.get_parameter('scenario_speed_mps').value),
            bounds=Bounds(
                low=np.array(self.get_parameter('arena_low').value),
                high=np.array(self.get_parameter('arena_high').value)),
            safety_radius=self.cbf_safety_radius,
            seed=int(self.get_parameter('scenario_seed').value),
            **scenario_kwargs)
        self.scenario_name = scenario_name
        self.mission_active = False
        if scenario_name == 'squeeze':
            posts = self.scenario.holder_posts
            gap = float(np.linalg.norm(posts[0] - posts[1]))
            self.get_logger().info(
                f'squeeze geometry: posts {gap:.2f} m apart '
                f'(2r keep-out = {2 * self.cbf_safety_radius:.2f} m), '
                f'intruder A={self.scenario.intruder_waypoints[0]} '
                f'B={self.scenario.intruder_waypoints[1]}')

        state_tmpl = str(self.get_parameter('state_topic_template').value)
        default_cmd_tmpl = str(
            self.get_parameter('velocity_command_topic_template').value)
        default_srv_tmpl = str(
            self.get_parameter('robot_command_service_template').value)
        sim_cmd_tmpl = str(
            self.get_parameter('sim_velocity_command_topic_template').value)
        sim_srv_tmpl = str(
            self.get_parameter('sim_robot_command_service_template').value)
        real_cmd_tmpl = str(
            self.get_parameter('real_velocity_command_topic_template').value)
        real_srv_tmpl = str(
            self.get_parameter('real_robot_command_service_template').value)
        real_traj_tmpl = str(
            self.get_parameter('real_trajectory_command_topic_template').value)
        teleop_tmpl = str(self.get_parameter('teleop_topic_template').value)
        goal_tmpl = str(self.get_parameter('goal_command_topic_template').value)
        speed_tmpl = str(self.get_parameter('speed_command_topic_template').value)
        xyzt_tmpl = str(self.get_parameter('goal_xyzt_topic_template').value)

        def command_templates(mode):
            """(output kind, command topic, robot_command service) for a mode."""
            if not self._use_mode_templates:
                return 'velocity', default_cmd_tmpl, default_srv_tmpl
            if mode == 'real':
                if real_command_mode == 'trajectory':
                    return 'trajectory', real_traj_tmpl, real_srv_tmpl
                return 'velocity', real_cmd_tmpl, real_srv_tmpl
            return 'velocity', sim_cmd_tmpl, sim_srv_tmpl

        # ---- Per-drone wiring --------------------------------------------
        takeoff_targets = self.scenario.initial_positions()
        self.drones = []
        for i, name in enumerate(names):
            role = ('teleop' if name in teleop_names
                    else 'external' if name in external_names else 'auto')
            drone = DroneHandle(name, role)
            drone.mode = drone_modes[i]
            drone.position_offset = position_offsets[i].copy()
            drone.takeoff_target = takeoff_targets[i].copy()
            drone.hold_target = takeoff_targets[i].copy()
            if drone.commanded:
                drone.output, cmd_t, srv_t = command_templates(drone.mode)
                drone.cmd_pub = self.create_publisher(
                    MultiDOFJointTrajectory if drone.output == 'trajectory'
                    else TwistStamped,
                    cmd_t.format(name=name), 10)
                drone.robot_command_client = self.create_client(
                    RobotCommand, srv_t.format(name=name))
            if role == 'teleop':
                self.create_subscription(
                    TwistStamped, teleop_tmpl.format(name=name),
                    lambda msg, d=drone: self.teleop_callback(d, msg), 10)
            if scenario_name == 'goal' and drone.commanded:
                self.create_subscription(
                    PoseStamped, goal_tmpl.format(name=name),
                    lambda msg, idx=i: self.goal_callback(idx, msg), 10)
                self.create_subscription(
                    Float32, speed_tmpl.format(name=name),
                    lambda msg, idx=i: self.speed_callback(idx, msg), 10)
                self.create_subscription(
                    Float64MultiArray, xyzt_tmpl.format(name=name),
                    lambda msg, idx=i: self.goal_xyzt_callback(idx, msg), 10)
            self.create_subscription(
                Odometry, state_tmpl.format(name=name),
                lambda msg, d=drone: self.odometry_callback(d, msg), 10)
            self.drones.append(drone)

        # ---- Formation profiles (single-command swarm re-targeting) --------
        # 'formation_profiles' lists profile names; each name needs a matching
        # 'formation_<name>' flat array [x1,y1,z1, x2,y2,z2, ...] (one world
        # ENU row per drone, in drone_names order). Publishing a profile name
        # on the formation topic retargets every scenario-driven drone's goal
        # in one shot ('goal' scenario only):
        #   ros2 topic pub --once /svg/formation_command std_msgs/msg/String \
        #     "{data: line}"
        self.declare_parameter('formation_profiles', '')
        self.declare_parameter('formation_command_topic', '/svg/formation_command')
        self.formations = {}
        for pname in name_list('formation_profiles'):
            if pname == 'next':
                raise ValueError(
                    '"next" is reserved (it advances the profile cycle) and '
                    'cannot be a formation profile name')
            if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', pname):
                raise ValueError(
                    f'formation profile "{pname}" is not a valid name '
                    '(letters/digits/underscore, starting with a letter)')
            self.declare_parameter(f'formation_{pname}', [0.0])
            flat = list(self.get_parameter(f'formation_{pname}').value)
            if len(flat) != 3 * len(names):
                raise ValueError(
                    f'formation_{pname} needs {3 * len(names)} values '
                    f'(x,y,z per drone in drone_names order), got {len(flat)}')
            self.formations[pname] = np.array(flat, dtype=float).reshape(-1, 3)
        # "next" cycling state: profiles roll in formation_profiles order;
        # -1 = nothing applied yet, so the first "next" lands on profile 0.
        # Naming a profile explicitly re-anchors the cycle at that profile.
        self._formation_order = list(self.formations)
        self._formation_index = -1
        if self.formations:
            if hasattr(self.scenario, 'set_goal'):
                self.create_subscription(
                    String,
                    str(self.get_parameter('formation_command_topic').value),
                    self.formation_callback, 10)
                self.get_logger().info(
                    'formation profiles: ' + ', '.join(sorted(self.formations)))
            else:
                self.get_logger().warn(
                    f'formation_profiles set but scenario "{scenario_name}" '
                    'has no retargetable goals; profiles ignored')

        # ---- Operator services ---------------------------------------------
        self.create_service(Trigger, '~/takeoff', self.handle_takeoff)
        self.create_service(Trigger, '~/start', self.handle_start)
        self.create_service(Trigger, '~/hold', self.handle_hold)
        self.create_service(Trigger, '~/land', self.handle_land)
        self.create_service(Trigger, '~/reset_fence', self.handle_reset_fence)

        # ---- Visualization -------------------------------------------------
        self.publish_viz = bool(self.get_parameter('publish_viz').value)
        self.viz_frame = str(self.get_parameter('viz_frame').value)
        self.viz_pub = (self.create_publisher(MarkerArray, '/svg/viz/markers', 10)
                        if self.publish_viz else None)

        # ---- CBF activity (consumed by led_controller -> onboard LEDs red) ---
        # Comma-separated names of commanded drones whose command the CBF is
        # altering THIS tick (empty string = none). Published every control tick;
        # any hold/latch is the consumer's job.
        self.cbf_active_pub = self.create_publisher(String, '/svg/cbf_active', 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.control_dt = 1.0 / rate
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        # `ros2 param set` used to answer "successful" and change nothing:
        # every value above is read once at construction. The speed-related
        # ones are now applied live; anything else is refused with a reason
        # instead of being silently ignored.
        self.add_on_set_parameters_callback(self.on_parameter_change)
        self._cbf_warn_count = 0

        self.get_logger().info(
            f'SwarmCommander up | scenario={scenario_name} | '
            + ', '.join(
                f'{d.name}({d.role}/{d.mode}'
                + ('/cbf-exempt' if d.name in self.cbf_exempt_names else '')
                + ')'
                for d in self.drones)
            + f' | speed={self.scenario.nominal_speed} m/s'
            + f' (accel {self.scenario.tracker.accel} m/s2, settle '
            + f'{self.scenario.tracker.settle} s, lead {self.goal_lead} m)'
            + ' | real output: ' + ', '.join(sorted({
                f'{d.output}' for d in self.drones if d.commanded and d.mode == 'real'
            }) or ['none'])
            + f' | CBF r={self.cbf_safety_radius} m, vmax={self.cbf_max_speed} m/s,'
            + f' alpha={self.cbf_alpha}, ext_vel_gain={self.cbf_external_velocity_gain}'
            + (f' | FENCE {self.fence_behavior} {self.fence_min}..{self.fence_max}'
               if self.fence_enabled else ' | fence OFF'))
        if np.any(position_offsets):
            self.get_logger().info(
                'position offsets (local->world): '
                + ', '.join(f'{d.name}: {d.position_offset}' for d in self.drones))
        else:
            self.get_logger().warn(
                'drone_position_offsets are all zero — correct for mocap, but '
                'in SIM each PX4 local origin is its spawn point; set the '
                'offsets to the spawn positions or all geometry is per-drone!')

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def odometry_callback(self, drone: DroneHandle, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        # position_offset shifts each drone's local-origin odometry into the
        # shared world frame (velocities are origin-independent).
        drone.position = np.array([p.x, p.y, p.z]) + drone.position_offset
        drone.velocity = np.array([v.x, v.y, v.z])
        drone.last_odom_time = self.get_clock().now()

    def teleop_callback(self, drone: DroneHandle, msg: TwistStamped):
        l = msg.twist.linear
        drone.teleop_twist = np.array([l.x, l.y, l.z])
        # Yaw bypasses the CBF: the filter constrains positions, and turning
        # in place cannot change separation.
        drone.teleop_yaw_rate = float(msg.twist.angular.z)
        drone.last_teleop_time = self.get_clock().now()

    def goal_callback(self, index: int, msg: PoseStamped):
        # World-frame goal for the 'goal' scenario; ignored otherwise. The
        # orientation's yaw is the heading; an all-zero (unset) quaternion,
        # which is what a position-only `topic pub` sends, means nose on +X.
        q = msg.pose.orientation
        yaw = 0.0
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5:
            yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        p = msg.pose.position
        self.retarget(index, np.array([p.x, p.y, p.z]), yaw)

    def goal_xyzt_callback(self, index: int, msg: Float64MultiArray):
        """[x, y, z, theta_deg]: theta 0 = +X, clockwise positive; 3 values = +X."""
        data = list(msg.data)
        if len(data) not in (3, 4):
            self.get_logger().warn(
                f'{self.drones[index].name}: goal_xyzt needs [x, y, z] or '
                f'[x, y, z, theta_deg], got {len(data)} values')
            return
        theta = data[3] if len(data) == 4 else 0.0
        self.retarget(index, np.array(data[:3], dtype=float),
                      heading_to_yaw(theta))

    def retarget(self, index: int, point: np.ndarray, yaw: float):
        if not hasattr(self.scenario, 'set_goal'):
            return
        self.scenario.set_goal(index, point, yaw)
        self.get_logger().info(
            f'{self.drones[index].name}: goal -> [{point[0]:.2f}, {point[1]:.2f}, '
            f'{point[2]:.2f}] heading {yaw_to_heading(yaw):.0f} deg '
            '(0 = +X, clockwise)')

    def speed_callback(self, index: int, msg: Float32):
        if hasattr(self.scenario, 'set_speed'):
            self.scenario.set_speed(index, msg.data)
            self.get_logger().info(
                f'{self.drones[index].name}: speed -> {msg.data:.2f} m/s'
                + self.speed_cap_note(msg.data))

    def speed_cap_note(self, speed: float) -> str:
        """Why a requested speed will not be flown as asked, if it will not."""
        notes = []
        if speed > self.cbf_max_speed:
            notes.append(f'capped to cbf_max_speed_mps={self.cbf_max_speed}')
        tracker = getattr(self.scenario, 'tracker', None)
        if tracker is not None:
            reach = stopping_distance(min(speed, self.cbf_max_speed),
                                      tracker.accel, tracker.settle)
            notes.append(f'reached only with a goal > {reach:.2f} m away '
                         f'(goal_accel_mps2={tracker.accel}, '
                         f'goal_settle_s={tracker.settle})')
        return f' ({"; ".join(notes)})' if notes else ''

    def on_parameter_change(self, params):
        """Live parameter updates (`ros2 param set /swarm_commander ...`)."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'scenario_speed_mps':
                speed = float(p.value)
                if hasattr(self.scenario, 'set_all_speeds'):
                    self.scenario.set_all_speeds(speed)
                else:
                    self.scenario.nominal_speed = max(0.0, speed)
                self.get_logger().info(
                    f'scenario_speed_mps -> {speed:.2f} m/s (live, all scenario '
                    f'drones){self.speed_cap_note(speed)}')
            elif p.name == 'cbf_max_speed_mps':
                self.cbf_max_speed = float(p.value)
                self.get_logger().info(f'cbf_max_speed_mps -> {self.cbf_max_speed} (live)')
            elif p.name == 'teleop_max_speed_mps':
                self.teleop_max_speed = float(p.value)
                self.get_logger().info(f'teleop_max_speed_mps -> {self.teleop_max_speed} (live)')
            elif p.name == 'goal_accel_mps2':
                self.scenario.tracker.accel = max(0.1, float(p.value))
                self.get_logger().info(f'goal_accel_mps2 -> {p.value} (live)')
            elif p.name == 'goal_settle_s':
                self.scenario.tracker.settle = max(0.0, float(p.value))
                self.get_logger().info(f'goal_settle_s -> {p.value} (live)')
            elif p.name == 'goal_velocity_only_settle_s':
                self.scenario.velocity_only_settle = max(0.0, float(p.value))
                self.get_logger().info(
                    f'goal_velocity_only_settle_s -> {p.value} (live)')
            elif p.name == 'goal_lead_m':
                self.goal_lead = float(p.value)
                self.get_logger().info(f'goal_lead_m -> {p.value} (live)')
            elif p.name in ('teleop_kp', 'teleop_lead_m', 'hover_kp',
                            'hold_lead_m', 'takeoff_speed_mps'):
                setattr(self, {'teleop_kp': 'teleop_kp', 'teleop_lead_m': 'teleop_lead',
                               'hover_kp': 'hover_kp', 'hold_lead_m': 'hold_lead',
                               'takeoff_speed_mps': 'takeoff_speed'}[p.name],
                        float(p.value))
                self.get_logger().info(f'{p.name} -> {p.value} (live)')
            else:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} is read once at startup; change the YAML '
                           'and relaunch (live: scenario_speed_mps, '
                           'cbf_max_speed_mps, teleop_max_speed_mps, '
                           'goal_accel_mps2, goal_settle_s, goal_lead_m, '
                           'goal_velocity_only_settle_s, teleop_kp, '
                           'teleop_lead_m, hover_kp, hold_lead_m, '
                           'takeoff_speed_mps)')
        return SetParametersResult(successful=True)

    def formation_callback(self, msg: String):
        """Retarget every scenario-driven drone to a named formation profile.

        The reserved name "next" advances through the profiles in their
        formation_profiles order (wrapping around); an explicit profile name
        re-anchors the cycle there, so "next" continues from it.
        """
        name = msg.data.strip()
        if name == 'next':
            self._formation_index = (
                (self._formation_index + 1) % len(self._formation_order))
            name = self._formation_order[self._formation_index]
        elif name in self.formations:
            self._formation_index = self._formation_order.index(name)
        profile = self.formations.get(name)
        if profile is None:
            self.get_logger().warn(
                f'unknown formation "{name}" (available: next, '
                + ', '.join(sorted(self.formations)) + ')')
            return
        moved = []
        for i, d in enumerate(self.drones):
            if not d.commanded:
                continue      # external drones have no goal to retarget
            self.scenario.set_goal(i, profile[i])
            moved.append(f'{d.name} -> {profile[i].tolist()}')
        self.get_logger().info(f'formation "{name}": ' + '; '.join(moved))

    # ------------------------------------------------------------------
    # Operator services
    # ------------------------------------------------------------------

    def handle_takeoff(self, request, response):
        now = self.get_clock().now()
        started = []
        for d in self.drones:
            if not d.commanded or d.state != FlightState.IDLE:
                continue
            if d.position is None:
                self.get_logger().warn(
                    f'{d.name}: no odometry yet, refusing takeoff')
                continue
            d.state = FlightState.ARMING
            d.arming_start = now
            d.arming_steps_done = set()
            d.hold_target = d.takeoff_target.copy()
            started.append(d.name)
        response.success = bool(started)
        response.message = ('takeoff: ' + ', '.join(started)) if started \
            else 'no drone eligible for takeoff (missing odometry or not IDLE)'
        return response

    def handle_start(self, request, response):
        if self.fence_breached:
            response.success = False
            response.message = 'geofence breached — call ~/reset_fence first'
            return response
        not_ready = [d.name for d in self.drones
                     if d.commanded and d.state != FlightState.ACTIVE]
        if not_ready:
            response.success = False
            response.message = 'not all drones holding yet: ' + ', '.join(not_ready)
            return response
        self.mission_active = True
        self.scenario.reset_tracking()
        for d in self.drones:
            d.ref = None                # seed from where the drone IS, now
        response.success = True
        response.message = f'scenario "{self.scenario_name}" running'
        self.get_logger().info(response.message)
        return response

    def handle_hold(self, request, response):
        self.mission_active = False
        held = []
        for d in self.drones:
            if d.commanded and d.position is not None \
                    and d.state in (FlightState.ASCEND, FlightState.ACTIVE):
                d.hold_target = d.position.copy()
                d.ref = None
                d.state = FlightState.ACTIVE
                held.append(d.name)
        response.success = bool(held)
        response.message = 'holding: ' + ', '.join(held) if held else 'nothing to hold'
        return response

    def handle_land(self, request, response):
        self.mission_active = False
        landing = []
        for d in self.drones:
            if d.commanded and d.state in (FlightState.ASCEND, FlightState.ACTIVE):
                d.state = FlightState.LANDING
                d.ref = None
                landing.append(d.name)
        response.success = bool(landing)
        response.message = ('landing: ' + ', '.join(landing)) if landing \
            else 'no airborne drone to land'
        return response

    def handle_reset_fence(self, request, response):
        still_out = [d.name for d in self.drones if d.position is not None
                     and (np.any(d.position < self.fence_min)
                          or np.any(d.position > self.fence_max))]
        self.fence_breached = False
        response.success = True
        response.message = 'geofence latch cleared' + (
            f' (WARNING still outside: {", ".join(still_out)})' if still_out else '')
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Geofence
    # ------------------------------------------------------------------

    def enforce_fence(self, now):
        """Latch a breach if any airborne drone is outside the fence box.

        On breach: stop the scenario and freeze every airborne commanded
        drone at its current position (the control loop then holds it). The
        latch persists until ~/reset_fence.
        """
        if not self.fence_enabled or self.fence_breached:
            return
        if self.fence_behavior == 'keep_in':
            self.report_keep_in(now)
            return
        airborne = (FlightState.ASCEND, FlightState.ACTIVE, FlightState.LANDING)
        for d in self.drones:
            # Police every role: commanded drones once they have finished
            # taking off (ACTIVE; ASCEND climbs through the fence floor and
            # LANDING descends through it on purpose), and external drones
            # whenever they are in the air (they have no state machine).
            if not self.policed(d, now):
                continue
            below = d.position < self.fence_min
            above = d.position > self.fence_max
            if not (below.any() or above.any()):
                continue
            self.fence_breached = True
            self.mission_active = False
            viol = violation_text(d.position, self.fence_min, self.fence_max)
            for o in self.drones:
                if o.commanded and o.position is not None and o.state in airborne:
                    o.hold_target = o.position.copy()
                    o.ref = None
                    o.state = FlightState.ACTIVE
            self.get_logger().error(
                f'GEOFENCE BREACH by {d.name} ({d.role}) at '
                f'[{d.position[0]:.2f}, {d.position[1]:.2f}, {d.position[2]:.2f}] '
                f'({viol}) — ALL DRONES HOLD. Call ~/reset_fence to clear.')
            return

    def policed(self, d: DroneHandle, now) -> bool:
        """Whether the fence watches this drone right now (either behaviour).

        Commanded drones: only while ACTIVE. External (RC-flown) drones have
        no flight state, so they count while airborne — fresh odometry and
        higher than the landing-complete altitude — and are ignored on the
        ground, where being outside the box means nothing.
        """
        if d.position is None:
            return False
        if d.role != 'external':
            return d.state == FlightState.ACTIVE
        fresh = (d.last_odom_time is not None
                 and (now - d.last_odom_time) < Duration(seconds=self.state_timeout))
        return fresh and d.position[2] > self.land_complete_alt

    def report_keep_in(self, now):
        """keep_in mode: nothing to latch, but say who is out and unsteerable."""
        for d in self.drones:
            if not self.policed(d, now):
                continue
            if not outside(d.position, self.fence_min, self.fence_max).any():
                continue
            if d.role == 'external':
                self.get_logger().warn(
                    f'{d.name} (external, not commanded) is outside the fence '
                    f'({violation_text(d.position, self.fence_min, self.fence_max)}) '
                    '— keep_in cannot steer it',
                    throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(
                    f'{d.name} outside the fence '
                    f'({violation_text(d.position, self.fence_min, self.fence_max)}), '
                    'keep_in pushing it back',
                    throttle_duration_sec=2.0)

    def keep_in(self, drone: DroneHandle, velocity: np.ndarray) -> np.ndarray:
        """Clip a commanded velocity at the fence walls (keep_in behaviour)."""
        clipped = keep_in_velocity(velocity, drone.position, self.fence_min,
                                   self.fence_max, self.fence_keep_in_gain,
                                   self.fence_margin)
        if np.linalg.norm(clipped - velocity) > 0.05:
            self.get_logger().info(
                f'fence keep-in: {drone.name} limited on '
                + ''.join('xyz'[k] for k in range(3)
                          if abs(clipped[k] - velocity[k]) > 1e-6),
                throttle_duration_sec=1.0)
        return clipped

    # ------------------------------------------------------------------
    # Robot interface helpers
    # ------------------------------------------------------------------

    def send_robot_command(self, drone: DroneHandle, command: int, label: str):
        client = drone.robot_command_client
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{drone.name}: robot_command service not ready, skipping {label}')
            return
        req = RobotCommand.Request()
        req.command = command
        future = client.call_async(req)

        def report(fut, name=drone.name, label=label):
            try:
                ok = fut.result().success
            except Exception as e:  # noqa: BLE001 - log any service failure
                self.get_logger().error(f'{name}: {label} failed: {e}')
                return
            level = self.get_logger().info if ok else self.get_logger().error
            level(f'{name}: {label} -> success={ok}')

        future.add_done_callback(report)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def teleop_command(self, drone: DroneHandle, now) -> np.ndarray:
        """The operator's stick velocity (zero when the teleop topic is stale)."""
        stale = (drone.last_teleop_time is None
                 or (now - drone.last_teleop_time)
                 > Duration(seconds=self.teleop_timeout))
        cmd = np.zeros(3) if stale else drone.teleop_twist.copy()
        speed = np.linalg.norm(cmd)
        if speed > self.teleop_max_speed:
            cmd *= self.teleop_max_speed / speed
        return cmd

    def teleop_position_mode(self, drone: DroneHandle, now) -> np.ndarray:
        """Position-mode teleop (see position_hold.py).

        The sticks are a velocity that moves ``drone.ref`` (advanced from
        what is published, leashed to ``teleop_lead_m``, clamped into a
        keep_in fence — all in ``advance_reference``). On the trajectory
        output PX4 holds the reference itself, so the sticks are pure
        feedforward; on the velocity output the commander adds the P term.
        """
        stick = self.teleop_command(drone, now)
        if drone.output == 'trajectory' or drone.ref is None:
            return stick
        return tracking_velocity(drone.ref, drone.position, stick,
                                 self.teleop_kp, self.teleop_max_speed)

    def advance_reference(self, drone: DroneHandle):
        """Step a commanded drone's reference point by what it was told to fly."""
        seeded = drone.ref is None
        if drone.state == FlightState.ACTIVE and self.mission_active:
            lead = self.teleop_lead if drone.role == 'teleop' else self.goal_lead
        else:
            lead = self.hold_lead     # takeoff, landing, holding: stay close
        drone.ref, drone.applied = advance_reference(
            drone.ref, drone.position, drone.applied, drone.velocity,
            self.control_dt, lead)
        if self.fence_enabled and self.fence_behavior == 'keep_in':
            drone.ref = clamp_to_box(drone.ref, self.fence_min, self.fence_max,
                                     self.fence_margin)
        if seeded and drone.role == 'teleop' and self.mission_active:
            self.get_logger().info(
                f'{drone.name}: sticks live, holding '
                f'[{drone.position[0]:.2f}, {drone.position[1]:.2f}, '
                f'{drone.position[2]:.2f}] until moved')

    def control_loop(self):
        now = self.get_clock().now()

        # Advance ARMING state machines (time-staged, while zeros stream below).
        for d in self.drones:
            if d.state != FlightState.ARMING:
                continue
            elapsed = (now - d.arming_start).nanoseconds * 1e-9
            if elapsed >= ARMING_OFFBOARD_S and 'offboard' not in d.arming_steps_done:
                d.arming_steps_done.add('offboard')
                self.send_robot_command(d, RobotCommand.Request.REQUEST_CONTROL,
                                        'request offboard')
            if elapsed >= ARMING_ARM_S and 'arm' not in d.arming_steps_done:
                d.arming_steps_done.add('arm')
                self.send_robot_command(d, RobotCommand.Request.ARM, 'arm')
            if elapsed >= ARMING_DONE_S:
                d.state = FlightState.ASCEND
                self.get_logger().info(f'{d.name}: ascending to {d.hold_target}')

        # Geofence: may latch a breach and freeze everyone before commanding.
        self.enforce_fence(now)

        # Swarm state: every drone with a known position (any role) feeds the
        # CBF; freshness only gates whether a drone gets commands published.
        tracked = [d for d in self.drones if d.position is not None]
        if not tracked:
            return
        index = {d.name: i for i, d in enumerate(tracked)}
        positions = np.stack([d.position for d in tracked])

        # Reference points: where each commanded drone was told to be. PX4's
        # position setpoint on the trajectory output, and the point the
        # go-to-goal profile is evaluated at.
        for d in self.drones:
            if d.commanded and d.state != FlightState.IDLE and d.position is not None:
                self.advance_reference(d)

        # Scenario nominal velocities — only meaningful (and stateful: goal
        # resampling, wall bounces, reference profiles) once the mission runs
        # and all drones are tracked, so it is stepped exactly then.
        scenario_nominal = None
        scenario_accel = None
        if self.mission_active and len(tracked) == len(self.drones):
            all_positions = np.stack([d.position for d in self.drones])
            references = np.full((len(self.drones), 3), np.nan)
            applied = np.zeros((len(self.drones), 3))
            for k, d in enumerate(self.drones):
                if d.output == 'trajectory' and d.ref is not None:
                    references[k] = d.ref
                applied[k] = d.applied
            scenario_nominal = self.scenario.nominal_velocity(
                all_positions, references=references, applied=applied,
                dt=self.control_dt)
            scenario_accel = self.scenario.nominal_acceleration

        scenario_exempt = (set(self.scenario.cbf_exempt_indices)
                           if self.mission_active else set())

        nominal = np.zeros((len(tracked), 3))
        accel_ff = np.zeros((len(tracked), 3))   # feedforward, trajectory rows
        exempt_rows = set()   # commanded obstacle rows: published uncorrected
        # Rows the solver must NOT adjust (their nominal is what that drone
        # will fly regardless): external drones + exempt obstacles. Without
        # this the solver assigns them half of every pairwise correction --
        # evasion that is never executed.
        fixed_rows = np.zeros(len(tracked), dtype=bool)
        for d in tracked:
            i = index[d.name]
            if d.role == 'external':
                # RC-flown obstacle: feed its MEASURED velocity (PX4 EKF2
                # estimate, ENU world via px4_interface) so the constraint
                # sees the true approach speed and the holders yield BEFORE
                # it reaches the barrier, not after. Stale odometry falls
                # back to zero velocity (= previous, position-only behavior).
                fixed_rows[i] = True
                fresh = (d.last_odom_time is not None
                         and (now - d.last_odom_time)
                         < Duration(seconds=self.state_timeout))
                if fresh:
                    nominal[i] = self.cbf_external_velocity_gain * d.velocity
                continue
            if d.state in (FlightState.IDLE, FlightState.ARMING):
                nominal[i] = 0.0
            elif d.state == FlightState.LANDING:
                nominal[i] = np.array([0.0, 0.0, -self.land_speed])
            elif d.state == FlightState.ASCEND:
                error = d.hold_target - d.position
                nominal[i] = seek_velocity(
                    d.position[None], d.hold_target[None], self.takeoff_speed,
                    self.scenario.tracker.accel,
                    self.scenario.velocity_only_settle)[0]
                if np.linalg.norm(error) < self.arrival_threshold:
                    d.state = FlightState.ACTIVE
                    self.get_logger().info(f'{d.name}: holding takeoff position')
            elif d.state == FlightState.ACTIVE:
                if d.role == 'teleop' and self.mission_active:
                    nominal[i] = self.teleop_position_mode(d, now)
                elif self.mission_active and scenario_nominal is not None:
                    drone_index = self.drones.index(d)
                    nominal[i] = scenario_nominal[drone_index]
                    accel_ff[i] = scenario_accel[drone_index]
                    if drone_index in scenario_exempt:
                        exempt_rows.add(i)
                else:
                    nominal[i] = self.hover_kp * (d.hold_target - d.position)
                # CBF-exempt list (config): leave this drone's command
                # uncorrected while it flies. Union with the scenario's own
                # exempt indices. Climb-out / landing stay collision-protected
                # (only ACTIVE drones are eligible). Independent of role, so a
                # policy-driven obstacle (e.g. the squeeze intruder) or an
                # unprotected teleop drone can both be exempt.
                if d.name in self.cbf_exempt_names:
                    exempt_rows.add(i)

        # ================= CBF SAFETY FILTER =================
        # Real velocity-CBF (ported from drone_soccer). Exempt rows are the
        # deliberate moving obstacles (teleop drones + scenario-designated
        # ones like the squeeze intruder): they are speed-capped up front —
        # the cap is what will actually be published — and pinned as FIXED,
        # so the solver leaves them uncorrected and the other drones absorb
        # the full evasion. External drones are likewise fixed at their
        # measured velocity (set above). Filtering an obstacle would push it
        # back from the conflict instead of letting it force the others to
        # yield — worse, the pushed-back command would never be executed.
        for i in exempt_rows:
            speed = np.linalg.norm(nominal[i])
            if speed > self.cbf_max_speed:
                nominal[i] *= self.cbf_max_speed / speed
            fixed_rows[i] = True
        result = filter_velocities(
            nominal, positions,
            safety_radius=self.cbf_safety_radius,
            max_speed=self.cbf_max_speed,
            alpha=self.cbf_alpha,
            fixed=fixed_rows,
        )
        safe = result.velocities
        if result.used_emergency_stop:
            self.get_logger().warn(
                'CBF emergency push-apart engaged '
                f'(infeasible pairs: {result.num_infeasible})',
                throttle_duration_sec=1.0)
        elif result.corrected.any():
            self._cbf_warn_count += 1
            if self._cbf_warn_count % 20 == 1:  # ~1 Hz at 20 Hz loop
                active = [tracked[i].name
                          for i in np.flatnonzero(result.corrected)
                          if i not in exempt_rows]
                if active:
                    self.get_logger().info(
                        f'CBF active on: {", ".join(active)} '
                        f'(residual {result.residual:.4f})')
        # Which commanded drones are being corrected right now (for the LEDs).
        # Exempt obstacles are not "corrected" (their row is fixed); an
        # emergency push-apart involves every commanded drone.
        if result.used_emergency_stop:
            cbf_names = [d.name for d in tracked if d.commanded]
        else:
            cbf_names = [tracked[i].name
                         for i in np.flatnonzero(result.corrected)
                         if i not in exempt_rows and tracked[i].commanded]
        self.cbf_active_pub.publish(String(data=','.join(cbf_names)))
        # ======================================================

        # Publish commands; handle landing completion.
        for d in self.drones:
            if not d.commanded or d.state == FlightState.IDLE:
                continue
            fresh = (d.last_odom_time is not None
                     and (now - d.last_odom_time)
                     < Duration(seconds=self.state_timeout))
            if not fresh:
                self.get_logger().warn(
                    f'{d.name}: odometry stale, commanding zero velocity',
                    throttle_duration_sec=1.0)
                self.publish_command(d, np.zeros(3), np.zeros(3), now)
                continue

            if d.state == FlightState.LANDING \
                    and d.position[2] <= self.land_complete_alt:
                self.send_robot_command(d, RobotCommand.Request.DISARM, 'disarm')
                d.state = FlightState.IDLE
                self.get_logger().info(f'{d.name}: landed, disarmed')
                continue

            row = index[d.name]
            velocity = safe[row]
            # keep_in fence: the wall is the last word, after the CBF (a
            # per-axis clip never turns a stop into motion). Only ACTIVE
            # drones: ASCEND/LANDING legitimately cross the floor.
            if self.fence_enabled and self.fence_behavior == 'keep_in' \
                    and d.state == FlightState.ACTIVE:
                velocity = self.keep_in(d, velocity)
            # The acceleration feedforward belongs to the profile; once the
            # CBF or the fence has altered the velocity it would push toward
            # the very thing they steered away from, so it is dropped.
            accel = accel_ff[row]
            if np.linalg.norm(velocity - nominal[row]) > 1e-6:
                accel = np.zeros(3)
            self.publish_command(d, velocity, accel, now)

        self.publish_markers(now)

    def publish_command(self, drone: DroneHandle, velocity: np.ndarray,
                        acceleration: np.ndarray, now):
        """Send one drone its command and remember it as the applied velocity.

        Trajectory output: one MultiDOFJointTrajectory point carrying the
        reference point (position setpoint), the velocity and the
        acceleration feedforward. Velocity output: a TwistStamped.
        """
        velocity = np.asarray(velocity, dtype=float)
        yaw_rate = 0.0
        if drone.role == 'teleop':
            stale = (drone.last_teleop_time is None
                     or (now - drone.last_teleop_time)
                     > Duration(seconds=self.teleop_timeout))
            yaw_rate = 0.0 if stale else drone.teleop_yaw_rate
        if drone.output == 'trajectory':
            msg = MultiDOFJointTrajectory()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'map'
            msg.joint_names = [drone.name]
            point = MultiDOFJointTrajectoryPoint()
            ref = drone.ref if drone.ref is not None else drone.position
            pose = Transform()
            pose.translation.x = float(ref[0])
            pose.translation.y = float(ref[1])
            pose.translation.z = float(ref[2])
            # Heading: an absolute ENU yaw for scenario drones (nose on +X
            # unless a goal says otherwise); teleop keeps the yaw-rate stick,
            # so its rotation is left zero (= "no yaw setpoint" downstream).
            if drone.role != 'teleop':
                yaw = self.desired_heading(drone)
                pose.rotation.z = float(np.sin(0.5 * yaw))
                pose.rotation.w = float(np.cos(0.5 * yaw))
            vel = Twist()
            vel.linear.x = float(velocity[0])
            vel.linear.y = float(velocity[1])
            vel.linear.z = float(velocity[2])
            vel.angular.z = float(yaw_rate)
            acc = Twist()
            acc.linear.x = float(acceleration[0])
            acc.linear.y = float(acceleration[1])
            acc.linear.z = float(acceleration[2])
            point.transforms = [pose]
            point.velocities = [vel]
            point.accelerations = [acc]
            msg.points = [point]
        else:
            msg = TwistStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'map'
            msg.twist.linear.x = float(velocity[0])
            msg.twist.linear.y = float(velocity[1])
            msg.twist.linear.z = float(velocity[2])
            msg.twist.angular.z = float(yaw_rate)
        drone.cmd_pub.publish(msg)
        drone.applied = velocity.copy()

    # ------------------------------------------------------------------
    # Visualization (RViz MarkerArray, world frame)
    # ------------------------------------------------------------------

    def desired_heading(self, drone: DroneHandle) -> float:
        """ENU yaw (rad) a scenario drone should hold right now."""
        return float(self.scenario.headings[self.drones.index(drone)])

    def _drone_color(self, drone: DroneHandle):
        if self.fence_breached:
            return (1.0, 0.3, 0.0)                 # orange = frozen on breach
        if drone.role == 'teleop':
            return (1.0, 0.85, 0.1)                # yellow = operator obstacle
        if drone.role == 'external':
            return (0.6, 0.6, 0.6)                 # gray = tracked, uncommanded
        return (0.9, 0.2, 0.2) if drone.mode == 'real' else (0.2, 0.7, 1.0)

    def publish_markers(self, now):
        if self.viz_pub is None:
            return
        arr = MarkerArray()
        stamp = now.to_msg()
        goals = getattr(self.scenario, 'goals', None) if self.mission_active else None

        for di, d in enumerate(self.drones):
            if d.position is None:
                continue
            r, g, b = self._drone_color(d)
            base = di * 10

            body = Marker()
            body.header.frame_id = self.viz_frame
            body.header.stamp = stamp
            body.ns = 'body'
            body.id = base
            body.type = Marker.SPHERE
            body.action = Marker.ADD
            body.pose.position.x = float(d.position[0])
            body.pose.position.y = float(d.position[1])
            body.pose.position.z = float(d.position[2])
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = body.scale.z = 0.3
            body.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            arr.markers.append(body)

            if d.role != 'teleop':
                nose = Marker()
                nose.header.frame_id = self.viz_frame
                nose.header.stamp = stamp
                nose.ns = 'heading'
                nose.id = base + 4
                nose.type = Marker.ARROW
                nose.action = Marker.ADD
                nose.pose.position = body.pose.position
                yaw = self.desired_heading(d)
                nose.pose.orientation.z = float(np.sin(0.5 * yaw))
                nose.pose.orientation.w = float(np.cos(0.5 * yaw))
                nose.scale.x, nose.scale.y, nose.scale.z = 0.45, 0.05, 0.05
                nose.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
                arr.markers.append(nose)

            keepout = Marker()
            keepout.header.frame_id = self.viz_frame
            keepout.header.stamp = stamp
            keepout.ns = 'safety_radius'
            keepout.id = base + 1
            keepout.type = Marker.SPHERE
            keepout.action = Marker.ADD
            keepout.pose.position = body.pose.position
            keepout.pose.orientation.w = 1.0
            keepout.scale.x = keepout.scale.y = keepout.scale.z = \
                2.0 * self.cbf_safety_radius
            keepout.color = ColorRGBA(r=r, g=g, b=b, a=0.12)
            arr.markers.append(keepout)

            label = Marker()
            label.header.frame_id = self.viz_frame
            label.header.stamp = stamp
            label.ns = 'label'
            label.id = base + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(d.position[0])
            label.pose.position.y = float(d.position[1])
            label.pose.position.z = float(d.position[2]) + 0.4
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'{d.name} [{d.mode}/{d.role}]'
            arr.markers.append(label)

            if goals is not None and di < len(goals):
                goal = Marker()
                goal.header.frame_id = self.viz_frame
                goal.header.stamp = stamp
                goal.ns = 'goal'
                goal.id = base + 3
                goal.type = Marker.SPHERE
                goal.action = Marker.ADD
                goal.pose.position.x = float(goals[di][0])
                goal.pose.position.y = float(goals[di][1])
                goal.pose.position.z = float(goals[di][2])
                goal.pose.orientation.w = 1.0
                goal.scale.x = goal.scale.y = goal.scale.z = 0.15
                goal.color = ColorRGBA(r=r, g=g, b=b, a=0.6)
                arr.markers.append(goal)

        if self.fence_enabled:
            arr.markers.append(self._fence_marker(stamp))

        self.viz_pub.publish(arr)

    def _fence_marker(self, stamp):
        lo, hi = self.fence_min, self.fence_max
        corners = [
            (lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]),
            (hi[0], hi[1], lo[2]), (lo[0], hi[1], lo[2]),
            (lo[0], lo[1], hi[2]), (hi[0], lo[1], hi[2]),
            (hi[0], hi[1], hi[2]), (lo[0], hi[1], hi[2]),
        ]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7),
                 (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
        m = Marker()
        m.header.frame_id = self.viz_frame
        m.header.stamp = stamp
        m.ns = 'fence'
        m.id = 9000
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        breached = self.fence_breached
        m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9) if breached \
            else ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.5)
        from geometry_msgs.msg import Point
        for a, c in edges:
            for idx in (a, c):
                m.points.append(Point(x=float(corners[idx][0]),
                                      y=float(corners[idx][1]),
                                      z=float(corners[idx][2])))
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
