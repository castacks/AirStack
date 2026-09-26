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
               clamped into the box. The cap is a braking envelope
               (``fence_brake_accel_mps2``, ``fence_keep_in_gain``): cruise
               speed until the true braking distance, then a firm brake, with
               the envelope's deceleration sent to PX4 as the acceleration
               feedforward on the trajectory output. External drones cannot
               be steered, so they are only reported.
A second, smaller box — the TELEOP FENCE (``teleop_fence_enabled``,
``teleop_fence_min``/``max``) — bounds hand-flown drones the same keep_in
way whatever ``fence_behavior`` is, so the sticks meet a soft wall well inside
the geofence and the geofence stays the outer safety net. It must lie inside
the geofence.

Visualization: every drone's WORLD position (offset-corrected, so real and
simulated drones share one frame) is published as a MarkerArray on
``/svg/viz/markers`` for RViz.

Status: a JSON snapshot (std_msgs/String) goes out on ``status_topic``
(default ``/svg/commander_status``) at ``status_rate_hz``: mission state,
the outcome of the last lifecycle command, the live CBF gains, and per drone
its flight state, world position, odometry freshness and the result of its
last robot_command (offboard / arm / disarm). The SVG Basestation Foxglove
panel reads it to confirm a command actually reached the commander and to
show numeric positions. See ``build_status``.

Runtime tuning: the CBF gains ``cbf_alpha``, ``cbf_safety_radius_m`` and
``cbf_max_speed_mps`` are applied on the next control tick when set at
runtime (``ros2 param set /swarm_commander cbf_alpha 4.0`` or the panel's CBF
sliders); non-positive or non-finite values are rejected. The scenario keeps
the safety radius it was built with for its own spacing checks (holder posts,
random goals) — only the filter, the speed cap and the viz spheres follow.
The speed and tracking gains (``SwarmCommander.RUNTIME_PARAMS``) are live
as well; anything else is read once at startup and a runtime set is
refused with a reason.

Lifecycle (std_srvs/Trigger services):
    ~/takeoff — arm + offboard + ascend everyone to the scenario's initial
                positions, then HOLD there
    ~/start   — begin the scenario (nominal policies go live)
    ~/hold    — pause: every drone holds its current position (panic button)
    ~/land    — descend all commanded drones, disarm on touchdown
    ~/reset_fence — clear a latched geofence breach
"""

import json
import math
import os
import re
import signal
import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.event_handler import SubscriptionEventCallbacks, UnsupportedEventTypeError
from rclpy.parameter import Parameter

from geometry_msgs.msg import Point, PoseStamped, Transform, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import ColorRGBA, Float32, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from airstack_msgs.srv import RobotCommand

from svg_ground_control.cbf_filter import filter_velocities
from svg_ground_control.fence import (BEHAVIORS as FENCE_BEHAVIORS, box_contains,
                                      clamp_to_box, keep_in_acceleration,
                                      keep_in_velocity, outside, violation_text)
from svg_ground_control.position_hold import (advance_reference, leash, ramp_velocity,
                                              tracking_velocity)
from svg_ground_control.scenarios import Bounds, make_scenario
from svg_ground_control.trajectory import seek_velocity, stopping_distance


def heading_to_yaw(theta_deg: float) -> float:
    """Operator heading (degrees, 0 = +X, clockwise positive) -> ENU yaw (rad)."""
    return -np.radians(float(theta_deg))


def yaw_to_heading(yaw: float) -> float:
    """ENU yaw (rad) -> operator heading in degrees (0 = +X, clockwise)."""
    return float((-np.degrees(yaw) + 180.0) % 360.0 - 180.0)


def yaw_of(q) -> float:
    """ENU yaw (rad) of an (x, y, z, w) quaternion."""
    x, y, z, w = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


class FlightState(Enum):
    IDLE = 0       # on the ground, not commanded
    ARMING = 1     # streaming zero setpoints, requesting offboard + arm
    ASCEND = 2     # climbing to the takeoff target
    ACTIVE = 3     # holding / following the scenario or teleop
    LANDING = 4    # descending; disarm at land_complete_altitude


# Drone body mesh. Same asset and axis convention as the GCS visualiser
# (gcs_visualizer/foxglove_visualizer_node.py) so both views agree. Foxglove
# resolves package:// server-side through foxglove_bridge's asset capability,
# so robot_descriptions must be built into this workspace.
# STL, not the OBJ: the OBJ carries an `mtllib` line, so Foxglove fetches the
# sibling .mtl and takes the "mesh provides its own material" path, which fights
# the status colour. STL is self-contained (one asset, no sibling fetch) and
# Foxglove always treats it as material-less, so marker.color is what shows.
DRONE_MESH = 'package://robot_descriptions/iris/meshes/base_link_body_body.stl'
# Rotates the OBJ from its authored axes to belly -Z / nose +X.
AXIS_CORRECTION = (-0.5, -0.5, 0.5, 0.5)

# Drone name label (TEXT_VIEW_FACING). Foxglove's 3D renderer always draws a
# text marker on a contrasting box — black behind light text, white behind
# dark text (relative luminance < 0.5) — with the box's alpha equal to the
# text's, and the font is its own sans-serif atlas (same family the panel
# uses). Neither the box nor the font can be switched off from the message, so
# the text is the panel's dark slate (#1f2937): it gets a white chip instead
# of a black one, matching the panel's chips, and 0.9 alpha keeps both text
# and chip slightly translucent over the scene.
LABEL_COLOR = ColorRGBA(r=0.122, g=0.161, b=0.216, a=0.9)


def _quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


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
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # (x,y,z,w), identity until first odometry
        self.velocity = np.zeros(3)
        self.last_odom_time = None        # rclpy Time
        # Odometry reception counters for the status topic. odom_lost_total is
        # the DDS reader's own count of samples it saw go missing (RTPS
        # sequence-number gaps, via the message_lost subscription event) — a
        # measured loss, unlike anything derived from arrival timing.
        # odom_loss_counter is 'dds' when that event is wired, else
        # 'unsupported' (rmw without the event) and odom_lost_total stays 0.
        self.odom_rx_total = 0
        self.odom_lost_total = 0
        self.odom_loss_counter = 'unsupported'
        self.arming_start = None          # rclpy Time
        self.arming_steps_done = set()
        self.cmd_pub = None
        self.robot_command_client = None
        self.teleop_twist = np.zeros(3)
        self.teleop_yaw_rate = 0.0
        self.last_teleop_time = None
        # Stick acceleration ramp (position_hold.ramp_velocity): the ramp's
        # last velocity, None = start from what was last published.
        self.teleop_profile = None
        # ENU yaw held while the yaw stick is centred (trajectory output);
        # None = adopt the measured yaw on the next tick.
        self.teleop_yaw_hold = None
        # Reference point (world ENU): where the drone was told to be,
        # integrated from the published velocity (position_hold.advance_
        # reference). PX4's position setpoint on the trajectory output, the
        # point the go-to-goal profile is evaluated at, and the teleop hold
        # point. None = re-seed at the drone next tick (control hand-over).
        self.ref = None
        # Velocity the reference last moved with (what was published, or the
        # measured velocity after a seed/leash) — the scenario profile
        # re-attaches to it.
        self.applied = np.zeros(3)
        # What was actually published last tick (post-CBF, post-fence). The
        # teleop stick ramp re-attaches to THIS, never to `applied`: after a
        # leash pull `applied` is the drone's measured velocity, and a ramp
        # restarting from that adopts the drone's sink and overrun as the
        # command (bag run_060352: altitude walked down 1.1 m on pure x-y
        # stick, and released sticks kept it coasting).
        self.published = np.zeros(3)
        # Outcome of the most recent robot_command (offboard / arm / disarm)
        # sent to this drone, for the status topic:
        # {'label', 'result': 'pending'|'ok'|'rejected'|'error'|'skipped',
        #  'message', 'stamp'}. None until the first one is sent.
        self.last_robot_command = None

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
        # keep_in (and the teleop fence): the wall is a braking envelope.
        #   fence_brake_accel_mps2  deceleration the fence brakes with. The
        #             outward speed is capped at the speed from which a stop
        #             AT the wall is still reachable at this deceleration
        #             (sqrt(2 a d) far out), so cruise speed is kept until the
        #             true braking distance and then the brake is firm; on
        #             the trajectory output the deceleration is also sent to
        #             PX4 as the acceleration feedforward. Higher = brake
        #             later and harder. 0 = the plain `gain * distance`
        #             barrier, which overshot by 0.5 m at 6 m/s (bag
        #             run_045417): it starts slowing far too early and is
        #             still faster than the vehicle's lag can follow.
        #   fence_keep_in_gain  near the wall the outward speed is at most
        #             gain * distance (1/s) — the tail into the wall, and the
        #             push-back rate when outside. 1/gain is also the response
        #             lag the envelope allows for, so 2 brakes later and harder
        #             than 1. Use 2 on the trajectory output (feedforward;
        #             3 rings at the wall through the 0.3 s loop delay) and
        #             0.7 with brake 2 for a bare velocity setpoint (sim:
        #             ~0.7 s lag, no feedforward).
        #   fence_margin_m  shrinks the box so the wall is met that early (m).
        self.declare_parameter('fence_brake_accel_mps2', 4.0)
        self.declare_parameter('fence_keep_in_gain', 1.0)
        self.declare_parameter('fence_margin_m', 0.0)
        # ---- Teleop fence -------------------------------------------------
        # A smaller box for hand-flown (teleop) drones only, INSIDE the
        # geofence: the sticks meet this soft wall (keep_in barrier, same
        # gain / brake / margin as above) whatever fence_behavior is, so a
        # pilot cannot reach the geofence, which stays the outer safety net.
        # Scenario-driven and external drones ignore it. Off = teleop drones
        # are bounded by the geofence like everyone else.
        self.declare_parameter('teleop_fence_enabled', False)
        self.declare_parameter('teleop_fence_min', [-1000.0, -1000.0, -1000.0])
        self.declare_parameter('teleop_fence_max', [1000.0, 1000.0, 1000.0])
        # Ground-plane grid drawn on the fence floor, clipped to the fence
        # footprint and aligned to world multiples of this cell size (so x=0 /
        # y=0 fall on lines and whole metres are drawn brighter). Replaces the
        # 3D panel's built-in grid, which is a fixed square centred on the
        # origin and never matches the fence. 0 disables it.
        self.declare_parameter('fence_grid_cell_m', 0.5)

        # ---- Visualization ----------------------------------------------
        self.declare_parameter('publish_viz', True)
        self.declare_parameter('viz_frame', 'map')

        # ---- Status snapshot (std_msgs/String, JSON) ----------------------
        # Read by the SVG Basestation panel. 0 disables it.
        self.declare_parameter('status_topic', '/svg/commander_status')
        # On start-up, kill any other swarm_commander process in this
        # container (a forgotten launch in another terminal) — unless its
        # status says it has a drone in the air, in which case that one keeps
        # flying and THIS one refuses takeoff/start until it is landed.
        self.declare_parameter('takeover_twins', True)
        self.declare_parameter('status_rate_hz', 5.0)

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
        # Stick acceleration (m/s^2): the stick velocity is ramped at this
        # rate and the ramp's acceleration is fed forward to PX4 on the
        # trajectory output, like PX4's own Position mode (MPC_ACC_HOR_MAX,
        # default 5). A bare velocity step is only followed at ~4 m/s^2 by
        # the velocity loop (drone_2, bag run_053740), which is what kept it
        # at 3.9 m/s in a 7.7 m box: in the plant model 5 lifts the
        # wall-to-wall peak from 4.3 to 5.2 m/s, 8 to 5.4 m/s (~40 deg bank).
        # 0 = no ramp (the stick is sent as is). Live: ros2 param set.
        self.declare_parameter('teleop_accel_mps2', 0.0)
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
        self.fence_brake_accel = float(self.get_parameter('fence_brake_accel_mps2').value)
        self.fence_margin = float(self.get_parameter('fence_margin_m').value)
        self.fence_grid_cell = float(self.get_parameter('fence_grid_cell_m').value)
        self.fence_breached = False
        if not (self.fence_keep_in_gain > 0.0):
            raise ValueError(f'fence_keep_in_gain must be > 0, got {self.fence_keep_in_gain}')
        if self.fence_enabled and np.any(self.fence_min >= self.fence_max):
            raise ValueError(f'fence_min {self.fence_min} must be below fence_max '
                             f'{self.fence_max} on every axis')
        # Teleop fence: a smaller keep_in box for hand-flown drones.
        self.teleop_fence_enabled = bool(self.get_parameter('teleop_fence_enabled').value)
        self.teleop_fence_min = np.array(self.get_parameter('teleop_fence_min').value, dtype=float)
        self.teleop_fence_max = np.array(self.get_parameter('teleop_fence_max').value, dtype=float)
        if self.teleop_fence_enabled:
            if np.any(self.teleop_fence_min >= self.teleop_fence_max):
                raise ValueError(
                    f'teleop_fence_min {self.teleop_fence_min} must be below '
                    f'teleop_fence_max {self.teleop_fence_max} on every axis')
            if self.fence_enabled and not box_contains(
                    self.teleop_fence_min, self.teleop_fence_max,
                    self.fence_min, self.fence_max):
                raise ValueError(
                    f'the teleop fence {self.teleop_fence_min}..{self.teleop_fence_max} '
                    f'must lie inside the geofence {self.fence_min}..{self.fence_max}')

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
        self.teleop_accel = float(self.get_parameter('teleop_accel_mps2').value)
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
        # mission_active alone cannot tell "never started" from "stopped after
        # running" — after ~/hold both are ACTIVE with mission_active False.
        # This latch separates them, which is what the viz colour keys off.
        self.mission_ever_started = False
        self.mission_started_at = None    # seconds, wall/ROS clock of last ~/start
        # Last lifecycle service outcome, for the status topic: {'seq',
        # 'name', 'success', 'message', 'stamp'}. The operator's panel shows
        # this to prove a command reached the commander, independently of
        # whether the service reply made it back over the link.
        self._command_seq = 0
        self._last_command = None
        # Names the CBF corrected on the latest control tick (status topic).
        self._cbf_active_names = []
        self._cbf_emergency = False
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
            self._subscribe_odometry(drone, state_tmpl.format(name=name))
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

        # ---- Status snapshot -------------------------------------------------
        self.status_topic = str(self.get_parameter('status_topic').value)
        status_rate = float(self.get_parameter('status_rate_hz').value)
        self.status_pub = None
        self.status_timer = None
        if self.status_topic and status_rate > 0.0:
            self.status_pub = self.create_publisher(String, self.status_topic, 10)
            self.status_timer = self.create_timer(1.0 / status_rate, self.publish_status)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.control_dt = 1.0 / rate
        self.timer = self.create_timer(self.control_dt, self.control_loop)
        self._cbf_warn_count = 0

        # ---- Runtime-tunable parameters ---------------------------------------
        # `ros2 param set` used to answer "successful" and change nothing:
        # every value above is read once at construction. RUNTIME_PARAMS are
        # applied live — the CBF gains (alpha, safety radius, max speed) are
        # read from their self.cbf_* attributes on every control tick, so a
        # `ros2 param set` or the basestation panel's CBF sliders take effect
        # on the next tick; the speed / tracking gains likewise — and anything
        # else is refused with a reason instead of being silently ignored.
        # Validation happens in the pre-set callback; the value is applied
        # only once the parameter has actually been stored, so a rejected
        # batch never leaves the node running with an unset gain.
        # Registered LAST: rclpy also runs these callbacks for every
        # declare_parameter above.
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # Two commanders on one drone (a second ground_control.launch.py in
        # another terminal) both publish setpoints and goal markers; bags
        # run_041842 / run_044243 show every log line twice and the goal
        # sphere flipping between two points. Watch the graph for a twin.
        self.duplicate_commander = False
        self.create_timer(1.0, self.check_duplicate_commander)
        # Take over from twin processes once their status has had time to
        # arrive (status_rate_hz is a few Hz): see takeover_twins().
        self._twin_snapshots = {}
        self.create_subscription(String, str(self.get_parameter('status_topic').value),
                                 self._on_twin_status, 10)
        if bool(self.get_parameter('takeover_twins').value):
            self._takeover_timer = self.create_timer(2.0, self.takeover_twins)
        if hasattr(self, 'add_post_set_parameters_callback'):   # rclpy >= Iron
            self.add_post_set_parameters_callback(self._apply_parameters)
        else:
            self._apply_in_validate = True

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
               if self.fence_enabled else ' | fence OFF')
            + (f' | TELEOP FENCE {self.teleop_fence_min}..{self.teleop_fence_max}'
               if self.teleop_fence_enabled else '')
            + (f' | keep_in brake {self.fence_brake_accel} m/s2, gain '
               f'{self.fence_keep_in_gain} 1/s, margin {self.fence_margin} m'
               if self.teleop_fence_enabled
               or (self.fence_enabled and self.fence_behavior == 'keep_in') else ''))
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
    # Runtime parameters
    # ------------------------------------------------------------------

    # Parameters that may change while flying (`ros2 param set
    # /swarm_commander ...`, or the basestation panel's set_parameters), and
    # how they are applied. Everything else is wiring/geometry read once at
    # startup; setting it is refused with a reason (on_parameter_change).
    # The CBF gains must be finite and > 0; the rest are plain numbers.
    CBF_PARAMS = ('cbf_alpha', 'cbf_safety_radius_m', 'cbf_max_speed_mps')
    # parameter name -> attribute the control loop reads
    _RUNTIME_ATTRS = {
        'cbf_alpha': 'cbf_alpha',
        'cbf_safety_radius_m': 'cbf_safety_radius',
        'cbf_max_speed_mps': 'cbf_max_speed',
        'teleop_max_speed_mps': 'teleop_max_speed',
        'goal_lead_m': 'goal_lead',
        'teleop_kp': 'teleop_kp',
        'teleop_lead_m': 'teleop_lead',
        'teleop_accel_mps2': 'teleop_accel',
        'hover_kp': 'hover_kp',
        'hold_lead_m': 'hold_lead',
        'takeoff_speed_mps': 'takeoff_speed',
        # keep_in / teleop fence dynamics (the boxes themselves are geometry:
        # startup only). Read by keep_in() on every control tick.
        'fence_keep_in_gain': 'fence_keep_in_gain',
        'fence_brake_accel_mps2': 'fence_brake_accel',
        'fence_margin_m': 'fence_margin',
    }
    # Must stay > 0 (1/gain is the envelope's lag; 0 would divide by it).
    POSITIVE_PARAMS = CBF_PARAMS + ('fence_keep_in_gain',)
    # Applied through the scenario (its speeds / go-to-goal tracker).
    _SCENARIO_PARAMS = ('scenario_speed_mps', 'goal_accel_mps2', 'goal_settle_s',
                        'goal_velocity_only_settle_s')
    RUNTIME_PARAMS = tuple(_RUNTIME_ATTRS) + _SCENARIO_PARAMS
    _apply_in_validate = False

    def on_parameter_change(self, params):
        """Pre-set callback: validate the live parameters, refuse the rest."""
        for p in params:
            if p.name not in self.RUNTIME_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} is read once at startup; change the YAML '
                           'and relaunch (live: '
                           + ', '.join(self.RUNTIME_PARAMS) + ')')
            if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must be a number, got {p.type_.name}')
            value = float(p.value)
            if p.name in self.POSITIVE_PARAMS and (
                    not math.isfinite(value) or value <= 0.0):
                return SetParametersResult(
                    successful=False,
                    reason=f'{p.name} must be finite and > 0, got {p.value}')
        if self._apply_in_validate:
            self._apply_parameters(params)
        return SetParametersResult(successful=True)

    def _apply_parameters(self, params):
        """Post-set callback: the parameter is stored, apply it to the node."""
        for p in params:
            if p.name not in self.RUNTIME_PARAMS:
                continue
            value = float(p.value)
            if p.name == 'scenario_speed_mps':
                if hasattr(self.scenario, 'set_all_speeds'):
                    self.scenario.set_all_speeds(value)
                else:
                    self.scenario.nominal_speed = max(0.0, value)
                self.get_logger().info(
                    f'scenario_speed_mps -> {value:.2f} m/s (live, all scenario '
                    f'drones){self.speed_cap_note(value)}')
            elif p.name == 'goal_accel_mps2':
                self.scenario.tracker.accel = max(0.1, value)
                self.get_logger().info(f'goal_accel_mps2 -> {p.value} (live)')
            elif p.name == 'goal_settle_s':
                self.scenario.tracker.settle = max(0.0, value)
                self.get_logger().info(f'goal_settle_s -> {p.value} (live)')
            elif p.name == 'goal_velocity_only_settle_s':
                self.scenario.velocity_only_settle = max(0.0, value)
                self.get_logger().info(
                    f'goal_velocity_only_settle_s -> {p.value} (live)')
            else:
                attr = self._RUNTIME_ATTRS[p.name]
                old = getattr(self, attr)
                if value != old:
                    self.get_logger().info(
                        f'{p.name} {old:g} -> {value:g} '
                        '(applied on the next control tick)')
                setattr(self, attr, value)

    # ------------------------------------------------------------------
    # Status snapshot
    # ------------------------------------------------------------------

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _finite_list(vec):
        """3-vector -> list of rounded floats, or None if any entry is not finite.

        JSON has no NaN/Inf; a drone with a broken estimate must read as
        "no position" rather than poison the whole snapshot.
        """
        if vec is None:
            return None
        vals = [float(v) for v in vec]
        if not all(math.isfinite(v) for v in vals):
            return None
        return [round(v, 3) for v in vals]

    def _record_command(self, name: str, response):
        """Remember a lifecycle service outcome for the status topic."""
        self._command_seq += 1
        self._last_command = {
            'seq': self._command_seq,
            'name': name,
            'success': bool(response.success),
            'message': str(response.message),
            'stamp': self._now_s(),
        }
        return response

    def build_status(self) -> dict:
        """The snapshot published on status_topic (JSON-serialisable)."""
        now = self.get_clock().now()
        timeout = Duration(seconds=self.state_timeout)
        drones = []
        for d in self.drones:
            odom_age = (None if d.last_odom_time is None
                        else (now - d.last_odom_time).nanoseconds * 1e-9)
            fresh = d.last_odom_time is not None and (now - d.last_odom_time) < timeout
            position = self._finite_list(d.position)
            speed = None
            if position is not None:
                s = float(np.linalg.norm(d.velocity))
                speed = round(s, 3) if math.isfinite(s) else None
            drones.append({
                'name': d.name,
                'role': d.role,
                'mode': d.mode,
                'commanded': d.commanded,
                'cbf_exempt': d.name in self.cbf_exempt_names,
                'state': d.state.name,
                # World ENU = odometry + this offset. Published so the panel
                # can put its own odometry-derived numbers (and the goals it
                # sends) in exactly the frame the commander plans in.
                'position_offset': self._finite_list(d.position_offset),
                'position': position,
                'speed_mps': speed,
                'hold_target': (self._finite_list(d.hold_target)
                                if d.state in (FlightState.ASCEND, FlightState.ACTIVE)
                                else None),
                'odom_fresh': bool(fresh),
                'odom_age_s': None if odom_age is None else round(odom_age, 3),
                # Cumulative counters; the panel differences them over its
                # window to get a measured drop rate (lost / (lost + received)).
                'odom_rx_total': d.odom_rx_total,
                'odom_lost_total': d.odom_lost_total,
                'odom_loss_counter': d.odom_loss_counter,
                'cbf_active': d.name in self._cbf_active_names,
                'robot_command': d.last_robot_command,
            })
        return {
            'stamp': round(now.nanoseconds * 1e-9, 3),
            'node': self.get_fully_qualified_name(),
            'pid': os.getpid(),
            'scenario': self.scenario_name,
            'mission_active': self.mission_active,
            'mission_ever_started': self.mission_ever_started,
            'mission_started_at': self.mission_started_at,
            'fence_enabled': self.fence_enabled,
            'fence_breached': self.fence_breached,
            'fence': {
                'behavior': self.fence_behavior,
                'min': self._finite_list(self.fence_min),
                'max': self._finite_list(self.fence_max),
                'keep_in_gain': self.fence_keep_in_gain,
                'brake_accel_mps2': self.fence_brake_accel,
                'margin_m': self.fence_margin,
            },
            'teleop_fence': {
                'enabled': self.teleop_fence_enabled,
                'min': self._finite_list(self.teleop_fence_min),
                'max': self._finite_list(self.teleop_fence_max),
            },
            'cbf': {
                'alpha': self.cbf_alpha,
                'safety_radius_m': self.cbf_safety_radius,
                'max_speed_mps': self.cbf_max_speed,
                'external_velocity_gain': self.cbf_external_velocity_gain,
                'active': list(self._cbf_active_names),
                'emergency': self._cbf_emergency,
            },
            'command_seq': self._command_seq,
            'last_command': self._last_command,
            'drones': drones,
        }

    def publish_status(self):
        if self.status_pub is None:
            return
        self.status_pub.publish(String(data=json.dumps(
            self.build_status(), separators=(',', ':'), allow_nan=False)))

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def _subscribe_odometry(self, drone: DroneHandle, topic: str):
        """Subscribe to a drone's odometry with DDS loss accounting.

        The reader-side ``message_lost`` event is the only true drop counter
        available: the RTPS layer numbers every sample a writer sends and the
        reader reports the gaps it could not fill. (With a RELIABLE pairing a
        gap only counts once the writer's history has aged the sample out, i.e.
        once it is genuinely unrecoverable — which is the loss the commander
        actually experiences.) An rmw without the event falls back to a plain
        subscription and the status topic says so.
        """
        callback = (lambda msg, d=drone: self.odometry_callback(d, msg))
        events = SubscriptionEventCallbacks(
            message_lost=lambda info, d=drone: self._on_odometry_lost(d, info))
        try:
            self.create_subscription(Odometry, topic, callback, 10,
                                     event_callbacks=events)
            drone.odom_loss_counter = 'dds'
        except UnsupportedEventTypeError:
            self.get_logger().warn(
                f'{drone.name}: rmw has no message_lost event; odometry drop '
                'count unavailable (status reports odom_loss_counter=unsupported)')
            self.create_subscription(Odometry, topic, callback, 10)

    def _on_odometry_lost(self, drone: DroneHandle, info):
        # info.total_count is cumulative for the life of the subscription.
        drone.odom_lost_total = int(info.total_count)
        self.get_logger().warn(
            f'{drone.name}: odometry samples lost: +{int(info.total_count_change)} '
            f'(total {drone.odom_lost_total})',
            throttle_duration_sec=2.0)

    def _on_twin_status(self, msg: String):
        """Snapshots on the status topic that are NOT ours (a twin's)."""
        try:
            snap = json.loads(msg.data)
        except ValueError:
            return
        if snap.get('pid') == os.getpid():
            return
        self._twin_snapshots[snap.get('pid', 'unknown')] = (time.monotonic(), snap)

    @staticmethod
    def twin_pids():
        """PIDs of other swarm_commander node processes in this container.

        The node executable is the last token of a python cmdline ending in
        ``/swarm_commander``; ``ros2 run …`` / ``ros2 launch …`` wrappers carry
        the name only as a bare argument and are left alone (they exit on
        their own once their child is gone).
        """
        pids = []
        me = os.getpid()
        for entry in os.listdir('/proc'):
            if not entry.isdigit() or int(entry) == me:
                continue
            try:
                with open(f'/proc/{entry}/cmdline', 'rb') as f:
                    argv = f.read().split(b'\0')
            except OSError:
                continue
            if any(tok.endswith(b'/swarm_commander') for tok in argv[:3]):
                pids.append(int(entry))
        return pids

    def takeover_twins(self):
        """One shot, 2 s after start-up: kill twin commanders that are idle."""
        self._takeover_timer.cancel()
        twins = self.twin_pids()
        if not twins:
            return
        airborne = []
        for pid, (_, snap) in self._twin_snapshots.items():
            for d in snap.get('drones', []):
                if d.get('state') not in (None, 'IDLE'):
                    airborne.append(f"{d.get('name')} {d.get('state')} (pid {pid})")
        if airborne:
            self.get_logger().error(
                f'another swarm_commander (pids {twins}) has drones in the air: '
                + ', '.join(airborne) + ' — NOT killing it. Land with THAT one '
                '(or pkill -f swarm_commander), then relaunch; takeoff/start '
                'are refused here meanwhile.')
            return
        for pid in twins:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and any(
                os.path.exists(f'/proc/{pid}') for pid in twins):
            time.sleep(0.1)
        for pid in twins:
            if os.path.exists(f'/proc/{pid}'):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.get_logger().warn(
            f'took over: killed idle swarm_commander process(es) {twins} '
            '(left behind by an earlier launch). Set takeover_twins:=false '
            'to disable this.')
        self.duplicate_commander = False

    def check_duplicate_commander(self):
        twins = sum(1 for name, ns in self.get_node_names_and_namespaces()
                    if name == self.get_name() and ns == self.get_namespace())
        self.duplicate_commander = twins > 1
        if self.duplicate_commander:
            self.get_logger().error(
                f'{twins} swarm_commander nodes are running — a second '
                'ground_control launch is commanding the same drones. '
                'Kill the extra one (takeoff/start are refused meanwhile).',
                throttle_duration_sec=2.0)

    def odometry_callback(self, drone: DroneHandle, msg: Odometry):
        drone.odom_rx_total += 1
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        # position_offset shifts each drone's local-origin odometry into the
        # shared world frame (velocities are origin-independent).
        q = msg.pose.pose.orientation
        drone.position = np.array([p.x, p.y, p.z]) + drone.position_offset
        drone.orientation = (q.x, q.y, q.z, q.w)
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
        # A new sortie has not run the planner yet.
        self.mission_ever_started = False
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
            # A stored goal from the previous sortie must not be what ~/start
            # flies to: every takeoff resets this drone's goal (and heading)
            # to its takeoff point. Retarget after takeoff, before or after
            # start — goal commands are accepted at any time.
            if hasattr(self.scenario, 'set_goal'):
                self.scenario.set_goal(self.drones.index(d), d.takeoff_target, 0.0)
            started.append(d.name)
        if started and hasattr(self.scenario, 'set_goal'):
            self.get_logger().info(
                'takeoff: goals reset to the takeoff points ('
                + ', '.join(f'{d.name} {d.takeoff_target.round(2).tolist()}'
                            for d in self.drones if d.name in started) + ')')
        if self.duplicate_commander:
            response.success = False
            response.message = ('REFUSED: another swarm_commander is running '
                                '(ros2 node list) — kill it first')
            return self._record_command('takeoff', response)
        response.success = bool(started)
        response.message = ('takeoff: ' + ', '.join(started)) if started \
            else 'no drone eligible for takeoff (missing odometry or not IDLE)'
        return self._record_command('takeoff', response)

    def handle_start(self, request, response):
        if self.duplicate_commander:
            response.success = False
            response.message = ('REFUSED: another swarm_commander is running '
                                '(ros2 node list) — kill it first')
            return self._record_command('start', response)
        if self.fence_breached:
            response.success = False
            response.message = 'geofence breached — call ~/reset_fence first'
            return self._record_command('start', response)
        not_ready = [d.name for d in self.drones
                     if d.commanded and d.state != FlightState.ACTIVE]
        if not_ready:
            response.success = False
            response.message = 'not all drones holding yet: ' + ', '.join(not_ready)
            return self._record_command('start', response)
        self.mission_active = True
        self.mission_ever_started = True
        self.mission_started_at = self._now_s()
        self.scenario.reset_tracking()
        for d in self.drones:
            d.ref = None                # seed from where the drone IS, now
        response.success = True
        response.message = f'scenario "{self.scenario_name}" running'
        self.get_logger().info(response.message)
        return self._record_command('start', response)

    def handle_hold(self, request, response):
        self.mission_active = False
        held = []
        for d in self.drones:
            if d.commanded and d.position is not None \
                    and d.state in (FlightState.ASCEND, FlightState.ACTIVE):
                d.hold_target = self.stop_point(d)
                d.ref = None
                d.state = FlightState.ACTIVE
                ahead = float(np.linalg.norm(d.hold_target - d.position))
                held.append(d.name + (f' (braking, stops {ahead:.1f} m ahead)'
                                      if ahead > 0.1 else ''))
        response.success = bool(held)
        response.message = 'holding: ' + ', '.join(held) if held else 'nothing to hold'
        if held:
            self.get_logger().info(response.message)
        return self._record_command('hold', response)

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
        return self._record_command('land', response)

    def handle_reset_fence(self, request, response):
        still_out = [d.name for d in self.drones if d.position is not None
                     and (np.any(d.position < self.fence_min)
                          or np.any(d.position > self.fence_max))]
        self.fence_breached = False
        response.success = True
        response.message = 'geofence latch cleared' + (
            f' (WARNING still outside: {", ".join(still_out)})' if still_out else '')
        self.get_logger().info(response.message)
        return self._record_command('reset_fence', response)

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
                    o.hold_target = self.stop_point(o)
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

    def keep_in_box(self, drone: DroneHandle):
        """The box that bounds this drone's commands as a keep_in wall, or None.

        Teleop drones get the teleop fence when it is enabled — it lies
        inside the geofence, so it is the tighter of the two on every axis
        and the only one that needs applying. Everyone else (and teleop
        drones without one) gets the geofence when it is in keep_in mode.
        Under hold_all the geofence never clips; it latches (enforce_fence).
        """
        if drone.role == 'teleop' and self.teleop_fence_enabled:
            return self.teleop_fence_min, self.teleop_fence_max
        if self.fence_enabled and self.fence_behavior == 'keep_in':
            return self.fence_min, self.fence_max
        return None

    def keep_in(self, drone: DroneHandle, velocity: np.ndarray):
        """Clip a commanded velocity at the drone's keep_in walls.

        Returns ``(velocity, acceleration)``: the clipped velocity and the
        braking feedforward for the limited axes (zeros when nothing was
        limited or no box applies) — see fence.keep_in_acceleration.
        """
        box = self.keep_in_box(drone)
        if box is None:
            return velocity, np.zeros(3)
        lo, hi = box
        clipped = keep_in_velocity(velocity, drone.position, lo, hi,
                                   self.fence_keep_in_gain, self.fence_margin,
                                   self.fence_brake_accel)
        accel = keep_in_acceleration(velocity, clipped, drone.velocity,
                                     drone.position, lo, hi,
                                     self.fence_keep_in_gain, self.fence_margin,
                                     self.fence_brake_accel)
        if np.linalg.norm(clipped - velocity) > 0.05:
            which = 'teleop fence' if (drone.role == 'teleop'
                                       and self.teleop_fence_enabled) else 'fence'
            self.get_logger().info(
                f'{which} keep-in: {drone.name} limited on '
                + ''.join('xyz'[k] for k in range(3)
                          if abs(clipped[k] - velocity[k]) > 1e-6)
                + (f' (brake {np.linalg.norm(accel):.1f} m/s2)'
                   if np.any(accel) else ''),
                throttle_duration_sec=1.0)
        if drone.role == 'teleop' and self.teleop_fence_enabled \
                and drone.state == FlightState.ACTIVE \
                and outside(drone.position, lo, hi).any():
            self.get_logger().warn(
                f'{drone.name} outside the teleop fence '
                f'({violation_text(drone.position, lo, hi)}), pushing it back',
                throttle_duration_sec=2.0)
        return clipped, accel

    # ------------------------------------------------------------------
    # Robot interface helpers
    # ------------------------------------------------------------------

    def send_robot_command(self, drone: DroneHandle, command: int, label: str):
        def note(result, message=''):
            # Surfaced on the status topic so the operator can see whether
            # offboard / arm / disarm actually reached this drone's interface.
            drone.last_robot_command = {
                'label': label, 'result': result, 'message': message,
                'stamp': self._now_s(),
            }

        client = drone.robot_command_client
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{drone.name}: robot_command service not ready, skipping {label}')
            note('skipped', 'robot_command service not ready')
            return
        req = RobotCommand.Request()
        req.command = command
        future = client.call_async(req)
        note('pending')

        def report(fut, name=drone.name, label=label):
            try:
                ok = fut.result().success
            except Exception as e:  # noqa: BLE001 - log any service failure
                self.get_logger().error(f'{name}: {label} failed: {e}')
                note('error', str(e))
                return
            # rclpy caches severity per call site; success and failure need
            # separate sites or alternating async replies raise
            # "Logger severity cannot be changed between calls" and kill the node.
            if ok:
                self.get_logger().info(f'{name}: {label} -> success={ok}')
                note('ok')
            else:
                self.get_logger().error(f'{name}: {label} -> success={ok}')
                note('rejected', 'interface returned success=False')

        future.add_done_callback(report)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def teleop_stick(self, drone: DroneHandle, now) -> np.ndarray:
        """The operator's stick velocity (zero when the teleop topic is stale)."""
        stale = (drone.last_teleop_time is None
                 or (now - drone.last_teleop_time)
                 > Duration(seconds=self.teleop_timeout))
        cmd = np.zeros(3) if stale else drone.teleop_twist.copy()
        speed = np.linalg.norm(cmd)
        if speed > self.teleop_max_speed + 1e-3:
            cmd *= self.teleop_max_speed / speed
            self.get_logger().warn(
                f'{drone.name}: stick velocity {speed:.2f} m/s capped to '
                f'teleop_max_speed_mps={self.teleop_max_speed} (raise it in the '
                'commander block or: ros2 param set /swarm_commander '
                'teleop_max_speed_mps X)', throttle_duration_sec=2.0)
        return cmd

    def teleop_command(self, drone: DroneHandle, now):
        """The stick velocity, ramped at ``teleop_accel_mps2``.

        Returns ``(velocity, acceleration)``: the ramped stick and the
        ramp's acceleration (the feedforward on the trajectory output; zeros
        without a ramp). See position_hold.ramp_velocity.
        """
        stick = self.teleop_stick(drone, now)
        velocity, accel, drone.teleop_profile = ramp_velocity(
            drone.teleop_profile, drone.published, stick, self.teleop_accel,
            self.control_dt)
        return velocity, accel

    def teleop_position_mode(self, drone: DroneHandle, now):
        """Position-mode teleop (see position_hold.py).

        The sticks are a velocity that moves ``drone.ref`` (advanced from
        what is published, leashed to ``teleop_lead_m``, clamped into a
        keep_in fence — all in ``advance_reference``). On the trajectory
        output PX4 holds the reference itself, so the sticks are pure
        feedforward; on the velocity output the commander adds the P term.
        Returns ``(velocity, acceleration)`` like ``teleop_command``.
        """
        stick, accel = self.teleop_command(drone, now)
        if drone.output == 'trajectory' or drone.ref is None:
            return stick, accel
        return tracking_velocity(drone.ref, drone.position, stick,
                                 self.teleop_kp, self.teleop_max_speed), accel

    def stop_point(self, drone: DroneHandle) -> np.ndarray:
        """Where a moving drone can come to rest with the hold law.

        The hold target after ``~/hold`` or a fence latch. A drone at speed
        cannot stop on the spot: pinning the target to the position at the
        instant of the call made it overshoot and then fly back (bag
        run_041842, ~1.5 m from 6 m/s). Braking at goal_accel_mps2 with the
        hold law's tail, it stops ``v^2/(2a) + v/hover_kp`` ahead; that is
        where it is held, clamped into the fence box when there is one.
        """
        v = np.asarray(drone.velocity, dtype=float)
        speed = float(np.linalg.norm(v))
        if speed < 0.2:
            return drone.position.copy()
        dist = stopping_distance(speed, self.scenario.tracker.accel,
                                 1.0 / max(self.hover_kp, 1e-3))
        point = drone.position + v / speed * dist
        if self.fence_enabled:
            point = clamp_to_box(point, self.fence_min, self.fence_max,
                                 self.fence_margin)
        return point

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
        # Only an ACTIVE drone is held inside its keep_in box: ASCEND and
        # LANDING cross the floor on purpose, and a reference clamped to a
        # fence floor above the ground is a position setpoint PX4's stiff
        # altitude loop holds — the drone hovers at the floor and never
        # touches down (teleop_fence_min z = 0.3, 2026-09-25).
        box = self.keep_in_box(drone)
        if box is not None and drone.state == FlightState.ACTIVE:
            drone.ref = clamp_to_box(drone.ref, box[0], box[1], self.fence_margin)
        if seeded:
            # Control changed hands: the stick ramp restarts from what is
            # published and the yaw hold re-adopts the measured heading.
            drone.teleop_profile = None
            drone.teleop_yaw_hold = None
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
                    nominal[i], accel_ff[i] = self.teleop_position_mode(d, now)
                elif self.mission_active and scenario_nominal is not None:
                    drone_index = self.drones.index(d)
                    nominal[i] = scenario_nominal[drone_index]
                    accel_ff[i] = scenario_accel[drone_index]
                    if drone_index in scenario_exempt:
                        exempt_rows.add(i)
                else:
                    # Braking law toward the hold target (stop_point): from
                    # speed it decelerates at goal_accel_mps2 and eases in
                    # with gain hover_kp, instead of the old P-law that
                    # pulled a fast drone back to where it was when hold
                    # was called (bag run_041842: a 1.5 m bounce).
                    nominal[i] = seek_velocity(
                        d.position[None], d.hold_target[None],
                        self.cbf_max_speed, self.scenario.tracker.accel,
                        1.0 / max(self.hover_kp, 1e-3))[0]
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
        self._cbf_active_names = cbf_names
        self._cbf_emergency = bool(result.used_emergency_stop)
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
                if all(o.state == FlightState.IDLE
                       for o in self.drones if o.commanded):
                    self.mission_ever_started = False
                self.get_logger().info(f'{d.name}: landed, disarmed')
                continue

            row = index[d.name]
            velocity = safe[row]
            # keep_in fence (geofence in keep_in mode, or the teleop fence):
            # the wall is the last word, after the CBF (a per-axis clip never
            # turns a stop into motion). Only ACTIVE drones: ASCEND/LANDING
            # legitimately cross the floor.
            fence_accel = np.zeros(3)
            if d.state == FlightState.ACTIVE:
                velocity, fence_accel = self.keep_in(d, velocity)
            # The acceleration feedforward belongs to the profile; once the
            # CBF or the fence has altered the velocity it would push toward
            # the very thing they steered away from, so it is dropped — and
            # replaced by the fence's own braking feedforward on the axes the
            # wall limited, so PX4 brakes with the command, not a lag later.
            accel = accel_ff[row]
            if np.linalg.norm(velocity - nominal[row]) > 1e-6:
                accel = fence_accel
            # The CBF overrode the profile, so the reference may be ahead of
            # the drone along a path the filter no longer endorses. PX4's
            # onboard pull toward the reference is not filtered; keep it on
            # the short hold leash until the drone is free. Only the CBF: a
            # fence clip is a per-axis speed cap that the reference already
            # integrates consistently, and leashing on it turned every
            # fence-limited cruise into a bare velocity setpoint (bag
            # run_035852: 2.5-3.3 m/s actual for 4.7-5.9 commanded).
            cbf_touched = (result.used_emergency_stop
                           or (row not in exempt_rows and bool(result.corrected[row])))
            if cbf_touched and d.ref is not None:
                lead, pulled = leash(d.ref - d.position, self.hold_lead)
                if pulled:
                    d.ref = d.position + lead
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
            # unless a goal says otherwise). A teleop drone yaws with the
            # stick: while it is deflected the rotation is ALL-ZERO — x, y,
            # z AND w, since a Quaternion message defaults to w = 1, which
            # px4_interface read as "hold ENU yaw 0" and dropped the yaw
            # rate (bag run_053740) — so the yaw rate below is what PX4
            # gets; the moment it is centred the measured heading is adopted
            # and held as an absolute yaw, as PX4's own Position mode does.
            if drone.role != 'teleop':
                yaw = self.desired_heading(drone)
                pose.rotation.z = float(np.sin(0.5 * yaw))
                pose.rotation.w = float(np.cos(0.5 * yaw))
            elif abs(yaw_rate) > 1e-3:
                drone.teleop_yaw_hold = None
                pose.rotation.w = 0.0
            else:
                if drone.teleop_yaw_hold is None:
                    drone.teleop_yaw_hold = yaw_of(drone.orientation)
                yaw = drone.teleop_yaw_hold
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
        drone.published = velocity.copy()

    # ------------------------------------------------------------------
    # Visualization (RViz MarkerArray, world frame)
    # ------------------------------------------------------------------

    def desired_heading(self, drone: DroneHandle) -> float:
        """ENU yaw (rad) a scenario drone should hold right now."""
        return float(self.scenario.headings[self.drones.index(drone)])

    def _drone_color(self, drone: DroneHandle):
        """Body colour = CBF planner status, with role/safety overrides on top.

        The overrides come first on purpose: an 'external' drone is tracked but
        never commanded by the planner, so painting it a planner state would
        claim something untrue. Mode (sim/real) is deliberately NOT encoded —
        it is in the basestation panel's Mode column.
        """
        if self.fence_breached:
            return (1.0, 0.3, 0.0)                 # orange = frozen on breach
        if drone.role == 'teleop':
            return (1.0, 0.85, 0.1)                # yellow = operator obstacle
        if drone.role == 'external':
            return (0.6, 0.6, 0.6)                 # gray = tracked, uncommanded
        if drone.state == FlightState.LANDING:
            return (0.45, 0.45, 0.45)              # dim gray = descending
        if self.mission_active:
            return (0.2, 0.7, 1.0)                 # blue  = planner running
        if self.mission_ever_started:
            return (0.9, 0.2, 0.2)                 # red   = planner stopped
        return (0.2, 0.85, 0.35)                   # green = planner not launched

    def publish_markers(self, now):
        if self.viz_pub is None:
            return
        arr = MarkerArray()
        stamp = now.to_msg()
        # Stored goals are drawn always (dim before start), so a stale or
        # freshly sent goal can be checked before the drone is released.
        goals = getattr(self.scenario, 'goals', None)

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
            body.type = Marker.MESH_RESOURCE
            body.action = Marker.ADD
            body.mesh_resource = DRONE_MESH
            # The OBJ ships a flat-grey .mtl that Foxglove does not fetch, so the
            # marker colour is what actually tints the mesh — leave embedded
            # materials off or the status colour would be ignored.
            body.mesh_use_embedded_materials = False
            body.pose.position.x = float(d.position[0])
            body.pose.position.y = float(d.position[1])
            body.pose.position.z = float(d.position[2])
            qx, qy, qz, qw = _quat_mul(d.orientation, AXIS_CORRECTION)
            body.pose.orientation.x = qx
            body.pose.orientation.y = qy
            body.pose.orientation.z = qz
            body.pose.orientation.w = qw
            body.scale.x = body.scale.y = body.scale.z = 1.0
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
            label.scale.z = 0.22
            label.color = LABEL_COLOR
            # Name only: mode and role are in the basestation panel's Agents /
            # Wiring / Agent State views, and a shorter label stays legible.
            label.text = d.name
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
                goal.color = ColorRGBA(r=r, g=g, b=b,
                                       a=0.6 if self.mission_active else 0.25)
                arr.markers.append(goal)

        if self.fence_enabled:
            arr.markers.append(self._fence_marker(stamp))
            grid = self._fence_grid_marker(stamp)
            if grid is not None:
                arr.markers.append(grid)
        if self.teleop_fence_enabled:
            arr.markers.append(self._teleop_fence_marker(stamp))

        self.viz_pub.publish(arr)

    def _fence_marker(self, stamp):
        breached = self.fence_breached
        color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9) if breached \
            else ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.5)
        return self._box_marker(stamp, self.fence_min, self.fence_max,
                                'fence', 9000, color)

    def _teleop_fence_marker(self, stamp):
        """The teleop fence box: amber, thinner than the geofence."""
        return self._box_marker(stamp, self.teleop_fence_min, self.teleop_fence_max,
                                'teleop_fence', 9002,
                                ColorRGBA(r=1.0, g=0.75, b=0.2, a=0.6), width=0.02)

    def _box_marker(self, stamp, lo, hi, ns: str, marker_id: int, color,
                    width: float = 0.03):
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
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color = color
        for a, c in edges:
            for idx in (a, c):
                m.points.append(Point(x=float(corners[idx][0]),
                                      y=float(corners[idx][1]),
                                      z=float(corners[idx][2])))
        return m

    # Grid lines are only ever drawn for a footprint this many cells across;
    # a huge default fence (±1000 m) with a 0.5 m cell would be 8000 lines.
    FENCE_GRID_MAX_LINES = 400

    @staticmethod
    def _grid_ticks(lo: float, hi: float, cell: float) -> list:
        """World-aligned tick positions in [lo, hi]: multiples of ``cell``."""
        first = math.ceil(lo / cell - 1e-9)
        last = math.floor(hi / cell + 1e-9)
        return [round(k * cell, 6) for k in range(first, last + 1)]

    def _fence_grid_marker(self, stamp):
        """Ground grid on the fence floor, clipped to the fence footprint.

        Lines sit on world multiples of ``fence_grid_cell_m`` (not on the
        fence corner), so x=0 / y=0 are on the grid and a drone's position can
        be read off it directly; whole-metre lines are drawn brighter. Returns
        None when the grid is disabled or the fence is too large to grid.
        """
        cell = self.fence_grid_cell
        if not (cell > 0.0):
            return None
        lo, hi = self.fence_min, self.fence_max
        xs = self._grid_ticks(float(lo[0]), float(hi[0]), cell)
        ys = self._grid_ticks(float(lo[1]), float(hi[1]), cell)
        if not xs or not ys or len(xs) + len(ys) > self.FENCE_GRID_MAX_LINES:
            return None
        z = float(lo[2])
        minor = ColorRGBA(r=0.55, g=0.7, b=1.0, a=0.22)
        major = ColorRGBA(r=0.55, g=0.7, b=1.0, a=0.55)

        m = Marker()
        m.header.frame_id = self.viz_frame
        m.header.stamp = stamp
        m.ns = 'fence_grid'
        m.id = 9001
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.012

        def is_major(v: float) -> bool:
            return abs(v - round(v)) < 1e-6

        for x in xs:
            c = major if is_major(x) else minor
            m.points.append(Point(x=x, y=float(lo[1]), z=z))
            m.points.append(Point(x=x, y=float(hi[1]), z=z))
            m.colors.extend([c, c])
        for y in ys:
            c = major if is_major(y) else minor
            m.points.append(Point(x=float(lo[0]), y=y, z=z))
            m.points.append(Point(x=float(hi[0]), y=y, z=z))
            m.colors.extend([c, c])
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
