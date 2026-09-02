"""Pure logic for the native MIGHTY -> PX4 setpoint path ("Option A").

This module imports NOTHING from ROS so every rule below is testable on a
bare host (``test/test_native_setpoint.py``). The ROS node that uses it is
``native_setpoint_node.py``.

WHAT THIS PATH IS
-----------------
``mighty_node`` already publishes ``dynus_interfaces/msg/Goal`` at 1/dc =
100 Hz on ``goal`` (``mighty_node.cpp::publishGoal``, timer period ``par_.dc``
= 0.01 s in ``mighty_disaster.yaml``). Nothing consumed it. Today MIGHTY
reaches PX4 through a lossy chain instead:

    mighty Trajectory -> mighty_bridge (stride + 1 Hz throttle)
      -> trajectory_override -> trajectory_controller -> tracking_point
      -> pid_controller (position/velocity cascade)
      -> RollPitchYawrateThrust -> mavros_interface
      -> mavros/setpoint_raw/attitude -> PX4

The native path is what MIT-ACL actually fly
(``jrached/ros2_px4_stack``'s ``dynus_offboard_node.py``): take the Goal
stream as-is and hand PX4 a full position+velocity+acceleration+yaw setpoint
at the Goal rate. We publish ``mavros_msgs/PositionTarget`` on
``mavros/setpoint_raw/local`` (equivalent to, and simpler than, their
MultiDOFJointTrajectory on ``setpoint_trajectory``).

FRAMES (verified in this tree, 2026-09-02)
------------------------------------------
* ``mighty_node`` stamps every Goal with ``par_.map_frame_id`` — ``map`` for
  every AirStack config (``mighty_node.cpp:535`` declares the default and
  neither ``mighty_airstack.yaml`` nor ``mighty_disaster.yaml`` overrides it).
  Goal p/v/a are therefore in AirStack's ``map``, ENU.
* AirStack's ``map`` IS the MAVROS local frame. ``interface.launch.py`` runs
  ``odometry_conversion`` with ``odometry_output_type: 2`` (= OVERWRITE) on
  ``/<robot>/interface/mavros/local_position/odom``: OVERWRITE only rewrites
  ``header.frame_id`` to ``map`` and ``child_frame_id`` to ``base_link``
  (``odometry_conversion.cpp``, the OVERWRITE branch) — it applies NO
  transform. So map == mavros local_position ENU, origin at the PX4 EKF
  origin. (The separate ``world`` frame the map_anchor node relates to
  ``map`` by the takeoff offset is NOT involved here; MIGHTY plans in
  ``map``.)
* ``mavros_msgs/PositionTarget`` with ``coordinate_frame =
  FRAME_LOCAL_NED``: the MAVROS ``setpoint_raw`` plugin applies its own
  ENU->NED transform for that frame constant, so the values we put in the
  message must be ENU, i.e. the Goal's numbers verbatim.

=> the Goal -> PositionTarget mapping is an IDENTITY on the coordinates.
   There is no rotation and no offset anywhere in ``goal_to_setpoint``.
   (Contrast ``mavros_interface.cpp::velocity_callback``, which uses
   ``FRAME_BODY_NED`` — that one IS body-relative. Ours must not be.)

YAW
---
``Goal.yaw`` is the DYNUS spline yaw (Sec. III of arXiv 2103.06372), NOT the
body rXYZ yaw, and in this tree it was measured to be 0 for a whole flight —
``bridge_node._traj_cb`` documents the drone flying sideways because of it
and overrides it with the direction of travel. The native path inherits that
finding: ``yaw_source='velocity'`` (the default) reproduces the convention
the vehicle has actually flown; ``yaw_source='goal'`` passes ``Goal.yaw``
through for whoever fixes MIGHTY's yaw output.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# ── mavros_msgs/msg/PositionTarget constants ────────────────────────────────
# Mirrored here so the mapping is testable with no ROS on the box. The node
# asserts these against the real message class at construction
# (assert_position_target_constants, called from native_setpoint_node) so a
# version bump can never silently desync them.
FRAME_LOCAL_NED = 1
FRAME_LOCAL_OFFSET_NED = 7
FRAME_BODY_NED = 8
FRAME_BODY_OFFSET_NED = 9

IGNORE_PX = 1
IGNORE_PY = 2
IGNORE_PZ = 4
IGNORE_VX = 8
IGNORE_VY = 16
IGNORE_VZ = 32
IGNORE_AFX = 64
IGNORE_AFY = 128
IGNORE_AFZ = 256
FORCE = 512
IGNORE_YAW = 1024
IGNORE_YAW_RATE = 2048

#: position + velocity + acceleration + yaw active, yaw_rate ignored.
#: This is the mask the streamer sends by default. Everything the FCU is
#: allowed to use is UNMASKED; PX4 treats v and a as feedforward on top of
#: the position setpoint, which is the whole point of the native path.
#: FORCE must stay 0 — with it set PX4 reads afx/afy/afz as a force, not an
#: acceleration.
TYPE_MASK_PVA_YAW = IGNORE_YAW_RATE

#: ... and the same with yaw_rate additionally active (opt-in; PX4's
#: combined yaw + yaw_rate handling is version-dependent, hence not default).
TYPE_MASK_PVA_YAW_RATE = 0

YAW_SOURCE_VELOCITY = 'velocity'
YAW_SOURCE_GOAL = 'goal'
YAW_SOURCES = (YAW_SOURCE_VELOCITY, YAW_SOURCE_GOAL)

Vec3 = Tuple[float, float, float]

#: every constant this module mirrors, checked against the real message class
_MIRRORED_CONSTANTS = (
    'FRAME_LOCAL_NED', 'FRAME_LOCAL_OFFSET_NED', 'FRAME_BODY_NED',
    'FRAME_BODY_OFFSET_NED', 'IGNORE_PX', 'IGNORE_PY', 'IGNORE_PZ',
    'IGNORE_VX', 'IGNORE_VY', 'IGNORE_VZ', 'IGNORE_AFX', 'IGNORE_AFY',
    'IGNORE_AFZ', 'FORCE', 'IGNORE_YAW', 'IGNORE_YAW_RATE',
)


def assert_position_target_constants(msg_cls) -> None:
    """Fail loudly if the mirrored constants disagree with mavros_msgs.

    Called once at node construction. Without it, a MAVROS release that
    renumbered a bit would turn into a wrong ``type_mask`` on the wire — the
    FCU would silently ignore, say, the acceleration, and the only symptom
    would be worse tracking.
    """
    bad = []
    for name in _MIRRORED_CONSTANTS:
        mine = globals()[name]
        theirs = getattr(msg_cls, name, None)
        if theirs is None:
            bad.append(f'{name}: missing from {msg_cls.__name__}')
        elif int(theirs) != int(mine):
            bad.append(f'{name}: mirrored {mine}, actual {theirs}')
    if bad:
        raise RuntimeError(
            'native_setpoint.py mirrors mavros_msgs/PositionTarget constants '
            'that no longer match the installed message definition: '
            + '; '.join(bad))


def env_flag(value: Optional[str], default: bool = False) -> bool:
    """Parse an env-var flag. Unset OR EMPTY falls back to `default`.

    Empty-is-unset matters: compose passes these bare-name (see
    robot-base-docker-compose.yaml), and an env var that exists with an empty
    value must not beat the node-side default.
    """
    if value is None:
        return default
    text = value.strip().lower()
    if not text:
        return default
    return text in ('1', 'true', 'yes', 'on', 'y', 't')


def env_float(value: Optional[str], default: float) -> float:
    """Parse a numeric env knob. Unset, empty or unparseable -> `default`.

    Never raises: these are read at node construction, and a typo in an env
    var must not stop the planner module from coming up.
    """
    if value is None:
        return default
    try:
        return float(value.strip())
    except (AttributeError, TypeError, ValueError):
        return default


def frame_is_native(frame_id: Optional[str], world_frame: str) -> bool:
    """True when a Goal's stamped frame needs NO conversion to reach MAVROS.

    The Goal -> PositionTarget mapping is an identity only because MIGHTY's
    map frame IS the MAVROS local ENU frame (module docstring). That argument
    holds for ``map`` and for an unstamped message; for anything else the
    numbers mean something different and must not be flown as if they did.
    Refusing is the right answer here rather than transforming: a positional
    setpoint stream is not the place to discover a missing TF at 50 Hz.
    """
    return (frame_id or '').strip() in ('', world_frame)


# ── yaw ─────────────────────────────────────────────────────────────────────

def heading_from_velocity(vx: float, vy: float, prev_yaw: float,
                          min_speed: float = 0.3) -> float:
    """Yaw the body should hold: the direction of horizontal travel, or the
    previous heading when moving too slowly for the direction to mean much.

    Single source of truth for both command paths — ``bridge_node`` imports
    this exact function for the trajectory_override path, so the native path
    and the legacy path point the nose the same way.
    """
    if math.hypot(vx, vy) < min_speed:
        return prev_yaw
    return math.atan2(vy, vx)


# ── Goal -> PositionTarget ──────────────────────────────────────────────────

@dataclass(frozen=True)
class SetpointFields:
    """Everything the streamer writes into a ``mavros_msgs/PositionTarget``.

    A plain value object so the mapping can be asserted field by field with
    no ROS message classes in the loop.
    """
    coordinate_frame: int
    type_mask: int
    position: Vec3
    velocity: Vec3
    acceleration: Vec3
    yaw: float
    yaw_rate: float


def goal_vectors(goal) -> Tuple[Vec3, Vec3, Vec3, float, float]:
    """Pull ``(p, v, a, yaw, dyaw)`` out of a ``dynus_interfaces/msg/Goal``.

    Duck-typed on purpose: anything with ``.p/.v/.a`` Vector3-likes and
    ``.yaw``/``.dyaw`` floats works, which is what makes it testable. The
    Goal's ``j`` (jerk) is deliberately dropped — SET_POSITION_TARGET_LOCAL_NED
    has no jerk field.
    """
    return (
        (float(goal.p.x), float(goal.p.y), float(goal.p.z)),
        (float(goal.v.x), float(goal.v.y), float(goal.v.z)),
        (float(goal.a.x), float(goal.a.y), float(goal.a.z)),
        float(goal.yaw),
        float(goal.dyaw),
    )


def goal_to_setpoint(position: Vec3,
                     velocity: Vec3,
                     acceleration: Vec3,
                     goal_yaw: float,
                     goal_dyaw: float,
                     *,
                     yaw_source: str = YAW_SOURCE_VELOCITY,
                     prev_yaw: float = 0.0,
                     yaw_min_speed: float = 0.3,
                     send_yaw_rate: bool = False) -> SetpointFields:
    """Map one MIGHTY Goal onto the PositionTarget fields.

    IDENTITY on the coordinates: MIGHTY's map frame is MAVROS's local ENU
    frame and MAVROS does the ENU->NED conversion itself for
    ``FRAME_LOCAL_NED`` (see the module docstring). Any rotation here would
    be a double conversion.
    """
    if yaw_source not in YAW_SOURCES:
        raise ValueError(f'yaw_source must be one of {YAW_SOURCES}, '
                         f'got {yaw_source!r}')
    if yaw_source == YAW_SOURCE_GOAL:
        yaw = float(goal_yaw)
    else:
        yaw = heading_from_velocity(velocity[0], velocity[1], float(prev_yaw),
                                    min_speed=yaw_min_speed)
    return SetpointFields(
        coordinate_frame=FRAME_LOCAL_NED,
        type_mask=(TYPE_MASK_PVA_YAW_RATE if send_yaw_rate
                   else TYPE_MASK_PVA_YAW),
        position=(float(position[0]), float(position[1]), float(position[2])),
        velocity=(float(velocity[0]), float(velocity[1]), float(velocity[2])),
        acceleration=(float(acceleration[0]), float(acceleration[1]),
                      float(acceleration[2])),
        yaw=float(yaw),
        yaw_rate=float(goal_dyaw) if send_yaw_rate else 0.0,
    )


def hold_setpoint(position: Vec3, yaw: float,
                  *, send_yaw_rate: bool = False) -> SetpointFields:
    """A position-hold setpoint: stay HERE, zero velocity, zero acceleration.

    Used in two places, both of them about never leaving PX4 without a
    setpoint (its OFFBOARD failsafe trips after 500 ms of silence):

    * MIGHTY's Goal stream went stale while we are still engaged — freeze at
      the last commanded point rather than replaying a stale velocity;
    * the handback overlap — see ``HandoffStateMachine``.
    """
    return SetpointFields(
        coordinate_frame=FRAME_LOCAL_NED,
        type_mask=(TYPE_MASK_PVA_YAW_RATE if send_yaw_rate
                   else TYPE_MASK_PVA_YAW),
        position=(float(position[0]), float(position[1]), float(position[2])),
        velocity=(0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
        yaw=float(yaw),
        yaw_rate=0.0,
    )


# ── engagement / handoff state machine ──────────────────────────────────────
#
# THE CONSTRAINT: two command streams to PX4 at once is a fault. PX4 acts on
# whichever setpoint arrived last and its `offboard_control_mode` flags flip
# with the setpoint TYPE, so interleaving attitude targets (the pid path) and
# position targets (this path) makes the flight task switch at the stream
# rate. Exactly one path may be live at any moment, except during the
# deliberate handback overlap below where both command the SAME hold.
#
# THE MUZZLE: `pid_controller`'s new `command_muted` dynamic parameter, set
# over its `set_parameters` service. Justification for that choice over the
# alternatives, smallest blast radius first:
#   * pid_controller is a LEAF: the only publisher of
#     interface/cmd_roll_pitch_yawrate_thrust, and it publishes from exactly
#     one place (`tracking_point_callback`). One `if` guards the whole path,
#     and it is inert (`false`) for every stack that does not opt in.
#   * a trajectory_controller MODE is not a muzzle: `tracking_point_pub->
#     publish(...)` is unconditional in its timer for PAUSE / ROBOT_POSE /
#     TRACK alike (trajectory_controller.cpp:470), so the pid keeps emitting
#     attitude in all of them.
#   * gating inside mavros_interface / robot_interface would put a new
#     branch in the SAFETY BOUNDARY that arming, takeoff and land all route
#     through, and in a class shared with px4_interface. Bigger blast radius
#     for the same effect.
#
# THE STATES:
#   IDLE      nothing streamed, pid owns the vehicle (today's behaviour).
#   MUTING    a `command_muted := true` request is in flight. Still IDLE as
#             far as PX4 is concerned: the pid is still commanding, so there
#             is no gap. We enter STREAMING only on a CONFIRMED mute — if
#             the service is missing or refuses we stay on the legacy path
#             rather than risk two masters.
#   STREAMING we own the vehicle; PositionTarget at stream_rate_hz.
#   GRACE     the engagement signal went inactive, but we KEEP the vehicle,
#             holding, for disengage_grace_s. See below.
#   HANDBACK  ordered return of control (see `_handback`).
#
# WHY GRACE EXISTS (measured 2026-09-02, first raven-search flight with
# MIGHTY_NATIVE_SETPOINTS=1): the handoff above is correct per goal, but
# raven's search issues a NEW NavigateTask every few seconds. Each completion
# disengages mighty_bridge and each new task re-engages it, so a bare
# disengage->handback rule produced a continuous
# engage -> stream -> full handback -> pid -> engage cycle: back-to-back
# MUTED/UNMUTED lines in the pid log and flight as jerky as the legacy path,
# because the vehicle spent most of its time being handed between two
# controllers instead of being flown by either. The demo looked clean only
# because its three goals were long-lived.
#
# So a disengage is treated as PROVISIONAL. We hold — still muted, still
# streaming, still feeding the deadman — and only if the quiet lasts
# disengage_grace_s do we run the ordered handback. Under goal churn the next
# task re-engages inside the window and the stream never breaks: zero service
# calls, zero mode changes, no un-mute.
#
# NOTHING ON THE BRIDGE SIDE NEEDS TO CHANGE for this, which is worth
# recording because it is not obvious: we now stream straight across
# mighty_bridge's `_finish` -> `_set_mode(TRACK)` call. Audited 2026-09-02 —
# the bridge's only mode calls are TRACK on engage (`_follow_route`,
# `_execute_navigate`) and TRACK on task finish (`_finish`), and all three land
# while the pid is MUTED, so they move the trajectory controller's own timeline
# and nothing else; no attitude reaches PX4 from any of them. The bridge's
# other `_route_active` reads (the trajectory start guard and its carried yaw)
# likewise feed only the trajectory_override path, which is inert behind the
# mute. The grace also REMOVES the ROBOT_POSE-vs-TRACK race that
# `handback_pin_s` guards: by the time a grace expires, the bridge's TRACK is
# seconds old.
#
# Two disengage reasons deliberately SKIP the grace, because for them waiting
# is not conservative, it is a hazard:
#   * the master switch going off — an operator asking for the legacy path
#     back should get it now;
#   * MIGHTY's goals going stale past hold_max_s — the planner is gone, and
#     land / RTL / fixed_trajectory are inert behind our mute until we release
#     it.

STATE_IDLE = 'idle'
STATE_MUTING = 'muting'
STATE_STREAMING = 'streaming'
STATE_GRACE = 'grace'
STATE_HANDBACK = 'handback'

STREAM_NONE = None
STREAM_GOAL = 'goal'
STREAM_HOLD = 'hold'


@dataclass(frozen=True)
class TickAction:
    """What the node must do this tick. Every field is an instruction, not a
    fact: the node performs them and reports the outcome back through
    ``note_mute_result`` / ``note_pid_command``."""
    state: str
    #: STREAM_NONE / STREAM_GOAL / STREAM_HOLD
    stream: Optional[str]
    #: issue a ``command_muted := <value>`` set_parameters call, or None
    request_mute: Optional[bool]
    #: ask the trajectory controller for ROBOT_POSE (pin the tracking point
    #: to the vehicle) before the pid resumes
    request_robot_pose: bool = False
    #: human-readable reason for a state change this tick ('' = no change)
    note: str = ''


@dataclass(frozen=True)
class HandoffConfig:
    #: master switch (env MIGHTY_NATIVE_SETPOINTS)
    enabled: bool = False
    #: engagement heartbeat older than this counts as DISENGAGED. The bridge
    #: republishes at 1 Hz, so this also covers "the bridge died".
    engage_timeout_s: float = 3.0
    #: how long a disengage is treated as PROVISIONAL — we keep the vehicle,
    #: holding and still muted, waiting for the next task to engage. Sized for
    #: raven's search goal churn (a new NavigateTask every few seconds); see
    #: the STATE_GRACE commentary above. 0 disables the grace and restores the
    #: hand-back-immediately behaviour. Note this rides ON TOP of
    #: engage_timeout_s: a silent bridge costs engage_timeout_s + this before
    #: control comes back, so keep the sum comfortably under any mission-level
    #: watchdog.
    disengage_grace_s: float = 4.0
    #: During the GRACE window, FOLLOW mighty's live Goals instead of freezing
    #: on the last commanded setpoint. mighty_node never stops publishing, so
    #: under raven's goal churn this removes even the sub-second decel/accel
    #: pulse a hold costs at every task swap — the spline flows straight
    #: through the gap (user 2026-09-02 night). The cost: a route the task
    #: layer genuinely finished keeps being tracked for at most
    #: disengage_grace_s before the handback; the stale-goal (hold_max_s) and
    #: master-switch hazards still bypass the grace entirely. Default False =
    #: the conservative hold; our missions turn it on via
    #: MIGHTY_NATIVE_GRACE_FOLLOW.
    grace_follow_goals: bool = False
    #: a Goal older than this stops being flown; we hold instead.
    goal_stale_s: float = 0.5
    #: ... and after this long holding on stale Goals we hand back, so that
    #: land / RTL / fixed_trajectory (all of which go through the muted pid)
    #: are not left inert if MIGHTY dies mid-route.
    hold_max_s: float = 5.0
    #: retry cadence for a mute request that failed or was refused
    mute_retry_s: float = 1.0
    #: refresh cadence for the mute watchdog in pid_controller
    mute_refresh_s: float = 0.2
    #: handback ends once this many pid commands have been observed ...
    handback_confirm_msgs: int = 3
    #: ... or after this long, whichever comes first.
    handback_timeout_s: float = 1.0
    #: longest the handback waits for the trajectory controller to acknowledge
    #: ROBOT_POSE before un-muting anyway. Bounds a race that is otherwise won
    #: only by luck: mighty_bridge asks the SAME service for TRACK at the
    #: instant it disengages, and TRACK with a cleared trajectory freezes the
    #: tracking point at the last MIGHTY waypoint — which is exactly the stale
    #: target the pid must not wake up onto.
    handback_pin_s: float = 0.3


class HandoffStateMachine:
    """Decides, tick by tick, who is allowed to command PX4.

    Deliberately free of ROS and of wall-clock reads: ``tick()`` takes the
    time and the observations, so the whole ordering contract is unit
    testable.
    """

    def __init__(self, config: Optional[HandoffConfig] = None):
        self.cfg = config or HandoffConfig()
        self.state = STATE_IDLE
        self._state_since = 0.0
        self._mute_pending = False
        self._last_mute_request = float('-inf')
        self._last_mute_refresh = float('-inf')
        self._hold_since: Optional[float] = None
        self._pid_cmds = 0
        self._handback_step = 0
        self._robot_pose_acked = False
        self._mute_lost = False
        #: pid commands seen while WE were streaming — every one of them is a
        #: two-masters fault (the mute did not take). The node logs it.
        self.pid_commands_while_streaming = 0

    # -- observations the node feeds back -----------------------------------

    def note_mute_result(self, now: float, requested: bool, ok: bool) -> None:
        """Outcome of a ``command_muted := requested`` service call."""
        self._mute_pending = False
        if requested and self.state == STATE_MUTING:
            if ok:
                self._to(now, STATE_STREAMING, 'mute confirmed')
            # not ok: stay in MUTING; tick() retries after mute_retry_s and
            # nothing is streamed in the meantime, so the pid keeps flying.
        elif (requested and not ok
                and self.state in (STATE_STREAMING, STATE_GRACE)):
            # A refused deadman REFRESH means the mute is about to lapse under
            # us: pid_controller will un-mute itself within
            # command_mute_timeout and start commanding while we are still
            # streaming. Hand back deliberately instead of arriving there.
            # GRACE counts: we still own the vehicle there.
            self._mute_lost = True
        elif not requested and not ok and self.state == STATE_HANDBACK:
            # A refused UN-mute is the dangerous one: the pid stays silent
            # while we are winding the stream down. Re-arm the handback's
            # un-mute step so the next tick asks again. (pid_controller's
            # deadman is the backstop if it keeps refusing.)
            self._handback_step = 1

    def note_robot_pose_result(self, _ok: bool = True) -> None:
        """The trajectory controller answered our ROBOT_POSE request (or the
        node decided it never will — a missing service, or the request
        disabled). Either way the handback may proceed to the un-mute: an
        answer is all we needed, because it means our request was ordered
        AFTER mighty_bridge's simultaneous TRACK rather than before it.
        """
        self._robot_pose_acked = True

    def note_pid_command(self) -> None:
        """One command observed on the pid's output topic."""
        if self.state in (STATE_STREAMING, STATE_GRACE):
            # We own the vehicle in both — the pid must be silent.
            self.pid_commands_while_streaming += 1
        elif self.state == STATE_HANDBACK:
            self._pid_cmds += 1

    # -- the tick -----------------------------------------------------------

    def tick(self, now: float, *, engagement_age: Optional[float],
             engaged: bool, goal_age: Optional[float]) -> TickAction:
        """
        ``engagement_age`` — seconds since the last engagement message from
        mighty_bridge (None = never heard one). ``engaged`` — the value that
        message carried. ``goal_age`` — seconds since the last MIGHTY Goal
        (None = never).

        Ages are clamped at 0 so a sim-time jump backwards cannot fake
        freshness in the wrong direction.
        """
        cfg = self.cfg
        engagement_age = _clamp_age(engagement_age)
        goal_age = _clamp_age(goal_age)
        fresh_engage = (engaged and engagement_age is not None
                        and engagement_age <= cfg.engage_timeout_s)
        fresh_goal = goal_age is not None and goal_age <= cfg.goal_stale_s

        # Disabled is exactly "permanently disengaged", not "return early":
        # if the switch goes off mid-flight we still owe the ordered handback
        # below, and from IDLE it collapses to doing nothing at all — which
        # is what makes MIGHTY_NATIVE_SETPOINTS=0 bit-identical to today.
        disengage_note = None
        if not cfg.enabled:
            fresh_engage = False
            disengage_note = 'native setpoints disabled'

        if self.state == STATE_IDLE:
            if fresh_engage and fresh_goal:
                # Mute BEFORE the first PositionTarget, never after. A few
                # tens of ms with no new setpoint is nothing to PX4 (it holds
                # the last one and its OFFBOARD failsafe is 500 ms), whereas
                # overlapping the two streams flips its control mode at the
                # stream rate. Requiring a FRESH Goal first is what stops us
                # muting into silence when MIGHTY is not actually producing.
                self._to(now, STATE_MUTING, 'engaged with a fresh MIGHTY goal')
                self._mute_pending = True
                self._last_mute_request = now
                return TickAction(self.state, STREAM_NONE, True,
                                  note='engaged with a fresh MIGHTY goal')
            return TickAction(self.state, STREAM_NONE, None)

        if self.state == STATE_MUTING:
            if not fresh_engage:
                return self._abandon_mute(
                    now, disengage_note or 'disengaged before the mute landed')
            if self._mute_pending:
                return TickAction(self.state, STREAM_NONE, None)
            if now - self._last_mute_request >= cfg.mute_retry_s:
                self._mute_pending = True
                self._last_mute_request = now
                return TickAction(self.state, STREAM_NONE, True,
                                  note='retrying the pid mute')
            return TickAction(self.state, STREAM_NONE, None)

        # GRACE is evaluated BEFORE streaming and falls through into it on a
        # re-engage, so the very tick that sees the new task already streams
        # its fresh Goal. Returning here and picking the goal up next tick
        # would put a hold in the middle of an otherwise continuous stream —
        # small, but it is exactly the stutter this state exists to remove.
        carry_note = ''
        if self.state == STATE_GRACE:
            if disengage_note is not None:
                return self._handback(now, disengage_note)
            if fresh_engage:
                self._to(now, STATE_STREAMING, 're-engaged within the grace '
                                               'window (no handback)')
                carry_note = 're-engaged within the grace window (no handback)'
            elif self._mute_lost:
                return self._handback(now, 'pid refused the mute refresh')
            elif now - self._state_since >= cfg.disengage_grace_s:
                return self._handback(now, 'disengage grace expired')
            else:
                # Still ours: keep feeding the deadman either way. Dropping
                # the refresh here would let pid_controller un-mute itself
                # mid-grace and start commanding underneath us.
                if cfg.grace_follow_goals and fresh_goal:
                    # Follow the live spline straight through the gap — no
                    # decel pulse per task swap (see HandoffConfig).
                    return TickAction(self.state, STREAM_GOAL,
                                      self._mute_refresh(now),
                                      note='grace: following live goals')
                return TickAction(self.state, STREAM_HOLD,
                                  self._mute_refresh(now))

        if self.state == STATE_STREAMING:
            if not fresh_engage:
                if disengage_note is not None or cfg.disengage_grace_s <= 0.0:
                    return self._handback(
                        now, disengage_note or 'follower disengaged')
                # Provisional disengage: keep the vehicle and wait for the
                # next task rather than paying a full handoff per goal.
                self._to(now, STATE_GRACE, 'disengaged — holding for '
                         f'{cfg.disengage_grace_s:.1f} s before handing back')
                return TickAction(
                    self.state, STREAM_HOLD, self._mute_refresh(now),
                    note='disengaged — holding for '
                         f'{cfg.disengage_grace_s:.1f} s before handing back')
            if self._mute_lost:
                return self._handback(now, 'pid refused the mute refresh')
            if fresh_goal:
                self._hold_since = None
                return TickAction(self.state, STREAM_GOAL,
                                  self._mute_refresh(now), note=carry_note)
            # Goals went stale. Hold the last commanded point: that keeps
            # OFFBOARD alive without replaying a stale velocity.
            if self._hold_since is None:
                self._hold_since = now
            if now - self._hold_since >= cfg.hold_max_s:
                # MIGHTY has been silent too long. Give the vehicle back so
                # the tasks that go through the pid (land, RTL, fixed
                # trajectory) are not inert behind our mute.
                return self._handback(now, 'MIGHTY goals stale')
            return TickAction(self.state, STREAM_HOLD,
                              self._mute_refresh(now), note=carry_note)

        # STATE_HANDBACK
        if fresh_engage and fresh_goal:
            # re-engaged mid-handback: re-mute from scratch
            self._to(now, STATE_MUTING, 're-engaged during handback')
            self._mute_pending = True
            self._last_mute_request = now
            return TickAction(self.state, STREAM_HOLD, True,
                              note='re-engaged during handback')
        # Never finish before the un-mute has actually been issued (step 1),
        # however small handback_timeout_s is set: stopping the stream while
        # the pid is still muted is the one outcome that leaves PX4 with no
        # commander at all.
        if self._handback_step >= 2:
            confirmed = self._pid_cmds >= cfg.handback_confirm_msgs
            if confirmed or now - self._state_since >= cfg.handback_timeout_s:
                why = 'pid commanding again' if confirmed else 'handback timeout'
                self._to(now, STATE_IDLE, why)
                return TickAction(self.state, STREAM_NONE, None, note=why)
        return self._handback(now, '')

    # -- internals ----------------------------------------------------------

    def _mute_refresh(self, now: float) -> Optional[bool]:
        """Re-assert ``command_muted := true`` on a cadence.

        pid_controller's mute is watchdogged: it un-mutes itself if the flag
        is not refreshed within ``command_mute_timeout``. That is the only
        thing standing between "this node crashes mid-flight" and "the pid
        is muted forever and PX4 loses its setpoint stream". Refreshing is
        therefore not optional bookkeeping — it is the deadman.
        """
        if now - self._last_mute_refresh >= self.cfg.mute_refresh_s:
            self._last_mute_refresh = now
            return True
        return None

    def _abandon_mute(self, now: float, why: str) -> TickAction:
        """Leave MUTING without ever having streamed.

        Always ASKS FOR THE UNMUTE on the way out even though we may never
        have been muted: the request that took us into MUTING is async and
        may still land after we have given up on it. Without this, a race
        between "disengaged" and "mute confirmed" leaves the pid muted with
        nothing streaming — recoverable only by pid_controller's own
        watchdog, and only after ``command_mute_timeout`` of silence.
        """
        self._to(now, STATE_IDLE, why)
        return TickAction(self.state, STREAM_NONE, False, note=why)

    def _handback(self, now: float, why: str) -> TickAction:
        """The ordered handback, spread over consecutive ticks.

        ORDERING, and why it is the reverse of the engage ordering:

          step 0  switch our own stream to HOLD (the vehicle's current
                  position, zero velocity/acceleration) and ask the
                  trajectory controller for ROBOT_POSE, which pins its
                  tracking point to the vehicle with zero velocity
                  (trajectory_controller.cpp:398) — so when the pid wakes up
                  its target is "stay here", not a stale point on a MIGHTY
                  timeline it never flew;
          step 1  once that request has been ACKNOWLEDGED (or handback_pin_s
                  has passed), un-mute the pid. Waiting for the ack, not just
                  for the next tick, is what settles the race with
                  mighty_bridge's own TRACK request on the same service —
                  TRACK clears the trajectory, and a cleared trajectory in
                  TRACK freezes the tracking point at the last MIGHTY
                  waypoint instead of at the vehicle;
          step 2+ KEEP streaming HOLD until we have actually SEEN the pid
                  publishing again — or handback_timeout_s elapses — then
                  stop.

        The last step is a deliberate OVERLAP, the opposite of the gap we
        accept on engage, because the failure it guards against is worse and
        silent: pid_controller returns early without publishing whenever a
        TF lookup fails ("failed to transform tracking point"), so "un-muted"
        does not imply "commanding". If we stopped streaming first and the
        pid never resumed, PX4 would sit with no setpoints and trip its
        OFFBOARD failsafe. The overlap is safe precisely because of step 0:
        both paths are commanding the SAME hold, so it does not matter which
        one PX4 acted on last.
        """
        if self.state != STATE_HANDBACK:
            self._to(now, STATE_HANDBACK, why)
            self._pid_cmds = 0
            self._handback_step = 1
            self._robot_pose_acked = False
            return TickAction(self.state, STREAM_HOLD, None,
                              request_robot_pose=True, note=why)
        if self._handback_step == 1:
            pinned = (self._robot_pose_acked
                      or now - self._state_since >= self.cfg.handback_pin_s)
            if not pinned:
                return TickAction(self.state, STREAM_HOLD, None)
            self._handback_step = 2
            return TickAction(self.state, STREAM_HOLD, False,
                              note='un-muting the pid')
        return TickAction(self.state, STREAM_HOLD, None)

    def _to(self, now: float, state: str, why: str) -> None:
        self.state = state
        self._state_since = now
        if state != STATE_STREAMING:
            self._hold_since = None
        if state != STATE_HANDBACK:
            self._handback_step = 0
        # Only clear the lost-mute flag where the mute is genuinely being
        # (re-)established. Clearing it on every transition would let a
        # STREAMING -> GRACE hop swallow a refusal that arrived in the same
        # tick, and we would keep holding the vehicle on a mute that is
        # already lapsing.
        if state in (STATE_IDLE, STATE_MUTING):
            self._mute_lost = False
        if state == STATE_IDLE:
            self._pid_cmds = 0
            self._mute_pending = False
            self._last_mute_refresh = float('-inf')


def _clamp_age(age: Optional[float]) -> Optional[float]:
    if age is None:
        return None
    return max(0.0, float(age))
