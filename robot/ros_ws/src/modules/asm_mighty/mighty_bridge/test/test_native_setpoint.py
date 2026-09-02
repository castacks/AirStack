"""Goal -> PositionTarget mapping, frames and flag parsing.

Pure functions only — no ROS on the box. What each group of tests is actually
protecting is written above it, because most of these values are load-bearing
in a way that reading the code does not show: a wrong `type_mask` bit or a
wrong frame constant produces a drone that flies, just not where it was told.
"""
import math

import pytest

from mighty_bridge import native_setpoint as ns


# ── the exact wire values ───────────────────────────────────────────────────
# These are the numbers PX4 sees. A regression here is invisible in every
# other test: a bad frame constant makes MAVROS skip its ENU->NED conversion
# (the vehicle flies a mirrored path), and a bad type_mask makes the FCU drop
# whichever component got masked (tracking silently degrades to the old
# behaviour it was the point of this path to improve on).

def test_frame_constant_is_local_ned_not_body():
    # mavros_interface.cpp's velocity_callback uses FRAME_BODY_NED because its
    # command IS body-relative. Ours must not: MIGHTY plans in the map frame.
    assert ns.FRAME_LOCAL_NED == 1
    assert ns.FRAME_BODY_NED == 8
    sp = ns.goal_to_setpoint((1, 2, 3), (0, 0, 0), (0, 0, 0), 0.0, 0.0)
    assert sp.coordinate_frame == ns.FRAME_LOCAL_NED


def test_default_type_mask_activates_p_v_a_yaw_and_nothing_else():
    sp = ns.goal_to_setpoint((1, 2, 3), (4, 5, 6), (7, 8, 9), 0.5, 0.25)
    assert sp.type_mask == 2048            # IGNORE_YAW_RATE, and only that
    for bit in (ns.IGNORE_PX, ns.IGNORE_PY, ns.IGNORE_PZ,
                ns.IGNORE_VX, ns.IGNORE_VY, ns.IGNORE_VZ,
                ns.IGNORE_AFX, ns.IGNORE_AFY, ns.IGNORE_AFZ,
                ns.IGNORE_YAW):
        assert not sp.type_mask & bit
    # FORCE would make PX4 read afx/afy/afz as a force, not an acceleration.
    assert not sp.type_mask & ns.FORCE
    assert sp.type_mask & ns.IGNORE_YAW_RATE


def test_yaw_rate_opt_in_clears_the_last_ignore_bit():
    sp = ns.goal_to_setpoint((0, 0, 0), (5, 0, 0), (0, 0, 0), 0.0, 0.75,
                             send_yaw_rate=True)
    assert sp.type_mask == 0
    assert sp.yaw_rate == pytest.approx(0.75)


def test_yaw_rate_is_zeroed_when_masked():
    sp = ns.goal_to_setpoint((0, 0, 0), (5, 0, 0), (0, 0, 0), 0.0, 0.75)
    assert sp.yaw_rate == 0.0


# ── the identity ────────────────────────────────────────────────────────────
# MIGHTY's map frame IS the MAVROS local ENU frame (odometry_conversion runs in
# OVERWRITE mode, which only relabels the header), and MAVROS itself does the
# ENU->NED conversion for FRAME_LOCAL_NED. So any rotation, sign flip or axis
# swap in this mapping would be a DOUBLE conversion.

def test_position_velocity_acceleration_are_copied_verbatim():
    sp = ns.goal_to_setpoint((1.5, -2.5, 3.5), (0.1, -0.2, 0.3),
                             (-9.0, 8.0, -7.0), 1.1, 2.2)
    assert sp.position == (1.5, -2.5, 3.5)
    assert sp.velocity == (0.1, -0.2, 0.3)
    assert sp.acceleration == (-9.0, 8.0, -7.0)


def test_no_axis_is_negated_or_swapped():
    """Explicitly pin the sign of every axis: an ENU->NED slip would show as
    y/z negation or an x<->y swap, both of which the verbatim test above
    would also catch — but only if someone remembered to use asymmetric
    numbers. This one cannot be fooled."""
    for i, axis in enumerate('xyz'):
        p = [0.0, 0.0, 0.0]
        p[i] = 1.0
        sp = ns.goal_to_setpoint(tuple(p), (0, 0, 0), (0, 0, 0), 0.0, 0.0)
        assert sp.position[i] == 1.0, axis
        assert sum(abs(v) for v in sp.position) == 1.0, axis


def test_goal_vectors_reads_a_dynus_goal():
    class V:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    class G:
        p = V(1, 2, 3)
        v = V(4, 5, 6)
        a = V(7, 8, 9)
        j = V(10, 11, 12)       # dropped: PositionTarget has no jerk field
        yaw = 0.4
        dyaw = 0.5

    assert ns.goal_vectors(G()) == ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0),
                                    (7.0, 8.0, 9.0), 0.4, 0.5)


# ── yaw ─────────────────────────────────────────────────────────────────────
# Goal.yaw is the DYNUS spline yaw, measured to be 0 for a whole flight in this
# tree (bridge_node._traj_cb: "the drone flew sideways"). The native path
# defaults to the same heading-from-velocity override the legacy path applies.

def test_default_yaw_follows_the_direction_of_travel():
    sp = ns.goal_to_setpoint((0, 0, 0), (0.0, 2.0, 0.0), (0, 0, 0),
                             goal_yaw=0.0, goal_dyaw=0.0)
    assert sp.yaw == pytest.approx(math.pi / 2)


def test_yaw_holds_previous_heading_below_min_speed():
    sp = ns.goal_to_setpoint((0, 0, 0), (0.01, 0.0, 0.0), (0, 0, 0),
                             goal_yaw=0.0, goal_dyaw=0.0, prev_yaw=1.234)
    assert sp.yaw == pytest.approx(1.234)


def test_goal_yaw_source_passes_the_spline_yaw_through():
    sp = ns.goal_to_setpoint((0, 0, 0), (0.0, 2.0, 0.0), (0, 0, 0),
                             goal_yaw=0.77, goal_dyaw=0.0,
                             yaw_source=ns.YAW_SOURCE_GOAL)
    assert sp.yaw == pytest.approx(0.77)


def test_unknown_yaw_source_is_rejected():
    with pytest.raises(ValueError):
        ns.goal_to_setpoint((0, 0, 0), (0, 0, 0), (0, 0, 0), 0.0, 0.0,
                            yaw_source='compass')


def test_heading_from_velocity_matches_the_legacy_path():
    """The two command paths must point the nose the same way. bridge_node
    imports this very function, so this test is really pinning the shared
    definition against the behaviour the legacy path already flew."""
    assert ns.heading_from_velocity(1.0, 1.0, 0.0) == pytest.approx(math.pi / 4)
    assert ns.heading_from_velocity(-1.0, 0.0, 0.0) == pytest.approx(math.pi)
    assert ns.heading_from_velocity(0.0, 0.0, 2.0) == 2.0


# ── hold ────────────────────────────────────────────────────────────────────

def test_hold_setpoint_is_a_full_stop_at_a_point():
    sp = ns.hold_setpoint((5.0, 6.0, 7.0), 0.9)
    assert sp.position == (5.0, 6.0, 7.0)
    assert sp.velocity == (0.0, 0.0, 0.0)
    assert sp.acceleration == (0.0, 0.0, 0.0)
    assert sp.yaw == pytest.approx(0.9)
    assert sp.yaw_rate == 0.0
    # Same mask and frame as a normal setpoint: PX4 must not see the setpoint
    # TYPE change mid-stream, only the values.
    assert sp.type_mask == ns.TYPE_MASK_PVA_YAW
    assert sp.coordinate_frame == ns.FRAME_LOCAL_NED


# ── frames ──────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('frame', ['map', '', '  ', None, ' map '])
def test_native_frames_are_flown(frame):
    assert ns.frame_is_native(frame, 'map')


@pytest.mark.parametrize('frame', ['world', 'odom', 'base_link', 'robot_1/map'])
def test_foreign_frames_are_refused(frame):
    # Refusing (rather than transforming) is the design: the coordinates would
    # otherwise be flown as if they were map, silently.
    assert not ns.frame_is_native(frame, 'map')


# ── the mirrored-constants guard ────────────────────────────────────────────

def test_constants_guard_accepts_a_matching_definition():
    class FakePositionTarget:
        pass
    for name in ns._MIRRORED_CONSTANTS:
        setattr(FakePositionTarget, name, getattr(ns, name))
    ns.assert_position_target_constants(FakePositionTarget)     # no raise


def test_constants_guard_catches_a_renumbered_bit():
    class FakePositionTarget:
        pass
    for name in ns._MIRRORED_CONSTANTS:
        setattr(FakePositionTarget, name, getattr(ns, name))
    FakePositionTarget.IGNORE_YAW_RATE = 4096
    with pytest.raises(RuntimeError, match='IGNORE_YAW_RATE'):
        ns.assert_position_target_constants(FakePositionTarget)


def test_constants_guard_catches_a_missing_constant():
    class FakePositionTarget:
        pass
    for name in ns._MIRRORED_CONSTANTS[:-1]:
        setattr(FakePositionTarget, name, getattr(ns, name))
    with pytest.raises(RuntimeError, match='IGNORE_YAW_RATE'):
        ns.assert_position_target_constants(FakePositionTarget)


# ── the env switch ──────────────────────────────────────────────────────────

@pytest.mark.parametrize('value', ['1', 'true', 'TRUE', 'yes', 'on', ' 1 '])
def test_env_flag_true_forms(value):
    assert ns.env_flag(value) is True


@pytest.mark.parametrize('value', ['0', 'false', 'no', 'off', 'nonsense'])
def test_env_flag_false_forms(value):
    assert ns.env_flag(value) is False


def test_env_flag_unset_and_empty_fall_back_to_the_default():
    # compose passes MIGHTY_NATIVE_SETPOINTS bare-name, so "set but empty" is
    # a real state and must not beat the node-side default.
    assert ns.env_flag(None) is False
    assert ns.env_flag('') is False
    assert ns.env_flag(None, default=True) is True
    assert ns.env_flag('   ', default=True) is True
