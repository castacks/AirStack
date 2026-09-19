"""fence.keep_in_velocity / clamp_to_box and position_hold target logic."""

import numpy as np
import pytest

from svg_ground_control.fence import (BEHAVIORS, clamp_to_box, keep_in_velocity,
                                      outside, violation_text)
from svg_ground_control.position_hold import advance_target, tracking_velocity

LO = np.array([-2.0, -2.0, 0.0])
HI = np.array([2.0, 2.0, 3.0])


def test_behaviors_listed():
    assert BEHAVIORS == ('hold_all', 'keep_in')


def test_outside_and_text():
    assert not outside([0, 0, 1], LO, HI).any()
    assert outside([2.5, 0, 1], LO, HI).tolist() == [True, False, False]
    assert violation_text([2.5, -3.0, 1], LO, HI) == 'x>max, y<min'


def test_keep_in_inside_far_from_walls_is_untouched():
    v = np.array([1.0, -1.0, 0.5])
    np.testing.assert_allclose(keep_in_velocity(v, [0, 0, 1], LO, HI, gain=1.0), v)


def test_keep_in_decays_outward_speed_to_zero_at_the_wall():
    # 0.3 m from the +x wall, gain 1 -> at most 0.3 m/s outward; inward free.
    v = keep_in_velocity([1.0, 0.0, 0.0], [1.7, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(0.3)
    v = keep_in_velocity([-1.0, 0.0, 0.0], [1.7, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(-1.0)
    v = keep_in_velocity([1.0, 0.0, 0.0], [2.0, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(0.0)


def test_keep_in_pushes_back_when_already_outside():
    # 0.5 m past the +x wall: even a stick pushing outward becomes -0.5 m/s.
    v = keep_in_velocity([1.0, 0.0, 0.0], [2.5, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(-0.5)
    # ...and a stop (zero) becomes the same push-back, never the reverse.
    v = keep_in_velocity([0.0, 0.0, 0.0], [2.5, 0, 1], LO, HI, gain=2.0)
    assert v[0] == pytest.approx(-1.0)


def test_keep_in_is_per_axis_and_respects_margin():
    v = keep_in_velocity([1.0, 1.0, 1.0], [1.9, 0.0, 2.9], LO, HI, gain=1.0,
                         margin=0.2)
    assert v[0] == pytest.approx(-0.1)   # 1.9 is 0.1 past the shrunk wall 1.8
    assert v[1] == pytest.approx(1.0)
    assert v[2] == pytest.approx(-0.1)


def test_clamp_to_box():
    np.testing.assert_allclose(clamp_to_box([5, -5, 1], LO, HI), [2, -2, 1])
    np.testing.assert_allclose(clamp_to_box([5, 0, 1], LO, HI, margin=0.5),
                               [1.5, 0, 1])


def test_target_seeds_from_position_when_none():
    t = advance_target(None, [1.0, 2.0, 1.2], [0, 0, 0], 0.05, lead_max=0.5)
    np.testing.assert_allclose(t, [1.0, 2.0, 1.2])


def test_target_moves_with_the_sticks_and_stays_when_released():
    t = advance_target(None, [0, 0, 1], [1.0, 0, 0], 0.1, 0.5)
    t = advance_target(t, [0, 0, 1], [1.0, 0, 0], 0.1, 0.5)
    np.testing.assert_allclose(t, [0.2, 0, 1])
    held = advance_target(t, [0.2, 0, 1], [0, 0, 0], 0.1, 0.5)
    np.testing.assert_allclose(held, t)


def test_target_is_leashed_to_the_drone():
    # Drone stuck at the origin (CBF / wall): the target cannot run away.
    t = None
    for _ in range(100):
        t = advance_target(t, [0, 0, 1], [1.0, 0, 0], 0.1, lead_max=0.5)
    assert np.linalg.norm(t - [0, 0, 1]) == pytest.approx(0.5)
    # lead_max 0 disables the leash
    t = advance_target([3, 0, 1], [0, 0, 1], [0, 0, 0], 0.1, lead_max=0.0)
    np.testing.assert_allclose(t, [3, 0, 1])


def test_tracking_velocity_holds_and_feeds_forward():
    # Drift 0.2 m below target with sticks released -> climb at kp*0.2.
    v = tracking_velocity([0, 0, 1.2], [0, 0, 1.0], [0, 0, 0], kp=1.0, max_speed=1.0)
    np.testing.assert_allclose(v, [0, 0, 0.2])
    # On target with a stick -> exactly the stick (feedforward).
    v = tracking_velocity([0, 0, 1], [0, 0, 1], [0.7, 0, 0], kp=1.0, max_speed=1.0)
    np.testing.assert_allclose(v, [0.7, 0, 0])
    # Capped.
    v = tracking_velocity([5, 0, 1], [0, 0, 1], [2, 0, 0], kp=1.0, max_speed=1.0)
    assert np.linalg.norm(v) == pytest.approx(1.0)


def test_altitude_drop_scenario_is_gone():
    """Old bug: a target seeded on the ground (0.02 m) chased after takeoff to 1.2."""
    t = advance_target(None, [0, 0, 1.2], [0, 0, 0], 0.05, 0.5)   # seeded at handover
    v = tracking_velocity(t, [0, 0, 1.2], [0, 0, 0], 1.0, 1.0)
    np.testing.assert_allclose(v, [0, 0, 0])
