import math

import numpy as np

from svg_ground_control.trajectory_commander_core import (
    ScriptedTrajectory,
    clamp_position,
)


def make_trajectory(script_type, **overrides):
    values = {
        'center': [1.0, 2.0, 3.0],
        'radius_m': 2.0,
        'amplitude': [2.0, 1.0],
        'period_s': 4.0,
        'waypoints': [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        'segment_durations': [2.0],
        'loop': True,
    }
    values.update(overrides)
    return ScriptedTrajectory(script_type=script_type, **values)


def test_clamp_position_limits_each_axis():
    result = clamp_position(
        [4.0, -4.0, 1.0],
        [-2.0, -3.0, 0.5],
        [2.0, 3.0, 2.0],
    )
    np.testing.assert_allclose(result, [2.0, -3.0, 1.0])


def test_circle_starts_at_positive_x_radius():
    trajectory = make_trajectory('circle')
    np.testing.assert_allclose(trajectory.sample(0.0), [3.0, 2.0, 3.0])
    np.testing.assert_allclose(trajectory.sample(1.0), [1.0, 4.0, 3.0])


def test_figure8_returns_to_center_after_half_period():
    trajectory = make_trajectory('figure8')
    np.testing.assert_allclose(
        trajectory.sample(trajectory.period_s / 2.0),
        trajectory.center,
        atol=1e-12,
    )


def test_waypoint_loop_interpolates_and_wraps():
    trajectory = make_trajectory('waypoints')
    np.testing.assert_allclose(trajectory.sample(1.0), [1.0, 0.0, 0.0])
    np.testing.assert_allclose(trajectory.sample(2.5), [0.5, 0.0, 0.0])
    assert math.isclose(trajectory.period_s, 4.0)
