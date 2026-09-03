"""Tests for policy observation wiring (no ROS graph required for core math)."""

from __future__ import annotations

import numpy as np
import pytest

from drone_soccer.deploy.observation import (
    OBS_DIM,
    arrays_to_observation,
    quaternion_to_rotation_matrix,
)
from svg_ground_control.policy_commander import (
    RandomGoalSpawner,
    compute_bounded_waypoint,
    compute_goal_reference,
    enu_waypoint_to_ned,
)


def test_observation_dim() -> None:
    rng = np.random.default_rng(1)
    q = rng.standard_normal(4)
    q /= np.linalg.norm(q)
    rot = quaternion_to_rotation_matrix(*q).astype(np.float32)
    obs = arrays_to_observation(
        rng.standard_normal(3).astype(np.float32),
        rng.standard_normal(3).astype(np.float32),
        rot,
        rng.standard_normal(3).astype(np.float32),
        rng.standard_normal(3).astype(np.float32),
        rng.standard_normal(3).astype(np.float32),
        np.array([3.0, 0.0], dtype=np.float32),
    )
    assert obs.shape == (OBS_DIM,)


def test_enu_waypoint_to_ned() -> None:
    """Policy waypoints must use PX4's North-East-Down axis ordering."""
    waypoint_enu = np.array([1.25, -0.5, 0.8], dtype=np.float32)
    expected_ned = np.array([-0.5, 1.25, -0.8], dtype=np.float32)
    np.testing.assert_array_equal(
        enu_waypoint_to_ned(waypoint_enu),
        expected_ned,
    )


def test_fixed_goal_reference_is_time_invariant() -> None:
    """Fixed goals must ignore elapsed time and circle-only parameters."""
    center = np.array([2.0, -4.0], dtype=np.float32)
    np.testing.assert_array_equal(
        compute_goal_reference('fixed', 123.0, center, 99.0, 0.0),
        center,
    )


def test_circle_goal_reference_cardinal_points_and_periodicity() -> None:
    """The requested circle must hit each cardinal point every two seconds."""
    center = np.array([0.0, 0.0], dtype=np.float32)
    expected_by_time = {
        0.0: [1.5, 0.0],
        2.0: [0.0, 1.5],
        4.0: [-1.5, 0.0],
        6.0: [0.0, -1.5],
        8.0: [1.5, 0.0],
    }
    for elapsed_s, expected in expected_by_time.items():
        actual = compute_goal_reference(
            'circle', elapsed_s, center, radius_m=1.5, period_s=8.0)
        np.testing.assert_allclose(actual, expected, atol=1.0e-6)


def test_figure8_goal_reference_crossings_extrema_and_periodicity() -> None:
    """A Gerono figure eight must cross its center twice per period."""
    center = np.array([0.25, -0.5], dtype=np.float32)
    expected_by_time = {
        0.0: [1.75, -0.5],
        1.0: [0.25 + 1.5 / np.sqrt(2.0), 0.25],
        2.0: [0.25, -0.5],
        4.0: [-1.25, -0.5],
        6.0: [0.25, -0.5],
        8.0: [1.75, -0.5],
    }
    for elapsed_s, expected in expected_by_time.items():
        actual = compute_goal_reference(
            'figure8', elapsed_s, center, radius_m=1.5, period_s=8.0)
        np.testing.assert_allclose(actual, expected, atol=1.0e-6)


def test_figure8_goal_reference_can_rotate_long_axis_to_y() -> None:
    """A 90-degree spatial rotation must put the figure-eight long axis on Y."""
    center = np.array([0.0, 0.0], dtype=np.float32)
    expected_by_time = {
        0.0: [0.0, 1.5],
        1.0: [-0.75, 1.5 / np.sqrt(2.0)],
        2.0: [0.0, 0.0],
        4.0: [0.0, -1.5],
        8.0: [0.0, 1.5],
    }
    for elapsed_s, expected in expected_by_time.items():
        actual = compute_goal_reference(
            'figure8',
            elapsed_s,
            center,
            radius_m=1.5,
            period_s=8.0,
            rotation_rad=np.pi / 2.0,
        )
        np.testing.assert_allclose(actual, expected, atol=1.0e-6)


def test_periodic_goal_reference_rejects_invalid_configuration() -> None:
    """Invalid trajectory types, radii, and periods must fail at startup."""
    center = np.zeros(2, dtype=np.float32)
    with pytest.raises(ValueError, match='goal_radius'):
        compute_goal_reference('circle', 0.0, center, -1.0, 8.0)
    with pytest.raises(ValueError, match='goal_period_s'):
        compute_goal_reference('figure8', 0.0, center, 1.5, 0.0)
    with pytest.raises(ValueError, match='goal_trajectory'):
        compute_goal_reference('spiral', 0.0, center, 1.5, 8.0)


def test_random_goal_spawner_resamples_only_after_arrival() -> None:
    spawner = RandomGoalSpawner(
        minimum_xy=np.array([-2.0, 1.0]),
        maximum_xy=np.array([3.0, 4.0]),
        arrival_radius_m=0.25,
        seed=7,
    )
    first = spawner.goal_xy.copy()
    assert np.all(first >= [-2.0, 1.0])
    assert np.all(first <= [3.0, 4.0])

    unchanged, reached = spawner.update(first + [1.0, 0.0])
    assert not reached
    np.testing.assert_array_equal(unchanged, first)

    second, reached = spawner.update(first + [0.1, 0.0])
    assert reached
    assert np.all(second >= [-2.0, 1.0])
    assert np.all(second <= [3.0, 4.0])
    assert not np.array_equal(second, first)


def test_random_goal_spawner_rejects_invalid_region_and_radius() -> None:
    with pytest.raises(ValueError, match='strictly less'):
        RandomGoalSpawner([0.0, 0.0], [0.0, 1.0], 0.25)
    with pytest.raises(ValueError, match='positive'):
        RandomGoalSpawner([-1.0, -1.0], [1.0, 1.0], 0.0)


def test_policy_waypoint_clamps_xyz_when_drone_is_near_xy_fence() -> None:
    """XY snapping activates near either horizontal geofence boundary."""
    drone_position = np.array([2.8, -2.8, 0.9], dtype=np.float32)
    policy_action = np.array([2.0, -2.0, 1.7], dtype=np.float32)
    waypoint = compute_bounded_waypoint(
        drone_position,
        policy_action,
        bounds_min=np.array([-3.0, -3.0, 0.3], dtype=np.float32),
        bounds_max=np.array([3.0, 3.0, 1.2], dtype=np.float32),
    )
    np.testing.assert_array_equal(
        waypoint,
        np.array([3.0, -3.0, 1.2], dtype=np.float32),
    )


def test_policy_waypoint_xy_is_unconstrained_in_safe_interior() -> None:
    """A far-away policy target is allowed until the drone nears the fence."""
    waypoint = compute_bounded_waypoint(
        drone_position=np.array([1.0, -1.0, 0.9], dtype=np.float32),
        policy_action=np.array([3.0, -3.0, 1.7], dtype=np.float32),
        bounds_min=np.array([-3.0, -3.0, 0.3], dtype=np.float32),
        bounds_max=np.array([3.0, 3.0, 1.2], dtype=np.float32),
        geofence_buffer_m=0.5,
    )
    np.testing.assert_array_equal(
        waypoint,
        np.array([4.0, -4.0, 1.2], dtype=np.float32),
    )


@pytest.mark.parametrize('drone_x', [2.5, 3.2])
def test_policy_waypoint_clamps_at_buffer_edge_and_outside(
    drone_x: float,
) -> None:
    """The buffer boundary is inclusive, and an escaped drone stays guarded."""
    waypoint = compute_bounded_waypoint(
        drone_position=np.array([drone_x, 0.0, 0.9], dtype=np.float32),
        policy_action=np.array([2.0, 0.0, 0.9], dtype=np.float32),
        bounds_min=np.array([-3.0, -3.0, 0.3], dtype=np.float32),
        bounds_max=np.array([3.0, 3.0, 1.2], dtype=np.float32),
        geofence_buffer_m=0.5,
    )
    np.testing.assert_array_equal(
        waypoint,
        np.array([3.0, 0.0, 0.9], dtype=np.float32),
    )


def test_policy_waypoint_rejects_invalid_geofence_buffer() -> None:
    with pytest.raises(ValueError, match='geofence_buffer_m'):
        compute_bounded_waypoint(
            drone_position=np.zeros(3, dtype=np.float32),
            policy_action=np.zeros(3, dtype=np.float32),
            bounds_min=-np.ones(3, dtype=np.float32),
            bounds_max=np.ones(3, dtype=np.float32),
            geofence_buffer_m=-0.1,
        )
