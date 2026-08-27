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


def test_absolute_policy_waypoint_clamps_xyz_to_fence() -> None:
    """Policy waypoints must remain inside the absolute XYZ fence."""
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
