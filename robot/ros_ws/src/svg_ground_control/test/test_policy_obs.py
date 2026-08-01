"""Tests for policy observation wiring (no ROS graph required for core math)."""

from __future__ import annotations

import numpy as np

from drone_soccer.deploy.observation import (
    OBS_DIM,
    arrays_to_observation,
    quaternion_to_rotation_matrix,
)
from svg_ground_control.policy_commander import (
    compute_bounded_waypoint,
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
