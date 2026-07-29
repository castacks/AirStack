"""Tests for policy observation wiring (no ROS graph required for core math)."""

from __future__ import annotations

import numpy as np

from drone_soccer.deploy.observation import (
    OBS_DIM,
    arrays_to_observation,
    quaternion_to_rotation_matrix,
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
