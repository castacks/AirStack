"""Unit tests for mocap velocity helper."""

from __future__ import annotations

import numpy as np

from svg_ground_control.mocap_velocity import MocapVelocityEstimator


def test_velocity_estimator_constant_speed() -> None:
    est = MocapVelocityEstimator(alpha=1.0)
    v = est.update(np.array([0.0, 0.0, 0.0]), 0.0)
    np.testing.assert_allclose(v, 0.0, atol=1e-6)
    v = est.update(np.array([0.1, 0.0, 0.0]), 0.01)
    np.testing.assert_allclose(v, [10.0, 0.0, 0.0], atol=1e-5)
