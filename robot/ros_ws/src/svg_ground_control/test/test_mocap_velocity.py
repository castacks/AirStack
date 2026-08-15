"""Unit tests for mocap velocity estimators."""

from __future__ import annotations

import numpy as np
import pytest

from svg_ground_control.mocap_velocity import (
    create_velocity_estimator,
    FiniteDifferenceVelocityEstimator,
    KalmanVelocityEstimator,
    LowPassVelocityEstimator,
    MocapVelocityEstimator,
)


def test_velocity_estimator_constant_speed() -> None:
    estimator = MocapVelocityEstimator(alpha=1.0)
    velocity = estimator.update(np.array([0.0, 0.0, 0.0]), 0.0)
    np.testing.assert_allclose(velocity, 0.0, atol=1.0e-6)
    velocity = estimator.update(np.array([0.1, 0.0, 0.0]), 0.01)
    np.testing.assert_allclose(velocity, [10.0, 0.0, 0.0], atol=1.0e-5)


def test_low_pass_reduces_stationary_measurement_noise() -> None:
    random = np.random.default_rng(7)
    timestamps = np.arange(600, dtype=float) / 120.0
    positions = random.normal(0.0, 0.005, size=(timestamps.size, 3))
    raw = FiniteDifferenceVelocityEstimator()
    low_pass = LowPassVelocityEstimator(cutoff_hz=5.0)

    raw_velocity = np.asarray([
        raw.update(position, stamp)
        for position, stamp in zip(positions, timestamps)
    ])
    filtered_velocity = np.asarray([
        low_pass.update(position, stamp)
        for position, stamp in zip(positions, timestamps)
    ])

    raw_rms = np.sqrt(np.mean(np.square(raw_velocity[20:])))
    filtered_rms = np.sqrt(np.mean(np.square(filtered_velocity[20:])))
    assert filtered_rms < 0.45 * raw_rms


def test_kalman_velocity_is_more_accurate_than_raw_difference() -> None:
    random = np.random.default_rng(17)
    timestamps = np.arange(1200, dtype=float) / 120.0
    expected_velocity = np.array([1.5, -0.4, 0.2])
    positions = timestamps[:, None] * expected_velocity
    positions += random.normal(0.0, 0.005, size=positions.shape)
    raw = FiniteDifferenceVelocityEstimator()
    kalman = KalmanVelocityEstimator(
        position_stddev_m=0.005,
        acceleration_stddev_mps2=10.0,
    )

    raw_velocity = np.asarray([
        raw.update(position, stamp)
        for position, stamp in zip(positions, timestamps)
    ])
    kalman_velocity = np.asarray([
        kalman.update(position, stamp)
        for position, stamp in zip(positions, timestamps)
    ])
    expected = np.broadcast_to(expected_velocity, kalman_velocity.shape)
    raw_rmse = np.sqrt(np.mean(np.square(raw_velocity[100:] - expected[100:])))
    kalman_rmse = np.sqrt(
        np.mean(np.square(kalman_velocity[100:] - expected[100:]))
    )

    assert kalman_rmse < 0.2 * raw_rmse
    np.testing.assert_allclose(
        np.mean(kalman_velocity[100:], axis=0),
        expected_velocity,
        atol=0.03,
    )
    assert np.all(kalman.velocity_variance > 0.0)


@pytest.mark.parametrize(
    ('filter_name', 'expected_type'),
    [
        ('finite_difference', FiniteDifferenceVelocityEstimator),
        ('low-pass', LowPassVelocityEstimator),
        ('kalman', KalmanVelocityEstimator),
    ],
)
def test_estimator_factory(filter_name, expected_type) -> None:
    assert isinstance(create_velocity_estimator(filter_name), expected_type)


def test_estimator_factory_rejects_unknown_filter() -> None:
    with pytest.raises(ValueError, match='finite_difference.*low_pass.*kalman'):
        create_velocity_estimator('moving_average')


def test_low_pass_factory_rejects_negative_cutoff() -> None:
    with pytest.raises(ValueError, match='cutoff_hz must be >= 0'):
        create_velocity_estimator('low_pass', low_pass_cutoff_hz=-1.0)


def test_duplicate_stamp_does_not_replace_last_valid_sample() -> None:
    estimator = FiniteDifferenceVelocityEstimator()
    estimator.update([0.0, 0.0, 0.0], 0.0)
    estimator.update([0.1, 0.0, 0.0], 0.1)
    estimator.update([99.0, 0.0, 0.0], 0.1)
    velocity = estimator.update([0.2, 0.0, 0.0], 0.2)
    np.testing.assert_allclose(velocity, [1.0, 0.0, 0.0])


@pytest.mark.parametrize(
    'estimator',
    [FiniteDifferenceVelocityEstimator(), KalmanVelocityEstimator()],
)
def test_estimator_resets_when_timestamp_moves_backwards(estimator) -> None:
    estimator.update([0.0, 0.0, 0.0], 10.0)
    estimator.update([0.1, 0.0, 0.0], 10.1)
    velocity_after_reset = estimator.update([0.0, 0.0, 0.0], 1.0)
    np.testing.assert_allclose(velocity_after_reset, 0.0)
