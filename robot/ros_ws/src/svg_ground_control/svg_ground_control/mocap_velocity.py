"""Linear-velocity estimators for timestamped motion-capture positions."""

from __future__ import annotations

import math

import numpy as np


MIN_DT_S = 1.0e-6
DEFAULT_MAX_DT_S = 0.5


def _validated_position(position: np.ndarray) -> np.ndarray:
    value = np.asarray(position, dtype=float).reshape(3)
    if not np.all(np.isfinite(value)):
        raise ValueError(f'position must contain finite values, got {value}')
    return value


def _validated_stamp(stamp_s: float) -> float:
    value = float(stamp_s)
    if not math.isfinite(value):
        raise ValueError(f'stamp_s must be finite, got {stamp_s}')
    return value


class _DifferenceEstimator:
    """Common timestamp and finite-difference handling."""

    def __init__(self, max_dt_s: float = DEFAULT_MAX_DT_S) -> None:
        if max_dt_s <= MIN_DT_S:
            raise ValueError(f'max_dt_s must be > {MIN_DT_S}, got {max_dt_s}')
        self.max_dt_s = float(max_dt_s)
        self.prev_position: np.ndarray | None = None
        self.prev_stamp_s: float | None = None
        self.velocity = np.zeros(3, dtype=float)

    @property
    def velocity_variance(self) -> np.ndarray:
        """Return per-axis variance when known; differences have no model."""
        return np.full(3, np.nan, dtype=float)

    def reset(self) -> None:
        self.prev_position = None
        self.prev_stamp_s = None
        self.velocity = np.zeros(3, dtype=float)
        self._reset_filter()

    def _reset_filter(self) -> None:
        """Reset subclass-specific state."""

    def _difference(
        self,
        position: np.ndarray,
        stamp_s: float,
    ) -> tuple[np.ndarray | None, float | None]:
        position = _validated_position(position)
        stamp_s = _validated_stamp(stamp_s)

        if self.prev_position is None or self.prev_stamp_s is None:
            self.prev_position = position.copy()
            self.prev_stamp_s = stamp_s
            return None, None

        dt = stamp_s - self.prev_stamp_s
        if dt < -MIN_DT_S:
            # ROS bag loops and clock resets should restart the estimator
            # instead of leaving it frozen behind a future timestamp.
            self.prev_position = position.copy()
            self.prev_stamp_s = stamp_s
            self.velocity = np.zeros(3, dtype=float)
            self._reset_filter()
            return None, None
        if dt <= MIN_DT_S:
            # Duplicate/out-of-order stamps must not replace the last valid
            # sample, otherwise the next derivative uses the wrong interval.
            return None, None
        if dt > self.max_dt_s:
            self.prev_position = position.copy()
            self.prev_stamp_s = stamp_s
            self.velocity = np.zeros(3, dtype=float)
            self._reset_filter()
            return None, None

        difference = (position - self.prev_position) / dt
        self.prev_position = position.copy()
        self.prev_stamp_s = stamp_s
        return difference, dt


class FiniteDifferenceVelocityEstimator(_DifferenceEstimator):
    """Unfiltered finite-difference velocity, primarily for comparison."""

    def update(self, position: np.ndarray, stamp_s: float) -> np.ndarray:
        difference, _ = self._difference(position, stamp_s)
        if difference is not None:
            self.velocity = difference
        return self.velocity.copy()


class LowPassVelocityEstimator(_DifferenceEstimator):
    """
    First-order low-pass filter applied to finite-difference velocity.

    A cutoff frequency is rate independent and is preferred when mocap timing
    varies. If ``cutoff_hz`` is omitted, the legacy fixed ``alpha`` is used.
    """

    def __init__(
        self,
        alpha: float = 0.4,
        cutoff_hz: float | None = None,
        max_dt_s: float = DEFAULT_MAX_DT_S,
    ) -> None:
        super().__init__(max_dt_s=max_dt_s)
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha must be in (0, 1], got {alpha}')
        if cutoff_hz is not None and cutoff_hz <= 0.0:
            raise ValueError(f'cutoff_hz must be > 0, got {cutoff_hz}')
        self.alpha = float(alpha)
        self.cutoff_hz = None if cutoff_hz is None else float(cutoff_hz)

    def update(self, position: np.ndarray, stamp_s: float) -> np.ndarray:
        difference, dt = self._difference(position, stamp_s)
        if difference is None or dt is None:
            return self.velocity.copy()

        alpha = self.alpha
        if self.cutoff_hz is not None:
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz * dt)
        self.velocity = alpha * difference + (1.0 - alpha) * self.velocity
        return self.velocity.copy()


class KalmanVelocityEstimator:
    """
    Constant-velocity Kalman filter driven by 3-D position measurements.

    The six-state model is ``[x, y, z, vx, vy, vz]``. Process noise models an
    unknown acceleration independently on each axis, while measurement noise
    describes mocap position uncertainty.
    """

    def __init__(
        self,
        position_stddev_m: float = 0.005,
        acceleration_stddev_mps2: float = 10.0,
        initial_velocity_stddev_mps: float = 2.0,
        max_dt_s: float = DEFAULT_MAX_DT_S,
    ) -> None:
        if position_stddev_m <= 0.0:
            raise ValueError(
                'position_stddev_m must be > 0, '
                f'got {position_stddev_m}'
            )
        if acceleration_stddev_mps2 <= 0.0:
            raise ValueError(
                'acceleration_stddev_mps2 must be > 0, '
                f'got {acceleration_stddev_mps2}'
            )
        if initial_velocity_stddev_mps <= 0.0:
            raise ValueError(
                'initial_velocity_stddev_mps must be > 0, '
                f'got {initial_velocity_stddev_mps}'
            )
        if max_dt_s <= MIN_DT_S:
            raise ValueError(f'max_dt_s must be > {MIN_DT_S}, got {max_dt_s}')

        self.position_variance = float(position_stddev_m) ** 2
        self.acceleration_variance = float(acceleration_stddev_mps2) ** 2
        self.initial_velocity_variance = (
            float(initial_velocity_stddev_mps) ** 2
        )
        self.max_dt_s = float(max_dt_s)
        self.state = np.zeros(6, dtype=float)
        self.covariance = np.zeros((6, 6), dtype=float)
        self.prev_stamp_s: float | None = None
        self._initialized = False

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:].copy()

    @property
    def velocity_variance(self) -> np.ndarray:
        if not self._initialized:
            return np.full(3, np.nan, dtype=float)
        return np.diag(self.covariance)[3:].copy()

    def reset(self) -> None:
        self.state = np.zeros(6, dtype=float)
        self.covariance = np.zeros((6, 6), dtype=float)
        self.prev_stamp_s = None
        self._initialized = False

    def _initialize(self, position: np.ndarray, stamp_s: float) -> None:
        self.state[:3] = position
        self.state[3:] = 0.0
        self.covariance = np.diag(
            [self.position_variance] * 3
            + [self.initial_velocity_variance] * 3
        )
        self.prev_stamp_s = stamp_s
        self._initialized = True

    def update(self, position: np.ndarray, stamp_s: float) -> np.ndarray:
        position = _validated_position(position)
        stamp_s = _validated_stamp(stamp_s)
        if not self._initialized or self.prev_stamp_s is None:
            self._initialize(position, stamp_s)
            return self.velocity

        dt = stamp_s - self.prev_stamp_s
        if dt < -MIN_DT_S:
            self._initialize(position, stamp_s)
            return self.velocity
        if dt <= MIN_DT_S:
            return self.velocity
        if dt > self.max_dt_s:
            self._initialize(position, stamp_s)
            return self.velocity

        identity_3 = np.eye(3)
        transition = np.block(
            [
                [identity_3, dt * identity_3],
                [np.zeros((3, 3)), identity_3],
            ]
        )
        acceleration_gain = np.vstack(
            (0.5 * dt * dt * identity_3, dt * identity_3)
        )
        process_noise = (
            self.acceleration_variance
            * acceleration_gain
            @ acceleration_gain.T
        )

        predicted_state = transition @ self.state
        predicted_covariance = (
            transition @ self.covariance @ transition.T + process_noise
        )

        measurement_model = np.hstack((identity_3, np.zeros((3, 3))))
        measurement_noise = self.position_variance * identity_3
        innovation = position - measurement_model @ predicted_state
        innovation_covariance = (
            measurement_model
            @ predicted_covariance
            @ measurement_model.T
            + measurement_noise
        )
        covariance_times_h = predicted_covariance @ measurement_model.T
        gain = np.linalg.solve(
            innovation_covariance,
            covariance_times_h.T,
        ).T

        self.state = predicted_state + gain @ innovation
        # Joseph form keeps the covariance symmetric and positive semidefinite
        # despite floating-point roundoff.
        identity_6 = np.eye(6)
        residual_transform = identity_6 - gain @ measurement_model
        self.covariance = (
            residual_transform
            @ predicted_covariance
            @ residual_transform.T
            + gain @ measurement_noise @ gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.prev_stamp_s = stamp_s
        return self.velocity


def create_velocity_estimator(
    filter_type: str = 'kalman',
    *,
    alpha: float = 0.4,
    low_pass_cutoff_hz: float = 0.0,
    kalman_position_stddev_m: float = 0.005,
    kalman_acceleration_stddev_mps2: float = 10.0,
    kalman_initial_velocity_stddev_mps: float = 2.0,
    max_dt_s: float = DEFAULT_MAX_DT_S,
):
    """Construct an estimator from a ROS-friendly filter name and settings."""
    normalized = str(filter_type).strip().lower().replace('-', '_')
    if normalized in ('raw', 'none', 'finite_difference'):
        return FiniteDifferenceVelocityEstimator(max_dt_s=max_dt_s)
    if normalized in ('ema', 'low_pass', 'lowpass'):
        if low_pass_cutoff_hz < 0.0:
            raise ValueError(
                'low_pass_cutoff_hz must be >= 0, '
                f'got {low_pass_cutoff_hz}'
            )
        cutoff = low_pass_cutoff_hz if low_pass_cutoff_hz > 0.0 else None
        return LowPassVelocityEstimator(
            alpha=alpha,
            cutoff_hz=cutoff,
            max_dt_s=max_dt_s,
        )
    if normalized == 'kalman':
        return KalmanVelocityEstimator(
            position_stddev_m=kalman_position_stddev_m,
            acceleration_stddev_mps2=kalman_acceleration_stddev_mps2,
            initial_velocity_stddev_mps=(
                kalman_initial_velocity_stddev_mps
            ),
            max_dt_s=max_dt_s,
        )
    raise ValueError(
        'filter_type must be finite_difference|low_pass|kalman, '
        f'got {filter_type!r}'
    )


class MocapVelocityEstimator(LowPassVelocityEstimator):
    """Backward-compatible name for the original fixed-alpha estimator."""
