"""EMA finite-difference linear velocity from mocap poses."""

from __future__ import annotations

import numpy as np


class MocapVelocityEstimator:
    """Track world-frame linear velocity from successive pose samples."""

    def __init__(self, alpha: float = 0.4) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha must be in (0, 1], got {alpha}')
        self.alpha = float(alpha)
        self.prev_position: np.ndarray | None = None
        self.prev_stamp_s: float | None = None
        self.velocity = np.zeros(3, dtype=float)

    def reset(self) -> None:
        self.prev_position = None
        self.prev_stamp_s = None
        self.velocity = np.zeros(3, dtype=float)

    def update(self, position: np.ndarray, stamp_s: float) -> np.ndarray:
        """Ingest a pose sample; return the filtered velocity (m/s)."""
        position = np.asarray(position, dtype=float).reshape(3)
        if self.prev_position is not None and self.prev_stamp_s is not None:
            dt = stamp_s - self.prev_stamp_s
            if 1e-6 < dt < 0.5:
                raw = (position - self.prev_position) / dt
                self.velocity = (
                    self.alpha * raw + (1.0 - self.alpha) * self.velocity
                )
        self.prev_position = position.copy()
        self.prev_stamp_s = float(stamp_s)
        return self.velocity.copy()
