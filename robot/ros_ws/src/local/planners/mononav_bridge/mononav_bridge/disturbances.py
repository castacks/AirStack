"""Deterministic vision disturbances used by the WS2 test bench."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np


def add_rgb_noise(
    image: np.ndarray, standard_deviation: float, seed: int, sequence: int
) -> np.ndarray:
    """Add replayable Gaussian noise in 8-bit pixel-intensity units."""
    if standard_deviation <= 0.0:
        return image
    rng = _rng(seed, sequence, stream=0)
    noisy = image.astype(np.float32) + rng.normal(
        0.0, standard_deviation, image.shape
    ).astype(np.float32)
    return np.clip(np.rint(noisy), 0.0, 255.0).astype(np.uint8)


def add_depth_noise(
    depth_m: np.ndarray, standard_deviation_m: float, seed: int, sequence: int
) -> np.ndarray:
    """Add replayable Gaussian noise to valid metric-depth pixels."""
    if standard_deviation_m <= 0.0:
        return depth_m
    rng = _rng(seed, sequence, stream=1)
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    result = depth_m.copy()
    result[valid] += rng.normal(0.0, standard_deviation_m, int(valid.sum())).astype(
        np.float32
    )
    result[valid] = np.maximum(result[valid], 0.0)
    return result


class DelayedSampleBuffer:
    """Release the newest sample whose configured clock-time delay has
    elapsed. Clock-agnostic: the caller supplies `now_s` from whatever clock
    it wants delay measured against (wall or simulation time)."""

    def __init__(self, delay_s: float) -> None:
        if delay_s < 0.0:
            raise ValueError("delay_s must be nonnegative")
        self.delay_s = float(delay_s)
        self._pending: deque[tuple[float, Any]] = deque()
        self._latest: Any | None = None
        self._last_now_s: float | None = None

    def push(self, sample: Any, now_s: float) -> None:
        self._pending.append((float(now_s) + self.delay_s, sample))

    def latest(self, now_s: float) -> Any | None:
        """Release samples due by `now_s`.

        Simulation time can jump backwards (a sim reset), which would
        otherwise strand pending samples at deadlines that never arrive
        again. Re-base pending deadlines by the observed backward jump
        instead of dropping them.
        """
        now_s = float(now_s)
        if self._last_now_s is not None and now_s < self._last_now_s:
            shift = self._last_now_s - now_s
            self._pending = deque(
                (deadline - shift, sample) for deadline, sample in self._pending
            )
        self._last_now_s = now_s
        while self._pending and self._pending[0][0] <= now_s:
            _, self._latest = self._pending.popleft()
        return self._latest

    @property
    def pending_count(self) -> int:
        return len(self._pending)


def _rng(seed: int, sequence: int, *, stream: int) -> np.random.Generator:
    words = [
        int(seed) & 0xFFFFFFFF,
        (int(seed) >> 32) & 0xFFFFFFFF,
        int(sequence) & 0xFFFFFFFF,
        int(stream) & 0xFFFFFFFF,
    ]
    return np.random.default_rng(np.random.SeedSequence(words))
