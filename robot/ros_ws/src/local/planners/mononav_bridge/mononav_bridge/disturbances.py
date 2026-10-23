"""Deterministic vision disturbances used by the WS2 test bench."""

from __future__ import annotations

from collections import deque
from typing import Any
import json
import math

import numpy as np


def parse_schedule(value: str) -> dict | None:
    """Validate ROS JSON schedule parameters without trusting the caller."""
    raw=json.loads(value)
    if raw=={}:return None  # Legacy constant disturbance, including preflight.
    if not isinstance(raw,dict) or set(raw)!={'start_s','duration_s','period_s'}:
        raise ValueError('Schedule requires start_s, duration_s, period_s')
    for key,item in raw.items():
        if type(item) not in (int,float) or not math.isfinite(item) or not 0<=item<=500:
            raise ValueError(f'Schedule {key} must be finite and in 0..500 seconds')
    if raw['period_s'] and not 0<raw['duration_s']<raw['period_s']:
        raise ValueError('Repeating schedule requires 0 < duration_s < period_s')
    return {key:float(item) for key,item in raw.items()}


def schedule_active(schedule: dict | None, elapsed_s: float | None) -> bool:
    """Half-open windows evaluated on the sensor capture simulation clock."""
    if schedule is None:return True
    if elapsed_s is None or elapsed_s<schedule['start_s']:return False
    offset=elapsed_s-schedule['start_s']
    if schedule['period_s']:offset%=schedule['period_s']
    return schedule['duration_s']==0 or offset<schedule['duration_s']


class SensorScheduleClock:
    """Anchor at first executed command; a simulator reset disarms the clock."""
    def __init__(self):
        self.start_s=None
        self.last_s=None

    def reset(self):
        self.start_s=None
        self.last_s=None

    def elapsed(self,now_s):
        now_s=float(now_s)
        if self.last_s is not None and now_s<self.last_s-1e-6:self.start_s=None
        self.last_s=now_s
        return None if self.start_s is None else max(0.,now_s-self.start_s)

    def arm(self,now_s):
        self.elapsed(now_s)
        if self.start_s is None:self.start_s=float(now_s)


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

    def push(self, sample: Any, now_s: float, delay_s: float | None = None) -> None:
        """Optional capture-specific delay supports a time-varying schedule."""
        delay=self.delay_s if delay_s is None else float(delay_s)
        if not math.isfinite(delay) or delay<0:raise ValueError('delay_s must be finite and nonnegative')
        self._pending.append((float(now_s)+delay,sample))

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
        # Delays may fall between captures. A later undelayed frame must not
        # wait behind an older delayed frame, and late old frames must never
        # replace the newest released image. Pending order is capture order.
        newest_due=None
        for index,(deadline,_) in enumerate(self._pending):
            if deadline<=now_s:newest_due=index
        if newest_due is not None:
            for _ in range(newest_due+1):_,self._latest=self._pending.popleft()
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
