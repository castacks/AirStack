"""Geofence math shared by the commander (no ROS here, so it is unit-testable).

Two behaviours, selected by the commander's ``fence_behavior`` parameter:

``hold_all``  (latch) — any policed drone outside the box freezes every drone
              until ``~/reset_fence``. Implemented in the commander; this
              module only supplies ``outside()``.

``keep_in``   (barrier) — nobody stops. Each commanded drone's velocity is
              clipped per axis so it cannot cross the wall, and is pushed back
              inward if it is already outside. ``keep_in_velocity()`` below.
              External (RC-flown) drones cannot be commanded, so keep-in only
              ever *reports* them.
"""

from __future__ import annotations

import numpy as np

BEHAVIORS = ('hold_all', 'keep_in')


def outside(position: np.ndarray, fence_min: np.ndarray,
            fence_max: np.ndarray) -> np.ndarray:
    """Boolean (3,) per axis: position is beyond the box on that axis."""
    p = np.asarray(position, dtype=float)
    return (p < np.asarray(fence_min)) | (p > np.asarray(fence_max))


def violation_text(position: np.ndarray, fence_min: np.ndarray,
                   fence_max: np.ndarray) -> str:
    """'x<min, z>max' style summary of which walls a position is past."""
    p = np.asarray(position, dtype=float)
    below = p < np.asarray(fence_min)
    above = p > np.asarray(fence_max)
    return ', '.join(f'{"xyz"[k]}{"<min" if below[k] else ">max"}'
                     for k in range(3) if below[k] or above[k])


def clamp_to_box(point: np.ndarray, fence_min: np.ndarray, fence_max: np.ndarray,
                 margin: float = 0.0) -> np.ndarray:
    """The point moved to the nearest place inside the (shrunk) box."""
    lo = np.asarray(fence_min, dtype=float) + margin
    hi = np.asarray(fence_max, dtype=float) - margin
    return np.minimum(np.maximum(np.asarray(point, dtype=float), lo), hi)


def keep_in_velocity(velocity: np.ndarray, position: np.ndarray,
                     fence_min: np.ndarray, fence_max: np.ndarray,
                     gain: float = 1.0, margin: float = 0.0) -> np.ndarray:
    """Clip a velocity so the drone stays inside the box (one axis at a time).

    On each axis the outward component may not exceed ``gain`` times the
    remaining distance to the wall, so it decays to zero at the wall (a
    velocity barrier). Already past the wall, the bound is negative: the
    drone is commanded back inside at ``gain`` times its overshoot. Inward
    motion is never limited. ``margin`` shrinks the box so the wall is met a
    little early.

    Purely per-axis, so it composes with anything that produced ``velocity``
    (a scenario, the sticks, the CBF) and never turns a stop into motion.
    """
    v = np.array(velocity, dtype=float)
    p = np.asarray(position, dtype=float)
    lo = np.asarray(fence_min, dtype=float) + margin
    hi = np.asarray(fence_max, dtype=float) - margin
    v_max = gain * (hi - p)          # largest allowed +v: room to the top wall
    v_min = -gain * (p - lo)         # most negative allowed v: room to bottom
    return np.minimum(np.maximum(v, v_min), v_max)
