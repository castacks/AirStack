"""Position-mode teleop: the sticks move a target, the drone tracks it.

This is what PX4's Position mode does and what a hand-flown drone needs so it
does not drift when the sticks are released. Instead of sending the stick
velocity straight to the vehicle:

    target   += stick_velocity * dt              (the sticks steer a setpoint)
    command   = kp * (target - position) + stick_velocity   (P + feedforward)

Release the sticks and the target stops where it is; any drift is corrected
back to it. The commander owns the target because it knows when control is
handed over (``~/start``), where the drone is, the geofence and the CBF.

Two guards keep the target honest:

* it is **re-seeded from the drone's position** whenever control is handed
  over, so nothing is ever chased from before takeoff or before ``start``
  (that was the altitude drop: a target adopted on the ground);
* a **leash** — it may not lead the drone by more than ``lead_max`` metres, so
  a drone held back by the CBF, the fence or a wall does not accumulate an
  ever-growing error that it later races to close.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


def advance_target(target: Optional[np.ndarray], position: np.ndarray,
                   stick_velocity: np.ndarray, dt: float,
                   lead_max: float) -> np.ndarray:
    """Next target: seeded at ``position`` if None, moved by the sticks, leashed."""
    p = np.asarray(position, dtype=float)
    t = p.copy() if target is None else np.asarray(target, dtype=float).copy()
    t += np.asarray(stick_velocity, dtype=float) * dt
    lead = t - p
    dist = float(np.linalg.norm(lead))
    if lead_max > 0.0 and dist > lead_max:
        t = p + lead * (lead_max / dist)
    return t


def tracking_velocity(target: np.ndarray, position: np.ndarray,
                      stick_velocity: np.ndarray, kp: float,
                      max_speed: float) -> np.ndarray:
    """P-term toward the target plus the stick feedforward, speed-capped."""
    v = kp * (np.asarray(target, dtype=float) - np.asarray(position, dtype=float))
    v = v + np.asarray(stick_velocity, dtype=float)
    speed = float(np.linalg.norm(v))
    if max_speed > 0.0 and speed > max_speed:
        v = v * (max_speed / speed)
    return v


def advance_reference(ref: Optional[np.ndarray], position: np.ndarray,
                      applied: np.ndarray, measured_velocity: np.ndarray,
                      dt: float, lead_max: float):
    """Move a drone's reference point by the velocity it was last told to fly.

    The reference is what PX4 is asked to hold (position setpoint) and where
    the go-to-goal profile is evaluated (trajectory.py). It is seeded at the
    drone when ``None`` and leashed to ``lead_max`` like ``advance_target``.

    Returns ``(ref, applied)``: the velocity the reference really moved with.
    It equals ``applied`` normally, but is the drone's ``measured_velocity``
    when the reference was just seeded or pulled back by the leash — the
    signal for the profile to re-attach to what the drone is actually doing.
    """
    p = np.asarray(position, dtype=float)
    v_meas = np.asarray(measured_velocity, dtype=float).copy()
    if ref is None:
        return p.copy(), v_meas
    v_applied = np.asarray(applied, dtype=float)
    r = np.asarray(ref, dtype=float) + v_applied * dt
    lead = r - p
    dist = float(np.linalg.norm(lead))
    if lead_max > 0.0 and dist > lead_max:
        return p + lead * (lead_max / dist), v_meas
    return r, v_applied.copy()
