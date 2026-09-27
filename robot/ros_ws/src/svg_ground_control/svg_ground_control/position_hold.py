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
  ever-growing error that it later races to close. The horizontal and the
  vertical lead are leashed **separately** (``leash``): a drone lagging its
  reference by 0.5 m in x must not have its altitude reference dragged toward
  a momentary sag. Bag ``run_060352`` (2026-09-25): with one 3-D leash the
  reference altitude fell 1.1 m over a flight of pure x-y stick, every step
  while the leash was engaged.
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


def leash(lead: np.ndarray, max_xy: float, max_z: Optional[float] = None):
    """Shorten a reference-minus-position vector to the leash, per axis group.

    The horizontal part is limited to ``max_xy`` and the vertical part to
    ``max_z`` (``max_xy`` when None), each on its own: pulling the reference
    back along the 3-D lead would scale its z component with the horizontal
    lag and walk the altitude reference toward every sag. 0 = no limit on
    that part. Returns ``(lead, pulled)``.
    """
    lead = np.asarray(lead, dtype=float).copy()
    if max_z is None:
        max_z = max_xy
    pulled = False
    horizontal = float(np.hypot(lead[0], lead[1]))
    if max_xy > 0.0 and horizontal > max_xy:
        lead[:2] *= max_xy / horizontal
        pulled = True
    if max_z > 0.0 and abs(lead[2]) > max_z:
        lead[2] = np.sign(lead[2]) * max_z
        pulled = True
    return lead, pulled


def advance_reference(ref: Optional[np.ndarray], position: np.ndarray,
                      applied: np.ndarray, measured_velocity: np.ndarray,
                      dt: float, lead_max: float, lead_max_z: Optional[float] = None):
    """Move a drone's reference point by the velocity it was last told to fly.

    The reference is what PX4 is asked to hold (position setpoint) and where
    the go-to-goal profile is evaluated (trajectory.py). It is seeded at the
    drone when ``None`` and leashed to ``lead_max`` horizontally and
    ``lead_max_z`` (default ``lead_max``) vertically — see ``leash``.

    Returns ``(ref, applied)``: the velocity the reference really moved with.
    It equals ``applied`` normally, but is the drone's ``measured_velocity``
    when the reference was just seeded or pulled back by the leash — the
    signal for a *scenario* profile (trajectory.ReferenceTracker) to
    re-attach to what the drone is actually doing. The teleop stick ramp
    must NOT use it: it re-attaches to what was last *published*
    (SwarmCommander.teleop_command), or it adopts the drone's measured sink
    and drift as its own command.
    """
    p = np.asarray(position, dtype=float)
    v_meas = np.asarray(measured_velocity, dtype=float).copy()
    if ref is None:
        return p.copy(), v_meas
    v_applied = np.asarray(applied, dtype=float)
    r = np.asarray(ref, dtype=float) + v_applied * dt
    lead, pulled = leash(r - p, lead_max, lead_max_z)
    if pulled:
        return p + lead, v_meas
    return r, v_applied.copy()


def ramp_velocity(profile: Optional[np.ndarray], applied: np.ndarray,
                  target: np.ndarray, accel: float, dt: float,
                  reattach_tol: float = 0.05):
    """Move a stick velocity toward ``target`` at no more than ``accel``.

    The sticks are a velocity *step*; a vehicle following a bare step has
    only its velocity loop to accelerate with (drone_2: ~4 m/s^2, bag
    run_053740), so it never reached the fence's cap in a 7.7 m box. This is
    the profile PX4's own Position mode flies (``MPC_ACC_HOR_MAX``): the
    commanded velocity ramps at ``accel`` and the ramp's acceleration goes to
    PX4 as the feedforward, so the vehicle banks with the command.

    ``profile`` is the ramp's last velocity (None = start from ``applied``);
    ``applied`` is what was actually published last tick (post-CBF,
    post-fence) — where it differs from the profile the ramp re-attaches to
    it, so a drone held back by a wall resumes with a ramp instead of a step.
    ``accel`` <= 0 disables the ramp (the target is passed through).

    Returns ``(velocity, acceleration, profile)``.
    """
    t = np.asarray(target, dtype=float)
    if accel <= 0.0:
        return t.copy(), np.zeros(3), None
    a = np.asarray(applied, dtype=float)
    p = a.copy() if profile is None else np.asarray(profile, dtype=float).copy()
    if np.linalg.norm(a - p) > reattach_tol:
        p = a.copy()
    dv = t - p
    norm = float(np.linalg.norm(dv))
    dv *= min(1.0, accel * dt / max(norm, 1e-9))
    p = p + dv
    return p.copy(), dv / max(dt, 1e-9), p
