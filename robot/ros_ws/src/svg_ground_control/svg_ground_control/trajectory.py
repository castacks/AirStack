"""Go-to-goal velocity references a PX4 drone can actually follow.

Why this exists (ULogs of 2026-09-20, drone_2 at 5 m/s, see experiment.md):

* PX4 Position mode stops from 5.8 m/s in 4.2 m with 0.35 m overshoot because
  it flies a *trajectory*: the stick is turned into an acceleration-limited
  velocity setpoint that is fed to the vehicle together with its acceleration
  (feedforward), and the position is locked once the setpoint reaches zero.
  With the feedforward the velocity loop tracks with ~0.1 s of lag.
* The old ground controller sent ``v = min(v_max, 1.5 * distance)`` as a bare
  velocity setpoint. That is a step to full speed (infinite acceleration
  requested, tilt saturates, the velocity integrator winds up) followed by an
  exponential ramp that assumes the vehicle follows instantly. Without an
  acceleration feedforward PX4's velocity loop lags 0.15 s + 0.55 s, so the
  drone was still doing 3.4 m/s when the command had dropped to 0.5 m/s and
  overshot the goal by 1.0 m (bag drone_2_auto_goal_0920_192853).

So the commander now does what PX4 does internally:

1. :func:`brake_speed` — the speed from which a stop ``distance`` away is
   still reachable with ``accel`` and a first-order lag ``settle``:
   ``v = -a*L + sqrt((a*L)^2 + 2*a*d)`` (PX4's ``computeMaxSpeedFromDistance``
   with ``L = 2a/jerk``). Far out this is constant deceleration
   ``sqrt(2ad)``, close in it is ``d / L`` — an exponential tail with time
   constant ``settle`` instead of a hard stop.
2. :class:`ReferenceTracker` — a per-drone velocity reference that moves
   toward that speed with bounded acceleration, so the setpoint is a profile
   the vehicle can track, and reports the acceleration for the feedforward.
   It is evaluated at the commander's *reference point* (where the drone was
   told to be, integrated from the published velocity), not at the drone, so
   the trajectory is deterministic and the drone's own lag does not shorten
   the braking distance.
3. :func:`seek_velocity` — the stateless version, evaluated at the drone with
   a larger ``settle`` that absorbs the velocity-loop lag, for drones that can
   only take a velocity setpoint (the MAVROS sim path).

The reference point, the leash that keeps it from running away from a blocked
drone, and the position/velocity/acceleration output to PX4 live in the
commander (``swarm_commander.py``); position_hold.py holds the leash maths.
"""

from __future__ import annotations

import numpy as np

DEFAULT_ACCEL = 3.0                  # m/s^2, PX4's MPC_ACC_HOR default
DEFAULT_SETTLE = 0.3                 # s, exponential tail of the reference
DEFAULT_VELOCITY_ONLY_SETTLE = 1.0   # s, same law at the drone, lag absorbed
DEFAULT_LEAD = 2.0                   # m, reference leash (PX4: MPC_XY_ERR_MAX)
REATTACH_TOL = 0.05                  # m/s, applied != reference -> re-attach


def brake_speed(distance, accel: float, settle: float):
    """Fastest speed from which a stop ``distance`` away is reachable.

    ``accel`` is the deceleration the vehicle will be asked for (m/s^2),
    ``settle`` the lag of whoever follows the command (s). Elementwise.
    """
    d = np.maximum(np.asarray(distance, dtype=float), 0.0)
    a_l = float(accel) * float(settle)
    return -a_l + np.sqrt(a_l * a_l + 2.0 * float(accel) * d)


def stopping_distance(speed: float, accel: float, settle: float) -> float:
    """Inverse of :func:`brake_speed`: how far a goal must be to reach ``speed``."""
    v = max(0.0, float(speed))
    return v * v / (2.0 * float(accel)) + v * float(settle)


def seek_velocity(positions, goals, max_speed, accel: float = DEFAULT_ACCEL,
                  settle: float = DEFAULT_VELOCITY_ONLY_SETTLE) -> np.ndarray:
    """Stateless go-to-goal velocity, capped at ``max_speed``, braking law at
    the drone. ``max_speed`` may be a scalar or shape (N,)/(N,1).

    This is what velocity-only drones fly. With the default ``settle`` of
    1.0 s it is close to the old ``1.0 * distance`` P-law near the goal but
    brakes from high speed at constant deceleration instead of an exponential
    that no lagging vehicle can follow.
    """
    positions = np.asarray(positions, dtype=float)
    goals = np.asarray(goals, dtype=float)
    to_goal = goals - positions
    distance = np.linalg.norm(to_goal, axis=-1, keepdims=True)
    cap = np.asarray(max_speed, dtype=float)
    if cap.ndim == 1:
        cap = cap[:, None]
    speed = np.minimum(cap, brake_speed(distance, accel, settle))
    direction = to_goal / np.maximum(distance, 1e-9)
    return direction * speed


class ReferenceTracker:
    """Acceleration-limited velocity references toward per-drone goals.

    ``step`` moves each drone's reference velocity toward the braking-law
    speed at its reference point by at most ``accel * dt`` and returns the
    velocity and the acceleration it used (the feedforward). ``applied`` is
    the velocity the reference point actually moved with since the last step
    (what the commander published after the CBF and fence, or the measured
    velocity after a leash re-seed); where it differs from the reference the
    reference is re-attached to it, so a drone that was held back resumes
    with a ramp instead of a step, and a CBF speed cap is never exceeded by
    more than one tick of acceleration.
    """

    def __init__(self, num_drones: int, accel: float = DEFAULT_ACCEL,
                 settle: float = DEFAULT_SETTLE) -> None:
        self.num_drones = int(num_drones)
        self.accel = float(accel)
        self.settle = float(settle)
        self.velocity = np.zeros((self.num_drones, 3))
        self.acceleration = np.zeros((self.num_drones, 3))

    def reset(self) -> None:
        self.velocity[:] = 0.0
        self.acceleration[:] = 0.0

    def step(self, references, goals, speeds, dt: float, applied=None):
        refs = np.asarray(references, dtype=float).reshape(self.num_drones, 3)
        goals = np.asarray(goals, dtype=float).reshape(self.num_drones, 3)
        speeds = np.broadcast_to(np.asarray(speeds, dtype=float).reshape(-1),
                                 (self.num_drones,)).astype(float)
        dt = float(dt)
        if applied is not None:
            applied = np.asarray(applied, dtype=float).reshape(self.num_drones, 3)
            moved = np.linalg.norm(applied - self.velocity, axis=1) > REATTACH_TOL
            self.velocity[moved] = applied[moved]
        to_goal = goals - refs
        distance = np.linalg.norm(to_goal, axis=-1, keepdims=True)
        speed = np.minimum(speeds[:, None],
                           brake_speed(distance, self.accel, self.settle))
        desired = to_goal / np.maximum(distance, 1e-9) * speed
        dv = desired - self.velocity
        norm = np.linalg.norm(dv, axis=-1, keepdims=True)
        scale = np.minimum(1.0, self.accel * dt / np.maximum(norm, 1e-9))
        dv = dv * scale
        self.acceleration = dv / max(dt, 1e-9)
        self.velocity = self.velocity + dv
        return self.velocity.copy(), self.acceleration.copy()
