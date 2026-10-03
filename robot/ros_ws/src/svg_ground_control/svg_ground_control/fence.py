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

The same barrier serves the smaller *teleop fence* (``teleop_fence_min`` /
``teleop_fence_max`` in the commander): the box differs, the maths does not.

Why a braking envelope and not a plain ``gain * distance`` barrier
------------------------------------------------------------------
The original keep-in let the outward speed be at most ``gain × distance``,
i.e. an exponential approach to the wall with time constant ``1/gain``. Bag
``run_045417`` (drone_2, 6 m/s at a wall, gain 1, no feedforward) shows why
that overshoots by half a metre: the command starts falling 6 m out and is
zero AT the wall, but PX4's velocity loop follows a bare velocity setpoint
with ~0.1 s of transport delay and a ~0.55 s closed-loop time constant, so
the drone was still doing 1.5 m/s when it crossed and coasted 0.53 m past
(0.35-0.68 m over eleven crossings). Raising the gain makes it worse, not
better: the command drops even faster than the vehicle can follow.

``wall_speed`` therefore uses PX4's own braking law
(``trajectory.brake_speed``): the outward speed is capped at the speed from
which a stop AT the wall is still reachable with ``brake_accel`` and a lag of
``1/gain`` seconds — constant-deceleration ``sqrt(2 a d)`` far out, the old
``gain × d`` in the last stretch. That is the fastest profile the vehicle can
actually fly into the wall for a given deceleration, so the cruise speed is
kept until the true braking distance, then the brake is firm. And
``keep_in_acceleration`` hands the commander the time derivative of that
setpoint along the drone's motion, which the trajectory output sends PX4 as
its acceleration feedforward, so the vehicle brakes the moment the wall
speaks instead of a velocity-loop lag later. In the PX4 plant model of
``test_trajectory.py`` (brake 4 m/s^2, gain 2) the overshoot drops from
0.4 m at 6 m/s and 2.3 m at 8 m/s to under 0.02 m, and the drone is at rest
on the wall in 2.4-4.3 s (``test_fence_and_position_hold.py``). The gain is
also the stiffness of the last stretch: 3 rings at the wall through the
0.3 s loop delay, 2 does not.
"""

from __future__ import annotations

import numpy as np

from svg_ground_control.trajectory import brake_speed

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


def box_contains(inner_min, inner_max, outer_min, outer_max) -> bool:
    """Whether the box [inner_min, inner_max] lies inside [outer_min, outer_max]."""
    return bool(np.all(np.asarray(inner_min, dtype=float) >= np.asarray(outer_min, dtype=float))
                and np.all(np.asarray(inner_max, dtype=float) <= np.asarray(outer_max, dtype=float)))


def clamp_to_box(point: np.ndarray, fence_min: np.ndarray, fence_max: np.ndarray,
                 margin: float = 0.0) -> np.ndarray:
    """The point moved to the nearest place inside the (shrunk) box."""
    lo = np.asarray(fence_min, dtype=float) + margin
    hi = np.asarray(fence_max, dtype=float) - margin
    return np.minimum(np.maximum(np.asarray(point, dtype=float), lo), hi)


def wall_speed(distance, gain: float, brake_accel: float = 0.0) -> np.ndarray:
    """Largest outward speed allowed ``distance`` metres from a wall (elementwise).

    ``distance`` is positive inside the box and negative past the wall, and
    the result has the same sign: positive = may still move toward the wall
    this fast, negative = must move back inside at least this fast.

    With ``brake_accel`` <= 0 this is the plain barrier ``gain * distance``.
    With ``brake_accel`` > 0 it is the braking envelope: the speed from which
    a stop at the wall is reachable decelerating at ``brake_accel`` after a
    response lag of ``1/gain`` seconds (PX4's ``computeMaxSpeedFromDistance``
    with ``L = 1/gain``, see ``trajectory.brake_speed``). Near the wall this
    is ``gain * distance`` again — the tail into the wall and the push-back
    from outside are unchanged — and far out it is ``sqrt(2 a d)``, which is
    always <= ``gain * d``, so the envelope only ever *lowers* the cap.
    """
    d = np.asarray(distance, dtype=float)
    if brake_accel <= 0.0:
        return gain * d
    lag = 1.0 / float(gain)
    return np.sign(d) * brake_speed(np.abs(d), float(brake_accel), lag)


def keep_in_velocity(velocity: np.ndarray, position: np.ndarray,
                     fence_min: np.ndarray, fence_max: np.ndarray,
                     gain: float = 1.0, margin: float = 0.0,
                     brake_accel: float = 0.0) -> np.ndarray:
    """Clip a velocity so the drone stays inside the box (one axis at a time).

    On each axis the outward component may not exceed ``wall_speed`` of the
    remaining distance to the wall, so it decays to zero at the wall (a
    velocity barrier). Already past the wall, the bound is negative: the
    drone is commanded back inside. Inward motion is never limited.
    ``margin`` shrinks the box so the wall is met a little early;
    ``brake_accel`` > 0 turns the barrier into the braking envelope.

    Purely per-axis, so it composes with anything that produced ``velocity``
    (a scenario, the sticks, the CBF) and never turns a stop into motion.
    """
    v = np.array(velocity, dtype=float)
    p = np.asarray(position, dtype=float)
    lo = np.asarray(fence_min, dtype=float) + margin
    hi = np.asarray(fence_max, dtype=float) - margin
    v_max = wall_speed(hi - p, gain, brake_accel)    # largest allowed +v
    v_min = -wall_speed(p - lo, gain, brake_accel)   # most negative allowed v
    return np.minimum(np.maximum(v, v_min), v_max)


def keep_in_acceleration(velocity: np.ndarray, clipped: np.ndarray,
                         measured_velocity: np.ndarray, position: np.ndarray,
                         fence_min: np.ndarray, fence_max: np.ndarray,
                         gain: float = 1.0, margin: float = 0.0,
                         brake_accel: float = 0.0) -> np.ndarray:
    """Acceleration feedforward (3,) for the axes ``keep_in_velocity`` limited.

    On a limited axis the published velocity is the wall's setpoint
    ``wall_speed(d)``. As the drone moves at its measured velocity ``u`` the
    distance ``d`` changes and that setpoint moves with it; this is its time
    derivative — what the vehicle must do to *track* the wall's profile:

        a = -u * d(wall_speed)/dd,   d(wall_speed)/dd = brake_accel
                                                 / (|wall_speed(d)| + brake_accel / gain)

    Moving toward the wall it brakes: ``-brake_accel`` at cruise speed,
    ``-u * gain`` in the linear tail, harder than the envelope's own
    deceleration while the drone is faster than the envelope (it must catch
    up). Moving back in after an overshoot it *eases* the return (the
    setpoint is rising toward the wall), so the drone lands on the wall
    instead of bouncing — with a braking-only feedforward the PX4 plant model
    rings at the wall (2 m/s: at rest after 7.8 s instead of 3.3 s). Sent to
    PX4 as the acceleration feedforward on the trajectory output, so the
    brake starts with the command rather than a velocity-loop lag later.
    Zero on axes that were not limited and when ``brake_accel`` is 0 (the
    plain barrier has no profile to feed forward); never larger than
    ``brake_accel``.
    """
    a = np.zeros(3)
    if brake_accel <= 0.0:
        return a
    v = np.asarray(velocity, dtype=float)
    c = np.asarray(clipped, dtype=float)
    u = np.asarray(measured_velocity, dtype=float)
    p = np.asarray(position, dtype=float)
    lo = np.asarray(fence_min, dtype=float) + margin
    hi = np.asarray(fence_max, dtype=float) - margin
    lag = 1.0 / float(gain)
    for k in range(3):
        if c[k] < v[k] - 1e-9:          # limited at the +wall (hi)
            dist = hi[k] - p[k]
        elif c[k] > v[k] + 1e-9:        # limited at the -wall (lo)
            dist = p[k] - lo[k]
        else:
            continue
        # d(wall_speed)/dd, the same on both sides of the wall; the wall
        # setpoint on this axis moves at -slope * u[k] for either wall.
        setpoint = abs(float(wall_speed(dist, gain, brake_accel)))
        slope = float(brake_accel) / (setpoint + float(brake_accel) * lag)
        a[k] = float(np.clip(-slope * u[k], -brake_accel, brake_accel))
    return a
