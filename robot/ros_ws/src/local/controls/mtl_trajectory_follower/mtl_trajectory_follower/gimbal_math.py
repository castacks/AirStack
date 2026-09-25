"""Gimbal pointing kinematics for the MTL follower (pure Python, no ROS).

Convention — shared by ``gimbal/cmd_pitch_yaw`` and ``gimbal/state`` and by the
Isaac scene that executes them (``search_mission_scene.py``):

    geometry_msgs/Vector3  x = roll, y = pitch, z = yaw   [rad]

are the Z-Y-X (yaw, pitch, roll) Euler angles of the CAMERA frame relative to
the **earth** frame (ENU; the robot's ``map`` frame is ENU-aligned), with the
camera frame FLU-like: x = boresight, y = image-left, z = image-up. So

    R_cam = Rz(yaw) * Ry(pitch) * Rx(roll),    boresight = R_cam * [1, 0, 0]
          = (cos(yaw) cos(pitch), sin(yaw) cos(pitch), -sin(pitch))

**pitch > 0 looks down** (nadir = +pi/2; REP-103 right-hand rotation about the
left axis), yaw is counter-clockwise from East. Roll only rotates the image
about the boresight. This is exactly the task formula

    yaw_earth = atan2(dy, dx),   pitch_earth = atan2(-dz, hypot(dx, dy))

evaluated in ENU (dz < 0 for a ground point below the vehicle -> pitch > 0).

Single-axis mount (``singleAxisGimbal = true`` in mtl::planner): the camera
rides a 1-DOF cross-track axis on a bracket tilted forward by ``tau``, so the
boresight can only sweep the plane standing ``h tan(theta)`` ahead of the
airframe, ``theta = tau - dp`` with the airframe pitch nudge ``|dp| <= 5 deg``.
:func:`single_axis_command` projects a desired ground point onto that
constraint (cross-track angle ``phi`` clamped to the gimbal travel) and returns
the resulting earth-frame Euler angles.
"""

from __future__ import annotations

import math
from typing import Sequence

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]  # row-major

__all__ = [
    "wrap_pi",
    "euler_zyx_to_matrix",
    "matrix_to_euler_zyx",
    "quat_to_matrix",
    "matrix_to_quat",
    "boresight_from_euler",
    "look_at_angles",
    "two_axis_command",
    "single_axis_command",
    "camera_matrix_from_boresight",
    "slew_limit",
    "OPTICAL_FROM_GIMBAL_QUAT",
]

# camera_gimbal_link (x fwd, y left, z up) -> camera_optical_frame
# (z along the boresight, x image-right, y image-down): the ROS optical convention.
OPTICAL_FROM_GIMBAL_QUAT = (-0.5, 0.5, -0.5, 0.5)  # (x, y, z, w)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(c * c for c in v))


def _unit(v: Sequence[float]) -> Vec3:
    n = _norm(v)
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def euler_zyx_to_matrix(roll: float, pitch: float, yaw: float) -> Mat3:
    """R = Rz(yaw) Ry(pitch) Rx(roll), row-major."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def matrix_to_euler_zyx(r: Mat3, yaw_hint: float = 0.0) -> Vec3:
    """Inverse of :func:`euler_zyx_to_matrix` -> (roll, pitch, yaw).

    At the gimbal-lock singularity (boresight exactly vertical) yaw and roll are
    not separable; ``yaw_hint`` (e.g. the vehicle heading) picks the yaw so the
    decomposition stays continuous, and roll absorbs the remainder.
    """
    sp = -r[2][0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    cp = math.cos(pitch)
    if cp > 1e-6:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        yaw = yaw_hint
        # r[0][1] = cy sp sr - sy cr ; r[1][1] = sy sp sr + cy cr
        cy, sy = math.cos(yaw), math.sin(yaw)
        # rotate the second column back by -yaw: sp*sr = cy r01 + sy r11 ; cr = -sy r01 + cy r11
        spsr = cy * r[0][1] + sy * r[1][1]
        cr = -sy * r[0][1] + cy * r[1][1]
        roll = math.atan2(spsr * (1.0 if sp >= 0 else -1.0), cr)
    return (roll, pitch, yaw)


def quat_to_matrix(x: float, y: float, z: float, w: float) -> Mat3:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def matrix_to_quat(r: Mat3) -> tuple[float, float, float, float]:
    """Rotation matrix -> quaternion (x, y, z, w), w >= 0."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    return (x, y, z, w)


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))  # type: ignore[return-value]


def mat_t(a: Mat3) -> Mat3:
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def boresight_from_euler(pitch: float, yaw: float) -> Vec3:
    cp = math.cos(pitch)
    return (math.cos(yaw) * cp, math.sin(yaw) * cp, -math.sin(pitch))


def look_at_angles(pos: Sequence[float], target: Sequence[float]) -> tuple[float, float]:
    """Earth-frame (pitch, yaw) pointing from ``pos`` at ``target`` (pitch > 0 = down)."""
    dx, dy, dz = target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]
    horiz = math.hypot(dx, dy)
    return (math.atan2(-dz, horiz), math.atan2(dy, dx) if horiz > 1e-9 else 0.0)


def two_axis_command(pos: Sequence[float], target: Sequence[float],
                     yaw_hint: float = 0.0) -> Vec3:
    """2-DOF pan/tilt: roll 0 (horizon-level image), pitch/yaw straight at the target."""
    pitch, yaw = look_at_angles(pos, target)
    dx, dy = target[0] - pos[0], target[1] - pos[1]
    if math.hypot(dx, dy) < 1e-6:
        yaw = yaw_hint  # directly below: keep the image aligned with the airframe
    return (0.0, pitch, yaw)


def camera_matrix_from_boresight(b: Sequence[float], forward: Sequence[float]) -> Mat3:
    """Camera frame with x = ``b`` and y = normalize(forward x b) (image-left).

    For a nadir camera on a forward-flying airframe the image top then points
    along-track, which is how a cross-track 1-DOF mount presents the ground.
    """
    x = _unit(b)
    y = _cross(forward, x)
    if _norm(y) < 1e-9:  # boresight along the forward axis: use world up as the reference
        y = _cross((0.0, 0.0, 1.0), x)
        if _norm(y) < 1e-9:
            y = (0.0, 1.0, 0.0)
    y = _unit(y)
    z = _cross(x, y)
    # columns are the camera axes in the earth frame
    return ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))


def single_axis_command(pos: Sequence[float], target: Sequence[float], vehicle_yaw: float,
                        tilt: float, phi_max: float, pitch_nudge_max: float
                        ) -> tuple[Vec3, dict[str, float]]:
    """1-DOF cross-track mount on a forward-tilted bracket.

    Returns ``((roll, pitch, yaw), diag)`` in the earth frame, where diag carries
    the mount-frame solution: ``phi`` (cross-track angle, + right), ``theta``
    (look angle ahead of the cross-track plane), ``dp`` (pitch nudge used),
    ``phi_clipped`` / ``dp_clipped`` flags and ``miss_m`` (how far the
    constrained boresight lands from the requested ground point).
    """
    f = (math.cos(vehicle_yaw), math.sin(vehicle_yaw), 0.0)
    left = (-math.sin(vehicle_yaw), math.cos(vehicle_yaw), 0.0)
    d = (target[0] - pos[0], target[1] - pos[1], target[2] - pos[2])
    dist = _norm(d)
    if dist < 1e-9:
        d = (0.0, 0.0, -1.0)
        dist = 1.0
    du = _unit(d)
    df, dl, dd = _dot(du, f), _dot(du, left), -du[2]

    phi_req = math.atan2(-dl, dd)
    theta_req = math.atan2(df, math.hypot(dl, dd))
    phi = max(-phi_max, min(phi_max, phi_req))
    dp = max(-pitch_nudge_max, min(pitch_nudge_max, tilt - theta_req))
    theta = tilt - dp

    st, ct = math.sin(theta), math.cos(theta)
    sphi, cphi = math.sin(phi), math.cos(phi)
    b = tuple(st * f[i] - ct * sphi * left[i] + ct * cphi * (-1.0 if i == 2 else 0.0)
              for i in range(3))
    r = camera_matrix_from_boresight(b, f)
    euler = matrix_to_euler_zyx(r, yaw_hint=vehicle_yaw)

    # where the constrained boresight hits the target's height plane
    miss = float("nan")
    if b[2] < -1e-9:
        s = (target[2] - pos[2]) / b[2]
        hx, hy = pos[0] + s * b[0], pos[1] + s * b[1]
        miss = math.hypot(hx - target[0], hy - target[1])
    return euler, {
        "phi": phi, "theta": theta, "dp": dp,
        "phi_clipped": float(phi != phi_req), "dp_clipped": float(abs(tilt - theta_req) > pitch_nudge_max),
        "miss_m": miss,
    }


def slew_limit(prev: Sequence[float], target: Sequence[float], max_step: float,
               wrap: Sequence[bool] = (True, False, True)) -> Vec3:
    """Per-axis rate limit toward ``target``; wrapped axes take the short way round."""
    out = []
    for p, t, w in zip(prev, target, wrap):
        delta = wrap_pi(t - p) if w else t - p
        delta = max(-max_step, min(max_step, delta))
        v = p + delta
        out.append(wrap_pi(v) if w else v)
    return tuple(out)  # type: ignore[return-value]
