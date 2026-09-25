"""Kinematics of the native MTL camera gimbal (stdlib only).

Command / state convention (identical to mtl_trajectory_follower.gimbal_math):
``geometry_msgs/Vector3`` x = roll, y = pitch, z = yaw [rad] are the Z-Y-X Euler
angles of the camera frame (x = boresight, y = image-left, z = image-up) in the
EARTH frame (Isaac world ENU): ``R = Rz(yaw) Ry(pitch) Rx(roll)``; pitch > 0 looks
down, nadir = +pi/2. The mount is earth-stabilised (a real 3-axis gimbal's
behaviour), so the camera orientation does not follow airframe attitude; its
POSITION rides the airframe: ``p_cam = p_body + R_body * mount_offset``.

The USD camera looks along its local -Z with +Y up, so the camera prim carries a
fixed rotation :data:`USD_CAMERA_IN_GIMBAL_QUAT` under the gimbal Xform.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

Quat = tuple[float, float, float, float]  # (x, y, z, w)
Vec3 = tuple[float, float, float]

__all__ = ["USD_CAMERA_IN_GIMBAL_QUAT", "quat_mul", "quat_conj", "quat_rotate", "quat_from_euler_zyx",
           "euler_zyx_from_quat", "quat_from_matrix", "gimbal_world_pose", "relative_pose",
           "focal_length_mm", "GimbalLimits", "GimbalAxis"]


def quat_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def quat_norm(q: Quat) -> Quat:
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)  # type: ignore[return-value]


def quat_rotate(q: Quat, v: Sequence[float]) -> Vec3:
    p = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), quat_conj(q))
    return (p[0], p[1], p[2])


def quat_from_euler_zyx(roll: float, pitch: float, yaw: float) -> Quat:
    """q = qz(yaw) * qy(pitch) * qx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def euler_zyx_from_quat(q: Quat) -> Vec3:
    x, y, z, w = quat_norm(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


def quat_from_matrix(m: Sequence[Sequence[float]]) -> Quat:
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)
    return quat_norm(q if q[3] >= 0 else tuple(-c for c in q))  # type: ignore[arg-type]


# USD camera axes expressed in the gimbal frame (columns): camera +X = gimbal -Y
# (image right), camera +Y = gimbal +Z (image up), camera +Z = gimbal -X (the USD
# camera looks down its -Z, i.e. along the gimbal boresight +X).
USD_CAMERA_IN_GIMBAL_QUAT: Quat = quat_from_matrix(((0.0, 0.0, -1.0),
                                                     (-1.0, 0.0, 0.0),
                                                     (0.0, 1.0, 0.0)))


def gimbal_world_pose(body_pos: Sequence[float], body_quat: Quat, mount_offset: Sequence[float],
                      rpy: Sequence[float]) -> tuple[Vec3, Quat]:
    """World pose of the (earth-stabilised) gimbal frame."""
    off = quat_rotate(quat_norm(body_quat), mount_offset)
    pos = (body_pos[0] + off[0], body_pos[1] + off[1], body_pos[2] + off[2])
    return pos, quat_from_euler_zyx(*rpy)


def relative_pose(parent_pos: Sequence[float], parent_quat: Quat, child_pos: Sequence[float],
                  child_quat: Quat) -> tuple[Vec3, Quat]:
    """Child pose expressed in the parent frame: T_parent^-1 * T_child."""
    inv = quat_conj(quat_norm(parent_quat))
    d = (child_pos[0] - parent_pos[0], child_pos[1] - parent_pos[1], child_pos[2] - parent_pos[2])
    return quat_rotate(inv, d), quat_norm(quat_mul(inv, child_quat))


def focal_length_mm(fov_deg: float, horizontal_aperture_mm: float = 20.955) -> float:
    """Pinhole focal length giving a horizontal field of view of ``fov_deg``."""
    return horizontal_aperture_mm / (2.0 * math.tan(math.radians(fov_deg) / 2.0))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class GimbalLimits:
    roll_min: float = math.radians(-80.0)
    roll_max: float = math.radians(80.0)
    pitch_min: float = math.radians(-20.0)
    pitch_max: float = math.radians(110.0)
    slew_rate: float = math.radians(120.0)   # [rad/s] per axis


class GimbalAxis:
    """Actuator model: clamp the command to the travel, slew toward it at a bounded rate.

    ``state`` is what the camera actually points at and what gimbal/state reports.
    """

    def __init__(self, limits: GimbalLimits, initial: Sequence[float]) -> None:
        self.limits = limits
        self.state = self.clamp(initial)

    def clamp(self, rpy: Sequence[float]) -> Vec3:
        lim = self.limits
        return (min(max(rpy[0], lim.roll_min), lim.roll_max),
                min(max(rpy[1], lim.pitch_min), lim.pitch_max),
                _wrap(rpy[2]))

    def step(self, command: Sequence[float], dt: float) -> Vec3:
        target = self.clamp(command)
        if dt <= 0.0:
            return self.state
        m = self.limits.slew_rate * dt
        out = []
        for i, (s, t) in enumerate(zip(self.state, target)):
            d = _wrap(t - s) if i == 2 else t - s
            d = max(-m, min(m, d))
            out.append(_wrap(s + d) if i == 2 else s + d)
        self.state = tuple(out)  # type: ignore[assignment]
        return self.state
