"""Ball (or other rigid body) state from mocap pose or odometry."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from svg_ground_control.mocap_velocity import MocapVelocityEstimator


@dataclass
class BallState:
    position: np.ndarray
    velocity: np.ndarray
    stamp_s: float


class BallStateTracker:
    """Latest ball position and linear velocity (world frame)."""

    def __init__(self, velocity_alpha: float = 0.4) -> None:
        self._vel = MocapVelocityEstimator(alpha=velocity_alpha)
        self._state: BallState | None = None

    @property
    def state(self) -> BallState | None:
        return self._state

    def update_from_odometry(self, msg: Odometry) -> BallState:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        position = np.array([p.x, p.y, p.z], dtype=np.float32)
        velocity = np.array([v.x, v.y, v.z], dtype=np.float32)
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._state = BallState(position=position, velocity=velocity, stamp_s=stamp_s)
        return self._state

    def update_from_pose(self, msg: PoseStamped) -> BallState:
        p = msg.pose.position
        position = np.array([p.x, p.y, p.z], dtype=float)
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        velocity = self._vel.update(position, stamp_s).astype(np.float32)
        self._state = BallState(
            position=position.astype(np.float32),
            velocity=velocity,
            stamp_s=stamp_s,
        )
        return self._state
