"""Contract shared by every policy deployment.

Each control tick the commander assembles a RobotState, an ObservationBuilder
turns it into the policy's observation, the Policy returns a Setpoint, and the
commander publishes it. A new policy plugs in by supplying these two protocols
without changing commander code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, Union, runtime_checkable

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class RobotState:
    """World-frame state of one drone at a single control tick."""

    position_enu: np.ndarray
    velocity_enu: np.ndarray
    orientation_xyzw: np.ndarray
    goal_enu: Union[np.ndarray, None] = None
    perception: Union[np.ndarray, None] = None

    @property
    def rotation_enu(self) -> np.ndarray:
        """FLU-body to ENU-world rotation matrix from the orientation quaternion."""
        return Rotation.from_quat(self.orientation_xyzw).as_matrix()


@dataclass
class AttitudeSetpoint:
    """ENU/FLU attitude quaternion [x, y, z, w] with normalized thrust in [0, 1]."""

    attitude_enu_flu_xyzw: np.ndarray
    thrust_norm: float


@dataclass
class VelocitySetpoint:
    """World-ENU linear velocity setpoint in meters per second."""

    velocity_enu: np.ndarray


@dataclass
class WaypointSetpoint:
    """Absolute world-ENU position setpoint with optional ENU yaw in radians."""

    position_enu: np.ndarray
    yaw: Union[float, None] = None


Setpoint = Union[AttitudeSetpoint, VelocitySetpoint, WaypointSetpoint]


@runtime_checkable
class ObservationBuilder(Protocol):
    """Turns a RobotState into the observation a specific policy was trained on."""

    def build(self, state: RobotState) -> Any:
        ...

    def reset(self) -> None:
        ...


@runtime_checkable
class Policy(Protocol):
    """Maps an observation to a Setpoint for one drone."""

    def compute(self, obs: Any) -> Setpoint:
        ...

    def reset(self) -> None:
        ...
