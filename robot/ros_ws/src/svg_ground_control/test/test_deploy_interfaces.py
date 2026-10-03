"""Contract tests for the deploy layer (no ROS graph required)."""

from __future__ import annotations

import numpy as np

from svg_ground_control.deploy.interfaces import (
    AttitudeSetpoint,
    ObservationBuilder,
    Policy,
    RobotState,
    VelocitySetpoint,
    WaypointSetpoint,
)


def test_rotation_enu_is_identity_for_zero_quaternion() -> None:
    state = RobotState(
        position_enu=np.zeros(3),
        velocity_enu=np.zeros(3),
        orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    np.testing.assert_allclose(state.rotation_enu, np.eye(3), atol=1e-9)


def test_setpoint_payloads_round_trip() -> None:
    att = AttitudeSetpoint(np.array([0.0, 0.0, 0.0, 1.0]), 0.5)
    vel = VelocitySetpoint(np.array([1.0, 0.0, 0.0]))
    wpt = WaypointSetpoint(np.array([1.0, 2.0, 1.0]), yaw=0.0)
    assert att.thrust_norm == 0.5
    np.testing.assert_array_equal(vel.velocity_enu, [1.0, 0.0, 0.0])
    assert wpt.yaw == 0.0


def test_minimal_policy_and_builder_satisfy_protocols() -> None:
    class Builder:
        def build(self, state: RobotState):
            return state.position_enu

        def reset(self) -> None:
            pass

    class ConstantPolicy:
        def compute(self, obs) -> VelocitySetpoint:
            return VelocitySetpoint(np.zeros(3))

        def reset(self) -> None:
            pass

    assert isinstance(Builder(), ObservationBuilder)
    assert isinstance(ConstantPolicy(), Policy)

    state = RobotState(
        position_enu=np.array([1.0, 2.0, 3.0]),
        velocity_enu=np.zeros(3),
        orientation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        goal_enu=np.array([5.0, 0.0, 1.0]),
    )
    setpoint = ConstantPolicy().compute(Builder().build(state))
    assert isinstance(setpoint, VelocitySetpoint)
