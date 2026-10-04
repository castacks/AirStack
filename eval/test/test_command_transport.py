"""CPU contract checks for SAFE action frames and exclusive AirStack control."""

import math
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from eval.envs.AirstackStackEnv import AirstackStackEnv


class FakeBridge:
    def __init__(self):
        self.calls = []

    def publish_setpoint(self, *args):
        self.calls.append(('setpoint', args))

    def publish_attitude_thrust(self, *args):
        self.calls.append(('attitude', args))

    def clear_external_command(self):
        self.calls.append(('clear', ()))

    def cancel_navigate(self):
        self.calls.append(('cancel', ()))


def _env():
    env = object.__new__(AirstackStackEnv)
    bridge = FakeBridge()
    gate = []
    modes = []
    env.stack = SimpleNamespace(
        bridge=bridge, set_pid_output=gate.append,
        set_trajectory_mode=modes.append)
    env._ros_offset = np.array([10., 20., 1.])
    env._external_control_active = False
    return env, bridge, gate, modes


def test_full_planner_setpoint_keeps_velocity_acceleration_and_yaw():
    env, bridge, _, _ = _env()
    env._apply_setpoint_action({
        'position': [15., 27., 3.], 'velocity': [1., 2., 3.],
        'acceleration': [0.1, 0.2, 0.3], 'heading': math.pi / 4,
        'heading_rate': None,
    })
    name, args = bridge.calls[-1]
    assert name == 'setpoint'
    np.testing.assert_allclose(args[0], [5., 7., 2.])
    np.testing.assert_allclose(args[1], [1., 2., 3.])
    np.testing.assert_allclose(args[2], [0.1, 0.2, 0.3])
    assert args[3] == math.pi / 4
    assert args[4] is None


def test_safe_ned_frd_attitude_becomes_ros_enu_flu():
    env, bridge, _, _ = _env()
    # A level NED/FRD identity orientation points north; in ENU/FLU its
    # body x axis also points north and its body y axis points west.
    env._apply_attitude_action({'attitude': [1., 0., 0., 0.], 'thrust': 0.48})
    name, args = bridge.calls[-1]
    assert name == 'attitude'
    rot = Rotation.from_quat(args[0]).as_matrix()
    np.testing.assert_allclose(rot[:, 0], [0., 1., 0.], atol=1e-7)
    np.testing.assert_allclose(rot[:, 1], [-1., 0., 0.], atol=1e-7)
    assert args[1] == 0.48


def test_episode_hold_restores_pid_before_trajectory_hold():
    env, bridge, gate, modes = _env()
    env._external_control_active = True
    env._hold_position()
    assert bridge.calls[:2] == [('clear', ()), ('cancel', ())]
    assert gate == [True]
    assert modes == [1]
    assert env._external_control_active is False
