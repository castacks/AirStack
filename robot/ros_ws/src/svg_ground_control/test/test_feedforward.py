"""The acceleration feedforward sent with a command (command_feedforward).

Bag run_020444 (three drones, random goals): every close pass (0.52, 0.65,
0.79 m against 1.1-1.3 m required) had the commanded closing speed already
at zero where the barrier says, but the drones were ~0.5 s behind their
commands because the feedforward was dropped on every CBF-corrected tick.
A corrected command now carries the rate of change of the published
command, capped at the profile's acceleration; the fence's braking
feedforward still owns the axes the wall limited.
"""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("rclpy")

from svg_ground_control.swarm_commander import command_feedforward  # noqa: E402

DT = 0.05
LIMIT = 10.0


def test_untouched_command_keeps_the_profile_feedforward() -> None:
    nominal = np.array([3.0, 0.0, 0.0])
    profile = np.array([8.0, 0.0, 0.0])
    a = command_feedforward(profile, nominal, nominal, np.array([2.6, 0.0, 0.0]),
                            nominal, np.zeros(3), DT, LIMIT)
    np.testing.assert_allclose(a, profile)


def test_cbf_corrected_command_carries_its_own_rate_of_change() -> None:
    nominal = np.array([3.0, 0.0, 0.0])
    velocity = np.array([2.0, 1.2, 0.0])       # the CBF turned the drone aside
    published = np.array([2.4, 0.9, 0.0])      # last tick's command
    a = command_feedforward(np.array([8.0, 0.0, 0.0]), nominal, velocity, published,
                            velocity, np.zeros(3), DT, LIMIT)
    np.testing.assert_allclose(a, (velocity - published) / DT)
    assert a[0] < 0.0 and a[1] > 0.0             # brake along, accelerate aside


def test_cbf_feedforward_is_capped_at_the_profile_acceleration() -> None:
    nominal = np.array([6.0, 0.0, 0.0])
    velocity = np.zeros(3)                       # emergency stop from 6 m/s
    a = command_feedforward(np.array([8.0, 0.0, 0.0]), nominal, velocity, nominal,
                            velocity, np.zeros(3), DT, LIMIT)
    assert np.linalg.norm(a) == pytest.approx(LIMIT)
    assert a[0] < 0.0


def test_cbf_holding_the_velocity_sends_no_feedforward() -> None:
    nominal = np.array([4.0, 0.0, 0.0])
    held = np.array([1.5, 0.0, 0.0])            # same corrected command as last tick
    a = command_feedforward(np.array([8.0, 0.0, 0.0]), nominal, held, held,
                            held, np.zeros(3), DT, LIMIT)
    np.testing.assert_allclose(a, 0.0)


def test_fence_limited_axis_takes_the_wall_feedforward_only_there() -> None:
    nominal = np.array([3.0, 4.0, 0.0])
    profile = np.array([5.0, 6.0, 0.0])
    clipped = np.array([3.0, 1.0, 0.0])         # the +y wall clipped y
    fence = np.array([0.0, -4.0, 0.0])
    a = command_feedforward(profile, nominal, nominal, nominal, clipped, fence, DT, LIMIT)
    np.testing.assert_allclose(a, [5.0, -4.0, 0.0])


def test_cbf_and_fence_compose_per_axis() -> None:
    nominal = np.array([3.0, 4.0, 0.0])
    velocity = np.array([1.0, 4.0, 0.0])        # CBF slowed x
    published = np.array([1.4, 4.0, 0.0])
    clipped = np.array([1.0, 1.0, 0.0])         # wall clipped y
    fence = np.array([0.0, -4.0, 0.0])
    a = command_feedforward(np.array([8.0, 0.0, 0.0]), nominal, velocity, published,
                            clipped, fence, DT, LIMIT)
    assert a[0] == pytest.approx(-0.4 / DT)
    assert a[1] == pytest.approx(-4.0)
    assert a[2] == 0.0
