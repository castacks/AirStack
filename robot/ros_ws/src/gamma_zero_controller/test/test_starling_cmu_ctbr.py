from pathlib import Path

import numpy as np
import pytest

from gamma_zero_controller.ctbr import CTBRConfig, CommandGate, action_to_ctbr, airstack_to_px4, ctbr_to_airstack, px4_state_to_policy


def test_hover_units_and_flu_frd_boundary():
    physical = action_to_ctbr([0., .4, -.3, .2])
    assert physical[0] == pytest.approx(.602*9.81)
    mapped = ctbr_to_airstack(physical)
    assert mapped["normalized_collective"] == pytest.approx(CTBRConfig().hover_normalized)
    px4 = airstack_to_px4(mapped["rates_flu"], mapped["normalized_collective"])
    np.testing.assert_allclose([px4["roll"], px4["pitch"], px4["yaw"]], [.4, .3, -.2])
    np.testing.assert_allclose(px4["thrust_body"], [0., 0., -CTBRConfig().hover_normalized])
    assert px4["offboard"]["body_rate"] and sum(px4["offboard"].values()) == 1


@pytest.mark.parametrize("command", [[float("nan"), 0, 0, 0], [6, float("inf"), 0, 0], [-1, 0, 0, 0], [6, 0, 0]])
def test_invalid_command_rejected(command):
    with pytest.raises(ValueError):
        ctbr_to_airstack(command)


def test_limits_are_reported_and_never_wrap_axes():
    mapped = ctbr_to_airstack([100., 10., -10., 10.])
    assert mapped["saturated"]
    np.testing.assert_allclose(mapped["rates_flu"], [np.deg2rad(130), -np.deg2rad(130), 2.6])
    assert 0 < mapped["normalized_collective"] < 1


def test_ned_state_uses_body_velocity_and_projected_gravity():
    state = px4_state_to_policy([1., 0., 0., 0.], [2., 3., -1.], [4., 5., -6.], [.1, .2, .3])
    np.testing.assert_allclose(state["position_enu"], [3., 2., 1.])
    np.testing.assert_allclose(state["linear_velocity_flu"], [4., -5., 6.])
    np.testing.assert_allclose(state["angular_velocity_flu"], [.1, -.2, -.3])
    np.testing.assert_allclose(state["projected_gravity_flu"], [0., 0., -1.])
    state = px4_state_to_policy([np.sqrt(.5), 0., 0., np.sqrt(.5)], [0, 0, 0], [0, 1, 0], [0, 0, 0])
    np.testing.assert_allclose(state["linear_velocity_flu"], [1, 0, 0], atol=1e-14)


def test_fault_latches_and_fresh_messages_do_not_reenable():
    gate = CommandGate()
    gate.update_command([6, 0, 0, 0], 100., 10.)
    gate.update_state([1, 0, 0, 0], [0, 0, -1], [0, 0, 0], [0, 0, 0], 100., 10.)
    assert gate.output(100., 10.) is None
    assert gate.enable(100., 10.)
    assert gate.output(100.05, 10.05) is not None
    assert gate.output(100.11, 10.11) is None and not gate.enabled
    gate.update_command([6, 0, 0, 0], 100.12, 10.12)
    gate.update_state([1, 0, 0, 0], [0, 0, -1], [0, 0, 0], [0, 0, 0], 100.12, 10.12)
    assert gate.output(100.12, 10.12) is None
    assert gate.enable(100.12, 10.12)
    assert gate.output(100.12, 10.12, exclusive=False) is None


@pytest.mark.parametrize("stamp,received", [(99., 10.), (101., 10.), (100., 9.), (100., 11.)])
def test_stale_queued_future_and_receipt_clock_errors_rejected(stamp, received):
    gate = CommandGate()
    gate.update_command([6, 0, 0, 0], stamp, received)
    gate.update_state([1, 0, 0, 0], [0, 0, -1], [0, 0, 0], [0, 0, 0], 100., 10.)
    assert not gate.enable(100., 10.)


def test_repeated_source_stamp_and_invalid_state_latch():
    gate = CommandGate()
    gate.update_command([6, 0, 0, 0], 100., 10.)
    with pytest.raises(ValueError):
        gate.update_command([6, 0, 0, 0], 100., 10.01)
    assert gate.command is None
    with pytest.raises(ValueError):
        gate.update_state([0, 0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 100., 10.)
    with pytest.raises(ValueError):
        gate.update_state([1, 0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 100., 10., velocity_frame=2)
