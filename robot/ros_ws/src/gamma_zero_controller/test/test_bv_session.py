import numpy as np
import pytest

from gamma_zero_controller.session import FlightSession


class Policy:
    assets = {"max_command_velocity": [1., 1., 1., 1.]}


    def reset_history(self):
        self.age = 0


    def act(self, state, goal, previous):
        self.age += 1
        return np.zeros(4), np.array([5.90562, 0., 0., 0.])


@pytest.fixture
def session():
    s = FlightSession(Policy(), {"position_min_enu": [-4., -4., -1.], "position_max_enu": [4., 4., 4.],
        "body_origin_to_imu_flu": [0., 0., 0.], "state_timeout_s": .1, "max_tick_gap_s": .06,
        "max_tilt_deg": 20., "max_tracking_error_m": .5, "completion_hold_s": 30.})
    s.update_state([1., 0., 0., 0.], [0., 0., -1.], [0.]*3, [0.]*3, 100., 10., 0)
    return s


def test_stale_state_fault_latches_until_manual_restart(session):
    session.start(100., 10.)
    assert session.step(100.02, 10.02)["phase"] == "settling"
    with pytest.raises(ValueError, match="Stale"):
        session.step(100.11, 10.11)
    assert not session.active
    session.update_state([1., 0., 0., 0.], [0., 0., -1.], [0.]*3, [0.]*3, 100.12, 10.12, 0)
    assert session.step(100.12, 10.12) is None
    session.start(100.12, 10.12)
    assert session.policy.age == 0
    assert session.step(100.12, 10.12) is not None


def test_estimator_reset_disables_and_resets_history(session):
    session.start(100., 10.)
    session.step(100.02, 10.02)
    session.update_state([1., 0., 0., 0.], [0., 0., -1.], [0.]*3, [0.]*3, 100.03, 10.03, 1)
    assert not session.active and session.policy.age == 0
    assert "estimator reset" in session.fault


def test_missed_tick_rejected_even_with_fresh_state(session):
    session.start(100., 10.)
    session.update_state([1., 0., 0., 0.], [0., 0., -1.], [0.]*3, [0.]*3, 100.08, 10.08, 0)
    with pytest.raises(ValueError, match="deadline"):
        session.step(100.08, 10.08)
    assert not session.active


def test_lever_arm_velocity_sign(session):
    session.lever = np.array([.1, 0., 0.])
    session.update_state([1., 0., 0., 0.], [0., 0., -1.], [0.]*3, [0., 0., -2.], 100.01, 10.01, 0)
    np.testing.assert_allclose(session.state["linear_velocity_flu"], [0., .2, 0.])


def test_route_must_fit_with_margin(session):
    session.low[0] = -.2
    with pytest.raises(ValueError, match="route"):
        session.start(100., 10.)
    assert not session.active


def test_source_clock_is_not_replaced_by_receipt_time(session):
    with pytest.raises(ValueError, match="clocks"):
        session.start(10000., 10.)
    assert not session.active


def test_completion_hold_is_bounded(session):
    session.start(100., 10.)
    t = session.trajectory.duration_s+31.
    session.update_state([1., 0., 0., 0.], [0., 0., -1.], [0.]*3, [0.]*3, 100.+t, 10.+t, 0)
    session.last_tick_s = 10.+t-.02
    with pytest.raises(ValueError, match="Completion hold"):
        session.step(100.+t, 10.+t)
    assert not session.active


def test_duplicate_start_does_not_interrupt_active_session(session):
    session.start(100., 10.)
    with pytest.raises(ValueError, match="already active"):
        session.start(100.01, 10.01)
    assert session.active
