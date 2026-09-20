"""The go-to-goal law (trajectory.py) against the PX4 offboard plant we measured.

Plant identified from drone_2's ULogs of 2026-09-20 (px4_logs/): ground ->
vehicle link delay 0.15 s, PX4 velocity loop on acceleration P=1.8 I=0.4
(MPC_XY_VEL_P_ACC / I_ACC), attitude lag 0.1 s, |a| <= 8 m/s^2, position loop
P=0.95 (MPC_XY_P) when a position setpoint is given, acceleration setpoint
added as feedforward (PositionControl.cpp). The old P-law
(v = min(v_max, 1.5 d) as a bare velocity) overshot a 7 m / 5 m/s leg by
1.0 m in the bag drone_2_auto_goal_0920_192853; the reference-trajectory
output must not.
"""

from __future__ import annotations

import numpy as np
import pytest

from svg_ground_control.position_hold import advance_reference
from svg_ground_control.trajectory import (ReferenceTracker, brake_speed,
                                           seek_velocity, stopping_distance)

DT = 0.05


# ---------------------------------------------------------------- the law

def test_brake_speed_is_zero_at_the_goal_and_monotone():
    assert brake_speed(0.0, 3.0, 0.3) == 0.0
    d = np.linspace(0.0, 10.0, 200)
    v = brake_speed(d, 3.0, 0.3)
    assert np.all(np.diff(v) > 0.0)


def test_brake_speed_limits():
    # Far out: constant deceleration sqrt(2 a d). Close in: d / settle.
    assert brake_speed(500.0, 3.0, 0.3) == pytest.approx(np.sqrt(2 * 3 * 500), rel=0.02)
    assert brake_speed(0.01, 3.0, 0.3) == pytest.approx(0.01 / 0.3, rel=0.05)


def test_stopping_distance_inverts_brake_speed():
    for v in (0.3, 1.2, 5.0):
        d = stopping_distance(v, 3.0, 0.3)
        assert brake_speed(d, 3.0, 0.3) == pytest.approx(v)


def test_seek_velocity_caps_points_and_stops():
    p = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    g = np.array([[8.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    v = seek_velocity(p, g, 1.2)
    assert v[0] == pytest.approx([1.2, 0.0, 0.0])
    assert v[1] == pytest.approx([0.0, 0.0, 0.0])
    # per-drone speeds, shape (N,)
    v = seek_velocity(p, g, np.array([0.5, 2.0]))
    assert np.linalg.norm(v[0]) == pytest.approx(0.5)


# ------------------------------------------------------------ the tracker

def test_tracker_ramps_at_the_acceleration_limit_to_cruise():
    t = ReferenceTracker(1, accel=3.0, settle=0.3)
    ref = np.array([[0.0, 0.0, 1.0]])
    goal = np.array([[20.0, 0.0, 1.0]])
    last = np.zeros(3)
    for _ in range(40):
        v, a = t.step(ref, goal, 5.0, DT)
        assert np.linalg.norm(v[0] - last) <= 3.0 * DT + 1e-9
        last = v[0]
        ref = ref + v * DT
    assert v[0, 0] == pytest.approx(5.0)          # cruise reached (1.67 s)
    assert a[0] == pytest.approx([0.0, 0.0, 0.0])  # and no more acceleration


def test_tracker_reattaches_to_the_applied_velocity():
    t = ReferenceTracker(1, accel=3.0, settle=0.3)
    ref = np.array([[0.0, 0.0, 1.0]])
    goal = np.array([[20.0, 0.0, 1.0]])
    for _ in range(40):
        t.step(ref, goal, 5.0, DT)
    # the CBF held the drone: what was applied is zero -> the profile
    # restarts from zero instead of asking for cruise again
    v, _ = t.step(ref, goal, 5.0, DT, applied=np.zeros((1, 3)))
    assert np.linalg.norm(v[0]) == pytest.approx(3.0 * DT)
    t.reset()
    assert np.all(t.velocity == 0.0)


# -------------------------------------------------- closed loop with PX4

class Px4OffboardPlant:
    """1-D model of the measured PX4 offboard behaviour (see module doc)."""

    def __init__(self, link_delay=0.15, meas_delay=0.05):
        self.p = self.v = self.a = self.i = 0.0
        self.link = [(np.nan, 0.0, 0.0)] * (int(round(link_delay / DT)) + 1)
        self.meas = [(0.0, 0.0)] * (int(round(meas_delay / DT)) + 1)

    def measure(self):
        self.meas.append((self.p, self.v))
        return self.meas.pop(0)

    def command(self, pos_sp, vel_sp, acc_ff):
        self.link.append((pos_sp, vel_sp, acc_ff))
        pos_sp, vel_sp, acc_ff = self.link.pop(0)
        vsp = vel_sp + (0.0 if np.isnan(pos_sp) else 0.95 * (pos_sp - self.p))
        err = vsp - self.v
        self.i = float(np.clip(self.i + 0.4 * err * DT, -3.0, 3.0))
        asp = float(np.clip(acc_ff + 1.8 * err + self.i, -8.0, 8.0))
        self.a += (asp - self.a) * DT / 0.1
        self.v += self.a * DT
        self.p += self.v * DT


def fly_trajectory(goal, speed, accel=3.0, settle=0.3, lead=2.0,
                   hold=None, seconds=12.0):
    """The commander's real-drone loop: reference point + profile -> PX4."""
    plant = Px4OffboardPlant()
    tracker = ReferenceTracker(1, accel, settle)
    ref = None
    applied = np.zeros(3)
    log = []
    for k in range(int(seconds / DT)):
        t = k * DT
        pm, vm = plant.measure()
        ref, applied = advance_reference(ref, [pm, 0, 0], applied, [vm, 0, 0], DT, lead)
        v, a = tracker.step(ref[None], [[goal, 0, 0]], speed, DT, applied[None])
        v, a = v[0], a[0]
        if hold and hold[0] < t < hold[1]:       # a CBF holding the drone
            v, a = np.zeros(3), np.zeros(3)
        plant.command(ref[0], v[0], a[0])
        applied = v.copy()
        log.append((t, plant.p, plant.v))
    return np.array(log)


def fly_velocity_only(goal, speed, accel=3.0, settle=1.0, seconds=12.0):
    """The sim/MAVROS loop: stateless law at the drone, bare velocity."""
    plant = Px4OffboardPlant()
    log = []
    for k in range(int(seconds / DT)):
        pm, _ = plant.measure()
        v = seek_velocity([[pm, 0, 0]], [[goal, 0, 0]], speed, accel, settle)[0]
        plant.command(np.nan, v[0], 0.0)
        log.append((k * DT, plant.p, plant.v))
    return np.array(log)


def fly_old_p_law(goal, speed, gain=1.5, seconds=12.0):
    plant = Px4OffboardPlant()
    log = []
    for k in range(int(seconds / DT)):
        pm, _ = plant.measure()
        plant.command(np.nan, float(np.clip(gain * (goal - pm), -speed, speed)), 0.0)
        log.append((k * DT, plant.p, plant.v))
    return np.array(log)


def overshoot(log, goal):
    return float((log[:, 1] - goal).max())


def settle_time(log, goal, tol=0.1):
    err = np.abs(log[:, 1] - goal)
    for k in range(len(err)):
        if err[k:].max() < tol:
            return log[k, 0]
    return None


@pytest.mark.parametrize('goal, speed, max_over, max_reach, max_settle', [
    (7.0, 5.0, 0.10, 4.0, 5.5),   # the flight in the bag: 1.0 m over, 4.7 s
    (3.5, 5.0, 0.10, 3.0, 4.5),
    (1.5, 1.2, 0.05, 2.0, 3.0),   # was 0.39 m over, 5.2 s
    (0.5, 1.2, 0.05, 1.2, 2.0),
])
def test_trajectory_output_stops_on_the_goal(goal, speed, max_over, max_reach,
                                             max_settle):
    log = fly_trajectory(goal, speed)
    assert overshoot(log, goal) < max_over
    reach = next((t for t, p, _ in log if abs(p - goal) < 0.15), None)
    assert reach is not None and reach < max_reach
    s = settle_time(log, goal)          # the tail is PX4's own position loop
    assert s is not None and s < max_settle
    # PX4's position loop may briefly add to the feedforward, but the
    # cruise cap is respected within the tracking error it corrects.
    assert log[:, 2].max() <= speed * 1.25


def test_trajectory_output_resumes_cleanly_after_a_cbf_hold():
    log = fly_trajectory(7.0, 5.0, hold=(1.0, 2.5))
    assert overshoot(log, 7.0) < 0.15
    assert settle_time(log, 7.0) is not None


def test_velocity_only_law_is_tame_where_the_old_p_law_overshot():
    # No feedforward and no onboard position loop: the plant lags ~0.7 s.
    # The stateless law brakes early enough to keep the overshoot small;
    # the old 1.5 P-ramp did not.
    for goal, speed, limit in ((1.5, 1.2, 0.35), (7.0, 5.0, 0.9)):
        assert overshoot(fly_velocity_only(goal, speed), goal) < limit
    assert overshoot(fly_old_p_law(7.0, 5.0), 7.0) > 1.0
    assert overshoot(fly_old_p_law(1.5, 1.2), 1.5) > 0.3


def test_reference_is_leashed_and_reattaches():
    ref, applied = advance_reference(None, [1, 2, 3], [9, 9, 9], [0.1, 0, 0], DT, 2.0)
    assert ref == pytest.approx([1, 2, 3])            # seeded at the drone
    assert applied == pytest.approx([0.1, 0, 0])      # moved with the drone
    ref, applied = advance_reference([1, 2, 3], [1, 2, 3], [1, 0, 0], [0.5, 0, 0], DT, 2.0)
    assert ref == pytest.approx([1.05, 2, 3])          # integrated
    assert applied == pytest.approx([1, 0, 0])
    ref, applied = advance_reference([5, 2, 3], [1, 2, 3], [1, 0, 0], [0.5, 0, 0], DT, 2.0)
    assert ref == pytest.approx([3, 2, 3])             # leashed to 2 m
    assert applied == pytest.approx([0.5, 0, 0])       # -> re-attach signal
