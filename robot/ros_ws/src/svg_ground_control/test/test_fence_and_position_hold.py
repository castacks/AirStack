"""fence.keep_in_velocity / clamp_to_box and position_hold target logic."""

import numpy as np
import pytest

from svg_ground_control.fence import (BEHAVIORS, clamp_to_box, keep_in_velocity,
                                      outside, violation_text)
from svg_ground_control.position_hold import advance_target, tracking_velocity

LO = np.array([-2.0, -2.0, 0.0])
HI = np.array([2.0, 2.0, 3.0])


def test_behaviors_listed():
    assert BEHAVIORS == ('hold_all', 'keep_in')


def test_outside_and_text():
    assert not outside([0, 0, 1], LO, HI).any()
    assert outside([2.5, 0, 1], LO, HI).tolist() == [True, False, False]
    assert violation_text([2.5, -3.0, 1], LO, HI) == 'x>max, y<min'


def test_keep_in_inside_far_from_walls_is_untouched():
    v = np.array([1.0, -1.0, 0.5])
    np.testing.assert_allclose(keep_in_velocity(v, [0, 0, 1], LO, HI, gain=1.0), v)


def test_keep_in_decays_outward_speed_to_zero_at_the_wall():
    # 0.3 m from the +x wall, gain 1 -> at most 0.3 m/s outward; inward free.
    v = keep_in_velocity([1.0, 0.0, 0.0], [1.7, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(0.3)
    v = keep_in_velocity([-1.0, 0.0, 0.0], [1.7, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(-1.0)
    v = keep_in_velocity([1.0, 0.0, 0.0], [2.0, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(0.0)


def test_keep_in_pushes_back_when_already_outside():
    # 0.5 m past the +x wall: even a stick pushing outward becomes -0.5 m/s.
    v = keep_in_velocity([1.0, 0.0, 0.0], [2.5, 0, 1], LO, HI, gain=1.0)
    assert v[0] == pytest.approx(-0.5)
    # ...and a stop (zero) becomes the same push-back, never the reverse.
    v = keep_in_velocity([0.0, 0.0, 0.0], [2.5, 0, 1], LO, HI, gain=2.0)
    assert v[0] == pytest.approx(-1.0)


def test_keep_in_is_per_axis_and_respects_margin():
    v = keep_in_velocity([1.0, 1.0, 1.0], [1.9, 0.0, 2.9], LO, HI, gain=1.0,
                         margin=0.2)
    assert v[0] == pytest.approx(-0.1)   # 1.9 is 0.1 past the shrunk wall 1.8
    assert v[1] == pytest.approx(1.0)
    assert v[2] == pytest.approx(-0.1)


def test_clamp_to_box():
    np.testing.assert_allclose(clamp_to_box([5, -5, 1], LO, HI), [2, -2, 1])
    np.testing.assert_allclose(clamp_to_box([5, 0, 1], LO, HI, margin=0.5),
                               [1.5, 0, 1])


def test_target_seeds_from_position_when_none():
    t = advance_target(None, [1.0, 2.0, 1.2], [0, 0, 0], 0.05, lead_max=0.5)
    np.testing.assert_allclose(t, [1.0, 2.0, 1.2])


def test_target_moves_with_the_sticks_and_stays_when_released():
    t = advance_target(None, [0, 0, 1], [1.0, 0, 0], 0.1, 0.5)
    t = advance_target(t, [0, 0, 1], [1.0, 0, 0], 0.1, 0.5)
    np.testing.assert_allclose(t, [0.2, 0, 1])
    held = advance_target(t, [0.2, 0, 1], [0, 0, 0], 0.1, 0.5)
    np.testing.assert_allclose(held, t)


def test_target_is_leashed_to_the_drone():
    # Drone stuck at the origin (CBF / wall): the target cannot run away.
    t = None
    for _ in range(100):
        t = advance_target(t, [0, 0, 1], [1.0, 0, 0], 0.1, lead_max=0.5)
    assert np.linalg.norm(t - [0, 0, 1]) == pytest.approx(0.5)
    # lead_max 0 disables the leash
    t = advance_target([3, 0, 1], [0, 0, 1], [0, 0, 0], 0.1, lead_max=0.0)
    np.testing.assert_allclose(t, [3, 0, 1])


def test_tracking_velocity_holds_and_feeds_forward():
    # Drift 0.2 m below target with sticks released -> climb at kp*0.2.
    v = tracking_velocity([0, 0, 1.2], [0, 0, 1.0], [0, 0, 0], kp=1.0, max_speed=1.0)
    np.testing.assert_allclose(v, [0, 0, 0.2])
    # On target with a stick -> exactly the stick (feedforward).
    v = tracking_velocity([0, 0, 1], [0, 0, 1], [0.7, 0, 0], kp=1.0, max_speed=1.0)
    np.testing.assert_allclose(v, [0.7, 0, 0])
    # Capped.
    v = tracking_velocity([5, 0, 1], [0, 0, 1], [2, 0, 0], kp=1.0, max_speed=1.0)
    assert np.linalg.norm(v) == pytest.approx(1.0)


def test_altitude_drop_scenario_is_gone():
    """Old bug: a target seeded on the ground (0.02 m) chased after takeoff to 1.2."""
    t = advance_target(None, [0, 0, 1.2], [0, 0, 0], 0.05, 0.5)   # seeded at handover
    v = tracking_velocity(t, [0, 0, 1.2], [0, 0, 0], 1.0, 1.0)
    np.testing.assert_allclose(v, [0, 0, 0])


# ------------------------------------------------- the braking envelope
#
# Bag run_045417 (drone_2, teleop at 8 m/s stick, geofence keep_in, gain 1,
# margin 0): eleven wall approaches at ~6 m/s, every one 0.35-0.68 m past
# the wall. The `gain * distance` cap is zero AT the wall, but the vehicle
# follows a bare velocity setpoint with ~0.1 s of delay and a ~0.55 s
# velocity-loop time constant, so it crossed at 1.5 m/s. wall_speed()
# with brake_accel > 0 is PX4's braking law instead, and
# keep_in_acceleration() is the feedforward that makes PX4 brake with it.

from svg_ground_control.fence import keep_in_acceleration, wall_speed   # noqa: E402
from svg_ground_control.position_hold import advance_reference   # noqa: E402
from svg_ground_control.trajectory import brake_speed   # noqa: E402


def test_wall_speed_is_the_plain_barrier_without_a_brake():
    d = np.array([-0.5, 0.0, 0.3, 4.0])
    np.testing.assert_allclose(wall_speed(d, gain=2.0), 2.0 * d)
    np.testing.assert_allclose(wall_speed(d, gain=2.0, brake_accel=0.0), 2.0 * d)


def test_wall_speed_envelope_is_gain_times_distance_at_the_wall_and_sqrt_far_out():
    a, g = 4.0, 3.0
    # Near the wall: the old barrier (tail into the wall, push-back outside).
    assert wall_speed(0.01, g, a) == pytest.approx(g * 0.01, rel=0.05)
    assert wall_speed(-0.01, g, a) == pytest.approx(-g * 0.01, rel=0.05)
    # Far out: constant deceleration sqrt(2 a d) (plus the lag margin).
    assert wall_speed(2000.0, g, a) == pytest.approx(np.sqrt(2 * a * 2000.0), rel=0.02)
    # It is PX4's law with L = 1/gain, odd in the distance, and never above gain*d.
    d = np.linspace(0.0, 12.0, 400)
    np.testing.assert_allclose(wall_speed(d, g, a), brake_speed(d, a, 1.0 / g))
    np.testing.assert_allclose(wall_speed(-d, g, a), -wall_speed(d, g, a))
    assert np.all(wall_speed(d, g, a) <= g * d + 1e-9)
    assert np.all(np.diff(wall_speed(d, g, a)) > 0.0)


def test_keep_in_velocity_with_the_envelope_keeps_cruise_until_the_braking_distance():
    a, g, stick = 4.0, 3.0, 8.0
    # 8 m/s needs v^2/(2a) + v/g = 8 + 2.67 m: untouched at 12 m, capped at 8 m.
    v = keep_in_velocity([stick, 0, 0], [HI[0] - 12.0, 0, 1], LO, HI, g, brake_accel=a)
    assert v[0] == pytest.approx(stick)
    v = keep_in_velocity([stick, 0, 0], [HI[0] - 8.0, 0, 1], LO, HI, g, brake_accel=a)
    assert 6.5 < v[0] < 7.0
    # The other axes and inward motion are never touched (the far wall is
    # 3.9 m away: its own envelope allows 4.4 m/s toward it).
    v = keep_in_velocity([-3.0, 1.0, -1.0], [HI[0] - 0.1, 0, 1], LO, HI, g, brake_accel=a)
    np.testing.assert_allclose(v, [-3.0, 1.0, -1.0])


def test_keep_in_acceleration_brakes_only_the_limited_axes_toward_the_inside():
    a, g = 4.0, 3.0
    pos = np.array([HI[0] - 8.0, LO[1] + 0.1, 1.0])
    nominal = np.array([8.0, -2.0, 0.5])
    clipped = keep_in_velocity(nominal, pos, LO, HI, g, brake_accel=a)
    assert clipped[0] < nominal[0] and clipped[1] > nominal[1] and clipped[2] == 0.5
    # Cruising at the +x wall's envelope speed: the envelope's own deceleration,
    # capped at brake_accel; creeping toward the -y wall: ~ gain * speed.
    acc = keep_in_acceleration(nominal, clipped, [8.0, -0.3, 0.0], pos, LO, HI, g,
                               brake_accel=a)
    assert -a <= acc[0] < -0.9 * a
    assert acc[1] == pytest.approx(g * 0.3, rel=0.3) and acc[1] > 0.0
    assert acc[2] == 0.0
    # Faster than the envelope: harder, but never beyond brake_accel.
    acc = keep_in_acceleration(nominal, clipped, [20.0, 0.0, 0.0], pos, LO, HI, g,
                               brake_accel=a)
    assert acc[0] == pytest.approx(-a)
    # Moving back in on a limited axis the wall setpoint is rising toward
    # the wall and the feedforward eases the return (toward the wall, small).
    acc = keep_in_acceleration(nominal, clipped, [-1.0, 0.5, 0.0], pos, LO, HI, g,
                               brake_accel=a)
    assert 0.0 < acc[0] <= a and -a <= acc[1] < 0.0 and acc[2] == 0.0
    assert acc[0] == pytest.approx(-keep_in_acceleration(
        nominal, clipped, [1.0, -0.5, 0.0], pos, LO, HI, g, brake_accel=a)[0])
    # Nothing limited, or no brake: zero.
    far = np.array([0.0, 0.0, 1.0])
    acc = keep_in_acceleration(nominal, nominal, [8.0, -2.0, 0.5], far, LO, HI, g,
                               brake_accel=a)
    np.testing.assert_allclose(acc, 0.0)
    acc = keep_in_acceleration(nominal, clipped, [8.0, -0.3, 0.0], pos, LO, HI, g,
                               brake_accel=0.0)
    np.testing.assert_allclose(acc, 0.0)


# ---------------------------------------- closed loop against the PX4 plant

DT = 0.05
WALL = HI[0]


class Px4OffboardPlant:
    """1-D PX4 offboard model, same as test_trajectory.py (identified from
    drone_2's ULogs): 0.15 s link delay, velocity loop P=1.8 I=0.4 on
    acceleration, attitude lag 0.1 s, |a| <= 8, position loop P=0.95 when
    a position setpoint is given, acceleration setpoint fed forward."""

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


def fly_at_the_wall(stick, start, gain, brake_accel, trajectory=True,
                    seconds=8.0, teleop_lead=0.5, hold_lead=0.2, warmup=3.0):
    """The commander's teleop loop with a stick held into the +x wall.

    Trajectory output: reference + clipped velocity + fence feedforward,
    the reference leashed to hold_lead while the wall limits (as the
    control loop does). Velocity output: the clipped velocity alone. Like
    the flight in the bag the drone accelerates from rest under the stick:
    the first ``warmup`` seconds fly with the wall pushed far away, so the
    log starts at steady cruise at least ``start`` metres from the wall.
    """
    plant = Px4OffboardPlant()
    plant.p = WALL - start - stick * warmup
    plant.meas = [(plant.p, 0.0)] * len(plant.meas)
    ref, published, log = None, np.zeros(3), []
    lo = LO - np.array([1e3, 0.0, 0.0])          # the run-up is behind the -x wall
    for k in range(int((warmup + seconds) / DT)):
        t = k * DT - warmup
        hi = HI + (0.0 if t >= 0.0 else np.array([1e3, 0.0, 0.0]))
        pm, vm = plant.measure()
        pos, meas = np.array([pm, 0.0, 1.0]), np.array([vm, 0.0, 0.0])
        ref, _ = advance_reference(ref, pos, published, meas, DT, teleop_lead)
        ref = clamp_to_box(ref, lo, hi)
        nominal = np.array([stick, 0.0, 0.0])
        v = keep_in_velocity(nominal, pos, lo, hi, gain, 0.0, brake_accel)
        a = keep_in_acceleration(nominal, v, meas, pos, lo, hi, gain, 0.0, brake_accel)
        if np.linalg.norm(v - nominal) > 1e-6:
            lead = ref - pos
            dist = float(np.linalg.norm(lead))
            if dist > hold_lead:
                ref = pos + lead * (hold_lead / dist)
        plant.command(ref[0] if trajectory else np.nan, v[0], a[0] if trajectory else 0.0)
        published = v.copy()
        if t >= 0.0:
            log.append((t, plant.p, plant.v))
    return np.array(log)


def overshoot(log):
    return float((log[:, 1] - WALL).max())


def settled_at(log, tol_p=0.15, tol_v=0.1):
    """Time after which the drone stays within tol_p of the wall and slower than tol_v."""
    ok = (np.abs(log[:, 1] - WALL) < tol_p) & (np.abs(log[:, 2]) < tol_v)
    for k in range(len(ok)):
        if ok[k:].all():
            return log[k, 0]
    return None


def test_plain_barrier_overshoots_like_the_bag():
    # The flight in run_045417: 6 m/s into the wall, gain 1, no feedforward.
    log = fly_at_the_wall(stick=6.0, start=10.0, gain=1.0, brake_accel=0.0)
    assert overshoot(log) > 0.25
    assert overshoot(fly_at_the_wall(8.0, 14.0, gain=1.0, brake_accel=0.0)) > 0.4
    # A stiffer plain barrier is worse: the command drops faster than the
    # vehicle can follow.
    assert overshoot(fly_at_the_wall(6.0, 10.0, gain=3.0, brake_accel=0.0)) > 1.0


@pytest.mark.parametrize('stick, start', [(6.0, 10.0), (8.0, 14.0), (2.0, 3.0), (0.5, 1.0)])
def test_envelope_stops_a_lagging_drone_at_the_wall(stick, start):
    log = fly_at_the_wall(stick, start, gain=2.0, brake_accel=4.0)
    assert log[0, 1] <= WALL - start
    assert overshoot(log) < 0.10
    # ...and the whole manoeuvre — cruise, brake, at rest on the wall — is
    # over sooner than the plain barrier needs to overshoot and come back.
    old = fly_at_the_wall(stick, start, gain=1.0, brake_accel=0.0)
    assert settled_at(log) is not None
    assert settled_at(old) is None or settled_at(log) < settled_at(old)
    # ...and the wall then stays quiet (no ringing through the loop delay).
    assert np.abs(log[-40:, 2]).max() < 0.08
    # Still travelling: the drone holds its cruise speed past the wall's
    # braking distance (v^2/(2a) + v/gain) and only brakes inside it. (The
    # cruise speed is the stick plus PX4's pull on the leashed reference,
    # as in test_trajectory.py — nothing to do with the wall.)
    braking_distance = stick ** 2 / (2 * 4.0) + stick / 2.0
    cruising = log[log[:, 1] < WALL - braking_distance - 0.3]
    assert len(cruising) > 0
    assert cruising[:, 2].min() > 0.9 * cruising[:, 2].max()
    assert cruising[:, 2].max() > 0.85 * stick


def test_envelope_with_a_bare_velocity_setpoint_needs_the_softer_settings():
    # Sim / velocity-only drones get no feedforward and no position loop:
    # the plant lags ~0.7 s, so the envelope must allow for it — gain 0.7
    # (a 1.4 s lag margin) and a gentler brake. Still far better than the
    # plain barrier, but not the trajectory output's centimetres.
    for stick, start, plain_min in ((2.0, 3.0, 0.5), (6.0, 10.0, 1.0)):
        plain = overshoot(fly_at_the_wall(stick, start, 1.0, 0.0, trajectory=False))
        soft = overshoot(fly_at_the_wall(stick, start, 0.7, 2.0, trajectory=False))
        assert plain > plain_min
        assert soft < 0.2
        # The trajectory-output settings are too stiff for a bare velocity setpoint.
        assert overshoot(fly_at_the_wall(stick, start, 2.0, 4.0, trajectory=False)) > 0.5


# ------------------------------------------------ the stick acceleration ramp

from svg_ground_control.position_hold import ramp_velocity   # noqa: E402


def test_ramp_velocity_limits_acceleration_and_reports_it():
    v, a, prof = ramp_velocity(None, [0, 0, 0], [8.0, 0, 0], accel=5.0, dt=0.05)
    np.testing.assert_allclose(v, [0.25, 0, 0])
    np.testing.assert_allclose(a, [5.0, 0, 0])
    np.testing.assert_allclose(prof, v)
    # Runs on from the profile, not from the (lagging) applied velocity...
    v2, a2, prof = ramp_velocity(prof, v, [8.0, 0, 0], 5.0, 0.05)
    np.testing.assert_allclose(v2, [0.5, 0, 0])
    # ...reaches the target exactly and stays there with zero acceleration.
    v3, a3, prof = ramp_velocity([7.9, 0, 0], [7.9, 0, 0], [8.0, 0, 0], 5.0, 0.05)
    np.testing.assert_allclose(v3, [8.0, 0, 0])
    np.testing.assert_allclose(a3, [2.0, 0, 0])
    v4, a4, prof = ramp_velocity(v3, v3, [8.0, 0, 0], 5.0, 0.05)
    np.testing.assert_allclose(v4, [8.0, 0, 0])
    np.testing.assert_allclose(a4, 0.0)


def test_ramp_velocity_reattaches_to_what_was_published_and_can_be_off():
    # A wall clipped the published velocity to 2 while the ramp was at 6:
    # the ramp restarts from 2, so the release is a ramp, not a step.
    v, a, prof = ramp_velocity([6.0, 0, 0], [2.0, 0, 0], [8.0, 0, 0], 5.0, 0.05)
    np.testing.assert_allclose(v, [2.25, 0, 0])
    # accel 0: the stick passes through untouched, no feedforward.
    v, a, prof = ramp_velocity([6.0, 0, 0], [2.0, 0, 0], [8.0, 0, 0], 0.0, 0.05)
    np.testing.assert_allclose(v, [8.0, 0, 0])
    np.testing.assert_allclose(a, 0.0)
    assert prof is None


def fly_across_box(stick, span, gain, brake_accel, ramp, seconds=8.0,
                   teleop_lead=0.5, hold_lead=0.2):
    """From rest at one wall, stick held toward the far wall ``span`` away.

    The goal_single teleop box is 7.7 m across: the drone can only
    accelerate for part of it and must brake for the rest, so this is what
    bounds the speed a pilot actually sees. The commander's teleop loop:
    stick ramp -> keep_in clip -> reference -> PX4 (position + velocity +
    acceleration feedforward).
    """
    lo = np.array([0.0, -50.0, 0.0])
    hi = np.array([span, 50.0, 3.0])
    plant = Px4OffboardPlant()
    plant.p = 0.05
    plant.meas = [(plant.p, 0.0)] * len(plant.meas)
    ref, published, profile, log = None, np.zeros(3), None, []
    for k in range(int(seconds / DT)):
        pm, vm = plant.measure()
        pos, meas = np.array([pm, 0.0, 1.0]), np.array([vm, 0.0, 0.0])
        ref, _ = advance_reference(ref, pos, published, meas, DT, teleop_lead)
        ref = clamp_to_box(ref, lo, hi)
        nominal, accel, profile = ramp_velocity(profile, published, [stick, 0.0, 0.0],
                                                ramp, DT)
        v = keep_in_velocity(nominal, pos, lo, hi, gain, 0.0, brake_accel)
        if np.linalg.norm(v - nominal) > 1e-6:
            accel = keep_in_acceleration(nominal, v, meas, pos, lo, hi, gain, 0.0,
                                         brake_accel)
            lead = ref - pos
            dist = float(np.linalg.norm(lead))
            if dist > hold_lead:
                ref = pos + lead * (hold_lead / dist)
        plant.command(ref[0], v[0], accel[0])
        published = v.copy()
        log.append((k * DT, plant.p, plant.v))
    return np.array(log)


def test_stick_ramp_makes_the_box_run_faster_and_still_stops_at_the_wall():
    # goal_single: 7.7 m teleop box, 8 m/s stick, brake 4, gain 2.
    step = fly_across_box(8.0, 7.7, 2.0, 4.0, ramp=0.0)
    px4 = fly_across_box(8.0, 7.7, 2.0, 4.0, ramp=5.0)      # MPC_ACC_HOR_MAX
    hard = fly_across_box(8.0, 7.7, 2.0, 4.0, ramp=8.0)
    for log in (step, px4, hard):
        assert (log[:, 1] - 7.7).max() < 0.10
        assert np.abs(log[-40:, 2]).max() < 0.08
    assert step[:, 2].max() < 4.8
    assert px4[:, 2].max() > 4.9
    assert hard[:, 2].max() > 5.25 and hard[:, 2].max() >= px4[:, 2].max()
    # 8 m/s is not reachable in this box with a stop at the wall, whatever
    # the settings: accelerate half the span, brake the other half.
    assert hard[:, 2].max() < np.sqrt(2 * 8.0 * 7.7 / 2)



# ------------------------------------- the leash, per axis group, and release
#
# Bag run_060352 (drone_2, x-y stick only): the reference altitude walked
# down 1.1 m, every step while the 0.5 m leash was engaged, and released
# sticks sometimes kept the drone coasting. One 3-D leash scaled the z
# component of the lead with the horizontal lag, and advance_reference hands
# back the MEASURED velocity after a leash pull -- which the stick ramp then
# adopted as its own command, sink rate and overrun included.

from svg_ground_control.position_hold import leash   # noqa: E402


def test_leash_limits_horizontal_and_vertical_lead_separately():
    lead, pulled = leash([1.2, 0.0, 0.03], 0.5)
    assert pulled
    np.testing.assert_allclose(lead, [0.5, 0.0, 0.03])     # z untouched
    lead, pulled = leash([0.3, 0.4, -0.8], 0.5, max_z=0.2)
    assert pulled
    np.testing.assert_allclose(lead, [0.3, 0.4, -0.2])
    lead, pulled = leash([0.3, 0.0, 0.1], 0.5)
    assert not pulled
    np.testing.assert_allclose(lead, [0.3, 0.0, 0.1])
    lead, pulled = leash([3.0, 0.0, 3.0], 0.0)              # 0 = no leash
    assert not pulled


def test_reference_altitude_survives_a_horizontal_lag():
    # Reference 0.9 m ahead in x, 3 cm above the drone: pulled back to 0.5 m
    # in x, still 3 cm above -- before, it was scaled to 0.5/0.9 of that.
    ref, applied = advance_reference([0.9, 0.0, 1.03], [0.0, 0.0, 1.0], [0, 0, 0],
                                     [2.0, 0.0, -0.3], 0.05, 0.5)
    np.testing.assert_allclose(ref, [0.5, 0.0, 1.03])
    np.testing.assert_allclose(applied, [2.0, 0.0, -0.3])   # the scenario re-attach signal


def fly_release(sag_gain, reattach_to, stick=4.0, seconds=6.0, ramp=5.0,
                teleop_lead=0.5, hold_lead=0.2):
    """Cruise at ``stick`` with the leash engaged, release, watch z.

    x is the PX4 plant. z is a caricature of the bag: the vehicle sinks by
    ``sag_gain`` * |x acceleration| below its commanded altitude (bank ->
    lost lift), so the measured vz is nonzero while the z stick is zero.
    ``reattach_to`` = 'published' (the commander) or 'measured' (the bug).
    Returns ``(release_time, log)`` with rows (t, x, vx, ref_z, z).
    """
    plant = Px4OffboardPlant()
    plant.p = -20.0
    plant.meas = [(plant.p, 0.0)] * len(plant.meas)
    ref, published, profile, log = None, np.zeros(3), None, []
    z_cmd_track, z_prev, ax_prev = 1.0, 1.0, 0.0
    t_release = None
    for k in range(int(seconds / DT)):
        t = k * DT
        pm, vm = plant.measure()
        ax = (vm - (log[-1][2] if log else 0.0)) / DT
        # z: the vehicle holds ref_z minus a sag proportional to |ax|.
        z = (ref[2] if ref is not None else 1.0) - sag_gain * abs(ax)
        vz = (z - z_prev) / DT
        z_prev = z
        pos, meas = np.array([pm, 0.0, z]), np.array([vm, 0.0, vz])
        ref, applied = advance_reference(ref, pos, published, meas, DT, teleop_lead)
        target = [stick if t < 2.5 else 0.0, 0.0, 0.0]
        if t >= 2.5 and t_release is None:
            t_release = t
        base = published if reattach_to == 'published' else applied
        v, a, profile = ramp_velocity(profile, base, target, ramp, DT)
        plant.command(ref[0], v[0], a[0])
        published = v.copy()
        log.append((t, plant.p, plant.v, ref[2], z))
    return t_release, np.array(log)


def test_altitude_reference_does_not_walk_down_on_x_y_stick():
    _, good = fly_release(sag_gain=0.02, reattach_to='published')
    assert good[:, 3].min() > 1.0 - 0.01
    # The bug for the record: re-attaching the ramp to the measured velocity
    # turns every sag into a descent command.
    _, bad = fly_release(sag_gain=0.02, reattach_to='measured')
    assert bad[:, 3].min() < 1.0 - 0.05


def test_released_sticks_stop_within_the_ramp_distance():
    t0, log = fly_release(sag_gain=0.0, reattach_to='published')
    after = log[log[:, 0] >= t0]
    v0 = after[0, 2]
    stopped = after[np.abs(after[:, 2]) < 0.1]
    assert len(stopped)
    t_stop, x_stop = stopped[0, 0] - t0, stopped[0, 1] - after[0, 1]
    # Ramp-down at 5 m/s^2 plus the 0.5 m reference lead and the vehicle lag.
    assert t_stop < v0 / 5.0 + 1.0
    assert x_stop < v0 ** 2 / (2 * 5.0) + 1.0
