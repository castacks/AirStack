"""Tests for the scenario policies, including a kinematic squeeze rollout."""

from __future__ import annotations

import numpy as np
import pytest

from svg_ground_control.cbf_filter import filter_velocities
from svg_ground_control.scenarios import Bounds, make_scenario

ARENA = Bounds(low=np.array([-2.0, -2.0, 0.8]), high=np.array([2.0, 2.0, 2.0]))


def make(name, n, **kwargs):
    return make_scenario(
        name, num_drones=n, nominal_speed=0.6, bounds=ARENA,
        safety_radius=0.55, seed=7, **kwargs)


def test_all_scenarios_produce_valid_initial_positions() -> None:
    for name, n in [('random_walk', 5), ('random_goals', 5),
                    ('head_on', 6), ('antipodal', 6)]:
        scenario = make(name, n)
        positions = scenario.initial_positions()
        assert positions.shape == (n, 3)
        assert np.all(positions >= ARENA.low - 1e-9)
        assert np.all(positions <= ARENA.high + 1e-9)
        nominal = scenario.nominal_velocity(positions)
        assert nominal.shape == (n, 3)
        assert np.all(np.isfinite(nominal))


def test_goal_scenario_live_retarget_and_speed() -> None:
    initial = np.array([[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]])
    s = make('goal', 2, initial_goals=initial)
    np.testing.assert_allclose(s.initial_positions(), initial)

    # Default: seek the initial goals.
    pos = initial + np.array([[0.5, 0.0, 0.0], [0.0, 0.0, 0.0]])
    v = s.nominal_velocity(pos)
    assert v[0, 0] < 0.0                          # drone 0 pulled back -x
    np.testing.assert_allclose(v[1], 0.0, atol=1e-9)

    # Retarget drone 1 live; speed cap respected.
    s.set_goal(1, np.array([5.0, 0.0, 1.2]))
    s.set_speed(1, 0.5)
    v = s.nominal_velocity(pos)
    assert v[1, 0] > 0.0
    assert abs(np.linalg.norm(v[1]) - 0.5) < 1e-6   # far goal -> capped at speed


def test_tracked_rows_follow_the_reference_profile() -> None:
    """With a reference the row ramps at the acceleration limit and reports
    the feedforward; a NaN reference row keeps the stateless law."""
    initial = np.array([[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]])
    s = make('goal', 2, initial_goals=initial, accel=2.0, settle_s=0.3)
    s.set_goal(0, np.array([5.0, 0.0, 1.2]))
    s.set_goal(1, np.array([6.0, 0.0, 1.2]))
    refs = np.array([[0.0, 0.0, 1.2], [np.nan, np.nan, np.nan]])
    v = s.nominal_velocity(initial, references=refs, applied=np.zeros((2, 3)), dt=0.05)
    assert np.linalg.norm(v[0]) == pytest.approx(2.0 * 0.05)     # one tick of accel
    assert s.nominal_acceleration[0] == pytest.approx([2.0, 0.0, 0.0])
    assert np.linalg.norm(v[1]) == pytest.approx(0.6)             # stateless: capped
    assert s.nominal_acceleration[1] == pytest.approx([0.0, 0.0, 0.0])
    # keeps ramping while the reference moves with what was applied
    for _ in range(20):
        refs[0] += v[0] * 0.05
        v = s.nominal_velocity(initial, references=refs, applied=v, dt=0.05)
    assert np.linalg.norm(v[0]) == pytest.approx(0.6)             # cruise
    s.reset_tracking()
    assert np.all(s.tracker.velocity == 0.0)


def test_hover_scenario_seeks_targets() -> None:
    targets = np.array([[-1.0, 0.0, 1.2], [1.0, 0.0, 1.2]])
    scenario = make('hover', 2, hover_positions=targets)
    np.testing.assert_allclose(scenario.initial_positions(), targets)
    # Displaced drone gets pulled back toward its target.
    displaced = targets + np.array([[0.5, 0.0, 0.0], [0.0, 0.0, 0.0]])
    nominal = scenario.nominal_velocity(displaced)
    assert nominal[0, 0] < 0.0           # pulled back along -x
    np.testing.assert_allclose(nominal[1], 0.0, atol=1e-9)


HOLDER_POSTS = [0.0, -0.69, 1.2, 0.0, 0.69, 1.2]
INTRUDER_WAYPOINTS = [-1.5, 0.0, 1.2, 1.5, 0.0, 1.2]


def make_squeeze():
    return make('squeeze', 3, holder_positions=HOLDER_POSTS,
                intruder_waypoints=INTRUDER_WAYPOINTS)


def test_squeeze_intruder_is_cbf_exempt_by_default() -> None:
    assert make_squeeze().cbf_exempt_indices == [2]
    filtered = make('squeeze', 3, holder_positions=HOLDER_POSTS,
                    intruder_waypoints=INTRUDER_WAYPOINTS,
                    intruder_cbf_exempt=False)
    assert filtered.cbf_exempt_indices == []


def test_squeeze_geometry() -> None:
    scenario = make_squeeze()
    initial = scenario.initial_positions()
    # Holders take off exactly at their configured posts.
    np.testing.assert_allclose(initial[0], HOLDER_POSTS[:3])
    np.testing.assert_allclose(initial[1], HOLDER_POSTS[3:])
    # Intruder takes off at waypoint A and its nominal points toward B (+x).
    np.testing.assert_allclose(initial[2], INTRUDER_WAYPOINTS[:3])
    nominal = scenario.nominal_velocity(initial)
    assert nominal[2, 0] > 0.0


def test_squeeze_rejects_overlapping_posts() -> None:
    try:
        make('squeeze', 3,
             holder_positions=[0.0, -0.3, 1.2, 0.0, 0.3, 1.2],  # 0.6 m < 2r
             intruder_waypoints=INTRUDER_WAYPOINTS)
    except ValueError as e:
        assert 'keep-out' in str(e)
    else:
        raise AssertionError('overlapping posts were not rejected')


def test_squeeze_kinematic_rollout_holders_yield_and_return() -> None:
    """Single-integrator rollout: barrier holds, holders yield then return."""
    safety_radius = 0.55
    max_speed = 1.2
    dt = 0.05
    scenario = make_squeeze()
    positions = scenario.initial_positions().copy()
    posts = positions[:2].copy()

    min_pair_distance = np.inf
    max_holder_displacement = 0.0
    for _ in range(400):  # 20 s — more than one full crossing
        nominal = scenario.nominal_velocity(positions)
        result = filter_velocities(
            nominal, positions, safety_radius, max_speed, alpha=2.5)
        # The intruder is CBF-exempt (the commander restores its row).
        safe = result.velocities
        safe[2] = nominal[2]
        positions = positions + safe * dt

        distances = np.linalg.norm(
            positions[:, None] - positions[None, :], axis=-1)
        np.fill_diagonal(distances, np.inf)
        min_pair_distance = min(min_pair_distance, float(distances.min()))
        max_holder_displacement = max(
            max_holder_displacement,
            float(np.linalg.norm(positions[:2] - posts, axis=-1).max()))

    # Holders were genuinely displaced by the crossing...
    assert max_holder_displacement > 0.2
    # ...the intruder actually made it through to the +x side at least once
    # (it shuttles, so just check it covered the run)...
    assert positions[2, 0] > ARENA.center[0] - 1.6
    # ...and, with the exempt intruder pushing through, the holders never let
    # the *holder pair* breach its own barrier; holder-intruder distance may
    # dip slightly below 2r since one party is uncontrolled — require the
    # holders to keep at least 1.5 r body margin from the intruder.
    holder_pair = np.linalg.norm(positions[0] - positions[1])
    assert holder_pair >= 0.0  # sanity
    assert min_pair_distance >= 1.5 * safety_radius

    # After the crossing settles (intruder far from center), holders return.
    for _ in range(100):
        nominal = scenario.nominal_velocity(positions)
        result = filter_velocities(
            nominal, positions, safety_radius, max_speed, alpha=2.5)
        safe = result.velocities
        safe[2] = nominal[2]
        positions = positions + safe * dt
    settle_error = np.linalg.norm(positions[:2] - posts, axis=-1).max()
    assert settle_error < 0.6  # back near the posts (intruder keeps shuttling)


# ------------------------------------------------------------ figure_eight

def make_eight(**overrides):
    kwargs = dict(center=[0.5, 0.0, 1.5], radius=1.0, center_distance=2.0,
                  axis_deg=0.0, tilt_deg=30.0, directions=[1.0, -1.0],
                  track_gain=1.0, intruder_start=[0.5, -2.5, 1.5])
    kwargs.update(overrides)
    return make_scenario('figure_eight', num_drones=3, nominal_speed=1.5,
                         bounds=ARENA, safety_radius=0.55, seed=7, **kwargs)


def test_figure_eight_geometry_and_takeoff_layout() -> None:
    sc = make_eight()
    np.testing.assert_allclose(sc.centers, [[-0.5, 0.0, 1.5], [1.5, 0.0, 1.5]])
    np.testing.assert_allclose(sc.touching_point, [0.5, 0.0, 1.5])
    initial = sc.initial_positions()
    # Far ends of the two lobes, a half lap from the crossing, intruder at its start.
    np.testing.assert_allclose(initial[0], [-1.5, 0.0, 1.5], atol=1e-9)
    np.testing.assert_allclose(initial[1], [2.5, 0.0, 1.5], atol=1e-9)
    np.testing.assert_allclose(initial[2], [0.5, -2.5, 1.5])
    assert sc.cbf_exempt_indices == [2]
    assert make_eight(intruder_cbf_exempt=False).cbf_exempt_indices == []
    # The lobes are drawn: two closed polylines of radius 1 about their centers.
    paths = sc.paths
    assert len(paths) == 2
    for lobe, path in enumerate(paths):
        r = np.linalg.norm(path - sc.centers[lobe], axis=1)
        np.testing.assert_allclose(r, 1.0, atol=1e-9)
        np.testing.assert_allclose(path[0], path[-1], atol=1e-9)


def test_figure_eight_tilt_rolls_the_plane_about_the_center_line() -> None:
    for tilt in (0.0, 30.0, 60.0):
        sc = make_eight(tilt_deg=tilt)
        z = np.concatenate([p[:, 2] for p in sc.paths])
        y = np.concatenate([p[:, 1] for p in sc.paths])
        assert np.ptp(z) == pytest.approx(2.0 * np.sin(np.radians(tilt)), abs=1e-6)
        assert np.ptp(y) == pytest.approx(2.0 * np.cos(np.radians(tilt)), abs=1e-6)
    # axis_deg turns the whole eight in the horizontal plane
    sc = make_eight(axis_deg=90.0)
    np.testing.assert_allclose(sc.centers[:, :2], [[0.5, -1.0], [0.5, 1.0]], atol=1e-9)


def test_figure_eight_carrots_meet_at_the_crossing_together() -> None:
    dt = 0.05
    sc = make_eight()
    sc.reset_tracking()
    p = sc.initial_positions().copy()
    closest = np.inf
    vel_at_meet = None
    for _ in range(int(2.5 * 2 * np.pi / 1.5 / dt)):     # 2.5 laps
        v = sc.nominal_velocity(p, dt=dt)
        g = sc.goals
        d = np.linalg.norm(g[0] - g[1])
        if d < closest:
            closest, vel_at_meet = d, (v[0].copy(), v[1].copy())
        p[:2] = g[:2]                                       # perfect tracking
    # collision-negligent: the two carrots coincide at the crossing ...
    assert closest < 0.05
    # ... with parallel velocities for [+1, -1] (a merge), speed = nominal
    v0, v1 = vel_at_meet
    assert np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)) > 0.99
    assert np.linalg.norm(v0) == pytest.approx(1.5, rel=0.05)
    # same sense: head-on at the crossing
    sc = make_eight(directions=[1.0, 1.0]); sc.reset_tracking()
    p = sc.initial_positions().copy(); best = (np.inf, None)
    for _ in range(int(1.2 * 2 * np.pi / 1.5 / dt)):
        v = sc.nominal_velocity(p, dt=dt); g = sc.goals
        d = np.linalg.norm(g[0] - g[1])
        if d < best[0]:
            best = (d, np.dot(v[0], v[1]) / (np.linalg.norm(v[0]) * np.linalg.norm(v[1])))
        p[:2] = g[:2]
    assert best[0] < 0.05 and best[1] < -0.99


def test_figure_eight_tracked_rows_get_the_centripetal_feedforward() -> None:
    sc = make_eight()
    p = sc.initial_positions().copy()
    refs = np.array([p[0], p[1], [np.nan, np.nan, np.nan]])
    v = sc.nominal_velocity(p, references=refs, applied=np.zeros((3, 3)), dt=0.05)
    a = sc.nominal_acceleration
    # v^2 / R = 2.25 m/s^2 toward the lobe center, on the tracked rows only
    for lobe in (0, 1):
        assert np.linalg.norm(a[lobe]) == pytest.approx(2.25, rel=0.02)
        inward = sc.centers[lobe] - p[lobe]
        assert np.dot(a[lobe], inward) > 0.0
        assert np.linalg.norm(v[lobe]) == pytest.approx(1.5, rel=0.1)
    np.testing.assert_allclose(a[2], 0.0)
    # a drone pushed off its lobe is pulled back at track_gain, capped at the speed
    off = p.copy(); off[0] += [0.0, 3.0, 0.0]
    v = sc.nominal_velocity(off, dt=0.05)
    assert np.linalg.norm(v[0]) <= 1.5 + 1.5 + 1e-6 and v[0][1] < -1.0


def test_figure_eight_kinematic_rollout_cbf_keeps_the_pair_apart() -> None:
    """Single-integrator rollout: the nominal would collide, the filter does not."""
    safety_radius = 0.55
    dt = 0.05
    sc = make_eight()
    sc.reset_tracking()
    positions = sc.initial_positions().copy()
    min_pair = np.inf
    min_nominal_gap = np.inf
    for _ in range(int(3 * 2 * np.pi / 1.5 / dt)):       # 3 laps
        nominal = sc.nominal_velocity(positions, dt=dt)
        g = sc.goals
        min_nominal_gap = min(min_nominal_gap, np.linalg.norm(g[0] - g[1]))
        result = filter_velocities(nominal, positions, safety_radius, 6.0, alpha=2.5)
        positions = positions + result.velocities * dt
        min_pair = min(min_pair, np.linalg.norm(positions[0] - positions[1]))
    assert min_nominal_gap < 0.05                          # the conflict is real
    assert min_pair >= 2.0 * safety_radius - 0.05          # and the barrier holds
    # both lobe drones are still on (or near) their carrots after the laps
    assert np.linalg.norm(positions[0] - sc.goals[0]) < 0.6
    assert np.linalg.norm(positions[1] - sc.goals[1]) < 0.6
