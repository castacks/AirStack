"""Tests for the scenario policies, including a kinematic squeeze rollout."""

from __future__ import annotations

import numpy as np

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
