"""Tests for the multi-agent CBF velocity filter.

Kinematic subset of the drone_soccer test suite (the MuJoCo physics test
stays in that repo); run with ``pytest`` or ``colcon test``.
"""

from __future__ import annotations

import numpy as np

from svg_ground_control.cbf_filter import (
    build_collision_constraints,
    filter_velocities,
)


def test_no_conflict_passes_nominal_through() -> None:
    """Well-separated, slow drones should keep their nominal velocity intact."""
    positions = np.array([[0.0, 0.0, 1.0], [5.0, 0.0, 1.0], [0.0, 5.0, 1.0]])
    nominal = np.array([[0.5, 0.0, 0.0], [-0.5, 0.0, 0.0], [0.0, -0.5, 0.0]])

    result = filter_velocities(nominal, positions, safety_radius=0.4, max_speed=1.0)

    assert not result.corrected.any()
    assert result.converged
    np.testing.assert_allclose(result.velocities, nominal, atol=1e-6)


def test_speed_cap_is_enforced() -> None:
    """A nominal velocity above the cap is scaled down to the cap."""
    positions = np.array([[0.0, 0.0, 1.0], [10.0, 0.0, 1.0]])
    nominal = np.array([[3.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    result = filter_velocities(nominal, positions, safety_radius=0.4, max_speed=1.0)

    assert np.linalg.norm(result.velocities[0]) <= 1.0 + 1e-6
    assert result.corrected[0]


def test_constraint_builder_shapes_and_barrier() -> None:
    """The builder yields one constraint per pair with the right barrier sign."""
    positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    constraints = build_collision_constraints(positions, safety_radius=0.4, alpha=2.0)

    assert constraints.num_pairs == 3  # 3 choose 2
    assert constraints.gradient.shape == (3, 3)
    # Pair (0,1) is 1 m apart with 2r=0.8 -> h = 1 - 0.64 = 0.36 > 0 (safe).
    np.testing.assert_allclose(constraints.barrier[0], 1.0 - 0.64)


def test_head_on_never_violates_barrier_kinematically() -> None:
    """Two head-on drones (single integrators) must never breach 2r."""
    safety_radius = 0.4
    max_speed = 1.0
    min_distance = 2.0 * safety_radius
    dt = 0.02

    positions = np.array([[-2.0, 0.0, 1.0], [2.0, 0.0, 1.0]])
    # Slight y-offset breaks the symmetric deadlock so the filter has a way out.
    nominal = np.array([[1.0, 0.05, 0.0], [-1.0, -0.05, 0.0]])

    worst = np.inf
    for _ in range(600):
        result = filter_velocities(
            nominal, positions, safety_radius, max_speed, alpha=4.0
        )
        positions = positions + result.velocities * dt
        worst = min(worst, float(np.linalg.norm(positions[0] - positions[1])))

    # Allow a tiny numerical slack below the exact 2r contact distance.
    assert worst >= min_distance - 1e-2


def test_dense_feasible_swarm_solver_converges() -> None:
    """A tight but feasible swarm is filtered without solver anomalies."""
    rng = np.random.default_rng(0)
    # 2 x 2 x 2 grid at 0.9 m spacing: just above the 0.8 m required
    # separation, so every pair is safe but barely.
    grid = np.array(
        [[x, y, z] for x in (0.0, 0.9) for y in (0.0, 0.9) for z in (1.0, 1.9)]
    )
    nominal = rng.uniform(-1.0, 1.0, size=(8, 3))

    result = filter_velocities(nominal, grid, safety_radius=0.4, max_speed=1.0)

    assert result.converged
    assert np.all(np.linalg.norm(result.velocities, axis=-1) <= 1.0 + 1e-6)
    assert result.residual < 1e-2


def test_overlapping_cluster_triggers_emergency_stop() -> None:
    """An already-violating cluster falls back to the push-apart action."""
    rng = np.random.default_rng(0)
    # Drones packed well inside each other's safety spheres: the velocity-CBF
    # QP is infeasible under the speed cap, so the fallback must engage.
    positions = rng.uniform(-0.3, 0.3, size=(6, 3)) + np.array([0.0, 0.0, 1.0])
    nominal = rng.uniform(-1.0, 1.0, size=(6, 3))

    result = filter_velocities(nominal, positions, safety_radius=0.4, max_speed=1.0)

    assert result.used_emergency_stop
    assert not result.converged
    assert np.all(np.linalg.norm(result.velocities, axis=-1) <= 1.0 + 1e-6)
