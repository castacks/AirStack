"""Multi-agent Control-Barrier-Function velocity filter (3D).

Ported from ~/drone_soccer (drone_soccer/cbf.py), where it is validated in
MuJoCo against the Starling 2 Max airframe. The public API is
``filter_velocities(...) -> CBFResult``; the swarm commander calls only that.

Math (first-order / velocity CBF). For drones i, j with positions p_i, p_j
and a shared safety radius r, the pairwise barrier is

    h_ij = ||p_i - p_j||^2 - (2 r)^2        (>= 0 means safe)

Requiring ``h_dot + alpha * h >= 0`` gives a constraint that is *linear in the
velocities*:

    2 (p_i - p_j) . (v_i - v_j) + alpha * h_ij >= 0

Together with the per-drone cap ``||v_i|| <= v_max``, the safe-velocity set is
an intersection of convex sets, and the least-squares projection onto it is
solved by Dykstra's alternating projection in *parallel (product-space) form*:
every constraint is projected simultaneously in vectorized numpy and the
per-drone results are involvement-weighted-averaged each sweep. Pairs too far
apart to possibly activate this step (``alpha h >= ||grad|| * 2 v_max``) are
pruned up front, so cost scales with the number of *active* conflicts, not
all N^2 pairs. A Gauss-Seidel polish phase runs only when the parallel phase
stalls, and an emergency push-apart fallback engages when the QP is
infeasible (drones already inside each other's safety spheres).

HOCBF-readiness: the solver (`solve_safe_commands`) is written against a
generic `PairwiseConstraints` bundle (each constraint is
``gradient . (x_i - x_j) + bias >= 0`` over per-drone decision vectors x) plus
a per-drone norm cap. Swapping the velocity-CBF constraint builder
(`build_collision_constraints`) for an acceleration-input HOCBF builder -- same
bundle shape, ``x`` becomes commanded acceleration -- upgrades the filter
without touching the solver.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PairwiseConstraints:
    """Affine pairwise constraints over per-drone decision vectors.

    Each row ``m`` encodes ``gradient[m] . (x[idx_i[m]] - x[idx_j[m]]) +
    bias[m] >= 0``. For the velocity CBF the decision vector ``x`` is a
    drone's velocity; for a future HOCBF it would be commanded acceleration.

    Attributes:
        idx_i: shape (M,) first drone index of each pair.
        idx_j: shape (M,) second drone index of each pair.
        gradient: shape (M, D) constraint gradient w.r.t. ``x_i`` (the
            gradient w.r.t. ``x_j`` is its negation).
        bias: shape (M,) constraint offset (the ``alpha * h`` term).
        barrier: shape (M,) barrier value ``h`` per pair, kept for diagnostics.
    """

    idx_i: np.ndarray
    idx_j: np.ndarray
    gradient: np.ndarray
    bias: np.ndarray
    barrier: np.ndarray

    @property
    def num_pairs(self) -> int:
        return int(self.idx_i.shape[0])


@dataclass(frozen=True)
class CBFResult:
    """Output of the CBF filter for one solve.

    Attributes:
        velocities: shape (N, D) safe per-drone velocities.
        corrected: shape (N,) bool, True where the safe velocity differs from
            the nominal beyond a small tolerance.
        converged: whether Dykstra met the residual + step tolerances.
        residual: worst constraint violation after solving (0 = all satisfied).
        iterations: number of Dykstra sweeps actually run.
        used_emergency_stop: whether the push-apart fallback was triggered.
        num_infeasible: count of pairs that were immediately infeasible
            (drones coincident with a negative barrier).
    """

    velocities: np.ndarray
    corrected: np.ndarray
    converged: bool
    residual: float
    iterations: int
    used_emergency_stop: bool
    num_infeasible: int


def build_collision_constraints(
    positions: np.ndarray,
    safety_radius: float,
    alpha: float,
) -> PairwiseConstraints:
    """Build the first-order (velocity) collision-avoidance constraints.

    Args:
        positions: shape (N, D) world-frame drone positions.
        safety_radius: per-drone safety radius r (m); two drones are "safe"
            when their centers are at least ``2 r`` apart.
        alpha: CBF class-K gain; larger = the filter reacts later but harder.

    Returns:
        `PairwiseConstraints` over all ``N*(N-1)/2`` unordered drone pairs.
    """
    num_drones = positions.shape[0]
    idx_i, idx_j = np.triu_indices(num_drones, k=1)

    relative_position = positions[idx_i] - positions[idx_j]
    squared_distance = np.einsum("md,md->m", relative_position, relative_position)
    barrier = squared_distance - (2.0 * safety_radius) ** 2

    # d/dv_i of h_dot is 2 (p_i - p_j); the bias is the alpha * h term.
    gradient = 2.0 * relative_position
    bias = alpha * barrier

    return PairwiseConstraints(
        idx_i=idx_i,
        idx_j=idx_j,
        gradient=gradient,
        bias=bias,
        barrier=barrier,
    )


def solve_safe_commands(
    nominal: np.ndarray,
    constraints: PairwiseConstraints,
    norm_cap: float,
    alpha_unused: float = 0.0,
    max_iterations: int = 200,
    tolerance: float = 2e-3,
) -> CBFResult:
    """Project `nominal` onto the safe set (hybrid Dykstra projection).

    Minimizes ``sum_k ||x_k - nominal_k||^2`` subject to every pairwise
    half-space in `constraints` and the per-drone cap ``||x_k|| <= norm_cap``.

    Two phases over the *pruned* active constraint set:

    1. Parallel (product-space) Dykstra: every constraint projected at once
       in vectorized numpy, per-drone results involvement-weighted-averaged.
       Cheap per sweep; resolves all but the densest conflict clusters.
    2. Gauss-Seidel Dykstra polish, warm-started from phase 1, only if phase
       1 did not converge. Sequential propagation converges in few sweeps
       where consensus averaging stalls (many constraints sharing drones).

    Args:
        nominal: shape (N, D) desired per-drone command (velocity).
        constraints: pairwise affine constraints from a builder.
        norm_cap: per-drone command magnitude cap (e.g. max speed).
        alpha_unused: accepted for signature symmetry; ignored (the gain is
            already baked into ``constraints.bias``).
        max_iterations: maximum sweeps per phase.
        tolerance: convergence threshold on both the residual and the
            per-sweep change.

    Returns:
        `CBFResult` with the safe commands and solver diagnostics.
    """
    del alpha_unused

    commands = np.asarray(nominal, dtype=float).copy()

    gradient_all = constraints.gradient
    bias_all = constraints.bias
    gradient_norm_all = np.sqrt(np.einsum("md,md->m", gradient_all, gradient_all))

    # A pair is immediately infeasible if the two drones coincide (zero
    # gradient) yet the barrier demands a positive margin.
    num_infeasible = int(
        np.count_nonzero(
            (gradient_norm_all < 1e-5) & (bias_all < -tolerance)
        )
    )

    # Prune pairs that no feasible commands can violate:
    # |g . (x_i - x_j)| <= ||g|| * 2 * norm_cap, so bias beyond that bound
    # makes the constraint inactive this step. Cost then scales with the
    # number of actual conflicts instead of all N^2/2 pairs.
    active = bias_all < gradient_norm_all * 2.0 * norm_cap
    idx_i = constraints.idx_i[active]
    idx_j = constraints.idx_j[active]
    gradient = gradient_all[active]
    bias = bias_all[active]

    commands, converged, residual, iterations = _parallel_dykstra(
        commands, idx_i, idx_j, gradient, bias, norm_cap, max_iterations, tolerance
    )
    if not converged:
        commands, converged, residual, polish_iterations = _gauss_seidel_dykstra(
            commands, idx_i, idx_j, gradient, bias, norm_cap, max_iterations,
            tolerance,
        )
        iterations += polish_iterations

    # Final safeguard (as in the 2D reference): the consensus average can
    # leave commands a hair over the cap -- clamp them, and zero any
    # non-finite rows.
    non_finite = ~np.all(np.isfinite(commands), axis=-1)
    if np.any(non_finite):
        commands[non_finite] = 0.0
        converged = False
    norms = np.linalg.norm(commands, axis=-1)
    over_cap = norms > norm_cap
    if np.any(over_cap):
        commands[over_cap] *= (norm_cap / norms[over_cap])[:, None]
    residual = _residual_active(commands, idx_i, idx_j, gradient, bias, norm_cap)

    used_emergency_stop = False
    if not converged or num_infeasible or residual > 5.0 * tolerance:
        commands = _emergency_push_apart(commands, constraints, norm_cap)
        residual = _residual_active(
            commands, idx_i, idx_j, gradient, bias, norm_cap
        )
        used_emergency_stop = True

    deviation = np.linalg.norm(commands - np.asarray(nominal, dtype=float), axis=-1)
    corrected = deviation > 1e-3

    return CBFResult(
        velocities=commands,
        corrected=corrected,
        converged=converged and num_infeasible == 0 and residual <= 5.0 * tolerance,
        residual=residual,
        iterations=iterations,
        used_emergency_stop=used_emergency_stop,
        num_infeasible=num_infeasible,
    )


def filter_velocities(
    nominal_velocities: np.ndarray,
    positions: np.ndarray,
    safety_radius: float,
    max_speed: float,
    alpha: float = 2.5,
    max_iterations: int = 200,
    tolerance: float = 2e-3,
) -> CBFResult:
    """Convenience wrapper: build velocity-CBF constraints and solve them.

    Args:
        nominal_velocities: shape (N, D) desired per-drone velocities (m/s).
        positions: shape (N, D) world-frame drone positions (m).
        safety_radius: per-drone safety radius r (m).
        max_speed: per-drone speed cap (m/s).
        alpha: CBF class-K gain.
        max_iterations: maximum Dykstra sweeps.
        tolerance: solver convergence tolerance.

    Returns:
        `CBFResult` with safe velocities and diagnostics.
    """
    constraints = build_collision_constraints(positions, safety_radius, alpha)
    return solve_safe_commands(
        nominal_velocities,
        constraints,
        norm_cap=max_speed,
        max_iterations=max_iterations,
        tolerance=tolerance,
    )


def _parallel_dykstra(
    commands: np.ndarray,
    idx_i: np.ndarray,
    idx_j: np.ndarray,
    gradient: np.ndarray,
    bias: np.ndarray,
    norm_cap: float,
    max_iterations: int,
    tolerance: float,
) -> tuple[np.ndarray, bool, float, int]:
    """Phase 1: parallel (product-space) Dykstra over the active set.

    Every constraint set is projected simultaneously in vectorized numpy and
    the per-drone results are averaged, weighted by how many sets each drone
    participates in (its pair count + the speed-cap set).

    Returns:
        (commands, converged, residual, iterations).
    """
    num_drones, dim = commands.shape
    num_active = int(idx_i.shape[0])
    pair_norm_sq = 2.0 * np.einsum("md,md->m", gradient, gradient)

    cap_correction = np.zeros_like(commands)
    pair_correction_i = np.zeros((num_active, dim))
    pair_correction_j = np.zeros((num_active, dim))
    involvement = np.bincount(
        np.concatenate([idx_i, idx_j]), minlength=num_drones
    ).astype(float)
    weight = (involvement + 1.0)[:, None]  # +1 for the speed-cap set

    converged = False
    residual = float("inf")
    iterations = 0

    for sweep in range(max_iterations):
        # Speed-cap projections (independent per drone).
        capped = commands + cap_correction
        norms = np.linalg.norm(capped, axis=-1)
        scale = np.where(norms > norm_cap, norm_cap / np.maximum(norms, 1e-12), 1.0)
        cap_projection = capped * scale[:, None]
        cap_correction = capped - cap_projection

        # Pair half-space projections, all pairs at once.
        yi = commands[idx_i] + pair_correction_i
        yj = commands[idx_j] + pair_correction_j
        lhs = np.einsum("md,md->m", gradient, yi - yj) + bias
        step = np.maximum(0.0, -lhs) / np.maximum(pair_norm_sq, 1e-12)
        wi = yi + step[:, None] * gradient
        wj = yj - step[:, None] * gradient
        pair_correction_i = yi - wi
        pair_correction_j = yj - wj

        # Consensus: involvement-weighted average of every set's projection.
        accumulator = cap_projection.copy()
        np.add.at(accumulator, idx_i, wi)
        np.add.at(accumulator, idx_j, wj)
        new_commands = accumulator / weight

        max_delta = float(np.max(np.abs(new_commands - commands)))
        commands = new_commands

        residual = _residual_active(commands, idx_i, idx_j, gradient, bias, norm_cap)
        iterations = sweep + 1
        if residual <= tolerance and max_delta <= tolerance:
            converged = True
            break

    return commands, converged, residual, iterations


def _gauss_seidel_dykstra(
    commands: np.ndarray,
    idx_i: np.ndarray,
    idx_j: np.ndarray,
    gradient: np.ndarray,
    bias: np.ndarray,
    norm_cap: float,
    max_iterations: int,
    tolerance: float,
) -> tuple[np.ndarray, bool, float, int]:
    """Phase 2: sequential Dykstra polish, warm-started from phase 1.

    Project onto the speed-cap balls, then sweep the pair half-spaces one at
    a time so each projection sees the previous one's update. Far better
    sweep-efficiency on densely coupled clusters, at python-loop cost per
    pair -- which is fine because it only ever runs on the pruned active set,
    and only when phase 1 stalled.

    Returns:
        (commands, converged, residual, iterations).
    """
    num_active = int(idx_i.shape[0])
    pair_norm_sq = 2.0 * np.einsum("md,md->m", gradient, gradient)

    cap_correction = np.zeros_like(commands)
    pair_correction_i = np.zeros((num_active, commands.shape[1]))
    pair_correction_j = np.zeros((num_active, commands.shape[1]))

    converged = False
    residual = float("inf")
    iterations = 0

    for sweep in range(max_iterations):
        capped = commands + cap_correction
        norms = np.linalg.norm(capped, axis=-1)
        scale = np.where(norms > norm_cap, norm_cap / np.maximum(norms, 1e-12), 1.0)
        projected = capped * scale[:, None]
        cap_correction = capped - projected
        max_delta = float(np.max(np.abs(projected - commands)))
        commands = projected

        for pair in range(num_active):
            i = idx_i[pair]
            j = idx_j[pair]
            yi = commands[i] + pair_correction_i[pair]
            yj = commands[j] + pair_correction_j[pair]
            grad = gradient[pair]
            lhs = float(grad @ (yi - yj) + bias[pair])

            new_i = yi
            new_j = yj
            if lhs < 0.0 and pair_norm_sq[pair] > 1e-12:
                step = (-lhs) / pair_norm_sq[pair]
                new_i = yi + step * grad
                new_j = yj - step * grad

            pair_correction_i[pair] = yi - new_i
            pair_correction_j[pair] = yj - new_j
            max_delta = max(
                max_delta,
                float(np.max(np.abs(new_i - commands[i]))),
                float(np.max(np.abs(new_j - commands[j]))),
            )
            commands[i] = new_i
            commands[j] = new_j

        residual = _residual_active(commands, idx_i, idx_j, gradient, bias, norm_cap)
        iterations = sweep + 1
        if residual <= tolerance and max_delta <= tolerance:
            converged = True
            break

    return commands, converged, residual, iterations


def _residual_active(
    commands: np.ndarray,
    idx_i: np.ndarray,
    idx_j: np.ndarray,
    gradient: np.ndarray,
    bias: np.ndarray,
    norm_cap: float,
) -> float:
    """Worst violation over the active pair constraints plus the speed cap.

    Pruned (inactive) pairs are satisfied by construction -- their bias
    exceeds the largest possible gradient term -- so checking the active set
    equals checking every pair.
    """
    speed_violation = np.maximum(0.0, np.linalg.norm(commands, axis=-1) - norm_cap)
    worst = float(np.max(speed_violation)) if speed_violation.size else 0.0

    if idx_i.shape[0]:
        relative = commands[idx_i] - commands[idx_j]
        lhs = np.einsum("md,md->m", gradient, relative) + bias
        worst = max(worst, float(np.max(np.maximum(0.0, -lhs))))
    return worst


def _emergency_push_apart(
    commands: np.ndarray,
    constraints: PairwiseConstraints,
    norm_cap: float,
) -> np.ndarray:
    """Fallback when the QP is infeasible: shove violated pairs apart.

    For any pair still violating its constraint, overwrite both drones'
    commands with an away-from-each-other velocity along the line between
    them, magnitude capped at ``norm_cap``. A best-effort safety action, not
    an optimal one.
    """
    commands = commands.copy()
    for pair in range(constraints.num_pairs):
        i = constraints.idx_i[pair]
        j = constraints.idx_j[pair]
        grad = constraints.gradient[pair]
        lhs = float(grad @ (commands[i] - commands[j]) + constraints.bias[pair])
        if lhs >= 0.0:
            continue

        # gradient = 2 (p_i - p_j), so it already points i away from j.
        direction = grad
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            direction = np.zeros_like(grad)
            direction[0] = 1.0
            norm = 1.0
        speed = min(norm_cap, 0.5 * norm_cap + max(0.0, -constraints.barrier[pair]) * 0.1)
        push = direction / norm * speed
        commands[i] = push
        commands[j] = -push
    return commands
