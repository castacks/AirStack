"""CBF collision safety filter — PLACEHOLDER.

This module is the drop-in point for the velocity-CBF collision filter from
~/drone_soccer (drone_soccer/cbf.py). The function signature and return type
below intentionally mirror that implementation:

    filter_velocities(nominal_velocities, positions, safety_radius, max_speed,
                      alpha, max_iterations, tolerance) -> CBFResult

so that the real filter can replace this file verbatim (copy
drone_soccer/cbf.py over this file, or import from it) with zero changes to
the swarm commander.

The intended math (first-order velocity CBF, per drone pair i, j):

    h_ij  = ||p_i - p_j||^2 - (2 r)^2          (>= 0 means safe)
    h_dot = 2 (p_i - p_j) . (v_i - v_j)
    constraint:  h_dot + alpha * h_ij >= 0      (linear in velocities)

    minimize   sum_i || v_safe_i - v_nominal_i ||^2
    s.t.       all pairwise constraints,  ||v_safe_i|| <= max_speed

The placeholder implementation below ONLY enforces the per-drone speed cap.
It does NOT prevent collisions. Keep generous spacing until the real filter
is dropped in.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class CBFResult:
    """Mirrors drone_soccer.cbf.CBFResult."""

    velocities: np.ndarray   # (N, 3) safe velocity commands [m/s]
    corrected: np.ndarray    # (N,) bool, True where nominal was modified
    converged: bool
    residual: float          # worst remaining constraint violation
    iterations: int
    used_emergency_stop: bool
    num_infeasible: int


def filter_velocities(
    nominal_velocities: np.ndarray,
    positions: np.ndarray,
    safety_radius: float,
    max_speed: float,
    alpha: float = 2.5,
    max_iterations: int = 200,
    tolerance: float = 2e-3,
) -> CBFResult:
    """Filter nominal velocities so the swarm stays pairwise-safe.

    Args:
        nominal_velocities: (N, 3) desired world-frame ENU velocities [m/s].
        positions: (N, 3) current world-frame ENU positions [m]. Row order
            matches nominal_velocities and includes EVERY tracked drone
            (hovering, teleoperated obstacle, and externally-flown drones),
            so the filter sees the full swarm state.
        safety_radius: per-drone radius r [m]; pairwise keep-out is 2r.
        max_speed: per-drone velocity norm cap [m/s].
        alpha: CBF class-K gain (higher = more aggressive recovery).
        max_iterations: solver iteration budget.
        tolerance: solver convergence tolerance.

    Returns:
        CBFResult with the (N, 3) safe velocities.
    """
    nominal = np.asarray(nominal_velocities, dtype=float)

    # =====================================================================
    # TODO(yikuan): replace this block with the real CBF-QP filter from
    # ~/drone_soccer/drone_soccer/cbf.py (hybrid Dykstra projection:
    # build_collision_constraints() + solve_safe_commands() + emergency
    # push-apart fallback). `positions`, `safety_radius`, and `alpha` are
    # unused until then.
    # =====================================================================
    speeds = np.linalg.norm(nominal, axis=1)
    scale = np.ones_like(speeds)
    over = speeds > max_speed
    scale[over] = max_speed / speeds[over]
    safe = nominal * scale[:, None]

    return CBFResult(
        velocities=safe,
        corrected=over,
        converged=True,
        residual=0.0,
        iterations=0,
        used_emergency_stop=False,
        num_infeasible=0,
    )
