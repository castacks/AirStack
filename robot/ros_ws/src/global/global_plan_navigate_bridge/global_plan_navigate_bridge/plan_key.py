# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Pure-python plan comparison used by the bridge (hermetic, no ROS import).

A global planner typically re-publishes its current plan periodically; the
bridge must only replace the active NavigateTask goal when the plan really
changed, otherwise droan_gl would be cancelled and restarted every tick.
"""
import math
from typing import Optional, Sequence, Tuple

PlanKey = Tuple[int, Tuple[float, float, float]]


def plan_key(points: Sequence[Tuple[float, float, float]]) -> Optional[PlanKey]:
    """(pose count, final xyz) of a plan, or None for an empty plan."""
    if not points:
        return None
    x, y, z = points[-1]
    return (len(points), (float(x), float(y), float(z)))


def plan_changed(prev: Optional[PlanKey], new: Optional[PlanKey], min_change_m: float) -> bool:
    """True when ``new`` should replace the goal built from ``prev``.

    Rules: empty<->non-empty is always a change; a different pose count is a
    change; otherwise the final pose must have moved more than ``min_change_m``.
    """
    if prev is None or new is None:
        return prev is not new and (prev is None) != (new is None)
    if prev[0] != new[0]:
        return True
    return math.dist(prev[1], new[1]) > min_change_m
