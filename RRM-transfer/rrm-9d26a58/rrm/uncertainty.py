"""Versioned, scene-independent aggregate uncertainty over world evidence."""

from __future__ import annotations

import math

from .schema import WorldState


UNCERTAINTY_PROVENANCE = "world_evidence_v2"


def aggregate_uncertainty(state: WorldState) -> float:
    """Conservative maximum deficit over object, relation and coverage evidence.

    This mock-reference rule uses exact simulation ticks. It is intentionally not a
    calibrated physical sensor model or wall-clock freshness threshold.
    """
    if type(state.t) is not int or state.t < 0:
        raise ValueError("world observation tick must be nonnegative")
    if not state.objects:
        return 1.0
    deficits: list[float] = []
    for obj in state.objects:
        if not math.isfinite(obj.confidence) or not 0.0 <= obj.confidence <= 1.0:
            raise ValueError("object confidence must be bounded")
        if type(obj.observed_t) is not int or obj.observed_t != state.t:
            deficits.append(1.0)
        else:
            deficits.append(1.0 - obj.confidence)
    for relation in state.relations:
        if not math.isfinite(relation.confidence) \
                or not 0.0 <= relation.confidence <= 1.0:
            raise ValueError("relation confidence must be bounded")
        if type(relation.observed_t) is not int or relation.observed_t != state.t:
            deficits.append(1.0)
        else:
            deficits.append(1.0 - relation.confidence)
    if state.relations_complete and (
            type(state.relations_observed_t) is not int
            or state.relations_observed_t != state.t):
        deficits.append(1.0)
    return max(deficits)


def validate_uncertainty(state: WorldState) -> None:
    if state.uncertainty_provenance != UNCERTAINTY_PROVENANCE:
        raise ValueError("world uncertainty is not evidence-bound")
    if state.uncertainty != aggregate_uncertainty(state):
        raise ValueError("world uncertainty contradicts world evidence")
