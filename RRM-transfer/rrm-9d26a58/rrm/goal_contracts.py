"""Embodiment-neutral goal intake and capability routing contracts.

This module sits before the existing C01 ``TaskRequest`` adapters.  It preserves a
qualitative operator objective without requiring coordinates, joint values, distances,
or even an embodiment choice.  Routing is capability based and proposal-only: it does
not establish physical feasibility, permission, safety, admission, or execution.
"""

from __future__ import annotations

from enum import Enum
import math

from pydantic import BaseModel, ConfigDict, model_validator

from .contracts import CapabilityDeclaration
from .task_contracts import TaskRequest


class _ImmutableRecord(BaseModel):
    model_config = ConfigDict(frozen=True)

    @staticmethod
    def _require(value: str, name: str) -> str:
        if not value.strip():
            raise ValueError(f"{name} is required")
        return value


class GoalRequest(_ImmutableRecord):
    """Operator-level objective before an embodiment or numeric command is chosen."""

    goal_id: str
    revision: str
    objective: str
    context_refs: tuple[str, ...] = ()
    required_operations: frozenset[str]
    required_resources: frozenset[str] = frozenset()
    qualitative_constraints: tuple[str, ...] = ()
    preferred_embodiment_id: str | None = None

    @model_validator(mode="after")
    def validate_goal(self) -> "GoalRequest":
        for name in ("goal_id", "revision", "objective"):
            self._require(getattr(self, name), name)
        if not self.required_operations:
            raise ValueError("at least one required operation is required")
        for name, values in (
            ("context_refs", self.context_refs),
            ("required_operations", self.required_operations),
            ("required_resources", self.required_resources),
            ("qualitative_constraints", self.qualitative_constraints),
        ):
            if any(not isinstance(value, str) or not value.strip() for value in values):
                raise ValueError(f"{name} values must be nonempty strings")
        if self.preferred_embodiment_id is not None:
            self._require(self.preferred_embodiment_id, "preferred_embodiment_id")
        return self


class RouteStatus(str, Enum):
    """Capability routing outcome; none of these values authorizes execution."""

    SELECTED = "SELECTED"
    CANDIDATES = "CANDIDATES"
    UNSUPPORTED = "UNSUPPORTED"
    UNAVAILABLE = "UNAVAILABLE"


class EmbodimentRoute(_ImmutableRecord):
    goal_id: str
    goal_revision: str
    status: RouteStatus
    candidate_embodiment_ids: tuple[str, ...] = ()
    selected_embodiment_id: str | None = None
    reason_codes: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_route(self) -> "EmbodimentRoute":
        self._require(self.goal_id, "goal_id")
        self._require(self.goal_revision, "goal_revision")
        if len(set(self.candidate_embodiment_ids)) != len(self.candidate_embodiment_ids):
            raise ValueError("candidate embodiment IDs must be unique")
        if any(not item.strip() for item in self.candidate_embodiment_ids):
            raise ValueError("candidate embodiment IDs must be nonempty")
        if any(not reason.strip() for reason in self.reason_codes):
            raise ValueError("reason codes must be nonempty")
        if self.status is RouteStatus.SELECTED:
            if (self.selected_embodiment_id is None
                    or self.candidate_embodiment_ids != (self.selected_embodiment_id,)
                    or self.reason_codes):
                raise ValueError("selected route requires exactly its selected candidate")
        elif self.status is RouteStatus.CANDIDATES:
            if len(self.candidate_embodiment_ids) < 2 or self.selected_embodiment_id is not None:
                raise ValueError("candidate route requires multiple unselected embodiments")
        elif (self.candidate_embodiment_ids or self.selected_embodiment_id is not None
              or not self.reason_codes):
            raise ValueError("failed route requires reasons and no candidates")
        return self


def route_goal(goal: GoalRequest,
               declarations: tuple[CapabilityDeclaration, ...]) -> EmbodimentRoute:
    """Return semantic capability candidates without ranking or feasibility claims.

    A preferred embodiment is a routing constraint, not a product-name branch in the
    reasoner.  Multiple candidates stay explicit for a later feasibility/policy choice.
    """
    by_id = {item.embodiment_id: item for item in declarations}
    if len(by_id) != len(declarations):
        raise ValueError("capability declarations must have unique embodiment IDs")

    considered = declarations
    if goal.preferred_embodiment_id is not None:
        preferred = by_id.get(goal.preferred_embodiment_id)
        if preferred is None:
            return EmbodimentRoute(
                goal_id=goal.goal_id, goal_revision=goal.revision,
                status=RouteStatus.UNSUPPORTED,
                reason_codes=("unknown_preferred_embodiment",),
            )
        considered = (preferred,)

    semantically_supported = tuple(
        item for item in considered
        if goal.required_operations <= item.operations
        and goal.required_resources <= item.resources
        and item.limits_ref.strip()
    )
    available = tuple(
        item for item in semantically_supported
        if goal.required_resources <= item.available_resources
    )
    if not semantically_supported:
        return EmbodimentRoute(
            goal_id=goal.goal_id, goal_revision=goal.revision,
            status=RouteStatus.UNSUPPORTED,
            reason_codes=("no_semantic_capability_match",),
        )
    if not available:
        return EmbodimentRoute(
            goal_id=goal.goal_id, goal_revision=goal.revision,
            status=RouteStatus.UNAVAILABLE,
            reason_codes=("matching_resources_unavailable",),
        )
    candidate_ids = tuple(sorted(item.embodiment_id for item in available))
    if len(candidate_ids) == 1:
        return EmbodimentRoute(
            goal_id=goal.goal_id, goal_revision=goal.revision,
            status=RouteStatus.SELECTED,
            candidate_embodiment_ids=candidate_ids,
            selected_embodiment_id=candidate_ids[0],
        )
    return EmbodimentRoute(
        goal_id=goal.goal_id, goal_revision=goal.revision,
        status=RouteStatus.CANDIDATES,
        candidate_embodiment_ids=candidate_ids,
    )


class C01TaskBinding(_ImmutableRecord):
    """Auditable proposal-only binding from one selected route to C01.

    The binding intentionally carries no feasibility, safety, approval, admission, or
    dispatch result.  The selected embodiment adapter must still perform its normal
    state, capability, and feasibility checks after receiving ``task``.
    """

    goal: GoalRequest
    capability_revision: str
    route: EmbodimentRoute
    task: TaskRequest

    @model_validator(mode="after")
    def validate_binding(self) -> "C01TaskBinding":
        self._require(self.capability_revision, "capability_revision")
        if self.route.status is not RouteStatus.SELECTED:
            raise ValueError("C01 binding requires one selected embodiment route")
        if (self.route.goal_id != self.goal.goal_id
                or self.route.goal_revision != self.goal.revision
                or self.task.task_id != self.goal.goal_id
                or self.task.revision != self.goal.revision
                or self.task.objective != self.goal.objective
                or self.task.context_refs != self.goal.context_refs
                or self.task.requested_embodiment_id != self.route.selected_embodiment_id):
            raise ValueError("C01 binding references do not match")
        return self


def bind_selected_route_to_c01(
    goal: GoalRequest,
    route: EmbodimentRoute,
    capability: CapabilityDeclaration,
    *,
    constraints_revision: str,
    issuer_id: str,
    permission_revision: str,
) -> C01TaskBinding:
    """Translate one selected semantic route into the existing C01 task shape.

    The capability is rechecked instead of trusting a detached route record.  Required
    C01 authority references have no defaults so qualitative intake cannot invent
    permission.  Successful translation remains proposal-only.
    """
    if route.goal_id != goal.goal_id or route.goal_revision != goal.revision:
        raise ValueError("route does not reference this goal revision")
    if route.status is not RouteStatus.SELECTED or route.selected_embodiment_id is None:
        raise ValueError("route must select exactly one embodiment")
    if capability.embodiment_id != route.selected_embodiment_id:
        raise ValueError("selected route and capability embodiment do not match")

    rechecked = route_goal(goal, (capability,))
    if (rechecked.status is not RouteStatus.SELECTED
            or rechecked.selected_embodiment_id != route.selected_embodiment_id):
        raise ValueError(f"selected capability is no longer routable: {rechecked.status.value}")

    task = TaskRequest(
        task_id=goal.goal_id,
        revision=goal.revision,
        objective=goal.objective,
        context_refs=goal.context_refs,
        constraints_revision=constraints_revision,
        issuer_id=issuer_id,
        permission_revision=permission_revision,
        requested_embodiment_id=route.selected_embodiment_id,
    )
    return C01TaskBinding(
        goal=goal,
        capability_revision=capability.revision,
        route=route,
        task=task,
    )


class ParameterSource(str, Enum):
    OPERATOR = "OPERATOR"
    OBSERVATION = "OBSERVATION"
    CONSTRAINT_PROFILE = "CONSTRAINT_PROFILE"
    VERSIONED_POLICY = "VERSIONED_POLICY"
    ADAPTER_DEFAULT = "ADAPTER_DEFAULT"


class ParameterBinding(_ImmutableRecord):
    """One adapter-local value with inspectable provenance."""

    name: str
    value: bool | int | float | str
    unit: str
    source: ParameterSource
    source_ref: str

    @model_validator(mode="after")
    def validate_binding(self) -> "ParameterBinding":
        for name in ("name", "unit", "source_ref"):
            self._require(getattr(self, name), name)
        if isinstance(self.value, float) and not math.isfinite(self.value):
            raise ValueError("numeric parameter value must be finite")
        return self


class GroundingStatus(str, Enum):
    RESOLVED = "RESOLVED"
    NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
    INFEASIBLE = "INFEASIBLE"


class ParameterResolution(_ImmutableRecord):
    """Adapter-local parameter result, still before safety and permission checks."""

    goal_id: str
    goal_revision: str
    embodiment_id: str
    adapter_revision: str
    status: GroundingStatus
    bindings: tuple[ParameterBinding, ...] = ()
    unresolved_refs: tuple[str, ...] = ()
    reason_codes: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_resolution(self) -> "ParameterResolution":
        for name in ("goal_id", "goal_revision", "embodiment_id", "adapter_revision"):
            self._require(getattr(self, name), name)
        names = [item.name for item in self.bindings]
        if len(set(names)) != len(names):
            raise ValueError("parameter names must be unique")
        if any(not item.strip() for item in self.unresolved_refs + self.reason_codes):
            raise ValueError("unresolved references and reasons must be nonempty")
        if self.status is GroundingStatus.RESOLVED:
            if self.unresolved_refs or self.reason_codes:
                raise ValueError("resolved parameters cannot retain unresolved reasons")
        elif self.status is GroundingStatus.NEEDS_CLARIFICATION:
            if not self.unresolved_refs or self.reason_codes:
                raise ValueError("clarification requires unresolved references only")
        elif not self.reason_codes or self.unresolved_refs:
            raise ValueError("infeasible grounding requires reason codes only")
        return self
