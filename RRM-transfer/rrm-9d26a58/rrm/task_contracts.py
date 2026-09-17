"""Proposal-only C01, C04 and C05 logical contract records.

These records deliberately stop before safety admission or execution.  They have no
ROS dependency and cannot construct a command for an embodiment.
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .schema import AbstractAction, Predicate
from .verbs import VERB_TABLE


class InteractionType(str, Enum):
    CLARIFICATION = "CLARIFICATION"
    APPROVAL = "APPROVAL"


class InteractionStatus(str, Enum):
    PENDING = "PENDING"
    GRANTED = "GRANTED"
    DECLINED = "DECLINED"
    EXPIRED = "EXPIRED"


class IntentStatus(str, Enum):
    READY = "READY"
    NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
    UNSUPPORTED = "UNSUPPORTED"


class _ImmutableRecord(BaseModel):
    model_config = ConfigDict(frozen=True)

    @staticmethod
    def _require(value: str, name: str) -> str:
        if not value.strip():
            raise ValueError(f"{name} is required")
        return value


class TaskRequest(_ImmutableRecord):
    """C01 task revision; permission is a reference, never an implicit allow."""

    task_id: str
    revision: str
    objective: str
    context_refs: tuple[str, ...] = ()
    constraints_revision: str
    issuer_id: str
    permission_revision: str
    requested_embodiment_id: str

    @model_validator(mode="after")
    def validate_refs(self) -> "TaskRequest":
        for name in ("task_id", "revision", "objective", "constraints_revision",
                     "issuer_id", "permission_revision", "requested_embodiment_id"):
            self._require(getattr(self, name), name)
        if any(not ref.strip() for ref in self.context_refs):
            raise ValueError("context references must be nonempty")
        return self


class Interaction(_ImmutableRecord):
    """C01 clarification or approval scoped to exactly one task revision."""

    interaction_id: str
    task_id: str
    task_revision: str
    kind: InteractionType
    issue: str
    status: InteractionStatus = InteractionStatus.PENDING
    response: str | None = None
    scope_revision: str

    @model_validator(mode="after")
    def validate_interaction(self) -> "Interaction":
        for name in ("interaction_id", "task_id", "task_revision", "issue", "scope_revision"):
            self._require(getattr(self, name), name)
        if self.status is InteractionStatus.PENDING and self.response is not None:
            raise ValueError("pending interaction cannot contain a response")
        if self.status is InteractionStatus.GRANTED and not (self.response or "").strip():
            raise ValueError("granted interaction requires an explicit response")
        return self


class ReasoningResult(_ImmutableRecord):
    """C04 interpreted intent; it is a proposal and cannot authorize execution."""

    task_id: str
    task_revision: str
    state_revision: str
    capability_revision: str
    status: IntentStatus
    grounded_goal: Predicate | None = None
    grounded_entities: tuple[str, ...] = ()
    ambiguity_refs: tuple[str, ...] = ()
    explanation: str = ""

    @model_validator(mode="after")
    def validate_intent(self) -> "ReasoningResult":
        for name in ("task_id", "task_revision", "state_revision", "capability_revision"):
            self._require(getattr(self, name), name)
        if any(not entity.strip() for entity in self.grounded_entities):
            raise ValueError("grounded entities must be nonempty")
        if self.status is IntentStatus.READY:
            if self.grounded_goal is None or self.ambiguity_refs:
                raise ValueError("ready intent requires a goal and no unresolved ambiguity")
        elif self.grounded_goal is not None:
            raise ValueError("unresolved intent cannot carry an executable goal")
        if self.status is IntentStatus.NEEDS_CLARIFICATION and not self.ambiguity_refs:
            raise ValueError("clarification status requires ambiguity references")
        return self


class PlannedAction(_ImmutableRecord):
    """One C05 action with authored semantics and a prior feasibility result."""

    action: AbstractAction
    dependencies: tuple[str, ...] = ()
    semantics_revision: str
    feasibility_ref: str
    expected_effect_window_revision: str

    @model_validator(mode="after")
    def validate_action(self) -> "PlannedAction":
        if not self.action.id.strip():
            raise ValueError("action ID is required")
        if len(self.action.targets) != VERB_TABLE[self.action.verb].arity:
            raise ValueError(f"{self.action.verb.value} has invalid target arity")
        for name in ("semantics_revision", "feasibility_ref", "expected_effect_window_revision"):
            self._require(getattr(self, name), name)
        if any(not dependency.strip() for dependency in self.dependencies):
            raise ValueError("dependencies must be nonempty action IDs")
        return self


class PlanProposal(_ImmutableRecord):
    """C05 versioned plan proposal, intentionally before C06 safety admission."""

    plan_id: str
    version: int = Field(ge=0)
    task_id: str
    task_revision: str
    intent: ReasoningResult
    state_revision: str
    capability_revision: str
    actions: tuple[PlannedAction, ...]
    recovery_budget: int = Field(ge=0)

    @model_validator(mode="after")
    def validate_plan(self) -> "PlanProposal":
        for name in ("plan_id", "task_id", "task_revision", "state_revision", "capability_revision"):
            self._require(getattr(self, name), name)
        if self.intent.status is not IntentStatus.READY:
            raise ValueError("only ready intent may be planned")
        if (self.intent.task_id != self.task_id or self.intent.task_revision != self.task_revision
                or self.intent.state_revision != self.state_revision
                or self.intent.capability_revision != self.capability_revision):
            raise ValueError("plan references do not match its intent")
        action_ids = [item.action.id for item in self.actions]
        if len(set(action_ids)) != len(action_ids):
            raise ValueError("plan action IDs must be unique")
        known = set(action_ids)
        graph = {item.action.id: set(item.dependencies) for item in self.actions}
        if any(not dependencies <= known for dependencies in graph.values()):
            raise ValueError("action dependency is not in this plan")
        if any(action_id in dependencies for action_id, dependencies in graph.items()):
            raise ValueError("action cannot depend on itself")
        visited: set[str] = set()
        active: set[str] = set()

        def visit(action_id: str) -> None:
            if action_id in active:
                raise ValueError("plan dependencies contain a cycle")
            if action_id not in visited:
                active.add(action_id)
                for dependency in graph[action_id]:
                    visit(dependency)
                active.remove(action_id)
                visited.add(action_id)

        for action_id in graph:
            visit(action_id)
        return self
