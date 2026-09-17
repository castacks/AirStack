"""Dry-run embodiment bridge from an RRM semantic navigation task to AirStack.

This is an example adapter, not RRM's task-level architecture.  It consumes the
same C01/C02/C03 records that a hand, ground robot, or other adapter would consume;
only this module knows that its local target binding is a map-frame flight route.
It creates a typed proposal but has no ROS or dispatch dependency.
"""

from __future__ import annotations

import re
from enum import Enum

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .airstack_drone import DroneTaskKind, DroneTaskProposal, MapWaypoint
from .contracts import CapabilityDeclaration, Truth
from .schema import AbstractAction, Predicate, SELF, Verb
from .state_contracts import FactKey, StateSnapshot
from .task_contracts import IntentStatus, PlanProposal, PlannedAction, ReasoningResult, TaskRequest


_NAVIGATE_OBJECTIVE = re.compile(r"^navigate to ([A-Za-z0-9][A-Za-z0-9_.-]*)$", re.IGNORECASE)
_AIRFRAME_RESOURCE = frozenset({"airframe"})


class DroneDecisionStatus(str, Enum):
    """A proposal state; no value in this enum authorizes execution."""

    READY = "READY"
    HOLD = "HOLD"
    NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
    UNSUPPORTED = "UNSUPPORTED"


class DroneNavigationTarget(BaseModel):
    """Adapter-local grounding for one semantic target entity.

    Map coordinates appear here, at the embodiment boundary, rather than in the
    RRM task request, reasoning result, or semantic plan.
    """

    model_config = ConfigDict(frozen=True)

    entity_id: str
    waypoints: tuple[MapWaypoint, ...]
    goal_tolerance_m: float = Field(gt=0)

    @model_validator(mode="after")
    def validate_target(self) -> "DroneNavigationTarget":
        if not self.entity_id.strip() or not self.waypoints:
            raise ValueError("target entity ID and at least one waypoint are required")
        return self


class DroneDecision(BaseModel):
    """C04/C05 decision plus a C07-shaped proposal, strictly before admission."""

    model_config = ConfigDict(frozen=True)

    status: DroneDecisionStatus
    task_id: str
    reasons: tuple[str, ...] = ()
    intent: ReasoningResult | None = None
    plan: PlanProposal | None = None
    proposal: DroneTaskProposal | None = None

    @model_validator(mode="after")
    def validate_decision(self) -> "DroneDecision":
        if not self.task_id.strip():
            raise ValueError("task ID is required")
        if any(not reason.strip() for reason in self.reasons):
            raise ValueError("decision reasons must be nonempty")
        ready = self.status is DroneDecisionStatus.READY
        present = (self.intent is not None, self.plan is not None, self.proposal is not None)
        if ready and (not all(present) or self.reasons):
            raise ValueError("ready decision requires complete artifacts and no refusal reason")
        if not ready and any(present):
            raise ValueError("non-ready decision cannot contain a plan or proposal")
        return self


class AirStackDroneDecisionBridge:
    """Compile one generic ``NAVIGATE_TO`` plan node into an AirStack proposal.

    The recognized objective grammar is deliberately narrow and deterministic:
    ``navigate to <semantic-target-id>``.  A future text/VLM component may create a
    conforming :class:`TaskRequest` or target selection, but cannot bypass this
    adapter's evidence and capability checks.
    """

    def __init__(self, *, embodiment_id: str, robot_name: str,
                 targets: tuple[DroneNavigationTarget, ...],
                 semantics_revision: str = "rrm/verbs/v1",
                 effect_window_revision: str = "airframe/navigation-effect/v1") -> None:
        if not embodiment_id.strip() or not robot_name.strip():
            raise ValueError("embodiment ID and robot name are required")
        if not semantics_revision.strip() or not effect_window_revision.strip():
            raise ValueError("semantic and effect-window revisions are required")
        ids = [target.entity_id for target in targets]
        if len(ids) != len(set(ids)):
            raise ValueError("drone target entity IDs must be unique")
        self.embodiment_id = embodiment_id
        self.robot_name = robot_name
        self.targets = {target.entity_id: target for target in targets}
        self.semantics_revision = semantics_revision
        self.effect_window_revision = effect_window_revision

    def decide(self, task: TaskRequest, snapshot: StateSnapshot,
               capabilities: CapabilityDeclaration, *, now_monotonic_s: float) -> DroneDecision:
        """Return a complete dry-run proposal or an explicit no-motion decision."""
        if task.requested_embodiment_id != self.embodiment_id:
            return self._refusal(DroneDecisionStatus.UNSUPPORTED, task, "wrong_embodiment")
        if snapshot.task_id != task.task_id:
            return self._refusal(DroneDecisionStatus.HOLD, task, "snapshot_task_mismatch")
        match = _NAVIGATE_OBJECTIVE.fullmatch(task.objective.strip())
        if match is None:
            return self._refusal(DroneDecisionStatus.NEEDS_CLARIFICATION, task,
                                 "unrecognized_navigation_objective")
        target_id = match.group(1)
        target = self.targets.get(target_id)
        if target is None:
            return self._refusal(DroneDecisionStatus.NEEDS_CLARIFICATION, task,
                                 f"unresolved_target:{target_id}")
        reasons = capabilities.rejection_reasons(Verb.NAVIGATE_TO.value, _AIRFRAME_RESOURCE)
        if reasons:
            return self._refusal(DroneDecisionStatus.UNSUPPORTED, task, *reasons)
        localization = snapshot.resolve(
            FactKey(subject=target_id, predicate="localized"), now_monotonic_s=now_monotonic_s,
        )
        if localization is not Truth.TRUE:
            reason = "target_localization_false" if localization is Truth.FALSE else "target_localization_unknown"
            return self._refusal(DroneDecisionStatus.HOLD, task, reason)

        intent = ReasoningResult(
            task_id=task.task_id,
            task_revision=task.revision,
            state_revision=snapshot.revision,
            capability_revision=capabilities.revision,
            status=IntentStatus.READY,
            grounded_goal=Predicate(name="near", subject=SELF, obj=target_id),
            grounded_entities=(target_id,),
            explanation="deterministic semantic navigation baseline",
        )
        action = AbstractAction(id=f"navigate-{target_id}", verb=Verb.NAVIGATE_TO, targets=[target_id])
        plan = PlanProposal(
            plan_id=f"{task.task_id}/navigation",
            version=0,
            task_id=task.task_id,
            task_revision=task.revision,
            intent=intent,
            state_revision=snapshot.revision,
            capability_revision=capabilities.revision,
            actions=(PlannedAction(
                action=action,
                semantics_revision=self.semantics_revision,
                feasibility_ref=f"{capabilities.revision}/NAVIGATE_TO/{target_id}",
                expected_effect_window_revision=self.effect_window_revision,
            ),),
            recovery_budget=0,
        )
        proposal = DroneTaskProposal(
            task_id=task.task_id,
            action_id=action.id,
            robot_name=self.robot_name,
            kind=DroneTaskKind.NAVIGATE,
            frame_id="map",
            waypoints=target.waypoints,
            goal_tolerance_m=target.goal_tolerance_m,
        )
        return DroneDecision(
            status=DroneDecisionStatus.READY,
            task_id=task.task_id,
            intent=intent,
            plan=plan,
            proposal=proposal,
        )

    def compile_plan(self, plan: PlanProposal, task: TaskRequest, snapshot: StateSnapshot,
                     capabilities: CapabilityDeclaration, *, now_monotonic_s: float) -> DroneDecision:
        """Compile a learned C05 unchanged; never re-interpret the objective text.

        This first embodiment adapter supports exactly one semantic navigation node.
        Its explicit limitations must not silently truncate a more capable brain's plan.
        """
        if (task.requested_embodiment_id != self.embodiment_id
                or capabilities.embodiment_id != self.embodiment_id):
            return self._refusal(DroneDecisionStatus.UNSUPPORTED, task, "wrong_embodiment")
        if (plan.task_id != task.task_id or snapshot.task_id != task.task_id
                or plan.task_revision != task.revision
                or plan.state_revision != snapshot.revision
                or plan.capability_revision != capabilities.revision):
            return self._refusal(DroneDecisionStatus.HOLD, task, "plan_context_mismatch")
        if len(plan.actions) != 1 or plan.actions[0].dependencies:
            return self._refusal(DroneDecisionStatus.UNSUPPORTED, task, "requires_single_navigation_node")
        node = plan.actions[0]
        action = node.action
        if (action.verb is not Verb.NAVIGATE_TO or len(action.targets) != 1
                or action.params):
            return self._refusal(DroneDecisionStatus.UNSUPPORTED, task, "unsupported_navigation_node")
        target_id = action.targets[0]
        if target_id not in plan.intent.grounded_entities or plan.intent.ambiguity_refs:
            return self._refusal(DroneDecisionStatus.HOLD, task, "ungrounded_or_ambiguous_plan")
        target = self.targets.get(target_id)
        if target is None:
            return self._refusal(DroneDecisionStatus.HOLD, task, "missing_map_binding")
        reasons = capabilities.rejection_reasons(action.verb.value, _AIRFRAME_RESOURCE)
        if reasons:
            return self._refusal(DroneDecisionStatus.UNSUPPORTED, task, *reasons)
        if snapshot.resolve(FactKey(subject=target_id, predicate="localized"),
                            now_monotonic_s=now_monotonic_s) is not Truth.TRUE:
            return self._refusal(DroneDecisionStatus.HOLD, task, "target_localization_not_true")
        return DroneDecision(
            status=DroneDecisionStatus.READY, task_id=task.task_id,
            intent=plan.intent, plan=plan,
            proposal=DroneTaskProposal(
                task_id=task.task_id, action_id=action.id, robot_name=self.robot_name,
                kind=DroneTaskKind.NAVIGATE, frame_id="map", waypoints=target.waypoints,
                goal_tolerance_m=target.goal_tolerance_m,
            ),
        )

    @staticmethod
    def _refusal(status: DroneDecisionStatus, task: TaskRequest, *reasons: str) -> DroneDecision:
        return DroneDecision(status=status, task_id=task.task_id, reasons=tuple(reasons))
