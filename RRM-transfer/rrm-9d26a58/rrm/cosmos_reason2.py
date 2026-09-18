"""Body-agnostic Cosmos Reason2 C01/C02/C03 to C04/C05 reasoning boundary.

This module deliberately has no Transformers, ROS, simulator, execution, physics or
control import.  A runtime supplies raw VLM text; the deterministic boundary below
serializes evidence and admits only a narrow, schema-validated semantic candidate.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from .contracts import CapabilityDeclaration, Truth
from .schema import AbstractAction, Predicate, Verb
from .state_contracts import StateSnapshot
from .task_contracts import IntentStatus, PlanProposal, PlannedAction, ReasoningResult, TaskRequest
from .verbs import VERB_TABLE


class CosmosCandidateStatus(str, Enum):
    """Outcome of model-output validation; none of these values authorizes execution."""

    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
    UNSUPPORTED = "UNSUPPORTED"


@dataclass(frozen=True)
class CosmosReasoningInput:
    """The body-agnostic C01/C02/C03 context given to a learned reasoner."""

    task: TaskRequest
    snapshot: StateSnapshot
    capabilities: CapabilityDeclaration
    now_monotonic_s: float

    def __post_init__(self) -> None:
        if self.snapshot.task_id != self.task.task_id:
            raise ValueError("snapshot task must match task request")
        if self.now_monotonic_s < 0:
            raise ValueError("reasoning clock must be nonnegative")


class CosmosReasoningCandidate(BaseModel):
    """Replayable learned candidate before C06/C07/C08/C09 boundaries."""

    model_config = ConfigDict(frozen=True)

    status: CosmosCandidateStatus
    task_id: str
    raw_response: str
    reasons: tuple[str, ...] = ()
    intent: ReasoningResult | None = None
    plan: PlanProposal | None = None

    def model_post_init(self, __context: Any) -> None:
        if not self.task_id.strip():
            raise ValueError("task ID is required")
        if not self.raw_response.strip():
            raise ValueError("raw model response is required")
        if any(not item.strip() for item in self.reasons):
            raise ValueError("candidate reasons must be nonempty")
        accepted = self.status is CosmosCandidateStatus.ACCEPTED
        if accepted != (self.intent is not None and self.plan is not None):
            raise ValueError("accepted candidate requires exactly C04 and C05 artifacts")
        if not accepted and (self.intent is not None or self.plan is not None):
            raise ValueError("non-accepted candidate cannot carry a plan")


def _evidence_record(item: Any, *, now_monotonic_s: float) -> dict[str, Any]:
    return {
        "subject": item.key.subject,
        "predicate": item.key.predicate,
        "object": item.key.obj,
        "truth": item.truth.value,
        "provenance": item.provenance.value,
        "source_ref": item.source_ref,
        "fresh": item.is_fresh(now_monotonic_s),
    }


def render_cosmos_prompt(context: CosmosReasoningInput) -> str:
    """Render only declared C01/C02/C03 facts; coordinates stay outside RRM semantics."""
    payload = {
        "task": context.task.model_dump(mode="json"),
        "state": {
            "snapshot_id": context.snapshot.snapshot_id,
            "revision": context.snapshot.revision,
            "episode_id": context.snapshot.episode_id,
            "entity_ids": sorted(_fresh_entities(context)),
            "evidence": [_evidence_record(item, now_monotonic_s=context.now_monotonic_s)
                         for item in context.snapshot.evidence],
        },
        "capabilities": {
            "embodiment_id": context.capabilities.embodiment_id,
            "revision": context.capabilities.revision,
            "operations": sorted(context.capabilities.operations),
            "resources": sorted(context.capabilities.resources),
            "available_resources": sorted(context.capabilities.available_resources),
            "limits_ref": context.capabilities.limits_ref,
        },
    }
    allowed_verbs = sorted(context.capabilities.operations & {verb.value for verb in Verb})
    action_semantics = {
        verb: {
            "target_count": VERB_TABLE[Verb(verb)].arity,
            "expected_effect_templates": [effect.model_dump(mode="json")
                                          for effect in VERB_TABLE[Verb(verb)].expected_effects],
        }
        for verb in allowed_verbs
    }
    return "\n".join((
        "You are the learned reasoning component of a body-agnostic robotics reasoning model.",
        "Interpret the task using only the evidence and capabilities supplied below. ",
        "Never invent an entity, fact, coordinate, capability, safety approval, or execution result.",
        "You propose semantic intent and plan candidates only. You do not control a robot.",
        "Return exactly one JSON object with this schema:",
        '{"status":"READY|NEEDS_CLARIFICATION|UNSUPPORTED",'
        '"grounded_goal":{"name":str,"subject":str,"obj":str|number|null},'
        '"grounded_entities":[str],"ambiguity_refs":[str],"explanation":str,'
        '"actions":[{"id":str,"verb":str,"targets":[str],"dependencies":[str]}],'
        '"recovery_budget":nonnegative_integer}',
        "For READY, give a grounded_goal, no ambiguity_refs, and one or more actions. ",
        "For READY, grounded_entities is mandatory: list every fresh C02 entity named "
        "by grounded_goal.subject, grounded_goal.obj, or an action target. For example, "
        "if an action targets loading_bay_marker, then grounded_entities must include "
        "loading_bay_marker. Do not leave it empty. ",
        "Use exact strings from state.entity_ids for grounded_entities and action targets. "
        "Evidence object values such as kind/color descriptions are properties, not entity IDs. "
        "Never replace an ID with its descriptive label. Do not put $self in grounded_entities.",
        "grounded_goal.name is a symbolic predicate, never the task sentence. "
        "grounded_goal.subject is an entity ID or $self (the acting embodiment). "
        "An entity-valued grounded_goal.obj must use its exact ID. "
        "For NAVIGATE_TO with targets=[TARGET_ID], the goal is "
        '{"name":"near","subject":"$self","obj":TARGET_ID}. '
        "Select TARGET_ID from the evidence according to the task; this template does not select it.",
        "Authored action semantics follow. $0/$1 refer to action targets by index; "
        "$self refers to the acting embodiment. Expected effects describe intended outcomes, "
        "not observations that those outcomes have occurred:",
        json.dumps(action_semantics, sort_keys=True, separators=(",", ":")),
        "For NEEDS_CLARIFICATION, provide nonempty ambiguity_refs and no actions. ",
        "For UNSUPPORTED, provide no goal and no actions.",
        f"Allowed action verbs: {json.dumps(allowed_verbs)}.",
        "C01/C02/C03 input follows:",
        json.dumps(payload, sort_keys=True, separators=(",", ":")),
    ))


def _extract_json_object(raw_response: str) -> dict[str, Any]:
    """Extract one balanced JSON object, permitting a model's surrounding prose."""
    start = raw_response.find("{")
    if start < 0:
        raise ValueError("missing_json_object")
    depth = 0
    in_string = False
    escaped = False
    for index in range(start, len(raw_response)):
        char = raw_response[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                value = json.loads(raw_response[start:index + 1])
                if not isinstance(value, dict):
                    raise ValueError("candidate_json_is_not_object")
                return value
    raise ValueError("unclosed_json_object")


def _fresh_entities(context: CosmosReasoningInput) -> frozenset[str]:
    return frozenset(
        item.key.subject for item in context.snapshot.evidence
        if item.is_fresh(context.now_monotonic_s) and item.truth is Truth.TRUE
    )


def _rejected(context: CosmosReasoningInput, raw_response: str,
              status: CosmosCandidateStatus, *reasons: str) -> CosmosReasoningCandidate:
    return CosmosReasoningCandidate(status=status, task_id=context.task.task_id,
                                    raw_response=raw_response, reasons=tuple(reasons))


def parse_cosmos_candidate(raw_response: str, context: CosmosReasoningInput, *,
                           semantics_revision: str = "rrm/verbs/v1",
                           effect_window_revision: str = "rrm/learned-effect/v1") -> CosmosReasoningCandidate:
    """Parse model text into C04/C05 or a replayable refusal.

    A valid JSON shape alone is insufficient.  Grounded entities must be fresh in C02,
    verbs must be declared in C03, and Pydantic's C04/C05 contracts validate the plan.
    This function stops before safety admission and embodiment execution.
    """
    try:
        value = _extract_json_object(raw_response)
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                         f"malformed_model_json:{error}")
    try:
        status = IntentStatus(str(value.get("status", "")))
    except ValueError:
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED, "invalid_intent_status")

    actions_value = value.get("actions", [])
    if not isinstance(actions_value, list):
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED, "actions_not_list")
    ambiguity_value = value.get("ambiguity_refs", [])
    if not isinstance(ambiguity_value, list) or not all(isinstance(item, str) and item.strip()
                                                        for item in ambiguity_value):
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED, "invalid_ambiguity_refs")

    if status is IntentStatus.NEEDS_CLARIFICATION:
        if actions_value:
            return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                             "clarification_candidate_includes_actions")
        return _rejected(context, raw_response, CosmosCandidateStatus.NEEDS_CLARIFICATION,
                         *tuple(ambiguity_value) or ("model_omitted_ambiguity",))
    if status is IntentStatus.UNSUPPORTED:
        if actions_value or value.get("grounded_goal") is not None:
            return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                             "unsupported_candidate_includes_goal_or_actions")
        return _rejected(context, raw_response, CosmosCandidateStatus.UNSUPPORTED,
                         "model_reported_unsupported")
    if ambiguity_value:
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                         "ready_candidate_has_ambiguity")
    if not actions_value:
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                         "ready_candidate_missing_actions")

    entities_value = value.get("grounded_entities", [])
    if not isinstance(entities_value, list) or not entities_value or not all(
            isinstance(item, str) and item.strip() for item in entities_value):
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                         "invalid_grounded_entities")
    fresh_entities = _fresh_entities(context)
    invented = sorted(set(entities_value) - fresh_entities)
    if invented:
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                         *(f"ungrounded_entity:{item}" for item in invented))
    goal_value = value.get("grounded_goal")
    if not isinstance(goal_value, dict):
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED, "missing_grounded_goal")
    goal_object = goal_value.get("obj")
    if isinstance(goal_object, str) and goal_object not in fresh_entities and goal_object != "$self":
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                         f"ungrounded_goal_object:{goal_object}")

    try:
        goal = Predicate(name=goal_value["name"], subject=goal_value["subject"], obj=goal_object)
        intent = ReasoningResult(
            task_id=context.task.task_id,
            task_revision=context.task.revision,
            state_revision=context.snapshot.revision,
            capability_revision=context.capabilities.revision,
            status=IntentStatus.READY,
            grounded_goal=goal,
            grounded_entities=tuple(entities_value),
            explanation=str(value.get("explanation", "")),
        )
        actions: list[PlannedAction] = []
        for raw_action in actions_value:
            if not isinstance(raw_action, dict):
                raise ValueError("action must be an object")
            verb = Verb(raw_action["verb"])
            if verb.value not in context.capabilities.operations:
                raise ValueError(f"unsupported_verb:{verb.value}")
            targets = raw_action.get("targets", [])
            if not isinstance(targets, list) or any(target not in fresh_entities for target in targets):
                raise ValueError("action targets must be fresh grounded entities")
            action = AbstractAction(
                id=raw_action["id"], verb=verb, targets=targets,
                rationale=str(raw_action.get("rationale", "")),
            )
            actions.append(PlannedAction(
                action=action,
                dependencies=tuple(raw_action.get("dependencies", [])),
                semantics_revision=semantics_revision,
                feasibility_ref=f"{context.capabilities.revision}/{verb.value}/{action.id}",
                expected_effect_window_revision=effect_window_revision,
            ))
        plan = PlanProposal(
            plan_id=f"{context.task.task_id}/cosmos-reason2",
            version=0,
            task_id=context.task.task_id,
            task_revision=context.task.revision,
            intent=intent,
            state_revision=context.snapshot.revision,
            capability_revision=context.capabilities.revision,
            actions=tuple(actions),
            recovery_budget=value.get("recovery_budget", 0),
        )
    except (KeyError, TypeError, ValueError) as error:
        return _rejected(context, raw_response, CosmosCandidateStatus.REJECTED,
                         f"invalid_ready_candidate:{error}")
    return CosmosReasoningCandidate(status=CosmosCandidateStatus.ACCEPTED,
                                    task_id=context.task.task_id, raw_response=raw_response,
                                    intent=intent, plan=plan)
