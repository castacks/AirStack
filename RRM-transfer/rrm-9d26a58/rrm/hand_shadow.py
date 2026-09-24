"""Proposal-only C01–C05 baseline for a context-bound manipulation task.

This module reasons over entity IDs and explicit facts. It has no simulator, ROS,
controller, geometry, safety-admission, or dispatch dependency. Its semantic
feasibility references are not physical feasibility results.
"""

from __future__ import annotations

from enum import Enum
import math
import re
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .contracts import CapabilityDeclaration, Truth
from .schema import AbstractAction, Predicate, SELF, Verb
from .state_contracts import FactKey, StateSnapshot
from .task_contracts import IntentStatus, PlanProposal, PlannedAction, ReasoningResult, TaskRequest


_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_OBJECTIVE = "place the context-selected block on the tray"
_RESOURCES = frozenset({"arm", "hand"})


class SceneEntity(BaseModel):
    model_config = ConfigDict(frozen=True)

    entity_id: str
    kind: str

    @model_validator(mode="after")
    def validate_entity(self) -> "SceneEntity":
        if not self.entity_id.strip() or not self.kind.strip():
            raise ValueError("scene entity ID and kind are required")
        return self


class ContextBinding(BaseModel):
    model_config = ConfigDict(frozen=True)

    ref: str
    entity_id: str

    @model_validator(mode="after")
    def validate_binding(self) -> "ContextBinding":
        if not self.ref.strip() or not self.entity_id.strip():
            raise ValueError("context reference and entity ID are required")
        return self


class HandSceneManifest(BaseModel):
    """Frozen semantic binding for a proposed, separate Kuka-Allegro scene."""

    model_config = ConfigDict(frozen=True)

    schema_version: Literal["rrm-hand-scene/v1"] = "rrm-hand-scene/v1"
    scene_id: str
    revision: str
    episode_id: str
    embodiment_id: str
    asset_url: str
    asset_sha256: str
    support_entity_id: str
    limits_ref: str
    entities: tuple[SceneEntity, ...]
    context_bindings: tuple[ContextBinding, ...]

    @model_validator(mode="after")
    def validate_manifest(self) -> "HandSceneManifest":
        for name in ("scene_id", "revision", "episode_id", "embodiment_id",
                     "asset_url", "support_entity_id", "limits_ref"):
            if not getattr(self, name).strip():
                raise ValueError(f"{name} is required")
        if not _SHA256.fullmatch(self.asset_sha256):
            raise ValueError("asset_sha256 must be lowercase SHA-256")
        catalog = {item.entity_id: item.kind for item in self.entities}
        if len(catalog) != len(self.entities) or not catalog:
            raise ValueError("scene entity IDs must be unique")
        if catalog.get(self.support_entity_id) != "tray":
            raise ValueError("support entity must be a declared tray")
        refs = [item.ref for item in self.context_bindings]
        if len(refs) != len(set(refs)) or not refs:
            raise ValueError("context references must be unique")
        if any(catalog.get(item.entity_id) != "block" for item in self.context_bindings):
            raise ValueError("context selection must identify a declared block")
        return self

    def catalog(self) -> dict[str, str]:
        return {item.entity_id: item.kind for item in self.entities}

    def binding(self, ref: str) -> str | None:
        return next((item.entity_id for item in self.context_bindings
                     if item.ref == ref), None)


class HandShadowStatus(str, Enum):
    PROPOSED = "PROPOSED"
    HOLD = "HOLD"
    NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"
    UNSUPPORTED = "UNSUPPORTED"


class HandShadowDecision(BaseModel):
    model_config = ConfigDict(frozen=True)

    schema_version: Literal["rrm-hand-shadow-decision/v1"] = "rrm-hand-shadow-decision/v1"
    task_id: str
    scene_revision: str
    state_revision: str
    capability_revision: str
    status: HandShadowStatus
    reasons: tuple[str, ...] = ()
    selected_entity_id: str | None = None
    context_ref: str | None = None
    intent: ReasoningResult | None = None
    plan: PlanProposal | None = None
    execution_dispatch: Literal[False] = False

    @model_validator(mode="after")
    def validate_decision(self) -> "HandShadowDecision":
        proposed = self.status is HandShadowStatus.PROPOSED
        if proposed != (self.intent is not None and self.plan is not None):
            raise ValueError("only a proposed decision carries complete C04/C05 records")
        if proposed and (self.reasons or not self.selected_entity_id or not self.context_ref):
            raise ValueError("proposed decision requires a clean context binding")
        if not proposed and (self.intent is not None or self.plan is not None or not self.reasons):
            raise ValueError("non-proposed decision requires reasons and no plan")
        return self


class HandShadowBridge:
    """Deterministic baseline for one contextual two-action manipulation request."""

    def __init__(self, manifest: HandSceneManifest) -> None:
        self.manifest = manifest

    def decide(self, task: TaskRequest, snapshot: StateSnapshot,
               capabilities: CapabilityDeclaration, *,
               now_monotonic_s: float) -> HandShadowDecision:
        base = dict(
            task_id=task.task_id, scene_revision=self.manifest.revision,
            state_revision=snapshot.revision, capability_revision=capabilities.revision,
        )

        def refuse(status: HandShadowStatus, *reasons: str) -> HandShadowDecision:
            return HandShadowDecision(status=status, reasons=tuple(reasons), **base)

        if not math.isfinite(now_monotonic_s) or now_monotonic_s < 0:
            return refuse(HandShadowStatus.HOLD, "invalid_observation_clock")
        if (task.requested_embodiment_id != self.manifest.embodiment_id
                or capabilities.embodiment_id != self.manifest.embodiment_id):
            return refuse(HandShadowStatus.UNSUPPORTED, "wrong_embodiment")
        if (snapshot.task_id != task.task_id
                or snapshot.episode_id != self.manifest.episode_id):
            return refuse(HandShadowStatus.HOLD, "state_task_or_episode_mismatch")
        if " ".join(task.objective.lower().split()) != _OBJECTIVE:
            return refuse(HandShadowStatus.UNSUPPORTED, "unsupported_objective")
        if len(task.context_refs) != 1:
            return refuse(HandShadowStatus.NEEDS_CLARIFICATION, "one_selection_context_required")
        context_ref = task.context_refs[0]
        block_id = self.manifest.binding(context_ref)
        if block_id is None:
            return refuse(HandShadowStatus.NEEDS_CLARIFICATION, "unresolved_context_selection")
        if capabilities.limits_ref != self.manifest.limits_ref:
            return refuse(HandShadowStatus.UNSUPPORTED, "limits_profile_mismatch")
        for operation in (Verb.GRASP, Verb.PLACE):
            reasons = capabilities.rejection_reasons(operation.value, _RESOURCES)
            if reasons:
                return refuse(HandShadowStatus.UNSUPPORTED,
                              *(f"{operation.value}:{reason}" for reason in reasons))

        tray_id = self.manifest.support_entity_id
        required = (
            FactKey(subject=block_id, predicate="exists"),
            FactKey(subject=block_id, predicate="kind", obj="block"),
            FactKey(subject=block_id, predicate="localized"),
            FactKey(subject=block_id, predicate="graspable"),
            FactKey(subject=block_id, predicate="reachable"),
            FactKey(subject=tray_id, predicate="exists"),
            FactKey(subject=tray_id, predicate="kind", obj="tray"),
            FactKey(subject=tray_id, predicate="localized"),
            FactKey(subject=tray_id, predicate="reachable"),
            FactKey(subject=SELF, predicate="gripper_empty"),
        )
        for key in required:
            truth = snapshot.resolve(key, now_monotonic_s=now_monotonic_s)
            if truth is not Truth.TRUE:
                reason = "false" if truth is Truth.FALSE else "unknown"
                return refuse(HandShadowStatus.HOLD,
                              f"required_fact_{reason}:{key.subject}/{key.predicate}/{key.obj}")
        if snapshot.resolve(FactKey(subject=block_id, predicate="on", obj=tray_id),
                            now_monotonic_s=now_monotonic_s) is Truth.TRUE:
            return refuse(HandShadowStatus.HOLD, "goal_already_satisfied")

        intent = ReasoningResult(
            task_id=task.task_id, task_revision=task.revision,
            state_revision=snapshot.revision, capability_revision=capabilities.revision,
            status=IntentStatus.READY,
            grounded_goal=Predicate(name="on", subject=block_id, obj=tray_id),
            grounded_entities=(block_id, tray_id),
            explanation="deterministic context-bound hand shadow baseline",
        )
        grasp_id = f"grasp-{block_id}"
        place_id = f"place-{block_id}-on-{tray_id}"
        plan = PlanProposal(
            plan_id=f"{task.task_id}/hand-shadow", version=0,
            task_id=task.task_id, task_revision=task.revision, intent=intent,
            state_revision=snapshot.revision, capability_revision=capabilities.revision,
            actions=(
                PlannedAction(
                    action=AbstractAction(id=grasp_id, verb=Verb.GRASP, targets=[block_id]),
                    semantics_revision="rrm/verbs/v1",
                    feasibility_ref=f"{snapshot.revision}/shadow-semantic-only/{grasp_id}",
                    expected_effect_window_revision="hand-shadow-effects/v1",
                ),
                PlannedAction(
                    action=AbstractAction(id=place_id, verb=Verb.PLACE,
                                          targets=[block_id, tray_id]),
                    dependencies=(grasp_id,), semantics_revision="rrm/verbs/v1",
                    feasibility_ref=f"{snapshot.revision}/shadow-semantic-only/{place_id}",
                    expected_effect_window_revision="hand-shadow-effects/v1",
                ),
            ),
            recovery_budget=0,
        )
        return HandShadowDecision(
            status=HandShadowStatus.PROPOSED, selected_entity_id=block_id,
            context_ref=context_ref, intent=intent, plan=plan, **base,
        )
