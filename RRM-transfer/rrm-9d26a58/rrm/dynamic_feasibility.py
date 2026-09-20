"""Body-agnostic dynamic feasibility and single-use admission contracts.

The RRM core never computes embodiment geometry.  An injected adapter evaluates one
grounded command against current observations, limits, resources, controller state,
and physics.  This module validates and consumes that evidence; it has no ROS,
simulator, subprocess, or actuator dependency.
"""
from __future__ import annotations

from enum import Enum
import hashlib
import json
import math
import threading
from typing import Protocol
import uuid

from pydantic import BaseModel, ConfigDict, Field, model_validator

from rrm.airstack_drone import DroneTaskProposal
from rrm.cosmos_reason2 import CosmosReasoningInput


REQUIRED_CHECKS = frozenset({
    "grounding", "body_limits", "physics", "controller", "resources", "stop_channel",
})


class FeasibilityVerdict(str, Enum):
    FEASIBLE = "FEASIBLE"
    INFEASIBLE = "INFEASIBLE"
    UNCERTAIN = "UNCERTAIN"


class EvidenceAuthority(str, Enum):
    AUTHORITATIVE = "AUTHORITATIVE"
    ADVISORY = "ADVISORY"


class FeasibilityCheck(BaseModel):
    """One adapter-owned check and its immutable evidence reference."""

    model_config = ConfigDict(frozen=True)

    name: str
    passed: bool | None
    authority: EvidenceAuthority
    evidence_ref: str
    source_revision: str
    detail: str

    @model_validator(mode="after")
    def validate_check(self) -> "FeasibilityCheck":
        for field in ("name", "evidence_ref", "source_revision", "detail"):
            if not getattr(self, field).strip():
                raise ValueError(f"feasibility {field} is required")
        return self


class DynamicFeasibilityResult(BaseModel):
    """C03 result bound to one grounded command and one current context."""

    model_config = ConfigDict(frozen=True)

    result_id: str = Field(default_factory=lambda: uuid.uuid4().hex)
    task_id: str
    action_id: str
    embodiment_id: str
    proposal_sha256: str
    observation_sha256: str
    state_revision: str
    capability_revision: str
    scene_revision: str
    profile_revision: str
    stop_generation: int = Field(ge=0)
    checked_monotonic_s: float = Field(ge=0)
    expires_monotonic_s: float = Field(gt=0)
    verdict: FeasibilityVerdict
    checks: tuple[FeasibilityCheck, ...]
    evidence_payload_json: str | None = None
    evidence_sha256: str | None = None

    @model_validator(mode="after")
    def validate_result(self) -> "DynamicFeasibilityResult":
        for field in (
            "result_id", "task_id", "action_id", "embodiment_id", "state_revision",
            "capability_revision", "scene_revision", "profile_revision",
        ):
            if not getattr(self, field).strip():
                raise ValueError(f"feasibility {field} is required")
        for field in ("proposal_sha256", "observation_sha256"):
            value = getattr(self, field)
            if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
                raise ValueError(f"{field} must be a lowercase SHA-256")
        if not math.isfinite(self.checked_monotonic_s) or not math.isfinite(self.expires_monotonic_s):
            raise ValueError("feasibility clocks must be finite")
        if self.expires_monotonic_s <= self.checked_monotonic_s:
            raise ValueError("feasibility result must expire after it is checked")
        if (self.evidence_payload_json is None) != (self.evidence_sha256 is None):
            raise ValueError("inline feasibility evidence requires payload and checksum")
        if self.evidence_payload_json is not None:
            try:
                decoded = json.loads(self.evidence_payload_json)
            except json.JSONDecodeError as error:
                raise ValueError("inline feasibility evidence must be JSON") from error
            canonical = json.dumps(decoded, sort_keys=True, separators=(",", ":"))
            if canonical != self.evidence_payload_json:
                raise ValueError("inline feasibility evidence must use canonical JSON")
            if hashlib.sha256(canonical.encode()).hexdigest() != self.evidence_sha256:
                raise ValueError("inline feasibility evidence checksum mismatch")
        names = [check.name for check in self.checks]
        if len(names) != len(set(names)):
            raise ValueError("feasibility check names must be unique")
        if self.verdict is FeasibilityVerdict.FEASIBLE:
            authoritative = {
                check.name for check in self.checks
                if check.authority is EvidenceAuthority.AUTHORITATIVE and check.passed is True
            }
            if not REQUIRED_CHECKS <= authoritative:
                raise ValueError("feasible result is missing a required embodiment check")
            if any(check.authority is EvidenceAuthority.AUTHORITATIVE and check.passed is not True
                   for check in self.checks):
                raise ValueError("feasible result requires every authoritative check to pass")
        elif self.verdict is FeasibilityVerdict.INFEASIBLE:
            if not any(check.authority is EvidenceAuthority.AUTHORITATIVE and check.passed is False
                       for check in self.checks):
                raise ValueError("infeasible result requires an explicit failed check")
        elif not any(check.authority is EvidenceAuthority.AUTHORITATIVE and check.passed is None
                     for check in self.checks):
            raise ValueError("uncertain result requires an unknown check")
        return self


class DynamicFeasibilityEvaluator(Protocol):
    """Embodiment-owned implementation; learned evidence may advise but not admit."""

    def evaluate(
        self,
        semantic_action: dict,
        proposal: DroneTaskProposal,
        context: CosmosReasoningInput,
        observation: dict,
        scene_state: dict,
        *,
        stop_generation: int,
    ) -> DynamicFeasibilityResult: ...


def proposal_sha256(proposal: DroneTaskProposal) -> str:
    payload = json.dumps(
        proposal.model_dump(mode="json"), sort_keys=True, separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


class FailClosedFeasibilityEvaluator:
    """Default when no embodiment physics provider has been configured."""

    def evaluate(self, semantic_action, proposal, context, observation, scene_state, *, stop_generation):
        checked = context.now_monotonic_s
        return DynamicFeasibilityResult(
            task_id=proposal.task_id,
            action_id=proposal.action_id,
            embodiment_id=context.capabilities.embodiment_id,
            proposal_sha256=proposal_sha256(proposal),
            observation_sha256=observation["sha256"],
            state_revision=context.snapshot.revision,
            capability_revision=context.capabilities.revision,
            scene_revision=str(scene_state.get("provenance", "unknown")),
            profile_revision=str(context.capabilities.limits_ref),
            stop_generation=stop_generation,
            checked_monotonic_s=checked,
            expires_monotonic_s=checked + 0.001,
            verdict=FeasibilityVerdict.UNCERTAIN,
            checks=(FeasibilityCheck(
                name="physics", passed=None,
                authority=EvidenceAuthority.AUTHORITATIVE,
                evidence_ref="missing:embodiment-physics-provider",
                source_revision="unconfigured",
                detail="No embodiment physics/feasibility evaluator is configured.",
            ),),
        )


class SingleUseAdmission:
    """Validate exact current dependencies and consume one feasibility result once."""

    def __init__(self):
        self._lock = threading.Lock()
        self._consumed: set[str] = set()

    def consume(
        self,
        result: DynamicFeasibilityResult,
        proposal: DroneTaskProposal,
        context: CosmosReasoningInput,
        observation: dict,
        scene_state: dict,
        *,
        stop_generation: int,
        now_monotonic_s: float,
    ) -> dict:
        if not isinstance(result, DynamicFeasibilityResult):
            raise ValueError("Embodiment feasibility evaluator returned an invalid result.")
        expected = {
            "task_id": proposal.task_id,
            "action_id": proposal.action_id,
            "embodiment_id": context.capabilities.embodiment_id,
            "proposal_sha256": proposal_sha256(proposal),
            "observation_sha256": observation.get("sha256"),
            "state_revision": context.snapshot.revision,
            "capability_revision": context.capabilities.revision,
            "scene_revision": str(scene_state.get("provenance", "unknown")),
            "profile_revision": str(context.capabilities.limits_ref),
            "stop_generation": stop_generation,
        }
        mismatches = [name for name, value in expected.items() if getattr(result, name) != value]
        if mismatches:
            raise ValueError("Feasibility result is stale or detached: " + ",".join(mismatches))
        if result.verdict is not FeasibilityVerdict.FEASIBLE:
            raise ValueError("Current embodiment feasibility did not explicitly pass.")
        if (not math.isfinite(now_monotonic_s)
                or now_monotonic_s < result.checked_monotonic_s
                or now_monotonic_s >= result.expires_monotonic_s):
            raise ValueError("Feasibility result is expired or uses an invalid clock.")
        with self._lock:
            if result.result_id in self._consumed:
                raise ValueError("Feasibility admission was already consumed.")
            self._consumed.add(result.result_id)
        return {
            "schema_version": "rrm-single-use-admission/v1",
            "decision": "ALLOW",
            "result_id": result.result_id,
            "proposal_sha256": result.proposal_sha256,
            "observation_sha256": result.observation_sha256,
            "state_revision": result.state_revision,
            "capability_revision": result.capability_revision,
            "scene_revision": result.scene_revision,
            "profile_revision": result.profile_revision,
            "stop_generation": stop_generation,
            "single_use_consumed": True,
            "feasibility_evidence_sha256": result.evidence_sha256,
            "execution_dispatch": False,
        }
