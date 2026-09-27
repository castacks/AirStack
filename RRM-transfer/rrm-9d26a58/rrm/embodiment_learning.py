"""Advisory RRM-EM evidence model over independently checked task effects.

This module has no dispatch, admission, ROS, simulator, or controller dependency.
Estimates are scoped to an exact body/scene/binding revision and cannot turn a
capability declaration or a task-server success flag into execution authority.
"""

from __future__ import annotations

from enum import Enum
import hashlib
import json

from pydantic import BaseModel, ConfigDict, model_validator

from .goal_contracts import EmbodimentRoute, RouteStatus


class EffectVerdict(str, Enum):
    VERIFIED = "VERIFIED"
    UNMET = "UNMET"
    UNKNOWN = "UNKNOWN"


class EvidenceScope(BaseModel):
    """The exact context within which an observed result may be compared."""

    model_config = ConfigDict(frozen=True)

    embodiment_id: str
    operation: str
    scene_revision: str
    capability_revision: str
    adapter_revision: str
    controller_revision: str

    @model_validator(mode="after")
    def validate_scope(self) -> "EvidenceScope":
        if any(not value.strip() for value in self.model_dump().values()):
            raise ValueError("RRM-EM evidence scope requires every revision and ID")
        return self


class EffectEvidence(BaseModel):
    """One physical attempt with a separate, independently observed effect."""

    model_config = ConfigDict(frozen=True)

    attempt_id: str
    scope: EvidenceScope
    action_success: bool | None
    effect_verdict: EffectVerdict
    verification_ref: str
    verifier_revision: str
    reasons: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_evidence(self) -> "EffectEvidence":
        if not self.attempt_id.strip() or not self.verification_ref.strip():
            raise ValueError("RRM-EM evidence requires attempt and verification refs")
        if not self.verifier_revision.strip():
            raise ValueError("RRM-EM evidence requires a verifier revision")
        if any(not reason.strip() for reason in self.reasons):
            raise ValueError("RRM-EM reason codes must be nonempty")
        if self.effect_verdict is EffectVerdict.VERIFIED and self.action_success is not True:
            raise ValueError("verified effect requires a successful task result")
        return self


class CapabilityEstimate(BaseModel):
    """An advisory empirical estimate; it is neither feasibility nor admission."""

    model_config = ConfigDict(frozen=True)

    scope: EvidenceScope
    evidence_revision: str
    verified_count: int
    unmet_count: int
    unknown_count: int
    posterior_mean: float | None


class EmbodimentEvidenceLedger:
    """Append-only, idempotent in-memory ledger with exact-context queries."""

    def __init__(self) -> None:
        self._attempts: dict[str, EffectEvidence] = {}

    def append(self, evidence: EffectEvidence) -> None:
        previous = self._attempts.get(evidence.attempt_id)
        if previous is not None and previous != evidence:
            raise ValueError("attempt ID was reused with different RRM-EM evidence")
        self._attempts[evidence.attempt_id] = evidence

    def estimate(self, scope: EvidenceScope) -> CapabilityEstimate:
        records = sorted(
            (item for item in self._attempts.values() if item.scope == scope),
            key=lambda item: item.attempt_id,
        )
        verified = sum(item.effect_verdict is EffectVerdict.VERIFIED for item in records)
        unmet = sum(item.effect_verdict is EffectVerdict.UNMET for item in records)
        unknown = sum(item.effect_verdict is EffectVerdict.UNKNOWN for item in records)
        canonical = json.dumps(
            [item.model_dump(mode="json") for item in records],
            sort_keys=True, separators=(",", ":"),
        ).encode("utf-8")
        return CapabilityEstimate(
            scope=scope,
            evidence_revision=hashlib.sha256(canonical).hexdigest(),
            verified_count=verified,
            unmet_count=unmet,
            unknown_count=unknown,
            # A Beta(1,1) prior prevents a single success from claiming certainty.
            posterior_mean=(verified + 1) / (verified + unmet + 2)
            if verified + unmet else None,
        )


def evidence_from_drone_outcome(
    *, attempt_id: str, scope: EvidenceScope, outcome: dict,
    verification_ref: str, verifier_revision: str,
) -> EffectEvidence:
    """Normalize the existing AirStack verifier without trusting task success alone."""

    verdict = outcome.get("verdict")
    effect = {
        "VERIFIED": EffectVerdict.VERIFIED,
        "MISMATCH": EffectVerdict.UNMET,
        "UNCONFIRMED": EffectVerdict.UNKNOWN,
    }.get(verdict)
    if effect is None:
        raise ValueError("unsupported drone outcome verdict")
    if outcome.get("action_success") not in (True, False):
        raise ValueError("drone outcome needs an explicit task-result flag")
    if outcome.get("kind") != scope.operation:
        raise ValueError("drone outcome operation differs from evidence scope")
    reasons = outcome.get("reasons")
    if not isinstance(reasons, list) or any(not isinstance(item, str) for item in reasons):
        raise ValueError("drone outcome needs verifier reason codes")
    return EffectEvidence(
        attempt_id=attempt_id, scope=scope,
        action_success=outcome["action_success"], effect_verdict=effect,
        verification_ref=verification_ref, verifier_revision=verifier_revision,
        reasons=tuple(reasons),
    )


def assess_route(
    route: EmbodimentRoute, scopes: tuple[EvidenceScope, ...],
    ledger: EmbodimentEvidenceLedger,
) -> tuple[CapabilityEstimate, ...]:
    """Attach experience to C03 route candidates without selecting or authorizing one."""

    if route.status not in (RouteStatus.SELECTED, RouteStatus.CANDIDATES):
        return ()
    by_body = {scope.embodiment_id: scope for scope in scopes}
    if len(by_body) != len(scopes):
        raise ValueError("RRM-EM candidate scopes must have unique embodiment IDs")
    if set(by_body) != set(route.candidate_embodiment_ids):
        raise ValueError("RRM-EM candidate scopes must exactly match the route")
    return tuple(ledger.estimate(by_body[body]) for body in route.candidate_embodiment_ids)


def compile_command_mission_evidence(
    *, plan_bytes: bytes, mission: dict, embodiment_id: str, active_scene: str,
    scene_revision: str, capability_revision: str, adapter_revision: str,
    controller_revision: str,
) -> tuple[EffectEvidence, ...]:
    """Import one immutable command mission for offline, advisory RRM-EM study.

    The caller supplies exact installed binding revisions. A skipped action is
    omitted because no physical attempt occurred; failed or uncertain dispatched
    attempts remain UNKNOWN unless independent verification reports a mismatch.
    """

    plan_hash = hashlib.sha256(plan_bytes).hexdigest()
    if mission.get("schema_version") != "rrm-airstack-command-outcome/v1":
        raise ValueError("unsupported command mission outcome schema")
    if mission.get("plan_sha256") != plan_hash:
        raise ValueError("mission outcome is not bound to the supplied plan bytes")
    plan = json.loads(plan_bytes)
    if plan.get("schema_version") != "rrm-airstack-command-plan/v1":
        raise ValueError("unsupported command plan schema")
    if not active_scene.strip() or plan.get("active_scene") != active_scene:
        raise ValueError("declared active scene differs from the bound command plan")
    declared = {item["action_id"]: item for item in plan.get("actions", [])}
    if plan.get("recovery"):
        recovery = plan["recovery"]["action"]
        declared[recovery["action_id"]] = recovery
    if not declared:
        raise ValueError("command plan has no actions")

    ledger = EmbodimentEvidenceLedger()
    records: list[EffectEvidence] = []
    attempts = list(mission.get("results", []))
    if mission.get("recovery") and mission["recovery"].get("execution_dispatch") is not False:
        attempts.append(mission["recovery"])
    for item in attempts:
        if item.get("dispatch_skipped") is True:
            continue
        action_id = item.get("action_id")
        expected = declared.get(action_id)
        if expected is None:
            raise ValueError("mission result names an undeclared action")
        outcome = item.get("outcome")
        if not isinstance(outcome, dict):
            raise ValueError("dispatched action lacks outcome evidence")
        if outcome.get("task_id") != expected.get("task_id") or outcome.get("action_id") != action_id:
            raise ValueError("outcome identity differs from the bound plan")
        scope = EvidenceScope(
            embodiment_id=embodiment_id, operation=expected["kind"],
            scene_revision=scene_revision, capability_revision=capability_revision,
            adapter_revision=adapter_revision, controller_revision=controller_revision,
        )
        digest = hashlib.sha256(json.dumps(
            outcome, sort_keys=True, separators=(",", ":"),
        ).encode("utf-8")).hexdigest()
        record = evidence_from_drone_outcome(
            attempt_id=f"{plan_hash}:{action_id}", scope=scope, outcome=outcome,
            verification_ref=f"sha256:{digest}",
            verifier_revision="airstack-drone-outcome/v1",
        )
        ledger.append(record)
        records.append(record)
    return tuple(sorted(records, key=lambda record: record.attempt_id))
