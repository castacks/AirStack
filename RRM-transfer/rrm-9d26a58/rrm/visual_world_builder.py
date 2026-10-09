"""Catalog-bound VLM observations to C02 evidence, plus teacher scoring.

This is a semantic evidence boundary.  It deliberately imports no model, Isaac, ROS,
geometry, physics or execution code.  A runtime supplies raw VLM text and a trusted
media manifest; this module validates claims before they can enter RRM world state.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
import re
from typing import Any, Mapping

from pydantic import BaseModel, ConfigDict, Field

from .contracts import Truth
from .state_contracts import FactEvidence, FactKey, FactProvenance, StateSnapshot


_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_VISUAL_PREDICATES = frozenset({"exists", "kind", "localized"})
VISUAL_PROMPT_REVISION = "visual-claims/v3"


def visual_output_schema() -> dict[str, Any]:
    """JSON Schema for the prompt, not constrained decoding or a truth oracle."""
    return {
        "type": "object", "required": ["status", "claims"], "additionalProperties": False,
        "properties": {
            "status": {"enum": ["READY", "NEEDS_CLARIFICATION"]},
            "claims": {"type": "array", "items": {
                "type": "object", "required": ["subject", "predicate", "obj", "truth"],
                "additionalProperties": False,
                "properties": {
                    "subject": {"type": "string"},
                    "predicate": {"enum": ["exists", "kind", "localized"]},
                    "obj": {"type": ["string", "null"]},
                    "truth": {"enum": ["TRUE", "FALSE", "UNKNOWN"]},
                },
            }},
        },
    }


class VisualCandidateStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    NEEDS_CLARIFICATION = "NEEDS_CLARIFICATION"


@dataclass(frozen=True)
class MediaArtifact:
    """Durable identity for a frozen image/video, never a coordinate claim."""

    source_ref: str
    sha256: str
    observed_monotonic_s: float

    def __post_init__(self) -> None:
        if not self.source_ref.strip():
            raise ValueError("media source reference is required")
        if not _SHA256_RE.fullmatch(self.sha256):
            raise ValueError("media sha256 must be lowercase hexadecimal")
        if self.observed_monotonic_s < 0:
            raise ValueError("media observation time must be nonnegative")


@dataclass(frozen=True)
class VisualGroundingInput:
    """The trusted context for one VLM observation; catalog IDs are evaluation-scoped."""

    task_id: str
    episode_id: str
    state_revision: str
    entity_catalog: Mapping[str, str]
    media: MediaArtifact
    received_monotonic_s: float
    max_age_s: float
    model_ref: str

    def __post_init__(self) -> None:
        if not all(value.strip() for value in (
                self.task_id, self.episode_id, self.state_revision, self.model_ref)):
            raise ValueError("task, episode, state revision and model reference are required")
        if not self.entity_catalog or any(not key.strip() or not value.strip()
                                          for key, value in self.entity_catalog.items()):
            raise ValueError("entity catalog must contain nonempty IDs and kinds")
        if self.received_monotonic_s < self.media.observed_monotonic_s:
            raise ValueError("visual evidence cannot arrive before its media observation")
        if self.max_age_s <= 0:
            raise ValueError("visual evidence max age must be positive")


class VisualGroundingCandidate(BaseModel):
    """Replayable model candidate; only accepted candidates carry C02 evidence."""

    model_config = ConfigDict(frozen=True)

    status: VisualCandidateStatus
    task_id: str
    raw_response: str
    reasons: tuple[str, ...] = ()
    snapshot: StateSnapshot | None = None

    def model_post_init(self, __context: Any) -> None:
        if not self.task_id.strip() or not self.raw_response.strip():
            raise ValueError("candidate task and raw response are required")
        if (self.status is VisualCandidateStatus.ACCEPTED) != (self.snapshot is not None):
            raise ValueError("only accepted visual candidates carry a C02 snapshot")


class TeacherScore(BaseModel):
    """Fact-level score; no unobserved VLM fact is silently treated as FALSE."""

    model_config = ConfigDict(frozen=True)

    task_id: str
    teacher_revision: str
    candidate_revision: str
    exact_matches: int = Field(ge=0)
    mismatches: int = Field(ge=0)
    missed_teacher_facts: int = Field(ge=0)
    extra_candidate_facts: int = Field(ge=0)
    precision_denominator: int = Field(ge=0)
    recall_denominator: int = Field(ge=0)

    @property
    def precision(self) -> float | None:
        return None if not self.precision_denominator else self.exact_matches / self.precision_denominator

    @property
    def recall(self) -> float | None:
        return None if not self.recall_denominator else self.exact_matches / self.recall_denominator


def render_visual_prompt(context: VisualGroundingInput) -> str:
    """Prompt a VLM for semantic candidate facts, never world truth or control."""
    catalog = [{"id": entity_id, "kind": kind}
               for entity_id, kind in sorted(context.entity_catalog.items())]
    return "\n".join((
        "Inspect the supplied image or video and report catalog-bound visual evidence, not actions.",
        f"Output contract revision: {VISUAL_PROMPT_REVISION}",
        "For EACH confidently recognized entity, return TWO TRUE claims: exists with obj null, and kind with obj equal to its exact catalog kind.",
        "Use only actual catalog IDs/kinds. Catalog membership is not image evidence.",
        "Not seeing an entity does not prove absence: use UNKNOWN or request clarification, never FALSE from absence from view.",
        'If no entity can be confidently recognized, return {"status":"NEEDS_CLARIFICATION","claims":[]}, not READY with empty claims.',
        "A 2-D image is not physical localization: do not claim localized TRUE without physical localization evidence.",
        "Do not invent coordinates, safety states, capabilities, actions, plans or control commands.",
        "FORMAT EXAMPLE ONLY (fictional catalog/image, not evidence for the supplied image):",
        'If a fictional catalog maps demo_ball to violet ball and that ball is clearly recognized, output:',
        '{"status":"READY","claims":[{"subject":"demo_ball","predicate":"exists","obj":null,"truth":"TRUE"},{"subject":"demo_ball","predicate":"kind","obj":"violet ball","truth":"TRUE"}]}',
        "Do not copy demo_ball or violet ball unless they occur in the actual catalog and are supported by the actual image.",
        "Return only one JSON object, no Markdown or commentary. Every claim must contain subject, predicate, obj and truth. For exists/localized obj is null. Schema:",
        json.dumps(visual_output_schema(), sort_keys=True, separators=(",", ":")),
        "ACTUAL ENTITY CATALOG:",
        json.dumps(catalog, sort_keys=True, separators=(",", ":")),
    ))


def complete_visual_identities(snapshot: StateSnapshot, *, entity_catalog: Mapping[str, str],
                               now_monotonic_s: float, require_localized: bool = False) -> tuple[str, ...]:
    """Qualify candidate fact completeness, not visual truth or motion authority.

    Catalog membership supplies allowed IDs/kinds, never missing evidence. Unknown,
    false, contradictory or expired facts cannot satisfy a positive qualification.
    A model's localized claim remains a claim, not independent physical verification.
    """
    if type(now_monotonic_s) not in (int, float) or not math.isfinite(now_monotonic_s) or now_monotonic_s < 0:
        raise ValueError("invalid visual qualification clock")
    if not entity_catalog or any(not isinstance(key, str) or not isinstance(kind, str)
                                 or not key.strip() or not kind.strip()
                                 for key, kind in entity_catalog.items()):
        raise ValueError("invalid visual qualification catalog")
    identified = []
    for entity, kind in sorted(entity_catalog.items()):
        keys = [FactKey(subject=entity, predicate="exists"), FactKey(subject=entity, predicate="kind", obj=kind)]
        if require_localized:
            keys.append(FactKey(subject=entity, predicate="localized"))
        if all(snapshot.resolve(key, now_monotonic_s=now_monotonic_s) is Truth.TRUE for key in keys):
            identified.append(entity)
    return tuple(identified)


def _extract_json_object(raw_response: str) -> dict[str, Any]:
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


def _rejected(context: VisualGroundingInput, raw: str, status: VisualCandidateStatus,
              *reasons: str) -> VisualGroundingCandidate:
    return VisualGroundingCandidate(status=status, task_id=context.task_id,
                                    raw_response=raw, reasons=tuple(reasons))


def parse_visual_candidate(raw_response: str, context: VisualGroundingInput) -> VisualGroundingCandidate:
    """Validate raw VLM output and produce C02 `INFERRED` evidence or a refusal."""
    try:
        value = _extract_json_object(raw_response)
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        return _rejected(context, raw_response, VisualCandidateStatus.REJECTED,
                         f"malformed_model_json:{error}")
    status_value = value.get("status")
    if not isinstance(value.get("claims"), list):
        return _rejected(context, raw_response, VisualCandidateStatus.REJECTED,
                         "visual_claims_must_be_explicit_array")
    if status_value == "NEEDS_CLARIFICATION":
        if value.get("claims", []):
            return _rejected(context, raw_response, VisualCandidateStatus.REJECTED,
                             "clarification_candidate_includes_claims")
        return _rejected(context, raw_response, VisualCandidateStatus.NEEDS_CLARIFICATION,
                         "model_needs_clarification")
    if status_value != "READY":
        return _rejected(context, raw_response, VisualCandidateStatus.REJECTED,
                         "invalid_visual_status")
    claims = value.get("claims")
    if not isinstance(claims, list) or not claims:
        return _rejected(context, raw_response, VisualCandidateStatus.REJECTED,
                         "ready_candidate_missing_claims")
    evidence: list[FactEvidence] = []
    seen: set[tuple[str, str, str | None]] = set()
    try:
        for claim in claims:
            if not isinstance(claim, dict):
                raise ValueError("claim_must_be_object")
            if not {"subject", "predicate", "obj", "truth"}.issubset(claim):
                raise ValueError("claim_missing_required_fields")
            subject = claim["subject"]
            predicate = claim["predicate"]
            obj = claim.get("obj")
            if not isinstance(subject, str) or subject not in context.entity_catalog:
                raise ValueError(f"unknown_catalog_entity:{subject}")
            if predicate not in _VISUAL_PREDICATES:
                raise ValueError(f"unsupported_visual_predicate:{predicate}")
            if predicate == "kind":
                if obj != context.entity_catalog[subject]:
                    raise ValueError(f"catalog_kind_mismatch:{subject}")
            elif obj is not None:
                raise ValueError(f"non_kind_claim_has_object:{predicate}")
            truth = Truth(claim["truth"])
            identity = (subject, predicate, obj)
            if identity in seen:
                raise ValueError("duplicate_visual_claim")
            seen.add(identity)
            evidence.append(FactEvidence(
                key=FactKey(subject=subject, predicate=predicate, obj=obj), truth=truth,
                provenance=FactProvenance.INFERRED,
                source_ref=f"{context.model_ref}/{context.media.source_ref}/sha256:{context.media.sha256}",
                observed_monotonic_s=context.media.observed_monotonic_s,
                received_monotonic_s=context.received_monotonic_s,
                max_age_s=context.max_age_s,
            ))
    except (KeyError, TypeError, ValueError) as error:
        return _rejected(context, raw_response, VisualCandidateStatus.REJECTED,
                         f"invalid_visual_claim:{error}")
    snapshot = StateSnapshot(
        snapshot_id=f"{context.episode_id}/vlm-{context.media.sha256[:12]}",
        revision=context.state_revision,
        task_id=context.task_id,
        episode_id=context.episode_id,
        evidence=tuple(evidence),
    )
    return VisualGroundingCandidate(status=VisualCandidateStatus.ACCEPTED,
                                    task_id=context.task_id, raw_response=raw_response,
                                    snapshot=snapshot)


def _resolved_facts(snapshot: StateSnapshot, *, now_monotonic_s: float,
                    entity_catalog: Mapping[str, str]) -> dict[FactKey, Truth]:
    keys = {item.key for item in snapshot.evidence
            if item.key.subject in entity_catalog and item.key.predicate in _VISUAL_PREDICATES}
    return {key: snapshot.resolve(key, now_monotonic_s=now_monotonic_s)
            for key in keys if snapshot.resolve(key, now_monotonic_s=now_monotonic_s) is not Truth.UNKNOWN}


def score_visual_snapshot(candidate: StateSnapshot, teacher: StateSnapshot, *,
                          now_monotonic_s: float,
                          entity_catalog: Mapping[str, str]) -> TeacherScore:
    """Compare explicit C02 facts; absence remains a missed/unknown observation, not FALSE."""
    if candidate.task_id != teacher.task_id or candidate.episode_id != teacher.episode_id:
        raise ValueError("candidate and teacher task/episode IDs must match")
    candidate_facts = _resolved_facts(candidate, now_monotonic_s=now_monotonic_s,
                                      entity_catalog=entity_catalog)
    teacher_facts = _resolved_facts(teacher, now_monotonic_s=now_monotonic_s,
                                    entity_catalog=entity_catalog)
    exact = mismatches = missed = extra = 0
    for key, truth in teacher_facts.items():
        candidate_truth = candidate_facts.get(key)
        if candidate_truth is None:
            missed += 1
        elif candidate_truth is truth:
            exact += 1
        else:
            mismatches += 1
    extra = len(set(candidate_facts) - set(teacher_facts))
    return TeacherScore(
        task_id=candidate.task_id, teacher_revision=teacher.revision,
        candidate_revision=candidate.revision, exact_matches=exact, mismatches=mismatches,
        missed_teacher_facts=missed, extra_candidate_facts=extra,
        precision_denominator=exact + mismatches + extra,
        recall_denominator=exact + mismatches + missed,
    )
