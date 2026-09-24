"""Catalog-bound VLM observations to C02 evidence, plus teacher scoring.

This is a semantic evidence boundary.  It deliberately imports no model, Isaac, ROS,
geometry, physics or execution code.  A runtime supplies raw VLM text and a trusted
media manifest; this module validates claims before they can enter RRM world state.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import re
from typing import Any, Mapping

from pydantic import BaseModel, ConfigDict, Field

from .contracts import Truth
from .state_contracts import FactEvidence, FactKey, FactProvenance, StateSnapshot


_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
_VISUAL_PREDICATES = frozenset({"exists", "kind", "localized"})


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
        "You are a visual world-building component for a body-agnostic robotics reasoning model.",
        "Inspect the supplied image or video. Use only the entity IDs and kinds in the catalog.",
        "Do not invent an entity ID, coordinate, safety state, capability, action, plan, or control command.",
        "Return exactly one JSON object:",
        '{"status":"READY|NEEDS_CLARIFICATION","claims":['
        '{"subject":str,"predicate":"exists|kind|localized","obj":str|null,'
        '"truth":"TRUE|FALSE"}]}.',
        "For READY provide one or more claims. For NEEDS_CLARIFICATION provide no claims.",
        "For predicate kind, obj must equal the catalog kind for that entity. For exists and localized, obj is null.",
        "Entity catalog follows:",
        json.dumps(catalog, sort_keys=True, separators=(",", ":")),
    ))


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
