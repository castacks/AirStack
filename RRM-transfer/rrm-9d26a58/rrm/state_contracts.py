"""C02 immutable state evidence, separate from the legacy mock ``WorldState``."""

from __future__ import annotations

import math
from enum import Enum

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .contracts import Truth


class FactProvenance(str, Enum):
    SIMULATOR = "SIMULATOR"
    SENSOR = "SENSOR"
    INFERRED = "INFERRED"
    OPERATOR = "OPERATOR"


class FactKey(BaseModel):
    """Ground semantic fact identity; geometry stays behind the embodiment boundary."""

    model_config = ConfigDict(frozen=True)

    subject: str
    predicate: str
    obj: str | float | None = None

    @model_validator(mode="after")
    def validate_identity(self) -> "FactKey":
        if not self.subject.strip() or not self.predicate.strip():
            raise ValueError("fact subject and predicate are required")
        if isinstance(self.obj, float) and not math.isfinite(self.obj):
            raise ValueError("fact object must be finite")
        return self


class FactEvidence(BaseModel):
    """An explicit observation; absence of an instance is never evidence by itself."""

    model_config = ConfigDict(frozen=True)

    key: FactKey
    truth: Truth
    provenance: FactProvenance
    source_ref: str
    observed_monotonic_s: float = Field(ge=0)
    received_monotonic_s: float = Field(ge=0)
    max_age_s: float = Field(gt=0)

    @model_validator(mode="after")
    def validate_evidence(self) -> "FactEvidence":
        if not self.source_ref.strip():
            raise ValueError("source reference is required")
        if not all(math.isfinite(value) for value in (
                self.observed_monotonic_s, self.received_monotonic_s, self.max_age_s)):
            raise ValueError("evidence times must be finite")
        if self.received_monotonic_s < self.observed_monotonic_s:
            raise ValueError("evidence cannot arrive before it was observed")
        return self

    def is_fresh(self, now_monotonic_s: float) -> bool:
        return (math.isfinite(now_monotonic_s) and now_monotonic_s >= self.received_monotonic_s
                and now_monotonic_s - self.received_monotonic_s <= self.max_age_s)


class StateSnapshot(BaseModel):
    """C02 task-relevant state view with explicit evidence gaps and revisions."""

    model_config = ConfigDict(frozen=True)

    snapshot_id: str
    revision: str
    task_id: str
    episode_id: str
    evidence: tuple[FactEvidence, ...] = ()
    complete_domains: frozenset[str] = frozenset()

    @model_validator(mode="after")
    def validate_snapshot(self) -> "StateSnapshot":
        for name in ("snapshot_id", "revision", "task_id", "episode_id"):
            if not getattr(self, name).strip():
                raise ValueError(f"{name} is required")
        if any(not domain.strip() for domain in self.complete_domains):
            raise ValueError("complete domains must be nonempty")
        return self

    def resolve(self, key: FactKey, *, now_monotonic_s: float, negated: bool = False) -> Truth:
        """Resolve only fresh, explicit and non-contradictory evidence.

        Coverage metadata deliberately does not turn an absent fact into FALSE: callers
        need a domain-specific closed-world rule before making that stronger claim.
        """
        truths = {item.truth for item in self.evidence if item.key == key and item.is_fresh(now_monotonic_s)}
        truth = truths.pop() if len(truths) == 1 else Truth.UNKNOWN
        return truth.negate() if negated else truth
