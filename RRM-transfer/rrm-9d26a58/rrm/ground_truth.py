"""Ground-truth teacher pipeline for C02 world-state evaluation.

The builder turns explicitly labelled simulator observations into the same immutable
semantic facts future sensor/VLM adapters must produce.  It intentionally contains no
Isaac, ROS, geometry, physics, or control dependency: simulator-specific pose/prim
handling belongs upstream, and coordinates remain at the embodiment boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .contracts import Truth
from .state_contracts import FactEvidence, FactKey, FactProvenance, StateSnapshot


@dataclass(frozen=True)
class _EvidenceIdentity:
    source_ref: str
    subject: str
    predicate: str
    obj: str | float | None


class GroundTruthWorldBuilder:
    """Accumulate labelled observations into versioned C02 snapshots.

    A source can revise only its own fact with a strictly later receive time. Evidence
    from distinct sources deliberately remains separate, allowing ``StateSnapshot``
    to expose contradiction as ``UNKNOWN`` rather than picking an arbitrary winner.
    """

    def __init__(self, *, task_id: str, episode_id: str,
                 default_source_ref: str = "isaac-ground-truth",
                 default_max_age_s: float = 1.0) -> None:
        if not task_id.strip() or not episode_id.strip() or not default_source_ref.strip():
            raise ValueError("task, episode and source IDs are required")
        if default_max_age_s <= 0:
            raise ValueError("default_max_age_s must be positive")
        self.task_id = task_id
        self.episode_id = episode_id
        self.default_source_ref = default_source_ref
        self.default_max_age_s = default_max_age_s
        self._sequence = 0
        self._evidence: dict[_EvidenceIdentity, FactEvidence] = {}

    def ingest_fact(self, key: FactKey, truth: Truth, *, observed_monotonic_s: float,
                    received_monotonic_s: float, source_ref: str | None = None,
                    max_age_s: float | None = None) -> FactEvidence:
        """Store one explicit simulator fact without collapsing other sources."""
        source = self.default_source_ref if source_ref is None else source_ref
        age = self.default_max_age_s if max_age_s is None else max_age_s
        evidence = FactEvidence(
            key=key,
            truth=truth,
            provenance=FactProvenance.SIMULATOR,
            source_ref=source,
            observed_monotonic_s=observed_monotonic_s,
            received_monotonic_s=received_monotonic_s,
            max_age_s=age,
        )
        identity = _EvidenceIdentity(source, key.subject, key.predicate, key.obj)
        previous = self._evidence.get(identity)
        if previous is not None and evidence.received_monotonic_s <= previous.received_monotonic_s:
            raise ValueError("source fact evidence must advance received time")
        self._evidence[identity] = evidence
        return evidence

    def ingest_entity(self, *, entity_id: str, entity_kind: str, exists: bool,
                      localized: bool | None, observed_monotonic_s: float,
                      received_monotonic_s: float, source_ref: str | None = None,
                      max_age_s: float | None = None) -> tuple[FactEvidence, ...]:
        """Convenience conversion for explicit simulator entity labels.

        ``localized=None`` means that the label source made no localization claim; it
        is not converted into a false fact.  This mirrors future VLM partial output.
        """
        if not entity_id.strip() or not entity_kind.strip():
            raise ValueError("entity ID and kind are required")
        common: dict[str, Any] = {
            "observed_monotonic_s": observed_monotonic_s,
            "received_monotonic_s": received_monotonic_s,
            "source_ref": source_ref,
            "max_age_s": max_age_s,
        }
        records = [
            self.ingest_fact(FactKey(subject=entity_id, predicate="exists"),
                             Truth.TRUE if exists else Truth.FALSE, **common),
        ]
        if exists:
            records.append(self.ingest_fact(
                FactKey(subject=entity_id, predicate="kind", obj=entity_kind), Truth.TRUE, **common,
            ))
        if localized is not None:
            records.append(self.ingest_fact(
                FactKey(subject=entity_id, predicate="localized"),
                Truth.TRUE if localized else Truth.FALSE, **common,
            ))
        return tuple(records)

    def ingest_relation(self, *, subject: str, predicate: str, obj: str | float,
                        truth: Truth, observed_monotonic_s: float,
                        received_monotonic_s: float, source_ref: str | None = None,
                        max_age_s: float | None = None) -> FactEvidence:
        """Store an explicit labelled relation such as ``near(robot, marker)``."""
        return self.ingest_fact(
            FactKey(subject=subject, predicate=predicate, obj=obj), truth,
            observed_monotonic_s=observed_monotonic_s,
            received_monotonic_s=received_monotonic_s,
            source_ref=source_ref,
            max_age_s=max_age_s,
        )

    def snapshot(self, *, complete_domains: frozenset[str] = frozenset()) -> StateSnapshot:
        """Produce an immutable task-relevant snapshot; no fact is inferred here."""
        if any(not domain.strip() for domain in complete_domains):
            raise ValueError("complete domains must be nonempty")
        self._sequence += 1
        return StateSnapshot(
            snapshot_id=f"{self.episode_id}/snapshot-{self._sequence}",
            revision=f"{self.episode_id}/state-{self._sequence}",
            task_id=self.task_id,
            episode_id=self.episode_id,
            evidence=tuple(self._evidence.values()),
            complete_domains=complete_domains,
        )

    def reset_episode(self, episode_id: str) -> None:
        """Drop prior-episode belief; reset never carries facts across simulation reset."""
        if not episode_id.strip() or episode_id == self.episode_id:
            raise ValueError("reset requires a distinct nonempty episode ID")
        self.episode_id = episode_id
        self._sequence = 0
        self._evidence.clear()
