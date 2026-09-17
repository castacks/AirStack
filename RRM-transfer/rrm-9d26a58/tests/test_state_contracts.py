"""C02 state evidence must fail closed for stale, missing and contradictory facts."""

import unittest

from pydantic import ValidationError

from rrm.contracts import Truth
from rrm.state_contracts import FactEvidence, FactKey, FactProvenance, StateSnapshot


KEY = FactKey(subject="cup-1", predicate="on", obj="table-1")


def evidence(truth=Truth.TRUE, **changes):
    values = dict(key=KEY, truth=truth, provenance=FactProvenance.SIMULATOR,
                  source_ref="sim/frame-1", observed_monotonic_s=10.0,
                  received_monotonic_s=10.1, max_age_s=1.0)
    values.update(changes)
    return FactEvidence(**values)


def snapshot(*facts):
    return StateSnapshot(snapshot_id="snapshot-1", revision="state/v1", task_id="task-1",
                         episode_id="episode-1", evidence=facts)


class StateContractTests(unittest.TestCase):
    def test_fresh_explicit_truth_and_negative_evidence_resolve(self):
        self.assertIs(snapshot(evidence()).resolve(KEY, now_monotonic_s=10.5), Truth.TRUE)
        self.assertIs(snapshot(evidence(Truth.FALSE)).resolve(KEY, now_monotonic_s=10.5, negated=True), Truth.TRUE)

    def test_missing_stale_unknown_and_negated_unknown_stay_unknown(self):
        self.assertIs(snapshot().resolve(KEY, now_monotonic_s=10.5), Truth.UNKNOWN)
        self.assertIs(snapshot(evidence()).resolve(KEY, now_monotonic_s=11.2), Truth.UNKNOWN)
        self.assertIs(snapshot(evidence(Truth.UNKNOWN)).resolve(KEY, now_monotonic_s=10.5, negated=True), Truth.UNKNOWN)

    def test_contradictory_fresh_evidence_fails_closed(self):
        state = snapshot(evidence(Truth.TRUE), evidence(Truth.FALSE, source_ref="sim/frame-2"))
        self.assertIs(state.resolve(KEY, now_monotonic_s=10.5), Truth.UNKNOWN)

    def test_coverage_does_not_convert_absence_to_negative_evidence(self):
        state = StateSnapshot(snapshot_id="snapshot-1", revision="state/v1", task_id="task-1",
                              episode_id="episode-1", complete_domains={"supports"})
        self.assertIs(state.resolve(KEY, now_monotonic_s=10.5, negated=True), Truth.UNKNOWN)

    def test_invalid_time_and_identity_are_rejected(self):
        with self.assertRaises(ValidationError):
            FactKey(subject="", predicate="on")
        with self.assertRaises(ValidationError):
            evidence(received_monotonic_s=9.0)
        with self.assertRaises(ValidationError):
            evidence(max_age_s=float("inf"))


if __name__ == "__main__":
    unittest.main()
