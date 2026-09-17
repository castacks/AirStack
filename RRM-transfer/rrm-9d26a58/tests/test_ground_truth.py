"""Tests for the labelled-simulator C02 teacher pipeline."""

import unittest

from rrm.airstack_drone import MapWaypoint
from rrm.contracts import CapabilityDeclaration, Truth
from rrm.drone_decision import AirStackDroneDecisionBridge, DroneDecisionStatus, DroneNavigationTarget
from rrm.ground_truth import GroundTruthWorldBuilder
from rrm.schema import Verb
from rrm.state_contracts import FactKey
from rrm.task_contracts import TaskRequest


def builder():
    return GroundTruthWorldBuilder(task_id="task-1", episode_id="episode-1")


def task():
    return TaskRequest(
        task_id="task-1", revision="task/v1", objective="navigate to marker-a",
        constraints_revision="constraints/v1", issuer_id="operator-1",
        permission_revision="permission/v1", requested_embodiment_id="iris-v1",
    )


def profile():
    return CapabilityDeclaration(
        embodiment_id="iris-v1", revision="iris/v1", operations=frozenset({Verb.NAVIGATE_TO.value}),
        resources=frozenset({"airframe"}), available_resources=frozenset({"airframe"}),
        limits_ref="iris/limits/v1",
    )


def bridge():
    return AirStackDroneDecisionBridge(
        embodiment_id="iris-v1", robot_name="robot_1",
        targets=(DroneNavigationTarget(
            entity_id="marker-a", waypoints=(MapWaypoint(x=1.0, y=2.0, z=3.0),), goal_tolerance_m=0.5,
        ),),
    )


class GroundTruthWorldBuilderTests(unittest.TestCase):
    def test_labelled_entity_creates_fresh_semantic_facts_and_revisions(self):
        world = builder()
        world.ingest_entity(entity_id="marker-a", entity_kind="inspection-marker", exists=True,
                            localized=True, observed_monotonic_s=10.0, received_monotonic_s=10.1)
        first = world.snapshot(complete_domains=frozenset({"simulator-labels"}))
        second = world.snapshot(complete_domains=frozenset({"simulator-labels"}))
        self.assertEqual(first.resolve(FactKey(subject="marker-a", predicate="exists"), now_monotonic_s=10.5),
                         Truth.TRUE)
        self.assertEqual(first.resolve(FactKey(subject="marker-a", predicate="localized"), now_monotonic_s=10.5),
                         Truth.TRUE)
        self.assertEqual(first.resolve(FactKey(subject="marker-a", predicate="kind", obj="inspection-marker"),
                                       now_monotonic_s=10.5), Truth.TRUE)
        self.assertNotEqual(first.revision, second.revision)

    def test_stale_or_contradictory_evidence_stays_unknown(self):
        world = builder()
        world.ingest_entity(entity_id="marker-a", entity_kind="marker", exists=True, localized=True,
                            observed_monotonic_s=1.0, received_monotonic_s=1.0)
        stale = world.snapshot()
        self.assertEqual(stale.resolve(FactKey(subject="marker-a", predicate="localized"), now_monotonic_s=2.1),
                         Truth.UNKNOWN)
        world.ingest_relation(subject="marker-a", predicate="localized", obj="map", truth=Truth.TRUE,
                              observed_monotonic_s=3.0, received_monotonic_s=3.0, source_ref="sim-a")
        world.ingest_relation(subject="marker-a", predicate="localized", obj="map", truth=Truth.FALSE,
                              observed_monotonic_s=3.0, received_monotonic_s=3.1, source_ref="sim-b")
        conflicting = world.snapshot()
        self.assertEqual(conflicting.resolve(FactKey(subject="marker-a", predicate="localized", obj="map"),
                                             now_monotonic_s=3.2), Truth.UNKNOWN)

    def test_source_time_regression_and_episode_carryover_are_rejected(self):
        world = builder()
        world.ingest_entity(entity_id="marker-a", entity_kind="marker", exists=True, localized=True,
                            observed_monotonic_s=10.0, received_monotonic_s=10.0)
        with self.assertRaises(ValueError):
            world.ingest_entity(entity_id="marker-a", entity_kind="marker", exists=True, localized=True,
                                observed_monotonic_s=9.0, received_monotonic_s=9.0)
        world.reset_episode("episode-2")
        reset = world.snapshot()
        self.assertEqual(reset.episode_id, "episode-2")
        self.assertEqual(reset.resolve(FactKey(subject="marker-a", predicate="exists"), now_monotonic_s=10.1),
                         Truth.UNKNOWN)

    def test_ground_truth_snapshot_can_drive_only_a_dry_run_rrm_proposal(self):
        world = builder()
        world.ingest_entity(entity_id="marker-a", entity_kind="inspection-marker", exists=True, localized=True,
                            observed_monotonic_s=10.0, received_monotonic_s=10.0)
        result = bridge().decide(task(), world.snapshot(), profile(), now_monotonic_s=10.5)
        self.assertIs(result.status, DroneDecisionStatus.READY)
        self.assertEqual(result.proposal.preview()["action_name"], "/robot_1/tasks/navigate")
        world.ingest_entity(entity_id="marker-a", entity_kind="inspection-marker", exists=False, localized=False,
                            observed_monotonic_s=11.0, received_monotonic_s=11.0)
        held = bridge().decide(task(), world.snapshot(), profile(), now_monotonic_s=11.1)
        self.assertIs(held.status, DroneDecisionStatus.HOLD)
        self.assertIsNone(held.proposal)


if __name__ == "__main__":
    unittest.main()
