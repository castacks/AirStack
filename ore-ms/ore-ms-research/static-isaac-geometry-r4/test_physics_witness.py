import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock

from physics_witness_r4 import PostStepLog, classify_controls


def state(native=0, world=3, manager=3, time_s=.3, *, invalid=0):
    return {"native_count": native, "world_count": world, "manager_count": manager,
            "invalid_count": invalid, "playing": True, "native_monotonic_s": time_s,
            "clock": {"graph": {"ports": {"Time.outputs:simulationTime": time_s}}}}


class PhysicsWitnessTests(unittest.TestCase):
    def test_subscription_and_lossless_ordered_logging(self):
        with TemporaryDirectory() as path:
            physx = Mock()
            log = PostStepLog(Path(path), physx)
            physx.subscribe_physics_on_step_events.assert_called_once_with(
                log.record, pre_step=False, order=0)
            self.assertIs(log.subscription, physx.subscribe_physics_on_step_events.return_value)
            for dt in (.1, .1, float('nan')):
                log.record(dt)
            events = [json.loads(line) for line in
                      (Path(path) / "physics-post-steps.jsonl").read_text().splitlines()]
            self.assertEqual([event["sequence"] for event in events], [1, 2, 3])
            self.assertEqual([event["dt_s"] for event in events], [.1, .1, 'nan'])
            self.assertEqual(log.counters(), (3, 1))

    def test_negative_and_positive_controls(self):
        start, stepped = state(), state(1, 4, 4, .4)
        self.assertEqual(classify_controls("A", start, start, start), "PASS")
        self.assertEqual(classify_controls("A", start, stepped, stepped), "INCONCLUSIVE")
        self.assertEqual(classify_controls("B", start, stepped, stepped), "PASS")
        self.assertEqual(classify_controls("B", start, start, start), "WITNESS_NOT_QUALIFIED")
        self.assertEqual(classify_controls("B", start, stepped, state(2, 5, 5)), "INCONCLUSIVE")

    def test_replicator_classification(self):
        start, stepped = state(), state(1, 4, 4, .4)
        disagreement = state(1, 3, 4, .4)
        clock = {"status": "PASS", "accepted_epoch_ns": 400000000}
        self.assertEqual(classify_controls("C", start, stepped, stepped, clock), "PASS")
        self.assertEqual(classify_controls("C", start, start, start, clock), "NO_PHYSICS_ADVANCEMENT")
        self.assertEqual(classify_controls("C", start, disagreement, disagreement), "COUNTER_DISAGREEMENT")
        self.assertEqual(classify_controls("C", start, stepped, stepped), "INCONCLUSIVE")
        self.assertEqual(classify_controls("C", start, stepped, state(1, 4, 4, .41), clock), "INCONCLUSIVE")

    def test_conflicting_or_invalid_post_step_data_cannot_pass(self):
        start = state()
        for ending in (state(0, 4, 4), state(1, 4, 4, invalid=1), state(1, 4, 5, .4)):
            self.assertEqual(classify_controls("C", start, ending, ending), "INCONCLUSIVE")
