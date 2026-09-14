"""Unit tests for the read-only AirStack RRM shadow adapter."""

from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from rrm.airstack_shadow import (
    AirStackShadowAdapter,
    EvidenceStatus,
    JsonlEvidenceWriter,
)


class ShadowAdapterTests(unittest.TestCase):
    def make_adapter(self, **kwargs):
        return AirStackShadowAdapter(
            run_id="run-1", robot_name="robot_1", max_observation_age_s=1.0, **kwargs
        )

    def test_missing_evidence_is_incomplete_and_execution_stays_inhibited(self):
        snapshot = self.make_adapter().snapshot(now_monotonic_s=10.0, wall_time_ns=1)
        self.assertEqual(snapshot.readiness, "OBSERVATION_INCOMPLETE")
        self.assertTrue(snapshot.execution_inhibited)
        self.assertIs(snapshot.odometry.status, EvidenceStatus.MISSING)
        self.assertIsNone(snapshot.position)

    def test_fresh_canonical_observations_are_available_only_for_shadow_reasoning(self):
        adapter = self.make_adapter()
        adapter.ingest_odometry(x=1.0, y=2.0, z=3.0, frame_id="map",
                                child_frame_id="base_link", source_stamp_ns=99,
                                received_monotonic_s=10.0)
        adapter.ingest_mavros_state(connected=True, armed=False, mode="AUTO.LOITER",
                                    received_monotonic_s=10.0)
        adapter.ingest_map_to_base_link_transform(received_monotonic_s=10.0)
        snapshot = adapter.snapshot(now_monotonic_s=10.5, wall_time_ns=2)
        self.assertEqual(snapshot.readiness, "OBSERVATION_READY")
        self.assertEqual(snapshot.position.frame_id, "map")
        self.assertEqual(snapshot.mavros_mode, "AUTO.LOITER")
        self.assertTrue(snapshot.execution_inhibited)
        report = adapter.completeness_report(snapshot)
        self.assertTrue(report["observation_complete"])
        self.assertFalse(report["execution_dispatch_enabled"])

    def test_stale_or_disconnected_evidence_cannot_appear_ready(self):
        adapter = self.make_adapter()
        adapter.ingest_odometry(x=0.0, y=0.0, z=0.0, frame_id="map",
                                child_frame_id="base_link", source_stamp_ns=None,
                                received_monotonic_s=1.0)
        adapter.ingest_mavros_state(connected=False, armed=False, mode="", received_monotonic_s=3.0)
        adapter.ingest_map_to_base_link_transform(received_monotonic_s=3.0)
        snapshot = adapter.snapshot(now_monotonic_s=3.1, wall_time_ns=3)
        self.assertIs(snapshot.odometry.status, EvidenceStatus.STALE)
        self.assertIsNone(snapshot.position)
        self.assertIn("odometry_stale", snapshot.reasons)
        self.assertIn("mavros_disconnected", snapshot.reasons)
        self.assertFalse(adapter.completeness_report(snapshot)["observation_complete"])

    def test_writer_produces_replayable_observer_only_bundle(self):
        with TemporaryDirectory() as directory:
            output = Path(directory)
            writer = JsonlEvidenceWriter(output, {"run_id": "run-1", "robot_name": "robot_1"})
            adapter = self.make_adapter(writer=writer)
            adapter.ingest_task_status(action="takeoff", status_codes=[2], received_monotonic_s=1.0)
            adapter.snapshot(now_monotonic_s=1.0, wall_time_ns=4)
            writer.close()
            manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
            events = [json.loads(line) for line in (output / "events.jsonl").read_text(encoding="utf-8").splitlines()]
        self.assertTrue(manifest["observer_only"])
        self.assertFalse(manifest["execution_dispatch_enabled"])
        self.assertEqual([event["kind"] for event in events],
                         ["run_start", "task_status_observed", "shadow_snapshot", "run_end"])

    def test_ros_runner_contains_no_control_primitives(self):
        runner = (Path(__file__).parents[1] / "scripts" / "airstack_shadow.py").read_text(
            encoding="utf-8"
        )
        for prohibited in ("ActionClient", "create_publisher", "create_client",
                           "send_goal", "call_async"):
            self.assertNotIn(prohibited, runner)
        self.assertIn("create_subscription", runner)


if __name__ == "__main__":
    unittest.main()
