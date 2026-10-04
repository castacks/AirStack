"""Safety labels finalize before worker success without per-gate storage sync."""

import json
import os
import tempfile
import unittest
from pathlib import Path
from threading import Event, Thread
from unittest.mock import patch

from rrm.acceptance_campaign import digest, evaluate, specification, worker, write_json
from rrm.acceptance_fixtures import FixtureTracer
from rrm.core_deadlines import BoundedTracer


class SidecarFinalizationTests(unittest.TestCase):
    def tracer(self, directory):
        return FixtureTracer(Path(directory) / "events.jsonl", {
            "task_id": "sidecar", "expect_abort": False,
            "scenario_id": "nominal-pick", "configuration_sha256": "fixture-config",
        }, "nominal")

    def gate(self, tracer, tick=0):
        return tracer.event("safety1", verdict="PASS", sim_t=tick,
                            action_id="pick", action_digest="action", plan_version=0,
                            state_digest="state")

    def test_labels_are_visible_before_one_final_sync_and_close_is_idempotent(self):
        real_sync = os.fsync
        with tempfile.TemporaryDirectory() as directory:
            tracer = self.tracer(directory)
            path = Path(directory) / "safety-labels.jsonl"
            synced = []

            def sync(fd):
                synced.append((fd, path.read_text()))
                real_sync(fd)

            with patch("rrm.acceptance_fixtures.os.fsync", side_effect=sync):
                self.gate(tracer)
                self.gate(tracer, 1)
                labels = [json.loads(line) for line in path.read_text().splitlines()]
                self.assertEqual([label["sim_t"] for label in labels], [0, 1])
                self.assertEqual(synced, [])
                tracer.close()
                tracer.close()
            self.assertEqual(len(synced), 1)
            self.assertEqual(synced[0][1], path.read_text())
            self.assertTrue(tracer._labels.closed)
            self.assertIsNone(tracer._fh)

    def test_sync_failure_propagates_and_closes_both_files(self):
        with tempfile.TemporaryDirectory() as directory:
            tracer = self.tracer(directory)
            self.gate(tracer)
            with patch("rrm.acceptance_fixtures.os.fsync", side_effect=OSError("storage fault")):
                with self.assertRaisesRegex(OSError, "storage fault"):
                    tracer.close()
                tracer.close()
            self.assertTrue(tracer._labels.closed)
            self.assertIsNone(tracer._fh)

    def test_label_flush_failure_still_closes_both_files(self):
        with tempfile.TemporaryDirectory() as directory:
            tracer = self.tracer(directory)
            with patch.object(tracer._labels, "flush", side_effect=OSError("flush fault")):
                with self.assertRaises(OSError):
                    tracer.close()
            self.assertTrue(tracer._labels.closed)
            self.assertIsNone(tracer._fh)

    def test_stalled_final_sync_does_not_enter_a_gate_callback(self):
        entered, release, finished = Event(), Event(), Event()
        errors = []
        with tempfile.TemporaryDirectory() as directory:
            tracer = self.tracer(directory)
            bounded = BoundedTracer(tracer, 0.2)

            def sync(fd):
                entered.set()
                if not release.wait(2):
                    raise RuntimeError("test did not release finalization")

            def close():
                try:
                    tracer.close()
                except BaseException as error:
                    errors.append(error)
                finally:
                    finished.set()

            with patch("rrm.acceptance_fixtures.os.fsync", side_effect=sync):
                self.gate(bounded)
                self.assertFalse(entered.is_set())
                thread = Thread(target=close, daemon=True)
                thread.start()
                try:
                    self.assertTrue(entered.wait(1))
                    self.assertFalse(finished.is_set())
                finally:
                    release.set()
                    thread.join(2)
            self.assertTrue(finished.is_set())
            self.assertEqual(errors, [])

    def test_worker_finalization_failure_retains_unknown_attempt_without_result(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = specification(["nominal-pick"])
            config_hash = digest(spec)
            write_json(root / "campaign.json", {"spec": spec, "sha256": config_hash})
            attempt = spec["attempts"][0]
            target = root / "attempts" / attempt["attempt_id"]
            target.mkdir(parents=True)
            with patch("rrm.acceptance_fixtures.os.fsync", side_effect=OSError("final sync")):
                with self.assertRaisesRegex(OSError, "final sync"):
                    worker(target, attempt["attempt_id"])
            self.assertFalse((target / "result.json").exists())
            self.assertTrue((target / "safety-labels.jsonl").is_file())
            self.assertTrue((target / "events.jsonl").is_file())
            report = evaluate(target, attempt, config_hash, {
                "status": "EXITED", "returncode": 1, "elapsed_s": 0.5,
            })
            self.assertTrue(report["replay_valid"], report["findings"])
            self.assertFalse(report["evidence_complete"])
            self.assertFalse(report["acceptance_pass"])
            self.assertIsNone(report["goal_met"])
            self.assertEqual(report["stop_status"], "UNKNOWN")
            self.assertFalse(report["safety_adjudication"]["qualified"])


if __name__ == "__main__":
    unittest.main()
