"""Task intake and HTTP boundary tests; no model, ROS or drone execution."""
import json
from pathlib import Path
import sys
import threading
import tempfile
import unittest
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from rrm_command_console import Console, make_handler, save_request
from rrm_cosmos_reason2 import load_context
from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal, MapWaypoint
from rrm.execution_supervisor import ExecutionSupervisor, RunningDispatch
import test_office_import


class FakeProcess:
    def __init__(self):
        self.finished = threading.Event()
        self.return_code = 0

    def poll(self):
        return self.return_code if self.finished.is_set() else None

    def wait(self):
        self.finished.wait(2)
        return self.return_code


def navigation_proposal():
    return DroneTaskProposal(
        task_id="office-task", action_id="NAVIGATE_TO", kind=DroneTaskKind.NAVIGATE,
        frame_id="map", waypoints=(MapWaypoint(x=3.2, y=0.0, z=1.5),),
        goal_tolerance_m=0.3,
    )


class ExecutionSupervisorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.launches = []
        self.stop_calls = 0

        def launch(dispatch_id, run_dir, proposal_path):
            process = FakeProcess()
            self.launches.append((dispatch_id, run_dir, proposal_path, process))

            def stop():
                self.stop_calls += 1
                process.return_code = 130
                process.finished.set()

            return RunningDispatch(
                process=process,
                request_stop=stop,
                finalize=lambda code: {"return_code": code, "verdict": "UNCONFIRMED"},
            )

        self.supervisor = ExecutionSupervisor(
            navigation_proposal(), Path(self.temp.name), launch
        )

    def test_exact_explicit_approval_records_before_single_launch(self):
        with self.assertRaisesRegex(ValueError, "changed"):
            self.supervisor.decide("APPROVE", "0" * 64)
        self.assertFalse(self.launches)
        status = self.supervisor.decide(
            "APPROVE", self.supervisor.proposal_sha256
        )
        self.assertEqual(status["state"], "RUNNING")
        self.assertEqual(len(self.launches), 1)
        dispatch_id, run_dir, proposal_path, _ = self.launches[0]
        admission = json.loads((run_dir / "admission.json").read_text())
        self.assertEqual(admission["dispatch_id"], dispatch_id)
        self.assertEqual(admission["proposal_sha256"], self.supervisor.proposal_sha256)
        self.assertTrue(admission["execution_requested"])
        self.assertEqual(json.loads(proposal_path.read_text())["action_id"], "NAVIGATE_TO")
        with self.assertRaisesRegex(ValueError, "already active"):
            self.supervisor.decide("APPROVE", self.supervisor.proposal_sha256)

    def test_reject_is_durable_and_never_launches(self):
        status = self.supervisor.decide("REJECT", self.supervisor.proposal_sha256)
        self.assertEqual(status["state"], "REJECTED")
        self.assertEqual(status["last_result"]["verdict"], "NOT_DISPATCHED")
        self.assertFalse(self.launches)
        admission = next(Path(self.temp.name).glob("*/admission.json"))
        self.assertFalse(json.loads(admission.read_text())["execution_requested"])

    def test_stop_latches_without_motion_and_blocks_admission(self):
        status = self.supervisor.request_stop()
        self.assertTrue(status["stop_latched"])
        self.assertEqual(status["safety_claim"], "SAFE_UNCONFIRMED")
        self.assertEqual(self.stop_calls, 0)
        with self.assertRaisesRegex(ValueError, "stopped"):
            self.supervisor.decide("APPROVE", self.supervisor.proposal_sha256)

    def test_restart_with_prior_admission_requires_reconciliation(self):
        self.supervisor.decide("REJECT", self.supervisor.proposal_sha256)
        restarted = ExecutionSupervisor(
            navigation_proposal(), Path(self.temp.name), lambda *_: self.fail("must not launch")
        )
        status = restarted.status()
        self.assertEqual(status["state"], "RECONCILIATION_REQUIRED")
        self.assertTrue(status["stop_latched"])

    def test_stop_interrupts_active_dispatch_once_without_safe_claim(self):
        self.supervisor.decide("APPROVE", self.supervisor.proposal_sha256)
        status = self.supervisor.request_stop()
        self.assertEqual(self.stop_calls, 1)
        self.assertEqual(status["state"], "STOP_REQUESTED")
        self.assertEqual(status["safety_claim"], "SAFE_UNCONFIRMED")
        stop_record = next(Path(self.temp.name).glob("*/stop-1.json"))
        self.assertTrue(json.loads(stop_record.read_text())["cancel_requested"])


class CommandConsoleTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_office_import.OfficeImportTests("test_actual_plan_ids_survive_import")
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.record["execution_dispatch"] = False
        self.fixture.save()
        self.bundle = self.fixture.bundle
        self.output = self.bundle / "requests"

    def test_goal_reuse_and_restart_preserve_independent_runs(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        one = app.save("Approach the blue marker.")
        two = app.save("Approach the blue marker.", one["goal_id"])
        self.assertNotEqual(one["request_id"], two["request_id"])
        restarted = Console(self.bundle, self.output, "/unused-capture.py")
        goal = next(g for g in restarted.store.history() if g["goal_id"] == one["goal_id"])
        self.assertEqual(len(goal["runs"]), 2)
        self.assertTrue(all(run["status"] == "SAVED_NOT_SUBMITTED" for run in goal["runs"]))
        self.assertTrue(all(run["execution_state"] == "NOT_DISPATCHED" for run in goal["runs"]))
        self.assertEqual(len(restarted.store.history()), 2)  # plus prior accepted reference
        reference = next(g for g in restarted.store.history() if g["goal_id"] != one["goal_id"])
        self.assertEqual(len(reference["runs"]), 1)
        self.assertEqual(reference["runs"][0]["status"], "CANDIDATE_ACCEPTED")

    def test_selected_goal_cannot_be_rewritten(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        original = app.save("Blue marker only.")
        before = sorted(self.output.glob("*/request.json"))
        with self.assertRaisesRegex(ValueError, "new goal"):
            app.save("Orange marker instead.", original["goal_id"])
        self.assertEqual(before, sorted(self.output.glob("*/request.json")))
        self.assertEqual(app.store.get_goal(original["goal_id"])["objective"], "Blue marker only.")

    def test_legacy_and_unindexed_requests_are_recovered_once(self):
        saved = save_request(self.bundle, self.output, "Stored before database existed.")
        manifest_path = self.output / saved["request_id"] / "request.json"
        legacy = json.loads(manifest_path.read_text())
        del legacy["goal_id"]
        manifest_path.write_text(json.dumps(legacy))
        before = manifest_path.read_bytes()
        app = Console(self.bundle, self.output, "/unused-capture.py")
        app.recover_requests()
        self.assertEqual(len(app.store.get_goal(saved["request_id"])), 5)
        goal = next(g for g in app.store.history() if g["goal_id"] == saved["request_id"])
        self.assertEqual(len(goal["runs"]), 1)
        self.assertEqual(manifest_path.read_bytes(), before)

    def test_recovery_refuses_changed_input_evidence(self):
        saved = save_request(self.bundle, self.output, "Original command.")
        (self.output / saved["request_id"] / "input.png").write_bytes(b"modified")
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            Console(self.bundle, self.output, "/unused-capture.py")

    def test_new_request_preserves_frozen_context_and_has_unique_identity(self):
        before = (self.bundle / "input.json").read_bytes()
        one = save_request(self.bundle, self.output, "Inspect whether the blue marker is visible.")
        two = save_request(self.bundle, self.output, "Locate the orange marker.")
        self.assertNotEqual(one["task_id"], two["task_id"])
        context = load_context(self.output / one["request_id"] / "input.json")
        original = load_context(self.bundle / "input.json")
        self.assertEqual(context.snapshot.evidence, original.snapshot.evidence)
        self.assertEqual(context.now_monotonic_s, original.now_monotonic_s)
        self.assertEqual(context.capabilities, original.capabilities)
        self.assertEqual(context.task.permission_revision, "inference-only")
        self.assertEqual(context.task.task_id, context.snapshot.task_id)
        self.assertNotEqual(context.snapshot.revision, original.snapshot.revision)
        self.assertEqual(one["status"], "SAVED_NOT_SUBMITTED")
        self.assertFalse(one["execution_dispatch"])
        self.assertEqual((self.bundle / "input.json").read_bytes(), before)

    def test_invalid_objectives_do_not_create_requests(self):
        for objective in (None, "  ", "x" * 5001, []):
            with self.subTest(objective_type=type(objective).__name__), self.assertRaises(ValueError):
                save_request(self.bundle, self.output, objective)
        self.assertFalse(self.output.exists())

    def test_http_intake_requires_nonce_and_serves_only_request_files(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f"http://127.0.0.1:{server.server_port}"
        with urlopen(base + "/api/state") as response:
            state = json.load(response)
        data = json.dumps({"objective": "Approach the blue marker."}).encode()
        with self.assertRaises(HTTPError) as error:
            urlopen(Request(base + "/api/requests", data=data))
        self.assertEqual(error.exception.code, 403)
        with urlopen(Request(base + "/api/requests", data=data,
                            headers={"X-RRM-Token": state["token"]})) as response:
            self.assertEqual(response.status, 201)
            saved = json.load(response)
        with urlopen(base + f"/requests/{saved['request_id']}/input.json") as response:
            self.assertEqual(json.load(response)["task"]["objective"], "Approach the blue marker.")
        with urlopen(base + "/api/goals") as response:
            goals = json.load(response)["goals"]
            self.assertEqual(len(goals), 2)
        with urlopen(base + "/api/execution") as response:
            execution = json.load(response)
            self.assertEqual(execution["state"], "READY_FOR_APPROVAL")
            self.assertFalse(execution["active"])
            self.assertEqual(len(execution["proposal_sha256"]), 64)
        with urlopen(base + f"/runs/{saved['request_id']}/request.json") as response:
            self.assertEqual(json.load(response)["goal_id"], saved["goal_id"])
        with self.assertRaises(HTTPError) as error:
            urlopen(base + "/requests/../result.json")
        self.assertEqual(error.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
