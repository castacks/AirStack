"""Task intake and HTTP boundary tests; no model, ROS or drone execution."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import secrets
import sys
import threading
import tempfile
import time
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from rrm_command_console import Console, isaac_scene_catalog, make_handler, private_cosmos_worker_url, save_request
from rrm_cosmos_reason2 import load_context
from rrm.cosmos_reason2 import parse_cosmos_candidate, render_cosmos_prompt
from rrm.airstack_drone import DroneTaskKind, DroneTaskProposal, MapWaypoint
from rrm.execution_supervisor import ExecutionSupervisor, RunningDispatch
from rrm.live_replan import LiveCycleResponse
import test_office_import


class PrivateCosmosWorkerUrlTests(unittest.TestCase):
    def test_explicit_process_value_takes_precedence(self):
        with tempfile.TemporaryDirectory() as directory:
            init_environment = Path(directory) / "environ"
            init_environment.write_bytes(b"RRM_COSMOS_WORKER_URL=http://init-worker:8090\0")
            self.assertEqual(
                private_cosmos_worker_url(
                    environment={"RRM_COSMOS_WORKER_URL": "http://explicit-worker:8090"},
                    init_environment_path=init_environment,
                ),
                "http://explicit-worker:8090",
            )

    def test_falls_back_to_osmo_init_environment_only_when_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            init_environment = Path(directory) / "environ"
            init_environment.write_bytes(
                b"UNRELATED=value\0RRM_COSMOS_WORKER_URL=http://private-worker:8090\0"
            )
            self.assertEqual(
                private_cosmos_worker_url(environment={}, init_environment_path=init_environment),
                "http://private-worker:8090",
            )

    def test_missing_or_unreadable_init_environment_leaves_worker_unconfigured(self):
        self.assertEqual(
            private_cosmos_worker_url(
                environment={}, init_environment_path=Path("/nonexistent/rrm-init-environ")
            ),
            "",
        )

    def test_malformed_init_value_leaves_worker_unconfigured(self):
        with tempfile.TemporaryDirectory() as directory:
            init_environment = Path(directory) / "environ"
            init_environment.write_bytes(b"RRM_COSMOS_WORKER_URL=http://\xff:8090\0")
            self.assertEqual(
                private_cosmos_worker_url(environment={}, init_environment_path=init_environment), ""
            )


class CommandConsoleUiTests(unittest.TestCase):
    def test_goal_field_starts_empty_without_losing_its_label(self):
        ui = (Path(__file__).parents[1] / "scripts" / "ui" / "command_console.html").read_text()
        self.assertIn('<label for="objective">What should RRM accomplish?</label>', ui)
        self.assertIn('id="objective" maxlength="5000" required', ui)
        self.assertNotIn("byId('objective').value=data.context.task.objective", ui)

    def test_compiled_plan_panel_exposes_grounding_and_replan_policy(self):
        ui = (Path(__file__).parents[1] / "scripts" / "ui" / "command_console.html").read_text()
        self.assertIn("Compiled task plan and parameter grounding", ui)
        self.assertIn("grounding:result.plan.grounding", ui)
        self.assertIn("replan_policy:result.plan.replan_policy", ui)

    def test_cross_embodiment_preview_is_visibly_non_executing(self):
        ui = (Path(__file__).parents[1] / "scripts" / "ui" / "command_console.html").read_text()
        self.assertIn("Cross-embodiment goal preview", ui)
        self.assertIn('id="preview-hand"', ui)
        self.assertIn("/api/goal-previews/hand", ui)
        self.assertIn("cannot send a simulator action", ui)
        self.assertIn("numeric_feasibility_verified:result.numeric_feasibility_verified", ui)
        self.assertIn("simulator_action_sent:result.simulator_action_sent", ui)

    def test_mission_console_removes_manual_shadow_workflow_controls(self):
        ui = (Path(__file__).parents[1] / "scripts" / "ui" / "command_console.html").read_text()
        self.assertIn("RRM mission", ui)
        self.assertIn("Plan and run", ui)
        self.assertIn("AirStack task planning and execution", ui)
        self.assertIn("/execute", ui)
        self.assertIn("/api/mission/stop", ui)
        self.assertNotIn("/propose", ui)
        self.assertNotIn("PSC", ui)
        self.assertNotIn("/submit", ui)
        self.assertNotIn("rrm_psc_bridge_manual.sh", ui)
        self.assertNotIn('id="live-run-id"', ui)
        self.assertNotIn('id="live-entities"', ui)
        self.assertNotIn('id="live-review"', ui)
        self.assertNotIn("Approve &amp; send", ui)
        self.assertIn('id="scene-select"', ui)
        self.assertIn("/api/scene", ui)
        self.assertNotIn("/api/reset", ui)
        self.assertIn("Isaac Sim edits and terminal launch parameters remain valid", ui)
        self.assertIn("max-height:min(72vh,680px);overflow:auto", ui)
        self.assertIn('id="mission-plan"', ui)
        self.assertIn('id="mission-events"', ui)
        self.assertIn("max-height:240px;overflow:auto", ui)
        self.assertNotIn("confirm('Run this command", ui)

    def test_goal_save_retries_are_idempotent_and_scene_status_is_explicit(self):
        ui = (Path(__file__).parents[1] / "scripts" / "ui" / "command_console.html").read_text()
        self.assertIn("request_key:pendingSaveKey", ui)
        self.assertIn("run.run_id===pendingSaveKey", ui)
        self.assertIn("Use saved attempt", ui)
        self.assertIn("same request key will be reused", ui)
        self.assertIn("entity-grounded RRM is inhibited", ui)
        self.assertIn("Flight commands use live AirStack discovery", ui)

    def test_mission_log_exposes_only_recent_structured_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.log"
            path.write_text(
                "ROS noise\n"
                + "\n".join(json.dumps({"event": "feedback", "sequence": number})
                              for number in range(5))
                + "\n",
                encoding="utf-8",
            )
            self.assertEqual(
                [event["sequence"] for event in Console._mission_events(path, limit=2)],
                [3, 4],
            )

    def test_mission_log_collapses_identical_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.log"
            feedback = json.dumps({"event": "feedback", "action_id": "takeoff", "status": ""})
            path.write_text("\n".join([feedback, feedback, feedback]) + "\n", encoding="utf-8")
            self.assertEqual(Console._mission_events(path), [{
                "event": "feedback", "action_id": "takeoff", "status": "", "repeat_count": 3,
            }])


class TaskDiscoveryPackagingTests(unittest.TestCase):
    def test_fresh_robot_container_receives_discovery_dependency_before_read(self):
        """A fresh container has no local RRM module until the console stages it."""
        staged = {"script": False, "package": False}
        calls = []
        report = {
            "schema_version": "airstack-task-discovery/v1",
            "execution_dispatch": False,
            "task_servers": {},
        }

        def clean_container(command, **_kwargs):
            calls.append(command)
            if command[:4] == ["docker", "exec", "airstack-robot-desktop-1", "env"]:
                return type("Completed", (), {"returncode": 0, "stdout": ""})()
            if command[:2] == ["docker", "cp"]:
                source = Path(command[2])
                if source.name == "airstack_task_discovery.py":
                    staged["script"] = True
                elif source.name == "rrm":
                    staged["package"] = True
            elif command[:3] == ["docker", "exec", "-e"]:
                self.assertTrue(staged["script"] and staged["package"])
                self.assertIn("/tmp/rrm-airstack-task-discovery:$PYTHONPATH", command[-1])
                return type("Completed", (), {"stdout": json.dumps(report) + "\n"})()
            return type("Completed", (), {"stdout": ""})()

        with patch("rrm_command_console.subprocess.run", side_effect=clean_container), \
                patch.object(Console, "_clock_epoch_consistent", return_value=True):
            discovered = Console.__new__(Console).discover_tasks()

        self.assertTrue(discovered["clock_epoch_consistent"])
        self.assertEqual(discovered["task_servers"], {})
        self.assertEqual(len(calls), 5)
        self.assertEqual(calls[1][-3:], ["mkdir", "-p", "/tmp/rrm-airstack-task-discovery"])

    def test_robot_restart_restores_ephemeral_dependency_cache_before_discovery(self):
        calls = []

        def missing_then_restored(command, **_kwargs):
            calls.append(command)
            is_probe = command[:4] == [
                "docker", "exec", "airstack-robot-desktop-1", "env",
            ]
            if is_probe and len([item for item in calls if item[:4] == command[:4]]) == 1:
                return type("Completed", (), {"returncode": 1})()
            return type("Completed", (), {"returncode": 0})()

        with patch("rrm_command_console.subprocess.run", side_effect=missing_then_restored):
            Console._ensure_robot_rrm_dependencies()

        self.assertEqual(len(calls), 4)
        self.assertEqual(calls[1][-3:], ["mkdir", "-p", "/tmp/rrm-canonical-deps"])
        self.assertEqual(calls[2][:2], ["docker", "cp"])
        self.assertTrue(calls[2][3].endswith(":/tmp/rrm-canonical-deps/"))
        self.assertIn("import pydantic", calls[3][-1])


class IsaacSceneCatalogTests(unittest.TestCase):
    def test_catalog_accepts_leaf_and_scaled_usd_isaac_entries_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scenes.yaml"
            path.write_text("""scenes:
  office:
    isaac: Office
  custom:
    isaac:
      ref: omniverse://example/Custom.usd
      stage_scale: 0.01
  blocks:
    msairsim: blocks
""")
            self.assertEqual(isaac_scene_catalog(path), {
                "office": {"ref": "Office", "stage_scale": "1.0"},
                "custom": {"ref": "omniverse://example/Custom.usd", "stage_scale": "0.01"},
            })


class IsaacSceneSwitchTests(unittest.TestCase):
    def test_clock_epoch_rejects_robot_started_before_current_isaac(self):
        starts = {
            "airstack-robot-desktop-1": "2026-09-20T23:35:35+00:00\n",
            "isaac-sim-livestream": "2026-09-21T02:22:48+00:00\n",
        }

        def inspect(command, **_kwargs):
            container = command[-1]
            return type("Completed", (), {
                "returncode": 0 if container in starts else 1,
                "stdout": starts.get(container, ""),
            })()

        with patch("rrm_command_console.subprocess.run", side_effect=inspect):
            self.assertFalse(Console._clock_epoch_consistent())

    def test_switch_uses_selected_catalog_entry_and_persists_it(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "output"
            output.mkdir()
            app = object.__new__(Console)
            app.output = output
            app.isaac_scenes = {
                "custom": {"ref": "omniverse://example/Custom.usd", "stage_scale": "0.01"},
            }
            app.manifest_scene_shortname = "office"
            app.scene_switch_lock = threading.Lock()
            app.mission_lock = threading.Lock()
            app.mission_runtime = None
            app.latest_camera = object()
            app.latest_camera_metadata = object()
            with patch("rrm_command_console.subprocess.run") as run:
                result = app.switch_scene("custom")

            self.assertEqual(run.call_count, 2)
            first_environment = run.call_args_list[0].kwargs["env"]
            second_environment = run.call_args_list[1].kwargs["env"]
            self.assertEqual(first_environment["ISAAC_SIM_SCENE"], "omniverse://example/Custom.usd")
            self.assertEqual(second_environment["ISAAC_SIM_STAGE_SCALE"], "0.01")
            self.assertEqual(second_environment["COMPOSE_PROFILES"], "desktop,isaac-sim-livestream")
            self.assertEqual(result["scene"], "custom")
            self.assertFalse(result["rrm_live_enabled"])
            self.assertTrue(result["command_execution_enabled"])
            self.assertEqual(result["scene_context_status"], "COMMAND_ONLY")
            self.assertEqual(json.loads((output / "active_isaac_scene.json").read_text())["scene"], "custom")
            with self.assertRaisesRegex(RuntimeError, "matching RRM manifest"):
                app._require_scene_context()


class FakeProcess:
    def __init__(self):
        self.finished = threading.Event()
        self.return_code = 0
        self.cancel_acknowledged = False

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


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition was not reached before timeout")


def grounded_evidence(**changes):
    values = {
        "schema_version": "rrm-grounded-observation/v1",
        "observed_at": datetime.now(timezone.utc).isoformat(),
        "robot_name": "robot_1",
        "connected": True,
        "armed": False,
        "frame_id": "map",
        "child_frame_id": "base_link",
        "source_stamp_ns": 123,
        "odometry_samples": 3,
        "x": 0.0,
        "y": 0.0,
        "z": 0.02,
        "linear_speed_m_s": 0.01,
    }
    values.update(changes)
    return values


def live_observation(**changes):
    values = {
        "capture_mode": "read_only",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "source_stamp_ns": 456,
        "source_stamp_advanced": True,
        "frame_id": "camera_left",
        "sha256": "a" * 64,
        "vehicle": {
            "connected": True,
            "armed": False,
            "odometry_frame_id": "map",
            "odometry_child_frame_id": "base_link",
            "odometry_stamp_ns": 123,
            "x": 0.0,
            "y": 0.0,
            "z": 0.02,
            "linear_speed_m_s": 0.01,
        },
    }
    values.update(changes)
    return values


def seed_live_capture(app):
    app.latest_camera = b"live-image"
    app.latest_camera_metadata = live_observation(
        sha256=hashlib.sha256(app.latest_camera).hexdigest()
    )


def write_bound_psc_result(request_dir, bundle, fixture_bundle, actions):
    """Create a transport-only PSC result with exact saved request evidence."""
    bundle.mkdir()
    for name in ("input.json", "input.png"):
        shutil.copy(request_dir / name, bundle / name)
    shutil.copy(fixture_bundle / "scene_manifest.json", bundle / "scene_manifest.json")
    context = load_context(bundle / "input.json")
    raw = json.dumps({
        "status": "READY", "grounded_entities": ["blue_marker", "orange_marker"],
        "grounded_goal": {"name": "near", "subject": "$self", "obj": "blue_marker"},
        "ambiguity_refs": [], "explanation": "fresh synthetic result",
        "actions": actions, "recovery_budget": 0,
    })
    (bundle / "result.json").write_text(json.dumps({
        "raw_response": raw, "prompt": render_cosmos_prompt(context),
        "candidate": parse_cosmos_candidate(raw, context).model_dump(mode="json"),
        "input_sha256": hashlib.sha256((bundle / "input.json").read_bytes()).hexdigest(),
        "media_sha256": hashlib.sha256((bundle / "input.png").read_bytes()).hexdigest(),
        "execution_dispatch": False,
    }))


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
                process.cancel_acknowledged = True
                process.return_code = 130
                process.finished.set()

            return RunningDispatch(
                process=process,
                request_stop=stop,
                finalize=lambda code: {
                    "return_code": code,
                    "verdict": "UNCONFIRMED",
                    "cancel_acknowledged": process.cancel_acknowledged,
                },
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
        self.assertIn(status["state"], {"STOP_REQUESTED", "STOPPED_UNCONFIRMED"})
        self.assertEqual(status["safety_claim"], "SAFE_UNCONFIRMED")
        stop_record = next(Path(self.temp.name).glob("*/stop-1.json"))
        self.assertTrue(json.loads(stop_record.read_text())["cancel_requested"])

    def test_stop_reports_verified_only_from_physical_motion_evidence(self):
        def launch(*_):
            process = FakeProcess()

            def stop():
                process.return_code = 7
                process.finished.set()

            return RunningDispatch(
                process=process,
                request_stop=stop,
                finalize=lambda code: {
                    "return_code": code,
                    "verdict": "MOTION_STOPPED",
                    "cancel_acknowledged": True,
                    "physical_stop_verified": True,
                },
            )

        supervisor = ExecutionSupervisor(
            navigation_proposal(), Path(self.temp.name) / "verified-stop", launch
        )
        supervisor.decide("APPROVE", supervisor.proposal_sha256)
        supervisor.request_stop()
        wait_for(lambda: supervisor.status()["state"] == "STOPPED_VERIFIED")
        self.assertEqual(supervisor.status()["safety_claim"], "MOTION_STOP_VERIFIED")

    def test_land_completion_distinguishes_verified_from_unconfirmed(self):
        for verdict, expected_state, expected_claim in (
            ("VERIFIED", "LAND_VERIFIED", "GROUNDED_VERIFIED"),
            ("UNCONFIRMED", "LAND_FINISHED_UNCONFIRMED", "SAFE_UNCONFIRMED"),
        ):
            with self.subTest(verdict=verdict):
                root = Path(self.temp.name) / verdict.lower()

                def launch(*_):
                    process = FakeProcess()
                    return RunningDispatch(
                        process=process,
                        request_stop=lambda: None,
                        finalize=lambda code: {"return_code": code, "verdict": verdict},
                    )

                supervisor = ExecutionSupervisor(navigation_proposal(), root, launch)
                supervisor.request_land()
                supervisor.running.process.finished.set()
                wait_for(lambda: supervisor.status()["state"] == expected_state)
                self.assertEqual(supervisor.status()["safety_claim"], expected_claim)

    def test_land_now_launches_typed_public_land_action_when_idle(self):
        status = self.supervisor.request_land()
        self.assertEqual(status["state"], "LANDING")
        self.assertTrue(status["stop_latched"])
        self.assertEqual(status["active_command"], "LAND")
        self.assertEqual(len(self.launches), 1)
        _, run_dir, proposal_path, process = self.launches[0]
        proposal = json.loads(proposal_path.read_text())
        self.assertEqual(proposal["kind"], "LAND")
        self.assertEqual(proposal["velocity_m_s"], 1.0)
        record = json.loads((run_dir / "operator-land.json").read_text())
        self.assertTrue(record["normal_admission_blocked"])
        self.assertTrue(record["execution_requested"])
        duplicate = self.supervisor.request_land()
        self.assertEqual(duplicate["dispatch_id"], status["dispatch_id"])
        self.assertEqual(len(self.launches), 1)
        process.finished.set()

    def test_land_now_cancels_active_command_before_launching_land(self):
        self.supervisor.decide("APPROVE", self.supervisor.proposal_sha256)
        status = self.supervisor.request_land()
        self.assertIn(status["state"], {"LAND_CANCELING_ACTIVE", "LANDING"})
        self.assertEqual(self.stop_calls, 1)
        wait_for(lambda: len(self.launches) == 2)
        proposal = json.loads(self.launches[1][2].read_text())
        self.assertEqual(proposal["kind"], "LAND")
        self.assertEqual(self.supervisor.status()["active_command"], "LAND")
        self.launches[1][3].finished.set()

    def test_land_is_blocked_when_active_cancel_is_not_acknowledged(self):
        launches = []

        def launch(dispatch_id, run_dir, proposal_path):
            process = FakeProcess()
            launches.append((dispatch_id, run_dir, proposal_path, process))

            def stop():
                process.return_code = 130
                process.finished.set()

            return RunningDispatch(
                process=process, request_stop=stop,
                finalize=lambda code: {
                    "return_code": code, "verdict": "UNCONFIRMED",
                    "cancel_acknowledged": False,
                },
            )

        supervisor = ExecutionSupervisor(navigation_proposal(), Path(self.temp.name) / "no-ack", launch)
        supervisor.decide("APPROVE", supervisor.proposal_sha256)
        supervisor.request_land()
        wait_for(lambda: supervisor.status()["state"] == "LAND_BLOCKED_UNCONFIRMED")
        self.assertEqual(len(launches), 1)
        self.assertFalse(supervisor.status()["active"])

    def test_stop_hold_can_cancel_active_landing(self):
        self.supervisor.request_land()
        status = self.supervisor.request_stop()
        self.assertEqual(self.stop_calls, 1)
        self.assertIn(status["state"], {"STOP_REQUESTED", "STOPPED_UNCONFIRMED"})
        wait_for(lambda: self.supervisor.status()["state"] == "STOPPED_UNCONFIRMED")

    def test_grounded_reconciliation_persists_and_reopens_exact_plan(self):
        self.supervisor.request_stop()
        status = self.supervisor.reconcile_grounded(grounded_evidence())
        self.assertEqual(status["state"], "READY_FOR_APPROVAL")
        self.assertFalse(status["stop_latched"])
        self.assertIsNotNone(status["reconciliation_id"])
        record_path = next(Path(self.temp.name).glob("reconciliations/*.json"))
        record = json.loads(record_path.read_text())
        self.assertTrue(record["normal_admission_reopened"])
        self.assertFalse(record["evidence"]["armed"])

    def test_reconciliation_rejects_unproven_ground_state(self):
        bad_cases = (
            {"connected": False},
            {"armed": True},
            {"z": 0.31},
            {"linear_speed_m_s": 0.11},
            {"frame_id": "odom"},
            {"odometry_samples": 2},
            {"observed_at": "not-a-time"},
        )
        for index, changes in enumerate(bad_cases):
            root = Path(self.temp.name) / f"bad-{index}"
            supervisor = ExecutionSupervisor(navigation_proposal(), root, lambda *_: None)
            supervisor.request_stop()
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, "Cannot"):
                supervisor.reconcile_grounded(grounded_evidence(**changes))
            self.assertTrue(supervisor.status()["stop_latched"])


class CommandConsoleTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_office_import.OfficeImportTests("test_actual_plan_ids_survive_import")
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.record["execution_dispatch"] = False
        self.fixture.save()
        self.bundle = self.fixture.bundle
        self.output = self.bundle / "requests"

    def test_live_only_mode_needs_no_historical_psc_bundle(self):
        office = Path(__file__).parents[1] / "examples" / "office_visual_eval"
        app = Console(None, self.output, "/unused-capture.py",
                      context_template=office / "navigation_context.json",
                      scene_manifest=office / "scene_manifest.json")
        self.assertIsNone(app.decision)
        self.assertIsNone(app.execution)
        self.assertEqual(app.store.history(), [])
        seed_live_capture(app)
        saved = app.save("Approach the blue marker from the current scene.")
        self.assertEqual(saved["context_mode"], "live-isaac-observation")
        self.assertEqual(len(app.store.history()), 1)

    def test_movement_command_can_be_saved_without_scene_specific_camera(self):
        office = Path(__file__).parents[1] / "examples" / "office_visual_eval"
        app = Console(None, self.output, "/unused-capture.py",
                      context_template=office / "navigation_context.json",
                      scene_manifest=office / "scene_manifest.json")
        saved = app.save("Take off and explore for 30 seconds, then land.")
        self.assertEqual(saved["context_mode"], "command-only")
        self.assertFalse((self.output / saved["request_id"] / "observation.json").exists())
        payload = json.loads((self.output / saved["request_id"] / "input.json").read_text())
        self.assertEqual(payload["task"]["constraints_revision"], "airstack-live-command-v1")
        self.assertEqual(payload["snapshot"]["evidence"], [])
        self.assertNotIn("office", payload["task"]["task_id"])

    def test_request_key_retry_returns_one_durable_run_after_restart(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        seed_live_capture(app)
        request_key = secrets.token_hex(16)
        first = app.save("Take off to one meter and land.", request_key=request_key)
        retry = app.save("Take off to one meter and land.", request_key=request_key)
        self.assertEqual(first["request_id"], request_key)
        self.assertEqual(retry["request_id"], request_key)
        self.assertTrue(retry["idempotent_replay"])
        restarted = Console(self.bundle, self.output, "/unused-capture.py")
        after_restart = restarted.save(
            "Take off to one meter and land.", request_key=request_key,
        )
        self.assertTrue(after_restart["idempotent_replay"])
        runs = [run for goal in restarted.store.history() for run in goal["runs"]
                if run["run_id"] == request_key]
        self.assertEqual(len(runs), 1)
        with self.assertRaisesRegex(ValueError, "different goal evidence"):
            restarted.save("A different command.", request_key=request_key)

    def test_response_disconnect_is_not_reclassified_as_storage_failure(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        handler = object.__new__(make_handler(app))
        messages = []
        handler.send_response = lambda _status: (_ for _ in ()).throw(BrokenPipeError("closed"))
        handler.log_error = lambda message, *args: messages.append(message % args)
        handler.respond({"status": "durable"}, status=201)
        self.assertEqual(messages, ["response transport closed: closed"])

    def test_hand_goal_preview_persists_neutral_route_and_shadow_plan_without_motion(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        result = app.preview_hand_goal(
            "place the context-selected block on the tray", "red_block",
        )
        self.assertEqual(result["route"]["status"], "SELECTED")
        self.assertEqual(result["decision"]["status"], "PROPOSED")
        self.assertEqual(
            [node["action"]["verb"] for node in result["plan"]["actions"]],
            ["GRASP", "PLACE"],
        )
        self.assertFalse(result["numeric_feasibility_verified"])
        self.assertFalse(result["execution_dispatch"])
        self.assertFalse(result["simulator_action_sent"])
        artifact_dir = Path(result["artifact_dir"])
        manifest = json.loads((artifact_dir / "manifest.json").read_text())
        self.assertEqual(manifest["status"], "PROPOSED")
        for name, digest in manifest["records_sha256"].items():
            self.assertEqual(
                hashlib.sha256((artifact_dir / f"{name}.json").read_bytes()).hexdigest(),
                digest,
            )

    def test_hand_goal_preview_fails_closed_on_unknown_goal_target_or_evidence(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        with self.assertRaisesRegex(ValueError, "supports exactly"):
            app.preview_hand_goal("grasp anything", "red_block")
        with self.assertRaisesRegex(ValueError, "declared"):
            app.preview_hand_goal(
                "place the context-selected block on the tray", "unknown_block",
            )
        held = app.preview_hand_goal(
            "place the context-selected block on the tray", "blue_block",
        )
        self.assertEqual(held["decision"]["status"], "HOLD")
        self.assertIsNone(held["plan"])
        self.assertFalse(held["execution_dispatch"])

    def test_command_mission_compiles_against_discovered_public_tasks(self):
        office = Path(__file__).parents[1] / "examples" / "office_visual_eval"
        app = Console(None, self.output, "/unused-capture.py",
                      context_template=office / "navigation_context.json",
                      scene_manifest=office / "scene_manifest.json")
        saved = app.save("Take off and explore for 30 seconds, then land.")
        discovery = {
            "schema_version": "airstack-task-discovery/v1",
            "task_servers": {
                "/robot_1/tasks/takeoff": ["task_msgs/action/TakeoffTask"],
                "/robot_1/tasks/exploration": ["task_msgs/action/ExplorationTask"],
                "/robot_1/tasks/land": ["task_msgs/action/LandTask"],
            },
            "missing_state": [], "stale_state": [], "connected": True,
            "airborne": False, "frame_id": "map", "child_frame_id": "base_link",
            "position": {"x": 0.0, "y": 0.0, "z": 0.0}, "yaw_rad": 0.0,
            "vdb_map_fresh": True, "vdb_map_frame_id": "map",
            "vdb_map_point_count": 100,
            "vdb_map_bounds": {"min_x": -5.0, "max_x": 5.0,
                               "min_y": -4.0, "max_y": 4.0,
                               "min_z": 0.0, "max_z": 3.0},
            "execution_dispatch": False,
        }
        process = FakeProcess()
        with patch.object(app, "discover_tasks", return_value=discovery), \
                patch("rrm_command_console.subprocess.run"), \
                patch("rrm_command_console.subprocess.Popen", return_value=process):
            status = app.start_command_mission(saved["request_id"])
        self.addCleanup(app.mission_runtime["log_handle"].close)
        self.assertTrue(status["active"])
        self.assertEqual([item["kind"] for item in status["plan"]["actions"]],
                         ["TAKEOFF", "EXPLORE", "LAND"])
        self.assertEqual(status["plan"]["recovery"]["trigger"],
                         "verified_mission_halt_while_airborne")
        self.assertEqual(status["plan"]["recovery"]["action"]["kind"], "LAND")
        plan = json.loads((self.output / saved["request_id"] / "command-plan.json").read_text())
        self.assertTrue(plan["execution_dispatch"])
        self.assertEqual(plan["discovery"]["task_servers"], discovery["task_servers"])
        self.assertEqual(plan["grounding"]["schema_version"], "rrm-grounded-command/v1")
        self.assertTrue(plan["grounding"]["parameter_grounding"])
        self.assertEqual(plan["replan_policy"]["mode"], "observe_between_actions")
        self.assertFalse(plan["replan_policy"]["blind_retry"])
        self.assertEqual(app.store.get_run(saved["request_id"])["execution_state"],
                         "DISPATCHING")
        events = next(run for goal in app.store.history() for run in goal["runs"]
                      if run["run_id"] == saved["request_id"])["events"]
        self.assertEqual(events[-1]["kind"], "command_plan")

    def test_exploration_is_not_dispatched_without_fresh_vdb_map(self):
        office = Path(__file__).parents[1] / "examples" / "office_visual_eval"
        app = Console(None, self.output, "/unused-capture.py",
                      context_template=office / "navigation_context.json",
                      scene_manifest=office / "scene_manifest.json")
        saved = app.save("Explore for 30 seconds.")
        discovery = {
            "schema_version": "airstack-task-discovery/v1",
            "task_servers": {
                "/robot_1/tasks/takeoff": ["task_msgs/action/TakeoffTask"],
                "/robot_1/tasks/exploration": ["task_msgs/action/ExplorationTask"],
                "/robot_1/tasks/land": ["task_msgs/action/LandTask"],
            },
            "missing_state": [], "stale_state": [], "connected": True,
            "airborne": False, "frame_id": "map", "child_frame_id": "base_link",
            "position": {"x": 0.0, "y": 0.0, "z": 0.0}, "yaw_rad": 0.0,
            "vdb_map_fresh": False, "vdb_map_frame_id": None,
            "execution_dispatch": False,
        }
        with patch.object(app, "discover_tasks", return_value=discovery), \
                self.assertRaisesRegex(RuntimeError, "VDB"):
            app.start_command_mission(saved["request_id"])
        self.assertFalse((self.output / saved["request_id"] / "command-plan.json").exists())

    def test_contradictory_airborne_state_allows_only_reconciliation(self):
        office = Path(__file__).parents[1] / "examples" / "office_visual_eval"
        app = Console(None, self.output, "/unused-capture.py",
                      context_template=office / "navigation_context.json",
                      scene_manifest=office / "scene_manifest.json")
        saved = app.save("Explore briefly, then land.")
        discovery = {
            "schema_version": "airstack-task-discovery/v1",
            "task_servers": {
                "/robot_1/tasks/exploration": ["task_msgs/action/ExplorationTask"],
                "/robot_1/tasks/land": ["task_msgs/action/LandTask"],
            },
            "missing_state": [], "stale_state": [], "connected": True,
            "armed": True, "airborne": True,
            "frame_id": "map", "child_frame_id": "base_link",
            "position": {"x": 0.6, "y": 1.2, "z": 0.015}, "yaw_rad": 0.0,
            "flight_state_consistent": False,
            "flight_state_reasons": ["airborne_flag_below_0.3m_map_altitude"],
            "vdb_map_fresh": True, "vdb_map_frame_id": "map",
            "execution_dispatch": False,
        }
        with patch.object(app, "discover_tasks", return_value=discovery), \
                self.assertRaisesRegex(RuntimeError, "Only an explicit landing"):
            app.start_command_mission(saved["request_id"])
        self.assertFalse((self.output / saved["request_id"] / "command-plan.json").exists())

    def test_goal_reuse_and_restart_preserve_independent_runs(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        seed_live_capture(app)
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
        seed_live_capture(app)
        original = app.save("Blue marker only.")
        before = sorted(self.output.glob("*/request.json"))
        with self.assertRaisesRegex(ValueError, "new goal"):
            app.save("Orange marker instead.", original["goal_id"])
        self.assertEqual(before, sorted(self.output.glob("*/request.json")))
        self.assertEqual(app.store.get_goal(original["goal_id"])["objective"], "Blue marker only.")

    def test_live_observation_refuses_stale_paused_or_wrong_frame_before_save(self):
        cases = (
            {"source_stamp_advanced": False},
            {"frame_id": "other_camera"},
            {"captured_at": "2000-01-01T00:00:00+00:00"},
            {"vehicle": {"connected": False}},
        )
        for changes in cases:
            with self.subTest(changes=changes):
                app = Console(self.bundle, self.output / secrets.token_hex(2), "/unused-capture.py")
                seed_live_capture(app)
                if "vehicle" in changes:
                    app.latest_camera_metadata["vehicle"].update(changes["vehicle"])
                else:
                    app.latest_camera_metadata.update(changes)
                with self.assertRaises(ValueError):
                    app.save("Approach the blue marker.")

    def test_live_worker_cycle_requires_reviewed_verified_outcome_before_replan(self):
        class FakeWorkerClient:
            requests = []

            def __init__(self, url):
                self.url = url

            def propose(self, request):
                self.requests.append(request)
                raw = json.dumps({
                    "status": "READY", "grounded_entities": ["blue_marker", "orange_marker"],
                    "grounded_goal": {"name": "near", "subject": "$self", "obj": "blue_marker"},
                    "ambiguity_refs": [], "explanation": "shadow-only live candidate",
                    "actions": [
                        {"id": "blue", "verb": "NAVIGATE_TO", "targets": ["blue_marker"], "dependencies": []},
                        {"id": "orange", "verb": "NAVIGATE_TO", "targets": ["orange_marker"], "dependencies": ["blue"]},
                    ], "recovery_budget": 0,
                })
                return LiveCycleResponse(cycle_id=request.cycle_id, step_index=request.step_index,
                    observation_sha256=request.observation["sha256"],
                    candidate=parse_cosmos_candidate(raw, request.context))

        app = Console(self.bundle, self.output, "/unused-capture.py")
        app.cosmos_worker_url = "http://cosmos-worker:8090"
        # This fixture is explicitly the Office scene; model a completed GUI
        # selection rather than relying on an unknown initial simulator state.
        app._save_active_scene("office")
        app.active_scene_shortname = "office"
        app.scene_context_matches = True
        seed_live_capture(app)
        saved = app.save("Approach the blue marker.")
        scene = {"source_stamp_ns": app.latest_camera_metadata["source_stamp_ns"],
                 "verified_entities": ["blue_marker", "orange_marker"],
                 "provenance": "test-entity-verifier/v1"}
        with patch("rrm_command_console.CosmosWorkerClient", FakeWorkerClient):
            first = app.start_live_replan(saved["request_id"], scene)
            self.assertEqual(first["state"], "REVIEW_REQUIRED")
            self.assertEqual(first["next_action"]["action"]["id"], "blue")
            with self.assertRaisesRegex(ValueError, "verified outcome"):
                app.replan_live(saved["request_id"], scene)
            app.review_live_action(saved["request_id"], "blue")
            app.record_live_outcome(saved["request_id"], "blue", verified=True,
                                    detail="operator recorded independently verified shadow outcome")
            app.latest_camera_metadata["source_stamp_ns"] += 1
            app.latest_camera_metadata["vehicle"]["odometry_stamp_ns"] += 1
            second = app.replan_live(saved["request_id"], {
                **scene, "source_stamp_ns": app.latest_camera_metadata["source_stamp_ns"],
            })
        self.assertEqual(second["state"], "REVIEW_REQUIRED")
        self.assertEqual(FakeWorkerClient.requests[1].prior_outcome["action_id"], "blue")
        self.assertFalse((self.output / "execution").exists())

    def test_gui_proposal_uses_worker_entity_evidence_and_remains_motion_inhibited(self):
        class FakeWorkerClient:
            def __init__(self, url):
                self.url = url

            def propose(self, request):
                raw = json.dumps({
                    "status": "READY", "grounded_entities": ["blue_marker"],
                    "grounded_goal": {"name": "near", "subject": "$self", "obj": "blue_marker"},
                    "ambiguity_refs": [], "explanation": "grounded live candidate",
                    "actions": [{"id": "blue", "verb": "NAVIGATE_TO",
                                 "targets": ["blue_marker"], "dependencies": []}],
                    "recovery_budget": 0,
                })
                return LiveCycleResponse(cycle_id=request.cycle_id, step_index=request.step_index,
                    observation_sha256=request.observation["sha256"],
                    candidate=parse_cosmos_candidate(raw, request.context))

        class FakeEntityVerifier:
            calls = []

            def __init__(self, url):
                self.url = url

            def verify(self, **values):
                self.calls.append(values)
                return ({
                    "source_stamp_ns": values["metadata"]["source_stamp_ns"],
                    "observation_sha256": values["metadata"]["sha256"],
                    "verified_entities": ["blue_marker"],
                    "provenance": "test-physics-independent-entity-verifier/v1",
                }, values["context"])

        app = Console(self.bundle, self.output, "/unused-capture.py")
        app.cosmos_worker_url = "http://cosmos-worker:8090"
        app._save_active_scene("office")
        app.active_scene_shortname = "office"
        app.scene_context_matches = True
        seed_live_capture(app)
        saved = app.save("Approach the blue marker.")
        with patch("rrm_command_console.CosmosEntityVerifierClient", FakeEntityVerifier), \
                patch("rrm_command_console.CosmosWorkerClient", FakeWorkerClient):
            result = app.propose_live_goal(saved["request_id"])
        self.assertEqual(result["state"], "REVIEW_REQUIRED")
        self.assertEqual(result["next_action"]["action"]["targets"], ["blue_marker"])
        self.assertEqual(FakeEntityVerifier.calls[0]["entity_catalog"]["blue_marker"], "blue navigation marker")
        evidence = self.output / saved["request_id"] / "live-cycle" / "steps" / "0000"
        self.assertTrue((evidence / "provider-response.json").is_file())
        self.assertFalse((self.output / "execution").exists())

    def test_async_psc_result_is_run_bound_validated_and_requires_approval(self):
        fixture_bundle = self.bundle

        class FakeBridge:
            def run(self, request_dir):
                bundle = request_dir.parent / "fake-psc-result"
                bundle.mkdir()
                shutil.copy(request_dir / "input.json", bundle / "input.json")
                shutil.copy(request_dir / "input.png", bundle / "input.png")
                shutil.copy(fixture_bundle / "scene_manifest.json", bundle / "scene_manifest.json")
                context = load_context(bundle / "input.json")
                raw = json.dumps({
                    "status": "READY", "grounded_entities": ["blue_marker"],
                    "grounded_goal": {"name": "near", "subject": "$self", "obj": "blue_marker"},
                    "ambiguity_refs": [], "explanation": "fresh synthetic result",
                    "actions": [{"id": "fresh-nav", "verb": "NAVIGATE_TO",
                                 "targets": ["blue_marker"], "dependencies": []}],
                    "recovery_budget": 0,
                })
                (bundle / "result.json").write_text(json.dumps({
                    "raw_response": raw, "prompt": render_cosmos_prompt(context),
                    "candidate": parse_cosmos_candidate(raw, context).model_dump(mode="json"),
                    "input_sha256": hashlib.sha256((bundle / "input.json").read_bytes()).hexdigest(),
                    "media_sha256": hashlib.sha256((bundle / "input.png").read_bytes()).hexdigest(),
                    "execution_dispatch": False,
                }))
                return {"job_id": "999", "bundle_dir": str(bundle)}

        app = Console(self.bundle, self.output, "/unused-capture.py", psc_bridge=FakeBridge())
        seed_live_capture(app)
        saved = app.save("Approach the blue marker.")
        app.submit_to_psc(saved["request_id"])
        wait_for(lambda: app.store.get_run(saved["request_id"])["status"] == "CANDIDATE_ACCEPTED")
        run = app.store.get_run(saved["request_id"])
        self.assertEqual(run["execution_state"], "REVIEW_REQUIRED")
        self.assertEqual(run["psc_job_id"], "999")
        with self.assertRaisesRegex(ValueError, "changed"):
            app.approve_candidate(saved["request_id"], "0" * 64)
        approved = app.approve_candidate(saved["request_id"], run["proposal_sha256"])
        self.assertEqual(approved["execution_state"], "APPROVED")
        events = next(run for goal in app.store.history() for run in goal["runs"]
                      if run["run_id"] == saved["request_id"])["events"]
        self.assertEqual({event["kind"] for event in events},
                         {"live_observation", "psc_submission", "psc_receipt", "psc_result",
                          "proposal", "candidate_approval"})
        self.assertFalse((self.output / "execution").exists())

    def test_async_psc_mismatched_result_fails_without_candidate_or_dispatch(self):
        fixture_bundle = self.bundle

        class BadBridge:
            def run(self, request_dir):
                bundle = request_dir.parent / "bad-psc-result"
                bundle.mkdir()
                for name in ("input.json", "input.png"):
                    shutil.copy(request_dir / name, bundle / name)
                shutil.copy(fixture_bundle / "scene_manifest.json", bundle / "scene_manifest.json")
                (bundle / "input.png").write_bytes(b"substituted-image")
                (bundle / "result.json").write_text("{}")
                return {"job_id": "998", "bundle_dir": str(bundle)}

        app = Console(self.bundle, self.output, "/unused-capture.py", psc_bridge=BadBridge())
        seed_live_capture(app)
        saved = app.save("Approach the blue marker.")
        app.submit_to_psc(saved["request_id"])
        wait_for(lambda: app.store.get_run(saved["request_id"])["status"] == "INFERENCE_FAILED")
        run = app.store.get_run(saved["request_id"])
        self.assertEqual(run["execution_state"], "NOT_DISPATCHED")
        events = next(run for goal in app.store.history() for run in goal["runs"]
                      if run["run_id"] == saved["request_id"])["events"]
        self.assertEqual(events[-1]["kind"], "psc_failure")
        self.assertFalse((self.output / "execution").exists())

    def test_async_psc_multi_action_result_is_rejected_before_retention(self):
        fixture_bundle = self.bundle

        class MultiActionBridge:
            def run(self, request_dir):
                bundle = request_dir.parent / "multi-action-psc-result"
                write_bound_psc_result(request_dir, bundle, fixture_bundle, [
                    {"id": "blue", "verb": "NAVIGATE_TO", "targets": ["blue_marker"],
                     "dependencies": []},
                    {"id": "orange", "verb": "NAVIGATE_TO", "targets": ["orange_marker"],
                     "dependencies": ["blue"]},
                ])
                return {"job_id": "997", "bundle_dir": str(bundle)}

        app = Console(self.bundle, self.output, "/unused-capture.py", psc_bridge=MultiActionBridge())
        seed_live_capture(app)
        saved = app.save("Approach the blue marker.")
        app.submit_to_psc(saved["request_id"])
        wait_for(lambda: app.store.get_run(saved["request_id"])["status"] == "CANDIDATE_REJECTED")
        request_dir = self.output / saved["request_id"]
        self.assertFalse((request_dir / "psc-result").exists())
        run = app.store.get_run(saved["request_id"])
        self.assertEqual(run["execution_state"], "NOT_DISPATCHED")
        events = next(run for goal in app.store.history() for run in goal["runs"]
                      if run["run_id"] == saved["request_id"])["events"]
        rejection = next(event for event in events if event["kind"] == "psc_rejected")
        self.assertEqual(rejection["summary"]["reason"], "adapter_refused_plan")
        self.assertFalse((self.output / "execution").exists())

    def test_manual_import_recovers_old_partial_copy_as_terminal_rejection(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        seed_live_capture(app)
        saved = app.save("Approach the blue marker.")
        request_dir = self.output / saved["request_id"]
        source = request_dir.parent / "manual-multi-action-result"
        write_bound_psc_result(request_dir, source, self.bundle, [
            {"id": "blue", "verb": "NAVIGATE_TO", "targets": ["blue_marker"],
             "dependencies": []},
            {"id": "orange", "verb": "NAVIGATE_TO", "targets": ["orange_marker"],
             "dependencies": ["blue"]},
        ])
        shutil.copytree(source, request_dir / "psc-result")
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        endpoint = f"http://127.0.0.1:{server.server_port}/api/requests/manual-import"
        payload = json.dumps({"run_id": saved["request_id"], "job_id": "996",
                              "bundle_dir": str(source)}).encode()
        with urlopen(Request(endpoint, data=payload,
                             headers={"Content-Type": "application/json"})) as response:
            self.assertEqual(json.load(response)["status"], "CANDIDATE_REJECTED")
        self.assertEqual(app.store.get_run(saved["request_id"])["status"], "CANDIDATE_REJECTED")
        self.assertFalse((request_dir / "psc-result" / "proposal.json").exists())
        with urlopen(Request(endpoint, data=payload,
                             headers={"Content-Type": "application/json"})) as response:
            self.assertEqual(json.load(response)["status"], "already_rejected")

    def test_manual_submission_is_idempotent_and_cannot_rebind_a_job(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        seed_live_capture(app)
        saved = app.save("Approach the blue marker.")
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        endpoint = f"http://127.0.0.1:{server.server_port}/api/requests/manual-submitted"

        def submit(job_id):
            return Request(endpoint, data=json.dumps({"run_id": saved["request_id"],
                                                       "job_id": job_id}).encode(),
                           headers={"Content-Type": "application/json"})

        with urlopen(submit("995")) as response:
            self.assertEqual(json.load(response)["status"], "recorded")
        with urlopen(submit("995")) as response:
            self.assertEqual(json.load(response)["status"], "already_recorded")
        with self.assertRaises(HTTPError) as error:
            urlopen(submit("994"))
        self.assertEqual(error.exception.code, 409)
        run = app.store.get_run(saved["request_id"])
        self.assertEqual((run["status"], run["psc_job_id"]), ("INFERENCE_RUNNING", "995"))

    def test_reference_execution_evidence_is_visible_and_allow_listed(self):
        app = Console(self.bundle, self.output, "/unused-capture.py")
        app.execution.decide("REJECT", app.execution.proposal_sha256)
        app.index_execution_evidence()
        reference = next(run for goal in app.store.history() for run in goal["runs"]
                         if run["run_id"] == app.reference_run_id)
        admission = next(event for event in reference["events"] if event["kind"] == "admission")
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with urlopen(f"http://127.0.0.1:{server.server_port}/runs/{app.reference_run_id}/evidence/{admission['event_id']}") as response:
            self.assertEqual(json.load(response)["decision"], "REJECT")

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
        seed_live_capture(app)
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
        preview_data = json.dumps({
            "objective": "place the context-selected block on the tray",
            "selected_entity_id": "red_block",
        }).encode()
        with urlopen(Request(
            base + "/api/goal-previews/hand", data=preview_data,
            headers={"X-RRM-Token": state["token"]},
        )) as response:
            preview = json.load(response)
        self.assertEqual(preview["decision"]["status"], "PROPOSED")
        self.assertFalse(preview["execution_dispatch"])
        self.assertFalse(preview["simulator_action_sent"])
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
        land_calls = []
        app.execution.request_land = lambda: land_calls.append(True) or {"state": "LANDING"}
        with urlopen(Request(base + "/api/land", data=b"{}",
                            headers={"X-RRM-Token": state["token"]})) as response:
            self.assertEqual(json.load(response)["state"], "LANDING")
        self.assertEqual(land_calls, [True])
        reconcile_calls = []
        app.reconcile_grounded = lambda: reconcile_calls.append(True) or {
            "state": "READY_FOR_APPROVAL"
        }
        with urlopen(Request(base + "/api/reconcile", data=b"{}",
                            headers={"X-RRM-Token": state["token"]})) as response:
            self.assertEqual(json.load(response)["state"], "READY_FOR_APPROVAL")
        self.assertEqual(reconcile_calls, [True])
        with urlopen(base + f"/runs/{saved['request_id']}/request.json") as response:
            self.assertEqual(json.load(response)["goal_id"], saved["goal_id"])
        with self.assertRaises(HTTPError) as error:
            urlopen(base + "/requests/../result.json")
        self.assertEqual(error.exception.code, 404)

    def test_dispatch_launcher_preserves_ros_python_path_and_uses_measured_state_rate(self):
        source = (Path(__file__).parents[1] / "scripts" / "rrm_command_console.py").read_text()
        self.assertIn(":$PYTHONPATH", source)
        self.assertIn('"--observation-timeout-s 10 --max-observation-age-s 2 "', source)

    def test_grounded_observer_has_no_command_surface(self):
        source = (Path(__file__).parents[1] / "scripts" / "airstack_vehicle_observe.py").read_text()
        self.assertIn("create_subscription(", source)
        for prohibited in ("ActionClient", "create_publisher(", "create_client("):
            self.assertNotIn(prohibited, source)


if __name__ == "__main__":
    unittest.main()
