import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from rrm.isaac_runtime import probe_isaac_runtime
from rrm.airstack_drone import DroneTaskProposal
from rrm_command_console import Console, make_handler


def completed(stdout, code=0):
    return subprocess.CompletedProcess([], code, stdout, "")


class RuntimeProbeTests(unittest.TestCase):
    def probe(self, record):
        with patch("rrm.isaac_runtime.subprocess.run", side_effect=[
            completed("true"), completed("", 1), completed(record)
        ]) as run:
            result = probe_isaac_runtime()
        self.assertEqual(run.call_args.args[0][3], "/isaac-sim/python.sh")
        self.assertEqual(run.call_args.kwargs["timeout"], 15)
        self.assertFalse(result["execution_dispatch"])
        return result

    def test_exact_image_pin_passes(self):
        self.assertTrue(self.probe('startup\n{"numpy_version":"1.26.4"}\n')["compatible"])

    def test_unqualified_versions_fail_closed(self):
        for version in ("2.4.6", "3.0.0", "1.26.3", "garbage"):
            with self.subTest(version=version):
                self.assertEqual(self.probe(json.dumps({"numpy_version": version}))["status"],
                                 "INCOMPATIBLE")

    def test_invalid_records_fail_closed(self):
        for record in ('', '{}', '{bad', '{"numpy_version":true}',
                       '{"numpy_version":null}', '{"numpy_version":""}',
                       '{"numpy_version":"1.26.4"}\n{"numpy_version":"1.26.4"}'):
            with self.subTest(record=record):
                self.assertEqual(self.probe(record)["status"], "UNAVAILABLE")

    def test_missing_and_ambiguous_containers_fail_closed(self):
        for outputs in ((completed("", 1), completed("false")),
                        (completed("true"), completed("true"))):
            with self.subTest(outputs=outputs), patch(
                "rrm.isaac_runtime.subprocess.run", side_effect=outputs
            ) as run:
                self.assertFalse(probe_isaac_runtime()["compatible"])
                self.assertEqual(run.call_count, 2)

    def test_base_container_is_supported(self):
        with patch("rrm.isaac_runtime.subprocess.run", side_effect=[
            completed("", 1), completed("true"), completed('{"numpy_version":"1.26.4"}')
        ]):
            self.assertEqual(probe_isaac_runtime()["container"], "isaac-sim")

    def test_invalid_inspect_evidence_blocks_other_container(self):
        with patch("rrm.isaac_runtime.subprocess.run", side_effect=[completed("garbage"),
                                                                  completed("true")]) as run:
            self.assertEqual(probe_isaac_runtime()["status"], "UNAVAILABLE")
            self.assertEqual(run.call_count, 1)

    def test_execution_errors_fail_closed_without_leaking_output(self):
        for error in (OSError("private"), subprocess.TimeoutExpired("probe", 15),
                      subprocess.CalledProcessError(1, "probe", stderr="private")):
            with self.subTest(error=error), patch(
                "rrm.isaac_runtime.subprocess.run", side_effect=[completed("true"),
                                                               completed("", 1), error]
            ):
                result = probe_isaac_runtime()
                self.assertEqual(result["status"], "UNAVAILABLE")
                self.assertNotIn("private", json.dumps(result))


class RuntimeAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)
        office = Path(__file__).parents[1] / "examples/office_visual_eval"
        self.app = Console(None, self.output, "/unused-capture.py",
                           context_template=office / "navigation_context.json",
                           scene_manifest=office / "scene_manifest.json")
        self.discovery = {
            "task_servers": {"takeoff": ["task_msgs/action/TakeoffTask"],
                             "land": ["task_msgs/action/LandTask"]},
            "missing_state": [], "stale_state": [], "connected": True,
            "airborne": True, "frame_id": "map", "child_frame_id": "base_link",
            "position": {"x": 0., "y": 0., "z": 1.}, "yaw_rad": 0.,
            "clock_epoch_consistent": True,
        }

    def test_command_blocks_before_mission_plan_or_staging(self):
        saved = self.app.save("Take off then land.")
        self.discovery["airborne"] = False
        with patch.object(self.app, "discover_tasks", return_value=self.discovery), \
             patch("rrm_command_console.probe_isaac_runtime",
                   return_value={"compatible": False, "reason": "bad runtime"}), \
             patch("rrm_command_console.subprocess.run") as run, \
             patch("rrm_command_console.subprocess.Popen") as launch, \
             self.assertRaisesRegex(RuntimeError, "bad runtime"):
            self.app.stage_command_mission(saved["request_id"])
        run.assert_not_called()
        launch.assert_not_called()
        self.assertFalse((self.output / saved["request_id"] / "command-plan.json").exists())
        self.assertEqual(self.app.store.get_run(saved["request_id"])["execution_state"],
                         "NOT_DISPATCHED")

    def test_land_command_launches_without_dependency_probe(self):
        saved = self.app.save("Land.")
        self.discovery['task_servers'] = {'/robot_1/tasks/land':['task_msgs/action/LandTask']}
        with patch.object(self.app, "discover_tasks", return_value=self.discovery), \
             patch("rrm_command_console.probe_isaac_runtime") as probe, \
             patch.object(self.app, "_command_runtime_identity", return_value=['identity']), \
             patch.object(self.app, "_launch_staged_command", return_value={'active':False}):
            staged = self.app.stage_command_mission(saved["request_id"])
            self.app.start_command_mission(saved["request_id"], staged['plan_sha256'])
        probe.assert_not_called()
        plan = json.loads((self.output / saved["request_id"] / "command-plan.json").read_text())
        self.assertEqual(plan["isaac_runtime"]["status"], "LAND_EXEMPT")

    def test_model_adapter_blocks_before_dependency_mutation(self):
        proposal = DroneTaskProposal(task_id="t", action_id="a", kind="TAKEOFF",
                                     target_altitude_m=1, velocity_m_s=.5)
        path = self.output / "proposal.json"
        path.write_text(proposal.model_dump_json())
        with patch("rrm_command_console.probe_isaac_runtime",
                   return_value={"compatible": False, "reason": "bad runtime"}), \
             patch.object(self.app, "_ensure_robot_rrm_dependencies") as deps, \
             patch("rrm_command_console.subprocess.Popen") as launch, \
             self.assertRaisesRegex(RuntimeError, "bad runtime"):
            self.app._launch_dispatch("id", self.output, path)
        deps.assert_not_called()
        launch.assert_not_called()

    def test_model_land_remains_launchable(self):
        path = self.output / "proposal.json"
        path.write_text(DroneTaskProposal(task_id="t", action_id="a", kind="LAND",
                                          velocity_m_s=.5).model_dump_json())
        with patch("rrm_command_console.probe_isaac_runtime") as probe, \
             patch("rrm_command_console.subprocess.run"), \
             patch("rrm_command_console.subprocess.Popen") as launch:
            dispatch = self.app._launch_dispatch("id", self.output, path)
        probe.assert_not_called()
        launch.assert_called_once()
        self.assertEqual(json.loads((self.output / "isaac-runtime.json").read_text())["status"],
                         "LAND_EXEMPT")
        # Dispatcher normally closes this on finalization.
        launch.call_args.kwargs["stdout"].close()

    def test_display_state_never_authorizes_a_later_launch(self):
        handler = object.__new__(make_handler(self.app))
        handler.path = "/api/state"
        handler.respond = lambda value: value
        with patch("rrm_command_console.probe_isaac_runtime", side_effect=[
            {"compatible": True}, {"compatible": False, "reason": "runtime changed"}
        ]) as probe:
            state = handler.do_GET()
            self.assertTrue(state["isaac_runtime"]["compatible"])
            self.assertTrue(state["ordinary_motion_dependency_compatible"])
            self.assertEqual(state["command_execution_enabled"],
                             self.app.active_scene_shortname is not None)
            with self.assertRaisesRegex(RuntimeError, "runtime changed"):
                Console.require_isaac_runtime((DroneTaskProposal(
                    task_id="t", action_id="a", kind="TAKEOFF",
                    target_altitude_m=1, velocity_m_s=.5),))
        self.assertEqual(probe.call_count, 2)

    def test_truthy_non_boolean_compatibility_cannot_admit(self):
        proposal = DroneTaskProposal(task_id="t", action_id="a", kind="TAKEOFF",
                                     target_altitude_m=1, velocity_m_s=.5)
        with patch("rrm_command_console.probe_isaac_runtime",
                   return_value={"compatible": "true", "reason": "invalid evidence"}), \
             self.assertRaisesRegex(RuntimeError, "invalid evidence"):
            Console.require_isaac_runtime((proposal,))

    def test_ui_reports_preflight_without_removing_recovery_controls(self):
        ui = (Path(__file__).parents[1] / "scripts/ui/command_console.html").read_text()
        self.assertIn("Simulator dependency preflight", ui)
        self.assertIn("byId('runtime-status').textContent", ui)
        self.assertIn("Landing and STOP remain available", ui)
        self.assertIn("/api/mission/stop", ui)
