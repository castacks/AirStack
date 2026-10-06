"""Evidence/cap/error tests with a read-only Dynamic Control stand-in."""
import ast
import importlib.util
import sys
from unittest.mock import Mock, patch
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("physical_truth", Path(__file__).parents[2] /
    "simulation/isaac-sim/launch_scripts/physical_truth.py")
truth = importlib.util.module_from_spec(spec)
spec.loader.exec_module(truth)


class DC:
    def get_rigid_body(self, path):
        return 1

    def get_rigid_body_pose(self, body):
        return NS(p=NS(x=1., y=2., z=3.), r=NS(x=0., y=0., z=0., w=1.))

    def get_rigid_body_linear_velocity(self, body):
        return NS(x=.1, y=.2, z=.3)

    get_rigid_body_angular_velocity = get_rigid_body_linear_velocity


def vehicle():
    return NS(get_dc_interface=lambda: DC(), state=NS(position=[4., 5., 6.],
                                                   attitude=[0., 0., 0., 1.]))


class TruthCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name)
        self.warnings = []
        self.capture = truth.PhysicalTruthCapture(self.path, warn=self.warnings.append)
        self.addCleanup(lambda: self.capture.close("test_end"))

    def request(self, name="attempt", enabled=True):
        (self.path / "capture.json").write_text(json.dumps({"capture_id": name, "enabled": enabled}))

    def sample(self, now=10, sim=1, playing=True):
        self.capture.sample(sim, {"/World/drone": vehicle()}, now=now, playing=playing)

    def test_physics_and_sensor_are_independent_incremental_records(self):
        self.request(); self.sample()
        data = json.loads((self.path / "attempt.jsonl").read_text())
        self.assertEqual(data["rigid_body_position_m"], [1., 2., 3.])
        self.assertEqual(data["sensor_state_position_m"], [4., 5., 6.])
        self.assertEqual(data["sim_time_s"], 1)
        self.assertEqual(self.capture.records, 1)

    def test_no_request_or_paused_world_emits_no_records(self):
        self.sample(); self.assertIsNone(self.capture.file)
        self.request(); self.sample(now=11, playing=False)
        self.assertEqual(self.capture.records, 0)
        self.sample(now=11.1); self.assertEqual(self.capture.records, 1)

    def test_record_and_byte_caps(self):
        self.capture.max_records = 1
        self.request(); self.sample(); self.sample(now=11)
        self.assertEqual(self.capture.reason, "record_limit")
        self.request("next"); self.capture.max_bytes = 1; self.sample(now=12)
        self.assertEqual(self.capture.reason, "byte_limit")
        self.assertEqual(self.capture.records, 0)

    def test_duration_expires_even_when_simulation_paused(self):
        self.capture.max_duration_s = 2
        self.request(); self.sample(); self.sample(now=12, playing=False)
        self.assertEqual(self.capture.reason, "duration_limit")
        self.assertIsNone(self.capture.file)

    def test_existing_file_never_truncated_and_new_id_recovers(self):
        (self.path / "attempt.jsonl").write_text("retained\n")
        self.request(); self.sample()
        self.assertIsNone(self.capture.file)
        self.assertEqual((self.path / "attempt.jsonl").read_text(), "retained\n")
        self.request("new"); self.sample(now=11)
        self.assertEqual(self.capture.records, 1)

    def test_invalid_request_and_nonfinite_sample_fail_only_recorder(self):
        self.request("../escape"); self.sample()
        self.assertIsNone(self.capture.file)
        self.request("valid"); self.sample(now=11, sim=float("nan"))
        self.assertEqual(self.capture.reason, "error")
        self.assertEqual(self.capture.records, 0)
        self.assertTrue(self.warnings)

    def test_invalid_request_after_active_updates_status(self):
        self.request(); self.sample()
        self.assertTrue(json.loads((self.path / "status.json").read_text())["recording"])
        (self.path / "capture.json").write_text("invalid JSON")
        self.sample(now=11)
        status = json.loads((self.path / "status.json").read_text())
        self.assertFalse(status["recording"])
        self.assertEqual(status["reason"], "error")
        self.assertTrue(status["last_error"])
        self.assertEqual(status["records"], 1)

    def test_cap_status_is_current_and_status_io_does_not_escape(self):
        self.capture.max_records = 1
        self.request(); self.sample(); self.sample(now=10.2)
        status = json.loads((self.path / "status.json").read_text())
        self.assertFalse(status["recording"])
        self.assertEqual(status["reason"], "record_limit")
        (self.path / "status.json").unlink()
        (self.path / "status.json").mkdir()
        self.sample(now=11.3)
        self.assertTrue(any("status unavailable" in w for w in self.warnings))

    def test_observer_close_failure_still_closes_simulator(self):
        source = Path(__file__).parents[2] / "simulation/isaac-sim/launch_scripts/pegasus_app.py"
        tree = ast.parse(source.read_text())
        run = next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
                   for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "run")
        module = ast.Module(body=[run], type_ignores=[])
        sim = NS(is_running=lambda: False, close=Mock())
        namespace = {"SIMULATION_APP": sim, "carb": NS(log_warn=Mock())}
        exec(compile(module, str(source), "exec"), namespace)
        kit_app = NS(get_app=lambda: None)
        omni = NS(kit=NS(app=kit_app))
        modules = {"omni": omni, "omni.kit": omni.kit, "omni.kit.app": kit_app,
                   "omni.isaac.core.world": NS(World=NS(instance=lambda: None))}
        timeline = NS(play=Mock(), stop=Mock())
        app = NS(play_on_start=False, timeline=timeline,
                 clock_observation=None,
                 physical_truth=NS(close=Mock(side_effect=OSError("disk failed"))))
        with patch.dict(sys.modules, modules):
            namespace["run"](app)
        self.assertEqual(timeline.stop.call_count, 2)
        sim.close.assert_called_once()
        app.physical_truth.close.assert_called_once()

    def test_cadence_and_stop_do_not_reopen_same_id(self):
        self.request(); self.sample(); self.sample(now=10.01)
        self.assertEqual(self.capture.records, 1)
        self.request(enabled=False); self.sample(now=11)
        self.assertIsNone(self.capture.file)
        self.request(); self.sample(now=12)
        self.assertIsNone(self.capture.file)


if __name__ == "__main__":
    unittest.main()
