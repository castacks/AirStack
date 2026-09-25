"""CPU-only gateway tests; no Isaac import, simulator launch, or real motion."""

from dataclasses import replace
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest

from rrm.contracts import DispatchContext, SafetyDecision
from rrm.hand_execution_boundary import (
    DurableJournal, HandCommand, HandExecutionBoundary,
)
from simulation.hand_isaac_adapter import GatewayError, IsaacHandAdapter


class FakeArticulation:
    def __init__(self):
        self.positions = [0.0] * 23
        self.velocities = [0.0] * 23
        self.actions = []
        self.fail_apply = False

    def get_joint_positions(self):
        return self.positions

    def get_joint_velocities(self):
        return self.velocities

    def apply_action(self, action):
        if self.fail_apply:
            raise RuntimeError("simulator fault")
        self.actions.append(action)


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.ledger_path = Path(self.temp.name) / "gateway.jsonl"
        self.articulation = FakeArticulation()
        self.mode = "POSITION_HOLD"
        self.external_active = False
        self.speed = 0.0
        self.names = IsaacHandAdapter.ARM_JOINTS + tuple(f"finger_{i}" for i in range(16))

    def adapter(self, *, enabled=False):
        return IsaacHandAdapter(articulation=self.articulation,
            action_factory=lambda **kwargs: kwargs, ledger_path=self.ledger_path,
            episode_id="episode-1", scene_recipe_sha256="scene-1",
            joint_names=self.names, lower_rad=(-1.0,) * 23,
            upper_rad=(1.0,) * 23, max_velocity_rad_s=(1.0,) * 23,
            object_speeds=lambda: (self.speed,),
            controller_mode=lambda: self.mode,
            external_motion_active=lambda: self.external_active,
            motion_enabled=enabled)

    def command(self):
        return HandCommand("iiwa7_joint_1", 0.0, 0.01, time.monotonic(),
                           "episode-1", "scene-1", "probe", "qualification")

    def test_disabled_default_and_single_durable_target(self):
        adapter = self.adapter()
        with self.assertRaises(GatewayError):
            adapter.submit("dispatch-1", 1, self.command())
        self.assertEqual(self.articulation.actions, [])
        adapter = self.adapter(enabled=True)
        command = self.command()
        adapter.submit("dispatch-1", 1, command)
        self.assertEqual(self.articulation.actions, [])
        self.assertIn("C09_ADAPTER_ENQUEUE", self.ledger_path.read_text())
        self.assertEqual(adapter.tick(now=time.monotonic()), "TARGET_APPLIED")
        self.assertEqual(self.articulation.actions[0]["joint_indices"], (0,))
        with self.assertRaises(GatewayError):
            adapter.submit("dispatch-1", 1, command)
        restarted = self.adapter(enabled=True)
        self.assertFalse(restarted.motion_enabled)

    def test_wrong_episode_grasp_stale_and_changed_state_denied(self):
        adapter = self.adapter(enabled=True)
        original = self.command()
        for command in (replace(original, episode_id="old"),
                        replace(original, operation="GRASP"),
                        replace(original, observed_at_monotonic=time.monotonic() - 1),
                        replace(original, target_position_rad=0.3)):
            with self.subTest(command=command):
                with self.assertRaises(GatewayError):
                    adapter.submit("dispatch-1", 1, command)
        self.articulation.positions[0] = 0.1
        with self.assertRaises(GatewayError):
            adapter.submit("dispatch-1", 1, original)
        self.assertEqual(self.articulation.actions, [])

    def test_stop_fences_queued_target_and_safe_requires_five_samples(self):
        adapter = self.adapter(enabled=True)
        adapter.submit("dispatch-1", 1, self.command())
        self.assertTrue(adapter.request_stop(2))
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        self.assertEqual(self.articulation.actions[0]["joint_positions"], (0.0,) * 7)
        self.assertEqual(len(self.articulation.actions), 1)
        self.external_active = True
        self.assertIsNone(adapter.sample(2, now=time.monotonic()))
        self.external_active = False
        for _ in range(4):
            self.assertIsNone(adapter.sample(2, now=time.monotonic()))
        evidence = adapter.sample(2, now=time.monotonic())
        self.assertIsNotNone(evidence)
        self.assertEqual(evidence.episode_id, "episode-1")
        self.speed = 0.1
        self.assertIsNone(adapter.sample(2, now=time.monotonic()))
        with self.assertRaises(GatewayError):
            adapter.safe_state(2)

    def test_apply_failure_inhibits_and_queues_hold(self):
        adapter = self.adapter(enabled=True)
        adapter.submit("dispatch-1", 1, self.command())
        self.articulation.fail_apply = True
        with self.assertRaisesRegex(GatewayError, "apply_uncertain"):
            adapter.tick(now=time.monotonic())
        self.assertFalse(adapter.motion_enabled)
        self.assertEqual(self.articulation.actions, [])
        self.articulation.fail_apply = False
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        self.assertEqual(self.articulation.actions[0]["joint_positions"], (0.0,) * 7)

    def test_runtime_limit_breach_inhibits_and_holds(self):
        adapter = self.adapter(enabled=True)
        adapter.submit("dispatch-1", 1, self.command())
        self.assertEqual(adapter.tick(now=time.monotonic()), "TARGET_APPLIED")
        self.articulation.velocities[0] = 1.1
        self.assertIsNone(adapter.sample(1, now=time.monotonic()))
        self.assertFalse(adapter.motion_enabled)
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        with self.assertRaises(GatewayError):
            adapter.safe_state(1)

    def test_boundary_to_gateway_fake_lifecycle(self):
        adapter = self.adapter(enabled=True)
        for _ in range(5):
            adapter.sample(0, now=time.monotonic())
        probe = {"scene_recipe_sha256": "scene-1",
            "joint_limits": [{"name": "iiwa7_joint_1", "lower_rad": -1.0,
                              "upper_rad": 1.0, "max_velocity_rad_s": 1.0}],
            "safe_state": {"joint_velocity_threshold_rad_s": 0.1,
                           "object_velocity_threshold_m_s": 0.01,
                           "consecutive_window_required": 5}}
        probe_bytes = json.dumps(probe, sort_keys=True).encode()
        qualification = {"schema_version": "rrm-hand-qualification/v1",
            "probe_sha256": hashlib.sha256(probe_bytes).hexdigest(),
            "scene_recipe_sha256": "scene-1",
            "gates": {name: {"status": "PASS"} for name in
                ("controller_limits", "post_command_reset", "contact_observer",
                 "safe_state", "independent_stop")},
            "ready_for_single_bounded_contact_trial": True,
            "execution_dispatch": False}
        qualification_bytes = json.dumps(qualification, sort_keys=True).encode()
        boundary = HandExecutionBoundary(adapter=adapter,
            journal=DurableJournal(Path(self.temp.name) / "boundary.jsonl"),
            qualification_bytes=qualification_bytes, probe_bytes=probe_bytes)
        self.assertTrue(boundary.reset(authorized=True, generation=0,
                                       now=time.monotonic()))
        command = HandCommand("iiwa7_joint_1", 0.0, 0.01, time.monotonic(),
            "episode-1", "scene-1", qualification["probe_sha256"],
            hashlib.sha256(qualification_bytes).hexdigest())
        context = DispatchContext("run-1", "task-1", "plan-1", "calibration-1",
            "dispatch-1", command.digest, "state-1", "capability-1",
            "permission-1", "approval-1", "constraints-1", boundary.epoch,
            boundary.generation)
        decision = SafetyDecision("decision-1", context, "ALLOW",
                                  time.monotonic() - 0.1, time.monotonic() + 1)
        boundary.dispatch(decision, context, command, now=time.monotonic())
        self.assertEqual(self.articulation.actions, [])
        self.assertEqual(adapter.tick(), "TARGET_APPLIED")
        self.assertTrue(boundary.stop(intervention_id="stop-1", reason="test"))
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        for _ in range(5):
            adapter.sample(boundary.generation, now=time.monotonic())
        self.assertTrue(boundary.confirm_stopped(generation=boundary.generation,
                                                now=time.monotonic()))
        self.assertEqual(len(self.articulation.actions), 2)


if __name__ == "__main__":
    unittest.main()
