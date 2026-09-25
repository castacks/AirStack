"""CPU-only gateway tests; no Isaac import, simulator launch, or real motion."""

from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import time
import unittest

from rrm.contracts import DispatchContext, SafetyDecision
from rrm.hand_execution_boundary import (
    DurableJournal, HandAuthorityVerifier, HandCommand, HandExecutionBoundary,
    dispatch_authorization_scope, issue_hand_authorization,
    reset_authorization_scope,
)
from simulation.hand_isaac_adapter import GatewayError, IsaacHandAdapter


class FakeArticulation:
    def __init__(self):
        self.positions = [0.0] * 23
        self.velocities = [0.0] * 23
        self.actions = []
        self.fail_apply = False
        self.on_apply = None

    def get_joint_positions(self):
        return self.positions

    def get_joint_velocities(self):
        return self.velocities

    def apply_action(self, action):
        if self.fail_apply:
            raise RuntimeError("simulator fault")
        self.actions.append(action)
        if self.on_apply is not None:
            self.on_apply()


class FakeClock:
    def __init__(self, value=100.0):
        self.value = value

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


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

    def adapter(self, *, enabled=False, clock=None, max_tick_gap_s=0.10,
                max_stop_to_hold_s=0.10):
        options = {} if clock is None else {"clock": clock}
        return IsaacHandAdapter(articulation=self.articulation,
            action_factory=lambda **kwargs: kwargs, ledger_path=self.ledger_path,
            episode_id="episode-1", scene_recipe_sha256="scene-1",
            joint_names=self.names, lower_rad=(-1.0,) * 23,
            upper_rad=(1.0,) * 23, max_velocity_rad_s=(1.0,) * 23,
            object_speeds=lambda: (self.speed,),
            controller_mode=lambda: self.mode,
            external_motion_active=lambda: self.external_active,
            motion_enabled=enabled, max_tick_gap_s=max_tick_gap_s,
            max_stop_to_hold_s=max_stop_to_hold_s, **options)

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

    def test_restart_restores_generation_holds_and_durably_reconciles_without_motion(self):
        adapter = self.adapter(enabled=True)
        adapter.submit("dispatch-1", 3, self.command())
        self.assertEqual(adapter.tick(now=time.monotonic()), "TARGET_APPLIED")
        self.assertEqual(len(self.articulation.actions), 1)

        restarted = self.adapter(enabled=True)
        self.assertFalse(restarted.motion_enabled)
        self.assertEqual(restarted.tick(), "HOLD_APPLIED")
        self.assertEqual(len(self.articulation.actions), 2)
        self.assertEqual(self.articulation.actions[-1]["joint_positions"], (0.0,) * 7)
        for _ in range(4):
            self.assertIsNone(restarted.sample(3, now=time.monotonic()))
        evidence = restarted.sample(3, now=time.monotonic())
        self.assertIsNotNone(evidence)
        self.assertFalse(restarted.motion_enabled)
        self.assertIn("C08_ADAPTER_SAFE_RECONCILED", self.ledger_path.read_text())

        clean_restart = self.adapter(enabled=True)
        self.assertTrue(clean_restart.motion_enabled)
        with self.assertRaisesRegex(GatewayError, "duplicate_dispatch_id"):
            clean_restart.submit("dispatch-1", 3, self.command())
        self.assertEqual(len(self.articulation.actions), 2)

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

    def test_idle_heartbeat_stress_and_stale_watchdog_never_apply_action(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=False, clock=clock, max_tick_gap_s=0.05)
        for _ in range(1000):
            self.assertEqual(adapter.tick(), "IDLE")
            clock.advance(0.00001)
        evidence = adapter.liveness()
        self.assertTrue(evidence.healthy)
        self.assertEqual(evidence.tick_count, 1000)
        self.assertEqual(evidence.max_tick_duration_s, 0.0)
        self.assertEqual(self.articulation.actions, [])

        with ThreadPoolExecutor(max_workers=8) as pool:
            snapshots = list(pool.map(lambda _: adapter.liveness(), range(100)))
        self.assertTrue(all(item.healthy for item in snapshots))
        clock.advance(0.051)
        fault = adapter.watchdog_check()
        self.assertFalse(fault.healthy)
        self.assertEqual(fault.reason, "TICK_STALE")
        self.assertFalse(adapter.motion_enabled)
        self.assertEqual(self.articulation.actions, [])
        adapter.watchdog_check()
        ledger = self.ledger_path.read_text()
        self.assertEqual(ledger.count("C08_ADAPTER_LIVENESS_FAULT"), 1)

    def test_completed_idle_tick_overrun_latches_durably_without_action(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=False, clock=clock, max_tick_gap_s=0.05)
        clock.advance(0.051)
        with self.assertRaisesRegex(GatewayError, "tick_deadline_exceeded"):
            adapter.tick(now=100.0)
        evidence = adapter.liveness()
        self.assertFalse(evidence.healthy)
        self.assertEqual(evidence.reason, "TICK_DEADLINE_EXCEEDED")
        self.assertAlmostEqual(evidence.last_tick_duration_s, 0.051)
        self.assertEqual(self.articulation.actions, [])
        self.assertEqual(self.ledger_path.read_text().count(
            "C08_ADAPTER_LIVENESS_FAULT"), 1)

        restarted = self.adapter(enabled=True, clock=clock)
        self.assertFalse(restarted.motion_enabled)
        self.assertEqual(restarted.liveness().reason, "TICK_DEADLINE_EXCEEDED")
        self.assertEqual(self.articulation.actions, [])

    def test_completed_active_tick_overrun_fences_and_requires_hold(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock, max_tick_gap_s=0.05)
        command = replace(self.command(), observed_at_monotonic=clock())
        adapter.submit("dispatch-1", 1, command)
        self.articulation.on_apply = lambda: clock.advance(0.051)
        with self.assertRaisesRegex(GatewayError, "tick_deadline_exceeded"):
            adapter.tick()
        self.assertFalse(adapter.motion_enabled)
        self.assertEqual(adapter.liveness().reason, "TICK_DEADLINE_EXCEEDED")
        self.assertEqual(len(self.articulation.actions), 1)

        self.articulation.on_apply = None
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        self.assertEqual(len(self.articulation.actions), 2)
        self.assertEqual(self.articulation.actions[-1]["joint_positions"], (0.0,) * 7)
        with self.assertRaises(GatewayError):
            adapter.submit("dispatch-2", 1, command)

    def test_watchdog_fences_while_fake_simulator_call_is_blocked(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock, max_tick_gap_s=0.05)
        command = replace(self.command(), observed_at_monotonic=clock())
        adapter.submit("dispatch-1", 1, command)
        entered, release = Event(), Event()

        def block_apply():
            entered.set()
            self.assertTrue(release.wait(1.0))

        self.articulation.on_apply = block_apply
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(adapter.tick)
            self.assertTrue(entered.wait(1.0))
            self.assertEqual(adapter.watchdog_fence(now=clock()), "HEALTHY")
            self.assertTrue(adapter.motion_enabled)
            clock.advance(0.051)
            started = time.monotonic()
            self.assertEqual(adapter.watchdog_fence(now=clock()), "TICK_STALE")
            self.assertLess(time.monotonic() - started, 0.05)
            self.assertFalse(future.done())
            self.assertFalse(adapter.motion_enabled)
            self.assertEqual(len(self.articulation.actions), 1)
            release.set()
            with self.assertRaisesRegex(GatewayError, "watchdog_fenced"):
                future.result(timeout=1.0)

        self.assertEqual(self.ledger_path.read_text().count(
            "C08_ADAPTER_LIVENESS_FAULT"), 1)
        self.articulation.on_apply = None
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        self.assertEqual(len(self.articulation.actions), 2)

    def test_stop_returns_and_is_durable_while_fake_simulator_call_is_blocked(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock, max_tick_gap_s=1.0,
            max_stop_to_hold_s=0.05)
        command = replace(self.command(), observed_at_monotonic=clock())
        adapter.submit("dispatch-1", 1, command)
        entered, release = Event(), Event()

        def block_apply():
            entered.set()
            self.assertTrue(release.wait(1.0))

        self.articulation.on_apply = block_apply
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(adapter.tick)
            self.assertTrue(entered.wait(1.0))
            clock.advance(0.01)
            started = time.monotonic()
            self.assertTrue(adapter.request_stop(2, now=clock()))
            self.assertLess(time.monotonic() - started, 0.05)
            self.assertFalse(future.done())
            self.assertFalse(adapter.motion_enabled)
            self.assertIn("C08_ADAPTER_STOP_REQUEST", self.ledger_path.read_text())
            self.assertEqual(len(self.articulation.actions), 1)
            release.set()
            with self.assertRaisesRegex(GatewayError, "stop_fenced"):
                future.result(timeout=1.0)

        self.articulation.on_apply = None
        clock.advance(0.01)
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        evidence = adapter.liveness()
        self.assertFalse(evidence.stop_pending)
        self.assertAlmostEqual(evidence.stop_to_hold_s, 0.01)
        self.assertEqual(len(self.articulation.actions), 2)

    def test_restart_reconstructs_unfulfilled_stop_before_enabling_motion(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock)
        self.assertTrue(adapter.request_stop(3, now=clock()))
        self.assertEqual(self.articulation.actions, [])

        restarted = self.adapter(enabled=True, clock=clock)
        self.assertFalse(restarted.motion_enabled)
        self.assertTrue(restarted.liveness().stop_pending)
        self.assertEqual(restarted.tick(), "HOLD_APPLIED")
        self.assertEqual(len(self.articulation.actions), 1)

        clean_restart = self.adapter(enabled=True, clock=clock)
        self.assertTrue(clean_restart.motion_enabled)
        self.assertFalse(clean_restart.liveness().stop_pending)

    def test_tick_completion_clock_regression_is_durable(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock)
        with self.assertRaisesRegex(GatewayError, "invalid_gateway_clock"):
            adapter.tick(now=101.0)
        self.assertEqual(adapter.liveness(now=101.0).reason, "CLOCK_REGRESSION")
        self.assertEqual(self.articulation.actions, [])
        restarted = self.adapter(enabled=True, clock=clock)
        self.assertFalse(restarted.motion_enabled)

    def test_timely_stop_records_exact_hold_latency(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock,
            max_tick_gap_s=0.10, max_stop_to_hold_s=0.05)
        self.articulation.on_apply = lambda: clock.advance(0.02)
        self.assertTrue(adapter.request_stop(1, now=clock()))
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        evidence = adapter.liveness()
        self.assertTrue(evidence.healthy)
        self.assertFalse(evidence.stop_pending)
        self.assertEqual(evidence.stop_requested_at, 100.0)
        self.assertAlmostEqual(evidence.hold_applied_at, 100.02)
        self.assertAlmostEqual(evidence.stop_to_hold_s, 0.02)
        self.assertAlmostEqual(evidence.last_tick_duration_s, 0.02)
        self.assertIn('"deadline_met":true', self.ledger_path.read_text())

    def test_late_stop_watchdog_fences_before_simulator_thread_recovers(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock,
            max_tick_gap_s=1.0, max_stop_to_hold_s=0.05)
        self.assertTrue(adapter.request_stop(1, now=clock()))
        clock.advance(0.051)
        fault = adapter.watchdog_check()
        self.assertEqual(fault.reason, "STOP_DEADLINE_EXCEEDED")
        self.assertFalse(adapter.motion_enabled)
        self.assertEqual(self.articulation.actions, [])
        self.assertEqual(adapter.tick(), "HOLD_APPLIED")
        evidence = adapter.liveness()
        self.assertEqual(evidence.reason, "STOP_DEADLINE_EXCEEDED")
        self.assertGreater(evidence.stop_to_hold_s, 0.05)
        self.assertEqual(len(self.articulation.actions), 1)
        restarted = self.adapter(enabled=True, clock=clock)
        self.assertFalse(restarted.motion_enabled)

    def test_invalid_stop_and_clock_regression_fail_closed_without_action(self):
        clock = FakeClock()
        adapter = self.adapter(enabled=True, clock=clock)
        self.assertFalse(adapter.request_stop(-1, now=clock()))
        self.assertFalse(adapter.request_stop(1, now=float("nan")))
        self.assertEqual(adapter.tick(), "IDLE")
        clock.value = 99.0
        fault = adapter.watchdog_check()
        self.assertEqual(fault.reason, "CLOCK_REGRESSION")
        self.assertFalse(adapter.motion_enabled)
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
            authority_verifier=HandAuthorityVerifier(
                issuer_keys={"test-issuer": b"test-hand-authority-key-32-bytes!"},
                allowed_roles=frozenset({"hand-calibration-operator"})),
            qualification_bytes=qualification_bytes, probe_bytes=probe_bytes)
        now = time.monotonic()
        evidence = adapter.safe_state(0)
        reset_scope = reset_authorization_scope(generation=0, evidence=evidence,
            qualification_sha256=boundary.qualification_sha256,
            probe_sha256=boundary.qualification["probe_sha256"])
        reset_authorization = issue_hand_authorization(
            signing_key=b"test-hand-authority-key-32-bytes!",
            authorization_id="reset-1", issuer_id="test-issuer",
            subject_id="operator-1", role="hand-calibration-operator", purpose="RESET",
            authority_epoch=boundary.epoch, stop_generation=0,
            scope_digest=reset_scope, issued_at_monotonic=now - 0.1,
            expires_at_monotonic=now + 1.0)
        self.assertTrue(boundary.reset(authorization=reset_authorization,
                                       generation=0, now=now))
        command = HandCommand("iiwa7_joint_1", 0.0, 0.01, time.monotonic(),
            "episode-1", "scene-1", qualification["probe_sha256"],
            hashlib.sha256(qualification_bytes).hexdigest())
        context = DispatchContext("run-1", "task-1", "plan-1", "calibration-1",
            "dispatch-1", command.digest, "state-1", "capability-1",
            "permission-1", "approval-1", "constraints-1", boundary.epoch,
            boundary.generation)
        decision = SafetyDecision("decision-1", context, "ALLOW",
                                  time.monotonic() - 0.1, time.monotonic() + 1)
        now = time.monotonic()
        dispatch_scope = dispatch_authorization_scope(decision=decision, current=context,
            command=command, qualification_sha256=boundary.qualification_sha256,
            probe_sha256=boundary.qualification["probe_sha256"])
        dispatch_authorization = issue_hand_authorization(
            signing_key=b"test-hand-authority-key-32-bytes!",
            authorization_id="dispatch-authorization-1", issuer_id="test-issuer",
            subject_id="operator-1", role="hand-calibration-operator", purpose="DISPATCH",
            authority_epoch=boundary.epoch, stop_generation=boundary.generation,
            scope_digest=dispatch_scope, issued_at_monotonic=now - 0.1,
            expires_at_monotonic=now + 1.0)
        boundary.dispatch(decision, context, command,
                          authorization=dispatch_authorization, now=now)
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
