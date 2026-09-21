"""Tests for the dry-run semantic RRM-to-AirStack drone decision bridge."""

import unittest

from rrm.airstack_drone import MapWaypoint
from rrm.contracts import CapabilityDeclaration, Truth
from rrm.drone_decision import (
    AirStackDroneDecisionBridge,
    DroneDecisionStatus,
    DroneNavigationTarget,
)
from rrm.schema import AbstractAction, Predicate, SELF, Verb
from rrm.state_contracts import FactEvidence, FactKey, FactProvenance, StateSnapshot
from rrm.task_contracts import IntentStatus, PlanProposal, PlannedAction, ReasoningResult, TaskRequest


def task(objective="navigate to waypoint-a", embodiment="iris-v1"):
    return TaskRequest(
        task_id="task-1", revision="task/v1", objective=objective,
        constraints_revision="constraints/v1", issuer_id="operator-1",
        permission_revision="permission/v1", requested_embodiment_id=embodiment,
    )


def snapshot(*, truth=Truth.TRUE, received=10.0, task_id="task-1"):
    return StateSnapshot(
        snapshot_id="state-1", revision="state/v1", task_id=task_id, episode_id="episode-1",
        evidence=(FactEvidence(
            key=FactKey(subject="waypoint-a", predicate="localized"), truth=truth,
            provenance=FactProvenance.SIMULATOR, source_ref="shadow/run-1",
            observed_monotonic_s=received, received_monotonic_s=received, max_age_s=1.0,
        ),),
    )


def profile(*, operations=frozenset({Verb.NAVIGATE_TO.value}), resources=frozenset({"airframe"})):
    return CapabilityDeclaration(
        embodiment_id="iris-v1", revision="iris/v1", operations=operations,
        resources=resources, available_resources=resources, limits_ref="iris/limits/v1",
    )


def bridge():
    return AirStackDroneDecisionBridge(
        embodiment_id="iris-v1", robot_name="robot_1",
        targets=(DroneNavigationTarget(
            entity_id="waypoint-a", waypoints=(MapWaypoint(x=1.0, y=2.0, z=3.0),),
            goal_tolerance_m=0.5,
        ),),
    )


class DroneDecisionBridgeTests(unittest.TestCase):
    def test_targetless_takeoff_uses_adapter_owned_flight_parameters(self):
        request = task("Take off", embodiment="iris-v1")
        state = snapshot()
        capabilities = profile(operations=frozenset({Verb.TAKEOFF.value}))
        intent = ReasoningResult(
            task_id=request.task_id, task_revision=request.revision,
            state_revision=state.revision, capability_revision=capabilities.revision,
            status=IntentStatus.READY,
            grounded_goal=Predicate(name="airborne", subject=SELF),
            grounded_entities=(),
        )
        action = AbstractAction(id="takeoff-1", verb=Verb.TAKEOFF, targets=[])
        plan = PlanProposal(
            plan_id="task-1/takeoff", version=0, task_id=request.task_id,
            task_revision=request.revision, intent=intent,
            state_revision=state.revision, capability_revision=capabilities.revision,
            actions=(PlannedAction(
                action=action, semantics_revision="rrm/verbs/v1",
                feasibility_ref="iris/v1/TAKEOFF/takeoff-1",
                expected_effect_window_revision="airframe/takeoff-effect/v1",
            ),), recovery_budget=0,
        )
        result = AirStackDroneDecisionBridge(
            embodiment_id="iris-v1", robot_name="robot_1", targets=(),
            takeoff_altitude_m=1.75, takeoff_velocity_m_s=0.4,
        ).compile_plan(plan, request, state, capabilities, now_monotonic_s=10.5)
        self.assertIs(result.status, DroneDecisionStatus.READY)
        self.assertEqual(result.proposal.kind.value, "TAKEOFF")
        self.assertEqual(result.proposal.target_altitude_m, 1.75)
        self.assertEqual(result.proposal.velocity_m_s, 0.4)

    def test_multi_waypoint_binding_is_preserved_as_one_route(self):
        route_bridge = AirStackDroneDecisionBridge(
            embodiment_id="iris-v1", robot_name="robot_1",
            targets=(DroneNavigationTarget(
                entity_id="waypoint-a",
                waypoints=(MapWaypoint(x=1.0, y=1.0, z=1.5),
                           MapWaypoint(x=3.0, y=2.0, z=1.5)),
                goal_tolerance_m=0.4,
            ),),
        )
        result = route_bridge.decide(task(), snapshot(), profile(), now_monotonic_s=10.5)
        self.assertIs(result.status, DroneDecisionStatus.READY)
        self.assertEqual(len(result.proposal.waypoints), 2)
        self.assertEqual(result.proposal.waypoints[-1].x, 3.0)

    def test_learned_plan_is_preserved_without_objective_reinterpretation(self):
        original = bridge().decide(task(), snapshot(), profile(), now_monotonic_s=10.5)
        natural_task = task("Please approach the blue marker beside the doorway")
        result = bridge().compile_plan(original.plan, natural_task, snapshot(), profile(),
                                       now_monotonic_s=10.5)
        self.assertIs(result.status, DroneDecisionStatus.READY)
        self.assertEqual(result.plan, original.plan)
        self.assertEqual(result.proposal.action_id, original.plan.actions[0].action.id)

    def test_learned_plan_rechecks_context_capability_and_freshness(self):
        plan = bridge().decide(task(), snapshot(), profile(), now_monotonic_s=10.5).plan
        cases = (
            (task().model_copy(update={"revision": "new"}), snapshot(), profile(), 10.5),
            (task(), snapshot(), profile(), 100.0),
            (task(), snapshot(truth=Truth.FALSE), profile(), 10.5),
            (task(), snapshot(), profile(operations=frozenset()), 10.5),
        )
        for request, state, caps, now in cases:
            result = bridge().compile_plan(plan, request, state, caps, now_monotonic_s=now)
            self.assertIsNone(result.proposal)

    def test_generic_navigation_plan_grounds_to_exact_air_stack_proposal(self):
        result = bridge().decide(task(), snapshot(), profile(), now_monotonic_s=10.5)
        self.assertIs(result.status, DroneDecisionStatus.READY)
        self.assertEqual(result.intent.grounded_entities, ("waypoint-a",))
        self.assertEqual(result.plan.actions[0].action.verb, Verb.NAVIGATE_TO)
        self.assertEqual(result.plan.actions[0].action.targets, ["waypoint-a"])
        self.assertEqual(result.plan.actions[0].action.params, {})
        preview = result.proposal.preview()
        self.assertEqual(preview["action_name"], "/robot_1/tasks/navigate")
        self.assertEqual(preview["goal"]["global_plan"]["header"]["frame_id"], "map")
        self.assertFalse(preview["execution_requested"])

    def test_unknown_or_stale_location_holds_without_a_plan_or_proposal(self):
        for state, now in ((snapshot(truth=Truth.UNKNOWN), 10.5), (snapshot(received=1.0), 10.5)):
            with self.subTest(now=now):
                result = bridge().decide(task(), state, profile(), now_monotonic_s=now)
                self.assertIs(result.status, DroneDecisionStatus.HOLD)
                self.assertIn("target_localization_unknown", result.reasons)
                self.assertIsNone(result.intent)
                self.assertIsNone(result.plan)
                self.assertIsNone(result.proposal)

    def test_ambiguous_or_unsupported_requests_never_ground_a_motion_proposal(self):
        unresolved = bridge().decide(task("navigate to unknown-pad"), snapshot(), profile(), now_monotonic_s=10.5)
        unsupported = bridge().decide(task(), snapshot(), profile(operations=frozenset()), now_monotonic_s=10.5)
        wrong_body = bridge().decide(task(embodiment="ground-base-v1"), snapshot(), profile(), now_monotonic_s=10.5)
        for result in (unresolved, unsupported, wrong_body):
            self.assertIsNone(result.plan)
            self.assertIsNone(result.proposal)
        self.assertIs(unresolved.status, DroneDecisionStatus.NEEDS_CLARIFICATION)
        self.assertIs(unsupported.status, DroneDecisionStatus.UNSUPPORTED)
        self.assertIn("unsupported_operation", unsupported.reasons)
        self.assertEqual(wrong_body.reasons, ("wrong_embodiment",))

    def test_bridge_has_no_ros_or_execution_control_surface(self):
        from pathlib import Path
        source = (Path(__file__).parents[1] / "rrm" / "drone_decision.py").read_text(encoding="utf-8")
        runner = (Path(__file__).parents[1] / "scripts" / "rrm_drone_decision.py").read_text(encoding="utf-8")
        for prohibited in ("rclpy", "ActionClient", "create_publisher", "create_client", "--execute"):
            self.assertNotIn(prohibited, source)
            self.assertNotIn(prohibited, runner)


if __name__ == "__main__":
    unittest.main()
