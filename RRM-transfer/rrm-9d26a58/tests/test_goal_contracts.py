"""Embodiment-neutral goal intake, routing, and parameter provenance tests."""

import unittest

from pydantic import ValidationError

from rrm.contracts import CapabilityDeclaration
from rrm.goal_contracts import (
    C01TaskBinding, EmbodimentRoute, GoalRequest, GroundingStatus, ParameterBinding,
    ParameterResolution, ParameterSource, RouteStatus, bind_selected_route_to_c01,
    route_goal,
)


def capability(embodiment_id, operations, resources, available=None):
    resources = frozenset(resources)
    return CapabilityDeclaration(
        embodiment_id=embodiment_id, revision=f"{embodiment_id}/v1",
        operations=frozenset(operations), resources=resources,
        available_resources=resources if available is None else frozenset(available),
        limits_ref=f"{embodiment_id}/limits/v1",
    )


class GoalContractTests(unittest.TestCase):
    def test_qualitative_goal_needs_no_numbers_or_embodiment(self):
        goal = GoalRequest(
            goal_id="inspect-1", revision="goal/v1",
            objective="inspect that shelf carefully", context_refs=("pointing/17",),
            required_operations=frozenset({"INSPECT"}),
            required_resources=frozenset({"camera"}),
            qualitative_constraints=("carefully",),
        )
        self.assertIsNone(goal.preferred_embodiment_id)
        self.assertNotIn("coordinates", GoalRequest.model_fields)
        self.assertNotIn("parameters", GoalRequest.model_fields)

    def test_same_goal_exposes_multiple_embodiment_candidates(self):
        goal = GoalRequest(
            goal_id="inspect-1", revision="goal/v1", objective="inspect shelf-2",
            required_operations=frozenset({"INSPECT"}),
            required_resources=frozenset({"camera"}),
        )
        route = route_goal(goal, (
            capability("drone-1", {"INSPECT", "NAVIGATE_TO"}, {"airframe", "camera"}),
            capability("rover-1", {"INSPECT", "NAVIGATE_TO"}, {"base", "camera"}),
            capability("fixed-arm-1", {"GRASP"}, {"arm", "hand"}),
        ))
        self.assertIs(route.status, RouteStatus.CANDIDATES)
        self.assertEqual(route.candidate_embodiment_ids, ("drone-1", "rover-1"))

    def test_operation_and_resources_select_manipulation_embodiment(self):
        goal = GoalRequest(
            goal_id="pick-1", revision="goal/v1", objective="pick up the selected cup",
            required_operations=frozenset({"GRASP"}),
            required_resources=frozenset({"arm", "hand"}),
        )
        route = route_goal(goal, (
            capability("drone-1", {"INSPECT"}, {"airframe", "camera"}),
            capability("mobile-manipulator-1", {"NAVIGATE_TO", "GRASP"},
                       {"base", "arm", "hand", "camera"}),
        ))
        self.assertIs(route.status, RouteStatus.SELECTED)
        self.assertEqual(route.selected_embodiment_id, "mobile-manipulator-1")

    def test_unsupported_and_temporarily_unavailable_are_distinct(self):
        goal = GoalRequest(
            goal_id="pick-1", revision="goal/v1", objective="pick up cup-1",
            required_operations=frozenset({"GRASP"}),
            required_resources=frozenset({"arm", "hand"}),
        )
        unsupported = route_goal(
            goal, (capability("drone-1", {"INSPECT"}, {"airframe", "camera"}),),
        )
        unavailable = route_goal(goal, (
            capability("arm-1", {"GRASP"}, {"arm", "hand"}, available={"arm"}),
        ))
        self.assertIs(unsupported.status, RouteStatus.UNSUPPORTED)
        self.assertIs(unavailable.status, RouteStatus.UNAVAILABLE)

    def test_preference_constrains_routing_without_changing_goal_semantics(self):
        goal = GoalRequest(
            goal_id="inspect-1", revision="goal/v1", objective="inspect shelf-2",
            required_operations=frozenset({"INSPECT"}),
            required_resources=frozenset({"camera"}),
            preferred_embodiment_id="rover-1",
        )
        route = route_goal(goal, (
            capability("drone-1", {"INSPECT"}, {"camera"}),
            capability("rover-1", {"INSPECT"}, {"camera"}),
        ))
        self.assertIs(route.status, RouteStatus.SELECTED)
        self.assertEqual(route.selected_embodiment_id, "rover-1")

    def test_resolved_numeric_value_requires_visible_provenance(self):
        result = ParameterResolution(
            goal_id="inspect-1", goal_revision="goal/v1", embodiment_id="drone-1",
            adapter_revision="drone-grounder/v3", status=GroundingStatus.RESOLVED,
            bindings=(ParameterBinding(
                name="standoff_distance", value=2.0, unit="m",
                source=ParameterSource.CONSTRAINT_PROFILE,
                source_ref="inspection-profile/v2",
            ),),
        )
        self.assertEqual(result.bindings[0].source_ref, "inspection-profile/v2")
        with self.assertRaises(ValidationError):
            ParameterBinding(name="speed", value=0.5, unit="m/s",
                             source=ParameterSource.ADAPTER_DEFAULT, source_ref="")
        with self.assertRaises(ValidationError):
            ParameterBinding(name="speed", value=float("inf"), unit="m/s",
                             source=ParameterSource.ADAPTER_DEFAULT,
                             source_ref="drone-defaults/v1")

    def test_clarification_infeasibility_and_safety_remain_separate(self):
        clarification = ParameterResolution(
            goal_id="move-1", goal_revision="goal/v1", embodiment_id="rover-1",
            adapter_revision="rover-grounder/v1",
            status=GroundingStatus.NEEDS_CLARIFICATION,
            unresolved_refs=("termination_condition",),
        )
        infeasible = ParameterResolution(
            goal_id="move-1", goal_revision="goal/v1", embodiment_id="rover-1",
            adapter_revision="rover-grounder/v1", status=GroundingStatus.INFEASIBLE,
            reason_codes=("target_outside_reachable_region",),
        )
        self.assertEqual(clarification.unresolved_refs, ("termination_condition",))
        self.assertEqual(infeasible.reason_codes, ("target_outside_reachable_region",))
        self.assertNotIn("UNAUTHORIZED", GroundingStatus.__members__)
        self.assertNotIn("SAFETY_REJECTED", GroundingStatus.__members__)

    def test_invalid_route_shapes_fail_closed(self):
        with self.assertRaises(ValidationError):
            EmbodimentRoute(
                goal_id="g", goal_revision="v1", status=RouteStatus.SELECTED,
                candidate_embodiment_ids=("drone-1",),
                selected_embodiment_id="rover-1",
            )

    def test_selected_route_binds_to_c01_without_inventing_authority(self):
        goal = GoalRequest(
            goal_id="pick-1", revision="goal/v1",
            objective="place the context-selected block on the tray",
            context_refs=("selection/red-block/v1",),
            required_operations=frozenset({"GRASP", "PLACE"}),
            required_resources=frozenset({"arm", "hand"}),
            qualitative_constraints=("keep the other block undisturbed",),
        )
        profile = capability(
            "hand-1", {"GRASP", "PLACE"}, {"arm", "hand"},
        )
        route = route_goal(goal, (profile,))
        binding = bind_selected_route_to_c01(
            goal, route, profile,
            constraints_revision="hand-constraints/v3",
            issuer_id="operator-1",
            permission_revision="shadow-only/no-dispatch",
        )
        self.assertIsInstance(binding, C01TaskBinding)
        self.assertEqual(binding.goal, goal)
        self.assertEqual(binding.goal.qualitative_constraints,
                         ("keep the other block undisturbed",))
        self.assertEqual(binding.task.task_id, goal.goal_id)
        self.assertEqual(binding.task.revision, goal.revision)
        self.assertEqual(binding.task.context_refs, goal.context_refs)
        self.assertEqual(binding.task.requested_embodiment_id, "hand-1")
        self.assertEqual(binding.task.permission_revision, "shadow-only/no-dispatch")
        self.assertNotIn("dispatch", C01TaskBinding.model_fields)
        self.assertNotIn("authorized", C01TaskBinding.model_fields)

    def test_c01_binding_rechecks_route_and_rejects_unselected_or_stale_capability(self):
        goal = GoalRequest(
            goal_id="inspect-1", revision="goal/v1", objective="inspect shelf-2",
            required_operations=frozenset({"INSPECT"}),
            required_resources=frozenset({"camera"}),
        )
        drone = capability("drone-1", {"INSPECT"}, {"camera"})
        rover = capability("rover-1", {"INSPECT"}, {"camera"})
        with self.assertRaisesRegex(ValueError, "exactly one"):
            bind_selected_route_to_c01(
                goal, route_goal(goal, (drone, rover)), drone,
                constraints_revision="constraints/v1", issuer_id="operator-1",
                permission_revision="shadow-only/no-dispatch",
            )

        selected = route_goal(goal, (drone,))
        unavailable = capability("drone-1", {"INSPECT"}, {"camera"}, available=set())
        with self.assertRaisesRegex(ValueError, "no longer routable"):
            bind_selected_route_to_c01(
                goal, selected, unavailable,
                constraints_revision="constraints/v1", issuer_id="operator-1",
                permission_revision="shadow-only/no-dispatch",
            )
        with self.assertRaises(ValidationError):
            bind_selected_route_to_c01(
                goal, selected, drone,
                constraints_revision="constraints/v1", issuer_id="operator-1",
                permission_revision="",
            )


if __name__ == "__main__":
    unittest.main()
