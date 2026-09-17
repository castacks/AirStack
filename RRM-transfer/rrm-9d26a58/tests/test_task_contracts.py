"""C01/C04/C05 records are proposal-only and reject stale or invalid plans."""

import unittest

from pydantic import ValidationError

from rrm.schema import AbstractAction, Predicate, Verb
from rrm.task_contracts import (
    IntentStatus, Interaction, InteractionStatus, InteractionType, PlanProposal,
    PlannedAction, ReasoningResult, TaskRequest,
)


def intent(**changes):
    values = dict(task_id="task-1", task_revision="task/v1", state_revision="state/v1",
                  capability_revision="hand/v1", status=IntentStatus.READY,
                  grounded_goal=Predicate(name="holding", subject="$self", obj="cup-1"))
    values.update(changes)
    return ReasoningResult(**values)


def action(action_id="a1", dependencies=()):
    return PlannedAction(
        action=AbstractAction(id=action_id, verb=Verb.GRASP, targets=["cup-1"]),
        dependencies=dependencies, semantics_revision="verbs/v1",
        feasibility_ref=f"feasible/{action_id}", expected_effect_window_revision="window/v1",
    )


def plan(actions=(action(),), **changes):
    values = dict(plan_id="plan-1", version=0, task_id="task-1", task_revision="task/v1",
                  intent=intent(), state_revision="state/v1", capability_revision="hand/v1",
                  actions=actions, recovery_budget=2)
    values.update(changes)
    return PlanProposal(**values)


class TaskContractTests(unittest.TestCase):
    def test_task_and_scoped_interaction_are_immutable(self):
        request = TaskRequest(task_id="task-1", revision="task/v1", objective="pick cup",
                              constraints_revision="constraints/v1", issuer_id="operator-1",
                              permission_revision="permission/v1", requested_embodiment_id="hand-1")
        self.assertEqual(request.task_id, "task-1")
        with self.assertRaises(ValidationError):
            request.objective = "changed"
        interaction = Interaction(interaction_id="i1", task_id="task-1", task_revision="task/v1",
                                  kind=InteractionType.CLARIFICATION, issue="which cup?",
                                  scope_revision="task/v1")
        self.assertIs(interaction.status, InteractionStatus.PENDING)
        with self.assertRaises(ValidationError):
            Interaction(interaction_id="i2", task_id="task-1", task_revision="task/v1",
                        kind=InteractionType.APPROVAL, issue="approve", status=InteractionStatus.GRANTED,
                        scope_revision="task/v1")

    def test_ambiguity_cannot_become_an_executable_intent(self):
        unresolved = intent(status=IntentStatus.NEEDS_CLARIFICATION, grounded_goal=None,
                            ambiguity_refs=("two_drinkable_objects",))
        self.assertIs(unresolved.status, IntentStatus.NEEDS_CLARIFICATION)
        with self.assertRaises(ValidationError):
            plan(intent=unresolved)

    def test_plan_rejects_context_mismatch_and_invalid_semantics(self):
        with self.assertRaises(ValidationError):
            plan(state_revision="state/v2")
        with self.assertRaises(ValidationError):
            PlannedAction(action=AbstractAction(id="bad", verb=Verb.PLACE, targets=["cup-1"]),
                          semantics_revision="verbs/v1", feasibility_ref="f",
                          expected_effect_window_revision="window/v1")

    def test_plan_rejects_duplicate_missing_and_cyclic_dependencies(self):
        with self.assertRaises(ValidationError):
            plan(actions=(action("a1"), action("a1")))
        with self.assertRaises(ValidationError):
            plan(actions=(action("a1", ("missing",)),))
        with self.assertRaises(ValidationError):
            plan(actions=(action("a1", ("a2",)), action("a2", ("a1",))))

    def test_valid_plan_has_no_execution_authority_field(self):
        proposal = plan()
        self.assertEqual(proposal.actions[0].action.verb, Verb.GRASP)
        self.assertNotIn("dispatch", PlanProposal.model_fields)
        self.assertNotIn("authorization", PlanProposal.model_fields)


if __name__ == "__main__":
    unittest.main()
