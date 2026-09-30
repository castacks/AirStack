"""Core predicates preserve unknown evidence under both polarities."""

import unittest

from rrm.contracts import Truth
from rrm.reasoning import ScriptedOracle
from rrm.safety import SafetyVerifier
from rrm.schema import AbstractAction, Relation, RobotState, Verb, WorldObject, WorldState
from rrm.verbs import _p, holds, predicate_truth


def state(*, properties=None, relations=(), relations_complete=False) -> WorldState:
    return WorldState(
        objects=[
            WorldObject(id="item", cls="object", pose=(0.2, 0.0, 0.0),
                        properties=properties or {}),
            WorldObject(id="surface", cls="surface", pose=(0.4, 0.0, 0.0)),
        ],
        relations=list(relations),
        relations_complete=relations_complete,
    )


class PredicateTruthTests(unittest.TestCase):
    def test_missing_and_unsupported_evidence_stays_unknown_when_negated(self) -> None:
        ws = state()
        for predicate in (
            _p("exists", "missing"),
            _p("open", "item"),
            _p("localized", "missing"),
            _p("unsupported", "item"),
        ):
            self.assertIs(predicate_truth(predicate, ws), Truth.UNKNOWN)
            negated = predicate.model_copy(update={"negated": True})
            self.assertIs(predicate_truth(negated, ws), Truth.UNKNOWN)
            self.assertFalse(holds(predicate, ws))
            self.assertFalse(holds(negated, ws))

    def test_explicit_boolean_property_supports_both_polarities(self) -> None:
        for value, positive, negative in (
            (True, Truth.TRUE, Truth.FALSE),
            (False, Truth.FALSE, Truth.TRUE),
        ):
            ws = state(properties={"open": value})
            self.assertIs(predicate_truth(_p("open", "item"), ws), positive)
            self.assertIs(predicate_truth(_p("open", "item", negated=True), ws), negative)

    def test_relation_absence_requires_declared_complete_snapshot(self) -> None:
        positive = _p("on", "item", "surface")
        negative = _p("on", "item", "surface", negated=True)

        incomplete = state()
        self.assertIs(predicate_truth(positive, incomplete), Truth.UNKNOWN)
        self.assertIs(predicate_truth(negative, incomplete), Truth.UNKNOWN)

        complete = state(relations_complete=True)
        self.assertIs(predicate_truth(positive, complete), Truth.FALSE)
        self.assertIs(predicate_truth(negative, complete), Truth.TRUE)

        observed = state(
            relations=[Relation(subject="item", predicate="on", obj="surface")],
        )
        self.assertIs(predicate_truth(positive, observed), Truth.TRUE)
        self.assertIs(predicate_truth(negative, observed), Truth.FALSE)

    def test_contradictory_state_cannot_establish_a_fact(self) -> None:
        inconsistent_robot = state().model_copy(update={
            "robot": RobotState(gripper="holding", holding=None),
        })
        self.assertIs(
            predicate_truth(_p("gripper_empty", "$self"), inconsistent_robot),
            Truth.UNKNOWN,
        )

        dangling_relation = state(
            relations=[Relation(subject="missing", predicate="on", obj="surface")],
            relations_complete=True,
        )
        self.assertIs(
            predicate_truth(_p("on", "missing", "surface"), dangling_relation),
            Truth.UNKNOWN,
        )

    def test_unknown_and_false_preconditions_are_attributable(self) -> None:
        verifier = SafetyVerifier()
        open_action = AbstractAction(id="open", verb=Verb.OPEN, targets=["item"])
        unknown = verifier.verify(open_action, state())
        self.assertEqual(unknown.verdict, "FAIL")
        self.assertIn("precondition_unknown", [v.check for v in unknown.violations])

        close_action = AbstractAction(id="close", verb=Verb.CLOSE, targets=["item"])
        false = verifier.verify(close_action, state(properties={"open": False}))
        self.assertEqual(false.verdict, "FAIL")
        self.assertIn("precondition_false", [v.check for v in false.violations])

    def test_oracle_does_not_treat_unknown_negative_goal_as_achieved(self) -> None:
        goal = _p("open", "item", negated=True)
        with self.assertRaisesRegex(ValueError, "oracle cannot achieve"):
            ScriptedOracle(goal).plan("ensure item is closed", state())


if __name__ == "__main__":
    unittest.main()
