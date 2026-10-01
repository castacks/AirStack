"""Core replanning must converge without scene-specific retry rules."""

import json
import tempfile
import unittest
from pathlib import Path

from rrm.loop import run
from rrm.benchmark import (
    MOCK_CAPABILITIES, MOCK_NUMERIC_PROFILE, SyntheticApprovalProvider,
    mock_admission, mock_permission,
)
from rrm.policy import MockPolicy
from rrm.reasoning import ScriptedOracle
from rrm.safety import NumericSafetyVerifier, SafetyVerifier
from rrm.schema import AbstractAction, Divergence, Task, TaskGraph, Verb, WorldState
from rrm.trace import Tracer
from rrm.verbs import _p
from rrm.world import MockWorld


CUP = "obj_cup"
TABLE = "obj_table"


class AlternativeReasoner:
    """Return one rejected proposal, then a materially different valid plan."""

    name = "alternative_test_reasoner"

    def __init__(self) -> None:
        self.oracle = ScriptedOracle(_p("on", CUP, TABLE))

    def plan(self, mission: str, ws: WorldState) -> TaskGraph:
        return TaskGraph(
            mission_id="m0",
            mission_text=mission,
            nodes=[AbstractAction(id="bad", verb=Verb.GRASP, targets=["missing_target"])],
        )

    def replan(self, mission: str, ws: WorldState, graph: TaskGraph,
               div: Divergence) -> TaskGraph:
        replacement = self.oracle.plan(mission, ws)
        return replacement.model_copy(update={"version": graph.version + 1})


class ReplanConvergenceTests(unittest.TestCase):
    def test_unchanged_rejected_action_aborts_after_one_replan(self) -> None:
        task = Task(
            id="unsafe-context",
            mission="place the cup on the table",
            goal=_p("on", CUP, TABLE),
            expect_abort=True,
        )
        world = MockWorld(human=True)
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.jsonl"
            tracer = Tracer(trace_path, {"task_id": task.id})
            try:
                metrics = run(
                    task, world, ScriptedOracle(task.goal), SafetyVerifier(),
                    MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE), tracer,
                    capabilities=MOCK_CAPABILITIES,
                    permission=mock_permission(task.id),
                    approval=SyntheticApprovalProvider(),
                    admission=mock_admission(),
                )
            finally:
                tracer.close()

            events = [json.loads(line) for line in trace_path.read_text().splitlines()]

        convergence = [event for event in events
                       if event["kind"] == "replan_convergence"]
        self.assertTrue(metrics.task_success)
        self.assertTrue(metrics.aborted)
        self.assertEqual(metrics.replans, 1)
        self.assertEqual(metrics.safety_rejections, 1)
        self.assertEqual(metrics.action_count, 0)
        self.assertEqual(len(convergence), 1)
        self.assertEqual(convergence[0]["reason"], "UNCHANGED_REJECTED_ACTION")
        self.assertEqual(convergence[0]["rejected_action_id"], "a0")
        self.assertEqual(convergence[0]["replacement_action_id"], "a0")
        self.assertEqual(convergence[0]["rejected_plan_version"], 0)
        self.assertEqual(convergence[0]["replacement_plan_version"], 1)
        self.assertEqual(convergence[0]["rejected_action"], {
            "verb": "GRASP", "targets": [CUP], "params": {},
        })
        self.assertEqual(convergence[0]["replacement_action"],
                         convergence[0]["rejected_action"])
        self.assertEqual(sum(event["kind"] == "safety1" for event in events), 1)

    def test_materially_different_replan_continues_to_goal(self) -> None:
        task = Task(
            id="alternative-plan",
            mission="place the cup on the table",
            goal=_p("on", CUP, TABLE),
        )
        metrics = run(
            task, MockWorld(), AlternativeReasoner(), SafetyVerifier(),
            MockPolicy(), NumericSafetyVerifier(MOCK_NUMERIC_PROFILE),
            capabilities=MOCK_CAPABILITIES,
            permission=mock_permission(task.id),
            approval=SyntheticApprovalProvider(),
            admission=mock_admission(),
        )

        self.assertTrue(metrics.task_success)
        self.assertFalse(metrics.aborted)
        self.assertEqual(metrics.replans, 1)
        self.assertEqual(metrics.safety_rejections, 1)
        self.assertEqual(metrics.action_count, 2)


if __name__ == "__main__":
    unittest.main()
