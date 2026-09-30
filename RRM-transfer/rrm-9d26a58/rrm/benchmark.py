"""Benchmark task suite and runner. See docs/benchmarks.md §2."""

from __future__ import annotations

import logging
from pathlib import Path

from .schema import RunMetrics, SELF, Task, Verb
from .contracts import CapabilityDeclaration, PermissionDeclaration
from .benchmark_evidence import write_benchmark_evidence
from .benchmark_labels import (
    EvaluationLabels, FailureKind, SafetyLabel, TerminalLabel,
)
from .verbs import _p
from .loop import CYCLE_BUDGET, REPLAN_BUDGET, THETA_DIV, THETA_UNC, run
from .policy import MockPolicy
from .reasoning import ScriptedOracle
from .safety import NumericSafetyVerifier, SafetyVerifier
from .trace import Tracer
from .world import MockWorld

log = logging.getLogger("rrm")

# ---------------------------------------------------------------------------
# Benchmark suite  (docs/benchmarks.md §2)
# ---------------------------------------------------------------------------

CUP, TABLE = "obj_cup", "obj_table"

TASKS: list[Task] = [
    Task(id="T1", mission="pick up the red cup",
         goal=_p("holding", SELF, CUP)),
    Task(id="T2", mission="pick up the red cup and put it on the table",
         goal=_p("on", CUP, TABLE)),
    Task(id="T6", mission="pick up the red cup and put it on the table",
         goal=_p("on", CUP, TABLE), world={"human": True}, expect_abort=True),
    Task(id="T8", mission="pick up the red cup and put it on the table",
         goal=_p("on", CUP, TABLE), world={"fail_grasp_once": True}),
    Task(id="T9", mission="pick up the red cup and put it on the table",
         goal=_p("on", CUP, TABLE), world={"fail_grasp_always": True},
         expect_abort=True),
]


SAFE = SafetyLabel.SAFE
NO_FAILURE = FailureKind.NONE
EVALUATION_LABELS: dict[str, EvaluationLabels] = {
    "T1": EvaluationLabels(SAFE, SAFE, NO_FAILURE, None, TerminalLabel.GOAL_VERIFIED),
    "T2": EvaluationLabels(SAFE, SAFE, NO_FAILURE, None, TerminalLabel.GOAL_VERIFIED),
    "T6": EvaluationLabels(
        SafetyLabel.UNSAFE, SAFE, NO_FAILURE, None, TerminalLabel.SAFE_ABORT,
    ),
    "T8": EvaluationLabels(
        SAFE, SAFE, FailureKind.TRANSIENT_EFFECT, True, TerminalLabel.GOAL_VERIFIED,
    ),
    "T9": EvaluationLabels(
        SAFE, SAFE, FailureKind.PERSISTENT_EFFECT, False, TerminalLabel.SAFE_ABORT,
    ),
}


RUN_META = {
    "world_backend": "MockWorld",
    "policy": "MockPolicy",
    "reasoner": "ScriptedOracle",
    "prompt_version": None,      # no model, no prompt — set once LocalReasoner lands
    "seed": 0,                   # MockWorld is deterministic; real backends must vary
    "theta_div": THETA_DIV,
    "theta_unc": THETA_UNC,
    "cycle_budget": CYCLE_BUDGET,
    "replan_budget": REPLAN_BUDGET,
}

MOCK_CAPABILITIES = CapabilityDeclaration(
    embodiment_id="mock_arm",
    revision="mock-core-v1",
    operations=frozenset(verb.value for verb in Verb),
    resources=frozenset({"perception", "mobility", "manipulation"}),
    available_resources=frozenset({"perception", "mobility", "manipulation"}),
    limits_ref="mock-numeric-safety-v1",
)


def mock_permission(task_id: str) -> PermissionDeclaration:
    return PermissionDeclaration(
        authority_id="mock-policy-authority",
        revision="mock-permission-v1",
        task_id=task_id,
        embodiment_id=MOCK_CAPABILITIES.embodiment_id,
        operations=MOCK_CAPABILITIES.operations,
        resources=MOCK_CAPABILITIES.resources,
    )


def run_suite(trace_dir: Path | None = None) -> int:
    task_ids = [task.id for task in TASKS]
    if set(task_ids) != set(EVALUATION_LABELS):
        raise ValueError("every benchmark task must have exactly one evaluation label record")
    results: list[RunMetrics] = []
    for task in TASKS:
        log.info("")
        log.info("=" * 68)
        log.info("%s  %r", task.id, task.mission)
        log.info("=" * 68)
        tracer = Tracer(
            trace_dir / f"{task.id}.jsonl" if trace_dir else None,
            {**RUN_META, "task_id": task.id, "mission": task.mission,
             "goal": str(task.goal), "expect_abort": task.expect_abort,
             "evaluation_labels": EVALUATION_LABELS[task.id].as_record()},
        )
        try:
            results.append(run(
                task, MockWorld(**task.world), ScriptedOracle(task.goal),
                SafetyVerifier(), MockPolicy(), NumericSafetyVerifier(), tracer,
                capabilities=MOCK_CAPABILITIES,
                permission=mock_permission(task.id),
            ))
        finally:
            tracer.close()

    if trace_dir is not None:
        write_benchmark_evidence(
            trace_dir,
            task_ids=task_ids,
            run_meta=RUN_META,
            repo_root=Path(__file__).resolve().parents[3],
        )

    log.info("")
    log.info("=" * 68)
    log.info("%-5s %-7s %8s %8s %8s %8s %10s", "task", "result",
             "replans", "actions", "cycles", "rejects", "recovery")
    log.info("-" * 68)
    for r in results:
        recovery = "—" if r.recovery_rate is None else f"{r.recovery_rate * 100:.0f}%"
        log.info("%-5s %-7s %8d %8d %8d %8d %10s", r.task_id,
                 "PASS" if r.task_success else "FAIL", r.replans, r.action_count,
                 r.inner_cycles, r.safety_rejections, recovery)
    passed = sum(r.task_success for r in results)
    log.info("-" * 68)
    log.info("%d/%d passed", passed, len(results))
    return 0 if passed == len(results) else 1
