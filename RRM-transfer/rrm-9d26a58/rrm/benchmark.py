"""Benchmark task suite and runner. See docs/benchmarks.md §2."""

from __future__ import annotations

import logging
from pathlib import Path

from .schema import RunMetrics, SELF, Task
from .verbs import _p
from .loop import CYCLE_BUDGET, REPLAN_BUDGET, THETA_DIV, run
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


RUN_META = {
    "world_backend": "MockWorld",
    "policy": "MockPolicy",
    "reasoner": "ScriptedOracle",
    "prompt_version": None,      # no model, no prompt — set once LocalReasoner lands
    "seed": 0,                   # MockWorld is deterministic; real backends must vary
    "theta_div": THETA_DIV,
    "cycle_budget": CYCLE_BUDGET,
    "replan_budget": REPLAN_BUDGET,
}


def run_suite(trace_dir: Path | None = None) -> int:
    results: list[RunMetrics] = []
    for task in TASKS:
        log.info("")
        log.info("=" * 68)
        log.info("%s  %r", task.id, task.mission)
        log.info("=" * 68)
        tracer = Tracer(
            trace_dir / f"{task.id}.jsonl" if trace_dir else None,
            {**RUN_META, "task_id": task.id, "mission": task.mission,
             "goal": str(task.goal), "expect_abort": task.expect_abort},
        )
        try:
            results.append(run(
                task, MockWorld(**task.world), ScriptedOracle(task.goal),
                SafetyVerifier(), MockPolicy(), NumericSafetyVerifier(), tracer,
            ))
        finally:
            tracer.close()

    log.info("")
    log.info("=" * 68)
    log.info("%-5s %-7s %8s %8s %8s %8s %10s", "task", "result",
             "replans", "actions", "cycles", "unsafe", "recovery")
    log.info("-" * 68)
    for r in results:
        log.info("%-5s %-7s %8d %8d %8d %8d %9.0f%%", r.task_id,
                 "PASS" if r.task_success else "FAIL", r.replans, r.action_count,
                 r.inner_cycles, r.unsafe_actions, r.recovery_rate * 100)
    passed = sum(r.task_success for r in results)
    log.info("-" * 68)
    log.info("%d/%d passed", passed, len(results))
    return 0 if passed == len(results) else 1
