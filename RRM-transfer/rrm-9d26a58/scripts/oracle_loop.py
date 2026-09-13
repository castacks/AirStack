#!/usr/bin/env python3
"""CLI for the RRM reference loop.

Runs the benchmark suite, or a single episode, against the mock world. No GPU,
no Isaac Sim, no model weights and no API key required — which is what makes it
the CI target (docs/benchmarks.md §5).

    python3 scripts/oracle_loop.py --suite --trace-dir traces/
    python3 scripts/oracle_loop.py --fail-grasp
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rrm import (  # noqa: E402
    MockPolicy, MockWorld, NumericSafetyVerifier, SafetyVerifier, ScriptedOracle,
    Task, run, run_suite,
)
from rrm.benchmark import CUP, TABLE  # noqa: E402
from rrm.verbs import _p  # noqa: E402

log = logging.getLogger("rrm")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fail-grasp", action="store_true",
                    help="make the first grasp slip, to exercise divergence + replan")
    ap.add_argument("--human", action="store_true",
                    help="place a person near the table, to exercise safety rejection")
    ap.add_argument("--suite", action="store_true",
                    help="run the benchmark suite and report metrics")
    ap.add_argument("--trace-dir", type=Path, default=None,
                    help="write per-episode JSONL traces here (see docs/benchmarks.md §5)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if args.suite:
        return run_suite(args.trace_dir)

    task = Task(id="adhoc", mission="pick up the red cup and put it on the table",
                goal=_p("on", CUP, TABLE),
                expect_abort=args.human)
    log.info("MISSION: %s", task.mission)
    world = MockWorld(fail_grasp_once=args.fail_grasp, human=args.human)
    m = run(task, world, ScriptedOracle(task.goal), SafetyVerifier(),
            MockPolicy(), NumericSafetyVerifier())
    return 0 if m.task_success else 1


if __name__ == "__main__":
    raise SystemExit(main())
