#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""``airstack doctor`` — observe-and-report health checks (RFC #379 §4).

Default (compose-time) mode runs, in order:

1. module manifests valid (``tools/validate_module.py``) — reports;
2. overlay integrity (``tools/module_overlay.py --check``) — reports;
3. **hard gate #1**: module dep conflicts
   (``tools/compose_module_layers.py --check-conflicts``, RFC #379 §6);
4. stack folder anatomy, incl. the split-stack ⇒ ``bridge.yaml`` rule — reports;
5. **hard gate #2**: control-setpoint / trajectory-group names in any
   ``bridge.yaml`` (``tools/gen_dds_router.py --check``, RFC #380 §2).

Only the two enumerated hard gates set a non-zero exit in default mode;
everything else is reported and stepped aside from. Doctor never edits
anything (``--snapshot`` writes exactly one file — the stack's ``wiring.md`` —
because that file is *defined* as an observed artifact).

Modes::

    doctor                         # compose-time battery (above)
    doctor --live  [--stack NAME]  # diff the RUNNING graph vs the stack's
                                   #   committed wiring.md (exit 1 on drift)
                                   #   + safety-floor publisher scan (WARN;
                                   #   --strict makes those fatal)
    doctor --snapshot [--stack NAME]  # same capture, WRITTEN to the stack's
                                   #   wiring.md with hardware provenance
                                   #   ('observed on <host>, <date>, <sha> —
                                   #   unverified-in-CI')

``--stack`` is inferred from ``AIRSTACK_STACK_DIR`` when omitted (the env var
``airstack up ...stack <name>`` exports).
"""
import argparse
import sys
from pathlib import Path

try:
    from .checks import (  # noqa: F401  (re-exported for tests)
        OK, WARN, FAIL,
        CheckResult,
        capture_live_graph,
        check_bridge_gates,
        check_layer_conflicts,
        check_module_manifests,
        check_overlay,
        check_safety_floor,
        check_stack_layout,
        infer_stack,
        run_compose_time_checks,
        run_live,
        run_snapshot,
    )
except ImportError:  # executed as a script: python3 tools/doctor/__init__.py
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from checks import (  # noqa: F401
        OK, WARN, FAIL,
        CheckResult,
        capture_live_graph,
        check_bridge_gates,
        check_layer_conflicts,
        check_module_manifests,
        check_overlay,
        check_safety_floor,
        check_stack_layout,
        infer_stack,
        run_compose_time_checks,
        run_live,
        run_snapshot,
    )

_STATUS_LABEL = {OK: "[ OK ]", WARN: "[WARN]", FAIL: "[FAIL]"}


def _print_results(results, log=print):
    for result in results:
        gate = " (hard gate)" if result.hard else ""
        log(f"{_STATUS_LABEL[result.status]} {result.name}{gate}")
        for message in result.messages:
            for line in message.splitlines():
                log(f"       {line}")


def run_default(root, stack=None, log=print):
    """Compose-time battery; exit 1 iff a hard gate failed."""
    results = run_compose_time_checks(root, stack=stack)
    _print_results(results, log=log)
    gated = [r for r in results if r.gates]
    warned = [r for r in results if r.status != OK and not r.gates]
    if gated:
        log(f"doctor: {len(gated)} hard-gate failure(s) "
            f"({', '.join(r.name for r in gated)}) — see RFC #379 §4 for the "
            "two enumerated gates")
        return 1
    if warned:
        log(f"doctor: findings reported in {len(warned)} check(s) — doctor "
            "observes and steps aside (exit 0)")
    else:
        log("doctor: all checks clean")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="airstack doctor",
        description="Observe-and-report health checks for an AirStack "
                    "checkout or a running stack (RFC #379 §4).",
    )
    parser.add_argument(
        "--live", action="store_true",
        help="diff the running ROS graph against the stack's committed "
             "wiring.md (exit 1 on drift) and scan for unblessed "
             "control-setpoint publishers")
    parser.add_argument(
        "--snapshot", action="store_true",
        help="capture the running graph and WRITE it as the stack's "
             "wiring.md with an unverified-in-CI provenance line "
             "(hardware bring-up path)")
    parser.add_argument(
        "--stack",
        help="stack name under stacks/ (default: inferred from "
             "AIRSTACK_STACK_DIR)")
    parser.add_argument(
        "--strict", action="store_true",
        help="--live only: safety-floor warnings become fatal (exit 1)")
    parser.add_argument(
        "--project-root",
        default=str(Path(__file__).resolve().parent.parent.parent),
        help="AirStack checkout root (default: the repo containing this tool)")
    args = parser.parse_args(argv)

    root = Path(args.project_root)
    if args.live and args.snapshot:
        parser.error("--live and --snapshot are exclusive modes")
    try:
        if args.snapshot:
            return run_snapshot(root, args.stack)
        if args.live:
            return run_live(root, args.stack, strict=args.strict)
    except FileNotFoundError as exc:  # docker binary missing
        print(f"[doctor] cannot capture the running graph: {exc}")
        return 1
    except RuntimeError as exc:  # no containers / identity resolution failed
        print(f"[doctor] {exc}")
        return 1
    return run_default(root, stack=args.stack)


if __name__ == "__main__":
    sys.exit(main())
