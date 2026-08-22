#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Compare two stacks' observed wiring (RFC #379 §3: ``airstack stack diff``).

Copy-drift between stack folders is the accepted trade for folder
independence; this tool bounds it by diffing the stacks' *generated*
``wiring.md`` trailers (normalized, CI-verified observed graphs), so the diff
shows topology differences — nodes/edges/topics added or removed, QoS and
type mismatches — never XML formatting noise.

Thin wrapper over ``tests/wiring_snapshot.py``'s ``extract_graph_from_md`` +
``diff_graphs`` (the same machinery the CI drift check uses).

CLI::

    stack_diff.py <stack_a> <stack_b> [--stacks-dir DIR] [--json]

Stack arguments are stack names under ``stacks/`` or paths to stack folders /
``wiring.md`` files. Exit 0 when the graphs are identical, 1 when they
differ, 2 when a wiring.md is missing or unreadable.
"""
import argparse
import json
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
_TESTS_DIR = _REPO / "tests"
if str(_TESTS_DIR) not in sys.path:
    sys.path.insert(0, str(_TESTS_DIR))

import wiring_snapshot as ws  # noqa: E402  (tests/wiring_snapshot.py, stdlib-only)


def resolve_wiring_path(spec, stacks_dir):
    """Resolve a stack name / stack folder / wiring.md path to a wiring.md path."""
    path = Path(spec)
    if path.is_file():
        return path
    if path.is_dir():
        return path / "wiring.md"
    return Path(stacks_dir) / spec / "wiring.md"


def load_graph(spec, stacks_dir):
    path = resolve_wiring_path(spec, stacks_dir)
    if not path.is_file():
        raise FileNotFoundError(
            f"{spec}: no wiring.md at {path} — generate it first "
            "(airstack test -m wiring --stack <name>, or airstack doctor "
            "--snapshot on a real bring-up)"
        )
    return ws.extract_graph_from_md(path.read_text(encoding="utf-8")), path


def _print_section(title, items, limit=50):
    print(f"  {title}: {len(items)}")
    for item in items[:limit]:
        print(f"    {item}")
    if len(items) > limit:
        print(f"    ... and {len(items) - limit} more")


def summarize(name_a, name_b, verdict):
    """Human summary of a diff_graphs verdict (a→b: 'missing' = only in a)."""
    if verdict["identical"]:
        print(f"{name_a} and {name_b} have identical wiring "
              "(normalized observed graphs match).")
        return
    print(f"Wiring differs between {name_a} (A) and {name_b} (B):")
    _print_section(f"nodes only in {name_a} (A)", verdict["missing_nodes"])
    _print_section(f"nodes only in {name_b} (B)", verdict["extra_nodes"])
    _print_section(f"edges only in {name_a} (A)", verdict["missing_edges"])
    _print_section(f"edges only in {name_b} (B)", verdict["extra_edges"])
    _print_section(
        "QoS mismatches on shared edges",
        [f'{m["edge"]}: A={m["expected"]} B={m["observed"]}'
         for m in verdict["qos_mismatches"]],
    )
    _print_section(
        "type mismatches on shared topics",
        [f'{m["topic"]}: A={m["expected"]} B={m["observed"]}'
         for m in verdict["type_mismatches"]],
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Diff two stacks' generated wiring.md graphs "
                    "(RFC #379 §3, airstack stack diff).",
    )
    parser.add_argument("stack_a", help="stack name, stack folder, or wiring.md path")
    parser.add_argument("stack_b", help="stack name, stack folder, or wiring.md path")
    parser.add_argument(
        "--stacks-dir", default=str(_REPO / "stacks"),
        help="where stack names resolve (default: <repo>/stacks)",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="print the raw diff_graphs verdict JSON instead of the summary",
    )
    args = parser.parse_args(argv)

    try:
        graph_a, path_a = load_graph(args.stack_a, args.stacks_dir)
        graph_b, path_b = load_graph(args.stack_b, args.stacks_dir)
    except (FileNotFoundError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    verdict = ws.diff_graphs(graph_a, graph_b)
    if args.json:
        print(json.dumps(verdict, indent=2))
    else:
        print(f"A: {path_a}")
        print(f"B: {path_b}")
        summarize(args.stack_a, args.stack_b, verdict)
    return 0 if verdict["identical"] else 1


if __name__ == "__main__":
    sys.exit(main())
