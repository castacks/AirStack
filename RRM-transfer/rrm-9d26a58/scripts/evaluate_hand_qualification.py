#!/usr/bin/env python3
"""Evaluate an isolated hand-controller probe without adding execution authority."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))

from rrm.hand_qualification import evaluate_hand_probe_file


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-json", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite qualification evidence: {args.output}")
    result = evaluate_hand_probe_file(args.probe_json)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))
    return 0 if result["ready_for_single_bounded_contact_trial"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
