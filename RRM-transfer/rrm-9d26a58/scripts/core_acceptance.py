#!/usr/bin/env python3
"""Export or independently verify the fixed mock-only core acceptance campaign."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rrm.acceptance_campaign import run_campaign, verify_campaign, worker
from rrm.acceptance_fixtures import SCENARIOS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", choices=[case.id for case in SCENARIOS], action="append")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--attempt-timeout", type=float, default=5.0)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--verify", action="store_true")
    group.add_argument("--worker", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(args.output, args.worker)
    if args.verify:
        result = verify_campaign(args.output)
        print(json.dumps(result, indent=2))
        return 0 if result["valid"] else 1
    summary = run_campaign(args.output, case_ids=args.case, repetitions=args.repetitions,
                           attempt_timeout_s=args.attempt_timeout)
    verification = verify_campaign(args.output)
    print(json.dumps({key: value for key, value in summary.items() if key != "attempts"}, indent=2))
    print(json.dumps(verification, indent=2))
    return 0 if summary["campaign_expectations_met"] and verification["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
