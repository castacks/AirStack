#!/usr/bin/env python3
"""Export or verify a paired report from two existing mock acceptance campaigns."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rrm.acceptance_campaign import write_json
from rrm.campaign_comparison import ComparisonError, compare_campaigns, verify_comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New JSON report outside both input bundles")
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    if args.verify:
        result = verify_comparison(args.reference, args.candidate, args.output)
        print(json.dumps(result, indent=2))
        return 0 if result["valid"] else 1
    try:
        if any(args.output.resolve().is_relative_to(directory.resolve())
               for directory in (args.reference, args.candidate)):
            raise ComparisonError("report_must_be_outside_input_bundles")
        if args.output.exists():
            raise ComparisonError("report_already_exists")
        report = compare_campaigns(args.reference, args.candidate)
        write_json(args.output, report)
    except (OSError, ValueError) as error:
        print(json.dumps({"valid": False, "findings": [str(error)]}, indent=2))
        return 1
    print(json.dumps({key: value for key, value in report.items() if key not in {"pairs", "arms"}}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
