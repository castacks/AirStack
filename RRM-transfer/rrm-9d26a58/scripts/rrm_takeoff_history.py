#!/usr/bin/env python3
"""Print a read-only takeoff repeatability summary from command evidence."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))

from rrm.command_history import summarize_takeoffs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--command-root", type=Path,
        default=Path("/root/AirStack/.rrm-artifacts/command-requests"),
    )
    args = parser.parse_args()
    print(json.dumps(summarize_takeoffs(args.command_root), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
