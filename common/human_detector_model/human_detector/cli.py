"""JSONL interface: one {key, observation} object in, one scored object out."""

import argparse
import json
from pathlib import Path
import sys

from .model import Detector, DetectorConfig, Observation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--profile", default="baseline", choices=[
        "baseline", "stronger", "weaker", "small_target", "occlusion_tolerant"])
    mode.add_argument("--config", help="path to a custom JSON configuration")
    parser.add_argument("--input", default="-", help="JSONL file or - for stdin")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--write-config", help="write the selected configuration here and exit")
    args = parser.parse_args()
    source = None
    line_number = 0
    try:
        config = DetectorConfig.load(args.config) if args.config else DetectorConfig.profile(args.profile)
        if args.write_config:
            Path(args.write_config).write_text(json.dumps(config.to_dict(), indent=2) + "\n")
            return 0
        detector = Detector(config)
        source = sys.stdin if args.input == "-" else open(args.input)
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            if not isinstance(row, dict) or set(row) != {"key", "observation"}:
                raise ValueError("each row must contain exactly key and observation")
            result = detector.detect(Observation.from_dict(row["observation"]), seed=args.seed, key=row["key"])
            print(json.dumps({"key": row["key"], **result}, allow_nan=False))
    except (OSError, ValueError) as exc:
        where = f" at line {line_number}" if line_number else ""
        print(f"human-detector{where}: {exc}", file=sys.stderr)
        return 2
    finally:
        if source is not None and source is not sys.stdin:
            source.close()
    return 0
