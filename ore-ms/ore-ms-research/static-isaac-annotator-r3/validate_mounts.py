"""Check the stationary R3 fixture's bind mounts from Docker inspection JSON."""

import argparse
from collections import Counter
import json
from pathlib import Path, PurePosixPath


def absolute_path(value):
    return (isinstance(value, str) and value.startswith("/") and not value.startswith("//")
            and "\0" not in value and str(PurePosixPath(value)) == value
            and ".." not in PurePosixPath(value).parts)


def overlaps(left, right):
    left, right = PurePosixPath(left), PurePosixPath(right)
    return left.is_relative_to(right) or right.is_relative_to(left)


def validate_mounts(records, input_source, capture_source, *, exact=False):
    required = {
        "/capture-input": ("bind", input_source, False),
        "/capture-output": ("bind", capture_source, True),
    }
    if not isinstance(records, list):
        raise ValueError("expected a JSON array of Docker mounts")

    counts = Counter()
    for index, record in enumerate(records):
        if not isinstance(record, dict) or not {"Type", "Source", "Destination", "RW"} <= record.keys():
            raise ValueError(f"incomplete mount record at index {index}")
        kind, source, destination, writable = (record[key] for key in ("Type", "Source", "Destination", "RW"))
        if (kind not in ("bind", "volume", "tmpfs") or type(writable) is not bool
                or not absolute_path(destination)
                or not (absolute_path(source) or (kind == "tmpfs" and source == ""))):
            raise ValueError(f"invalid mount record at index {index}")

        counts[destination] += 1
        if counts[destination] > 1:
            raise ValueError(f"duplicate mount destination {destination}")
        if destination in required:
            if (kind, source, writable) != required[destination]:
                raise ValueError(f"incorrect type, source or access for {destination}")
        elif (any(overlaps(destination, target) for target in required)
              or (source and any(overlaps(source, entry[1]) for entry in required.values()))):
            raise ValueError(f"conflicting mount at {destination}")
        elif exact:
            raise ValueError(f"unexpected mount at {destination} in the isolated fixture")

    missing = required.keys() - counts.keys()
    if missing:
        raise ValueError(f"missing required mount(s): {', '.join(sorted(missing))}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inspection", type=Path)
    parser.add_argument("input_source")
    parser.add_argument("capture_source")
    parser.add_argument("--exact", action="store_true", help="require only the two approved fixture mounts")
    args = parser.parse_args()
    try:
        records = json.loads(args.inspection.read_text())
        validate_mounts(records, args.input_source, args.capture_source, exact=args.exact)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Mount preflight failed: {error}\n")
    print("Structured mount validation passed")


if __name__ == "__main__":
    main()
