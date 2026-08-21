"""Structured Docker image-preparation outcomes for CI artifacts."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

OUTCOMES = {
    "already-present",
    "pulled-versioned",
    "cache-retagged",
    "locally-built",
    "missing",
    "delegated-to-build-docker",
}


def build_image_preparation(
    outcome: str,
    *,
    pulled=(),
    retagged=(),
    built=(),
    missing=(),
) -> dict:
    if outcome not in OUTCOMES:
        raise ValueError(f"unknown image preparation outcome: {outcome}")
    return {
        "schema_version": 1,
        "outcome": outcome,
        "versioned_pulled": sorted(filter(None, pulled)),
        "cache_retagged": sorted(filter(None, retagged)),
        "locally_built": sorted(filter(None, built)),
        "missing": sorted(filter(None, missing)),
    }


def write_from_environment(path: Path) -> Path:
    def lines(name):
        return os.environ.get(name, "").splitlines()

    payload = build_image_preparation(
        os.environ["IMAGE_OUTCOME"],
        pulled=lines("PULLED_IMAGES"),
        retagged=lines("RETAGGED_IMAGES"),
        built=lines("BUILT_IMAGES"),
        missing=lines("MISSING_IMAGES"),
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    write_from_environment(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
