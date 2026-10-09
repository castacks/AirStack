#!/usr/bin/env python3
"""Inspect a frozen visual/teacher bundle offline; never acquires or dispatches."""
import argparse
import json
from pathlib import Path

from rrm.visual_evaluation import validate_visual_pair


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--frame", type=Path, required=True)
    parser.add_argument("--registration", type=Path)
    parser.add_argument("--teacher", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--launcher-sha256", required=True)
    parser.add_argument("--teacher-source-ref", required=True)
    parser.add_argument("--assessment-episode-id", required=True)
    parser.add_argument("--assessment-source-stamp-ns", type=int, required=True)
    parser.add_argument("--max-age-ns", type=int, required=True)
    args = parser.parse_args()
    read = lambda path: None if path is None else json.loads(path.read_text())
    report = validate_visual_pair(
        image=args.image.read_bytes(), frame=read(args.frame),
        registration=read(args.registration), teacher=read(args.teacher),
        manifest=read(args.manifest), launcher_sha256=args.launcher_sha256,
        expected_teacher_source_ref=args.teacher_source_ref,
        assessment_episode_id=args.assessment_episode_id,
        assessment_source_stamp_ns=args.assessment_source_stamp_ns, max_age_ns=args.max_age_ns,
    )
    print(json.dumps(report, sort_keys=True))
    return 0 if report["status"] == "BOUND_FOR_ASSESSMENT" else 2


if __name__ == "__main__":
    raise SystemExit(main())
