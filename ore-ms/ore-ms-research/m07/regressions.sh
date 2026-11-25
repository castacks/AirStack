#!/usr/bin/env bash
# Replay prior milestones from their original archives.
set -euo pipefail
archives=$(cd "$1" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export PYTHONDONTWRITEBYTECODE=1
name=ORE-MS-EXP-PER-001-M06-R1
tar -xzf "$archives/$name.tar.gz" -C "$work"
(
    cd "$work/$name"
    sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
    cd m06
    sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
    bash regressions.sh "$archives"
    python3 experiment.py config.json "$work/m06"
    python3 verify.py "$work/m06"
    for table in frames.csv pairs.csv segments.csv metadata.json statistics.csv; do
        cmp "raw/$table" "$work/m06/$table"
    done
)
(cd "$archives" && sha256sum "$name.tar.gz")
echo 'PASS: M01–M06 frozen manifests, regression suites and capture replays.'
