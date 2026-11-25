#!/usr/bin/env bash
# Verify all earlier archived milestones without changing the originals.
set -euo pipefail
archives=$(cd "$1" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export PYTHONDONTWRITEBYTECODE=1
name=ORE-MS-EXP-PER-001-M05-R1
tar -xzf "$archives/$name.tar.gz" -C "$work"
(
    cd "$work/$name"
    sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
    cd m05
    bash regressions.sh "$archives"
    sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
    python3 experiment.py config.json "$work/m05"
    python3 verify.py "$work/m05" "$work/m05/statistics.csv"
    for table in ensemble.csv forced.csv trajectories.csv stress.csv statistics.csv; do
        cmp "raw/$table" "$work/m05/$table"
    done
)
(cd "$archives" && sha256sum "$name.tar.gz")
echo 'PASS: M01–M05 archived manifests, harnesses and capture replays.'
