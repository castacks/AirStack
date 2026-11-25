#!/usr/bin/env bash
# Reproduce M01–M07 from frozen archive contents.
set -euo pipefail
archives=$(cd "$1" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export PYTHONDONTWRITEBYTECODE=1
name=ORE-MS-EXP-PER-001-M07-R1
tar -xzf "$archives/$name.tar.gz" -C "$work"
(
    cd "$work/$name"
    sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
    cd m07
    sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
    bash regressions.sh "$archives"
    python3 generate.py input/m06 "$work/m07"
    python3 estimate.py "$work/m07/features.csv" "$work/m07/estimates.csv"
    python3 verify.py "$work/m07"
    for table in features.csv labels.csv estimates.csv metrics.csv observability.csv oracle.csv; do
        cmp "raw/$table" "$work/m07/$table"
    done
)
(cd "$archives" && sha256sum "$name.tar.gz")
echo 'PASS: M01–M07 frozen manifests, regression suites and capture replays.'
