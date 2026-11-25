#!/usr/bin/env bash
# Replay previously frozen packages without writing to the archive directory.
set -euo pipefail
archives=$(cd "$1" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export PYTHONDONTWRITEBYTECODE=1

m01=ORE-MS-EXP-PER-001-M01-R2
m02=ORE-MS-EXP-PER-001-M02-R1
m03=ORE-MS-EXP-PER-001-M03-R1
m04=ORE-MS-EXP-PER-001-M04-R1
for name in "$m01" "$m02" "$m03" "$m04"; do
    tar -xzf "$archives/$name.tar.gz" -C "$work"
    (cd "$work/$name" && sha256sum --check --quiet PACKAGE_SHA256SUMS.txt)
    (cd "$archives" && sha256sum "$name.tar.gz")
done
(
    cd "$work/$m01"
    python3 simulate.py scene.json "$work/m01.csv"
    cmp evidence/observations.csv "$work/m01.csv"
    python3 verify.py "$work/m01.csv" > "$work/m01.log" 2>&1
    cat "$work/m01.log"
    python3 audit_reconciliation.py "$work/m01.csv" "$work/m01.log"
)
(
    cd "$work/$m02"
    python3 experiment.py scene.json "$work/primary"
    python3 experiment.py headroom_scene.json "$work/headroom"
    for run in primary headroom; do
        for table in pixels.csv metrics.csv; do cmp "raw/$run/$table" "$work/$run/$table"; done
    done
    python3 verify.py "$work/primary"
    python3 verify_headroom.py "$work/primary" "$work/headroom"
)
(
    cd "$work/$m03"
    python3 experiment.py scene.json "$work/m03"
    for table in spectra.csv detector.csv metrics.csv; do cmp "raw/$table" "$work/m03/$table"; done
    python3 verify.py "$work/m03"
)
(
    cd "$work/$m04/m04"
    sha256sum --check --quiet PACKAGE_SHA256SUMS.txt
    python3 experiment.py scene.json "$work/m04"
    for table in geometry.csv radiometry.csv frames.csv; do cmp "raw/$table" "$work/m04/$table"; done
    python3 verify.py "$work/m04"
)
echo 'PASS: M01–M04 manifest, regression and replay checks.'
