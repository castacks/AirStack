#!/usr/bin/env bash
# Read-only replay of the arithmetic and checksummed historical evidence links.
set -euo pipefail
review=$(cd "$(dirname "$0")" && pwd)
research=$(cd "$review/../.." && pwd)
archives=$(cd "$1" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
python3 -B "$review/audit.py" > "$work/numeric_audit.csv"
cmp "$review/evidence/numeric_audit.csv" "$work/numeric_audit.csv"
printf '%s\n' 'PASS: deterministic readiness arithmetic.'
for i in $(seq -w 1 9); do
    milestone=0$i
    rev=R1
    if [[ $milestone == 01 ]]; then rev=R2; fi
    name=ORE-MS-EXP-PER-001-M$milestone-$rev
    tar -xzf "$archives/$name.tar.gz" -C "$work"
    (
       cd "$work/$name"
       sha256sum --check --quiet PACKAGE_SHA256SUMS.txt
       if [[ -f m$milestone/PACKAGE_SHA256SUMS.txt ]]; then
           cd "m$milestone"
           sha256sum --check --quiet PACKAGE_SHA256SUMS.txt
       fi
    )
    (cd "$archives" && sha256sum "$name.tar.gz")
done
for file in     m06/evidence/statistics.csv     m07/evidence/observability.csv m07/evidence/oracle.csv m07/evidence/metrics.csv m07/evidence/verification.log     m08/evidence/verification.log m08/evidence/scores.csv     m09/evidence/scores.csv m09/evidence/ood.csv m09/evidence/paired.csv; do
    milestone=${file:1:2}
    cmp "$research/$file" "$work/ORE-MS-EXP-PER-001-M$milestone-R1/$file"
done
printf '%s\n' 'PASS: all 10 numeric input files match archived members.'
