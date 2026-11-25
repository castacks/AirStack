#!/usr/bin/env bash
# Reproduce M01–M08 from unchanged archived milestone packages.
set -euo pipefail
archives=$(cd "$1" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export PYTHONDONTWRITEBYTECODE=1
name=ORE-MS-EXP-PER-001-M08-R1
tar -xzf "$archives/$name.tar.gz" -C "$work"
(
 cd "$work/$name"
 sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
 cd m08
 sha256sum -c --quiet PACKAGE_SHA256SUMS.txt
 bash regressions.sh "$archives"
 python3 generate.py "$work/m08"
 python3 predict.py "$work/m08/issues.csv" "$work/m08/forecasts.csv"
 python3 oracle.py "$work/m08/oracle_input.csv" "$work/m08/oracle_forecasts.csv"
 python3 scores.py "$work/m08"
 python3 verify.py "$work/m08"
 for file in issues.csv truth.csv oracle_input.csv forecasts.csv oracle_forecasts.csv scores.csv reliability.csv paired.csv; do
  cmp "raw/$file" "$work/m08/$file"
 done
 sha256sum raw/issues.csv raw/truth.csv raw/forecasts.csv raw/oracle_forecasts.csv raw/scores.csv
)
(cd "$archives" && sha256sum "$name.tar.gz")
echo 'PASS: M01–M08 frozen manifests, regressions and exact M08 benchmark replay.'
