#!/usr/bin/env bash
set -euo pipefail
pf=$(cd "$(dirname "$0")/../preflight" && pwd)
raw=$(realpath "${1:?archived PF1 raw directory}")
out=$(realpath -m "${2:?new replay directory}")
mkdir "$out"
ln -s "$raw/"{inference,scoring} "$out/"
(cd "$pf" && sha256sum --quiet -c PACKAGE_SHA256SUMS.txt)
(cd "$raw" && sha256sum --quiet -c "$pf/evidence/corpus.sha256")
python3 "$pf/baseline.py" "$out/inference" "$out/predictions.csv"
python3 "$pf/score.py" partition "$out/scoring/labels.csv" "$out/predictions.csv" "$out/partitions"
mkdir "$out/calibration_only"
cp "$out/partitions/"{development,calibration}_*.csv "$out/calibration_only/"
python3 "$pf/score.py" calibrate "$out/calibration_only" "$out/calibration.json"
cp "$raw/calibration.sha256" "$out/"
python3 "$pf/score.py" evaluate "$out/partitions" "$out/calibration.json" "$out/calibration.sha256" "$out/holdout.json"
(cd "$out" && sha256sum --quiet -c "$pf/evidence/corpus.sha256")
python3 "$pf/verify.py" "$out" | cmp "$pf/evidence/independent_verification.json" -
echo 'PASS: all PF1 output hashes, labels, split sizes, candidates, B03 selection, thresholds and holdout metrics reproduced from archived raw data.'
