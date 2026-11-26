#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
out=$(realpath -m "${1:?provide a new output directory}")
sha256sum -c EXPERIMENT_PLAN.sha256
python3 corpus.py "$out"
python3 baseline.py "$out/inference" "$out/predictions.csv" --runtime "$out/runtime.json"
python3 score.py partition "$out/scoring/labels.csv" "$out/predictions.csv" "$out/partitions"
mkdir -p "$out/calibration_only"
cp "$out/partitions/"{development,calibration}_*.csv "$out/calibration_only/"
python3 score.py calibrate "$out/calibration_only" "$out/calibration.json"
sha256sum "$out/calibration.json" | cut -d' ' -f1 > "$out/calibration.sha256"
python3 score.py evaluate "$out/partitions" "$out/calibration.json" "$out/calibration.sha256" "$out/holdout.json"
