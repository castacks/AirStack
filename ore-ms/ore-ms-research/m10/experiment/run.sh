#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
phase=${1:?use freeze or holdout}
raw=$(realpath "${2:?archived PF1 raw directory}")
out=$(realpath -m "${3:?output directory}")
sha256sum --quiet -c PROTOCOL.sha256
case "$phase" in
 freeze)
  mkdir -p "$out"
  python3 stages.py stage "$raw" development "$out/development_obs"
  python3 stages.py fit "$out/development_obs" "$raw/partitions/development_labels.csv" "$out/model.json"
  python3 stages.py stage "$raw" calibration "$out/calibration_obs"
  mkdir "$out/calibration"
  python3 isolate.py "$out/calibration_obs" "$out/model.json" "$out/calibration/proposals.csv"
  cp "$raw/partitions/calibration_labels.csv" "$out/calibration/labels.csv"
  cp "$raw/partitions/calibration_predictions.csv" "$out/calibration/baselines.csv"
  python3 evaluate.py calibrate "$out/calibration" "$raw/calibration.json" "$out/model.json" "$out/calibration.json"
  sha256sum "$out/calibration.json" | cut -d' ' -f1 > "$out/calibration.sha256"
  python3 evaluate.py deployment "$out/model.json" "$out/calibration.json" "$out/calibration.sha256" "$out/deployment.json"
  (cd "$out" && sha256sum model.json calibration.json calibration.sha256 deployment.json) > "$out/pre_holdout.sha256"
  ;;
 holdout)
  (cd "$out" && sha256sum --quiet -c pre_holdout.sha256)
  python3 evaluate.py deployment "$out/model.json" "$out/calibration.json" "$out/calibration.sha256" "$out/deployment_check.json"
  cmp "$out/deployment.json" "$out/deployment_check.json"
  python3 stages.py stage "$raw" holdout "$out/holdout_obs"
  mkdir "$out/holdout"
  python3 isolate.py "$out/holdout_obs" "$out/model.json" "$out/holdout/proposals.csv"
  time python3 isolate.py "$out/holdout_obs" "$out/deployment.json" "$out/holdout/issued.csv" --issue
  cp "$raw/partitions/holdout_labels.csv" "$out/holdout/labels.csv"
  cp "$raw/partitions/holdout_predictions.csv" "$out/holdout/baselines.csv"
  python3 evaluate.py evaluate "$out/holdout" "$out/model.json" "$out/calibration.json" "$out/calibration.sha256" "$out/results"
  python3 verify.py audit "$out"
  ;;
 *) exit 2 ;;
esac
