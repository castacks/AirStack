#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
python_bin=${M11_PYTHON:-python3}
out=$(mktemp -d)
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
sha256sum -c PACKAGE_SHA256SUMS.txt --quiet
sha256sum -c INPUT_SHA256SUMS.txt --quiet
"$python_bin" run.py "$out"
(cd "$out" && sha256sum -c "$OLDPWD/EVIDENCE_SHA256SUMS.txt" --quiet)
M11_DECISIONS="$out/fusion_decisions.csv" "$python_bin" -m unittest -v test_verification
printf 'Independent replay matched. Fresh timing and outputs: %s\n' "$out"
