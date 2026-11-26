#!/usr/bin/env bash
set -euo pipefail
archives=$(cd "$1" && pwd)
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
name=ORE-MS-EXP-PER-001-M09-R1
tar -xzf "$archives/$name.tar.gz" -C "$work"
(
 cd "$work/$name"
 sha256sum --check --quiet PACKAGE_SHA256SUMS.txt
 cd m09
 sha256sum --check --quiet PACKAGE_SHA256SUMS.txt
 bash regressions.sh "$archives"
 out="$work/out"
 mkdir "$out"
 python3 prepare.py "$out/data"
 python3 train.py "$out/data" "$out/train"
 python3 infer.py input/m08/issues.csv "$out/train/checkpoints" "$out/issued.csv"
 python3 assess.py "$out/issued.csv" input/m08 "$out/data/prior.json" "$out/evaluation"
 python3 verify.py "$out" input/m08
 for file in data/issues.csv data/labels.csv data/teacher.csv data/prior.json issued.csv evaluation/scores.csv evaluation/reliability.csv evaluation/paired.csv evaluation/ood.csv; do cmp "raw/$file" "$out/$file"; done
 cmp evidence/curves.csv "$out/train/curves.csv"
 for file in checkpoints/*.json; do cmp "$file" "$out/train/$file"; done
)
name=ORE-MS-EXP-PER-001-M10-RG1
tar -xzf "$archives/$name.tar.gz" -C "$work"
(
 cd "$work/$name"
 sha256sum --check --quiet PACKAGE_SHA256SUMS.txt
 cd m10/readiness
 sha256sum --check --quiet PACKAGE_SHA256SUMS.txt
 bash reproduce.sh "$archives" | cmp evidence/archive_integrity.log -
)
echo 'PASS: full M01–M09 regression and frozen RG1 integrity.'
