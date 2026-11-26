# Reproduction

Use Linux x86-64 with Landlock ABI 4 or newer, Python 3.12.3 and NumPy 2.5.5. The reference CPU was AMD EPYC-Genoa with one OpenBLAS thread. The M10-R1 evidence archive includes the original PF1 archive under `dependencies/`. An existing copy with the same checksum can also be used. For the historical regression replay, place the prior M01–M09 and RG1 tarballs together in an archive directory.

Run from `m10/experiment` in the repository or the expanded evidence package. Set `pf1_archive` to the absolute path of the original M10-PF1 tarball. Set `history` to the directory holding the prior milestone packages.

```bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
work="$(mktemp -d)"
tar -xzf "$pf1_archive" -C "$work"
pf="$work/ORE-MS-EXP-PER-001-M10-PF1"
(cd "$pf" && sha256sum --quiet -c PACKAGE_SHA256SUMS.txt)
raw="$pf/raw"
bash replay_preflight.sh "$raw" "$work/pf1_replay"
bash ../preflight/regressions.sh "$history"
python3 verify.py smoke
bash run.sh freeze "$raw" "$work/run"
bash run.sh holdout "$raw" "$work/run" > "$work/run/verification.json"
mkdir "$work/run/description"
python3 describe.py "$work/run" "$work/run/description"
cmp evidence/metrics.json "$work/run/results/metrics.json"
cmp evidence/verification.json "$work/run/verification.json"
cmp evidence/scenario_metrics.csv "$work/run/description/scenario_metrics.csv"
```

The freeze command only stages development and calibration rows. The holdout command validates the checksum receipt, stages observation rows and executes isolated inference before copying holdout labels into the scoring stage. It scores the original corpus and does not regenerate holdout data. The raw output directory in the evidence package contains the holdout proposals and issued decision log. The archived implementation and deterministic evidence files are checked from the package root with `sha256sum -c PACKAGE_SHA256SUMS.txt`. [Prospective protocol](PROTOCOL.md)
