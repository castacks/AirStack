# Reproduction

Use CPython from `ore-ms/ore-ms-research/m05/` in the repository or from `m05/` in the archive. The archive includes exact M02–M04 kernel copies needed for interface verification. Check the package-wide manifest at the archive root, and the M05 manifest from this directory.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
export PYTHONDONTWRITEBYTECODE=1
out=$(mktemp -d)
python3 experiment.py config.json "$out/first"
python3 verify.py "$out/first" "$out/first/statistics.csv"
python3 experiment.py config.json "$out/repeat"
python3 verify.py "$out/repeat" "$out/repeat/statistics.csv"
for table in ensemble.csv forced.csv trajectories.csv stress.csv statistics.csv; do
  cmp "$out/first/$table" "$out/repeat/$table"
done
```

The archive contains all four raw output CSVs and the full statistics table under `m05/raw/`. The repository carries lightweight summary evidence. `ensemble.csv` records stationary co-moving time samples, initial spatial samples, and a separate heat latent. `forced.csv` records selected checkpoints from all driven realizations. `trajectories.csv` records full selected spatial trajectories. `stress.csv` retains per-realization extrema, evaluation counts and violation counts. The [replay log](evidence/replay.log) records both runs' hashes and runtime.

With frozen M01–M04 archives in a directory, run `bash regressions.sh /path/to/archives` to reproduce [prior regression evidence](evidence/prior_regressions.log). The [roadmap mandate](../RESEARCH_ROADMAP_MANDATE.md) and [acceptance register](../ACCEPTANCE_CONDITIONS_REGISTER.md) govern later claims and scope.
