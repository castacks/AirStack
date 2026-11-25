# Reproduction

Run from `ore-ms/ore-ms-research/m06/` in Git or `m06/` in the downloadable package. The archive root provides a covering manifest and unchanged copies of the four imported kernels. Run the following from the M06 directory after checking the root manifest.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
export PYTHONDONTWRITEBYTECODE=1
out=$(mktemp -d)
python3 experiment.py config.json "$out/first"
python3 verify.py "$out/first"
python3 experiment.py config.json "$out/repeat"
python3 verify.py "$out/repeat"
for table in frames.csv pairs.csv segments.csv metadata.json statistics.csv; do
  cmp "$out/first/$table" "$out/repeat/$table"
done
```

The archive stores `m06/raw/` with the five verified outputs. Lightweight evidence includes the statistical table, replay hashes and qualification logs. Core simulation and tests use CPython and its standard library. To regenerate the sample visualization with Matplotlib and NumPy, run `MPLBACKEND=Agg python3 plot_examples.py "$out/first/pairs.csv" "$out/examples.png"`. The visualization is outside the data replay equality criterion.

With prior packages on disk, run `bash regressions.sh /path/to/archives`. This verifies the archived manifests and compares all earlier generated outputs. The [provenance log](evidence/provenance.log) records the starting commit, configuration and imported kernel identities.
