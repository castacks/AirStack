# Reproduction

M03 uses CPython and only its standard library. Run from the M03 directory.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
export PYTHONDONTWRITEBYTECODE=1
out=$(mktemp -d)
python3 experiment.py scene.json "$out/first"
python3 verify.py "$out/first"
python3 experiment.py scene.json "$out/repeat"
for table in spectra.csv detector.csv metrics.csv; do
  cmp "$out/first/$table" "$out/repeat/$table"
done
```

The repository package includes lightweight metrics and verification logs. The downloadable archive also holds `raw/spectra.csv`, `raw/detector.csv` and `raw/metrics.csv`, all covered by its manifest. Spectral columns with suffix `_W_m2_sr_um` are densities per µm. Band, analog and decoded values are in W·m⁻²·sr⁻¹. Codes and response weights are dimensionless. Output hashes and the Python version are in [replay evidence](evidence/replay.log).

The starting repository revision and prior milestone checks are in [provenance](evidence/provenance.log) and [regression evidence](evidence/prior_regressions.log). Use the archived milestones to replay their original roadmap status. The [live roadmap](../RESEARCH_ROADMAP_MANDATE.md) records current authorization.
