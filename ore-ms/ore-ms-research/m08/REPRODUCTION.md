# Reproduction

Run from `m08/` in the repository or extracted archive. The archive contains the exact imported M02–M07 dependencies. Use the recorded CPython runtime and verify both manifests.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
export PYTHONDONTWRITEBYTECODE=1
out=$(mktemp -d)/first
python3 generate.py "$out"
python3 predict.py "$out/issues.csv" "$out/forecasts.csv"
python3 oracle.py "$out/oracle_input.csv" "$out/oracle_forecasts.csv"
python3 scores.py "$out"
python3 verify.py "$out"
```

Only `issues.csv` reaches the predictor. The generator writes `truth.csv` for scoring and `oracle_input.csv` for the separately invoked privileged reference. The archive stores all eight exact output tables under `m08/raw/`. Burden samples in the forecast tables represent empirical equally weighted marginal distributions. Apply the fixed monotone spectral maps in `dynamics.py` to those samples to reproduce RGB/LWIR. For a second run compare issues, truth, oracle_input, forecasts, oracle_forecasts, scores, reliability and paired CSVs. See [replay](evidence/replay.log) and [provenance](evidence/provenance.log).

With the original archives available, run `bash regressions.sh /path/to/archives` to reproduce prior evidence. The M07 archive contains its own verified M06 input tables and the regression chain verifies M01–M07. The predictor and scoring have no third-party runtime dependencies.
