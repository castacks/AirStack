# Reproduction

Run from the Git M07 directory or the extracted archive's `m07/` directory. The archive contains pinned M02–M06 dependencies and verified M06 input tables at `m07/input/m06/`. In Git, supply the path to `m06/raw/` extracted from the already frozen M06 R1 archive. Verify the archive root and M07 manifests first.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
export PYTHONDONTWRITEBYTECODE=1
work=$(mktemp -d)
python3 generate.py input/m06 "$work/run"
python3 estimate.py "$work/run/features.csv" "$work/run/estimates.csv"
python3 verify.py "$work/run"
```

`generate.py` consumes the private generation fixture and M06 legacy source tables. `estimate.py` opens only public settings and the strict feature file. `verify.py` joins estimates to labels for evaluation and writes separate metrics and oracle/observability outputs. The isolated test copies only runtime dependencies, public configuration and allowed features. It omits labels, raw M06 and private generation settings. Repeat all three commands into a second directory and compare all six outputs. Use the recorded CPython runtime for byte equality.

Run `bash regressions.sh /path/to/archives` with the prior archives to reproduce M01–M06 archive verification. See [replay](evidence/replay.log) and [provenance](evidence/provenance.log).
