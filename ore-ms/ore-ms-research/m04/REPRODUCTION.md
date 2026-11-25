# Reproduction

In the repository run from `ore-ms/ore-ms-research/m04/`. In the downloaded archive run from `m04/`. The archived siblings contain the exact M02 RGB and M03 thermal modules. Check the archive-wide manifest at the archive root and the local M04 manifest from `m04/`.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
export PYTHONDONTWRITEBYTECODE=1
out=$(mktemp -d)
python3 experiment.py scene.json "$out/first"
python3 verify.py "$out/first"
python3 experiment.py scene.json "$out/repeat"
for table in geometry.csv radiometry.csv frames.csv; do
  cmp "$out/first/$table" "$out/repeat/$table"
done
```

The downloaded `m04/raw/` directory includes the three raw tables. For PNG generation only, use Matplotlib with an Agg backend and run `python3 plot_visibility.py "$out/first/frames.csv" "$out/visibility.png"`. The measured version is in the [replay log](evidence/replay.log). Byte-level acceptance applies to the three scientific CSVs. Data generation and verification require only CPython's standard library. PNGs are a visualization of sampled counts, while the raw CSVs and reference calculations are the evidence.

The [provenance log](evidence/provenance.log) pins the starting revision and the two reused kernels. The [prior regression log](evidence/prior_regressions.log) records archive and regression checks for M01–M03. The live roadmap status is independent of the archived copies.
