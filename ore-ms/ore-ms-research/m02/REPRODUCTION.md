# Reproduction

M02 uses Python 3.12.3 and its standard library only. Run from the M02 package directory. This directory contains source, configurations, plans and lightweight evidence in Git. The downloadable package contains both raw capture tables under `raw/` and a manifest that also covers them.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
sha256sum -c HEADROOM_DIAGNOSTIC.sha256
export PYTHONDONTWRITEBYTECODE=1
out=$(mktemp -d)
python3 experiment.py scene.json "$out/primary"
python3 experiment.py scene.json "$out/repeat"
python3 verify.py "$out/primary"
cmp "$out/primary/pixels.csv" "$out/repeat/pixels.csv"
cmp "$out/primary/metrics.csv" "$out/repeat/metrics.csv"
python3 experiment.py headroom_scene.json "$out/headroom"
python3 verify_headroom.py "$out/primary" "$out/headroom"
```

The checked-in metrics and logs correspond to these commands. `pixels.csv` records every texel and channel for each case, with boundary, direct, path and clear reference band radiances plus noiseless and noisy codes and upper clipping indicators. Recorded byte-level replay applies to the verified runtime. To read or replay M01 with its historical roadmap status, use the M01 R2 archive or Git revision `1a4e4e973`. The live [roadmap mandate](../RESEARCH_ROADMAP_MANDATE.md) has the updated milestone status.

The M02 package manifest is local to this directory. The original M01 checksums belong to its frozen Git revision and archive. The live roadmap is a controlled progress record and advances separately.
