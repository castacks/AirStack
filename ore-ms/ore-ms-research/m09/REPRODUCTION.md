# Reproduction

Use CPython 3.12.3, pinned NumPy from `requirements.txt`, one OpenBLAS thread, and a disposable environment. Run from `m09/` in Git or the bundled archive. The extracted package contains the unchanged earlier dependency files and `m09/input/m08/` copied from the verified M08 archive. In Git set `raw` to the verified M08 archive's `m08/raw/`.

```sh
sha256sum -c PACKAGE_SHA256SUMS.txt
sha256sum -c EXPERIMENT_PLAN.sha256
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
raw=input/m08
out=$(mktemp -d)
python3 prepare.py "$out/data"
python3 train.py "$out/data" "$out/train"
python3 infer.py "$raw/issues.csv" "$out/train/checkpoints" "$out/issued.csv" --timing "$out/inference_runtime.csv"
python3 assess.py "$out/issued.csv" "$raw" "$out/data/prior.json" "$out/evaluation"
python3 verify.py "$out" "$raw"
python3 benchmark.py "$raw" "$out/train/checkpoints" "$out/distribution_runtime.csv"
python3 teacher_audit.py "$raw" "$out/teacher_projection.csv"
```

The supplied `checkpoints/` hold the selected weights and training hashes. The downloadable `raw/` contains generated data, issued parameters, and evaluation tables. Timing and RSS differ across runs. Compare data issues/labels/teachers/priors, checkpoint JSON, curves, issued parameters and score CSVs for deterministic replay. Optional figure requires Matplotlib. Use `MPLBACKEND=Agg python3 plot_results.py "$out/train/curves.csv" "$out/evaluation/scores.csv" "$out/learning_curves.png"`.

With the original archives available, `bash regressions.sh /path/to/archives` verifies M01–M08 independently. The bundle-level manifest additionally covers copied inputs and dependencies. The estimator-facing CLI needs the checkpoint and features only and never opens labels or teacher files.
