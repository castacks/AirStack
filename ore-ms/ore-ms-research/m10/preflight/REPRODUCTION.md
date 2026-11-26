# Reproduction

Use Linux with Python 3.12.3 and NumPy 2.5.5. Keep the sibling M02–M06 kernel files in their packaged relative paths. Run from this preflight directory in the repository or archive. The historical regression command also requires the previously delivered M01–M09 and M10-RG1 tarballs in the supplied archive directory. The PF1 archive includes all dependencies for PF1 generation, scoring and verification.

```bash
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
out="$(mktemp -d)"
manifest="$PWD/evidence/corpus.sha256"
bash reproduce.sh "$out/run"
python3 verify.py "$out/run"
(cd "$out/run" && sha256sum -c "$manifest")
bash regressions.sh /mnt/data
```

The pipeline stages are generation, isolated baseline inference, split materialization, development/calibration input staging, calibration and lock hashing, then locked holdout scoring. For the policy-facing contract, supply only the three files in `inference/`. The scorer-only store and split joins stay with the evaluator. `runtime.json` reports timing for the current run.

The expanded archive root has `PACKAGE_SHA256SUMS.txt`. Run `sha256sum -c PACKAGE_SHA256SUMS.txt` there before use. The original frozen design checksum is `EXPERIMENT_PLAN.sha256`. The deterministic output checksum manifest is `m10/preflight/evidence/corpus.sha256`, relative to the run output directory. From the archive root run the following to verify the packaged corpus. [Corpus manifest](CORPUS_MANIFEST.md)

```bash
archive_root="$PWD"
(cd raw && sha256sum -c "$archive_root/m10/preflight/evidence/corpus.sha256")
```
