# PF01–PF12 verification report

Use the preflight directory, declared Python environment and a new output directory. The commands below create the referenced run, check the archived deterministic outputs, and exercise all automated verification predicates. `repo` is the root of the AirStack checkout.

```bash
export OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
out="$(mktemp -d)/run"
manifest="$PWD/evidence/corpus.sha256"
bash reproduce.sh "$out"
python3 verify.py "$out"
(cd "$out" && sha256sum -c "$manifest")
bash regressions.sh /mnt/data
git -C "$repo" diff --name-only 86709781f HEAD
```

| Gate | Criterion and measured result | Independent check and reproduction | Disposition |
|---|---|---|---|
| PF01 | Prior accepted artifacts and RG1 unchanged. Historical archives passed internal integrity and replay. | `bash regressions.sh /mnt/data` and compare the Git path list to the single roadmap row and this preflight directory. | PASS |
| PF02 | 512 realization groups, 27 cases each, 13,824 records. All 16 hashes repeated exactly. | `bash reproduce.sh "$out"` and `(cd "$out" && sha256sum -c "$manifest")`. | PASS |
| PF03 | Group counts 128/128/256, no duplicate case/join or cross-partition group. | `python3 verify.py "$out"` independently rebuilds groups from scene labels and acquisition rows. | PASS |
| PF04 | 0 mismatched projections, first-hit planes, centroids, feasibility or negative labels. | `python3 verify.py "$out"` uses integer-center projection and exact rational geometry. | PASS |
| PF05 | All observations on the 33×25 grid. 1900 ms and injected 2001 ms LWIR rejected. Known shift excluded. Hidden shift retained at 3 px. | `python3 verify.py "$out"` checks expected metadata and detector code displacements without geometry correction. | PASS |
| PF06 | Standalone candidate bytes unchanged after label mutation. Seven forbidden fields rejected. All 512 blind future pairs had identical outputs although future burdens differed. | `python3 verify.py "$out"` runs an isolated subprocess and checks pair equality. | PASS |
| PF07 | All opportunities scored. RGB crossover 256/256 correct on holdout, LWIR 0 candidates. Both abstained under clipping and full occlusion. | `python3 verify.py "$out"` compares detector/shape controls and evaluates baseline rows independently. | PASS |
| PF08 | Fixed equal map passed four-pixel fixture, registration conflict and shared eligibility/fallback. | `python3 verify.py "$out"` compares the candidate position and score to a hand-calculated fixture. | PASS |
| PF09 | Three distinct mappings, calibration provenance and cutoff selection verified. Separate absent and hidden upper bound 1.478% at holdout for each arm. | `python3 verify.py "$out"` checks both fit partitions, independent risk denominators and a numeric Wilson root to 1e−12. | PASS |
| PF10 | Independent scoring and Brier agreement for all arm/case cells. Paired bootstrap endpoints agreed exactly. Matched distractor and 4/20 false support were tracked as failed operating cells. | `python3 verify.py "$out"` uses rational distance, independent bootstrap multiplicities and percentile calculations. | PASS |
| PF11 | M01–M09 regressions and RG1 check passed. All deterministic PF1 outputs replayed byte-identically. | `bash regressions.sh /mnt/data` and `(cd "$out" && sha256sum -c "$manifest")` after a fresh replay. | PASS |
| PF12 | Three nonadaptive comparators and scorer/calibration only. Adaptive M10 and downstream milestones untouched. | `git -C "$repo" diff --name-only 86709781f HEAD` and inspect this preflight directory's code changes. | PASS |

Gate PASS indicates the verified benchmark criterion was met. The distractor and 4/20 partial controls failed as operating cells and carry no localization claim. [Raw independent verification](evidence/independent_verification.json) · [Historical regressions](evidence/prior_regressions.log) · [Replay](evidence/replay.log)
