# M09 verification report

Use the pinned runtime and [exact commands](REPRODUCTION.md), ending with `python3 verify.py "$out" "$raw"`. The verifier checks gradients independently by centered differences, maps test transmissions through the accepted forward kernels, checks the archived score parity and compares paired CRPS using an independent all-pairs formula.

| Gate | Reference and result | Frozen tolerance | Outcome |
|---|---|---|---|
| V01 input integrity | 768/256/128 disjoint whole-realization IDs. Isolated predictor reproduces results without labels, teacher, oracle or prior source. Extra future column rejected. | Exact keys and no access | PASS |
| V02 M08 reproduction | M08 full replay byte-identical, matched populations exact. Rescored classical coverage/CRPS/Brier discrepancy at most 2.78e−17. | Exact M08 replay, same holdout | PASS |
| V03 objective and loading | Gradient max difference 1.02e−9. Every checkpoint loads with correct dimensions and earliest minimum validation burden NLL. | ≤2e−6 and exact selection | PASS |
| V04 physics | Checks of 23,040 issued learned distributions give finite bounded samples. Coupled maps agree with independent M02/M03 transmissions at test points with zero measured error. Data's independent transmission quantiles deviate from its mapped burden quantiles by mean 0.01045 in the frozen probe. | All values in [0,1], physics mapping within 2e−12 | PASS for declared structures |
| V05 learning/ablation | 18 fits, 720 finite epoch records, selected epochs 12–40, matched B/C training shapes, optimizers, starting arrays and permutations. | Complete frozen budgets without skill floor | PASS |
| V06 scores and calibration | Independent CRPS/Brier arithmetic checks and parity passed. Full matched coverage and event reliability retained, along with undercoverage in shifted controls. | ≤2e−12 arithmetic; complete scores | PASS calculation |
| V07 mismatch/OOD | Exact forced fallback and abstention cases; dropped/invalid LWIR equals RGB-only. Blind dynamics novelty AUROC 0.5. Wrong-air AUROC 0.92987 in joint full-size. | Correct guards and retain failures | PASS handling |
| V08 grouped comparisons | 234 cells have 128 paired IDs. Independent all-pairs comparison error 4.34e−18. Future mean averages four horizons by realization then seed. | Exact grouping and score tolerance 2e−12 | PASS |
| V09 replay/efficiency | Fresh preparation, 18 fits and inference produced byte-identical data, curves, weights, outputs and metrics. Measured runtime/RSS reported separately. | Exact deterministic files, pinned NumPy and BLAS | PASS |
| V10 invariants | M01–M08 frozen archive/test replays and source/evidence preservation checks passed. | Exact manifests, roadmap and scope | PASS |

The first scorer incorrectly classified a few zero-width oracle intervals due to NumPy and scalar exponential rounding. The final classical parity path uses the frozen M08 scalar arithmetic and restores exact issue-time oracle coverage. No prediction, hyperparameter, split or acceptance threshold changed. Numerical verification remains procedural within Codex. Physical validation and external assurance remain open.

[Verification log](evidence/verification.log) · [Replay and model hashes](evidence/replay.log) · [Prior regression log](evidence/prior_regressions.log) · [Compute report](COMPUTE.md)
