# EXP106 — Matched stage/objective comparison

The frozen final-trace objective improves the primary development ranking metric by **3.05%** versus final-diagonal training, with worse aggregate diagonal likelihood and joint coverage. The recurrent-diagonal arm still ranks best on the two-record macro. This is a tradeoff; no model is promoted.

## Frozen comparison

Execution declaration and host orchestration committed as `c7c2c7fd3` before training. All arms start from the original RGB checkpoint and use the same 48 provided-flow training pairs, round-robin order, full covariance-update block (24 tensors / 3,075,202 scalars), AdamW settings, seed 0, clip norm 1, and 48 updates. Each also replays steps 24–48 from its step-24 checkpoint: 144 scientific updates plus 72 verification updates. Every declared command passed on its first attempt.

Evaluation retains all 36 already observed synthetic RGB development pairs: office 16, Japanesealley 16, and legacy amusement 4. Means, reference support, retention fractions, stable tie ordering, random/oracle controls, coverage levels and record aggregation follow EXP103. The primary comparison is final trace versus final diagonal at 50% retained mass, pooled within each recording and then equally averaged over the two added recordings. All per-record results, five retention curves, coverage levels, standardized moments, recurrent/final NLL and macro/micro groups are retained.

## Primary two-record development macro

| Model | MSE at 50% retention (px²) | Joint coverage at nominal 95% | Final diagonal NLL |
|---|---:|---:|---:|
| Original checkpoint | 1.409787 | 87.331% | -0.188703 |
| Recurrent diagonal | 1.098781 | 85.837% | -0.123313 |
| Final diagonal | 1.268779 | 85.539% | -0.089741 |
| Final trace | 1.230109 | 85.351% | -0.064285 |

Final trace reduces primary MSE from 1.268779 to 1.230109 px² (−3.0478%). Its coverage falls from 85.539% to 85.351%, farther from nominal 95%; final diagonal NLL worsens from −0.089741 to −0.064285 (lower is better). Recurrent-diagonal MSE is 1.098781 px², better than both final-stage arms. No dominance criterion is satisfied.

The aggregate improvement is recording-dependent: office MSE worsens while Japanesealley improves. Equal-record aggregation preserves substantially different absolute error scales; it does not make their relative improvements equal. Legacy amusement ranking also worsens.

## Per-record final-stage results

| Recording | Model | MSE at 50% (px²) | Joint 95% coverage | Final diagonal NLL |
|---|---|---:|---:|---:|
| office | Original checkpoint | 0.015347 | 91.908% | -1.537873 |
| office | Recurrent diagonal | 0.015847 | 92.599% | -1.527826 |
| office | Final diagonal | 0.015274 | 92.350% | -1.544817 |
| office | Final trace | 0.015743 | 91.880% | -1.507080 |
| japanesealley | Original checkpoint | 2.804227 | 82.754% | 1.160467 |
| japanesealley | Recurrent diagonal | 2.181714 | 79.076% | 1.281200 |
| japanesealley | Final diagonal | 2.522284 | 78.727% | 1.365334 |
| japanesealley | Final trace | 2.444474 | 78.823% | 1.378510 |
| amusement | Original checkpoint | 0.080048 | 92.632% | -0.403946 |
| amusement | Recurrent diagonal | 0.079514 | 90.603% | -0.360315 |
| amusement | Final diagonal | 0.080223 | 91.332% | -0.366570 |
| amusement | Final trace | 0.080522 | 91.498% | -0.388674 |

## Training and verification

| Objective | Clipped updates | Gradient norm range before clipping | Changed / frozen state entries |
|---|---:|---:|---:|
| Recurrent diagonal | 48/48 | 16.140–1404.835 | 24 / 404 |
| Final diagonal | 48/48 | 2.913–269.791 | 24 / 404 |
| Final trace | 48/48 | 1.726–422.784 | 24 / 404 |

All 48 steps clip in every arm. Own training-objective values have different semantics and are not cross-compared. First/last journal losses are on different input pairs and do not establish training descent.

The training audit passes 10,829 checks; the final audit passes 15,863 checks, including repeated training checks. Full checkpoints match each arm’s replay exactly: model, optimizer, Python/NumPy/torch/CUDA RNG, identity, manifest and trainable names. The final 24 journal rows and validation scores also match exactly. All 404 frozen state entries remain unchanged, with exactly 24 covariance tensors changed. All 432 common recurrent mean fields are identical across the four evaluated models. Independent saved-array accounting checks support, all declared coverage counts, final NLL, stable-tie ranking, oracle sums and the primary macro aggregation. CUDA32 and NumPy64 recurrent/final likelihood calculations agree at the frozen tolerances.

A supplemental historical comparison finds baseline and recurrent-arm mean/raw hashes and NumPy64 recurrent/final scores exact for all 432 stages each. The newly trained recurrent arm’s 428 model-state entries are also bitwise equal to the previous broad48 candidate.

## Recovery and retained failures

This fresh workspace initially lacked datasets, results and the old GPU image. Eight Box packets were verified (1,417 archive members), and required ignored artifacts were restored without changing tracked historical source. The original RGB checkpoint SHA256 matches `bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a`.

Rebuilt the committed pinned-base `Dockerfile.blackwell`, yielding image `sha256:9c0ea2714e913f310b99bd37ddbcae7b616789f8fae856f325030d57b4f4ac26`; the old image was `8f7bf1c6…`. Full package list, Docker inspect/build log and the exact rebuilt image are retained for recovery. The native zero-update preflight matches previous model digests, all objective scores, gradient hashes and mean fields exactly. Fresh isolated CPU tests: 111 passed / 3 CUDA-only skipped; targeted CUDA tests: 3 passed.

Retained harness failures: the first test invocation named a nonexistent test file (zero tests); a combined GPU-visible invocation passed 112 tests but failed two CPU assertions expecting no visible CUDA RNG. Exact replay assertions had passed. Separate CPU-hidden/CUDA-visible invocations pass without source changes. Initial freeze also found a missing historical final-layer candidate; its verified prerequisite packet was restored before the successful freeze. No model-training retries, outcome-selected settings, new records or metric changes occurred.

## Decision and limits

Retain all candidates as research artifacts. Final-trace training shows a modest primary ranking gain over final-diagonal training, with recording-specific regressions and likelihood/coverage costs. It does not improve on the recurrent arm’s aggregate ranking. The trace objective identifies total variance rather than its directional allocation; the frozen diagonal likelihood/ellipse diagnostics remain reported as declared. These outcomes do not identify a failure mechanism.

These are exposed synthetic RGB development prefixes with unknown base-network pretraining overlap. There is no independent-test, thermal, physical VO, generalization or deployment claim. The prior seven MS² reference-readiness requirements and protected reserves remain unchanged. Before further tuning or adoption, declare separate recording-level evaluation evidence and exposure rules; do not select new data or metrics to repair these outcomes.

Artifacts: `results/objective-comparison/run-001/` and `results/objective-comparison-preflight/run-001/`. Recovery target: `ore_ms:ore-ms/backups/2026-11-02-objective-comparison`, incremental from `ea412f4e6`, with a separately saved exact runtime image.
