# Fixed-budget final-output stereo residual: accuracy outcome

**10 November 2026 · frozen execution `06a1477a9` · error and accuracy**

The final-output residual can learn and modestly improve thermal native VO on this rain prefix. It does **not** meet the predeclared advancement rule: depth accuracy worsens, translation tails do not improve, and RGB remains better. Close this bounded adapter recipe without another training-budget, learning-rate or rain-checkpoint search. This is a result about this fit and these exposed development frames; it does not establish that stereo adaptation or the broader RGB–thermal study is a dead end.

## What was run

Followed the [frozen protocol](../../experiments/final-residual-fit-freeze-2026-11-10.json) after the [descent contract](mean-descent-contract-findings-2026-11-10.md). Started a new zero residual, independently of the three-update control. Fit 2,305 scalars: a one-channel 3×3 convolution on frozen final decoder features, upsampled with the frozen native mask and added only to horizontal stereo flow. The entire pretrained decoder stays fixed; there is no feedback into its recurrent coordinates.

Used the same 20 day and 20 night frames, 342,569 eligible label-pixel uses and equal-frame sparse SmoothL1 beta1 objective. Full-batch normalized gradient, initial step 0.001, Armijo coefficient 0.0001, halving and at most 12 trials are unchanged from the numerical contract. Strict GPU FP32, TF32 disabled during fitting. All 40 updates accept the first trial and decrease the objective. Training loss **1.71774722 → 1.41455805 (−17.65%)**. No rain pixels, labels or scores were read during fitting.

The final checkpoint was locked at **2026-11-10 16:23:30 UTC**, before all five evaluation processes. SHA256: `b6ae2bd21b35eedd2b43d33f5f89abde54022b44493c46b419b3fbe4e72b7259`. Both earlier adaptation/control artifacts remain unchanged. Evaluation uses inherited native inference math; its original thermal stereo and VO baselines reproduce the previous results.

## Rain stereo: pixel improvement does not yield depth improvement

Twenty rain frames, identical supplied own-camera sparse labels and fixed eligibility masks; equal-frame means. No clipping or prediction-dependent filtering.

| Metric | Original thermal | Fitted adapter | Change |
|---|---:|---:|---:|
| Signed horizontal-flow MAE, px | 2.190303 | 2.134281 | −2.56% |
| Depth AbsRel | 0.179222 | 0.206939 | +15.47% |
| Depth MAE, m | 2.613372 | 2.988819 | +14.37% |
| Mean per-frame depth RMSE, m | 6.958122 | 12.843320 | +84.58% |

Nonnegative horizontal-flow fraction is zero originally and 0.00001514 with adaptation. Depth remains scored through native absolute disparity, including those sign failures. Sparse disparity SmoothL1 and depth metrics are distinct objectives; these results demonstrate their mismatch for this recipe. They do not identify a general physical cause of the motion errors.

## Native VO: modest mean gains, mixed tails, below RGB

All four arms finish 20 frames. Original, zero residual and adapted thermal have 18/19 intervals eligible under inherited nominal common-frame/time mapping; RGB has 19/19. Thermal interval1 remains outside the existing mapping horizon. Scores below use the same **18 common intervals**. This is interval motion error, not trajectory ATE.

| Motion error | Original thermal | Fitted adapter | RGB |
|---|---:|---:|---:|
| Translation mean, m | 0.960001 | 0.930810 | 0.670000 |
| Translation RMSE, m | 1.048964 | 1.030478 | 0.867444 |
| Translation p95, m | 1.500730 | 1.510397 | 1.383230 |
| Rotation mean, deg | 0.075270 | 0.067631 | 0.031285 |
| Rotation RMSE, deg | 0.095380 | 0.085099 | 0.045285 |
| Rotation p95, deg | 0.180671 | 0.166454 | 0.066400 |

Against original thermal, translation mean/RMSE improve 3.04%/1.76% and rotation mean/RMSE improve 10.15%/10.78%. Translation median worsens 1.024987 → 1.041204 m; p95 worsens 0.64%. The zero-residual control reproduces every saved original pose/time array exactly, including intervals excluded from scoring. Coverage does not change.

Temporal-flow tensors, vertical flow and disparity variance stay original. The native disparity-to-depth Jacobian propagates the changed disparity into depth covariance; depth covariance is not numerically held fixed. No default or upstream network source changed.

## Verification and audit limitations

Nine tests pass: the five prior descent/operator checks plus four stereo-batch integration and context-restoration checks. Independent NumPy/Pillow/SciPy reconstruction verifies 358 frozen hashes, final predictions on 40 training caches, all 40 recorded update acceptance conditions, 200 stereo metric values, 73 mapped native motions and the common metric summaries. Independent training final loss is 1.414558028. Maximum training prediction discrepancy is 2.24e-6 px. The audit does not regenerate the native feature caches or every intermediate optimizer gradient; those depend on the preceding cache/state/gradient contracts.

The initial frozen auditor fails on a variable-name collision. The amended auditor also corrects its assumption of 20 frontend calls: the frozen runner captures first depth separately, then the system initializes and processes 19 pairs, giving batches `[1,1,2×19]`. These are auditor-only corrections; no training, inference, scoring or selection was repeated. The original source and failed logs are retained.

Its original strict reconstruction tolerance also fails on rain inference, where native cuDNN TF32 remains allowed. The [auditor amendment](../../experiments/final-residual-fit-audit-amendment-2026-11-10.json) records the precision-aware check: independent positive-operator bounds for TF32 input/weight rounding, FP32 accumulation and output rounding. Maximum residual discrepancy is **0.00017565 px**; every pixel lies within its derived bound (largest bound 0.002228 px). Score reconstruction from saved predictions retains the original tolerances. A separately reconstructed FP64 residual still yields depth AbsRel **0.20693957**, MAE **2.988827 m**, RMSE **12.844262 m**. Depth degradation therefore persists independently of that precision discrepancy.

Full [outcome and hashes](../../experiments/final-residual-fit-outcome-2026-11-10.json). Local evidence: `results/final-residual-fit/run-001`. Reproduce fitting/evaluation with `final_residual_fit.py`; use `audit_final_residual_fit_v2.py` for the amended independent audit.

## Decision and next work

The predeclared rule requires better rain AbsRel and translation mean/RMSE without worse rotation mean/RMSE or coverage. AbsRel fails. Keep the original models and close this recipe. The evidence now separates successful optimization from mixed accuracy transfer, resolving the uncertainty left by the first fit's failed descent.

Next work should return to the RGB–thermal correspondence/representation question. Before a new fit, select one materially different intervention with a measurable correspondence endpoint and downstream motion endpoint, and check it against the prior experiment inventory. These data remain exposed development prefixes with supplied references and nominal alignment. A broader accuracy claim requires separate recording-level evidence. No runtime benchmark or deployment claim is made.
