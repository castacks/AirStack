# Bearing-space rotation: frozen rain development gate passes

12 November 2026. **The primary original-flow PNEC prototype passes the frozen
rain development gate.** Rotation mean/RMSE improve against both matched NEC
and native thermal context, with unchanged eligible coverage. This is a
rotation-only result on one exposed development prefix, not a full-VO,
RGB–thermal fusion, calibrated-uncertainty or recording-transfer result.

## Fixed execution

Followed the [design/input lock](../../experiments/bearing-rotation-assay-protocol-2026-11-12.json)
and [execution freeze](../../experiments/bearing-rotation-execution-freeze-2026-11-12.json).
Used the same first20rain groups000000–000019 of `_2021-08-06-16-19-00`.
There are19adjacent slots, two fixed mean sources and three objectives:114
solver calls total, each attempted once. No neural network was run in this
execution; means and covariance come from the earlier verified recovery.

The fixed source grid starts with585points per pair. Joint finite/inbound/SPD
support retains10,198observations. Essential-matrix RANSAC uses seed0,
probability0.999, threshold1native pixel and5000iterations. Intersecting the
original/replacement masks retains8,990observations,402–537per pair. Both means
and every objective receive the same retained source points. No reference
pose, uncertainty threshold, candidate residual or error score selects points.
This remains a conditional, jointly filtered population; it is not all image
pixels or independently verified correspondence identity.

All114solves report convergence under the declared optimizer/geometry checks.
The inherited nominal time horizon excludes pair1from scoring for all methods;
its prediction is retained. All six methods and native thermal have18/19eligible
slots. No failure, cap or horizon rule was relaxed.

## Rotation accuracy on the identical18intervals

| Means / objective | Mean, deg | RMSE, deg | p95, deg |
|---|---:|---:|---:|
| Original / NEC | 0.076703 | 0.080089 | 0.114687 |
| Original / unit pixel-covariance PNEC | 0.068908 | 0.072334 | 0.107381 |
| **Original / original-covariance PNEC — primary** | **0.056581** | **0.059519** | **0.094656** |
| Replacement / NEC | 0.059050 | 0.067540 | 0.125043 |
| Replacement / unit pixel-covariance PNEC | 0.052644 | 0.060998 | 0.112338 |
| Replacement / original-covariance PNEC | 0.059662 | 0.068353 | 0.132961 |
| Native thermal context | 0.075270 | 0.095380 | 0.180672 |

The primary reduces mean/RMSE by26.23%/25.68%against NEC and24.83%/37.60%
against native thermal. It passes all six checks: nonempty matched support,
no NEC interval-coverage loss, lower mean and RMSE than NEC, and no higher mean
or RMSE than native thermal. These are the same primary candidate and rules
fixed before solving; the lowest secondary mean does not replace the primary.

The previously recorded RGB context on these18common rain intervals remains
better:0.031285degrees mean and0.045285degrees RMSE from the
[earlier native comparison](thermal-matcher-pilot-findings-2026-11-12.md).
This assay does not establish a gain over RGB.

## Interpretation informed by the papers

The objective is inspired by [PNEC, CVPR2022](https://arxiv.org/html/2204.02256v1).
Its published method motivates estimating rotation from uncertain bearings
without stereo feature depths. Our first-order covariance propagation,
eight-point start and SciPy local joint solver differ from the authors'
unscented propagation and alternating/globalized optimizer; this is not a full
paper reproduction.

On original means, the learned-covariance version improves over both NEC and
unit pixel-covariance PNEC. On replacement means, reusing original covariance
does not improve over either alternative. That pattern is consistent with the
need for covariance compatible with its correspondences; it does not prove
calibration or identify the cause of the preceding native-flow regression.
Isotropic pixel covariance still yields geometry-dependent residual variance;
it is not simply an unweighted NEC duplicate.

The NEC/PNEC objectives share input points, initialization code and optimizer
budget. The comparison to native thermal is contextual: native VO also differs
in keypoint population, geometric model and optimization. It is not an isolated
causal depth ablation. A lower optimized cost is not used as evidence of lower
motion error; the reported errors come from the separate locked scorer.

[DNLS, CVPR2023](https://arxiv.org/html/2305.09527v2) remains a possible later
task-aligned learning route. Neither a successful rotation assay nor learning
covariance through pose loss establishes absolute covariance calibration.
No training or symmetric endpoint-covariance model was introduced here.

## Verification and source amendment

Before any real candidate solve, a new exact-eight-point fixture caught a
reduced-SVD bug: its missing ninth right vector gave an incorrect initialization
at the minimum support. The fix requests the full basis only for fewer than9
matches. The original source is preserved under
`results/bearing-rotation-assay-validation/source-v1/`; the failed log, corrected
hash and amendment are bound in the execution freeze. The21earlier tests used
larger sets and did not cover this boundary. No real prediction or score was
rerun after the correction.

All31synthetic numerical/mapping/filter/decision tests pass before execution.
The freeze binds301source/input/reference files. Earlier references are already
exposed; integrity hashing can read their bytes before execution, but the
prediction code does not use reference values. All predictions and support masks
were locked before this scoring step.

The independent saved-output audit passes: all114method slots,114objective
costs,108mapped rotations/errors and108aggregate values, plus support logic,
coverage and primary decision. Mapping uses an independent matrix-power
calculation; costs use scalar covariance/residual arithmetic. The calibration
loader is shared with the inherited verified scorer. This audit neither reruns
the optimizer nor independently certifies physical calibration/reference truth.

The unchanged recovered runtime is
`sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`.
No new runtime build, package change, model, frame selection or numerical
parameter search was performed.

## Decision and next boundary

Retain **RAIN_DEVELOPMENT_GATE_PASS**. The next justified experiment is the same
frozen assay on existing exposed day/night prefixes, with input/metadata locks
before candidate execution and per-recording gains/coverage required. Do not
tune the prototype on rain or automatically start DNLS training, camera fusion,
a hybrid tracker or production integration.

Unit translation direction is not metric translation. Do not combine these new
rotations with translation gains from the preceding matcher experiment and
claim a full-motion improvement. Translation-direction scoring remains
unavailable without its camera-local reference contract. The current scores
use the supplied references and inherited nominal frame/clock model.

Artifacts: `results/bearing-rotation-assay/run-001/` holds all114attempts,
filter masks, prediction lock, scores and audit. Validation logs and original
source are under `results/bearing-rotation-assay-validation/`. Code:
`run_bearing_rotation_assay.py`, `audit_bearing_rotation_assay.py`,
`test_bearing_rotation_assay.py`, and the corrected `bearing_rotation.py`.
