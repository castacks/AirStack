# Real-data shared measurement benchmark: no advancement

12 November 2026. Branch `ore-ms-orb-development`. Initial freeze9b7c1c1e1;
pre-prediction storage-dtype amendment e6a912bf8. All three exposed first100-group
day/night/rain prefixes,297 nominal adjacent intervals. No new frontend inference,
acquisition, training or physical synchronization qualification.

**Decision: CLOSE_THIS_CONFIGURATION_NO_RETUNING.** The primary original-
covariance joint solver fails every recording's advancement rule. On identical
all-method support it has worse translation and rotation means than the same-
solver RGB-only covariance control in all three conditions. Adding thermal does
not demonstrate a useful improvement in this configuration. This fills the real-
data joint-factor control absent from the checked prior pose-fusion records.
It does not rule out jointly learned measurement influence or other estimators.

## What was new, and what was reused

Earlier full-covariance experiments combined independently completed camera
poses. This run instead places individual RGB and thermal target UV/disparity
residuals in one motion objective. All measurements come from the saved unchanged
native100-frame captures in `results/unweighted-longer/run-001/native`.
Both source depths and correspondence means stay fixed. Original native first-
write poses are retained as context, with equal pose combination also reported.
No frontend/network inference or learned-weight update is performed.

Controls are RGB-only, thermal-only and joint, each with identity native-pixel
covariance or original full target2×2 UV covariance plus target disparity
variance. No per-camera factor normalization or weight fitting. Both weighting
arms use identical source support and require the same valid covariance packet;
an invalid camera packet is unavailable as a whole, never repaired by dropping
selected factors. Source points are backprojected at native K/depth and transformed
into RGB-left optical body coordinates with the supplied rig calibration.
Native NED pose controls receive the explicit optical/body conjugation.

This is a conditional source-landmark prototype. It omits source-landmark
uncertainty, robust kernels and native world-map optimization. The authors'
[MAC-VO objective](https://arxiv.org/html/2409.09479v2) uses spatial covariance
from both endpoints; this experiment is not its full-system reproduction or a
claim that target covariance alone is the complete original noise model.

## Nominal benchmark time contract

The separate adapter explicitly models supplied same-ID groups as nominal
simultaneous stereo/temporal groups, following the dataset's
[provided-pair convention](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format).
Every original integer left/right timestamp is retained in binding and prediction
rows. No camera timestamp is overwritten, no reference bridge or nominal duration
correction is applied, and the prior strict-time prototype guard is unchanged.

The earlier [exact-interval binding](shared-motion-input-contract-findings-2026-11-12.md)
remains closed. This run is a different, declared modeling assumption; it does
not verify physical exposure synchronization. RGB reference poses are joined by
source ID on the same nominal group axis. These results must not be conflated
with earlier duration-transported600-frame comparisons or physical trajectory
accuracy. All recordings are already development-exposed; checkpoint overlap
remains unknown.

## Execution, coverage and safeguards

Eight mechanics tests pass: source/target direction, camera-specific/full-
covariance recovery, independent weighted SciPy fit, residual Jacobian,
observation duplication/order, explicit nominal-contract admission, invalid-
covariance rejection, optical binding/receipt checks and storage-dtype identity
(the recovery test covers multiple arms). CPU float64 NumPy/SciPy; existing
imagefd7be8a3b1fe, BLAS/OMP threads1. Fixed identity initialization and12 GN steps,
diagonal damping1e-8, Jacobian singular-ratio cutoff1e-10. No convergence claim is
inferred from completing the fixed budget. Finite/rank/depth and final objective
checks apply without posthoc changes.

All333 inputs and8 source files are pinned by the amended protocol. Preparation
binds99 RGB and99 thermal packets in day/night; rain binds99 RGB and98 thermal
packets, with one unavailable native first ordinary write retained. Across the
six numerical arms,1,780 solves complete their fixed budgets and two thermal-only
slots are unavailable. Both joint methods retain297/297 output intervals; each
uses the declared RGB-only fallback on the same one rain interval. The three
native controls are reused rather than rerun. Every prediction file is locked
across all conditions before reference values are opened for this run's scoring.

Initial preparation failed before packet creation or pose prediction because a
check compared float32 stored baseline directly to float64 source metadata with
an inappropriate1e-12 tolerance. The values match exactly after the metadata is
cast to the capture storage dtype. Amendment e6a912bf8 changes that check to
exact storage-dtype identity, adds a regression test, and retains the failed
attempt receipt. Inputs, solver parameters and scientific gates are unchanged.
Preparation then succeeds; prediction, lock, scoring and independent audit each
complete on their first attempt. No failed scientific result was retried.

## Primary comparison on identical support

All-method common support is day99, night99, rain98. Translation and rotation
are per-interval errors against the supplied RGB poses, not accumulated ATE.
The rows below use that common support, including native RGB for fair context.

| Recording | Method | Translation mean / RMSE, m | Rotation mean / RMSE, deg |
|---|---|---:|---:|
| day | native_rgb | 0.046390 / 0.052889 | 0.047218 / 0.051366 |
| day | rgb_cov | 0.045538 / 0.051786 | 0.059202 / 0.063692 |
| day | joint_cov | 0.050236 / 0.055649 | 0.062062 / 0.065458 |
| night | native_rgb | 0.022281 / 0.035981 | 0.034836 / 0.052225 |
| night | rgb_cov | 0.020738 / 0.034281 | 0.036297 / 0.053587 |
| night | joint_cov | 0.022050 / 0.035317 | 0.040978 / 0.065168 |
| rain | native_rgb | 0.271722 / 0.504434 | 0.036861 / 0.042467 |
| rain | rgb_cov | 0.188712 / 0.369957 | 0.040485 / 0.046656 |
| rain | joint_cov | 0.192323 / 0.375659 | 0.040611 / 0.046837 |


Rain's joint-cov translation improvement over native RGB is not evidence of
thermal benefit: RGB-only covariance replay improves further on identical
support. The same-solver RGB control is essential to separate frontend/backend
changes from the contribution of the additional camera.

The primary gate requires at least5% lower translation mean and RMSE than each
of native RGB, same-solver RGB-cov and joint-fixed, with no rotation mean/RMSE
increase and no coverage loss, on each recording's matched intervals. Day and
rain pass the comparison with joint-fixed but fail the RGB comparisons; night
fails all three. No secondary arm replaces the declared primary. Own-support,
all-method-common and primary/baseline-matched metrics, including median,p95,
maximum, RMSE and complete rows, remain in the saved score files.

## All numerical controls on the same common support

| Recording | Method | Translation mean / RMSE, m | Rotation mean / RMSE, deg |
|---|---|---:|---:|
| day | rgb_fixed | 0.047131 / 0.053261 | 0.063416 / 0.068865 |
| day | thermal_fixed | 0.298528 / 0.320363 | 0.238572 / 0.269036 |
| day | joint_fixed | 0.077598 / 0.082463 | 0.087096 / 0.092374 |
| day | rgb_cov | 0.045538 / 0.051786 | 0.059202 / 0.063692 |
| day | thermal_cov | 0.297243 / 0.318801 | 0.164522 / 0.182491 |
| day | joint_cov | 0.050236 / 0.055649 | 0.062062 / 0.065458 |
| day | native_rgb | 0.046390 / 0.052889 | 0.047218 / 0.051366 |
| day | native_thermal | 0.295733 / 0.315595 | 0.146984 / 0.160207 |
| day | native_equal | 0.147271 / 0.155816 | 0.084724 / 0.093500 |
| night | rgb_fixed | 0.024477 / 0.033670 | 0.050454 / 0.078808 |
| night | thermal_fixed | 0.059494 / 0.088036 | 0.079651 / 0.125399 |
| night | joint_fixed | 0.027172 / 0.036972 | 0.052983 / 0.081184 |
| night | rgb_cov | 0.020738 / 0.034281 | 0.036297 / 0.053587 |
| night | thermal_cov | 0.057394 / 0.088070 | 0.077642 / 0.125404 |
| night | joint_cov | 0.022050 / 0.035317 | 0.040978 / 0.065168 |
| night | native_rgb | 0.022281 / 0.035981 | 0.034836 / 0.052225 |
| night | native_thermal | 0.063191 / 0.096946 | 0.071294 / 0.116047 |
| night | native_equal | 0.030929 / 0.047810 | 0.044028 / 0.070304 |
| rain | rgb_fixed | 0.171901 / 0.332549 | 0.052091 / 0.059477 |
| rain | thermal_fixed | 0.654444 / 0.786728 | 0.157622 / 0.207716 |
| rain | joint_fixed | 0.253570 / 0.401974 | 0.055031 / 0.063747 |
| rain | rgb_cov | 0.188712 / 0.369957 | 0.040485 / 0.046656 |
| rain | thermal_cov | 0.701627 / 0.814457 | 0.133021 / 0.158905 |
| rain | joint_cov | 0.192323 / 0.375659 | 0.040611 / 0.046837 |
| rain | native_rgb | 0.271722 / 0.504434 | 0.036861 / 0.042467 |
| rain | native_thermal | 0.744435 / 0.849828 | 0.095768 / 0.127011 |
| rain | native_equal | 0.493754 / 0.628874 | 0.055223 / 0.071437 |


## Independent audit and disposition

The saved-output auditor imports no producer/solver code and reruns no optimizer.
It independently projects every completed pose through camera geometry,
reconstructs identity/full-covariance quadratic costs, checks body-transform
inverses and retained timestamps, and recomputes errors, statistics, common
support and the primary decision. All13,374 checks pass;1,780 objective costs
agree within their fixed tolerances. Maximum cost discrepancy is8.54e-6 of the
allowed tolerance. This audit does not independently rerun feature extraction
or rederive every packet covariance from the raw network tensors.

Close this exact fixed-mean/original-covariance conditional solver configuration.
Do not add iterations, tune modality weights, change masks, switch reference
alignment or train a new head as an automatic rescue. The new result supplies
an explicit joint-measurement baseline for future hypothesis selection; it does
not exhaust joint estimation, task-aligned learning or multisensor VO.

Protocol: `experiments/nominal-joint-motion-freeze-v2-2026-11-12.json`;
initial declaration remains in the unversioned freeze file. Outcome:
`experiments/nominal-joint-motion-outcome-2026-11-12.json`.
Raw immutable binding packets, source associations, prediction locks, all
condition-wise predictions/scores and failed-preparation receipt:
`results/nominal-joint-motion/run-001`. No runtime or real-time performance metric
is claimed.
