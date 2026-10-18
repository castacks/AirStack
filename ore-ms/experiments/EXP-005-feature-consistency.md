# EXP-005: Retained-feature consistency and thermal scale sensitivity

Date: October 18, 2026 (America/New_York). Branch `ore_ms`, base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` plus the uncommitted diagnostic script.
Inputs are the unchanged 20-frame moving-window graphs and proxy-reference
camera poses from EXP-004. No new model inference, model training, fusion,
pose correction, or source depth/calibration changes were performed.

## Main findings and interpretation

The thermal trajectory's larger disagreement has a strong scale-sensitive
component on this segment. A translation scalar fitted on its first ten frames
reduces disagreement on its next ten frames from 2.012 m RMS to 0.167 m RMS.
This adjacent temporal split is not independent validation. It supports testing
a scale hypothesis; it does not establish a calibrated correction or identify
whether depth, preprocessing, geometry, or reference error caused the difference.
Rotational disagreement is unaffected by translation scaling.

Constructed covariance trace has a positive pooled association with feature
consistency residual in both modalities. That relationship depends strongly on
depth range and sample selection. In addition, 75–89% of the depth-variance
components are at the configured minimum. These measurements cannot be treated
as raw learned confidence or used yet as evidence for adaptive RGB/thermal fusion.

## Measurement definition

For each retained temporal match, reconstruct `p0` and `p1` in their respective
camera NED coordinates from the saved `(u,v,depth)` and per-frame K. Use explicit
graph match-to-frame indices and the proxy's camera poses at the actual
acquisition times. Define:

`r = p1 - (R_proxy_1_from_0 p0 + t_proxy_1_from_0)`

This is two-view 3D consistency in meters, conditional on model stereo depths,
static-scene behavior, reference motion, and provisional calibration. It is not
single-feature error against an independently known 3D point. It can be affected
by correspondence/depth error, reference error, calibration, timing, and moving
or vehicle-attached features. No additional dynamic-object mask was applied.

Approximate residual covariance:

`C_approx = C1 + R_proxy C0 R_proxy.T`

Endpoint dependence, flow/depth correlations, and reference/calibration/time
uncertainty are omitted. Cholesky whitening is used only for a normalized-residual
diagnostic; no chi-square/NIS coverage or probabilistic calibration claim is made.
All 7,059 retained observations have finite positive depths and a finite
positive-definite combined covariance, across 19 temporal frame pairs per modality.
The arrays preserve original match/frame indices and acquisition timestamps.

## Raw and depth-controlled associations

| Modality | Retained observations | Median residual (m) | Median endpoint-mean depth (m) | Pooled Spearman | Pooled depth-partial rank |
|---|---:|---:|---:|---:|---:|
| RGB | 3734 | 0.0625 | 5.60 | 0.621 | 0.413 |
| THERMAL | 3325 | 0.4354 | 13.96 | 0.404 | 0.180 |

Spearman compares propagated covariance trace to Euclidean residual magnitude.
The depth-partial statistic linearly removes endpoint-mean-depth ranks from both
rank variables; it is an approximate confound diagnostic, not a causal adjustment.
No p-values or independence assumptions over thousands of spatially/temporally
related features are used. RGB and thermal do not observe paired identical
features; their retained depth distributions and fields of view differ.

The following range sweep was added after seeing extreme distant-depth tails.
It is an explicit exploratory sensitivity check, not a predeclared evaluation
split. It keeps the unfiltered all-retained analysis and does not modify VO.
Both endpoint depths must satisfy each common bound:

| Modality | Both endpoint depth cap (m) | Count | Median residual (m) | Median within-pair Spearman | Median within-pair depth-partial rank |
|---|---:|---:|---:|---:|---:|
| RGB | 10 | 2407 | 0.0386 | 0.201 | 0.325 |
| RGB | 20 | 2725 | 0.0416 | 0.107 | 0.294 |
| RGB | 50 | 2872 | 0.0432 | 0.204 | 0.321 |
| RGB | 100 | 3000 | 0.0452 | 0.290 | 0.374 |
| THERMAL | 10 | 1127 | 0.2560 | -0.269 | -0.303 |
| THERMAL | 20 | 2676 | 0.3681 | 0.145 | 0.055 |
| THERMAL | 50 | 3081 | 0.4041 | 0.247 | 0.148 |
| THERMAL | 100 | 3207 | 0.4233 | 0.337 | 0.180 |

For example, within the 50 m cap the median within-pair depth-partial rank
association is about 0.321 RGB and 0.148 thermal. Thermal's near-range association
is negative and changes with the depth cap. This is not a stable cross-modal
reliability model, and feature selection/outlier retention already depend on
predicted uncertainty.

## Floors, distant tails, and scale fits

The configured depth-variance minimum is 0.05 m². The first diagonal entry of
the stored camera-NED covariance corresponds to that depth variance:

| Modality | First endpoint at variance floor | Second endpoint at variance floor |
|---|---:|---:|
| RGB | 75.1% | 78.7% |
| THERMAL | 84.7% | 88.7% |

The 3D covariances also include a default first-observation pixel prior, predicted
flow covariance after minimum clamping, and local depth-patch statistics. A floor
on one component does not mean the entire matrix is constant or floor-dominated.

Maximum predicted endpoint depths exceed 211 km RGB and 385 km thermal in the
retained graph. These are near-zero-disparity/distant observations; unweighted
squared 3D residuals are dominated by their tails. All-retained unweighted
least-squares depth-scale fits collapse near zero or negative and are explicitly
not usable physical calibration. Even bounded-depth fits are range-sensitive.
This does not by itself prove the original covariance-weighted optimizer is
wrong: the diagnostic's unweighted Euclidean objective differs from its objective.
No such fit is applied to outputs or model weights.

Trajectory sensitivity is a different diagnostic. After first-pose anchoring,
fit `s` to `sum ||s * estimated_translation - proxy_translation||²` on the first
half and evaluate the second half without refitting:

| Modality | Scalar fitted on first 10 frames | Last 10 raw RMS (m) | Last 10 scalar-adjusted RMS (m) |
|---|---:|---:|---:|
| RGB | 0.9752 | 0.1654 | 0.0823 |
| THERMAL | 0.7683 | 2.0115 | 0.1669 |

All-frame in-sample fits are also retained: approximately 0.979 RGB and 0.774
thermal, reducing full-window translation RMS to about 0.061 m and 0.133 m.
No rotation/SE(3) fitting or baseline rewrite is performed. Same-segment temporal
dependence, selected high-motion window, and uncertain geometry/reference mean
these are diagnostic fits, not validated calibration parameters.

## Validation and reproduction

25 focused tests pass. New cases cover exact rigid residuals, covariance rotation,
known Mahalanobis distance, singular covariance rejection, known depth-scale
recovery, NED unprojection, constant-rank handling, and verifying that sequential
scale fitting never uses the test-half samples.

Inside the existing runtime, from `MAC-VO-main/`, use a fresh output directory:

```bash
python Scripts/Setup/td3_feature_consistency.py \
  --trajectory-root ../results/moving-vo/run-001 \
  --output ../results/feature-consistency/new-run --max-depths-m 10 20 50 100
python -m pytest Scripts/UnitTest/test_research_setup.py \
  Scripts/UnitTest/test_td3_sequence.py Scripts/UnitTest/test_td3_reference.py \
  Scripts/UnitTest/test_td3_features.py -q
```

Final local artifacts are under `results/feature-consistency/run-004/`:
per-modality `observations.npz` and `report.json`, shared `run-meta.json`, and
`diagnostic.png`. The plotting script is `results/feature-consistency/plot_diagnostic.py`.
Earlier run directories retain stages before range/floor/sequential diagnostics.
Metadata records graph/reference/config and analysis-source hashes. The final
report scope wording was clarified after execution; numeric outputs are unchanged.

## Next research step

Instrument raw learned flow/covariance before backend clamping and depth-mixture
construction, and record all candidate observations before selection. Validate
thermal scale and preprocessing on separate predeclared windows, including
reference/calibration sensitivity. Then evaluate a clearly defined error target
with measured reference quality before learning or claiming cross-modal reliability.

## Backend clarification from EXP-007

These runs use the `disp` backend: image and disparity residuals are weighted
by target pixel/disparity covariance. The measured projected 3D covariance and
local depth-variance floors are diagnostic quantities, not direct pose weights
in this configuration. Pixel floors do affect active weights. See
[EXP-007](EXP-007-thermal-failure.md) for the source audit and residual tests.
