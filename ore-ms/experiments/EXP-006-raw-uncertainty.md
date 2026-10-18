# EXP-006: Raw uncertainty and separate-window scale checks

Date: October 18, 2026 (America/New_York). Branch `ore_ms`, base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` plus uncommitted research changes.

## Question and protocol

EXP-005 measured constructed 3D covariance after feature selection and floors.
This experiment records raw learned temporal flow covariance before those
operations and checks whether the prior window's translation scale transfers
to disjoint windows of the same clip. No model, depth, calibration, trajectory,
or optimizer corrections are applied.

`results/uncertainty/run-001/plan.json` was written before inference on the new
windows. Select the maximum body-reference endpoint displacement among
continuous 20-frame windows within the first and last manifest quarters,
respectively; require RGB gaps ≤150 ms. This selects starts 306 and 1132,
compared with original 348. The segments span approximately 1.90 s each, with
reference body displacement 7.545 m and 8.743 m. These are disjoint samples of
one clip, sharing conditions, calibration and a recorded SLAM proxy. They are
not independent dataset validation or ground truth.

Freeze the translation scalar fitted only on original frames 0–9:
RGB 0.9751532169, thermal 0.7682596439. Anchor each evaluation trajectory at
its own first pose; multiply only estimated translation by the frozen scalar,
then measure disagreement with its camera reference at actual acquisition
timestamps. No rotation fit or scale fit uses the evaluation windows.

## Instrumentation and interpretation

`run_td3_execution.py --record-uncertainty` enables an observer for this bounded,
synchronous, all-keyframe, mapping-disabled runner. At each pair it records:

- A fixed 16 px grid with the configured 32 px border, before selection, and
  the covariance-aware selector's candidate locations.
- Predicted temporal flow covariance (`uu,vv,uv`, pixel squared units), depth
  at both endpoints, and inbound / outlier retention masks.
- At both endpoints, pixel covariance before/after the 0.0625px² diagonal
  floor, and local weighted depth variance before/after the 0.05 m² floor.
  Endpoint 0 pixel covariance is a fixed quantization prior, not learned.

Arrays are detached copies. Callback mutation cannot change inference; this
is tested synthetically. Replaying original 20 frames in both modalities
produced graphs bitwise equal to EXP-004, including poses and observations.

Raw-variance association uses the same two-view 3D consistency residual as
EXP-005, not directly measured optical-flow error. Depth, motion, calibration,
reference error, moving objects and vehicle pixels all affect it. This report
uses the ≤50 m cap on both endpoint depths declared before new inference.
All recorded arrays, including distant or invalid samples, remain saved.
The grid population is filtered by target bounds and finite positive depths;
selected/retained populations additionally reflect backend selection. Grid
sampling does not make those populations spatially matched.

Rank statistics include pooled and median within-pair depth-partial ranks
(linearly residualized ranks controlling mean endpoint depth). Temporal and
spatial dependence preclude interpreting sample counts as independent trials.
None of these correlations establishes calibrated uncertainty or fusion gains.

## Results and decision

| Separate window | Modality | Original translation RMS (m) | Frozen-scale RMS (m) | Rotation RMS (deg) |
|---|---|---:|---:|---:|
| early | rgb | 0.090 | 0.127 | 0.383 |
| early | thermal | 0.703 | 0.545 | 4.176 |
| late | rgb | 0.100 | 0.096 | 0.328 |
| late | thermal | 2.897 | 3.391 | 1.713 |

The thermal scalar helps modestly on the early window and worsens the late
window. RGB also worsens early. The large within-window gain in EXP-005 does
not transfer uniformly. Do not adopt a global translation/depth scale correction.
Scaling does not change rotations. These numbers measure proxy disagreement,
not independently known estimation accuracy.

Median within-pair depth-partial rank of raw temporal covariance trace versus
3D consistency residual, with both depths ≤50 m:

| Window | Modality | Fixed grid | Selected, inbound | Retained |
|---|---|---:|---:|---:|
| original | rgb | 0.409 | 0.268 | 0.268 |
| original | thermal | 0.273 | 0.147 | 0.147 |
| early | rgb | 0.377 | 0.276 | 0.276 |
| early | thermal | 0.200 | 0.199 | 0.199 |
| late | rgb | 0.436 | 0.311 | 0.311 |
| late | thermal | 0.076 | 0.081 | 0.081 |

Every inbound candidate passed the covariance sanity outlier filter in these
runs, so selected-inbound and retained associations coincide. This filter
checks covariance validity; it does not remove all geometrically bad matches.
Raw thermal association weakens markedly in the late window. Cross-modal
comparability and a reliable modality score remain unestablished.

On the original window, 52.5% of RGB and 37.6% of thermal learned flow diagonal
components are below the pixel floor before projection. Endpoint depth
variances fall below their floor in 75.1% / 78.7% of RGB endpoint0 / endpoint1
samples and 84.7% / 88.7% of thermal samples. The observer confirms those floors
are applied as expected. A floored 3D covariance trace conflates these effects
with learned variance and depth-dependent projection.

Validation: all six runs have 20 finite poses, exact acquisition timestamps,
positive-definite retained observation covariance and no interpolated frames.
Both original graphs are bitwise equal with recording on/off. 27 focused tests
pass, including a callback mutation-isolation test and an explicit candidate /
inbound / retained-mask association test.

**Next measurement:** inspect the late thermal failure before proposing fusion
or recalibration. Compare reference-conditioned reprojection residuals and
stereo disparity/depth distributions across the three windows, stratified by
image region and range, with original calibration and preprocessing preserved.
This can test whether the failure tracks depth/scene content or temporal
matching without fitting another scalar to the evaluation window.

## Artifacts and reproduction

Artifacts: `results/uncertainty/run-001/{plan.json,analysis.json,scale-check.png}`
and `{original,early,late}/{rgb,thermal}/`. Each run includes per-pair NPZ arrays,
metadata, original graph/trajectory, input rows, model hashes and effective
configuration. Analysis includes source hashes and training trajectory hashes.

Use `Scripts/Setup/td3_uncertainty_check.py prepare`, run the six 20-frame
recorded runs (original 348, early 306, late 1132; both modalities), then use
`evaluate` with the same dataset/manifest/calibration/training/output arguments.
See the setup README for exact shared arguments. Output directories refuse
replacement. Original trajectories are preserved; scale is an analysis only.

## Backend clarification from EXP-007

These runs use the `disp` backend: image and disparity residuals are weighted
by target pixel/disparity covariance. The measured projected 3D covariance and
local depth-variance floors are diagnostic quantities, not direct pose weights
in this configuration. Pixel floors do affect active weights. See
[EXP-007](EXP-007-thermal-failure.md) for the source audit and residual tests.
