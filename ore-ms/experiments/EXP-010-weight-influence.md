# EXP-010: Common-pose pixel precision and local influence

Date: October 19, 2026 (America/New_York). Branch `ore_ms`; base commit
`3ac07f047` plus the diagnostic implementation. **All six real-data cases completed.**

## Question and frozen diagnostic

EXP-009 found a large late thermal translation change under fixed pixel
variance, together with worse rotation. Examine whether learned pixel precision
concentrates on particular image regions or forward depths and how the local
pose linearization responds to deleting those groups. Include early/original
windows and RGB controls. Do not change calibration, matches, depth, selection,
preprocessing, native optimizer defaults or frozen weight constants.

The offline tool consumes the EXP-006 tensor maps and EXP-009 frozen plan.
Learned and fixed_uv arms use the **same original fitted target pose**, K,
baseline, source world points, observations and target disparity variance.
Fixing only pixel variance isolates its effect on the common-pose diagnostic.
The fixed_uv linearization need not be stationary at that pose; full-step
subtraction accounts for its nonzero gradient in the deletion diagnostic.

Predeclared groups are image row and column thirds plus forward-depth intervals
[0,10), [10,20), [20,50), [50,100), [100,infinity) meters. No depth cap.
Forward depth is camera NED x, not Euclidean range. Image sizes must be supplied
from actual camera metadata. Behind-camera points and nonzero uv covariance
cross terms are rejected, not dropped. No reference enters this analysis.

For each observation, whiten the analytic NED projection/disparity Jacobian
and residual by channel standard deviations. Report channel precision shares
and trace blocks of the OLS hat matrix `A @ pinv(A)`. At full column rank their
leverage sum is six. Report the difference between full and group-deleted
linear least-squares updates, as translation norm in meters and rotation-vector
norm in degrees. If either design lacks rank six, deletion sensitivity is null.
All Jacobians use the right local camera tangent, meters/radians.

These quantities precede Huber and exclude damping, native step acceptance,
stopping and nonlinear reoptimization. They describe a local linearized system,
not actual native solver influence, calibrated pose uncertainty or a new VO
trajectory. Group deletion can produce steps outside the linear regime. Pose
coordinates, numerical rank thresholds and source geometry affect the results.
Selected features, provisional thermal geometry and unknown preprocessing
remain limitations. Raw sums across overlapping partitions are invalid.

## Validation and execution status

Four synthetic tests pass: finite-difference pose Jacobians, precision shares
and rank-six leverage, deletion versus direct linear least-squares solutions,
invalid covariance/behind-camera rejection, and saved-graph grouping/input
preservation. These are implementation checks, not real-data findings.

Box access was reauthorized. Restored selected inputs from folder
`428559630945`, `backups/2026-10-18-3ac07f047/research-artifacts.tar.gz`.
The archive is 109,719,262 bytes and its SHA-256 matches the saved manifest:
`3a4e08fb9851dcdeac3a185672dab2ceb3d4d9845b13d789b010583d19f2ec49`.
All 18 EXP-009 source hashes match before and after analysis. Image dimensions
come from the restored EXP-001 input audit: RGB 544×1024, thermal 512×640;
retained target observations are within those bounds. No full dataset or
checkpoint download was needed.

All six window/modality cases complete: 114 consecutive pairs, two weight
arms each. Full Jacobians and every group-deleted Jacobian have rank six.
Leverage sums to six, and per-pair count, precision and leverage shares sum
to one within each partition. Source inputs remain unchanged. Outputs are
`results/weight-influence/run-001/{early,original,late}-{rgb,thermal}.json`,
`summary.csv`, `summary.json`, and `precision-by-row.png/pdf`. The local
`summarize.py` regenerates the summaries and figure. Each diagnostic JSON
records source/plan/script hashes, dimensions, NumPy version and UTC time.

## Measured concentration and local sensitivity

All numbers below are **unweighted medians over 19 consecutive pairs** per
window. These are dependent observations from one clip, not independent
replicates. Marginal medians across groups need not sum to one. Precision is
inverse active variance, including the existing pixel floor, not raw network
confidence. Fixed-uv precision shares equal count shares within each channel.

Upper image third:

| Window | Modality | Feature share (%) | Learned u precision (%) | Learned v precision (%) | Learned leverage (%) | Fixed-uv leverage (%) |
|---|---|---:|---:|---:|---:|---:|
| early | rgb | 37.8 | 26.3 | 32.0 | 29.6 | 32.1 |
| early | thermal | 41.1 | 41.5 | 44.5 | 29.3 | 30.5 |
| original | rgb | 32.3 | 12.3 | 24.8 | 21.3 | 29.0 |
| original | thermal | 45.2 | 36.7 | 45.2 | 29.3 | 33.2 |
| late | rgb | 51.3 | 39.7 | 51.4 | 39.3 | 39.3 |
| late | thermal | 56.6 | 74.0 | 73.5 | 50.8 | 39.9 |

Late thermal's lower third contains 10.9% of features but receives only 1.4%
of learned u precision and 0.4% of v precision. Its leverage share rises from
9.4% learned to 25.4% fixed_uv. Points within 10 m forward depth account for
30.4% of features but only 13.2%/9.2% of learned u/v precision; their leverage
share rises from 36.9% to 51.5% under fixed_uv. These show strong redistribution
conditional on the fixed selected features and original fitted geometry.

Thermal upper-third deletion sensitivity (difference of full and deleted
linear least-squares steps, **not measured pose error or nonlinear replay**):

| Window | Learned translation (m) | Fixed-uv translation (m) | Learned rotation (deg) | Fixed-uv rotation (deg) |
|---|---:|---:|---:|---:|
| early | 0.0398 | 0.0340 | 0.2636 | 0.3569 |
| original | 0.0348 | 0.0605 | 0.2284 | 0.2641 |
| late | 0.3427 | 0.1567 | 0.6226 | 0.2827 |

Late thermal's upper-third local sensitivity is much larger than in its two
controls. Fixed pixel weights reduce this upper-third sensitivity while
increasing leverage on lower/near points. This supports a bounded hypothesis
about weight distribution and scene geometry; it does not establish that the
upper region is wrong, justify masking it, or identify the cause of drift.
Leverage includes disparity rows, whose weights remain unchanged. Fixed_uv
also changes total channel precision and its strength relative to disparity;
this experiment does not isolate redistribution from that change.

EXP-009 still supplies the nonlinear replay evidence: late thermal translation
RMS improves 2.897→0.790 m but rotation RMS worsens 1.713→4.375 degrees against
recorded SLAM. No new trajectory errors are measured in EXP-010. Do not infer
that reducing local sensitivity improves accuracy or that fixed weights should
be adopted. The figure is a descriptive precision-share view; use it with the
leverage and translation/rotation tables.

## Decision and next controlled test

Keep current VO defaults. Before trying image masks or a learned fusion method,
use native fixed-input replay to separate pixel precision mass from distribution:
retain learned and fixed_uv controls, then rescale fixed u/v precision per pair
to match the learned total separately in each channel, preserving disparity
weights, matches, points, initialization and native solver. Declare the rule
before evaluation, use no reference to choose constants, and report both local
and accumulated translation/rotation across all three windows and both modalities.
This is a proposed follow-up, not an adopted method or executed experiment.
Require a separate clip before changing defaults or claiming cross-modal
confidence comparability. Provisional geometry, unknown thermal preprocessing
and the recorded-SLAM proxy remain unresolved.
