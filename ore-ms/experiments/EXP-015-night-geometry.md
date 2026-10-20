# EXP-015: Night fixed-grid uncertainty versus geometric consistency

Date: 2026-10-20 UTC. Branch `ore_ms`, base commit `3ac07f047` plus frozen working source. Completed.

RGB uncertainty's window-level surge accompanies large reference-conditioned geometric residuals, including depth-independent epipolar disagreement. This adds geometric evidence beyond EXP-014's covariance-dependent selection collapse. Individual-point ranking remains inconsistent, so no calibrated weighting or cross-modal reliability rule is adopted.

| Camera/window | Trace px² | Reprojection px | Sampson px | ρ reprojection | ρ Sampson |
|---|---:|---:|---:|---:|---:|
| rgb/early | 5.743 | 5.668 | 0.776 | -0.079 | 0.035 |
| thermal/early | 1.103 | 2.393 | 0.963 | 0.554 | 0.345 |
| rgb/middle | 6499.885 | 232.417 | 47.578 | 0.292 | -0.134 |
| thermal/middle | 15.464 | 7.266 | 2.167 | 0.681 | 0.351 |
| rgb/late | 6435.585 | 139.061 | 35.687 | 0.545 | 0.371 |
| thermal/late | 0.538 | 0.779 | 0.035 | 0.133 | -0.163 |

Values are unweighted medians over 19 pair medians or within-pair Spearman correlations. Pixel units are camera native; this table does not establish physical cross-camera comparability. All 114 grid pairs have defined correlations. RGB middle's negative epipolar correlation (-0.134) and thermal late's (-0.163) prevent a universal within-pair ranking claim. Thermal reprojection correlations are positive but vary from 0.133 to 0.681.

## Frozen measurement

Use all six EXP-014 cases, starts 123/619/789, with original predictions and camera acquisition timestamps. No new inference, scale fit, alignment, threshold tuning, or VO changes. Frozen `plan.json` checks 631 source/input hashes before and after analysis. Read recorded `super_odometry` vehicle poses, interpolate translation/Slerp without extrapolation, apply unchanged camera extrinsics/rectification and compute camera1-from-camera0. Reuse unchanged EXP-007 `pair_geometry`.

Primary population: all fixed-grid source points (1800 RGB or 1008 thermal per pair), including points unavailable to native selection. Grid samples are not individually labeled selector rejects: the sparse diagnostic grid differs from dense NMS candidates. Selected/retained points are a separate population, never a substitute. Preserve 228 population NPZs, including empty retained populations from failed RGB. No trajectory is required for late RGB.

Reprojection uses positive finite source forward depth and positive transformed depth; target depth is not required. Sampson residual is square-root Sampson distance in pixels, computed from finite coordinates independently of either depth. Translation <=1e-6m is declared epipolar-degenerate; none of the 114 pairs meet this criterion. A nonzero translation does not guarantee a well-conditioned epipolar reference. Store all points and validity masks, including target-out-of-image predictions, which are descriptive predictions rather than verified visible matches.

Sensitivity populations are inbound and inbound with both positive depths <=50m. For RGB, inbound-only reprojection medians are 5.736/247.609/139.716px and Sampson 0.778/52.240/33.742px, so the window-level result persists. Both-depth restriction gives 6.658/247.609/139.186px and 0.800/52.240/33.742px. All-grid counts are 34200 RGB and 19152 thermal per window, all with finite residuals. Inbound counts RGB:33783/29797/30718; thermal:18433/17162/18784. Both-depth counts RGB:30977/29493/29863; thermal:16785/15677/17765.

Report source image thirds and forward-depth bins [0,10), [10,20), [20,50), [50,infinity), invalid, in analysis.json/cases.csv. Pair-level translation bins [0,.01), [.01,.1), [.1,infinity)m contain 1/31/25 pairs per camera. Rotation bins [0,.1), [.1,1), [1,infinity)degrees contain22/34/1. These strata are uneven and confounded with window/motion/scene. No significance tests, pooled-point inference or causal attribution.

## Validation and limits

54 focused/regression tests pass. Synthetic geometry checks cover exact motion, erroneous flow, depth independence, invalid depth, behind-camera projections, pure-rotation epipolar degeneracy, masks and empty populations. All 631 hashes remain unchanged; 114 source pairs produce 228 population files and160056 fixed-grid observations. The figure was visually inspected.

Reference is recorded SLAM, not independent ground truth; dynamic TF is the same reference. Calibration retains the known thermal stereo physical inconsistency (0.6701m horizontal discrepancy, 0.617degree rotation discrepancy). Original thermal processing and checkpoint training overlap remain unresolved. Reprojection combines depth and flow error; epipolar residual also depends on pose/calibration and scene motion. Three selected windows cannot establish generalization or probability calibration. Keep defaults.

Next bounded analysis: measure temporal, within-window covariation and repeatability against simple flow/motion/depth context, without fitting a fusion rule. Independent reference and additional recordings remain necessary for a cross-modal accuracy claim.

Artifacts: `results/night-geometry/run-001/` (plan, analysis, cases.csv, points, validation, PNG/PDF). Runner: `MAC-VO-main/Scripts/Setup/td3_night_geometry.py`. Recovery snapshot: Box `backups/2026-10-20-exp-015`, requiring EXP-013 and EXP-014 snapshots plus original tree/models as described in its manifest.
