# EXP-007: Diagnose the late thermal failure

Date: October 18, 2026 (America/New_York). Branch `ore_ms`; base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` plus uncommitted research changes.

## Findings

The late thermal trajectory substantially underestimates recorded reference
travel (3.540 m estimated versus 8.747 m reference net displacement). The
original window overestimates it (11.377 versus 8.857 m). Yet the late window
has smaller median reference-conditioned reprojection, epipolar and disparity
residuals. A larger pooled residual is therefore not a useful explanation for
this failure. No new fitted scalar, pose correction or calibration change was
made.

Primary retained population: both positive endpoint depths ≤50 m. All values
below are pooled medians over 19 consecutive pairs; sample counts are dependent.

| Window | Modality | Matches | Depth0 (m) | Disparity0 (px) | Reprojection (px) | Epipolar Sampson (px) | Disparity residual (px) |
|---|---|---:|---:|---:|---:|---:|---:|
| early | rgb | 2917 | 5.224 | 19.172 | 3.157 | 1.443 | 0.133 |
| early | thermal | 2628 | 13.035 | 8.946 | 7.603 | 1.886 | 0.444 |
| original | rgb | 2872 | 4.576 | 21.884 | 2.446 | 1.004 | 0.154 |
| original | thermal | 3081 | 13.503 | 8.636 | 6.655 | 1.055 | 0.438 |
| late | rgb | 2125 | 4.639 | 21.589 | 1.682 | 0.510 | 0.187 |
| late | thermal | 2993 | 14.143 | 8.245 | 5.688 | 0.468 | 0.323 |

Fixed-grid thermal measurements confirm the direction of the change: original
versus late median reprojection 6.632→5.976 px and epipolar 1.083→0.543 px.
Lower epipolar error does not establish correct flow magnitude: a match can
slide along its epipolar line while remaining epipolar-consistent. Reprojection
adds depth and reference translation, but is still conditional on provisional
geometry and recorded SLAM.

## Spatial and range changes

Image regions are equal thirds in width and height, numbered 0–8 row-major.
These fractions are conditional on the primary depth cap, not all candidates.

| Thermal window | Upper third (%) | Bottom third (%) |
|---|---:|---:|
| early | 43.5 | 21.5 |
| original | 46.7 | 13.3 |
| late | 59.1 | 5.6 |

The late retained population shifts toward upper image regions and away from
near foreground. Retained source depths 0–5 m contribute 411/2993 (13.7%), versus
539/3081 (17.5%) in the original window. Counts in some bottom regions are
small (late bottom-right 11 matches across the window). A visual contact sheet
shows persistent dark structure across the upper thermal images and vehicle
body near the bottom. This is a qualitative observation; those regions were
not semantically labeled or masked, and their effect is not isolated here.

Range stratification also changes the interpretation. In the late thermal
window, median reference reprojection is 8.404 px at source depth 0–5 m,
9.015 px at 5–10 m,4.898 px at 10–20 m and1.585 px at 20–50 m. These strata are
still mixtures of different image regions and scene content. They do not prove
that stereo depth, thermal preprocessing, calibration or feature coverage is
the cause of the pose failure.

## Active objective: correction to the earlier interpretation

All six saved configurations specify `graph_type=disp` and `autodiff=false`.
`Analytic_ReprojDisp_TwoFramePGO` uses a three-component residual:
`[predicted_u-observed_u, predicted_v-observed_v, predicted_disparity-observed_disparity]`.
Its covariance contains target pixel covariance and target disparity variance.
The optimizer uses its pseudoinverse and a Huber kernel/corrector (delta 0.1).
See `Module/Optimization/TwoFramePGO/{Graphs,Optimizer}.py`.

**Projected 3D covariance and the local depth-variance floor are not direct pose
weights in this backend.** They are constructed/stored and participate in
covariance sanity filtering. The pixel floor does affect active weights through
the existing in-place clamp. EXP-005/006's floor measurements remain valid,
but do not explain the pose result through 3D optimization weights here.

As an additional diagnostic, squared image/disparity residuals are divided by
their saved diagonal variances. These energies omit robustification, source
uncertainty and correlations; they are not the actual optimizer loss, Hessian,
or effective influence. At final saved VO poses, thermal disparity contributes
1.84% of summed energy in the original primary population and 3.74% late;
median target disparity residuals are 0.388 px original and0.207 px late. This
does not establish that disparity weights caused the late failure.

## Methods and validation

`Scripts/Setup/td3_reprojection.py` reuses the EXP-006 arrays and trajectories,
with no inference. For camera1-from-camera0 reference motion `(R,t)`, unproject
`p0` from recorded depth, predict `p1=R p0+t`, and project into camera1. Forward
residual is observed minus predicted pixel; reverse residual uses the inverse
motion and endpoint1 depth. Sampson distance uses
`F=K^-T [t_optical]_x R_optical K^-1`, independent of predicted depth. Depth
behind camera, nonfinite projection and degenerate epipolar denominators are
marked invalid. Their finite counts are reported separately per metric.

Grid disparity is algebraically derived as `fx*execution_baseline/depth`;
it is not a separate stereo measurement. Retained disparity is checked against
the graph's saved model magnitude. Signed disparity was not preserved in these
runs; this experiment cannot resolve the signed thermal calibration question.
Per-observation arrays retain uncapped values and masks. Pair timestamps,
retained UV associations and saved/derived disparity must match exactly or
within numerical tolerance before summary generation.

Five new tests pass: exact rigid geometry, depth/epipolar separation,
perpendicular match perturbation, behind-camera/zero-baseline rejection,
rotated-camera basis and inverse direction, plus agreement with the active
backend's synthetic residual and covariance. (Related assertions share test
functions.) The backend test also confirms that multiplying stored 3D
covariances leaves the disparity graph's covariance unchanged. Previous 27
research tests remain unchanged.

Final artifacts: `results/thermal-failure/run-002/analysis.json`, modality/window
`{grid,retained}.npz`, `diagnostics.png` and `representative-matches.png`.
`run-001` preserves the initial reprojection-only analysis. All six final offline
analyses completed with timestamp/observation/disparity checks passing. Source
and input SHA-256 values accompany the results. `backend-audit.json` additionally
records the inspected backend source hashes. Reproduction command is in the
setup README.

The limitation remains one clip, dependent observations, recorded SLAM proxy,
provisional calibration, changed scene content and unmatched RGB/thermal
feature populations. No calibration, confidence-ranking or causal conclusion
is supported.

## Next controlled experiment

Replay the same saved feature pairs through reprojection-only versus the
current disparity backend, with fixed initialization/configuration and no
new feature selection or scale fitting, on all three windows. Compare local
motion, image/disparity residuals and conditioning with the recorded reference.
This isolates whether adding the disparity term changes the failure before
changing preprocessing, calibration, feature masks or fusion logic.
