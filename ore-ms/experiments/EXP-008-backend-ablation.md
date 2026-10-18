# EXP-008: Fixed-match reprojection versus disparity backend

Date: October 18, 2026 (America/New_York). Branch `ore_ms`; base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` plus uncommitted research changes.

## Result and decision

Removing the target-disparity residual does not resolve the late thermal
failure. Its translation disagreement with recorded SLAM slightly worsens,
from 2.897 m to 2.948 m RMS; estimated net travel remains much too small,
3.540 m versus 3.482 m respectively, against 8.747 m recorded reference travel.
The early/original thermal windows improve modestly, without recovering the
RGB level of disagreement. No backend default, confidence model, preprocessing,
calibration or saved source trajectory is changed.

All values below use first-pose anchoring, with no fitted alignment or scale.
These are composed independent pair estimates, not fresh end-to-end VO runs.

| Window | Modality | Disparity translation RMS (m) | Reprojection-only RMS (m) | Disparity local translation RMS (m) | Reprojection-only local RMS (m) |
|---|---|---:|---:|---:|---:|
| early | rgb | 0.0904 | 0.0900 | 0.0439 | 0.0440 |
| early | thermal | 0.7027 | 0.5899 | 0.1169 | 0.1067 |
| original | rgb | 0.1277 | 0.1227 | 0.0309 | 0.0307 |
| original | thermal | 1.5171 | 1.4465 | 0.1415 | 0.1350 |
| late | rgb | 0.0995 | 0.0978 | 0.0256 | 0.0254 |
| late | thermal | 2.8966 | 2.9478 | 0.3447 | 0.3467 |

## Replay control and fixed-input protocol

Use the six saved 20-frame runs from EXP-006. Each of their 19 consecutive
pairs uses the saved world points (via explicit match-to-point indices),
target observations and covariance, target K and baseline, and saved prior
camera pose as the common static initializer. Raw graph inputs are copied
before each optimization. SHA-256 fingerprints agree between both arms;
after optimization the inputs must remain unchanged.

Each arm uses the native analytic graph, double-precision CPU optimization,
pseudoinverse covariance weighting, LM minimum 1e-6, PINV solver, trust radius
1e3, Huber/FastTriggs delta 0.1, and native StopOnPlateau settings
(steps 10, patience 2, decreasing 1e-5). Only `graph_type` differs. The small
research loop mirrors the native optimizer loop, omitting runtime timers and
recording losses. No reference poses enter optimization.

The original disparity backend is replayed as a control. Maximum local
translation deviation from the original across all 114 pairs is
4.56e-07 m; maximum rotation deviation is 2.76e-06 degrees. All six
controls pass the prespecified 1 mm / 0.01 degree local tolerance.

For each solved world-from-camera1 pose, compute
`camera0_from_camera1 = inverse(saved_world_from_camera0) @ solved_world_from_camera1`.
Compose those increments from identity for trajectory diagnostics. All pairs
retain the frozen source prior/world points; a reprojection-only result does
not alter later pairs' inputs. This isolates the per-pair objective effect
conditional on baseline-selected features/depths/priors. It is not a test of
reprojection-driven feature selection or a newly accumulated VO state.

## Conditioning and residuals

Median condition number of the inverse-variance weighted local projection
Jacobian at each fitted pose:

| Window | Modality | Disparity | Reprojection-only |
|---|---|---:|---:|
| early | rgb | 11.33 | 11.40 |
| early | thermal | 23.49 | 23.83 |
| original | rgb | 12.42 | 12.69 |
| original | thermal | 22.34 | 22.79 |
| late | rgb | 14.48 | 14.94 |
| late | thermal | 52.48 | 53.46 |

The late thermal Jacobian is less well conditioned in both arms than the
other thermal windows. The six local coordinates use translation in meters
and rotation in radians; condition numbers depend on this convention and the
weights, ranges, sample geometry and fitted pose. They omit robustification,
damping, source uncertainty and correlations, and are not the actual solver
Hessian or a calibrated pose uncertainty. This measurement supports examining
weights and feature geometry but cannot identify a cause by itself.

Per-pair output also includes image residual medians, disparity residual
medians for the disparity arm, solver-reported loss histories and singular
values. Native losses differ in residual dimension and are not compared
between objectives. Both arms still depend on source stereo depth; disabling
target disparity does not remove all stereo/depth dependence.

## Validation and artifacts

Two new tests pass: both backends recover consistent synthetic motion without
input mutation, and increment composition respects rotated coordinate frames.
All six source associations/timestamps and twelve objective arms complete;
poses and solver losses are finite, quaternion norms pass, arm fingerprints
agree, and all original replay controls pass.

Final output: `results/backend-ablation/run-002/analysis.json`,
`backend-comparison.png`, and `{early,original,late}/{rgb,thermal}/{disp,reproj}.npz`.
Each modality output preserves actual acquisition times and local reference
errors. Source/input hashes, Torch version, CPU thread count and UTC run time
are in the report. A full reproduction command is in the setup README.

One CPU thread was used in both final arms to avoid overhead on these small
graphs. The original-replay control verifies this setting's numerical effect.
Interrupted default-thread `run-001` partial outputs and logs are preserved;
they are not the final experiment. Original archived/source runs are unchanged.

These are three windows of one clip, dependent observations, provisional
calibration and recorded SLAM reference rather than independent ground truth.
The small differences do not establish a general backend ranking or an
accuracy gain, and absence of improvement does not exonerate disparity,
source depth or their uncertainty in every configuration.

## Next controlled experiment

Hold matches, depths and initialization fixed, and compare learned covariance
weights with fixed per-channel weights across the same windows. Define those
weights before evaluation and preserve the relative image/disparity units.
This tests whether confidence weighting contributes to the late thermal
failure before changing feature selection, calibration or fusion.
