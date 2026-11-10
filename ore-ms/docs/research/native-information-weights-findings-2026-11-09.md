# Native poses with information-based RGB–thermal weights

9 November 2026, America/New_York. Branch `ore-ms-orb-development`. Readout freeze `16ebde6b5`; isolated driver `70aa4e1d0`; guard test `5a3a997cb`; independent audit `713e5b178`.

The primary calibrated full-matrix combination does not improve native RGB accuracy. Holding native means fixed removes the backend regression from the previous experiment, but the tested information readout and recording-excluded scale calibration still do not provide useful cross-camera weights. Retain the native backend and RGB baseline. This result applies to these weights on three exposed development prefixes.

## What changed

Reused the first 600 source groups of day, night and rain, with 599 expected motion intervals each. No new frontend/optimizer execution, training of network weights or runtime measurement. Native RGB/thermal means are exact copies of the previous locked estimates.

At each actual native first-write pose, reconstructed `H = Jᵀ C⁻¹ J` for the native disparity/reprojection residual using stored UV/disparity covariance and saved landmark associations. Its inverse conditions on fixed world landmarks. Native convergence is unknown; robust-loss corrections, damping, landmark uncertainty and marginalization are omitted. It is a **conditional local information approximation**, not an exported or calibrated native pose posterior. MACThermal already propagates uncertainty into pose estimation; doing so is not the contribution being tested.

Transported each readout into the same camera/body and interval convention as the fixed native means. Compared nine methods: native RGB/thermal, equal/angular combination, raw scalar/full information-based combination, calibrated scalar/full combination, and training-best-fixed camera. Scalar controls retain translation/rotation block traces; full controls retain direction and cross terms. Covariance intersection uses a common RGB-anchored tangent and a fixed 21-weight log-determinant search.

For each target recording, fitted translation and rotation marginal Mahalanobis scales per camera from the other two recordings, equally weighted by recording. Applied `D P D`, without clipping or hyperparameter search. Training-best-fixed chose RGB in all three folds. Target references and previous score files were blocked during fold fit/application; all predictions were locked before this scoring. These inputs had already been examined in previous work: recording exclusion does not make this a fresh validation set.

## Accuracy

Mean relative errors, on identical intervals for native RGB and primary calibrated full combination:

| Recording | Intervals | RGB rotation (deg) | Calibrated full (deg) | RGB translation (m) | Calibrated full (m) |
|---|---:|---:|---:|---:|---:|
| Day | 599 | 0.017523 | 0.022993 | 0.046604 | 0.048137 |
| Night | 598 | 0.030350 | 0.031404 | 0.077280 | 0.088229 |
| Rain | 599 | 0.041860 | 0.043152 | 0.231334 | 0.231129 |

All-method common support is 590/551/594 intervals (day/night/rain). Equal-record averages of mean errors on that support:

| Method | Rotation (deg) | Translation (m) |
|---|---:|---:|
| Native RGB / training-best-fixed | 0.029798 | 0.117703 |
| Native thermal | 0.130283 | 0.372243 |
| Native equal | 0.071524 | 0.221087 |
| Native angular | 0.029803 | 0.117590 |
| Raw scalar | 0.029964 | 0.117686 |
| Raw full | 0.029860 | 0.117679 |
| Calibrated scalar | 0.032811 | 0.118253 |
| **Calibrated full (primary)** | **0.032464** | **0.122114** |

The primary increases aggregate mean rotation error by 8.95% and translation error by 3.75% against RGB. Rain's small translation reduction accompanies worse rotation. Raw full remains near RGB; calibration worsens both aggregate metrics relative to raw full. Full-matrix calibration improves rotation over its scalar control but worsens translation, so that comparison supplies no overall accuracy claim.

All valid native estimates yielded a valid readout: RGB 599/598/599 and thermal 590/551/594. All combinations preserve union availability 599/598/599. Sole-camera fallback occurs 9/47/5 times; one night interval has neither estimate. No missing-information fallback was needed. This experiment therefore avoids the replay variant's missing-covariance equal fallback.

The fitted RGB translation scale varies from 531 to 14,901 across excluded-record folds; RGB rotation from 11.90 to 85.01. Thermal scales also vary. These are error-to-conditional-information scale factors under the supplied reference/model, not sensor noise measurements. Global scale correction does not establish transferable relative reliability; the observed error increases are the relevant outcome.

## Verification and scope

39 numerical, scoring, calibration and label-guard tests pass in the pinned Docker environment. An early fixture omitted required strategy metadata; corrected before fitting; production outputs were unaffected. Independent SciPy reconstruction verifies 3,594 native mean slots, 6,940 combination calculations, all three fold scales, and 32,206 error components. It does not independently rederive every native information matrix from raw tensors. Jacobian finite differences and inherited factor audits cover that numerical primitive separately.

The [frozen protocol](../../experiments/native-information-weights-freeze-2026-11-09.json) and [machine-readable outcome](../../experiments/native-information-weights-outcome-2026-11-09.json) retain source hashes, coverage, all controls and audit counts. Raw readouts, predictions, locks, scores and audit receipts live in `results/native-information-weights/run-001` and the companion Box backup packet. Inputs depend on the earlier accuracy/prototype backup packets.

Errors use provided RGB reference products, rigid-pose qualification and the existing nominal frame/time model. They measure per-interval relative accuracy; physical ground truth, trajectory accuracy and unseen-record transfer are not established.

## Next experiment

Measure cross-sensor opportunity before choosing another weight model: on the same exposed prefixes, characterize intervals where thermal beats RGB in rotation and translation separately, joint wins/tradeoffs, contiguous opportunities, and an oracle per-interval selection bound. Compare the bound with fixed RGB and existing methods on identical support, retaining missing estimates. Oracle results are a diagnostic upper bound, never an executable method result.

Then test whether existing reference-free geometry/support/residual cues predict those opportunities with recording-excluded fitting and a training-best-fixed control. Keep native means fixed initially. If those cues miss the useful cases, target the responsible thermal frontend/covariance or shared-representation mechanism rather than extending global scale tuning. Full trajectory evaluation and new recordings follow any credible interval gain. This sequence keeps the original accuracy question central while leaving frontend, representation and inertial extensions open.
