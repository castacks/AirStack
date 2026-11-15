# Next question: distinguish selection robustness from pose accuracy

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.

## Decision

Pursue one precise evaluation question before designing a correction:

**Under an exact known camera-motion target, does query regeneration change
systematic pose error, centered dispersion, or both?**

The paired natural-input study and crossed replay already establish a selection
mechanism. They measure baseline-relative sensitivity, so they cannot tell whether
a changed pose moves toward or away from the true pose. A stable biased estimate
and an accurate estimate are different outcomes. This is a useful original-paper
V3/I2/I4 assessment, without alleging that the papers ignore accuracy or that
selection effects are new. Other original-paper openings remain available.

## Prior-art check and contribution boundary

- [R2D2, published NeurIPS2019 paper](https://proceedings.neurips.cc/paper_files/paper/2019/file/3198dfd0aef271d22f7bcddd6f12f5cb-Paper.pdf): introduction and3.1–3.2 inspected. The method separates repeatability from descriptor reliability and trains repeatability with warped local maps and a peakiness term. Known synthetic transforms can provide correspondence targets. It is not our dense thermal query selector or a pose-covariance validation. No model/code execution.
- [Good Feature Selection](https://arxiv.org/pdf/1905.07807): introduction and selected sectionII bias/variance discussion rechecked. Structural information, appearance and accuracy/uncertainty distinctions are already part of its argument. Generic geometry-aware or bias-aware selection is not a new proposal. Prior released-code inspection remains unchanged.
- [SIVO v2](https://arxiv.org/abs/1811.11946v2): abstract/metadata only. It combines network semantic uncertainty with information-based feature selection. This is an overlap lead, not a full method/code review or demonstrated equivalence to a specific future candidate.

This bounded check supports neither “train a repeatability head” nor “combine
stability and information” as an established novel contribution. It also does
not exhaust the selection literature or close the research direction. The
current contribution is a source-pinned characterization of learned-query
selection sensitivity, with a new exact-target evaluation capability.

## Completed reference qualification

Six artificial scenes use the same three already exposed thermal textures, each
with two geometries: a single fronto-parallel plane at4m and a two-depth scene
with a foreground plane at4m restricted to world X≤0 and background at8m.
The textures are image atlases, not reconstructed natural scene radiance.

Each scene has source/target left/right views at256×640 with the existing camera
K and0.3038995142414822m stereo baseline. The exact target-left-to-source-left pose
is optical-Y rotation0.5° and translation[0.03,0,0.10]m. The right target center is
t+R[baseline,0,0], preserving the rotated stereo rig. Ray-plane intersections
choose the nearest permitted surface. Canonical-left texture coordinates use
bilinear sampling and8bit rounding, with explicit zero texture outside the atlas.
Boundary coordinates within1e−9pixel are snapped to their exact atlas boundary.

The renderer emits exact world points, visible surface IDs, texture coordinates,
geometry/texture-support masks, camera extrinsics and true relative pose alongside
the grayscale inputs and continuous samples. The texture support is95.16–100%
across all views. Both surfaces appear in every depth-discontinuity view. These
masks describe rendered support; they do not certify natural visibility or timing.

**473 independent checks pass** across six scenes/24views and3,932,160pixel rays.
They independently reconstruct inverse-K rays, Rodrigues rotations, nearest
plane visibility, projections, pose transforms and stereo disparity identities.
Every emitted file hash and source texture is checked. Continuous interpolation
is independently checked within2e−6; quantization matches the producer exactly,
and any one-code difference in independent interpolation is restricted to
half-code ties. A repeated target-view render is bitwise identical.

Stereo identities reproject the same world points into both cameras; they do not
assert visibility in both views across an occlusion boundary. No dense optical-flow
truth is claimed solely from those identities. The exact pose target itself is
known by construction and does not require natural motion/depth labels.

Protocol: `experiments/known-pose-thermal-renderer-freeze-2026-11-14.json`.
Artifacts: `results/known-pose-thermal-renderer/run-001/`.

![Source stereo / target stereo, two-depth day texture](../../results/known-pose-thermal-renderer/run-001/day-two-depths.png)

## Next finite numeric design, not yet executed

Keep the existing native MACT checkpoint, dense prediction, NoDepth query policy,
packet construction and explicit IRLS harness. Use all six fixed scenes; no
geometry, motion, texture or jitter sweep. Original query IDs versus regenerated
queries remain the two paired arms. Reuse the existing half-LSB perturbation
magnitude with128draws/scene plus unperturbed baseline and identical-input null.
Bind exact trial seeds, runtime/source/checkpoint hashes and every rendered input
in a separate execution freeze before learned predictions. No renderer change
is permitted after inspecting those predictions.

Evaluate both arms in the common true-pose-relative left SE(3) Log tangent:

- Retain the unperturbed estimation error, mean error vector, centered sample
  covariance, error second moment, translation/rotation error norms and every
  attempt/status. Report availability without treating failed fits as zero error.
- Verify the finite-sample identity: mean(xxᵀ) = ((n−1)/n)Cov_ddof1(x) + mean(x)mean(x)ᵀ.
  Compare error second moment separately from centered variability; covariance
  alone does not identify systematic error.
- Report all six scenes independently. The three texture labels do not turn
  artificial geometry into physical day/night/rain trials. Paired noise draws are
  conditional samples, not independent recordings or a generalization benchmark.
- Retain pose-covariance containment only as a secondary statistic under this
  declared synthetic generator. It cannot establish physical thermal calibration.

A later qualification must confirm native input preparation and packet/solver
receipt compatibility with these rendered arrays, then bind the execution
freeze. If an arm fails or error does not track spread, report that outcome;
do not alter geometry, jitter, point budgets, initialization or solver thresholds.
No new selector, correction, head, training recipe or method claim is selected.

## Status and stopping rule

Status: **EXACT_REFERENCE_QUALIFIED; NO_MODEL_ACCURACY_RUN**. This turn completes
the target qualification and prior-art/contribution decision. It does not claim
that stability improved accuracy, that the native estimator fails, or that an
algorithm has been corrected. The next numeric comparison can now distinguish
systematic synthetic pose error from the previously measured robustness effect.
The wider thermal-supervision, transfer, continuity and persistent-estimation
gaps are preserved rather than filtered through this one assessment.
