# EXP-009: Learned versus fixed covariance weights

Date: October 18, 2026 (America/New_York). Branch `ore_ms`; base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` plus uncommitted research changes.

## Result and decision

The late thermal translation failure is strongly sensitive to pixel covariance
weighting in this fixed-input experiment. Fixing target pixel variance reduces
translation disagreement with recorded SLAM from 2.897 m to 0.790 m RMS, while
fixing only target disparity variance leaves it near 2.896 m. This isolates a
weight-setting effect conditional on the saved matches, depths, initialization
and native optimizer; it does not identify the physical error source.

The gain has a rotation tradeoff: late thermal rotation disagreement increases
from 1.713° to 4.375° with fixed pixel weights. Early thermal accumulated
translation improves too, but consecutive translation disagreement worsens.
Do not adopt a fixed-weight replacement or rank uncertainty methods from
accumulated translation alone. No backend default or original trajectory,
model, calibration, preprocessing or selected features changed.

## Frozen protocol

Before optimizing any new arm, `plan.json` freezes per-channel medians of
active saved target covariance on original-window frames 0–9 (nine edges 0→1
through 8→9), separately per modality. No reference poses or residuals enter
this rule. Median channel variances `(u,v,disparity)`, in px²:

| Modality | u | v | disparity | Retained training observations |
|---|---:|---:|---:|---:|
| rgb | 0.06561724 | 0.06250000 | 0.03216033 | 1758 |
| thermal | 0.08894412 | 0.06250000 | 0.28550828 | 1592 |

These are fixed experimental covariance settings, not fitted pose/depth scales
or calibrated noise estimates. Pixel values are taken after existing floors;
source uv cross-covariance is zero. Units and channel anisotropy are retained,
but equalizing variance across observations changes aggregate precision and
relative influence. It is not an exact control of total precision mass.

Four arms use the current disparity objective:

- `learned`: original active covariance (including pixel floors).
- `fixed_uv`: fix u/v variance, retain learned disparity variance.
- `fixed_disp`: retain learned pixel variance, fix disparity variance.
- `fixed_both`: fix all three channels.

All 114 saved pairs (three windows × two modalities × 19 edges) are solved
independently in each arm: 456 solves. World points, depths, matches, K,
baseline and static prior pose remain fixed. The solver is reused from
EXP-008: analytic double-precision CPU disparity graph, original LM/PINV,
trust region, Huber/FastTriggs and stopping configuration, one CPU thread.
All retained matches are replayed, including extreme-depth tails; the 50 m
diagnostic cap from EXP-007 is not applied. Only covariance entries differ.
Nonweight input fingerprints must match and
inputs must remain unchanged after optimization. The learned arm replays the
original within 1 mm / 0.01° local-motion tolerances before interpretation.
Reference poses are used only for evaluation.

As in EXP-008, independent increments are composed from identity, while each
pair keeps its baseline prior pose/world points. These are fixed-input replay
trajectories, not a fresh end-to-end fixed-weight VO run. Constants come from
the first half of the original window; its reported result overlaps those
source frames and is descriptive. Early/late are separate windows of the same
clip, with shared calibration and conditions, not independent datasets.

## Translation results

First-pose anchored translation RMS against recorded SLAM, without any fitted
scale or best-fit trajectory alignment:

| Window | Modality | Learned (m) | Fixed pixel (m) | Fixed disparity (m) | Fixed both (m) |
|---|---|---:|---:|---:|---:|
| early | rgb | 0.0904 | 0.0891 | 0.0901 | 0.0888 |
| early | thermal | 0.7027 | 0.4515 | 0.5314 | 0.5521 |
| original | rgb | 0.1277 | 0.1265 | 0.1246 | 0.1239 |
| original | thermal | 1.5171 | 1.2878 | 1.4752 | 1.2363 |
| late | rgb | 0.0995 | 0.1000 | 0.0988 | 0.0996 |
| late | thermal | 2.8966 | 0.7898 | 2.8955 | 0.7897 |

Late thermal net travel changes from 3.540 m learned to 9.588 m fixed pixel,
versus 8.747 m recorded reference. Fixed disparity alone estimates 3.578 m;
fixed both 9.611 m. Pixel weighting accounts for most of this arm difference.
Translation gains on these selected segments are not evidence of a generally
better uncertainty model, physical calibration or cross-modal comparability.

## Rotation and local-motion tradeoffs

Thermal rotation RMS and consecutive translation RMS (local increments):

| Window | Arm | Rotation RMS (deg) | Consecutive translation RMS (m) |
|---|---|---:|---:|
| early | learned | 4.176 | 0.1169 |
| early | fixed_uv | 4.382 | 0.1816 |
| early | fixed_disp | 3.501 | 0.1305 |
| early | fixed_both | 4.123 | 0.1958 |
| original | learned | 5.688 | 0.1415 |
| original | fixed_uv | 5.461 | 0.1250 |
| original | fixed_disp | 5.589 | 0.1376 |
| original | fixed_both | 5.303 | 0.1209 |
| late | learned | 1.713 | 0.3447 |
| late | fixed_uv | 4.374 | 0.1485 |
| late | fixed_disp | 1.791 | 0.3420 |
| late | fixed_both | 4.405 | 0.1474 |

Early thermal fixed-pixel consecutive translation error increases 0.117→0.182 m
while accumulated error falls 0.703→0.452 m. This difference warns against
interpreting a shorter-segment endpoint or anchored improvement as a uniform
per-step gain. Late thermal improves both translation summaries but worsens
rotation substantially. All RGB changes are smaller in absolute translation
units on these segments; they do not establish a general modality ranking.

The late thermal median weighted local Jacobian condition number changes
52.48→28.16 with fixed pixel weights, versus 51.67 with fixed disparity.
This is evaluated at fitted poses using meters/radians and inverse-variance
weights before robustification. It depends on both weights and the fitted
pose; it is not the solver Hessian, a causal diagnosis of scene geometry,
or calibrated pose uncertainty.

## Validation, artifacts and limits

Seven new test cases pass: intended covariance entries change while nonweight
inputs/source controls remain unchanged; nonpositive, nonfinite and malformed
constants are rejected; fixed weights recover consistent synthetic motion.
All 24 arm/window/modality combinations complete, covering 456 pair solves.
Poses and loss histories are finite, quaternion norms pass, all geometry
fingerprints agree, exact source pose/time associations pass, and all learned
controls reproduce original motions within the declared tolerances.

Artifacts: `results/weight-ablation/run-001/{plan.json,analysis.json,weight-comparison.png}`
and `{early,original,late}/{rgb,thermal}/{learned,fixed_uv,fixed_disp,fixed_both}.npz`.
Actual acquisition timestamps, local reference errors, iteration/loss histories,
source/input/reference hashes and Torch/thread settings are preserved. The
plot shows translation only; assess it with the rotation/local-motion table.
Reproduce with `Scripts/Setup/td3_weight_ablation.py prepare`, then `evaluate`,
using the shared arguments in the setup README. Existing result directories
are refused. No new model inference or scale fitting occurs.

Native Huber/LM behavior is retained; changing weights also interacts with
native robust correction and step acceptance/stopping. This experiment does
not isolate those solver interactions, calibrate covariance, remove selection
bias from learned-covariance feature selection, or account for source-depth,
reference, timing and provisional geometry uncertainty. Recorded SLAM is a
proxy rather than independent ground truth. No adaptive fusion is justified
by these results alone.

## Next measurement

Audit where pixel precision is concentrated and how it changes pose influence:
compare image-region/range precision shares and local motion sensitivity for
learned versus fixed pixel weights on the same saved matches. Inspect the
late window alongside both controls, including the translation/rotation
tradeoff. Use this to design a bounded weighting or selection hypothesis;
require a separate clip before adopting a default or cross-modal confidence
claim.
