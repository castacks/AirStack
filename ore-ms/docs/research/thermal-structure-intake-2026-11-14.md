# Thermal structure and incremental VO contribution: first intake

14 November2026 America/New_York; branch ore-ms-orb-development; baseline53afa19d1.
Exploratory descriptor intake, not a new VO method or accuracy result.

## Question

Can temporal/spatial thermal evidence predict an incremental pose benefit beyond
reliable RGB? Keep pose/trajectory accuracy as primary goal; matched-accuracy
compute savings is a separate alternative. Sensor appearance is an input, not the
success endpoint. Do not assume RGB always succeeds or thermal must win alone.

## Executed

Restored60raw thermal frames(first20 each day/night/rain) from hash-verified
u2flow-thermal-warp packet, plus13saved pose/fusion artifacts from the verified
translation-fusion packet. Every selected member hash checked; no source or old
result overwritten. The thermal archive includes one recovery JSON in addition
to60PNGs. Current raw images are integer sensor counts, not calibrated temperature.
Used isolated NumPy2.2.6/Pillow11.3.0 packages; host lacks NumPy and venv ensurepip.
No neural inference or new pose fit, learning, threshold sweep or RGB comparison.

All60images256x640,57adjacent pairs, unchanged file hashes. Normalize each image
using its own p5/p95. Interior central gradients define the2x2global spatial
structure tensor; minor fraction=lambda_min/trace. This describes directional
image gradients, not6DoF pose observability or landmark information.

| Median descriptor | Day | Night | Rain |
|---|---:|---:|---:|
| Raw p95-p5 counts |1337.5|388.0|570.0|
| Contrast-normalized gradient RMS |0.025717|0.032540|0.033477|
| Structure-tensor minor fraction |0.269152|0.375327|0.304076|
| Unregistered normalized adjacent-frame absolute change |0.020367|0.004301|0.023688|
| Unregistered adjacent-frame correlation |0.989389|0.999784|0.987768|

Narrow raw contrast does not imply weak normalized gradients in these samples.
Near-identical night images do not establish persistent geometric correspondences:
small motion, per-image normalization, smooth areas and sensor patterns can all
contribute. These3recording excerpts do not establish effects of night/rain as
causal conditions. No correlation with pose-error improvement has been tested.
2352scalar lattice gradient-component checks pass; all input hashes unchanged.
No new physical calibration or sensor-noise qualification.

## Why this differs from the old summary-cue experiment

Existing8feature fusion uses factor counts,residuals,information traces/conditioning
and cross-camera pose disagreement. It does not retain spatial image neighborhoods
or explicit cross-frame thermal feature persistence. Previous geometry audits,
natural-flow assays and depth interventions remain completed evidence; do not
repeat them as new. The old convex fusion oracle19.47%gain is reference-informed
and partly bias cancellation, not proof of a learnable image-based correction.

## Bounded next comparison

Use matched original RGB/thermal image windows and native accepted observations,
with explicit original timestamps/rig transport. First establish exact source-file
binding: matching condition names or frame ordinals is insufficient. Recovered
pose rows and thermal-frame filenames alone do not certify that alignment.

One question: does motion-conditioned thermal persistence/spatial structure
explain translation improvement beyond the old8summary cues? Predeclare a small
feature set: motion-compensated thermal track survival, spatial/directional
coverage, stereo/depth consistency; corresponding RGB features and disagreements.
Keep raw contrast and unregistered change as weak controls. Exclude reference
poses/errors from feature construction. Cross-camera pixel overlap requires
calibration/depth/visibility checks; do not equate thermal edges with RGB edges.

Controls: RGB-only, thermal-only, old8feature model, constant translation blend,
and the same model with new image/temporal inputs. Fix estimator means and RGB
rotation initially to isolate incremental translation benefit. Train/score by
recording with blocked temporal windows, no noise-draw random split. All current
records are exposed development; report that. Report mean/RMSE,p95,coverage and
rotation separately; favorable likelihood or image metrics cannot replace pose
improvement. If this finite comparison adds no pose benefit, report it without
a new head/threshold sweep. Accumulated trajectory and reference-independent
validation follow an actual interval improvement, not descriptor success.

This is an intake, not an execution freeze for the next comparison. A precise
source/time binding and feature-cost contract remain required. No new data upload
is requested; use existing Box archives first. Current60thermal images lack
restored corresponding RGB inputs and per-feature geometry for that comparison.

## Prior art

TP-TIO already learns thermal-specific keypoints and uses full-radiometric
tracking: https://arxiv.org/abs/2012.03455 . ETIO uses edge-based thermal
representation and adaptive tracking: https://arxiv.org/abs/2210.10033 . Fresh
reading in this intake is primary abstracts only; not full-method/code review.
These establish that thermal-specific structure/tracking is prior work. Our open
question is its incremental predictive value for an already reliable RGB estimate,
not novelty of thermal edges, descriptors, or generic condition-aware fusion.
