# Known-warp thermal assay: released baseline handles the fixed perturbation

Run/prediction lock at 2026-11-13T21:48:29.618978-05:00. 13November2026 America/New_York.
Branch ore-ms-orb-development, baseline a79658f63 plus hash-pinned working source.
Source/checkpoint/input/metric protocol frozen before first thermal inference.

**DESCRIPTIVE_TARGET_ASSAY_COMPLETE_NO_METHOD_PROMOTION.** The released U²Flow
baseline performs well on the declared small-displacement/appearance stressor.
This assay does not expose a high-error consistency failure or justify a new
appearance-invariance training recipe. Do not increase nuisance severity, change
frames or search transformations after seeing this outcome to manufacture one.

## Fixed data and execution

First20raw thermal-left frames of each existing exposed day/night/rain prefix,
60total. Their old source-manifest SHA hashes match. Recovery packet archive
SHA `cb716171eed25be7879bbd8de7bd47b447a23689acc320364c4d0bdf90df6b0b`
and all1070regular-member hashes verify. Only selected raw PNGs are restored;
no new natural-flow labels, poses or reference depths are used. The archive
verification reads other members as opaque bytes for integrity, not as labels.

Apply the exact existing percentile/CLAHE/bilateral producer AST, then one fixed
INTER_AREA resize from640x256 to320x128. Model receives[0,1] replicated gray.
Four cases per image:identity, exact integer(4,-3)displacement, and each with
fixed target-only gamma1.6. The warp is applied AFTER resizing/preprocessing,
so target displacements are exact at evaluation resolution. No claim that the
perturbation simulates physical NUC, sensor drift or natural thermal dynamics.

Qualified author Sintel checkpoint/source unchanged. CPUFP32,12iterations,
no training,240inferences, no retries or outcome-selected exclusions. All2880
upstream uncertainty-head outputs are finite; model state hashes are unchanged.
All predictions globally hash-locked before the scoring command. Identity
eligibility is40960pixels/frame; shifted eligibility39500pixels/frame. Clean
and appearance-changed versions share their geometric masks.

## Actual correspondence error and learned uncertainty ranking

All60frames and all9655200eligible case-pixel predictions are retained. Every
eligible prediction is finite and below1px error: maximum0.734148px. Both1px
and3px success fractions are1.0 in every case/frame. These are repeated pixel
predictions, not independent experimental samples or a natural-flow benchmark.
The table uses equal-frame means of per-image metrics; RMSE is a mean of
per-image RMSEs, not pooled-pixel RMSE.

| Recording | Case | Mean EPE,px | Mean per-frame RMSE,px | Alpha/EPE Spearman | Normalized AUSE |
|---|---|---:|---:|---:|---:|
| day | identity | 0.006449 | 0.007662 | +0.139 | 0.361 |
| day | shift4minus3 | 0.075294 | 0.088831 | +0.399 | 0.228 |
| day | identity_gamma1.6 | 0.009808 | 0.011985 | +0.270 | 0.312 |
| day | shift4minus3_gamma1.6 | 0.077169 | 0.090455 | +0.412 | 0.221 |
| night | identity | 0.006151 | 0.007259 | +0.250 | 0.278 |
| night | shift4minus3 | 0.068304 | 0.084748 | +0.529 | 0.183 |
| night | identity_gamma1.6 | 0.008944 | 0.010828 | +0.180 | 0.300 |
| night | shift4minus3_gamma1.6 | 0.070918 | 0.087339 | +0.557 | 0.155 |
| rain | identity | 0.005954 | 0.007336 | +0.130 | 0.408 |
| rain | shift4minus3 | 0.082365 | 0.100956 | +0.365 | 0.233 |
| rain | identity_gamma1.6 | 0.011749 | 0.014395 | -0.041 | 0.506 |
| rain | shift4minus3_gamma1.6 | 0.079917 | 0.096187 | +0.400 | 0.210 |

Alpha is the original scalar log uncertainty, not a calibrated2D covariance.
AUSE uses20fixed retained fractions1.0to0.05, trapezoid integration over removed
fraction0to0.95 and normalization by each frame's full EPE. Low absolute errors
make normalized ranking diagnostics a different endpoint from physical accuracy.
Reported Spearman values are descriptive; no IID significance claim.

## Predictive inconsistency versus true error

For each geometry, disagreement is the L2 difference between clean-target and
appearance-altered predictions. True EPE always uses the imposed mapping,
not either prediction as a pseudo-label.

| Recording | Clean/altered pair | Mean disagreement/EPE Spearman | Pixels EPE>1px and disagreement<0.1px per frame |
|---|---|---:|---:|
| day | identity | +0.597 | 0 |
| day | shift | +0.134 | 0 |
| night | identity | +0.607 | 0 |
| night | shift | -0.235 | 0 |
| rain | identity | +0.792 | 0 |
| rain | shift | +0.135 | 0 |

On shifted pairs, disagreement/error ranking is weak on day/rain and negative
on night, while the model's scalar uncertainty has positive error-ranking
correlation. This does not establish general ranking superiority: it is one
checkpoint, a narrow nuisance and an already low-error population. Identity
pair correlations are stronger and remain separately reported.

The predeclared high-error diagnostic EPE>1px has ZERO qualifying pixels in
all cases. Therefore “fraction consistent among bad pixels” is undefined,
reported null with0valid frames, rather than0% or100%. Zero consistent-but-wrong
counts here do not prove consistency sufficient: the assay has no above-threshold
errors with which to test that failure mode. Conversely, low disagreement/EPE
correlation does not show the baseline fails at matching these inputs.

## Independent verification

NumPy/OpenCV/SciPy-only auditor imports neither Torch nor producer. It uses
partition-based percentile reconstruction rather than producer sorting, verifies
all prepared images, independent coordinate masks, EPE/success, rank correlation,
discrete sparsification curves, disagreement and equal-frame aggregates.
3199 checks pass, max metric difference7.44e-15.
It reruns no model and does not certify natural-flow accuracy.44source pins,
60raw hashes, checkpoint and all locked prediction hashes remain unchanged.
No score/tolerance amendments or scientific reruns occurred.

## Research consequence

A published self-supervised flow/uncertainty model is now executable on these
thermal inputs and robust to this ordinary controlled perturbation. The simple
claim “thermal appearance changes require a new representation” is not supported
by this test. Keep the positive baseline evidence instead of fitting another
head to solve a failure not observed here.

This is an assessment of a training signal under exact imposed correspondence,
not a trained candidate, author benchmark reproduction, natural temporal-flow
validation or VO improvement. No original MACThermal model was run, so no relative
model-superiority result follows. No fresh-record or physical-calibration claim.
A future method should address a distinct evidenced limitation or use an
independently justified natural-motion endpoint; this outcome selects neither
an augmentation sweep nor a new acquisition campaign.

Protocol: `experiments/u2flow-thermal-warp-freeze-2026-11-13.json`.
Source/auditor: `tools/u2flow_thermal_warp.py`, `audit_u2flow_thermal_warp.py`.
Artifacts: `results/u2flow-thermal-warp` includes60raw inputs, recovery receipt,
all predictions, lock, scores, audit, source freeze and attempt logs.
Checkpoint/runtime recovery: prior U²Flow inference-qualification packet.
