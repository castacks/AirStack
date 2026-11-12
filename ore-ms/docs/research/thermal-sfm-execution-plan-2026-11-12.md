# ThermalSfMLearner-MS: fixed outdoor reproduction attempt

12 November 2026. Reference key: [thermal-sfm](references/README.md).
Author code `d2ab0fc5e92972e248b641572f4b595dd67af83d`; paper
[2103.00760v3](https://arxiv.org/html/2103.00760v3). User supplied the author
Drive data/model links. This plan precedes real model inference or accuracy scores.

Use the two official outdoor test sequences, `outdoor_robust_night1` and
`outdoor_robust_night2`, and only the supplied outdoor depth/pose checkpoints.
Strict state-dict loading passes (depth epoch139, pose epoch88). Checkpoints
contain epoch/state_dict only; training-loss variant and split are not embedded.
Do not infer those from whichever published result is closest. The author raw
loader also lists night1 as validation; these are source-protocol reproduction
sequences, not new independent holdouts for the original model.

Source preparation uses the original `VIVIDRawLoader` and `dump_example` on the
two test scenes, retaining cumulative speed selection at0.15, original frame
IDs/poses/depth, calibration, OpenCV resize to256×320 and native uint16 samples.
The indoor-only well-lit extraction and training-list writing are not invoked.
Preserve full raw archive hashes and every member hash. Drive exposed sizes,
not authoritative checksums; intake SHA256 pins establish our received bytes.

Inference uses unchanged author depth/pose scripts under an isolated modern
runtime. Compatibility consists of explicit legacy NumPy scalar aliases and
legacy Torch checkpoint-load behavior. Core MAC runtime is not modified. No
training or hyperparameter search, checkpoint/frame selection or scientific
source patch is permitted. Label this as a modern-runtime source-protocol
attempt; old CUDA10.2 runtime equivalence has not been established.

## Targets and evaluation conventions fixed before outcomes

Depth: original inverse-depth resize, valid GT range(0.001,80)m, per-image
median scaling and original per-image metrics. Primary outdoor table comparison
pools the individual frame metrics across both sequences; also report the
unweighted two-sequence mean as a predeclared aggregation sensitivity. All seven
paper TableI Ours-MS targets must be within0.0005 for a strict rounded-table match:
AbsRel0.146, SqRel0.873, RMSE4.697, RMSlog0.184, a1/2/3=0.801/0.973/0.993.
No choice between aggregation conventions may rescue a failed primary match.

Pose: retain five-frame snippets, original matrix convention, snippet-fitted
translation scale and original ATE/RE formulas. ATE is a snippet score, not
unscaled metric VO error; RE is the source angle in radians. Source inspection
finds `__len__` counts images, but the generator emitsN−4 snippets. The author
script allocatesN rows and averages zero-padded errors. Reproduce that behavior
and separately report actual-snippet means; do not silently fix it to match a
paper number. Save both allocated and actual counts and audit the padding.

Paper TableIII Ours-MS targets (mean, std) for night1 are ATE(0.0562,0.0310),
RE(0.0287,0.0144); night2 ATE(0.0598,0.0316), RE(0.0274,0.0124). Strict rounded
match requires every value within0.00005 under the original script convention.

**A source discrepancy is known before execution:** the repository README's
night1 MS row instead lists ATE0.0572/RE0.0087. Report those separately with
0.00005 tolerance; they do not replace the paper-version targets. Distinguish
code/README/paper results and unknown exact checkpoint variant. No success
claim follows from retrospectively choosing the closest row or metric formula.

Write full input/code/checkpoint/runtime hashes and this plan into an execution
freeze before inference. Predictions from depth are sealed before separate
depth scoring. The unchanged author pose script computes its own reference
scores during inference, but no reference-dependent prediction/filter/training
or checkpoint selection is added. Independent auditing uses saved outputs.
Preserve every failed attempt and source/runtime deviation. No MS² transfer,
RGB-versus-MAC ranking or generalization claim is part of this reproduction.
