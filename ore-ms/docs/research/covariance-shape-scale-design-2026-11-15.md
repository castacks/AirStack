# Fixed-prediction oracle shape/scale diagnostic

15 November2026UTC. Baseline `a79658f63`, branch `ore-ms-orb-development`.
Design before new diagnostic outputs; the five-parameter recipe remains closed.

Question: do the existing covariance shapes help after separating aggregate scale
mismatch? Profile the Gaussian likelihood of every one of the30locked predictions
along a single positive scalar. This is an evaluation-error-informed diagnostic,
not a new predictive model, validation run or online calibration procedure.

For native-whitened covarianceS and unchanged zero-mean evaluation momentM,

    beta = trace(S^-1 M)/6,
    ell_original = .5[logdetS + trace(S^-1 M)],
    ell_shape = .5[logdetS + 6 log(beta) + 6],
    ell_original - ell_shape = 3[beta - 1 - log(beta)].

The last quantity is nonnegative. Native and any scalar-native prediction must
have the same oracle-profile score. For every common/control pair, the original
NLL gap equals the oracle-shape gap plus the difference in scale penalties.
These are Gaussian score identities, not new statistical theory or physical
causal attribution. One scalar is used for the complete6Dcovariance; no separate
translation/rotation/directional adjustment or evaluation-mean fitting.

All five old models(native, scalar, independent modes, common modes, pooled
shape), all six scenes and all three recording averages are retained. Previous
recording-excluded fitting remains unchanged. All scenes are exposed development
data and evaluation labels are explicitly used to compute beta. Do not replace
old predictions, plot corrected containment as a deployable result or select a
new calibration rule from this diagnostic. No model/pose/neural rerun.

42pre-analysis mechanics checks compare analytic profiling with1Dminimization,
scale/congruence invariance and exact excess-score identities. Independent audit
will reconstruct likelihoods in physical tangent coordinates using original pose
receipts and baseline information, then numerically minimize each scalar.
Stop after this one interpretation step; no mode/rank/scale-predictor rescue.

[DNLS5](https://arxiv.org/html/2305.09527v2) already separates improved VO from
calibrated absolute noise scale, which its pose objective does not identify.
Our explicit perturbation-likelihood target differs from that objective; do not
claim this diagnostic discovers a DNLS defect or solves physical scale calibration.
Existing Deeper-DICE GaussianNLL/source reading is reused; no author system runs.
