# Covariance transfer: separate directional shape from variance scale

15 November2026UTC, branch `ore-ms-orb-development`, baseline `a79658f63` plus
source-pinned working changes. All30locked predictions diagnosed; no retraining,
new predictor, neural inference, pose fit or correction of the original scores.

## Finding and next-direction decision

The shared-mode model's partial transfer gains are not purely scalar effects,
but its directional advantage is small and mixed. After each fixed covariance
receives its best single scale using evaluation errors, common-mode shape beats
native shape on all four day/rain scenes and loses on both night scenes. The
recording-average shape gaps are−0.029853/+0.066061/−0.037088nats for day/night/rain.
One rain-scene benefit is only0.000049nats; do not label every sign as practically
meaningful. No significance or advancement threshold was specified.

Scale also matters substantially. The common model's remaining scale penalties
are1.159674/0.173329/0.368752nats on the same recording averages. Against the
stronger pooled-shape control, its original day win is a scale advantage despite
a worse directional shape. Rain retains a small directional advantage; night's
original loss to pooled shape includes worse scale despite a slightly better
shape. Thus “only scale is left” and “the predictor learned no shape” would both
misstate the evidence.

The next research decision should require separate evidence for scale and shape,
not append a scalar calibration layer to the closed recipe and call it a solution.
No new method is selected by this diagnostic. A future scale signal must be
available without evaluation-trial errors; a future shape model must distinguish
itself from the existing native/pooled-shape controls. These requirements do not
close the original papers' broader uncertainty or thermal-learning openings.

## What was computed

Use all six exposed development scenes,128saved draws per scene and all five
existing predictive methods from the
[completed transfer comparison](covariance-transfer-findings-2026-11-15.md).
Their models, fitted coefficients, feature definitions and30predictive covariance
matrices remain byte-identical. Previous recording-excluded fitting remains
unchanged; this is not new blind validation.

For fixed native-whitened covarianceS and unchanged evaluation second momentM
around the unperturbed estimate, profile one positive scalar beta:

    beta = trace(S^-1 M)/6,
    ell_original = .5[logdetS + trace(S^-1 M)],
    ell_profile = .5[logdetS + 6 log(beta) + 6],
    scale_penalty = ell_original - ell_profile
                  = 3[beta - 1 - log(beta)] >= 0.

Only scale changes; all directional eigenvalue ratios and eigenvectors of each
covariance remain fixed. Native and scalar-native models must give identical
profile scores. For every common/control pair,

    original NLL gap = profile-shape gap + scale-penalty gap.

These are Gaussian likelihood identities, not new mathematics or a physical
causal decomposition. The scale beta uses the **evaluation errors**, so every
profile is an **oracle-only diagnostic**. It is not an available uncertainty
prediction or an online correction. We do not report corrected containment,
replace the old predictions or select a new calibration rule. The fixed zero
mean and baseline-relative perturbation target remain unchanged; there is no
separate translation/rotation scaling or evaluation-mean fitting.

## Recording-average decomposition

Negative pairwise gaps favor common modes. All entries are nats, averaged equally
over the two geometries in each excluded recording.

Common versus the train-fitted scalar:

| Recording | Original NLL gap | Oracle shape gap | Scale-penalty gap |
|---|---:|---:|---:|
| day |−0.110945|−0.029853|−0.081092|
| night |+0.043947|+0.066061|−0.022114|
| rain |−0.098903|−0.037088|−0.061816|

Common versus the train-pooled full shape:

| Recording | Original NLL gap | Oracle shape gap | Scale-penalty gap |
|---|---:|---:|---:|
| day |−0.118450|+0.073191|−0.191642|
| night |+0.055958|−0.009324|+0.065281|
| rain |−0.015284|−0.014217|−0.001067|

![Exact score decomposition with unchanged predictors](../../results/covariance-shape-scale/shape-scale-decomposition.png)

The same identity is checked against the independent-mode control and native
output for all24scene/control pairs. All30profiled method/scene slots remain
available in the CSV/JSON; no scene or model is chosen after observing the signs.

## Per-scene common-mode shape and scale

| Scene | Common shape minus native shape | Common original scale penalty | Oracle multiplier beta |
|---|---:|---:|---:|
| day-plane4m |−0.015007|1.945221|2.606363|
| day-two-depths |−0.044698|0.374127|1.585795|
| night-plane4m |+0.021974|0.096591|0.767235|
| night-two-depths |+0.110149|0.250068|0.645271|
| rain-plane4m |−0.074126|0.133212|0.730831|
| rain-two-depths |−0.000049|0.604292|0.491851|

The opposite multiplier directions reinforce the limits of one transferred scale:
day's oracle would enlarge these predictions, while night/rain's would shrink
them. This is an evaluation-error-informed description, not a procedure available
at prediction time. Multiplying these matrices by beta does not change pose means,
remove original pose bias or establish physically correct uncertainty.

A positive shape case remains: common-mode anisotropy has a rain recording-average
advantage over both native and pooled shapes after optimal scaling. The day
advantage over native shape is also retained, while the pooled shape remains
better there. The negative night-versus-native shape result and negligible
rain-two-depths difference remain equally important limits.

## Paper connection and target limits

[DNLS section5](https://arxiv.org/html/2305.09527v2) explicitly reports pose benefits
while noting that calibration is not guaranteed and absolute noise scale is not
identified by its pose-estimation objective. This targeted passage was freshly
rechecked; no new author code was run. The result is not a newly discovered DNLS
failure. Nor do we infer that all likelihood-based or repeated-measurement scale
estimation is impossible. Our explicit perturbation-likelihood target exposes
scale through a declared ensemble, which is a different source of information.

Earlier Deeper-DICE Gaussian-NLL and Censi/Bonnabel covariance-sensitivity reviews
remain the source background. This diagnostic is not another learned covariance
algorithm or author reproduction. Native uncertainty and our half-LSB sampling
ensemble refer to different error distributions, so their score-scale difference
is not a physical calibration verdict. Baseline-relative perturbation spread,
matching uncertainty and true-pose error remain separate targets.

All scene/noise/input/exposure limits from the prior findings apply. Profiling
on the same128evaluation draws does not supply an out-of-sample scale predictor.
Gaussian likelihood algebra does not establish Gaussian tails or95%physical
coverage. We leave the previous prediction and containment tables intact.

## Verification and stopping

42mechanics checks pass before the new diagnostic: analytic scale profiling
against numerical1Dminimization, scale invariance, fixed-linear-coordinate
invariance of pairwise shape gaps and exact excess-score identities. A25pin
freeze binds sources, locked old predictions, labels, qualifications and design.

The independent auditor reconstructs the768pose changes from original receipts
using matrix logarithms, works with physical tangent covariance matrices rather
than the producer's whitened representation, and numerically minimizes each of
30scalar likelihoods. It passes1,192checks, including all scalar/native ties,
nonnegative scale penalties, old-score identities,24pair decompositions and
recording means. No source/input/tolerance amendment is made.

Close this finite interpretation step. No rank, mode, scale predictor, mean head,
noise ensemble or optimization adjustment follows automatically. The five-
parameter predictor stays closed with its positive and mixed results. Broader
original-paper gaps and a distinct future mechanism remain available.

Artifacts: [freeze](../../experiments/covariance-shape-scale-freeze-2026-11-15.json),
[producer](../../tools/covariance_shape_scale.py),
[independent auditor](../../tools/audit_covariance_shape_scale.py),
[all30profiles and pair decompositions](../../results/covariance-shape-scale/run-001/results.json),
[audit](../../results/covariance-shape-scale/audit-001.json).
