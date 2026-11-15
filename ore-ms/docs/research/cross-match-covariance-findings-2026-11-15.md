# Cross-match dependence materially changes centered pose spread

15 November2026UTC, branch `ore-ms-orb-development`, baseline `a79658f63` plus
frozen working changes. Reanalysis of768saved fixed-query trials in six artificial
scenes. Zero new image renders, neural predictions, pose fits or trained parameters.

## Finding and contribution

Cross-match covariance terms matter under the existing input-perturbation model.
Keeping the measured cross terms produces6.28–12.05times the normalized variance
obtained when only within-match covariance blocks are retained. This result
holds with exactly the same accepted query IDs in every draw.

The frozen first-order response tracks the actual centered pose variation well:
its normalized covariance trace is within0.44% of the actual trace in all six
scenes. The paired centered linear-remainder RMS is0.54–2.53% of actual RMS.
Those are separate checks; neither is an “explained variance” percentage.

This adds evidence for MAC-VO's covariance-structure and MAC-I²'s posterior-
reliability questions beyond the completed query-reselection comparisons. It
shows that cross-match dependence is observable and consequential in this
conditional ensemble. It does not identify a new covariance formula, a physical
noise law, a generally miscalibrated published system or a validated correction.

## What is compared, and what is not

The denominator is an empirical **within-match-block** counterfactual, not the
network's reported match variances or the estimator's inverse-information output.
Both compared calculations use the same empirical marginal residual covariances,
the same baseline pose, Jacobians and weights. Only cross-match terms differ.
Thus the factor6–12 must not be read as a native covariance underestimation factor.

All studies reuse the prior independently audited
[known-pose assessment](known-pose-estimator-findings-2026-11-14.md). Its128draws
per scene add independent uniform half-LSB noise to the prepared pixels, clip
them, replicate channels and reuse the same left view in stereo and temporal
calls. This is a controlled input-sensitivity model, not measured sensor noise
or the checkpoint's training-error distribution. Three textures and two planar
geometries are exposed development cases, not768independent recordings.

Use only the previously saved `fixed_ids` arm. The unperturbed baseline and exact
repeat plus all128draws retain identical IDs:111/69/87/94/82/54points in the table
order below. No feature intersection, subset selection, scale, noise magnitude,
solver change or new failure-conditioned population is introduced. Original
pose fits all converged; their existing source/input/audit records are reused.

## Fixed linear-response definition

Let the unperturbed solution be T0=(R0,t0). At that fixed solution, define

    r_i = R0 X1_i + t0 - X0_i,
    C_i = C0_i + R0 C1_i R0^T,
    J_i = [I, -hat(R0 X1_i+t0)],
    H = sum_i J_i^T C_i^-1 J_i,
    G_i = -H^-1 J_i^T C_i^-1.

H is checked against the saved baseline raw inverse-information covariance.
For each draw, compute the residual change delta_r_i from its saved3D means,
then the individual pose influence v_i=G_i delta_r_i. Center every influence
across the128draws. With ddof1,

    V_full  = Cov(sum_i v_i),
    V_block = sum_i Cov(v_i),
    V_cross = V_full - V_block.

The signed cross term need not be positive semidefinite. It can cancel variance;
common, anticorrelated and orthogonal mechanics fixtures explicitly cover these
cases. We do not reinterpret a signed trace as an explained-error fraction.

Actual pose changes use the translation-first **left** tangent
`Log(T_draw inverse(T0))`, centered across draws. Both the linear response and
actual poses therefore use one fixed tangent. Six-dimensional trace means
`trace(H V)`, an explicitly shared normalization, not a new calibrated probability.
Translation and rotation covariance-block traces are also saved separately.

The linear response freezes baseline geometry and weights. The original solver
changes Jacobians/weights with the observations and state; those changes and
nonlinear effects remain in the measured linear remainder. This is not an exact
implicit derivative of the entire learned pipeline or a replacement posterior.
Original bias and true-target error are separate from centered dispersion and
remain in the prior findings; this analysis does not remove or calibrate them.

## Complete six-scene results

| Scene | Fixed points | Full / block normalized trace | Linear / actual normalized trace | Centered remainder / actual RMS |
|---|---:|---:|---:|---:|
| day-plane4m |111|12.0467|0.999852|2.5334%|
| day-two-depths |69|9.7981|1.004366|1.7006%|
| night-plane4m |87|9.2600|0.996355|1.2989%|
| night-two-depths |94|8.9416|0.999145|0.5380%|
| rain-plane4m |82|11.2838|0.998185|1.9929%|
| rain-two-depths |54|6.2816|1.003310|1.1987%|

![Cross-match covariance and paired linear-response fidelity](../../results/cross-match-covariance/cross-match-covariance.png)

For day-plane4m, the normalized full/block traces are0.00423034/0.000351161;
actual is0.00423096. The translation variance traces are1.44613e−7/1.98417e−8m²,
with actual1.45698e−7m². These are absolute spread values under the specified
perturbations. The baseline inverse-information covariance has normalized trace6
by construction, substantially larger than this ensemble's centered trace.
A large cross/block ratio therefore does not imply that the native reported
uncertainty is too small for these perturbations. Nor does a large native
readout here certify physical calibration or accurate true-pose coverage.

The cross-match terms are not simply another name for pose/depth state coupling.
All G_i already share the same normal-equation inverse H. The counterfactual
changes residual-noise covariance, not that geometry coupling. Dependence may
reflect shared image/network computations, stereo/temporal view reuse and
inferred depth/flow interactions. This analysis does not isolate those causes,
measure a spatial correlation length or prove that adjacent pixels are the sole
source. It also does not show that existing learned weights cannot empirically
absorb some dependence on their evaluated distributions.

## Prior work and source boundaries

[Censi2007, Proposition1 and surrounding discussion](https://censi.science/pub/research/2007-icra-icpcov.pdf)
already allows a full measurement covariance and shared measurements across
correspondences in registration sensitivity analysis. The covariance sensitivity
formula is explicitly established work. Targeted primary PDF web extraction read;
no author implementation or full-paper reproduction. A direct local download
failed certificate validation; the primary PDF's indexed web extraction supplied
the inspected sections. No TLS bypass or external PDF copy is included here.

[Bonnabel et al., III-A–C](https://arxiv.org/html/1410.7632v1) already retains
cross-residual terms and analyzes correlated resolution errors in point-to-plane
ICP. That sensor/error model is not asserted for learned thermal flow. This is
a targeted reread beyond the earlier rematching review, not an author benchmark
reproduction. The [earlier source-overlap review](residual-conditioning-prior-art-decision-2026-11-14.md)
already distinguishes shared latent geometry, endpoint covariance and cross-
observation measurement dependence. No novelty for generic correlated GLS,
sandwich propagation or a low-rank covariance representation is claimed.

## Verification and stopping decision

Eight mechanics checks pass before the scientific analysis. The complete
789-source/input pin freeze precedes all new covariance outputs. The independent
auditor uses a full3N×3N empirical residual covariance and a block-diagonal
version, propagates both with an independently assembled dense influence matrix,
and compares actual pose tangents via SciPy matrix logarithms. It passes3,270checks
covering all768draws, fixed IDs, baseline information, full/cross/block matrices,
normalization, metrics and source/output identities. No source/tolerance amendment
or original estimator rerun is needed.

The empirical covariance uses the same128draws for characterization; it is not
an out-of-sample covariance predictor. No large residual covariance is inverted.
No significance test, physically calibrated coverage or universal dependence
model follows. Cross terms can have different signs for other ensembles.

Close this finite attribution. The measured dependence can motivate a future
predictive model, but that model would have to estimate dependence without using
evaluation-draw errors, compare against variance-matched scalar and within-match
controls, and retain bias/accuracy/availability endpoints. This step validates
neither a low-rank head nor a covariance correction. Other original-paper gaps
remain open; no automatic training, scene/noise search or recipe rescue follows.

Artifacts: [freeze](../../experiments/cross-match-covariance-freeze-2026-11-15.json),
[producer](../../tools/cross_match_covariance.py),
[independent auditor](../../tools/audit_cross_match_covariance.py),
[all matrices/results](../../results/cross-match-covariance/run-001/results.json),
[audit](../../results/cross-match-covariance/audit-001.json).
