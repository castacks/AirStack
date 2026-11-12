# Research direction informed by papers and completed experiments

12 November 2026. Targeted review of 12 primary sources, with deeper method and
limitations reading for PNEC and DNLS. This is not a systematic literature review
or 12 full reproductions. [Source/reading-level registry](../../experiments/research-direction-literature-2026-11-12.json).
No new neural inference, fitting, candidate scoring or reserve access occurred.

## Decision

Prioritize a **bearing-space rotation and uncertainty experiment informed by
PNEC/DNLS**, before another frontend swap or covariance fit. Retain thermal
tracking refinement as the alternative, rather than implementing both.

This is our research recommendation, not an author claim or a verified cause of
the observed regression. It connects uncertainty learning to an explicit motion
objective while separating rotation estimation from stereo depth. It also
offers a concrete hypothesis to test before paying for another training run.

## What our results actually constrain

| Existing evidence | What it establishes | What remains open |
|---|---|---|
| Final residual fit, Nov 10 | Training loss falls 17.65%; rain depth AbsRel rises 15.47%. Thermal VO translation/rotation means improve 3.04%/10.15%, but RGB remains better. | This particular pixel-loss adapter does not meet its accuracy rule; task-aligned learning was not exhausted. |
| Frozen thermal matcher, Nov 12 | Translation mean/RMSE improve 6.59%/7.32%; rotation worsens 16.01%/4.08%; coverage unchanged. | Original covariance was reused for different flow. Mean quality, covariance compatibility and geometric sensitivity are not individually identified. |
| EXP015/027/030 | Epipolar disagreement and independent LK disagreement were characterized. | Those were residual/ranking diagnostics, not NEC/PNEC rotation estimation or hybrid tracker interventions. |
| EXP082–086 and recent full-covariance studies | Solver acceptance/whitening and pose weighting have already been tested. | Repeating a weighted solver or scalar reliability cue is not a new contribution. A bearing-space residual is a distinct model, but still needs controlled comparison. |

Sources: [final residual](final-residual-fit-findings-2026-11-10.md),
[matcher comparison](thermal-matcher-pilot-findings-2026-11-12.md),
[repetition review](fusion-mechanism-review-2026-11-09.md), and the experiment
records hashed in the registry. These studies use different populations and
reference conventions; their percentages are not pooled.

## Paper-to-mechanism map

The last column is our assessment of relevance and cost, not published evidence
on our recordings. Only the first two entries received method-level reading
here; the registry distinguishes all other inspection levels.

| Paper | Relevant mechanism | Implication for this project |
|---|---|---|
| [PNEC, CVPR 2022](https://arxiv.org/html/2204.02256v1) | Uses anisotropic positional uncertainty in a bearing-based relative-pose objective. Rotation is estimated with unit translation direction, without reconstructing feature depths. | Best immediate geometry hypothesis. Read method, optimization, singularities and limitations. Covariance quality and outlier handling matter; metric translation scale is unavailable. |
| [Learning Correspondence Uncertainty via DNLS, CVPR 2023](https://arxiv.org/html/2305.09527v2) | Extends PNEC to both endpoint uncertainties and learns positional covariance through pose optimization; includes supervised and cycle-based self-supervised training. | Strong longer-term uncertainty-learning route. The authors explicitly distinguish improved pose estimation from calibrated covariance: absolute noise scale is not observable from this objective alone. |
| [Self-TIO, RA-L 2025](https://ieeexplore.ieee.org/document/10803066/) | Self-supervised 16-bit thermal features and hybrid learned-flow/KLT tracking. Publisher abstract inspected; full page blocked by verification. | Concrete alternative: coarse matching plus local precision and thermal-specific input handling. No faithful reproduction or released-code readiness is established here. |
| [TP-TIO, IROS 2020](https://arxiv.org/abs/2012.03455) | ThermalPoint and radiometric tracking use thermal-specific information within a visual-inertial system. | Supports retaining raw thermal information. Its complete system introduces IMU/feature-model dependencies beyond our current camera-only pilot. |
| [Thermal multi-spectral consistency, Shin et al.](https://arxiv.org/abs/2103.00760) | Learns thermal depth and ego-motion with thermal reconstruction and RGB photometric supervision through cross-camera warping. | Missing dense flow labels does not eliminate learning options. Cross-camera geometry, temporal association and visibility still require an explicit contract. |
| [D3VO, CVPR 2020](https://arxiv.org/abs/2003.01060) | Couples learned depth, pose and photometric uncertainty with direct VO; predicts brightness transformations during self-supervised training. | A precedent for matching learning and downstream residuals. Wholesale direct-VO integration would be a larger method change; this is not a drop-in thermal result. |
| [DROID-SLAM, NeurIPS 2021](https://arxiv.org/abs/2108.10869) | Recurrently updates motion and depth through dense bundle adjustment. | The correspondence/geometry coupling is relevant, but this project already ran DROID baselines. Another DROID run alone would repeat established context. |
| [MAC-I², 2026 preprint](https://arxiv.org/abs/2609.07116) | Propagates visual matching uncertainty to pose covariance and learns inertial uncertainty for initialization/calibration. | Relevant foundation, but pose propagation is already implemented/tested here. The paper does not establish RGB–thermal covariance comparability. |
| [Self-Supervised Spatial Correspondence Across Modalities, CVPR 2025](https://arxiv.org/abs/2506.03148) | Learns cross-/intra-modal correspondence with cycle-consistent representations from unlabeled data. | Keeps a shared representation route scientifically open. Its [author README](https://github.com/ayshrv/cmrw) still states code is forthcoming when checked; do not silently turn it into an implementation project. |
| [TartanMatch, October 2026 preprint](https://arxiv.org/abs/2610.14513) | A shared correspondence model covers 25 ordered modality pairs. | Already evaluated here as a within-thermal flow replacement. Our closed configuration is not a test of cross-modal feature fusion or all possible uses of that model. |
| [MACThermal thesis, CMU 2025](https://publications.ri.cmu.edu/3d-thermal-perception-for-autonomous-navigation-in-visually-degraded-unstructured-environments) | Describes dynamic-range normalization and geometry-consistent synthetic flow augmentation. | Synthetic supervision is already part of the baseline's stated approach. Proposing it again without a distinct hypothesis/ablation is insufficient. Full thesis was not parsed in this pass. |
| [KLTNet, August 2026 preprint](https://arxiv.org/abs/2608.24544) | Combines coarse flow with patch refinement and predicts anisotropic weights through differentiable triangulation. | Useful design lead for temporal precision and task-aligned weights. Thermal transfer, code and checkpoints were not established here; no integration recommendation follows yet. |

## Recommended next hypothesis and finite experiment

**Hypothesis:** on fixed thermal correspondences, a bearing-space objective can
estimate rotation more accurately than the existing depth-coupled estimator,
and positional covariance may help only when it matches those correspondences.
This is untested here. The recent rotation regression motivates it; it does not
prove depth error or covariance mismatch caused the regression.

Start with a fixed-input rotation assay, without training or replacing the
production solver. Compare unweighted NEC and asymmetric PNEC on identical
source pixels/endpoints, with native rotation as context. Keep original and
replacement-flow populations separately identified. Add an isotropic positional
covariance control within the PNEC objective: equal pixel covariance still
produces geometry-dependent residual variance and is not equivalent to NEC.

Use the asymmetric model first because the existing network covariance describes
flow conditional on source coordinates. Do not manufacture two independent
endpoint covariances by copying it into both slots of the symmetric model.
Paper endpoint assumptions and our conditional flow interface are different.

Before scoring, fix masks, source points, initialization, regularization,
iteration limits, degeneracy handling and the exact motion convention. Explicitly
include low-parallax/pure-rotation and near-epipole numerical cases. Do not use
reference rotations or translation directions to initialize/filter predictions.
Compare original and replacement means on the same fixed source grid, without
reselecting points for whichever solver looks better.

The cached 19 flow packets do **not** contain covariance tensors. One bounded,
source-pinned extraction from the original frontend would be required, with
parity checked against cached original flow. These covariances remain a control
for replacement flow, not its learned uncertainty. No such extraction or pose
solve has run in this review.

Use the existing exposed rain prefix for mechanical qualification. Before
claiming a direction worth training, lock the same assay on existing exposed
day/night prefixes, retaining every missing/degenerate pair. Report rotation
mean/RMSE, angular tails and coverage per recording, and translation-direction
error separately. This is a rotation-method assay: no reference-scaled metric
translation and no grafting a new rotation onto old translation to manufacture
a full-VO gain. A successful assay would require a separate full-motion
integration contract before further claims.

For a subsequent numerical freeze, require lower rotation mean and RMSE than
the declared unweighted control per recording without coverage loss; compare
against native rotation as well. If only one favorable recording improves,
report the condition dependence and stop the promotion decision. Numeric
budgets and exact input pins must be specified in that execution freeze.

## Implementation cost and alternatives

The [official PNEC source](https://github.com/tum-vision/pnec/tree/51dc8ddfa93a9622efcaf534908a2b30928a2e04)
lists Ceres, SuiteSparse and Basalt/opengv dependencies and supplies bindings.
The [DNLS source](https://github.com/DominikMuhle/dnls_covs/tree/73d2e5c8dcd9afe2e6a82db3425d7d2704c54ae0)
also requires PNEC/opengv bindings. Code availability does not mean installation
is trivial. A small equation-level prototype in the existing numerical runtime
could check feasibility; it must be labeled as such, not a reproduction of the
authors' full optimizer. Do not begin a full new SLAM stack or dependency port
to get this diagnostic running.

Self-TIO-inspired local tracking refinement is the alternative if geometry
feasibility fails or the rotation assay does not support a useful direction.
Earlier LK agreement studies do not test that intervention, but a custom hybrid
would not be a Self-TIO reproduction. Do not start it concurrently or use it as
an automatic rescue of the closed TartanMatch configuration.

Self-supervised DNLS, thermal reconstruction and synthetic augmentation show why
absence of dense correspondence labels is not a global research stop. They do
not supply independent evaluation truth or establish thermal transfer by
themselves. Training requires a stable forward/gradient contract, a declared
target and recording-separated evidence. Existing small exposed prefixes are
development data throughout.
