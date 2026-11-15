# Shortlist after the original-paper gap map

13 November2026, America/New_York. Targeted later-work comparison and local
source/asset inspection. No new accuracy experiment, fitting or inference.

## Recommendation

Keep two principal method directions and one lower-cost alternative. My preference
for the first technical investigation is A, because its assumption is explicit
in recovered source and its tracks are already available. That is a feasibility
judgment, not proof of novelty or a requirement to stay with uncertainty research.
B is a substantive learning/representation direction and should not be rejected
because it does not answer the old camera-ranking question.

| Candidate | Specific contribution possibility | Main risk | Relative implementation burden |
|---|---|---|---|
| A — dependence-aware persistent tracks | A compact joint observation-error model that avoids counting correlated track evidence as independent, with accuracy/consistency evidence. | Classical correlated-noise estimation is established; specific learned-track treatment and benefit must be distinguished. | Medium; saved tracks/solver exist, cross-time covariance and whitening do not. |
| B — thermal correspondence learning under appearance change | A supervision/representation mechanism that separates geometric displacement from radiometric or preprocessing change. | Self-supervised thermal features and multispectral consistency already exist; a different loss name or normalization is insufficient. | Higher; requires a trainable frontend, nuisance/input contract and defensible correspondence evaluation. |
| C — uncertainty plus geometric diversity in selection | A fixed-budget selection objective that accounts for directional constraints and redundancy rather than only individual reliability. | Information-based feature selection is mature; may be an integration result rather than a strong method contribution. | Lower for saved candidate sets; full-frontend coverage remains to be verified. |

## Later-work comparison: what cannot be claimed new

- [MomentBA v1](https://arxiv.org/html/2609.13691v1), III–IV: derives a full2D
  anisotropic covariance from localized matching-response moments and uses it in
  differentiable BA. The inspected objective sums per-observation weighted
  residuals. It is direct overlap for “derive anisotropic covariance from a cost
  volume,” not evidence here of a full cross-time observation covariance model.
  Methods inspected; no code, checkpoint or independent result reproduction.
- [CoProU](https://link.springer.com/article/10.1007/s11263-026-02915-y),3–4/9:
  combines target and projected reference uncertainty in a self-supervised
  photometric loss, including a multi-frame architecture. Its derivation uses
  independent source/target noise and a moment-matched effective scale. This
  differs from modeling off-diagonal dependence among a track's repeated
  geometric observations. Its limitations include weak texture supervision,
  unsupervised dynamic-region depth and intrinsics sensitivity. Targeted methods/
  limitations read, not full-system or code reproduction.
- [Self-TIO publisher abstract](https://ieeexplore.ieee.org/document/10803066/)
  already describes self-supervised16-bit features and hybrid learned/KLT
  tracking. Its [author repository](https://github.com/junwoonlee/self-tio-dataset)
  states no public code/network release is planned. This recheck reads the
  abstract and repository scope only, not the full method. Our raw-patch pilot
  was not a reproduction of its learned extractor.
- [ThermalSfMLearner v3](https://arxiv.org/html/2103.00760v3),III: raw thermal
  normalization, thermal/visible reconstruction and geometry-consistency losses
  already provide self-supervision. Targeted method recheck only. Its direct
  checkpoint transfer was tested locally; a learned correspondence objective is
  a different intervention, but generic multispectral consistency is not new.
- [Good Feature author page](https://sites.google.com/site/zhaoyipu/good-feature-visual-slam)
  and [released integration README](https://github.com/ivalab/GF_ORB_SLAM)
  describe feature-subset selection for pose estimation and an ORB-SLAM
  integration. This is primary project/repository-level reading, not a fresh
  full-paper audit. A generic information-matrix/log-determinant selector would
  need this baseline rather than a novelty claim.

These are overlap findings from a targeted comparison, not an exhaustive survey.
The original-paper map remains the basis for connecting each direction to
MAC-VO/MACThermal/MAC-I². None is declared an unfilled field-wide gap.

## A: distinguish temporal dependence from merely adding frames

**Hypothesis:** shared image/network information and accumulated track error
make independent-observation weighting overconfident or distort measurement
influence. A structured joint covariance could correct that mismatch. This is
not an established cause of the previous joint-window regression.

**Concrete source evidence:** recovered `tools/persistent_tracks.py` explicitly
states independent increments and ignored shared network/image correlation. Its
`WindowTracks.advance` propagates marginal covariance as
`C_next=(I+J_flow) C_prev (I+J_flow)^T+C_flow`. The packet retains track identities,
per-frame UV/stereo observations, covariance and sampling witnesses. The existing
windowed solver already refines depths; “add persistent tracks/depth BA” repeats
completed work. Those verified sources/results are in the separate recovery
extraction, not a claimed current native runtime.

**First bounded falsification:** before learning a new model, compare a declared
small block-covariance model against independent blocks in synthetic tracks with
known joint noise and the same geometry. Include zero correlation, correlated
increments and a systematic-bias control; preserve identical marginal variances.
Compare empirical pose dispersion/coverage and error, with a fixed global-scale
control so covariance inflation cannot masquerade as dependence modeling. An
oracle joint-noise arm is a diagnostic ceiling, not a deployable method.

The candidate must construct a positive-definite joint covariance from permitted
inputs; correlation cannot be inferred from marginal covariances alone. Saved
real tracks permit later replay, but one realized trajectory does not provide
known noise samples or prove physical correlation. A reference-assisted estimate
of correlation would require training-only use and separate prediction locks.

**Stopping decision:** no real-data training if mechanics fail on the zero-
correlation control, improvement depends on reference-only correlations, or the
scale control explains the claimed effect. If the synthetic distinction survives,
real-data support and prior-art depth still gate a recording-excluded study.
Synthetic success alone is not a paper result. Seed, cases, statistical precision,
thresholds and solver source must be frozen before execution.

## B: thermal-specific supervision and representations

**Hypothesis:** geometry-generated training pairs do not cover some real temporal
appearance changes; learning correspondences with a distinct nuisance-aware
supervision mechanism can improve tracking while keeping informative geometry.
The mechanism must differ from existing raw normalization, cycle consistency,
Self-TIO hybrid tracking and a pretrained matcher swap.

**Assets:** original thermal frontend/source and recoverable native16-bit data;
existing author thermal-SfM reproduction tools; previous input/flow/depth reports.
The raw-patch pilot changes local endpoint refinement with unchanged covariance;
the six-parameter head changes observation weights. Neither tests learning a new
thermal representation. The failed heavily clipped default depth transfer is
also not such a test.

**First bounded falsification:** select one declared, geometry-preserving thermal
appearance perturbation family, with its physical relevance separately justified.
Compare the original frontend, equally budgeted geometry-only training and one
candidate learning mechanism. Use exact known correspondences for this controlled
assay; do not grade a model against its own matches. Preserve spatial geometry,
input dynamic range and support across arms. Report correspondence displacement
error and retained support together; pose is a later endpoint.

A synthetic appearance assay tests only the chosen nuisance. A real-recording
comparison requires independent reference support or qualified geometry, not an
unbounded label-collection campaign. The precise mechanism and feasible training
budget remain undesigned, so B is a research direction rather than an executable
protocol. Reject it if differentiation from published methods reduces to a
normalization/augmentation sweep or gains come only from dropping hard pixels.

## C: geometry-aware selection as a bounded alternative

Hold available feature means, covariances, solver and observation count fixed.
Compare original reliability selection, spatially stratified random selection and
a standard information-based selector before proposing a new selection method.
Use no supplied poses to choose features. Evaluate directional conditioning as a
diagnostic and actual motion error/support separately; better conditioning does
not establish better estimation under biased measurements.

Availability must be checked: packets containing only already selected features
cannot evaluate selection over rejected image candidates. Existing geometry/
conditioning experiments must be mapped before implementation. Even a positive
baseline comparison would establish value of integration, not a new algorithm.
This makes C cheaper to test but weaker as a novelty-led first choice.

## Why other families are not the first shortlist

Thermal-inertial recovery, online calibration and active perception remain valid
paper-derived directions. They require more persistent state, input qualification
or closed-loop/hardware infrastructure than the recovered offline comparison.
That affects present feasibility; it does not mean those gaps are closed.
Shared RGB–thermal representations also remain open, but direct cross-modal
matching would need an additional visibility/geometry evaluation contract.

No production code changed, no model/runtime was installed, and no experiment
was selected or executed by this review. The recommendation is to develop A's
finite feasibility protocol first while retaining B as a distinct learning path.
It replaces the earlier habit of automatically returning to camera-ranking or
another scalar weight. No promise of a positive result or verified novelty.
