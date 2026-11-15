# Residual conditioning: prior-art comparison and contribution decision

14 November2026, America/New_York; source intake began13November.
Branch `ore-ms-orb-development`,baseline `a79658f6341194b0954852226f200313ccdf5408`
plus working changes. Bounded primary-source review of five methods,13 pinned
Python source files. No author code import/execution, model run, fitting or new
accuracy comparison.

## Decision

**Do not promote “account for the fixed/uncertain source” into a new-method claim.**
Both-endpoint uncertainty propagation already exists in inspected methods, and
joint BA already represents shared geometry and eliminates latent variables.
The review does not establish a specific omitted source-noise term or estimator
bug in those systems. Our derivation remains useful implementation/modeling
analysis; the earlier day improvement does not establish novelty or prove that
published methods neglect source uncertainty.

This narrows the proposed conditioning thread. It does not close the many
original-paper gaps, prove that every possible dependency is modeled, or establish
that an explicit learned residual-dependence model could not help. No further
numerical variant is selected by this review.

## Five-method comparison

| Method/version | Optimized quantities and uncertainty treatment | Consequence for our proposed gap |
|---|---|---|
| [MAC-VO v2,III-C/D](https://arxiv.org/html/2409.09479v2); official source8683b532 | The published3D registration residual weights both map/source covariance and rotated observation covariance. Pinned `ICP_TwoframePGO.covariance_array` implements their sum. Its reprojection graphs are separate code paths. | Fixed map coordinates in the pose solve do not imply zero map uncertainty. Our conditional-ray/depth experiment is a different formulation; it cannot diagnose a missing term in this native ICP graph. |
| [DNLS v2,3.2–3.3](https://arxiv.org/html/2305.09527v2); source73d2e5c | Symmetric PNEC propagates uncertainty from both image bearings. Active Theseus cost passes both endpoint covariances and differentiates its state-dependent denominator. The original asymmetric model is explicitly distinguished. | Adding source-endpoint uncertainty or residual-dependent covariance derivatives is not a new generic contribution. This is relative epipolar pose estimation, not our multi-frame conditional-ray BA. |
| [DROID v1,3.2–3.3](https://arxiv.org/html/2108.10869v1); source2dfd39f0 | Learned correspondence revisions/confidence weights enter joint pose/inverse-depth optimization. Pinned Python BA constructs pose/depth blocks and calls a Schur solver. Reference pixel grids define source rays. | Fixed source grid coordinates and separate observation weights do not establish missing geometry coupling. The learned optimizer weights are not, merely by notation, calibrated physical measurement covariances. |
| [DPVO v2,3.1/AppH](https://arxiv.org/html/2208.04726v2); current public source0ac95b65 | The paper explicitly keeps patch pixel coordinates constant while optimizing poses/inverse depth; confidence weights are learned through BA supervision. Inspected public Python code updates depth and uses the Schur complement. | The broad fixed-ray/joint-depth architecture already exists. Adding it or reimplementing Schur elimination is not new. Current public code is not claimed to be the exact2023 experiment revision. |
| [MomentBA v1,IV-A–C](https://arxiv.org/html/2609.13691v1) | The paper specifies poses and3D landmarks as variables, anisotropic correspondence weights and a reprojection objective. No implementation was inspected for this method. | Do not silently substitute our one-depth fixed-ray state for its full landmark state. The inspected equations do not resolve source-observation bookkeeping or establish a missing term. |

Reading is targeted to the listed methods/equations and source functions, not
full-paper/benchmark reproduction. MACThermal inherits covariance-aware estimation
in the previously reviewed thesis, but no separate released MACThermal optimizer
was newly audited here; do not claim a sixth implementation comparison.

## Critical distinction: fixed optimization variable versus exact measurement

The old derivation labels one arm “exact/conditioned source” and explicitly sets
e0=0. That is an **assumption that source error is absent**, not a consequence of
fixing the observed source coordinate in the solver. Conditioning on a noisy
observed value generally does not make the observation's error zero.

A scalar example makes this distinction explicit. Let

    z_s = x + eps_s,
    z_t = x + theta + eps_t,
    eps_s ~ N(0,q), eps_t ~ N(0,r), independent.

The estimate theta_hat=z_t-z_s has error eps_t-eps_s and variance q+r, even
when z_s is numerically held fixed during the theta solve. Setting q=0 changes
the noise model. With an unknown x and a flat prior, conditioning on observed
z_s still leaves x|z_s uncertain with variance q; the predictive target variance
is q+r. Known true source x, an exact reference model, and an observed noisy
source are different statements.

Thus the day control's4.80%translation/31.05%rotation mean reductions support
its **tested weighting choice**, not the claim that removing source uncertainty
is physically correct. A held-fixed ray may define a useful conditional
correspondence problem, but its covariance semantics must follow the actual
measurement-generation/training model. The experiment did not identify that
physical model from reference poses. Its historical labels/results remain fixed.

Root UV residual cancellation in the anchored parameterization is still an
algebraic identity: it does not add an independent geometric constraint. It does
not imply source uncertainty cancels from every later residual. The existing
uncertain-source derivation retains the prediction's source-ray Jacobian and
cross terms for those later observations.

## Why diagonal observation weights do not prove missing coupling

There are at least three distinct sources of dependence:

1. Multiple residuals depend on the same unknown pose, depth or landmark.
2. Measurement errors have source/target endpoint uncertainty.
3. Different learned correspondences have correlated errors beyond their shared
   latent geometry, for example through reused network/image information.

Ordinary joint optimization already handles the first algebraically. MAC-VO
and symmetric PNEC explicitly address versions of the second. An explicit
cross-observation noise model concerns the third. Conflating these would create
a false novelty claim. Schur coupling is not proof that the third is fully
modeled either.

At a fixed linearization, consider a pose perturbation p, latent geometry z,
independent Gaussian residual noise covariance R and a proper Gaussian prior
z~N(0,P). The joint quadratic is

    E(p,z) = (A p + B z - y)^T R^-1 (A p + B z - y) + z^T P^-1 z.

Eliminating z gives the pose quadratic precision factor

    W_eff = R^-1 - R^-1 B (P^-1 + B^T R^-1 B)^-1 B^T R^-1
          = (R + B P B^T)^-1.

A shared latent variable therefore produces coupled pose information even when
R is block diagonal. This is a standard linear-Gaussian/Schur identity, not a
new derivation claim. Flat-prior/gauge cases require their own rank treatment;
the proper-prior identity is not an assertion about a solver's damping being
physical uncertainty.

Profiling a joint MAP quadratic and marginalizing a random latent variable are
also different operations. A Gaussian marginal likelihood contains a covariance
normalizer/log determinant; a profile objective need not contain that same term.
Its relevance depends on which objective is claimed and whether the quantities
vary with state. A weighted geometric optimizer is not automatically an exact
Gaussian marginal likelihood. Do not call absence of a log determinant an
implementation bug without establishing that objective contract.

## What remains unresolved, precisely

The inspected DROID/DPVO paths use separate learned confidence weights, and
MomentBA's displayed objective sums weighted observation terms. This inspection
does not establish a statistical model for cross-observation learned matching
errors beyond shared geometry. It also does not establish that none exists in
other work, that those errors are materially correlated in our data, or that
an extra model improves accuracy.

Our earlier experiment already tested one particular first-order
independent-increment joint model and did not obtain consistent benefit. That
configuration stays closed. Renaming it “residual-consistent,” adding generic
source covariance, or treating existing Schur coupling as a missing innovation
would not make another run informative.

A genuinely distinct future proposal would need to identify an observable
noise dependence **after** accounting for latent geometry, specify what its
network covariance means under conditioning, compare with relevant correlated-
measurement estimation work, and show why existing learned weighting cannot
already represent the claimed correction. No such mechanism or deployable
estimator has been established by this source review. No training or estimator
variant is launched here.

## Effect on the report and research direction

Keep the [technical-report draft](uncertainty-interfaces-report-draft-2026-11-13.md)
as implementation analysis and bounded empirical findings. The source-conditioning
analysis can explain our modeling choices; it is not presently a defended
new-method thesis. This note supersedes any inference from the preceding
[decision](shortlist-contribution-decision-2026-11-13.md) or conversation that
our local day gain establishes a missing component in existing learned VO/BA.
Neither historical numerical results nor their freezes are altered.

The [original-paper map](original-paper-gap-map-2026-11-13.md) still includes
continuity/recovery, posterior approximation, sensor adaptation, thermal
supervision/representation, multispectral estimation and active perception.
A method contribution should be chosen from a precise supported question in
that broader space. Do not automatically narrow the user's goal back to
incremental thermal utility, or conclude “no gaps” because this generic claim
has substantial prior overlap.

## Source record and verification

Thirteen pinned Python files were downloaded and parsed, not imported or run.
`results/residual-conditioning-prior-art/source-review.json` records hashes,
function line anchors, five static corroborations and reading depth. Three
intake receipts preserve successful URLs and unavailable local-DPVO revision
requests. A public DPVO commit was resolved and pinned separately; local-fork
provenance is not promoted to the author's experiment revision. An incorrect
preliminary DPVO arXiv lookup was discarded before relying on its content;
2208.04726v2 is the actual inspected DPVO paper.

No inference, covariance fit, pose solve, new image/reference payload, protected
split or production estimator change occurred. This is a bounded overlap audit,
not an exhaustive literature survey or a finding that existing methods are
statistically complete. The generic claim fails the contribution test; a
specific new omission has not been established.
