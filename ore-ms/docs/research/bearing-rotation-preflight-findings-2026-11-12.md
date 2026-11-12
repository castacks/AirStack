# Bearing-space rotation: numerical feasibility and inputs prepared

Execution update: the later [rain assay](bearing-rotation-assay-findings-2026-11-12.md)
is complete and supersedes the pending-execution status below. It records a
pre-execution exact-eight SVD correction; the original source matching this
preflight's hashes is preserved in `results/bearing-rotation-assay-validation/source-v1/`.
The21tests and input receipts below describe the original preparation step.

12 November 2026. Follows the [literature decision](research-direction-literature-2026-11-12.md).
**The equation-level prototype and covariance recovery pass. No real-data pose
candidate has been solved or scored.** This completes the numerical/input
preparation step, not a new accuracy result.

## What is implemented

`tools/bearing_rotation.py` implements unit-ray unprojection and its analytic
pixel Jacobian, conditional target-bearing covariance propagation, unweighted
NEC and regularized asymmetric PNEC residuals. The convention is
`X_source = R X_target + t`, with unit translation direction.

It follows the objective in [Muhle et al., CVPR 2022](https://arxiv.org/html/2204.02256v1).
The implementation differs from the published complete method: first-order
rank-two covariance propagation replaces the unscented transform, and an
eight-point/cheirality start plus SciPy joint least squares replaces the
authors' alternating/globalized optimization. It is an equation-level prototype,
not a reproduction. The [DNLS extension](https://arxiv.org/html/2305.09527v2)
remains a later learning option; no differentiation-through-solver or training
was implemented in this step.

The prototype uses 5 local parameters, a positive bearing-variance regularizer
of 1e-12, SciPy `max_nfev=200`, three-point numerical Jacobians, linear loss and
termination tolerances 1e-10. Eight-point initialization rejects insufficient
rank. Low parallax and final Jacobian rank failures remain explicit unavailable
statuses. The optimizer reports budget exhaustion rather than treating a capped
solve as convergence. These are fixed prototype choices, not paper-equivalent
hyperparameters or tuned settings.

## Verification

Seventeen synthetic tests pass: nine exact nonplanar scene solves (three seeds
by three covariance/objective cases), analytic-bearing finite differences,
scalar residual equations, translation-sign invariance, covariance scaling,
pure-rotation degeneracy, near-epipole regularization, bad-input rejection,
isotropic pixel covariance versus NEC weighting and evaluation-budget status.
They establish these implementation contracts, not robustness to arbitrary noise
or outliers, real-data convergence or improved thermal accuracy.

Four additional synthetic tests verify the optical/NED axis conversion and the
inherited nominal frame/time rotation mapping, including inverse motion and
independence from arbitrary translation/lever-arm terms. They validate arithmetic
under the declared model, not physical synchronization or mounting.

All 21 tests run in the recovered, unchanged MAC runtime with NumPy 2.2.6 and
SciPy 1.17.1. No new environment or native dependency build was needed. The
official PNEC/Basalt/Ceres optimizer was not installed.

## Real inputs recovered without pose scoring

The acquisition was frozen before inference. The same exposed rain sequence
`_2021-08-06-16-19-00`, stems 000000–000019, supplies 19 adjacent pairs.
The fixed source grid is x=16..624, y=16..240, both inclusive in steps of16:
585 integer source pixels per pair.

Reused the original thermal frontend's saved effective configuration, checkpoint
and source preparation. The bounded recovery performs two first-depth calls and
19 batched pair calls. **Every recovered dense original temporal flow is
bitwise identical to its cached array.** There is one recovery attempt; no
parity tolerance adjustment or source/model change. Only grid covariance and
endpoints are saved; the replacement matcher is not run again.

All 11,115 grid covariance matrices are finite and strictly SPD. Original and
replacement means share 10,198 finite inbound grid observations, 529–547 per
pair. The same support is kept for all prospective objectives. Inbound is not
visibility or correspondence truth. The original covariance describes original
flow; using it with replacement endpoints remains an explicit control, not a
calibration claim.

The saved-input audit checks 29 source/packet hashes, all19 cached endpoint
samplings and masks, covariance SPD/schema and support counts. It does not
independently rerun the neural covariance or establish its statistical validity.
All40 thermal source image hashes pass. No reference value is parsed or used by
the acquisition or inventory scripts, and no real pose solver is called.
Inherited integrity verification does hash earlier reference artifacts without
interpreting them; the acquisition receipt's `reference_reads=0` means no
value-consuming reference access, not zero file bytes hashed.

The first saved-input audit was interrupted because it repeatedly decompressed
an array inside its scalar sampling loop. Caching each array once fixes the
auditor's runtime; the completed audit checks the same packets and masks. No
neural extraction, candidate solve or scientific score was repeated.

## Locked design and remaining execution work

The [assay protocol](../../experiments/bearing-rotation-assay-protocol-2026-11-12.json)
binds inputs and numerical choices. Its explicit status is
`DESIGN_AND_INPUT_LOCKED_EXECUTION_CODE_FREEZE_PENDING`. The [acquisition freeze](../../experiments/bearing-rotation-acquisition-freeze-2026-11-12.json)
is separate and complete.

The planned rain assay has at most114 solves:19slots ×2 fixed mean sources ×3
objectives (NEC, unit isotropic pixel-covariance PNEC, original-covariance PNEC).
Original means plus original-covariance PNEC are the primary candidate;
replacement means are secondary context and cannot rescue a failed primary.

Before those real solves, implement/test and freeze the declared common
geometric outlier filter and scoring wrapper. The design fixes essential-matrix
RANSAC seed0, probability0.999, threshold1 native pixel and5000iterations, with
original/replacement mask intersection and at least8points. There is no filter
or threshold search; rejected/degenerate slots remain in coverage. These filter
choices have not been exercised on the real packet in this step.

The primary development gate requires lower rotation mean AND RMSE than NEC,
no higher matched error than native thermal, and no coverage loss relative to
NEC. A rain pass would motivate an identically specified day/night comparison,
not training or adoption. A fail closes this prototype configuration, without
rejecting PNEC/DNLS as published methods.

Metric translation cannot be recovered from the unit direction. In particular,
do not conjugate a unit-length translation through a displaced sensor frame and
claim a metric or directionally comparable body motion. A camera-local reference
contract is needed for translation-direction scoring; it remains unavailable in
this design. Rotation-only mapping can reuse the existing nominal convention.

No new frontend swap, neural fitting, full-VO backend integration, reserve access
or production-default change follows automatically. Local receipts and packets:
`results/bearing-rotation-preflight/run-001/`. Source:
`bearing_rotation.py`, `prepare_bearing_rotation.py`, and their two test modules.
The saved-input audit is `audit_bearing_rotation_preflight.py`.
