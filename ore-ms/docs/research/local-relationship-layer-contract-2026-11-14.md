# RGB-conditioned local thermal relationship layer: interface and input qualification

**Execution update:** The [local relationship learning comparison](local-relationship-learning-findings-2026-11-14.md)
is complete. The design and next-step language below records its pre-execution
state; do not treat qualification or the40update run as pending.


14 November2026 America/New_York; ore-ms-orb-development baseline53afa19d1.
Concrete candidate interface and reference-free cache completed. No candidate
training, new pose optimization or accuracy result in this preparation.

## Selected interface and hypothesis

Use the logical RGB MAC-VO and thermal MACThermal branches as sources of accepted
local measurements for a shared geometric backend. Keep RGB residual weights at
their original covariance precision. A proposed learned layer assigns each
thermal residual a bounded task precision multiplier using local thermal evidence
and how its constraints relate to the RGB fit. It does not estimate a scalar
conversion between RGB/thermal uncertainty; it does not certify task multipliers
as new statistical covariances. No IMU branch is included.

Hypothesis: conditional local evidence identifies useful thermal observations
more effectively than own-camera geometry/covariance summaries. The proposed
nonlinear model must be tested, not inferred successful from oracle opportunity.
This is an application-specific adaptation hypothesis; generic learned weighting
and task-aware uncertainty already exist. Shared6parameter affine weighting was
already executed and failed; this proposal differs by same-camera source/target
patch evidence, per-factor RGB-relative directional constraints and nonlinear
interaction, not merely a larger scalar confidence score. Existing geometric
logDet selection did not establish those directional gains as accuracy signals.

## Input definition:24values per accepted correspondence

Own-camera16: source normalizedu/v,target normalizedu/v; log source camera depth;
log target disparity; log targetUVcovariance trace and disparity variance;
normalized flowu/v; source/target11x11patch standard deviation; source/target
patch gradientRMS; source/target patch-valid masks. RGB grayscale and thermal
sensor counts use their own imagep5/p95clipping for descriptor extraction only.
Native frontends and stored measurements remain unchanged. Patch contrast is
not calibrated temperature or verified matching confidence.

RGB-relative8: six log1p normalized information contributions in RGB-information
eigendirections, own whitened residual energy at RGB-cov pose, and meanRGB
whitened residual energy there. Each factor's projection Jacobian and covariance
use the same scaled optical-body left tangent as RGB; translation scaled by
medianRGBsource camera depth. RGB information eigenvalues normalize quadratic
contributions. No reference pose enters these calculations. RGB's fitted pose
and residuals are available processing outputs, not independent error truth.

Eigenvector sign is irrelevant under quadratic projection; repeated/near-repeated
eigenvalues can make eigendirection order unstable. No exact eigenvalue ties
observed; smallest adjacent gap divided by maximum eigenvalue0.0001404. This
observed gap is not a general robustness guarantee. A later model must retain
this interface limitation; basis/tolerance changes require explicit amendment.

Pixels on the two sensors are not assumed aligned. Source and target patches
belong to the SAMEcamera; corresponding surfaces acrossRGB/thermal are not
established by this representation. Inputs describe two-frame accepted matches,
not three-frame or longer persistent tracks. The source/targetK matrices match
within the fixed1e-8bound in these packets. Original actual timestamps are retained
through the parent binding; same-ID nominal simultaneity remains an assumption.

## Completed preparation

Three previously exposed100group day/night/rain sequences.297expected motion
intervals,593available camera packets and88671accepted factors. One rain thermal
packet(pair43)is unavailable, explicitly recorded; RGB remains available. Missing
patches would be zero descriptors plus explicit masks without factor pruning;
all current patches fit within image bounds. Bothbranches have caches for
matched-capacity RGB-only and modality/input ablations.

Every packet hash checked against original binding. Every source/target image
hash checked against original source manifest before descriptors are computed.
Per-factor feature shapeN×24and finite-value checks pass. Direct whitened-Jacobian
quadratics match independent assembled-information projection; reverse-order
permutation checks pass. These check representation mechanics, not prediction
or physical calibration. No fold statistics are fitted and no reference/score
files are opened by local_relationship_cache.py. Source and geometry SHA pins,
all593file hashes and missing cases are in cache-001/lock.json. Prior supplied
reference results remain exposed historically, so this is not blind validation.
CPU preprocessing/cache time11.70seconds; it is extra work, not compute savings.

## Concrete proposed model and bounded comparison

Candidate:24input→16tanhhidden→1sigmoid output per thermal factor. Multiplier in
[0,1], originalRGBprecision fixed; no learned point/depth correction. This
specification permits thermal rejection but not negative weights or precision
amplification. The gate is task utility, not a covariance head. The unchanged
12iteration source-conditioned solver supplies motion, and the training target
is translation plus rotation error in explicitly fixed units. A learning protocol
must bind initialization, units,optimizer/update schedule and failure handling
before the first update; the cache does not constitute a training freeze.

Required controls: nativeRGB context; same-backend RGB-only,thermal-only and
unweighted/full-covariance joint; train-fitted constant thermal gate; prior six-
parameter affine head; same24→16→1architecture with local16inputs only, context8
only and all24inputs(mask excluded inputs to zero); equally trained RGB-only
counterpart to distinguish thermal benefit from learned reweighting. Training
and normalization use two complete recordings, excluded third used for evaluation;
allfold predictions lock before scores. Budget/update comparisons explicit,
not assumed compute-matched because architecture size matches.

Evaluate per-intervalT/Rmean,RMSE,p95,coverage and failure counts. Retain partial
benefits with regressions. A repeatable gain in a stated condition is meaningful;
no demand for universal success. A pose claim still must compare against strong
RGB and applicable simple fusion controls. Current shared solver is a conditional
prototype; interval benefit precedes any full nativeVO/trajectory claim. No
runtime or cost benefit is implied by a small gate or fewer residuals.

## Prior work and next execution boundary

[DNLS](https://arxiv.org/abs/2305.09527) already learns correspondence uncertainty
through pose optimization; [Deeper-DICE](https://arxiv.org/abs/2007.14943) learns
VO correction/error models. Existing local primary-source reviews remain the
reading basis; this contract introduces no new full-paper/code-reading claim.
[Shared weight learning](shared-weight-learning-findings-2026-11-12.md),
[local geometry](local-thermal-geometry-findings-2026-11-14.md) and the
[architecture reset](rgb-thermal-architecture-and-contribution-reset-2026-11-14.md)
bound this proposal. MSO remains a direct multispectral estimator comparator.
The proposed layer's novelty is unresolved; its precise sensor-conditioned input
ablation and pose utility must be demonstrated rather than presumed.

Next implement the candidate and qualify zero/constant-gate parity plus gradients
through the actual finite solver. Then bind one training protocol and run it.
Do not rerun the old6parameter head or context-free image-regression recipe as
new work. The present deliverable is a qualified input interface and candidate
comparison, not a trained fusion system.
