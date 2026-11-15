# Next original-paper direction: fixed-budget feature selection

13 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408`. Source/asset intake and test design,
not a selector experiment or training run.

## Decision

Finish the thermal-learning target investigation at its present descriptive
boundary. Preserve its positive baseline results and keep the original MACThermal
supervision/transfer questions open. The completed controlled-warp assay did not
expose matching failure; natural supplied-geometry disagreement does not identify
true matching error. Neither result justifies replacing the existing uncertainty
head or treating its clean predictions/nominal geometry as independent labels.
A distinct thermal representation remains possible; it needs a specific mechanism
and defensible evaluation, not more appearance severity to generate failures.

Advance shortlist C to a finite **accepted-match subset selection** comparison.
This is MAC-VO gap V3 in the original-paper map: does a fixed observation budget
preserve complementary motion constraints after reliability filtering? This
moves to another original-paper question; it does not claim it is the sole gap,
that the thermal direction failed, or that a new selection method is established.
The concrete next deliverable is a qualified, frozen offline comparison with
existing means/covariances held fixed. No new inference or data acquisition is
needed for its restricted scope.

## Original method and prior overlap

[MAC-VO v2, III-B](https://arxiv.org/html/2409.09479v2) already combines NMS,
geometry and uncertainty filtering. NMS explicitly addresses image-space
clustering. The inspected local `CovAwareSelector.select_point` then applies
`torch.randperm` to its eligible set before the point budget. Thus the gap is
not an absence of spatial filtering or a claim that it simply picks the lowest
variance points. Local source is pinned separately from the publication.

[Good Feature Selection, IROS 2018 / arXiv v1](https://arxiv.org/pdf/1905.07807),
II–IV and Algorithm 1, already studies map/measurement error and spectral subset
selection, including Max-logDet and randomized greedy approximation. Its paper
also distinguishes bias from variance. Generic information gain, whitening or
adding a geometric diversity term is not novel. Our proposed exhaustive greedy
control uses a different stereo 3D residual and is not a reproduction of its
combined-Jacobian/ORB system or stochastic acceleration.

The previous local geometry-support studies selected **camera experts** using
conditional pose information and observed mixed/negative transfer. They do not
perform fixed-budget within-camera point-subset selection. Conversely, repeating
camera selection under a new score would not test this question. Persistent
five-frame/temporal-weighting configurations remain closed without retuning.

## Available pool and limits

The SHA-verified persistent-window recovery retains six native-off tensor maps.
For each accepted match these store source/target pixels, depths, both 3D spatial
covariances, frame-pair identity, intrinsics and timestamps. Graph provenance and
schema are recorded in `results/feature-selection-intake/inventory.json`.

| Recording | Camera | Matches across first 19 pairs | Per-pair minimum–maximum | Pairs with more than 32 matches |
|---|---|---:|---:|---:|
| Day | RGB |3707|191–199|19/19|
| Day | Thermal |2192|76–180|19/19|
| Night | RGB |3799|199–200|19/19|
| Night | Thermal |1366|40–128|19/19|
| Rain | RGB |3736|193–200|19/19|
| Rain | Thermal |1187|23–101|18/19|

Total 15,987 accepted matches and 114 camera-pair slots. No supplied pose
reference values, new selector outputs or pose errors were read for this inventory.
Twelve schema/adjacency checks and an independent six-stream histogram/schema
cross-check pass. These are availability checks, not accuracy evidence.

Dense pre-selection candidate fields are not retained in these six graphs.
Track witnesses contain only four-corner samples around previously selected
tracks, not full-image flow/depth/uncertainty maps. Thus the executable comparison
can test **subselection from already accepted native matches**, conditional on
prior NMS, reliability filtering, random sampling and downstream acceptance.
It cannot test recovering rejected features, replacing the complete selector,
end-to-end VO tracking survival, or frontend compute savings.

A speculative 128-point budget was considered before counting: it would reduce
only three thermal day pools and none of the thermal night/rain pools. The
proposal is therefore 32 points, capped by available N, using counts alone and
before any subset or pose outcome. This reduces 113/114 pools; the remaining
23-point case stays in every arm unchanged. No budget curve or later favorable
budget selection. The original inventory preserves its tentative 128 proposal;
the separate design records this explicit count-based amendment.

## Proposed comparison, before execution qualification

Population: the first 19 adjacent pairs per camera in each of the three exposed
prefixes. No new recording or holdout claim; no RGB–thermal joint solve. Use
camera-local observations and clocks. Retain every one of the 114 slots.

Each arm receives exactly the same accepted-factor pool and k=min(32,N):

- Uniform sampling without replacement, four fixed seeds (20261113–20261116).
- A 4x4 image-grid round-robin sampler, the same four seeds; select within cells
  without replacement and use remaining eligible cells until k is filled.
- Lowest trace of source-plus-target 3D covariance at the common identity pose,
  deterministic ties by native factor index. This is a reliability control,
  not an assertion that the native selector uses this ranking.
- Standard greedy regularized Max-logDet of whitened pose information, with
  deterministic native-index ties. This is an established-method control.
- All N accepted factors as context, using the same research solver. It has a
  different budget and is not a matched selector or a native-run reproduction.

This gives eleven runs per camera-pair, 1,254 fits, with no seed selection.
Report each stochastic replicate and the four-seed aggregate. If k=N, all subset
arms must return identical native factor sets and identical solver results.

**Common residual:** for optical source/target points X0,X1, fit the
camera-target-to-source rigid motion using r_i=R X1_i+t-X0_i. Compute the
fixed weight S_i=C0_i+C1_i at identity and retain it for every arm/iteration.
This is a declared fixed-covariance 3D registration approximation, not the native
state-dependent covariance R C1 R^T+C0. Convert the serialized native cyclic
basis to optical for both covariances, consistently with `Utility/Point.py`.
Reconstruct points using each frame's own K and its saved metric depth.

Do not learn means/covariances, add a robust kernel or change factors according
to reference errors. Pre-qualification must check finite positive depths,
symmetric positive-definite S, transform direction, analytic Jacobians and a
single solver's convergence behavior. Invalid data produce explicit unavailable
pair slots for all arms; no tuned covariance floors or favorable point pruning.
Solver steps, tolerances and failure rules require a separate execution freeze.

**Information control:** at the common identity pose, use
J_i=[I,-[X1_i]_x]. Set L to the median positive source depth over the full pool,
D=diag(L,L,L,1,1,1), A_i=chol(S_i)^-1 J_i D. With H_all=sum A_i^T A_i and
lambda=1e-6 trace(H_all)/6, greedily maximize
logdet(lambda I+sum_selected A_i^T A_i) until k points. No supplied poses enter
selection. The scale and ridge are engineering conventions to be fixed before
outcomes, not learned priors. If qualification reveals an undefined scale or
information block, stop that configuration rather than select another rule.
The ridge is for selection only; it must not hide solver rank deficiency.

Record selection time separately; these are already-computed matches, so no
claim of total frontend speed improvement. Information eigenvalues/conditioning
are diagnostics, not evidence that actual pose error improved under biased means.

## Evaluation and stopping decision

Lock all subset IDs and fitted poses before supplied reference values are read.
Use existing same-stem RGB reference matrices under the inherited camera-to-first
convention. RGB uses its own relative reference. Thermal reference motion is a
nominal rig conjugation of the RGB same-ID motion, explicitly inheriting the
already documented cross-clock/static-rig assumption. No separately initialized
sensor worlds are equated, offsets fitted or physical synchronization claimed.
This does not upgrade the natural-flow proxy to measured correspondence truth.

For all six camera/recording streams separately, report translation/rotation
mean and RMSE, attempted/successful/failed counts, matched-pair denominators,
selection counts, seeds, convergence status and model-information diagnostics.
No pooled-pixel statistic or aggregate that conceals a failed recording. Compare
the information arm to both uniform and spatial controls at matched budget;
retain the reliability and all-pool context even when they perform better.

A reasonable predeclared continuation bar is at least 5% lower translation mean
and RMSE against both four-seed control summaries, no worse rotation mean/RMSE,
and no coverage loss in every camera/recording stream. Bind the exact aggregation
and bar in the execution freeze before running it. This conservative engineering
bar does not define statistical significance. Pair samples within a recording
are dependent; six exposed streams cannot establish broad generalization.

One qualified frozen comparison, then stop and report. Failure closes this
specific accepted-pool/budget/residual configuration without another budget,
ridge, residual or seed rescue. Success would establish value of a standard
baseline in this restricted setting. A novel method would still require a
specific additional mechanism and later evaluation; an integration gain alone
is not a new feature-selection algorithm.

## Retained scope

The original-paper map still includes temporal continuity, posterior approximation,
sensor transfer, thermal supervision, multispectral representation and active
perception. This is a feasible next technical test, not evidence those openings
are closed. No new subset selection, fitting, training or accuracy scoring ran
in this decision step. The draft is reviewable but is not an execution freeze.
