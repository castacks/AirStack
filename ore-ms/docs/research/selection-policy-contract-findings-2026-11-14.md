# Query-policy corpus design and mechanics qualification

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus pinned working sources.

## Completed deliverable

The [selected hypothesis](selection-task-method-decision-2026-11-14.md) now has
an exact planned corpus/split, adapter and training/control design. Its ordered
subset-policy gradient and reference-visibility mechanics pass qualification.
No corpus has been rendered, inferred or used to fit a utility adapter in this
step. Ten small synthetic registration fits supply mechanics-test losses only.

**693 independent manifest/split checks and 230 mechanics checks pass.** These
qualify the specified interfaces, not a policy's performance or a new algorithm.

## Planned corpus and exclusion

All60existing raw thermal source images are SHA-bound: frames000000–000019 from
each day/night/rain prefix. These prefixes and images are development-exposed.
They are texture sources, not sixty independent recordings or a fresh test set.

|Profile partition|Textures|Depth/boundary geometries|Motions|Planned scenes|
|---|---:|---:|---:|---:|
|Training profiles|60|2|3|360|
|Evaluation profiles|60|2|2|240|
|Total|60|4|5, partitioned|600|

Geometries are front/background planes with a foreground half-plane in world XY:

|ID|Partition|Near/far depths m|Unit XY boundary normal|Offset m|
|---|---|---|---|---:|
|train-g0|training|3 / 6|[1,0]|−0.20|
|train-g1|training|5 / 10|[0,1]|+0.30|
|eval-g0|evaluation|4 / 9|[1,1]/sqrt(2)|+0.10|
|eval-g1|evaluation|3.5 / 7|[1,−1]/sqrt(2)|−0.15|

A near-plane point exists when normal·[X,Y]≤offset; the far plane is background.
Each motion is a full optical rotation-vector exponential, not Euler angles:

|ID|Partition|Rotation vector deg|Translation m|
|---|---|---|---|
|train-m0|training|[0.30,−0.40,0.20]|[0.04,−0.02,0.08]|
|train-m1|training|[−0.40,0.20,−0.30]|[−0.03,0.01,0.06]|
|train-m2|training|[0.20,0.40,−0.20]|[0.02,0.03,−0.05]|
|eval-m0|evaluation|[−0.25,−0.35,0.40]|[−0.02,−0.03,0.09]|
|eval-m1|evaluation|[0.45,−0.15,−0.30]|[0.05,0.02,−0.04]|

Original K, baseline,256×640 inputs and native preparation remain fixed. Three
folds exclude one complete texture recording. Each fold has240training scenes
(40textures×6training profiles) and80evaluation scenes (20textures×4evaluation
profiles), with disjoint recording, texture, motion and geometry IDs between its
training/evaluation populations. The600scene IDs are reconstructed independently
as the complete permitted Cartesian product. No new profile matches the previous
six fixed-motion diagnostic scenes. The old examples remain exposed diagnostics.

This is procedural separation in an artificial developmental benchmark. It does
not establish natural transfer, unseen sensor generalization or physical thermal
noise. Adjacent frames remain grouped by their original recording.

## Fixed adapter and candidate contract

The candidate pool is the unchanged native NoDepth requested200 output followed
by the original range/finite-positive mean/covariance admission. Canonical source
(v,u) row order fixes ties. Every matched arm uses k=min(32,N); invalid pools
remain unavailable. This selects within a native accepted/capped pool, not among
all dense pixels or candidates already discarded by NMS.

Eight features are fixed before any new model predictions: normalized source
ray u/v, log source/target depth, log source/target3D covariance trace, log summed
2D flow variance, and log1p temporal-flow magnitude. Units are1m,1m² and1px where
needed. Features contain no pose labels, visibility mask, rendering profile,
recording ID, frame ID or uncertainty-ensemble result. Mean/population-standard-
deviation normalization uses only each fold's training eligible candidates;
exactly constant scales become1. No covariance floor or label-based pruning is
added by this adapter.

All learned controls use a bias-free linear head with eight FP64 parameters,
initialized at zero. Query policies use ordered Plackett–Luce sampling without
replacement during training; primary deployment is stable top32 with canonical
ties. Sample order is retained for log probability, while selected rows are sorted
before solving so floating-point row order is not a learned action. Four seeded
stochastic evaluations are additional context, never a best-seed selection.

The weight-only control keeps the first uniform-seed subset and uses positive
normalized exponential utilities with mean1 over that selected subset. Dividing
both endpoint covariance arrays by those utilities implements task weighting in
the objective; it does not estimate new statistical measurement variances.
Native dense predictions, covariance outputs, calibration and pose solver remain
unchanged. Frozen covariance outputs do not certify calibration after selection
changes the assessed population.

## Bounded training and comparison design

Six arms are bound: native variable-count context; uniform32; standard
information32; pose-task weight-only32; match-reward policy32; pose-reward policy32.
The information control retains the existing Max-logDet selector's small ridge
only for selection; it is never inserted into the pose solver.

Each learned control has64Adam updates,2scenes/update, learning rate0.01,
betas[0.9,0.999], optimizer epsilon1e−8 and gradient norm clip1. Final parameters
only, no early stopping or evaluation-guided checkpoint choice. Matched learned
controls share the same deterministic minibatch scene order. Policies use four
ordered subsets/scene and an independent other-sample leave-one-out loss
baseline. Weight-only central finite differences use parameter step1e−3 and
retain all perturbed solves; that native-cache derivative interface is not yet
qualified by this step.

For successful solves, pose loss is1−exp(−R), where R is squared translation Log
error normalized by0.01m plus squared rotation Log error normalized by0.1deg in
radians. Failed solves receive loss1, and are not replaced. The bounded form
avoids rewarding failure below a successful task loss. These are declared task
scales, not estimated sensor variances or tuned physical tolerances.

The match-reward control uses forward pixel reprojection quality with1px scale.
Known occluded/outside-canvas/atlas queries get zero match reward; genuine unknown
support must be represented separately. Geometry/visibility labels never prune
the candidate pool, enter features or act as an inference-time oracle.

All80evaluation scenes/fold retain translation/rotation mean/RMSE, availability,
conditioning and own/common-support populations. Pose-reward advancement requires
at least5%lower translation mean and RMSE against both trained match-reward and
weight-only controls in every excluded recording, with no rotation/availability
regression. This is a developmental mechanism gate, not significance evidence or
system promotion. A full system claim additionally needs native/information
accuracy–availability–cost comparisons. Actual inference and training fit counts
must be reported; equal update counts do not imply equal compute.

## Qualified mechanics and independent checks

|Check family|Evidence|
|---|---|
|Ordered subset probabilities|Exact normalization and zero expected score for12/24/60ordered possibilities|
|Expected-loss gradient|Independent scalar probability products/central differences and PyTorch autograd agree; maximum finite-difference discrepancy1.81e−11|
|Task-dependent gradient|Ten converged unchanged-IRLS synthetic3point fits form a fixed loss table over five candidates; all60ordered3point samples enumerated|
|Variance-reduction baseline|Exact two-draw leave-one-out expectation equals the unbaselined gradient; constant reward has zero gradient|
|Sampler/interface|4,096fixed-seed draws match declared ordered probabilities; no duplicates, seed repeat, boundary budgets, stable ties and positive mean1utilities pass|
|Generalized scene rays|80rays across all20geometry/motion combinations match independent scalar closest-plane intersections|
|Prior geometry preservation|6,528world-point probes over12previously audited two-depth camera views match within1e−10|
|Visibility controls|In-image occlusion, disocclusion, behind-camera, out-of-field and closed foreground-boundary cases pass|
|Manifest/splits|All60source hashes and600scene IDs checked; each fold independently reconstructs240training/80evaluation IDs and exclusion axes|

A projected background point can be inside the target image yet hidden by the
foreground. Visibility therefore compares it with the nearest permitted surface
on the target ray. This prevents the old stereo-identity qualification from being
mistaken for a complete dense correspondence reference. Qualification here is on
fixed analytic cases; all materialized pixel masks still need a separate audit.

The policy-gradient mathematics is established prior art. These checks qualify a
local subset adapter and geometry interface; no NG-RANSAC/DISK implementation or
benchmark is reproduced, and no utility head is trained.

## Readiness and next work

Status: **DESIGN_BOUND_AND_MECHANICS_PASS; CORPUS_AND_NATIVE_CACHE_PENDING**.

The immutable mechanics freeze is
`experiments/selection-policy-contract-2026-11-14.json`; the linked execution
clarifications are in
`experiments/selection-policy-training-design-2026-11-14.json`.
Raw receipts: `results/selection-policy-contract/run-001/`.

Next materialize exactly the600planned scenes and independently audit generalized
rendering/support/visibility; then capture the unchanged native dense fields,
candidate packets and training-only feature statistics. Qualify the weight-only
finite-difference interface on that cache and bind a separate training execution
freeze. No utility training is ready merely because the finite gradient tests
pass. No geometry, features, losses, budget or optimizer search follows from
materialization or model outcomes. This step does not close the broader original-
paper directions or establish a new method's benefit.
