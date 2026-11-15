# Frozen query-policy learning: no full advancement; retain the night objective gain

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus source-pinned working changes.

## Outcome

The pose-reward query policy does not pass the complete predeclared advancement
bar in any of the three excluded-texture-recording folds. It beats the identically
parameterized match-reward policy on the night family: translation mean/RMSE are
39.19%/43.76% lower and rotation mean/RMSE 40.71%/46.48% lower, with full availability.
It does not beat pose-trained residual weighting there, and its deterministic
top-k policy is worse than both learned controls on day and rain.

Keep that positive night comparison and the full mixed result. The unchanged
native full-pool context has the lowest primary translation and rotation RMSE
in all three families, using more points. This is not evidence that the original
MAC papers fail, that thermal sensing is ineffective or that pose-aware learning
has no value. It closes this exact eight-parameter/64-update adapter recipe as
an advancement candidate under the specified developmental criterion.

## Completed fixed execution

Nine eight-parameter utility adapters complete64Adam updates each: three learned
controls in each of three folds,576optimizer updates total. Each control uses the
same128scheduled scene-loss calls from its240scene training pool; the remaining
training scenes still contribute only unlabeled feature normalization. The two
policy controls sample four subsets per scene. No early stopping, checkpoint
selection, feature change or seed search follows the outcomes.

All three fold model sets complete before evaluation. The240evaluation scenes
have17arms/contexts each: native full pool, four uniform seeds, information,
weight-only, two deterministic policies and four stochastic contexts for each
policy. All4,080evaluation fit attempts are retained and converge. Training uses
6,528weight-control fits and1,536pose-policy fits; match-reward learning uses
correspondence rewards and no training pose fits. All12,144training/evaluation
fits converge, with zero unavailable pools or discarded failures.

The dense model, images, predictions and measurement-covariance outputs remain
frozen in the qualified cache. This run performs no new native neural inference.
Utility weights are task weights, not newly calibrated statistical covariance.
The pose stage is the same explicit optical3D IRLS registration harness, not
the historical robust/disparity-BA optimizer or a full MAC-I² VI system.
All evaluation predictions are globally locked before true-pose scoring.

## Deterministic primary results

Every entry below uses80/80evaluation scenes in that family; own and common
primary support coincide. Translation RMSE is in mm:

|Arm|Day texture family|Night texture family|Rain texture family|
|---|---:|---:|---:|
|Native full pool|1.977|2.197|1.925|
|Uniform 32, seed 0|2.409|2.367|2.092|
|Standard information 32|2.247|2.732|2.195|
|Pose-trained weights 32|2.517|2.310|2.126|
|Match-reward policy, top-k|3.788|5.480|9.533|
|Pose-reward policy, top-k|10.487|3.082|28.291|

Rotation RMSE is in degrees:

|Arm|Day texture family|Night texture family|Rain texture family|
|---|---:|---:|---:|
|Native full pool|0.024604|0.023683|0.022233|
|Uniform 32, seed 0|0.029306|0.026080|0.025566|
|Standard information 32|0.033298|0.034015|0.033591|
|Pose-trained weights 32|0.028367|0.024263|0.024935|
|Match-reward policy, top-k|0.049485|0.082958|0.071268|
|Pose-reward policy, top-k|0.158682|0.044397|0.166800|

Errors are the translation/rotation components of the common true-target-relative
left SE(3) Log, not trajectory ATE. Texture labels denote the original image
atlas families; these are artificial scenes, not real-weather motion trials.
Native full pool has variable point count; the five matched-budget primary arms
use32points. Its accuracy context is not a compute-matched control.

![Primary comparison](../../results/selection-policy-learning/scoring-001/primary-policy-comparison.png)

The night pose-versus-match comparison meets all pairwise criteria. Every fold
still fails the required comparison against the weight-only control, and day/rain
also fail against match reward. Thus the all-fold/two-control gate is false, with
no availability or positive-headroom ambiguity. No pooled-only score overrides it.

## Predeclared stochastic contexts are different from top-k deployment

All four fixed stochastic contexts remain reported, without choosing a winner
or changing the primary deployment rule. Translation RMSE, mm:

|Family|Policy|Primary top-k|Stochastic contexts1 / 2 / 3 / 4|
|---|---|---:|---|
|day|Match reward|3.788|2.035 / 2.244 / 2.199 / 2.155|
|day|Pose reward|10.487|2.197 / 2.254 / 2.228 / 1.933|
|night|Match reward|5.480|2.441 / 2.400 / 2.468 / 2.383|
|night|Pose reward|3.082|2.400 / 2.363 / 2.396 / 2.326|
|rain|Match reward|9.533|2.160 / 2.129 / 2.130 / 2.166|
|rain|Pose reward|28.291|2.386 / 2.227 / 2.110 / 2.413|

![Deployment contexts](../../results/selection-policy-learning/scoring-001/policy-deployment-contexts.png)

The pose-policy contexts are2.110–2.413mm on rain versus28.291mm for deterministic
top-k; on day they are1.933–2.254mm versus10.487mm. All use the same trained theta
and same cached dense fields. They are alternative point-set draws, not new model
training seeds, ensembles or a certified replacement method. Their better behavior
is a concrete deployment-distribution distinction within this fixed experiment;
it does not establish stochastic selection as a generally superior algorithm.
Do not replace the failed primary with the best context after looking at scores.

## Planned conditioning diagnostics and exploratory spatial context

The normalized final information condition is computed from singular values of
the final whitened mean Jacobian, using the common full-pool depth-length scaling.
The4×4image-cell count is an exploratory spatial summary, not an advancement
criterion. Median information condition / occupied cells:

|Family|Uniform 32|Match-reward top-k|Pose-reward top-k|
|---|---:|---:|---:|
|day|210.3 / 11|416.2 / 6|556.0 / 7|
|night|266.8 / 11|4764.5 / 6|509.2 / 6|
|rain|233.9 / 11|714.0 / 6|8882.7 / 5|

On rain, pose top-k concentrates on a median5cells versus11for uniform and has
much poorer local information conditioning. Night's pose policy is better
conditioned than its match-reward policy. These are descriptive associations,
not an identified sole cause of pose bias or a newly validated selection score.
All4080conditioning rows and all17contexts are retained in the separate report;
no primary score or gate is modified by this analysis. No extra pose fit is run.

## Independent verification

The independent audit passes **109,916 checks** on its first execution. It verifies
all1,828frozen source/input pins, model/prediction locks, fold/schedule/normalization
binding, exact subset draws and independent probability scores, all training losses,
finite-difference and leave-one-out gradients, independent NumPyAdam steps and
all576parameter updates. It verifies every12,144fit receipt's independent local
stationarity and information inverse, selected sets, analytic SE(3)errors,
availability denominators, scalar metrics, paired populations and exact gate.
No failed-fit replay is needed because all solves converge.

The auditor source was bound before evaluation predictions and score inspection.
There is no producer, optimizer, scorer or tolerance amendment in this execution.
Earlier corpus/reference/cache qualification amendments remain in their own
retained parent packet; they are not new policy-run retries.

Freeze: `experiments/selection-policy-training-freeze-2026-11-14.json`.
Audit source binding: `experiments/selection-policy-learning-audit-freeze-2026-11-14.json`.
Raw models/journals, predictions and scores:
`results/selection-policy-learning/{run-001,evaluation-001,scoring-001}/`.

## Interpretation and disposition

This is a controlled baseline/adaptation test of a selection-versus-weighting
hypothesis. The pose-versus-match objective is beneficial in one excluded family
but does not produce an across-family improvement over task weighting. Successful
numerical differentiation and optimizer execution do not imply a successful
selection policy or justify promoting it.

Capacity, update count and scene schedule are matched, but training compute is
not: finite-difference weighting uses4.25times as many pose fits as pose-policy
learning, while match rewards need no pose fits. The stochastic gradient and
finite-difference optimizers also provide different gradient information. Thus
this test does not isolate an objective-independent theorem about weighting or
selection, or establish that a more general policy family cannot work.

The three original texture prefixes are development-exposed and adjacent frames
are dependent; held-out geometry/motion profiles and recording exclusion do not
make an untouched real thermal benchmark. This is one fixed optimization seed
per adapter/fold. Synthetic rendering and fixed dense predictions limit transfer
claims; physical covariance calibration and full native VO/MAC-I² fusion were
not evaluated. Policy-gradient/pose-task learning themselves remain prior art.

Close this exact recipe without increasing training length/capacity, changing
features, switching primary deployment to a favorable stochastic seed, adding a
geometry penalty, or changing budgets/solver/labels. The selected broader research
hypothesis and other original-paper gaps remain open. Any next numerical method
needs a separate contribution decision and informative controls, rather than an
automatic rescue of this failed developmental gate.
