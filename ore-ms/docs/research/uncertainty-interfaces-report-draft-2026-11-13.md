# Uncertainty interfaces in thermal and visual estimation

Working technical-report draft,13 November2026; updated15 November2026 UTC.
Branch `ore-ms-orb-development`,baseline `a79658f6341194b0954852226f200313ccdf5408`
plus source-pinned research changes. This is an empirical/modeling report draft,
not a claimed new algorithm or verified publication contribution.

## Abstract

Learned matching uncertainty, estimator sensitivity and pose accuracy are distinct
targets. Separately frozen visual/thermal studies examine their connection using
native learned frontends and explicitly scoped estimation harnesses. Paired
frontend replay separates query selection from dense-measurement changes. An
exact-pose extension retains a positive scene with25.08%lower translation RMSE
despite21.76×higher translation variance, alongside five opposite outcomes.
Fixed-query reanalysis of768saved draws shows that empirical cross-match terms
increase propagated normalized variance6.28–12.05× over retaining within-match
blocks alone; a frozen linear response closely matches actual centered pose
variation. This is not a native uncertainty underestimation factor. A five-
parameter shared-image-mode predictor adds modest covariance-score value over
scalar/independent controls in some recording-excluded development cases, while
transfer and containment remain mixed. An oracle-only diagnostic separates its
directional shape from scale mismatch without replacing any prediction.
Earlier selection-learning, continuity and sensor-time comparisons retain both
successful baselines and mixed outcomes. The evidence supports a controlled
empirical account of uncertainty interfaces and specific remaining questions.
No generally superior algorithm, physically calibrated posterior or field-wide
publication novelty is established.

## 1. Question and scope

The question is not whether learned uncertainty or thermal sensing works in
general. Published systems report benefits under their evaluated conditions;
our extensions are different configurations and are not exact reproductions.
We ask which assumptions connect a predicted uncertainty signal to the residual,
target or selection decision where it is used.

The original-paper openings are broader than this report: MAC-VO exposes temporal
estimation, covariance structure and selection questions; MACThermal identifies
supervision, transfer, continuity and multispectral extensions; MAC-I² includes
persistent estimation and posterior-model questions. The
[paper-first map](original-paper-gap-map-2026-11-13.md) is the source index.
We examine several interfaces with available offline evidence, without declaring
the others closed or the examined questions field-wide novelties.

These studies use different checkpoints, resolutions, populations and objectives.
They are separate vignettes, not one matched end-to-end ablation or a causal chain.

## 2. Residual conditioning and temporal dependence

For an anchored ray and optimized source depth, the source UV residual and its
Jacobian vanish identically. Source disparity remains one depth factor. If the
source is declared exact, the target covariance excludes propagated source noise;
if it remains uncertain, the prediction's source-ray Jacobian and cross terms
must also be represented. In either model, independent flow increments can give
correlated accumulated observations. The
[full derivation](temporal-residual-derivation-2026-11-13.md) states these
assumptions and the implications for whitening pose/depth Jacobians.

A four-arm comparison separates original marginal weights, conditioned marginals,
conditioned joint temporal weights and a uniform4x covariance control. All480
attempts are retained over60five-frame windows and two camera modes. There are
409gradient stops,50budget completions and21line-search failures. A budget
completion is not a stationary optimum. The audit verifies final costs, historical
control parity and1,836delivered motion intervals.

| Day RGB arm | Delivered intervals | Translation mean m | Rotation mean deg |
|---|---:|---:|---:|
| Original |80/80|0.042382|0.049162|
| Conditioned marginals |80/80|0.040348|0.033898|

These changes isolate the declared conditioning intervention from the additional
temporal-block intervention; they do not prove that exact-source conditioning is
physically correct. The full joint temporal arm misses the continuation gates on
all3recordings. The uniform-scale control gives identical point estimates. Known-
covariance synthetic GLS success therefore does not validate the approximate
real-data covariance. See the [full comparison](temporal-window-comparison-findings-2026-11-13.md)
for matched outcomes, failed windows and positive/negative cases.

## 3. Correspondence targets and appearance consistency

The released U²Flow Sintel model was qualified with strict state loading and
unchanged author inference code in a declared CPUFP32 runtime. It was not retrained.
On60thermal source images, four fixed identity/shift/gamma cases give240predictions.
All9,655,200eligible prediction pixels have endpoint error below1pixel in this
controlled known-warp assay. The target diagnostic for errors above1pixel has
no qualifying errors: its conditional fraction is undefined. This is positive
baseline evidence; the assay does not demonstrate a matching failure.

Natural motion was evaluated separately on57adjacent pairs, clean and gamma1.6,
using projected depth and nominal same-ID supplied pose geometry. Three support
tiers were retained. The strictest tier yields:

| Recording | Scored source points | Valid pairs | Clean geometry disagreement mean px | Zero-flow geometry disagreement mean px |
|---|---:|---:|---:|---:|
| Day |16852|19/19|1.659041|2.460240|
| Night |101736|19/19|0.038742|0.036645|
| Rain |3290|19/19|6.973755|8.609658|

The baseline improves the day/rain zero-flow context. Mean clean/gamma prediction
change on the same strict-tier support is0.043681px day and0.296827px rain.
Existing alpha ranks geometry disagreement more strongly than that consistency
signal in those two records. Visibility, static-point identity and timing remain
unverified: these are consistent **proxy disagreements**, not demonstrated true
matching errors. Source-depth support and geometric plausibility do not resolve
those alternatives. The controlled assay and natural diagnostic cannot be pooled
as one flow-accuracy benchmark.

See [controlled results](u2flow-thermal-warp-findings-2026-11-13.md) and
[natural results](natural-flow-geometry-findings-2026-11-13.md). Generic
augmentation consistency and self-supervised uncertainty are established by the
reviewed ARFlow/Sun-RAFT/U²Flow sources; this report does not introduce them.

## 4. Reliability and complementary geometric constraints

The native selector already includes NMS and reliability/geometric filtering.
The recovered graphs retain accepted matches, not full rejected candidate fields.
We fix those means/covariances and select k=min(32,N) from each of114camera-local
pair pools: uniform/spatial controls with four seeds each, minimum covariance
trace, standard greedy information selection and all-pool context. All arms use
the same fixed source-plus-target 3D covariance registration solver.

All1,254attempts are locked before references:1,235gradient convergences and19
retained line-search failures. The one23-factor capped pool produces11identical
fits. Independent verification reconstructs all114greedy information sets by the
matrix determinant lemma and all1,254fit receipts/scores.

Rain RGB meets the full criterion on19/19pairs:

| Selector | Translation mean m | Rotation mean deg |
|---|---:|---:|
| Four-seed uniform summary |2.188126|0.151721|
| Four-seed spatial summary |1.130166|0.105957|
| Standard information |0.975898|0.077386|

Information selection improves translation mean/RMSE by13.65%/13.84% against
spatial and55.40%/46.25% against uniform, with lower rotation and no coverage loss.
The other5streams do not meet the complete criterion: night RGB beats uniform
but not spatial, day rotation worsens, thermal results are mixed and some streams
lose coverage. The [all-stream report](feature-selection-findings-2026-11-13.md)
retains every seed and matched-population denominator.

This is an established selection baseline transferred to a restricted stereo3D
residual. [Good Feature Selection](https://arxiv.org/pdf/1905.07807) already
provides Max-logDet selection and bias/variance analysis. This replay establishes
neither a new algorithm, full-image selector replacement nor full-trajectory VO
benefit. Better modeled information alone is not an accuracy guarantee.

## 5. Selection sensitivity and the uncertainty target

The [paired native frontend assessment](paired-frontend-assessment-findings-2026-11-14.md)
holds the checkpoint/input draws/solver fixed and compares original source-query
IDs with regenerated NoDepth selection. On three exposed thermal pairs with128
paired draws each, baseline-information-normalized pose variance rises6.34× day,
2.96× night and2.04× rain. All780fits converge;13,883audit checks pass. This uses
native MACT predictions and native selection/covariance bodies with an explicit
IRLS pose harness, not full historical BA or MAC-I² VI fusion.

The [crossed replay](selection-measurement-replay-findings-2026-11-14.md) applies
the saved regenerated query sets to unchanged baseline dense fields. All390added
fits converge and7,575independent checks pass. Selection-only mean translation
shifts are2.115/1.698/208.978mm, versus measurement-only0.665/1.313/256.332mm.
Exact covariance decomposition retains correlated selection/measurement/interaction
cross terms. Query identity, count and downstream admission are not isolated from
one another. None of these baseline-relative shifts is a true-pose error.

This provides a concrete mechanism assessment of the estimator/uncertainty target.
Generic rematching effects, repeatable keypoints and geometry-aware selection are
prior work; no new algorithm is established. A [six-scene known-pose assessment](known-pose-estimator-findings-2026-11-14.md)
now separates synthetic pose error from spread: all1,560fits converge and31,999
independent checks pass. On day-two-depths, regenerated queries reduce translation
RMSE25.08% (2.648→1.984mm) and rotation RMSE8.44%, despite21.76×translation and
11.43×rotation variance. The other five scenes worsen both RMSEs. This positive
accuracy case alongside mixed outcomes demonstrates that lower sensitivity is
not an accuracy ranking. These artificial atlases/planes and half-LSB draws
are an exact-reference assessment, not physical thermal calibration.

## 6. Interpretation and limitations

Each study supports a specific boundary:

1. Conditioning and temporal dependence change different parts of a residual
   covariance; they require separate controls.
2. Predictive consistency, imposed-warp error and supplied-geometry disagreement
   are different targets; one does not certify the others.
3. Reliable accepted matches may offer different geometric value under a point
   budget, but a standard information selector's benefit does not transfer
   uniformly across these streams.
4. A data-dependent query policy can change estimator spread even with dense
   fields held fixed; robustness and accuracy require distinct targets.

The findings do not identify one physical cause for all regressions. Numerical
qualification/audits check implementation integrity, not independent ground truth,
calibration, novelty or generalization. The inputs are exposed development
prefixes; pair observations within a recording are dependent. RGB-to-thermal
reference conjugation assumes nominal same-ID rig motion; clocks/mounting are
not fitted or newly verified. Native full-system runtime and publisher benchmarks
were not reproduced. Conservative per-stream continuation bars are engineering
rules, not significance tests or claims that broader research gaps are closed.

## 7. Present contribution and disposition

The supported deliverable is a scoped technical report with explicit residual
models, comparison controls, positive baseline cases and reproducible limits.
Its publication novelty and suitability as a thesis contribution need a separate
assessment. No new superior uncertainty model or fusion algorithm is established.
The exact tested recipes remain closed without retuning. Other original-paper
gaps remain open; any next method needs a specific prior-art distinction and
an informative, bounded comparison.

The [current contribution decision](contribution-synthesis-decision-2026-11-14.md)
incorporates selection, continuity, recovery and sensor-time evidence and supersedes
older next-step recommendations. The [historical contribution decision](shortlist-contribution-decision-2026-11-13.md)
retains the original three-study recommendation; the
[exact-target findings](known-pose-estimator-findings-2026-11-14.md) document the completed accuracy comparison.
The original three-study source ledger remains in
`results/shortlist-contribution-decision/evidence-ledger.json`; subsequent paired,
synthetic-target and utility-learning studies link their own freezes, predictions
and audits. The [latest learning findings](selection-policy-learning-findings-2026-11-14.md)
state the current completed recipe disposition. No reserve-data exposure or
physical-transfer claim is introduced by this synthesis.


## 8. Method hypothesis and prepared controls

The [source/code-backed method decision](selection-task-method-decision-2026-11-14.md)
motivated a separate pose-reward query policy with frozen dense predictions and
covariance outputs, compared against match-reward query selection, pose-trained
weighting and standard sampling/information controls. This is a baseline/adaptation
hypothesis, not a new policy-gradient principle. NG-RANSAC, DISK, DNLS and
Deeper-DICE already cover the generic task-selection, weighting and correction
ideas. The six earlier fixed-motion diagnostic scenes and their noise draws are
not independent transfer data. Frozen covariance outputs also do not establish
calibration after a policy changes the selected population.

The [corpus/adapter contract](selection-policy-contract-findings-2026-11-14.md)
bound 600 scenes with disjoint training/evaluation motion and geometry profiles,
three recording-excluded developmental folds and eight-parameter matched controls.
It passed 230 mechanics and 693 manifest/split checks. The subsequent
[corpus/cache preparation](selection-corpus-cache-findings-2026-11-14.md)
materialized all 600 scenes: 22,934 reference checks, 12,701 native-cache/label/
training-only-normalization checks and 237 derivative checks pass. All 198 native
weight-probe solves converge. Reference-boundary and auditor-precision amendments
retain their original attempts. Those readiness stages are completed, and the
source/input-bound learning experiment is now completed below. Other original-
paper directions remain available; this candidate does not close them.

## 9. Completed task-policy baseline/adaptation experiment

The [frozen learning comparison](selection-policy-learning-findings-2026-11-14.md)
completes nine eight-parameter adapters at64updates each. All12,144training/
evaluation fits converge;109,916independent checks reconstruct every update,
fit and score. The full advancement criterion fails in allthreefolds. Night's
pose-reward policy improves translationRMSE43.76% and rotationRMSE46.48% over
its same-capacity match-reward policy, but not the task-weight control. Native
fullpool is the best primaryaccuracy context at a larger point budget. Allfour
stochastic contexts are reported separately; their better behavior than hard
topkday/rain does not authorize choosing a replacement after seeing results.
The exact low-capacity recipe is closed without retuning. This does not close
pose-aware learning, the original-paper selection gap or other research directions.


## 10. Continuity and connection baselines

The [continuity study](continuity-sequence-findings-2026-11-14.md) uses six
artificial ten-frame sequences with frames 3–5 withheld. All 72 pair fits converge.
Retained cross-gap and supplied correct-anchor edges outperform adjacent chains
on these cases. This does not imply missing frames help: edge count, geometry
and spacing differ. A [matched-count comparison](matched-reference-paths-findings-2026-11-14.md)
then fixes endpoints and two-edge paths, with translation favoring one reference
in five of six cases and rotation in three. Reference timing and geometry still
change together. Neither experiment implements a new retrieval or map-merging
system.

The [connection-validity study](anchor-validity-findings-2026-11-14.md) compares
12 positive and 24 negative constructed-world candidates. Native/geometry controls
retain all positives and accept no negatives. All 48 negative directions abstain
before fitting because the selector supplies no queries. Conditional uncertainty
ranking is undefined in that surviving population. Adding a cycle gate removes
one accurate positive and rejects no additional negative. This successful
baseline supplies no incremental confidence-head target in the tested assay.

## 11. Sensor integration and timestamp-specific pose error

The [qualified adapter](sensor-image-adapter-findings-2026-11-14.md) provides
nine cases: three texture proxies crossed with stationary, constant-body-twist
and accelerating motion. Four image arms share identical reference poses. Its
response/readout parameters describe one virtual profile, not measured MS2
physics; prepared thermal textures are radiance proxies. No single exact
observed-pixel flow label is asserted for an integrated image.

The [estimator comparison](sensor-estimator-findings-2026-11-14.md) completes
36 dense batches and 72 converged fits, with 1,841 independent checks. Stationary
inputs and outputs repeat bitwise across arms. For six moving cases, memory
reduces median matching covariance in six while worsening translation and
rotation error in four. Combined inputs reduce the readout in six and worsen
each pose endpoint in three. Counts persist with fixed sharp-seeded query IDs.
Day-twist combined translation improves 4.238→2.208 mm; rain-accelerating memory
worsens 2.338→4.382 mm. The corresponding fixed-ID values are 1.394 and 3.575 mm.

This rules out query reselection as the sole explanation for the observed
responses. It does not identify endpoint means, depth or covariance as the sole
cause: all three change together. Median matching covariance and the pose
inverse-information proxies are separate readouts; the latter do not uniformly
fall. There is no physical noise ensemble or integrated-pixel flow truth, hence
no coverage or flow-miscalibration conclusion.

[MBA-VO](https://arxiv.org/html/2103.13684v1) already models image exposure in
tracking. [CUAHN-VIO](https://arxiv.org/html/2208.13935v1) already discusses a
blur-related uncertainty/mean-accuracy mismatch affecting filter weighting,
although its uncertainty response differs from this thermal experiment.
Together with the existing TRNeRF/TRGS-SLAM review, these sources limit generic
sensor-aware or blur/uncertainty novelty claims. The current contribution is the
specific controlled evidence and explicit target distinctions; publication
novelty and a superior method remain unestablished. See the current decision for
the bounded real-sensor evidence qualification selected next and the other
original-paper gaps that remain open.


## 12. Matched acquisition histories at equal progress and deadline

The [matched-motion assay](active-motion-estimator-findings-2026-11-15.md) connects
MACThermal's active-perception opening to acquisition timing. Three prescribed
C2speed schedules share a4cm path,0.2sdeadline and endpoint states. The reversed
schedules also tie full-horizon squared speed/image-velocity energy under the
planar lateral-motion fixture. Recent-box and exponential history controls both
distinguish them; generic blur-aware planning remains established prior work.

Nine cases produce72views,18native dense batches and36converged optical IRLS
pose fits. Image and estimator audits pass2309and955checks respectively. Sharp
images/dense outputs/fits repeat exactly across schedules. Memory-action
translation-error ranges are0.015–0.079mm with native reselection and0.002–0.042mm
with fixed IDs. The corresponding sharp baseline errors are1.668–5.344mm.
Native nightmemory improves both pose endpoints relative to sharp; dayworsens,
and rain's endpoint/query-control behavior is mixed. Less recent motion does
not consistently improve both errors. This is bounded evidence of action timing
changing estimator outputs, not a new planning method or demonstrated useful
scheduling policy. The exact virtual profile is closed without severity tuning;
physical transfer and the other original-paper gaps remain open.


## 13. Cross-match dependence under fixed query identities

The [cross-match covariance reanalysis](cross-match-covariance-findings-2026-11-15.md)
uses all768saved fixed-query draws from the six known-pose scenes. Baseline
frozen geometry/weights map each residual change into a six-dimensional pose
influence. Keeping empirical cross-match covariance gives6.28–12.05times the
normalized variance of retaining within-match blocks alone. The same baseline
normal-equation inverse appears in both calculations, so this is distinct from
shared pose/landmark coupling or query reselection.

The full linear response's normalized trace is within0.44% of actual centered
pose variation; paired centered remainder RMS is0.54–2.53% of actual RMS. An
independent dense residual-covariance/matrix-log audit passes3270checks. No
estimator is rerun and no correction is fitted. The denominator is empirical
block covariance, not native predicted covariance: this is no claim of a6–12×
underestimated native readout. The half-LSB ensemble is not physical sensor noise,
and centered dispersion remains separate from systematic pose error.

Censi2007 sensitivity analysis and Bonnabel's correlated registration-error model
already cover the generic covariance propagation. The local increment is direct
measurement of consequential cross-match dependence in saved learned frontend
outputs, with a checked linear approximation. A deployable predictive dependence
model, calibrated uncertainty and improved odometry remain unestablished.


## 14. Predictive covariance transfer with shared image-error modes

The [recording-excluded development comparison](covariance-transfer-findings-2026-11-15.md)
fits four propagated shared image-error modes plus a native covariance scale,
with matched five-parameter independent modes, a train scalar and pooled full
shape as controls. Three folds exclude entire texture recordings(two geometries
each); all scenes are previously exposed. All six fits converge and all30
predictive covariance matrices lock before evaluation scoring.117mechanics and
2108independent audit checks pass; no image/model/pose rerun occurs.

Common modes improve likelihood over scalar and independent modes in five of
six scenes and over all three fitted controls in two. Day/rain recording averages
are positive; night is negative. A test-scalar oracle scores better on all three
recording averages, and common-mode95%baseline-relative containment is60.16% on
day and99.22% on night/rain. These are perturbation-distribution scores about a
fixed-zero-mean unperturbed estimate, not physical true-pose coverage. The limited
transfer increment does not validate general covariance correction or improve
any pose estimate. Close this recipe with both its positive cases and its limits;
generic NLL/correlated covariance prediction already has prior coverage.


## 15. Separate covariance shape from scale without a new predictor

The [oracle-only shape/scale diagnostic](covariance-shape-scale-findings-2026-11-15.md)
profiles all30fixed predictions along one scalar using evaluation errors. Native
and scalar-native shapes tie, and every old NLL gap decomposes exactly into a
profile-shape gap and a scale-penalty gap. Common shape helps day/rain versus
native but hurts night. Against pooled shape, the old day win is a scale advantage
despite worse shape; rain retains a small directional advantage. Thus scale is
substantial but is not the only remaining issue.42mechanics and1192independent
checks pass. No trained model, predictive covariance, containment score or pose
is changed; oracle profiling is not an available calibration method. DNLS already
states the distinction between pose benefit and absolute-scale calibration.
