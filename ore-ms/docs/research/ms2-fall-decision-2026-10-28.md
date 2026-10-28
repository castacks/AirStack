# RGB–thermal uncertainty: Fall research assessment through EXP070

2026-10-28, America/New_York. Starting commit `58830510d`, branch `ore_ms`.
This is an assessment of completed work, not an experiment, a new method,
or an advisor-approved change of scope.

The current results support reporting a bounded within-camera ranking finding
and a negative cross-camera transfer finding. They do not support deploying
adaptive RGB–thermal selection or fusion. Constant uncertainty scaling and
additive disparity bias plus scaling have now been tested; neither resolves
the observed selection failures. Keep the original inference contract and
defaults, and preserve the final two unexamined captures.

## Answer to the original research question

The project asks whether RGB and thermal uncertainty predicts their relative
geometric reliability. The answer depends on which part of that question is
being tested:

| Question | Evidence-supported assessment | Boundary |
|---|---|---|
| Does uncertainty contain useful within-camera ranking information? | Yes on the tested supplied-reference windows, including controlled mean-distance selection comparisons. | Reference-supported, dependent samples; central-pixel counterexamples remain. |
| Are native RGB/thermal uncertainty magnitudes directly comparable? | Not established. | Different pixel coordinates, scales and measurement targets; propagation supplies units, not calibration. |
| Do the tested scores select the better camera consistently across captures? | No. Prospective and retrospective counterexamples remain. | A failed method family does not disprove every possible reliability representation. |
| Is a global scale or additive bias/scale correction sufficient? | Not supported by EXP060/070. | Tests cover the declared constant controls, not all conditional calibration models. |
| Is uncertainty-based VO/fusion improvement established? | No. | These measurement-level diagnostics do not measure downstream pose benefit. |

The working Fall assessment is that raw comparability is unestablished and
the tested simple calibration approaches are insufficient. Refinement of the
reliability representation/evaluation is justified as a research question;
selection integration and a large training campaign are not justified by these
results. This follows the approved characterization/fallback scope in the
[research handoff](../../MS_RESEARCH_HANDOFF.md), rather than declaring a new
advisor decision.

## Evidence chain and what each diagnostic resolves

| Evidence | Finding | Consequence for the next method |
|---|---|---|
| [EXP046–054 synthesis](ms2-findings-2026-10-26.md) | Controlled within-camera mean-L2 selective risk improves over matched random, with group median reductions about 9–37%. | Retain this bounded positive result; it does not supply a cross-camera scale. |
| [EXP056–059](../../experiments/EXP-059-ms2-score-failure-diagnosis.md) | Spatial uncertainty benefits vary by recording; camera-median normalization and reference-depth Jacobian diagnostics do not universally repair selection. | Neither a camera-scale-only explanation nor a reference-depth substitution is an established solution. |
| [EXP062–064 transfer synthesis](ms2-transfer-findings-2026-10-27.md) | Prediction-geometry/uncertainty fits show development gains but fail the declared consistent prospective improvement test. | Repeating the same features or target-loss fitting is not a new, untested method. |
| [EXP065](../../experiments/EXP-065-ms2-signed-residuals.md) | Thermal has larger empirical spatial-mean components; global centering still leaves RGB lower squared error in every frame of both tested night windows. | Signed mean structure matters, but a global empirical mean does not explain away the gap or identify physical sensor bias. |
| [EXP066](../../experiments/EXP-066-ms2-disparity-diagnostics.md) | Tiny positive disparities dominate one capture's depth tail; raw variance ranking changes with error domain and conditioning. | Define the deployment loss explicitly; disparity calibration and metric risk need separate evaluation. |
| [EXP067](../../experiments/EXP-067-ms2-distribution-risk.md) | Moment-matched lognormal/Gamma risks differ and do not yield a consistent replacement; Gamma includes infinite-risk cases. | Mean, variance and positivity do not determine inverse-depth risk. Do not choose a distribution from favorable observed outcomes. |
| [EXP068](../../experiments/EXP-068-ms2-training-semantics.md) | Available covariance training code and the inference parameterization conflict; exact checkpoint training history is unverified. | Preserve the measured interface. A source inconsistency is not evidence of the cause of these failures. |
| [EXP069](../../experiments/EXP-069-ms2-disparity-coverage.md) | Coverage varies by capture and group; near-95% marginal coverage coexists with signed/asymmetric residuals. | Report signed tails, conditional coverage and interval positivity alongside marginal coverage. |
| [EXP070](../../experiments/EXP-070-ms2-bias-scale-calibration.md) | Constant bias/scale improves some disparity metrics without consistent camera-selection gains. | Close the constant-calibration branch without adopting a correction or changing point predictions. |

These are linked stages using overlapping recordings and arrays. They are not
nine independent replications. In particular, the 140 frames/240003 pairs in
EXP065 include the four-capture 80-frame/106562-pair population reused by
EXP066/067/069/070. Adding those counts would inflate the evidence.

## Quantitative anchors from the saved reports

The following EXP070 entries are medians of 20 per-frame paired mean-3D-distance
risk ratios at the midpoint supplied-reference target. Values below one favor
the named numerator. Each fit excludes the evaluated recording; all four
recordings were already inspected, so this is retrospective evidence.

| Excluded recording | Scale / original | Bias+scale / original | Bias+scale / scale | Bias+scale better / tied / worse than scale |
|---|---:|---:|---:|---|
| exp063_rain | 1.23733 | 1.15677 | 0.92894 | 20 / 0 / 0 |
| exp064_day | 1.07390 | 1.11402 | 1.03281 | 0 / 0 / 20 |
| exp064_night | 1.07004 | 1.08639 | 1.01471 | 1 / 0 / 19 |
| exp064_rainy | 1.09695 | 1.11635 | 1.00779 | 5 / 0 / 15 |

Bias plus scale improves over scale only in the first rain recording, but
worsens the primary loss in the other three. Both fitted families worsen the
original selector on all four primary median ratios. That statement is about
mean distance: day scale-only squared-error risk has a small improvement over
the original (median ratio 0.98978). Do not expand the headline to every loss,
frame or comparator. The original selector is also not a reliable default
camera winner; its own failures against fixed-camera controls remain in the
earlier experiments.

Three diagnostic anchors show why a single favorable metric is inadequate:

- In EXP066 day RGB, a median 0.6836% of points with predicted/reference
  disparity below 0.1 contributes 99.0463% of axial-depth squared error. These
  points were retained. This is finite-error accounting, not proof of bad
  labels or grounds for retrospective clipping.
- In EXP069 newer-rain thermal, median 95% interval coverage is 95.11% and mean
  standardized squared residual is about 1.025, while the signed standardized
  mean is about 0.714 and no frame has a negative 1.96-sigma tail. Those
  marginal matches do not establish a centered or conditionally calibrated law.
- In EXP070, bias plus scale improves thermal native-disparity NLL versus scale
  only in all four recording-level medians. Nevertheless its camera-selection
  loss worsens in three of the four. In newer-night thermal its median 95%
  lower interval endpoint is nonpositive at 74.76% of evaluated points;
  interval inversion cannot silently produce a finite positive depth bound.

## Keep the measurement targets separate

The disparity diagnostics use each camera's own supplied endpoint, with
`d_ref = fx*B/Z_ref`. Camera selection instead compares the original predicted
3D points against a nominal shared-frame midpoint reference, with both endpoint
anchors as sensitivities. Better likelihood for the first target is not a
guarantee of lower selection loss for the second.

EXP070 interval recentering describes an implied reference-disparity
distribution; it does not alter the deployed point estimate. Its selection
score therefore retains squared bias. A study that also corrects disparity
would change both prediction and risk and would be a different, undeclared
method. No such correction has been evaluated here.

Supplied depth, calibration and correspondence products share a production
chain. Arithmetic checks and source hashes validate implementation and
preservation. They do not certify physical common points, visibility, timing,
absolute geometric accuracy or checkpoint training-set exclusion. Claims about
sensor causes, independent physical error and unseen-by-training performance
remain outside the evidence.

## Research decision and next work

Report the current result now. No further nearby-frame replication, unchanged
capture evaluation, global scaling sweep, distribution choice or automatic
increase in model capacity is necessary to establish this bounded result.
The Fall deliverable can include the reproducible baseline, measurement/error
definitions, controlled ranking evidence, failed transfer tests and the
diagnostic chain above. A negative comparability result is an outcome of the
approved research question; it is not a reason to manufacture a fusion gain.

Before executing a new method, its proposal needs a concrete difference from
EXP062–064 and a falsifiable reason that difference could address the observed
failure. A proposal to fit geometry and uncertainty directly to relative error
does not meet that requirement by itself; that family has already been tested.
Any subsequent design must specify:

1. The geometric target and primary loss, inference-time inputs, output
   semantics, and how the new representation differs from the tested controls.
2. Training/development recordings and complete frozen parameters before
   prospective evaluation; no evaluation labels or reference-range features
   in deployable scores.
3. Fixed RGB/thermal, random, camera-scale and original-score controls; both
   endpoint sensitivities, loss tails, failures, ties and availability.
4. A recording-level prospective success criterion declared before new capture
   access, with all outcomes retained. Choosing a successful comparator after
   scoring cannot establish transfer.

These requirements define missing evidence, not authorization to consume the
reserved captures or start a new training campaign. Method novelty and a
physical reference claim are separate questions. Independent geometric
reference data would strengthen physical-error attribution; no usable CATS
access is confirmed in the reviewed project records. The needed material would
be a bounded synchronized RGB-stereo/thermal-stereo sample with applicable
calibration, timestamps and an independently justified geometric reference.
No additional source upload is needed for the completed MS² result. Training
membership and the thermal checkpoint recipe remain separate provenance needs.

## Report-ready result narrative

On the selected MS² windows, learned uncertainty contains useful within-camera
measurement-ranking information against the supplied geometric references.
Controlled selective-risk comparisons retain a mean-distance advantage over
matched random selection, although simple spatial baselines provide
counterexamples. This finding concerns ranking within a camera and does not
establish that RGB and thermal uncertainty magnitudes are calibrated to a
common reliability scale.

Cross-camera comparisons use fixed nominal shared support and a declared
metric reference target. Frozen camera-selection methods exhibit
recording-dependent gains and failures, including failure of a prospective
consistent-improvement criterion. Subsequent diagnostics identify signed
residual structure, disparity-domain versus depth-domain differences,
inverse-depth tail sensitivity, and uneven marginal and conditional interval
coverage. These observations constrain interpretation but do not identify a
physical sensor cause.

In a retrospective four-recording exclusion study, constant per-camera
scaling and additive bias plus scaling fail to improve median mean-distance
selection risk over the original score on any of the four evaluated captures.
Bias-aware fitting improves thermal native-disparity likelihood relative to
scale-only fitting without consistent selection gains. The tested uncertainty
representations and simple corrections therefore do not establish transferable
cross-modal reliability or justify adaptive VO integration. The broader
reliability hypothesis remains open, subject to a distinct method, declared
prospective evaluation and the limitations of the supplied reference chain.

## Synthesis verification and recovery

Numerical anchors were recomputed from saved EXP066/069/070 scalar reports and
checked against their stored aggregates. Source report hashes and extracted
values are preserved in the synthesis recovery artifact. This is a summary
consistency check, not a rerun of native-array audits, new inference, a new fit
or additional evaluation data. EXP065/067/068 and earlier selector findings
above cite the committed experiment records; their original validation results
were not rerun for this document. No new literature or advisor decision is
asserted.

Recovery target: `ore_ms:ore-ms/backups/2026-10-28-fall-decision`.
Source result dependencies: EXP066/069/070; historical experiment records are
tracked in Git. Credentials, checkpoints and full datasets are excluded.
