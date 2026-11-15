Scope correction: the user subsequently clarified that the next task is a broad gap review of the original papers. The narrow direction-selection recommendation below is historical; use [the original-paper gap map](original-paper-gap-map-2026-11-13.md). Recovery/source-check facts remain valid.

# Contribution decision after source and evidence recovery

13 November 2026, America/New_York. Evidence review, not a new method experiment.

## Decision

The defensible question remains **whether measurement uncertainty predicts the
incremental benefit of thermal to a matched RGB estimator**. Keep that question
active. Generic covariance weighting, covariance propagation, task-trained
weights and adding a shared window are already covered by relevant prior work
and/or our completed inventory. They are not untouched contribution proposals.

Proceed with an evidence-backed characterization of the distinction between
measurement reliability and incremental estimation utility. A method extension
needs a specific demonstrated mechanism and correction; none is selected here.
The present evidence does not establish a universal failure of thermal,
uncertainty learning, joint geometry or the published systems.

## What the public recheck resolved

- The [author homepage](https://shibowing.github.io/) still links an MSO video,
  with no manuscript/code link in its MSO entry. Its status label is a source
  statement, not independent confirmation of publication status.
- The [DFR organizer page](https://jonasfrey96.github.io/data_in_field_robotics/)
  lists the later MSO submission17. It supports both short papers and previously
  published posters, with non-archival workshop submissions. The listing alone
  does not identify MSO's submission type or establish a method revision.
- The linked [OpenReview venue](https://openreview.net/group?id=IEEE.org/IROS/2026/Workshop/DFR)
  returns a loading page; the linked forum requires browser verification.
  One direct API query for this venue returns HTTP403. No later manuscript was
  obtained. This bounded access result is not proof that the manuscript is
  unavailable everywhere.
- The [linked SuperOdom repository](https://github.com/superxslam/SuperOdom)
  describes a slim LiDAR/IMU system. README scope was rechecked; no new complete
  code/tree audit or MSO reproduction is claimed.

Retain the complete thesis-Chapter5 comparison in the
[prior review](mso-method-overlap-review-2026-11-12.md). Later-version novelty
remains unresolved. Do not wait indefinitely for that source or replace it with
an assertion of novelty.

## The gaps, with the correct strength of claim

| Candidate claim | Inspected prior-work boundary | Local evidence and decision |
|---|---|---|
| Comparable RGB/thermal measurement uncertainty | Statistical cross-modal calibration is not demonstrated in inspected MSO Chapter5; common image-space weighting is present. | EXP060/061/069/070 already test transfer, target dependence and likelihood/coverage. Characterization exists; another scalar calibration alone adds little. |
| Predicting relative camera error | Confidence-driven adaptation is present in MSO, but a common-target quantitative relative-error test is not demonstrated in that chapter. | Positive rain and thermal-win subsets coexist with failed transfer. State the evaluated target and recording; do not claim a universal result. |
| Predicting incremental thermal benefit | Neither covariance weighting nor a thermal-only accuracy gain logically establishes this target. Full applicable prior-art novelty is unresolved. | Shared learning and joint-window controls already executed and failed their gates. A new candidate must explain something those tests did not identify, not rerun their mechanism. |

MAC-VO/MACThermal and MAC-I² already establish relevant propagation/weighting
mechanisms in their evaluated systems. See [the covariance source review](published-covariance-claims-2026-11-12.md).
Absence of one ablation in one chapter is a study opportunity, not a novelty proof.

## Why the last gap is a different target

As an elementary scalar illustration, let RGB and thermal errors be e_R and e_T.
For a fixed blend x_a=(1-a)x_R+a*x_T and squared error, its change from RGB risk is

    E[e_a^2] - E[e_R^2]
      = 2*a*(E[e_R*e_T] - E[e_R^2]) + a^2*E[(e_T-e_R)^2].

Thus each camera's marginal variance alone does not determine benefit. The
cross-error moment and, for biased estimates, their means also matter. For
example, independent zero-mean unit-variance errors yield risk0.5 at a=0.5;
identical unit-variance errors yield risk1 at the same blend. The marginal
uncertainties are identical in these two cases. This is elementary established
estimation reasoning, not a novel theorem or a model fitted to our recordings.
Adaptive a, nonlinear pose states and shared-factor optimizers require their own
analysis; this illustration does not explain our measured regressions.

The missing evidence is a measured link from a deployable signal to a particular
failure mechanism and improved conditional thermal contribution. Bias, shared
error and directional geometry are possible explanations, not identified causes.
Covariance intersection and directional/conditioning controls already occur in
our inventory; naming them again does not create a new method.

## Finite path and stopping decision

1. Assemble the existing report around three separate endpoints: within-camera
   reliability, common-target cross-camera prediction and matched downstream
   utility. Retain positive subsets, failures and support counts. Do not pool
   different cohorts as a controlled causal chain.
2. The method-selection gate is a concrete explanation of one observed failure
   that distinguishes competing explanations and has a deployable correction.
   A proposal must identify its prior-work difference, exact unused comparison,
   necessary input contract and why existing closed tests do not already answer it.
   Do not select another learned head, covariance scale or window by default.
3. If that gate is met, freeze ONE test: unchanged frontend means/support/solver
   wherever required to isolate the mechanism; matched RGB-only, thermal-only,
   uniform-joint and existing covariance-joint controls; equally trained RGB
   control if learning is involved. Lock the hypothesis, data exposure, failure
   handling, rotation/translation/coverage endpoints and numeric advancement
   criteria before execution. No aggregate-only promotion or threshold rescue.
4. If no distinct mechanism can be defended from existing evidence, finish the
   scoped characterization and discuss narrowing/pivoting with the advisor.
   No advisor message, agreement or time budget is assumed. Access failure for
   later MSO sources limits novelty wording; it does not close the scientific question.

This is a contribution decision, not an executable protocol or a request to
reopen the strict-time configuration. Supplied nominal-pair evaluation is a
separate existing benchmark assumption, not verified physical synchronization.
No new training, inference, protected payload or accuracy scoring occurred.

## Recovered evidence and verification

The contribution draft and handoff linked two findings absent from this checkout
and available Git history. Box packet
`backups/2026-11-12-persistent-tracks-window-geometry-51cc40db/` contains both,
plus the predecessor findings. Archive SHA256:
`51cc40db87e8ee6e172475270026d836e384782d149a29dbf7ebea2aec43e2c4`.
Manifest SHA256:
`6155dc2c09a0b272a4259d06e172d67c7fea7504acd994eda0b4d1d4cc1fe761`.
Both match the separately stored archive receipt; all1558 manifest file hashes
match after safe extraction into a separate recovery tree. This verifies packet
integrity, not a fresh independent rerun of scientific results.

Restored only six absent research notes to this checkout, preserving existing
files: nominal-joint findings, shared-weight next decision/findings, windowed
next decision/track admission, and persistent-track/windowed findings. Source,
results and notebooks remain available in the separate verified recovery tree;
no prior outputs or production code were overwritten.

The [shared-weight findings](shared-weight-learning-findings-2026-11-12.md)
confirm the six40-update heads and three failed recording gates. The
[windowed findings](persistent-tracks-windowed-geometry-findings-2026-11-12.md)
confirm matched translation-mean regressions of19.74%/18.51%/0.55%. Their positive
mechanics and negative advancement results both remain part of the evidence.
Earlier proposals are historical; neither trial is pending or untouched.
