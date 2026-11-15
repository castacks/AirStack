# Fixed-group diagnosis: a temporal comparison confound

15 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`53afa19d1` plus recorded working sources. Nine previously fixed groups were
examined. No new fusion model, pose optimization, training, or corrected fusion
score was produced.

**Interpretation qualification:** the [completed source review](measurement-time-contract-review-2026-11-15.md)
confirms that recorded-duration differences do not establish different physical
exposure intervals. Here, “comparison confound” means sensitivity to an unresolved
benchmark assumption, not a demonstrated synchronization failure.

## Main finding

Some thermal groups previously called harmful are internally consistent by the
available image checks. In the harmful daytime group, median patch correlation
is **0.9815**, all **14/14** matches pass a classical forward/backward tracking
cycle, and native endpoints differ from classical tracking by a median **0.49 px**.
Yet they disagree with the nominal reference projection by **23.53 px**.

Their recorded thermal interval is **72.43 ms**, while the paired RGB interval is
**133.32 ms**. A single fixed reference-motion probe based on that duration ratio
reduces disagreement to **3.32 px**. The useful rain group similarly drops from
**36.05 to 1.37 px**.

This corroborates a substantial **comparison confound**: the nominal same-ID
benchmark imposes one motion on observations with different recorded intervals.
Conflict with that benchmark cannot be attributed solely to thermal quality or
environmental change. Apparent benefit can also reflect error cancellation.

The timestamp differences were already documented, and strict interval binding
was previously closed. What is new here is their measured effect on the exact
localized groups. I should have tested this sensitivity before interpreting the
nominal fusion results as evidence about thermal complexity or predictability.
The original benchmark numbers remain unchanged; their physical interpretation
must be restricted. This does not invalidate MACThermal's standalone thermal VO,
within-camera studies, or earlier experiments with different timing contracts.

## The diagnostic is not a timing fix

Let `T` be the inverse of the provided RGB relative pose, and let `alpha` be the
recorded thermal duration divided by the RGB duration. The posthoc probe uses
`Exp(alpha * Log(T))`, assuming constant body twist, to project the **same inferred
source points**. It fits no offset or scale and searches no frame associations.
It changes no original measurements, fusion outputs, or scores.

This does not qualify the physical clock or exposure semantics, validate an
asynchronous estimator, or supply a deployable correction. Source-depth bias,
nonconstant motion, reference errors, and calibration remain possible factors.
Translation and depth jointly affect projection, so changing reference-motion
magnitude can partly compensate depth error. Exact source-depth support is
missing for the selected helpful/harmful groups, preventing a unique diagnosis.
All nine cases, including cases where the probe worsens agreement, are retained.

## All fixed-case results

Patch correlation uses original same-camera 11×11 grayscale patches. Classical
tracking uses a fixed 21×21 window and three pyramid levels, without initializing
from native target coordinates; cycle support requires ≤1 px return error.
Neither image similarity nor classical tracking is correspondence ground truth.

| Case | Pair | Selected matches | Median patch NCC | LK cycle support | Native/LK endpoint difference, px | Thermal/RGB duration | Nominal reference residual, px | Duration-probe residual, px |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| day / helps | 6 | 10 | 0.6673 | 6/10 | 3.817 | 0.488 | 5.479 | 0.531 |
| day / hurts | 0 | 14 | 0.9815 | 14/14 | 0.487 | 0.543 | 23.528 | 3.320 |
| day / neutral | 92 | 5 | 0.7592 | 5/5 | 0.545 | 2.063 | 5.581 | 4.165 |
| night / helps | 91 | 15 | 0.9695 | 14/15 | 0.411 | 1.101 | 3.365 | 4.158 |
| night / hurts | 62 | 15 | 0.5832 | 9/15 | 3.660 | 0.613 | 7.982 | 3.638 |
| night / neutral | 17 | 24 | 0.9847 | 24/24 | 0.080 | 1.952 | 0.167 | 0.126 |
| rain / helps | 78 | 4 | 0.9589 | 3/4 | 1.964 | 0.437 | 36.048 | 1.370 |
| rain / hurts | 1 | 9 | 0.8071 | 8/9 | 0.731 | 0.459 | 9.496 | 3.163 |
| rain / neutral | 77 | 2 | 0.8338 | 2/2 | 0.327 | 1.084 | 8.218 | 9.038 |

The daytime harmful group is not corroborated as a simple low-contrast or bad
tracking failure. Timing/model sensitivity is substantial; its remaining 3.32 px
residual is not automatically a depth error.

The nighttime harmful group does show tracking ambiguity: NCC 0.5832, only 9/15
valid cycles, and 3.66 px native/LK disagreement versus 0.67 px in the remaining
pool. Its duration probe also improves agreement. More than one issue may coexist.

The probe worsens the helpful night and neutral rain groups. The neutral night
group has tiny residuals under both models despite a nearly twofold duration
ratio. Neither a duration discrepancy nor a tracking score alone determines
pose contribution; motion and geometry matter.

## Recorded interval differences across the existing cohorts

The following uses a descriptive ±10% duration band. It is not a synchronization
tolerance, an exposure qualification, or a new admission rule. No samples were
removed from earlier metrics.

| Cohort | Day outside ±10% | Night | Rain |
|---|---:|---:|---:|
| primary100 | 90/99 | 35/99 | 25/99 |
| longer600 | 316/599 | 331/599 | 331/599 |

Ratios span roughly 0.40–2.20 across the examined cohorts even though their medians
are near one. Same file index or average frame rate therefore does not establish
that two measurements describe identical recorded intervals. This is a plausible
confound for changing fusion outcomes, not a causal explanation of every earlier
model result.

## What the depth evidence can establish

Sixty sparse, own-thermal projected depth maps were recovered from the verified
`2026-11-13-natural-flow-reference-contract` packet, archive SHA256
`5b25f88bbb3dcdecda0f97da88d83e6d0d7aa833ce559ad55011a9843ec9c09a`.
They cover frames 0–19 in each primary recording. Archive/member hashes, sequence,
and intrinsics were checked. The uint16 values represent metres×256; zero is
missing. Sampling uses the original nearest pixel, with no filling, resizing,
neighborhood expansion, or replacement cases.

**None of the selected helpful/harmful source groups has a valid supplied depth
sample.** One selected neutral-night point does. Some points outside the selected
groups have support; they cannot establish depth accuracy inside those groups.
The helpful daytime group has one supported target-depth sample. Later cases
lie outside the recovered map range.

Artifacts retain source-depth error, target-disparity disagreement, and
source-depth-substituted projection only where those exact samples exist.
These supplied single-scan products also have unqualified projection, occlusion,
timing, and uncertainty. They are not certified physical ground truth. No claim
of moving objects follows from low patch correlation or image appearance alone.

## Evidence chart

![Fixed-case image checks and reference-duration probe](../../results/fixed-group-diagnosis/run-001/fixed-case-diagnosis.png)

The six illustrated groups and three earlier neutral controls were fixed from
previous reference-based outcomes. This is explanatory diagnosis, not independent
predictive validation or a new rejection policy. The duration probe and duration
inventory were added posthoc after the initial correspondence checks; all their
positive and negative outcomes are reported.

## Verification and next prerequisite

Independent checks reconstruct 98 selected-point projections, 18 SE3 ratio-one
null/reconstruction cases, and 25 exact sparse-depth decodes. Source images and
packet hashes remain unchanged. NCC identity, brightness-scale, sign-reversal,
and constant-patch fixtures pass, alongside scalar Pearson reconstruction.

Before another physically interpreted RGB–thermal fusion experiment, establish
what the measurement timestamps mean and how the camera intervals relate—or use
a verified synchronized setup and an estimator that models each sensor's actual
interval. Consult the existing producer/loader/clock review first. This diagnostic
does not reopen the failed offset search or justify treating reference-assisted
duration scaling as a deployable solution.

The nominal fusion and learned-gate results remain valid software-benchmark
observations. They do not settle whether thermal's environmental contribution is
random, unpredictable, or intrinsically difficult to learn. The current priority
is a valid temporal comparison, rather than another weighting head.

See the [previous strict-input decision](shared-motion-input-contract-findings-2026-11-12.md)
and [fixed-group localization](dynamic-thermal-evidence-findings-2026-11-15.md).
Artifacts: `results/fixed-group-diagnosis/{recovery,run-001}`. No new paper-method
claim or author-system reproduction is introduced.
