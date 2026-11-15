# Pose-posterior reliability: a distinct original-paper assessment

14 November2026, America/New_York. Branch `ore-ms-orb-development`,baseline
`a79658f6341194b0954852226f200313ccdf5408` plus working changes. Targeted
source/inventory review; no new inference, fitting, covariance score or experiment.

## Decision and precise question

Retain original-paper I2/I4 as a separate **empirical reliability question**:

**For a fixed learned visual estimator, how much does its reported pose covariance
predict uncertainty conditional on accepted correspondences, and how much does it
predict variation when matching, selection and initialization are regenerated?**

This has a clear next comparison: paired frozen-frontend and regenerated-frontend
trials under one declared input ensemble. It is not a new covariance algorithm,
a demonstrated deficiency of MAC-I², or a field-wide novelty claim. A generic
replacement of inverse Gauss–Newton information with an exact Hessian, sensitivity
formula or initialization ensemble is not sufficiently distinct prior art.

## What the original paper already establishes

[MAC-I² v1](https://arxiv.org/html/2609.07116v1),AppendixB, explicitly linearizes
at a stationary solution, freezes residual covariance there and uses inverse
Gauss–Newton information as a local Gaussian approximation; it also transports
that covariance into the pose residual. AppendixE evaluates pose benefit and
AppendixF reports visual covariance binning/generalization. Thus “no uncertainty
validation” or “the approximation is undisclosed” would be false. Those reported
results do not, by themselves, specify the repeated-sampling experiment proposed
here. No figures were independently reanalyzed and no author code was run.

## Prior work already covers broad corrections

[Bonnabel, Barczyk and Goulette](https://arxiv.org/pdf/1410.7632) analyze
registration covariance and rematching, distinguishing point-to-point from
point-to-plane ICP. Their argument is direct prior overlap for claiming that a
fixed-correspondence local approximation need not characterize a rematching
algorithm. It is not a theorem that learned optical-flow matching has the same
failure, or that MAC-I²'s empirical calibration is invalid.

[Brossard, Bonnabel and Barrau v2](https://arxiv.org/html/1909.05722v2),selectedII-B/F/G andIII-B,
explicitly model initialization uncertainty and its correlation with registration
output, using an unscented calculation. Generic initialization-aware covariance
or a small pose ensemble is therefore not new. Targeted method text was read;
no implementation or benchmark reproduction was performed.

This is a bounded comparison, not an exhaustive uncertainty-estimation survey.

## Separate the quantities before evaluating them

| Quantity | Meaning | Evidence needed |
|---|---|---|
| Conditional local covariance | Local uncertainty under a stated residual/noise model, accepted graph and stationary solution | Correct residual contract, tangent convention, rank and stationary solution; local approximation checks |
| Repeated-estimator covariance | Dispersion of the complete estimator when declared random inputs, initialization and algorithmic choices are rerun | A defined sampling ensemble and regeneration of all claimed stages |
| Error second moment | Dispersion plus systematic error relative to the declared target | Known or explicitly qualified target; bias reported separately |

Writing a covariance for one quantity does not automatically establish the
others. Conversely, learned weights can empirically absorb effects not explicitly
present in the analytic approximation; absence of an explicit term alone is not
proof of miscalibration. Posterior uncertainty, repeated-sampling dispersion and
reference-error scores must not be treated as interchangeable.

For an estimator A(Y), its mean bias b and covariance V give error second moment
V+b b^T. Covariance inflation alone cannot identify or remove b. This identity
is ordinary statistical bookkeeping, not a new mechanism. Any covariance
comparison should separate centered dispersion from error about a known target.

## Existing local work prevents repeating easier tests

- The [November9 prototype](pose-covariance-prototype-findings-2026-11-09.md)
  already implements a rotation-dependent Gaussian likelihood, expected Fisher
  and mean-only information, plus a160-trial small-noise check. Another local
  known-noise sanity check alone would repeat that stage.
- [Native-information weighting](native-information-weights-findings-2026-11-09.md)
  already holds native means fixed, compares scalar/full covariance and fits
  recording-excluded scales. Repeating scale calibration or RGB–thermal fusion
  would not test regeneration effects.
- The [stationarity audit](shared-motion-stationarity-findings-2026-11-12.md)
  already distinguishes solver termination from sufficient stationarity for
  derivatives. Convergence should be verified, not rediscovered as a new method.

These outcomes do not settle the paired frontend-regeneration question. Saved
accepted-match packets cannot reconstruct the match/rejection choices that were
not retained. Resampling those packets only tests a conditional approximation.

## One bounded assessment to design, not an execution freeze

Use one estimator/checkpoint and a single declared perturbation-generating model.
The distribution and magnitudes must have an independent rationale fixed before
outcomes; do not escalate them until failures appear. Identify whether its target
is known imposed geometry or a supplied-pose proxy. A synthetic perturbation
model is not physical sensor calibration or a fresh-real-world benchmark.

For each original input pair, use paired trial IDs and perturbations:

1. **Frozen frontend:** retain its correspondence graph/selection, then propagate
   the declared uncertainty through that fixed residual problem. State exactly
   which quantities are resampled and how they relate to the raw-input model.
2. **Regenerated frontend:** rerun matching, masks, selection, initialization and
   the same pose solver under the same declared input draws. Record every graph,
   selected ID, state and failure. Do not give the two arms different noise budgets.

The first arm is informative only if it is a genuine conditional counterpart to
the second. Independently adding arbitrary 3D noise to one arm and gamma changes
to the other would confound the comparison. Building this shared generative
contract is a required design step, not already solved by existing packets.

Report mean bias, centered empirical covariance, target-relative error second
moment, directional/full-tangent coverage where valid, conditioning, and all
failure/availability counts. Keep per-pair populations and paired differences;
do not turn neighboring video frames into independent sample claims. Separate
converged and failed outcomes without silently deleting failed trials from the
complete population. Conditional coverage among successful solves is not complete
pipeline reliability. No candidate correction should be fitted on these outcomes.

Bind exact inputs, number of trials, permitted randomness, no-change controls,
source/runtime, statistical precision and endpoints in a later execution freeze.
At present those details are not bound; no numerical test has run.

**Stopping rule:** run one paired assessment only after the shared generative
and replay contracts are qualified. If conditional and regenerated behavior
agree within predeclared precision, report that positive validity boundary and
stop. If they differ, identify which regenerated stage explains the difference
before designing a correction; a larger empirical covariance alone does not
justify a new head. Do not automatically sweep perturbation severity, increase
trials, change the solver or fit a scale to obtain a preferred conclusion.

## Readiness and contribution boundary

Current status is **DESIGN_REQUIRED_FOR_PAIRED_REGENERATION**, not executable.
Existing frozen graphs and stationary fits support the conditional side. Full
native frontend replay needs its exact runtime/checkpoint/preparation contract;
the original MAC runtime has not been restored in this continuation. The
qualified U²Flow CPU runtime is a different network and cannot silently stand
in for a MAC-I² reproduction. Applicable stereo/raw inputs and the shared
perturbation model must also be bound. No new payload was restored here.

If a learned-matching-specific regeneration effect is measured, its practical
size and prior-work difference could motivate a method. Neither is established
by this review. The immediate contribution possibility is a scoped assessment
of a learned estimator's uncertainty target; generic covariance correction is
already well populated. Other original-paper openings remain available.

Source/reading depth and hashes of seven existing local files are recorded in
`results/pose-posterior-intake/intake.json`. This note is distinct from the closed
fixed/uncertain-source novelty claim and does not reopen that experiment.
