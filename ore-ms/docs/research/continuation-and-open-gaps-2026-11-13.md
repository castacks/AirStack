# Recovered continuation: core-paper gaps and next decision

13 November 2026, America/New_York. Repository onboarding and synthesis of the
existing source reviews; no new paper reading, experiment or result.

## Where the discussion stopped

Continue on `ore-ms-orb-development`, not the historical `ore_ms` baseline.
The latest committed scope decision requires a finite contribution decision
before further experiment variants. The research objective remains active.
A failed local configuration does not show that a published system fails or
that there are no remaining research gaps.

## Gaps supported by the inspected sources

These are boundaries of the inspected evidence, not verified publication novelty.

| Core work | Established overlap | Question still open in the inspected evidence |
|---|---|---|
| MAC-VO / MACThermal | Learned correspondence uncertainty, geometric propagation and covariance-weighted estimation; thermal adaptation has positive published results. | Does uncertainty predict relative RGB-versus-thermal geometric error on a common target, across recordings? MACThermal's joint matcher/covariance changes do not isolate that claim. |
| MAC-I² | Propagated visual pose covariance and learned visual–inertial weighting, with positive initialization/calibration ablations. | Does the same reliability meaning transfer to adding thermal measurements to RGB? Its vision–IMU evidence does not answer this sensor-pair question. |
| MSO, inspected thesis Chapter 5 | RGB–thermal image-covariance-weighted sliding-window estimation and dynamic factor removal. | Statistical cross-camera calibration, quantitative relative-error prediction and a matched fixed-input weighting ablation are not demonstrated in the inspected chapter. Later manuscript/code remain unverified. |
| DNLS / PNEC | Correspondence uncertainty tied to pose optimization; bearing-space uncertainty for rotation. | A shared RGB–thermal learning objective needs a justified temporal/input contract and downstream evidence. The local synthetic derivative check now passes; strict recorded shared-interval binding does not. Neither result closes these published methods. |

Source depth and exact qualifications: [prior-art review](mso-method-overlap-review-2026-11-12.md),
[published covariance claims](published-covariance-claims-2026-11-12.md),
[research reset](multisensor-research-reset-2026-11-12.md), and the
[living paper registry](references/README.md). This table does not assert a
new external-source verification.

## What our results already answer

Do not restart calibration/ranking as untouched work: EXP060/061/069/070 cover
scale transfer, error-target dependence, coverage and likelihood. Some rain and
thermal-win subsets are positive. Better likelihood did not reliably improve
selection; several specific weighting, learning and windowed recipes missed
their gates. Those are scoped outcomes, not a general impossibility result.
The independent stationarity audit resolved the prior numerical discrepancy;
calling that prototype numerically failed is stale. The recorded strict input
contract remains closed for the inspected prefix, separately from supplied-pair
nominal benchmark assumptions.

See the [contribution draft](contribution-statement-2026-11-13.md) for the
completed evidence and [scope decision](research-scope-decision-2026-11-13.md)
for the current boundary. Different cohorts and targets do not establish one
causal explanation of the pose regressions.

## Concrete next decision

The candidate question is: **when does an uncertainty signal predict the
incremental benefit of adding thermal to a matched RGB estimator, rather than
merely predicting its own residual?** This is an open question, not a selected
new method or an established novelty claim.

First complete a version-specific claim comparison against the accessible later
MSO manuscript/implementation, if available, and the existing local inventory.
Then specify one finite test only if a distinct mechanism or untested claim is
identified. Its controls must separate RGB-only, thermal-only, uniform joint
weighting and the proposed joint method with matched means, support and solver;
report rotation, translation and availability separately. A result must satisfy
fixed recording-wise benefit and coverage criteria, with the signal available
at its declared decision time. Numerical thresholds and an appropriate data
partition must be justified before execution; exposed data cannot become fresh
holdout by renaming it.

Stop this proposed route if it merely repeats a closed recipe, lacks an
identifiable distinction from prior work, or misses its predeclared benefit/
coverage gate. No automatic threshold, window or model sweep follows. This
records a reviewable decision frame; it does not invent an advisor agreement,
time budget, new execution freeze or experiment authorization.
