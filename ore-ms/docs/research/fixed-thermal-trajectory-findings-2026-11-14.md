# Frozen RGB–thermal increments: short-trajectory accuracy

**Interpretation update:** [Fixed-group diagnosis](fixed-group-diagnosis-findings-2026-11-15.md)
quantifies sensitivity to mismatched recorded RGB/thermal intervals. The same-ID
nominal results below remain unchanged; they do not isolate physical sensorquality
or environmental contribution. No corrected fusion result is claimed.


**Longer-check follow-up:** The [600groupvalidation](longer-fixed-thermal-findings-2026-11-15.md)
finds smaller/mixed gains and a night continuity gap. The short-prefix results
below remain valid for their original populations.


14 November2026 America/New_York; ore-ms-orb-development baseline53afa19d1.
New accumulation/evaluation endpoint only. No new weights,learning,posefits,
frontend inference or selection of a new recording/window. All18composed paths
locked before new aggregated trajectory metrics.

## Finding

Conservative thermalprecision0.1reduces short-trajectory positionRMSE compared
with same-backendRGB in day/night, but slightly worsens rain and increases
orientation error. The learned relationship layer remains almost identical to
this simple control. This is scoped evidence of useful thermal contribution,
not a demonstrated increment from the richer learned model.

The unchanged fulljoint control also has larger day/night position gains with
larger orientation costs. That differs from its earlier per-interval losses:
smaller individual increment error is not a guaranteed ordering of accumulated
trajectory error. Keep both outcomes; the earlier interval gates and negative
learning findings remain unchanged.

## Evaluation contract

Use the exact previously exposed100group nominal day/night/rain cohort.99saved
body-frame relative increments per method, right-composed from common identity
initialpose. Reference paths composed from the provided same-ID reference
increments; no new physical groundtruth. Existing rigidtransformserialization
screen1e-5 and nearestproperSO3 roundoff handling applied to all reference/method
increments; maximum changes recorded perpath. No fittedSE3/Sim3,scale,yaw,offset,
time or best-windowalignment. PositionRMSE over all100poses including initial0.
Original actualtimestamps remain in sourcebinding; same-ID simultaneity is a
nominal modeling assumption, not verified cross-camera exposure synchronization.

Maincomparison specified before execution: fixedthermalprecision0.1versus RGBcov
in the same conditional source-landmark12step backend. Learnedfullgate,fulljoint
and originalnativeRGB as declared contexts. Gate0.1was already seen in the exposed
development study; this is a new endpoint on the same data, NOT freshvalidation.
No current method has a missing increment; every path has100poses without gap
filling or interpolation. Thermal-only has a missing rainincrement and is not
silentlystitched into this continuouspath comparison.

## Position and orientation results

| Recording | RGBcov position RMSE m | Fixed0.1 position RMSE m | Learnedfull position RMSE m | RGBcov orientation RMSE deg | Fixed0.1 orientation RMSE deg |
|---|---:|---:|---:|---:|---:|
|day|1.567092|1.429680|1.429551|2.863735|2.875082|
|night|0.427860|0.395952|0.398682|0.472467|0.495655|
|rain|13.675134|13.717521|13.717806|1.183378|1.183392|

Fixed0.1changes relative to same-backendRGB:

- day: positionRMSE -8.77%; orientationRMSE +0.40%; finalpositionerror 3.3603→3.1504m.
- night: positionRMSE -7.46%; orientationRMSE +4.91%; finalpositionerror 1.4218→1.3456m.
- rain: positionRMSE +0.31%; orientationRMSE +0.00%; finalpositionerror 16.0968→16.1559m.

No metric is collapsed into an arbitrary mixed-unit pose score. The orientation
tradeoff is part of the finding, not omitted because position improved.

### Secondary fulljoint context

| Recording | Fulljoint position RMSE m | Fulljoint orientation RMSE deg |
|---|---:|---:|
|day|1.070480|3.005218|
|night|0.306366|0.661933|
|rain|14.072516|1.184370|

Fulljoint is a prespecified secondary context, not a new winner selected to
replace the primary0.1control after scoring. NativeRGB context and allfinalerror,
mean,p95 metrics remain in scores.json. NativeRGB orientation is better than the
conditional backend on day/rain; native-to-backend position gains are not solely
attributable to thermal. Do not describe this as a full-system accuracy win over
MAC-VO on everyposeendpoint.

## Duration and scale of evidence

| Recording | RGB source duration s | Provided-reference path length m |
|---|---:|---:|
|day|10.265844|50.801549|
|night|9.998790|19.031398|
|rain|9.998917|94.114757|

These are three≈10second prefixes. Rain has large reference-relative deviation
for all methods. No long-run stability,broadgeneralization,physicalreference
independence,real-timecadence or compute-saving claim. Reference paths,source
calibration and nominal frame/time assumptions are unchanged, not strengthened
by accumulating poses.

## Verification and consequence

Verified all39parentpredictionlock hashes before composition. A noncommuting
rotation/translation fixture checks right-composition order; independent component
R,t recurrence agrees with every homogeneous path step. Scalar coordinate sums
independently reproduce positionRMSE. ExistingSO3serialization policy preserved,
with bounded roundoff corrections and allposefilehashes recorded. All18paths have
100poses; no failure/tolerance amendment or newscientificrun retry.

The next question, if continuing this candidate, is whether the position gain and
orientation tradeoff persist beyond these already exposed shortprefixes. That
would be validation of a simple existingfusion baseline, not another learned-head
variant or a novelty claim. No longer-run/new-record experiment is selected here.
The completed weighting family stays consolidated; retain this distinct trajectory
endpoint alongside its interval-error evidence.

Prerequisites: [Completed learning](local-relationship-learning-findings-2026-11-14.md),
[weighting consolidation](rgb-thermal-weighting-consolidation-2026-11-14.md).
Artifacts: results/fixed-thermal-trajectory/run-001; original sourcebindings and
locked increments remain in prior verified Box backup packets.
