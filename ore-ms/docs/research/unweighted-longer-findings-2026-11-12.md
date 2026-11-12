# Longer fixed unweighted rotation test: close the route

12 November 2026. **CLOSE_UNWEIGHTED_ROUTE_NO_RETUNING.** The fixed thermal
replacement-flow plus unweighted NEC candidate fails the predeclared gate on
day, night and rain. Its promising20-frame comparisons against thermal do not
support advancement against RGB on the longer extensions. No full-motion
integration or further parameter search follows.

## What was fixed before execution

The [execution freeze](../../experiments/unweighted-longer-freeze-2026-11-12.json)
binds2,512source/input/model/reference hashes before new inference. Use exactly
the first100original groups000000–000099 of:

- Day `_2021-08-06-11-23-45`.
- Night `_2021-08-13-21-18-04`.
- Rain `_2021-08-06-16-19-00`.

These are the same exposed recordings as the small development studies. Source
pixels come from verified existing EXP137 recovery; exact RGB pose files come
from verified EXP142 recovery. Both complete recovery archives and all regular
member hashes pass before selected extraction. Source0reuses its already
verified archived pixels; records1–99retain original raw uint8RGB/uint16thermal
pixels and source timestamps. There are1,200selected raw images and300provided
RGB reference poses. No new full dataset or recording was acquired.

The primary population is the added80adjacent intervals, pair IDs19–98
(frame19→20 through98→99). It excludes the19previously scored opening
intervals. The full99-interval prefix is secondary context. The extension is
new to this candidate comparison, not a held-out recording or known
model-training exclusion.

The candidate remains frozen TartanMatch thermal-to-thermal flow followed by
the same equation-level unweighted NEC solver. No covariance weights enter its
objective. Its585-point grid, original/replacement joint validity screening,
RANSAC mask intersection, seed0,1-pixel threshold,0.999probability,
5000iterations, eight-point initialization, regularization-independent NEC
objective,200max_nfev, termination tolerances and geometry gates are unchanged.
Original covariance is retained only in the prior finite/SPD input validity
screen; all173,745grid covariance matrices pass, so it excludes no points here.
The geometry follows the NEC objective summarized in the
[PNEC paper](https://arxiv.org/html/2204.02256v1); the matcher uses the frozen
[author implementation](https://github.com/castacks/tartanmatch/tree/b3f65ffdf9723a028d82f36ab0beb8457d0def72).

Six fresh native controls complete100frames each: RGB and thermal on every
recording. The thermal observer records packets and calls the same frozen
matcher while returning original depth/match objects unchanged. Replacement
flow is never injected into the native VO controls. All three candidate
prediction locks are sealed globally before any reference-value scoring.

The gate requires a nonempty triple-common population, no candidate interval
coverage loss against either control, and strictly lower rotation mean AND
RMSE than BOTH native RGB and thermal per recording. Every recording must pass;
neither pooling nor full-prefix context rescues a failed extension.

## Primary extension results

Rotation errors are degrees. Each metric comparison uses the identical
candidate/RGB/thermal intersection. Coverage counts below are each method's
own eligible intervals out of80; they need not equal the metric intersection.

| Recording | Matched intervals | Candidate mean / RMSE | RGB mean / RMSE | Thermal mean / RMSE | Coverage candidate / RGB / thermal |
|---|---:|---:|---:|---:|---:|
| Day |80|0.077166 / 0.094713|0.050779 / 0.054640|0.116982 / 0.133229|80 / 80 / 80|
| Night |78|0.096194 / 0.120254|0.041825 / 0.058575|0.074364 / 0.097337|78 / 80 / 78|
| Rain |78|0.036048 / 0.041203|0.037558 / 0.041102|0.095485 / 0.122605|79 / 80 / 78|

Day improves against thermal but loses to RGB on both primary metrics. Night
loses to both controls and loses interval coverage against RGB. Rain slightly
improves RGB mean but has slightly higher RMSE and lower coverage than RGB;
it improves against thermal. Thus all three recording gates fail.

## Full-prefix context also retained

These99-interval results include the previously inspected opening and are not
the primary endpoint. Again, metrics use each recording's triple intersection.

| Recording | Matched intervals | Candidate mean / RMSE | RGB mean / RMSE |
|---|---:|---:|---:|
| Day |99|0.068692 / 0.086580|0.047218 / 0.051366|
| Night |97|0.079494 / 0.107953|0.035375 / 0.052741|
| Rain |96|0.040361 / 0.047273|0.036382 / 0.041918|

Native thermal full-prefix metrics and every per-interval outcome are retained
in `scores.json`. Full-prefix own coverage candidate/RGB/thermal is99/99/99
for day,97/99/97for night and97/99/96for rain. No aggregate score replaces
the per-recording failure decision.

## Availability, execution and audit

All297candidate solves converge under the fixed checks; all600native control
frames complete. Availability gaps therefore must not be described as solver
nonconvergence. The inherited nominal horizon excludes candidate pair23and27
at night, and pair1and78in rain. Pair1is outside the primary extension.
Native thermal also lacks an eligible result at rain pair43. Those gaps remain
explicit rather than being filled or removed from the coverage denominator.

Across each99-pair prefix, shared finite/inbound support before/after the fixed
geometric filter is54,551/54,305(day),57,068/56,527(night) and
53,074/49,304(rain). These are predicted, geometrically filtered matches, not
independently verified visible-point correspondences.

Forty synthetic numerical/filter/mapping/decision tests pass before execution.
The independent saved-output audit reconstructs297NEC costs,882mapped
candidate/control errors, both extension/full-prefix statistics, availability
and all recording/overall decisions. It does not rerun the optimizer or networks.
Calibration lookup and reference serialization qualification remain shared with
the inherited checked scorer; physical calibration/reference truth is not
independently certified.

A supplementary stricter cost audit verifies all297costs at rtol1e-8,
atol1e-14, with maximum absolute difference0. Its initial script inverted stored
float32intrinsics without first casting to the producer's declared float64.
That auditor-only precision mismatch is corrected and the original script and
amendment retained. No scientific code, input, prediction, score, tolerance or
decision changed; no inference, solve or scoring was repeated.

The recovered MAC runtime, original RGB/thermal checkpoints and public frozen
matcher are unchanged. No package build/update, training, covariance fit,
checkpoint selection or scene/parameter search occurred. This is completed-output
rotation evaluation on supplied references with nominal frame/clock assumptions,
not latency, fused-pose, metric-translation or full-trajectory evidence.

## Decision

Close promotion of this exact pretrained replacement-flow/unweighted-NEC route.
The earlier secondary20-frame gains were a legitimate lead, but the longer
added intervals and RGB comparator do not meet its advancement criterion.
Keep the negative longer evidence alongside the positive small-window results;
do not tune thresholds, select another interval or replace the primary endpoint
to rescue this route.

This is a result about the fixed configuration and these development recordings,
not a rejection of thermal VO, learned correspondence or the broader
uncertainty literature. It does not reproduce or reject an author's complete
PNEC/DNLS system. A future research direction needs a distinct paper-supported
hypothesis and evaluation; this task starts no further branch.

Artifacts: `results/unweighted-longer/run-001/` includes input/metadata locks,
six native controls,297candidate attempts/masks, global prediction lock, all
scores, decision, tests, independent audit and strict-cost amendment. Code:
`unweighted_longer.py`, `test_unweighted_longer.py`, `audit_unweighted_longer.py`.
