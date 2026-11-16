# Full-span day thermal ablation with explicit RGB fallback

Use all599increments0→1 through598→599 of the existing first600frame originalday
`_2021-08-13-16-31-10`. This is exposed development data; do not change the population,
select a successful prefix or use the later span with failed native reproduction.
Preserve the earlier stopped attempt as a separate failed contract.

Four own-state arms: native RGB, extra-full-pose RGB, extra-translation RGB, and
fixed0.1thermal translation refinement with RGB-only fallback. Retain native graph,
registration, float32 writeback, ten-step scheduler, loss/precision and stopping
settings. Each refined arm runs its own RGB first solve, then its second solve.
Final state seeds the next interval. Constrained arms keep exactly their own
first-pass quaternion; they need not equal the separate native stream.

Fallback is triggered ONLY by absent/unsupported saved native thermal first-write,
not by scores, loss size, poor matches, timeouts or solve failures. Whole-span
preflight already identifies pairs25,27,28,32. At those four intervals the thermal
arm executes the same RGB-only translation second solve as its RGB control,
using its OWN parent state and own first-pass solution. It does not copy another
arm's pose or reuse a stale thermal packet. On the other595intervals use the
unchanged native calibrated thermal factors at precision0.1. No new learned gate,
weight search, timing correction, raw inference or training. Any other bind/solve
failure stops the capture; do not widen fallback conditions after predictions.

Before capture, verify source/input hashes, configs/timestamps and bindings for
all599RGB and thermal pairs. Record support/reason per pair; require599RGB and
exactly595thermal with the four declared fallback pairs. Check at each fallback
pair that packet=None native-thermal graph and RGB graph have identical residuals,
covariances and translation Jacobians and identical final committed solutions
under the same initialization. Freeze preflight, qualification, adapter, reused
runner and protocol hashes before predictions. Adapter supplies missing-event
sentinels and packet=None only for declared fallback, without editing frozen code.

Rotate four-arm execution order each interval. Record all599outputs, own states,
first/second losses, scheduler calls and CPU/wall costs. Lock predictions before
reference scoring. Primary endpoint:600pose positionRMSE versus BOTH refinedRGB
controls with full599coverage. Also report interval T/R and path position/orientation
mean,RMSE,p95, no alignment/scale/gap fill. Report thermal usage595 and fallback4
separately. Runtime is one cached solver capture excluding graph construction and
frontend; no timing significance or end-to-end performance claim.

Audit state continuity, source/input locks, initialization/rotation, pose deltas,
all scores and costs via separate scalar/matrix arithmetic. Check exact fallback
identities against preflight and graph-equivalence qualification; reject semantic
state/rotation/cost/score/fallback corruptions. Report native reproduction against
frozen1mm/1e-4rad on all599pairs. A failed gate remains diagnostic without waiver.
Stop after the comparison; preserve positive short-day and negative longer-night
results. No automatic new recording, parameter search, model or physical claim.
