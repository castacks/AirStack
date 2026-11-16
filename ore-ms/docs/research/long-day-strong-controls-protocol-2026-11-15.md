# Longer day thermal ablation against stronger RGB controls

Use the first600frames, all599increments0→1 through598→599, of the original
candidate day `_2021-08-13-16-31-10`. This is a historically exposed development
population with earlier scoped gains, not the short day transfer recording and
not a blind test. The later599→1199span remains excluded because its native
reproduction gate failed. No outcome-based window selection or waiver.

Run four own-state arms from the same recorded source0parent: native RGB,
extra-full-pose RGB, extra-translation RGB and extra-translation RGB+thermal.
Each refined arm has its own native RGB first solve and identical initial state
for its second solve; retain its final float32 pose for next-pair registration.
The constrained arms hold exactly their own first-pass quaternion. Thermal uses
the unchanged calibrated native thermal graph at precision0.1, never a new weight.
Use the same native loss/precision, ten-step scheduler, stopping and solver bodies.
No new inference, training, timing correction, tuning or gap filling.

Verify original graph/event/source/calibration hashes and static/all-keyframe,
mapping-disabled disparity configurations, source timestamps and source pins.
Freeze protocol/source/input/reference hashes before predictions. Bind all thermal
packets without reference scoring. Rotate four-arm execution order each interval.
Record both solve stages, outer scheduler calls, losses and CPU/wall time.
A failed solve/binding leaves a partial capture, not a qualified complete-path gain.
Every native delta must pass the unchanged1mm/1e-4rad reproduction gate for qualified
aggregate claims; retain failure rather than adjusting settings or excluding pairs.

Lock predictions before scoring. Report599pair coverage and interval translation/
rotation mean,RMSE,p95, plus600pose path position/orientation mean,RMSE,p95 with only
initial gauge, no alignment or fitted scale. Primary thermal endpoint is position
RMSE improvement against BOTH refined RGB controls with full coverage. Report
orientation tradeoffs separately. CPU is cached solver time including both stages,
excluding frontend, registration and graph construction; one capture is not timing
significance or end-to-end performance. No universal-success or novelty claim.

Independently audit population, own-state continuity, pose-delta bindings, both-stage
costs/steps, fixed quaternion retention, source locks and reference statistics with
separate scalar/matrix arithmetic. Corruptions of state,rotation,cost,score must fail.
Stop after this comparison. Keep its result with short day gains and longer-night
regression; no automatic new model, weight/window search or next dataset follows.
