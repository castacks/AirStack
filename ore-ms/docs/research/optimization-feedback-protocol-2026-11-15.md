# Continuous-feedback RGB optimization control

This development comparison checks whether the completed fixed-pair optimization
result survives each arm's own state feedback. Use all night intervals 599→600
through 1198→1199, with the same restored accepted measurements and references.
No new recording, neural inference, thermal factors, weights or solver settings.

All arms begin at the archived native parent pose for source599. Native runs one
unchanged native solve at its own state. Full-pose and translation-only arms each
run that same first solve at their own state, then an additional solve initialized
at its float32 committed solution. Translation-only holds that solution's quaternion
exactly; full-pose retains the native parameterization. The final float32 solution
becomes the next state, rebuilding source point coordinates and covariance through
the unchanged feedback implementation. States can diverge across arms. This is
cached-measurement state feedback, not fresh frontend or full live VO.

Freeze this protocol, implementation and input/reference hashes before predictions.
Retain native Huber .1, PINV, TrustRegion1e3, minimum1e-6 and ten-step scheduler
with patience2/decreasing1e-5 for every solve. No tuning or output exclusions.
Use the existing three-pair fixed-input qualification for the unchanged solver and
wrapper. Native must reproduce archived relative motions within1mm/1e-4rad on all
600 intervals for qualified claims. Failed solves remain unavailable and invalidate
continuous-path summaries; do not gap-fill, restart or choose successful blocks.

Record both first and second solver CPU/wall times and actual scheduler calls.
Rotate arm execution order each interval. Lock predictions before reference scores.
Report all600pair availability and matched-support interval T/R mean,RMSE,p95;
601pose path position/orientation mean,RMSE,p95 from the common initial gauge,
without alignment/scale fitting. Keep interval and accumulated rotation separate.
Use the existing score implementation; independently audit state continuity,
source locks, first-to-second initialization, fixed quaternion, matrix composition,
reproduction gates, support and all scores/cost totals. Preserve failures.

Stop after this comparison. Attribute gains to extra optimization when full-pose
matches the constrained gain; assess constraints separately for rotation/cost.
No fresh-record, timing significance, physical calibration, novelty or end-to-end
runtime claim. No automatic new recording or thermal model follows a mixed result.
