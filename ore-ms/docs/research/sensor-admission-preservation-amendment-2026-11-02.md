# EXP112 preservation scope amendment before policy prediction and labels

Original all-serialized-array bitwise gate FAILED and remains failed. No pose
quality labels or policy scores have been computed. All 12 final captures remain
unchanged; no recapture, threshold, policy, input population or optimizer change.

The complete mismatch scan of 14,716 arrays identifies 198 nonidentical arrays:
three `match//obs2_covTc` fields and 195 endpoint1 local-depth-variance diagnostic
arrays (before/after floors). Maximum relative observed difference 1.108461e-6.
This is an observation of numerical drift, not a proven cause or an accuracy bound.
Large absolute differences on very large values remain in the report.

The active `Analytic_ReprojDisp_TwoFramePGO` returns uv/disparity covariance and
uses `pos_Tw`, K, baseline and native pose. It does not use `obs2_covTc` as an
optimization weight. CovarianceSanityFilter can inspect derived covariances, so
its actual masks/health must also remain exactly equal; do not merely assume
inactive use based on the optimizer. All other serialized fields remain subject
to exact equality with no floating tolerance. All native worker events except
observation clock/sequence and the new geometry events, all effective configs,
original actions/availability and all first-write identities must match exactly.

A separately frozen scoped verifier excludes only those exact three diagnostic
field families from bitwise acceptance, retains every observed mismatch with
magnitude and requires unchanged finite masks. It must fail for any changed active
graph input, source/target pose, point/selection/filter mask, native event or raw
policy decision. It uses no numeric tolerance to pass changed active values.

If that check passes, compute the unchanged fixed selector's qualified descriptive
proxy scores on the fresh motions, with actions locked before reference semantics
are read. This does not retroactively pass the original stronger engineering gate.
Report both statuses. Original all-array preservation is false in every completion
record; any policy result remains development-only and cannot constitute complete
success of the original frozen experiment. The strict per-record policy quality/
coverage criterion is unchanged and is reported separately.

Files: original failed `admission-parity-attempt-1.log`, complete diagnostic mismatch
JSON, and `tools/sensor_admission_evaluate_scoped.py`. No source/model/selection or
post-outcome quality tuning. No assertion that the observer caused or did not cause
the diagnostic drift; native off/on equality is scoped to what is actually checked.
