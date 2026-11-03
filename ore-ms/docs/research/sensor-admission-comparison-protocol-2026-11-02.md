# Fixed RGB–thermal admission comparison: executable development protocol

2026-11-02 America/New_York. Direction approved at `69f1d960e`, preparation parent
`c13f47f7c`. This is an implementation/development protocol. It is not a completed
capture freeze, prospective test assignment, empirical result or adopted method.

## Intervention and target

Current infrastructure runs two independent native VO estimators and selects a
whole motion projected onto an RGB interval through the existing stationary-body-
twist model. The primary candidate changes this selection only. It does not
combine measurements into a common graph or modify native covariance/optimization.
Both original frontends keep running on every frame. There is no compute-saving
claim. Reference target remains nominal body recorded-SLAM interval motion; no
physical accuracy or verified checkpoint-training holdout is asserted.

Reuse the entire six-window, three-recording EXP091 development population for the
first bounded native-execution preservation/comparison capture: night early/middle,
Day 0, Day 1 original/early/late. Repeated captures are development. Do not replace
failed or inconvenient windows, count nearby windows as recordings, or open a
reserve. Exact original image IDs/config/checkpoint/runtime hashes, matched fresh
flag-off/on runs and static metadata must be frozen before that capture. This
preparation does not claim that these inputs have already been restored/pinned.

## Geometry observation and interval mapping

For each submitted native disparity graph, copy its fixed points, K, baseline,
active uv/disparity covariance and edge binding. When the native optimizer result
is actually passed to writeback, compute the Gaussian information at that returned
pose **before first writeback**, without an extra optimizer poll. Emit it with the
same native interval, graph indices and submission ID. Repeated stale writes do
not emit fresh evidence. Recorder failures remain explicit and do not modify
native control flow. The original recorder/runner/optimizer files are unchanged.

This returned-pose choice supersedes the initial-pose observation considered during
implementation: it binds geometry to the actual available motion and avoids
transporting an uncertainty summary linearized at a different estimate. No
scientific capture occurred during that engineering refinement.

Use the exact native disparity residual: `(u,v,disp)` with native float32-created
covariance buffer promoted to float64 and `pinv(Sigma)` (rcond 1e-15). For NED camera
point p, right pose perturbation gives `dp/dxi = [-I, skew(p)]`. Accumulate
`H_camera = sum J_i^T pinv(Sigma_i) J_i`, six tangent coordinates ordered metres
then radians. Singular PSD covariance is retained as in native pseudoinverse
weighting. Invalid/nonfinite inputs produce unavailable observation, not invented
information. No robust correction, damping, map/parent/extrinsic uncertainty or
marginalization is included. This is a conditional Gaussian geometry diagnostic,
not the exact robust solver Hessian, a calibrated pose covariance or a proof of
observability/accuracy.

The contract preparer binds geometry to the same first supported motion in the
observed controller prefix. Nominal camera-to-body conjugation and EXP088's
stationary-twist interval projection are reused. Numerically differentiate that
same motion mapping in right tangents (central steps 1e-6 m/rad), then transport
information as `H_target = A^-T H_camera A^-1`. Reject ill-conditioned transport,
near-pi branch ambiguity and the existing one-native-duration horizon violation.
Require the projected mean to match the original interval model; no new mean or
reference-assisted bridge. Keep source/target interval IDs, transport Jacobians,
source file hashes, geometry receipt, original motion receipt and deadline.

This adds no model, timing, extrinsic or cross-camera covariance. Its uncalibrated
absolute scale is an explicit hypothesis limitation. Do not sum camera information
as if the experts were independent factors in a joint graph.

## One fixed policy, no fitting

Normalize the nominal-body information by
`D=diag(0.1m,0.1m,0.1m,1deg,1deg,1deg)` expressed in metres/radians.
Call a current motion geometrically adequate iff `lambda_min(D^T H_target D)>=1`.
These are fixed **engineering design budgets**, not tolerances inferred from
measured errors or calibrated probability bounds. No threshold or budget sweep.

Maintain separate consecutive adequate/supported counts per camera and per
recording/window. Reset on a missing/unsupported motion, inadequate information,
nonconsecutive ordinal or broken interval chain. Require two consecutive adequate
supported motions for confirmed recovery; a camera with only one is recovering.
At every original decision boundary:

1. Choose a confirmed, currently adequate camera if any; fixed RGB preference if
   both qualify, preserving the original policy preference as a control.
2. If neither is confirmed and adequate, reconsider a currently adequate recovering
   camera; fixed RGB preference if both qualify. This is the user's “remaining
   sensors insufficient” exception, based on conditional modeled information.
3. If none is currently adequate, abstain. Scarcity does not make bad constraints
   good. Preserve that coverage cost rather than assigning a zero error.

Missing geometry for a supported available motion is an input-contract failure
requiring a new capture, not a zero matrix or an implicit policy fallback.

Controls: unchanged native RGB-only, thermal-only, raw availability advice, existing
history rule, and current-geometry-only selection using the exact same adequacy
criterion without confirmation. The last control isolates the incremental history
component. Native covariance weighting remains active inside both experts.

## Outputs and success criterion

Predict and lock actions without labels. Score in a separate invocation with exact
sample-keyed proxy losses from those same fresh native expert motions. **Do not
reuse historical expert losses for new executions.** Report by recording, with all
windows retained: own-population coverage, exact matched IDs, translation RMSE,
rotation RMSE and abstentions. Preserve per-decision intervals, adequacy and recovery
confirmation counts. Report discarded supported motions, deferred first-recovery
opportunities and ignored motions with lower translation proxy error. These are
opportunity counts, not measured live recovery latency or independent observations.

Strict exploratory diagnostic: on every recording, candidate coverage at least raw,
nonempty pairwise matched populations, strictly lower translation RMSE than every
control and no higher rotation RMSE on each comparison. No pooled win can override
a per-record failure; ties do not pass. This criterion may fail, especially because
adequacy-based abstention costs coverage. No post-outcome relaxation or candidate
promotion. Passing would motivate recording-level external evaluation, not prove
physical improvement or novelty.

## Executable commands and present readiness

From `ore-ms/MAC-VO-main` inside the recovered runtime:

```bash
# Same explicit data flags as shared_arbiter_replay; requires a separate capture freeze.
python -m Scripts.Setup.shared_geometry_replay --dataset DATASET --manifest MANIFEST --output NEW_CAPTURE --frames FRAMES --start START --record-root-initialization --acknowledge-provisional-geometry

# Generate the existing prefix-motion predictions/static contract from that fresh capture.
# Then bind causal geometry to those projections, without reference labels:
python -m Scripts.Setup.prepare_sensor_admission_contract --predictions FRESH_PREFIX_PREDICTIONS --static FRESH_STATIC_CONTRACT --timeline NEW_CAPTURE/timeline.jsonl --recording-id RECORDING --window WINDOW --output admission-inputs.json
python -m Scripts.Setup.sensor_admission_comparison predict --input admission-inputs.json --output locked-actions.json
python -m Scripts.Setup.sensor_admission_comparison score --input locked-actions.json --labels FRESH_MATCHED_PROXY_LABELS --output scores.json
```

Paths in capitals are required artifacts, not files claimed to exist. The existing
prefix-model generator needs its new capture/source pins; historical hardcoded
source plans must not be silently reused. Geometry and expert outputs must first
pass matched flag-off/on native preservation before the empirical comparison.

Actual readiness inspection: all six historical shared timelines exist and have
**zero geometry events**. The historical-input report is saved at
`results/sensor-admission-preparation/run-001/historical-input-readiness.json`.
The new scorer is executable on its defined contract; the actual three-record
scientific comparison is not executed by this preparation. Next necessary work is
the fresh matched capture/input freeze and preservation runs, not another fit or
reserve assignment.
