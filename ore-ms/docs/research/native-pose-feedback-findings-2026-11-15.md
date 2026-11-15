# Native pose feedback: state integration works, accuracy tradeoffs persist

15 November 2026 UTC. Branch `ore-ms-orb-development`, baseline `53afa19d1`
plus source-pinned working tools. Same fixed thermal precision0.1, same exposed
600-group recordings, no new learned inference or training.

## Result and decision

Each estimator arm now carries its own committed float32 native pose forward.
RGB source points are rebuilt at that pose; it initializes the next solve and
places thermal source points into that arm's world frame. Fused estimates no
longer borrow historical RGB parents. State integration and independent auditing
pass, but they do not resolve the previously observed accuracy tradeoff.

Day full-path position RMSE improves **0.910%**, with orientation RMSE worsening
**7.317%**. Rain worsens **7.908%** in position and **2.742%** in orientation.
Night retains missing pair247, so no complete600-pose path is scored.
The same **four of17complete predefined blocks** improve both metrics:
day200/300 and night300/500. Preserve these scoped positive cases alongside all
regressions. They are not newly discovered independent successes or validation
of a general activation rule; the same exposed observations are reused.

| Condition | Method | Position RMSE m | Orientation RMSE deg |
|---|---|---:|---:|
| day | RGB feedback | 11.115217 | 4.036908 |
| day | RGB + thermal feedback | 11.014103 | 4.332279 |
| night | RGB feedback | Incomplete | Incomplete |
| night | RGB + thermal feedback | Incomplete | Incomplete |
| rain | RGB feedback | 64.141241 | 4.346644 |
| rain | RGB + thermal feedback | 69.213445 | 4.465814 |

Day final position error worsens17.656844→17.798951m, despite lower path RMSE.
Rain final position error worsens124.093053→135.123278m. No outcome is omitted.

The fixed-weight recipe now has a qualified native solver and pose-feedback
baseline. Keep it as reusable engineering; do not replace the native RGB default
or tune another scalar from these exposed scores. This closes this feedback
integration check, not thermal research or every possible fusion mechanism.

## All fixed100-group blocks

Percent changes compare thermal feedback with RGB feedback; negative favors
thermal. These same six starts per condition were fixed before prediction.

| Condition | Start | Position RMSE change | Orientation RMSE change |
|---|---:|---:|---:|
| day | 0 | -0.581% | +1.527% |
| day | 100 | -1.209% | +8.039% |
| day | 200 | -1.055% | -1.805% |
| day | 300 | -0.580% | -16.390% |
| day | 400 | -1.006% | +11.003% |
| day | 500 | -1.088% | +12.595% |
| night | 0 | +0.041% | +0.656% |
| night | 100 | -1.594% | +5.195% |
| night | 200 | Incomplete | Incomplete |
| night | 300 | -2.223% | -0.876% |
| night | 400 | -0.070% | +16.448% |
| night | 500 | -0.690% | -0.822% |
| rain | 0 | +9.968% | -4.949% |
| rain | 100 | +4.056% | +3.456% |
| rain | 200 | +0.842% | +1.862% |
| rain | 300 | -1.606% | +3.605% |
| rain | 400 | +16.639% | +0.353% |
| rain | 500 | +10.475% | +6.897% |

No new thresholds, weights, windows, offsets or pair associations were selected
from these results. Adjacent poses and the repeated fixed windows are not
independent scientific trials.

## What the feedback replay implements

Original RGB configurations are explicitly verified for all three conditions:
`StaticMotionModel`, `AllKeyframe`, `CovarianceSanityFilter`, `mapping=false`,
and native disparity PGO. Native keypoint selection takes images/depth/flow, not
pose; the covariance sanity filter uses only covariance validity. With those
fixed accepted measurements, the state-dependent native operations are source
point registration, covariance rotation and static-pose initialization.

`tools/native_pose_feedback.py` performs those operations using native
`pixel2point_NED` and PyPose rigid action. Both RGB source world coordinates and
source covariance are regenerated from each arm's current state. Source covariance
is retained even though the native disparity residual does not use that term.
The actual native disparity graph/Jacobian and solver bodies remain unchanged;
the already qualified thermal graph appends calibrated residuals to the same pose.
Each solved pose is committed to float32 exactly as the native writer does,
without introducing a new normalization policy. That committed pose becomes the
next frame's parent and initialization. Published relative matrices use the
existing normalized-quaternion convention for metric calculation.

Missing thermal inputs use an RGB-only solve at the **fused arm's own state**,
not a copied RGB-control result. Missing RGB intervals hold the internal static
state in both arms while publishing a null increment. The known nighttime gap
is not interpolated or converted to a valid zero-motion measurement. Six block
paths use their own initial gauges; the missing block is retained as incomplete.

3592optimizer calls complete without an exception:1796per arm. Actual thermal
factor additions are595day/558night/598rain; the other45fused solves use RGB alone
at the fused state. Both arms retain599/598/599supported increments. There is no
new availability improvement, live failure-rate measurement or compute-saving claim.

A two-frame solve with a static initializer is geometrically invariant to a
common global-pose change when its source points move consistently. Therefore
similar results to the fixed-parent experiment were expected; a different global
coordinate history is not a new accuracy mechanism. Finite-precision arithmetic
and native damping/acceptance make small numerical differences possible. This
assay verifies the state wiring rather than proving exact solver gauge invariance.

## Qualification, checks and preserved failure

First20RGB increments per condition (60total) pass the original frozen motion
bounds: maximum relative translation difference7.142e-5m and rotation7.345e-7rad
versus original native receipts, below1e-3m/1e-4rad. Rebuilt source covariance is
exactly equal to archived covariance at the historical parent in all60cases.

Qualification001 failed an additional metre-only source-coordinate gate. The
worst inspected difference was one float32 step (0.0078125m) on a point roughly
70954m away. The failed record and numerical analysis are preserved. Qualification002
uses a conservative standard rounding budget, gamma64=64eps32/(1−64eps32), times
1+3maxabs(local point)+maxabs(parent translation). This accounts for fewer than
64scalar float32 backprojection/rigid-action operations; it is not fitted from
pose scores. The largest observed error uses0.594%of that budget. No geometry,
solver, original motion bound or accuracy score changed. This is a qualification
amendment, not a claim that the original absolute source-coordinate gate passed.

Predictions are hashed and locked before new reference scoring. Independent
auditor passes **10884checks**, including input/source hashes, every parent-to-prior-
commit equality, float32 commit representation, direct reconstruction of emitted
relative transforms, held-state/missing-output semantics, and full/block metrics
computed directly from committed global states rather than accumulated deltas.

The RGB feedback control remains close to original native output. Full-path
position RMSE differs by−0.00001298m day and+0.00027657m rain; orientation RMSE
by−0.00003532deg day and+0.00043279deg rain. Small differences include CPU versus
original GPU source-backprojection rounding and accumulated float32 writebacks;
the replay is not byte-identical to the original runtime.

## Scope

This is a sequential native pose/point-registration replay with cached accepted
measurements, not fresh dense neural inference, live processing, map optimization,
pose-dependent candidate reselection or deployment. Its applicability relies on
the verified original configuration; it does not establish that arbitrary VO
frontends are pose independent. Missing physical exposure/clock semantics and
nominal provided-reference limitations remain. No timestamp correction, new
calibration, alignment fit, reserve access or claim of physical ground truth.

The [native-factor report](native-thermal-constraints-findings-2026-11-15.md)
retains solver/prior-art qualification. Generic shared RGB/thermal estimation is
existing work; this replay is application engineering, not MSO reproduction or
a verified novel algorithm. No new paper was read for this state-integration step.

Artifacts: `results/native-pose-feedback/{qualification-001,qualification-002,
config-contract.json,run-001}`. Source: `tools/native_pose_feedback.py` and
`tools/audit_native_pose_feedback.py`. Runtime remains Torch2.7.1+cpu,
PyPose0.6.8, one CPU thread.
