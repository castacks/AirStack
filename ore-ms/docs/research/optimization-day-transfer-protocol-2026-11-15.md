# Unchanged RGB optimization control on the day transfer prefix

Test all99increments of the existing100frame `_2021-08-06-11-23-45` capture.
This historically exposed recording differs from the night development recording;
it is not a blind test or long-run transfer benchmark. No outcome-based interval
selection, solver tuning, thermal factors, new inference or training.

Use the same three own-state feedback arms: native, native plus full-pose second
solve, native plus translation-only second solve. Initialize all at the recorded
source0native parent. Reuse native graph, feedback registration, timed optimizer
and constrained wrapper unchanged. Every first and second solve retains the
existing ten-step cap and stopping/loss/precision settings. Each final float32
state seeds the next interval. Rotation retention means each constrained arm's
own first-pass quaternion, not the independent native rotation stream.

Verify recovered RGB capture/source/config/reference hashes against the earlier
transfer backup manifest. Check source timestamps and static/all-keyframe,
mapping-disabled disparity-graph configuration before capture. Freeze protocol,
source/input/reference hashes and initial state before any new predictions.
Reuse the qualified unchanged solver/wrapper. All99native deltas must reproduce
within1mm/1e-4rad. Preserve any failure; no exclusion, restart, threshold waiver or
accuracy promotion on an unqualified native baseline. Failed capture remains partial.

Rotate arm execution order. Record both-stage CPU/wall costs, losses and outer
scheduler calls. Lock all predictions before scoring. Report99pair availability,
interval T/R mean/RMSE/p95 and100pose path position/orientation mean/RMSE/p95,
from identity/reference initial gauge without scale/alignment or gap fill.
Independently check continuity, quaternion retention, pose-delta composition,
reproduction gates, all score statistics and cost accounting from saved outputs;
reject deliberate state/cost/score corruptions. Retain audit errors without rescue.

Stop after this prefix. Mixed metric results remain mixed. Do not generalize this
approximately10second exposed prefix to long-run/live/full-frontend performance,
physical calibration or novelty. CPU measurements are one cached capture, not
end-to-end timing or repeated statistics. Another recording or longer validation
requires a separate finite protocol; no automatic thermal gate or tuning follows.
