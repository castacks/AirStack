# Fixed thermal increment against the qualified day RGB controls

Use all99intervals of the exposed100frame day `_2021-08-06-11-23-45` prefix.
This isolates the thermal increment after qualification of stronger RGB baselines.
Reuse the three locked own-state RGB streams from optimization-day-transfer,
verified by their prediction/source/input locks. Do not recompute or select RGB
outputs. Add one thermal arm with the existing fixed precision0.1 and the native
thermal-factor graph plus translation-only wrapper, unchanged.

The thermal arm starts at the same source0parent. At each pair it runs an RGB
first solve at its own parent state, then a translation-only solve initialized
at that float32 solution with native RGB residuals plus calibrated thermal rows.
The second pass holds its own first-pass quaternion exactly. Its final float32
pose drives registration at the next pair. Use the unchanged thermal binding,
native precision/loss/solver/stopping settings and ten-step cap. No weighting,
head, timing correction, solver rescue, new inference or training.

Restore all three thermal capture members and verify hashes against the existing
transfer backup. Freeze protocol, source, RGB prediction lock and input/reference
hashes before new predictions. Record both-stage losses, scheduler calls and CPU/
wall time. Preserve any failed binding/solve and stop without deleting intervals;
a partial run cannot qualify a complete-path gain. Lock all99new outputs before
scoring. The supplied references and all recording outcomes are already exposed;
this is a development ablation, not blind testing or physical timing calibration.

Primary endpoint:100pose positionRMSE versus BOTH locked extra-full-pose and
extra-translation RGB controls, with99/99availability. Report orientation separately,
plus interval translation/rotation mean,RMSE,p95 and path mean,RMSE,p95. No alignment,
scale fit or gap fill. Improvement over only native is insufficient for an incremental
thermal claim. Do not tune in response to the result; any orientation cost remains
explicit. Costs include both solver stages and exclude frontend/graph construction.
RGB timings and thermal timings come from separate captures, so comparisons are
descriptive, not randomized same-run or statistically qualified timing evidence.

Independently verify copied RGB byte-level objects/locks, thermal own-state continuity,
first-pass rotation retention, pose composition, cost/loss accounting, support and
all scores. Reject state,rotation,cost,score corruptions. Stop after this prefix;
no automatic longer span, new recording, learned gate or model change follows.
