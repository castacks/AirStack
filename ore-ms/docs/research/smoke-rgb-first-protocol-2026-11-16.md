# Fixed RGB-first / thermal-on-missing-output replay

16 November 2026. Existing-baseline characterization on exposed SmokeRoom caches.
No new inference, training, threshold search, fitted timing/extrinsics or stateful
VO run. The prior pose-error results are known. The fixed rule below precedes
its replay; no reference error or future interval is a selection input.

## Rule, outputs and populations

For each original anchor, request RGB. If its frozen status is `ok`, retain that
proposal unchanged. Otherwise, request thermal if its original image input is
qualified; retain thermal only if its frozen status is `ok`. Otherwise return no
proposal. This neither inserts identity nor rescues rejected finite rotations.
The `ok` gate is the parent solver's ≥8 cheirality-inlier validity gate. Reuse the
existing pure fixed-preference helper, without editing its frozen implementation.
Compare fixed RGB, fixed thermal and RGB-first on all 838 declared anchors, for
SIFT and learned frontends separately. Preserve every failure and static segment.

Before errors are joined, lock all choices and explicit source/target timestamps,
selected sensor and output-frame metadata. Re-express selected rotation in IMU
axes using the published effective camera→IMU rotation: R_I=R_IC R_C R_IC^T.
Keep unit translation direction tagged to the selected target camera. A monocular
unit translation cannot supply the metric lever-arm correction required to move
translation to the IMU origin. Do not splice these proposals into a common-body
SE(3) trajectory or claim a complete VO fallback. Different selected camera
exposure times remain explicit; no resampling to common timestamps is invented.

Use the parent's frozen 837 common-reference anchors, 483 paired translation≥1cm
anchors and the same 236 anchors where both reference rotations are≥2°. Preserve
full recording, fixed thirds and rejected outputs. Reference labels define
reporting strata only, not the policy. The supplied/fused-reference, nominal
sensor-origin, time and exposed-data limitations all remain.

## Endpoints and costs

Primary: full relative R≤1° success on the 236 paired-moving anchors, counting
missing outputs as failures. Secondary: R≤5°, nominal joint R≤5°/directed-t≤15°
on the 483 translation-qualified anchors, and the original whole/fixed-third
populations. Score chosen proposals using their own calibrated sensor frame/time,
not another camera's error label. Verify invariance of re-expressed rotation error.

Report additions to RGB: valid thermal outputs, newly correct outputs at each
endpoint, newly incorrect outputs, and attempts producing no output. RGB-first
never replaces an accepted RGB pose, so its unconditional success count cannot
decrease by construction. This is not proof of safe persistent-state behavior:
a newly wrong output can be worse than abstention in an actual estimator.
Conditional precision, missing counts and additional wrong estimates must remain.
Compare the per-endpoint hindsight union only as an upper bound; it is not a
reference-informed deployment policy.

Cost primary: exact counts of original pair-level matcher calls. A call includes
both temporal directions in the learned frontend, as in the parent. All RGB calls
are counted; thermal is counted for every requested, input-supported attempt,
even when it returns no valid pose. Report unsupported input separately.

Secondary cost is a **projection from measured parent per-pair seconds**:
RGB time plus requested thermal time. It excludes startup/resident-model memory,
sensor power, routing overhead and stateful tracking; it is not a new latency
benchmark. SIFT descriptor-cache reuse would change with a lazy schedule, so its
timing sum is especially approximate. Learned calls are independent per pair but
GPU state/scheduling/startup are still not remeasured. No real-time speedup or
end-to-end energy/latency claim is permitted from this accounting.

## Qualification, audit and stopping

Preflight every cached pair/sensor and all frame/time/status/timing bindings,
score/population/normalization pins and completeness. Synthetic truth-table and
metamorphic checks must show that accepted RGB is never replaced, thermal errors
cannot influence choices, missing input is not called, and neither output yields
abstention. Freeze source/protocol/inputs, lock choices, then join scores.
An independent auditor reconstructs selection/costs, checks camera→IMU rotation
conjugation and chosen-error invariance, and recomputes every endpoint/stratum.
Stop after this replay. No new gate, model, threshold or window rescue follows;
this establishes a finite baseline, not a new selection method or held-out gain.
