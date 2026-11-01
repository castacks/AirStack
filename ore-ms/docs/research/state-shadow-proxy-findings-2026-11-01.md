# Locked state-shadow conditional proxy evaluation

2026-11-01 America/New_York; branch `ore_ms`; declaration `de51e50b3`, using
unchanged shadow actions from `0a520d92e` and existing EXP088/089 interval model.

The shadow boundary lowers matched night proxy errors versus raw, entirely by
replacing the single previously known RGB recovery. It equals history on their
matched support and equals thermal-only on this night window. Day matched
errors are unchanged. Witnessed initialization adds one scored interval per
case versus history; this is a coverage distinction, not evidence of generally
better quality selection. Production remains unchanged.

## Frozen method and recovered references

The [declaration](state-shadow-proxy-protocol-2026-11-01.md) preceded new reference
restoration and scoring. Keep all236locked decisions /472camera slots across
EXP092day and EXP093middle-night flag-off/on captures. Repeated runs of two
already-inspected development windows are not independent validation scenes.
The known night failure informed the boundary design.

Verified both original archive hashes and1,717archive-member hashes
(EXP0901,016; EXP085701). Restored only nine existing model/reference/proof JSONs.
Day references are the independently audited EXP090day-original recorded-SLAM
deltas and nominal transforms. Night references are EXP085original-arm common
reference poses and nominal transforms. All native/common timestamp IDs and
original source rows match the new captures. No direction, scale, frame, clock
or alignment was chosen by new score agreement. Historical estimator outputs
are not assumed numerically identical to the new capture outputs.

A sanitized predictor contract contains only static nominal body-from-camera
matrices; the reference contract is separate. Lock640source, action, reference,
proof and script hashes before projection. Reuse unchanged full-SE3 stationary-
twist projection with the original one-native-duration horizon and RGB target
interval. Only each original received prefix supplies native expert motion.
Original raw/history/shadow actions are not changed; fixed RGB/thermal controls
are derived from original expert availability. Invalid projections abstain with
no expert substitution.

All236predictions are locked before scoring;270of472expert projections are
available. A label-read guard forbids semantic reference-file reads during
projection; injecting corrupted extra label fields into the static contract
leaves all predictions identical. All50existing prefix/interval-model tests pass.
No new neural inference, training, tuning, image decoding, reserve access or
production change. Model/policy sources and locked advice remain unchanged.

## Coverage, including abstentions

Every target has an archived common reference. There are no new projection
abstentions or missing-reference omissions; score support equals locked model
availability. All236rows and original/model abstention/status fields remain.

| Case | Decisions | Raw scored | History scored | Shadow scored | RGB-only scored | Thermal-only scored |
|---|---:|---:|---:|---:|---:|---:|
| Day control |19|18|17|17|18|18|
| Day witness |19|18|17|18|18|18|
| Night control |99|98|97|97|1|98|
| Night witness |99|98|97|98|1|98|

The shadow totals230scored/6abstentions versus raw232/4 and history228/8.
Those aggregate counts describe repeated runs only. Missing control root
records cost one initial shadow abstention in each control capture. Witnesses
restore that evidence, avoiding history's fixed cold-start abstention. Every
terminal pre-drain abstention is retained. RGB-only night support is one pair;
its own error cannot be compared to full thermal support as a camera ranking.

## Exact-matched conditional proxy errors

Reference target: nominal recorded-SLAM body motion over each RGB interval.
Error=inv(reference_delta)@predicted_delta; translation norm in metres and
SO3geodesic magnitude in degrees. No alignment/scale/time fitting or clipping.

| Raw versus shadow matched case | Intervals | Raw translation RMSE m | Shadow translation RMSE m | Raw rotation RMSE deg | Shadow rotation RMSE deg |
|---|---:|---:|---:|---:|---:|
| Day control |17|0.031772|0.031772|0.104335|0.104335|
| Day witness |18|0.030958|0.030958|0.103875|0.103875|
| Night control |97|0.110529|0.043138|5.142568|0.705300|
| Night witness |98|0.109979|0.042956|5.116296|0.701928|

History and shadow select the identical expert on all of their matched intervals:
17day-control,17day-witness,97night-control and97night-witness. Their errors are
therefore identical on each matched population. Common raw/history/shadow
cohorts have these same17/17/97/97intervals and are separately reported.

On night witness, shadow and thermal-only are identical on all98scored intervals:
translation RMSE0.042956m and rotation RMSE0.701928deg. Night control shadow
matches thermal-only on its97matched intervals. Day shadow matches fixed RGB;
thermal-only has0.033067m/0.406010deg RMSE on all18day-witness intervals versus
RGB/shadow0.030958m/0.103875deg. These are condition-specific, support-qualified
recorded-SLAM proxy observations, not independent physical camera accuracy.

All10pairwise comparisons per case retain exact pair IDs and both metrics'
mean,median,RMSE,max. Method-own support and exact-matched support are stored
separately. For example, day-control raw's own18interval RMSE0.030958m is not
the17interval matched raw RMSE0.031772m; excluding the initial interval changes
the denominator, not the estimate on the shared intervals.

## Where the night gain comes from

The same known pair96accounts for100%of matched raw-to-shadow squared-error
reduction in each night arm, for both translation and rotation:
1.004516682998369m² and2517.010196642615deg². All other matched raw/shadow choices
and losses are identical. The two arms repeat that same development event; do
not count them as two independently discovered improvements. Day matched gains
and all shadow/history matched gains are zero; corresponding fraction fields
are null rather than inventing a percentage.

This reproduces the known outlier mechanism already traced before the method
was declared. It does not show a new general quality advantage over history,
thermal-only or unseen conditions. The witness coverage difference and the
known-recovery action change must remain separate from a transfer claim.

## Independent validation and recorded audit amendment

The auditor imports no predictor, production model, scorer or policy code. It
reconstructs source quaternion matrices, nominal frame conjugation, native/
common intervals, original receipts and270SE3logm/expm projections. It reads
original authenticated reference exports independently of the scoring contract,
checks reference bindings, computes translation difference norms and rotation
atan2geodesics, and recomputes all losses/coverage/summary/matched/gain fields.

49,569recursive type/value/schema comparisons pass. Maximum independently
reconstructed model entry difference4.44e-15; maximum numeric output difference
2.73e-12. Nineteen semantic corruptions reject wrong losses/rotation/actions,
reference/receipt/matrix/source, missing row/group/comparison/matched ID, changed
coverage/RMSE/gain share, zero-filled abstention and false physical/fit/reference-
feature flags. Key reordering passes. All640original locks still match.
Status: `PASS_INDEPENDENT_SHADOW_PROXY`.

The first audit stopped because absent pair96subtotals in the day exports were
JSON integer0 from sum(empty), while the independent fsum(empty) gave float0.0.
A diagnosis isolated eight fields with that exact zero-representation mismatch.
Original auditor and failed log are preserved. A separately frozen amendment
binds auditor_v2 plus the unchanged prediction/score hashes; v2 handles absent
known-pair subtotal as integer0. No numeric tolerance, score, model, action,
cohort or acceptance metric changed. Strict types for all other fields remain.
These are arithmetic/provenance bookkeeping checks, not independent physical
observations or authenticated rig/clock/training membership.

## Decision and recovery

Keep the boundary opt-in shadow-only and preserve the raw/history/thermal
controls. The completed result is a known-development diagnostic with a fully
concentrated benefit, not evidence for a default switch or a larger fitted gate.
Further threshold/model sweeps on these windows would not provide independent
validation. The next useful step is a prospective recording-level evaluation
contract, with exposure/training-membership accounting, source-backed frame/time/
reference scope and untouched test criteria frozen before new outcomes or
reserve access. Data eligibility and provenance must be explicit rather than
asserted from a new alias or clip offset.

Artifacts: `results/state-shadow-proxy/run-001/` contains the declaration,
640-file protocol, sanitized static/reference contracts, locked270expert matrices
across236rows, prediction label-exclusion proof, full per-case/per-pair scores
and matched populations, audit amendment and independent validation. Recovery
includes frozen standalone scripts, source exports/proofs, model-test results
and both audit attempts. No checkpoints, runtime binaries, original data images
or new model/estimator outputs are required for this CPU metadata replay.

Recovery: `ore_ms:ore-ms/backups/2026-11-01-state-shadow-proxy`.
