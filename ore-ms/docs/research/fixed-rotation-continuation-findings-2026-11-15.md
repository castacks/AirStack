# Longer continuation: no added thermal gain over RGB-only refinement

15 November 2026 UTC. Branch `ore-ms-orb-development`, baseline `53afa19d1`
plus source-pinned orchestration. The candidate and predictions remain unchanged.

## Main conclusion

The **qualified night continuation does not preserve the thermal-specific gain**.
Across the next600intervals, position RMSE is33.482921m for native RGB,
31.131610m for RGB-only translation refinement and31.276695m with thermal.
Thermal is **0.466%worse than the matched RGB-only control**, despite being
6.589%better than native. The larger gain is already present in RGB refinement.
Native RGB rotation is unchanged in allarms.

Day cannot support a qualified native-baseline improvement claim: one native
control reproduction exceeds the frozen bound. Its diagnostic new-span results
also favor RGB-only refinement (18.586589m) over thermal (18.770234m,0.988%worse).
The diagnostic native RGB replay is19.815393m. Those day numbers are retained,
not counted as a passing replication or hidden from the negative conclusion.

The earlier100-frame day/night transfer gains remain valid under their stated
scope. They do not establish persistent thermal benefit across later parts of
a recording. This check limits the operating-scope claim; it does not show that
thermal is inherently random, universally harmful or incapable of adding value.
Four ofsixnew night100-group blocks still favor thermal over both controls,
but the primary continuous continuation does not.

## What was extended

The short transfer records have only100native MAC-VO capture groups. Full-record
DROID outputs are a different estimator and were not substituted. Instead, this
assay uses the available full MAC-VO captures from the originalcandidate records:

| Record | Sequence | Full native groups available | New source span | Recorded RGB span |
|---|---|---:|---|---:|
|day|_2021-08-13-16-31-10|5695|599–1199|60.459169s|
|night|_2021-08-13-22-16-02|5234|599–1199|67.992321s|

This is a continuation beyond their original600-group evaluation, **not a longer
run of the two successful short-transfer recordings**. That input limitation and
scope change were communicated before new predictions. The source range and
all six new100-group blocks were fixed before reading new support or scores.
All records were historically development-exposed; no blind validation is claimed.

The original599increments are reused exactly from locked prior predictions.
Each arm's own committed float32 state atsource599 seeds the next600intervals,
599→600 through1198→1199. There is no estimator restart or reference reset in
the prediction stream. Scoring the new601-pose span uses only its initial gauge.
The full1200-pose paths and all six newblocks are secondary endpoints, with
missing estimates preserved.

## All new601-pose span results

| Record | Method | Position RMSE m | Final position error m | Orientation RMSE deg | Eligibility |
|---|---|---:|---:|---:|---|
| day | Native RGB replay | 19.815393 | 31.853273 | 4.982388 | Diagnostic only |
| day | RGB-only refinement | 18.586589 | 29.917747 | 4.982388 | Diagnostic only |
| day | RGB + thermal refinement | 18.770234 | 30.220531 | 4.982388 | Diagnostic only |
| night | Native RGB replay | 33.482921 | 35.047516 | 4.989385 | Qualified |
| night | RGB-only refinement | 31.131610 | 32.543167 | 4.989385 | Qualified |
| night | RGB + thermal refinement | 31.276695 | 32.710111 | 4.989385 | Qualified |

Thermal also loses the new-span final position endpoint to RGB-only refinement
in both records. All600new intervals produce outputs for every arm. All2400
references pass the unchanged proper-rotation serialization screen. There is
no source exclusion, temporal correction, SE3/Sim3 fit, fitted scale or gap fill.

## All six new100-group blocks per record

Position RMSE change for thermal against native RGB replay and RGB-only refinement.
Negative favors thermal. Rotation metrics remain native RGB by construction.

| Record | Block start | vs native RGB | vs RGB-only refinement | Eligibility |
|---|---:|---:|---:|---|
| day | 600 | -2.457% | +1.631% | Diagnostic only |
| day | 700 | +0.374% | -0.556% | Diagnostic only |
| day | 800 | -2.804% | -1.098% | Diagnostic only |
| day | 900 | -3.323% | +0.705% | Diagnostic only |
| day | 1000 | -5.049% | +0.647% | Diagnostic only |
| day | 1100 | -0.065% | -0.065% | Diagnostic only |
| night | 600 | -6.967% | +0.343% | Qualified |
| night | 700 | -2.100% | +0.433% | Qualified |
| night | 800 | -0.287% | -0.286% | Qualified |
| night | 900 | -0.726% | -0.725% | Qualified |
| night | 1000 | -0.399% | -0.404% | Qualified |
| night | 1100 | -0.118% | -0.128% | Qualified |

The successful night blocks are800/900/1000/1100. Thermal loses against RGB-only
refinement in600/700. Blocks are not independent trials and were not selected
for favorable signs. Day qualification fails at the recording level, so even
blocks outside the flagged pair remain diagnostic in this report.

## Full1200-pose context

| Record | Method | Position RMSE m |
|---|---|---:|
| day | Native RGB replay | 39.687251 |
| day | RGB-only refinement | 39.344413 |
| day | RGB + thermal refinement | 39.286650 |
| night | Native RGB replay | Incomplete: pair247 |
| night | RGB-only refinement | Incomplete: pair247 |
| night | RGB + thermal refinement | Incomplete: pair247 |

The day full-path thermal result is numerically lower than both controls, but
is still diagnostic and includes the already scored positiveprefix. It cannot
replace the predeclared new-span endpoint or repair the qualification failure.
Night fullpaths refuse the historical missing pair247. That gap lies outside
the new continuation, which is complete; it was not interpolated or stitched.

## Preserved qualification failure and bounded diagnosis

The frozen native reproduction bounds remain1e-3m translation and1e-4rad rotation.
Day pair1034 differs from the archived native motion by **0.002943945m** and
5.8234e-5rad. The translation bound fails; rotation passes. Other599new day
intervals and all600new night intervals pass. Night's worst difference is
6.2914e-5m and7.5755e-7rad. No tolerance was increased.

The original scorer stopped on day before new reference scoring. An initial
dependent audit was mistakenly invoked after that stop and found no scorefile;
it produced no validation result. A separate evaluation protocol then applies
the **same gate per recording**, retaining failed-day metrics as diagnostic only
while allowing the independent qualified-night result. Original frozen source,
prediction hashes and failure metadata remain unchanged. This is a correction
to evaluation orchestration, not a new passing day result.

Two source-only diagnostic fits at the flagged interval use the archived parent
pose. Archived worldpoints reproduce native within26.98micrometres; CPU-rebuilt
points within27.70micrometres. Their maximum coordinate difference is1.526e-5m.
Thus CPU backprojection alone does not explain the2.94mm feedback-run discrepancy.
The larger change appears when the carried feedback history is used. Its exact
numerical mechanism was not isolated; no offset search, solver change, replacement
prediction or reference-informed rescue was performed. Do not assign this to
thermal/environmental physics—the flagged control is RGB-only.

## Implementation, recovery and audit

All previous candidate implementation hashes match. The same native RGB step,
three-coordinate refinement step, precision0.1, native residual/Jacobian/covariance,
LM/Huber/PINV/trust/scheduler, warm start and float32 writeback are used. Each arm
has its own translation history; constrained rotations follow the current RGB
backbone. This remains cached accepted-measurement replay under static motion,
all-keyframe selection and mapping=false, not new dense-frontend inference.

3600new prediction optimizer calls complete. Day adds thermal factors in600/600
attempts; night593/600, with7RGB-only solves at the thermal arm's own state.
The two isolated diagnosis fits are additional and never replace predictions.
Each arm publishes all600new increments. Prefix payloads are reused exactly.

Recovered2400reference members from the already SHA-verified decisive archive;
each historical member hash is checked. The first1200reference files (600each
record) exactly match the previous600-group reference bytes. New references were
not parsed until prediction lock and per-record evaluation eligibility were fixed.

Independent audit passes **28946structural and eligibility checks**: source/input/
reference/prefix hashes, all reused prefix payloads, exact seed/parent chains,
fixed rotation, held-state semantics, endpoint-derived relative transforms and
full/newspan/block metrics computed directly from committed states. The audit
explicitly records **all_native_reproduction_gates_passed=false**. A passing
structural audit must not be described as both accuracy runs qualifying.

## Decision

Retain the successful short-prefix results and scoped positiveblocks. Do not
promote constant thermal translation refinement as a persistent improvement over
the matched RGB-only refiner. The qualified longer-night endpoint fails that
incremental-benefit test; the long-day diagnostic does not establish a qualified
counterexample or success. RGB-only refinement is the stronger control on the
newspan and should remain visible as its own engineering result.

No new weight/head/selected window or physical timing correction is justified
by this continuation. A future contribution claim must distinguish matched RGB
backend improvement from added sensor benefit and specify its validated scope.
Original source-depth/calibration/clock and nominal-reference limitations remain.
Generic component separation and shared RGB–thermal factors retain their prior-art
status; no new paper reading or novelty claim is made by this validation.

Artifacts: `results/fixed-rotation-continuation/recovery`, `run-001` (frozen
predictions and preserved gate failure) and `evaluation-001` (per-record eligible
scores and independent audit). Tools: `recover_fixed_rotation_continuation.py`,
`fixed_rotation_continuation.py`, `score_qualified_continuation.py`, and
`audit_fixed_rotation_continuation.py`. Runtime remains Torch2.7.1+cpu/PyPose0.6.8.
