# Longer native control: no established pipeline improvement

15 November 2026 UTC. Branch `ore-ms-orb-development`, baseline `53afa19d1`
plus local source. This adds a missing comparator to the completed 600-group
study, not a new fusion method or native execution.

## Finding

Native MAC-VO has better day position and orientation than the conditional RGB
replay and thermal-fused replay. In rain, RGB replay improves position over native,
but thermal addition slightly worsens it again. Rain improvement over native
therefore cannot be credited to thermal. Night has the same unsupported pair247
for all three methods and no continuous full-path score.

| Condition | Method | Position RMSE m | Orientation RMSE deg |
|---|---|---:|---:|
| day | Native MAC-VO | 11.115 | 4.037 |
| day | RGB replay | 12.044 | 6.870 |
| day | RGB + thermal 0.1 | 11.961 | 7.127 |
| night | Native MAC-VO | Incomplete | Incomplete |
| night | RGB replay | Incomplete | Incomplete |
| night | RGB + thermal 0.1 | Incomplete | Incomplete |
| rain | Native MAC-VO | 64.141 | 4.346 |
| rain | RGB replay | 56.367 | 4.917 |
| rain | RGB + thermal 0.1 | 56.605 | 4.931 |

The thermal result's small daytime gain against replay RGB is retained, but it
is insufficient to claim an improved MAC-VO architecture. The replay changes the
backend objective and source-landmark treatment. A same-backend control isolates
thermal within that replay, while the native control measures whether the overall
replacement is competitive. Both comparisons are needed.

## All fixed blocks and coverage

All six predefined100-group blocks per condition remain included. Seventeen have
complete paths. Thermal fusion improves both position and orientation relative
to native in0/6day,2/5night,1/6rain complete blocks. RGB replay alone also improves
both in those same three night/rain blocks. This does not show that thermal adds
nothing within those blocks; it shows that joint improvement over native in
those cases is not exclusive to the thermal arm. Per-block numbers are saved.

Native supported increments:599/599day,598/599night,599/599rain, matching both
replay comparators. Missing/interpolated native inputs are never gap-filled.
These are counts of saved supported receipts, not live failure rates or inference
availability measurements. No compute comparison is possible from this scoring.

## Construction and verification

Original first-write RGB motion receipts are bound to the exact source IDs and
recorded endpoint timestamps. Supported deltas are independently reconstructed
as inverse(parent pose) times committed pose and compared within1e-8. A fixed
NED-to-optical permutation matches the already qualified100-group comparator;
no learned/fitted axis, scale, time or alignment is introduced. Predictions are
hashed and locked before reference scoring. All1796supported deltas pass the
reconstruction check. Previous replay prediction hashes remain unchanged.

The existing qualified path scorer composes deltas in one initial identity gauge,
with no SE3/Sim3 fit, and refuses paths with missing increments. Rain first100
native position RMSE, orientation RMSE and final position error reproduce the
previous100-group control within1e-8. The same supplied RGB references and all
measurement-time limitations apply. This is an already exposed cohort, not a
fresh test or independently certified physical ground truth. Saved first-write
deltas do not establish new causal full-pipeline runtime or reproduce all later
map revisions.

## Engineering consequence

Keep native MAC-VO as the application baseline. Do not integrate the current
conditional fused replay as a validated replacement or launch another weighting
head. Any next thermal integration must retain the native RGB estimator as an
explicit control and separate changes to its backend from the thermal input.
The strongest remaining practical question is whether thermal can add useful
constraints while preserving the existing native RGB accuracy; this check does
not implement or validate that mechanism. Generic shared-state fusion already
has prior art, so such integration would first be an application baseline rather
than a novelty claim.

Artifacts: `results/longer-native-control/run-001/{prediction-lock,scores}.json`
and three prediction files. Tool: `tools/longer_native_control.py`.
