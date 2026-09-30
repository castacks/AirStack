# Change note: information-aware abstraction search (`PlannerParams::infoAware`)

**Package:** `cpp_planner/` (namespace `mtl`). **Date:** 2026-09-29. **Origin:** written in the
AirStack vendored copy (`mtl_search_planner/third_party/mtl_planner`), offered upstream as
`third_party/info_aware_upstream.patch`.

**Opt-in. With `infoAware.enabled = false` (the default) every plan is bit-identical to before:**
the only code path that changes is behind that flag, and the existing five test suites are
untouched and pass.

## Why

The plain pipeline commits to one abstraction, k-means clusters of radius `maxClusterRadius`,
and optimises the route against its own proxy: the mass of the cells the boresight is aimed at.
On a peaked prior both cost detection:

* a proximity cluster mixes a peak's dense core with its thin tail, so the route can take both
  or neither, and flights spend their second half at a peak collecting about half the belief per
  km of the first half;
* "aimed at" is not "detected". Centres aimed at past the detection range (`sensor.beta`), cells
  swept twice, and everything the footprint sees in transit are all valued wrongly, so a plan
  that is better on the proxy can search less. The planner's info mass and the residual belief
  disagree on which of two plans is better.

## What

| New code | Purpose |
|---|---|
| `mapping/peak_clusters.{hpp,cpp}` | `findPeakBasins` (discrete watershed of cell mass on the cell lattice, persistence merging); `clusterByPeaks` (basin -> cumulative-mass levels -> split under the radius); helpers `splitUnderRadius`, `clusterSetFromMembers` |
| `planning/coverage_score.{hpp,cpp}` | `CoverageModel`: the residual-belief metric computed from the cells (each cell spread over a sub-lattice), same footprint and sigmoid as `eval::computeResidualBelief`. Lives in `mtl_planner`, so no link to `mtl_eval` |
| `planning/info_aware.{hpp,cpp}` | `planInfoAware`: candidate abstractions x seeds, all scored with the coverage model, then PEEL / SPLIT / MERGE moves; `detectionReach` |
| `ClusterSet::basin / level / parent` | the hierarchy of a peak abstraction (empty for k-means) |
| `PlanningResult::infoAware` (`InfoAwareReport`) | every candidate's label, reach, coverage score, info mass, flown length; the chosen one and the plain plan's score |
| `PlannerParams::infoAware` (`InfoAwareParams`) | `enabled`, `levelSets`, `persistence`, `reachScales`, `slantMargin`, `capGimbalToDetection`, `subsample`, `lookStride`, `restarts`, `maxMoves`, `splitMerge`, `peelKeep`, `threads`; validated when enabled |
| `Planner::planFromCells` | dispatches to `planInfoAware` when enabled (so `plan()` does too); `planFromClusters` unchanged |
| `apps/mtl_plan_json.cpp` | reads the optional scenario block `info_aware` (same keys, snake_case) |
| `tests/test_info_aware.cpp` | the sixth ctest suite |
| `CMakeLists.txt` | the three sources, the test, `Threads::Threads` (candidates are planned on worker threads) |

Every candidate is an ordinary `planFromClusters` plan, so the budget guarantee (measured on the
flown arc), the gimbal schedule and the geometry audit are exactly the plain planner's. The
result does not depend on the thread count.

## Verification

* `ctest`: 6/6. `test_info_aware` also passes under `-fsanitize=address,undefined` and
  `-fsanitize=thread`.
* The coverage model agrees with `computeResidualBelief` to about 0.01 on the test scenario
  (0.557 vs 0.564 detected).
* Offline, on the AirStack `mtl_search_small` scenario (1000 s budget, one agent), planned
  residual 0.423 -> 0.347 (the flown TIGRIS baseline was 0.428); in a closed-loop kinematic
  rehearsal, flown 0.338 vs 0.428 planned for the plain planner. The benchmark over 216 varied
  priors is in the AirStack run notes.
* Cost: 8 abstractions x (1 + `restarts`) plans + up to `maxMoves` re-plans, each a plain plan
  (0.3-1.5 s at 1000-2500 s budgets on this scenario), spread over `threads` workers.
