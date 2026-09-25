# mtl — Multi-Agent, Endurance-Budgeted Target Localization

A C++17 port of the MATLAB pipeline in `main_new.m` / `init_params.m` / `functions/`,
restructured as an **integrable package**: the planner is one object you construct with a
parameter set and call, and scenario generation and scoring are separate libraries you can
leave out entirely when your simulation brings its own.

---

## What it does

Given a prior belief over a search area and a team of fixed-wing aircraft with an endurance
budget, decide **where each aircraft flies**, **in what order**, and **where the gimballed
camera points at every time step**, so that the prior belief actually swept by the sensor is
maximal and each aircraft's flown arc fits its budget.

The chain, and the one idea behind each link:

| Stage | What it does | Why |
|---|---|---|
| `extractValidCells` | dice the belief into blocks, keep those whose belief **mass** exceeds `minimumBeliefMass` | reduces a grid to a few hundred points to aim at, each carrying its mass — the probability the target is in it (the prior sums to 1), which is additive over cells |
| `clusterCells` | raise K until every cell is within `maxClusterRadius` of its centroid | that radius is the guarantee the gimbal sweep of a cluster covers the cells the cluster owns |
| `computeClusterRewards` | lift per-cell mass onto the centroids | the route problem goes from hundreds of nodes to tens; **exact in reward**, approximate in cost |
| `solveBudgetedOrienteering` | choose which clusters to visit and in what order, together | truncating a TSP tour fixes the order first and then cuts — the order that is optimal for visiting everything is a poor prefix |
| `planBudgetedMacroRoute` | shrink the Euclidean budget until the **measured Dubins arc** fits | the solver reasons in straight lines; the aircraft flies arcs 0–40% longer, route-dependently |
| `refineWithCellAnchors` | spend the leftover crumbs on individual cells | clusters are all-or-nothing; gimbal-aware, so an anchor is priced as a one-off cluster over the detour that brings its **reach ball** into view |
| `allocateBudgetedTeam` | re-offer unreached clusters to agents with slack | k-means balances cluster count, not information per metre |
| `generateTrajectories` | Dubins track + advisory micro-TSP sweep, time-parameterised | everything downstream is parameterised by arc length |
| `extendTrajForLateralCoverage` | keep flying straight past the end of the route, a set distance | a 1-DOF gimbal can only see a cell as it crosses the swept line, and the route stops dead at the last centroid; the stranded cells lie **ahead of the last leg**, so flying on carries them past it. A fixed length keeps the extension's cost off the route's budget |
| `optimizeDroneSensorTraj` | schedule which pass each centre is served on | with one gimbal axis the *instant* is handed over by geometry; the freedom is *which crossing* plus a few degrees of pitch |
| `computeAirframeRPY` | read attitude off the trajectory | with a multi-axis gimbal there is nothing to schedule |

The budget is a **hard guarantee**, not an estimate: it is enforced on the trajectory that
comes out, not the plan that went in (see *The budget is measured, not estimated* below).

### The prior is a probability mass function

`mapgen::generateBeliefMap` normalises the prior so it **sums to 1 over the whole grid**:
`values(r, c)` is the probability that the target is in that pixel, and the bump heights,
cap and floor in `BeliefMapParams` only shape it. Every mass downstream — a cell's, a
cluster's reward, a route's information, the residual after the search — is therefore a
probability. `extractValidCells` and `eval::computeResidualBelief` normalise a host grid that
does not sum to 1 themselves, so a host may pass any non-negative field and
`minimumBeliefMass` still means the same thing.

---

## Layout

```
include/mtl/
  types.hpp                  value types shared by every library (no algorithms)
  params.hpp                 EVERY tunable, in structs — the port of init_params.m
  planner.hpp                the Planner class: the integration surface
  core/       numeric, dubins, kmeans
  mapping/    cells                  (extract, cluster, reward)
  routing/    tsp                    (macro TSP, micro sweep, sensor path)
  planning/   info_score, orienteering, macro_route, cell_anchors,
              agent_sortie, team_allocation
  trajectory/ trajectory_gen, lateral_coverage
  sensing/    abeam, airframe, gimbal_scheduler
  mapgen/     scenario               → library mtl_mapgen
  eval/       detection, geometry_audit, report   → library mtl_eval
src/          mirrors include/
apps/         demo_pipeline.cpp      the C++ equivalent of main_new.m
tests/        five suites, run by ctest, no external framework
```

### Three libraries, on purpose

| Target | Contents | Depends on |
|---|---|---|
| **`mtl::planner`** | the whole planning chain | Eigen only |
| `mtl::mapgen` | Gaussian prior (normalised) + ground-truth targets | `mtl::planner` |
| `mtl::eval` | detection physics, residual belief, geometry audit, reports | `mtl::planner` |

A host simulation normally has its own prior and its own sensor model. Link `mtl::planner`
alone and neither of the other two is compiled into your binary. The planner's own objective
is *"was the sensor pointed at this cell"* — modular, cheap, and exactly what makes the
budgeted route an orienteering problem. The detection physics belongs to whoever is judging
the run, which is why it lives in `mtl_eval`.

---

## Build

Requires CMake ≥ 3.16, a C++17 compiler, and Eigen ≥ 3.3. Eigen is found if installed and
**fetched automatically** if not, so no manual setup is needed.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/mtl_demo                      # the reference scenario
./build/mtl_demo --help
```

Options: `MTL_BUILD_APPS`, `MTL_BUILD_TESTS`, `MTL_ENABLE_WARNINGS`, `MTL_ENABLE_OPENMP`.

### Installing and consuming

```bash
cmake --install build --prefix /opt/mtl
```

```cmake
find_package(mtl_planner REQUIRED)
target_link_libraries(my_simulation PRIVATE mtl::planner)
# and, only if you want them:
# target_link_libraries(my_simulation PRIVATE mtl::mapgen mtl::eval)
```

Or vendor it directly:

```cmake
add_subdirectory(third_party/mtl_planner)
target_link_libraries(my_simulation PRIVATE mtl::planner)
```

---

## Integrating

```cpp
#include "mtl/planner.hpp"

mtl::PlannerParams params;            // every tunable, with the reference defaults
params.numAgents        = 4;
params.maxFlightTime    = 400.0;      // [s] per agent; Inf for unlimited
params.droneAltitude    = 300.0;
params.sensorTiltAngle  = mtl::deg2rad(50);
params.budget.orienteering.nStarts = 12;

mtl::Planner planner(params);         // validates and finalises once, here

const std::vector<mtl::Vec2> starts = {{2000, 2000}, {2001, 2001}, {2002, 2002}, {2003, 2003}};
const mtl::PlanningResult r = planner.plan(belief, starts);

for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
    const mtl::AgentTrajectory& t = r.trajectories[a];
    for (mtl::Index k = 0; k < r.numSteps(); ++k) {
        // t.drone.row(k)  = [x y h]           aircraft state
        // t.sensor.row(k) = [x y]             boresight ground point
        // t.rpy.row(k)    = [roll pitch yaw]  airframe attitude, radians
    }
}
```

Every agent is padded onto **one** timeline (`r.timeVec`), holding its final state after it
lands — so a host stepping all agents together never sees a ragged result.

### Three entry points

| Call | Use when |
|---|---|
| `plan(belief, starts)` | you have a belief grid and want the planner to do the mapping |
| `planFromCells(cells, starts)` | your simulation has its own mapping stack; supply cell centres and masses (set every mass to 1 to maximise cell **count** instead) |
| `planFromClusters(cells, clusters, starts)` | you also want to control the macro-clustering |

`setParams()` retunes between calls; the derived quantities are re-resolved each time.

### What comes back

`PlanningResult` carries the flight-ready trajectories, and the full audit trail: which
clusters were reached and which the budget refused, which cells were serviced and which
dropped, how much information the plan collected versus how much the gimbal actually
realised, plus per-agent `GimbalDiagnostics` naming *why* each unobserved centre was missed —
`NeverOnLine` (fix the ground track), `OutOfReach` (needs altitude or tilt), `DoubleBooked`
(a genuine scheduling loss).

---

## Parameters

Everything lives in `mtl/params.hpp`, grouped by the stage that reads it, and every field
carries the comment that explains what moving it costs. The constructor calls `finalize()`
and `validate()` once:

* **`finalize()`** resolves derived quantities — `budgetDist()`, `sensorStandOff()`,
  `extendForLateralCoverage()` — and pushes shared values (`dt`, the turn radius, the mount
  tilt, the FOV, verbosity) down into the sub-structs, so two stages cannot disagree about a
  value that has only one meaning.
* **`validate()`** throws `std::invalid_argument` on a set that cannot produce a
  trajectory — a tilt past the look-angle stop, a zero budget, fewer launch points than
  agents. The failure lands at construction, not three minutes into a run.

The knobs worth knowing first:

| Parameter | Effect |
|---|---|
| `maxFlightTime` / `maxFlightDistance` | the endurance budget; the tighter one binds. `Inf` on both gives the full-coverage pipeline (a plain TSP over every cluster). |
| `singleAxisGimbal` | `true` runs the whole abeam/scheduling machinery; `false` flies the nominal trajectory and reads attitude off it. |
| `sensorTiltAngle` | stands the swept line off `h·tan(tilt)` **ahead**, for free — a bracket is a mount, not a manoeuvre. Costs slant range as `1/cos` and cross-track reach as `cos`. |
| `maxClusterRadius` | the coarseness of the whole abstraction: bigger means fewer, fatter clusters and a cheaper but blunter route problem. |
| `minimumBeliefMass` | how much of the map is worth visiting at all: a 200 m cell is kept when the probability of the target being in it exceeds this (default `5e-5`, ~400–490 cells holding ~99.8% of the reference prior). Lower keeps many more cells and a much longer sortie; it scales with `targetCellSize²`. Replaces the old `meanInformationThresh` (a mean-belief-per-cell test); the JSON key is `mapping.minimum_belief_mass`. |
| `budget.orienteering.nPerturb` | left unset keeps the solver's adaptive thinning above 60 candidate clusters; setting it explicitly **disables** that thinning, because an explicit value is treated as a deliberate choice. |

---

## Scoring a plan: residual belief

`mtl::eval::computeResidualBelief` is the metric to compare planners on. Once the search is
flown it Bayes-updates **every pixel** of the prior (1 m on the reference map) against every
look every agent took, using exactly the detection model `updateTargetDetectionProbs` applies
to the targets (the sigmoid in slant range inside the footprint, `pOutOfRangeMulti` past
`beta`, nothing outside the footprint):

```
residual(x)  = prior(x) · Π_looks (1 − P(Z|x))   = P(target at x AND every look missed it)
residualMass = Σ_x residual(x)                    = P(the search missed the target)
```

**Lower is better** — 1 for a search that looked at nothing, falling toward 0 as more of the
belief is seen, and seen well. Unlike the planner's own information score (was the boresight
aimed at a cell centre), it credits everything the footprint actually swept, discounted by
how well it was seen, so it can compare planners that do not share the cell abstraction.

```cpp
#include "mtl/eval/detection.hpp"
#include "mtl/eval/report.hpp"

const mtl::eval::ResidualBelief rb =
    mtl::eval::computeResidualBelief(belief, result.trajectories, params.fov, params.sensor);
mtl::eval::reportResidualBelief(std::cout, rb);   // prints rb.residualMass
const mtl::MatX post = rb.posterior();            // Bayes posterior given no detection
```

`mtl_demo` prints it after the detection summary, and with `--csv DIR` also writes
`residual_belief.csv` (prior and residual summed into 10×10-pixel blocks, `--residual-block N`;
`--no-residual` skips the pass). At a target's pixel `residual/prior` equals that target's
`pMissTotal` — asserted by `test_pipeline` — and the MATLAB `computeResidualBelief.m` agrees
with this port to 1e-12 on identical inputs.

Cost: each look only touches its footprint, identical consecutive states (hover padding, a
grounded agent) are integrated once, and `log(1 − P)` is tabulated in the squared slant range.
On the reference scenario (5001×5001 grid, ~7400 looks) that is ~12 s on one core; configure
with `-DMTL_ENABLE_OPENMP=ON` to split each footprint across threads.

---

## Design notes

### The budget is measured, not estimated

Three separate mechanisms, because the cost enters at three different places:

1. **Dubins re-costing** (`planBudgetedMacroRoute`) — the orienteering solver is metric-
   Euclidean so that 2-opt stays valid; the loop around it draws the real arc, measures it,
   and shrinks the budget handed to the solver until the arc fits. Converges in 3–5 passes.
2. **Re-measurement per anchor** (`refineWithCellAnchors`) — candidates are *ranked* on a
   scaled Euclidean detour (fast), but every accepted insertion is re-measured, and one that
   busts the budget is rolled back. The detour it was refused at becomes that candidate's
   ceiling, so only strictly cheaper rungs of the stand-off ladder are retried.
3. **Reserve bisection** (`planAgentSortie`) — the lateral-coverage extension's cost is
   decided by the cell layout, not by the route planner, so it is charged to a reserve. That
   cost is *discontinuous* in the reserve (shorten the route enough and the extension is not
   needed at all, so the measurement reads zero, the reserve is dropped, the longer route
   brings the extension back — oscillating forever). But feasibility is **monotone** in the
   reserve, so the right move is a bisection for the smallest reserve that still comes out
   feasible.

If an agent still cannot be planned inside its budget, the planner keeps the best feasible
plan it found and warns, naming the two knobs that help (`budget.maxOuterIter`,
`budget.reserveFrac0`). This happens when a single sweep lane costs a large fraction of a
tight budget.

### One solver, both mounts

Tilting the camera bracket is not a different scheduling problem — it is one extra term in
the geometry. The bracket rotates about the same axis as pitch and sits outboard of the
gimbal joint, so the two rotations simply add, and a tilted mount is algebraically identical
to flying a nadir camera at a constant pitch of `−tau`. Every formula is the nadir one with
the pitch replaced by the effective look angle `theta = tau − pitch`, and `tau = 0` collapses
each back exactly.

### The geometry audit

`eval::verifySensorGeometry` re-derives the sensor geometry **from the published outputs
alone** and checks five things, including the counter-test that roll cannot move the look
point forward (if it could, the whole single-axis argument would be wrong). On the reference
scenario all three agents pass at ~1e-12 m. The audit is what makes the scheduler's claims
falsifiable rather than assumed, and `tests/test_geometry.cpp` and `tests/test_pipeline.cpp`
both run it.

---

## Tests

```bash
ctest --test-dir build --output-on-failure
```

| Suite | Covers |
|---|---|
| `test_dubins` | all six word types, endpoint-pose exactness, the `pi·R` U-turn bound, monotone arc length, and that the bisector heading rule removes the 360° loop at a hairpin |
| `test_kmeans` | blob recovery, k clamping, and the radius guarantee `clusterCells` must uphold for the gimbal argument to hold |
| `test_orienteering` | budget never exceeded, reward monotone in the budget, unreachable nodes pruned, the Dubins re-costing loop, free-harvest anchoring |
| `test_geometry` | the abeam gate (including the tilt shifting it), level cruise being exactly level, banking into a turn, the full audit passing at three mount tilts, and the reach repair loop recovering out-of-reach centres without breaking the geometry or the timeline |
| `test_pipeline` | end to end in every mode; the prior summing to 1, mass-threshold cell extraction (and its invariance to an un-normalised host grid), the budget invariant measured on the flown arc, exact cell/cluster bookkeeping, the residual belief (whole prior with no search, never above the prior, and equal to each target's miss probability at its pixel), determinism, and construction-time rejection of bad parameters |

All five pass clean under `-fsanitize=address,undefined`, as do the single-axis, multi-axis,
grounded-agent and empty-cell-set paths.

---

## Differences from the MATLAB original

* **Dubins paths are implemented here** rather than taken from the Navigation Toolbox — all
  six word types, shortest wins. Routes are measured on the *sampled* track, because that is
  what the aircraft flies and what the outer loop measures.
* **k-means** is k-means++ seeded Lloyd with replicates, matching MATLAB's contract
  (squared-Euclidean, lowest within-cluster sum of squares over restarts, singleton repair
  for empty clusters).
* **Random numbers differ.** The RNG is `std::mt19937_64`, so a given seed does not reproduce
  MATLAB's exact scenario or its exact GRASP restarts. Results are statistically equivalent,
  not bit-identical. Runs *are* reproducible within this package: same parameters, same
  result (asserted by `test_pipeline`).
* **Visualization is not ported.** The demo can write CSV (`--csv DIR`) for plotting
  elsewhere — including `residual_belief.csv`, the data behind MATLAB's
  `plotResidualBelief` figure; the planner has no rendering dependency.
* **Diagnostics are padded with the trajectory.** Padding an agent's track to the team
  timeline pads its per-step gimbal signals too, so the audit always compares the schedule
  against the track that was actually delivered.
* Legacy call shims (`optimizeDroneSensorTrajTilted`, the 3- and 4-argument forms of the
  scheduler) are not carried over — there is one entry point per function.
* The yaw/climb **repair loop is implemented and off by default**, exactly as in
  `init_params.m` (`gimbal.enableRepairLoop = false`). Turning it on bends the ground track
  (and, failing that, climbs) toward centres the gimbal cannot otherwise reach, accepting a
  pass only if the observable count improves — a cluster has centres on both sides of the
  track, so bending toward one can strand another. The perturbation is applied to the
  *heading* and re-integrated at the same speed, so arc length, timeline and sample count are
  invariants. `test_geometry` exercises it: 2/6 centres → 6/6, with the geometry audit still
  passing.

## Performance

The reference scenario — 5 km map, 1 m belief grid (5001×5001), 412 valid cells, 45 clusters,
3 agents, ~2400 trajectory steps each — plans end to end in **under one second** on a single
core, including the geometry audit. The belief map is built as separable outer products
rather than an `n²` exponential evaluation, which is most of that. Scoring it with the
residual belief adds ~12 s (see *Scoring a plan* above).
