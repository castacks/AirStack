# mtl_curve — Multi-Agent Parameterized-Curve Search Planner

A C++17 port of the MATLAB parameterized-curve planner (`main_curve_planner.m`,
`init_curve_params.m`, `functions/curve_planning`, `functions/optimization`,
`functions/trajectory/generateCurveTrajectories.m`), packaged exactly like its sibling
[`cpp_planner`](../cpp_planner/README.md):

- the planner is one object you construct with a parameter set and call;
- scenario generation and scoring are separate libraries you can leave out;
- it depends on Eigen only;
- it has a JSON adapter that is a drop-in for `mtl_plan`.

Everything lives in namespace **`mtl::curve`** with headers under **`mtl_curve/`**, so this
package and `cpp_planner` (`mtl::`, `mtl/`) can be linked into one host side by side.

---

## What it does

Given a prior belief over a search area and a team of fixed-wing aircraft with an endurance
budget, it gives each aircraft **one continuous flight curve**. Each curve:

- is **exactly** the budget long;
- never turns tighter than `minTurnRadius`;
- is optimised **jointly** with the others to minimise the team's residual belief.

The single-axis gimbal sweeps its cross-track line along the curve. This replaces
`cpp_planner`'s chain of cells → clusters → budgeted orienteering → Dubins → gimbal schedule →
run-out. There are no waypoints, no Dubins re-costing and no lateral-coverage extension.

| Stage | What it does | Why |
|---|---|---|
| `extractValidCells`, `clusterCells`, `computeClusterRewards` | the same abstraction as `cpp_planner` (copied code) | the clusters seed and audit the curves |
| k-means partition | **the same call and seed as `mtl::Planner`** | both planners start from the same split |
| `computeSweepParams` | sweep amplitude and frequency from slant range, gimbal travel, reach and slew rate | the swath is what the payload can actually sweep |
| `calibrateSwathKernel` | aircraft-frame look kernel, calibrated from the real detection model (see below) | the fast objective must predict what `computeResidualBelief` will score |
| `enforceEndpointConstraints` | decision vector, bounds, pinning; open / return-home / fixed-destination | |
| `initParametricSpline` | a **flown** seed of exactly the budget length: pursue the ordered clusters, then chase unswept belief | a scaled or extended spline breaks the start pin and the curvature bound |
| `optimizeAgentCurve` | L-BFGS + augmented Lagrangian; explore → refine; Gauss-Newton feasibility projection | analytic gradients, so one evaluation per step |
| `reallocateCurveClusters` | multi-start seeding, Gauss-Seidel team solve, service audit, slack, reallocation rounds | the plan's PHASE 5 |
| `generateCurveTrajectories` | 2 m resampling, coordinated-turn RPY (`computeAirframeRPY`, copied), sinusoidal cross-track sweep, padding | flight-ready output |

### Representations

| `CurveParams::representation` | Decision vector | Length | Curvature | Nonlinear constraints |
|---|---|---|---|---|
| **`Curvature`** (default) | `[c (Nk); theta0]`: κ(s) is piecewise linear in arc length | exact by construction | box bound on `c` holds **everywhere** | only the endpoint in the closed modes (2 equations) |
| `BSpline` (the plan) | free control points of a clamped cubic B-spline | equality (Simpson, 500 intervals) | inequalities every ~5 m at 0.99·κmax | length + curvature |

### The fast objective is calibrated, not assumed

The plan scores a pixel with one look, `P_det(d_perp)`. The evaluator instead multiplies
`(1 − P)` over **every** time step the pixel is inside the swept footprint. A forward-tilted
mount also looks `h·tan(tau)` ahead, and that swings round in a turn. So the kernel `k(a, d)`
(log-miss per metre flown at along/cross offset from the aircraft) is built from:

- the **same** footprint, sigmoid, `beta` cut-off and `pOut` as `eval::computeResidualBelief`;
- averaged over the sweep phase;
- with the hard edges replaced by logistic steps set **inside** the physical reach;
- scaled row by row so that a straight pass reproduces the exact along-track-averaged profile,
  or is more conservative, and never better.

It rotates with the aircraft, and deposits from overlapping swaths add. That is the plan's
joint `prod_a (1 − P_det_a)`, with no ad-hoc de-confliction. Next to it is an **exploration
kernel**: the same kernel plus a Gaussian-blurred tail. The optimiser runs a first stage on it
over a coarser grid so that belief beyond the swath edge still pulls on the curve.

---

## Layout

```
include/mtl_curve/
  types.hpp                  value types (BeliefField/CellSet/ClusterSet/Target copied; curve types new)
  params.hpp                 EVERY tunable, in structs — the port of init_curve_params.m
  planner.hpp                mtl::curve::Planner: the integration surface
  core/           numeric, kmeans (copied), bspline
  mapping/        cells (copied)
  planning/       orienteering (copied; used to ORDER each agent's clusters)
  curve_planning/ endpoint_constraints, parametric_curve, arc_length, init_spline,
                  swath_polygon, team_curves
  optimization/   fast_grid, swath_kernel, objective, optimizer
  sensing/        sweep, airframe (copied)
  trajectory/     curve_trajectory
  mapgen/         scenario (copied)                    → library mtl_curve_mapgen
  eval/           detection (copied), report           → library mtl_curve_eval
src/              mirrors include/
apps/             demo_curve_pipeline.cpp  (mtlc_demo, the C++ main_curve_planner.m)
                  mtl_curve_plan_json.cpp  (mtlc_plan, drop-in for mtl_plan)
                  compare_planners.cpp     (mtlc_compare, head-to-head vs cpp_planner)
                  json_mini.hpp            (copied)
tests/            six suites, run by ctest, no external framework
```

**Copied from `cpp_planner`, with only the namespace, include paths and guards changed:**
`core/numeric`, `core/kmeans`, `mapping/cells`, `planning/orienteering`, `sensing/airframe`
(its parameter struct is renamed `GimbalParams`), `mapgen/scenario`, `eval/detection`,
`apps/json_mini.hpp` and `tests/test_util.hpp`. `tests/test_kmeans.cpp` is copied whole, and
`tests/test_orienteering.cpp` keeps its solver sections.

Because mapgen and detection are copied, a given seed produces the **same scenario** in both
packages, and both plans are scored by the **same code**. `mtlc_compare` asserts both of these.

### Three libraries, as in `cpp_planner`

| Target | Contents | Depends on |
|---|---|---|
| **`mtl_curve::planner`** | the whole planning chain | Eigen only |
| `mtl_curve::mapgen` | Gaussian prior (normalised) + ground-truth targets | `mtl_curve::planner` |
| `mtl_curve::eval` | detection physics, residual belief, reports | `mtl_curve::planner` |

---

## Build

The requirements are the same as `cpp_planner`: CMake ≥ 3.16, a C++17 compiler, and Eigen ≥ 3.3.
Eigen is fetched automatically if it isn't installed.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/mtlc_demo                      # the reference scenario
./build/mtlc_demo --help
```

| Option | Default | |
|---|---|---|
| `MTLC_BUILD_APPS` | ON | `mtlc_demo`, `mtlc_plan` |
| `MTLC_BUILD_TESTS` | ON | the six suites |
| `MTLC_ENABLE_WARNINGS` | ON | the same strict set as `cpp_planner` |
| `MTLC_ENABLE_OPENMP` | OFF | run the multi-start seeds (and the residual pass) in parallel |
| `MTLC_BUILD_COMPARE` | OFF | `mtlc_compare`: builds `../cpp_planner` in-tree and runs both planners on the same scenarios |

```bash
cmake -S . -B build -DMTLC_BUILD_COMPARE=ON        # expects ../cpp_planner
./build/mtlc_compare --seeds 21,1,2,3,4,5
```

### Installing and consuming

```bash
cmake --install build --prefix /opt/mtl_curve
```

```cmake
find_package(mtl_curve_planner REQUIRED)
target_link_libraries(my_simulation PRIVATE mtl_curve::planner)
# and, only if you want them:  mtl_curve::mapgen  mtl_curve::eval
```

To vendor it, use `add_subdirectory(third_party/mtl_curve_planner)`, or compile its source
list directly. That is what AirStack's `cmake/mtl_vendored.cmake` does for `cpp_planner`; the
lists are in `CMakeLists.txt`.

---

## Integrating

```cpp
#include "mtl_curve/planner.hpp"

mtl::curve::PlannerParams params;            // every tunable, with the reference defaults
params.numAgents       = 4;
params.maxFlightTime   = 400.0;              // [s] per agent - must be finite: the curve IS the budget
params.sensorTiltAngle = mtl::curve::deg2rad(50);
params.curve.endpointModes = {mtl::curve::EndpointMode::ReturnHome};

mtl::curve::Planner planner(params);         // validates and finalises once, here
const mtl::curve::PlanningResult r = planner.plan(belief, starts);

for (const mtl::curve::AgentTrajectory& t : r.trajectories) {
    // t.drone.row(k)  = [x y h]             aircraft state
    // t.sensor.row(k) = [x y]               boresight ground point
    // t.rpy.row(k)    = [roll pitch yaw]    airframe attitude, radians
    // t.gimbalCmd(k)  = sweep angle + roll  what the 1-DOF gimbal is commanded
}
```

| Call | Use when |
|---|---|
| `plan(belief, starts)` | you have a belief grid; the curves are optimised against it directly |
| `planFromCells(cells, starts)` | your host passes cells only; the objective's prior is each cell's mass spread over its `targetCellSize` block |
| `planFromClusters(cells, clusters, starts[, &belief])` | you also control the macro-clustering |

`PlanningResult` has the same shape as `cpp_planner`'s: `cells`, `clusters`,
`agentOfCluster`, `plans`, `trajectories` padded onto one `timeVec`, and `team`. `plans[a]`
carries:

- the curve parameters and the representation;
- the seed that won (`clusters`, `greedy`, `realloc:insert` or `realloc:direct`);
- the altitude, sweep and calibrated swath half-width;
- the measured length, maximum curvature and endpoint error;
- `servicedCellIdx`: cells whose centre this agent's swath detects with P ≥ 0.5.

`team` carries:

- the fast-objective history (`Jhist`, `Jstatic`, `Jfinal`);
- every reallocation trial;
- the final cluster audit, with unserviced clusters and slack.

`staticTrajectories` is the pre-reallocation plan, flown the same way, for the
static-vs-reallocated comparison.

### `mtlc_plan` — the JSON adapter

```bash
mtlc_plan --scenario scenario.json --out plan.json
```

It reads the **same** `mtl.scenario/1` and writes the **same** `mtl.plan/1` schema as
`mtl_plan`, with the same frames (mission NED, south-west anchored) and field-for-field
parameter mapping.

- Every key `mtl_plan` writes is present. Scheduler-only diagnostics appear only when
  `scheduled` is true, exactly as in `mtl_plan`, and `scheduled` is always false here.
- Curve additions: `samples.gimbal`, the curve diagnostics, and `meta.curve`.
- Curve options come from an optional `"curve"` block; see the header of
  `apps/mtl_curve_plan_json.cpp`.
- A **scaled mission** gets the reference lengths scaled automatically:
  - grid, sample and knot spacing by `size_m / 5000`;
  - kernel geometry and altitude stagger by `beta / 610`.

---

## Parameters

Everything is in `mtl_curve/params.hpp`, in the same order as `init_curve_params.m`. Sections
1–4 (belief map, clustering, detection model, orienteering) are copied from `cpp_planner`.

| Parameter | Effect |
|---|---|
| `maxFlightTime` / `maxFlightDistance` | the curve length; must be finite |
| `sensorTiltAngle` | mount tilt; the swept line stands off `h·tan(tilt)` ahead |
| `curve.representation` | `Curvature` (exact length and turn radius) or `BSpline` (the plan) |
| `curve.endpointModes` | one mode, or one per agent |
| `team.altitudeStagger` | 25 m in the plan; each altitude gets its own kernel. Higher costs reach: at 350 m and 50° tilt, 500 m instead of 531 m. **0 is like-for-like with `cpp_planner`.** |
| `team.initStrategies` | `{Clusters, Greedy}` multi-start; `{Greedy}` halves the first sweep |
| `optimizer.exploreIter` | the exploration stage; 0 disables it |
| `team.fastGridStep` | 25 m; the optimiser never touches the full-resolution grid |
| `team.maxReallocTrials`, `team.coordinationSweeps` | the main runtime knobs |

`finalize()` derives the curvature limit (`1/minTurnRadius`), the dense step (`V·dt`), the
control-point box and the gimbal `dt`. `validate()` throws `std::invalid_argument` at
construction on sets that cannot produce a trajectory, for example:

- an infinite budget;
- an agent whose boresight is out of range even at zero gimbal angle;
- the wrong number of endpoint modes or destinations;
- a fixed destination farther away than the budget.

---

## Benchmark: `mtlc_compare` against `cpp_planner`

Each seed uses the same scenario from both packages' mapgen, which `mtlc_compare` checks are
bit-identical. The reference parameters are 5 km map, 1 m grid, 3 agents × 5000 m, single-axis
gimbal at 50° tilt, and 50 targets. Both plans are scored by `mtl::eval::computeResidualBelief`
(**lower is better**). The curve planner runs its defaults, including the plan's 25 m altitude
stagger.

| Seed | `mtl::Planner` residual | targets | `mtl::curve::Planner` residual | targets | change |
|---|---|---|---|---|---|
| 21 | 0.1159 | 45 | **0.0257** | 49 | −78% |
| 1 | 0.1063 | 46 | **0.0627** | 48 | −41% |
| 2 | 0.2587 | 38 | **0.1303** | 43 | −50% |
| 3 | 0.0929 | 47 | **0.0370** | 49 | −60% |
| 4 | 0.2054 | 44 | **0.0608** | 46 | −70% |
| 5 | 0.0830 | 48 | **0.0159** | 50 | −81% |
| **mean** | **0.1437** | 44.7 | **0.0554** | 47.5 | **−61%** |

On one core, planning takes 0.4–0.7 s for `mtl::Planner` and 45–75 s for the curve planner.
Most of that time goes to the multi-start seeds and the reallocation trials; see
`team.initStrategies`, `team.maxReallocTrials` and `team.coordinationSweeps`, or build with
`MTLC_ENABLE_OPENMP`. The copied `mtl::curve::eval` scores every curve plan identically to
`cpp_planner`'s eval, to 1e-12.

---

## Tests

```bash
ctest --test-dir build --output-on-failure
```

| Suite | Covers |
|---|---|
| `test_kmeans` | copied from `cpp_planner` |
| `test_orienteering` | the solver sections of `cpp_planner`'s suite |
| `test_curve_geometry` | B-spline basis and derivatives; exact length and curvature bound; the VJP (both representations) and endpoint Jacobian against finite differences; pinning; infeasible destination; uniform resampling |
| `test_swath_kernel` | sweep limits; the kernel is never materially more optimistic than the real sensor on a straight pass; nothing beyond the reach; exploration kernel reaches further; grids conserve mass; objective gradient against finite differences (both representations) |
| `test_optimizer` | acceptance checks 1–3 for every representation × endpoint mode, on the flown trajectory, and the optimiser never worsens the seed |
| `test_pipeline` | end to end from a belief and from cells (return-home); padded timeline; exact length and curvature; the fast model within 0.05 of `computeResidualBelief`; residual = target miss probability at the target's pixel; determinism; bad parameters rejected at construction |

---

## Differences from the MATLAB original

* **No fmincon path.** The internal L-BFGS + augmented-Lagrangian solver is the only
  optimiser. It is the same algorithm as MATLAB's `'internal'` option, step for step.
* **Random numbers differ.** As in `cpp_planner`, the RNG is `std::mt19937_64`. A given seed
  does not reproduce MATLAB's scenario, but it **does** reproduce `cpp_planner`'s.
* **Visualisation is not ported.** `mtlc_demo --csv DIR` writes the trajectories, swath
  boundaries, clusters, targets and the residual map for plotting elsewhere.
* **The kernel's straight-line correction is taken conservatively across the reach cliff.**
  At a table row between two profile samples it uses the less negative of the two, where MATLAB
  originally interpolated linearly. The fix was ported back to `calibrateSwathKernel.m`, and
  both now agree. It only matters at the last few metres of the swath edge, most for a nadir
  mount.
* **Negligible kernel entries are trimmed** (below 1e-7 log-miss per metre, which cannot move
  a pixel by 0.01% over a kilometre). This lets the deposit skip the part of the stencil a
  forward-tilted mount never sees.
