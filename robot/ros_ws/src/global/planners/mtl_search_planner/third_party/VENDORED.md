# Vendored: `mtl_planner`

| | |
|---|---|
| Upstream | `multi_agent_target_localization/cpp_planner` (AirLab, CMU), commit `8ba811a` ("added new sector absed seeding made single optimization and a massive unclean code dump"; previously `e186a1b` plus the AirStack info-aware addition) |
| Vendored into | `third_party/mtl_planner/` |
| Changes | **none**: a byte-identical copy of the 66 git-tracked upstream files (the upstream `.gitignore` is omitted). Checked file by file with `cmp` |
| Upstream tree hash | `91c9294bc515e734` = first 16 hex digits of `sha256` over the `sha256sum` of every file, sorted by path (command below). It equals the hash of the previous `e186a1b` + info-aware tree: upstream `8ba811a` carries exactly that addition |
| Built by | `cmake/mtl_vendored.cmake`: static `mtl_planner_vendored` (always) and `mtl_eval_vendored` (`EXCLUDE_FROM_ALL`); the upstream `CMakeLists.txt` is not used |
| Tests | the six upstream tests (`test_dubins`, `test_kmeans`, `test_orienteering`, `test_geometry`, `test_pipeline`, `test_info_aware`) are registered with `colcon test` via `mtl_vendored_add_selftests()` |

## What the `8ba811a` update changed (and what the adapter does about it)

- **The information-aware abstraction search is upstream.** It was an AirStack addition on top
  of `e186a1b` (`PlannerParams::infoAware`, `mapping/peak_clusters`, `planning/coverage_score`,
  `planning/info_aware`, `tests/test_info_aware.cpp`, `CHANGES_info_aware.md`), carried here as
  a patch. Upstream `8ba811a` contains it byte for byte (all 66 files `cmp`-identical to the
  patched tree), so this tree is unmodified again and the patch was retired
  (`info_aware_upstream.patch`, moved out of the package).
- Nothing else changed in `cpp_planner`: `apps/mtl_plan_json.cpp` already read `info_aware`, so
  `src/search_problem.cpp`, `scenario.py`, `mission.yaml` and the orienteering plans are unchanged.
- `mtl_vendored.cmake` needs no source change (the info-aware sources were already listed).

## What the `e186a1b` update changed

See `mtl_planner/CHANGES_belief_mass_and_residual.md` for the upstream change note.

- **The prior is a probability mass function** (sums to 1) and cells are kept by per-cell
  **belief mass**, not mean belief: `PlannerParams::meanInformationThresh` became
  `minimumBeliefMass`. The adapter (`src/search_problem.cpp`) reads
  `mapping.minimum_belief_mass`; `mean_information_thresh` is ignored. The host generator
  (`mtl_search_planner/scenario.py`) normalises the prior and sends cell masses as
  probabilities with `cells.total_map_mass = 1`.
- **Residual belief** (`mtl::eval::computeResidualBelief`, in `mtl_eval_vendored` only) is
  the planner-comparison metric, `P(target missed)`, lower is better. The flown runs are
  scored with a Python port of the same model in `mtl_metrics_logger` (`detection.py`).
- `MTL_HAVE_OPENMP` is left undefined, so `computeResidualBelief` builds serially.

The adapter code in this package (`src/search_problem.cpp`, `mtl_search_planner/scenario.py`)
mirrors the parameter mapping of the upstream `apps/mtl_plan_json.cpp`. The information-aware
search (see `mtl_planner/CHANGES_info_aware.md`) is opt-in: `PlannerParams::infoAware.enabled`
(scenario `info_aware.enabled`, from `stacks/mtl_search/config/mission.yaml`); disabled, the
planner is the plain one, plan for plan.

## Updating

```bash
SRC=/path/to/multi_agent_target_localization
DST=robot/ros_ws/src/global/planners/mtl_search_planner/third_party/mtl_planner
# copy the git-tracked files only (no .gitignore, build/ or .DS_Store)
(cd "$SRC" && git ls-files cpp_planner | grep -v '/\.gitignore$') | while read f; do
  r=${f#cpp_planner/}; mkdir -p "$DST/$(dirname "$r")"; cp -p "$SRC/$f" "$DST/$r"; done
(cd "$DST" && find . -type f | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -c1-16)   # -> update the hash above
```

Upstream removed a file? Delete it here too (ask first). If sources were added or removed,
update `cmake/mtl_vendored.cmake`. Then rebuild with `bws --packages-select mtl_search_planner`
and run the gtests plus the upstream self-tests. Never patch this tree: report upstream bugs and
re-vendor.

---

# Vendored: `mtl_curve_planner`

| | |
|---|---|
| Upstream | `multi_agent_target_localization/cpp_curve_planner` (AirLab, CMU), commit `8ba811a` ("added new sector absed seeding made single optimization and a massive unclean code dump"; previously `7a45427` + the then-uncommitted `lambdaCap` fix, which `8ba811a` commits) |
| Vendored into | `third_party/mtl_curve_planner/` |
| Changes | **none**: a byte-identical copy of the 86 git-tracked upstream files (the upstream `.gitignore` is omitted). Checked file by file with `cmp` |
| Upstream tree hash | `e0acdb7b6d879d5d` (previous: `a48d80580279722a`; `7a45427` as committed: `9097128aaaa5f542`; `a2c5e9d`: `3ce200088c6f6db5`) = first 16 hex digits of `sha256` over the `sha256sum` of every file, sorted by path (same command as for `mtl_planner`, above) |
| Built by | `cmake/mtl_vendored.cmake`: static `mtl_curve_planner_vendored` (always; Eigen only, `-O2` even in Debug) and `mtl_curve_eval_vendored` (`EXCLUDE_FROM_ALL`: mapgen + detection/report); the upstream `CMakeLists.txt` is not used. `MTLC_HAVE_OPENMP` stays undefined (a plan is serial and deterministic) and so does `MTLC_HAVE_SCS` (= upstream `MTLC_ENABLE_GCS=OFF`: SCS is neither vendored nor fetched) |
| Tests | ten upstream suites (`test_kmeans`, `test_orienteering`, `test_curve_geometry`, `test_swath_kernel`, `test_optimizer`, `test_path_insert`, `test_pipeline`, `test_sector_orienteering`, `test_info_aware_clusters`, `test_quadrotor`) are registered with `colcon test` as `mtl_curve_vendored_*` via `mtl_curve_vendored_add_selftests()`. `test_gcs` is not built: it calls the SCS solver |

Namespace `mtl::curve`, headers under `mtl_curve/`: several files are copies of `mtl_planner`
files in the new namespace, so both planners link into one binary (`mtl_search_core`).

## What the `8ba811a` update changed (and what the adapter does about it)

See `mtl_curve_planner/README.md` (*Information-aware clusters and the GCS seed*, *Quadrotor with a
body-fixed camera*, the `team.*` table) and the upstream `README_curve_planner.md` (*Defaults
(2026-10-23)*).

- **New default first solve.** `TeamParams::initStrategies` `{Clusters, Greedy}` → `{Sector}`,
  `coordinationSweeps` 2 → 1, `reallocate` true → false, `orderBudgetFrac` 0.8 → 1.0. The new
  `Sector` seed (`planning/sector_orienteering`) is a coverage orienteering over a sector grid,
  planned per agent on the belief the agents before it leave unswept. Upstream: about −3 % mean
  residual at about 40 % of the time over 118 single-agent maps. Adopted for AirStack (decided by
  Burhan, 2026-10-23) and written out in `stacks/mtl_search/config/mission.yaml`, so a later
  default change cannot silently change the flown plan.
- **`orderBudgetFrac` had been patched here.** AirStack commit `5ee6f4209` edited the vendored
  `params.hpp` default to 0.95 (the tree was then not byte-identical, although this file said
  so). Re-vendoring restores upstream; the 0.95 tune is now `curve.order_budget_frac: 0.95` in
  `mission.yaml` (unused with `[sector]`).
- **Equivalence check.** With `init_strategies: [clusters, greedy]`, `coordination_sweeps: 2`,
  `reallocate: true`, `order_budget_frac: 0.95`, this build plans the committed 1-agent bundle
  **byte for byte** like the previous build (`plan.json` samples and `track.json` identical), so
  every plan difference comes from the changed defaults.
- **Plans, 5 km mission (1 m raster).** 1 agent: fast residual 0.4925 → 0.4921, planning
  9.4 s → 3.4 s; offline mission flown 0.4918 / planned 0.4904 (was 0.4925 / 0.4911), 26/50
  targets (was 25). **3 agents** (homes 30 m apart, a synthetic variant): `[sector]` 0.138 in
  10 s vs the old pipeline 0.073 in 58 s (`[clusters, greedy]` with 1 sweep: 0.103; `[sector]` +
  reallocation: 0.105). Known and accepted until the multi-agent pipeline is revamped; worth
  reporting upstream, whose study was single-agent.
- **New `mtlc_plan` keys, mirrored by `src/search_problem.cpp` and validated by `scenario.py`:**
  `curve.init_strategies` accepts `sector`; `curve.sector` {`size_m`, `grid_step_m` (both
  auto-scaled × `size_m / 5000` when absent, like the grids), `box_frac`, `budget_frac`,
  `starts`, `perturb`, `max_candidates`}. The adapter refuses unknown `sector` keys (mtlc_plan
  ignores them). `order_budget_frac` was already read by the adapter and is now also accepted by
  `scenario.py`.
- **New upstream features that `mtlc_plan` does not read, so the adapter does not expose them**
  (strict mirror): the GCS seed (`InitStrategy::Gcs`, `TeamParams::gcs`; needs SCS, not built;
  `gcs` refused), the curve planner's own information-aware cluster search
  (`PlannerParams::infoAware`), the quadrotor body-fixed-camera conversion (`PlannerParams::quad`;
  the follower keeps flying the gimbal plan), kernel clusters (`team.clusterMode`) and the
  corridor penalty. All stay at their defaults (off); their blocks are refused in the scenario.
- **Host additions to `plan.json` → `meta.curve`:** `init_strategies` and `seeds` (per agent
  `j_seed` / `j_optimised` from `TeamInfo::seedJ0` / `seedJ1`, and the sector route
  `sector_route_ned`, `sector_reward`, `sector_length_m`). The sector wall time goes to the CLI /
  node log only, so `plan.json` stays deterministic.
- **`cmake/mtl_vendored.cmake`** builds the new sources (`planning/sector_orienteering`,
  `curve_planning/info_aware`, `mapping/peak_clusters`, `quadrotor/flatness`,
  `quadrotor/quad_trajectory`, `gcs/{kernel_clusters,convex_sets,conic,tour,plan_agent}`; `gcs/`
  must compile because `planner.cpp` and `team_curves.cpp` call into it, and its solver throws
  without SCS) and registers the three new runnable tests. The new apps (`bench_gcs`,
  `bench_study`) are vendored but not built.
- Unchanged: `eval/detection` (the Python port in `mtl_metrics_logger/detection.py` still matches
  `computeResidualBelief` to ~1e-14 in residual mass), the curve geometry, the gimbal convention.
- MATLAB only: `init_params.m` now defaults the ORIENTEERING planner's `budgetOpts.reallocate`
  to false, but `cpp_planner` (which AirStack vendors) keeps its default, so the orienteering
  comparison plans are unchanged. Upstream reference and port now disagree on that default.

## What the `7a45427` update changed

See `mtl_curve_planner/README.md` (`pathInsertSeed`) and the upstream `README_curve_planner.md`
("Reallocation seed: 'path'") for the method and benchmark.

- **New reallocation seed `ReallocStrategy::Path`** (`curve_planning/path_insert.{hpp,cpp}`,
  splice support in `init_spline`): a slack agent's claim is spliced into its current
  OPTIMISED curve. The kept arcs are copied, the length overrun is cut from the lowest-yield
  arcs, and the result is re-solved warm with `reallocPathMaxIter` = 75 refine iterations.
  A claim that cannot fit is skipped without a solve.
- **Changed default:** `TeamParams::reallocInitStrategies` went from `{Insert, Direct}` to
  `{Path, Direct}`. `Insert` and `Direct` are unchanged: the new build with
  `[insert, direct]` reproduces the `a2c5e9d` plans byte for byte.
- **New scenario keys in `mtlc_plan`:** `curve.realloc_strategies` (`path | insert | direct`),
  `realloc_path_warm` and `realloc_path_max_iter`. `src/search_problem.cpp` reads them
  exactly as `mtlc_plan` does, and `mtl_search_planner/scenario.py` validates them. An empty
  list, an unknown seed or a negative iteration count is refused.
  `TeamParams::reallocPathWpSpacing` (100 m) and `reallocPathMargin` (π/2·Rmin) are not
  `mtlc_plan` keys and stay at the planner defaults.
- **New `TeamInfo::staticTime` / `reallocTime`:** these go to the CLI and node logs only, so
  `plan.json` stays deterministic. `plan.json` → `meta.curve` gains the host-only
  `realloc_strategies` and `realloc_log` fields (every `ReallocTrial`).
- `cmake/mtl_vendored.cmake` builds `src/curve_planning/path_insert.cpp` and registers
  `test_path_insert`.
- **`lambdaCap` 15 → 150.** The same commit raised `init_curve_params.m`
  `kernelOpts.lambdaCap` (the log-miss saturation of the calibrated swath kernel) from 15 to
  150, but left the C++ `KernelParams::lambdaCap` at 15. The fix was made upstream in the
  working tree and is committed in `8ba811a`. It is not an `mtlc_plan` key (the adapter reads
  `curve.kernel.lambda_cap`, a host addition), so `mission.yaml` has nothing to set.

## What the adapter does with it

`src/search_problem.cpp` mirrors the upstream `apps/mtl_curve_plan_json.cpp` (`mtlc_plan`)
field for field, exactly as it mirrors `mtl_planner/apps/mtl_plan_json.cpp` for the
orienteering planner:

- **Selection.** The scenario's `planner.type` (`orienteering` | `curve`, default
  `orienteering`) picks the planner that is FLOWN. With `orienteering` the curve library is
  never called and the plan is the one this package produced before the curve planner existed.
- **Parameters.** `curveParamsFromScenario()` = `mtlc_plan`'s `paramsFromScenario()` +
  `curveFromScenario()`: the shared keys as `mtlc_plan` reads them, the optional `"curve"`
  block, and the same automatic scaling of absent keys (grids / samples / knots by
  `size_m / 5000`, kernel geometry and altitude stagger by `beta / 610`). The planner needs a
  finite budget; `validate()` throws at construction otherwise.
- **Result.** `SearchResult` carries either result type in a `std::variant`, so each planner's
  full diagnostics reach `plan.json`; `buildAgentTrack()` produces the same `TrackSample`
  fields for both. For the curve: `bx, by` from `sensor`, `roll, pitch` from `rpy`, `speed` =
  V, `arc` = cumulative length, `z` = the planned (staggered) altitude, and
  `gimbalPhi = -gimbalCmd` (the curve's `gimbalAngle` is the level-frame sweep angle, + left;
  the host's `phi` is + right with `crossAngle = roll + phi`; proven by
  `test_search_problem.cpp` `CurveGimbalSignRebuildsBoresight`).
- **plan.json.** `teamPlanJson()` writes everything `mtlc_plan` writes (`samples.gimbal`, the
  curve diagnostics, `meta.curve`, `generator.planner_type = "curve"`) plus the host's
  `meta.planner_type` / `meta.planner_mode`.
- **Comparison.** When the curve planner is flown and `planner.compare_orienteering` is true,
  the orienteering planner is also planned in BOTH modes (plain and info_aware) for the report
  only, never flown (`plan_alt_<mode>.json`, `track_alt_<mode>.json`).

## Updating

```bash
SRC=/path/to/multi_agent_target_localization
DST=robot/ros_ws/src/global/planners/mtl_search_planner/third_party/mtl_curve_planner
# copy the git-tracked files only (no .gitignore, build/ or .DS_Store)
(cd "$SRC" && git ls-files cpp_curve_planner | grep -v '/\.gitignore$') | while read f; do
  r=${f#cpp_curve_planner/}; mkdir -p "$DST/$(dirname "$r")"; cp -p "$SRC/$f" "$DST/$r"; done
(cd "$DST" && find . -type f | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -c1-16)   # -> update the hash above
```

Upstream removed a file? Delete it here too (ask first), and update the source list in
`cmake/mtl_vendored.cmake` if sources were added or removed. Never patch this tree: report
upstream bugs and re-vendor. Then diff `apps/mtl_curve_plan_json.cpp` against the previous
commit and mirror every change in `src/search_problem.cpp`.
