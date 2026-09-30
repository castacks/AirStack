# Vendored: `mtl_planner`

| | |
|---|---|
| Upstream | `multi_agent_target_localization/cpp_planner` (AirLab, CMU), commit `e186a1b` ("total belief mass scales up to 1, and final performance metric is residual probability mass") |
| Vendored into | `third_party/mtl_planner/` |
| Changes | **one AirStack addition**, the information-aware abstraction search (`PlannerParams::infoAware`, off unless the scenario enables it; see below). Everything else is a byte-identical copy of the 58 upstream files (the upstream `.gitignore` and untracked `.DS_Store` files are omitted) |
| Upstream tree hash | `14d9722343a28659` for the unmodified `e186a1b` tree = first 16 hex digits of `sha256` over the `sha256sum` of every file, sorted by path |
| Tree hash with the addition | `91c9294bc515e734` (66 files; recompute with the command below) |
| Patch | `info_aware_upstream.patch` (next to this file): the addition as a unified diff against `e186a1b`, to apply upstream or re-apply after re-vendoring |
| Built by | `cmake/mtl_vendored.cmake`: static `mtl_planner_vendored` (always) and `mtl_eval_vendored` (`EXCLUDE_FROM_ALL`); the upstream `CMakeLists.txt` is not used |
| Tests | the five upstream tests plus `tests/test_info_aware.cpp` are registered with `colcon test` via `mtl_vendored_add_selftests()` |

## What the `e186a1b` update changed (and what the adapter does about it)

See `mtl_planner/CHANGES_belief_mass_and_residual.md` for the upstream change note.

- **The prior is a probability mass function** (sums to 1) and cells are kept by per-cell
  **belief mass**, not mean belief: `PlannerParams::meanInformationThresh` became
  `minimumBeliefMass`. The adapter (`src/search_problem.cpp`) now reads
  `mapping.minimum_belief_mass`; `mean_information_thresh` is ignored. The host generator
  (`mtl_search_planner/scenario.py`) normalises the prior and sends cell masses as
  probabilities with `cells.total_map_mass = 1`.
- **Residual belief** (`mtl::eval::computeResidualBelief`, in `mtl_eval_vendored` only) is
  the new planner-comparison metric, `P(target missed)`, lower is better. The flown runs are
  scored with a Python port of the same model in `mtl_metrics_logger` (`detection.py`).
- `mtl_vendored.cmake` needs no change: no source files were added or removed, and
  `MTL_HAVE_OPENMP` is left undefined, so `computeResidualBelief` builds serially.

The adapter code in this package (`src/search_problem.cpp`, `mtl_search_planner/scenario.py`)
mirrors the parameter mapping of the upstream `apps/mtl_plan_json.cpp`.

## The AirStack addition: information-aware abstraction search

See `mtl_planner/CHANGES_info_aware.md` for the full note. In short:

- **Opt-in.** `PlannerParams::infoAware.enabled` (scenario `info_aware.enabled`, set from
  `stacks/mtl_search/config/mission.yaml`) switches `Planner::planFromCells` to
  `planning::planInfoAware`. Disabled, the planner is the upstream one, plan for plan.
- **New files:** `include/mtl/mapping/peak_clusters.hpp`, `src/mapping/peak_clusters.cpp`,
  `include/mtl/planning/coverage_score.hpp`, `src/planning/coverage_score.cpp`,
  `include/mtl/planning/info_aware.hpp`, `src/planning/info_aware.cpp`,
  `tests/test_info_aware.cpp`, `CHANGES_info_aware.md`.
- **Edited files:** `include/mtl/types.hpp` (`ClusterSet::basin/level/parent`,
  `InfoAwareCandidate`, `InfoAwareReport`, `PlanningResult::infoAware`), `include/mtl/params.hpp`
  (`InfoAwareParams`, `PlannerParams::infoAware`), `src/params.cpp` (validation),
  `src/planner.cpp` (dispatch), `apps/mtl_plan_json.cpp` (reads `info_aware`),
  `CMakeLists.txt` + `cmake/mtl_plannerConfig.cmake.in` (sources, test, `Threads`), `README.md`.
- `cmake/mtl_vendored.cmake` lists the three new sources, links `Threads::Threads` and registers
  `test_info_aware`.
- Until it is merged upstream, re-apply `info_aware_upstream.patch` after every re-vendor
  (below). Once upstream carries it, this section and the patch go away.

## Updating

```bash
SRC=/path/to/multi_agent_target_localization/cpp_planner
DST=robot/ros_ws/src/global/planners/mtl_search_planner/third_party/mtl_planner
rm -rf "$DST" && cp -r "$SRC" "$DST" && rm -f "$DST/.gitignore" && rm -rf "$DST/build"
find "$DST" -name .DS_Store -delete                                                   # macOS litter
# until upstream has the info-aware search: re-apply it (skip if upstream already contains it)
(cd "$DST" && patch -p1 --dry-run < ../info_aware_upstream.patch && patch -p1 < ../info_aware_upstream.patch)
(cd "$DST" && find . -type f | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -c1-16)   # -> update the hash above
```

If `cmake/mtl_vendored.cmake` globs miss new source files, add them there. Then rebuild with
`bws --packages-select mtl_search_planner` and run the gtests plus the upstream self-tests.
