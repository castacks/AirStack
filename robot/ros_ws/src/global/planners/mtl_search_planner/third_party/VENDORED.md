# Vendored: `mtl_planner`

| | |
|---|---|
| Upstream | `multi_agent_target_localization/cpp_planner` (AirLab, CMU), commit `e186a1b` ("total belief mass scales up to 1, and final performance metric is residual probability mass") |
| Vendored into | `third_party/mtl_planner/` |
| Changes | **none**: a byte-identical copy of all 58 tracked files (the upstream `.gitignore` and untracked `.DS_Store` files are omitted) |
| Tree hash | `14d9722343a28659` = first 16 hex digits of `sha256` over the `sha256sum` of every file, sorted by path; recompute with the command below |
| Built by | `cmake/mtl_vendored.cmake`: static `mtl_planner_vendored` (always) and `mtl_eval_vendored` (`EXCLUDE_FROM_ALL`); the upstream `CMakeLists.txt` is not used |
| Tests | the five upstream tests (`tests/test_*.cpp`) are registered with `colcon test` via `mtl_vendored_add_selftests()` |

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
mirrors the parameter mapping of the upstream `apps/mtl_plan_json.cpp`. The planner itself is
never patched.

## Updating

```bash
SRC=/path/to/multi_agent_target_localization/cpp_planner
DST=robot/ros_ws/src/global/planners/mtl_search_planner/third_party/mtl_planner
rm -rf "$DST" && cp -r "$SRC" "$DST" && rm -f "$DST/.gitignore" && rm -rf "$DST/build"
find "$DST" -name .DS_Store -delete                                                   # macOS litter
(cd "$DST" && find . -type f | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -c1-16)   # -> update the hash above
```

If `cmake/mtl_vendored.cmake` globs miss new source files, add them there. Then rebuild with
`bws --packages-select mtl_search_planner` and run the gtests plus the upstream self-tests.
