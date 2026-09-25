# Vendored: `mtl_planner`

| | |
|---|---|
| Upstream | `multi_agent_target_localization/cpp_planner` (AirLab, CMU) |
| Vendored into | `third_party/mtl_planner/` |
| Changes | **none**: a byte-identical copy of all 57 source files (the upstream `.gitignore` is omitted) |
| Tree hash | `5edeb4e826ab5e25` = first 16 hex digits of `sha256` over the `sha256sum` of every file, sorted by path; recompute with the command below |
| Built by | `cmake/mtl_vendored.cmake`: static `mtl_planner_vendored` (always) and `mtl_eval_vendored` (`EXCLUDE_FROM_ALL`); the upstream `CMakeLists.txt` is not used |
| Tests | the five upstream tests (`tests/test_*.cpp`) are registered with `colcon test` via `mtl_vendored_add_selftests()` |

The adapter code in this package (`src/search_problem.cpp`, `mtl_search_planner/scenario.py`)
mirrors the parameter mapping of the upstream `apps/mtl_plan_json.cpp`. The planner itself is
never patched.

## Updating

```bash
SRC=/path/to/multi_agent_target_localization/cpp_planner
DST=robot/ros_ws/src/global/planners/mtl_search_planner/third_party/mtl_planner
rm -rf "$DST" && cp -r "$SRC" "$DST" && rm -f "$DST/.gitignore" && rm -rf "$DST/build"
(cd "$DST" && find . -type f | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -c1-16)   # -> update the hash above
```

If `cmake/mtl_vendored.cmake` globs miss new source files, add them there. Then rebuild with
`bws --packages-select mtl_search_planner` and run the gtests plus the upstream self-tests.
