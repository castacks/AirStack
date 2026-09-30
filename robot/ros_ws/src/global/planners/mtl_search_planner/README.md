# mtl_search_planner

The global planner of the [`mtl_search`](../../../../../../stacks/mtl_search/README.md) stack. It
wraps two vendored planners (see [VENDORED.md](third_party/VENDORED.md)), and the scenario's
`planner.type` picks the one that is flown:

| `planner.type` | Planner | Output per agent |
|---|---|---|
| `orienteering` (default) | `mtl::planner`, `third_party/mtl_planner` | a Dubins search track within the endurance budget, and the gimbal boresight schedule for it |
| `curve` | `mtl::curve::Planner`, `third_party/mtl_curve_planner` (byte-identical upstream `cpp_curve_planner`) | ONE continuous curve, exactly the budget long and never tighter than the turn radius, optimised jointly with the team's for the lowest residual belief, and the boresight of a sinusoidal cross-track gimbal sweep along it |

Both produce the same `search/plan` (`mtl_msgs/SearchPlan`) and the same run files, so the
follower, the logger and the scripts never branch on the planner type.

## What it does

Every robot runs one node. Each node solves the **whole team problem** from the shared
`scenario.json`. The solve is deterministic, so all robots get the same plan without having
to talk to each other; robots are domain-isolated. Each node then publishes **its own agent
row** in its `map` frame. That frame's origin is the robot's spawn/home, so
`p_map = p_world_ENU − home_ENU`.

| Interface | Type | Notes |
|---|---|---|
| `/<robot>/search_mission` | action `mtl_msgs/SearchMission` | goal `{start_mission, run_id, scenario_file}` → plan, write the run folder, publish, wait for the follower's COMPLETE/ABORTED; feedback: phase, progress, cross-track |
| `search/plan` | `mtl_msgs/SearchPlan` (transient local) | full track + boresight + gimbal schedule; `start_mission=false` for the boot preview |
| `search/planned_trajectory` | `airstack_msgs/TrajectoryXYZVYaw` | the track as a standard AirStack trajectory |
| `search/planned_boresight` | `nav_msgs/Path` | planned boresight ground points, every `viz_path_step_m` of arc (the full-rate list is in `search/plan`) |
| `search/planned_path`, `search/markers` | `nav_msgs/Path`, `MarkerArray` | RViz/Foxglove (teammates optional) |
| `search/follower_status` (in) | `mtl_msgs/FollowerStatus` | completion / abort / progress |
| `search/abort` (out) | `std_msgs/Empty` | on goal cancel or timeout |

At mission start it writes `runs/<run_id>/<robot>/{plan.json, track.json, scenario.json}` and
points `runs/latest` at the run.

### Planner selection (`planner`) and the curve planner (`curve`)

`planner.type` (`orienteering` | `curve`, default `orienteering`, so a scenario without the
block plans exactly as before). The curve planner:

- reads the scenario exactly as upstream `mtlc_plan` does (`src/search_problem.cpp` mirrors
  `third_party/mtl_curve_planner/apps/mtl_curve_plan_json.cpp` field for field), including
  the optional `curve` block and the automatic scaling of every absent reference length
  (grids / samples / knots × `size_m / 5000`, kernel geometry and altitude stagger ×
  `beta / 610`). Unknown `planner` / `curve` keys are refused; so is an infinite budget (the
  curve IS the budget);
- flies each agent at its planned altitude `altitude_m + a · curve.altitude_stagger_m`
  (`team.agents[].altitude_offset_m` is not added on top);
- maps its output onto the host's track samples: boresight from `sensor`, roll / pitch from
  `rpy`, speed = V, yaw = the curve tangent the boresight was built on, and
  `gimbal_phi = −gimbalCmd` (the curve's `gimbalAngle` is the level-frame sweep, + left; the
  host's `phi` is + right with `crossAngle = roll + phi`; `test_search_problem.cpp`
  rebuilds every boresight from that to < 1e-6 m);
- writes `plan.json` as `mtlc_plan` writes it (`samples.gimbal`, the curve diagnostics,
  `meta.curve`) plus `meta.planner_type` / `meta.planner_mode = "curve"`, and a `curve`
  block in `track.json` (sweep amplitude / frequency / peak rate, swath half-width, max
  curvature, endpoint, fast objective).

With `planner.compare_orienteering: true` (default) a curve mission goal also plans the
**orienteering** planner in both modes, never flown: `plan_alt_plain.json` /
`track_alt_plain.json` and `plan_alt_info_aware.json` / `track_alt_info_aware.json`.

**Planning time.** Seconds for one agent at the 5 km mission, tens of seconds for three. It
never runs on the executor: the boot preview plans on a worker thread, a goal re-uses that
plan when the scenario text is unchanged (both planners are deterministic) and otherwise
plans in the goal thread while publishing `PLANNING` feedback every second, and the
comparison plans run on a detached worker after the flown plan is published, so they never
delay the sortie.

**Gimbal law.** `airstack.follower.gimbal_law` (`mission.yaml` `follower.gimbal_law`,
default `open_loop`) is copied into every `SearchPlan.gimbal_law`, with the planned roll
(`planned_roll_rad`), for `mtl_trajectory_follower`.

### Orienteering mode toggle (`info_aware`)

With `planner.type: orienteering`, the scenario's `info_aware` block (from
`stacks/mtl_search/config/mission.yaml`) selects the planner mode that is **flown**:

| `info_aware.enabled` | Flown plan |
|---|---|
| `false` | the plain planner: k-means clusters by proximity, route scored by the mass the boresight is aimed at |
| `true` | the information-aware search (`mtl::PlannerParams::infoAware`, see `third_party/mtl_planner/CHANGES_info_aware.md`): clusters follow the prior's peaks, cut into core / shoulder / tail mass levels; several abstractions are planned, each flown plan is scored by what its footprint would detect, and split / peel / merge moves refine the best |

With `info_aware.report_both: true` (the default) a mission goal also plans the **other** mode and
writes it as `plan_alt.json` / `track_alt.json` next to the flown plan. It is never published or
flown; `mtl_metrics_logger` and `scripts/analyze_mtl_run.py` score its planned residual belief
with the same model, and the report shows both modes side by side. `plan.json` carries
`meta.planner_mode` and, for the information-aware mode, `meta.info_aware` (every candidate
abstraction, its coverage score, the chosen one); `track.json` carries `planner_mode`. The
information-aware search adds a few seconds of planning (on worker threads) before the sortie
starts; the boot preview plans the flown mode only, and the other mode is planned after the
flown plan is published.

Also built:

- `mtl_search_plan`: offline CLI, `--scenario F --out-dir D [--agent N] [--no-alt]`, for both
  planner types. It writes the same `plan.json` / `<agent>_track.json` as the node, and the
  comparison plans (`plan_alt.json` / `<agent>_track_alt.json`, or `plan_alt_<mode>.json` /
  `<agent>_track_alt_<mode>.json` when the curve flies); `scripts/mtl_offline_mission.py`
  uses it.
- `mtl_search_planner.scenario`: stdlib Python that generates scenarios from
  `stacks/mtl_search/config/mission.yaml`. It is used by `scripts/mtl_generate_scenario.py`.

## Prior and cells

The prior is a **probability mass function** over the scenario raster: it sums to 1, and
each pixel holds the probability that the target is there. This matches the vendored
planner from upstream commit `e186a1b`; see `third_party/mtl_planner/CHANGES_belief_mass_and_residual.md`.

- `mtl_search_planner.scenario` makes the raster, normalises it, and cuts it into
  `target_cell_size_m` blocks.
- It keeps a block when the block's **belief mass** (the probability that the target is in
  it) is greater than `mapping.minimum_belief_mass`. That value is a per-cell probability in
  `[0, 1)`. It replaces the old `mean_information_thresh`, which was a mean-belief test; a
  `mission.yaml` that still sets only the old key is rejected.
- `cells.mass` in `scenario.json` is therefore a probability, and `cells.total_map_mass = 1`.
- The planner's team info (`info_mass`, `info_total`) is in the same units.
- The route optimiser compares rewards as ratios, so the scale of the masses does not change
  a plan. The set of kept cells does.
- The threshold depends on how finely the map is cut: it scales with `cell² / area²`. Work it
  out again for each mission. `mission.yaml` explains the current value: `2e-3` keeps the same
  144 cells as the old `0.08`.

Planners are compared on the **residual belief mass**, `P(target missed)`, where lower is
better. `mtl_metrics_logger` scores it from the flown looks and from the planned track and
boresight schedule. The C++ reference, `mtl::eval::computeResidualBelief`, is built only
into `mtl_eval_vendored`, which the self-tests use.

## Frames

- `mtl::planner` and `mtl::curve::Planner` work internally in `x = East, y = North` over
  `[0, size]`, with the origin at the area's SW corner.
- Scenario files use mission NED: `(n, e)`.
- The Isaac world is ENU: `x_ENU = e`, `y_ENU = n`, `z_ENU = −d`.
- Yaw in the plan is ENU yaw.

## Parameters

See `config/mtl_search_planner.yaml`. The launch args set `scenario_file`, `agent_name`
(default `$ROBOT_NAME`) and `runs_root` (default `$MTL_RUNS_ROOT` or `/root/AirStack/runs`).

## Tests

- `test/test_search_problem.cpp` (gtest): scenario parsing, frames, the team solve, track
  export. It also checks that cell masses are probabilities, that
  `minimum_belief_mass` is read and validated (the old key is ignored), and that scaling
  every cell mass leaves the plan unchanged. For the curve planner: the `planner` / `curve`
  blocks and their auto-scaling (= `mtlc_plan`), refusal of unknown keys and bad values,
  the finite-budget rule, the map-frame round trip, the gimbal sign convention (every
  boresight rebuilt from pose, roll, phi and tilt to < 1e-6 m; the opposite sign misses by
  metres), track length = budget ± 0.2 %, curvature ≤ 1/R + 1e-4, return-home and
  fixed-destination endpoints < 1 m, determinism, the comparison plans, and that
  `planner.type: orienteering` through the dispatch is the orienteering plan unchanged.
- The five upstream `mtl_planner` self-tests plus `test_info_aware` (`mtl_vendored_*`), and the
  six upstream `mtl_curve_planner` self-tests (`mtl_curve_vendored_*`).
- `test/test_scenario.py`: the scenario generator, including a byte-for-byte `--check` of the
  committed bundle, the `planner` / `curve` / `follower` blocks (normalised, unknown keys and
  misspelt mission blocks refused, a curve mission checked flyable at generation), and the
  committed mission's curve calibration. It checks that the prior sums to 1, that cells are kept by mass, that the
  result does not depend on the grid's units, that a stricter threshold keeps a subset of the
  cells, and that a mission using the old key is rejected.
