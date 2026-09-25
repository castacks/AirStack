# mtl_search_planner

The global planner of the [`mtl_search`](../../../../../../stacks/mtl_search/README.md) stack. It
wraps `mtl::planner`, which is vendored unchanged in `third_party/mtl_planner`
(see [VENDORED.md](third_party/VENDORED.md)). The planner turns a prior belief and a team
of aircraft into:

- per-agent Dubins search tracks within an endurance budget;
- gimbal boresight schedules for those tracks.

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

Also built:

- `mtl_search_plan`: offline CLI, `--scenario F --out-dir D [--agent N]`. It writes the same
  `plan.json` / `<agent>_track.json` as the node, and `scripts/mtl_offline_mission.py`
  uses it.
- `mtl_search_planner.scenario`: stdlib Python that generates scenarios from
  `stacks/mtl_search/config/mission.yaml`. It is used by `scripts/mtl_generate_scenario.py`.

## Frames

- `mtl::planner` works internally in `x = East, y = North` over `[0, size]`, with the origin
  at the area's SW corner.
- Scenario files use mission NED: `(n, e)`.
- The Isaac world is ENU: `x_ENU = e`, `y_ENU = n`, `z_ENU = −d`.
- Yaw in the plan is ENU yaw.

## Parameters

See `config/mtl_search_planner.yaml`. The launch args set `scenario_file`, `agent_name`
(default `$ROBOT_NAME`) and `runs_root` (default `$MTL_RUNS_ROOT` or `/root/AirStack/runs`).

## Tests

- `test/test_search_problem.cpp` (gtest): scenario parsing, frames, the team solve, track
  export.
- The five upstream `mtl_planner` self-tests.
- `test/test_scenario.py`: the scenario generator, including a byte-for-byte `--check` of the
  committed bundle.
