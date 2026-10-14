# `ipp_baselines_search` — the ia-tigris baselines (greedy, rect_coverage, MCTS)

This is the `ia_tigris_search` stack with one module swapped: the global planner is
[`ipp_baselines_search_planner`](../../robot/ros_ws/src/global/planners/ipp_baselines_search_planner/README.md),
which runs one of the baselines released with IA-TIGRIS (`ia-tigris/src/ipp_planners`):
`greedy`, `rect_coverage` (lawnmower coverage) or `mcts_search` (Monte Carlo tree search).
Which one is set by `planner:` in `config/ipp_baselines_search_planner.yaml`, read at launch
(no rebuild), or per sortie with `scripts/ipp_baselines_start_mission.sh --planner NAME`.

Everything else is the same as `ia_tigris_search` / `tigris_search`:

- the mission, prior, targets and budget (`config/mission.yaml`, only `mission.name` differs);
- the gimbal setup (`gimbal_actuation`: body-fixed camera or a constant-rate left/right sweep);
- the follower, controllers, metrics logger, run-folder layout and analysis.

So a baseline run is flown and scored exactly like an IA-TIGRIS, TIGRIS or MTL run.

Full walkthrough: [IPP baselines tutorial](../../docs/tutorials/ipp_baselines_search.md).

## What it launches

`launch/stack.launch.xml` launches, for `robot_1`:

| Block | Module | Notes |
|---|---|---|
| Interface, perception, tasks, safety | as in `mtl_search` | |
| Global | `ipp_baselines_search_planner` | `/<robot>/search_mission`, `search/plan` (revised in flight), `search/ipp_status` |
| Local | `mtl_trajectory_follower` | `config/mtl_trajectory_follower_ipp_baselines.yaml` = package defaults + `accept_plan_revisions: true` |
| Local | `trajectory_controller`, `pid_controller` | `config/pid_controller_ipp_baselines.yaml` lifts the clamp to ±6.5 m/s so the drone flies the planned 6 m/s |
| Logging | `mtl_metrics_logger` | `runs/<run_id>/<robot>/{telemetry.csv,detection.json,residual_belief.csv,report.html}` |
| Extras | DDS router (`config/dds_router_ipp_baselines_search.yaml`, allows `search/ipp_status`), gossip | |

## Configuration

| File | What |
|---|---|
| `config/mission.yaml` | **Source of truth** for the mission: a copy of `ia_tigris_search`'s. Only `mission.name` differs. |
| `config/scenario.json`, `ground_truth.json`, `belief.png` | Generated from `mission.yaml` + `config/fleets/ipp_baselines_search_fleet.yaml` |
| `config/ipp_baselines_search_planner.yaml` | `planner`, reward mode (default `matched`), planning times, replan period, grid, the per-planner parameters (`rect_coverage_*`, `mcts_*`). The stack loads this copy. |
| `config/mtl_trajectory_follower_ipp_baselines.yaml` | Follower parameters (revisions on) |
| `config/pid_controller_ipp_baselines.yaml` | PID gains (speed clamp) |
| `scripts/ipp_baselines_sortie.sh` | Per-robot sortie (preflight, `--planner`, rosbag, takeoff, goal watchdog), run inside the robot container |

After editing `mission.yaml`:

```bash
python3 scripts/ipp_baselines_generate_scenario.py                # regenerate the bundle
python3 scripts/ipp_baselines_generate_scenario.py --compare-mtl  # same problem as mtl_search?
```

## How to run

```bash
airstack up --sim isaac --fleet ipp_baselines_search_fleet --stack ipp_baselines_search --play --wait
airstack ready
bash scripts/ipp_baselines_start_mission.sh --planner mcts_search   # takeoff + search_mission, then analysis
python3 scripts/analyze_tigris_run.py --run-dir runs/latest
```

Offline (no Isaac): `python3 scripts/ipp_baselines_offline_mission.py --set planner=greedy`.

## Known limits

- Each in-flight solve blocks the planner's sortie thread for up to `planning_time_s` (5 s).
  greedy and rect_coverage finish their solve much sooner (they end the loop themselves).
- The commit point is snapped forward to the end of the segment the drone will be on
  (`ipp_planners_node.h`), so with long segments (greedy view points, the coverage lanes) the
  drone can be committed far ahead. rect_coverage's first segment is the transit to the
  area's corner, about 3.3 km here, so it is committed for the whole 3 km budget.
- The published track ends at the budget; greedy and rect_coverage plan past it (as written).
- Detection is model-based, as in `mtl_search`. The planner treats every look as a miss
  and never sees the ground truth.
- A `run_id` the node has already flown is given a `_2` suffix. Use a fresh id.
- `wiring.md` has not been generated yet (see `mtl_search/README.md` for how).
