# `ia_tigris_search` — single-agent IA-TIGRIS search stack

This is the `tigris_search` stack with one module swapped: the global planner is
[`ia_tigris_search_planner`](../../robot/ros_ws/src/global/planners/ia_tigris_search_planner/README.md),
which runs IA-TIGRIS (Moon et al., IEEE T-RO 2026). The tree is recycled between replans and
rewards are incremental; the reward is `matched` by default.

Everything else is the same as `tigris_search`:

- the mission, prior, targets and budget (`config/mission.yaml`);
- the gimbal setup (`gimbal_actuation` in `mission.yaml`: body-fixed camera or a constant-rate
  left/right sweep of the single-axis gimbal, on by default as in `tigris_search`);
- the follower, controllers, metrics logger, run-folder layout and analysis.

So an IA-TIGRIS run is flown and scored exactly like a TIGRIS or MTL (orienteering / curve) run.

Full walkthrough: [IA-TIGRIS search tutorial](../../docs/tutorials/ia_tigris_search.md).

## What it launches

`launch/stack.launch.xml` launches, for `robot_1`:

| Block | Module | Notes |
|---|---|---|
| Interface, perception, tasks, safety | as in `mtl_search` | |
| Global | `ia_tigris_search_planner` | `/<robot>/search_mission`, `search/plan` (revised in flight), `search/ia_tigris_status` |
| Local | `mtl_trajectory_follower` | `config/mtl_trajectory_follower_ia_tigris.yaml` = package defaults + `accept_plan_revisions: true` |
| Local | `trajectory_controller`, `pid_controller` | tracking point moved to `tracking_point_nominal`; `config/pid_controller_ia_tigris.yaml` lifts the clamp to ±6.5 m/s so the drone flies the planned 6 m/s |
| Logging | `mtl_metrics_logger` | `runs/<run_id>/<robot>/{telemetry.csv,detection.json,residual_belief.csv,report.html}` |
| Extras | DDS router (`config/dds_router_ia_tigris_search.yaml`), gossip | |

## Configuration

| File | What |
|---|---|
| `config/mission.yaml` | **Source of truth** for the mission: a copy of `tigris_search`'s (same seed, prior, targets, budget, speed, camera mount and gimbal sweep). Only `mission.name` differs. |
| `config/scenario.json`, `ground_truth.json`, `belief.png` | Generated from `mission.yaml` + `config/fleets/ia_tigris_search_fleet.yaml` |
| `config/ia_tigris_search_planner.yaml` | IA-TIGRIS tuning: reward mode (default `matched`), `recycle_tree`, planning times, replan period, extend/prune radii, grid. The stack loads this copy. |
| `config/mtl_trajectory_follower_ia_tigris.yaml` | Follower parameters (revisions on) |
| `config/pid_controller_ia_tigris.yaml` | PID gains (speed clamp) |
| `scripts/ia_tigris_sortie.sh` | Per-robot sortie (preflight, rosbag, takeoff, goal watchdog), run inside the robot container |

After editing `mission.yaml`:

```bash
python3 scripts/ia_tigris_generate_scenario.py                # regenerate the bundle
python3 scripts/ia_tigris_generate_scenario.py --compare-mtl  # same problem as mtl_search?
python3 scripts/ia_tigris_generate_scenario.py --compare-mtl --mtl-dir stacks/tigris_search/config   # ... as tigris_search?
```

## How to run

```bash
airstack up --sim isaac --fleet ia_tigris_search_fleet --stack ia_tigris_search --play --wait   # the fleet picks search_mission_scene.py + this bundle
airstack ready
bash scripts/ia_tigris_start_mission.sh              # takeoff + search_mission, then analysis
python3 scripts/analyze_tigris_run.py --run-dir runs/latest
```

## Known limits

- Each in-flight solve blocks the planner's sortie thread for `planning_time_s` (5 s, the
  same as `tigris_search`). Recycling the tree (recomputing every reward on the new belief)
  counts against that time, as in IA-TIGRIS. The drone keeps flying the committed track
  meanwhile.
- The commit point is snapped forward to the end of the tree edge the drone will be on, so the
  drone can be committed further ahead than with TIGRIS: up to one `extend_dist_m` (750 m).
- Detection is model-based, as in `mtl_search`. The planner treats every look as a miss
  and never sees the ground truth.
- A `run_id` the node has already flown is given a `_2` suffix. Use a fresh id; the launcher
  defaults to a UTC timestamp.
- `wiring.md` has not been generated yet (see `mtl_search/README.md` for how).
