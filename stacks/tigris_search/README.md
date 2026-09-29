# `tigris_search` — single-agent TIGRIS baseline search stack

This is the `mtl_search` stack with one module swapped: the global planner is
[`tigris_search_planner`](../../robot/ros_ws/src/global/planners/tigris_search_planner/README.md).
It runs TIGRIS (Moon et al., ICRA 2023) in a receding horizon. By default there is no gimbal
actuation; `gimbal_actuation.enabled: true` in `config/mission.yaml` sweeps the single-axis
gimbal left and right at a constant rate instead. The
follower, controllers, metrics logger, run-folder layout and analysis are the MTL ones, so a
TIGRIS run and an MTL run are flown and scored identically.

Full walkthrough: [TIGRIS baseline tutorial](../../docs/tutorials/tigris_baseline.md).

## What it launches

`launch/stack.launch.xml` launches, for `robot_1`:

| Block | Module | Notes |
|---|---|---|
| Interface, perception, tasks, safety | as in `mtl_search` | |
| Global | `tigris_search_planner` | `/<robot>/search_mission`, `search/plan` (revised in flight), `search/tigris_status` |
| Local | `mtl_trajectory_follower` | `config/mtl_trajectory_follower_tigris.yaml` = package defaults + `accept_plan_revisions: true` |
| Local | `trajectory_controller`, `pid_controller` | tracking point moved to `tracking_point_nominal`; `config/pid_controller_tigris.yaml` lifts the clamp to ±6.5 m/s so the drone flies the planned 6 m/s |
| Logging | `mtl_metrics_logger` | `runs/<run_id>/<robot>/{telemetry.csv,detection.json,residual_belief.csv,report.html}` |
| Extras | DDS router (`config/dds_router_tigris_search.yaml`), gossip | |

## Configuration

| File | What |
|---|---|
| `config/mission.yaml` | **Source of truth** for the mission. It is a copy of `mtl_search`'s, with the same seed, prior and targets. It holds the **budget** (`team.max_flight_time_s` / `max_flight_distance_m`), the **speed** (`aircraft.speed_mps`) and the **camera mount** (`sensor.fov_deg`, `sensor.tilt_deg`) and the optional **gimbal sweep** (`gimbal_actuation`: `enabled`, `sweep_rate_deg_s`, `sweep_amplitude_deg`). |
| `config/scenario.json`, `ground_truth.json`, `belief.png` | Generated from `mission.yaml` + `config/fleets/tigris_search_fleet.yaml` |
| `config/tigris_search_planner.yaml` | TIGRIS tuning: reward mode, sampler, planning times, replan period, extend/prune radii, grid. The stack loads this copy. |
| `config/mtl_trajectory_follower_tigris.yaml` | Follower parameters (revisions on) |
| `config/pid_controller_tigris.yaml` | PID gains (speed clamp) |
| `scripts/tigris_sortie.sh` | Per-robot sortie (preflight, rosbag, takeoff, goal watchdog), run inside the robot container |

After editing `mission.yaml`:

```bash
python3 scripts/tigris_generate_scenario.py                # regenerate the bundle
python3 scripts/tigris_generate_scenario.py --compare-mtl  # confirm it is the same problem as mtl_search
```

## How to run

```bash
airstack up --sim isaac --fleet tigris_search_fleet --stack tigris_search --play --wait   # the fleet picks search_mission_scene.py + this bundle
airstack ready
bash scripts/tigris_start_mission.sh                 # takeoff + search_mission, then analysis
python3 scripts/analyze_tigris_run.py --run-dir runs/latest
```

## Known limits

- Each in-flight solve blocks the planner's sortie thread for `planning_time_s` (5 s, the
  TIGRIS default). The drone keeps flying the committed track meanwhile.
- Detection is model-based, as in `mtl_search`. The planner treats every look as a miss
  and never sees the ground truth.
- A `run_id` the node has already flown is given a `_2` suffix. Use a fresh id; the launcher
  defaults to a UTC timestamp.
- `wiring.md` has not been generated yet (see `mtl_search/README.md` for how).
