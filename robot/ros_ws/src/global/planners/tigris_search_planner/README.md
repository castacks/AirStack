# tigris_search_planner

Single-agent **TIGRIS** (Moon, Chatterjee, Scherer, *TIGRIS: An Informed Sampling-based
Algorithm for Informative Path Planning*, ICRA 2023) as an AirStack global planner. It is the
baseline for the MTL planner: it is flown by the same follower and scored by the same logger,
so the two planners' run folders can be compared directly.

Full walkthrough: [TIGRIS baseline tutorial](../../../../../../docs/tutorials/tigris_baseline.md).
Stack: [`tigris_search`](../../../../../../stacks/tigris_search/README.md).

## What it does

The node serves the same `SearchMission` interface as `mtl_search_planner`. The unchanged
`mtl_trajectory_follower` flies the plan, and the unchanged `mtl_metrics_logger` scores it.

1. **Initial solve.** When the goal arrives, the node runs one TIGRIS solve from the hover
   pose with the whole budget (`initial_planning_time_s`).
2. **Receding horizon.** Every `replan_period_s`, or when the drone nears the end of the
   track, it replans:
   - it folds the looks actually flown into the belief (measured pose and gimbal);
   - it keeps the track up to a commit point ahead of the drone;
   - it runs TIGRIS again from that point with the remaining budget;
   - it republishes the whole track under the same `plan_id` with a newer stamp.
3. **Follower revisions.** The follower accepts these as in-flight revisions
   (`accept_plan_revisions: true` in the stack). Because the prefix is unchanged, its
   arc-length progress stays valid.
4. **No gimbal actuation.** TIGRIS has no gimbal trajectory, so the plan tells the follower
   the cross-track axis and pitch nudge are locked (`1e-6` rad). The camera is body-fixed at
   the mission's forward tilt and turns with the airframe.

| Interface | Type | Notes |
|---|---|---|
| `/<robot>/search_mission` | action `mtl_msgs/SearchMission` | goal `{start_mission, run_id, scenario_file}` |
| `search/plan` | `mtl_msgs/SearchPlan` (transient local) | the whole sortie; revised in flight (same `plan_id`, newer stamp) |
| `search/planned_trajectory`, `search/planned_path`, `search/planned_boresight` | standard AirStack / `nav_msgs` | the current track and the body-fixed boresight ground points |
| `search/markers` | `MarkerArray` | area, valid cells, cells the track sees |
| `search/tigris_status` | `std_msgs/String` (JSON, transient local) | the latest solve: trigger, commit point, budget left, iterations, tree size, rewards |
| in: `search/follower_status`, `odometry`, `gimbal/state`, `gimbal/cmd_pitch_yaw` | | progress, flown looks |

The planner writes these into `runs/<run_id>/<agent>/`:

- `plan.json` (`mtl.plan/1`);
- `track.json` (`mtl.agent_track/1`, rewritten at every replan);
- `scenario.json`;
- `tigris_replans.json`: every solve, plus the parameters.

## How close to the original

The algorithm is a line-by-line port of `tigris/src/ipp.cpp` and
`MapRepresentation.cpp` (ROS 1, OMPL), with `planner_type = OURS` and `use_entropy = true`.

**Kept as in the original:**

- the informed sampler (`updateInformedConfig` / `informedConfig`, including the int truncation);
- `steer`: sample walk, cut at `extend_dist` or the budget, `closed` flag, the
  straight-segment (`start_edge`/`end_edge`) bookkeeping, chord-sum cost;
- `prune`;
- the main loop and the near-node rewiring loop. This includes the original's quirk: the loop
  compares the *sampled* node (`motion_feasible`) with the best path, and still uses pruned
  nodes as the rewiring target;
- the root's information of about 0 (`ipp.cpp` scores the root before setting its state);
- `informationGain`: each node's footprint, then that node's straight edge, from root to leaf.
  Turning arcs earn nothing;
- cells count only when all four corners are inside, and range is measured to the cell corner;
- the node and edge reward formulas (entropy, `Rs = 2`, `Rf = 1`);
- `tpr` / `fpr = 1 - tpr` with the `p > 0.5` branch;
- the scaled launch parameters (`extend_dist 750 → 60 m`, `extend_radius 251 → 20 m`,
  `prune_radius 750 → 60 m`, `RESOLUTION 50 → 4 m`, `planning_time 5 s`).

**Different only because AirStack or the study requires it:**

| Original | Here | Why |
|---|---|---|
| OMPL GNAT nearest neighbours | exact spatial hash | no OMPL; returns the same neighbours |
| trochoid steer | Dubins | AirStack has no wind; a zero-wind trochoid is a Dubins path |
| rectangular frustum footprint (width/height/focal, pitch) | the mission's cone (`sensor.fov_deg`, `sensor.tilt_deg`) | the camera mount is a parameter shared with MTL and the logger. The node footprint is a ground disc; the straight-edge swath is that disc swept along the edge; "nearest viewing pose" is derived for the disc |
| `tpr` = TIGRIS sigmoid, flat 0.5 past 600 m | the scenario's Moon et al. sigmoid, 0.5 past `beta` | the same sensor model the logger scores |
| `std::random_device` | seeded (mission seed + replan index) | reproducible runs |
| one solve per request, map supplied externally | receding horizon; the belief is updated by the flown looks as misses | the study setting (MTL flies the whole budget; no detector runs) |

`reward_mode: matched` is an **addition**, not TIGRIS. It swaps only the reward, for the
expected drop in residual belief mass (the logger's metric) sampled along the whole path. The
default is `original`.

## Parameters

The **mission** is read from the scenario (`stacks/tigris_search/config/mission.yaml`),
exactly like MTL:

- budget: `team.max_flight_time_s`, `team.max_flight_distance_m`;
- speed, turn radius and altitude;
- camera mount: `sensor.fov_deg`, `sensor.tilt_deg`;
- detection model and prior.

TIGRIS tuning lives in `config/tigris_search_planner.yaml`; the stack loads its own copy from
`stacks/tigris_search/config/`. The overrides `budget_m`, `camera_fov_deg` and
`camera_tilt_deg` exist, but change the mission instead: the logger scores the scenario's FOV.

## Also built

- `tigris_search_plan`: a ROS-free CLI that runs the whole receding-horizon sortie
  kinematically and writes the same run files (`--scenario F --out-dir D [--set key=value]
  [--one-shot]`). `scripts/tigris_offline_mission.py` drives it.
- `tigris_search_planner.rewards`: a stdlib mirror of the two reward models, used by
  `scripts/analyze_tigris_run.py`.

## Tests

- `test/test_tigris_core.cpp` (gtest) covers:
  - scenario parsing;
  - Dubins endpoints;
  - footprint geometry;
  - grid mass;
  - both reward models;
  - determinism and budget of a solve;
  - the committed prefix across a replan;
  - the `track.json` layout.
- `test/test_rewards.py` covers the Python mirror.
- The follower's `test/test_plan_revisions.py` covers in-flight revisions.
