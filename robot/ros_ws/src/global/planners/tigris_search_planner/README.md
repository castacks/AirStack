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
   - it scores the rest of the plan it is already flying with the same TIGRIS reward on the
     same belief, and switches only if the new path scores higher (otherwise it keeps the
     current plan). Every solve grows a fresh random tree at the commit point, so replacing the
     plan unconditionally re-drew the heading every `replan_period_s` and the drone weaved back
     and forth;
   - on a switch, it republishes the whole track under the same `plan_id` with a newer stamp.
3. **Follower revisions.** The follower accepts these as in-flight revisions
   (`accept_plan_revisions: true` in the stack). Because the prefix is unchanged, its
   arc-length progress stays valid.
4. **Gimbal: locked by default, optional sweep.** TIGRIS has no gimbal trajectory, so by
   default the plan tells the follower the cross-track axis and pitch nudge are locked
   (`1e-6` rad). The camera is body-fixed at the mission's forward tilt and turns with the
   airframe. With `gimbal_actuation.enabled: true` in the mission file the cross-track axis
   sweeps left and right at a constant rate instead (see [Gimbal actuation](#gimbal-actuation)).

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
- the launch parameters (`extend_dist 750 m`, `extend_radius 251 m`, `prune_radius 750 m`,
  `RESOLUTION 50 m`, `planning_time 5 s`). The mission is at the original 5 km scale, so they
  are unscaled; scale them with the area if the mission shrinks (they were 60 / 20 / 60 / 4 m
  for the old 400 m mission, and left at that size on the 5 km map they made the track
  zig-zag every 60 m).

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

## Gimbal actuation

`mission.yaml` (TIGRIS only, copied to `scenario.json` as `airstack.gimbal_actuation`):

```yaml
gimbal_actuation:
  enabled: false            # true: sweep the single-axis gimbal left <-> right
  sweep_rate_deg_s: 30.0    # constant cross-track angular speed
  sweep_amplitude_deg: 45.0 # between -A (left) and +A (right)
```

When it is enabled, the cross-track angle is a triangle wave in **planned track time**:
`phi(t)` goes 0 → +A → 0 → −A → 0 at a constant `|dphi/dt|` = rate (period 4 A / rate).
It starts at the first track sample. Replans keep the track time running, so the phase is
continuous across revisions. Each look of the single-axis mount at `phi` hits the ground
`h tan(tilt) / cos(phi)` ahead and `h tan(phi)` to the right. Its slant is
`h / (cos(tilt) cos(phi))`, and its footprint radius is that slant times `tan(fov / 2)`.
This is the constraint plane that `mtl_trajectory_follower`'s `single_axis_command` points in.

- **Plan.** Every sample carries the swept boresight ground point and
  `planned_gimbal_phi_rad`. The cross-track travel opens to 80°, and the pitch nudge stays
  locked while `lock_gimbal: true`. The unchanged follower points the gimbal at the swept
  boresight.
- **Residual belief.** The committed and planned looks (and the CLI's stand-in for the flown
  looks) use the swept footprint. The node's flown looks come from the measured gimbal, as
  before. `residual_mass_after_flown` in `tigris_replans.json` and the logger's planned and
  flown residual therefore follow the sweep.
- **TIGRIS rewards** (the only change to the tree search, and only when the sweep is on):
  - `original`: the node footprint is the disc at the node's `phi`. The straight-edge swath is
    the union of the swept discs every `reward_step_m` along the segment. Each cell entirely
    inside one of them is updated once with the edge formula, at its range from the nearest
    such pose.
  - `matched`: the edge looks use `phi`.

  The sampler, steer, prune and rewiring are unchanged. With `enabled: false` the planner's
  output is byte-identical to the body-fixed baseline.
- **Checks at load.** The node logs a warning when the sweep's peak earth-frame axis rate
  exceeds the gimbal slew rate (`sim_gimbal.slew_rate_deg_s`). Near the centre, roll moves
  `1 / sin(tilt)` = 2× faster than `phi`. It also warns when `|roll|` exceeds
  `sim_gimbal.roll_limit_deg`, or when the end slant exceeds `detection.beta`.

## Parameters

The **mission** is read from the scenario (`stacks/tigris_search/config/mission.yaml`),
exactly like MTL:

- budget: `team.max_flight_time_s`, `team.max_flight_distance_m`;
- speed, turn radius and altitude;
- camera mount: `sensor.fov_deg`, `sensor.tilt_deg`;
- gimbal sweep: `gimbal_actuation` (off by default);
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
  - the `track.json` layout;
  - the gimbal sweep: parsing, the constant-rate triangle wave, the swept look geometry,
    the earth-frame kinematics against the follower's solution, the phase across a replan,
    the swept residual belief, and the sweep-aware tree rewards.
- `test/test_rewards.py` covers the Python mirror.
- The follower's `test/test_plan_revisions.py` covers in-flight revisions.
