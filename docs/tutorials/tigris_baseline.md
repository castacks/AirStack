# TIGRIS baseline search in Isaac Sim

This tutorial flies **one** PX4 multirotor with the TIGRIS informative path planner (Moon et
al., ICRA 2023) over the same 400 m × 400 m search area, prior and 15 ground targets as the
[MTL tutorial](mtl_target_localization.md). The camera does not move on its gimbal. The run
folder has exactly the MTL layout and metrics, so a 1-robot MTL run and a TIGRIS run can be
compared file for file.

| Piece | Where |
|---|---|
| Stack | [`stacks/tigris_search`](../../stacks/tigris_search/README.md) |
| Planner | [`tigris_search_planner`](../../robot/ros_ws/src/global/planners/tigris_search_planner/README.md) (TIGRIS port + receding horizon) |
| Follower, logger, messages | shared with MTL: `mtl_trajectory_follower`, `mtl_metrics_logger`, `mtl_msgs` |
| Fleet | `config/fleets/tigris_search_fleet.yaml` (`robot_1`, same spawn as MTL's `robot_1`) |
| Scripts | `scripts/tigris_generate_scenario.py`, `tigris_start_mission.sh`, `analyze_tigris_run.py`, `tigris_offline_mission.py` |

## How the pieces fit

- **Planning.** `tigris_search_planner` runs a TIGRIS solve from the hover pose with the
  whole budget, 540 m = 90 s × 6 m/s.
- **Following.** The follower flies the track by arc-length carrot pursuit.
- **Replanning.** Every 5 s, and early when the drone nears the end of the track, the planner:
  1. folds the looks actually flown into its belief;
  2. freezes the track up to a commit point ahead of the drone (carrot distance + 6 s of
     flight);
  3. runs TIGRIS again from there with the remaining budget;
  4. republishes the whole track under the same `plan_id`.

  The follower (`accept_plan_revisions: true`) swaps the track in flight without losing its
  progress.
- **No gimbal actuation.** The plan locks the cross-track axis, so the camera is body-fixed:
  60° cone, tilted 30° forward from nadir, the same mount as MTL.
- **Scoring.** The logger scores the flown pose and the measured gimbal with the Moon et al.
  sigmoid and writes the residual belief mass. That is the planner-comparison number; lower
  is better.
- **Analysis.** `analyze_tigris_run.py` produces the same team outputs as
  `analyze_mtl_run.py`. It also always plots **both** TIGRIS rewards, original and matched,
  for the flown and the planned path.

## 0. (Optional) change the mission, budget, speed or camera

Everything about the mission is in `stacks/tigris_search/config/mission.yaml`. It is a copy
of the MTL one, so both planners solve the same problem.

| What | Key |
|---|---|
| Budget (the tighter one binds) | `team.max_flight_time_s`, `team.max_flight_distance_m` |
| Speed / turn radius / altitude | `aircraft.speed_mps`, `aircraft.min_turn_radius_m`, `aircraft.altitude_m` |
| Camera mount (body-fixed) | `sensor.fov_deg` (full cone), `sensor.tilt_deg` (forward from nadir) |
| Detection model | `sensor.detection` |

After editing, regenerate the bundle and check it still matches MTL:

```bash
python3 scripts/tigris_generate_scenario.py                # -> stacks/tigris_search/config/{scenario.json,ground_truth.json,belief.png}
python3 scripts/tigris_generate_scenario.py --compare-mtl  # prior, cells, targets, sensor, aircraft, per-agent budget, robot_1 home
```

If you raise `aircraft.speed_mps` above 6.5 m/s, also raise `x_min/x_max/y_min/y_max` in
`stacks/tigris_search/config/pid_controller_tigris.yaml`. The position-loop clamp caps the
flown speed.

TIGRIS tuning is in `stacks/tigris_search/config/tigris_search_planner.yaml`. That file is
read at launch from the mounted stack folder, so a change needs only a restart, not a rebuild.

| Parameter | Default | What it does |
|---|---|---|
| `reward_mode` | `original` | TIGRIS's entropy reward, or `matched` (residual belief mass; an addition, not TIGRIS) |
| `sampler` | `informed` | or `uniform` (the paper's random benchmark) |
| `receding` | `true` | `false` = one plan for the whole budget |
| `initial_planning_time_s`, `planning_time_s` | 5, 5 | the TIGRIS `planning_time` |
| `replan_period_s` | 5 | cadence of the in-flight solves |
| `extend_dist_m`, `extend_radius_m`, `prune_radius_m`, `grid_res_m` | 60, 20, 60, 4 | the TIGRIS launch values 750 / 251 / 750 / 50 m, scaled by the mission's 1/12.5 |
| `max_iterations`, `seed` | 0, mission seed | set `max_iterations > 0` for exactly reproducible solves |

## 1. Build (first time, or after changing the packages)

Bring the stack up once so the robot container exists, build inside it, then restart:

```bash
airstack up --sim isaac --fleet tigris_search_fleet --stack tigris_search --play --wait

docker exec airstack-robot-desktop-1 bash -ic \
  "bws --packages-up-to mtl_msgs tigris_search_planner mtl_trajectory_follower mtl_metrics_logger"

airstack down
```

The first `airstack up` can log a launch error for `tigris_search_planner`, because the
package is not built yet. That is expected; the container stays up for `bws`.

!!! note "Why rebuild the follower"
    `mtl_trajectory_follower` gained the `accept_plan_revisions` parameter. It defaults to
    `false`, so MTL behaviour is unchanged, but the package must be rebuilt once.

Optional: run the unit tests in the container.

```bash
docker exec airstack-robot-desktop-1 bash -ic "cd /root/AirStack/robot/ros_ws && \
  colcon test --packages-select tigris_search_planner mtl_trajectory_follower && colcon test-result --verbose"
```

## 2. Bring up and check

```bash
airstack up --sim isaac --fleet tigris_search_fleet --stack tigris_search --play --wait
airstack ready
```

**Scene selection comes from the fleet file.** `tigris_search_fleet.yaml` sets
`sim.script: search_mission_scene.py` and `sim.scenario_dir: stacks/tigris_search/config`:

- `--fleet` launches the scene with the search area, the belief texture, the targets and
  the gimbal camera. The drone spawns at the SW corner of the search area (ENU −170, −170),
  exactly like `mtl_search_fleet`'s robot_1. The `airstack up` log shows
  `--fleet tigris_search_fleet → ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py [fleet sim.script]`.
- The scene reads this stack's bundle, so a changed `mission.yaml` is rendered too.
- Explicit `ISAAC_SIM_SCRIPT_NAME` / `MTL_SCENARIO_DIR` still override the fleet (the old
  two-variable command keeps working).

!!! warning "Drone in the middle of a plain grey field?"
    That is the generic `fleet_spawn.py` scene (no search area, no targets, a rigid ZED
    camera instead of the gimbal). Check `.airstack/runs/<latest>/effective_config.env`:
    it must say `ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py`. If it says
    `fleet_spawn.py`, an older `airstack.sh` or an exported `ISAAC_SIM_SCRIPT_NAME` is in
    play; run `unset ISAAC_SIM_SCRIPT_NAME`, then `airstack down` and `airstack up` again.

Sanity checks (robot 1, ROS domain 1):

```bash
R="docker exec airstack-robot-desktop-1 bash -ic"
$R "sws; ros2 node list | grep -E 'tigris|mtl_'"                        # /robot_1/tigris_search_planner, mtl_trajectory_follower, mtl_metrics_logger
$R "sws; ros2 topic echo --once /robot_1/search/plan --field plan_id"     # tigris_search_small/robot_1/preview  (boot preview solve)
$R "sws; ros2 topic echo --once /robot_1/search/tigris_status"            # JSON of the preview solve (iterations, tree, rewards)
$R "sws; ros2 param get /robot_1/mtl_trajectory_follower accept_plan_revisions"   # True
$R "sws; ros2 param get \$(ros2 node list | grep pid_controller | head -1) x_max"   # 6.5  (6 m/s is reachable)
$R "sws; ros2 topic echo --once /robot_1/gimbal/state"                    # parked: y ~ 1.047
```

## 3. Fly the TIGRIS sortie

```bash
bash scripts/tigris_start_mission.sh            # -n 1 by default; -r RUN_ID, --no-images, --no-record, --dry-run
```

This runs `stacks/tigris_search/scripts/tigris_sortie.sh` inside the robot container. That
script does the same as the MTL one, with the same sortie client
(`stacks/mtl_search/scripts/mtl_sortie_client.py --tag tigris_sortie`):

1. Preflight: `tigris_search_planner`, `/robot_1/search_mission` and `/robot_1/tasks/takeoff`
   must be up, or the drone stays on the ground (exit 3).
2. Starts a rosbag (MCAP) that also records `search/tigris_status`, and waits until the
   recorder has finished subscribing.
3. One ROS node waits for a healthy state estimate and takes off to 30 m. The takeoff goal is
   confirmed from the goal response **or** the server's `_action/status` topic, and re-sent
   (new goal id, 10 s per attempt, 4 attempts) if the request was lost.
4. The same node sends the `SearchMission` goal with the same confirmation and retries.
5. Streams feedback to the end. Ctrl-C (here or on the host launcher) cancels the active goal
   and closes the bag.

!!! note "Why not `ros2 action send_goal`"
    The first version of this script took off with the `ros2 action send_goal` CLI. Each CLI
    call is a new DDS participant; on the loaded sim graph its goal request can be dropped
    before the server has matched it, and the CLI then waits forever. The log stopped at
    `takeoff to 30.0 m at 2 m/s` and the drone never armed. That is the MTL "drone doesn't
    take off" bug, fixed the same way here.

When the sortie ends, the script runs `analyze_tigris_run.py` and the Foxglove export, and
points `runs/latest` at the run.

The same thing by hand (the plain CLI; it can hang if a goal request is lost, see above):

```bash
docker exec -e ROS_DOMAIN_ID=1 airstack-robot-desktop-1 bash -ic "sws; \
  ros2 action send_goal /robot_1/tasks/takeoff task_msgs/action/TakeoffTask '{target_altitude_m: 30.0, velocity_m_s: 2.0}' && \
  ros2 action send_goal --feedback /robot_1/search_mission mtl_msgs/action/SearchMission '{start_mission: true, run_id: \"tigris_001\"}'"
```

While it flies:

- The planner log (in the robot launch tmux, `airstack connect robot-desktop-1`) prints one
  line per solve: `replan N [period] at 122 m (commit 173 m, 367 m left): new segment 366 m in
  5.00 s, 8160 it, tree 991, ...`.
- `/robot_1/search/tigris_status` has the same data as JSON.
- `/robot_1/search/follower_status` shows `SEARCH`, the progress and the cross-track error. The
  follower log prints `sortie ... revised` at every new segment.
- Camera sanity check:
  - `gimbal/cmd_pitch_yaw` should hold `y = 1.047` (60° down = 30° forward tilt), with `z`
    equal to the vehicle heading;
  - `x` (roll) should stay 0;
  - that is the locked mount.

Cancelling the goal aborts the sortie, as in MTL.

## 4. Analyze

```bash
python3 scripts/analyze_tigris_run.py --run-dir runs/latest
```

**Part 1: identical to MTL.** It uses the same `mtl_metrics_logger` code as
`analyze_mtl_run.py` and writes:

- `runs/<run_id>/telemetry.csv`, `detection.json`, `residual_belief.csv`, `report.html`;
- a printed summary starting with the **residual belief mass**, flown and planned. The planned
  value scores the final stitched `track.json`.

**Part 2: both TIGRIS rewards, always.**

| File | Content |
|---|---|
| `tigris_rewards.json` | parameters, curves (flown, planned, logger residual), summary, and every solve from `robot_1/tigris_replans.json` |
| `tigris_rewards.csv` / `tigris_rewards_planned.csv` | `t_s, reward_original, reward_matched` |
| `tigris_report.html` | headline tiles; **original TIGRIS reward vs time** (flown vs planned); **matched reward = searched belief mass vs time** (flown and planned on the planning grid, plus the logger's 2 m curve); the replan table |

- **Original reward.** The flown and planned paths are cut into passes of `extend_dist_m`
  (60 m). Every cell a pass sees gets one TIGRIS update at the best range of the pass: entropy
  reduction × Rs/Rf, tpr/fpr Bayes update. The planner's own reward of each new segment is in
  the replan table.
- **Matched reward.** The matched curve is 1 − residual. The logger curve is the authoritative
  one.

Per-robot outputs: `runs/<run_id>/robot_1/`. They are:

- the logger's `telemetry.csv`, `detection.json`, `residual_belief.csv` and `report.html`;
- the planner's `plan.json`, `track.json`, `scenario.json` and `tigris_replans.json`;
- `bag/`.

Replay in Foxglove exactly as for MTL: `python3 scripts/mtl_foxglove.py --run-dir runs/latest`.

## 5. Compare with a 1-robot MTL run

1. Fly MTL with one agent (your `num agents = 1` setup) on the same `mission.yaml` values.
   Confirm the problems still match:
   ```bash
   python3 scripts/tigris_generate_scenario.py --compare-mtl
   ```
   It checks the prior, cells, targets, sensor, aircraft, per-agent budget and robot_1 home.
2. Headline metric: `summary.residual_belief_mass` in each run's `detection.json`. Also see
   `targets_detected`, `mean_time_to_discovery_s` and `total_path_length_m`. Lower residual is
   better.
3. Score the MTL run with both TIGRIS rewards too, without touching its MTL outputs:
   ```bash
   python3 scripts/analyze_tigris_run.py --run-dir runs/<mtl_run_id> --rewards-only
   ```
   An MTL run has no `tigris_replans.json`, so the TIGRIS default reward parameters apply.
   Those are the same values a default TIGRIS run uses, so the two `tigris_rewards.json` files
   are directly comparable.

Compare planners only on the same prior, sensor (`fov`, `a`, `b`, `c`, `beta`,
`p_out_of_range`), `dt_ref` and budget.

## 6. Offline rehearsal (no Isaac)

The ROS-free CLI runs the whole receding-horizon sortie kinematically, assuming the committed
track is flown perfectly. The script then flies the final track through the real follower
code, with the gimbal locked, and scores it with the logger:

```bash
docker exec airstack-robot-desktop-1 bash -ic "cd /root/AirStack && python3 scripts/tigris_offline_mission.py"
# -> runs/tigris-offline/{robot_1/, report.html, tigris_report.html, ...}
python3 scripts/tigris_offline_mission.py --set reward_mode=matched --run-id tigris-offline-matched   # overrides
```

It uses the parameters in `stacks/tigris_search/config/tigris_search_planner.yaml`. Pass
`--planner-bin PATH` if the CLI is not in the container's install tree. This is a rehearsal to
catch frame and indexing problems, not a flight-dynamics result.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Sortie log: *tigris_search_planner or its search_mission action is not running* | package not built (step 1), or a launch error; check the robot launch tmux |
| Goal accepted, then about 5 s with no motion | normal: the initial TIGRIS solve (`initial_planning_time_s`) |
| Track never extends; follower reaches `COMPLETE` early | the follower is not accepting revisions: the stack must load `mtl_trajectory_follower_tigris.yaml` (check `accept_plan_revisions`), or the follower was not rebuilt |
| Drone flies at about 3 m/s | `pid_controller_tigris.yaml` not loaded (stock ±3 m/s clamp) |
| Camera swings sideways | the plan was not locked: `lock_gimbal: true` in `tigris_search_planner.yaml` |
| *agent 'robot_1' is not in the scenario team* | bundle / fleet out of sync; rerun `tigris_generate_scenario.py` |
| Run folder name has `_2` | the node already flew that `run_id`; use a fresh one |
