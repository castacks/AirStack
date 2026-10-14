# IA-TIGRIS search in Isaac Sim

This tutorial flies **one** PX4 multirotor with the IA-TIGRIS informative path planner (Moon
et al., IEEE T-RO 2026) over the same search problem as the
[TIGRIS baseline](tigris_baseline.md): the same `mission.yaml` (area, prior, targets, budget,
speed, camera mount and gimbal sweep). The run folder has the MTL layout and metrics, so an
IA-TIGRIS run compares file for file with a TIGRIS run and with a 1-robot MTL run
(orienteering or curve).

| Piece | Where |
|---|---|
| Stack | [`stacks/ia_tigris_search`](../../stacks/ia_tigris_search/README.md) |
| Planner | [`ia_tigris_search_planner`](../../robot/ros_ws/src/global/planners/ia_tigris_search_planner/README.md) (IA-TIGRIS port + receding horizon) |
| Follower, logger, messages | shared with MTL and TIGRIS: `mtl_trajectory_follower`, `mtl_metrics_logger`, `mtl_msgs` |
| Fleet | `config/fleets/ia_tigris_search_fleet.yaml` (`robot_1`, same spawn as TIGRIS and MTL) |
| Scripts | `scripts/ia_tigris_generate_scenario.py`, `ia_tigris_start_mission.sh`, `ia_tigris_offline_mission.py`; analysis with `analyze_tigris_run.py` |

## What is different from TIGRIS

Only the planner's algorithm; the sensor (circular footprint), gimbal and rewards are modelled
the same way for both.

- **Tree recycling.** At every replan the commit point is snapped forward to the next node of
  the current path. The tree is re-rooted there, and every reward is recomputed on the updated
  belief. Sampling then continues on the recycled tree. TIGRIS grows a fresh tree every solve.
- **Incremental rewards.** Each node stores its own belief updates, so a node costs one reward
  evaluation instead of a walk of its whole chain.
- **Reward.** `matched` (residual belief mass, the logger's metric) by default. `original` is
  IA-TIGRIS's entropy reward: per node the edge before the footprint, and Rs = Rf = 1.
- **Tuning.** The same as `tigris_search`: extend 750 m, near radius 251 m, prune 750 m, 50 m
  grid, 5 s solves, 5 s period and 3 s commit margin. Only the algorithm differs. The released
  fixed-wing values (1500 / 1500 / 400 m, 30 m grid) are a `--set` away; they scored worse on
  this mission's 3 km budget.

## 0. (Optional) change the mission or the tuning

The mission is `stacks/ia_tigris_search/config/mission.yaml`. It is a copy of
`tigris_search`'s, and the two should stay in step. After editing it:

```bash
python3 scripts/ia_tigris_generate_scenario.py                                         # -> stacks/ia_tigris_search/config/{scenario.json,ground_truth.json,belief.png}
python3 scripts/ia_tigris_generate_scenario.py --compare-mtl                           # same problem as mtl_search?
python3 scripts/ia_tigris_generate_scenario.py --compare-mtl --mtl-dir stacks/tigris_search/config   # ... as tigris_search?
```

Regenerate the bundle you compare against too (`scripts/mtl_generate_scenario.py`,
`scripts/tigris_generate_scenario.py`). The check compares the generated `scenario.json`
files, not the yaml files.

IA-TIGRIS tuning is in `stacks/ia_tigris_search/config/ia_tigris_search_planner.yaml`, read at
launch (restart, no rebuild):

| Parameter | Default | What it does |
|---|---|---|
| `reward_mode` | `matched` | or `original` (IA-TIGRIS entropy reward) |
| `recycle_tree` | `true` | `false` = a fresh tree every solve (the paper's "replanning without recycling") |
| `initial_planning_time_s`, `planning_time_s`, `replan_period_s` | 5, 5, 5 | wall time per solve; recycling counts against it |
| `extend_dist_m`, `extend_radius_m`, `prune_radius_m`, `grid_res_m` | 750, 251, 750, 50 | = `tigris_search` (released IA-TIGRIS: 1500, 1500, 400, 30) |
| `max_iterations`, `seed` | 0, mission seed | `max_iterations > 0` bounds each solve by new samples |

## 1. Build (first time, or after changing the package)

```bash
MTL_SCENARIO_DIR=/root/AirStack/stacks/ia_tigris_search/config \
ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet ia_tigris_search_fleet --stack ia_tigris_search --play --wait

docker exec airstack-robot-desktop-1 bash -ic \
  "bws --packages-up-to mtl_msgs ia_tigris_search_planner mtl_trajectory_follower mtl_metrics_logger"

airstack down
```

The first `airstack up` can log a launch error for `ia_tigris_search_planner` because the
package is not built yet; the container stays up for `bws`. Optional unit tests:

```bash
docker exec airstack-robot-desktop-1 bash -ic "cd /root/AirStack/robot/ros_ws && \
  colcon test --packages-select ia_tigris_search_planner && colcon test-result --verbose"
```

## 2. Bring up and check

```bash
MTL_SCENARIO_DIR=/root/AirStack/stacks/ia_tigris_search/config \
ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet ia_tigris_search_fleet --stack ia_tigris_search --play --wait
airstack ready

R="docker exec airstack-robot-desktop-1 bash -ic"
$R "sws; ros2 node list | grep -E 'tigris|mtl_'"                          # /robot_1/ia_tigris_search_planner, mtl_trajectory_follower, mtl_metrics_logger
$R "sws; ros2 topic echo --once /robot_1/search/ia_tigris_status"          # JSON of the preview solve (iterations, tree, recycling, rewards)
$R "sws; ros2 param get /robot_1/ia_tigris_search_planner reward_mode"     # matched
$R "sws; ros2 param get /robot_1/mtl_trajectory_follower accept_plan_revisions"   # True
```

## 3. Fly the sortie

```bash
bash scripts/ia_tigris_start_mission.sh        # -r RUN_ID, --no-images, --no-record, --dry-run
```

The planner log prints one line per solve, for example (offline rehearsal of this mission):
`replan 18 [period] at 551 m (commit node 750 m, 2250 m budget left): kept current plan ...
tree 2665 (recycled 2636 kept / 0 dropped in 0.25 s) ...`. The commit point sits on a tree
node, so it stays at the same node (750 m) until the drone gets close to it, and meanwhile the
same tree keeps growing. `kept current plan` means the best path in the recycled tree is still
the one being flown. `search/ia_tigris_status` carries the same fields as JSON.

## 4. Analyze and compare

```bash
python3 scripts/analyze_tigris_run.py --run-dir runs/latest
```

It reads `robot_1/ia_tigris_replans.json` and writes the same outputs as for a TIGRIS run. The
headline metric is `summary.residual_belief_mass` in `detection.json` (lower is better). Its
`original` reward curve is the TIGRIS-mirror reward, a common yardstick for every planner. It is
not IA-TIGRIS's own reward; that one is in the replan table (`segment_reward`).

Offline, with no Isaac:

```bash
python3 scripts/ia_tigris_offline_mission.py                                    # stack params
python3 scripts/ia_tigris_offline_mission.py --set recycle_tree=false --run-id ia-tigris-offline-norecycle
```

The planner benchmark has IA-TIGRIS arms (`planner: "ia_tigris"`). Build the CLIs with
`scripts/planner_benchmark/build_offline_tools.sh`, then:

```bash
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide900 \
    --arms scripts/planner_benchmark/arms/ia_tigris_vs_curve.json --workers 2
python3 scripts/planner_benchmark/make_report.py --bench bench/wide900 --focus mtl_curve \
    --against ia_tigris_det_60dps,tigris_det_60dps --arms mtl_curve,ia_tigris_det_60dps,tigris_det_60dps \
    --out bench/wide900/report_ia_tigris.html --csv bench/wide900/results_ia_tigris.csv
```

The IA-TIGRIS arms are wall-time bounded as flown, at 5 s per solve. Use at most one worker per
core, or the solves get less compute than in flight. `ia_tigris_det_60dps_it150` is
iteration-bounded instead, for a like-for-like check with `tigris_det_60dps`.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| *ia_tigris_search_planner or its search_mission action is not running* | package not built (step 1), or a launch error; check the robot launch tmux |
| Every solve says `fresh` | `recycle_tree: false`, or the replans ran out of path (end of budget) |
| Long straight committed legs | expected: the commit is snapped to the end of the edge being flown (up to `extend_dist_m`) |
| Track never extends; follower `COMPLETE` early | the follower is not accepting revisions (`mtl_trajectory_follower_ia_tigris.yaml`) |
