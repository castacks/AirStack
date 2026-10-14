# IPP baselines (greedy, coverage, MCTS) in Isaac Sim

This tutorial flies **one** PX4 multirotor with one of the informative-path-planning baselines
released with IA-TIGRIS (`ia-tigris/src/ipp_planners`): `greedy`, `rect_coverage`
(lawnmower coverage) or `mcts_search` (Monte Carlo tree search). The mission is the one of the
[IA-TIGRIS](ia_tigris_search.md) and [TIGRIS](tigris_baseline.md) tutorials (same area,
prior, targets, budget, speed, camera mount and gimbal sweep), and the run folder has the MTL
layout, so a baseline run compares file for file with IA-TIGRIS, TIGRIS and MTL runs.

| Piece | Where |
|---|---|
| Stack | [`stacks/ipp_baselines_search`](../../stacks/ipp_baselines_search/README.md) |
| Planner | [`ipp_baselines_search_planner`](../../robot/ros_ws/src/global/planners/ipp_baselines_search_planner/README.md) (the three ports + receding horizon; per-planner fidelity tables) |
| Follower, logger, messages | shared: `mtl_trajectory_follower`, `mtl_metrics_logger`, `mtl_msgs` |
| Fleet | `config/fleets/ipp_baselines_search_fleet.yaml` (`robot_1`, same spawn as MTL / TIGRIS / IA-TIGRIS) |
| Scripts | `scripts/ipp_baselines_generate_scenario.py`, `ipp_baselines_start_mission.sh`, `ipp_baselines_offline_mission.py`; analysis with `analyze_tigris_run.py` |

## The planners in one paragraph each

- **greedy**: from the current pose, every cell is scored by its one-look reward (at the
  nominal range of the viewing geometry) divided by the length of the Dubins path to the point
  it is viewed from; fly to the best, update the belief with what that leg sees, repeat until the
  budget is used.
- **rect_coverage**: a lawnmower over the area's bounding box, lane spacing = the camera's
  footprint width at the aim point (354 m here), always starting at the first lane of the
  south-west corner. It ignores the prior and the budget (the track is cut at the budget).
- **mcts_search**: UCB1 tree search over 10 motion primitives (five headings −120…+120°, at
  300 m and 900 m), with random primitive roll-outs of depth 5; the plan descends the tree by
  value. Its rewards accumulate cumulatively, as in the released code.

Only the sensor (circular footprint), the gimbal and the reward option are adapted (as for
TIGRIS / IA-TIGRIS); every quirk of the released code is kept and listed, with file:line, in
the planner README.

## 0. (Optional) change the mission, the planner or its tuning

```bash
python3 scripts/ipp_baselines_generate_scenario.py                # after editing stacks/ipp_baselines_search/config/mission.yaml
python3 scripts/ipp_baselines_generate_scenario.py --compare-mtl  # same problem as mtl_search?
```

Regenerate the bundle you compare against first (`scripts/mtl_generate_scenario.py`): the check
compares the generated `scenario.json` files, and committed bundles can be stale.

The planner and its tuning are in `stacks/ipp_baselines_search/config/ipp_baselines_search_planner.yaml`,
read at launch (restart, no rebuild):

| Parameter | Default | What it does |
|---|---|---|
| `planner` | `greedy` | `greedy`, `rect_coverage` or `mcts_search` (or `--planner` on the start script) |
| `reward_mode` | `matched` | or `original` (the ia-tigris entropy reward) |
| `initial_planning_time_s`, `planning_time_s`, `replan_period_s`, `commit_margin_s` | 5, 5, 5, 3 | as `ia_tigris_search` |
| `grid_res_m` | 50 | = TIGRIS / IA-TIGRIS (released 30) |
| `mcts_*` | released `mcts_search.yaml` | depth 5, 10 actions, exploration 0.1414, radius 300 m, primitive roll-outs |
| `rect_coverage_enforce_turn_check` | `false` | the released turn-radius check would refuse this mission (2 R = 24 m < 354 m) |
| `max_iterations`, `seed` | 0, mission seed | `max_iterations > 0` bounds each solve (reproducible) |

## 1. Build (first time, or after changing the package)

```bash
MTL_SCENARIO_DIR=/root/AirStack/stacks/ipp_baselines_search/config \
ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet ipp_baselines_search_fleet --stack ipp_baselines_search --play --wait

docker exec airstack-robot-desktop-1 bash -ic \
  "bws --packages-up-to mtl_msgs ipp_baselines_search_planner mtl_trajectory_follower mtl_metrics_logger"

airstack down
```

Unit tests:

```bash
docker exec airstack-robot-desktop-1 bash -ic "cd /root/AirStack/robot/ros_ws && \
  colcon test --packages-select ipp_baselines_search_planner && colcon test-result --verbose"
```

## 2. Bring up and check

```bash
MTL_SCENARIO_DIR=/root/AirStack/stacks/ipp_baselines_search/config \
ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
  airstack up --sim isaac --fleet ipp_baselines_search_fleet --stack ipp_baselines_search --play --wait
airstack ready

R="docker exec airstack-robot-desktop-1 bash -ic"
$R "sws; ros2 node list | grep -E 'ipp_|mtl_'"                              # /robot_1/ipp_baselines_search_planner, follower, logger
$R "sws; ros2 topic echo --once /robot_1/search/ipp_status"                  # JSON of the preview solve ("planner": ...)
$R "sws; ros2 param get /robot_1/ipp_baselines_search_planner planner"       # greedy
$R "sws; ros2 param get /robot_1/mtl_trajectory_follower accept_plan_revisions"   # True
```

## 3. Fly the sortie

```bash
bash scripts/ipp_baselines_start_mission.sh --planner mcts_search   # -r RUN_ID, --no-images, --no-record, --dry-run
```

`--planner` sets the node's `planner` parameter before the goal; the node reads it for every
new sortie. The planner log prints one line per solve (`mcts_search replan 7 [period] at 214 m
(commit node 300 m, ...)`), and `search/ipp_status` carries the same fields as JSON.

## 4. Analyze and compare

```bash
python3 scripts/analyze_tigris_run.py --run-dir runs/latest
```

It reads `robot_1/ipp_replans.json` (the planner label comes from the file). The headline metric
is `summary.residual_belief_mass` in `detection.json` (lower is better). The `original` curve is
the TIGRIS-mirror reward, a common yardstick, not the planner's own reward.

Offline, with no Isaac:

```bash
python3 scripts/ipp_baselines_offline_mission.py --set planner=greedy
python3 scripts/ipp_baselines_offline_mission.py --set planner=mcts_search --set reward_mode=original --run-id ipp-mcts-original
```

The planner benchmark has `planner: "ipp"` arms (the baseline name in `tigris_set.planner`).
Build the CLIs with `scripts/planner_benchmark/build_offline_tools.sh`, then:

```bash
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/smoke \
    --arms scripts/planner_benchmark/arms/ipp_baselines.json --workers 2
```

The `_det_60dps` arms are wall-time bounded as flown (5 s solves, commit margin 3 s): use at
most one worker per core, or the solves get less compute than in flight. The `_it150` arms are
iteration-bounded (deterministic) for determinism checks only; iterations are not equal compute
across planners.

## Results

Offline evaluation on this stack's scenario (2026-10-13): 3 km budget, the mission's sweep (±45° at
30°/s), **wall-time 5 s solves** with a 3 s commit margin, as flown, matched reward, seeds 210 (the
mission seed) to 214. TIGRIS and IA-TIGRIS were run the same way from their stack yaml; the
curve planner (`mtl_search`'s curve block) once, since it is deterministic. Scored with
`score.residual(sc, [track], 10.0, hardware={})` (the benchmark's `residual_hw`; lower is
better). Bench: `bench/ipp_stack`, arms `scripts/planner_benchmark/arms/ipp_baselines_stack_eval.json`,
2 workers on 2 cores (one per core).

| Planner | Residual, mean ± std (5 seeds) | Wins vs TIGRIS / IA-TIGRIS / curve | Original reward (seed 210) | Planning wall time per sortie | Tree / plan size (first 5 s solve) | Track revisions |
|---|---|---|---|---|---|---|
| curve (MTL) | **0.495** (1 run) | — | — | 36 s (one solve) | — | — |
| IA-TIGRIS | 0.538 ± 0.007 | 4/5 · — · 0/5 | 0.538 | 471 s | 2263–2547 nodes | 11–23 |
| TIGRIS | 0.546 ± 0.005 | — · 1/5 · 0/5 | 0.611 | 485 s | 786–919 nodes | 5–12 |
| mcts_search | 0.597 ± 0.000 | 0/5 · 0/5 · 0/5 | 0.597 | 437 s | 12461–13381 nodes (6035–6379 iterations) | 86 |
| greedy | 0.745 ± 0.000 | 0/5 · 0/5 · 0/5 | 0.721 | 4 s | 9 nodes (8 view points) | 92 |
| rect_coverage | 0.885 ± 0.000 | 0/5 · 0/5 · 0/5 | 0.885 | 1 s | 29 way points (14 lanes) | 0 |

Per seed (210…214): TIGRIS 0.548, 0.539, 0.542, 0.551, 0.551; IA-TIGRIS 0.529, 0.546, 0.533,
0.544, 0.537.

What this says, with the spread in mind:

- **None of the three baselines beats TIGRIS, IA-TIGRIS or the curve planner on any seed.** The
  order is the same as in the IA-TIGRIS paper's Table I for these planners (IA-TIGRIS, then
  greedy / MCTS, then coverage), except that MCTS is clearly better than greedy here (0.597 vs
  0.745), where the paper has them close.
- **greedy and rect_coverage have no randomness**, so their five runs are identical. greedy
  scores each 50 m cell by its one-look reward divided by the path length to it, which makes it
  take short hops between nearby cells. rect_coverage spends almost the whole budget flying to
  the area's corner, where its lawnmower starts, as written.
- **MCTS gave the same track for all five seeds** at 5 s per solve (about 6000 iterations): the
  value descent that picks the plan converges, and the commit point is always the plan's first
  node, so only the root decision matters. With 150 iterations per solve the seed does change the
  plan (unit scenario and CLI checks), so its zero spread here is a property of the compute
  budget, not a bug.
- **IA-TIGRIS vs TIGRIS**: IA-TIGRIS is 0.009 lower on average and wins 4 of 5 seeds, but that gap
  is about 1–1.5 standard deviations with 5 seeds. In the IA-TIGRIS integration's first runs
  (same setup) it was 0.561 ± 0.036 and worse than TIGRIS, so treat the ordering of these two as
  not settled.
- The curve planner is best by about 0.04, with one deterministic run.

The smoke benchmark (`bench/smoke`, first scenario, all 9 arms of `arms/ipp_baselines.json`)
ran end to end: greedy 0.299 (original 0.265), rect_coverage 0.809, mcts_search 0.166 (original
0.268) on a 6 km drift-plume scenario.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| *ipp_baselines_search_planner or its search_mission action is not running* | package not built (step 1), or a launch error (an unknown `planner` / `mcts_rollout_type: learned` is refused at the first plan) |
| rect_coverage flies to the corner and nothing else | expected: the lawnmower starts at the south-west corner, 3.3 km away, and the budget is 3 km |
| Long straight committed legs | expected: the commit is snapped to the end of the segment being flown |
| Track never extends; follower `COMPLETE` early | the follower is not accepting revisions (`mtl_trajectory_follower_ipp_baselines.yaml`) |
