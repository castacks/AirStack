# ia_tigris_search_planner

Single-agent **IA-TIGRIS** (Moon, Suvarna, Jong, Chatterjee, Yuan, Cao, Scherer, *IA-TIGRIS: An
Incremental and Adaptive Sampling-based Planner for Online Informative Path Planning*, IEEE T-RO
2026) as an AirStack global planner. It serves the same `SearchMission` interface as
`mtl_search_planner` and `tigris_search_planner`, is flown by the same follower and scored by the
same logger, so its run folders compare file for file with MTL (orienteering or curve) and TIGRIS
runs.

Stack: [`ia_tigris_search`](../../../../../../stacks/ia_tigris_search/README.md).
Tutorial: [IA-TIGRIS search tutorial](../../../../../../docs/tutorials/ia_tigris_search.md).

## What it does

The receding horizon, gimbal handling, interface and run files are those of
`tigris_search_planner` (see its README). What is IA-TIGRIS:

1. **Initial solve** from the hover pose with the whole budget (`initial_planning_time_s`), on a
   fresh tree (`Tigris::reset_tree_to_pose`).
2. **Every replan** (`replan_period_s`, or early near the end of the track):
   - the looks actually flown are folded into the belief (measured pose and gimbal);
   - the commit target `progress + lookahead + speed × (planning_time + commit_margin)` is
     **snapped forward to the next node of the current IA-TIGRIS path**
     (`PlannerNode::find_expected_waypoint_reached_after_time_to_plan`: plan from the end of the
     tree edge the drone will be on). The track up to that node is never changed;
   - the **tree is recycled** from that node (`Tigris::update_tree` /
     `recycle_tree_from_node` / `add_children_to_tree`): everything that is not in its subtree is
     dropped, costs are re-based, nodes over the remaining budget are deleted with their subtrees,
     and every reward is recomputed, root to leaf, on the planning belief (flown + committed looks).
     If the node is still the root, the whole tree is kept. The recycling time counts against the
     solve's `planning_time_s`, as in `Planner::replan`;
   - sampling continues on the recycled tree. The tree still holds the rest of the current plan,
     so the best path is never worse than it; the track is revised only when the best path is a
     different one (same `plan_id`, newer stamp; the follower accepts in-flight revisions).
3. **Incremental rewards** (`SearchMap::search_information_gain_with_hashmaps`): every node keeps
   the sparse belief updates of its own edge and footprint (`local_search_map_updates`). A new
   node's reward is its parent's plus its own looks evaluated on the ancestors' updates replayed
   over the belief. It equals re-walking the whole chain from the root (unit-tested), at the
   cost of one node instead of the whole chain.
4. **Near-node loop** (`Tigris::extend_neighbors_towards_node`, `add_node_to_tree_or_prune`): every
   open node within `extend_radius_m` of a sample extends toward it, and every node that is added
   is compared with the best path.

Unchanged from TIGRIS (and from the ia-tigris code): the informed sampler (cell centre drawn with
probability ~ its one-look reward on the current belief, backed off along a random yaw to
`view_point_goal`), steer with the `closed` set, dominance pruning, no reward on turning arcs.

`recycle_tree: false` grows a fresh tree at every solve and always flies its new path (IA-TIGRIS
with `clear_tree_on_auto_replan: true`), which is the paper's "replanning without recycling"
ablation.

## How close to the released code

Algorithm: as in `ia-tigris/src/ipp_planners` (`Tigris.cpp`, `Planner.h`, `SearchMap.cpp`,
`InfoMapSearch.h`, `ipp_planners_node.h`). **Only these three things are adapted** (the same ones
as in `tigris_search_planner`, so the planners are compared on one sensor and gimbal model):

| Released IA-TIGRIS | Here | Why |
|---|---|---|
| rectangular frustum footprint (width/height/focal, pitch) | the mission's cone (`sensor.fov_deg`, `sensor.tilt_deg`): a disc on the ground; the edge swath is the disc swept along the straight segment, with the nearest viewing pose derived for the disc | one sensor model for every planner and the logger |
| fixed camera | the AirStack single-axis gimbal: body-fixed, or the constant-rate cross-track sweep of `mission.yaml` `gimbal_actuation` (footprints and rewards use the swept phase) | same gimbal setup as TIGRIS |
| entropy reward only | `reward_mode: matched` (default) = expected drop in residual belief mass, the logger's metric; `original` = the IA-TIGRIS entropy reward | the study's reward |

**ORIGINAL reward = IA-TIGRIS's, not TIGRIS's.** Per node, first the straight part of its edge
(on the parent's belief), then its footprint; both use `calc_reward_and_belief` (entropy
reduction × Rs if the belief rose, × Rf otherwise, Rs = Rf = 1, `confidence_threshold` 0.5). The
sensor is the scenario's Moon et al. sigmoid for tpr, with fpr = 1 − tpr (0.5 beyond `beta`).
TIGRIS scores the node before the edge, with a different edge formula and Rs = 2.

Implementation details that AirStack forces, shared with `tigris_search_planner`:
- an exact spatial hash instead of OMPL GNAT;
- Dubins steering instead of a zero-wind trochoid;
- a 0.5 m steer walk instead of 40 m path discretisation;
- a seeded RNG;
- the follower carrot lookahead and `commit_margin_s` (3 s, as `tigris_search`) instead of the
  ROS 1 node's 1.5 s.

One quirk of `Tigris::add_children_to_tree` is not reproduced: it `return`s after deleting an
over-budget child and skips that child's remaining siblings. With the receding horizon's
consistent budgets no child is ever over budget, so this never changes a result.

## Parameters

The mission (budget, speed, turn radius, altitude, camera, gimbal sweep, detection, prior) is
read from the scenario. IA-TIGRIS tuning lives in `config/ia_tigris_search_planner.yaml`. The
stack loads its own copy from `stacks/ia_tigris_search/config/`, read at launch, so a change
needs only a restart.

| Parameter | Default | |
|---|---|---|
| `reward_mode` | `matched` | or `original` (IA-TIGRIS entropy) |
| `recycle_tree` | `true` | `false` = a fresh tree every solve |
| `extend_dist_m`, `extend_radius_m`, `prune_radius_m` | 750, 251, 750 | the same as `tigris_search`, so the two differ only in the algorithm. The released `fixed-wing/ia-tigris.yaml` values (1500, 1500, 400) scored worse on this mission's 3 km budget: residual 0.561 vs 0.529 (one seed, offline, matched reward) |
| `grid_res_m` | 50 | the same as `tigris_search` (released `map_resolution`: 30) |
| `rs`, `rf` | 1, 1 | `SearchMap::Rs`, `Rf` |
| `initial_planning_time_s`, `planning_time_s`, `replan_period_s` | 5, 5, 5 | same compute as `tigris_search` |
| `max_iterations`, `seed` | 0, mission seed | `max_iterations > 0` bounds each solve by new samples (reproducible) |

Outputs (`runs/<run_id>/<agent>/`): `plan.json`, `track.json` (the gimbal block stays under the
`tigris` key, with `"planner": "ia-tigris"`, so every tool reads it unchanged), `scenario.json`,
and `ia_tigris_replans.json`. The last has every solve, with `recycled`, `nodes_kept`,
`nodes_dropped`, `recycle_s`, `kept_tail` and `commit_target_arc_m` added. Status topic:
`search/ia_tigris_status`.

## Offline comparison with TIGRIS (2026-10-13)

Setup:
- the stack mission (5 km area, 300 m altitude, 3 km budget, ±45° / 30°/s sweep, `beta` 900 m);
- the stack yaml of each planner (same lengths, 5 s wall-time solves, 3 s commit margin, matched
  reward);
- the kinematic CLI sortie, on a shared 2-core machine (one run per core);
- scored with `scripts/planner_benchmark/score.py` `residual(..., hardware={})`, the benchmark's
  `residual_hw`. Lower is better.

| Run (seed) | TIGRIS | IA-TIGRIS |
|---|---|---|
| mission seed | 0.548 | **0.529** |
| 1234 | **0.551** | 0.618 |
| 7 | **0.551** | 0.555 |
| 99 | **0.536** | 0.572 |
| 2026 | 0.537 | **0.533** |
| **mean ± std (5 seeds)** | **0.545 ± 0.007** | 0.561 ± 0.036 |

Other runs, all at the mission seed:

| Run | Residual |
|---|---|
| IA-TIGRIS, original reward | 0.550 |
| TIGRIS, original reward | 0.611 |
| IA-TIGRIS, released lengths (1500/1500/400 m, 30 m grid) | 0.561 |
| same, `recycle_tree: false` | 0.583 |

- **Variance.** IA-TIGRIS wins 2 of the 5 matched seeds and is about 3 % worse on average,
  with 5× the seed-to-seed spread. It flies the best path of a tree it keeps: in 60–85 % of
  solves the best path is still the current plan, so the first tree matters a lot. TIGRIS
  grows a fresh tree every solve and only switches when the new path scores higher.
- **Recycling ablation.** Recycling clearly helped at the released lengths (0.561 vs 0.583
  without it), the same direction as the paper's ablation.
- **Cost.** Recycling took 0.25–0.65 s of each 5 s solve (up to 2 s at the released lengths).
- These are single-scenario numbers. Use the benchmark arms (`arms/ia_tigris_vs_curve.json`)
  for a population-level comparison.

## Also built

- `ia_tigris_search_plan`: the ROS-free CLI (`--scenario F --out-dir D [--set key=value]
  [--one-shot]`, `--set recycle_tree=false` for the ablation). `scripts/ia_tigris_offline_mission.py`
  drives it.
- The Python reward mirror is not duplicated: `scripts/analyze_tigris_run.py` reads IA-TIGRIS runs
  (`ia_tigris_replans.json`) with `tigris_search_planner.rewards`.

## Tests

`test/test_ia_tigris_core.cpp` (gtest) covers:
- everything the TIGRIS core tests cover (scenario, Dubins, footprints, belief, budget and
  determinism, committed prefix, `track.json`, gimbal sweep);
- incremental reward = full re-walk (both modes, body-fixed and swept, every prefix);
- the ORIGINAL edge-then-node order with Rs = Rf = 1;
- tree recycling from the committed node: snapped commit, kept/dropped nodes, never worse than
  the tail, track untouched when the tail is kept;
- recycling the root keeps the whole tree;
- `recycle_tree: false`.
