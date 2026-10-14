# ipp_baselines_search_planner

The informative-path-planning baselines released with IA-TIGRIS (Moon et al., IEEE T-RO 2026;
`ia-tigris/src/ipp_planners`) as one AirStack global planner, so they can be benchmarked
against MTL (orienteering / curve), TIGRIS and IA-TIGRIS on the same mission, sensor, gimbal
and scorer:

| `planner` | ia-tigris source | Released config |
|---|---|---|
| `greedy` | `GreedySearchPlanner.h` | `fixed-wing/greedy.yaml` (no parameters) |
| `rect_coverage` | `CoveragePlanner.h` (`CoveragePlanner` + `RectangleCoverage`) | `fixed-wing/rect_coverage.yaml` (no parameters) |
| `mcts_search` | `MCTSSearch.h`, `MCTSNode.h` | `fixed-wing/mcts_search.yaml` |

`random`, `primtree` and `primtreebnb` (also in `ipp_planners_node.h`) are not ported (out of
scope); ReASC, a baseline of the IA-TIGRIS paper, is not in the released code.

Stack: [`ipp_baselines_search`](../../../../../../stacks/ipp_baselines_search/README.md).
Tutorial: [IPP baselines tutorial](../../../../../../docs/tutorials/ipp_baselines_search.md).

## Layout

- `planner_core.{hpp,cpp}`: what the baselines share (`Planner.h`, `TreeNode.h`): the node type,
  `Planner::extend_from_start_node_toward_goal_node` **exactly** (zero-length node when the
  first step already reaches the budget; straight-edge poses set by `maybe_set_edge_start_pose` /
  `maybe_set_edge_end_pose`, so an edge whose straight part is never followed by a turn has no
  edge reward), `find_viewing_point`, the ia-tigris `SearchMap` reward of one node (edge then
  footprint for the hashmap reward, footprint then edge for the full walk), and the
  `SearchPlanner` interface = `Planner::replan` (setup, loop while time / `max_iterations` allow
  and the loop says continue, teardown, best path). `makePlanner()` = `init_planner`.
- `greedy.cpp`, `rect_coverage.cpp`, `mcts_search.cpp`: one class per planner.
- `scenario`, `dubins`, `belief`, `json_mini`: forked from `ia_tigris_search_planner`, unchanged
  in behaviour (namespace `ipp_baselines_search`).
- `receding.{hpp,cpp}`: the IA-TIGRIS port's receding horizon with the `ipp_planners_node.h`
  commit rule for every planner (snap forward to the end of the current segment) and the
  track cut at the budget; JSON writers (`track.json` keeps the `tigris` gimbal block, with
  `"planner": "<name>"`; `ipp_replans.json` = the `ia_tigris_replans.json` layout + `"planner"`,
  `cut_at_budget_m` and `planner_stats`).
- `params.cpp`: the parameter names shared by the node and the CLI.

## How close to the original

**Only these three things are adapted**, the same three as in `tigris_search_planner` and
`ia_tigris_search_planner`:

| Released ia-tigris | Here | Why |
|---|---|---|
| rectangular frustum footprint (width / height / focal, pitch) | the mission's cone (`sensor.fov_deg`, `sensor.tilt_deg`): a ground disc; the edge swath is the disc swept along the straight segment; "cell entirely inside" and "range to the cell corner" kept | one sensor model for every planner and the logger |
| fixed camera | the AirStack single-axis gimbal: body-fixed, or the constant-rate cross-track sweep (`gimbal_actuation`); every planned look uses the swept phase at its track time | same gimbal setup as TIGRIS / IA-TIGRIS |
| entropy reward only | `reward_mode: matched` (default) = expected drop in residual belief mass; `original` = the ia-tigris `SearchMap` reward (`calc_reward_and_belief`, Rs = Rf = 1, threshold 0.5, tpr = the scenario sigmoid, fpr = 1 − tpr, 0.5 beyond beta) | the study's reward |

Forced by AirStack, shared with the other ports: Dubins instead of a zero-wind trochoid (0.5 m
walk; `final_path_discretization_distance` = one step of that walk), seeded RNGs (mission seed
+ replan index per solve), the receding-horizon glue (commit distance = follower lookahead +
speed × (planning_time + commit_margin_s), prefix never changed), the flown looks folded into
the belief as misses, back-to-back solves (`seconds_until_auto_replan` 0.1). Where a planner's
own replanning differs, its own is used (see below). Decisions taken with the user
(2026-10-13): D1 the exact `Planner.h` steer for these planners; D2 the coverage turn-radius
check off by default; D3 the published track cut at the budget; D4 the 50 m grid of the
TIGRIS / IA-TIGRIS ports (released `map_resolution` 30 m) and MCTS's per-node map copies kept.

### greedy (`GreedySearchPlanner.h`)

| Behaviour (file:line) | Ported as |
|---|---|
| Setup: root at the start, `search_information_gain(root)` → working map = map + root footprint (49–87) | same (`rewalk()`) |
| Cell score = `calc_reward_and_belief(nominal range, copy_map) × priority / trochoid length` to the cell's viewing point, nominal range = h / sin(pitch + vg·vfov/2) (105–130, 62–65) | same; cone: h / cos(tilt − vg·fov/2); matched mode: residual × P(nominal range) |
| Scan order row (x) outer, column (y) inner; first strict maximum above 0 wins (96–98, 92, 133) | same order |
| Quirk: a cell is evaluated only if its viewing point differs from the pose in x **and** y (`&&`, 117) | kept |
| Edge poses set **before** the budget test (189–195) | kept |
| Quirk: on the budget the whole segment is kept (`keepBefore` commented out, 198, 208) | kept in the plan; the published track is cut at the budget (D3) |
| Chain update: `search_information_gain` re-walks root → node, **footprint then edge** (212; SearchMap.cpp:1116–1148) | kept (the two updates commute unless a belief crosses 0.5) |
| Quirk: no cell scores > 0 → empty view point dereferenced (UB) (146–157) | cannot be reproduced: the solve ends |
| No state across solves | same |

### rect_coverage (`CoveragePlanner.h`)

| Behaviour (file:line) | Ported as |
|---|---|
| `x_delta = h / tan(pitch + vg·vfov/2)`, `width = 2 tan(hfov/2) √(x_delta² + h²)` (211–216) | **modification 1**: `x_delta = h tan(tilt − vg·fov/2)` (63.8 m here), `width = 2 tan(fov/2) h / cos(tilt − vg·fov/2)` (354 m here): the cone's diameter at the aim point |
| `RectangleCoverage` throws if 2 R < width (27–30) | behind `rect_coverage_enforce_turn_check` (default **false**, D2): 2 R = 24 m here |
| Lawnmower over the bounding box from corner 0; a square takes the `else` branch (lanes along +y, stepped along +x), `floor(edge / width)` lanes, end points shifted by the offset rotated into the lane heading (44–135) | same |
| Quirk: ignores the budget | kept in the plan; the published track is cut at the budget (D3) |
| Quirk: every replan restarts at lane 1 of corner 0 (in ROS 1 the drone oscillates on lane 1) | kept; here the 3.3 km transit to the corner commits the whole 3 km budget, so it never shows |
| `get_rect_coverage_length` unused; trochoid `valid` ignored (242) | same |

### mcts_search (`MCTSSearch.h`, `MCTSNode.h`)

| Behaviour (file:line) | Ported as |
|---|---|
| Per-node `InfoMapSearch` copies: twice per node (299–301, 339–341), once per `replan_loop` and twice per setup (`auto info_map = ...`, 139, 149, 213) | the same copies are made (same arrays); by default released right away, only the RNG state is kept (`mcts_retain_map_copies: true` keeps them all, ~3.7 GB per 5 s solve). The copies never change a result |
| Quirk: `transition` = the hashmap reward **including the parent's `search_info`** (cumulative), `edge_value = transition`, `information = parent.information + transition` (303–313) | kept (tested: `searchInfo` = full re-walk) |
| Quirk: `rollout_value += new_node->information` (483) and back-prop `future_value += child.get_value()` (446) | kept |
| UCB1 = `get_value / (budget / speed) + w √(ln(root.n_visits) / n)`, unvisited = DBL_MAX, strict `>` (91–118) | kept (root visits, not parent's) |
| Quirk: `tree_root` is a member: `n_visits`, `future_value`, `edge_value` never reset (137–160); counters reset in shadowing locals (157–158) | kept |
| Primitive: `num_angles = num_actions/2`, ×3 radius for `a ≥ num_angles`, −120°…+120° ("135" in the comment) (387–420) | kept |
| Informed: goal = a cell centre from the FROM node's map copy (its RNG), heading uniform [−π, π − 1e-4), no viewing-point back-off (355–377) | kept, with the per-node RNG copies; the root copy is seeded with the mission seed alone at every solve (ROS 1 copies a global map whose generator never advances) |
| Quirk: `mcts_extend_dist`, `mcts_extend_angle`, `mcts_branches_each_side` read but unused; steer unbounded (332) | kept (parameters accepted, unused) |
| Quirk: `NodeStuckException` thrown when the 6th attempt is reached, even if it succeeded (281–293) | kept (never happens: the Planner.h steer always returns a node) |
| Plan = descent by the highest `get_value` (504–521), not the logged best node | kept |
| `learned` rollouts (torch, commented out; null action → crash) | refused at load |
| UB: all-NaN UCB descent, `.at(0)` of an expansion with no children, all-NaN best-path descent (infinite loop) | cannot be reproduced: the solve ends / the descent stops |

### Receding horizon (`ipp_planners_node.h`)

| Behaviour | Here |
|---|---|
| Plan from the end of the segment the drone reaches after `speed·(max_planning_time + 1.5 s)` (567–620, 510–559) | commit target = lookahead + speed·(planning_time + commit_margin_s), snapped forward to the next plan node; prefix kept |
| `append_path_msg` publishes the newest best path (773–805) | the newest plan always replaces the tail (none of these planners keeps a tree) |
| `seconds_until_auto_replan: 0.1` | `replan_period_s = planning_time_s` (back to back) |
| Remaining budget from the plan (690–739) | budget − arc(commit node); the published track ends at the budget |

## Parameters

See `config/ipp_baselines_search_planner.yaml` (the stack loads its own copy). Released
values first; none was retuned. The released primitive lengths (300 m, ×3 for the long MCTS
actions) are of the order of this mission's 3 km budget, unlike TIGRIS's 1500 m extends.

## Outputs

`runs/<run_id>/<agent>/`: `plan.json`, `track.json` (`tigris` block, `"planner": "<name>"`),
`scenario.json`, `ipp_replans.json`. Status topic: `search/ipp_status` (JSON per solve).

## Also built

- `ipp_baselines_search_plan`: the ROS-free CLI (`--scenario F --out-dir D --set planner=NAME
  [--set key=value] [--one-shot]`); `scripts/ipp_baselines_offline_mission.py` drives it.
- Host build without ROS:
  `g++ -std=c++17 -O2 -Wall -Wextra -Iinclude src/{scenario,dubins,belief,planner_core,greedy,rect_coverage,mcts_search,planners,receding,params}.cpp src/ipp_baselines_search_plan_cli.cpp -o ipp_baselines_search_plan`

## Tests

`test/test_ipp_baselines_core.cpp` (25): scenario, Dubins, footprint, belief, gimbal sweep,
the Planner.h steer (budget, zero-length node, straight-edge rule, extend_dist), factory, and
for every planner the committed prefix, commit snapped to a node, budget cut, flyable
splices, swept phase continuity and `track.json` / `ipp_replans.json` layout.
`test/test_ipp_baselines_planners.cpp` (13): per planner budget, determinism, connected path,
positive reward in both modes; greedy's max-weight cell and footprint-then-edge walk; the
coverage lanes and turn check; MCTS incremental reward = full re-walk (both modes, body-fixed
and swept), the primitive fan, UCB1 on a toy tree, root statistics carried over, the
value-descent plan and informed cell-centre goals.

## Results

Offline evaluation on the `ipp_baselines_search` stack scenario (2026-10-13): 3 km budget, the mission's sweep (±45° at
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
