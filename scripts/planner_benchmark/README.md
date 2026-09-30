# Planner benchmark: plain MTL vs information-aware MTL vs TIGRIS

Offline and ROS-free. It generates many search problems, runs each planner's own CLI on every
one, and scores every plan with the same metric (residual belief = P(target missed), scored on the look points a real gimbal achieves,
lower is better). A report then shows where each planner wins.

```bash
# 0. once: build the two planner CLIs on the host (needs g++ and libeigen3-dev)
bash scripts/planner_benchmark/build_offline_tools.sh
# 1. scenarios: 240 priors across 10 families, 5 budgets, 3 altitudes, 3 home positions
python3 scripts/planner_benchmark/gen_scenarios.py --spec scripts/planner_benchmark/specs/wide.json --out bench/wide
# 2. plan + score every (scenario, arm); resumable, parallel
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide --arms scripts/planner_benchmark/arms/default.json --workers 8
# 3. report (HTML, opens offline) + a flat CSV for your own analysis
python3 scripts/planner_benchmark/make_report.py --bench bench/wide --focus mtl_info_aware,tigris_sweep
```

The fair comparison of 2026-09-30 (900 m detection range for everyone, each planner at its tuned best, scored through the common gimbal):

```bash
python3 scripts/planner_benchmark/gen_scenarios.py --spec scripts/planner_benchmark/specs/wide900.json --out bench/wide900
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide900 --arms scripts/planner_benchmark/arms/tigris_tuning.json --limit 30 --workers 8
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide900 --arms scripts/planner_benchmark/arms/mtl_tuning.json --limit 30 --workers 8
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide900 --arms scripts/planner_benchmark/arms/fair900.json --workers 8
python3 scripts/planner_benchmark/make_report.py --bench bench/wide900 --focus mtl_info_aware_r800,tigris_det_60dps --arms mtl_plain,mtl_plain_r800
```

`tigris_det_60dps_matched` is left out of the last two steps on purpose: the matched reward takes 20+ min per plan at 1000 s budgets. Run it with `--only-arms` on the short budgets if you want it.

Run `specs/smoke.json` first: 12 scenarios, to check the pipeline end to end. Needs Python 3 with
numpy; matplotlib is optional (it draws the scenario maps in the report), and PyYAML is optional
(it lets TIGRIS read the stack's tuning).

## Pieces

| File | What it does |
|---|---|
| `priors.py` | Ten prior families, all written as mixtures of Gaussian bumps plus a uniform floor. That is the one prior representation every consumer in the stack rebuilds identically (both planners, the logger, the Isaac scene, `score.py`). Families: `gaussian_blobs`, `clustered`, `heavy_tailed` (Pareto peak heights), `decoy` (weak blobs near home, a heavy group far away), `lines` (roads/rivers), `ring` (range-only annuli, optionally open), `drift_plume` (a search-and-rescue drift from a last-known point), `random_field` (log-normal lattice), `diffuse_plus_peaks` (a floor under a few peaks), `multi_scale` (wide weak regions with sharp strong peaks). |
| `gen_scenarios.py` | Spec -> `scenarios/<id>.json` (a copy of `stacks/mtl_search/config/scenario.json` with the prior, cells, budget, altitude and home replaced) and `index.json` (the factors plus prior descriptors: effective area, spread, budget ÷ spread, mass within half the budget of home, peak count, Gini, cells holding 50 / 90 % of the mass). |
| `run_benchmark.py` | Runs every arm in an arms file on every scenario: `mtl_search_plan --no-alt` for MTL, `tigris_search_plan` for TIGRIS (starting from the stack's TIGRIS yaml, then the arm's `tigris_set`). Scores each track, and appends a line to `results.jsonl` holding the residual, the anytime curve, flown length, plan time, a decimated track, and the info-aware choice. Resumable: a pair that already has a result is skipped. |
| `score.py` | A numpy port of `mtl::eval::computeResidualBelief`. It agrees with the C++ metric to 1e-6 on the flown runs of 2026-09-29 (0.42585 / 0.42303 at 10 m). |
| `plot_tracks.py` | A static trajectory map for every scenario, one panel per arm (`--arms A,B,C`): `track_maps/<id>.png` plus all of them in `track_maps/all_tracks.pdf`, sorted by where the second arm wins most. |
| `make_report.py` | `report.html` + `results_wide.csv` (both described below). `--metric` picks the score (default `residual_hw`); `--arms` limits the report to a set of arms, so scenarios missing a result for an unrelated arm are not dropped. |
| `arms/default.json` | `mtl_plain`, `mtl_info_aware`, `tigris_sweep` (the flown TIGRIS: sweeping gimbal), `tigris_fixed` (body-fixed camera); TIGRIS at 600 iterations per solve. |
| `arms/quick.json` | The same without `tigris_fixed`, and TIGRIS at 150 iterations per solve (about 4x faster to run). |
| `arms/tigris_tuning.json` | Both MTL modes plus five TIGRIS configurations (sweep amplitude × rate, body-fixed, matched reward), for picking TIGRIS's best before a comparison. |
| `arms/mtl_tuning.json` | Both MTL modes at cluster radius 400 and 800 m (the stack's 550 m was chosen for the 610 m sensor). |
| `arms/fair900.json` | The final comparison: `mtl_plain` (stack), `mtl_plain_r800`, `mtl_info_aware_r800`, `tigris_det_60dps`. |
| `arms/fair_sweep.json` | TIGRIS with the detection-matched sweep only. |
| `specs/*.json` | `wide` (240 scenarios, all families), `wide900` (160, detection range 900 m for everyone), `team` (60, 2–3 agents: MTL plans jointly, TIGRIS per agent), `smoke` (12). |

### What the report contains

- **Leaderboard:** mean and median residual, mean rank, the share of scenarios each arm wins outright, and its pairwise win rate against every other arm.
- **Head to head** of the two `--focus` arms:
  - win rate, and the "decisive" rate: wins by more than `--decisive` (default 0.05);
  - the mean advantage by prior family, budget, altitude and home, and by quartile of each prior descriptor;
  - a family × budget heat map.
- **Anytime curves:** mean residual against the fraction of distance flown.
- **Scatter plot:** one dot per scenario, with a family filter.
- **Maps** of the most and least decisive scenarios: the prior with both tracks drawn over it.
- **Full table:** every scenario, sortable.
- **Scene viewer:** click any scenario (a table row, a scatter dot or a map card) to open its prior with every arm's path and camera ground points. It shows the focus arms overlaid, with a toggle per arm and a side-by-side mode; hovering a path gives the distance flown at that point; ← / → step through scenarios. It adds about 20 KB per scenario; `--no-viewer` leaves it out.

Scenarios where every arm cleared the prior (all residuals below `--min-residual`, default 0.01) are left out of the analysis but kept in the CSV.

## Arms and TIGRIS settings

An arm is `{"name", "planner": "mtl"|"tigris", "scenario_overrides": {dotted.key: value|null}, "tigris_set": {...}}`.

- **MTL arms** toggle the planner through the scenario's `info_aware` block, the same block that `mission.yaml` writes.
- **TIGRIS arms** start from `stacks/tigris_search/config/tigris_search_planner.yaml`, the stack's tuning: 50 m grid, 750 m extend distance, 5 s replan period, entropy reward. They then add three settings:
  - `planning_time_s: 1000`;
  - `commit_margin_s: -992`, which keeps the flown commit distance of 5 s + 3 s;
  - `max_iterations`.

  With these, each solve is bounded by iterations rather than wall time. The result is then deterministic and does not depend on how many workers share the machine.

  On the robot, a 5 s solve ran about 500–700 iterations. On the Sep 29 scenario, 150 iterations scored 0.4234 and 600 iterations scored 0.4229, against 0.4257 for the flown plan, so TIGRIS is not sensitive to the iteration count here.

  To reproduce flight exactly, set `planning_time_s: 5` and `commit_margin_s: 3` and drop `max_iterations`. Runs are then wall-time bounded, so give each worker a whole core.

## Fairness protocol (read before trusting a win)

Every result line holds three scores of the same plan:

| Field | What is scored |
|---|---|
| `residual` | the boresight each planner *planned* |
| `residual_hw` (report default, `--metric`) | the boresight a real gimbal *achieves* when `mtl_trajectory_follower`'s single-axis law points it at the planned one: cross-track angle clamped to the planner's declared mount limit (80°), airframe pitch nudge clamped to its declared limit (5° for MTL, 0 for TIGRIS), each earth-frame axis slew-limited to 120°/s and clamped to roll ±80° / pitch −20..110° (the Isaac `sim_gimbal`). One model for every planner. |
| `residual_hw_nonudge` (MTL only) | as `residual_hw`, but with MTL's ±5° pitch nudge removed, so a win can be checked for depending on it |

Other things held equal:

- **Sensor and range:** all planners read the same `sensor.detection` sigmoid, `beta` and turn radius from the scenario. A spec's `overrides` (e.g. `specs/wide900.json`: `c = beta = max_slant_range_m = 900`) change the world for every planner and the scorer at once.
- **Budget, speed, altitude, home:** from the scenario. `flown_m` is stored, so budget use can be checked.
- **TIGRIS's gimbal:** TIGRIS plans a fixed sweep pattern, so its sweep is tuned rather than left at the flown ±45° / 30°/s. `arms/tigris_tuning.json` has ±45°, "detection" (the widest angle whose slant stays inside 0.97·beta, capped at 75°) at 30 and 60°/s (60°/s is the fastest a 120°/s axis follows), body-fixed, and the matched reward. Pick the best arm by `residual_hw` before comparing.
- **Compute:** TIGRIS iteration-bounded (150 per replan, insensitive, see above); MTL runs to completion (median 13 s for info-aware).

What is *not* equalised, because it is the algorithm: MTL schedules its gimbal look by look against the belief, while TIGRIS sweeps a pattern and plans the path under it; MTL optimises the residual metric itself, TIGRIS its entropy (or matched) reward.

Add your own arms freely. For example, TIGRIS with `reward_mode: matched`; MTL with a different `mapping.max_cluster_radius_m` through `scenario_overrides`; or info-aware with `info_aware.level_sets` changed.
