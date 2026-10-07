# Planner benchmark: MTL curve planner vs plain / information-aware MTL vs TIGRIS

Offline and ROS-free. It generates many search problems, runs each planner's own CLI on every
one, and scores every plan with the same metric (residual belief = P(target missed), scored on the look points a real gimbal achieves,
lower is better). A report then shows where each planner wins.

```bash
# 0. once: build the two planner CLIs on the host (needs g++ and libeigen3-dev); mtl_search_plan
#    contains both MTL planners (orienteering and curve; the scenario's planner.type picks one)
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

The curve planner (2026-09-30, see "The curve arm" below), added to the same bench without re-running anything:

```bash
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide900 --arms scripts/planner_benchmark/arms/curve_tuning.json --limit 30 --workers 2
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide900 --arms scripts/planner_benchmark/arms/curve900.json --workers 2
python3 scripts/planner_benchmark/make_report.py --bench bench/wide900 --focus mtl_curve --against tigris_det_60dps,mtl_info_aware_r800 \
    --arms mtl_curve,tigris_det_60dps,mtl_info_aware_r800,mtl_plain_r800,mtl_plain \
    --out bench/wide900/report_curve.html --csv bench/wide900/results_curve.csv
python3 scripts/planner_benchmark/plot_tracks.py --bench bench/wide900 --arms mtl_curve,tigris_det_60dps,mtl_info_aware_r800 \
    --out bench/wide900/track_maps_curve
# from scratch, all five arms of the comparison in one call (scenarios: unzip bench/wide900/scenarios.zip, do not regenerate):
python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide900 --arms scripts/planner_benchmark/arms/fair900_curve.json --workers 8
```

`tigris_det_60dps_matched` is left out of the last two steps on purpose: the matched reward takes 20+ min per plan at 1000 s budgets. Run it with `--only-arms` on the short budgets if you want it.

Run `specs/smoke.json` first: 12 scenarios, to check the pipeline end to end. Needs Python 3 with
numpy; matplotlib is optional (it draws the scenario maps in the report), and PyYAML is optional
(it lets TIGRIS read the stack's tuning).

## Pieces

| File | What it does |
|---|---|
| `build_offline_tools.sh` | Builds `bin/mtl_search_plan` (adapter + vendored `mtl::planner` + vendored `mtl::curve` planner, source lists read from `mtl_search_planner/cmake/mtl_vendored.cmake`) and `bin/tigris_search_plan`. Eigen only, `-O2`, no OpenMP: plans are deterministic. |
| `priors.py` | Ten prior families, all written as mixtures of Gaussian bumps plus a uniform floor. That is the one prior representation every consumer in the stack rebuilds identically (both planners, the logger, the Isaac scene, `score.py`). Families: `gaussian_blobs`, `clustered`, `heavy_tailed` (Pareto peak heights), `decoy` (weak blobs near home, a heavy group far away), `lines` (roads/rivers), `ring` (range-only annuli, optionally open), `drift_plume` (a search-and-rescue drift from a last-known point), `random_field` (log-normal lattice), `diffuse_plus_peaks` (a floor under a few peaks), `multi_scale` (wide weak regions with sharp strong peaks). |
| `gen_scenarios.py` | Spec -> `scenarios/<id>.json` (a copy of `stacks/mtl_search/config/scenario.json` with the prior, cells, budget, altitude and home replaced) and `index.json` (the factors plus prior descriptors: effective area, spread, budget ÷ spread, mass within half the budget of home, peak count, Gini, cells holding 50 / 90 % of the mass). |
| `run_benchmark.py` | Runs every arm in an arms file on every scenario: `mtl_search_plan --no-alt` for MTL, `tigris_search_plan` for TIGRIS (starting from the stack's TIGRIS yaml, then the arm's `tigris_set`). Scores each track, and appends a line to `results.jsonl` holding the residual, the anytime curve, flown length, plan time, a decimated track, and the info-aware choice. Resumable: a pair that already has a result is skipped. |
| `score.py` | A numpy port of `mtl::eval::computeResidualBelief`. It agrees with the C++ metric to 1e-6 on the flown runs of 2026-09-29 (0.42585 / 0.42303 at 10 m). |
| `plot_tracks.py` | A static trajectory map for every scenario, one panel per arm (`--arms A,B,C`): `track_maps/<id>.png` plus all of them in `track_maps/all_tracks.pdf`, sorted by where the second arm wins most. |
| `make_report.py` | `report.html` + `results_wide.csv` (both described below; `--out` / `--csv` rename them). `--metric` picks the score (default `residual_hw`); `--arms` limits the report to a set of arms, so scenarios missing a result for an unrelated arm are not dropped. `--focus A,B` compares two arms; `--focus A --against B,C` (primary-arm mode) makes A the subject and repeats every head-to-head section against each opponent. |
| `arms/default.json` | `mtl_plain`, `mtl_info_aware`, `tigris_sweep` (the flown TIGRIS: sweeping gimbal), `tigris_fixed` (body-fixed camera); TIGRIS at 600 iterations per solve. |
| `arms/quick.json` | The same without `tigris_fixed`, and TIGRIS at 150 iterations per solve (about 4x faster to run). |
| `arms/tigris_tuning.json` | Both MTL modes plus five TIGRIS configurations (sweep amplitude × rate, body-fixed, matched reward), for picking TIGRIS's best before a comparison. |
| `arms/mtl_tuning.json` | Both MTL modes at cluster radius 400 and 800 m (the stack's 550 m was chosen for the 610 m sensor). |
| `arms/fair900.json` | The final comparison: `mtl_plain` (stack), `mtl_plain_r800`, `mtl_info_aware_r800`, `tigris_det_60dps`. |
| `arms/fair_sweep.json` | TIGRIS with the detection-matched sweep only. |
| `arms/curve_tuning.json` | The curve planner's tuning arms (first 30 scenarios): `mtl_curve_f050`, `_f075` (stack), `_f100`, `_f140` = `sweep_freq_hz`, all with TIGRIS's sensor / gimbal model (below). |
| `arms/curve900.json` | The chosen curve arm, `mtl_curve` (= `mtl_curve_f140`), run on all 160 scenarios. |
| `sim_scenario.py` | Flies one scenario in Isaac Sim with the curve planner or TIGRIS: `list` / `new` (a scenario from family, budget, altitude, home and family parameters) / `derive` (an existing scenario's prior with other drones: budget, altitude, homes, team size) / `export` / `fly` / `compare` / `restore` (see "Flying a scenario in Isaac Sim"). |
| `arms/fair900_curve.json` | The four `fair900.json` arms plus `mtl_curve`: the whole comparison from scratch in one command. |
| `specs/*.json` | `wide` (240 scenarios, all families), `wide900` (160, detection range 900 m for everyone), `team` (60, 2–3 agents: MTL plans jointly, TIGRIS per agent), `smoke` (12). |

### What the report contains

- **Leaderboard:** mean and median residual, mean rank, the share of scenarios each arm wins outright, and its pairwise win rate against every other arm.
- **Head to head** of the two `--focus` arms (in primary-arm mode: the subject against each `--against` arm, each with all of the following):
  - win rate, and the "decisive" rate: wins by more than `--decisive` (default 0.05);
  - the mean advantage by prior family, budget, altitude and home, and by quartile of each prior descriptor;
  - a family × budget heat map.
- **Anytime curves:** mean residual against the fraction of distance flown.
- **Scatter plot:** one dot per scenario, with a family filter.
- **Maps** of the most and least decisive scenarios: the prior with both tracks drawn over it.
- **Full table:** every scenario, sortable.
- **Scene viewer:** click any scenario (a table row, a scatter dot or a map card) to open its prior with every arm's path and camera ground points. It shows the focus arms (primary-arm mode: the subject, solid in the first colour, and every opponent, each with its own dash) overlaid, with a toggle per arm and a side-by-side mode; hovering a path gives the distance flown at that point; ← / → step through scenarios. It adds about 20 KB per scenario; `--no-viewer` leaves it out.

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

## The curve arm (`mtl_curve`, 2026-09-30)

`mtl::curve::Planner` (vendored `third_party/mtl_curve_planner`, upstream `cpp_curve_planner` `a2c5e9d`) plans ONE
continuous curve per aircraft, exactly the budget long, with curvature never above 1 / turn radius, optimised for
the lowest residual belief while the 1-DOF gimbal sweeps `alpha(t) = alphaMax sin(2 pi f t)` across track. It runs
through the same CLI as the orienteering planner (`mtl_search_plan --no-alt`); the arm selects it with
`scenario_overrides.planner = {"type": "curve", "compare_orienteering": false}` and passes the `curve` block. No
runner change was needed for the plan itself; for curve arms `run_benchmark.py` additionally records
`residual_hw_openloop` / `curve_hw_openloop` (secondary score, below), `sweep_check` (planned max |phi| off the mount
axis, max |roll + phi|, max planned slew rate, `within_limits` against 80 deg and 120 deg/s) and `curve_meta` (the
planner's own diagnostics from `plan.json`: amplitude, frequency, peak rate, optimiser exit, fast residual, ...).
Existing arms are untouched.

### Fairness checks

- **Same world.** The adapter scales absent keys from the scenario: grids and knots by `size_m / 5000` (5 km ->
  x1), the swath kernel and altitude stagger by `beta / 610` (900 -> x1.475; the stagger is set to 1 m as in
  `mission.yaml`, irrelevant for one agent). The detection sigmoid (a 1.1, b 0.1, c = beta = 900 m, p_out 1e-6),
  FOV 60 deg, tilt 30 deg, altitude, speed 6 m/s, turn radius 12 m and the budget all come from the scenario, as for
  TIGRIS and the orienteering planner.
- **Same sensor and gimbal model as TIGRIS** (`curve_tuning.json` `_doc`):
  - **Reach:** the curve adapter reads the ground reach only from `sensor.max_sensor_reach_m`, default **600 m,
    not scaled with beta** (the orienteering adapter falls back to `max_slant_range_m` = 900). Left alone it caps
    the sweep at 60 deg at 300 m instead of the slant bound. The arm sets `max_sensor_reach_m = 900`. (Cost of the
    cap in a preliminary run: 0.0005 mean residual, `bench/wide900/side/curve_prelim_reach_fix.jsonl`.)
  - **Amplitude:** `sweep_range_margin = 0.97` and `gimbal.gimbal_max_deg = 75` give exactly TIGRIS's "detection"
    sweep: the angle where the boresight slant reaches 0.97 beta, capped at 75 deg (66.62 deg at 300 m for both). In
    `mtl::curve` `gimbalMax` only caps the amplitude.
  - **Scoring:** `score.py` flies curve tracks through the same mount as TIGRIS: 80 deg travel, 120 deg/s per axis,
    roll +-80 / pitch -20..110 deg, **no pitch nudge** (`hardware_for`: the curve plans at a fixed look angle, like
    TIGRIS's sweep; the orienteering MTL arms keep their +-5 deg nudge, which moves them < 0.002).
  - **Sweep rate** is tuned, as TIGRIS's was: `sweep_freq_hz` {0.05, 0.075 (stack), 0.10, 0.14}; 0.14 Hz is about
    60 deg/s peak at 300 m (66 deg/s at 200 m), TIGRIS's best rate. The sweep shape differs (sinusoid vs TIGRIS's
    triangle wave); that is each planner's own pattern.
- **Same scoring.** `residual`, `residual_hw`, `curve` / `curve_hw`, `flown_m`, `plan_s` exactly as for every arm;
  `residual_hw` is the headline.
- **Budget.** Every curve uses its whole budget and never more: `flown_m == budget_m` in every plan.
- **Gimbal.** The planned slew rate peaks at 67 deg/s (limit 120). The level-frame sweep never exceeds 74.7 deg, but
  phi off the mount axis adds the bank: in 3 of 160 plans (all at 200 m: `s0049`, `s0064`, `s0109`) it briefly
  reaches 80.1-82.2 deg, past the 80 deg travel (`sweep_check.within_limits = false`). The scorer clamps it to 80 deg
  like any plan; those three scores do not change (hardware = planned to 1e-4).
- **Open-loop law.** The flown stack uses `follower.gimbal_law: open_loop` (replay the planned cross-track angle).
  `residual_hw_openloop` scores that law: it equals the planned score (the offline pose is the planned pose and the
  sweep is well below the slew limit) and differs from the `aim_point` headline by -0.0002 on average (largest
  single difference 0.012, `s0058` at 200 m).
- **Compute.** Serial, deterministic (no OpenMP), runs to completion like the other MTL arms; tens of seconds per
  plan (see the report's leaderboard for mean / median / max).

What is not equalised, because it is the algorithm: the curve optimises its own smoothed swath-kernel
approximation of the same detection model and a sinusoidal sweep; TIGRIS samples paths under a triangle-wave sweep
and optimises its entropy reward; the orienteering MTL schedules looks one by one.

### Result (130 informative scenarios, `report_curve.html`)

Tuning (first 30 scenarios, 24 informative): `sweep_freq_hz` 0.05 / 0.075 / 0.10 / 0.14 -> mean residual_hw
0.3314 / 0.3297 / 0.3290 / **0.3285**; chosen 0.14 Hz (`mtl_curve`).

| Arm | Mean residual_hw | Median | Plan time mean / median / max [s] |
|---|---|---|---|
| **mtl_curve** | **0.238** | 0.146 | 55 / 44 / 235 |
| tigris_det_60dps | 0.289 | 0.242 | 81 / 49 / 452 |
| mtl_info_aware_r800 | 0.292 | 0.256 | 17 / 10 / 74 |
| mtl_plain_r800 | 0.324 | 0.319 | 0.6 / 0.4 / 3 |
| mtl_plain (stack) | 0.343 | 0.320 | 0.6 / 0.4 / 2 |

- vs TIGRIS det/60: curve better in 98 % (no loss; the rest are ties), decisively (> 0.05) in 38 %; mean -0.051
  (-45 % relative). It wins in every budget, home, altitude and family group; the margin is smallest at 400 s
  (+0.032), from corner homes (+0.030), and for drift plumes and rings (+0.016 / +0.027).
- vs info-aware MTL r800: better in 91 %, worse in 2 % (3 scenarios, each by < 0.0005), decisively better in 44 %;
  mean -0.054 (-40 %). Largest at 700-1000 s (+0.07-0.08) and for rings, random fields and gaussian blobs; smallest
  for clustered and decoy priors (+0.004 / +0.016), the info-aware planner's strongest families.

## Flying a scenario in Isaac Sim (`sim_scenario.py`)

The Isaac stacks build their scenario from `mission.yaml`, which only makes random Gaussian priors.
`sim_scenario.py` writes a benchmark scenario (or a new one) straight into the stack bundles, with
the arm's overrides applied by the same code as the benchmark (`run_benchmark.apply_arm`), so the
flight starts from exactly the problem the benchmark planned. Exported bundles re-plan offline to
the benchmark's numbers exactly (checked: `s0136` curve 0.1579, TIGRIS 0.3809).

```bash
cd bench/wide900 && mkdir -p scenarios && (cd scenarios && unzip -o ../scenarios.zip) && cd ../..   # once
P=scripts/planner_benchmark/sim_scenario.py
python3 $P list --budget-s 700 --home near_center --sort adv          # pick one of the 160 ...
python3 $P new --family gaussian_blobs --budget-s 700 --altitude-m 300 --home near_center \
    --param n_peaks=8 --param 'sigma=[150,300]' --seed 7 --offline    # ... or make one (-> bench/wide900/custom/)
python3 $P derive --from s0136_gaussian_blobs --agents 3 --home edge --budget-s 700 --offline   # same prior, other drones
python3 $P fly --id s0136_gaussian_blobs --planner curve              # export + airstack up + sortie + airstack down
python3 $P fly --id s0136_gaussian_blobs --planner tigris
python3 $P compare --id s0136_gaussian_blobs                          # flown vs planned vs offline
python3 $P restore                                                    # original bundles + fleet files back
```

- **What `fly` runs:** `airstack down`, `airstack up --sim isaac --fleet <mtl|tigris>_search_fleet --stack
  <mtl|tigris>_search --play --wait`, `scripts/<mtl|tigris>_start_mission.sh -n 1 -a <scenario altitude> -r <run_id>`,
  `airstack down`. Everything the start scripts always saved is saved as before under `runs/<run_id>/`: the MCAP
  rosbag (`robot_1/bag/`), telemetry, `detection.json`, `report.html`, the Foxglove file. The run is also
  recorded in `bench/wide900/sim/<id>/runs.json`; `export.json` there holds the export and the offline scores.
  `--print-only` prints the commands, `--keep-up` leaves the sim running, `--no-up` uses a sim you started.
- **Same problem for both planners:** prior, cells, budget, altitude, home (robot_1's spawn is set in both fleet
  files), sensor and detection model, and the ground-truth targets (sampled from the prior with
  `mission.yaml`'s `targets:` block, seed = scenario seed + 1).
- **`new`:** family parameters you do not give are sampled from `--seed` (printed). An unknown parameter
  lists the family's parameters. Homes: `near_center` / `edge` / `corner` (the benchmark's) or `N,E` in
  mission NED metres. `--offline` also plans and scores it with both arms (`bench/wide900/custom/results.jsonl`).
- **`derive`:** keeps the prior, the cells and the mission seed (so the same ground-truth targets) of `--from`
  and changes the drones: `--budget-s` (per drone), `--altitude-m`, `--agents N`, `--home` (one site; the team
  lines up `--spacing` m apart along east, 12 m by default, like the fleet file) or `--homes="N,E;N,E"` (one
  per drone; write it with `=` because the values start with `-`). Drone i flies i x 1 m higher (the stack's
  `team.altitude_separation_m` layers, = the curve's `altitude_stagger_m`). Default id
  `<from>__<N>a_<budget>s_<alt>m_<home>`. Teams fly with `--planner curve` only: the `tigris_search` stack
  flies one robot, and TIGRIS has no team coordination (offline it flies each drone separately on the full
  prior). `export` / `fly` rewrite the fleet's `robots:` block to robot_1..N and start N sorties; the
  mtl_search fleet was set up for up to 3 robots (more is untested).
- **Differences from offline, by design of a live flight:** TIGRIS replans every 5 s with a 5 s wall-time solve
  (the stack's `tigris_search_planner.yaml`) instead of 150 iterations per replan; the follower flies the real
  gimbal and controller. TIGRIS plans carry no gimbal law, so the follower aims (`aim_point`); the curve
  is exported with the same law (`--gimbal-law open_loop` gives the stack's own setting).
- **Before committing**, run `restore`: the stack bundles and both fleet files are modified while you fly
  (`mtl_generate_scenario.py --check` would report them stale). The originals are kept in
  `bench/wide900/sim/_stack_backup/` until then.
