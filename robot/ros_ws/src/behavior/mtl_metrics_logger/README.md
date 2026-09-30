# mtl_metrics_logger

The online scorer and run recorder of the
[`mtl_search`](../../../../../stacks/mtl_search/README.md) stack.

## What it does

The logger records while the follower is `INGRESS` or `SEARCH` for a started plan. It
samples at 20 Hz: odometry, measured gimbal state (the command is the fallback), and
follower progress. On every sample it scores detections of the scenario's ground-truth
targets:

- **Footprint:** radius `r = d_slant · tan(FOV/2)` around the boresight's ground point.
- **Gate:** a target counts only when it lies inside that footprint **and** its 3-D range
  `r` from the camera is at most `β`.
- **Per-look probability:** the Moon et al. sigmoid `P = 1 / (a + exp(b·(r − c)))`. Outside
  the gate `P = p_out`. The parameters come from `sensor.detection` in the scenario.
- **Miss product, rate-normalised:** `P_miss ← P_miss · (1 − P)^(dt / dt_ref)`, with
  `dt_ref = 0.1 s`. The result doesn't depend on the logging rate.
- **Discovery:** a target is discovered when `1 − P_miss ≥ threshold` (0.9), and its time is
  recorded.

**Residual belief: the headline score.** The prior is a probability mass function: the
scenario raster, rebuilt from `scenario.json`'s `airstack.belief` bumps, sums to 1. Every
pixel `x` of it gets the same miss update as a target standing there, with the same gates,
sigmoid and `dt / dt_ref` exponent:

```
residual(x)  = prior(x) · Π_looks (1 − P(z|x))^(dt/dt_ref)   = P(target at x AND every look missed it)
residualMass = Σ_x residual(x)                               = P(the search missed the target)
```

**Lower is better.** It is 1 when nothing was looked at and goes to 0 when everything was seen
well. Compare planners on this number, using the same prior, sensor model and `dt_ref`. It is
the Python port of `mtl::eval::computeResidualBelief` from the vendored planner (see
`third_party/mtl_planner/CHANGES_belief_mass_and_residual.md` §3). At `dt = dt_ref` both give
the same number: checked to 1e-12 on a planned team track. The **planned** residual scores
the planned track and its scheduled boresight points the same way, so plan and flight can be
compared directly. The planner writes these into each robot's `track.json`, and they also
arrive on `search/plan`.

The older **valid-cell coverage** numbers (`belief_mass_covered`, `belief_mass_fraction`) are
still reported. A cell counts, with its whole mass, once its centre falls inside any
footprint. Cell masses are probabilities now, so these numbers are probabilities too.

**Planners and comparison plans.** The scenario's `planner.type` picks the planner that is
flown: `orienteering` (plain or information-aware mode, `info_aware.enabled`) or `curve` (the
parameterized-curve planner). A track's `planner_mode` is `plain` | `info_aware` | `curve`.
The planner also writes comparison plans, never flown: the other orienteering mode as
`track_alt.json` (`info_aware.report_both`), or, when the curve flies, both orienteering modes
as `track_alt_plain.json` and `track_alt_info_aware.json` (`planner.compare_orienteering`).
The logger reads whichever are there (`analysis.alternatives_from_dir`), scores each one's
planned residual with the same model and adds `summary.planner_comparison` to
`detection.json` (`flown_mode`, `flown_type`, the flown plan's planned and flown residual,
and `alternatives`: every comparison plan's mode, type, planned residual and length; the
first is also kept as `alternative`), writes `planner_comparison.csv`, and gives the report a
"Planners" card and each comparison plan's track (grey, one dash pattern each) on the map. A
flown curve plan's diagnostics from `track.json` (sweep amplitude / frequency / peak rate vs
gimbal rate, swath half-width, max curvature vs `1/R`, endpoint, fast objective) become
`summary.curve` and a "Parameterized-curve plan" card. The flown score never depends on
any of it, and the detection / residual model is planner-agnostic (it scores sensor looks),
so older run folders re-score unchanged.

When the follower reports `COMPLETE` or `ABORTED`, the logger writes:

```
runs/<run_id>/<robot>/telemetry.csv    # 20 Hz: t, pose, speed, gimbal cmd/state, boresight, footprint, cumulative metrics
runs/<run_id>/<robot>/detection.json   # mtl.detection/1: residual belief (flown + planned), per-target P_det / time-to-detect, cell coverage, curves
runs/<run_id>/<robot>/residual_belief.csv   # x,y,prior,residual per 10 m block (world ENU; block sums, columns sum to 1 / residual)
runs/<run_id>/<robot>/planner_comparison.csv   # flown plan + every comparison plan: type, mode, planned / flown residual, length
runs/<run_id>/<robot>/report.html      # self-contained report: residual KPI, planners card, map over the prior, prior-vs-residual panels, curves, target table
runs/<run_id>/{ground_truth.json,belief.png}   # copied for the team analysis
```

Live outputs:

- `search/detection_markers`: `MarkerArray` of targets, coloured by P_det.
- `search/footprint`: `PolygonStamped`.
- `search/metrics`: `std_msgs/String` carrying compact JSON, for dashboards (includes
  `residual_mass`, this robot's residual belief so far).

**Team report.** `scripts/analyze_mtl_run.py --run-dir runs/latest` fuses every robot's
telemetry on one timeline. It re-scores that timeline against the ground truth and writes
the **team** `telemetry.csv` / `detection.json` / `residual_belief.csv` / `report.html` into
the run folder. Score the team as a whole: two robots that each see a target at P = 0.7 have
found it together, and the team residual belief is the one to compare planners on.

## Code

- `detection.py`: the detection model, the team scorer and the residual belief
  (`PriorGrid`, `prior_from_scenario`, `ResidualBelief`, `planned_residual`). Stdlib only.
- `report.py`: telemetry CSV I/O and the HTML report.
- `analysis.py`: per-agent and team scoring from recorded runs.
- `logger_node.py`: the rclpy shell.

## Tests

`test/test_detection.py` and `test/test_logger_node.py` use hermetic `rclpy` stubs. The
residual tests check the following against a brute-force reference:

- no looks gives 1, and `0 ≤ residual ≤ prior`;
- `residual / prior = P_miss` for targets that sit on a pixel centre;
- the result does not depend on the logging rate;
- the planned residual equals the flown one on an identical track;
- the prior rebuilt from the bumps matches `scenario.generate_belief`.
