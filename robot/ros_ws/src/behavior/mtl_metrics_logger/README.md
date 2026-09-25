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

When the follower reports `COMPLETE` or `ABORTED`, the logger writes:

```
runs/<run_id>/<robot>/telemetry.csv    # 20 Hz: t, pose, speed, gimbal cmd/state, boresight, footprint, cumulative metrics
runs/<run_id>/<robot>/detection.json   # mtl.detection/1: per-target P_det / time-to-detect, coverage, info (mass) gathered vs planned
runs/<run_id>/<robot>/report.html      # self-contained report: KPIs, map over the prior, curves, target table
runs/<run_id>/{ground_truth.json,belief.png}   # copied for the team analysis
```

Live outputs:

- `search/detection_markers`: `MarkerArray` of targets, coloured by P_det.
- `search/footprint`: `PolygonStamped`.
- `search/metrics`: `std_msgs/String` carrying compact JSON, for dashboards.

**Team report.** `scripts/analyze_mtl_run.py --run-dir runs/latest` fuses every robot's
telemetry on one timeline. It re-scores that timeline against the ground truth and writes
the **team** `telemetry.csv` / `detection.json` / `report.html` into the run folder. Scoring
the team is the right metric: two robots that each see a target at P = 0.7 have discovered
it together.

## Code

- `detection.py`: the detection model and the team scorer. Stdlib only.
- `report.py`: telemetry CSV I/O and the HTML report.
- `analysis.py`: per-agent and team scoring from recorded runs.
- `logger_node.py`: the rclpy shell.

## Tests

`test/test_detection.py` and `test/test_logger_node.py` use hermetic `rclpy` stubs.
