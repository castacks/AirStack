# Learned-depth versus simulator-depth diagnosis

This is a privileged-input **diagnostic**, not a normal RGB model evaluation,
attack trial, or agent comparison. It asks whether replacing learned depth makes
an otherwise unchanged clean flight work. It does not prove the cause by itself.

## Protocol

Provide one existing clean `scenario.json` for each target. The runner preserves
the actual scene coordinates, lighting, mission, speed, timeout and controller.
It disables perturbations and runs learned depth followed by GT depth, starting
a fresh world/worker for each episode. It records small evidence files, not bags.

```bash
python3 tools/ws2_bench/depth_diagnostic.py \
  --output robot/ros_ws/ws2_runtime/campaigns/my_depth_diagnostic \
  path/to/mononav/clean/scenario.json path/to/kim/clean/scenario.json
```

Use `--summarize-only` with the same arguments to rebuild the derived summary
without flights. It requires all planned episode results. Original results are
never rewritten. The runner takes the campaign/episode locks used by the bench.

`diagnostic_depth_source: learned|ground-truth` is optional and omitted from normal
resolved scenarios, preserving their configuration fingerprints. Agent actions
cannot choose it. A diagnostic scenario rejects noise/delay/patch settings.
Results carry `evaluation_track: depth_source_diagnostic`.

## Input checks and interpretation

- Both workers check RGB/GT resolution equality and reject timestamp skew over
  0.15 simulation seconds. This is a bounded approximate match, not exact sync.
- The current Isaac ROS camera helper maps `type=depth` to
  `DistanceToImagePlane`: optical-axis metric depth, not radial range.
- MonoNav GT depth uses the existing camera-intrinsic transform and millimetre
  conversion, then the same TSDF/primitives. It bypasses learned-depth scale and
  speckle correction, which exist to correct learned predictions.
- Kim GT depth uses the existing crop/resize/normalization for D3QN and bypasses
  FCRN. D3QN history/actions and the integration governor remain. This is not
  necessarily the same input distribution as the one used to train D3QN.
- Inference latency changes when perception is bypassed. `summary.json` includes
  observed depth source, inference time, input intervals and timestamp skew.
- A successful GT trial would suggest inspecting perception/scale/timing first,
  not prove that estimation error alone caused the earlier failure.
- Failure with GT may still involve mapping, planning, control, timing or input
  distribution. A runtime crash is inconclusive for avoidance performance.
  `interpretation_status` flags nonzero worker exits even in older results that
  classified them as `planner_stopped`. New episodes classify such exits as
  `infrastructure_error`; normal zero-exit/terminal stops remain separate.

The October 14 results and limits are recorded in [VALIDATION.md](VALIDATION.md).
They must not be pooled with the normal RGB pilot or attack success rates.
