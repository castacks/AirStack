# Stereo depth assessment and pixel-to-map preview

## Frozen candidate comparison — 2026-10-15

The evaluation-only `scripts/benchmark_stereo_candidates.py` separates `predict`
(RGB/stereo/calibration only) from `assess` (simulator truth). It compares recorded
BM, fixed OpenCV SGBM64/128 and official RAFT-Stereo Middlebury, without teacher
scale fitting or runtime deployment. All outputs are diagnostic; nothing produces
localization facts, waypoints, semantic identity or permission. See
`rrm/stereo_benchmark.py` for common-support and all-teacher yield accounting.

Eighteen frozen stationary Office pairs across two captures were processed; both
missing attempts remain recorded. Second capture, original disparity window:

| Candidate | Pooled coverage | Median / p95 absolute error |
| --- | ---: | --- |
| Recorded BM |18.0%|0.152 / 7.301 m|
| SGBM64 |45.3%|0.337 / 37.469 m|
| SGBM128 |36.2%|0.305 / 38.603 m|
| RAFT Middlebury |100%|2.060 / 60.889 m|

These own-support errors are not directly comparable accuracy scores. On identical
all-candidate support within the conservative observable-domain mask, BM median/p95
was 0.159/7.333 m versus RAFT 4.878/42.463 m (133,505 pixels). The mask uses the
same conservative image window and teacher depth >= fB/63; it does not prove actual
two-camera visibility. Synthetic 8 px shift recovered 8.001 px with RAFT, checking
integration sign/scale. Median timings were SGBM64 13.5 ms, SGBM128 17.6 ms and RAFT
333.4 ms on the simulator-shared RTX PRO 5000; not deployment deadlines.

No candidate adopted. Dense output and geometric coherence do not establish
accuracy. The single learned checkpoint failed this dataset; no general conclusion
about learned/monocular depth follows. Native pairing and controlled observable
target/registration validation remain necessary. All 23 focused tests and independent
arithmetic review pass. Raw results/source/weights/input checksum bundle is local
`.rrm-artifacts/depth-model-benchmark-20261015/`; export before workspace replacement.

### Offline reproduction

Use an isolated diagnostic environment with NumPy, OpenCV and GPU PyTorch. The
learned dependency is [upstream RAFT-Stereo](https://github.com/princeton-vl/RAFT-Stereo/tree/6e93ed2169bd858dbb43033988563f3b0bb49506)
at commit `6e93ed2169bd858dbb43033988563f3b0bb49506`, with its official Middlebury
checkpoint, SHA256 `d22e84c0e431bf31d7cc66902c40601859eb40b35ef7f4399ea81276c2915819`.
It additionally imports SciPy and opt_einsum (3.4.0 used). No dependencies were
added to RRM core or the active robot environment. Capture names must be distinct,
inputs must be original 480x300 RGB uint8, and output directories must not exist.

```bash
python scripts/benchmark_stereo_candidates.py predict \
  --captures /data/capture /data/validated-capture --output /results/predictions \
  --raft-repo /deps/RAFT-Stereo --weights /deps/raftstereo-middlebury.pth
python scripts/benchmark_stereo_candidates.py assess \
  --captures /data/capture /data/validated-capture \
  --predictions /results/predictions --output /results/assessment
```

Both commands preserve manifest provenance; predictions are raw positive
left-minus-right disparities (the RAFT flow is negated). Scoring includes all valid
positive teacher pixels in each declared domain, no teacher-fitted scale and no
post-hoc error clipping. Common-support metrics are separately labeled conditional
scores, not full-area coverage. Uncertainty is not calibrated.

## Read-only prototype

The read-only prototype in `rrm/stereo_localization.py` backprojects calibrated
stereo pixels into `camera_left`, then applies a time-matched `map` transform.
It returns a representative **visible surface point**, not an object center,
approach waypoint, localization fact or dispatch permission. Pixel regions are
explicit operator selections; their semantic association remains unverified.

Inputs bind an exact image checksum, image/right-image/calibration/disparity/TF
stamps and simulator episode. The adapter rejects mismatched image dimensions,
incompatible left/right rectified projection matrices, invalid/empty disparity
windows, missing TF, stale observations, nonfinite geometry, insufficient support
and excessive depth spread. The frozen development policy requires at least 32
samples, 20% region coverage, depth MAD/median <= 0.15, depth 0.2–30 m and receipt age <= 1 s.
Coordinate MAD and disparity half-bin depth sensitivity are descriptive quantities,
not calibrated uncertainty or physical accuracy bounds. Estimator reset ownership
and complete operational observation provenance remain further integration work.

The independent assessment in `rrm/stereo_depth_assessment.py` compares stereo
depth with Isaac's DistanceToImagePlane output. The installed Isaac 5.1 camera
helper uses this optical-axis depth for its `depth` topic. Teacher arrays are
evaluation-only and never enter the localizer. The candidate API works without
teacher data; the capture harness waits for it to produce paired assessment records.
No assessment verdict changes localizer acceptance or promotes its output to C02.

## Capture and reproduce

Run `scripts/airstack_stereo_depth_capture.py` in the robot container with sourced
ROS workspace, RRM/Pydantic v2, NumPy, OpenCV and the patched stereo dependency.
The RRM source folder must be copied into that container; it is not normally mounted.
Supply the verified simulator runtime identity and a new output directory:

```bash
ROS_DOMAIN_ID=1 python3 scripts/airstack_stereo_depth_capture.py \
  --output /tmp/new-stereo-depth-run --episode-id VERIFIED_SIMULATOR_RUNTIME_ID \
  --duration-s 15 --max-frames 10
```

The recorder creates only subscriptions and diagnostic files. It samples the
first disparity at each one-second interval, gives corresponding channels/TF up
to 0.5 s to arrive, and retains incomplete attempts. Each complete sample stores
raw stereo images, disparity, assessment depth, calibration, pose, source binding,
prototype results and separate error measurements. The three development regions
are frozen for 480x300 Office images; adapting regions/resolution is a new protocol
revision. Stop/flight capability is not supplied by this script.

ROS header matches establish transport pairing. They do not independently prove
native same-render RGB/depth correspondence, sensor acquisition time or moving-scene
registration. The current stationary results therefore remain diagnostic estimates.

## Measured checkpoint — 2026-10-14 EDT

Two grounded 15-second captures in standard Office each retained 9/10 paired attempts.
The first missed a left image; the second missed assessment depth. Calibration
rejection gates were strengthened after review and exercised in the second run.
Eighteen focused geometry, adapter and assessment tests pass in the robot container.

| Second-run region | Median frame teacher coverage | Pooled median absolute error | Preview /10 attempts |
| --- | ---: | ---: | ---: |
| Whole disparity window |17.99% |0.152 m | Not an operator region |
| Floor patch |0% | Unmeasurable |0/10 |
| Central view |22.47% |0.208 m |7/10 |
| Region labeled `low_texture` |62.95% |0.131 m |8/10 |

Coverage uses all finite positive teacher pixels within the disparity window/region.
Whole-window coverage restricted to teacher depths inside the configured near-range
envelope was 31.55%. Both denominators are reported; neither hides missing attempts.
Whole-window pooled mean absolute error was 2.222 m, with 95th-percentile absolute
error 7.301 m; rare large errors are retained. In the first run the central region's
median per-frame mean error was 3.664 m despite coherent geometry previews. Coherence
and metadata checks cannot establish depth accuracy.

The floor region's teacher median axial depth was 0.0819 m, entirely below the
configured stereo minimum of about 0.3505 m (`f*baseline/max_disparity`). Its missing
stereo depth is a geometry/search-range limit and cannot alone motivate replacing
the matcher with a learned model. A model comparison should separate observable
target regions from out-of-envelope pixels and evaluate coverage, outliers and
latency on held-out captures.

The vehicle remained connected/disarmed/landed, and no goal, flight or learned model
inference was issued. These are standard Office surface regions, not the seven
RRM marker cases. The existing semantic navigation provenance/admission gates stay
in force; this prototype emits no `DroneNavigationTarget`, C02 facts or route hash.
