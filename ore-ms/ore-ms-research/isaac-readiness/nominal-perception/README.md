# Nominal perception measurement

## Roadmap alignment

M12 is **Physical Sensor Verification**. It requires separately authorized, calibrated physical RGB/LWIR recordings and controlled reference measurements. This work is a stationary Isaac Sim measurement capability under R1. It leaves the authorized milestones and the separate M11 geometry fixture intact. [Roadmap mandate](../../RESEARCH_ROADMAP_MANDATE.md) · [Acceptance conditions](../../ACCEPTANCE_CONDITIONS_REGISTER.md) · [R1 infrastructure](../RUNTIME_BASELINE_R1.md) · [Geometry fixture](../GEOMETRY_FIXTURE_PROTOCOL.md)

The `full_default` stack routes rectified RGB and CameraInfo into `stereo_image_proc` for disparity and point clouds. The simulator provides right depth on a topic labeled ground truth. Ouster supplies a PointCloud2 input to the VDB mapper. MAVROS and odometry conversion expose estimator poses; TF supplies the live frame graph. A VDB marker is the visualized map. The standing ROS configuration did not provide a qualified LWIR stream, independent reference poses, or a verified alignment between disparity pixels, the right depth image and scene geometry. ROS header stamps and `/clock` were available. CameraInfo supplies declared calibration. Axial versus ray-depth semantics, optical conventions, clipping, exposure timing and a pinned reference scene remain unqualified. [Camera geometry audit](../CAMERA_GEOMETRY.md)

R1 verifies readiness and basic stream/TF health. Existing AirStack sensor probes test delivery, and the M11 frozen fixture defines independent geometry checks. This tool adds cadence, synchronization and payload usability measurements for the live ROS graph. The nominal hypothesis is that the standing runtime supplies usable, temporally coherent inputs for a later controlled test. Accuracy of disparity, LiDAR coverage of the scene, thermal response, fusion benefit and predictive skill require additional reference data or interventions.

## Measurement contract

All metrics use a 30-second wall-clock collection after four seconds of discovery. The listener drains TF for two seconds after collection. Use the same machine, clocks, DDS settings, scene and subscriptions when comparing runs. Topic paths below are relative to `/robot_1`; the JSON records their absolute names.

| Metric and source | Calculation and units | Proposed screening criterion and limits |
| --- | --- | --- |
| Cadence, jitter, gaps on all received streams | Received intervals from monotonic wall arrivals and, when strictly increasing, ROS header stamps. Rate is interval count divided by elapsed seconds. Report interval min, median, nearest-rank p95, max and population standard deviation in ms. Count intervals above 1.5× the stream median. | Require at least two messages and positive increasing stamps for both RGB, both CameraInfo streams, right depth and Ouster. Review gap counts against a repeated baseline. A gap is a candidate missed delivery. Publisher-side loss counts require an independent tick schedule or sequence counter. |
| Stereo synchronization on `sensors/front_stereo/{left,right}/image_rect` | Exact intersection of unique header stamps, unmatched counts per side and nearest-right absolute stamp difference in ms for each unique left stamp. Nearest-right statistics may reuse a right stamp. | Require an exact pair. Review unmatched fraction and offset tails before freezing tighter limits. Stamps measure declared pairing and include subscriber losses and window edges. |
| Sensor age against `/clock` | Last received simulation clock minus each message's header stamp in ms. Retain signed values. | Require advancing nondecreasing clock and at least one age sample for each required stream. Review offsets and tails across runs. DDS callback ordering, clock cadence and differing stamp domains affect the result. One-way latency requires a publisher-side timing reference. |
| Timestamped TF | Query `map` to the actual header frame at up to 12 evenly strided interior stamps per left/right RGB, right depth and Ouster. | Require every sampled lookup. R1 covers the latest named frame checks. Sampling and deferred lookup can miss outages and do not measure transform delivery delay. |
| Right depth and disparity | Every tenth message from `sensors/front_stereo/right/depth_ground_truth` and `perception/stereo_image_proc/disparity`. At most 4,096 regularly spaced flattened pixels per frame. Report per-frame and aggregate finite positive fraction, with disparity additionally constrained by its advertised bounds. Fraction is dimensionless. | Require readable right depth and some positive finite depth. Disparity is diagnostic. The denominator is the full sampled image including borders, with no valid-window crop. Positive values and advertised limits do not establish pixel accuracy or axial/range semantics. |
| Ouster and stereo cloud | Every tenth PointCloud2, at most 4,096 regularly spaced points. Report declared width×height and sampled finite XYZ fraction per frame and their distributions. | Require readable Ouster and some finite XYZ. Review point-count and finite-fraction stability over repeated windows. An emitted point count cannot establish return completeness, beam coverage, or physical range accuracy. Stereo cloud is diagnostic. |
| Downstream disparity, stereo cloud, odometry and VDB markers | Endpoint count at end, received-message count, observed cadence and available payload diagnostics. Monitor `odometry_conversion/odometry` and `vdb_mapping/vdb_map_visualization` alongside stereo outputs. | Record unavailable or quiet outputs explicitly. A marker count establishes delivery only. Map completeness and pose accuracy require a verified independent reference. |

A **pass** covers the required transport, clock, sampled TF and payload checks above. A **fail** identifies an unmet required check. **Inconclusive** indicates the collector hit its 4,096-record limit on any stream or clock. Treat an unknown runtime configuration, failed R1 prerequisite, interrupted observation or changed scene as inconclusive for baseline comparison. Exit codes are 0, 1 and 2 for the probe's three outcomes; a timeout or process error is an incomplete run. Subscribers can activate existing lazy publishers and add load. To differentiate payload behavior from transport artifacts, compare wall and stamp cadence, inspect publisher identities and repeat under the same subscriber load. Diagnose outliers before attributing them to stereo, rendering, scene texture, clipping or the mapper. Freeze acceptance thresholds independently before scoring a controlled experiment.

## Reproduce on an already-running R1 workspace

Use the R1 pinned deployment and the standing, disarmed `robot_1` in domain 1. Confirm R1 readiness and disarmed state using its documented read-only regression procedure. Run from the AirStack checkout root. Output is local and ignored under `notebook/`; preserve it with the source revision and image IDs. The checker uses NumPy and ROS packages already installed in the robot image.

```bash
set -e
probe=ore-ms/ore-ms-research/isaac-readiness/nominal-perception/measure.py
out="notebook/nominal-perception/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$out"
git rev-parse HEAD > "$out/source.txt"
git status --short >> "$out/source.txt"
git submodule status --recursive >> "$out/source.txt"
sha256sum "$probe" common/fastdds.xml >> "$out/source.txt"
docker inspect --format '{{.Name}} {{.Config.Image}} {{.Image}}' \
  airstack-robot-desktop-1 isaac-sim-livestream airstack-gcs-1 > "$out/images.txt"
timeout 60 ./airstack.sh ready --json > "$out/readiness.json"
timeout 45 docker exec -i airstack-robot-desktop-1 bash -c '
  source /root/AirStack/robot/ros_ws/install/setup.bash
  export ROS_DOMAIN_ID=1 FASTRTPS_DEFAULT_PROFILES_FILE=/root/AirStack/robot/ros_ws/src/fastdds.xml
  python3 - --seconds 30 --source-commit "$1"
' bash "$(git rev-parse HEAD)" < "$probe" > "$out/measurement.json"
```

The [measurement schema](measurement.schema.json) describes the JSON contract. ROS message traffic stays within the running workspace. The tool retains no images or point clouds on disk.

## Bounded verification on 26 November 2026

Readiness passed all five gates in 7 seconds. The 30-second measurement passed with all 48 sampled TF lookups available and 657 exact stereo pairs, with 11 left and 20 right unique stamps unmatched. Median required-stream age was approximately 30 ms versus the most recent received simulation clock. Left/right RGB received rates were 22.28/22.55 Hz wall time and 32.08/32.47 Hz simulation time. Ouster received at 7.07 Hz wall time and 10.18 Hz simulation time. Its sampled XYZ values were all finite, and declared points varied from 12,151 to 12,479. [Readiness evidence](evidence/readiness-2026-11-26.json) · [Measurement evidence](evidence/stationary-2026-11-26.json)

Right depth had a positive finite fraction of 46.325% in every sampled frame. Disparity published 575 messages, but none of the sampled pixels met the positive, finite, in-range definition. The stereo cloud published 404 messages with median sampled finite XYZ fraction zero and maximum 0.25%. These sparse downstream outputs merit investigation before using them for geometric scoring. The current window cannot attribute the sparsity to the scene, matcher, framing or calibration. The observer also received 619 estimator odometry messages and 59 VDB visualization markers. Odometry did not meet strict positive monotonic stamp criteria for simulation cadence. Publisher counts are DDS graph endpoints and can include router relays.

Runtime provenance matches the robot, Isaac and GCS image IDs in R1. The observation used the R1 checkout `bf24a8a2ad7ce2cdbed492e10a70bff1e6ab75c6` and probe SHA-256 `7e0fa43fd827ee388a1e0f39b450e14ccf256008e764a7d12b339b840b43ab5c`. The probe implementation was in the working tree during collection. Five unit tests exercised rates, gaps, timestamp failures, unique pairing, endian/stride handling and truncated cloud rejection. Schema validation against the recorded JSON, Python syntax, and research documentation checks passed. The framework is ready for a separately authorized repeated stationary capture. A geometry pilot, physical M12 validation and performance scoring remain outside this observation.
