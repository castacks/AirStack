# M2P2 night reference feasibility

The candidate is Dataverse file 5496, chunk 0000 of `r2c2_night_1`. The prior
[bag index audit](rgb-thermal-deployment-data-2026-11-16.md#m2p2-outdoor-follow-on)
established RGB, thermal, point cloud and two IMU streams over about 30 seconds.
This assessment used source and prior index evidence. It did not read sensor
messages or generate a trajectory.

| Check | Evidence and decision |
|---|---|
| Camera-independent trajectory generation | M2P2 §V illustrates LIO-SAM on Lake Braddock using LiDAR, IMU and GPS when available. That demonstrates a candidate method; the night chunk's exact estimator configuration and trajectory remain unverified. |
| Point cloud contract | Upstream Ouster ingestion expects ring and relative point time; topic and message counts cannot verify either field, its units, or scan-time and IMU coverage. |
| Inertial input | Upstream LIO-SAM requires an external IMU with attitude output for its Ouster setup. The built-in six-axis Ouster IMU alone fails that contract. Inspect the external Yahboom messages for usable attitude, angular velocity, acceleration, timestamps and frame convention before choosing it. |
| Calibration and time | Released static transforms connect both IMUs, LiDAR and the RGB/thermal optical frames through the sensor tree. The bag patcher injects calibration metadata. The paper documents LiDAR-triggered camera synchronization. Verify chunk headers, extrinsic direction and sensor time bindings before using that tree to express LiDAR motion in camera coordinates. |
| Independence and validation | Keep RGB, thermal, event, and any image-derived odometry out of the estimator. Record any shared IMU if a camera baseline uses inertial input. A LiDAR-inertial estimate still needs independently justified error bounds and interval-level support before it can score pose accuracy. |

**Disposition:** feasible in principle; the reference gate remains open. The next
bounded check is serialized PointCloud2 and external-IMU schema and header
inspection on file 5496, including cloud ring/time, IMU attitude availability,
frame IDs and point-cloud/IMU time overlap. Use bounded bag reads before
considering the full 2.9 GB payload. Preserve unsupported intervals and verify
reference quality before running or scoring RGB/thermal baselines.

Sources: [M2P2 paper §V](https://arxiv.org/html/2410.01105v2#S5),
[M2P2 calibration and bag patcher](https://github.com/RobotiXX/M2P2/tree/f724e7ad0b14c4d5ebb3550ab459678ddb9744e9/dataset_processing_scripts/rosbag_patcher),
[LIO-SAM requirements](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md),
[Ouster point ingestion](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp).

## Bounded bag probe

Verified 761,833 bytes of HTTP range responses from
[Dataverse file 5496](https://dataverse.orc.gmu.edu/api/access/datafile/5496).
The ROS bag v2 footer contains 3,506 storage chunks and 30 connections. It
identifies `/sensor_suite/ouster/points` as PointCloud2 and distinguishes
`/sensor_suite/witmotion_imu/imu` (external) from
`/sensor_suite/ouster/imu` (embedded). Inspected storage chunk headers declare
LZ4 compression, so uncompressed message index offsets cannot be used directly
as HTTP byte ranges. Fetch and decompress the indexed storage chunks containing
the target messages.

Dataverse returned HTTP 500 during index-range requests, followed by connection
refusals on both the file and the API version endpoint. No target message body
was decoded. Point ring/time, IMU attitude and sensor header time overlap remain
unverified. The range cache, SHA-256 manifest and probe were saved locally in
`/root/ore-ms-artifacts/m2p2-5496-range-probe/`. A retry of the file and API version endpoint still returned connection refusals.
The cached footer nevertheless locates every target-bearing storage chunk:

| Stream | Messages | First and last storage chunk (zero-based) |
|---|---:|---:|
| Embedded Ouster IMU | 3,743 | 1–3505 |
| External WitMotion IMU | 3,002 | 4–3504 |
| Ouster point cloud | 300 | 5–3505 |
| Left / right RGB | 299 each | 7–3496 / 7–3497 |
| Thermal | 285 | 168–3497 |

Thermal first appears in a storage chunk starting 1.466 s after bag start, versus
0.059 s for RGB. The final LiDAR storage chunk follows the final external IMU
storage chunk. These are bag record-time observations. Sensor header offsets and
actual deskew support remain unknown. The footer parser and per-chunk byte
offsets are saved alongside the range cache in `footer-chunks.json`. Use that
map to fetch target-bearing chunks and nearby IMU chunks when the provider
recovers, including the first thermal chunk and the LiDAR/IMU tail.

The retry probe now uses the footer to select target-bearing chunks in four
windows covering the start, first thermal frame, middle and tail. Each window
writes its own result after decoding. An offline LZ4 fixture passed checks for
IMU quaternion, gyro and acceleration; cloud ring and point time with padded
rows; and separate record and header timestamps. This validates the fixture
decode path only. The first cloud chunk request still returned a connection
refusal, so the live reference gate remains open.

## Source recovered and sampled payload decoded

Dataverse resumed serving file 5496. The complete 2,918,515,263-byte bag
matches the Dataverse MD5 `d4289c2dbe56700a6f14c206e9c9fa8d`. The four-window
probe decoded 12 point clouds and 111 external IMU messages. All sampled
clouds expose scalar `t` (UINT32) and `ring` (UINT16). The first and middle
serialized points have `t` near 99 million, while the last point is near
1 million. The order matters for any deskew implementation that uses the
last point as the end time. Check the driver timestamp convention and the
estimator's handling of this ordering before running it.

The external IMU reports frame `yahboom_imu`, unit-norm orientation
quaternions and a nonnegative orientation covariance. At the tail, its last
header stamp is 98.712 ms after the final cloud header. If cloud `t` is an
offset in nanoseconds from the header, the largest sampled point offset in
that scan is 98.95 ms and exceeds that IMU coverage by about 0.24 ms. This
conditional comparison leaves clock semantics and full-scan coverage open.
The four decoded JSON files are in the local range-probe evidence directory.

## Full-bag deskew timing audit

The [Ouster point conversion source](https://github.com/ouster-lidar/ouster-ros/blob/6a7693c/src/os_ros.cpp#L196-L225)
computes point time as a nanosecond offset from scan start. Applying the published
convention to file 5496, the audit decoded all 300 clouds (39,321,600 point
slots, 27,807,516 nonzero-range returns), all 3,002 external and 3,743 embedded
IMU headers, and all camera headers. Each stream's headers increased strictly.
Every external IMU message contains a unit quaternion and available orientation
covariance. This checks availability, with physical attitude accuracy and
cross-device clock calibration still open.

| Timing check | Result |
|---|---|
| True end by maximum serialized point time | 99.355–100.565 ms after header (median 99.901 ms). Restricting to nonzero-range returns gives the same maximum in every scan. |
| End by last serialized point | 1.064–1.089 ms (median 1.076 ms). The last point is earlier than the maximum in all 300 scans. |
| External IMU brackets full scan by headers | 298/300. Zero-based scan 0 lacks start coverage and scan 299 lacks end coverage by 1.306 ms. Largest observed bracketing IMU gap was 10.291 ms. |
| Embedded IMU brackets full scan by headers | 299/300. Scan 0 lacks start coverage. |

The [pinned LIO-SAM Ouster path](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp#L214-L240)
sets scan end from the last serialized point. On this bag that understates every
scan duration by about 99 ms and would pass the final scan's end-coverage check
using the external IMU. The cloud headers say `os_sensor`, and recorded
`/tf_static` distinguishes `os_sensor` from `os_lidar` by a roughly 180°
rotation around Z and 38.195 mm translation along Z. Resolve the publisher's
coordinate convention before selecting the IMU and camera extrinsics.

The stock estimator is held at input qualification. A subsequent estimator
test needs a verified cloud coordinate frame, a scan-end policy based on the
maximum point timestamp and an explicit policy for the two unsupported
boundary scans. No estimator was run. The full per-scan ledger and audit script
are in the local range-probe evidence directory.

## Cloud coordinate frame

Use `os_sensor` for the stored XYZ. All 300 cloud headers use that frame.
The [released M2P2 launch](https://github.com/RobotiXX/M2P2/blob/f724e7ad0b14c4d5ebb3550ab459678ddb9744e9/sensor_suite/sensor_suite_launch/launch/sensor_suite_all.launch#L4-L9)
does not override the point cloud frame. The
[inspected Ouster ROS1 implementation](https://github.com/ouster-lidar/ouster-ros/blob/6a7693c551a34fc9f8dc47c8dda09cc64e2c57c0/src/os_transforms_broadcaster.h#L43-L45)
defaults to the sensor frame and uses that same selection to
[apply the LiDAR-to-sensor transform](https://github.com/ouster-lidar/ouster-ros/blob/6a7693c551a34fc9f8dc47c8dda09cc64e2c57c0/src/point_cloud_processor.h#L38-L45).
The inspected driver revision models the observed behavior. The bag does not
pin the deployed driver revision.

A payload consistency check compared XYZ radius to the encoded range on
352,598 returns between 2 and 25 m across scans 0, 75, 150, 225 and 299.
With the recorded 38.195 mm sensor-to-LiDAR origin correction, per-scan
95th-percentile absolute residuals were 0.75–0.98 mm, versus 11.64–13.30 mm
with the stored origin. This checks the origin and cannot distinguish a yaw
rotation on its own. The publisher contract and bag headers establish the axis
convention. The script and results are saved as `audit_cloud_frame.py` and
`cloud-frame-audit.json` in the local evidence directory.

Recorded static transforms give identity rotation between `yahboom_imu` and
`os_sensor`, with the external IMU origin at z = −87.5 mm in `os_sensor`.
The separate `os_lidar` transform would change both position and orientation.
Frame binding is qualified for the input contract. Stock LIO-SAM's scan-end
calculation and the two unsupported boundary scans remain open. No estimator
or RGB/thermal scoring was run.

## Deskew and boundary policy

A source review of the [pinned LIO-SAM projection node](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp)
and an offline replay of all 300 scan and external IMU headers established four
requirements for a bounded estimator test:

1. Compute scan end from the maximum point offset. Keep the external IMU
   samples immediately before or at scan start and at or after scan end to
   support interpolation. The stock integration window cuts off at end plus
   10 ms. For scan 297, the first end-bracketing IMU sample arrives 10.014 ms
   after the corrected scan end, so the stock window would omit it even though
   the bag contains it. Under bag record-order replay, every scan the stock
   buffer releases has its corrected end bracket available at release.
2. Express deskew in the cloud header time frame. Stock projection anchors
   to the first accepted serialized point. With 128 rings, 1024 columns and
   the stock 1–1000 m range gate, the first range/ring-admitted point is
   98.815 ms after the header at the median (0.400–100.161 ms across scans).
   Initialize the deskew transform by interpolating orientation at the
   header time explicitly.
3. Keep all 300 entries in the ledger. Mark zero-based scan 0 unsupported at
   the start and scan 299 unsupported at the end. Exclude both from estimator
   input and reference scoring. Do not extrapolate the external IMU or
   substitute the embedded IMU at either boundary. Scans 1–298 remain eligible
   for the timing preflight, including scan 297 with its actual end bracket.
4. Drain any buffered eligible clouds at end of input using recorded IMU
   samples. Stock LIO-SAM waits for two later cloud callbacks and leaves scans
   298 and 299 queued at EOF in unfiltered replay. Scan 298 has both brackets
   and should reach the estimator; scan 299 remains unsupported.

These checks establish an input/replay policy, with scan-to-IMU clock alignment
and trajectory accuracy still to be qualified. No estimator was run. The
local evidence directory contains `audit_lio_sam_deskew.py` and
`lio-sam-deskew-preflight.json`.

## Recorded-time gyro agreement

Compared both recorded IMUs in `os_sensor` using the identity rotations from
the bag's static transforms. No bias, axis or clock parameters were fitted.
Interpolating the embedded gyro at 3,002 external header times gave X/Y/Z
rate correlations of 0.682/0.583/0.920 and a vector rate difference of
6.20°/s median and 22.58°/s at p95.

For each predeclared LiDAR scan interval, integrated each gyro component
with trapezoids and interpolated at the header and maximum point time.
Retained all 300 rows, including the two unbracketed boundary scans. This
is an angular-rate integral diagnostic for approximately 100 ms intervals,
with no pose or physical timing ground truth.

| Cohort | Scans | Gyro integral vector difference, median / p95 / max |
|---|---:|---:|
| All paired scans | 298 | 0.284° / 0.755° / 0.971° |
| Embedded gyro integral magnitude ≥0.5° | 189 | 0.389° / 0.788° / 0.971° |

The 1° RGB–thermal endpoint is of similar scale to the upper tail of the IMU
disagreement. The data support a bounded estimator feasibility test with the
recorded timestamps. Fine pose scoring still requires a separate reference
error assessment. This diagnostic cannot attribute the disagreement to a
clock offset, gyro noise, filtering, or mounting differences. The local
evidence directory contains `audit_imu_agreement.py` and `imu-agreement.json`.
