# CAVERS headlight transition intake

22 November 2026. Screen `loc_handheld_1` for a recorded change in
visibility with paired RGB and thermal images, IMU, and independent pose.
Choose it from the paper before opening the data. The authors describe
roughly one second of abrupt changes from motion of the rig-mounted LED,
loss of visual-only tracking without recovery, and full trajectories from
the visual–inertial and thermal–visual–inertial pipelines. The event is
physical headlight motion inside a cave. Its timing and track behavior
remain to be verified on the released images. [Paper, Sections II and III](https://arxiv.org/html/2604.15052)

The paper calls the Optris PI 640i near-IR. The manufacturer's
specification for that model gives an 8–14 µm LWIR band, and the pinned
exporter saves the thermal topic as 16-bit PNG.
[Manufacturer specification](https://optris.com/products/thermal-cameras/precision-line/pi-640i/) ·
[Pinned exporter](https://github.com/spaceuma/cavers/blob/706fe2bc5e05367f0a3a094b8945212a8ec2805e/export_logs.py)

From the 1.085 GB extracted-data ZIP, recover the central directory and
CRC-check eight timestamp, pose, transform and intrinsics sidecars using
bounded requests. The camera, IMU and pose timestamp arrays are strictly
increasing. Exported timestamps come from message headers. All 7,915
finite OptiTrack-derived poses report `map → base_link` and match the
dynamic TF rows exactly. Static TF connects `base_link` to the OptiTrack
markers, RGB and IMU optical frames, and thermal camera frames. The
paper describes CAD-derived rig extrinsics, camera calibration and an
independent OptiTrack system. The TF comparison verifies internal
consistency. It does not measure physical extrinsic error.
[Versioned dataset](https://zenodo.org/records/19367714)

| Stream | Samples | Median timestamp interval | Bracketed by pose | Pose bracket ≤25 ms |
| --- | ---: | ---: | ---: | ---: |
| RGB | 1,021 | 66.71 ms | 1,014 | 993 |
| Thermal | 503 | 131.00 ms | 502 | 493 |
| IMU | 13,584 | 4.99 ms | | |
| OptiTrack pose | 7,915 | 8.33 ms | | |

IMU brackets of at most 5.01 ms cover 1,015 RGB and 502 thermal
timestamps. The median nearest thermal timestamp is 33.93 ms from RGB.
The largest thermal interframe gap is 454 ms. The paper reports a
common logging clock with no hardware synchronization. Timestamp
proximity does not establish exposure alignment.

## Full-record image preflight

Recover and CRC-check all 1,021 released RGB and 503 thermal PNG members
using 55 bounded ZIP reads. Before seeing pixels, fix a scan of 1 s
intervals centered on RGB timestamps. Rank the 945 eligible intervals
by the drop in median RGB track retention relative to the previous 2 s.
Report the following 2 s on the same rule. Retention is the fraction
of up to 300 Shi–Tomasi points in the previous frame with successful
forward–backward Lucas–Kanade tracking within 1 pixel. Decode thermal
at native 16-bit depth and flag adjacent bit-identical decoded images.

The strongest drop occurs at **13.776–14.776 s**, measured from the
first RGB frame. Before, during and after the interval, median RGB track
retention is 93.2%, 14.0% and 66.7%. Median frame brightness is 59.5,
5.0 and 64.0 on the 8-bit grayscale scale. All 15 RGB and eight thermal
event frames meet the 25 ms pose and 5.01 ms IMU bracket bounds. Before
the event, one RGB and one thermal frame share a 25.098 ms pose bracket.
The complete 5 s interval therefore fails the pose preflight at those
two arrivals. Retain the interval and the original bound.

A separate diagnostic near the recording midpoint at
**33.390–34.390 s** agrees in timing and duration with the paper's
description. Median RGB track retention falls from 82.3% to 7.4%
and returns to 69.5%. Median brightness changes from 161 to 4 to
101.5. Every RGB and thermal event in its full 5 s context (75 RGB,
36 thermal) meets the pose and IMU bracket bounds. It contains no
adjacent identical thermal images. This interval was identified
after the sweep, so it is a development candidate. The evidence does
not establish that it is the authors' exact unlogged headlight event.

There are 74 adjacent identical thermal images across the full
recording. Image-space track retention measures degradation and partial
return. The pose reference and sensor coverage support a separately
declared replay with asynchronous camera events. No estimator ran and
trajectory recovery remains unmeasured. Keep the characterization
scoped to recorded active illumination. Protocol, range and hash
receipts, frame and window tables, and audit scripts are under
`/root/ore-ms-artifacts/cavers-intake/`.

## Midpoint replay inputs

Stage the five-second development context from 31.390 to 36.390 s.
The packet holds 75 RGB images, 36 thermal images, 1,004 IMU samples
and 587 reference poses, including the IMU and pose rows bracketing
the clip boundaries. The manifest preserves integer header timestamps
and per-image hashes. Every image has a pose bracket at most 25 ms
and an IMU bracket at most 5.01 ms. Compose the released TF edges
into both camera-to-IMU and IMU-to-camera matrices. Normalize the
RGB factory quaternion, whose norm differs from one by 1.7e-8.

Read an 8 MiB prefix of the native bag archive and CRC-check its
metadata sidecar. The first native headers agree with the image,
IMU and pose exports. They identify RGB as `rgb8` in
`camera_color_optical_frame`, thermal as `mono16` in
`camera_link_FLOATthermal`, IMU as `camera_imu_optical_frame`, and
pose as `map → base_link`. On 580 approximately 50 ms pose
increments, the median discrepancy between the OptiTrack rotation rate
and the gyro is 0.108 rad/s after applying the released rotation,
versus 0.689 rad/s with unrotated gyro. The rotated p95 residual is
0.531 rad/s. This corroborates the frame convention without fitting a
clock shift or claiming exposure alignment.

Write and read back three ROS 1 bags for RGB+IMU, thermal+IMU and
dual-camera+IMU. Each uses the same serialized IMU measurements in the
same order. The dual-camera bag carries the same image messages as
the corresponding single-camera bags. Images retain the native
encodings and the original asynchronous header timestamps. Bag record
times follow header order for deterministic offline replay. The original
ROS 2 publication delays are outside this reconstruction. Reference
poses stay outside the estimator inputs.

The CAVERS paper uses ROVTIO for its dual-camera benchmark. The pinned
public ROS 1 source accepts independent camera arrivals and preserves
16-bit image values in its float conversion. Its published example
uses another rig's calibration and a 120 ms wall-clock camera inactivity
setting, shorter than CAVERS' 131 ms median thermal interval. Before
scoring, qualify a reproducible build, validate startup on an earlier
prefix and verify how the offline loader schedules both camera queues,
including the last pending frames. The staged five seconds fix the
scoring context and do not establish sufficient initialization time.
Packet and bag receipts are under
`/root/ore-ms-artifacts/cavers-intake/midpoint-replay/`.
[Pinned ROVTIO source](https://github.com/ntnu-arl/rovtio/tree/5338eccd69c17642d7481f5e962c775e19da3ceb)

## ROVTIO build and startup smoke

Build one- and two-camera ROVTIO at `5338eccd` in a ROS Noetic image
pinned by digest, with kindr `c83d3d60` and lightweight_filtering
`0c8517a5`. Local compatibility patches cover OpenCV 4 color enums,
system yaml-cpp, Clang template syntax and the one-camera indexing
guard. Compose the CAVERS calibration into the estimator convention
with fixed extrinsics and native intrinsics. Use the upstream example
noise and feature settings as smoke-test inputs. Disable the viewer
for headless execution. The first GUI-enabled attempt initialized
and stopped at the display error.

An additional packet extends the estimator input by a fixed 10 s
before the five-second context. It carries 225 RGB, 109 thermal and
3,008 IMU messages in the dual arm. All image arrivals have IMU
support. The strict pose bracket applies within the scoring context.
The initial warmup runs exceeded the 70 s wall-time limit and did
not yield closed output bags.

Run the original loader scheduling on the five-second packet as
a transport smoke test. Read back finite, increasing odometry stamp
rows at 73 of 75 RGB epochs, 34 of 36 thermal epochs and 96 of 111
dual-camera epochs. In the dual run the loader reported 13 safe-time
drops, and wall-clock inactivity marked a camera inactive during
offline processing. Finalization exposed a callback-queue teardown
exception after the bags had been closed. Keep those smoke results
as failure evidence.

The offline loader variant uses the already verified bag header order,
allows a nonempty camera queue to advance without waiting for the
other camera, and disables waits for the external pose and velocity
topics excluded from the replay. Move the image callback queue before
its subscribers so it outlives them during destruction. Record the
first emitted safe state. These changes eliminate the observed
safe-time drops and recover 110 of 111 dual-camera epochs. The
remaining missing epoch is the final RGB image. The filter used a
strict upper bound when searching for the next update, even when the
bound was the timestamp of that final image.

Use an inclusive bound when selecting an update. All three re-runs
exit cleanly and emit finite odometry at every expected image epoch
(75 RGB-only, 36 thermal-only and 111 dual-camera). All 73 RGB-only
poses shared with the original smoke are numerically identical after
the scheduling and output corrections.

All fixed-extrinsic startup runs reported a prediction Jacobian
discrepancy in a camera translation state and a pixel-output
finite-difference discrepancy. The source test already flags the
fixed-extrinsic case as a known issue. A separate empty-bag check
temporarily enabling extrinsic calibration passes both tests in the
dual-camera configuration. This is diagnostic evidence about the
test state. The replay keeps fixed extrinsics and retains the
failed checks. No trajectory accuracy or recovery claims follow
from the transport smoke.

The build recipes, exact dependency versions, source patches,
logs, and per-arm output audits are under
`/root/ore-ms-artifacts/cavers-intake/`. The completed checks
are for the five-second transport packet. The 15-second inputs
are staged for initialization and control qualification.

## Fixed-extrinsic diagnostic and warmup replay

23 November 2026. Repeat the prediction finite difference at the
original step and tolerance, restricting the comparison to active
states while separately checking the influence of the frozen columns.
Across the RGB-only, thermal-only and dual builds, the maximum active
error is `8.73e-8`, against the `1e-6` bound. The frozen translation
and rotation columns exert zero influence on active rows in both the
analytic and numerical derivatives. With calibration held fixed, the
accessors return auxiliary calibration constants, which explains why
perturbing the unused translation state gives a different result from
the full-matrix test.

For each installed camera use 81 forward bearings and central
differences along both tangent directions. Compare the estimator's
analytic pixel Jacobian with a separate double-precision radial and
tangential projection. The maximum discrepancy is `9.32e-8` px/rad
for RGB and `6.27e-8` px/rad for thermal, against a `1e-5` bound.
Differencing the rounded float pixel outputs on the same grid and
step gives discrepancies up to 14.99 and 25.61 px/rad, respectively.
This demonstrates the effect of float rounding on the check. The
upstream synthetic startup messages are preserved. Neither the
filter equations nor the calibrated extrinsics changed for this
diagnostic.

The fixed 15 s replay uses ten seconds of warmup and the original
five-second scoring context. Each arm exits cleanly and publishes at
every expected camera epoch (225 RGB, 109 thermal, 334 dual). No
reference enters the estimator. Score `base_link` against OptiTrack
on a fixed 10 Hz grid, with one rigid alignment at the first planned
query. Use the declared 25 ms reference and 300 ms estimator bracket
limits. Of 50 planned queries, one dark query lacks reference support
and one return query lacks thermal-estimate interpolation support.
All figures below use the same 48 supported queries.

| Phase | Common queries | RGB+IMU RMSE (m / °) | Thermal+IMU RMSE (m / °) | Dual+IMU RMSE (m / °) |
| --- | ---: | ---: | ---: | ---: |
| Before | 20 | 3.726 / 1.257 | 5.557 / 1.782 | 5.557 / 1.798 |
| Dark | 9 | 9.104 / 0.913 | 13.123 / 1.477 | 13.121 / 1.424 |
| Return | 19 | 17.199 / 2.823 | 22.939 / 1.574 | 22.938 / 1.579 |

An independent SciPy implementation reproduces 146 supported
arm-query error rows to within `2.49e-14` m and `4.66e-12` degrees.
Pose publication is continuous on each arm's native camera schedule.
The maximum output gaps in the dark interval are approximately
67 ms for RGB and dual and 176 ms for thermal. These rows measure
published poses, without certifying feature-update acceptance.

**The translation control fails before darkness.** During the
scoring context OptiTrack moves 0.132 m from first to last pose, with
maximum displacement from the start of 0.229 m. The estimates move
23.53 m (RGB), 29.39 m (thermal) and 29.98 m (dual) over their
respective in-window endpoint pairs. The native example uses gravity
`g_z = -10.0854 m/s²`. The released IMU median specific-force norm is
9.7907 m/s² over 3,007 reference-covered samples, and the median
reference-rotated vertical component is 9.7849 m/s². The mismatch
identifies an initialization and configuration issue to qualify on a
fixed pre-event control. The diagnostic did not isolate the cause
of the drift or tune gravity to this scored interval.

Retain the replay as a failed translation baseline. Smaller
rotation errors for the thermal and dual arms during the return
show a scoped difference under this configuration. The joint
trajectory and recovery comparison remains unqualified. The exact
input protocol, source patches, bags, scores and independent audit
are under `/root/ore-ms-artifacts/cavers-intake/`.

## Pre-event gravity control

23 November 2026. Replay the same 10 s warmup and 2 s pre-event
interval with RGB+IMU. Change only `g_z` from the upstream example's
−10.0854 m/s² to the implementation constructor's −9.81 m/s².
Both runs read the same bag with 180 RGB and 2,406 IMU messages.
The last IMU sample brackets the cutoff and no RGB image reaches
the dark interval. Both runs emit all 180 expected outputs and
all 180 original-gravity messages match the earlier full replay
byte for byte.

| Gravity setting | Translation RMSE | Rotation RMSE | Endpoint displacement |
| --- | ---: | ---: | ---: |
| Upstream example | 3.726 m | 1.257° | 6.612 m |
| Constructor default | 0.316 m | 5.479° | 0.541 m |
| OptiTrack | | | 0.132 m |

All 20 predeclared queries support both arms. The default gravity
reduces translation RMSE by 91.5% and increases rotation RMSE.
Translation still exceeds the 0.15 m control bound, so retain the
failed baseline and leave the dark and recovery intervals unreplayed
under this configuration.

The first accelerometer sample has norm 9.371 m/s². With the released
IMU-to-rig rotation and interpolated OptiTrack attitude, the sample's
specific-force direction is 1.89° from vertical. The nearest reference
row reports 0.193 m/s speed, and the first half-second has a median
force-direction offset of 2.41°. The startup uses that single sample
to initialize attitude while the rig is moving. These observations
motivate a separate initialization control and do not establish the
cause of the remaining drift. Protocol and results are under
`/root/ore-ms-artifacts/cavers-intake/`.

## Pre-event initialization control

23 November 2026. Hold gravity at −9.81 m/s² and reuse the exact
RGB+IMU prefix and scoring rules from the gravity control. The
reference-assisted arm uses the last OptiTrack attitude before
the first IMU sample, 3.591 ms earlier, to express vertical in the
IMU frame. Map that direction to world +z using the same
shortest-rotation convention as the accelerometer startup. The
reference and accelerometer up directions are 1.93° apart. Initial
position and velocity remain zero. Reference yaw, position, velocity,
and subsequent OptiTrack updates are excluded from the estimator.

Both runs publish all 180 expected camera epochs. The accelerometer
control reproduces all 180 archived default-gravity messages byte for
byte. All 20 predeclared scoring queries support both runs. Independent
SciPy scoring reproduced all 40 arm-query rows.

| Initialization | Translation RMSE | Rotation RMSE | Endpoint displacement |
| --- | ---: | ---: | ---: |
| First accelerometer sample | 0.316 m | 5.479° | 0.541 m |
| OptiTrack-derived tilt | 0.040 m | 1.427° | 0.174 m |
| OptiTrack | | | 0.132 m |

The tilt-only intervention passes the 0.15 m translation bound on
the pre-event control. It establishes sensitivity to startup attitude
under the fixed CAVERS configuration. The result depends on external
tilt at startup. Qualify a sensor-only initialization on the same
pre-event control before interpreting a deployment recovery comparison.
No dark or return frames enter either arm. The frozen protocol, seed
receipt, source patch, output bags and score audit are under
`/root/ore-ms-artifacts/cavers-intake/`.

## Sensor-only tilt control

23 November 2026. Replace the single accelerometer reading with the
mean of the fixed 500 ms interval ending at the first replay IMU
timestamp. The 101 samples span 499.006 ms with a maximum interval of
4.991 ms. The seed uses no OptiTrack data and reaches the estimator
through the same startup quaternion hook as the preceding diagnostic.
Keep the −9.81 m/s² gravity setting and reuse the serialized RGB+IMU
prefix, scoring queries and 0.15 m translation bound.

| Initialization | Translation RMSE | Rotation RMSE | Endpoint displacement |
| --- | ---: | ---: | ---: |
| First accelerometer sample | 0.316 m | 5.479° | 0.541 m |
| Prior 500 ms accelerometer mean | 1.283 m | 1.951° | 2.233 m |
| OptiTrack | | | 0.132 m |

The mean-based arm emits all 180 camera outputs, including 30 in
the scoring window. Both arms support all 20 predeclared queries.
Independent SciPy scoring reproduces 40 arm-query rows. The
averaged direction is 1.60° from the earlier OptiTrack-derived up
direction, compared with 1.93° for the first sample, yet translation
degrades. The angular separation alone does not predict the
translation behavior of the initialized estimator.

Close this averaging variant for this prefix. The pre-event baseline
remains unqualified and there is no dark or return replay with this
seed. No alternate averaging window was selected. The frozen
protocol, source hash, seed and outputs are under
`/root/ore-ms-artifacts/cavers-intake/`.

## IMU-only stationary startup screen

23 November 2026. Before reading the lead-in, freeze a one-second
window with at least 190 samples, no gap above 10 ms, peak gyro
magnitude at most 0.02 rad/s, all accelerometer magnitudes within
0.15 m/s² of 9.81 m/s², and acceleration-vector RMS deviation at
most 0.05 m/s². Consider every trailing window ending before the
fixed pre-event scoring interval. Use the latest qualifying window
if one exists.

All 6,011 windows satisfy the sample-count and gap requirements.
Zero windows meet any of the three motion limits. The smallest peak
gyro magnitude over all windows is 0.153 rad/s. A separate rolling
scan reproduces all 6,011 gyro disqualifications. The screen used IMU
data only. There is no qualifying startup and no further estimator
replay under this route.

Close CAVERS `loc_handheld_1` as a sensor-only metric recovery
benchmark under the tested ROVTIO initialization contract. Retain
the measured RGB tracking degradation and partial return, the
reference-assisted initialization diagnostic, and the unsuccessful
sensor-only controls. Thermal recovery remains unmeasured on a
qualified baseline. Protocol, full scan and independent check are
under `/root/ore-ms-artifacts/cavers-intake/`.

## Matched tilt-assisted lead-in

23 November 2026. Apply the same one-time OptiTrack-derived tilt
and −9.81 m/s² gravity to RGB+IMU, thermal+IMU and dual-camera+IMU.
The three clipped bags use the same 2,406 IMU messages, with 180 RGB
and 87 thermal arrivals as applicable. The archived RGB result was
reused after matching its serialized input and configuration. No
RGB or thermal frame from the event or return enters any arm.

All expected camera epochs publish odometry. RGB and dual support
all 20 scheduled queries. The thermal-only trajectory does not
bracket the 1.95 s query at the end of the clear interval. On the
19 common queries, position RMSE is 0.040 m RGB, 12.000 m thermal
and 2.296 m dual. Rotation RMSE is 1.420°, 1.025° and 1.590°.
The independent scorer reproduces all 59 supported arm-query rows.
The 0.15 m translation bound is exceeded by both thermal-containing
arms even on the common cohort. The joint lead-in gate fails and
the event and return were not replayed. These results apply to the
frozen estimator configuration and provide no recovery comparison.
Inputs and audits are under `/root/ore-ms-artifacts/cavers-intake/assisted-pre-event/`.
