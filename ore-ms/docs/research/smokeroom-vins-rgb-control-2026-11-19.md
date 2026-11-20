# SmokeRoom clear-window VINS-Fusion control

Use the [clear moving window](smokeroom-stateful-recovery-intake-2026-11-19.md#clear-moving-rgb-control)
as a gate before any stateful obscuration replay. VINS-Fusion supports
monocular camera–IMU estimation. The pinned tracker calls CamOdoCal's
native MEI unprojection and converts the resulting rays to normalized
image coordinates. This keeps the released RGB camera model without an
image warp or a fitted focal length.
[Estimator paper](https://arxiv.org/abs/1901.03638v1),
[pinned code](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/be55a937a57436548ddfb1bd324bc1e9a9e828e0).

The released intrinsics contain the native 640 × 480 MEI model and
the extrinsics supply the RGB camera-to-IMU matrix. The control leaves
both fixed. An independent numeric check follows the source's
eight-iteration undistortion and projective lift for all 307,200 pixel
centers. The projective Z component stays above 0.1069, and
redistortion gives a maximum pixel round-trip error below 0.001 px.
This tests mathematical compatibility. Physical timing and calibration
accuracy remain outside the check.
[Released intrinsics](https://drive.google.com/file/d/1Q0JiqiIgGZ-7DZKZDNJ68F7rpC2rrLtT/view),
[released extrinsics](https://drive.google.com/file/d/1HjWlRVQQvgrFGlgRxcczRt92Xy_P-5Ij/view).

## Frozen control

The RGB input includes all 737 frames from 225 to 255 s, acquired in
bounded ranges and verified by ZIP CRC and exact header association.
The published sensor-rate IMU CSV supplies 6,111 samples including
boundary padding. It was previously bound to raw SmokeRoom messages.
This run uses nominal headers with zero added clock shift. The released
camera calibration files contain no IMU noise parameters, so the fixed
VINS-Fusion EuRoC preset supplies proxy noise assumptions. No clock or
spatial calibration is estimated.
[Prior sensor binding](smoke-reference-pose-findings-2026-11-16.md).

Use 225–245 s for startup, 245–250 s for unit-scale yaw and translation
alignment, and 250–255 s for scoring. Require at least 90% coverage in
each of the last two windows, with position RMSE at most 0.30 m in the
scoring window. Match predicted outputs to camera headers within 1 µs
and interpolate the supplied reference only across gaps of at most
20 ms. The estimator consumes RGB and IMU only. The supplied trajectory
is used by the scorer, with its previously documented dependence on
visual, LiDAR, map, and inertial inputs.

The serial runner gives VINS-Fusion an IMU sample beyond each image
timestamp before submitting that image. Solver settings follow the
upstream monocular preset. Build and replay artifacts are outside Git
under `/root/ore-ms-artifacts/smokeroom-recovery-intake/vins-build/`.

## Result

The control passes. The estimator consumes all 737 images, initializes
once, and publishes 727 corrected body poses. The first output is at
225.408248 s. All 123 alignment frames and all 123 scoring frames have
matched pose and reference support. Unit-scale position RMSE on the
held-out 250–255 s scoring window is 0.0430 m. Estimated and supplied
endpoint displacements there are 2.738 and 2.728 m. An independent
decoder checks all 737 RGB payloads, and an independent scorer reproduces
the frame associations and position RMSE.

The first build failed on legacy OpenCV constants. The compatibility
patch updates those names and the targeted build uses `-O1` in ROS Noetic
with OpenCV 4.2. The first replay wrapper allocated the estimator with
automatic storage and faulted in its constructor before consuming
frames. The corrected wrapper uses static storage, as the upstream ROS
node does. The rerun kept the input, calibration, noise, solver settings,
and metric protocol fixed and exited cleanly. The source patch, startup
failure and backtrace, input/output hashes, and build logs are retained.

This qualifies the fixed RGB configuration against the supplied
reference on the clear control. It does not validate the thermal camera,
a camera switch, or an independently measured recovery trajectory.
