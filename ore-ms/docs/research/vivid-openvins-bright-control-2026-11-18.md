# ViViD++ bright OpenVINS control

The first [continuous ViViD++ RRxIO replay](vivid-stateful-transfer-2026-11-17.md)
finished the bright bag with 0.718 m nominal position RMSE. Its prespecified
transfer bound was 0.30 m. This independent control uses the native ROS 1
serial monocular estimator from [OpenVINS v2.7](https://github.com/rpng/open_vins/tree/v2.7)
at commit `93adc241390d13e99232652cf05cbe18a93c7bea`. OpenVINS uses
feature tracks and an EKF with a window of cloned poses. The upstream
[tutorial](https://docs.openvins.com/gs-tutorial.html) and
[paper](https://yangyulin.net/papers/2020_icra_ov.pdf) describe its calibration
inputs and estimator.

The [release audit](vivid-calibration-materials-2026-11-18.md) subsequently located the supplied Microstrain
noise YAML. It preserves this experiment’s original EuRoC proxy and reports
a separate bright control with the published values.

## Frozen protocol

Replay the full staged `global_robust` recording, using only `/rgb/image`
and the external `/imu/data`. Start from the OpenVINS EuRoC preset and
configure one camera, a 31 Hz input ceiling, measured 100 Hz IMU rate,
single-threaded image processing, and the native dynamic initialization
option alongside static initialization. Use the published handheld
intrinsics, camera-to-IMU transform and +11.837 ms camera clock shift.
Keep the camera calibration fixed and use EuRoC inertial noise values as
a proxy for the unpublished ViViD++ noise parameters. Read no reference
inside the estimator.

Keep the previous scorer, including the fixed reference axis conversion,
100 ms interpolation bracket, translation and yaw alignment, unit scale,
and 0.30 m nominal position RMSE / 90% overlapping-frame coverage bounds.
Count missing startup frames against coverage. Run the other fixed RGB
controls only after passing the bright gate.

## Result

| Bright monocular control | Reference coverage | Nominal position RMSE | Process exit |
|---|---:|---:|---:|
| RRxIO prior replay | 99.87% | 0.718 m | 0 |
| OpenVINS, native shutdown | 89.68% | 171.17 m | 134 |
| OpenVINS, ordered shutdown | 89.88% | 322.96 m | 0 |

The ordered-shutdown replay initialized once and exported 1,385 poses for
1,541 eligible camera frames. Output began 5.89 s into the recording.
The rate ceiling permits 1,540 of 1,541 input images, and each exported
timestamp agrees with a calibrated image header within 5 microseconds.
The independent endpoint audit shows the clean estimate's endpoints
are 1,127 m apart while the nominal reference endpoints are 0.253 m apart
at the same times. A rigid coordinate change leaves this discrepancy
unchanged.

The first invocation failed to parse the released Kalibr YAML because
OpenCV 3 FileStorage requires sequence indentation inside the mapping
entry. Reindentation preserved every non-whitespace token. The next run
produced poses before an exception during ROS plugin teardown. Releasing
the visualizer and estimator before ROS shutdown resolved the exception.
Estimator settings and update code were held fixed. Dynamic initialization
converged on slightly different camera frames across executions, with
two initialization solver timeouts in the native-shutdown run and none in
the clean run. Both trajectories diverged. The experiment does not isolate
which sensor, initialization, or noise assumption caused the divergence.

The clean OpenVINS run misses both frozen bounds. The conditional varying
and dark RGB runs were not needed for this decision. The result closes
this fixed-calibration configuration as the required independent bright
baseline. The next useful dependency is a separate validation of the
camera–IMU binding and noise model under metric motion, before trying a
third estimator or a camera switch.

The source pin, build logs, frozen configs, teardown patch, failed
attempts, direct state exports, scorer and independent chord audit are
in `/root/ore-ms-artifacts/vivid-plusplus-intake/openvins/`. The original
bags remain in place. No dataset downloads, derived bags, or Box writes
were needed.
