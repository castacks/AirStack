# ViViD++ IMU metric acceleration and stationary noise check

The [RRxIO transfer](vivid-stateful-transfer-2026-11-17.md) and
[independent OpenVINS control](vivid-openvins-bright-control-2026-11-18.md)
both missed the bright trajectory gate. This check uses only the raw
external IMU and the nominal marker-to-IMU reference. It tests the
translational part of the inertial binding directly.

Use the already bound IMU-clock reference timestamps, +Y reference up,
and fixed gravity 9.81 m/s². At 100 ms-spaced anchors, compare the
reference IMU position's centered second difference to the raw specific
force rotated into the reference world and reduced by gravity. Use
identical triangular acceleration smoothing, with 150 ms half-width as
primary and 300 ms as a fixed sensitivity check. This follows from
the centered second-difference response to a time-varying acceleration.
Every supported bracket uses the existing transform and timestamp
convention. No time, orientation, gravity, scale or bias correction was
estimated from these comparisons.

The [release audit](vivid-calibration-materials-2026-11-18.md) subsequently located the supplied Microstrain
noise YAML. It preserves this experiment’s original EuRoC proxy and reports
a separate bright control with the published values.

## Metric and angular agreement

| Recording | Supported 150 ms checks | Mocap acceleration RMS | IMU–mocap vector RMSE | 300 ms vector RMSE |
|---|---:|---:|---:|---:|
| Bright | 522 / 522 | 0.196 m/s² | 1.586 m/s² | 1.574 m/s² |
| Varying flashlight | 354 / 354 | 0.223 m/s² | 1.985 m/s² | 1.974 m/s² |
| Dark | 347 / 347 | 0.247 m/s² | 2.499 m/s² | 2.490 m/s² |

All runs miss the 0.5 m/s² diagnostic bound and the minimum of 20
translation-exciting anchors. Using the larger half-window leaves the
same conclusion. An independent count checks the position and
acceleration columns. Removing the mean term algebraically from the
bright residual variance still leaves 1.365 m/s² RMS, so a constant
world-frame offset alone cannot explain the discrepancy. This
calculation is a diagnostic and supplies no estimator correction.

On separate 100 ms relative rotations, the raw gyro agrees with the
nominal reference much more closely. For intervals with at least 1°
of nominal rotation, p95 discrepancy is 1.22° bright, 1.31° varying
and 1.14° dark. Relative orientation agreement can coexist with
uncertain absolute tilt, low translational excitation, or errors in
the position-derived reference. This does not identify the source of
inertial disagreement or re-estimate the RGB-to-IMU clock shift.

## Short stationary windows

The frozen one-second gate uses low reference speed and acceleration,
low gyro norm, and sample support. Bright has three qualifying adjacent
seconds starting at 1 s, with 300 samples. Varying has two adjacent
seconds starting at 33 s, with 199 samples. Dark has none. The recorded
IMU covariance diagonals are zero in all three logs.

| Recording | Short-time accelerometer proxy across axes | Short-time gyro proxy across axes | Median apparent body acceleration offset norm |
|---|---:|---:|---:|
| Bright | 0.00117–0.00134 m/s²/√Hz | 0.000196–0.000272 rad/s/√Hz | 1.76 m/s² |
| Varying | 0.000725–0.00168 m/s²/√Hz | 0.000222–0.000346 rad/s/√Hz | 1.76 m/s² |

The proxies use scaled first differences within each accepted window.
The OpenVINS EuRoC proxy was 0.002 m/s²/√Hz and 0.000170 rad/s/√Hz.
The apparent acceleration offset is the difference between measured
specific force and nominal body-frame gravity in static windows.
It includes residual motion and reference attitude or gravity
uncertainty. The norm must not be treated as a calibrated sensor bias.

The short observations do not qualify a white-noise model or long-time
bias random walk. The local calibration archive contains no raw
stationary IMU recording. A dedicated stationary recording from the
same rig would support the Allan-deviation procedure documented by
[Kalibr](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) and
[OpenVINS](https://docs.openvins.com/gs-calibration.html#imu-noise-calibration-offline).
A separate translation-rich recording with a checked absolute
orientation and sensor clock is needed to isolate the metric residual.
No new estimator was launched.

The input hashes, protocols, synthetic gravity and rotation tests, full
ledgers, and independent audit are under
`/root/ore-ms-artifacts/vivid-plusplus-intake/imu-metric-audit/`.
No new bags or Box storage were needed.
