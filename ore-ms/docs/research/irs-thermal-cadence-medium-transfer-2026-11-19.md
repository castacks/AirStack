# IRS RTVI Medium thermal update-rate transfer

On IRS Medium, reducing thermal updates from about 20 Hz to 10 Hz
after full-rate initialization lowered metric position RMSE by 33%
while lengthening the largest gap between corrected poses by 50 ms.
This repeats the direction of the [Fast Dark result](irs-thermal-input-budget-intake-2026-11-19.md)
on a second recording from the same rig.

## Control and protocol

Use the restored public `mocap_medium` recording, released Vicon–IMU
reference and the same RRxIO binary, calibration, 15-feature presets,
camera shifts and direct odometry capture as Fast Dark. Before replay,
verify bag SHA-256, native 640 × 512 mono8 thermal format, 508 prefix
frames and 621 planned retained timestamps with reference support.
Qualify the native controls at 98% shared reference coverage and
within 0.030 m of each [historical Medium RMSE](irs-rtvi-bright-medium-metric-2026-11-18.md).
The restored controls qualified on 1,438 of 1,441 Vicon epochs, with
0.089 m RGB and 0.248 m thermal RMSE on 1,237 scoring epochs after
the common early fit.

Use the same cadence intervention and scoring as Fast Dark. Keep
every thermal frame before first reference plus 10 s, then keep every
other frame starting with the first eligible frame. Read back and
verify both canonical bags, including all 35,792 IMU messages. Use
the full-rate fit on 201 early reference positions to align both
runs, with fixed scale. Associate directly captured corrected poses
within 1 μs of planned retained camera times and interpolate only
the scoring reference with the same 100 ms bracket cap. Require
98% common support, half-rate RMSE at most 0.35 m and 120% of
full-rate RMSE, and an internal post-cutoff corrected gap at most
150 ms. The Medium script variants change data paths, expected
input and scoring counts, and native-control qualification. The
Fast Dark cadence thresholds and scripts are preserved.

## Results

The original-bag and canonical full-rate thermal outputs were
byte-identical. Full and half rate shared 507 identical pre-cutoff
poses, including orientation. Both runs matched the same 620 of
621 planned reference-supported times (99.8%). The half-rate
configuration passed all thresholds. An independent scorer
reproduced the RMSEs, association counts and maximum gaps.

| Medium thermal cadence | Frames after prefix | Position RMSE | Largest corrected gap | Loader wall / user / system time |
| --- | ---: | ---: | ---: | ---: |
| Native, about 20 Hz | 1,241 | 0.248 m | 50.24 ms | 4.061 / 3.553 / 0.350 s |
| Every other frame, about 10 Hz | 621 | 0.168 m | 100.23 ms | 3.216 / 2.836 / 0.294 s |

The one serial replay per arm showed a 21% reduction in loader
wall time, including bag I/O and setup. Fast Dark showed a 40%
RMSE reduction and a 50 ms increase in maximum corrected gap on
its own 611 common scoring times. The two recordings support
10 Hz thermal after full-rate initialization for this estimator
and rig. They differ in lighting and motion, so the experiment
cannot isolate what drives the reduction in position error.
The [10 Hz startup comparison](irs-thermal-cadence-startup-2026-11-19.md)
checks whether full-rate initialization is needed. Minimum cadence,
online delivery, smoke, and sensor transitions remain open.

The frozen protocol, input hashes, native and rate logs, pose CSVs,
per-timestamp errors and independent scoring audit are under
`/root/ore-ms-artifacts/irs-budget-intake/medium/`.
