# IRS RTVI Fast Dark thermal update-rate comparison

## Prior work and scope

Safa varies the resolution and number of consecutive thermal frames
used by a CNN for scalar rotational-speed estimation. Mohsen and Safa
add gyroscope fusion, use local pixel averaging for spatial subsampling,
and compare error and model complexity across input sizes. Both use
recordings made at 8 fps. Changing the number of stacked frames in
those experiments changes context at a fixed acquisition rate.
[Rotational Odometry, Sections 2–4](https://arxiv.org/html/2411.01227v1),
[thermal–gyro fusion, Sections II–IV](https://arxiv.org/html/2506.12536v1).

The narrower local experiment is about the error and cost of fewer
thermal updates in a stateful estimator with metric translation and
all IMU samples retained. Measure this first on an uninterrupted dark
recording where native thermal gave a clear benefit. A later study of
camera handoff, spatial resampling, or jointly reducing both budgets
would require a separate comparison. The targeted reading provides an
overlap check and does not establish publication novelty.

## Source and recording recovery

Restore the author RRxIO commit
`ae29afe0d7876f3838056b457292e4aecf9656be` and its recursive dependency
pins. The pulled ROS Melodic base image digest matches the historical
base image. Reacquire the Medium and Fast Dark bags and their companion
Vicon–IMU CSVs by bounded ranges from the public ZIP. All four members
pass CRC. The two bags reproduce the SHA-256 values recorded in the
historical reports. The released visible and thermal calibration
files are staged separately. The historical patched handoff loader,
raw pose outputs, and scorer packet are still absent. These restored
sources permit reconstruction of native controls.
[Author release](https://christopherdoer.github.io/datasets/irs_rtvi_datasets_iros2021),
[Fast Dark baseline](irs-rtvi-fast-dark-common-support-2026-11-18.md),
[Medium baseline](irs-rtvi-bright-medium-metric-2026-11-18.md).

## Cadence preflight

Read the thermal and IMU headers directly from both bags. Retain
every thermal frame up to the first reference timestamp plus 10 s,
apply the released −4 ms camera shift, and take every other thermal
frame afterward, beginning with the first eligible frame. Preserve
native 640 × 512 mono8 data and all IMU samples. These are input
headers and counts. Publication cadence and CPU cost require replay.

| Recording | Native / half-rate frames after prefix | Half-rate median / maximum input gap | Vicon–IMU support using 100 ms / 150 ms bracket |
| --- | ---: | ---: | ---: |
| Fast Dark | 1,231 / 616 | 100.00 / 116.68 ms | 776 / 1,223 of 1,223 |
| Medium | 1,241 / 621 | 100.00 / 100.23 ms | 771 / 1,238 of 1,238 |

The historical 100 ms interpolation cap excludes about two fifths
of the nominal reduced-rate grid, even with all selected frames
present. Keep the historical cap in the native baseline reproduction.
For the rate comparison, use the planned retained-camera timestamps
as the common scoring population. Interpolate the reference at those
times with the unchanged 100 ms cap. This supports 612 of 616 selected
Fast Dark frames and 621 of 621 Medium frames before replay. Actual
corrected-pose coverage must still be measured at those timestamps.
A separate 150 ms sensitivity analysis on the Vicon grid can show how
resampling affects trajectory summaries. Neither establishes runtime
delivery, which requires receiver-side timing.

## Replay protocol

Qualify the restored native Fast Dark controls on common Vicon epochs
with a unit-scale yaw and translation fit on the first 10 s of common
support. Require at least 98% common coverage, thermal RMSE no greater
than 0.35 m and visual RMSE of at least 2 m. The historical
common-support thermal and visual RMSE values were 0.282 m and 4.302 m.
[Historical replay](irs-rtvi-fast-dark-common-support-2026-11-18.md).

Compare the two rates on bags containing only the thermal and IMU
topics. A readback verifies the serialized payloads and bag record
times. Require that original-bag and canonical full-rate thermal poses
match exactly and that the full and half-rate poses match before the
cutoff. Apply the same full-rate 10 s alignment to both trajectories.
Score on the retained camera timestamps supported by the reference.
A corrected pose must lie within 1 μs of the planned timestamp.
The half-rate run passes if both arms cover at least 98% of the 612
planned reference-supported times, the half-rate RMSE stays at or below
both 0.35 m and 120% of the full-rate error, and the largest gap between
successive post-cutoff corrected poses is at most 150 ms.

The first subscriber-based collection lost different numbers of startup
poses across runs, while every timestamp shared by the two full-rate
captures had identical position and orientation. The replacement
recorder writes the estimator's odometry message immediately before
ROS publication. The frozen scoring scripts and thresholds apply
unchanged to the replay with direct capture.

## Fast Dark result

The native controls qualified on 1,418 of 1,420 Vicon epochs (99.9%).
After the early fit, thermal position RMSE was 0.282 m and visual
position RMSE was 4.302 m over the same 1,222 scoring epochs. The
canonical full-rate thermal capture was byte-identical to the
original-bag thermal capture. The two rates shared 505 identical
pre-cutoff poses, including orientation.

| Thermal cadence after prefix | Input frames after prefix | Corrected poses matched | Position RMSE | Largest corrected gap | Loader wall / user / system time |
| --- | ---: | ---: | ---: | ---: | ---: |
| Native, approximately 20 Hz | 1,231 | 611 / 612 | 0.282 m | 66.68 ms | 4.292 / 3.801 / 0.351 s |
| Every other frame, approximately 10 Hz | 616 | 611 / 612 | 0.171 m | 116.68 ms | 3.745 / 3.255 / 0.302 s |

Both runs used all 35,560 IMU messages. The half-rate arm passed every
frozen threshold. It lowered position RMSE by about 40% on the 611
common scoring timestamps, while adding 50 ms to the largest corrected
pose gap. The single sequential replay per arm showed a 13% reduction
in loader wall time, including bag I/O and setup. Receiver latency and
on-device compute were outside this measurement.

Approximately 10 Hz is a viable operating point for this Fast Dark
replay with the native pixel resolution and a full-rate initialization
prefix. A [separate Medium replay](irs-thermal-cadence-medium-transfer-2026-11-19.md)
checks transfer on the same rig. The test leaves the minimum feasible
cadence and behavior during smoke and camera transitions open.
A [fixed 2 × 2 pooling test](irs-thermal-spatial-pooling-fast-dark-2026-11-20.md)
missed the Fast Dark accuracy gate at this cadence.

The restored bags, range receipts, source pins, and run logs are under
`/root/ore-ms-artifacts/irs-budget-intake/`. The `native/` directory
contains the frozen protocols and scorers, direct-capture patch and
runner, pose CSVs, per-timestamp errors, and independent scoring audit.
