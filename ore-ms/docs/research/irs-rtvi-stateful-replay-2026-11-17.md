# IRS RTVI stateful replay and visual input outage

The released RRxIO monocular estimator ran over three existing IRS RTVI
recordings. Each recording used visual+IMU and thermal+IMU in separate runs.
Radar and external velocity were disabled. The loader used 15 features, the
authors' filter presets and camera YAMLs, and the published image time shifts
of +2 ms for visual and −4 ms for thermal. Ground truth was available only to
the scorer. The [earlier camera-pair results](irs-rtvi-nominal-camera-pose-2026-11-17.md)
used independent pairs with unit translation direction. These runs retain the
estimator state and emit metric IMU positions.

## Uninterrupted recorded-input controls

The table reports position RMSE against the released Vicon–IMU CSV after
fitting one translation and gravity-axis yaw per run over all associated
outputs. Reference position was interpolated only between samples at most
100 ms apart. There was no scale fit. Counts show received odometry and the
subset eligible for scoring. Tracked features use the native RRxIO feature
status. The reference retains the dataset's physical timing and calibration
limitations.

| Recording | Camera | Odometry rows | Scored rows | Median tracked | Position RMSE |
|---|---|---:|---:|---:|---:|
| Easy | Visual | 1,897 | 1,589 | 11 | 0.127 m |
| Easy | Thermal | 1,876 | 1,589 | 11 | 0.140 m |
| Dark | Visual | 2,705 | 2,396 | 3 | 5.402 m |
| Dark | Thermal | 2,936 | 2,396 | 11 | 0.213 m |
| Fast dark | Visual | 1,704 | 1,418 | 2 | 2.127 m |
| Fast dark | Thermal | 1,701 | 1,417 | 11 | 0.125 m |

All six processes exited successfully, initialized once and published finite
odometry. Median received output spacing was 50 ms in all six arms. Dark
visual published through most of its 2,707 recorded images even with large
position error. The shorter visual trace reflects the shorter camera stream.
Received output and healthy tracking are distinct signals. The recording
counts are small, and the per-arm reference supports differ slightly. The
whole-trajectory fit is a descriptive benchmark. The offline replay timings
include startup and file processing and supply no wall-clock recovery or
deployment-latency measurement.

## Two seconds of same-camera input withholding

The easy visual replay withheld 40 messages in one prespecified two-second
window and retained all 38,902 IMU messages. The input audit compares
serialized payloads and recorded timestamps. The 797 matching odometry
rows before the outage were identical across the two executions. The
uninterrupted run yielded two additional pre-outage subscriber rows.

The largest gap across the outage was 2.050 s in sensor time, versus 0.050 s
in the uninterrupted replay. The first restored image produced an odometry
row and a tracked feature. The five-consecutive-report criterion was
confirmed after 0.200 s of sensor time, with the same confirmation delay in
the uninterrupted control. The loader advances its published safe state on
image updates, so the absence of published odometry during withholding does
not diagnose state loss or quantify IMU-only drift.

A separate translation and yaw fit on the 10 seconds before withholding
was frozen for each replay. On 41 reference-associated outputs in the first
two seconds after return, position RMSE was 0.058 m after withholding and
0.041 m in the uninterrupted control. This is one easy-recording observation
of same-camera reacquisition. No cross-camera handoff was executed.

## Reproduction and next boundary

The native loader came from [RRxIO](https://github.com/christopherdoer/rrxio)
at `ae29afe0d7876f3838056b457292e4aecf9656be` in an Ubuntu Bionic ROS Melodic
container. Build fixes declared the existing radar dependency on YAML,
provided missing development packages and disabled unrelated yaml-cpp
tools/tests that could not link after the package's library rename. The
RRxIO estimator source and estimation settings were unchanged. The only
filter preset edit disabled image drawing. A first scoring command for the
outage omitted the reference argument. The recorded replay was retained and
the scorer was rerun with that argument.

The source pins, build changes, frozen protocols, CSVs, logs, and hashes
are under `/root/ore-ms-artifacts/irs-rtvi-intake/stateful/`. The 596 MiB
derived outage bag lives in `/osmo/run/workspace/irs-rrxio/fixture/` and
is reproducible from the retained public easy bag. No new bags were
downloaded and no Box space was used.

The default offline loader feeds only one camera. Its multi-camera image
callback groups images under a shared timestamp and waits for all configured
camera slots. IRS has separate camera timestamps and per-camera timing
shifts. A camera-switch study needs an explicit policy for independent
observations, propagation during loss and a shared body state, with
fixed-camera controls and error after return. A real-time run is needed to
measure wall-clock recovery.
