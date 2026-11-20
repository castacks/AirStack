# IRS RTVI thermal startup at 10 Hz

Starting thermal odometry at approximately 10 Hz passed on Medium
and failed the position accuracy gate on Fast Dark. Both recordings
passed the coverage, first-pose and maximum-gap gates. In Fast Dark,
preparing the estimator at full rate before dropping to 10 Hz
reduced post-cutoff RMSE from 0.255 m to 0.171 m.

## Protocol

Use the same rig, RRxIO binary, direct pose capture, camera calibration,
full-resolution thermal images and IMU messages as the qualified
[Fast Dark](irs-thermal-input-budget-intake-2026-11-19.md) and
[Medium](irs-thermal-cadence-medium-transfer-2026-11-19.md) warm-start
controls. The warm-start arms retain all images through the pre-reference
interval and first 10 s of Vicon reference. The startup arms extend the
same fixed every-other-frame phase back to the first camera image.
Verify that the post-cutoff thermal payloads and record times match
exactly and all IMU messages are identical between arms. The startup
bags contain 869 Fast Dark and 875 Medium thermal images, with 35,560
and 35,792 IMU samples respectively.

Before replay, freeze the selected timestamps, reference support and
decision gates. Associate corrected outputs to selected thermal headers
within 1 μs. Interpolate Vicon positions with a 100 ms bracket cap.
Fit separate unit-scale yaw and translation for each estimator using
the same shared pre-cutoff timestamps. Score both on shared post-cutoff
timestamps. This early fit uses selected camera timestamps for both arms
because the startup trajectory already runs at 10 Hz.

The startup arm must cover at least 98% of reference-supported samples
in both intervals, stay at or below 0.35 m RMSE and 120% of the warm-start
RMSE, produce a first corrected timestamp within 150 ms of its first
thermal input, and keep every internal corrected timestamp gap within
150 ms. These timing checks use sensor and pose headers from offline
replay.

## Results

| Recording | Shared early / post-cutoff samples | Warm-start / 10 Hz startup RMSE | First corrected pose after input, warm / startup | Maximum corrected gap, warm / startup | Startup gate |
| --- | ---: | ---: | ---: | ---: | --- |
| Fast Dark | 98 / 98 and 611 / 612 | 0.171 / 0.255 m | 50 / 100 ms | 116.68 / 116.68 ms | Fails position accuracy |
| Medium | 100 / 100 and 620 / 621 | 0.168 / 0.149 m | 50 / 100 ms | 100.23 / 100.31 ms | Passes |

Fast Dark startup increased RMSE by 49% despite meeting the same
post-cutoff coverage and gap checks. Its allowed RMSE was 0.205 m.
Medium startup reduced RMSE by 11%. The different results support
retaining the tested full-rate initialization for Fast Dark. The
tested cutoff is 10 s after first Vicon reference. The
[five-second prefix test](irs-thermal-cadence-five-second-startup-2026-11-20.md)
compares that cutoff with a shorter initialization measured from the
first camera image. The recordings differ in both
motion and lighting, so they cannot identify what drives the
difference. Receiver latency and live sensor handoff are untested.

Both results and their common timestamp counts, RMSE, first-pose onset
and largest gaps were independently recomputed. Frozen protocols,
input and output hashes, pose CSVs, logs, per-timestamp errors and
the auditor are under `/root/ore-ms-artifacts/irs-budget-intake/startup/`.
