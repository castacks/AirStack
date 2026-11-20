# IRS RTVI Fast Dark five-second thermal startup

Five seconds at 20 Hz before dropping to 10 Hz failed the Fast Dark
position accuracy gate. The first corrected pose and subsequent output
coverage met the timing and coverage limits. The **25.25 s** full-rate
prefix from the first image remains the shortest passing duration among
the three tested configurations.

## Comparison

Use the Fast Dark recording, RRxIO binary, calibration, 640 × 512
thermal images, 35,560 IMU messages, and direct odometry capture from
the [10 Hz startup study](irs-thermal-cadence-startup-2026-11-19.md).
Measure the intervention from the first shifted thermal camera header.
Keep all 101 thermal images with elapsed time below 5.0 s, then retain
the same images as the existing 10 Hz startup arm. The first kept
frame after the transition is at 5.10 s. Bag readback verified that
all IMU messages and the 818 post-transition image payloads and
record timestamps match the existing 10 Hz startup arm. The candidate
keeps 919 thermal frames in total, compared with 869 for the 10 Hz
startup and 1,122 for the longer warm start.

The original warm-start transition is first Vicon reference plus
10 s, or 25.25 s from the first thermal image. Vicon reference begins
15.25 s after the first image, so the five-second transition precedes
the reference and preserves the planned accuracy scoring population.
Preflight matched all 98 early and 612 post-cutoff reference-supported
camera timestamps from the previous study. Use the unchanged startup
scorer and independent auditor and fit a separate yaw and translation
for each run on the same shared early samples. The frozen gates
require 98% shared coverage in each interval, RMSE no greater than
0.35 m and 120% of the warm-start error, a first corrected pose
within 150 ms, and a maximum corrected pose gap of 150 ms.

## Results

| Time at 20 Hz from first image | Position RMSE on 611 common timestamps | First corrected pose after first input | Maximum corrected pose gap | Gate outcome |
| --- | ---: | ---: | ---: | --- |
| 0 s | 0.255 m | 100.01 ms | 116.68 ms | Fails position accuracy |
| 5.0 s | 0.292 m | 50.01 ms | 116.68 ms | Fails position accuracy |
| 25.25 s | 0.171 m | 50.01 ms | 116.68 ms | Passing reference arm |

The five-second candidate and the longer prefix shared all 98 early
points and 611 of 612 post-cutoff points. The new and previous
comparisons used the same 611 scoring timestamps and reproduced
identical per-timestamp warm-start errors. The candidate's RMSE
exceeded its 0.205 m accuracy limit and was also higher than starting
at 10 Hz. It cut the time to the first corrected pose by 50 ms relative
to starting at 10 Hz, based on sensor and pose headers. The first
corrected pose exactly matched the long-prefix arm.

These results show that a prompt corrected pose can coexist with
worse later trajectory accuracy. The three runs leave the minimum
passing initialization duration and any relationship between duration
and accuracy unresolved. The offline comparison provides no receiver
latency or cross-recording estimate.

The protocol, readback hashes, staged input hashes, replay logs,
pose CSVs, timestamp ledger and scorer audit are under
`/root/ore-ms-artifacts/irs-budget-intake/startup-five/`.
