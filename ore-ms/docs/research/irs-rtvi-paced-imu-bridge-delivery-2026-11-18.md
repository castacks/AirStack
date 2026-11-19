# IRS RTVI IMU-only output delivery at recorded speed

The [IMU-only bridge](irs-rtvi-imu-only-output-bridge-2026-11-18.md)
restored sensor timestamp coverage but published two distinct topics.
Replay the first 62 seconds of Medium and Difficult sequentially at
1× bag record cadence. Keep the source timestamps, detector, filter
and both publishers unchanged. Record subscriber receipt times on
Linux's monotonic clock in a single process. The nominal dimming
onset maps the frozen sensor stamp into that clock using the replay
origin. Measure receipt gaps intersecting onset through one second
after onset. The transport is local with no added load.

| Recording | Corrected-topic receipt gap | Both topics, raw receipt gap | Both topics, after stale-pose screen | Time to first IMU-only receipt |
| --- | ---: | ---: | ---: | ---: |
| Medium | 148.40 ms | 98.17 ms | 146.58 ms | 42.57 ms |
| Difficult | 147.08 ms | 95.66 ms | 143.75 ms | 40.49 ms |

Each raw combined replay has one sensor timestamp regression. The
last corrected pose before onset arrives after all three predicted
poses and carries a stamp about 150 ms older than the immediately
preceding receipt. Counting that stale message as fresh explains the
apparent reduction to 96–98 ms. A retrospective receiver-side screen
drops any message with a sensor stamp at or below the highest already
received. It discards exactly one late corrected message per recording,
maintains increasing sensor timestamps, and reduces the measured
gap by only 1.82 and 3.33 ms compared with corrected-only receipt.
All corrected-only and screened gaps meet 150 ms in these specific
runs. The limited margin and one replay per bag leave delivery
reliability unqualified. The stale screen was applied to the saved
receipt ledger and was not installed as a publisher or listener.

Both runs reproduce the unpaced trigger and thermal switch at exact
sensor times. Every corrected row through the filter's final emitted
stamp matches the unpaced capture exactly (1,235 Medium and 1,234
Difficult). The three prediction records also match for each bag,
including the covariance entries. Position scoring reproduces the
unpaced result through all three declared epochs. The stale screen
retains all reference rows, changes dimming RMSE by less than 1 μm,
and leaves the early recovery scores at 0.181 and 0.177 m. Both still
exceed 0.15 m. Separate calculations verify receipt gaps and regression
counts from the raw receipt sequence.

The frozen protocol, two pose captures, receipt ledger, pacing patch,
binary and checksums are under
`/root/ore-ms-artifacts/irs-author-rescore/paced-bridge/`.
