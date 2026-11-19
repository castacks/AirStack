# IRS RTVI IMU-only output for rejected visible frames

The [measurement-admission control](irs-rtvi-flagged-visible-update-ablation-2026-11-18.md)
withheld three dimmed visible updates and improved Difficult recovery,
while increasing the onset output gap to 196 ms. Reuse its two bags,
sensor timelines, detector, handoff, and accuracy protocol. At each
withheld image stamp, generate an output-only pose using the native
RRxIO merged IMU prediction and covariance propagation applied to an
isolated snapshot. Start with the safe filter state available when the
prediction is emitted. Use only IMU messages already received, with
emission gated on the first available IMU bracket. Publish snapshots
on a separate topic. No prediction payload feeds back into the filter.

| Recording | Corrected pose rows exactly reproduced | Largest onset gap, corrected only | Largest onset gap, combined | RMSE on 3 newly supported reference poses | First 2 s after light returns |
| --- | ---: | ---: | ---: | ---: | ---: |
| Medium | 1,744 | 195.50 ms | 50.01 ms | 0.021 m | 0.181 m |
| Difficult | 1,737 | 195.50 ms | 50.09 ms | 0.040 m | 0.177 m |

The combined gaps are measured after ordering the two pose topics by
sensor timestamp. This is an offline timestamp coverage result.
It does not qualify live latency or delivery order on a single stream.
The six predictions propagate from safe states roughly 100, 150 and
200 ms earlier. The supporting IMU brackets are 0.62–1.99 ms past
the requested stamps. Position covariance is exported, symmetric
and finite. Six poses do not qualify covariance calibration.

The corrected position and quaternion output matches the withholding
control at every timestamp. Both runs reproduce trigger and thermal
switch times and withhold the same three visible updates. All 200
dimming reference positions are now supported, compared with 197
under withholding alone. The combined dimming RMSE is 0.104 m on
Medium and 0.098 m on Difficult. Recovery RMSE is unchanged and both
remain above 0.15 m in the first two seconds after light returns.
Thus the bridge addresses short-term pose availability, while the
same accuracy limitation persists. An independent calculation
reproduces all scoring windows and the newly supported scores.

The first capture failed to open the prediction CSV. A later replay
crashed because copying the whole filter state also copied feature
managers' owning pointers. Both attempts are preserved. The corrected
version initializes a separate scratch filter state and copies only
the propagation state, covariance, time and mode required by prediction.
The protocol, loader diff, binary, raw streams, and hashes are under
`/root/ore-ms-artifacts/irs-author-rescore/bridge/`.
