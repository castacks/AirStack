# Track-confirmed return to visible after IRS dimming

A causal visible-track return preserved output cadence but increased
early recovery error on both recordings. Both switched controls
reproduced the expected initial handoff, so the comparison isolates
the tested return decisions within the reconstructed replay.

## What this tests

Previous IRS work tested a [brightness-triggered switch and hold](irs-rtvi-causal-dimming-2026-11-18.md),
[scheduled immediate return](irs-rtvi-medium-dimming-2026-11-18.md),
and [a fixed two-second return](irs-rtvi-medium-delayed-return-2026-11-18.md).
The [track-survival scout](track-survival-warning-scout-2026-11-20.md)
reviews VINS-Mono per-feature age and prior use of track age in stereo
odometry. This experiment uses persistent tracks in the inactive
visible stream as a return signal. Thermal keeps updating the same
body estimate while visible quality is monitored independently.

On Medium and Difficult, keep the earlier simulated dimming intervals
and the causal trigger of three consecutive visible frames with at
least 95% of pixels at or below 16. Apply the triggering image and
then switch to thermal. In return arms a visible frame is bright when
at least 20% of its pixels exceed 32. Compare staying on thermal,
returning at the first eligible bright frame at least two seconds
after bright detection, and returning after at least eight original
Shi–Tomasi tracks persist across five bright frames. Forward/backward
Lucas–Kanade uses a one-pixel round-trip check. Loss of brightness or
insufficient persistent support resets the monitor. The executed
return arms perform one return.

The single-slot RRxIO reconstruction inherits the
[monocular handoff](irs-rtvi-single-slot-handoff-2026-11-17.md)
design. It replaces the active camera model and extrinsic prior,
clears camera features and their covariance, and carries body state,
velocity and IMU bias. All IMU messages are passed. Camera admission
follows bag arrival order with a monotonic corrected timestamp check.
One out-of-order candidate is rejected at initial handoff in each
switched arm. All shared prior poses are byte-identical. At each
return, the last thermal frame admitted in hold has a corrected stamp
about 4.5 ms before the return stamp but arrived after the return
decision, so that frame is absent in the return replay. A stricter
initial check for identical timestamp sets up to the return flagged
this boundary difference. The final audit explicitly counts it and
retains the original qualification and accuracy criteria.

The reconstructed native visible traces match the original native
binary byte for byte on both recordings. The hold replays qualify
against the historical Vicon–IMU RMSEs within 0.03 m in all windows
and retain full reference support. The three arms have identical
initial switch timestamps and fit one common unit-scale pre-dimming
alignment, based on 199 Medium and 200 Difficult reference positions.

## Results

| Recording | Return rule | Dimming RMSE | First 2 s after light returns | 2–10 s after light returns | Maximum output gap |
| --- | --- | ---: | ---: | ---: | ---: |
| Medium | Hold thermal | 0.105 m | 0.183 m | 0.135 m | 50.11 ms |
| Medium | Fixed delay after bright detection | 0.105 m | 0.183 m | 0.154 m | 54.50 ms |
| Medium | Track-confirmed visible return | 0.105 m | 0.218 m | 0.173 m | 54.50 ms |
| Difficult | Hold thermal | 0.113 m | 0.254 m | 0.324 m | 50.22 ms |
| Difficult | Fixed delay after bright detection | 0.113 m | 0.254 m | 0.272 m | 54.50 ms |
| Difficult | Track-confirmed visible return | 0.113 m | 0.294 m | 0.314 m | 54.43 ms |

The track rule returned 232.46 ms after light returned on Medium
with 11 surviving tracks, and 229.60 ms after light returned on
Difficult with 15. The fixed-delay arm returned at 2.032 s and 2.080 s.
All arms score 200 dimming, 40 early recovery and 160 Medium or 159
Difficult late recovery positions with the 100 ms bracket cap.

The track-confirmed rule fails the frozen recovery test on both
recordings. The separate monitor established visible feature
persistence in its own tracker. The estimator still needed to seed
new visible tracks when it returned. The study leaves the value of
retaining a visible map or testing agreement with the current body
estimate unresolved. It also supplies no independent natural
transition result or wall-clock latency claim. No threshold sweep
was performed.

The audit reconstructs all 18 shared-grid RMSE values and verifies
common handoff timing. Source overlays, logs, captured poses, scoring
ledger and hashes are under
`/root/ore-ms-artifacts/irs-dual-preflight/recovery-mode/`.
