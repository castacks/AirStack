# Does signed velocity correction warn of trajectory drift?

The signed velocity correction rule missed the two-second warning window
on SmokeRoom despite complete score coverage. FlashLight also missed its
warning window and had insufficient coverage. This leaves the tested
rule unsuitable for triggering a handoff on these runs.

## Logging and score

The [SmokeRoom onset analysis](smokeroom-vins-fixed-replay-2026-11-19.md#first-drift-onset)
found that trajectory error grew mainly along the direction of travel.
The previous [correction magnitude check](vins-visual-correction-health-2026-11-19.md)
could lose a directional signal by taking the norm at each frame.
This test preserves the sign until aggregating past updates.

Replay SmokeRoom continuous RGB, FlashLight continuous RGB, and
FlashLight with the existing scheduled RGB track reset. Reuse the
VINS-Fusion serial logging runner and append the predicted horizontal
velocity and all three components of corrected minus predicted velocity.
Preflight verified the SHA-256 of all 3,597 distinct input frames across
the two image manifests and confirmed IMU support at every image.
The largest forward IMU brackets were 5.01 ms on SmokeRoom and
8.78 ms on FlashLight. Exact comparisons verify all 4,899 saved poses,
all 4,947 status rows, and all 4,896 prior correction norm rows.

Project the correction onto the current predicted horizontal direction
of travel when predicted horizontal speed is at least 0.15 m/s.
Take the absolute value of the signed median over the trailing 250 ms,
requiring at least four available components and support at the current
frame. Alert on the third consecutive supported frame above the
99th percentile in the previous clear interval, [250,255) s on
SmokeRoom and [85,90) s on FlashLight. Use the same FlashLight
threshold for the reset arm. These are changes in the estimator's own
state. The external reference defines the offline error onset and
does not enter the score.

## Result

| Replay and frozen decision interval | Scored frames | Alerts in interval |
| --- | ---: | --- |
| SmokeRoom, two seconds before the 263.80 s sustained error onset | 49 / 49 | None |
| FlashLight reset, from intervention at 90 s to the 90.80 s error onset | 6 / 19 | None |
| FlashLight continuous RGB, 90–100 s | 4 / 245 | None |

SmokeRoom had an alert at 256.77 s, about seven seconds before the
error onset, and none in the two seconds after onset. The clear
calibration intervals supplied 123/123 scores on SmokeRoom and
53/123 on FlashLight, below the frozen 80% support requirement for
FlashLight. The continuous FlashLight arm was quiet with only four
valid scores, so that interval offers little evidence about false
alerts. In the reset arm, support rose to 232/245 over 90–100 s and
the signal still emitted no alerts.

The test used exposed recordings and one frozen threshold rule.
The independent audit replicated all 4,896 score opportunities,
thresholds and alert times. Protocol, capture patch, input hashes,
replays, score ledgers and auditor are under
`/root/ore-ms-artifacts/smokeroom-recovery-intake/vins-build/longitudinal/`.
