# IRS RTVI scheduled handoff during sustained dimming

The [earlier continuous-frame test](irs-rtvi-photometric-transition-2026-11-18.md)
dimmed the easy recording for two seconds. Here the
[medium-motion bright replay](irs-rtvi-bright-medium-metric-2026-11-18.md)
supplies a separate 10-second intervention. Freeze the shifted image-header
interval `[1614701678, 1614701688)` and the first two seconds after it.
Divide visible mono8 pixel values by 16 in the intervention. All 200
visible frames still arrive, with an input gap of at most 50.11 ms.
Preflight verifies monotonic sensor and switched input stamps and all
35,792 IMU messages. The affected frames' native p95 values span
217–254. After dimming they span 13–15.

The uninterrupted visual and dimmed visual arms use the same visual
preset. The externally scheduled arm uses that preset, substitutes the
200 thermal images during dimming, and returns to visual immediately.
At each transition, a callback in the image update replaces the active
camera model and extrinsic prior, discards the outgoing feature tracks
and clears their covariance and the old extrinsic covariances. It
carries forward the body state and its covariance. Each camera retains
its released time shift. The existing fixed thermal run is a control
with the native thermal preset and thermal history from the start.

## Metric position against Vicon–IMU

Fit yaw and translation separately over the same 199 reference
timestamps in the 10 seconds before dimming, with scale fixed.
Interpolate only within supported brackets of at most 100 ms. All 200
intervention reference timestamps and all 40 return timestamps are
available in every arm. Gaps use emitted sensor timestamps.

| Arm | RMSE during dimming | RMSE in first 2 s after return | Largest output gap |
| --- | ---: | ---: | ---: |
| Visual, original frames | 0.080 m | 0.096 m | 50.11 ms |
| Visual, dimmed frames | 0.657 m | 0.462 m | 50.11 ms |
| Scheduled visual → thermal → visual | 0.124 m | 0.251 m | 54.51 ms |
| Fixed thermal | 0.102 m | 0.119 m | 50.11 ms |

Handoff reduces RMSE by 0.533 m during dimming and 0.210 m after return
relative to dimmed visual. It passes the frozen 0.15 m intervention
limit and 0.15 s gap limit, and misses the 0.15 m return limit.
Fixed thermal has the lowest error of the three arms exposed to the
dimming scenario, including after return. The scheduled substitution
uses advance knowledge of the intervention. This offline response
characterizes one artificial dimming interval, with no automatic
detection, wall-clock latency, or natural-transition score.

## Parity and evidence

The rebuilt uninterrupted visual control matches all 1,746 native
visible poses by timestamp and position. Before intervention, all 800
handoff positions match the rebuilt control exactly. Dimmed visual
matches all 793 shared pre-intervention positions and omits the first
seven output timestamps, before Vicon–IMU reference coverage. This
passes the predeclared 99% parity cutoff and leaves the scoring cohorts
intact. All three new arms initialize once and finish cleanly. The
independent score calculation reproduces all eight RMSEs within 1e-10 m.

The isolated loader patch, fixed windows, input preflight, raw pose
CSVs, hashes and score audit are stored under
`/root/ore-ms-artifacts/irs-author-rescore/medium/dimming/`.
