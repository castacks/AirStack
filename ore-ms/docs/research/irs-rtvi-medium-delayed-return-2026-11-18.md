# IRS RTVI return to visible two seconds after dimming

Extend the [medium-motion thermal hold replay](irs-rtvi-medium-thermal-hold-2026-11-18.md)
with one fixed schedule. Switch to thermal at the start of the same
10-second artificial dimming interval, keep thermal active for two
seconds after the light returns, then switch back to visible. Use the
same single-slot handoff, camera shifts, visual filter preset and
medium bag. The delayed boundary was fixed at shifted sensor stamp
1614701690 s before capture. The first processed visible frame after
that boundary has timestamp 1614701690.032474041 s.

Fit yaw and translation on the same 199 common Vicon–IMU positions in
the ten seconds before dimming and keep unit scale. Interpolate
within a 100 ms bracket and score every reference timestamp in the
declared windows. The 2–4 s window begins at the scheduled return,
32 ms before the first visible update. It overlaps the 2–10 s window.

| Arm | During dimming (200) | 0–2 s after light returns (40) | 2–4 s (40) | 2–10 s (160) |
| --- | ---: | ---: | ---: | ---: |
| Immediate return to visible | 0.124 m | 0.251 m | 0.167 m | 0.210 m |
| Return to visible after 2 s | 0.124 m | 0.230 m | 0.152 m | 0.206 m |
| Keep thermal active | 0.124 m | 0.230 m | 0.167 m | 0.143 m |
| Fixed thermal from start | 0.102 m | 0.119 m | 0.134 m | 0.215 m |

The delayed schedule falls short of the 0.150 m RMSE bound by 2.17 mm
in its primary post-switch window. Its 2–10 s RMSE improves by only
3.65 mm over immediate return, while keeping thermal active is 63.29 mm
lower than delayed return. The first two seconds after the light
returns remain above the bound for all three initial handoff schedules.
The delayed arm's largest output gap around the delayed boundary is
54.50 ms in sensor time, within the 0.15 s limit. This result is
specific to the fixed schedule and artificial dimming in one replay.

The delayed and continued-thermal arms match position exactly at all
1,031 shared output times before the scheduled switch. The delayed
capture lacks nine initial output rows, all before Vicon–IMU coverage,
and meets the frozen 99% parity gate. Both scoring and alignment use
full common support. An independent calculation reproduced all 16
RMSEs. The delayed run initialized once and exited cleanly.

The frozen protocol, loader diff and binary, source preflight, raw pose
CSV, shared timestamp ledger, score, and hashes are under
`/root/ore-ms-artifacts/irs-author-rescore/medium/dimming/delayed/`.
