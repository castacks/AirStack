# IRS RTVI keep thermal active after medium-motion dimming

The [scheduled return test](irs-rtvi-medium-dimming-2026-11-18.md) improved
accuracy during artificial dimming and missed the error limit after
returning to visual. This replay keeps the same initial visual-to-thermal
handoff and continues using thermal after the lighting returns. It tests
whether the immediate return explains the post-dimming error.

Use the same bag, 10-second intervention, source shifts, state handoff,
and visual filter preset. Rebuild and rerun the immediate-return arm for
parity. The existing fixed-thermal control uses the native thermal preset
and starts with thermal. For each arm fit yaw and translation on the
same 199 shared Vicon–IMU positions in the ten seconds before dimming.
Keep unit scale and score at common reference timestamps using the
existing 100 ms maximum interpolation bracket.

| Arm | During dimming (200 poses) | First 2 s after return (40) | 2–10 s after return (160) |
| --- | ---: | ---: | ---: |
| Switch back to visual immediately | 0.124 m | 0.251 m | 0.210 m |
| Keep thermal active | 0.124 m | 0.230 m | 0.143 m |
| Fixed thermal from the start | 0.102 m | 0.119 m | 0.215 m |

All values are 3D position RMSE. Continuing with thermal reduces the
early error by 0.022 m and still misses the 0.15 m limit. The immediate
switch back contributes to the error in this replay, while the thermal
hold shows most of the early error already persists without that switch.
Over seconds 2–10 the hold reduces RMSE by 0.067 m versus immediate return
and falls below 0.15 m on that later cohort. This tests one continued
thermal schedule and supplies no estimate of what a delayed switch back
would do.

All 1,746 rebuilt return outputs match the previous return run by
timestamp and position. The return and hold arms match exactly across
all 1,000 output positions before the return boundary. The last
intervention reference row can interpolate across the boundary, giving
a sub-0.001 mm difference in the intervention RMSEs. Every reference
timestamp in the three scoring windows has support in all three arms.
During the first two seconds after return, maximum sensor-stamp output
gaps are 54.51 ms for immediate return and 50.03 ms for both thermal
controls. All nine RMSEs pass the independent arithmetic audit.

The source diff, frozen windows, preflight, original and rebuilt binaries,
raw poses, shared support and hashes are under
`/root/ore-ms-artifacts/irs-author-rescore/medium/dimming/`, with the
new outputs in `hold/`. The experiment uses an offline schedule and
artificial dimming.
