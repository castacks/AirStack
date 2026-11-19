# IRS RTVI higher-motion dimming transfer

The medium run established that a scheduled thermal handoff can improve
position during dimming. [Keeping thermal active](irs-rtvi-medium-thermal-hold-2026-11-18.md)
still missed the early recovery bound there, and the
[two-second delayed return](irs-rtvi-medium-delayed-return-2026-11-18.md)
missed the primary post-switch bound. This transfer uses `mocap_difficult`
and a single thermal handoff with no switch back. No return-delay sweep
was run. This is a distinct recording on the same rig, with the same
artificial reduction of visible pixel intensities.

Freeze the interval from 40 to 50 s after bag start rounded to the
nearest integer second, giving shifted camera stamp bounds
`[1614701820, 1614701830)`. During this interval the reference traverses
12.60 m of sampled path versus 10.08 m in the medium intervention, with
peak reported speed 1.86 versus 1.45 m/s. Both cameras provide 200 frames.
The visible frame p95 range of 211–255 changes to 13–15 when divided by
16. The shift and calibration for each camera, filter constants and full
IMU sequence match the fixed protocols. The scheduled arm stays on thermal
after light returns. Fixed thermal uses its native preset from launch.

Use 200 shared Vicon–IMU timestamps in the preceding ten seconds to fit
each arm's yaw and translation at unit scale. Interpolate within 100 ms
brackets. All 399 reference timestamps in the three scoring windows
are supported in all four arms.

| Arm | Dimming (200 positions) | First 2 s after light returns (40) | 2–10 s after light returns (159) |
| --- | ---: | ---: | ---: |
| Visible, original frames | 0.060 m | 0.070 m | 0.092 m |
| Visible, dimmed frames | 1.015 m | 1.013 m | 0.544 m |
| Switch to thermal and keep it active | 0.118 m | 0.122 m | 0.127 m |
| Fixed thermal from the start | 0.182 m | 0.261 m | 0.220 m |

The scheduled arm reduces dimming RMSE by 88.3% relative to dimmed
visual and is below the frozen 0.15 m bound in all three windows.
Its largest sensor-time output gap across the windows is 50.22 ms,
below the 0.15 s bound. This reproduces the advantage over dimmed
visual on a different trajectory. It also beats the fixed-thermal
control in these windows, whereas fixed thermal beats the switched
arm in the medium run's first two seconds after restored light.
The contrast describes these complete configurations and trajectories.
It leaves causal selection and real lighting transitions untested.

The first handoff capture missed 16 startup output rows and failed
the frozen 99% pre-intervention parity gate. Its log and poses were
retained, and no scores were produced from it. Repeating the identical
arm met the gate. Each of the dimming and handoff arms matches all 784
native visible pre-intervention poses exactly at shared timestamps.
Every scored arm initialized once and exited cleanly. An independent
position calculation reproduces all 12 RMSEs within 1e-10 m.

The release ZIP member CRC and bag SHA-256 verified. The SHA-256 is
`a6aaa8c69aa297ea891d80872c6484a8b317760fcd3dc6543a45aab0aa630267`.
The frozen protocol, preflight, loader diff, failed capture, scored
poses and verification hashes are in
`/root/ore-ms-artifacts/irs-author-rescore/difficult/`.
