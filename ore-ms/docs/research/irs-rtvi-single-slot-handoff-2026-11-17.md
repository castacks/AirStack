# IRS RTVI handoff with the native monocular filter state

The [first handoff prototype](irs-rtvi-cross-camera-handoff-2026-11-17.md)
changed the visual-only trajectory when it reserved a second camera slot.
This replay keeps the native state dimension, one active camera slot and
the same three controls on the existing easy recording. At a transition,
the image update loads the incoming camera model and its published
extrinsic prior. It clears the previous camera's landmarks and their
covariance and replaces the active extrinsic rows and columns. The
body pose, velocity, IMU bias and their marginal covariance carry through
these operations. Each arm uses the same visual filter preset and all
38,902 IMU measurements.

## Scheduled handoff comparison

| Replay arm | Largest output gap | Position RMSE during outage | Position RMSE after visual return |
|---|---:|---:|---:|
| Uninterrupted visual | 0.050 s | 0.045 m | 0.041 m |
| Visual blackout | 2.050 s | No odometry | 0.058 m |
| Visual → thermal → visual | 0.054 s | 0.066 m | 0.065 m |

The outage window is the already fixed [40 s, 42 s) interval in the easy bag.
The post-return window is two seconds. Each arm uses a fixed translation
and yaw fit from the preceding 10 seconds, with scale held fixed. All 1,897
shared no-switch rows match native visual exactly, with the same 1,589 scored
positions and 0.127 m whole-record RMSE. This passes the frozen 1 cm and
99% parity margins. All three arms initialize once, and the prior
two-camera Jacobian warnings are absent. The thermal arm has 40 associated
positions during the outage and 41 after return. Its gap and both error
windows pass the frozen 0.15 s and 0.15 m bounds for the scheduled experiment.

The first handoff process crashed while constructing the separate calibration
holder. Reallocating it through the same direct allocation pattern used by the
native loader resolved the failure. The failed attempt was retained. All
three reported arms ran in the corrected binary.

The source and evidence are in
`/root/ore-ms-artifacts/irs-rtvi-intake/stateful/single-slot/`.
The scheduled replay knows the outage interval in advance. Its gaps are
measured in sensor time. An observation-driven selector replay is needed to
measure recovery while replaying at the original cadence.
