# IRS RTVI visual-to-thermal handoff pilot

The [stateful replay controls](irs-rtvi-stateful-replay-2026-11-17.md) showed
that losing visual input for two seconds interrupts published odometry.
This pilot tested whether thermal input fills that gap in a shared filter.
The prototype passed the local gap and position checks. It failed the
prespecified visual-only baseline parity check, which disqualifies the
prototype for a recovery claim.

## Replay and criteria

Use the existing mocap_easy recording and the previously fixed [40 s, 42 s)
window in bag time. Preflight decoded both full mono8 streams and found 40
visual frames, 40 thermal frames and 819 IMU messages inside the window.
The prototype uses one two-camera RRxIO state with 15 feature slots and the
published visual filter preset, plus both released camera calibrations and
initial extrinsics. It submits one camera image at a time and applies each
camera's published timestamp shift. It removes outgoing camera landmarks
and their covariance blocks while preserving the body state and IMU bias.
It disables stereo initialization and cross-camera patch matching. Each arm
provides all 38,902 original IMU samples. The switch schedule is supplied
ahead of time.

The primary gate requires the prototype visual-only arm to retain at least
99% of the native visual reference-associated outputs and match its whole-run
position RMSE within 0.01 m. The handoff also needs a single initialization,
an output gap at most 0.15 s, and position RMSE at most 0.15 m within the
outage and the first two seconds after the visual camera returns. The window
errors use a fixed yaw and translation alignment fitted over the preceding
10 seconds. Scale remains fixed.

## Findings

| Prototype arm | Largest output gap around outage | Outage position RMSE | Position RMSE after visual return |
|---|---:|---:|---:|
| Uninterrupted visual | 0.050 s | 0.033 m | 0.039 m |
| Visual blackout | 2.050 s | No odometry | 0.034 m |
| Visual → thermal → visual | 0.054 s | 0.062 m | 0.068 m |

All three arms initialized once and exited successfully. The handoff
published odometry at each of the 40 submitted thermal timestamps and on
the first restored visual frame. The associated nominal position sample counts
are 40 during the handoff and 41 after visual return. The prototype's
whole-run visual control retains all 1,589 reference-associated outputs.
Its 0.101 m RMSE differs from the native 0.127 m by 0.026 m, exceeding the
0.01 m parity bound. Lower error fails this check because the control must
isolate the effect of camera substitution. Repeats of the native and
prototype visual runs reproduce all shared output rows within each binary.
All shared rows before the intervention match exactly across prototype arms.

The two-camera startup self-tests emitted feature-transform and IMU-landmark
Jacobian discrepancies in every arm. The feature failure is for the tested
camera-1 feature projected into camera 0, a path excluded from image updates
in this pilot. The landmark Jacobian uses unindexed extrinsic columns even
for a camera-1 feature. It runs in point-cloud reporting here. The failures
do not identify the cause of the visual parity difference.

This closes the current two-camera configuration under the frozen gate.
A new replay candidate should first reproduce native monocular visual
behavior with the handoff disabled. The pilot uses recorded sensor time
and an external switch schedule. Live loss detection and wall-clock recovery
remain unmeasured.

The protocol, isolated source patches, input preflight, hashes, replay logs,
scores and repeats are in `/root/ore-ms-artifacts/irs-rtvi-intake/stateful/handoff/`.
No new bags, derived bag copies or Box writes were needed.
