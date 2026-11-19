# IRS RTVI fixed thermal extrinsics at handoff

Run one paired change on the existing Medium and Difficult interventions.
Reuse the [causal veto and IMU-only bridge](irs-rtvi-imu-only-output-bridge-2026-11-18.md).
At the same first thermal update, use RRxIO's fixed-extrinsics mode with
the released thermal camera-to-IMU transform. The adaptive arm starts
from the same thermal transform and continues estimating it. Both arms
retain the same detector, per-camera shifts, IMU, handoff feature reset,
and thermal preset choices. The visible prefix is adaptive in both.

Use the existing 10-second pre-dimming Vicon–IMU fit of yaw and translation
with unit scale, the 100 ms interpolation cap, the 0.15 m position bound,
and the 0.15 s output gap bound. All arms share 200 dimming, 40 early
recovery and 160 Medium or 159 Difficult later reference rows.

| Recording and arm | Dimming RMSE | First 2 s after light returns | 2–10 s after |
| --- | ---: | ---: | ---: |
| Medium, adaptive extrinsics at handoff | 0.104 m | 0.181 m | 0.134 m |
| Medium, fixed extrinsics at handoff | 0.069 m | 0.078 m | 0.111 m |
| Difficult, adaptive extrinsics at handoff | 0.098 m | 0.177 m | 0.209 m |
| Difficult, fixed extrinsics at handoff | 0.108 m | 0.175 m | 0.246 m |

The fixed mode reduces Medium early recovery RMSE by 0.103 m and brings
it below 0.15 m. The Difficult early result changes by 0.002 m, stays
above the bound, and the later error rises by 0.037 m. The fixed transform
does not support a common default for recovery across the two recordings.
These are position tests of the extrinsic estimation setting. They do not
identify a physical camera calibration error.

The native thermal-only control, with its sensor active from startup and
its released preset, scores 0.119 m on Medium and 0.261 m on Difficult in
the early recovery window. This control does not share the switch state.

The fixed arm matches all pre-switch corrected poses exactly (800 Medium,
798 Difficult), as well as each of the three IMU-only outputs including
covariance. Trigger, thermal switch and accepted input counts match. End
state checks confirm zero change in the fixed thermal transform. Combined
sensor-time gaps are at most 50.22 ms across measured windows. The two
pose topics remain separate and wall-clock ordering was not measured.

The initial scorer required the literal string `0 0` for the extrinsic
check. ROS logged decimal-form zeros. The corrected parser compares their
numeric values and retains the original scorer. No rerun or threshold
change was needed. Independent arithmetic reproduces all 18 window
scores. Protocol, patch, binary, captures and verified checksums are in
`/root/ore-ms-artifacts/irs-author-rescore/fixed-extrinsics/`.
