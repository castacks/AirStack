# IRS RTVI position error at the start of early recovery

Use the saved [matched extrinsics comparison](irs-rtvi-fixed-thermal-extrinsics-2026-11-18.md)
and the existing scheduled and native thermal controls. Carry forward
the 10 s pre-dimming yaw and translation fit and unit scale. Each arm
uses the same 40 reference timestamps in the two seconds after the
light returns. The first timestamp is 30.45 ms after the scheduled
boundary on Medium and 27.76 ms on Difficult.

At each timestamp, split the position error vector into the residual at
the first reference timestamp and its subsequent change. The change
RMSE measures displacement error relative to that first timestamp.
It is a retrospective diagnostic. Published poses and alignment were
unchanged. The two components are correlated, so their squares need
the cross term to reconstruct the original mean squared error.

| Recording and arm | Error at first reference pose | Change RMSE | Original early RMSE |
| --- | ---: | ---: | ---: |
| Medium, adaptive handoff | 0.188 m | 0.026 m | 0.181 m |
| Medium, fixed extrinsics at handoff | 0.090 m | 0.033 m | 0.078 m |
| Medium, scheduled hold | 0.233 m | 0.055 m | 0.230 m |
| Medium, native thermal from startup | 0.094 m | 0.040 m | 0.119 m |
| Difficult, adaptive handoff | 0.140 m | 0.076 m | 0.177 m |
| Difficult, fixed extrinsics at handoff | 0.153 m | 0.040 m | 0.175 m |
| Difficult, scheduled hold | 0.113 m | 0.041 m | 0.122 m |
| Difficult, native thermal from startup | 0.260 m | 0.023 m | 0.261 m |

Medium's gain with fixed extrinsics is already present when the light
returns. In Difficult, the adaptive arm starts below 0.15 m and
reaches 0.229 m by the last early-recovery sample. Fixing extrinsics
reduces subsequent change to roughly the scheduled arm's level, while
the starting offset increases to 0.153 m. Difficult therefore still
has two separable targets for analysis: the state error accumulated
during dimming and the subsequent pose change. The comparison cannot
identify the source of the position offset or establish that physically
recalibrating the camera would remove it.

The original RMSEs and squared-error identities reproduce for all
arms. The frozen calculation and signed vectors are in
`/root/ore-ms-artifacts/irs-author-rescore/recovery-error-origin/`.
