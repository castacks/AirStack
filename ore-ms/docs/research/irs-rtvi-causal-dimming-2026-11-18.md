# IRS RTVI causal brightness-triggered thermal handoff

The [medium](irs-rtvi-medium-thermal-hold-2026-11-18.md) and
[higher-motion](irs-rtvi-difficult-dimming-transfer-2026-11-18.md) scheduled
replays supply the two exposed 10-second interventions and fixed controls.
Reuse their recordings, camera and IMU shifts, calibrations, filter
settings, onset and recovery windows. The injector uses the schedule.
The detector sees incoming visible pixels and triggers when at least 95%
of pixels are at or below 16 for three consecutive frames. It passes
the triggering visible frame to the estimator and then selects thermal
for later sensor timestamps. It stays on thermal. The threshold was fixed
from the artificial dimming range. The route has no return schedule.

| Recording | Trigger after dimming onset | First thermal update after onset | False switches on unmodified bag |
| --- | ---: | ---: | ---: |
| Medium | 132.75 ms | 178.25 ms | 0 |
| Difficult | 129.74 ms | 175.24 ms | 0 |

Independent readback of the recorded image bytes and timestamps
reproduces both trigger and first thermal input times. There are 3,490
visible frames across the two unmodified captures and none qualifies
as a dark frame. The dimmed runs pass three dimmed visible frames and
945/940 unmodified thermal frames for medium/difficult respectively.

Fit yaw and translation on the preceding ten seconds of shared
Vicon–IMU positions, at unit scale. Sample epochs with the same 100 ms
bracket cap. All arms score the same 200 positions during dimming, 40
in the first two seconds after restored light, and 160 medium or 159
difficult in the subsequent eight seconds.

| Recording and arm | Dimming RMSE | First 2 s after light returns | 2–10 s after |
| --- | ---: | ---: | ---: |
| Medium, dimmed visual | 0.657 m | 0.462 m | 0.296 m |
| Medium, scheduled switch and hold | 0.124 m | 0.230 m | 0.143 m |
| Medium, image-triggered switch and hold | 0.105 m | 0.183 m | 0.135 m |
| Medium, fixed thermal | 0.102 m | 0.119 m | 0.215 m |
| Difficult, dimmed visual | 1.015 m | 1.013 m | 0.544 m |
| Difficult, scheduled switch and hold | 0.118 m | 0.122 m | 0.127 m |
| Difficult, image-triggered switch and hold | 0.113 m | 0.254 m | 0.324 m |
| Difficult, fixed thermal | 0.182 m | 0.261 m | 0.220 m |

The detector preserves the dimming accuracy gain and exceeds the
0.15 m early recovery bound on both recordings. Difficult also exceeds
the bound in the later window, where the scheduled control passed.
The changed onset and the intervening dimmed visual updates are both
part of this causal policy. The experiment cannot isolate their
individual effects. Causal output gaps stay below 51 ms in sensor time.
The test uses exposed recordings with a programmed intensity change
and does not establish detection or accuracy under natural illumination
changes, or wall-clock latency.

The initial replay accidentally applied synthetic dimming to selected
thermal frames after handoff. That run is invalid and retained. The
corrected run restricts the intervention to visible frames. Its first
medium unmodified capture missed 23 matched positions, failing the
99% parity gate. Retrying only that unchanged arm passed with 1,744
of 1,746 positions, all exact. Difficult's unmodified capture matches
all 1,726 positions. The two dimmed arms share 798 of 800 and 781 of
784 pre-intervention positions, all exact. Independent arithmetic
reproduces all 24 reported RMSEs.

The frozen detector and protocol, loader diff and binary, invalid
attempt, clean captures, shared timestamp grids, event logs, and
verified hashes are in `/root/ore-ms-artifacts/irs-author-rescore/causal/`.
