# IRS RTVI medium-motion bright metric position

Replay the author RRxIO visual–IMU and thermal–IMU controls on
`mocap_medium` with the same binary, 15-feature presets, signed camera
time shifts, and [shared-timestamp scoring](irs-rtvi-fast-dark-common-support-2026-11-18.md)
as fast dark. Use released Vicon–IMU body positions. Freeze and hash the
scorer before replay. Fit translation and gravity-axis yaw on the first
10 s of common support and hold both fixed for scores on the remaining
positions. Scale is fixed.

| Alignment and scoring | Shared positions scored | Visual RMSE | Thermal RMSE |
| --- | ---: | ---: | ---: |
| First 10 s fit, remainder scored | 1,237 | 0.089 m | 0.248 m |
| Full overlap fit and score | 1,438 | 0.082 m | 0.140 m |

Visual published 1,746 poses and thermal published 1,741. Of 1,441
reference timestamps, 1,438 have shared support and 201 fall inside
the alignment period. The early-fit RMSE difference is 0.159 m in
favor of visual. Maximum errors there are 0.155 m visual and 0.402 m
thermal. Independent arithmetic reproduces both shared-set RMSEs for
both alignments within 1e-10 m.

For an image-level condition check, sample the visible frame at the
one-quarter, one-half and three-quarter indices of each bag. The
medium recording's three image p95 values are 227, 253 and 241 on the
8-bit scale. The same fast-dark samples give 10, 12 and 10. These are
recorded image values and have no physical illuminance calibration.

The lower-error modality changes between these two recordings.
Visual gives 0.089 m versus 0.248 m in medium. In fast dark, thermal
gives 0.282 m versus 4.302 m. Each comparison uses its own common
timestamp set and the published camera-specific presets. The bags
contain distinct trajectories, so this comparison cannot isolate
lighting or establish the benefit of switching during one run.

The release ZIP CRC passed and the bag SHA-256 is
`5230f07e191b8cd1ccb57a0b5466a887142fd8e64f1dc94dd48dfa87bcb07cde`.
Run logs, raw poses, score, timestamp ledger, image samples and hashes
are under `/root/ore-ms-artifacts/irs-author-rescore/medium/`.
