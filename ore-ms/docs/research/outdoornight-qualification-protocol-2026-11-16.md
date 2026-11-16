# OutdoorNight full-export qualification protocol

16 November 2026, America/New_York; base a41fb5640.
Input qualification only; no matching, training, selection or reference fitting.

Use the official OutdoorNight sequence association at
[SubT-MRS](https://superodometry.com/datasets), its own intrinsics/extrinsics,
and the complete 9,634,557,274-byte extracted ZIP (Drive ID
`1QSwNaIFjEKvikcoyDeUtVDcnR8Wvc2ax`). Record exact hashes, member names, dates,
full camera timestamp lists and exported IMU. A raw bag with a different date
cannot supply missing frame/header/reference proof for this export.

Reuse `qualify_subt_export.py` and its independent auditor with configuration.
Explicit null `reference_archive` admits input-only qualification: validate IMU
but do not fabricate poses, motion cohorts, gyro/reference agreement or accuracy.
The inspected extension reference folder has four files without OutdoorNight;
check the primary reference listing as well. Absence is scoped to inspected
listings and archive members, not a universal dataset-absence assertion.

Decode every PNG; check ZIP CRC and PNG chunks, native RGB uint8 BGR 640x480 and
thermal uint16 640x512, unique contiguous ordinals, strictly increasing integer-ns
timestamps and own calibration. Keep RGB BGR-to-gray and thermal per-image
min/max-to-uint8 rounding unchanged. Validate published camera-to-IMU top 3x4;
only roundoff SO(3) correction below 1e-5 and explicit homogeneous-row repair.
Check the unchanged 4,096-point ray roundtrip below 0.001px.

Anchor each 0.5s from the first RGB timestamp while a full 0.5s remains. Nearest
sensor endpoints must be distinct and within 25ms; preserve unsupported cases.
No pose-supported cohorts exist absent a qualified reference. Do not use an IMU
orientation CSV as independent motion ground truth.

Inspect seven equally spaced RGB-anchored times over the full span, both cameras,
before any prediction. Summarize full-span grayscale ranges and time thirds;
names, appearance and camera-processed intensity do not establish ambient lux.
No condition- or error-selected trimming. Record findings, independent audit,
source/runtime pins and recovery receipts. Stop after qualification: any
reference-free matching study requires a separate prospective protocol and may
report support/availability/cost, not pose accuracy or stateful VO recovery.
