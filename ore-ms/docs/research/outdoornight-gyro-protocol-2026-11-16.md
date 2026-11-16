# OutdoorNight raw binding and gyro-agreement diagnostic protocol

16 November2026, America/New_York. Before gyro/image disagreement is computed.
Reuse locked3,884predictions and1,942status-only decisions from the
[completed reference-free comparison](outdoornight-matching-findings-2026-11-16.md).
No new inference, model/threshold selection, training or coordinate/clock/bias fit.

## Reference search and scope

Official missing-OutdoorNight-reference issue35 remains open without comments.
The maintainer-linked reference parent has the already checked fusion/LiDAR
folders plus six visual-inertial files, none OutdoorNight. M2P2's inspected684bag
catalog has no trajectory sidecar; wheel/filtered odometry cannot silently become
independent pose truth. Retain source receipts and close these listings absent
new evidence; no author outreach or access request is authorized by this protocol.

The finite diagnostic asks whether added supported thermal rotations agree with
measured angular motion better than a zero-rotation assumption. Gyro is a separate
instrument, not a certified pose reference: calibration/bias, raw header/exposure
semantics and independence remain qualified. Translation cannot be scored.

## Raw/export qualification, before diagnostic labels

Acquire official raw ZIP ID`1LIu46hN_W2My8pnMzhB2wlgKJrK4KTMD`,11,850,330,134bytes.
Verify strict ranges/assembled SHA and ZIP CRC. Inspect all six bags' connection
metadata, camera/IMU topics, frame IDs, image encodings and numeric fields.
Retain topic differences, absent data and duplicated/split timestamps.

Bind full exported camera chronology to raw image headers, with one-to-one
timestamp matches within1microsecond (numeric timestamp serialization only;
record exact signed differences). Require all26,371exported pixel arrays equal
their decoded raw camera arrays under declared RGB/BGR/mono16 encoding conversion.
Raw/export IMU numeric fields must match exactly at corresponding headers within
the same1microsecond serialization bound; do not fit any offset. Preserve raw-only
or export-only entries. Stop if the required camera or gyro binding is unresolved.
Camera-to-IMU axes retain the published calibration association, without fitting;
do not infer independent metrology from topic names or an image-byte match.

## Frozen gyro endpoints and comparison

Use every original sensor-specific source/target timestamp, not a common-index
or common-clock substitution. Left-held body gyro integrates with actual integer-ns
boundary durations. Reject extrapolation, reversed intervals and any enclosing
IMU gap>20ms. Retain input-unsupported and gyro-unsupported rows separately.
The target-from-source IMU rotation is the transpose of integrated source-from-
target rotation. Conjugate each locked camera proposal by its own fixed camera-to-
IMU rotation. Do not integrate IMU acceleration or use fused orientation as truth.

Before reading errors, freeze all input/gyro-supported pairs and both-camera gyro
rotation>=2degree anchors. Also retain all declared anchors and fixed time thirds.
Primary descriptive endpoints: added thermal proposals within1/5degrees of their
own gyro-derived rotation on the frozen paired-moving cohort; compare zero
rotation on exactly those same thermal endpoints. Report each policy's agreement,
missing proposal, missing input and missing gyro support with explicit denominators.
At all/reference-supported static cases, retain identity controls so a near-zero
output cannot masquerade as recovered moving accuracy. No IID significance claim.

Stop after a locked saved-output diagnostic and independent quaternion integration
audit, preserving every failure. Report angular disagreement/consistency only,
not ground-truth pose accuracy, thermal recovery success or trajectory improvement.
