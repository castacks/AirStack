# EXP-075: conditional pose-quality benchmark for the advisory rule

2026-10-29 America/New_York. Branch `ore_ms`, base `7fde85a6e`.

The availability rule now has a fixed offline pose-comparison implementation.
Under supplied nominal body transforms and the recorded SLAM proxy, its fixed
RGB preference is worse than thermal-only in both existing windows. In the
middle window, its single RGB choice has1.00308m translation and50.1876deg
rotation residual; thermal on the same interval has0.04029m and1.32685deg.
Tracking support alone does not establish good motion. No quality gate is fitted
or adopted, and these conditional proxy residuals are not independent accuracy.

## Sources and scope

Use every frozen decision from EXP073 (19 early) and EXP074 (99 middle), with
exactly their original event-prefix eligibility and deadline abstentions. No
new inference, images, reserved captures, selector threshold or model/default
change. Both windows are already examined portions of the same recording.
Only18 early and1 middle intervals have both experts eligible by the deadline;
these19 intervals are not independent generalization evidence.

Restore five original SO export files from verified EXP031 archive/member hashes.
The available metadata identifies `sensor_init -> vehicle`; interpolate supplied
positions linearly and rotations by SLERP at original integer nanosecond times,
without extrapolation. SO is recorded SLAM, not independent ground truth. The
separate GNSS export is position-only for this purpose, with uncertain physical
origin, quality and timing; it does not supply a verified independent SE3 target.

The supplied `yamaha.yaml` is recovered from the original2026-10-18 Box snapshot,
whose archive SHA256 is verified. The pinned GitHub URL returned404; no substitute
calibration was chosen. Its exact file SHA256 matches the historical audit:
`031c199619916bf1466a807d9ed91896d58b1428f0bdf74951dabc3c1b59e27d`.
Retain the established YAML parent-from-child convention. These supplied edges
permit nominal frame transport but do not independently verify the physical
body anchor, thermal signed geometry or reference accuracy.

## Fixed comparison contract

For each expert, E is nominal `T_body_from_rectified_left_camera_NED`, composed
from supplied static edges, that camera's rectification R and the existing
optical/NED permutation. The original estimator's identity T_BS is not relabeled
as a verified rig extrinsic. Transform only a copied first-write camera increment:

`D_body = E * D_camera * inverse(E)`.

The common comparison interval is the original RGB interval `[r0,r1]`. If the
expert's own interval is `[a,b]`, use recorded body reference R(t) for endpoint
bridges:

`D_common = inverse(R(r0)) * R(a) * D_body * inverse(R(b)) * R(r1)`.

This preserves actual camera timestamps and explicitly uses reference information
for **offline evaluation only**. It is not deployable timing compensation and
cannot be a causal predictor input. The common reference increment is
`D_ref = inverse(R(r0)) * R(r1)`.

Primary loss is the translation norm of `inverse(D_ref) * D_common`, in meters.
Secondary loss is that residual rotation's geodesic angle, in degrees. Keep them
separate: no fitted mixed-unit score, scale/trajectory alignment, offset search
or fitted extrinsics. Save own-interval losses alongside common-interval losses.
Missing results stay null; abstentions are never assigned zero error. Preserve
all118 decision rows and report error populations together with availability.
Compare per window, with a separate common-eligible population for camera ranking.
A descriptive per-interval translation oracle uses the proxy losses and is
explicitly unavailable to an online selector.

Serialized RGB R has orthogonality defect5.31699e-8. An initial CPU attempt rejects
it before producing losses under a stricter SE3 check. The declared numerical
policy then projects only near-rotation serialization roundoff to nearest proper
SO3 by SVD, rejecting defect>1e-6 or nonpositive determinant. Maximum RGB entry
correction is2.65849e-8; thermal correction is3.33e-16. This is recorded explicitly,
not fitted to scores, and never changes inference inputs. The completed result
is `run-002`; the incomplete `run-001` protocol is retained for provenance.

## Results at the frozen deadlines

Translation RMSE refers to conditional common-interval SLAM-proxy residuals.

| Window | Policy | Available / decisions | Translation RMSE (m) | Rotation RMSE (deg) |
|---|---|---:|---:|---:|
| Early | RGB-only / fixed advisory |18/19|0.009891|0.06955|
| Early | Thermal-only |18/19|0.009248|0.03231|
| Middle | RGB-only |1/99|1.003081|50.18764|
| Middle | Thermal-only |98/99|0.042620|0.67862|
| Middle | Fixed advisory |98/99|0.109850|5.11318|

The middle RGB-only row has a different error population from full thermal/advisory
rows; do not treat its single interval as a full-coverage camera aggregate. On
that one common eligible interval (pair ordinal96), both cameras are compared
against the same nominal body target: RGB1.00308m versus thermal0.04029m. The
RGB own-interval loss is also1.00308m, so that large residual is not introduced by
the asynchronous bridge. The fixed preference selects RGB there, worsening the
otherwise thermal-only middle result despite unchanged98/99 availability.

On the18 common early intervals, translation proxy loss favors RGB5 times and
thermal13 times. Common-support translation mean is0.008465m RGB,0.007953m thermal,
and0.007749m for the reference-assisted oracle. This small descriptive headroom
is not an achieved selector improvement. No training, threshold selection or
physical camera-winner claim follows from these observations.

Maximum absolute own-versus-common translation-loss difference across eligible
experts is2.56e-7m early and0.000301945m middle. The bridge remains a conditional
reference-assisted operation even when its numerical effect is small. No row is
removed based on proxy disagreement, support strength or reference outcome.

## Implementation and validation

`Scripts/Setup/arbiter_pose_proxy.py` implements nominal conjugation, offline
endpoint transport and separate losses. Its frozen protocol explicitly marks
physical-body verification, independent ground truth, causal-feature eligibility
and gate fitting as false. Output labels and reference matrices remain separate
from the original causal event journals; no original advice is overwritten.

Thirteen Docker CPU tests pass: rotated axes, pure rotation with a lever arm,
noncommuting asynchronous composition, perfect-motion recovery, world-anchor
invariance, exact large timestamps, malformed SE3 rejection, explicit roundoff
projection and null empty-population losses.
`validate_arbiter_pose_proxy.py` imports no producer functions. It independently
reconstructs reference interpolation, uses the established camera-reference path
for nominal extrinsics, expands body conjugation into rotation/translation
components, and checks7020 numerical components across118 rows plus all summary
populations. Five altered reports/protocols are rejected: changed loss, altered
body transform, unavailable-result backfill, changed population count and an
independent-ground-truth claim. Original input hashes remain unchanged.

Authoritative artifacts: `results/arbiter-pose-proxy/`, including restoration
proofs, roundoff evidence, CPU JUnit, incomplete first protocol and completed
`run-002/{protocol.json,pose-proxy.json,validation.json}`. Completed report SHA256:
`256cd1a47d85ef191cbc3c915182f48997bf4ec18cc646c98324623729db81c6`.
Run in the existing `macvo:event-tests` CPU image with ore-ms mounted at
`/workspace`, workdir `/workspace/MAC-VO-main`:

```bash
python -m Scripts.Setup.arbiter_pose_proxy --root /workspace --output <fresh-output>
python -m Scripts.Setup.validate_arbiter_pose_proxy --root /workspace --run <fresh-output>
python -m pytest Scripts/UnitTest/test_arbiter_pose_proxy.py -q
```

Recovery destination: `backups/2026-10-29-exp-075`. Preserve the exact supplied
calibration and restored reference with this snapshot; source event journals and
camera metadata remain in EXP071–074. No weights, credentials, full archives or
new image data are included.

## Decision

Keep the availability rule as a tested baseline. Its fixed RGB preference is
not a demonstrated quality selector and should not be promoted as one. Next
predeclare a causal recovery/history baseline and compare it with fixed experts
under this conditional benchmark before considering learned quality selection.
Do not tune it to the lone middle outlier and call the same recording validation.
Independent physical claims still need justified body/interval/reference
provenance; the proxy benchmark can support explicitly scoped software research
without waiting for those claims to become available.
