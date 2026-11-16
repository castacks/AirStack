# Paired SmokeRoom / FlashLight evidence restoration protocol

16 November 2026, America/New_York; exposed development restoration after
[priority-control bounds](camera-priority-findings-2026-11-16.md).
No new selector, quality threshold, training, axis/clock/bias fit or model search.
Historical results are known. This stage restores full paired data on the current
machine, not blind validation or a new research method.

## Fixed input contracts

SmokeRoom: official extracted ZIP `140UJgOCa-2itHUZQfwOK1zISD8XRGw51`,
9,248,440,073 bytes; reference ZIP `1YFm6F-c5NI84zKNqyCWA-_c-KABSKF4y`;
own RC7 intrinsics/extrinsics. FlashLight: extracted ZIP
`1_lxsaQN_s6MEedVsDhF6Z51Nf7xTGR_m`, 5,070,560,080 bytes; March reference ZIP
`1Kq1iKkDtwpkK9QOqv-3pJJrIJey4DrhL`; own SP1 intrinsics/extrinsics. Its July raw
bag is a different recording and cannot supply missing image-header proof.

Preserve official IDs/metadata, exact range/total/length checks, archive/member
CRC, observed SHA and all failures. FlashLight's known archive/reference hashes
are `67bcf7fc61511c2896be484d3fdec8bb2619aadfc8f2319d04a0f134ae35952d`
and `5ad55af1b50fadd5d72a08d372b4065ff156335fb73155ba3cd7cb20c65127dd`.
SmokeRoom's old complete archive hash is not in the available tracked findings;
report the newly observed hash without inventing old receipt parity.

Use `qualify_subt_export.py` / independent auditor. Decode all22,800SmokeRoom
and14,715FlashLight images, unchanged RGB BGR-to-gray / thermal per-image16-bit
min/max recipe, unique ordinal/time binding, strict integer-ns stamps and declared
MEI/pinhole ray roundtrip. Preserve published upper3x4 extrinsics, explicit thermal
bottom-row repair and sub1e-5 SO(3) serialization correction, without calibration fit.

Every0.5s from first RGB stamp while a full interval remains; nearest distinct
sensor endpoints within25ms. Reference interpolation: exact or <=50ms bracket,
linear position / quaternion SLERP, no extrapolation. Freeze common-reference,
both-camera>=2degree rotation and both-displacement>=1cm cohorts before predictions.
Retain missing inputs and references separately, including initial thermal gaps.

Recheck finite reference/IMU arrays, quaternion norms, exact numeric IMU membership
and all unmatched rows. Use existing generic reference-anchored0.5s left-held gyro
check, <=20ms IMU gaps, >=20moving intervals, median<=0.2degree / p95<=0.5degree.
These are the generic qualifier's unchanged nominal-consistency gates, not new
physical accuracy certification. Stop any failing contract without a threshold
rescue. Old SmokeRoom raw binding remains historical evidence; its raw bags are
not restored here and no fresh raw/header qualification is claimed.

## Reconstruct paired baseline rows

Use exact compiler image
`sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`,
original verified RGB/thermal checkpoints and unchanged `subt_export_comparison`.
Native-size SIFT, mutual0.75ratio matches,4,096-grid learned bidirectional flow,
<=1px cycle gate, own calibrated ray lifting, focal400 virtual camera and unchanged
RANSAC/cheirality gates. Pure status-only RGB-first fallback; fixed RGB/thermal
controls. No new priority/gate fitted from these rows.

Qualify both native shapes with bitwise repeated flow and unchanged model state,
selector truth table and ray/legacy lifting equivalence. Verify all input/source/
calibration/model/protocol pins. Finish both CPU qualifications before timed GPU
capture; GPU captures sequential, no competing inference. Capture all838SmokeRoom
and541FlashLight anchors,3,352and2,164rows respectively, retaining every failure.

Lock predictions, then choices, then supplied-reference scores. Independent audit
must reconstruct populations/errors/status/call/cost and source/frame/time bindings.
Compare populations and fixed moving1/5degree counts against old findings, retaining
all differences. Old raw predictions are unavailable here, so no original-array
bitwise parity assertion; repeated fresh flows alone establish repeatability.
Timing is descriptive fresh cached control computation, not historical bitwise
timing or actual lazy latency. No budget escalation or retuning follows mismatches.

## Evidence and stop

Save restored full row manifests, projections, locks, model/source/runtime pins,
reference qualifiers, audits, historical-parity report and verified local incremental
packet. Raw images/gray arrays/checkpoints/runtime remain external pinned inputs.
No Box upload, author outreach or Git publication. Stop after paired evidence is
ready. A subsequent selection protocol must distinguish provided-reference
SmokeRoom/FlashLight targets from nominal raw-gyro OutdoorNight, retain both fixed
cameras and both priority orders, use recording-level development splits, and
not relabel previously exposed data as blind or verified training-held-out.
