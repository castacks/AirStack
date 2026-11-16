# FlashLight whole-export qualification protocol

16 November 2026. Qualification only, before any matching predictions on this
recording. Transfer candidate for the unchanged [RGB-first baseline](smoke-rgb-first-protocol-2026-11-16.md).
No training, clock/extrinsic fit, threshold search or pose-error-selected windows.

Use the complete 5,070,560,080-byte extracted FlashLight ZIP and corresponding
March reference ZIP. Own FlashLight SP1 intrinsics/extrinsics come from the
[official sequence row](https://superodometry.com/datasets). Preserve source
payloads/hashes. The linked raw July bag is not a substitute for the March export.
Export timestamps lack raw ROS image-header evidence; qualify nominal supplied
bindings only, not physical exposure or independent reference truth.

Check every PNG's ZIP CRC, PNG chunks, decode, expected dtype/size and unique
contiguous ordinal→timestamp binding; require strictly increasing integer-ns
stamps. Preserve native grayscale conversion: RGB uint8 BGR→gray, thermal uint16
per-image min/max→uint8 with nearest rounding. Record raw ranges and grayscale
metrics, hashes and output paths. Inspect first/seven equally spaced image pairs
by recording time, without selecting by quality or reference error. No zero-lux
or quantitative smoke label follows from names or appearance.

Anchor every 0.5s from first RGB stamp while a full 0.5s remains. Bind each sensor
to nearest endpoint with ≤25ms error and distinct source/target. Retain all
unsupported anchors. References interpolate positions linearly/quaternions by
SLERP only with a bracket ≤50ms, no extrapolation. Freeze common-reference,
both-camera rotation≥2° and both-camera displacement≥1cm cohorts before matching.
Validate model types, image dimensions, projection/unprojection at the unchanged
fixed query grid (roundtrip <0.001px). Respect MEI RGB and pinhole thermal rays.
Published camera→IMU top3×4 is authoritative; normalize a zero homogeneous bottom
row explicitly, and SO(3) roundoff only if Frobenius correction <1e-5. Do not fit.

Compare the full export IMU CSV against reference-side IMU by integer timestamps
and exact numeric fields. Preserve unmatched/different rows and their coverage.
Check reference and IMU finite values, quaternion norms within1e-3, strict stamps.
At reference-start anchored 0.5s intervals integrate body gyro with left sample
hold and true boundary durations; reject intervals with >20ms IMU gaps. Compare
full relative SO(3) to supplied IMU-to-world reference convention. For ≥2° motion,
require at least20 intervals, median discrepancy≤0.2° and95th percentile≤0.5°
for nominal observed axes/time consistency. Report all intervals and rotation-axis
spread; no axis/clock correction follows failure. This consistency check is not
independent physical ground truth, since reference generation can use IMU.

Stop at a full-span preflight and independently audited qualification report.
Retain failures without trimming the recording. No inference in this stage.
Subsequent matching requires its own prospective plan and runtime qualification;
reuse source-pinned matcher primitives and unchanged fallback, rather than copy
frozen runners for a path change. Archive metadata/plots/scripts plus verified
input receipts; raw ZIPs remain outside Git and are not redundantly archived.
