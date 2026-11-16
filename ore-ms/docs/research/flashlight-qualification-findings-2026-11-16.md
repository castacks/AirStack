# FlashLight export qualification findings

16 November 2026. Completed [whole-export qualification protocol](flashlight-qualification-protocol-2026-11-16.md).
No matching, model execution, pose-error scoring, clock/calibration fitting or
selector tuning occurred. This finishes the input stage for an unchanged-rule
transfer of the [SmokeRoom RGB-first baseline](smoke-rgb-first-findings-2026-11-16.md).

## Decision

The March extracted FlashLight release qualifies for **nominal supplied-reference
pair-level evaluation**, with explicit missing support. It supplies an additional
recording, not independently certified physical truth or a guaranteed zero-light
experiment. Do not conflate it with the linked July raw bag or with SmokeRoom.
The new source/calibration/reference/appearance checks are completed before any
prediction on this recording; thresholds remain prospective and unchanged.

| Whole-release check | Result |
|---|---:|
| Extracted ZIP | 5,070,560,080 bytes |
| RGB / thermal images, all decoded and independently redecoded | 6,652 / 8,063 |
| 0.5s anchors / sensor intervals | 541 / 1,082 |
| Input-supported sensor intervals | 1,081 |
| Common-reference anchors | 538 |
| Both-camera reference rotation≥2° anchors | 192 |
| Both-camera reference displacement≥1cm anchors | 519 |
| Export and reference-side IMU rows, exact numeric match | 54,104 / 54,104 |
| Supplied reference poses | 53,768 |
| Independent audit checks | 80,634 passed |

The first thermal endpoint is 77.62ms after its requested anchor, exceeding the
unchanged25ms gate. It remains an explicit unsupported input; all541 anchors
remain declared. The common-reference cohort excludes three anchors at startup,
without treating reference absence as a tracking failure. Reference maximum gap
is70ms; endpoints requiring a bracket above50ms are unsupported. These gates are
applied to the entire recording, not a successful prefix.

## Nominal camera and reference contract

The [official sequence row](https://superodometry.com/datasets) identifies FlashLight
as SP1 with RGB/thermal/IMU and changing illumination. Its own released
[intrinsics](https://drive.google.com/file/d/13iTBn_po_GWxt3X8kNWVGZ2wwQ38Eou0/view)
and [extrinsics](https://drive.google.com/file/d/10YJQ3FMRw95F3_yhOsX2bbuMuvQbtnVV/view)
were fetched and hashed. RGB uses MEI plus radial-tangential distortion; thermal
uses pinhole plus radial-tangential distortion. Do not reuse SmokeRoom RC7 values.
All image dimensions and dtypes agree: RGB640×480 uint8BGR, thermal640×512 uint16.
At each unchanged4,096-point grid the projection roundtrip maximum is below
1.2e-12px. This verifies algebra/calibration compatibility, not a new calibration.

Published camera→IMU upper3×4 matrices are preserved. The thermal matrix has an
invalid all-zero homogeneous bottom row in the source; an explicit effective
[0,0,0,1] row is used. Only export-roundoff SO(3) correction below1e-5 is allowed;
no extrinsic estimate or fit was made. Translation remains at the declared sensor
origins. Reference positions interpolate linearly and rotations by SLERP; actual
selected image timestamps, source/target paths and both-camera motion eligibility
are frozen in `intervals.json` and `population.json` before matching.

The [official fusion-track convention](https://superodometry.com/iccv23_challenge_Mul)
declares IMU-to-world orientation and world-coordinate IMU position. Across539
reference-anchored half-second checks, no IMU gap exceeds20ms. On190 intervals
with reference rotation≥2°, left-held gyro integration gives median discrepancy
**0.0754°**,95th percentile **0.1978°**. Both meet the prespecified0.2°/0.5° gates.
The independent auditor recomputes integration using quaternion multiplication
rather than the runner's matrix chain and independently checks interpolation,
sensor-relative poses and cohort membership.

Reference rotation-vector RMS components are approximately0.0255/0.0144/0.2372rad:
yaw dominates, so agreement does not certify equally strong excitation on every
axis. Exact exported-IMU agreement establishes file identity, not an independent
sensor experiment. Reference generation may use IMU and other sensors; gyro
agreement is nominal axes/timing consistency, not independent truth. The extracted
images have ordinal timestamp text files, with no raw ROS header/topic proof for
this March recording. Physical exposure timing, calibration applicability beyond
the released sequence association, exact sensor origin and training overlap remain
qualified. Do not replace that missing proof with the mismatched July raw bag.

## Observed image conditions

The fixed-time sheet uses seven equally spaced recording times, both modalities,
selected before predictions. Views show indoor scene structure, ceiling lights,
variable shading and bright foreground glare; RGB remains visibly lit in these samples.
All6,652 RGB images have global grayscale means86.09–143.76, median113.10;
p05–p95 range is188–223, median203. Global statistics include bright rig/foreground
pixels and camera processing, so they do not establish calibrated ambient lux or
exclude localized darkness. Thermal uint16→uint8 per-image min/max scaling also
prevents cross-sensor intensity comparisons as physical irradiance measurements.

Consequently this is an officially labelled changing-illumination transfer, with
observed condition qualification. It should not be advertised as a demonstrated
no-illumination test. Preserve the complete recording and fixed time bins; do not
choose a darker-looking interval after inspecting prediction errors. The next
matching stage can characterize whether thermal adds value here without requiring
that it win. A distinct night/no-light condition needs its own evidence.

## Evidence, maintenance and next stage

New generic qualifier `tools/qualify_subt_export.py` accepts sequence configuration;
`tools/audit_subt_export.py` independently checks raw export/gray bytes, stamps,
IMU membership, poses, cohorts and gyro integration. Frozen older runners are
unchanged. The adapter reuses existing PNG/grid/hash helpers and adds the complete
export/calibration/reference input contract; it is not another inference runner
copied to change paths. Host CPU dependency versions are recorded in `runtime.json`.

Ignored evidence: `results/flashlight-qualification/`, including source pages and
receipts, input freeze, images/intervals/cohorts, raw-reference/IMU arrays, full
gyro comparisons, audit, completion, appearance diagnostics and fixed-time sheet.
The archive SHA256 is
`67bcf7fc61511c2896be484d3fdec8bb2619aadfc8f2319d04a0f134ae35952d`;
the reference ZIP SHA256 is
`5ad55af1b50fadd5d72a08d372b4065ff156335fb73155ba3cd7cb20c65127dd`.
Raw ZIP and grayscale inputs are retained under ignored data paths; redundant
verified download parts are removed with a receipt. An incremental recovery packet
is under `/root/ore-ms-artifacts/flashlight-qualification-2026-11-16/`, with all
members readback-verified; raw image data is retained separately, not duplicated.
No Box upload, Git publication or author outreach occurred.

Qualification stops here as prescribed. Next finite stage: freeze native-size
SIFT and the same two learned checkpoints, query/pose gates and missing-output
RGB-first rule for all541 anchors; qualify the exact GPU runtime, then lock all
predictions and choices before scoring. Report the same538 common-reference,
192 paired-moving and519 translation cohorts plus all anchors/fixed thirds,
new correct/incorrect outputs, unsupported inputs, exact calls and projected cost.
No new selector, thresholds or stateful/metric VO claim follows from this intake.

The prescribed unchanged matching/fallback transfer is now [completed](flashlight-transfer-findings-2026-11-16.md), including its learned null result and SIFT countercase. Qualification inputs/protocols are unchanged.
