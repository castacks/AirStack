# SmokeRoom: reference-frame qualification and relative-pose tradeoffs

16 November 2026. Completed the [prospective supplied-reference protocol](smoke-reference-pose-protocol-2026-11-16.md)
using all 3,352 locked predictions from the [real-smoke frontend study](deployment-characterization-findings-2026-11-16.md).
No new inference, clock fit, bias estimate, hand-eye calibration or model/threshold
search. This advances from support proxies and scalar rotation amount to full
relative SO(3) error under an explicit **nominal supplied-reference contract**.
It does not certify independent physical truth or metric VO trajectories.

## What now supports the reference contract

The [official sensor-fusion track](https://superodometry.com/iccv23_challenge_Mul)
defines submitted trajectories as IMU poses in a fixed world frame. Its published
SmokeRoom camera→IMU files provide the needed sensor transforms. The reference
CSV itself omits frame identifiers: applying that documented IMU convention to it
is explicit nominal engineering, not a newly verified sensor-origin measurement.
The paper describes a reference using map, visual, LiDAR and inertial constraints;
it is not independent optical ground truth or a solely LiDAR-derived trajectory.

Downloaded the actual 10,829,874,708-byte raw SmokeRoom ZIP, hash
`5682df7e9cc2c3578e55645463e0958e01897a68619569dfab1b868aabcab293`.
All five bag members decompress with ZIP CRC checks. Their only streams are RGB,
thermal and IMU; no raw odometry/TF message supplies the reference CSV's frame
headers. Raw RGB topic `/cmu_rc7/camera_1/image_raw` binds to export `cam_0`, which
is an export alias rather than proof of hardware camera numbering. Thermal is
`/thermal/image`; IMU is `/cmu_rc7/imu/data`, frame `epson`.

Every one of the 22,800 raw image header timestamps maps uniquely to exported
metadata. Applying the frozen grayscale/thermal preprocessing reproduces every
cached model-input NPY hash exactly. Thus this scoring consumes the same images
and nominal sensor header times as the locked predictions, not bag-arrival times
or a guessed sequence alias. The independent scoring auditor checks these binding
records against parent metadata; it does not independently redecode the raw images.

All 83,766 retained raw IMU samples match the supplied IMU CSV timestamps and
angular velocities exactly. The CSV contains two additional IMU samples at bag
splits; the 83,667 reference poses include those two timestamps. They are retained
and flagged, not synthesized or silently removed. Raw IMU also contains two gaps
of 25.008 and 30 ms, causing three half-second integration checks to be unsupported
under the unchanged 20 ms limit. The original exact-timestamp-equality gate
failure and subsequent explicit source-only qualification revision are preserved.
No model pose errors had been scored at that revision.

A no-fit gyro/reference check now supports the **observed rotational axes and
nominal timing**: integrate raw angular rates and compare directly with relative
reference rotations. On 237 supported intervals with ≥2° reference rotation,
median SO(3) discrepancy is 0.0385° and p95 is 0.1082°, inside the predeclared
0.5°/2° gates. No bias, axis or time shift was fitted. Motion is dominated by one
axis (rotation-excitation singular values 1.876, 0.0769, 0.0698), so this is not
rich all-axis calibration. IMU also contributes to the reference; this check
establishes consistency, not independence or physical origin metrology.

The thermal calibration's malformed zero last row is preserved in source. The
effective SE(3) uses the supplied upper3×4 and [0,0,0,1], with only tiny proper-
rotation roundoff normalization. Both rotation corrections pass the prospective
1e-5 Frobenius bound. No transform was estimated from these images or pose errors.

## Whole-span population and scoring

Interpolate reference position and quaternion at each cached image-header time,
with no extrapolation and ≤50 ms nonexact brackets. Compose camera world pose as
`T_WC=T_WI*T_IC`, then target-from-source as `inverse(T_WC1)*T_WC0`.
All 838 anchors remain declared. Anchor0 precedes reference startup; both cameras
are unscorable there. The frozen paired common-reference cohort has **837 anchors**.
There are **483 anchors** where both nominal camera translations are ≥1 cm, used
for the separate directed-translation/joint endpoints. No image or prediction
was regenerated and no rejected finite rotation was rescued.

Primary rotation success is a returned cached pose with full SO(3) error ≤1°.
The predeclared looser bound is ≤5°. Translation is direction only, not metric
scale. Joint success is R≤5° plus directed-t≤15° on the 483 eligible anchors.
Missing/rejected outputs are failures. Conditional medians describe returned poses
and must be read with availability and the full denominators.

## The useful tradeoff: availability versus precision

For a direct paired motion comparison, intersect the already-declared ≥2°
reference-angle strata across the two sensors' exposure times: **236 identical
anchors**. No new cutoff or favorable time window is selected; this remains an
exposed-data analysis of the locked predictions.

| Frontend | Input | Pose returned | Full R error≤1° | Full R error≤5° |
|---|---|---:|---:|---:|
| Learned | RGB | 221/236 (93.6%) | 151/236 (64.0%) | 181/236 (76.7%) |
| Learned | Thermal | 231/236 (97.9%) | 116/236 (49.2%) | 190/236 (80.5%) |
| SIFT | RGB | 162/236 (68.6%) | 24/236 (10.2%) | 85/236 (36.0%) |
| SIFT | Thermal | 134/236 (56.8%) | 60/236 (25.4%) | 100/236 (42.4%) |

Learned thermal returns ten more poses and supplies nine more ≤5° estimates,
but learned RGB supplies 35 more ≤1° estimates. Among returned poses, the ≤5°
fractions are nearly identical (RGB 181/221=81.9%, thermal 190/231=82.3%). Thus the
coarse-bound thermal advantage here is largely availability, while RGB is more
precise under the strict bound. For SIFT, thermal returns fewer poses but yields
more accurate rotation estimates in this moving cohort. The frontend matters.

A zero-motion predictor has zero ≤1° successes in this ≥2° cohort; its ≤5° counts
are RGB90 and thermal92, much lower than the learned estimates. Those separate
controls expose the difference between the strict and coarse bounds. Same-anchor
exposure times are not exactly simultaneous, and the native fields of view and
model training domains differ; this is a pipeline comparison, not intrinsic
sensor superiority or a causal smoke-transmission estimate.

## Whole mixed recording and fixed thirds

| Learned, 837 common-reference anchors | RGB | Thermal |
|---|---:|---:|
| Pose returned | 500/837 (59.7%) | 494/837 (59.0%) |
| R≤1° | 412/837 (49.2%) | 348/837 (41.6%) |
| R≤5° | 460/837 (55.0%) | 453/837 (54.1%) |
| Conditional median R error | 0.256° | 0.481° |
| Joint R≤5° / directed-t≤15°, translation-qualified | 349/483 (72.3%) | 250/483 (51.8%) |
| Conditional median directed-t error on eligible returned poses | 7.17° | 11.83° |

The fixed final third has 279 common-reference anchors. Learned R≤5° is RGB
169/279 (60.6%) versus thermal193/279 (69.2%), but R≤1° is RGB140/279 (50.2%) versus
thermal131/279 (47.0%). Nominal joint success on its 222 translation-qualified
anchors is RGB131/222 (59.0%) versus thermal101/222 (45.5%). The original ≤1°,
≤5° and translation endpoints all remain, with no threshold chosen to favor thermal.

The final third contains 41 thermal-only versus17 RGB-only ≤5° rotation successes,
but 32 thermal-only versus41 RGB-only ≤1° successes. At the nominal joint endpoint,
it has 32 thermal-only versus 62 RGB-only cases. These paired cells quantify
complementarity; their hindsight union is not a deployable fusion gain.

The entire recording includes long static/small-motion periods. Always returning
identity passes R≤1° on RGB561/837 (67.0%) and thermal560/837 (66.9%), and R≤5° on
691/837 and693/837. In the final third, identity passes R≤5° on193/279 for both
cameras, the same count as learned thermal. Therefore that coarse aggregate alone
does not establish informative motion tracking. The ≥2° motion stratum and strict
endpoint above are necessary context. All static segments remain; per-pair
essential-pose rejection is not a stateful VO failure/recovery result. No identity
fallback was inserted into either frozen model arm.

The mixed recording also includes clear and dynamic-person segments. Its thirds
are not independently labeled smoke-density groups. Neither this comparison nor
the previous global contrast bins isolate smoke from motion, illumination,
scene structure and camera/model characteristics. No zero-lux claim is made.

## Verification, decision and recovery

The independent auditor reconstructs the supplied CSV arrays, checks raw-IMU
sample binding, integrates quaternions independently, composes camera-relative
poses with rigid algebra, computes trace-based SO(3) and atan2-based directed-t
errors, and verifies cohorts, strata and counts: **45,488 checks passed**, plus 19 independent paired-motion/identity-control checks.
The auditor's first field-name mismatch was preserved and corrected without
changing data or scores. Source-timestamp and unsupported-gyro checks, plus a
NumPy Boolean serialization failure, are preserved with the metadata attempts.
These audits certify arithmetic/provenance, not independent reference truth.

This nominal contract supports supplied-reference relative-pose characterization.
Keep sensor-origin and fused-reference-dependence qualifications visible,
especially for translation. RGB is more precise in this setup; thermal can fill
missing/coarser rotation estimates. No general sensor winner or new mechanism is
established. Next finite baseline: RGB-first, thermal only when the frozen RGB
pose is unavailable, compared with both fixed inputs on the same cached cohort.
Prespecify that policy before measuring its result; do not choose thresholds or
switch using ground-truth errors. This is a baseline test, not a new selector or
full stateful VO system. No new model or training follows this result.

Evidence is ignored under `results/smoke-reference-contract/`, with source reads,
raw binding receipts, preserved failures, frozen reference/scoring manifests,
all 3,352 score rows, paired cells, controls and `rotation-availability-precision.png/.pdf`.
The recovery packet is `/root/ore-ms-artifacts/smoke-reference-contract-2026-11-16/`.
Raw sensor source ZIP remains under `data/deployment-characterization/`; the
packet contains reference/gyro/binding data and consumed cached predictions,
without duplicating the full raw sensor ZIP or checkpoints. No author outreach,
Box upload or Git publication occurred.

The prescribed status-only fallback is now [completed with cost and incorrect-output accounting](smoke-rgb-first-findings-2026-11-16.md). No parent scores, protocols or predictions changed.
