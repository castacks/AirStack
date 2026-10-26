# EXP-055: Can existing MS² data support a shared-scene RGB/thermal comparison?

Date: 2026-10-26 America/New_York. Branch `ore_ms`; starting commit `d38296885`.

**Decision: existing data supports a bounded cross-camera correspondence pilot under the supplied calibration, but does not yet provide verified shared-point evaluation support.** No additional image download or CATS access is necessary to test that pilot. The next step is correspondence/visibility validation on the fixed existing pairs, followed by a comparison in a common geometric quantity. Separate camera risk ratios alone cannot answer which modality is more reliable.

## Scope and evidence

Use all 20 inherited EXP-054 rain IDs `000000–000019`, both primary single-scan depth maps, their decoded calibration and original left timestamps. No score-based interval selection, new inference, clock correction, pose-direction choice, fitting, calibration change or RGB/thermal winner scoring. This audit does not read learned flow/covariance arrays. Rain was selected because its bounded inputs are already restored, not because its cross-modal result is known. The recording has already been studied and checkpoint training membership is unknown.

The supplied archive readme explicitly names the NIR-left→RGB-left and NIR-left→thermal-left transformations, with translations in millimetres. The official [data format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format) describes rectified-camera calibration, projected left-camera depth, sensor extrinsics and sensor-coordinate poses. The official [setup](https://sites.google.com/view/multi-spectral-stereo-dataset/setup) describes rectification/cropping and navigation-based sensor pose estimation for depth generation. Those documents permit a supplied-calibration geometric candidate calculation; they do not themselves certify each candidate's visibility or identity.

For documented actions `X_rgb = R_nr X_nir + t_nr` and `X_thr = R_nt X_nir + t_nt`, compose:

```
R_thr_from_rgb = R_nt R_nrᵀ
 t_thr_from_rgb = (t_nt - R_thr_from_rgb t_nr) / 1000
X_thr = R_thr_from_rgb X_rgb + t_thr_from_rgb
```

The resulting translation is approximately `(0.01146, 0.32978, 0.56823)` m, with camera-centre separation 0.65709 m. Both component rotations have determinant 1 and maximum orthogonality error below 3e-16. Inverse composition and pixel backproject/project roundtrips pass. These algebraic properties do not validate physical calibration or rectification-frame applicability. Zero left-camera transforms in each separate stereo pair do not mean RGB-left and thermal-left are co-located.

The supplied poses are sequence-initialized separately by sensor; they cannot simply be treated as a shared global frame. This audit uses no pose matrices and performs no motion compensation. That is an explicit supplied-pair diagnostic, not proof that a fixed rig transform describes the same moving-scene point at different times.

Two open repository reports describe difficulties with [cross-camera reprojection (#13)](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/issues/13) and [NIR/thermal calibration (#15)](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/issues/15). As inspected on 2026-10-26, neither page displayed a maintainer resolution. These are user reports, not a finding that the calibration is wrong. No alternate transform is selected or fitted from them.

## Quantitative candidate audit

Backproject every nonzero single-scan source depth pixel with its own intrinsics, apply the documented cross-camera transform, and retain positive-Z projections inside the target image. Query the nearest nonzero target depth pixel. Record candidate counts at fixed radii 0.5, 1 and 2 target pixels; no radius is accepted as a physical correspondence criterion based on this audit. Many sources can map near the same target pixel; all per-image unique-target counts are retained in the report.

Median in-FOV fractions of source-supported points are 100% RGB→thermal and 64.1% thermal→RGB. The asymmetry reflects camera geometry and sparse support, not modality reliability. Radius values are in the target camera's pixels, so the two directions' sampling tolerances are not equivalent angular tolerances.

Every table entry is a median across the 20 per-image summaries. Depth discrepancy is the absolute difference between transformed source Z and the nearest target label's Z, measured in the **target** camera frame; it is not Euclidean point distance or absolute calibration accuracy. Relative discrepancy divides by that target label's Z. Candidate fractions use all nonzero source depth pixels as denominator.

| Direction | Radius (target px) | Candidate fraction | Depth discrepancy (m) | Relative depth discrepancy |
|---|---:|---:|---:|---:|
| rgb → thr | 0.5 | 35.0% | 0.0584 | 0.42% |
| rgb → thr | 1 | 85.4% | 0.0583 | 0.46% |
| rgb → thr | 2 | 99.7% | 0.0582 | 0.51% |
| thr → rgb | 0.5 | 5.3% | 0.0574 | 0.45% |
| thr → rgb | 1 | 20.8% | 0.0576 | 0.45% |
| thr → rgb | 2 | 53.4% | 0.0579 | 0.50% |

At 1 target pixel the median candidate fraction is 85.4% RGB→thermal and 20.8% thermal→RGB; depth discrepancy medians are about 5.8 cm in both directions. These support investigating overlap, not treating nearest labels as verified common physical points. Both products share acquisition/calibration/navigation processing, so their consistency is not independent geometric truth. Fixed candidate-radius curves are diagnostics; no agreement threshold or favorable subset is adopted.

Recorded left-camera absolute time separation is 0.212 / 4.252 / 61.537 ms (min / median / max). These integers are verified against all 40 selected source-manifest timestamps. They are not measured exposure delays or an accuracy bound. Retain every pair; do not impose the inherited 30 ms metadata gate as a universal MS² benchmark acceptance criterion. Follow EXP-050: alignment matters to the precision claimed for a physical shared-point comparison, while ordinary provided-pair benchmarking can proceed.

## What prevents declaring shared points today?

- The staged depth PNGs do not preserve raw LiDAR point IDs. A nearby label with similar depth is only a candidate for the same surface. The two projections can have different sampling, occlusion or time association.
- This audit does not perform bidirectional pixel/depth consistency filtering, a z-buffer visibility test, duplicate resolution, dynamic-object handling or image-based correspondence inspection. Bidirectional **projection statistics** are not mutual-match validation.
- Rectification/cropping applicability has documentary support but has not been independently checked with cross-camera correspondences here. Inverse/roundtrip arithmetic cannot prove it.
- Shared-reference consistency cannot establish uncertainty calibration, independence or a physical RGB/thermal winner. Model outcomes were deliberately excluded from this support audit.

These are specific requirements for the proposed shared-point estimand, not reasons to suspend all current MS² research or request more calibration details from Geethika.

## Define the common target before scoring

Use the same candidate scene point visible in both **left** cameras, and evaluate each modality's reconstructed point in a common metric frame, such as RGB-left coordinates. Primary error can be Euclidean distance between that modality's predicted 3D point and the common supplied-reference point. Each camera's stereo prediction still uses its own original image pair and calibration. This would be a supplied-reference benchmark, not independent physical truth.

Raw flow covariance trace or focal-normalized trace cannot be compared directly to metric point error. For a fixed left ray `q = ((u-cx)/fx, (v-cy)/fy, 1)`, disparity `d = -flow_u` and baseline B:

```
Z_hat = fx B / d
X_hat = q Z_hat
J = -q fx B / d²                 # derivative with respect to disparity
C_X ≈ J Var(flow_u) Jᵀ          # depth-only first-order propagated uncertainty
```

Rigidly rotate this covariance into the common frame. Its trace has units m² and is a candidate predictor of squared 3D point error. This is a first-order, rank-one depth uncertainty approximation with fixed pixel/ray/calibration, not a full 3D observation covariance or calibrated confidence interval. Vertical flow disagreement remains a separate stereo-consistency diagnostic; silently adding vertical flow variance to depth variance is unjustified. Rejecting or retaining invalid disparity predictions must be specified as an outcome/availability policy before scoring, and their counts must remain visible.

The supplied common-reference point must be constructed independently of learned uncertainty and error. Agreement of the two projected reference products may screen their consistency but does not identify which is true. Choosing whichever reference makes a model look better would invalidate the comparison.

## Necessary next experiment, with bounded scope

On these same 20 pairs, freeze a candidate-support protocol before reading RGB/thermal winner scores. Validate roundtrip pixel/depth consistency and visibility, count every rejected/ambiguous/duplicate candidate, and inspect a fixed set of cross-camera overlays selected by source ID. Do not tune transform, latency, radius or depth tolerance using model performance. Sparse-depth visibility limitations must be retained; numerical mutual agreement alone does not certify a hidden surface is visible.

If that support check is credible, freeze the common metric target, prediction-availability handling and a simple baseline such as always RGB/always thermal on exactly the same support, then test whether the frozen uncertainty rule chooses the lower-error modality. Any fitted calibration needs a recording-disjoint evaluation protocol rather than fitting and testing on these inspected rain frames. No fusion or VO change follows automatically from a successful diagnostic.

If correspondence cannot be justified, report the concrete failure and retain the established within-camera finding. Do not substitute unmatched per-camera image medians for a physical same-point winner. CATS remains complementary independent-reference work when access arrives; training membership matters before training-held-out claims.

## Implementation, checks and replay

Script: `MAC-VO-main/Scripts/Setup/ms2_cross_modal_feasibility.py`. Output: `results/ms2-cross-modal-feasibility/run-002/audit.json`. Runtime: isolated Python 3.12.3, NumPy 2.2.6, SciPy 1.15.3, Pillow 11.3.0. No GPU or learned model library is needed.

All 46 frozen files match EXP-054 recovery member hashes before calculation and remain unchanged after it. Rotation, geometric inverse and pixel roundtrip checks pass across all 40 directional image cases. Independent scalar arithmetic checks 120 sampled projections and brute-force distances check 120 nearest sparse-label queries. All 40 selected timestamps match their original source-manifest rows. These are implementation/preservation checks, not 240 independent observations or physical calibration validation.

The initial `run-001` mistakenly treated integer-nanosecond timestamps as seconds. It is retained with an `INVALIDATED.txt` notice and must not be used. `run-002` corrects the parser and verifies timestamp units/joins against the source manifest. Geometry results are unchanged. Environment creation also first failed because ensurepip was unavailable; dependencies were then installed into an isolated environment with pip's `--python` option. No system dependencies were replaced.

```
python MAC-VO-main/Scripts/Setup/ms2_cross_modal_feasibility.py \
  --root /root/AirStack/ore-ms \
  --output /path/to/fresh-audit \
  --recovery-manifest /root/ore-ms-recovery/2026-10-25-exp-054/manifest.json
```

The script refuses existing output paths. Restore the bounded EXP-054 inputs and manifest, not full transport archives or checkpoints. The audit JSON records exact transforms, source hashes, script hash, timing values and all per-image counts/distributions. Neither the original scientific arrays nor model/default settings were changed.
