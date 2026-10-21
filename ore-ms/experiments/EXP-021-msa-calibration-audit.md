# EXP-021: Supplied MSA thermal calibration passes fixed image checks

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `3bfce192f` plus the frozen audit source. **A newly supplied source calibration passes all 24 diagnostic image checks without fitting. Applicability to the October recordings and metric/body accuracy remain unconfirmed; no VO calibration or defaults changed.**

The user supplied `msa_cal.zip` from NAS `offroad/calibration/msa_cal`. Its six members are two thermal intrinsic YAMLs, extrinsics YAML/URDF, and two display scripts. No JPEGs, calibration targets, time-offset file or internal calibration binary are in this ZIP. The screenshot separately lists four camera JPEGs; their contents have not been inspected. Bundled scripts were read as reference and not executed. Source comments identify Main Street Autonomy Calibration Anywhere, software `651fb91d6ea338d86cf0418740f1029c93e9f293`, log `CMAIR.Viking.Vicky.2025-01-26.15-13-39.01`. This is source provenance, not proof of unchanged mounting through October. Archive/member SHA256 values are preserved in `data/calibration/msa_cal/manifest.json` and recovery.

## What differs

Both cameras are 640x512 and explicitly **equidistant**, with four fisheye coefficients plus a trailing zero. The bundled reference code uses `cv2.fisheye` with the first four coefficients. The current producer uses different K and four radtan coefficients. These lens models cannot be interchanged by copying coefficient arrays or changing a model label. The bundled display script performs monocular undistortion with half focal length for viewing; it does not provide stereo rectification. Its output settings were not used in the audit.

MSA left K is fx413.301157, fy413.566967, cx314.907248, cy250.531212; right K is fx413.006442, fy413.318268, cx313.797659, cy253.403456. The files also record readout times about20.36ms left/19.45ms right; these values were not applied as timing corrections or independently verified. No inference about hardware shutter type is made from them.

The common-parent extrinsics explicitly identify `velodyne_1_velodyne` and xyzw quaternions. Taking parent-from-child poses, `T_right_from_left = inverse(T_parent_from_right) @ T_parent_from_left`. The resulting translation is [-0.326169286,-0.000924079,+0.003213239]m, baseline norm **0.326186422m**, and relative rotation magnitude0.444918deg. The old stored stereo translation norm is about0.335113m. No metric scale is recovered from image correspondences. YAML and URDF thermal poses agree to <4e-17m and <0.00033deg, consistent with serialized rounding. `velodyne_1_velodyne` is not silently equated with the vehicle frame; no new vehicle-body anchor is claimed.

## Fixed-observation diagnostic

Reuse all24 original EXP-020 pair sets: 12 Stage-A cases and12 Stage-B validation cases, 3,147 original SIFT/F-RANSAC inliers total. These images were already examined; they are not new held-out validation. No MSA parameter was fitted, no threshold was tuned, no candidate-specific rematching was performed, and no correspondences were discarded based on new cheirality. Original raw K/D/R/P assumptions and extraction provenance remain those recorded in EXP-017–020.

For each fixed processed-image correspondence, invert original R/P to its old raw undistorted ray, forward-project through the original radtan lens model to locate its raw distorted sensor pixel, then interpret that same sensor pixel with MSA fisheye K/D. Simply reusing the old normalized rays would incorrectly retain the old lens model. All3,147 pixels in each camera are within the640x512 sensor bounds. Roundtrip checks pass at <1e-8 in normalized-ray/raw-pixel coordinates.

Triangulate with the derived right-from-left transform. Keep finite points, positive-depth flags, all residuals and parallax; eligibility is the unchanged0.1deg minimum. Sampson residual is expressed in undistorted raw pixel units using each candidate's K. Vertical error uses MSA fisheye stereoRectify with zero-disparity,balance0,fov_scale1; because output focal lengths differ between calibrations, Sampson residual is the primary alignment comparison and rectified pixel error is secondary.

| Diagnostic over24 pairs | MSA source result |
|---|---:|
| Original correspondences | 3,147 |
| Eligible matches per pair | 47–304 |
| Positive depth among eligible, pairwise range | 98.55–100% |
| Median Sampson residual, pairwise range | 0.113–0.322px |
| Median vertical residual, pairwise range | 0.134–0.380px |
| Sampson ratio to better producer/as-stored control | 0.176–0.709 |
| Pairs with >=20 eligible, >=95% positive, residual ratio<=0.75 | 24/24 |

All24 pass the **per-image** numerical thresholds retained from EXP-020. The earlier fitted-candidate stability gate is not applied to this externally supplied, unfitted calibration; there are no training-only fits to compare. This is stronger image support than the rejected corrections, but it is not an independent metric/body calibration certificate. Feature selection remains inherited from F-RANSAC on originally processed images, and both recordings are the same October2025 day/night sequences used before.

## Verification and artifacts

Docker `macvo:geometry` with OpenCV4.10.0:10 relevant tests pass, including three new checks for lens-model conversion and incompatible-model rejection. Independent analytic radtan/fisheye forward projections, NumPy SVD triangulation/cheirality, atan2 parallax and OpenCV scalar Sampson checks pass on all3,147 pairs. All228/269 prior frozen hashes and34 new frozen inputs/source hashes pass. A namespace handling error in the initial URDF read was corrected before numeric outputs; empty run-001 remains, successful audit is run-002. The comparison figure was inspected.

Artifacts under `results/msa-calibration/`: `run-002/analysis.json` and per-case arrays, `validation.json`, `calibration-comparison.png/.pdf`, and `source-calibration.yaml`. Export status is `SOURCE_CALIBRATION_IMAGE_CHECKS_PASS_APPLICABILITY_UNCONFIRMED`. It includes derived stereo R/P/Q and source K/D but is not installed in the VO configuration. Existing processed images, depths, camera-info, checkpoints and VO outputs are untouched.

Replay in the pinned CPU Docker image with ore-ms mounted at `/workspace` and working directory `/workspace/MAC-VO-main`:

```bash
python -m Scripts.Setup.td3_msa_calibration_audit --output /workspace/results/msa-calibration/new-run
python -m pytest -o addopts= Scripts/UnitTest/test_td3_calibration_reconcile.py Scripts/UnitTest/test_td3_calibration_refine.py Scripts/UnitTest/test_td3_msa_calibration_audit.py
```

Recovery preserves the source ZIP/members, new source/tests/report, diagnostic outputs, validator/plot/export scripts, notebook and incremental Git bundle under Box `ore-ms/backups/2026-10-21-exp-021`. Reproduction also requires prior EXP-020 recovery and its inherited inputs. No credentials or model weights are added.

## Next use

This removes the lack of a credible source calibration as the immediate obstacle. Next, inspect the separately listed thermal JPEG pair and any acquisition/applicability notes, then create separate raw-image rectification/depth/VO results using this source while preserving the original baseline. Existing saved model depths and processed camera-info cannot just be relabeled with MSA R/P. The new source supports a controlled downstream experiment, not retrospective correction of old metric results or a claim that all calibration uncertainty is resolved.
