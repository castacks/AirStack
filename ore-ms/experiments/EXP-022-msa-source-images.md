# EXP-022: MSA source thermal JPEG pair

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `03a102194` plus frozen source. **The separately supplied thermal JPEG pair passes the source-calibration image checks, without fitting. Original pipeline calibration and outputs remain unchanged.**

## Source and scope

The user supplied `camera_images.zip` from NAS `offroad/calibration/msa_cal/camera_images`. It contains only `thermal_left.jpg` and `thermal_right.jpg`. Both decode to640x512,uint8,three channels. The originals show a natural outdoor scene with vegetation/trees and foreground vehicle structure; no calibration target is visible. Images were inspected before analysis. Archive/member SHA256 values are stored in `data/calibration/msa_camera_images/manifest.json`; the archive and members are preserved in recovery. JPEG intensity processing, capture timestamps and synchronization are unknown. Do not infer them from filesystem/ZIP modification times. These are new observations for our audit, but may have come from the source calibrator's own data, so they are not a claim of independent calibration validation or new October recording coverage.

Use the supplied MSA source exactly as in EXP-021: equidistant intrinsics, common-LiDAR-parent extrinsics, T_right_from_left and0.326186422m nominal baseline. No parameter is estimated from this pair. No vehicle anchor, metric scale or timing correction is inferred.

## Fixed protocol and results

Detect/match once on the original JPEGs: OpenCV4.10.0 SIFT2000, bidirectional mutual Lowe ratio0.7, F-RANSAC1.5px/.999 and seed0. There are760/784 left/right keypoints,242 mutual matches and214 F-RANSAC inliers. The same raw sensor pixel pairs are interpreted under MSA fisheye or original radtan intrinsics for each calibration candidate. Candidate-specific rematching, tuning, positive-depth filtering and depth clamping are not used.

Evaluate both all242 mutual matches and the214 original F inliers. Populations overlap; do not count456 as unique observations. Triangulation eligibility retains finite geometry and parallax>=0.1deg. Alignment is primarily compared by Sampson residual in undistorted raw pixel units; stereo-rectified vertical pixels are secondary because projection focal lengths differ.

| Population | Eligible | Positive eligible depth | MSA median Sampson | MSA median vertical | Sampson ratio to better original control |
|---|---:|---:|---:|---:|---:|
| All mutual |233|232/233 (99.57%)|0.220px|0.259px|0.384|
| F inliers |210|210/210 (100%)|0.234px|0.276px|0.453|

Both populations pass the retained per-image checks: >=20 eligible, >=95% positive depth and median Sampson ratio<=0.75. On the filtered population the producer-inverse control has90.19% positive eligible depth,0.517px median Sampson and0.620px median vertical mismatch. The old as-stored control has100% positive eligible depth but1.050px median Sampson and1.242px median vertical mismatch. This again illustrates why positive depth alone is insufficient.

MSA median rectified signed disparity is5.074px on the filtered population. No ground-truth depth is available, so no metric depth accuracy is asserted. The rectification preview was inspected with shared horizontal guides; it is a visual supplement to correspondence measurements.

## Verification and artifacts

Independent analytic fisheye forward projection reproduces original JPEG feature pixels. NumPy SVD DLT triangulation, cheirality, atan2 parallax, scalar OpenCV Sampson and separately composed R/P projections agree with saved arrays. Original mutual/inlier subset identities are exact. All13 new frozen source/input hashes and34 EXP-021 hashes pass. Ten relevant existing geometry/lens-model tests pass. No new unit tests were added for this bounded diagnostic; numeric validation covers the actual outputs.

Artifacts: `results/msa-source-images/run-001/analysis.json`, `matches.npz`, `all_mutual.npz`, `fundamental_inliers.npz`, two rectified PNGs and `stereo-preview.png`; independent `validation.json`, scripts/logs/image-id under `staging/`. No source image, original calibration, camera-info, model depth or VO result was overwritten.

Replay in `macvo:geometry`, ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`:

```bash
python -m Scripts.Setup.td3_msa_source_images --output /workspace/results/msa-source-images/new-run
```

Requires MSA source files and original radtan config from earlier recovery, plus the new JPEG archive. Derived source calibration from EXP-021 remains explicitly unadopted. Code/report, source ZIP/JPEGs, outputs/validation, notebook and incremental Git bundle are preserved under Box `ore-ms/backups/2026-10-21-exp-022`.

## Interpretation

Together with EXP-021's24 driving-image checks, this source pair supports using MSA in a separately saved raw-preprocessing/depth/VO experiment. It does not establish unchanged mounting through October, physical metric accuracy, the vehicle-body anchor, synchronization or improved VO. No further source data is required to begin that controlled image-pipeline experiment; calibration-target/applicability evidence would still strengthen physical validation. Existing processed images/depths cannot merely be relabeled with the new calibration.
