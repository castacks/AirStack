# Supplied calibration and archived transform comparison

Inspected October 17, 2026 (America/New_York), on `ore_ms`, base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` plus the current audit implementation.
Dataset: `20251027/1_salon_mac_cov_day`.

## Sources and local paths

- Supplied extrinsics: `data/calibration/yamaha.yaml`, SHA-256
  `031c199619916bf1466a807d9ed91896d58b1428f0bdf74951dabc3c1b59e27d`.
  User-provided source pointer: [pinned yamaha.yaml](https://github.com/castacks/static_tf_publisher/blob/688c34472166f93b5129173030acccc96499b8fc/config/offroad/yamaha.yaml).
  The supplied file is inspected locally; its GitHub blob was not independently
  fetched through this connection.
- Recorded TF, intrinsics, and processed-camera R/P: original extracted clip
  under `data/tartandrive3/20251027/1_salon_mac_cov_day/`.
- Downloaded snapshots of [thermal processing config](https://github.com/castacks/ros_torch_converter/blob/main/config/kitti_config/get_thermal_data.yaml)
  and [processing script](https://github.com/castacks/ros_torch_converter/blob/main/scripts/get_thermal_data.py)
  are under `data/calibration/upstream/`. Fetched GitHub blob SHAs respectively
  `8279e675e47e9a55f6a7b84f322e297ab43eacba` and
  `b0b16f3aba8471f3e33ac5f363be59282b7c4e1a`.

## Verified comparison

All 13 YAML transforms match the archived numeric transforms, but in directories
whose frame pairs are reversed relative to the YAML `from_frame` / `to_frame`.
The numeric transform is unchanged: reversing a label alone does not establish
that the stored matrix should also be inverted. Current TfManager expects a
different schema and direction convention, so a blind reader substitution is
not justified. Original archived metadata and matrices remain unchanged.

The inspected thermal producer interprets its configured stereo matrix through
an inverse before calling OpenCV `stereoRectify`. Repeating that call with OpenCV
5.0.0 reproduces every saved left/right R/P element exactly. Passing the matrix
directly changes the projection baseline sign and fails to reproduce those
matrices (right P maximum absolute difference about 232.44).

This identifies how the saved calibration was generated, not whether it is
physically correct. YAML-based body extrinsics still disagree with the thermal
projection geometry: horizontal baseline residual about +0.67011 m, off-axis
translation about 0.00614 m, relative rectification rotation about 0.617°.
Baseline magnitude alone cannot resolve those discrepancies.

## Image diagnostic

Three frames (0, 5, 9) were checked with SIFT, mutual ratio matches (0.7), and
fundamental-matrix RANSAC (1.5 px, confidence 0.999, seed 0). Raw thermal
rectification variants used declared per-image 1st/99th percentile clipping,
uint8 conversion, and CLAHE solely for feature matching; originals were not
overwritten. Different variants need not retain the same feature correspondences.

| Image input | Median absolute vertical feature residual, range over 3 frames |
|---|---:|
| Provided processed thermal | 0.581–0.593 px |
| Raw diagnostic, published inverse matrix | 0.586–0.634 px |
| Raw diagnostic, matrix directly | 1.892–1.936 px |

Both RGB and provided thermal matches have positive median left-minus-right
horizontal disparity on these frames. The saved thermal P implies a negative
baseline under the usual stereo projection interpretation. These diagnostics
do not support replacing the supplied processed images with the direct-matrix
variant. Signed depth, frame conventions, and reference-pose interpretation
remain unvalidated; no full VO accuracy or ground-truth claim is made.

## Reproduction and artifacts

From `MAC-VO-main/` inside the existing project runtime:

```bash
python Scripts/Setup/audit_td3_geometry.py \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --calibration ../data/calibration/yamaha.yaml \
  --thermal-config ../data/calibration/upstream/get_thermal_data.yaml \
  --output ../results/data-access/geometry-with-yamaha.json
```

Local ignored artifacts: `results/data-access/geometry-with-yamaha.json`,
`epipolar-diagnostic.json`, its diagnostic script `check_epipolar.py`, and
`layout-migration.json`. The latter maps historical paths to the current layout;
historical experiment logs retain the paths used at their execution time.
