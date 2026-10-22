# Stage the first TartanDrive 3 sample

The GPU runtime, model weights, and first real clip are restored locally.
`1_salon_mac_cov_day.zip` is staged from Box and extracted under
`data/tartandrive3/20251027/1_salon_mac_cov_day/`. It contains the camera streams,
timestamps, intrinsics, TF, and SLAM inputs needed for initial checks; no further
full-date download is needed now. See
[experiment notes](experiments/) and [calibration findings](docs/calibration.md)
for measured findings and remaining compatibility issues. The instructions below describe how to stage more inputs
when needed.

## Download using your signed-in AirLab browser

Source: `/offroad/datasets/tartandrive3`, sequence `20251027` or `20251106`.
The supplied storage URL opens a Synology NAS login interface; this compute
session has no authenticated NAS session or mounted dataset.

Download one short contiguous segment containing about 10 synchronized captures
from all four cameras: RGB left/right and thermal left/right. Preserve the
original files, names, encoding, directory structure, and timestamps. Do not
convert thermal images to JPEG, screenshots, or display-normalized images.
If selecting a segment is difficult, use the smallest existing clip containing
all four streams rather than downloading a full collection.

Include:

- `timestamps.txt` for each stream, or the corresponding original timestamp
  metadata. Similar frame indices alone do not establish synchronization.
- Camera `_camera_info` / `_intrinsics` files, including image size, camera
  matrix, distortion model/coefficients, and projection/rectification matrices
  where supplied.
- The pinned `config/offroad/yamaha.yaml` from `castacks/static_tf_publisher`
  at commit `688c34472166f93b5129173030acccc96499b8fc`.
- Any separate stereo calibration/rectification files and camera frame names.
- The extracted `tf/` directory, including `metadata.yaml`, static transforms,
  and dynamic transform samples/timestamps spanning the chosen segment.
- Existing projected depth and reference poses with their timestamps, if
  available for this segment. These are needed for later error evaluation;
  image-only inference can establish execution but not accuracy.
- If depth/poses have not been generated, include the corresponding
  `super_odometry/pointcloud_in_odom/` files and timestamps for depth extraction,
  plus available odometry outputs. Static extrinsics alone cannot supply
  time-varying camera poses.

The user supplied the extrinsics YAML; it is now at
`data/calibration/yamaha.yaml`. All 13 transforms match archived values with
reversed frame-pair labels. Intrinsics come from `_camera_info` / `_intrinsics`
directories. Calibration sign/frame consistency still needs resolving; see
[the measured comparison](docs/calibration.md) before reference extraction.

## Generate depth and poses if missing

The inspected `castacks/ros_torch_converter` README documents this command,
run from that repository in its own configured environment:

```bash
python3 scripts/get_slam_data.py --dataset /path/to/extracted/clip \
  --depth --odom --seq_to 10 \
  --config config/kitti_config/get_slam_data_rgb.yaml \
           config/kitti_config/get_slam_data_thermal.yaml
```

This has not been executed here. It expects an extracted clip, including `tf/`;
it does not download or convert raw bags. `--seq_to 10` processes indices 0–9,
so this command assumes the first ten frames exist with the original indexing.
Arbitrary sliced clips need a checked index/timestamp mapping first.

The current configs select `sensors/image_left_rect` and
`sensors/thermal_left_processed`; both reference
`super_odometry/pointcloud_in_odom`. Outputs are `slam_depth_rgb`,
`slam_depth_thermal`, `camera_poses_rgb`, and `camera_poses_thermal`.
These are left-camera reference outputs; retain right-camera images and
calibration separately for stereo inference.

Before running, validate the config intrinsics, rectification, frames, and
transform-chain settings against the chosen recording. The extraction script
loads `tf/` but does not call `update_from_calib_config` automatically. The
demo shows that separate YAML update explicitly. `TfManager` can skip updates
to dynamic transforms or conflicting parents, so inspect its update messages.
The thermal config's `use_chain: True` is annotated for data extracted before
October 1, 2025; do not copy that choice to another extraction unchecked.

Camera poses come from the recorded TF/odometry, and depth is projected from
SLAM point clouds. Record their provenance as reference estimates rather than
assuming independent ground truth.

## Stage in your Box account

Upload the selected files under:

```text
ore-ms/tartandrive3/<sequence>/sample-001/
```

Include a short `source.txt` with the original NAS folder, sequence/clip name,
selected time interval, timestamp units if known, and whether the images are
raw or rectified. Keep the original folder names; no adapter-specific layout
is required yet. Do not include passwords, browser cookies, or OAuth configs.

Once uploaded, the compute-side staging command is:

```bash
cd /root/AirStack/ore-ms
rclone copy ore_ms:ore-ms/tartandrive3/<sequence>/sample-001 \
  data/tartandrive3/<sequence>/sample-001 --progress
```

Replace `<sequence>` with the actual sequence. If using a separate shared Box
folder, provide its link instead; the shared checkpoint folder needed explicit
shared-link resolution and was not visible at the account root.

## Checks before inference

Inspect file dtype/range and image dimensions; check timestamp ordering and
cross-camera offsets; identify camera intrinsics, distortion/rectification,
stereo baseline, frame conventions, and RGB-to-thermal extrinsics. Confirm
calibration applicability to this recording. Only then select and record a
thermal preprocessing recipe. Unknown preprocessing must remain labeled an
experimental choice rather than a reproduction of MACThermal training.

[EXP-025](experiments/EXP-025-salon-so-day-validation.md) stages the added
`20251027/0_salon_so_day`:1335 synchronized groups/~163s, no timestamp overlap
with earlier two recordings. Six RGB/MSA-thermal20-frame runs complete. Thermal
translation proxy RMSE0.185/0.198/0.084m; RGB0.381/0.122/0.130m, with RGB lower
rotation error in all3. Source camera metadata matches;120 old thermal replays
exact. GNSS-labelled data are present, but public converter maps their name
to `/odometry/filtered_odom`; upstream sensor/fusion/accuracy/timing provenance
requires confirmation. Keep SLAM-proxy interpretation and unchanged defaults.


[EXP-031](experiments/EXP-031-new-night-validation.md) stages the new `night_mac_salon_warehousefig8`:1,500 paired groups, no overlap with three prior recordings. Three fixed first-eligible20-frame windows,751 prepared hashes. All three MSA-thermal attempts and early RGB complete without interpolation; middle/late RGB retain0 matches and fail, with0 independent LK support. Preserve failures and unavailable errors. Thermal median spatial angular uncertainty/disagreement rho0.005/−0.223/0.529; operational availability differs from calibrated reliability.24 tests,600 identical image decodes,798 scalar/finite-difference checks,114 pairs/160,056 points and267 analysis hashes pass; prior729/402/97 hashes unchanged. Rebuilt GPU image f429af4afa23 has recorded transitive version differences; CPU analysis unchanged. Keep defaults/provisional calibration/reference gates. No more upload needed now: unused intervals of this archive suffice for a next bounded study. Box recovery `backups/2026-10-22-exp-031`.

User preference: explicitly flag when additional data would materially strengthen a study, and name the recording or conditions needed. Check existing/Box coverage before asking for an upload.
