# EXP-028: Fixed-window reference and timestamp audit

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `46784eb6e` plus frozen diagnostic source. **The late-night disagreement is also visible between available odometry streams: filtered odometry reports0.0161m displacement while SLAM reports0.2608m at the vehicle origin. Mux and teleop command samples are all zero. This strengthens a reference-disagreement interpretation; it does not establish physical ground truth, source independence or a corrected thermal pose. All VO settings and previous outputs remain unchanged.**

## Frozen scope and source checks

Use the exact five EXP-027 windows: night early/middle/late, day-late moving and the first20 frames of the near-static day control. All19 consecutive pairs per window and original LK-valid masks stay fixed. No additional image acquisition, model inference, new window selection, offset fitting, scale fitting or optimizer intervention. Freeze402 scientific input/helper/test files before analysis and verify again afterward.

Recover52 bounded metadata/tabular members from `night_thermal_so_salon2.zip` using authenticated byte ranges:20,834,778 uncompressed bytes. Every selected member passes ZIP CRC and SHA256. The7.71GB whole archive is not downloaded or rehashed. New files are staged separately under `results/reference-timing/inputs/night`; original data/results are untouched. Inputs include source-rate and binding-grid odometry, TF, original thermal timestamp lists, IMU, command logs, recording audit and conversion statistics.

The [pinned public OdomRBState exporter](https://github.com/castacks/ros_torch_converter/blob/01bbd8dd7b0d9f5fdfceb0a78319a34e640c4887/ros_torch_converter/datatypes/rb_state.py) writes the source sample list through `to_interp` into `interp_data.txt` / `interp_timestamps.txt`. `from_interp` evaluates this series at target times, and the binding-grid export is saved to `data.txt`. Therefore the existing high-rate SLAM series is not established to be resampled merely because its filename says `interp`. The exact code/config revision used to produce these archives remains unverified; public source provides schema and provenance clues, not proof of a reproduced raw-bag conversion. Timestamp decimals retain int64 nanosecond precision in this audit.

Night `conversion_stats.json` names `/superodometry/integrated_to_init` with20,464 source samples, `/odometry/filtered_odom` with10,168 samples and `/novatel/imu/data` with19,436 samples. The [public NovAtel topic config](https://github.com/castacks/ros_torch_converter/blob/01bbd8dd7b0d9f5fdfceb0a78319a34e640c4887/config/topics/yamaha/novatel_topics.yaml) maps the directory label `novatel_gps_odom` to `/odometry/filtered_odom`. The archive also records that topic's10,168 samples and1,952 exported rows. This does not identify the upstream estimator, fused sensors, GNSS fix/quality or whether it shares SLAM inputs. No covariance or fix-quality fields are present in the13-column pose/twist export. Public schema/code is not certification of archive numeric or sensor accuracy.

## Reference extraction and exact timestamp controls

All1,952 supported binding-grid SLAM poses reproduce the source-rate series to numerical precision: maximum position difference3.56e-14m and rotation difference2.06e-14deg. Recorded vehicle TF has1,919 samples:1,916 inside the SLAM source support match to maximum4.50e-14m /2.46e-14deg; three outside support are explicitly excluded without extrapolation. These are consistency checks within the same SLAM source, not independent validation. Source time gaps have median0.01164s and maximum0.04087s.

All five saved camera-proxy arrays reconstruct using the original nominal body anchor and each window's own rectification. Maximum translation/rotation residuals are below4.4e-14m /5e-15deg. Image rows, VO trajectories, saved proxy arrays and each LK pair's two timestamps agree exactly. All120 night left/right processed-image indices match their acquisition timestamp files and have one exact raw timestamp match each; no ambiguity is repaired in this audit. This does not verify hardware clock alignment or raw-bag acquisition freshness.

## Motion context in the three night windows

| Fixed night window | SLAM vehicle-origin net displacement (m) | Filtered gps_frame-origin net displacement (m) | SLAM supplied linear-speed median (m/s) | Filtered supplied linear-speed median (m/s) |
|---|---:|---:|---:|---:|
| Early |1.7622|1.4298|0.6821|0.7148|
| Middle |6.1709|5.5887|2.8958|3.0735|
| Late |0.2608|0.0161|0.1030|0.0097|

Endpoints use each stream's source-rate poses evaluated at exact thermal acquisition times. Source metadata is `sensor_init -> vehicle` for SLAM and `map -> gps_frame` for the auxiliary stream. These are displacements from **different supplied origins**, without a shared-body alignment or fitted transform. Do not subtract the two pose vectors, score one stream against the other as ground truth, or interpret agreement in moving windows as a calibration certificate. Supplied twist values are described, not integrated to replace the poses.

In the late window, all57 `/controls/command` source samples and all38 teleop samples have exactly zero linear-x and angular-z requests. All19 auto-command samples are nonzero; their median linear-x request is0.9997m/s. These are different command streams, and requests do not measure motion; no mux/override mechanism is inferred. The97 filtered-odometry samples over this interval have median speed0.0097m/s. The192 IMU samples have median angular-speed norm0.0169rad/s and acceleration norm9.742m/s²; gravity is not removed, and neither IMU integration nor a standstill classification is attempted.

The exact late interval remains **2025-10-27 22:13:40.207–22:13:42.107 EDT**, recording `20251027/night_thermal_so_salon2`. Thermal VO camera-origin displacement is0.0085m; nominal camera-origin SLAM displacement is0.2610m. EXP-027 independently measured only0.053px median LK motion in these images. The extra stream and commands are compatible with low motion and make model-alone visible-motion suppression a weaker diagnosis; physical motion and the source of disagreement remain unconfirmed.

## Predefined reference-time offsets

Query the unchanged reference at image times plus each fixed offset `{-2,-1,-0.5,-0.25,-0.1,0,0.1,0.25,0.5,1,2}` seconds. Keep the same images, LK endpoints, LK-valid masks, predicted source depths and intrinsics. Never clip or extrapolate. All55 window/offset combinations have full reference support. Preserve every curve and per-pair count; no best offset is selected and no timing correction is adopted.

| Fixed window | Zero-offset LK–SLAM/depth endpoint error (px) | Zero-offset LK Sampson error (px) |
|---|---:|---:|
| Night early |2.367|0.719|
| Night middle |7.011|1.827|
| Night late |0.925|0.028|
| Day late moving |1.266|0.460|
| Day near-static |0.280|0.062|

Values are unweighted medians of19 per-pair medians on original LK-valid points; projection-invalid counts remain explicit. Sampson uses LK endpoints and is depth-free; translation<=1e-6m is formally degenerate. A small epipolar residual during tiny motion cannot validate translation magnitude or predicted depth.

| Late-night reference offset (s) | LK–SLAM/depth endpoint error (px) | SLAM camera-origin net displacement (m) |
|---:|---:|---:|
|-2.00|1.158|0.1803|
|-1.00|0.922|0.1763|
|-0.50|0.906|0.1421|
|-0.25|1.167|0.2279|
|-0.10|1.135|0.2564|
|0.00|0.925|0.2610|
|0.10|0.939|0.2551|
|0.25|1.059|0.2322|
|0.50|0.966|0.2207|
|1.00|1.073|0.1312|
|2.00|0.837|0.1663|

The fixed +/-0.1s changes leave the discrepancy visible; the full prescribed offsets do not eliminate it. This rejects neither arbitrary clock distortions nor other reference/depth/calibration errors, and is not an estimate of actual clock offset. Other windows have different sensitivity curves; all55 summaries and1,045 pair/offset rows are saved. No sensitivity outcome becomes a correction.

## Validation and artifacts

Eleven focused tests pass: four new tests cover precise timestamps/holes/duplicate indices, shifted reference support without extrapolation, known binding-grid interpolation with invalid rows and synthetic shifted image projection with depth-free/degenerate epipolar controls; seven EXP-027 geometric/tracking tests remain passing.

Independent quaternion SLERP/matrix composition, optical-frame projection, OpenCV Sampson, source/TF interpolation, auxiliary endpoints and exact command counts pass on1,045 pair/offset cases and882,684 LK-valid projection scores plus882,684 OpenCV Sampson scores. All402 frozen hashes pass after analysis. Independent zero-camera matrix differences are <=8.53e-14; maximum per-pair median projection difference is1.78e-5px. Independent optical algebra and the preserved float32 backprojection differ by at most3.53e-5px at zero offset; a1e-4px numeric validation tolerance accommodates this rounding without changing scientific criteria or outputs.

Initial validator failures are retained: saved K has singleton batch dimensions (fixed in the independent validator reshape), then equivalent optical/float32 backprojection exceeded an overly strict1e-6px control tolerance. The1e-4px tolerance only affects numeric verification. An initial plot import failed before rendering and was rerun with explicit PYTHONPATH. No scientific input, mask, output, uncertainty, pose, calibration or timing rule changed to pass these checks.

Artifacts: `results/reference-timing/` contains `acquisition-plan.json`, selected `inputs/night/`, `metadata-extraction.json`, frozen `run-001/plan.json`, `analysis.json`, full `offsets.csv` / `pairs.csv`, independent `validation.json`, PNG/PDF figure and exact scripts/commands/logs/runtime inventory under `staging/`. Runtime: rebuilt `macvo:geometry`, OpenCV4.10.0.84/NumPy2.2.6; actual image ID and package inventory are recorded. No GPU operation.

Replay from `MAC-VO-main` with ore-ms mounted at `/workspace`:

```bash
python -m Scripts.Setup.td3_reference_timing prepare --output /workspace/results/reference-timing/new-run
python -m Scripts.Setup.td3_reference_timing run --output /workspace/results/reference-timing/new-run
python -m pytest -o addopts= Scripts/UnitTest/test_td3_reference_timing.py Scripts/UnitTest/test_td3_low_motion.py Scripts/UnitTest/test_td3_night_geometry.py
```

Prepare requires restored EXP-023/026/027 context, selected new metadata/upstream source snapshots and a new output directory. Box recovery `ore-ms/backups/2026-10-21-exp-028` includes new source/tests/report, frozen plans, selected metadata, diagnostics, validation, figure, execution records and incremental Git bundle. Earlier code/source/calibration/model snapshots remain prerequisites; credentials, checkpoints, full source archives and disposable range blocks are excluded.

## Decision and next evidence

Keep MSA as the experimental thermal baseline and all VO/weighting defaults unchanged. No indexed timestamp association or reference-export mismatch is found in the checked files. The evidence now supports describing this late window as **disagreement among motion references during apparent low image motion**, not as established thermal-network failure or a calibrated pose-error target. SLAM drift, auxiliary-estimator behavior, clock alignment, body/calibration geometry and depth validity are not individually resolved.

The next targeted source question is: for the exact interval above, was the vehicle stopped/creeping; what produces `/odometry/filtered_odom`, which sensors/SLAM inputs does it consume; and where are its timing, quality/covariance and `gps_frame` extrinsic documented? Obtain that provenance or an independently justified reference before interpreting this window as physical VO error or training/calibrating a fusion rule from it. No message has been sent to a dataset owner.
