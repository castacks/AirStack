# EXP-025: Additional `0_salon_so_day` recording

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `9961a9971` plus frozen validation source. **The added recording is usable, has no timestamp overlap with the earlier two recordings, and supports a bounded RGB/MSA-thermal replay. All six20-frame attempts complete without interpolation. This expands capture coverage, not independent ground-truth coverage. No production defaults changed.**

## Input preflight

Box `0_salon_so_day.zip` is7,765,177,640 bytes, with16,541 ZIP entries and source `/media/airlab/offroad/rosbags/20251027/0_salon_so_day`. Selected metadata and360 images were recovered using byte spans, with ZIP member CRC and file SHA256 checks. The full archive was not downloaded/rehashed. Metadata acquisition was restarted with batched reads for efficiency; previously staged members were checked against the central-directory CRC/size before reuse. Attached/archive contents are treated as data, not executed instructions.

The extracted time series span about162.8s. RGB has1,493 unique acquisitions per side. Processed thermal has1,455 left/1,459 right unique acquisitions; left11 missing and27 duplicate rows, right7 missing and27 duplicate rows. Original sparse indices and timestamps are preserved. The recording audit reports full thermal-topic retention before this extracted/resampled representation; this does not mean every extracted timestamp row is unique.

The existing chronological-nearest/no-reuse policy (stereo<=2ms, each RGB/thermal acquisition pair<=30ms) yields **1,335 synchronized groups spanning162.70s**. There are zero shared acquisition timestamps with either earlier recording. All six inspected raw/processed thermal and RGB camera-info files match the earlier day recording exactly. File consistency is not a physical mounting certificate, and the original thermal processing still uses the previously audited radtan/inverse convention.

Source SLAM has `sensor_init`→`vehicle` frame labels and covers the selected windows. This remains the same type of recorded SLAM proxy as before.

## GNSS-labelled data: useful provenance lead, not verified independent reference

`sensors/novatel_gps_odom` is present, labelled `map`→`gps_frame`, with7,488 interpolated rows,13 numeric columns and163.2s coverage. Position changes span approximately224.18m/76.73m/3.82m by axis; it is not a constant placeholder. The export provides no documented covariance, fix-quality or RTK-status fields. Its accuracy, timing, upstream fusion dependencies and body/antenna transform applicability are not independently established here.

The [public converter config at commit01bbd8dd7](https://github.com/castacks/ros_torch_converter/blob/01bbd8dd7b0d9f5fdfceb0a78319a34e640c4887/config/topics/yamaha/novatel_topics.yaml) maps the output name `novatel_gps_odom` to **`/odometry/filtered_odom`**, type `OdomRBState`; the archive's conversion statistics also identify that topic as its binding topic. The exact config revision used for this archive is unverified, so this is a provenance clue, not proof of the topic's sensor inputs. Filtered odometry could be independent of SLAM or could share inputs; the folder name alone settles neither. The listed RTK IMU/earth-odometry conversion topics have zero messages and were skipped. These facts do not prove that no accurate GPS data existed in the original bags.

A focused lab question is now: **What generates `/odometry/filtered_odom` for this recording, which sensors does it fuse, does it consume Super Odometry, and where are fix quality/accuracy/covariance, timing and the `gps_frame` extrinsic documented?** No GNSS-derived ground-truth accuracy or alignment is asserted in this experiment. The public source snapshot/hash is retained in staging.

## Windows fixed before image/model outcomes

Use the same previous policy: choose maximum recorded body endpoint displacement within each chronological third of the1,335 paired groups, with20 consecutive rows, positive gaps<=150ms in both modalities and SLAM interpolation coverage. Ties choose earlier start. Windows are early start383,middle684,late1145, with selected body displacements8.9445/8.9790/8.9023m. This selects moving segments intentionally; it is not random sampling or proof of generalization to low-motion/night cases. Each inference window is about1.9s, despite the longer available recording.

Recover original RGB/processed thermal stereo images plus uniquely timestamp-associated raw thermal:360 source images total. The unchanged archived thermal producer reproduces **all120 original processed thermal images pixel-for-pixel**. No selected raw timestamp association is ambiguous. Generate MSA thermal rectification/intensity processing in separate datasets using EXP-023's fixed recipe. RGB files are copied byte-for-byte; thermal metadata carries source K/D and the new R/P. Original inputs remain unchanged.

First-frame SIFT/F-RANSAC fits have RGB709/630/606 and thermal58/285/163 inliers. MSA thermal median vertical mismatch is0.282/0.144/0.155px; source RGB mismatch is1.034/1.010/1.037px. These are correspondence diagnostics, not target/metric calibration certificates. Representative RGB and thermal previews were inspected before interpreting model results.

## Bounded camera-local VO

| Window | RGB translation RMSE (m) | MSA thermal translation RMSE (m) | RGB rotation RMSE (deg) | MSA thermal rotation RMSE (deg) |
|---|---:|---:|---:|---:|
| Early |0.3809|0.1846|1.2422|1.2694|
| Middle |0.1220|0.1981|0.4702|1.2454|
| Late |0.1300|0.0839|0.2231|0.6068|

Reference is recorded SLAM at each camera's exact acquisition times, with original nominal body extrinsics and each camera's own rectification/NED conversion. Anchor first pose only; no fitted rotation, translation or scale. Camera positions/time offsets differ, so per-camera reference displacement norms are about8.91–8.93m rather than numerically identical. No independent ground-truth accuracy or physically calibrated modality comparison is claimed.

All six attempts use the unchanged original modality checkpoints/effective configs, seed0, fp32, mappingfalse, synchronous optimizer, passive uncertainty/signed-stereo recording. They produce20 finite timestamp-aligned poses and zero interpolated frames. Retained match counts are RGB3631/3739/3656 and thermal2514/2495/3322 (see exact reports/status file). All observation covariance eigenvalues remain positive. Execution completion is not an accuracy guarantee.

Thermal has lower translation error in early/late, RGB in middle. RGB has lower rotation error in all three, with early nearly tied. Thus there is no uniformly superior modality across windows/targets. This supports MSA's usefulness on another capture while leaving unknown physical calibration/checkpoint training overlap and same-site/date dependence.

All six first-frame feature/model disparity sign agreements are100% on the fixed >1px observed/model magnitude population. Dense negative disparity predictions are RGB7.34/7.85/9.40% and thermal5.55/0.00/8.95%; they are retained and not masked. Absolute-disparity depth behavior remains unchanged. Captured signed outputs reproduce saved magnitude depths, but no ground-truth depth is available.

## Initial uncertainty/geometry characterization

| Window/camera | Raw trace (px²) | Reprojection (px) | Sampson (px) | Within-pair rho reprojection | Within-pair rho Sampson |
|---|---:|---:|---:|---:|---:|
| Early RGB |3.369|2.601|1.035|0.437|0.143|
| Early thermal |22.690|3.290|1.369|0.629|0.290|
| Middle RGB |0.252|2.434|0.774|0.545|0.341|
| Middle thermal |0.789|2.509|0.859|0.656|0.372|
| Late RGB |0.763|1.442|0.473|0.541|0.429|
| Late thermal |2.190|1.791|0.776|0.760|0.517|

Values are unweighted medians over19 pair medians/within-pair Spearman correlations, all fixed-grid source points. Inbound and inbound/both-depths<=50m sensitivities are retained. The same EXP-024 geometry definitions/depth/motion degeneracy rules apply. These are native image-space pixel uncertainties/residuals, not cross-camera calibrated quantities. Spatial reprojection ranking is positive in these new windows. Thermal raw trace is higher than RGB in every window while translation-error ordering changes, illustrating why raw trace ordering cannot simply be treated as a pose-reliability ranking. Different image scales/FOV and shared model/reference errors remain confounds. No temporal predictor, threshold or fusion rule is fitted.

## Verification and preservation

16 relevant adapter/reference/held-out selector/signed-flow tests pass in the pinned Blackwell image. All739 frozen preparation/input/model hashes and158 VO-analysis hashes pass. Independent timestamp bounds/no-reuse, exact old-producer controls, camera-reference RMSE, signed-flow/depth controls, optical reprojection, scalar OpenCV Sampson and rank checks pass on160,056 grid point scores. The comparison figure was inspected. No unsuccessful scientific run was replaced; all six statuses are retained.

Artifacts under `results/salon-so-day/`: archive inventory, metadata/image extraction records, paired manifest/preflight, frozen acquisition selection, `prepared-001/` source-consistent datasets/stereo previews/matches, `vo-001/` full six attempts with uncertainty/signed arrays, `geometry/`, proxy arrays, `analysis.json`, `validation.json`, `comparison.png/.pdf` and exact scripts/logs under staging. Original selected source images/metadata are at `data/tartandrive3/20251027/0_salon_so_day`.

Replay uses pinned geometry image `sha256:a8a2dfcd2246cbf18e9fb0529ca9f65d63586eda47e3ade8c57f27d14f3c2e73` for preprocessing/analysis and GPU image `sha256:ffce9f0f1142b71cb518382328a4196f87f7fc6f4bde1a0e36069c612e69cad9` for inference. Mount ore-ms at `/workspace`, workdir `/workspace/MAC-VO-main`. Run `python -m Scripts.Setup.td3_additional_recording prepare --output <new-prepared>` after preserved acquisition, then `execute --plan <new-prepared>/plan.json --case early --modality thermal --output <new-case>` (and corresponding other cases/modalities). Scripts/source/reference/runtime hashes and commands are preserved.

Box recovery: `ore-ms/backups/2026-10-21-exp-025`, including selected source metadata/images, preparation/VO/analysis/validation, public source note, notebook, source commit and incremental Git bundle. Excludes model checkpoints, credentials, full archive and disposable range caches. Earlier source/MSA recovery remains a prerequisite.

## Decision

This adds a third nonoverlapping recording and three moving paired windows to the experiment set. Keep MSA as the experimental thermal baseline and preserve modality/target distinctions. More recordings are useful, but independent-reference provenance is now a concrete topic-level question. Continue to treat SLAM-proxy results as provisional and keep production defaults/weights unchanged.
