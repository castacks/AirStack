# EXP-023: Raw-first MSA thermal preprocessing and bounded VO

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `90759fae1` plus frozen replay source. **MSA materially improves the three day-window VO comparisons to the recorded SLAM proxy. Night changes are mixed and late-night motion remains underestimated. All six20-frame attempts execute without interpolation. No production defaults or previous results were replaced.**

## Main result

| Fixed window | Original translation RMSE (m) | MSA translation RMSE (m) | Original rotation RMSE (deg) | MSA rotation RMSE (deg) |
|---|---:|---:|---:|---:|
| Day original |1.5171|0.0763|5.6879|0.8465|
| Day early |0.7027|0.1600|4.1763|1.6742|
| Day late |2.8966|0.1792|1.7129|1.0650|
| Night early |0.3011|0.1761|1.3427|1.6061|
| Night middle |0.7670|0.7417|2.6308|2.0346|
| Night late |0.1420|0.1411|0.1086|0.1414|

These are first-pose-anchored, unscaled camera-local errors against the **same recorded SLAM proxy**, not independent ground-truth accuracy. No best-fit scale, rotation, alignment, threshold or weight tuning was performed. The MSA camera reference uses its own rectification rotation while retaining the same original nominal vehicle-from-raw-left pose used by the original reference. New MSA LiDAR-parent poses are not equated to the vehicle frame. Both references therefore retain the same provisional body anchor.

Day translation error drops by approximately95%,77% and94%. Night early translation improves by42%, while its rotation worsens. Night middle anchored translation improves only3%; its estimated net displacement actually decreases from5.53m to4.77m against a6.16m proxy displacement. Late night remains nearly stationary: estimated net displacement0.0085m versus proxy0.2610m (original0.0112m). Its small RMSE decrease is not evidence of restored motion tracking. Rotation worsens in night early and late. Do not summarize this as universal improvement or a solved VO system.

## Fixed acquisition and preprocessing

Use exactly the20 `input_rows` from each original thermal report: day original/early/late and night early/middle/late. The original manifest starts are348/306/1132 and123/619/789, respectively. These are120 stereo pairs across the same two October2025 recordings, with short roughly1.9s windows. No RGB trajectory is rerun and no windows are selected using new outcomes. The original trajectories are archived baselines, not contemporaneous reruns; effective configuration, checkpoint and input rows are checked equal, and the GPU image is the same audited image used in EXP-018.

Recover240 native uint16 raw images from the original Box ZIPs by selected compressed byte spans. CRC checks pass on each selected member. Also recover120 original processed day images for replay controls; original night processed images already exist locally. Whole8.86GB/7.71GB archives were not downloaded or rehashed. All file hashes, timestamp candidates, member provenance and range records are retained. There are360 newly staged input images total,240 raw plus120 original day processed.

Association uses original raw timestamp lists and unique matches within2ms. One exception was found before image analysis: the last day-late left processed time repeats at raw indices1518–1522. Use source processed index1518 only because its timestamp is exact, retain all five candidates, and require pixel-exact old-producer replay. This control passes. No nearest-neighbor or outcome-based replacement is made. All240 selected raw/processed timestamp differences are zero.

First replay the original producer, with its original radtan calibration and inverse stereo transform, under OpenCV4.10. **All240 supplied processed thermal images reproduce pixel-for-pixel**, including all three channels. This validates extraction/indexing and the old processing recipe for these samples; it does not validate old physical geometry.

Separately generate MSA fisheye stereo maps from the source intrinsics/extrinsics (EXP-021), zero-disparity,balance0,fov_scale1,640x512. Apply the unchanged archived `firestereo_v2` percentile clipping, CLAHE and bilateral enhancement to the newly rectified uint16 images. Save replicated three-channel uint8 PNGs and corresponding raw K/D, new R/P camera-info. Preserve original manifest rows and acquisition timestamps. Intensity processing is the same function/parameters, though its image-dependent quantiles naturally change when rectification changes. The experiment changes the combined lens/stereo calibration and rectification; it does not isolate which calibration parameter causes each improvement.

## Signed stereo/depth check

Match each first new processed stereo pair with the unchanged SIFT2000,bidirectional ratio0.7 mutual,F-RANSAC1.5px/.999/seed0 protocol. Retain masks and all match coordinates. New first-pair fitted-match counts are183/294/137/151/82/52. Median vertical residuals are0.230/0.251/0.206/0.256/0.322/0.187px. These new rectified-image matches are diagnostic populations, not identical physical features to old-image matches.

Strictly load the unchanged original thermal checkpoint SHA256 `eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a`. Passive capture retains signed first stereo flow/covariance and per-call sign summaries. Model flow maps left to right; left-minus-right disparity is negative horizontal flow. The normal VO frontend still uses absolute disparity and absolute baseline; no sign filter or model-default correction is silently introduced.

| Window | Median observed/model horizontal error (px) | Eligible feature sign agreement | Dense negative disparity | Original/MSA median predicted depth (m) |
|---|---:|---:|---:|---:|
| Day original |0.276|100%|3.21%|10.788 /6.562|
| Day early |0.254|100%|5.44%|8.735 /5.379|
| Day late |0.271|99.19%|5.70%|9.494 /5.865|
| Night early |0.207|100%|6.42%|9.238 /5.894|
| Night middle |0.365|100%|9.03%|11.566 /6.133|
| Night late |0.144|100%|11.84%|9.156 /5.994|

Sign eligibility requires both observed and model disparity magnitude>1px; full counts are retained. The dense positive fraction rises from the prior60.5–67.2% to88.2–96.8% on the six first frames. Remaining negative predictions are explicitly retained. Dense statistics include all pixels without texture/visibility validity gates, so neither positive nor negative dense sign is an accuracy certificate. Predicted depth medians and large far-depth tails are model outputs, not known scene depths. Signed flow reconstructs saved frontend magnitude depths within float32 arithmetic tolerance; first captured and saved depth arrays are exactly equal. No metric depth ground truth is available.

## VO execution and comparison

All six runs use seed0, original MACT checkpoint, fp32 encoder/decoder, mappingfalse, original covariance/selection settings and synchronous two-frame optimization. The original adapter consumes new rectified P/K and positive MSA baseline norm0.326186422m. Identity body/sensor transform keeps execution in each rectified left-camera NED frame. The sole runtime wrapper passively records stereo outputs; it returns the original frontend result without mutation. Uncertainty recording is passive; no values from it tune this run.

All six attempts finish20 finite normalized poses with exact acquisition timestamps and zero interpolated frames. Retained match counts are recorded in `vo-status.json`; all retained observation covariances are finite with positive minimum eigenvalues. These execution controls do not establish motion accuracy, as the night-late result demonstrates. Evaluation uses recorded body SLAM at the exact thermal timestamps, original nominal left-body extrinsics and each calibration's own R transpose/NED conversion. Original and new reference net-displacement norms agree, while their local coordinate orientations properly reflect different rectification.

## Verification and artifacts

- 25 relevant tests pass:17 CPU geometry/preprocessing/signed tests and8 adapter/reference tests in the Blackwell image. An attempted test command using the previously named `macvo:replay-cpu` image failed because that image is absent; the same tests then passed in the available pinned Blackwell image. No missing tests are reported as passing.
- Independent analytic fisheye projection checks fixed-grid remap coordinates to <=3.04e-5px (float32 maps). Separate quaternion/matrix calculations reproduce anchored proxy RMSE. SciPy interpolation reproduces signed-feature errors/sign counts. Depth-from-flow, trajectory timestamps, configuration equality, source-image channel integrity and all six complete outputs pass.
- All788 frozen preprocessing/input/model hashes and50 execution source/reference hashes pass. The initial acquisition stopped on duplicate timestamps, preserving its log; the documented index-plus-exact-replay rule was fixed before image outcomes. No failed scientific attempt was replaced.
- GPU image: `sha256:ffce9f0f1142b71cb518382328a4196f87f7fc6f4bde1a0e36069c612e69cad9`, Torch2.7.1+cu128 on RTX PRO5000 Blackwell. CPU geometry image: `sha256:a8a2dfcd2246cbf18e9fb0529ca9f65d63586eda47e3ade8c57f27d14f3c2e73`, OpenCV4.10.0.84. Inventories, exact commands, logs and source hashes are retained.
- Artifacts: `results/msa-replay/inputs/extraction.json`, `prepared-001/plan.json`, generated per-case datasets/manifests/camera-info, first-pair matches/previews; `vo-001/` six full execution outputs/signed first-flow/uncertainty records; `analysis.json`, `validation.json`, proxy-reference arrays, signed samples and `comparison.png/.pdf`. The figure was inspected.

Replay requires selected original data/previous recovery and the supplied MSA source; use ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`:

```bash
# CPU geometry image, after acquisition via preserved staging/stage_raw.py
python -m Scripts.Setup.td3_msa_replay prepare --output /workspace/results/msa-replay/new-prepared
# Pinned GPU image, same environment as execution-plan.json
python -m Scripts.Setup.td3_msa_replay execute --plan /workspace/results/msa-replay/new-prepared/plan.json --case day-original --output /workspace/results/msa-replay/new-vo/day-original
```

Analysis/validation/plot scripts and acquisition/execution commands are saved under staging. Box recovery `ore-ms/backups/2026-10-21-exp-023` preserves selected raw/original-day inputs, all new preprocessing/VO outputs, source/tests/report/notebook and incremental Git bundle. Prior EXP-018–022 recovery supplies inherited baselines/calibration. Checkpoints, credentials, disposable range caches and whole archives are excluded. Original source calibration, checkpoint, processed datasets and all earlier experiment outputs remain unchanged.

## Decision

Use the MSA replay as the **experimental thermal baseline** for subsequent geometry/uncertainty diagnostics. The old calibration was materially confounding day VO; old thermal depth-based uncertainty/error measurements cannot be carried over unchanged. Recompute those diagnostics on the new outputs before drawing cross-modal reliability/fusion conclusions. Preserve this six-window comparison as evidence, and investigate the remaining night motion/rotation behavior separately. Longer sequences, additional recordings and independent physical/timing/body reference evidence remain necessary for broader accuracy claims. Production defaults are unchanged.
