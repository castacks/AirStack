# EXP-029: Stereo matching consistency without a pose reference

Date:2026-10-21 (America/New_York). Branch `ore_ms`, base `1f1a5ba05` plus frozen diagnostic source. **Saved stereo predictions agree with independent patch matching at accepted image points, while the independent matcher supplies sparse, spatially selected support in thermal images. Late-night thermal has68 accepted points out of1,008, with0.423px median model–matcher endpoint difference. This does not validate full-image depth, physical scale, camera mounting or pose accuracy. No new inference, reference replacement or VO/default change.**

## Why this is useful while source replies are pending

EXP-027/028 found low late-night image motion and disagreement between supplied odometry streams. Their remaining reference-provenance questions require external information. Stereo correspondence can be checked directly in saved left/right images without choosing either odometry stream as truth. This experiment extends the earlier sparse SIFT checks with a fixed source grid and explicit matching failures. Its uncertainty target is **stereo matching variance**, not the temporal flow variance or pose errors studied in other experiments.

The independent matcher is another algorithm applied to the same images. Reciprocal/texture/uniqueness checks improve its consistency but do not prove physical correspondences. Repeated textures, occlusion, rectification errors, finite search and integer-pixel quantization can affect both matching and apparent agreement. Do not treat this diagnostic as calibrated depth ground truth or a new transferable reliability label.

## Fixed inputs and matching protocol

Use every saved first stereo capture from EXP-023 (six MSA-thermal cases), EXP-025 (three RGB/three MSA-thermal) and EXP-026 (three RGB/three MSA-thermal):18 cases,36 images,12 thermal and6 RGB. These cover the same three recordings as before. RGB controls are all day images from the added salon recording; there is no matched RGB-night comparison here. Only the first stereo pair of each inherited window is inspected, not all20/100 frames. These are previously examined, intentionally selected windows, not random or unseen validation sequences.

All97 image/output/report/plan/helper/test hashes are frozen before each attempt. Capture flow/covariance shapes and finite positive variance are checked. Signed flow reconstructs the saved magnitude depth with relative tolerance1e-6; captured and original first-depth arrays are bitwise equal. This is an implementation consistency control, not a depth-accuracy certificate. Inputs and outputs use native resolution. Independent matching converts BGR to grayscale, without resizing or further image enhancement.

Use a16px grid with32px border:1,008 thermal points per image,1,800 RGB points per image,22,896 total. Exhaustively search11x11 patch normalized cross-correlation (NCC) over horizontal displacement[-128,+128]px and vertical[-2,+2]px, retaining only full-patch image support. Keep signed displacements. The runner-up excludes horizontal coordinates within3px of the best peak. Repeat target-to-source with the same search. Model flow and uncertainty never initialize, select or filter the independent matcher.

Primary acceptance requires both source/target texture standard deviations>=5, both forward/reverse NCC>=0.8 and peak-minus-runner-up gap>=0.05, neither peak on a search boundary, finite endpoints and reciprocal endpoint error<=1px. A fixed stricter sensitivity uses texture>=10,NCC>=0.9,gap>=0.1 with all other rules unchanged. Failures and undefined values remain visible at every source grid point. Failure counts overlap and must not be summed as disjoint causes. The matcher has integer-pixel precision; no subpixel fit or parameter tuning is introduced.

Compare observed/model horizontal and vertical displacement and endpoint difference at the exact same source pixels. Sign agreement requires both observed/model horizontal disparity magnitude>1px. Summaries retain all grid/primary/strict and three primary image-row populations. Secondary Spearman associations compare raw horizontal variance with horizontal discrepancy, and variance trace with endpoint discrepancy, within each image only. No pooled probability calibration, temporal prediction or cross-camera normalization is claimed; native pixel² quantities are not physically calibrated across cameras.

## Numeric stabilization: both attempts retained

Run-001 used direct uint8 OpenCV NCC. Independent float64 recomputation found an accepted score difference0.00010131, exceeding the original1e-4 verification tolerance. An explicitly recorded intermediate, input-scaled float32 allowance explained the high-DC correlation rounding: maximum accepted difference0.000444. More importantly, two of the70 accepted night-late points had forward uniqueness gaps just above0.05 in OpenCV and just below0.05 in double arithmetic. Raising only the verification tolerance would not address that gate sensitivity.

Before run-002, preserve run-001, exact source/tests/validator, logs, arrays and figures. Mean-center the source patch and target search stripe in float32 before the same NCC operation. NCC is algebraically invariant to these separate additive offsets; centering reduces high-DC cancellation. Keep the same native images, all18 cases, grid/patch/search ranges, thresholds, reciprocal gates and downstream metrics. Add a low-contrast additive-offset invariance/direct-NCC test. The only changed frozen files are the new module and its test; all95 other input/helper hashes are identical.

Run-002 restores the originally prescribed1e-4 NCC numeric tolerance. Maximum accepted scalar NCC difference is5.88e-7. All1,008 night-late exhaustive source searches agree with independent double maxima/runner-up scores within1e-4, and no forward score/gap gate differs under direct float64 computation. Late primary acceptance changes70→68 while its endpoint median stays0.423px; other primary case counts change by at most1. A complete mask-membership/count/median comparison is retained. Scientific interpretation is not selected between attempts; the same qualitative findings hold. No original prediction, pose, preprocessing input or VO setting is modified.

## Measurements from stabilized run-002

| Fixed first-frame case | Primary / grid | Strict | Primary median endpoint difference (px) | Variance-u / horizontal rho | Trace / endpoint rho | Strict trace / endpoint rho |
|---|---:|---:|---:|---:|---:|---:|
|EXP023 day-early-thermal|256/1008|60|0.492|-0.080|0.065|0.027|
|EXP023 day-late-thermal|191/1008|17|0.439|0.089|0.165|0.071|
|EXP023 day-original-thermal|216/1008|28|0.429|0.018|0.108|0.199|
|EXP023 night-early-thermal|105/1008|33|0.381|0.200|0.279|0.259|
|EXP023 night-late-thermal|68/1008|10|0.423|-0.078|0.056|-0.115|
|EXP023 night-middle-thermal|70/1008|15|0.681|0.073|0.129|-0.379|
|EXP025 early-rgb|935/1800|497|0.316|0.201|0.218|0.090|
|EXP025 early-thermal|95/1008|11|0.506|0.144|0.112|0.255|
|EXP025 late-rgb|993/1800|433|0.402|0.209|0.225|0.141|
|EXP025 late-thermal|235/1008|29|0.429|0.027|0.140|0.102|
|EXP025 middle-rgb|1087/1800|419|0.324|0.070|0.116|0.060|
|EXP025 middle-thermal|305/1008|65|0.409|0.121|0.188|0.035|
|EXP026 early-rgb|1264/1800|638|0.325|0.148|0.195|0.220|
|EXP026 early-thermal|263/1008|59|0.562|0.094|-0.041|-0.080|
|EXP026 late-rgb|1161/1800|565|0.381|0.227|0.237|0.201|
|EXP026 late-thermal|244/1008|45|0.482|0.168|0.245|0.105|
|EXP026 middle-rgb|740/1800|298|0.402|0.243|0.316|0.235|
|EXP026 middle-thermal|295/1008|99|0.424|0.139|0.230|0.301|

Each row is one image pair. Percentiles describe image points, not independent trajectories or confidence intervals. Thermal primary acceptance ranges6.75–30.26%; RGB41.11–70.22%. The fixed stricter gate retains10–99 thermal points per image versus298–638 RGB points. Different views, resolutions, focal lengths, scene texture, preprocessing and visibility prevent reading these rates or native pixel discrepancies as a physically matched modality ranking. They describe where this particular algorithm can supply support.

The twelve thermal median endpoint differences range0.381–0.681px; six RGB medians0.316–0.402px. Integer-pixel correspondence quantization contributes to these numbers.17 of18 primary sign-eligible populations agree100%; EXP-026 middle RGB agrees99.86% (one differing sign among737 eligible points). Signed predictions outside accepted populations, including negative-disparity image regions, remain unvalidated; no sign filter or absolute-depth correction is adopted.

## Late-night interpretation and spatial selection

The fixed late-night MSA pair supplies68/1,008 primary points (6.75%) and10/1,008 strict points (0.99%). Primary median observed/model left-minus-right disparity is9.000/9.045px. Median horizontal/vertical differences are0.305/0.170px, with0.423px median endpoint difference and1.048px90th percentile. All64 sign-eligible primary points agree. The strict endpoint median is0.234px, on only ten points.

This supplies local image evidence compatible with some saved stereo correspondences; it does not validate the remaining940 source grid locations or metric depth. The figure preserves all source images and accepted locations rather than presenting only successful patches. Thermal support is concentrated in image-dependent structures, and the near-blank/weakly textured regions remain weakly observed. The independent matcher's failure is an unavailable check, not proof that the learned network is wrong. Conversely, its acceptance is not a certificate that the learned depth or geometry is physically accurate.

The trace/endpoint association in late-night primary is0.056; horizontal variance/horizontal discrepancy is-0.078. The strict trace association is-0.115 with only ten points. Across thermal primary cases, trace associations range-0.041 to0.279, while the strict range is-0.379 to0.301. RGB primary associations range0.116–0.316 and strict associations0.060–0.235. Selection changes the population, and both algorithms share image ambiguities. These are descriptive matching-discrepancy associations, not a calibration or pose-reliability test; weak/mixed associations do not identify why predicted uncertainty behaves this way.

## Controls, artifacts and replay

Nine focused tests pass: signed positive/negative horizontal and vertical shifts, periodic ambiguity, textureless images, search boundaries, empty summaries, low-contrast additive-intensity invariance/direct NCC, plus prior signed/depth controls. Independent scalar NCC and standard deviations, reciprocal/boundary masks, signed errors, quantiles, sign counts and rank calculations pass on22,896 grid points,17,046 accepted forward/reverse NCC comparisons and108 case/population summaries. Independent double exhaustive search verifies all1,008 fixed night-late source points, including failures/constant patches. A supplemental32-seed synthetic control uses the actual128px search width:24 original arithmetic cases exceed1e-4 NCC error (maximum0.000343), while centered arithmetic stays below1.93e-7 and additive-offset differences below1.20e-7. A preceding small-stripe probe did not reproduce that failure, and its log is retained; an initial standalone import was rerun with explicit PYTHONPATH. All97 run-002 hashes pass after analysis. Earlier EXP-028's402 scientific hashes remain unchanged.

Runtime: existing CPU `macvo:geometry`, OpenCV4.10.0.84/NumPy2.2.6, one OpenCV thread/seed0; exact image ID, package inventory, source and commands are saved. No GPU operation, checkpoint reload, new image acquisition, source/GT download or VO run. The initial numeric validation failure and intermediate allowance audit remain preserved; final verification uses the original1e-4 absolute NCC tolerance. Failed/ambiguous points are not reclassified by inspecting model agreement.

Artifacts under `results/stereo-image-consistency/`: frozen `run-001/` and `run-002/` plans, full grid/match/model arrays, `analysis.json` and full `summary.csv`; independent `validation.json`, `night-late-exhaustive-checks.json`, input preservation and numeric-stabilization comparison; summary and all18-source-grid PNG/PDF figures; exact scripts/logs/runtime and attempt-001 source/tests/plots under `staging/`. No generated artifact path is required in committed code beyond the documented experiment inputs.

With ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`, CPU geometry runtime:

```bash
python -m Scripts.Setup.td3_stereo_image_consistency prepare --output /workspace/results/stereo-image-consistency/new-run
python -m Scripts.Setup.td3_stereo_image_consistency run --output /workspace/results/stereo-image-consistency/new-run
python -m pytest -o addopts= Scripts/UnitTest/test_td3_stereo_image_consistency.py Scripts/UnitTest/test_td3_signed_frontend.py
```

Prepare requires restored EXP-023/025/026 inputs/signed outputs, available source/helpers and a new output directory. Box `ore-ms/backups/2026-10-21-exp-029` preserves new source/tests/report, both attempts, numeric-failure/stabilization evidence, independent controls, figures, local reproducibility notes and incremental Git bundle. Prior source/model/image snapshots remain prerequisites; credentials, full archives and checkpoint files are excluded. A CPU-only rerun does not require the model weights or GPU runtime.

## Decision

This is useful progress independent of the pending reference replies: saved thermal stereo predictions have image support on a limited, explicitly measured subset; broad depth validity and reliable uncertainty-to-error ranking remain unresolved. Keep MSA as the experimental baseline and all VO/selection/weighting defaults unchanged. Do not turn these match scores into a fusion rule, label matcher failures as network failures, infer body/metric calibration from disparity agreement, or compare per-image native variances as physical cross-modal confidence.

The next external gate remains the recording-specific odometry producer, quality/timing and physical camera/body provenance. More reference-free characterization is possible, but these data already show why a pose-accuracy claim or adaptive fusion model would be premature. The present experiment completes the bounded diagnostic, not those external checks.
