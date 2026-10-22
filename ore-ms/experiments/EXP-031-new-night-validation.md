# EXP-031: New night recording — tracking availability and angular consistency

Date: 2026-10-22; acquisition began 2026-10-21 EDT. Branch `ore_ms`, base `859a58b44` plus frozen new source. **On the newly supplied `night_mac_salon_warehousefig8`, all three MSA-thermal20-frame runs complete without interpolation. RGB completes early but retains zero matches in middle and late; independent LK also supplies zero support in those two RGB windows. Thermal uncertainty/disagreement association is near zero early, negative middle and positive late. Fresh data reveals a useful availability contrast without establishing physical uncertainty calibration or a fusion rule.**

## New input and selection before outcomes

The user supplied Box `night_mac_salon_warehousefig8.zip`,8,076,964,663 bytes,22,531 unique safe ZIP paths. Archived source audit names `/media/airlab/offroad/rosbags/20251027/night_mac_salon_warehousefig8`. Use bounded ZIP byte ranges; verify selected-member CRC and SHA256. Stage103 metadata/tabular files and360 selected source images, not the full archive. The whole ZIP hash is not verified. Original source data are not executed as code.

The four exported streams span roughly213.4s and yield1,500 unique synchronized groups across211.4s with the established chronological nearest, no-reuse policy: stereo≤2ms and all RGB/thermal offsets≤30ms. RGB left/right each have2,051 existing unique acquisitions. Processed thermal left/right have1,953/1,987 after keeping the first existing row per timestamp;57/35 timestamp rows lack an eligible PNG or finite stamp and41/29 repeat an existing stamp. These are exported binding-grid availability counts, not demonstrated original sensor loss: the source conversion audit separately reports full raw-to-processed stamp retention. Original sparse indices and timestamp precision are preserved. All new paired timestamps are disjoint from all three prior recording manifests.

Before images or model outputs, split the paired timeline into thirds and select the **first** eligible20-frame block within each third. Both modality timelines must have positive gaps≤150ms and lie within recorded SLAM temporal support. The late third first becomes eligible at row1,152; no motion/texture/uncertainty objective or post-result replacement is used. Selection tests include timeline mismatch, reference support and gap rejection. Motion context is measured only after selection.

| Window | Paired start | EDT interval (2025-10-27) | Body SLAM displacement (m) |
|---|---:|---|---:|
| early | 0 | 21:34:50.010–21:34:51.910 | 0.0717 |
| middle | 500 | 21:35:55.809–21:35:57.709 | 0.1651 |
| late | 1152 | 21:37:39.508–21:37:41.408 | 5.3036 |


Each selected thermal image maps to one existing raw file at the exact acquisition timestamp; all120 selected raw associations are unique. Six camera-info exports match the earlier recording metadata exactly. This is metadata consistency, not proof that the MSA fisheye calibration or body mounting is physically correct. Apply unchanged experimental MSA preprocessing to native512×640 uint16 thermal; all120 supplied processed image sides reproduce exactly with the original producer/calibration. RGB uses native1024×544 images without enhancement.

## Fixed execution and image consistency

Run six20-frame attempts with the established RGB/thermal checkpoints, configurations, seed0, FP32, mapping disabled, synchronous optimizer and opt-in uncertainty recorder. No settings/window retries, fusion model or threshold tuning. All six produce19 saved temporal prediction pairs, including the two failures. Their logs contain19 `VOLostTrack` events each and the terminal error `No visual matches were retained`. The failed attempts have no validated saved VO trajectory and receive no pose-error score. Retained counts below are after the configured observation filter, separate from LK support.

Reuse the unchanged EXP-030 ray-Jacobian variance, exact model–LK angular difference and independent forward/reverse LK protocol on all114 temporal pairs,160,056 source-grid observations. Primary LK forward/backward error≤1px; sensitivities≤0.5px or primary with source21×21 intensity std≥5. Covariance is raw temporal uu,vv,uv before floors. Angle quantities depend on supplied K and are nominal; LK noise/source uncertainty/shared image ambiguity remain omitted. All-grid covariance summaries are model-only when LK fails; their unfiltered model–LK discrepancies are not evidence of valid correspondence.

| Window/modality | Execution | Retained per pair | LK support % (min–max) | Nominal σ on LK support (mrad) | Model–LK difference (mrad) | Spatial ρ | Temporal ρ |
|---|---|---:|---:|---:|---:|---:|---:|
| early-rgb | complete | 199–200 | 0.11–7.83 | 0.617 | 0.381 | 0.161 | 0.437 |
| early-thermal | complete | 142–200 | 94.44–97.62 | 0.503 | 0.226 | 0.005 | 0.530 |
| middle-rgb | no matches | 0–0 | 0.00–0.00 | unavailable | unavailable | unavailable | unavailable |
| middle-thermal | complete | 85–156 | 94.35–97.42 | 1.980 | 0.221 | -0.223 | 0.498 |
| late-rgb | no matches | 0–0 | 0.00–0.00 | unavailable | unavailable | unavailable | unavailable |
| late-thermal | complete | 145–200 | 73.21–85.32 | 1.603 | 1.140 | 0.529 | -0.411 |


The σ column is the square root of the median of the19 pair-median variances; angular difference and spatial ρ are medians of available per-pair medians/rhos. Temporal ρ associates contemporaneous pair-median variance and discrepancy. Undefined rows remain unavailable, never zero errors. In early RGB, pair support ranges only2–141 of1,800 grid points; some per-pair ranks are undefined. Thermal primary support ranges94.44–97.62% early,94.35–97.42% middle and73.21–85.32% late. Early/middle thermal source-to-LK angular motion medians are0.118/0.117mrad versus24.704mrad late, consistent with a strong image-motion contrast. This does not certify vehicle standstill.

With tighter reciprocal support, thermal spatial ρ is0.0046/−0.2227/0.4919; with texture support it is−0.0043/−0.1855/0.5350. Thus the low-motion mixed association seen previously persists in a new recording. Early RGB's texture sensitivity has only0–4 points per pair; its nominal medianρ0.8 is not meaningful broad support. Do not interpret an apparently strong statistic after such sparse selection as a reliable modality comparison.

All-grid model-only nominal σ for RGB is1.197/137.452/129.643mrad across early/middle/late, compared with0.505/1.989/2.101mrad for thermal. The two RGB failures coincide with much larger reported variance and unavailable independent tracking, a descriptive availability association in these windows. There is no measured RGB geometric-error target there, so no relative-error calibration, coverage, ranking accuracy or fusion threshold can be inferred. All160,056 raw covariance matrices are finite positive semidefinite; 50,937 observations have nominal σ>0.1rad (100 within thermal late primary support, none in other primary populations). Those are flagged and retained; broad uncertainty also limits the small-angle propagation approximation.

![Fixed-window angular comparison](../results/new-night-validation/angular-comparison.png)

The RGB images are visibly dark and weakly textured. Across20 frames, median whole-image grayscale mean/std is17.72/1.98 early,17.41/0.72 middle and17.31/0.71 late (uint8 scale); these are image statistics, not sensor exposure or illumination measurements. Thermal images retain visible scene structure. Neither image appearance nor successful execution identifies the cause of RGB failure or proves thermal pose accuracy.

![Native display values from frozen windows](../results/new-night-validation/source-windows.png)

## Recorded-SLAM context only

For the four successful trajectories, retain first-pose anchoring with no scale or best-fit alignment. References use original nominal body anchoring and modality rectification at exact acquisition timestamps. Body displacement above and camera displacement below are different origins/orientations and can differ. These are **proxy discrepancies**, not independent ground-truth errors.

| Window/modality | Estimated / nominal camera SLAM net displacement (m) | Anchored translation RMSE (m) | Anchored rotation RMSE (deg) |
|---|---:|---:|---:|
| early-rgb | 0.0097 / 0.0717 | 0.0362 | 0.1284 |
| early-thermal | 0.0039 / 0.0703 | 0.0339 | 0.1139 |
| middle-thermal | 0.0036 / 0.1644 | 0.1499 | 0.0841 |
| late-thermal | 3.7316 / 5.2220 | 0.9905 | 2.2131 |


The late thermal run supplies substantially more displacement than early/middle, but still differs from the nominal camera SLAM stream. The middle low-image-motion discrepancy persists in fresh data. Neither proves whether the learned geometry, supplied reference or nominal mounting is physically responsible. Recording-specific NovAtel/filtered-odom inputs, timing, quality and camera/body provenance remain pending.

## Verification, runtime and preservation

24 tests pass:7 selection/angular controls,12 preprocessing/calibration/signed controls and5 GPU adapter/uncertainty controls. Independently reproduce all1,500 pairing gates/unique indices and first eligible window starts;120 original-producer image controls pass. All600 source/prepared images decode identically under CPU OpenCV4.10.0 and rebuilt GPU OpenCV5.0.0. Recalculate798 scalar ray angles and798 finite-difference covariance propagations (maximum variance difference9.42e-13rad²), six full-grid forward/backward LK pairs,3,192 pair/population quantiles and912 rank coefficients. Independently verify four anchored proxy metric summaries and the two preserved19-pair no-match failures. All751 prepared hashes and267 angular-analysis hashes pass. Prior EXP-030729, EXP-028402 and EXP-02997 hashes remain unchanged.

An initial independent quantile check inadvertently used float32 interpolation while production summaries explicitly cast to float64; this caused a1.91e-6 difference. The validator now applies the documented float64 cast, retaining the original1e-12 comparison tolerance. Initial validator/log are preserved; no observations, scientific thresholds or analysis outputs were changed.

The restored `macvo:blackwell` image is `sha256:f429af4afa234fc1e37db6867ed9ee9c708af852ae755fe7d6e8150f70136300`, rebuilt from the existing pinned CUDA/PyTorch recipe. PyTorch2.7.1+cu128, NumPy2.2.6, PyPose0.9.5, OpenCV5.0.0; exact packages/image/source/config/checkpoint hashes and all commands are recorded. GPU smoke multiplication and actual inference pass despite container initialization diagnostics in stderr; diagnostics remain preserved. Transitive dependencies differ from the historical GPU runtime, so this is not an exact-image rerun of old trajectories. Decode equality controls that particular image-loading difference, not every possible numerical runtime difference. CPU preprocessing/analysis uses the unchanged `macvo:geometry` image101a3943f3d1/OpenCV4.10.0.84. GPU: NVIDIA RTX PRO5000 Blackwell48GB. OMP/OPENBLAS threads1; analysis OpenCV threads1/seed0.

Artifacts: `results/new-night-validation/` contains archive index/member extraction records, full paired manifest, frozen acquisition/prepared/execution/angular plans, prepared datasets, all six attempted VO outputs (including failures),114 full-grid angular/LK arrays, context/summaryCSV, proxy arrays, figures, tests/independent validators and exact scripts/logs/runtime under `staging/`. Original selected metadata/images are in `data/tartandrive3/20251027/night_mac_salon_warehousefig8/`. Exclude redundant compressed range-cache blocks from preservation; selected source bytes and CRC/SHA records are sufficient.

With ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`, CPU preparation and GPU execution:

```bash
python -m Scripts.Setup.td3_new_night prepare --output /workspace/results/new-night-validation/new-prepared
python -m Scripts.Setup.td3_new_night execute --plan /workspace/results/new-night-validation/new-prepared/plan.json --case early --modality rgb --output /workspace/results/new-night-validation/new-vo/early
```

Repeat execute explicitly for the frozen early/middle/late cases and rgb/thermal modalities with the recorded GPU flags. The saved scripts reproduce extraction, selection, preparation, execution and EXP-030 analysis; frozen paths require restored source images/metadata and original model/calibration prerequisites. Box recovery snapshot `ore-ms/backups/2026-10-22-exp-031` contains selected new source bytes, preparation, attempts, analysis, notebook evidence and incremental source bundle. Checkpoints, credentials and the full8GB source archive are excluded; prior model/calibration recovery remains required.

## Decision and further data

This recording materially extends the evidence: severe RGB tracking unavailability and thermal operational completion coexist with weak/mixed thermal uncertainty-to-disagreement ranking in low-image-motion windows. Availability is a valid operational outcome, distinct from a calibrated relative geometric-error target. Keep the MSA experimental baseline and all VO/weighting defaults; no adaptive fusion rule is justified by this check alone.

**No further upload is needed for the next bounded step:** most of this211s paired timeline is still unused. A prespecified longer or transition-focused analysis can use it without asking for another archive. More independent recordings with usable night RGB would later help cross-modal geometric-error comparison, but only request them when needed. The user explicitly asks to be told whenever additional data would materially strengthen a study, with specific folders/conditions requested from20251027. Physical calibration/reference provenance remains an external information gate, not something another image archive alone resolves.
