# EXP-024: Thermal uncertainty diagnostics after MSA geometry correction

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `a729420c9` plus frozen analysis source. **The MSA correction reduces day depth-dependent reprojection error, but does not establish a consistent temporal reliability score or calibrated fusion rule. Spatial reprojection ranking is positive on the primary grid in all six windows; late-night epipolar ranking remains negative. No defaults, weights or inference outputs changed.**

## What changed after correcting geometry

| Thermal window | Original/MSA raw trace (px²) | Original/MSA reprojection (px) | Original/MSA Sampson (px) | MSA spatial rho reprojection | MSA spatial rho Sampson |
|---|---:|---:|---:|---:|---:|
| Day original |0.771 /0.862|6.779 /2.869|1.103 /1.063|0.498|0.259|
| Day early |0.325 /0.331|6.806 /4.901|2.105 /2.166|0.464|0.205|
| Day late |15.687 /18.585|6.452 /1.381|0.598 /0.567|0.670|0.495|
| Night early |1.103 /1.070|2.393 /2.369|0.963 /0.939|0.707|0.306|
| Night middle |15.464 /16.461|7.266 /8.185|2.167 /2.202|0.714|0.351|
| Night late |0.538 /0.373|0.779 /0.889|0.035 /0.033|0.227|-0.216|

Each value is an unweighted median over19 consecutive pair medians, or over19 within-pair Spearman correlations. The primary population is all1008 fixed-grid source pixels per pair, including predictions outside the target image. Spatial rho means ranking points within one pair, not ranking frames/windows. The two arms sample the same pixel-grid coordinates under different rectification; these are not matched physical rays, and no pointwise treatment comparison or pooled independent-sample test is made.

Day late remains much more uncertain than day early despite substantially smaller reprojection and epipolar residuals. A globally monotonic relationship across these windows is therefore unsupported. Sampson residuals change relatively little while day reprojection errors fall, consistent with the latter's additional sensitivity to predicted depth. This is descriptive evidence; combined lens/extrinsics/rectification and scene-content changes are not isolated causal effects. Raw trace is temporal learned image-space covariance before floors; it is not stereo depth variance or calibrated pose uncertainty.

## Frozen inputs and measurement

Use all six EXP-02320-frame thermal windows, both original and MSA arms, with19 saved consecutive-pair uncertainty records per arm/case. Old day records come from the original uncertainty run. Old night records are EXP-014's previously validated passive baseline recorder. New records are EXP-023's passive captures. No model inference, additional image preprocessing, calibration/scale fitting, threshold search or VO rerun occurs. All278 input/helper/test-source hashes are frozen before analysis and verified afterward.

Reuse EXP-023's exact original/MSA camera proxy arrays at thermal acquisition timestamps. Both retain the original nominal vehicle-from-raw-left body anchor and use their own rectification/NED conversion. The reference remains recorded SLAM, not independent ground truth. Each source record timestamp and all report input rows agree exactly. Relative transform is camera1-from-camera0. Existing `measure/masks/summarize` and EXP-016 rank functions are reused without changing formulas.

Forward reprojection uses finite positive source forward depth and a finite front-facing transformed point, without requiring target depth. Sampson is square-root Sampson distance in pixels, independent of either predicted depth, undefined if relative translation<=1e-6m. None of228 arm-pairs meet that degeneracy rule. Nonzero translation does not establish a well-conditioned reference, particularly under tiny motion. Source depth is forward-axis depth, not Euclidean range. Target-out-of-image points remain in the all-grid primary population; they are predictions, not verified visible matches.

Preserve all-grid and selected/retained populations separately. Retained points use the original inbound/outlier association; no failed or empty population is substituted. Report inbound, inbound with both positive depths<=50m, image thirds, source-depth bins and invalid depths using the same fixed boundaries as before. There are456 point files,264 window/group summaries and5,016 pair/group summaries. These overlapping groups are diagnostic views, not independent replication.

## Sensitivity and selection

MSA inbound-only spatial reprojection rho is0.328/0.340/0.512 for day original/early/late and0.701/0.707/0.239 for night early/middle/late. With inbound and both depths<=50m it is0.337/0.418/0.554 and0.659/0.695/-0.019. Thus the primary positive reprojection association is not uniformly robust to visibility/depth restriction. Late-night Sampson rho stays negative: -0.219 inbound and-0.278 with both depths<=50m. Full masks/counts/strata and retained results are preserved; no subgroup is chosen to replace the primary outcome.

Median retained matches per pair original→MSA are172→179,147→150,169→167,197→199,193→194 and176→195 in the six fixed-window order. MSA median fraction of raw variance components below the native0.0625px² floor is18.3%,34.5%,11.1%,22.9%,0.69%,33.6%. These are passive characterizations before flooring, not tuned rejection thresholds. More retained observations or low uncertainty alone does not establish motion accuracy; EXP-023's late-night trajectory still estimates almost no displacement.

## Temporal associations remain inconsistent

Reuse EXP-016's protocol on19 per-pair finite medians in each case/arm, separately for all-grid and inbound points. Partial rank means Pearson correlation of ranked uncertainty/error after least-squares projection on intercept and ranked proxy translation magnitude, rotation magnitude, predicted flow norm and inverse positive source depth. A secondary adjustment adds ranked pair index. This is observational nuisance regression, not a fitted sensor correction, causal adjustment, uncertainty calibration or predictor. Predicted depth/flow and error targets share statistical dependencies; adjustment can remove signal or introduce bias.

| MSA window | Raw temporal rho reprojection | Context-adjusted reprojection | Context + time reprojection | Raw temporal rho Sampson | Context-adjusted Sampson |
|---|---:|---:|---:|---:|---:|
| Day original |-0.546|-0.417|-0.538|-0.521|-0.408|
| Day early |-0.004|-0.011|0.065|-0.204|-0.294|
| Day late |-0.321|-0.268|-0.183|-0.332|-0.395|
| Night early |-0.354|-0.440|-0.496|-0.254|-0.185|
| Night middle |0.344|0.264|0.264|0.398|0.101|
| Night late |0.130|-0.227|-0.190|0.089|0.003|

No shared positive temporal relationship emerges. The positive night-middle association does not transfer to other windows. Two of12 MSA all-grid target associations change raw sign between the fixed first9 and last10 pairs. First-difference, flow-only comparator, context+time, all/inbound and original-arm results are preserved in full CSV/JSON tables. No split, sign or target is selected after outcomes. These temporally dependent roughly1.9s windows support neither significance/forecasting claims nor a conclusion that useful uncertainty information cannot exist elsewhere.

## Verification, source controls and recovery

- 21 relevant tests pass in the pinned Blackwell image, including active-backend residual equivalence, depth-independent epipolar geometry, missing/degenerate inputs, retention masks and partial-rank controls. Initial CPU geometry-image test run had20 passes and one missing-Torch dependency failure; the complete21-test suite then passed in the Torch-equipped image. No model inference was performed.
- Independent optical reprojection, scalar OpenCV Sampson, mask/count/percentile checks and rank calculations agree across270,316 point scores in456 files. There are9,084 within-pair rank checks and96 inverse-correlation-matrix partial-rank checks. All66 original-night thermal group summaries reproduce EXP-015 to numerical tolerance.
- The first independent reprojection check used general float64 inverse-K algebra and found about5e-5px roundoff difference from the original float32 componentwise backprojection. The validator was corrected to honor source precision before double-precision pose/projection math; the original failed log is retained. No analysis outputs or tolerance were changed to repair a scientific outcome.
- All278 frozen hashes pass. Source images/calibration/checkpoints, earlier analyses and EXP-023 trajectories remain unchanged. Both PNG/PDF comparison figures were inspected.

Artifacts: `results/msa-uncertainty/run-001/{plan,analysis}.json`, full group/temporal/retention CSVs,456 point NPZs and24 temporal arrays; independent `validation.json`, `spatial-comparison.png/.pdf`, `temporal-associations.png/.pdf`, commands/logs/validator/plot under `staging/`. Analysis uses CPU image `sha256:a8a2dfcd2246cbf18e9fb0529ca9f65d63586eda47e3ade8c57f27d14f3c2e73` (OpenCV4.10.0); tests use the same pinned `macvo:blackwell` image as EXP-023. Exact image IDs/package inventories and command metadata are retained.

Replay with ore-ms mounted at `/workspace`, working directory `/workspace/MAC-VO-main`:

```bash
python -m Scripts.Setup.td3_msa_uncertainty prepare --output /workspace/results/msa-uncertainty/new-run
python -m Scripts.Setup.td3_msa_uncertainty run --output /workspace/results/msa-uncertainty/new-run
```

Preparation checks inherited EXP-023 sources, then freezes the actual saved prediction/proxy inputs used here. Existing-plan analysis needs no GPU or weights. Recovery under Box `ore-ms/backups/2026-10-21-exp-024` preserves analysis code/reports/notebook, all new outputs/validation, saved uncertainty/proxy inputs, source metadata and an incremental Git bundle. Prior EXP-014/023/source recovery supplies inherited context; models, source images, credentials and full archives are excluded.

## Decision

Keep MSA as the experimental thermal baseline. The better calibration strengthens the case that old day depth-based residuals were confounded, while the main uncertainty limitation persists: pointwise reprojection association does not automatically become a stable temporal or cross-modal reliability score. Retain the full original/MSA comparison and do not fit a new weighting/fusion rule from these windows. RGB outputs were not recomputed, and native pixel uncertainty values are not established as physically comparable between cameras. Broader evaluation needs longer/new recordings and an independent reference, with image-space matching uncertainty, depth error and pose error treated as different targets.
