# EXP-046: MS² saved stereo outputs against projected-depth proxies

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `edf86b41e`.
**The fixed first20 images show positive within-image covariance/error-proxy association in both modalities, with substantial support/reference dependence. This is conditional agreement with supplied projected references, not physical uncertainty calibration or a cross-modal winner result.** All120 image/depth-variant cases and independent validation complete; no new inference or VO.

## Frozen experiment

Execute [EXP045](EXP-045-ms2-reference-protocol.md) without changing IDs, timing, depth variants, calibration, preprocessing or weights. Use EXP044's20 saved stereo fields per modality, MS² night `_2021-08-13-21-18-04`, IDs000000–000019. Keep the primary `depth` single-scan label and separate `depth_multi`/`depth_filtered` sensitivities. The thermal recipe remains `td3-firestereo-v2-clahe-bilateral-v1`; its checkpoint-training equivalence is still unverified.

Before scoring, freeze the analysis definitions in the local design spec and run configuration. Per image/variant, retain every nonzero-depth pixel satisfying `0 <= u−fx*B/Z < width−1`, with no model-dependent selection. Compute expected forward flow `(-fx*B/Z,0)` and signed residual `observed−expected`. Save all selected coordinates, depth, target/predicted flow, diagonal covariance, signed residual and common-support flag. Preserve native float32 prediction values as float64 for analysis.

The common support is the intersection of all3 depth-product masks **within the same modality and image**, not shared physical points between RGB and thermal. No resizing, spatial interpolation, uncertainty clipping, confidence gating, residual threshold, pose use or fitted correction is introduced. The strict right-coordinate bound is inherited from the protocol; no image sampling is performed in this diagnostic.

Normalize residual by `(fx,fy)` and diagonal variance by `(fx²,fy²)`. Define `U=trace(normalized diagonal covariance)` and `E=L2(normalized residual)`; calculate per-image Spearman rank correlation. Also retain pixel-unit association and signed/absolute component summaries. Each modality has equal `fx=fy` within its own image, so pixel and normalized representations preserve this within-image rank association; they do not create matching scene populations across modalities.

All40 original stereo outputs are finite, correctly shaped and have positive diagonal covariance. All120 cases have nonempty own/common support; none is dropped or repaired. A computational PASS reports execution/validation, not a physical reference-quality pass.

## Per-product support result

Each table entry is the median of20 **per-image** summaries. Brackets show the image-correlation minimum and maximum; they are not confidence intervals. Support totals are coverage counts, not independent sample sizes. The20 images are adjacent and previously inspected, not independent or verified training-held-out evaluation cases.

|Modality|Depth product|Support count across20 images|Median image L2 residual (px)|Median image normalized L2 ×1000|Median image rho [min,max]|
|---|---|---:|---:|---:|---:|
|RGB|Single scan (primary)|141,472|1.8765|2.4545|0.353 [0.279,0.411]|
|RGB|Merged32|385,381|1.7328|2.2666|0.364 [0.276,0.417]|
|RGB|Network filtered|295,751|1.3109|1.7147|0.403 [0.317,0.495]|
|Thermal|Single scan (primary)|184,792|1.5453|3.9848|0.453 [0.406,0.505]|
|Thermal|Merged32|280,636|1.7200|4.4355|0.393 [0.346,0.496]|
|Thermal|Network filtered|219,300|1.5606|4.0243|0.453 [0.414,0.549]|

The primary within-image rank association is positive in20/20 RGB images and20/20 thermal images. This establishes an association with the chosen proxy on the observed populations. It does not quantify coverage calibration or imply covariance magnitudes are physically correct.

## Common-support sensitivity

|Modality|Depth product|Support count across20 images|Median image L2 residual (px)|Median image normalized L2 ×1000|Median image rho [min,max]|
|---|---|---:|---:|---:|---:|
|RGB|Single scan (primary)|12,497|1.5896|2.0792|0.120 [-0.027,0.220]|
|RGB|Merged32|12,497|1.5568|2.0363|0.091 [-0.056,0.269]|
|RGB|Network filtered|12,497|1.0923|1.4287|0.339 [0.094,0.540]|
|Thermal|Single scan (primary)|32,925|1.6256|4.1921|0.385 [0.332,0.481]|
|Thermal|Merged32|32,925|1.6115|4.1556|0.373 [0.316,0.467]|
|Thermal|Network filtered|32,925|1.6016|4.1302|0.449 [0.400,0.555]|

Common support totals12,497RGB pixels (8.83% of primary support) and32,925thermal pixels (17.82%). These are different, smaller scene populations. On this fixed support, RGB primary rho falls to0.120 (positive18/20 images); merged-depth rho is0.091 (positive16/20). Thermal primary rho is0.385 (positive20/20), with merged/filtered medians0.373/0.449. No depth variant or subset is selected as a preferred result after scoring.

RGB filtered-depth association remains0.339 on exactly the same common coordinates, versus0.120 for single scan. Thus support selection alone does not explain all reference sensitivity: changing the depth labels at fixed coordinates also changes the residual ranking. The filtered product includes network-dependent selection/construction and is not an independent physical accuracy label. Do not promote its stronger association to a calibrated reliability claim.

Signed horizontal/vertical medians, median/p90 absolute components, L2 median/p90 and covariance-trace medians are retained for every case in `report.json`. There are no pooled-pixel p-values or pooled rank correlations. Comparing the RGB and thermal rows as camera winners would confound resolution, FOV, support, label construction, timing and checkpoint training.

![Within-image covariance versus projected-reference disagreement](../results/ms2-stereo-depth-proxy/run-001/uncertainty_error_spearman.png)

![Per-image median normalized reference-proxy residual](../results/ms2-stereo-depth-proxy/run-001/l2_median.png)

Standalone PDF versions are preserved beside the PNG figures. Normalized image-plane residuals are not exact angular error or physical3D error.

## Timing and interpretation limits

Every record retains EXP045's four original camera timestamps and failed inherited joint gate;0/20 paired groups satisfy it. No clock shift, re-pairing or threshold relaxation is applied. Official same-ID pairing supports a supplied-pair diagnostic, not verified simultaneous exposure. Sparse projected LiDAR labels additionally retain motion, calibration, projection/occlusion and reference-noise uncertainty; merged products depend on navigation/ICP and filtered products on a stereo network. No NLL/calibration consistency claim is made without accounting for that label uncertainty.

This tests raw **stereo correspondence covariance**, not temporal matching covariance, full3D or pose uncertainty. It therefore does not validate EXP038/039's temporal cross-modal ranking rules. The observed association is a useful baseline diagnostic, with explicit population/reference sensitivity, and no fusion/default change follows.

## Reproduction and independent validation

CPU runtime in `macvo:ms2-smoke`: Python3.11.13, NumPy2.2.6, OpenCV4.10.0, SciPy from the recovered image. One thread is requested for NumPy/BLAS; no model or GPU computation is used. Startup driver diagnostics are retained in logs. From `MAC-VO-main`, with AirStack mounted at `/root/AirStack`:

```bash
python -m Scripts.Setup.ms2_stereo_depth_proxy \
 --root /root/AirStack/ore-ms \
 --manifest /root/AirStack/ore-ms/results/ms2-timestamp-semantics/source-protocol-input-manifest.json \
 --protocol-audit /root/AirStack/ore-ms/results/ms2-reference-protocol/audit.json \
 --output /path/to/fresh-run

python -m Scripts.Setup.validate_ms2_stereo_depth_proxy \
 --root /root/AirStack/ore-ms \
 --run /path/to/fresh-run \
 --output /path/to/fresh-validation.json
```

The validator's historical checks also require EXP043 and EXP044-complete manifests mounted at `/root/ore-ms-recovery`. It uses division-free support inequalities, reconstructs all selected arrays from raw PNG and original saved prediction fields, checks1,920 scalar camera-model points, independently computes every per-image quantile with scalar sorting/interpolation, and implements tied average ranks and Pearson reduction without calling the scorer or SciPy rankdata. It independently checks all aggregate medians/ranges and complete120-case coverage. Constant/tied rank controls are included.

All16,380 independent checks pass across1,507,332 selected points, with292 frozen inputs unchanged and350 historical source/result hash checks passing. Full arrays, correlations and summary statistics validate; the checks do not establish physical truth. The scorer refuses existing output directories; the validator refuses existing validation files.

Results: `results/ms2-stereo-depth-proxy/run-001/`, including protocol/frozen registry/report,120 per-case NPZs, validation JSON and figures. Source model outputs remain unchanged. Recovery target Box `backups/2026-10-25-exp-046`, with code/results/notes/plots/independent validation and incremental Git bundle. EXP043+EXP044-complete+EXP045 remain prerequisites; checkpoints/full dataset archives/packages/tokens are excluded.

## Next bounded work

Before applying a cross-modal selection rule, distinguish useful uncertainty ranking from scene/depth-dependent covariance and proxy-label effects, and obtain a justified shared error target under timing/reference assumptions. A fixed, explicitly exploratory depth/spatial-stratified control on these same saved fields could test whether the within-image association survives comparable depth/scene strata; it would not be a held-out or physical calibration result. Independent recording/condition variation and author acquisition/reference provenance would materially strengthen the study. No additional upload is needed to reproduce or stratify this pilot.
