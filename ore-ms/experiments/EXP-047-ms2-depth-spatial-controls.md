# EXP-047: Fixed depth/spatial controls of MS² stereo proxy association

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `3578ce400`.
**Positive proxy association survives coarse fixed depth/spatial controls on each product's own support. RGB common-support association remains weak and sensitive to eligibility; these controls do not establish physical calibration or cross-modal reliability.** No inference, new images, calibration fit, timing correction or default change.

## Question and frozen choices

[EXP046](EXP-046-ms2-stereo-depth-proxy.md) found positive within-image covariance/disagreement association with sparse projected-depth references and sensitivity to scene support/reference labels. This follow-up asks whether a descriptive within-bin association remains after coarse depth/spatial grouping. It is exploratory on the same previously inspected, near-static20 source IDs, not an independent test or optimized method.

Freeze all choices in the local design spec before scoring: depth bands `(0,5),[5,10),[10,20),[20,40),[40,80),[80,infinity)`meters, a3×3 equal image grid, and at least30 points per stratum with nonconstant uncertainty and error. Source zero-depth pixels were already excluded in EXP046. Labels at bin edges enter the upper band. Spatial bins use `floor(3*y/H),floor(3*x/W)` on original native pixel coordinates. Depth-only has6 possible strata, spatial-only9, joint54. The minimum30 is a fixed descriptive availability rule, not a statistical-independence guarantee.

Retain all120 original image/depth-product cases and both own/common support, yielding240 case/support combinations and720 controls. For each own-support case, group using that product's depth. For common support, use **primary single-scan depth at the exact shared coordinates for every depth variant**, so group assignments remain fixed when reference labels change. No new residual/confidence selection is added. Preserve sparse/constant strata explicitly; never adjust the bins to improve scores.

Use EXP046's saved normalized diagonal covariance trace `U` and L2 normalized residual `E`. Within each eligible stratum of size `n`, average tied ranks and define `a=(rank(U)−(n+1)/2)/n`, `b=(rank(E)−(n+1)/2)/n`. Report `sum(a*b)/sqrt(sum(a*a)*sum(b*b))` over all eligible strata within one image. Since each stratum's ranks are centered, this equals Pearson correlation of concatenated within-stratum centered normalized ranks. It retains stratum-size and tie weighting. It is **not a standard partial-Spearman statistic, causal adjustment, or significance test**.

Also report full-support raw Spearman and raw Spearman on the exact eligible pixels for each control. These baselines separate population loss from within-bin weighting. The within-bin coefficient and raw Spearman have different estimands/weighting; their differences are descriptive, not percentages of confounding or variance explained. Aggregate only medians/minima/maxima over the20 adjacent images.

## Primary single-scan result

Each coefficient is a median of20 per-image summaries. Bracketed limits are the observed image minimum/maximum, not confidence intervals. Common support means the intersection of all3 depth products within one modality/image; it is not shared scene points between modalities.

|Modality / support|Control|Full-support raw rho|Same-retained raw rho|Within-bin coefficient [min,max]|Median retained %|
|---|---|---:|---:|---:|---:|
|RGB / own|depth|0.353|0.353|0.289 [0.221,0.350]|99.97|
|RGB / own|spatial|0.353|0.353|0.213 [0.129,0.261]|100.00|
|RGB / own|joint|0.353|0.354|0.179 [0.123,0.227]|99.52|
|RGB / common|depth|0.120|0.086|0.063 [-0.086,0.222]|97.71|
|RGB / common|spatial|0.120|0.077|0.102 [-0.032,0.213]|89.35|
|RGB / common|joint|0.120|0.010|0.044 [-0.096,0.211]|82.04|
|Thermal / own|depth|0.453|0.454|0.521 [0.472,0.558]|99.99|
|Thermal / own|spatial|0.453|0.453|0.308 [0.274,0.346]|100.00|
|Thermal / own|joint|0.453|0.448|0.388 [0.307,0.420]|99.53|
|Thermal / common|depth|0.385|0.385|0.543 [0.477,0.623]|100.00|
|Thermal / common|spatial|0.385|0.385|0.335 [0.284,0.419]|100.00|
|Thermal / common|joint|0.385|0.406|0.365 [0.265,0.453]|89.34|

Own-support joint controls retain a median99.52% RGB/99.53% thermal pixels. Their within-bin coefficients remain positive in20/20 images for both cameras. Coarse depth grouping alone raises thermal association, whereas spatial grouping reduces it; the result does not support a universal story that all observed association is inflated by depth. The controls leave continuous-depth, texture, visibility, calibration and reference-noise dependencies unresolved.

RGB common-support joint conditioning retains a median82.04% of that already small support (image range67.42–90.29%). Raw rho changes from0.120 to0.010 on retained pixels **before** the within-bin coefficient0.044 is calculated; that coefficient is positive14/20 images and ranges−0.096 to0.211. It would be incorrect to attribute the full reduction from0.120 to0.044 solely to depth/spatial conditioning. Thermal common-support joint retention is89.34% (82.53–96.28%); same-retained raw rho0.406 and within-bin coefficient0.365, positive20/20 images.

The original EXP046 common-support populations are12,497RGB and32,925thermal points across20 images, versus primary own-support141,472/184,792. These are dependent coverage counts. All differences remain within these different selected scene populations; camera-winner conclusions are unsupported.

## Reference sensitivities

|Modality|Depth product|Own-support joint coefficient|Common-support joint coefficient|
|---|---|---:|---:|
|RGB|Single scan (primary)|0.179|0.044|
|RGB|Merged32|0.170|0.054|
|RGB|Network filtered|0.253|0.199|
|Thermal|Single scan (primary)|0.388|0.365|
|Thermal|Merged32|0.398|0.340|
|Thermal|Network filtered|0.302|0.354|

All three products remain separate, with unchanged primary designation. RGB common-support network-filtered coefficient0.199 exceeds single-scan0.044 and merged0.054 at fixed group assignments. Reference-product dependence therefore remains after these controls. The network-filtered product is not independent physical truth. Do not select it or tune bins on this result.

Additional descriptive depth diagnostics retain per-image Spearman of `U` and `E` with `1/Z` (the grouping depth; primary single scan for common support). Primary own-support medians: RGB `rho(U,1/Z)=0.093`, `rho(E,1/Z)=0.254`; thermal0.316/−0.152. On common support they are RGB0.010/−0.125 and thermal0.582/−0.260. Opposite or changing depth associations caution against interpreting pooled covariance/error relations as simple geometry-independent reliability. These numbers are descriptive, not a causal explanation.

![Primary fixed controls and retained-pixel baseline](../results/ms2-depth-spatial-controls/run-001/primary-controls.png)

Standalone PNG/PDF artifacts are retained. Black curves use the joint control's exact retained pixels; depth/spatial controls have their own retained-pixel baselines in the report.

## Validation and preservation

All240 case/support combinations and720 control results are available. Of16,560 potential strata,7,132 meet the fixed availability rule and9,428 are sparse (including empty strata); none is excluded for constant U/E in this run. Every unavailable stratum remains in the report with count/reason. Availability is computational, not physical validation.

Independent validation does not import the scorer. It implements scalar interval/grid membership, Python sorting with tied average ranks, scalar normalized-rank moments, and independently reconstructs all raw/restricted/within-bin coefficients, aggregates and nuisance correlations. It checks bin-edge/tie/constant controls, exact240-case coverage, source timing records and preservation. All41,416 checks pass across16,560 strata and1,643,598 point-membership checks;416 frozen inputs and350 historical source/result hashes remain unchanged. Counts include repeated support/variant memberships, not unique independent observations.

One-thread CPU analysis in recovered `macvo:ms2-smoke`; no network, GPU or model computation. NumPy2.2.6 and SciPy from that runtime; rank validation uses the independently implemented helpers from EXP046's validator. Startup driver diagnostics remain in logs. The failed physical joint timing gate remains0/20; reference direction, exposure semantics, navigation quality and training split remain unresolved. No image, model output, GT field, calibration or default changes.

From `MAC-VO-main`, with AirStack mounted at `/root/AirStack`:

```bash
python -m Scripts.Setup.ms2_depth_spatial_controls \
 --root /root/AirStack/ore-ms \
 --source /root/AirStack/ore-ms/results/ms2-stereo-depth-proxy/run-001 \
 --output /path/to/fresh-run

python -m Scripts.Setup.validate_ms2_depth_spatial_controls \
 --root /root/AirStack/ore-ms \
 --source /root/AirStack/ore-ms/results/ms2-stereo-depth-proxy/run-001 \
 --run /path/to/fresh-run \
 --output /path/to/fresh-validation.json
```

Historical hash checks require EXP043 and EXP044-complete manifests mounted at `/root/ore-ms-recovery`. Fresh output paths are required. Results `results/ms2-depth-spatial-controls/run-001/` contain configuration,416-input hash registry, all per-stratum counts/reasons/coefficients, per-image and aggregate scores, validation JSON and plots. Box recovery target `backups/2026-10-25-exp-047`, with incremental bundle after EXP046, notebooks/code/figures/results. EXP043–046 recoveries and original checkpoints remain prerequisites for provenance checks; full archives/packages/tokens/weights are excluded.

## Outcome and next evidence

This pilot contains residual within-bin covariance/proxy-disagreement association, with weak/variable RGB association on common support and continuing reference dependence. The coarse controls neither remove all scene effects nor certify calibrated physical uncertainty. Do not adopt fusion weights, a cross-modal ranking policy or finer outcome-tuned bins from this result.

Independent recording/condition variation would materially strengthen this single-pilot result. The fixed official test candidates already named in EXP040 are MS² day `_2021-08-06-11-23-45` and rain `_2021-08-06-16-19-00`; their existing acquisition pointers/metadata should be checked before requesting any upload. Reusing the unchanged proxy protocol on a fixed independent condition is more informative than tuning these pilot bins. Physical temporal/cross-modal accuracy additionally requires acquisition/reference provenance; a second proxy recording cannot resolve that alone. No extra image upload is needed to replay these controls.
