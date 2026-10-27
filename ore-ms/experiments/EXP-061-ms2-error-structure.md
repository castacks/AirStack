# EXP-061: Range, image-region and tail structure of cross-camera errors

Date: 2026-10-27 America/New_York. Branch `ore_ms`; starting commit `47bf0ed8b`.

**Finding: the tested night window has distinct conditional regimes: thermal has lower mean3D disagreement below20m; RGB has lower mean disagreement at20–80m.** Every frame shows this direction in each of those five bands. RGB’s overall absolute-risk advantage survives fixed shared1% and5% tail removals. Its advantage is therefore not a universal night-camera result or merely the worst few points. Rain’s camera ordering, by contrast, is highly tail-sensitive. No fit, rule or default is adopted.

## Frozen diagnostic

Before computing the new summaries, record six reference-range bands `[0,5),[5,10),[10,20),[20,40),[40,80),[80,infinity)` metres and a fixed3×3 native RGB image grid. Range is Euclidean distance from the RGB origin to the midpoint of original candidate reference endpoints, not either camera’s axial depth. RGB native shape384×1224 is verified in EXP044 and the fixed overlay layout:1224+640 wide,384+40 high. Grid rows/columns are128/408 pixels. All bands/cells, including empty and sparse ones, remain explicit.

Use all three existing opening20-ID recordings, unchanged both-valid masks, predictions/errors and candidate references. Report full,6 range,9 grid and54 joint strata per frame/anchor, without minimum-count exclusion. Median point error and mean/median relative error use the same support. Relative error divides by the fixed midpoint reference range; it changes range weighting and is a separate estimand, not a correction to absolute error.

Tail sensitivities remove `ceil(n*f)` mass at `f=1%` and`5%`, ranked by midpoint `max(E_RGB,E_thermal)`. Share cutoff ties fractionally for permutation invariance. Apply identical weights to both cameras and all anchors. These reference-assisted diagnostic masks are never the new primary support or deployable policies. Tail outputs report weighted mean risks and removed loss fractions. Midpoint/RGB/thermal anchors and mean-L2/squared losses are all retained. No frame/pixel pooling for inferential claims, causal attribution, significance test, camera-model training or inference.

## Night range results

Each error entry is the median over20 per-frame group means. Ratio entries are medians of paired per-frame ratios, so ratios of displayed median errors need not equal them. RGB-lower counts compare per-frame mean-distance risk. The >=80m group has only2–8 pairs per frame and should not drive conclusions.

| Range (m) | Pairs/frame min / median / max | RGB mean error (m) | Thermal mean error (m) | RGB/thermal paired ratio | RGB lower frames |
|---|---:|---:|---:|---:|---:|
| <5 | 153 / 173.5 / 184 | 13.562 | 0.287 | 35.992 | 0/20 |
| 5–10 | 1050 / 1075.0 / 1113 | 1.992 | 0.601 | 3.098 | 0/20 |
| 10–20 | 1139 / 1184.5 / 1229 | 3.802 | 2.853 | 1.366 | 0/20 |
| 20–40 | 1055 / 1092.0 / 1119 | 4.736 | 9.595 | 0.496 | 20/20 |
| 40–80 | 713 / 735.0 / 758 | 15.388 | 22.557 | 0.683 | 20/20 |
| >=80 | 2 / 5.0 / 8 | 52.313 | 67.484 | 0.778 | 20/20 |

Overall night median per-frame mean absolute errors are6.022m RGB and7.396m thermal, paired median ratio0.811 (RGB lower20/20). However median point errors are3.118m RGB and2.733m thermal; mean relative errors are0.372 RGB and0.231 thermal. Thus “better” depends on the target loss and range weighting. Comparing medians does not identify the camera winning each point; overall RGB pointwise win fraction remains56.5%. None of these quantities certifies physical accuracy.

## Fixed tail sensitivities across recordings

Ratios below1 favor RGB. Entries are medians of20 per-frame RGB/thermal risks, followed by RGB-lower frame counts. These are fixed-camera errors, not learned-selector performance.

| Recording | Loss | Full support | Remove top1% shared error | Remove top5% shared error |
|---|---|---:|---:|---:|
| rain | mean_l2 | 4.079 (0/20) | 0.993 (10/20) | 0.773 (20/20) |
| rain | mse | 211.074 (0/20) | 1.849 (3/20) | 0.499 (20/20) |
| day | mean_l2 | 0.594 (20/20) | 0.577 (20/20) | 0.591 (20/20) |
| day | mse | 0.447 (18/20) | 0.373 (20/20) | 0.400 (20/20) |
| night | mean_l2 | 0.811 (20/20) | 0.813 (20/20) | 0.815 (20/20) |
| night | mse | 0.640 (20/20) | 0.646 (20/20) | 0.616 (20/20) |

Night’s direction survives both removals under both losses. Rain’s top1% shared-error set contains a median70.3% of RGB mean-distance loss and99.5% of its squared loss, versus5.5%/10.6% of thermal losses; after the fixed5% removal RGB is lower in all20 frames under both losses. This is substantial tail sensitivity, not evidence to discard those predictions or declare their labels bad. Day remains RGB-favoring across all these cases.

## Image regions and scene context

Night RGB-grid middle-left cell has paired mean-distance RGB/thermal ratio0.460, median522.5 pairs/frame and RGB lower20/20. Bottom-left and bottom-right have ratios1.253/1.635 and thermal lower20/20. Range and image location are associated; this is not an isolated location or illumination effect. The54 joint-cell summaries preserve their counts/risks without selecting a rule. Per-image signed gap contributions across ranges/cells sum exactly to the full mean-L2/MSE gap; medians of separate group contributions do not generally sum to the median full gap.

Inspected original fixed night overlays000000/000009/000019: visible street lighting and a brightly illuminated roadway with RGB lane markings and edges; thermal renders surrounding structures/vegetation more visibly. This establishes scene context, not the light source’s exact identity or a causal explanation of the error ordering. RGB/thermal imagery has different resolution/display processing. The thermal overlay uses a display-only1–99% stretch. No new image/depth inference is performed.

![Range and tail structure](../results/ms2-error-structure/figures/error-structure.png)

## Endpoint sensitivity and validation

| Recording | Midpoint full mean-L2 RGB/thermal | RGB-endpoint | Thermal-endpoint |
|---|---:|---:|---:|
| rain | 4.078964 | 4.085945 | 4.071577 |
| day | 0.593886 | 0.595899 | 0.592187 |
| night | 0.811145 | 0.811110 | 0.811183 |

All per-frame endpoint, range, spatial, joint-cell, loss and tail summaries remain in the JSON report, including empty cells. Reference range/cell memberships and midpoint tail rankings are held fixed across endpoints. Anchor robustness remains conditional on inherited support/reference validity.

Independent validator PASS: 946,023 recorded checks: 126 hashes, 800,646 reconstructed error values, 131,481 group fields, 4,320 tail fields, 7,470 summary fields, 720 historical risks, 1,260 partition gap checks. The validator imports no diagnostic code, independently reconstructs3D errors from saved predictions/reference endpoints, assigns scalar range/grid memberships, recomputes statistics/tail weights/aggregation, reproduces historical full risks and verifies exact partition/gap accounting. These are dependent implementation checks, not independent scientific observations.

Separate synthetic controls pass fractional cutoff ties, permutation invariance, exact retained mass, range/grid boundaries and empty cells. All126 original support/metric/report hashes remain unchanged. This revalidates saved-array geometry/accounting; original raw network inputs, physical calibration and exact physical correspondence are not newly validated. Existing availability limitations remain inherited.

CPU Python3.12.3 / NumPy2.2.6; plot Matplotlib3.10.7. Scripts refuse existing output paths.

```bash
python MAC-VO-main/Scripts/Setup/ms2_error_structure.py --root /root/AirStack/ore-ms --output /path/to/fresh-run
python MAC-VO-main/Scripts/Setup/validate_ms2_error_structure.py --root /root/AirStack/ore-ms --source /path/to/fresh-run --output /path/to/fresh-validation.json
python MAC-VO-main/Scripts/Setup/plot_ms2_error_structure.py --source /path/to/fresh-run --output /path/to/fresh-figures
```

## Method implications and papers

The empirical next question is whether deployment-available predicted geometry/context adds cross-camera error information beyond learned uncertainty and camera-scale baselines. Reference range and error-tail masks cannot become inference-time features. Declare a model, target loss and recording-level evaluation before fitting; all inspected windows remain development evidence. Independent recordings/stronger references would materially strengthen transfer evaluation. Existing saved supports suffice for this audit; no extra upload is needed.

[Targeted paper reading](../docs/research/uncertainty-method-papers-2026-10-27.md) distinguishes calibration from useful ranking/selection (Levi et al.), stereo-error-distribution learning (Chen et al.) and shifted-distribution evaluation (Ovadia et al.). These motivate evaluation design, not adoption of a method or an explanation of our sensor result. Accessible primary-source versions were sufficient; no PDF request to the user is currently needed.

Recovery target: Box `ore-ms/backups/2026-10-27-exp-061`; includes new scripts/notes/results/figures, pre-analysis notebook and incremental Git bundle. Scientific input prerequisites are original EXP057/058 support+metric/report files. Full datasets/checkpoints/private credentials and full paper PDFs/text excluded.
