# EXP-019: Fixed-grid spatial stereo consistency

Date: 2026-10-20 (America/New_York). Branch `ore_ms`, base `618f28af3` plus frozen EXP-019 source. **Completed: the feature sign contrast is strongly dependent on spatial sampling; physical calibration remains unresolved. No defaults changed.**

## Findings and decision

All six thermal inputs share a spatial pattern: predicted left-minus-right disparity is mostly negative in the upper third and positive in every sampled lower-third point. The fixed grid does **not** show a recording-wide day/night sign reversal. Night fitted features mostly occupy the upper third; day features mostly occupy the middle third. Thus the earlier difference in feature medians samples different parts of a spatially mixed field.

The table reports **model** positive-disparity fractions, sampled either on the fixed grid or at EXP-018's fitted feature locations. They are not independent correspondence correctness rates.

| Thermal input | Grid positive | Feature-location positive | Grid upper-third positive | Grid lower-third positive | Features in upper third |
|---|---:|---:|---:|---:|---:|
| day original | 61.3% | 66.3% | 0.3% | 100% | 49/163 |
| day early | 63.4% | 67.6% | 6.8% | 100% | 113/309 |
| day late | 61.1% | 55.1% | 0.0% | 100% | 51/138 |
| night early | 65.0% | 2.5% | 5.9% | 100% | 160/163 |
| night middle | 67.2% | 1.3% | 15.7% | 100% | 78/78 |
| night late | 60.3% | 0.0% | 0.0% | 100% | 53/54 |

All 288 lower-third thermal grid points per case also exceed **+1px**, so that region's positive sign is not a nearzero-disparity artifact. Middle-third positive fractions are 80.8–87.9%. The upper/middle/lower grid populations contain 324/396/288 points respectively, because the common border32/stride16 grid intersects unequal numbers of rows in the three image thirds. No population was rebalanced after outcomes.

The mixed signs remain under the fixed sensitivity requiring an inbound model endpoint and model vertical displacement <=1.5px: positive fractions are 55.1–62.9%. Among points with defined patch NCC, positive fractions remain 60.2–66.5%. A simple vertical filter or removal of uninformative patches therefore does not resolve the sign mixture. These conditions describe model/image diagnostics, not validity masks to adopt in VO.

## Image consistency and selection

Thermal source patches away from fitted features generally still contain usable intensity variation. The fixed-grid model-target NCC median is 0.892–0.971, versus 0.971–0.991 at fitted feature locations. Night grid points farther than16px from any fitted inlier comprise 85.5–94.2% of the grid and remain 64.0–75.2% positive. This broad population is not represented by the almost entirely negative night feature locations.

The lower third has larger predicted vertical displacement and lower normalized patch agreement than the upper third in all six thermal inputs:

| Thermal input | Upper median model vertical px | Lower median model vertical px | Upper median NCC | Lower median NCC |
|---|---:|---:|---:|---:|
| day original | 0.992 | 1.772 | 0.982 | 0.907 |
| day early | 0.995 | 1.690 | 0.985 | 0.893 |
| day late | 0.799 | 1.901 | 0.986 | 0.906 |
| night early | 0.645 | 1.584 | 0.975 | 0.805 |
| night middle | 0.543 | 1.789 | 0.943 | 0.746 |
| night late | 0.652 | 1.705 | 0.982 | 0.850 |

NCC is computed only for fully inbound finite9x9 patches whose source and target intensity standard deviations both exceed1 gray level. Undefined counts remain explicit. A single predicted center displacement translates the whole target patch; NCC is conditional on the model prediction and local translation approximation. Perspective deformation, occlusion, repeated texture, preprocessing and low texture can affect it. High NCC does not prove a true match or physically correct signed depth; lower NCC does not identify a physical cause.

Night RGB has very little qualifying patch evidence:

| RGB input | Source grid patches std<=1 | NCC-defined points | Median model vertical px |
|---|---:|---:|---:|
| day original | 202/1800 | 1581/1800 | 1.015 |
| day early | 204/1800 | 1580/1800 | 1.014 |
| day late | 207/1800 | 1576/1800 | 1.029 |
| night early | 1784/1800 | 14/1800 | 2.843 |
| night middle | 1800/1800 | 0/1800 | 2.871 |
| night late | 1800/1800 | 0/1800 | 16.020 |

The night-early NCC median0.783 describes just14 points, not the image. Middle/late NCC is undefined, not zero error or successful image agreement. The three empty night RGB feature populations remain empty. This supports keeping availability and texture evidence separate from the existence of finite model outputs or positive absolute depths. It does not establish calibrated uncertainty or an independent error reference.

## Frozen protocol and artifacts

Only the same12 first-frame stereo pairs and saved signed outputs from EXP-018 are used. No new inference, input download, frame selection, model fitting, calibration modification or VO execution occurred. Before analysis, the protocol and input/source hashes were frozen in `results/spatial-stereo/run-001/plan.json`.

The grid has32px border and16px spacing:1008 points per thermal image,1800 per RGB image, **16,848 total**. Every point is retained. Disparity is negative horizontal model flow; vertical displacement is absolute vertical model flow. Raw positive fraction uses disparity>0; the separate sign categories use >1, <-1 and absolute value<=1px. The supplied signed-P compatibility diagnostic retains principal-point offsets and uses the preserved positive-depth arrays; it is a convention check, never a physical truth label.

Spatial groups are equal image row thirds,3x3 tiles and each sampled grid row. Texture strata use source9x9 grayscale patch std<=1,(1,5],>5, with a separate invalid group. Feature proximity is <=16px to the original fitted left-image inlier locations. With no fitted inliers every grid point is in the no-nearby group; no fabricated nearest point is inserted. Sensitivities retain endpoint inbound, model vertical<=1.5px, their intersection, and NCC-defined populations. The feature locations are evaluated with the same model/patch measurements, separately from their independently detected EXP-018 coordinates.

Full per-point values, all group masks, nearest-inlier distances, undefined NCCs and **900 group summaries** are saved. Raw covariance trace is retained as a camera-local descriptive quantity, without cross-modal calibration or a learned weighting rule. Dense whole-image row-third signs/vertical statistics are also retained; their support includes borders excluded by the grid and must not be treated as identical populations.

Runtime: `macvo:geometry`, image `sha256:a8a2dfcd2246cbf18e9fb0529ca9f65d63586eda47e3ade8c57f27d14f3c2e73`, OpenCV4.10.0.84 and NumPy2.2.6. Package inventory, commands, test output and validation code are in `results/spatial-stereo/staging/`. Primary artifacts: `run-001/{plan.json,analysis.json,groups.csv,points/,thermal-spatial.png,rgb-spatial.png,population-comparison.png}` and companion PDFs.

## Validation, recovery and limits

17 numeric/regression tests pass. They cover synthetic known2D translations, wrong-target patch evidence, constant/invalid patches, out-of-bounds point retention, sign/texture/spatial partitions, no-feature and proximity-boundary cases, and EXP-018 regressions. All **190 frozen file hashes** pass before/after analysis and independent validation. Independent SciPy interpolation, Euclidean-vector NCC, brute-force nearest-feature distances, all per-point sign counts, group partitions and900 summary counts pass. All24 grid/feature populations are retained, including empty night RGB feature populations. Figures visually inspected. Existing algorithm code, calibration, model weights, signed arrays, saved depths and VO defaults remain unchanged.

Box snapshot: `backups/2026-10-20-exp-019`. It contains new analysis arrays, frozen plan, validation, figures, runtime/test metadata, source/tests/report and local notebook evidence. It requires the existing repository and EXP-018 snapshot (plus its inherited original inputs) to rerun; it does not duplicate credentials, models or the signed inference files. Archive/member hashes and an incremental source Git bundle are preserved.

The result strengthens the feature-selection caveat and localizes the thermal sign disagreement spatially. It does not identify whether physical calibration, mounting, preprocessing geometry, scene ambiguity or another cause explains that field. No universal sign correction, region mask, metric-depth correction or fusion rule is adopted. The user supplied the previously incorporated converter, pinned Yamaha extrinsics, tf_manager/demo, intrinsics and timestamp pointers again; no additional recording-specific physical mounting evidence is currently known. Authoritative physical thermal camera identities and per-recording stereo/body calibration remain the next external evidence needed; independent geometric reference remains necessary for uncertainty calibration.
