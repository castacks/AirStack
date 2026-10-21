# EXP-027: Independent image-flow check of late-night low motion

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `75b8defff` plus frozen diagnostic source. **An independent Lucas–Kanade image tracker also measures very little motion in the late-night window. The current evidence identifies disagreement between image-derived motion and SLAM-plus-predicted-depth geometry; it does not isolate a model, optimizer, reference or calibration fault. No inference, weights or defaults changed.**

## Why this check matters

EXP-023's MSA late-night thermal VO estimated about0.0085m net displacement against the recorded SLAM camera proxy's0.2610m over1.9s, while retaining many observations. That established a proxy discrepancy, but did not establish that the thermal model failed to follow actual image motion. This experiment checks that distinction using a second image-tracking algorithm on the exact saved source pixels.

Fixed inputs: all19 consecutive pairs in MSA night early/middle/late and day-late from EXP-023, plus the first20 frames/19 pairs of EXP-026 early thermal as a near-static day control. These are previously examined windows, not fresh independent validation. No additional acquisition, GPU inference or outcome-based window reselection. Freeze222 source/image/pair/proxy/trajectory/helper/test hashes before analysis. Use all1008 saved source-grid points per pair, retaining failures and masks.

The late-night source is `20251027/night_thermal_so_salon2`, original paired-manifest start789. Thermal acquisition times are **2025-10-27 22:13:40.207–22:13:42.107 EDT (America/New_York)**. This exact interval is useful for checking the original bags/reference provenance with the dataset owner.

## Primary measurements

| Fixed thermal window | LK-valid points /19,152 | Learned flow (px) | LK flow (px) | Model–LK endpoint error (px) | LK–SLAM/depth endpoint error (px) | LK–VO/depth endpoint error (px) |
|---|---:|---:|---:|---:|---:|---:|
| Night early |17,031|2.042|1.529|0.412|2.367|0.456|
| Night middle |13,818|6.327|4.850|0.723|7.011|0.862|
| Night late |18,298|0.083|0.053|0.103|0.925|0.101|
| Day late moving |13,003|4.027|2.415|0.591|1.266|0.574|
| Day near-static |18,094|0.088|0.051|0.100|0.280|0.112|

Values are unweighted medians of19 per-pair medians on the **same LK-valid points**. They are not means over a shared set of physical features across windows. The learned and LK motion magnitudes are computed separately; their endpoint disagreement need not equal the difference of their median magnitudes. No tracker/SLAM/depth output is treated as ground truth.

Late-night LK motion resembles the near-static day control. Its small flow is not confined to sky or foreground vehicle structure: the middle image third has7,423 valid observations with0.043px median LK flow,0.067px learned flow,0.077px model–LK disagreement and0.945px LK–SLAM/depth disagreement. The upper third has very large model-predicted depth (median354m), so weak translation-induced image motion there is expected under the model; this is not a verified scene distance. The middle third's median predicted depth is6.08m. Depth validity cannot be established from the same model alone.

## Fixed tracking and geometry protocol

Decode original processed thermal PNGs as native uint8 grayscale without resizing. Track each saved grid point with OpenCV pyramidal Lucas–Kanade:21x21 windows,maxLevel3,30 iterations/epsilon0.01,minEigThreshold1e-4,flags0 and no initial flow. Run forward and backward. Primary validity requires both status flags, finite endpoints, target inside the same32px border and backward-return distance<=1px. Status failures are retained, not interpreted as stationary. This is algorithmic image registration with photometric/texture limitations, not independent physical motion truth.

Sensitivity uses the same tracks with backward-return distance<=0.5px, and separately primary tracks with source21x21 local intensity standard deviation>=5. Masks were fixed before outcomes. Report image thirds separately. Forward/backward agreement is a consistency gate, not proof that correspondence is correct; common errors and local convergence remain possible. Fast/occluded moving points may fail more often, so cross-window track acceptance fractions are explicit.

Use saved model endpoints at exactly the same source grid coordinates; no flow interpolation is needed. Preserve raw temporal covariance trace and source predicted forward depth. Project that depth under saved SLAM-relative camera motion, VO-relative camera motion and rotation-only SLAM motion using the same R/P/NED conventions as prior diagnostics. Translation contribution means the norm of full proxy-projected endpoint minus rotation-only endpoint; it is not a fitted independent translational flow. All source K,depth,pose and image-coordinate arrays are retained.

SLAM camera arrays retain the original nominal body anchor and own MSA rectification. This geometry and timing remain provisional. VO/depth internal consistency is expected because VO fits model observations; small LK–VO/depth residual is a useful consistency observation, not an independent certificate of correct poses. No pose, scale, depth, uncertainty or calibration parameters are fitted in this experiment.

The `all` population preserves every grid point and all status flags; raw LK numbers there can include invalid tracks and are not the primary LK result. Primary interpretation uses `lk_valid`. Full point arrays plus all/valid/tighter/texture/image-third summaries are saved.

## Late-night sensitivities and image checks

- 18,298/19,152 grid observations pass the primary tracker gate (**95.54%**), with948–978 valid tracks per pair. The stricter0.5px gate retains the same observations; primary median backward-return distance is0.0055px.
- Texture>=5 retains10,447 observations: LK motion0.043px,learned motion0.090px,model–LK disagreement0.102px, LK–SLAM/depth disagreement0.613px. Thus the discrepancy persists under the fixed texture restriction.
- Median SLAM/depth translation contribution is0.877px on primary valid tracks (0.602px under texture restriction), far above median observed LK motion. Full projection disagreement remains0.925px, so the check is not merely ignoring predicted rotation.
- All five windows contain20 distinct decoded grayscale image arrays and zero exactly equal consecutive frames. This excludes exact frame duplication in these selected processed images; it does not prove source acquisition freshness or exclude subtler resampling effects.
- Late-night median whole-image consecutive intensity MAE is0.206 on a0–255 scale; near-static day is0.319,night early2.557,night middle5.515,day late8.371. Intensity change is affected by enhancement and thermal variation, so these are descriptive context rather than motion truth.
- Late-night retained VO observations remain179–199 per pair. Selection starvation is not supported for this case. Retention does not imply that metric translation is observable.

Representative first/last frames and pair9 (the fixed midpoint pair) were inspected, including the LK-valid flow map. No images or region masks were selected after examining error outcomes. Image previews and figures are saved alongside numeric results.

## Verification and artifacts

Seven relevant tests pass in the pinned geometry image: known synthetic identity/translation recovery, textureless images, backward-return boundary/status/NaN handling, and previous geometric controls. Independent optical backprojection/projection, direct patch standard deviations, forward/backward masks, exact saved model-pixel association and scalar endpoint/aggregation calculations pass on95,760 grid scores and665 overlapping pair/population summaries. All222 frozen hashes remain unchanged.

The initial independent `hypot` versus source float32 `norm` check differed by at most7.63e-6px at roughly64px magnitude. The validator's numeric tolerance was corrected to absolute1e-6px/relative3e-7 to accommodate float32 rounding; original failed log retained. No data outputs, validity gates or scientific thresholds changed. Source code, exact commands, image ID and package inventory are preserved. No GPU/model operation was performed.

Artifacts under `results/night-low-motion/`: `run-001/plan.json`, `analysis.json`,95 point arrays, full population/pair summaries; independent `validation.json`, `decoded-image-checks.json`, `flow-comparison.png/.pdf`, `representative-images.png/.pdf`, scripts/logs under `staging/`. Prior experiment outputs, models/calibration/images/defaults are untouched.

Replay with ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`, geometry image `sha256:a8a2dfcd2246cbf18e9fb0529ca9f65d63586eda47e3ade8c57f27d14f3c2e73` (OpenCV4.10.0):

```bash
python -m Scripts.Setup.td3_low_motion prepare --output /workspace/results/night-low-motion/new-run
python -m Scripts.Setup.td3_low_motion run --output /workspace/results/night-low-motion/new-run
python -m pytest -o addopts= Scripts/UnitTest/test_td3_low_motion.py Scripts/UnitTest/test_td3_night_geometry.py
```

Box recovery `ore-ms/backups/2026-10-21-exp-027` includes fixed source images/pair/pose inputs, source/tests/report/notebook, all new diagnostics/validation/figures and incremental Git bundle. Prior EXP-023/026 source/calibration context is required. No checkpoints, credentials or whole source archives are included.

## Decision

Reclassify the late-night result as **unresolved image-motion versus proxy/depth disagreement**, retaining its measured0.0085m versus0.2610m trajectory discrepancy. Independent LK supports the learned model's low image-motion measurement, weakening the diagnosis that the network alone suppresses visible motion. The optimizer's near-zero trajectory is consistent with its input correspondences; this does not prove the optimizer is correct or exclude interaction with predicted depth/weights.

Possible remaining contributors include SLAM drift at low motion, timing/reference association, incorrect scene depth/stereo calibration, and shared limitations of two image trackers. These data do not identify one cause. Do not force a larger motion by changing weights, flip a depth scale or mark the thermal model failed solely from this proxy discrepancy. The next reference-provenance question should include the exact late-night interval and whether the vehicle was stopped or creeping, alongside upstream filtered-odometry sensors, timing and quality. Keep MSA as the experimental baseline and retain unresolved metric/body uncertainty.
