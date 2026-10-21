# EXP-026: Longer day RGB and MSA thermal runs

Date: 2026-10-21 (America/New_York). Branch `ore_ms`, base `b16a98c61` plus frozen longer-run source. **Six fixed100-frame RGB/thermal attempts; no parameter tuning, scale fitting or production-default change. Reference remains recorded SLAM with a nominal body anchor.**

## Acquisition and protocol

The source is `20251027/0_salon_so_day`, with1,335 synchronized groups/~163s. Three100-frame segments are selected by chronological thirds, excluding every prior EXP-02520-frame sample. Choose the eligible midpoint nearest each third midpoint, ties earlier. Selection uses only timestamps and reference coverage, without maximizing motion or observing model/image outcomes. These are longer windows from the same recording, not a new independent dataset or coverage of the whole163s.

The original150ms maximum-gap selector failed for middle/late; their longest strict contiguous runs have49/79 rows (early139). Preserve that failure. Before image/model outcomes, an explicit metadata-only revision uses250ms for all three runs. Actual capture times and all gaps are retained; missing images are not synthesized. This is a dropped-capture experiment, not a claim that the original continuity gate passed.

| Window | Manifest start | Frames | RGB duration s | Thermal duration s | RGB max gap ms | Thermal max gap ms | Gaps>150ms RGB/thermal |
|---|---:|---:|---:|---:|---:|---:|---:|
|early|58|100|9.900|9.900|100.030|100.000|0/0|
|middle|584|100|10.200|10.200|200.001|150.000|3/3|
|late|1045|100|10.002|10.000|200.007|150.000|1/1|

Selected source image count1,800: RGB stereo, original processed thermal stereo and native raw thermal stereo for300 acquisitions. ZIP member CRC/SHA checks pass; whole7.77GB archive not downloaded or rehashed. Eight raw timestamp associations are ambiguous within2ms; use the exact source processed index/timestamp, requiring exact old-producer image replay. All600 original processed thermal images reproduce pixel-for-pixel. Independent scalar selection/CRC/hash controls pass.

Generate separate MSA fisheye stereo images and camera-info using the unchanged EXP-023 recipe,640x512, zero-disparity,balance0,fov_scale1 and the same firestereo_v2/enhancement. RGB is copied byte-for-byte. Preserve original acquisition rows. The source MSA physical applicability and vehicle-body anchor remain provisional.

## VO results against recorded SLAM proxy

| Window | Camera | Status | Anchored translation RMSE m | Anchored rotation RMSE deg | Consecutive translation RMSE m | Consecutive rotation RMSE deg | Interpolated frames |
|---|---|---|---:|---:|---:|---:|---:|
|early|rgb|PASS_EXECUTION_ONLY|0.0246|0.1783|0.0056|0.0276|0|
|early|thermal|PASS_EXECUTION_ONLY|0.0325|0.6910|0.0059|0.0243|0|
|middle|rgb|PASS_EXECUTION_ONLY|0.4724|0.8602|0.0317|0.1236|0|
|middle|thermal|PASS_EXECUTION_ONLY|0.6442|1.4448|0.0768|0.6129|0|
|late|rgb|PASS_EXECUTION_ONLY|0.7929|1.4064|0.0257|0.1206|0|
|late|thermal|PASS_EXECUTION_ONLY|0.5107|1.7561|0.0514|0.5586|0|

Use original checkpoints/effective configs,seed0,fp32,mappingfalse and synchronous optimization, with passive uncertainty/first-signed-stereo capture. No weights, threshold, dropout repair, restart-based outcome selection or optimized alignment is introduced. Each trajectory is first-pose anchored only, without fitted scale/rotation/translation. SLAM is evaluated at exact modality acquisition times; use the original nominal vehicle-from-raw-left anchor and each modality own rectification/NED conversion. Filtered odometry was not promoted to independent GT; provenance remains EXP-025.

Anchored drift, consecutive translation error and consecutive rotation error are different targets. Temporal pose associations below use camera1-from-camera0 increments, the same direction as image residual geometry, while the standard trajectory metric uses forward world pose increments; both are explicitly retained. Errors are not normalized by traveled distance and tiny-motion rotational/reference sensitivity remains.

## Uncertainty and geometric errors

All fixed source-grid points are primary, selected/retained counts are passive context. Each complete case provides99 chronological frame-pair records. Use finite positive source forward depth/front-facing projection for reprojection, no target-depth gate. Sampson is depth-independent with translation<=1e-6m declared degenerate. Inbound sensitivity is separate. Fixed-grid pixels under different modality calibrations are not the same physical rays.

| Window/camera | Raw trace px² | Reprojection px | Sampson px | Spatial rho reprojection | Spatial rho Sampson |
|---|---:|---:|---:|---:|---:|
|early/rgb|0.001|0.347|0.099|-0.488|0.019|
|early/thermal|0.574|0.262|0.057|-0.300|-0.104|
|middle/rgb|2.470|2.206|0.664|0.495|0.244|
|middle/thermal|4.858|2.729|1.040|0.564|0.309|
|late/rgb|0.452|1.818|0.604|0.479|0.304|
|late/thermal|1.307|2.638|1.021|0.589|0.281|

These are medians over99 pair medians or within-pair Spearman correlations, not pooled independent samples. Native pixel uncertainty/residuals are not physically calibrated across cameras. Spatial ranking and temporal covariation are reported separately.

Temporal statistics use99 pair medians per case: raw Spearman, partial-rank nuisance adjustment for proxy translation/rotation magnitude, predicted flow norm and inverse positive source depth; secondary add ranked pair index. Correlate raw covariance trace with reprojection, Sampson, consecutive translation error and consecutive rotation error separately. Fixed first49/last50 splits and adjacent first differences are preserved. No target/sign/subgroup is chosen after outcomes, and no significance or predictive/causal/probability calibration claim is made. Model-derived depth/flow and errors remain statistically coupled; adjustment may remove signal or induce bias.

| Window/camera | Target | Raw temporal rho | Context adjusted | Context+time | First differences | First49 / last50 |
|---|---|---:|---:|---:|---:|---:|
|early/rgb|forward_px|0.346|0.224|0.212|0.321|0.267 / 0.410|
|early/rgb|sampson_px|0.383|0.166|0.165|0.357|0.222 / 0.568|
|early/rgb|pose_translation_error_m|0.068|0.102|0.116|0.224|0.062 / 0.061|
|early/rgb|pose_rotation_error_deg|0.323|0.090|0.087|0.242|0.119 / 0.573|
|early/thermal|forward_px|0.178|0.143|0.150|0.303|0.159 / 0.229|
|early/thermal|sampson_px|0.145|0.146|0.156|0.340|0.181 / 0.169|
|early/thermal|pose_translation_error_m|0.034|-0.012|-0.013|0.074|-0.015 / 0.119|
|early/thermal|pose_rotation_error_deg|0.338|0.274|0.272|0.597|0.302 / 0.399|
|middle/rgb|forward_px|0.185|0.143|-0.017|0.057|0.057 / 0.042|
|middle/rgb|sampson_px|0.095|0.041|-0.059|0.020|0.005 / -0.072|
|middle/rgb|pose_translation_error_m|-0.015|-0.081|-0.011|0.087|0.001 / -0.235|
|middle/rgb|pose_rotation_error_deg|0.208|0.193|-0.076|-0.177|-0.103 / 0.163|
|middle/thermal|forward_px|0.344|0.389|0.207|0.084|0.306 / 0.184|
|middle/thermal|sampson_px|0.341|0.324|0.058|-0.012|0.131 / 0.147|
|middle/thermal|pose_translation_error_m|0.197|0.226|0.080|0.248|0.137 / -0.067|
|middle/thermal|pose_rotation_error_deg|0.372|0.382|0.166|0.013|0.226 / 0.207|
|late/rgb|forward_px|-0.271|0.080|0.198|0.135|-0.007 / 0.052|
|late/rgb|sampson_px|-0.383|-0.128|0.068|0.134|-0.008 / -0.159|
|late/rgb|pose_translation_error_m|-0.108|0.151|0.251|0.265|0.149 / -0.016|
|late/rgb|pose_rotation_error_deg|-0.310|-0.221|-0.090|-0.013|-0.073 / -0.218|
|late/thermal|forward_px|-0.063|0.138|0.146|0.129|-0.177 / 0.180|
|late/thermal|sampson_px|-0.111|0.062|0.072|0.103|-0.198 / 0.064|
|late/thermal|pose_translation_error_m|-0.123|0.091|0.123|0.143|0.161 / 0.159|
|late/thermal|pose_rotation_error_deg|-0.202|-0.078|-0.069|0.065|-0.261 / -0.013|

Complete all/inbound statistics, masks and temporal arrays are retained. No frame-level reliability rule or fusion model is trained here. Interpretation and limits follow below.

## Verification and recovery

21 relevant tests pass, including three new selector tests for midpoint/exclusion, explicit dropped-capture gaps and coverage/insufficient data. All3039 preparation hashes and638 analysis hashes pass. Independent source selection/exclusion/image replay checks pass. Independent pose/quaternion errors, optical reprojection, scalar OpenCV Sampson, rank and inverse-correlation partial-rank controls pass on833976 grid point scores and96 partial-rank summaries.

Initial setup failures are retained: a staging file named select.py shadowed Python standard-library select, then was renamed before any image/model analysis; an acquisition-path variable indentation error was fixed before extraction. The strict150ms scientific eligibility failure is separately documented, not hidden as a setup failure. A first independent anchored-translation check differed by5.43e-10m because it used equivalent algebra in a different order; an absolute1e-8m numeric-control tolerance was added, without changing analysis outputs or any scientific criterion. Its original log is retained. Prior EXP-025 sources and outputs remain untouched.

Artifacts under `results/long-salon/`: frozen selection/acquisition and source hash records, `prepared-001/` separate datasets/camera-info/manifests, `vo-001/` all six attempts with signed depth/uncertainty capture, point/temporal arrays, proxy pose arrays, `analysis.json`, `validation.json`, complete pair/association CSVs and three PNG/PDF figures. Exact source/staging scripts, tests, commands/image IDs/package inventories are retained. Geometry image is the pinned OpenCV4.10 CPU image; GPU image is the same Torch2.7.1+cu128 Blackwell image used in EXP-023–025.

Replay with ore-ms mounted at `/workspace`, workdir `/workspace/MAC-VO-main`: `python -m Scripts.Setup.td3_long_recording prepare --output <new-prepared>`, then `execute --plan <prepared>/plan.json --case early --modality thermal --output <new-case>` and other cases/modalities. Acquisition/analysis/validation/plot scripts in recovery preserve exact choices and commands.

Box recovery `ore-ms/backups/2026-10-21-exp-026` includes selected source images/metadata, all new preprocessing/VO/analysis outputs, source/tests/report/notebook and incremental Git bundle. Excludes credentials, checkpoints, full archive and disposable range blocks; prior model/source/calibration recovery is a prerequisite. No production-default, checkpoint, old image/calibration or prior trajectory changes.

## Interpretation and next step

The longer windows introduce a different motion regime. Early is nearly stationary: proxy net displacement is0.0188m RGB/0.0192m thermal across9.9s, with median per-pair proxy translations0.0040/0.0046m. Spatial uncertainty/reprojection rho is **-0.488 RGB and-0.300 thermal**, contrasting with positive moving-window values0.479–0.589. These tiny motions and small image residuals are especially sensitive to reference noise, depth bias and scene dynamics; the negative association does not establish a cause or justify reversing uncertainty weights. It does show the earlier positive spatial association cannot be assumed universal outside the motion-selected samples. No epipolar pair meets the formal1e-6m degeneracy threshold, but that numerical criterion does not make near-static reference geometry physically reliable.

Middle and late proxy net displacements are about39.2m and40.6m. RGB has lower anchored translation error in middle (0.472m vs0.644m), while thermal has lower anchored translation error in late (0.511m vs0.793m). RGB has lower anchored rotation error in all three. On both moving windows, RGB also has lower consecutive translation and rotation RMSE even where thermal has less accumulated translation drift. Drift accumulation and per-step error rankings therefore must remain separate targets. Low early RMSE should not be compared directly to moving-segment RMSE as a modality-quality score.

There is some target-specific positive evidence: context-adjusted temporal reprojection associations are positive across all six primary cases (rho0.080–0.389), but adding time weakens/reverses middle RGB (-0.017). Raw reprojection/epipolar associations change sign in late windows. Per-step translation-error raw rho is weak/mixed (-0.123 to0.197), and rotation-error raw rho changes from positive early/middle to negative late. Context controls, time, first differences and chronological halves yield different strengths/signs. Inbound pose sensitivities use an inbound-derived uncertainty/context summary for the same camera-pose target; they are not new pose observations. Some positive rotation/difference associations exist, but no transferable predictive relation is established from one recording.

Retained matches never fall below10 (RGB early200 throughout, middle167–200,late181–200; thermal early173–200,middle57–193,late85–200). Thus successful retention across these runs cannot validate the failure-warning threshold or explain away pose drift. Signed stereo/depth consistency controls pass, but the frontend still uses absolute disparity and does not certify all dense predictions.

These results favor evaluating image-space matching uncertainty, translation reliability and rotation reliability separately, with motion regime and time controls, rather than using one raw trace score as a general pose-quality or cross-modal fusion weight. Keep MSA as the experimental thermal baseline; retain all weights/defaults. Next useful evidence is longer independent recordings or reference-provenance confirmation, and a separately scoped low-motion/night failure analysis. The previously discussed late-night failure investigation is not claimed completed here. No probability calibration, forecasting, independent physical accuracy or generalization beyond this capture is asserted.
