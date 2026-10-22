# EXP-035: night_so_waypoint_skip2 readiness

Recorded: 2026-10-22T21:09:07.897380+00:00; branch ore_ms; starting commit7b59219db.

## Result

The supplied archive adds another night capture with useful thermal structure. The three fixed sampled RGB windows remain nearly black and offer little independent LK support. A12-frame scout spanning the full capture also shows predominantly dark RGB, with small faint scene patches in some later frames. This is not an exhaustive rejection of every interval. No learned VO/uncertainty inference was run; no model failure is inferred solely from LK support.

## Source and timing

Box night_so_waypoint_skip2.zip, ID2543529906738,20,327,244,796bytes;54,855 safe unique ZIP entries under the expected root. Use bounded range reads; whole-ZIP SHA is not verified. Selected103 metadata/text members and360 images pass member CRC and SHA;12 extra RGB scout frames also pass CRC/SHA. No point clouds/model files are downloaded. Data resides under data/tartandrive3/20251027/night_so_waypoint_skip2.

Streams span approximately549.4s: RGB left/right4,991/4,993 unique acquisitions, thermal left/right4,927/4,944. Raw timestamp-binding rows have missing filenames/duplicates; preserve them in source and count them rather than inventing images. The six camera calibration metadata files equal prior baseline files. That supports consistent processing metadata, not independent physical calibration validation.

The existing chronological-nearest-without-reuse2ms stereo/30ms cross-camera policy retains184 groups and provides no eligible20-frame middle-third window under the150ms inter-frame cap. Preserve this strict manifest and failed preflight. Four prior paired-manifest overlap checks report zero matching acquisition stamps for these strict groups; this is not a guarantee of training holdout.

Nearest timing audit before any correction:

- RGB left/right: all4,991 left stamps have a right neighbor within2ms.
- Thermal left/right:2,897/4,927 nearest differences within2ms;4,899 within5ms. Many offsets lie just either side of−2ms, so float-export rounding affects threshold membership. Timestamp rounding does not establish sensor synchronization.
- RGB left/thermal left: nearest signed offset median−43.460ms (p10−46.420ms,p90−40.525ms); only295/4,991 within30ms. This is observed stream phase, not a fitted clock correction or known latency.

Stereo tolerance sweep1/2/3/5ms, with cross limit fixed30ms, retains91/184/281/281 groups. The cross-camera offset remains the main limitation on broad coverage. No axis/time/pose correction or baseline tolerance change is adopted.

## Separate image diagnostic

Before decoding selected frames, separately prescribe5ms stereo/30ms cross-camera diagnostic pairing. Its281 groups allow first eligible20-frame windows in each **group-count third**, at diagnostic row starts0/98/195. They do not evenly cover elapsed capture time, because pairing availability varies. All selected inter-frame gaps are positive and<=150ms and each thermal acquisition is within30ms of both RGB acquisitions. Image selection is fixed before image outcomes. Early/middle/late source SLAM body endpoint magnitudes are0.0104/6.3333/1.1720m, for motion context only.

Retrieve four processed streams plus exact-timestamp original thermal left/right for these60 groups:240uint8 processed images and120uint16 512x640 raw images. Original thermal source preprocessing is assessed here; **MSA rectification is not applied** in this readiness study.

Independent temporal LK uses the unchanged EXP02721x21 window,3 pyramid levels,30/.01 termination,minEig1e-4,16px native grid with32px border, forward/reverse status, finite inbounds endpoints and FB<=1px. Sensitivities FB<=.5px and local21x21std>=5 are retained. Repeat the same calculation on the three EXP031 source-image windows as a prior-dark control. No poses or learned outputs enter the support calculation. Native view/texture/resolution differences prevent interpreting these support fractions as physical modality accuracy.

| Recording / window | Modality | Median gray mean | Median gray std | Median LK support |
|---|---|---:|---:|---:|
| new-early | rgb | 18.021 | 1.611 | 4.056% |
| new-early | thermal | 168.011 | 65.387 | 95.933% |
| new-middle | rgb | 17.498 | 0.719 | 0.056% |
| new-middle | thermal | 168.596 | 66.638 | 69.544% |
| new-late | rgb | 17.527 | 0.717 | 0.000% |
| new-late | thermal | 176.497 | 60.454 | 95.337% |
| prior-dark-early | rgb | 17.723 | 1.979 | 6.056% |
| prior-dark-early | thermal | 186.203 | 60.363 | 96.131% |
| prior-dark-middle | rgb | 17.415 | 0.716 | 0.000% |
| prior-dark-middle | thermal | 179.738 | 62.936 | 95.635% |
| prior-dark-late | rgb | 17.312 | 0.715 | 0.000% |
| prior-dark-late | thermal | 182.430 | 64.674 | 80.655% |

![Selected windows; original display values](../results/waypoint-skip2-readiness/contact-sheet.png)

New RGB median valid grid counts are73/1/0 out of1,800; thermal median support is95.9/69.5/95.3%. These are correspondence availability measurements, not reliability calibration or proof of network failure.

## Full-capture RGB scout

After the fixed paired-window study, select12 equally spaced unique RGB acquisitions across all549s, independent of decoded image quality and pairing availability. No exposure enhancement. Their mean gray levels range17.32–18.38 and standard deviations0.71–3.16 on8-bit intensity. Some faint local structure appears near later samples. Single frames are not temporal tracking/accuracy labels; useful uninspected intervals may remain. Scout source selection and stats are separately preserved.

![12 evenly spaced RGB source frames](../results/waypoint-skip2-readiness/rgb-scout.png)

## Verification

All463 initial selected members and12 scout images pass source member CRC/SHA. All360 initial images decode at the expected raw/processed types. Independent scalar recomputation verifies320,112 LK gate points across228 temporal pairs (new and prior-dark controls), including tight/texture masks and absence of threshold-near rounding ambiguity.240 left images enter full-frame statistics.728 analysis source hashes pass. Earlier828/3,041/1,229/751/267 scientific hashes remain unchanged. Three existing low-motion tests pass. No GPU inference. CPU image sha256:101a3943f3d10fbd85611e0401364e94e343f0936d7cae535a396138987cdbb1, OpenCV4.10.0.

Initial metadata guard skipped the18.1MB source-rate SLAM file; an explicit32MB metadata cap plus8MB bounded reader chunks resolves it. Preserve earlier guard/preflight failures under staging/attempt-001 and logs. Primary2ms preflight failure remains a scientific availability result; the successful5ms diagnostic is separately named.

Reproduction with ore-ms mounted at /workspace, PYTHONPATH=/workspace/MAC-VO-main and macvo:geometry: run preserved staging/preflight.py, then from MAC-VO-main run python -m Scripts.Setup.td3_image_readiness. Reproduce in a separate restored workspace with results/waypoint-skip2-readiness/points absent; the module refuses to overwrite existing point outputs. Staging inspect_archive.py/stage_images.py/stage_scout.py retrieve exact Box members; staging/validate.py and analyze_scout.py verify source/support. Protocol, selections, all rows, decoded previews, point arrays and metadata are under results/waypoint-skip2-readiness. Reuse preserved outputs for exact archival reproduction; running preflight is deterministic but refreshes timestamps.

## Next evidence

No further upload is needed to use this recording for thermal diagnostics. For cross-modal numerical uncertainty comparison, visible RGB structure still matters: a candidate night recording preview or a known lit interval/folder name is more useful than another blind large ZIP upload. The faint late scout sections can also be inspected further within this existing recording. Any future relaxed pairing/inference study must retain its actual offsets and separate policy; do not silently replace the established strict baseline. Exact GNSS axes/origin/quality/timing and checkpoint training membership remain pending; this same-date recording is not verified training-held-out.
