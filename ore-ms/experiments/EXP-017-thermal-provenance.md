# EXP-017: Thermal preprocessing provenance and signed stereo geometry

Date: 2026-10-20T18:16:10.395704+00:00 UTC. Branch `ore_ms`; base commit `3ac07f047` plus source frozen in plan. **Completed: sampled preprocessing reproduced exactly; physical geometry remains unresolved.**

## Findings and decision

All **18 sampled processed thermal images match pixel-for-pixel** when replayed from timestamp-matched raw uint16 captures using the archived producer/config and OpenCV **4.10.0.84**. This covers 5,898,240 grayscale pixels and 17,694,720 channel values; all three supplied channels are exact replicas. There are no image or channel mismatches in this fixed sample. This resolves sampled computational compatibility with the inspected preprocessing pipeline. It does not prove the historical producer commit/runtime, all-frame equivalence, or checkpoint training provenance.

The same code/config under OpenCV 5.0.0 matches none of the18 images exactly: mean absolute image differences range0.0529–0.1512 on [0,255], maxima3–6. A single fixed OpenCV4.10 version sensitivity was declared before running it; no thresholds or image parameters were searched. Exact reproduction removes the need to explain those particular differences by an unknown normalization recipe. Prior VO runs consumed the original supplied PNGs; no earlier inference output is changed.

Saved left/right rectification R/P matrices reproduce exactly in both OpenCV versions using the producer's inverse matrix. The negative saved baseline is **not disproved by these night images**. Seven of nine fixed stereo samples have negative median left-minus-right disparity and a majority of positive triangulated depths under the saved signed projection. Middle positions9/19 are mixed, have weaker vertical alignment and favor the opposite sign among the fitted matches. No universal sign correction is adopted.

## Fixed sample and source provenance

Use positions0/9/19 of the original early/middle/late20-group night windows (manifest starts123/619/789). Retrieve18 raw PNGs and six raw metadata files from the original Box ZIP by byte ranges, with ZIP-member CRC verification. Raw and processed acquisition timestamps agree exactly for all18 associations. Missing raw timestamp rows are retained as missing when locating indices; no index compaction, replacement frames or nearest-index assumptions. Whole7.7GB archive SHA was not recomputed. All source images remain unchanged; newly extracted raw data live only under this experiment's inputs.

Public producer/config were fetched again and match the archived snapshots ignoring final newlines. Git blob IDs: script `b0b16f3aba8471f3e33ac5f363be59282b7c4e1a`, config `8279e675e47e9a55f6a7b84f322e297ab43eacba`. Local source SHA/URLs and fetch dates are in inputs/upstream/source-manifest.json. The producer's original function AST bodies execute without ROS imports or its main loop. Processing is rectification -> configured percentile conversion -> CLAHE -> bilateral filtering; source operation order and parameters are retained. Input copies prevent the original clipping function's in-place mutation from altering raw arrays. In OpenCV4.10, omitting enhancement or using the direct stereo matrix does not reproduce the supplied images; full differences are in images.csv.

Run-001 failed before image comparisons because OpenCV5 rejects strided R/T views passed by the original script. Its frozen plan, failed source/test snapshots and failure metadata are retained. Run-002 uses only a layout adapter (contiguous R; T reshaped3x1), with original numeric values and function bodies. Synthetic R/P equivalence tests pass. Run-003 uses the identical frozen analysis source with the declared OpenCV4.10 override; it is the exact preprocessing replay. No failed run is silently replaced.

## Signed geometry evidence

| Window / position | Inliers | Median disparity px | Median vertical error px | Front, saved P | Front, flipped translation |
|---|---:|---:|---:|---:|---:|
| early / 0 | 163 | -4.667 | 0.727 | 98.2% | 1.8% |
| early / 9 | 163 | -4.595 | 0.580 | 99.4% | 0.6% |
| early / 19 | 183 | -4.577 | 0.700 | 97.8% | 2.2% |
| middle / 0 | 78 | -1.005 | 1.436 | 97.4% | 2.6% |
| middle / 9 | 72 | 0.101 | 1.795 | 45.8% | 54.2% |
| middle / 19 | 94 | 0.770 | 2.059 | 26.6% | 73.4% |
| late / 0 | 54 | -3.462 | 0.259 | 100.0% | 0.0% |
| late / 9 | 49 | -3.397 | 0.338 | 91.8% | 8.2% |
| late / 19 | 50 | -3.507 | 0.350 | 88.0% | 12.0% |

Fixed SIFT cap2000; bidirectional ratio0.7 mutual matches; fundamental RANSAC1.5px/confidence0.999, seed0. All nine samples yield a fit, with906 inliers in total. All correspondences and masks are saved. A general fitted fundamental matrix can accept matches inconsistent with horizontal rectification; these are diagnostics, not verified true matches. Middle vertical discrepancies and nearzero disparities limit physical interpretation.

Using the same inliers, saved signed P triangulates778/906 in front of both cameras, versus128/906 when only right projection translation is flipped. The alternative has exactly opposite XYZ and identical median reprojection errors in every sample. This verifies why image reprojection alone cannot establish cheirality. Neither the aggregate count nor SIFT/RANSAC verifies metric scale, body extrinsics or absolute depth accuracy. The result differs from EXP-002's small day-image disparity diagnostic; day pixels were not re-staged here. Keep that cross-recording discrepancy explicit rather than extrapolating either sign across all recordings.

The body YAML's thermal left-from-right transform has translation approximately(+0.335056,0,0)m and identity relative rotation. Its disagreement with saved projection remains0.670113m horizontally,0.006144m off-axis and0.617019deg rotation. Relative to the body's right-from-left transform, configured Kalibr-as-stored differs0.006145m/0.617019deg; producer inverse differs0.670162m/0.617019deg. The inverse is internally consistent with saved R/P, but body extrinsics cannot be reconciled merely by changing labels. No metric calibration or body pose transform is replaced.

## Paper and API interpretation

[MACThermal thesis](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf), §2.1, provides the normalization/enhancement recipe inspected in EXP-016. The [producer](https://github.com/castacks/ros_torch_converter/blob/main/scripts/get_thermal_data.py) and [config](https://github.com/castacks/ros_torch_converter/blob/main/config/kitti_config/get_thermal_data.yaml) provide concrete implementation details. [Kalibr's format](https://github.com/ethz-asl/kalibr/wiki/yaml-formats) defines cam1 T_cn_cnm1 as cam0-to-cam1. [OpenCV stereoRectify](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html) also expects first-to-second camera coordinates. Thus the producer's inverse is a convention discrepancy relative to the declared Kalibr format. It is not sufficient evidence to change the supplied images/calibration, especially given night cheirality and unresolved physical camera identities. This interpretation is our inference from the inspected code, documented conventions and measured data.

## Validation, recovery and limits

67 tests pass under each of OpenCV5.0.0 and4.10.0.84. Synthetic checks cover original AST isolation, nonmutation, degenerate normalization, exact channels, array-layout compatibility, sign/depth/reprojection equivalence and consistent rectification. All783 inherited/new frozen hashes pass for both successful plans; raw ZIP CRCs, exact pixels and stored matches/triangulations are independently checked. PNG/PDF representative figure visually inspected.

Primary artifacts: `results/thermal-provenance/run-003/` (plan, analysis, images.csv, geometry.csv, arrays, validation, figure). Preserve run-001/002 and inputs/staging as well. Base CPU image remains `sha256:bf5d89c63af6c867c49aeee4db2bb4ef1486c1a65ac026521de35fb8495a1cc2`; run-003 overlays pinned opencv-python-headless4.10.0.84 without changing NumPy2.2.6. Exact commands/runtime extension hash are recorded in staging. Backup: Box `backups/2026-10-20-exp-017` with EXP-013/014/015/016 prerequisites. Full thesis PDF/text and credentials excluded.

The preprocessing gate is now substantially stronger on the night sample; training/test overlap, source-camera physical identity, stereo/body consistency and independent pose/depth reference remain open. We cannot resolve those physical facts from two conflicting calibration representations and image matches alone. Next bounded check: compare day/night raw camera ordering and the signed disparity predicted by the learned stereo model against these feature measurements, using the existing saved depth outputs before requesting new calibration evidence. Keep all current defaults. Cross-modal probability calibration/fusion remains premature.
