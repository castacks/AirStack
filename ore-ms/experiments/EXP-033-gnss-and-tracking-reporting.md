# EXP-033: Provisional GNSS displacement checks and accurate tracking diagnostics

Date: 2026-10-22 (America/New_York). Branch `ore_ms`, base `b203bb34d` plus frozen source. **A versioned reporting runner preserves original tracking flags before postprocessing, with poses/depth/tensor maps/raw uncertainties bitwise unchanged on three fixed controls. GNSS displacement magnitudes support reconsidering some SLAM-only short-window failure interpretations, while longer thermal early/late windows still show substantial net-displacement shortfall. GNSS quality, timing, axes, receiver origin and SO independence remain qualified; this is not a camera-pose accuracy calibration.**

## Scope and source evidence

The user authorized both tasks after the [owner replies](../docs/research/source-replies-2026-10-22.md). Geethika supports MSA applicability and tentatively believes NovAtel odometry is not consumed by Super Odometry. Matthew identifies direct GNSS-system output with approximately described UTM axes, jitter and unknown recording-specific RTK. These are attributed statements; the exact SO graph, receiver aiding/quality, timing and gps_frame physical origin have not been independently recovered. Yifei's checkpoint training split remains unknown.

No additional image acquisition is needed. The GNSS study uses all six EXP-031 short cases, all six EXP-032 longer cases and the original three MSA night thermal cases: 15 windows/modality cases, with no new selection based on GNSS agreement. Three predefined GNSS query offsets −0.1/0/+0.1s give45 supported case/offset comparisons. No offset is selected or adopted. The reporting controls reuse three fixed saved windows; inference is rerun only to verify that new diagnostics leave numerical behavior unchanged.

## Reporting change and future entry point

The original `run_td3_execution.py`, postprocessor and all frozen historical wrappers remain unchanged. New `run_td3_execution_v2.py` preserves their config/checkpoint/seed/frame/pose operations and adds a copy of graph `need_interp` flags **before** `system.terminate()`, plus a separate copy afterwards. `tracking-flags.npz` stores both. A pure helper adds `tracking_diagnostics` with original unsupported frame indices/count, post-refinement flag count, changed indices, retained observations and `ALL_PAIRS_SUPPORTED`, `SOME_PAIRS_UNSUPPORTED` or `NO_PAIRS_SUPPORTED`.

These states use the existing flags, with no fitted percentage cutoff. Flags may denote lost tracking or skipped keyframes; these controls use `AllKeyframe`. `PASS_EXECUTION_ONLY` remains a numerical execution status and is not silently redefined as sustained tracking or accuracy. The runner records completed-frame diagnostics before the final no-observation exception, so an all-zero failure keeps its original flags and zero observation count. Failed construction/partial inference does not gain invented completed measurements.

The baseline's `MotionInterpolate` still aliases and clears boundary flags; the reporting change observes rather than modifies that behavior. Both the original pre-refinement need-for-interpolation and the stored after-refinement flags are exposed. The new fields should be used to assess support; the historical `tracking.interpolated_frames` field retains its original post-refinement meaning.

| Fixed reporting control | Frames | Original unsupported pairs | Stored post-refinement flags | New support state | Bitwise-equal arrays |
|---|---:|---:|---:|---|---:|
| short-early-rgb | 20 | 0 | 0 | ALL_PAIRS_SUPPORTED | 531 |
| long-middle-rgb | 100 | 97 | 94 | SOME_PAIRS_UNSUPPORTED | 2611 |
| long-middle-thermal | 100 | 0 | 0 | ALL_PAIRS_SUPPORTED | 2611 |


All5,753 compared arrays across226 NPZ files have identical dtype, shape and logical array bytes, including poses, exact acquisition timestamps, first depth/covariance/validity, full tensor maps and217 raw temporal uncertainty files. NPZ container timestamps are not compared. Configs, checkpoints, original tracking summaries and retained counts match exactly;291 source/control files are frozen. The middle RGB run now exposes97 unsupported pairs despite94 final flags, without changing its filled trajectory. Supported short RGB and moving thermal controls remain at zero unsupported pairs.

Use the versioned runner for **new** bounded checks, leaving old wrappers available for exact historical reproduction. With GPU runtime and ore-ms mounted at `/workspace`:

```bash
python -m Scripts.Setup.run_td3_execution_v2   --dataset /workspace/results/long-night-validation/prepared-001/datasets/middle   --manifest /workspace/results/long-night-validation/prepared-001/datasets/middle/manifest.json   --output /workspace/results/new-reporting-check --modality thermal --frames 100   --record-uncertainty --acknowledge-provisional-geometry
```

The preserved control script records complete Docker flags/image ID. Old callers are intentionally not rewritten, because their source hashes are part of earlier scientific snapshots. New reporting is opt-in through the documented v2 entry point.

## Position-only GNSS method

Use the source-rate `interp_data.txt`/`interp_timestamps.txt` streams, preserving their established exporter convention rather than inferring resampling from filenames. Inputs have13 fields and `map -> gps_frame` metadata. Only finite XYZ positions with aligned strictly increasing integer-nanosecond timestamps are accepted; no extrapolation. Read positions as float64, subtract the global position/time origin, and linearly interpolate at each original modality acquisition timestamp plus the fixed offset. Large UTM coordinates would have float32 spacing0.5m on X and0.0625m on Y here, so casting before local subtraction would erase small increments. This is an arithmetic-risk demonstration, not a claim that the supplied files already lost those increments.

Report GNSS XY and3D displacement **magnitudes from each window's first sample**. Separately report source SLAM vehicle3D, nominal rectified-camera SLAM3D and saved VO3D magnitudes at original timestamps. Magnitudes tolerate rotation/reflection and a signed horizontal axis permutation; they discard direction and do not require guessing Matthew's approximate axis flip. GNSS XY is not relabelled camera-horizontal motion. Identical magnitudes can arise from different trajectories, so their differences are not ATE/RPE or vector position errors.

The exact point represented by gps_frame is unresolved: it may be an antenna, receiver-compensated point or other configured origin. No antenna-to-camera transform is assumed. Camera/vehicle/GNSS origins differ, and rotation/lever arm can alter their displacement magnitudes. No axis, heading, lever arm, scale or latency is fitted to the evaluated VO outputs. Receiver covariance, solution/fix/RTK flags are not available in the13-field export; no confidence weighting, jitter filtering or uncertainty calibration is introduced.

Both entire source streams contain occasional longer gaps (maximum1.852/2.541s), but all2,340 selected queries across45 case/offset combinations have scalar interpolation brackets at most41.560ms. Per-query support audits, source step/speed context and nearest-sample summaries are saved. This verifies numerical/temporal availability, not receiver accuracy or true synchronization. Source-rate jitter is not converted into a certified error bar. Net displacement is used instead of summed path length, which could accumulate GNSS jitter.

## Results at zero assumed GNSS offset

The last column is the endpoint-magnitude range under the three predefined offsets, **not a confidence interval**. Failed VO trajectories remain unavailable. The two long RGB outputs dominated by interpolation remain explicitly marked filled; their scalar values are not successful VO evaluations. No aggregate modality-accuracy ranking is computed.

| Fixed window/modality | GNSS XY / 3D net (m) | Vehicle / nominal camera SLAM net 3D (m) | VO net 3D (m) | GNSS3D fixed-offset range (m) |
|---|---:|---:|---:|---:|
| short-early-rgb | 0.0041 / 0.0100 | 0.0717 / 0.0717 | 0.0097 | 0.0098–0.0100 |
| short-early-thermal | 0.0041 / 0.0100 | 0.0703 / 0.0703 | 0.0039 | 0.0098–0.0100 |
| short-middle-rgb | 0.0201 / 0.0256 | 0.1651 / 0.1651 | unavailable | 0.0254–0.0256 |
| short-middle-thermal | 0.0200 / 0.0255 | 0.1644 / 0.1644 | 0.0036 | 0.0255–0.0257 |
| short-late-rgb | 3.8504 / 3.8519 | 5.3036 / 5.2276 | unavailable | 3.8270–3.8669 |
| short-late-thermal | 3.8525 / 3.8539 | 5.3041 / 5.2220 | 3.7316 | 3.8294–3.8690 |
| long-early-rgb | 31.7243 / 31.7263 | 30.7129 / 30.7256 | unavailable | 31.4639–31.9673 |
| long-early-thermal | 31.7130 / 31.7149 | 30.6866 / 30.7020 | 26.4536 | 31.4517–31.9554 |
| long-middle-rgb | 32.4676 / 32.5181 | 32.4592 / 32.4792 | 4.4313 (filled) | 32.4625–32.6084 |
| long-middle-thermal | 32.4713 / 32.5215 | 32.4599 / 32.4775 | 31.7170 | 32.4640–32.6112 |
| long-late-rgb | 31.4780 / 31.4810 | 32.0053 / 31.7936 | 0.0000 (filled) | 31.3404–31.6005 |
| long-late-thermal | 31.4686 / 31.4715 | 31.9953 / 31.7647 | 26.7432 | 31.3312–31.5912 |
| canonical-night-early-thermal | 1.4292 / 1.4298 | 1.7622 / 1.7589 | 1.4324 | 1.3373–1.4859 |
| canonical-night-middle-thermal | 5.5886 / 5.5887 | 6.1709 / 6.1621 | 4.7680 | 5.5177–5.6168 |
| canonical-night-late-thermal | 0.0161 / 0.0161 | 0.2608 / 0.2610 | 0.0085 | 0.0099–0.0161 |


In the canonical late-night1.9s interval, GNSS3D displacement is0.0161m and supplied median linear speed0.0097m/s, versus thermal VO0.0085m and nominal camera SLAM0.2610m. The GNSS offset range0.0099–0.0161m stays small. These observations weaken a network-only explanation of the earlier SLAM/VO discrepancy. They do not certify standstill, centimetre accuracy or which origin/reference is physically correct.

Canonical early thermal is1.4324m VO versus1.4298m GNSS and1.7589m nominal camera SLAM. The new short late thermal window is3.7316m VO versus3.8539m GNSS and5.2220m nominal camera SLAM. Thus the apparent size of short-window translation disagreement depends materially on the reference. Near equality of scalar magnitudes does not validate directions, calibration or full trajectories.

On the longer thermal windows, GNSS3D is31.7149/32.5215/31.4715m versus VO26.4536/31.7170/26.7432m. Early/late retain about5.26/4.73m net-displacement shortfall even against this provisional GNSS stream, while middle differs by about0.80m. Fixed ±0.1s sensitivity is retained in the table; no correction is adopted. Origin, reference quality and timing uncertainties still apply, so these are displacement discrepancies, not calibrated camera translation errors.

![Scalar displacement comparisons at distinct physical origins](../results/gnss-and-tracking/gnss-displacements.png)

## Validation and reproducibility

13 focused tests pass: three tracking-summary tests, four GNSS math tests, one controlled real-runner no-observation failure test and five prior adapter/uncertainty controls. They cover all/some/no original support, boundary flag changes without input mutation, invalid flag shapes/types, large-coordinate precision, translation/signed-axis invariance, timestamp boundaries/no extrapolation and invalid source chronology/data. The controlled failure confirms that `FAILED` output retains original flags and zero observation count even when the final stored flag was cleared.

Three real GPU reruns meet the primary **bitwise equality** criterion on5,753 arrays, with unchanged configs/checkpoints and original raw report status. Independent scalar bracket/interpolation/norm calculations verify all45 GNSS case/offset records and2,340 query positions, source SLAM body endpoint magnitudes and every available VO endpoint norm.45 GNSS point files/396 arrays and all case/source records are identical between the preserved initial run and final run; the protocol/figure label was corrected before publication from assumed “antenna” to exported `gps_frame`, with no numerical change. Initial source is preserved and all828 initial hashes verify using that source snapshot; final `gnss-002` has828 frozen hashes. Prior EXP-0323,041/1,229, EXP-031751/267, EXP-030729, EXP-028402 and EXP-02997 hashes remain unchanged.

Runtime: exact GPU `sha256:f429af4afa234fc1e37db6867ed9ee9c708af852ae755fe7d6e8150f70136300` on RTX PRO5000 Blackwell48GB, PyTorch2.7.1+cu128; same thread/seed/FP32/config as original saved controls. CPU GNSS/validation/plots use `macvo:geometry`101a3943f3d1/NumPy2.2.6. Exact images/packages/source/commands and logs are preserved. No model, camera calibration, weighting, source timestamp, interpolation algorithm or original output is changed.

Artifacts under `results/gnss-and-tracking/`: frozen `gnss-001/` and `gnss-002/` plans/full scalar curves, `reporting-001/` frozen control plan/three fresh outputs/flags, reporting and GNSS validators, source-gap and offset audits, CSV/figure, metadata, tests and replay scripts/logs. Earlier dataset/model/calibration snapshots remain prerequisites. Box `ore-ms/backups/2026-10-22-exp-033` preserves this new evidence/source and incremental Git bundle; credentials/checkpoints/full source archives are excluded.

## Decision and remaining inputs

Use v2 tracking diagnostics in new work and retain execution status separately from measurement support. GNSS is now useful as an explicitly provisional separately generated translation cross-check; retain both GNSS and SLAM results rather than replacing either with unquestioned ground truth. Short-window disagreement cannot be assigned to VO alone; longer thermal discrepancies remain measurable at the scalar level. Do not train fusion against these uncalibrated magnitude differences.

No additional image upload was needed. The remaining useful reference information is the exact GNSS axis convention/origin/lever-arm configuration and recording-specific covariance/solution/RTK-quality/timing evidence already requested from the owners. Yifei's training-recording list remains pending before training-held-out claims. New night data with visible RGB structure would help a subsequent numerical cross-modal comparison, as recorded earlier; it is not needed to reproduce these two completed checks.
