# EXP-040: Public paired-data access and reference audit

Date: 2026-10-23. Starting commit: `1fc0d767e`. This is a compatibility and metadata audit, not a MAC evaluation. The target remains relative geometric reliability of existing RGB/thermal MAC covariance. No training, inference, calibration fitting or fusion change.

## Decision and access

Prioritize **MS²**. The user supplied all three official download scripts during this audit; the nine fixed day/night/rain URLs were checked and return exact HTTP206 bzip2 bytes. It supplies RGB and thermal stereo, LiDAR depth products and camera-frame odometry. STheReO is a possible alternative; its small calibration/reference archives are downloaded, but the selected image archive currently returns Google Drive's quota-exceeded page. No STheReO image samples or MAC outputs were obtained. Do not claim either dataset is inference-ready.

| Candidate | Evidence and decision |
|---|---|
| MS² | Closest combined stereo/depth/pose fit. Access scripts now supplied by user; all nine fixed candidate URLs verified. |
| STheReO | Stereo RGB/thermal and LiDAR/IMU/GPS-derived reference. Actual metadata audited; image download quota blocks bounded inspection. |
| TartanRGBT | Current dataset card identifies `odometry/poses.npy` as raw MACVO output and depth as FoundationStereo predictions. These cannot serve as independent accuracy labels for MAC. Thermal-to-ZED timing also requires sidecar inspection. |
| CATS | Current official site requests access. Static scene disparity would address a separate stereo study, not the current temporal comparison. No hidden legacy links used. |
| FIReStereo | Thermal stereo/depth is documented; a paired stereo RGB stream is not verified. Does not yet satisfy the combined requirement. |

Primary sources: [MS² repository](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset), [MS² data format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format), [download](https://sites.google.com/view/multi-spectral-stereo-dataset/download); [STheReO download](https://sites.google.com/view/rpmsthereo/download), [paper](https://sangwoojung98.github.io/pdf/inter_conf/yun2022sthereo.pdf), [player](https://github.com/rpmsnu/file_player_sthereo_dataset); [TartanRGBT card](https://huggingface.co/datasets/theairlabcmu/TartanRGBT/blob/main/README.md); [CATS](https://bigdatavision.org/CAT/download.html); [FIReStereo](https://github.com/firestereo/FIReStereo).

## MS² acquisition and adapter requirements

Pin the inspected official loader at `0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284`. A bounded first pilot should use the first lexical sequence in each official test split: day `_2021-08-06-11-23-45`, night `_2021-08-13-21-18-04`, rain `_2021-08-06-16-19-00`. This is an acquisition plan, not a selected successful evaluation. After access, inspect the first 20 common timestamp stems per sequence, including both endpoints' reference availability; preserve missing matches rather than pairing sorted indices blindly. Confirm training overlap separately.

Request RGB left/right, raw thermal left/right, `calib.npy`, RGB/thermal odometry, and both single-scan and filtered projected depth for these sequences. Preserve uint16 thermal values; do not reuse an 8-bit `/255` adapter. Official processed sizes are RGB 1224x384 and thermal 640x256. Official calibration translations require mm-to-m conversion; encoded depth is uint16 divided by 256, zero invalid. Confirm these conventions against actual files before implementing an adapter.

The official filtered depth pipeline includes network-based stereo consistency filtering; it is not wholly model-independent ground truth. Keep single-scan versus filtered depth as an explicit sensitivity. The webpage describes 3x4 poses while the loader reshapes 4x4 then slices: inspect real files, transform direction, camera frame, timestamps and valid depth coverage before numerical comparison. No pose convention is inferred from a filename alone.

## Actual STheReO Valley Evening metadata

Public [folder](https://drive.google.com/drive/folders/1_eshMaMBBl2HqSXMUPumw5u2HrQnxRuj). Listed image archive: 9,774,131,200 bytes; sensor archive: 7,490,375,680 bytes. Only calibration (1,711 bytes) and poses (430,080 bytes) were fetched through the Drive connector. Calibration SHA256 `1b0fe2a2ef9f58bcb757f115b2722a05c11647d53e4836299c1c99190a9fee77`; pose SHA256 `ae4ac1c5b386aa65bfb4912b44f4cd93ac640dd0134a29e893b1e1c7109cdca1`. Despite its `.tar.gz` name, the pose archive is plain tar. Inspect archive signatures before choosing extraction mode.

Pinned player revision: `990da3e8534b7124a8086ee6e07283db196b4947`. Actual camera metadata gives RGB 1280x560 and thermal 640x512. Projection-derived versus raw translation-norm baselines:

| Modality | Projection baseline (m) | Extrinsic translation norm (m) | Difference (m) |
|---|---:|---:|---:|
| RGB | 0.5042234815 | 0.5042567402 | -0.0000332587 |
| Thermal | 0.6124668682 | 0.6159424486 | -0.0034755804 |

Thermal differs by approximately 0.564%. This requires checking source conventions/version consistency; no calibration is repaired or declared invalid from this alone.

Both local/global pose CSVs have 1,362 finite seven-column rows, identical strictly increasing timestamps and identical final three angle columns. Duration is 349.773665428 s. Median/p90/p99/max reference gap is 0.201269626/0.399049759/0.5997750284/7.399873495 s; 464 gaps exceed 0.25 s. Do not interpolate indiscriminately across these gaps. Angle units/order and camera/body transform remain unverified. The paper describes LiDAR/IMU with RTK GPS reference estimation; this is not error-free camera ground truth.

An initial 4,096-byte image-archive range returned binary data. A fixed eight-by-4MiB archive-format probe was then planned before pixels were seen; it failed exact HTTP range validation. Diagnostic requests returned HTTP 200 HTML titled **Google Drive - Quota exceeded**, not image bytes. Stop there: no quota workaround, complete image download, valid scout PNG or chronological sample exists. Failed probe logs and the fixed plan are preserved. The initial tiny header does not establish current download availability or archive-wide integrity.

## Validation, replay and preservation

`Scripts/Setup/audit_public_stereo_metadata.py --directory <restored-sthereo-folder> --output <analysis.json>` parses supplied camera matrices/extrinsics and Decimal CSV timestamps using CPU NumPy. Actual source archives and extracted files, analysis, independent scalar validator, pinned source snapshots and access/probe logs are under `results/public-data-audit/`. An independent validator passes **2,753 checks** covering source hashes/revisions, CSV schema/finite values/order, scalar medians/max gaps and baseline calculations. This count does not measure scientific model accuracy.

Preserve raw metadata without modifying calibration. Box recovery target: `backups/2026-10-23-exp-040`, with incremental Git bundle after EXP-039 and artifact manifest. Credentials, signed download URLs, source `.git` directories, packages and range caches are excluded. MS² download access is verified; STheReO image inspection remains pending server availability. Next dependent step is file-level schema/timing/depth validation, followed by a bounded existing-MAC comparison if data quality and reference coverage permit.

## MS² access received and bounded source scout

Original scripts are preserved under `results/public-data-audit/ms2-scripts/` with SHA256: sync_data `f923c9280dec4477d7a05d407b98a9eda1933ea2c03811a618f471e7ef2795dc`; proj_depth `387e078ce81d582be38997917713942ced7dfff1ab289b57fbceb0bcbf6693ca`; odom `41b35c9b290c8e9d818335d3c81887928347b405455b0c5f3ea18b026314526c`. They were inspected, never executed. They would download/extract all20 sequences per component.

Downloaded all three small odometry archives (2,302,619/4,066,785/4,655,533 bytes for day/night/rain) and only first8MiB of each selected sync/depth archive. Prefixes contain RGB-right images or RGB intensity, not complete paired RGB/thermal/depth. Thus the scout is not a paired evaluation. The fixed night sync/depth archives total39,732,010,515 bytes; full acquisition is a separately documented continuation. No day/rain image bulk acquisition.

Actual odometry files have16 finite values, 4x4 homogeneous form, resolving the webpage/loader shape discrepancy. Each RGB/thermal/NIR/LiDAR stream has5810 day,10131 night,11868 rain matrices. Independent CPU inspection passes222,538 schema/hash/scout-decode controls, while explicitly flagging22 matrices whose rotation orthogonality or determinant deviation exceeds1e-5 across all12 streams. Some deviations are large (maximum orthogonality deviation1.4101); do not blindly treat every row as a rigid transform or silently project rotations. The initial validator incorrectly assumed all source matrices would be rigid and failed; those failure logs are retained, and the final audit reports the invalid-source counts rather than treating them as valid poses. The1e-5 screen is an audit diagnostic, not a tuned scientific inclusion threshold.

The MS² scout resolves access and schema, not transform direction, image timing, calibration values or physical reference accuracy. The next fixed night pilot must inspect actual paired files and its own pose subset before existing-MAC execution. Large compressed archives are excluded from research artifact backups; small odometry archives and exact extraction/inspection scripts are retained.
