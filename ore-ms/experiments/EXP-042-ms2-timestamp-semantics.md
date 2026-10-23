# EXP-042: MS² timestamp semantics and MAC input contract

Date:2026-10-23 America/New_York;branch `ore_ms`;starting commit `c467fba67`. **No public source examined defines what the exported thermal timestamp marks. The signed offset flips8,810 times, ruling out a single constant shift as a way to put every recorded pair within2ms. Both official loaders nevertheless define stereo pairing by matching sorted image indices. Preserve that source protocol separately from physical exposure claims.**

This investigation changes no timestamps,pairing,preprocessing,calibration,model,training,weights or default. It prepares a20-ID source-input contract for a subsequent MAC adapter. No inference or accuracy comparison occurs.

## What the primary sources establish

The [CVPR2023 paper,Sec.3.1](https://openaccess.thecvf.com/content/CVPR2023/papers/Shin_Deep_Depth_Estimation_From_Thermal_Image_CVPR_2023_paper.pdf) distinguishes RGB/NIR synchronization via an external module,thermal stereo driven by the left thermal camera's sync output,and a software trigger aligning the systems at acquisition start. The [2025 extended manuscript,Sec.III-A](https://arxiv.org/html/2503.22060v1#S3.SS1) repeats that architecture. Neither inspected description specifies the exposure/reception event or clock represented in `img_*_timestamp.txt`. The six-page CVPR supplement and archived sequence readme add calibration/layout details but no export-timestamp definition.

The [original official loader](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/dataloader/MS2_dataset.py) and [later official benchmark loader](https://github.com/UkcheolShin/SupDepth4Thermal/blob/f6725678f9b5dba669ea1f0739506a613f2fb2fb/dataloaders/loaders/MS2_dataset.py) sort left/right PNG paths and take the same index. Their stereo loaders do not read timestamp text or apply a thermal clock correction. This supports preserving the published same-ID protocol as a documented experimental input convention. It does not independently verify simultaneity.

The complete examined trees are pinned at `0bb0fc1a...` and `f6725678...`;a supplementary scan of the author's DeepIR tree at `544d868a...` found no acquisition/timestamp implementation. Examined GitHub issue snapshots contain17 dataset issues and3 benchmark issues,no timestamp-semantics explanation. Related calibration issue comments are preserved with authorship;they do not answer the timing question. These are bounded source-search results,not a claim that no acquisition code exists anywhere.

The [manufacturer's Ax5 synchronization manual](https://docs.flir.com/T559770/en-US/latest/AboutIOSynchronizationAndMeasuremen-B1352D5B.html) documents master/slave frame synchronization and configuration constraints. It does not identify the settings or timestamp-writing driver used in this recording. Do not import assumptions from other FLIR models or turn a nominal frame period into a measured delay. Raw camera rate,software decimation,driver clock,and export-selection policy remain distinctions to resolve.

The user confirms the MS² access reply contained no timestamp notes. No email,issue or author message is sent. Primary-source PDFs/HTML/API trees/issues,loader snapshots,URLs,byte counts,revisions and hashes are retained in `results/ms2-timestamp-semantics/source-registry.json`.

## Recorded-time structure:fixed complete night sequence

Use EXP041's unchanged four camera timestamp arrays for `_2021-08-13-21-18-04`,all10,131 source IDs. Work with integerns;only convert differences to ms for reporting. Define `d_i = right_i - left_i` for thermal stereo.

| Diagnostic | Result |
|---|---:|
| Signed offset range |−49.753598 to+47.091087ms |
| Negative / positive offsets |4,572 /5,559 |
| Sign changes between adjacent IDs |8,810 /10,130 |
| Signed median |+20.544446ms |
| Median absolute offset,EXP041 |33.766417ms |
| Same-ID minimum absolute offset |15.219450ms |

Both signs occur throughout the ten fixed acquisition-index bins. This is not simply an initially positive offset that drifts through zero once. Native source intervals are irregular:RGB median133.313ms with a66.659ms lower mode;thermal median~125.3ms with shorter/longer modes. Do not assume the exported sequence is uniformly sampled at the nominal acquisition rate.

For any constant correction `c`,the maximum residual satisfies `max_i |d_i-c| >= (max(d)-min(d))/2 = 48.4223425ms`. Thus no constant can bring every recorded pair within2ms. This is a mathematical bound,not a fitted clock model,an applied correction or an estimate of exposure delay.

| Fixed right-index shift | Minimum absolute recorded difference | Pairs within2ms |
|---|---:|---:|
|−1 |29.030261ms |0/10,130 |
|0 |15.219450ms |0/10,131 |
|+1 |31.346563ms |0/10,130 |

These are diagnostics on fixed candidate indices;input images remain paired by their original IDs. EXP041's nearest-right audit also found0 qualifying pairs. This rules out these simple recorded-time fixes,not hardware synchronization itself. Recorded metadata cannot distinguish exposure offsets from reception timing,clock conversion or export selection without acquisition provenance.

![MS² signed source timestamp structure](../results/ms2-timestamp-semantics/run-002/timestamp-structure.png)

## Concrete input preparation for MAC

A generic loader cannot be reused unchanged. Current `GeneralStereoSequence` generates `local_index*1000` placeholder timestamps,and its `cv2.IMREAD_COLOR` path converts16-bit files to8-bit before `/255`. For the fixed first thermal-left frame `000000.png`,native values3051–3973 contain619 distinct levels;the generic path produces values11–15,only5 levels per channel. An exact array control confirms the conversion equals a right shift by8 followed by channel replication. The sourcePNG SHA remains `099d086bd6b05cd626319f5ce68b39c92c0c460b4c65d22eb21c34a660b2f11c`.

The existing TD3 adapter explicitly rejects uint16 input,which is appropriate for its own provided uint8 contract. Neither adapter currently supplies the required MS² source contract. Do not convert these raw thermal values using a display contrast function or invent a network input range. Match an eventual transform to the chosen pretrained thermal model's documented input contract.

Prepared `results/ms2-timestamp-semantics/source-protocol-input-manifest.json` records fixed IDs000000–000019,all80 original image paths/hashes/shapes/dtypes,all four recorded timestamps per ID,source cameraK and meter baselines,each camera's three depth variants with encoding/provenance,and40 original pose paths/hashes with unverified pose direction explicit. Referenced files are the preserved EXP041 pilot. No extra pixels or models are acquired.

The inherited2ms-stereo/30ms-cross diagnostic remains failed and unchanged. It is not a universal rule that makes an officially paired dataset unusable for every purpose. Software integration and explicitly labelled provided-pair diagnostics can proceed under the published protocol while retaining this discrepancy. A physical same-time error/calibration claim cannot be justified by renaming that protocol or suppressing the timestamps. No adaptive-camera benefit or physical covariance calibration follows from this audit.

## Validation and replay

10,174 independent controls pass:scalarCSV/integer timestamp identities,exact median/extrema/sign counts,index-shift sensitivities,a Fraction-based constant-shift bound,fixed-bin medians,inputSHA and frozenEXP041 registry hashes. The precision diagnostic independently checks the complete first-frame conversion array. Timestamp/output controls are not physical timing validation.

Reusable CPU NumPy command:

```
python Scripts/Setup/ms2_timestamp_structure.py --input <four-series-directory> --output <fresh-output>
```

Inputs are `rgb_left.txt`,`rgb_right.txt`,`thr_left.txt`,`thr_right.txt`,one integerns timestamp per line. Fresh output is required;the script reports signed structure and fixed sensitivities without fitting or modifying a clock. Raw CSV,analysisJSON,source registry,scalarvalidator,precision diagnostic,manifest builder,plotPNG/PDF and exact runtime context are preserved. Initial `run-001-initial` is retained;final `run-002` adds sign-count and analytical-bound reporting without changing inputs,pairing or a scientific threshold. Existing EXP041 results remain hash-identical.

## Question prepared for the dataset authors — not sent

> For MS² `_2021-08-13-21-18-04`,do `thr/img_left_timestamp.txt` and `thr/img_right_timestamp.txt` mark exposure,camera/driver time,or ROS reception? Same-ID right-minus-left times range−49.75 to+47.09ms and change sign8,810 times,while both official loaders pair sorted images by the same index. Do same-ID thermal images share an exposure interval,and which acquisition/export code or configuration defines this? What pairing/timestamp convention should be used for stereo-temporal geometric evaluation?

The next implementation is an MS²-specific MAC input adapter that preserves source values and recorded times,with an explicit pretrained-model thermal transform. Physical timing semantics remain an owner/provenance question. Recovery target:Box `backups/2026-10-23-exp-042`,including source snapshots,four timestamp arrays,diagnostics,manifest,notebook and incrementalGit bundle after EXP041. Restore EXP041 for referenced pilot images/poses/depth and precision replay. No full40GB archive,GPU or weights is needed for timestamp replay.
