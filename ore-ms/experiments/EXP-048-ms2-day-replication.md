# EXP-048: Fixed independent MS² day pilot replication

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `91b475527`.
**Positive within-image projected-depth proxy association replicates on the fixed day recording, but its strength and geometry-controlled behavior differ from the night pilot.** The primary day joint within-bin coefficients are0.130RGB/0.149thermal, versus night0.179/0.388. These are different scene/reference populations; no lighting-causal, physical calibration or cross-modal winner claim is made.

## Selection, access and source acquisition

Before inspecting day model outcomes, use EXP040's fixed first lexical official day-test sequence `_2021-08-06-11-23-45`, IDs000000–000019. Preserve all20 IDs with no quality/outcome substitution. This is a distinct recording from night `_2021-08-13-21-18-04`; the80 day source-image hashes have zero overlap with the night pilot. It remains previously known metadata and unknown checkpoint-training membership, not verified training-held-out evaluation.

User requested independent-recording work and asked whether more day/other public data were needed. Box project coverage contains the previously analyzed TD3 day recordings and bounded MS² backups; full MS² day images were not already staged. Restore EXP040's user-supplied access pointers/odometry after verifying its89MB recovery archive and every member SHA256. Recheck current exact HTTP206 ranges and bzip2 signatures; do not execute the supplied all-sequence download scripts.

Sequential bzip2 tar layout requires the two complete day transport archives to reach the fixed RGB/thermal/depth members. Acquire only this one day sequence,24,905,344,133bytes total, in bounded64MiB ranges with exact status/Content-Range/byte-count validation and ordered assembly. Keep local archive hashes, no publisher checksum was available:

|Archive|Bytes|Observed SHA256|
|---|---:|---|
|sync_data|14,795,109,417|`bbcaf396437cad44aa4b66a9479dac7fc792bd67518ca58421946e7b1142f60c`|
|proj_depth|10,110,234,716|`cfd5fb13b2238636c4445c29019243533ddc832e95ff7e7ab583324d0dd99489`|

Safe streamed tar scans reject absolute/traversal paths and links, require the fixed sequence prefix, and bound selected-member sizes. Stage only80 camera images,120 depth maps,40 pre-existing camera poses, source calibration/readmes/timestamps, and20 matching navigation-status files. Keep full tar indices and bzip2 block maps for further local selective extraction. Full archive bytes remain local and are excluded from Box replay backups.

## Actual day readiness and differences

Source RGB uint8 at384×1224×3 and thermal uint16 at256×640 match the adapter contract. Depth remains uint16/256meters with zero invalid. The decoded day/night calibration dictionaries are exactly equal; identity stereo rotations and horizontal baseline conventions remain unchanged. All40 day pilot pose matrices pass the existing1e-5 rigidity screen without correction or use as `gt_pose`.

|Recorded-reference/timing quantity|RGB|Thermal|
|---|---:|---:|
|Pilot left-camera duration (s)|1.933103216|1.875845826|
|Endpoint reference translation-column change (m), not accuracy|8.064762|7.839669|
|Full stereo rows within inherited2ms recorded-time gate|5806/5810|2884/5810|
|Full absolute stereo offset min/median/max (ms)|0.000032/0.041949/66.631937|0.000006/2.036318/75.506906|

Unlike the near-static night pilot, this opening day interval contains substantial recorded motion. The inherited2ms-stereo/30ms-left-cross gate admits3/20 day groups, versus0/20 night. Keep all20 provided-ID pairs for the predeclared diagnostic and retain every recorded time/gate flag. No relaxed thresholds, clock correction, motion fit or post-hoc timing-based subset is scored. Passing a receive/export-time threshold is not proof of exposure simultaneity; physical cross-modal error eligibility remains false.

All20 selected navigation records have source filter state2, dynamics mode2 and status flags0. These source-status fields do not establish centimeter accuracy, RTK status, covariance, exposure timing or camera/reference-frame correctness. They are not grounds for declaring navigation-derived labels physical truth. Reference pose direction/rectification/export-time interpretation remains qualified as in EXP045.

## Original frontend and unchanged analysis

Use the unchanged `ms2_frontend_smoke.py`, original RGB/thermal checkpoints and explicit `td3-firestereo-v2-clahe-bilateral-v1` recipe. No geometric re-rectification, resizing or model change. GPU image is the recovered `macvo:ms2-smoke`; all recorded runtime fields, checkpoint hashes/load rules, thermal recipe and effective frontend configuration equal the historical night run. Python3.11.13, Torch2.7.1+cu128, NumPy2.2.6, OpenCV4.10.0.84, PyPose0.7.3, timm0.9.12, einops0.8.2; RTX PRO5000 Blackwell, FP32/seed0/thread1, matmul TF32false/cuDNN TF32true.

All78 scheduled native-resolution stereo/temporal pair fields pass execution; all4 first-pair repeat checks are bitwise identical. Only the40 stereo fields enter this scientific replication;38 temporal fields are execution artifacts, with no pose or temporal-error score. Original-producer AST replay independently matches all80 day model-input tensors exactly, and all source IDs/left-right integer timestamps and unset GT fields verify.

The new sequence-specific runner handles paths/metadata and directly imports EXP046's `metrics`/`aggregate` and EXP047's `control`/`aggregate` functions. Their source hashes are unchanged; historical night files/runners are not edited. Reuse single-scan primary, merged/filtered sensitivities, own/common masks, signed expected flow, normalized covariance/residual units, per-image rho and fixed6 depth bands/3×3grid/min30 controls. Common-support depth bins still use primary single-scan depth at shared coordinates for every variant. No settings are selected on day outcomes.

## Primary replication result

Each entry is a median of20 image summaries. Brackets show observed image minima/maxima, not confidence intervals. Raw Spearman and within-bin coefficients have different weighting; their differences are descriptive, not percentages of explained confounding. Retained-pixel baselines separate sparse-bin eligibility from the coefficient calculation.

|Recording / modality / support|Raw rho|Joint-retained raw rho|Joint within-bin coefficient [min,max]|Median retained %|
|---|---:|---:|---:|---:|
|Night / RGB / own|0.353|0.354|0.179 [0.123,0.227]|99.52|
|Night / RGB / common|0.120|0.010|0.044 [-0.096,0.211]|82.04|
|Night / Thermal / own|0.453|0.448|0.388 [0.307,0.420]|99.53|
|Night / Thermal / common|0.385|0.406|0.365 [0.265,0.453]|89.34|
|Day / RGB / own|0.252|0.250|0.130 [0.056,0.243]|98.72|
|Day / RGB / common|0.209|0.207|0.087 [0.031,0.233]|97.00|
|Day / Thermal / own|0.392|0.391|0.149 [-0.029,0.310]|99.67|
|Day / Thermal / common|0.449|0.447|0.219 [0.093,0.332]|97.93|

Primary raw association is positive20/20 day images for both modalities, on both own/common support. Primary joint coefficients are positive20/20 RGB own/common,18/20 thermal own, and20/20 thermal common. Day thermal own-support joint coefficients range−0.029 to0.310; the night range was0.307–0.420. This shows the night association's magnitude is not reproduced unchanged, without establishing why. Motion, scene, depth population, timing, label error and illumination all differ.

Day joint retention is98.72% RGB/99.67% thermal on own support and97.00%/97.93% on common support. Day common support covers47.76% of primary RGB pixels and54.82% of thermal, versus night8.83%/17.82%; cross-recording populations are not matched physical points. Pixel counts are dependent coverage counts, not independent sample sizes.

## All reference sensitivities retained

|Day modality / depth product|Own / common support pixels|Own / common raw rho|Own / common joint coefficient|
|---|---:|---:|---:|
|RGB / Single scan (primary)|115,659 / 55,236|0.252 / 0.209|0.130 / 0.087|
|RGB / Merged32|1,761,160 / 55,236|0.330 / 0.233|0.219 / 0.117|
|RGB / Network filtered|1,613,141 / 55,236|0.210 / 0.211|0.105 / 0.055|
|Thermal / Single scan (primary)|157,777 / 86,501|0.392 / 0.449|0.149 / 0.219|
|Thermal / Merged32|1,117,812 / 86,501|0.498 / 0.451|0.200 / 0.221|
|Thermal / Network filtered|1,009,429 / 86,501|0.465 / 0.466|0.272 / 0.275|

Day own-support primary median per-image L2 proxy residual is1.7199px RGB and1.7459px thermal; normalized image-plane medians0.0022496/0.0045023. These are modality-specific reference-agreement summaries on different populations, not camera-winner scores. Merged and network-filtered products remain separate and retain their navigation/ICP/network-dependent provenance. No variant is promoted after observing its stronger correlation.

![Two fixed recordings under the same primary diagnostic](../results/ms2-day-replication/recording-comparison.png)

Standalone PNG/PDF figures, full per-image summaries, source and derived point arrays are preserved. No significance/generalization claim follows from20 adjacent frames per recording.

## Independent checks and execution history

Source-only validation passes7,264 checks; frontend validation passes8,286 checks including80 exact input tensors and all78 output fields. The complete source/frontend/proxy validation, expanded to include the latest night EXP046/047 artifacts, passes18,719 checks across120 cases,5,774,978 reconstructed points and1,920 scalar camera-model checks. The initial proxy validator output is preserved; its later rerun adds historical hash coverage, with identical scientific scores.

Control validation uses EXP047's independent scalar/rank implementation adapted only to the actual day manifest and recorded timing flags; it does not assume the night-specific0/20 gate. It passes41,580 checks across240 case/support combinations,720 controls,16,560 strata and6,200,189 point-membership checks. There are9,073 eligible strata and7,487 sparse/empty strata; none is hidden or replaced.321 proxy frozen inputs,442 control frozen inputs and488 historical source/result hash checks remain unchanged. Source arrays, full signed residual/variance arrays, quantiles, tied ranks, aggregates and case coverage are validated. Computational passes do not certify physical labels.

Archive extraction uses added `Docker/Dockerfile.ms2-pilot` with `indexed_bzip2==1.7.0`; GPU and scoring stay in the unchanged smoke image. The dependency availability probe initially lacked indexed_bzip2, then the extraction image built successfully. Startup driver diagnostics remain in run logs; Torch/CUDA execution and all reported checks pass. No original output, calibration, default, training or fusion change.

## Reproduction and preservation

Results live under `results/ms2-day-replication/`. Source acquisition/extraction scripts, exact range/size/hash manifests, selected-member manifests, native source pixels, calibration, timestamps, navigation-status files and poses are preserved. `replication-controls.json` records runtime/config/recipe/calibration/checkpoint equality and unchanged scoring-function hashes. Build `Dockerfile.ms2-pilot` only if re-extracting original archives. The source access pointers are in EXP040's prior recovery; URL tokens/headers are not copied into new logs/backups.

From `MAC-VO-main`, in `macvo:ms2-smoke`, after restoring/preparing source and original frontend fields:

```bash
python -m Scripts.Setup.ms2_day_replication score \
 --root /root/AirStack/ore-ms \
 --base /root/AirStack/ore-ms/results/ms2-day-replication \
 --source /root/AirStack/ore-ms/results/ms2-day-replication/frontend-run-001 \
 --output /path/to/fresh-proxy

python -m Scripts.Setup.ms2_day_replication controls \
 --root /root/AirStack/ore-ms \
 --base /root/AirStack/ore-ms/results/ms2-day-replication \
 --source /path/to/fresh-proxy \
 --output /path/to/fresh-controls
```

Use `prepare --root ... --base ...` only for a fresh base with safely extracted original inputs; it rejects an existing source manifest. Original frontend command/arguments are recorded in its `report.json` and EXP044, with the day manifest/output substituted. Validate using `Scripts.Setup.validate_ms2_day_replication --root ... --base ... --frontend ... --proxy ... --output <fresh.json>` and `Scripts.Setup.validate_ms2_day_controls --root ... --source <proxy> --run <controls> --output <fresh.json>`. Historical checks require EXP043,EXP044-complete,EXP046 andEXP047 manifests under `/root/ore-ms-recovery`. Use fresh outputs; no overwritten results.

Box recovery target `backups/2026-10-25-exp-048`: bounded source/metadata, both frontend outputs, proxy arrays, controls, figures, logs, notebook, scripts and incremental bundle after EXP047. Full24.9GB archives, packages, weights and credentials are excluded. EXP040/043–047 and original checkpoint recovery supply previous-state/provenance prerequisites. Both complete day archives remain locally available for additional fixed intervals.

## Data needs and next step

No extra day-data upload or new public dataset is needed for the next bounded replication controls: the full day sequence is now local, and unused MS² rain access pointers are already available. Additional fixed, separated intervals from this day recording can test temporal/scene breadth without another download; they are still the same recording, not new independent recordings.

A distinct recording with documented RGB/thermal exposure association, applicable calibration and independently justified depth/pose-reference quality would materially strengthen physical cross-modal reliability evaluation. More day images alone cannot establish those properties. Before requesting user uploads or switching datasets, check existing full-day/Box coverage and the known public candidates. At this point prioritize predeclared additional intervals/independent conditions, preserve all negative/variable cases, and resolve source timing/reference provenance before learning a calibration or applying a cross-modal selection rule.
