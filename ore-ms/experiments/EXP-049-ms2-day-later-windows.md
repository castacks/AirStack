# EXP-049: Fixed later windows in the existing MS² day recording

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `e7d43614f`.
**Positive median uncertainty/proxy-disagreement association persists in two fixed later day windows, with variable magnitude.** Primary joint coefficients are0.149RGB/0.209thermal in the middle window and0.131/0.307 late, versus opening0.130/0.149. These are additional intervals of one recording, not new independent recordings or physical calibration validation.

## Selection and source preservation

User clarified CATS data are not yet available and requested further work. Use only the complete local day archives from EXP048; no CATS access assumption, new public-data download, author message or user upload.

Freeze two20-ID windows from the5810-row `_2021-08-06-11-23-45` sequence before inspecting their pixels/model outcomes: `floor(N/3)=1936` through1955, and `floor(2N/3)=3873` through3892. No motion, timing, support, confidence or error selection/substitution. The earlier000000–000019 interval remains a separate baseline. Source IDs and all4 recorded camera times retain their original values; local array indices0–19 are explicitly distinct from source stems.

Selective extraction reuses EXP048's full tar indices and cached bzip2 block maps. Recheck compressed archive sizes against previously whole-hash-verified EXP048 identities, validate selected tar-header checksums/names/sizes, read bounded payloads at recorded offsets, and hash the selected members. A fresh whole-archive hash is not claimed. Copy unchanged calibration/readmes/full timestamp metadata by original member SHA; extract original camera poses and matching navigation records without modifying them. Independent PNG chunk CRC and decoded-schema checks follow.

Per new window retain80 camera PNGs,120 depth PNGs,40 poses,20 navigation records and applicable calibration/readmes/timestamps. All original stems remain in paths/manifests/model-output metadata; no source renumbering or fabricated time occurs.

|Context|Middle1936–1955|Late3873–3892|
|---|---:|---:|
|RGB / thermal recorded span (s)|1.866573104 /1.933932125|1.933103464 /1.871366233|
|Reference translation-column change RGB / thermal (m), not accuracy|7.778161 /8.046727|15.837422 /15.323350|
|Inherited joint recorded-time gate|2/20|9/20|
|Pilot pose matrices passing retained1e-5 rigidity screen|40/40|40/40|

Preserve the inherited2ms stereo/30ms-left-cross gate flags, including failures. All20 provided-ID pairs are scored under the prior conditional protocol; no post-hoc well-timed subset is selected. Recorded gate passage does not establish exposure simultaneity. Reference poses remain unused as GT; translation-column changes are contextual recorded quantities, not surveyed motion accuracy.

## Source-ID-safe execution and parity control

Keep all historical runners/analysis files unchanged to preserve their frozen hashes. Add opt-in `ms2_manifest_stereo.py`, derived from the original frontend runner: accept20 distinct original stems, keep local indices separate, name output files with original source stems, and execute stereo fields only. The unchanged scientific comparison already used stereo fields only; new temporal output is unnecessary. The log's i/j numbers are local indices; manifest paths, filenames and NPZ `source_ids` carry the actual original IDs.

Before new-window inference, rerun the first opening-day stereo pair for each modality with this runner. All10 saved-array checks are bitwise identical to EXP048, with exact runtime/configuration/checkpoint-load equality and bitwise repeat checks. This is an implementation parity control, not an additional scientific sample.

Both new windows then complete40 scheduled native-resolution stereo fields each, with two bitwise first-pair repeat checks per window. Runtime is the existing smoke image, original checkpoints, FP32,seed0,one thread,unchanged TF32 flags/config and explicit `td3-firestereo-v2-clahe-bilateral-v1`. All160 new input tensors independently match original producer/decode calculations. No resizing, repeat rectification, clock/pose fit, training, VO, fusion or dataset-registration/default change.

The source-ID-preserving wrapper delegates proxy scoring to the unchanged EXP048 function, which uses EXP046's metric/aggregate functions. Control grouping uses the same EXP047 control/aggregate functions with original-stem lookup rather than treating stems as local indices. New independent validators use the actual source window/index mapping and current gate flags; they do not assume zero-based source IDs or a0/20 timing result.

## Primary single-scan result

Execute the frozen EXP045–047 definitions: nonzero single-scan depth and right-projection bounds, signed forward stereo residual, normalized variance-trace/L2-error proxy, per-image Spearman, and fixed6 depth bands/3×3grid/min30 controls. Common-support grouping uses primary depth at fixed shared coordinates for all depth variants. These evaluation controls use supplied reference depth; they are not a deployable online reliability estimator.

Each table entry is a median of20 image summaries. Brackets are observed image minima/maxima, not confidence intervals. Raw and within-bin statistics differ in weighting/estimand; retained-pixel baselines separate sparse-bin eligibility from rank conditioning. No pooled-pixel correlation or significance claim.

|Window / modality / support|Raw rho|Joint-retained raw rho|Joint within-bin coefficient [min,max]|Median retained %|
|---|---:|---:|---:|---:|
|Opening / RGB / own|0.252|0.250|0.130 [0.056,0.243]|98.72|
|Opening / RGB / common|0.209|0.207|0.087 [0.031,0.233]|97.00|
|Opening / Thermal / own|0.392|0.391|0.149 [-0.029,0.310]|99.67|
|Opening / Thermal / common|0.449|0.447|0.219 [0.093,0.332]|97.93|
|Middle / RGB / own|0.137|0.134|0.149 [0.067,0.270]|98.97|
|Middle / RGB / common|0.141|0.126|0.194 [0.122,0.256]|92.31|
|Middle / Thermal / own|0.420|0.419|0.209 [0.062,0.317]|99.41|
|Middle / Thermal / common|0.502|0.507|0.221 [0.099,0.352]|96.29|
|Late / RGB / own|0.196|0.187|0.131 [0.056,0.243]|98.24|
|Late / RGB / common|0.203|0.159|0.152 [0.072,0.184]|92.73|
|Late / Thermal / own|0.477|0.475|0.307 [0.145,0.461]|99.19|
|Late / Thermal / common|0.399|0.418|0.255 [0.098,0.414]|97.16|

All new primary joint coefficients are positive20/20 images for each modality and each support. Middle RGB primary raw association is positive18/20 on own support and20/20 on common support; late RGB and both thermal windows are positive20/20 on both supports. Negative per-image raw cases remain in the report. Coarse grouping can change association direction/weighting and does not certify a causal source of error or confidence calibration.

New joint controls retain a median98.97%/98.24% primary own-support RGB pixels and99.41%/99.19% thermal pixels in middle/late windows. Common-support retention is92.31%/92.73% RGB and96.29%/97.16% thermal. Every result retains its support population, per-stratum count/reason and full/same-retained baseline. No threshold is relaxed to improve availability or scores.

Primary own-support coverage is129,533RGB/177,456thermal points middle and124,302/166,009 late; common support is43,413/69,377 middle and41,476/77,408 late. Counts are dependent coverage totals. Common support means intersection of depth products within the same camera/image, not corresponding physical points across modalities or windows.

## All reference sensitivities retained

|Later window / modality / product|Own / common raw rho|Own / common joint coefficient|
|---|---:|---:|
|Middle / RGB / Single scan (primary)|0.137 / 0.141|0.149 / 0.194|
|Middle / RGB / Merged32|0.292 / 0.107|0.218 / 0.198|
|Middle / RGB / Network filtered|0.107 / 0.097|0.117 / 0.185|
|Middle / Thermal / Single scan (primary)|0.420 / 0.502|0.209 / 0.221|
|Middle / Thermal / Merged32|0.516 / 0.516|0.207 / 0.207|
|Middle / Thermal / Network filtered|0.544 / 0.501|0.220 / 0.237|
|Late / RGB / Single scan (primary)|0.196 / 0.203|0.131 / 0.152|
|Late / RGB / Merged32|0.356 / 0.191|0.243 / 0.176|
|Late / RGB / Network filtered|0.218 / 0.177|0.146 / 0.137|
|Late / Thermal / Single scan (primary)|0.477 / 0.399|0.307 / 0.255|
|Late / Thermal / Merged32|0.405 / 0.404|0.275 / 0.286|
|Late / Thermal / Network filtered|0.377 / 0.398|0.292 / 0.271|

The primary stays single scan. Merged/filtered references retain navigation/ICP/network-dependent provenance; stronger sensitivity results do not become independent ground truth. Full signed-component/L2 quantiles, covariance traces, depths, model predictions, residual arrays and support flags are retained per case. No preferred variant, bin, window or weighting rule is selected after scoring.

![Three fixed windows, separate summary populations](../results/ms2-day-later-windows/window-comparison.png)

Standalone PNG/PDF figures are retained. Adjacent images and separated windows from one recording are not treated as independent recordings. Variation across these scene/time/motion/reference populations does not isolate a lighting, timing or sensor effect, and does not establish cross-modal winner selection.

## Independent validation

All80 new frontend fields,240 image/depth cases and1440 controls complete. Source-only and frontend-only checks gate subsequent analysis. Final middle validation passes19,604 source/frontend/proxy checks over4,785,124 reconstructed selected points; late passes19,582 over5,545,215 points. Each includes80 exact producer/model input tensors,40 saved frontend fields,120 proxy cases,1,920 scalar camera-model points and989 historical source/result hash checks.

Independent control validation passes42,083 checks per window, each covering240 case/support combinations and16,560 possible strata. Middle checks5,123,494 point memberships and late5,901,867; these include repeated variant/support membership, not independent samples. Middle has8,393 eligible/8,167 sparse strata; late9,230/7,330. Sparse/empty strata are retained with reasons; all primary controls remain available for all20 images.323 proxy frozen inputs and444 control frozen inputs per window remain unchanged. Historical checks now include the full bounded EXP048 artifacts in addition to EXP043/044/046/047.

Validators reconstruct original source-ID/time associations, PNG CRC/precision/shape, expected disparity/support, all saved arrays, scalar quantiles and tied rank statistics without importing the scorer's metric implementation. CPU analysis uses the existing smoke runtime; indexed extraction uses the prior pilot image. Startup driver diagnostics are retained. Computational passes do not validate physical labels, exposure timing, checkpoint-training split or a universal uncertainty/error mapping.

## Reproduction and recovery

Artifact root `results/ms2-day-later-windows/` contains fixed `selection.json`, extraction/provenance scripts, per-window source/metadata/manifests/audit, frontend/proxy/control outputs and independent validation, baseline parity evidence and figures. CATS remains unavailable as reported by the user; no new request was submitted.

In the existing smoke image, with AirStack mounted at `/root/AirStack`, from `MAC-VO-main`:

```bash
python -m Scripts.Setup.ms2_manifest_stereo \
 --root /root/AirStack/ore-ms \
 --manifest /root/AirStack/ore-ms/results/ms2-day-later-windows/middle/source-manifest.json \
 --rgb-checkpoint /root/AirStack/ore-ms/models/macslam/MACVO_FrontendCov.pth \
 --thermal-checkpoint /root/AirStack/ore-ms/models/macslam/MACT_Offroad.pth \
 --output /path/to/fresh-frontend

python -m Scripts.Setup.ms2_later_windows score \
 --root /root/AirStack/ore-ms \
 --base /root/AirStack/ore-ms/results/ms2-day-later-windows/middle \
 --source /path/to/fresh-frontend --output /path/to/fresh-proxy

python -m Scripts.Setup.ms2_later_windows controls \
 --root /root/AirStack/ore-ms \
 --base /root/AirStack/ore-ms/results/ms2-day-later-windows/middle \
 --source /path/to/fresh-proxy --output /path/to/fresh-controls
```

Repeat with `late`. Validate via `Scripts.Setup.validate_ms2_window --root ... --base ... --frontend ... --proxy ... --output <fresh.json>` and `Scripts.Setup.validate_ms2_window_controls --root ... --source <proxy> --run <controls> --output <fresh.json>`. Historical checks require exp-043,exp-044-complete,exp-046,exp-047,exp-048 manifests under `/root/ore-ms-recovery`. Source preparation requires the restored selection/selected inputs and fresh manifests; existing outputs are rejected.

Recovery target Box `backups/2026-10-25-exp-049`, with bounded source/metadata, model fields, derived point arrays, controls, validation, plots/logs/notebook, code and incremental Git bundle after `e7d43614f`. Previous recoveries and original checkpoints remain prerequisites for historical provenance checks. Full day archives/packages/weights/tokens are excluded; no archive download or user upload was needed in this study.

## Outcome and limits

The later intervals strengthen within-recording evidence of a positive raw-stereo-covariance/projected-disagreement relationship under coarse fixed controls. Its magnitude varies across windows and reference/support choices. This does not validate temporal covariance, full3D/pose uncertainty, physical coverage calibration, training-held-out generalization or relative RGB-versus-thermal error prediction. Keep weights/defaults unchanged.

External data with independent geometric labels and an applicable acquisition/calibration protocol remain valuable for the stronger claim. CATS access and Yifei/Matthew source replies can proceed in parallel with this characterization. More intervals or ordinary images alone cannot remove the current reference/timing assumptions; no additional user upload is required to reproduce these results.
