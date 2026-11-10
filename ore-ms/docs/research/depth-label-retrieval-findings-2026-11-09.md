# Exact rain depth recovery and native-factor support

9 November 2026, America/New_York. Branch `ore-ms-orb-development`; support-rule freeze `5d6b51196`.

**Recovered 40 exact rain depth maps, but none of the 38 native pair problems with both endpoint maps is eligible for the planned depth-mean/covariance ablation.** At the saved native correspondence locations, RGB has no jointly supported factor and thermal has at most one per pair. Even before the stereo-right-boundary check, joint counts are unchanged. The limiting factor is sparse endpoint support, not the unchanged-replay contract.

Park the reference-assisted four-arm test for this packet. No depth-hole filling, filtered/merged-depth substitution, lower rank requirement or further acquisition is introduced to make the planned arm run. This pilot does not prove that all recordings or alternative reference designs lack useful support.

## Recovery

The previous preflight inspected EXP147/151 depth backups, which belong to other recordings. This continuation found an earlier exact match: **EXP054** used the current rain sequence `_2021-08-06-16-19-00`, IDs000000–000019. Its scientific backup includes native own-camera projected depth for RGB and thermal, plus image provenance and historical archive receipts.

Restored the single 418,697,640-byte cached scientific archive and verified its whole-file SHA256 against the backup manifest. Extracted only the 40 single-scan depth maps; each PNG's SHA256 matches its exact archived sequence/camera/frame member. PNG signature/chunk CRC/schema and native dimensions pass. All 80 current raw RGB/thermal left/right images for those 20 frames match the historical image hashes; the independent audit rehashes the actual current files.

These are supplied depth products, not model predictions. Format is native uint16, metres multiplied by256, zero invalid, consistent with the [official data format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format). No full 10.9 GB publisher archive, new recording or filtered/merged depth was acquired. Old numerical/model outcomes were not restored. One 419 MB cached recovery was allowed within the declared 768 MiB total intake bound; publisher-range limits were not consumed.

Recovery covers 40 of the 126 maps originally listed. Frame000020is absent from the pilot, leaving two rain endpoint-map slots and 84 day/night maps outstanding. The support result below removes the justification for automatically retrieving those remaining maps during this bounded pass.

## Frozen support check

For each original native factor, sample source and target own-camera maps at the saved UV coordinates, rounding `floor(uv+0.5)`. Require a nonzero original code at both endpoints. No interpolation, dilation, local-window replacement or fallback depth. Also require the reference-implied stereo-right pixels to be inbound. The candidate pose problem requires at least three retained correspondences and valid six-degree-of-freedom conditional information; three alone would not guarantee sufficient geometry.

Evaluated all original first 20 pair slots per camera. Nineteen have both endpoint maps; pair 19→20 remains explicitly map-missing. The 38 map-complete problems contain 4,923 native factors:

| Camera | Native pair slots | Both maps available | Native factors on map-complete pairs | Source-depth factors | Target-depth factors | Joint factors, total | Maximum joint factors per pair | Eligible pairs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| RGB |20|19|3,736|11|13|0|0|0|
| Thermal |20|19|1,187|9|19|1|1|0|

Only thermal pair 10→11 has a joint factor. All 38 problems fall below the three-correspondence prerequisite, so none reaches the information-rank test. The two problems missing target maps are retained separately. This is coverage at the **existing native-selected correspondences**, not the global density of valid depth pixels or independent verification of correspondence identity.

No pose-reference error is joined, no frontend/model/solver intervention is executed, and no accuracy improvement is claimed. The 120 exact native replays from the preceding preflight remain reusable; they were not rerun.

## Independent verification

Three tests pass for depth units/rounding, invalid/out-of-bounds pixels, no hole filling and empty support. Independent Pillow decoding and scalar pixel sampling reproduce all support arrays/counts from raw labels and native tensor maps, without importing the producer. The audit verifies 54 frozen input/source hashes, 40 decoded maps, 80 actual image hashes, all 40 pair slots and 9,846 endpoint samples. All source maps remain unchanged.

The first audit completed its checks but failed while serializing a NumPy integer. Converted only that receipt field to a Python integer and reran the audit; the original log and partial receipt are preserved. Production code, masks, labels and support rules were unchanged.

See the [frozen protocol](../../experiments/depth-label-retrieval-freeze-2026-11-09.json) and [complete outcome](../../experiments/depth-label-retrieval-outcome-2026-11-09.json). Labels, factor masks, provenance and receipts are in `results/depth-mean-ablation/recovered-rain-001` and the companion Box packet.

## Decision

The missing-map dependency has been partly resolved and a more specific limitation measured. The proposed source-and-target mean intervention is not supported by this recovered rain packet under its declared native-factor rule. It remains **unexecuted**, so the role of depth means versus covariance in motion error remains unanswered.

Stop further map retrieval and qualification for this particular assay in this pass. Do not recycle the failed design into another uncertainty selector or describe sparse references as a failure of RGB–thermal fusion. Preserve the recovered maps and native parity evidence for useful future work.

A substantive mean-estimation/frontend experiment remains a candidate. It should use sparse own-camera supervision at valid pixels with a suitable evaluation contract, rather than require coincident LiDAR labels at both ends of native-selected tracks. Earlier disparity-error surveys and covariance-only adaptations already exist; repeating those summaries would not be that intervention. Any mean-changing model needs matched native VO controls, separate translation/rotation metrics and recording separation before an accuracy claim. No training or new method is launched by this report.
