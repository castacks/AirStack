# CATS approved access and bounded file inspection

**Follow-up:** the [convention and temporal-input audit](arbiter-input-audit-2026-10-28.md) checks the left mask against padding and tests signed correspondence hypotheses. It finds useful support evidence but does not resolve a trusted disparity sign/anchor. No model comparison has been run.

2026-10-28, America/New_York. Base `e80574d32`, branch `ore_ms`.
The user supplied a dataset-access approval and six download links. Metadata
was read through the connected Google Drive plugin. Byte-range reads then
verified actual ZIP contents for v2 outdoor and the v2 complete-release index.
This replaces the earlier pending-access status. It does not establish model
compatibility, benchmark performance, or readiness for VO evaluation.

## Supplied downloads

| User-labelled release | Subset | Drive ID | Observed ZIP bytes |
|---|---|---|---:|
| 1.1 | Indoor | `19bqcm81xgaWg6tBjf6MS715PoxN3VzwG` | 2738757115 |
| 1.1 | Outdoor | `1i9P-P5G1E8FM1bEYn4pocFQSmJT8GG4b` | 1311266543 |
| 1.1 | Entire | `1wfR80y1y2-p3nDOAk-OQdyId_PUMhasl` | 4050023636 |
| 2.0 | Indoor | `1RYd_cjRmhjPI_eBZ0Am_oVlEGaooEbcb` | 2977111796 |
| 2.0 | Outdoor | `1rfZksaXzIkenBRFoW4pxhfDhkI2lylWZ` | 1384498822 |
| 2.0 | Entire | `1jYh69ePRwGoiHOVKggw4Z3Xb2WkI0TK1` | 4361875586 |

All six metadata reads succeeded; metadata alone is not a content-access test.
Only the two v2 archives named below had their contents accessed this session.
No version-to-version performance or completeness comparison was made.

## What was actually retrieved

The v2 outdoor ZIP index has 2872 entries and 68 paths matching the expected
rectified-left-color scene layout. The v2 entire-release index has 10125 entries
and 168 such paths. These are structural path counts, not a revised official
scene census. The entire-release index read used 1404898 range bytes; its
scene pixels were not downloaded.

Before reading scene pixels or references, the selection was saved as the
first lexicographic indexed outdoor scene: `OUTDOOR/CARS/scene1/`. Its selected
23 files include original and rectified images, masks, sparse/interpolated
disparity maps, disparity-range text and the scene ASC point cloud. Retrieval
used 14689317 range bytes including index/header reads. The full 1.38 GB ZIP was
not downloaded. No other scenes were selected based on image or model outcomes.
This scene is now inspected development material; it must not later be labelled
an untouched prospective evaluation scene.

Every HTTP read required an exact 206 response, expected Content-Range and byte
count. ZIP CRC verification passed for all 23 extracted members; local SHA256
hashes and byte-range hashes are saved. This verifies retrieved members and
transfer integrity, not a publisher-supplied full-archive cryptographic digest.
Transient confirmation URLs and the full thesis PDF are excluded from recovery.

## Sample schema

| Sample component | Observed format | Immediate implication |
|---|---|---|
| Color raw and same-modality rectified stereo | 960×1280×3, uint8 | Preserve native dimensions and rectification coordinates. |
| Thermal raw and same-modality rectified stereo | 480×640, uint16, observed values 0–65535 | Requires a declared input conversion; it is not the previous uint8 thermal producer. |
| Cross-modality rectified pair | 480×640 thermal uint8 and color uint8 | This is a separate image product; do not substitute it for native thermal stereo or assume common pixels with the other rectifications. |
| Rectification masks | uint8 with values 0 and 255 | Mask meaning must be established before choosing evaluation support. |
| Sparse color disparity | 960×1280 comma-delimited float text, 134266 finite entries | NaNs represent absent stored values; sign and zero semantics remain to be verified. |
| Sparse thermal disparity | 480×640 comma-delimited float text, 118401 finite entries | Do not inherit the previous strictly-positive disparity filter. |
| Sparse cross disparity | 480×640 comma-delimited float text, 117752 finite entries | Cross-camera correspondence direction/coordinates require verification. |
| Interpolated disparity | Finite at every pixel in the inspected maps | Keep separate from sparse measured-reference support; interpolation is not new independent ground truth. |
| Point cloud | `gt/scene1.asc` text | File availability does not establish point-cloud units or camera projection transforms. |

Stored sparse disparity includes both signs: color −98 to 30 pixels, thermal
−24 to 55, cross −56 to 12. Their interpolated maps exceed some of those
stored range limits. Do not take absolute values, flip a sign, clip the map,
or treat `disp_range.txt` as a guaranteed bound for the interpolated product.
The initial whitespace-delimited text probe failed because these files are
comma-delimited; the schema inspection used the correct parser and changed no
source bytes.

No calibration/README-named entries or conventional MAT/YAML/XML calibration
files were found in the inspected v2 indices. The listed CSVs are label/bounding
box files. This is a naming/index observation, not proof that calibration is
unobtainable or absent from every distribution. Intrinsics, stereo projection
offsets, disparity direction, reference masks and cross-rectification mapping
remain unverified. None may be copied from MS² or invented to make the adapter run.

## Role in this research

The [official CATS setup](https://bigdatavision.org/CAT/setup.html) describes
four cameras and a slow laser scanner observing static scenes. The
[CVPR2017 benchmark](https://openaccess.thecvf.com/content_cvpr_2017/html/Treible_CATS_A_Color_CVPR_2017_paper.html)
provides intra- and cross-modality disparity references from that acquisition.
This offers a reference-construction chain distinct from the MS² products.
Projected-label accuracy still depends on calibration, alignment and visibility;
scanner accuracy alone is not total reference accuracy.

CATS is useful for an external **stereo measurement** study. It cannot directly
validate temporal tracking, agent decisions over motion windows, trajectory
accuracy or recovery after tracking loss. That requires temporal stereo data
with an applicable pose reference. Training membership also remains unverified.

The next bounded implementation task is an adapter/convention audit on this
already inspected scene: identify stored disparity direction/offset and mask
semantics, verify left/right rectified coordinates, and declare thermal
preprocessing before inference. Begin with native disparity-error/ranking
metrics that do not require invented metric calibration. A common-metric
cross-camera target requires the additional geometric information above.
No model inference or comparison was performed in this intake.

## Reproduction and artifacts

Scripts: `MAC-VO-main/Scripts/Setup/cats_bounded_intake.py` and
`cats_inspect_sample.py`. From the MAC-VO root:

```bash
/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.cats_bounded_intake \
  --file-id 1rfZksaXzIkenBRFoW4pxhfDhkI2lylWZ --archive-bytes 1384498822 \
  --output <fresh-output> --sample
/root/ore-ms-analysis-venv/bin/python -m Scripts.Setup.cats_inspect_sample \
  --run <fresh-output>
```

Saved outputs: `results/cats-intake/outdoor-v2/` and
`results/cats-intake/release-v2-index/`, including index, pre-pixel declaration,
member/range hashes, schema and a display-only overview. The overview's thermal
contrast uses percentiles for visualization only; no inference recipe is
selected. Private recovery target: `backups/2026-10-28-cats-selector-proposal`.
The user-approved download links are recorded; no credentials or full archives
are packaged. The [official download page](https://bigdatavision.org/CAT/download.html)
states the dataset license; no public redistribution or benchmark submission
was performed.
