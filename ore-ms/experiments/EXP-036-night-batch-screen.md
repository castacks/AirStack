# EXP-036: Batch readiness screen of remaining October 27 night recordings

Recorded: 2026-10-22T22:26:57.192147-04:00. Branch `ore_ms`; starting commit `9e40ba724` plus recorded acquisition/analysis sources. **All four newly available archives were screened. Most sampled RGB is very dark; redcourse-waypointskip1 has a brief visibly illuminated late window and the strongest sampled RGB support. It is a candidate for bounded follow-up, not established sustained VO availability or a completed cross-modal error comparison.**

## Fixed scope and selection

The user requested all remaining night-session folders together because names cannot identify brighter intervals. Authenticated Box folder `428559630945` supplied four archives totalling 81,078,773,336 bytes, beyond the three previously inspected recordings. Archive inventories and selected source members were retrieved through byte ranges; the whole ZIPs were neither downloaded nor SHA256-verified. Each selected member passed ZIP CRC and SHA256 registration. These are source processed thermal images, not MSA-regenerated images. No learned model, checkpoint, trajectory, calibration, weighting or default was changed.

For each modality independently: choose twelve evenly spaced unique left-image acquisitions over the full capture, plus the first eligible twenty-frame block in each chronological third, requiring positive gaps <=150ms. Freeze these choices before image decoding. Every one of the 24 modality/third windows is available. The three windows need not observe the same cross-modal acquisition times and must not be treated as synchronized errors. Invalid timestamp rows are excluded with original image indices and counts retained; duplicates/missing filenames are also excluded. Exact selected indices/times, full inventories and excluded counts are in each plan. Scout sampling can miss short bright intervals; this is not an exhaustive pixel audit.

## Native-image tracking readiness

| Recording | RGB LK support early/middle/late | Thermal LK support early/middle/late |
|---|---:|---:|
| redcourse-waypointskip1 | 1.39% / 0.00% / 10.56% | 96.53% / 69.25% / 83.33% |
| teleop-fig8 | 0.11% / 0.00% / 0.00% | 96.53% / 71.03% / 78.08% |
| thermal-salon | 0.00% / 0.00% / 0.06% | 96.92% / 78.08% / 96.53% |
| thermal-salon3 | 1.00% / 0.33% / 4.06% | 95.34% / 72.52% / 89.58% |

Entries are medians over 19 adjacent pairs in each fixed window. Native grayscale LK uses the inherited EXP-035 settings: 32px border/16px grid,21x21 window,3 pyramid levels,30 iterations/epsilon0.01,minEig1e-4, finite/inbound/status gates and <=1px forward/backward consistency. Also preserve <=0.5px and texture std>=5 sensitivities and raw point arrays. RGB is1024x544 and thermal640x512; their grids/views differ. Percentages describe algorithmic support in each stream, not calibrated modality accuracy.

All48 RGB scouts and48 thermal scouts were decoded into native/explicitly labelled CLAHE-display contact sheets. All four RGB sheets were inspected. RGB is predominantly near-black with intermittent vegetation/isolated lights. Redcourse final scout shows a gate; fixed late frames4173–4192 show illuminated vegetation and blur. That late block has10.56% median RGB support (pair range2.78–19.78%),8.72% under the tighter gate and4.33% with texture restriction. Its native grayscale std median14.30 contrasts with mostly~0.7–1.6 in the darker windows. Limited spatial illumination and pair variability remain important; no support threshold was fitted or declared a VO pass.

## Timestamp audit

| Recording | RGB stereo abs median ms | Thermal stereo abs median ms | Cross-modal signed median ms | Cross-modal abs p99 ms | Within 30ms |
|---|---:|---:|---:|---:|---:|
| redcourse-waypointskip1 | 0.000 | 1.000 | -41.481 | 59.8 | 11.0% |
| teleop-fig8 | 0.000 | 3.000 | 5.320 | 494.2 | 90.1% |
| thermal-salon | 0.000 | 1.000 | -3.383 | 2370.9 | 89.2% |
| thermal-salon3 | 0.000 | 0.000 | 0.109 | 21972.2 | 65.6% |

Cross-modal sign is nearest thermal-left time minus RGB-left time, over all available unique image timestamps; it is descriptive, not a corrected clock offset or one-to-one matching. RGB/thermal holes and nonoverlapping support inflate tails. Thermal-salon3 has an abs cross-modal99th percentile near22s despite a near-zero signed median. Stereo tails also contain gaps (thermal-salon3 abs99th~1.9s). Do not select a recording using median offsets alone, reuse an image to create artificial synchronized frames, or apply a time correction from these summaries. Redcourse's~42ms separation is relevant to any rapid-motion comparison.

## Validation and reproducibility

1,985,521 independently recomputed checks pass across640,224 point observations and456 temporal pairs: selected-member CRC/size/SHA, fixed selection, strictly increasing times, independent vectorized nearest matching, per-point status/boundary/finite/forward-backward masks, direct-patch texture and aggregate support. Historical synthetic known-translation and flat-image controls pass. This is computational consistency evidence, not physical camera/GNSS/VO accuracy.

An initial extra synthetic random-texture fixture failed a proposed95% reverse-tracking acceptance check (78.6% accepted, accepted-point translation error~0.00022px). The frozen historical EXP-027 fixture/control was then used unchanged instead. The initial log is retained; source images, real-data masks/outputs and scientific thresholds were unchanged. Two acquisition setup failures (nonfinite timestamp conversion and an incorrect RGB-right stream name) were corrected before final analysis; original logs remain. The final RGB-right stream is `multisense_right_rect`.

Runtime: Python3.12.3, NumPy2.2.6, OpenCV4.10.0, CPU, OpenCV single thread/seed0. Packages were installed into an isolated package directory, without changing system packages. Acquisition entry point: `Scripts/Setup/td3_batch_screen.py --config <private-config> --output <new-output> <four-recording-names>`; analysis/validation/documentation scripts and exact commands/runtime are preserved under `results/night-batch-screen/staging/`. Analysis reuses the two frozen historical LK helper functions without importing their unrelated runtime dependencies. Previous model/data results are not prerequisites for this source-image screen.

## Decision

Prefer a bounded follow-up on the redcourse late illuminated interval before concluding that all recordings lack RGB detail. It is outcome-selected exploratory follow-up and must remain separate from independent evaluation. Inspect nearby continuity, stereo/cross-modal pairing and sustained tracking before model inference. Teleop-fig8 and thermal-salon primarily supply RGB-unavailability cases; thermal-salon3 has sparse RGB structure and substantial timing coverage holes. None of these results alone establishes a usable full RGB/thermal VO comparison or resolves training membership/reference quality. No additional upload is required to examine the redcourse candidate. Public datasets remain a fallback if a suitable sustained interval cannot be found here.

Artifacts: `results/night-batch-screen/`: four source inventories/plans/extractions,569 unique selected left images plus timestamp files,24 point-array case sets, native/enhanced contact sheets, analysis/timing summaries, validation, exact scripts/logs and hash registry. Recovery target: Box `backups/2026-10-22-exp-036`; archive excludes private credentials, packages, disposable range caches and whole source ZIPs.
