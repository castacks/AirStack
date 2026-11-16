# Paired SmokeRoom and FlashLight evidence restored on the current machine

16 November 2026, America/New_York. Completed the
[restoration protocol](multirecord-restoration-protocol-2026-11-16.md) after the
[priority/headroom control](camera-priority-findings-2026-11-16.md).
The missing paired predictions are now available locally for both recordings.
This is unchanged re-execution of exposed baselines, not independent replication,
a new quality selector or a generalization experiment.

## Restored populations and retained failures

| Recording | Decoded images | Anchors | Common reference | Both-camera rotation ≥2° | Both displacements ≥1cm | Prediction rows |
|---|---:|---:|---:|---:|---:|---:|
| FlashLight | 14,715 | 541 | 538 | 192 | 519 | 2,164 |
| SmokeRoom | 22,800 | 838 | 837 | 236 | 483 | 3,352 |

All declared anchors remain. Each has four rows: SIFT/learned × RGB/thermal,
including unsupported inputs and failed proposals. Choices use the unchanged
status-only RGB-first fallback, locked before supplied-reference scoring.
The reference and motion cohorts were frozen before predictions. Both full CPU
qualifications and their independent audits finished before either timed GPU run;
FlashLight then SmokeRoom ran sequentially in the exact recovered compiler image.

46/46 historical descriptive count checks match; status `exact_count_parity`.
The pre-capture parity plan checks five population counts and eighteen moving
output/1°/5° counts per recording, spanning both frontends and three policies.
Full details and any differences remain in
`results/multirecord-restoration/historical-parity-report.json`.
Old raw prediction arrays are unavailable; count parity does not establish
bitwise equality with old predictions. Fresh native-size flow repeats were bitwise
identical and model states unchanged for each recording.

## Restored learned moving results

These are counts on each recording's own frozen moving cohort, not pooled rates.

| Recording | Fixed camera / policy | Outputs | Supplied-reference R≤1° | R≤5° |
|---|---|---:|---:|---:|
| FlashLight | rgb | 192 | 176 | 185 |
| FlashLight | thermal | 168 | 57 | 110 |
| FlashLight | rgb_first | 192 | 176 | 185 |
| SmokeRoom | rgb | 221 | 151 | 181 |
| SmokeRoom | thermal | 231 | 116 | 190 |
| SmokeRoom | rgb_first | 236 | 155 | 192 |

The restored comparison preserves mixed outcomes: FlashLight learned RGB has the
stronger angular result; SmokeRoom thermal contributes support/coarse estimates
while RGB is more precise. SIFT remains separately reported in full summaries;
it must not be replaced with the learned ranking. No new quality decision,
thermal-first replay, cue search, threshold tuning or training was performed.
Fresh timing is retained as descriptive cached matching computation, not live
lazy latency, a historical timing reproduction or stateful VO performance.

## Input provenance and qualification limits

All 37,515 exported images were decoded, hashed and ordinal/time-bound. Download
receipts retain strict byte-range lengths/totals, full SHA and ZIP member CRC.
FlashLight's full archive/reference SHA matches the previously documented values.
SmokeRoom's newly observed archive SHA is
`e931d4e975ab706763568824ebbd30a93343c8a8f20e715a512c68c89382ead4`;
no old complete-archive receipt was available for comparison.

Each recording's own published calibration file was independently fetched through
the connected Drive tool and public transport, with equal bytes. The published
SmokeRoom, FlashLight and OutdoorNight matrices currently have identical numeric
values; source IDs and file associations are preserved individually rather than
assuming different matrices or silently borrowing one recording's files.
The existing explicit thermal bottom-row repair and numerical SO(3) correction
are retained without calibration fitting.

SmokeRoom/FlashLight gyro/reference median discrepancies are respectively
0.03853°/0.07536°, with p95 0.10821°/0.19776° under the unchanged generic gates.
These support nominal supplied-reference consistency. They do not certify
independent physical pose truth, exposure metrology or absence of fused-reference
IMU dependence. Historical SmokeRoom raw-header proof is not freshly repeated;
its raw bags were not restored in this step. FlashLight's mismatched July raw
release was not substituted for the March export.

## Verification, paired index and decision

| Recording | Independent input checks | Independent comparison checks | Verdict |
|---|---:|---:|---|
| FlashLight | 80,628 | 37,968 | pass |
| SmokeRoom | 124,919 | 54,568 | pass |

Audits independently reconstruct full populations, errors, missing statuses,
frame/time pointers, source pins, cohorts, fallback choices and projected costs.
Both native shapes also pass repeated-flow, unchanged-model and selection/ray
mechanics checks. All failed estimates remain in the archived row population.

`results/multirecord-restoration/paired-evidence-index.json` now points to all
9,400 prediction rows across FlashLight, SmokeRoom and OutdoorNight. These are
four configurations per anchor, not 9,400 independent trials. The index labels
SmokeRoom/FlashLight as nominal supplied-reference relative pose targets and
OutdoorNight as nominal raw-gyro angular agreement only, with no translation
truth. These labels must remain separate; there is no pooled accuracy statistic
or verified training-held-out claim.

- FlashLight fresh prediction SHA256: `2537b9d19263cb45ef8781a3128e7b5999de7ab8dad530af304cc1ec2ca68bae`.
- SmokeRoom fresh prediction SHA256: `74064530576cd52a287e348df8631e2665a22f1f1aa54603ceb8e1b426ad2296`.

Full evidence: `results/multirecord-restoration/`; verified local incremental
packet and per-member SHA manifest:
`/root/ore-ms-artifacts/multirecord-restoration-2026-11-16/`.
It contains the actual fresh paired rows, reference/population manifests, locks,
source pins, download receipts, calibration and audits. Large images, checkpoints
and runtime remain external pinned inputs; OutdoorNight rows/gyro targets remain
in the separately pinned matching/gyro packets. No Box upload or Git publication.

Stop decision: paired evidence restoration is complete. The next bounded study
can replay both fixed priority orders on the restored recordings and measure
remaining headroom before proposing a new cue. Any subsequent quality protocol
must keep distinct reference tiers, fixed cameras/orders and recording-level
splits; these already exposed outcomes cannot become blind validation data.
