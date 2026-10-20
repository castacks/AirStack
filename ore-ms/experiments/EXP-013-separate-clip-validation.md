# EXP-013: Frozen u-only rule on a separate night recording

Date: October 20, 2026 (America/New_York). Branch `ore_ms`; base commit
`3ac07f047` plus EXP-010/011/012 working changes.
**Completed with two RGB tracking failures and mixed transfer of the frozen rule.**

## Result and decision

The day-clip u-only tradeoff does **not** transfer consistently to this night
recording. Thermal early accumulated translation improves 0.3011→0.2452 m
but rotation worsens 1.3427→1.4378 degrees. In middle, both worsen:
0.7670→0.9822 m and 2.6308→4.3995 degrees. Late changes are small, with
translation 0.1420→0.1423 m and rotation 0.1086→0.1143 degrees. The late
window also has little reference motion; it was retained as preregistered.

Thermal tracks all three windows without interpolation. RGB tracks only early;
middle retains four matches and interpolates 15 frames, while late retains no
matches and the runner explicitly fails. Those failures remain in the report
and are not replaced by easier windows or treated as usable accuracy estimates.
The eligible early RGB result changes little under redistribution.

Four eligible cases pass native replay checks (228 pair solves). Technical
checks pass while the proposed weighting benefit is mixed; these are distinct
outcomes. Do not adopt a fixed u-only replacement or tune on this validation
clip. Keep defaults and return to uncertainty/retention characterization around
tracking failures, alongside geometry/reference validation.

## Question and data gate

Does EXP-012's u-only translation/rotation tradeoff persist on another recording
when its rule is applied unchanged? This tests transfer of the frozen replay
rule; checkpoint training-split membership is unknown, so a new recording must
not be described as held out from network training without evidence.

The user staged `night_thermal_so_salon2.zip` in Box folder `428559630945`
(file ID `2538597547094`, 7,712,416,758 bytes). `recording_audit.json` identifies
`/media/airlab/offroad/rosbags/20251027/night_thermal_so_salon2`, distinct from
`20251027/1_salon_mac_cov_day`. Source acquisition timestamps do not overlap
those previously evaluated. No lab NAS mount or new access credential was needed.

Require a different original recording, with four processed camera streams,
their acquisition timestamps, camera-info/intrinsics including rectification
and projection matrices, applicable extrinsics, TF, and recorded odometry
(`info.yaml`, `interp_data.txt`, `interp_timestamps.txt`). Keep source paths,
recording identity and file hashes; a renamed copy or another window of
`20251027/1_salon_mac_cov_day` does not meet this gate. Compare source/acquisition
provenance and reject overlap with the already evaluated recording.

## Fixed protocol before new outcomes

1. Audit the clip's source identity, encodings, timestamps, camera frames,
   calibration applicability, rectification and reference provenance before
   geometry evaluation. Keep provided uint8 preprocessing with the existing
   BGR-to-RGB/float32-divide-by-255 recipe and grayscale replication. Reject raw
   uint16 rather than silently normalizing it. Do not change the original
   checkpoints, absolute baseline/disparity execution convention or camera-local
   frame to obtain a favorable result. Unresolved geometry remains explicit;
   incompatible metadata is a reason to stop and audit, not force a comparison.
2. Pair with the existing chronological-nearest-without-reuse tool, using
   unchanged exploratory bounds: stereo 2 ms, every RGB/thermal pair 30 ms.
   Preserve actual acquisition times and reject reuse/duplicates as the current
   tool does. Do not relax limits after examining trajectory results.
3. Divide the paired manifest chronologically into first/middle/last thirds.
   In each third, select a complete 20-group window by maximum recorded body
   endpoint displacement, breaking ties by earliest manifest row. Require each
   modality's left-camera time gaps at most 150 ms, interpolation coverage at
   every selected timestamp, and at least one eligible window in each third.
   Select using only timestamps and recorded body motion, before running VO.
   All three windows must be disjoint. If eligibility fails, record why and
   pause this protocol rather than silently changing it.
4. Freeze recording identity, selected rows, model/source hashes, calibration,
   reference inputs, runtime and all settings before baseline inference.
   Use the same RGB/thermal checkpoints and seed-0 fp32 bounded runner with
   mapping disabled and synchronous optimization. Run six new 20-frame baseline
   cases; do not regenerate or replace the old evaluation clip.
5. On saved baseline inputs replay three arms: `learned`, `fixed_u_mass`,
   `fixed_uv_mass`. Reuse EXP-012's weight function unchanged. A selected channel
   gets variance `N / sum(1 / original_variance)` separately per pair/channel;
   no reference enters the rule and no mixing strength is fitted. U-only keeps
   v bitwise unchanged. Disparity, matches, points/depths and priors stay fixed.
6. Use the existing CPU image and native analytic LM/Huber settings, one thread,
   seed 0. Require the learned replay to reproduce the new baseline local
   motions within 1 mm / 0.01 degree, all channels to preserve precision totals
   within relative 1e-6, unchanged unselected channels and nonpixel hashes, finite
   unit poses/losses, and unchanged frozen source files. The both-channel arm
   is a contemporaneous comparator on the new clip, not a required reproduction
   of old-clip motion. Do not require a preselected favorable metric outcome.

## Evaluation and decision

Report every window/modality/arm, with both accumulated and consecutive local
translation and rotation RMS against the recorded-SLAM proxy. Use first-pose
anchoring, no fitted scale or best-fit alignment. Report execution/tracking
failures and retention counts alongside completed cases; do not drop failures
or choose windows/arms based on their errors. Distinguish a technical replay
pass from whether the u-only tradeoff transfers. Report unfavorable or mixed
results without further tuning on this clip.

Even a favorable new-clip replay does not establish end-to-end VO benefit,
calibrated cross-modal confidence or independent ground-truth accuracy. Frozen
priors and baseline covariance-informed selection remain limitations; signed
thermal geometry, original preprocessing and reference quality remain open.
Keep defaults until the evidence and geometry justify a separately reviewed
integration experiment. Preserve source, plans and outputs in Box.

## Input audit and execution preparation

The immutable preregistration snapshot is
`results/separate-clip/protocol-2026-10-20/protocol.json`; its source fingerprints
match the unchanged old implementation. Metadata and 240 selected images were
restored by authenticated ZIP range reads, with CRC checks on selected members.
The full archive was not downloaded or SHA-256 rehashed. Central-directory
inventory and range-extraction manifests preserve membership, offsets and file
hashes. The 12 selected-image range blocks total 58,014,716 bytes.

Pairing produces 1,149 unique four-camera groups. Unique acquisition counts
before pairing are RGB left/right 1,948/1,949; thermal left/right 1,882/1,864.
Duplicates and absent samples are explicitly excluded by the existing policy.
Every selected RGB image is uint8 at 544×1024 (left color, right grayscale);
thermal is uint8 color at 512×640. No raw thermal normalization is introduced.

All 13 supplied YAML transforms match recorded matrices, with the same reversed
archive pair labels as the day clip. Left camera metadata, projection intrinsics
and signed baselines match the original run. The same thermal geometry residual
persists (horizontal discrepancy 0.6701 m, off-axis 0.00614 m, rotation 0.617°).
This supports reuse of the provisional execution convention, not physical
calibration validity. Recorded dynamic TF agrees with the interpolated odometry
to <4.5e-14 m / <2.5e-14 degrees; these are the same SLAM source, not independent
reference validation.

Frozen, disjoint windows selected without VO outcomes:

| Window | Manifest start | Manifest region | Body-reference displacement (m) |
|---|---:|---|---:|
| early | 123 | [0,383) | 1.7623 |
| middle | 619 | [383,766) | 6.1707 |
| late | 789 | [766,1149) | 0.2613 |

The late window remains despite low motion; no window was substituted after
selection. Thirty-seven focused tests pass, including six new selector tests
for thirds, ties, both-camera gaps, reference coverage, insufficient input and
maximum motion. The original model bundle SHA-256 matches and strict MACT GPU
inference passes (428 active checkpoint entries, finite positive covariance).
Per-model SHA-256 values match the earlier run. The original bounded runner,
checkpoint loader, covariance/optimizer code and EXP-012 rule remain unchanged.

The execution plan at `results/separate-clip/night-run-001/plan.json` freezes
481 source/input files before baseline inference, including selected images,
metadata, weights, configuration and source. `execution-plan.json` additionally
records image IDs and eager/one-thread/seed settings before any baseline run.
Both runtime package freezes are retained. All baseline attempts, including
failures, were evaluated against the frozen plan. No selected window, image,
checkpoint, geometry setting or weight rule was changed after inference.


## Tracking and replay eligibility

| Window | Modality | Retained matches | Interpolated frames | Replay status |
|---|---|---:|---:|---|
| early | RGB | 3791 | 0 | pass |
| early | thermal | 3748 | 0 | pass |
| middle | RGB | 4 | 15 | unavailable: interpolated baseline |
| middle | thermal | 3677 | 0 | pass |
| late | RGB | 0 | not exported | unavailable: no retained matches |
| late | thermal | 3313 | 0 | pass |

The unchanged runner labels middle RGB `PASS_EXECUTION_ONLY`, because it checks
finite execution rather than full tracking. The validation gate separately
rejects interpolation. Late RGB processes 20 frames and then raises
`RuntimeError: No visual matches were retained`; no successful graph/trajectory
artifact is claimed. No threshold, normalization, checkpoint or window changes
were made to rescue either case. The JSON aggregate `all_cases_pass` is false
because of these tracking failures; all four eligible replay controls pass.

Subsequent raw recording in EXP-014 clarifies the flag semantics: all 19 middle
RGB pairs have fewer than ten retained matches. MotionInterpolate clears the
first/last two motion flags, leaving the 15 saved flags above. These are
postprocessing flags, not a complete count of low-retention pairs; cleared
boundary flags do not establish successful tracking. Original outputs remain
unchanged.

## Accumulated and local proxy disagreement

Each cell is **translation RMS in meters / rotation RMS in degrees** against
recorded SLAM, first-pose anchored without scale or best-fit alignment.
Unavailable RGB cases have no accuracy estimate and remain visible.

| Window | Modality | Learned | U-only, mass preserved | Both, mass preserved |
|---|---|---:|---:|---:|
| early | rgb | 1.0960 / 2.5362 | 1.0946 / 2.4218 | 1.0894 / 2.2097 |
| early | thermal | 0.3011 / 1.3427 | 0.2452 / 1.4378 | 0.2088 / 1.5369 |
| middle | rgb | unavailable | unavailable | unavailable |
| middle | thermal | 0.7670 / 2.6308 | 0.9822 / 4.3995 | 0.8890 / 5.1733 |
| late | rgb | unavailable | unavailable | unavailable |
| late | thermal | 0.1420 / 0.1086 | 0.1423 / 0.1143 | 0.1410 / 0.1499 |

Consecutive local-motion disagreement, same units:

| Window | Modality | Learned | U-only, mass preserved | Both, mass preserved |
|---|---|---:|---:|---:|
| early | rgb | 0.1016 / 0.3213 | 0.1016 / 0.3153 | 0.1012 / 0.3002 |
| early | thermal | 0.0456 / 0.3134 | 0.0479 / 0.3165 | 0.0460 / 0.3161 |
| middle | rgb | unavailable | unavailable | unavailable |
| middle | thermal | 0.1700 / 1.0174 | 0.1566 / 1.0146 | 0.1564 / 1.1196 |
| late | rgb | unavailable | unavailable | unavailable |
| late | thermal | 0.0294 / 0.0132 | 0.0294 / 0.0137 | 0.0294 / 0.0164 |

Thermal middle local translation slightly improves under u-only
(0.1700→0.1566 m), despite accumulated translation worsening
(0.7670→0.9822 m). Endpoint/accumulated and local effects must both be reported.
U-only worsens accumulated thermal rotation in all three night windows.
Both-channel redistribution does not resolve that tradeoff. These are
conditional replay findings, not independent ground-truth accuracy or a
cross-modal reliability calibration result. Unknown original preprocessing,
provisional thermal geometry and baseline feature-selection bias remain.

## Validation, runtime and artifacts

Six new selector tests plus 31 relevant regression cases pass (37 total).
Selected-member CRCs, model-bundle SHA-256, per-model historical SHA-256,
encoding/dimension checks, source identity and calibration comparison pass.
The four eligible learned replays match new baseline local motion to maximum
1.82e-7 m / 7.51e-7 degrees, below 1 mm / 0.01 degree tolerances. All 228 solves
produce finite unit poses and finite loss histories. All channel/unselected-
channel/nonpixel invariants pass; precision conservation error is at most
5.62e-8 (limit 1e-6). All 481 frozen source/input hashes and all baseline file
hashes remain unchanged before/after replay. No eligible case is excluded
because of its error result.

GPU image: `sha256:04174182ea09037cdc6de6a3ae286ff0663db4ea6592a240008e9bb314e355b1`,
Torch 2.7.1+cu128 on RTX PRO 5000 Blackwell. CPU replay image:
`sha256:bf5d89c63af6c867c49aeee4db2bb4ef1486c1a65ac026521de35fb8495a1cc2`,
Torch 2.7.1+cpu. Both use PyPose 0.9.5, NumPy 2.2.6 and SciPy 1.17.1.
Baseline inference uses fp32, seed 0, eager math, mapping disabled and synchronous
CPU optimization, with one CPU thread. Images are launched by immutable image
ID. Full package freezes and exact commands are preserved.

Artifacts under `results/separate-clip/night-run-001/`: frozen plans, all six
baseline logs/reports, tracking/status table, successful baseline graphs and
poses, twelve replay motion NPZ files, full `replay/analysis.json`,
`replay/summary.csv` (including unavailable cases), input hashes,
`night-validation-comparison.png/pdf` and validation/runtime provenance.
`results/separate-clip/night-audit/` preserves source/archive inventory,
range-extraction manifests, input/calibration audit and exact staging/reporting
scripts. The reporting script regenerates both figures; hashed binary range
caches are disposable and are excluded from backups. Data/models/credentials
remain outside Git. No full input ZIP hash or network-training holdout claim
is made.

## Next bounded research step

Stop pursuing uniform u-only weighting as a default based on the day window.
Instrument the unchanged baseline with its existing opt-in raw-uncertainty
recorder on these same fixed night windows, retaining failed RGB cases. Verify
that enabling recording preserves successful baseline graphs, then inspect
raw preselection pixel variance, retained/inbound counts, depth and image-region
strata around tracking loss. This is characterization to test whether learned
uncertainty signals failure; no new weights, masks or scalars should be tuned
from this clip. Define justified comparable error measurements and resolve
thermal geometry/reference quality before calibration or fusion claims.
