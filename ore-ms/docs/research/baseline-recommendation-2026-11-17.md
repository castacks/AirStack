# Baseline recommendation and independent validation

17 November 2026 America/New_York. Decision from the completed
[matched-count comparison](matched-count-findings-2026-11-16.md),
[OutdoorNight transfer](outdoornight-crop-findings-2026-11-16.md) and
[temporal assay](temporal-output-bursts-findings-2026-11-17.md).
This is a synthesis of exposed development results, not a new experiment or
an approval to deploy. Branch `ore-ms-orb-development`, committed base `4408b7fb8`
plus the studies' frozen source identities.

## Recommendation

**Keep the original frozen learned RGB-first pipeline as the comparison anchor.
Retain the fixed crop as a challenger; do not enable it universally.** RGB-first
is an established reference configuration, not a claim that it is the best policy
for every condition. Preserve the original proposal population and its coverage
rather than replacing the baseline with whichever exposed score is highest.

| Baseline component | Role in the next independent evaluation | Reason |
|---|---|---|
| Original learned RGB-first | Fixed reference configuration | Established paired baseline; preserves comparison with all previous studies |
| Fixed RGB, fixed thermal, thermal-first | Mandatory comparators under original and cropped geometry | Camera preference changes with record and endpoint; output status cannot identify all inaccurate accepted poses |
| Fixed lower-third crop | Prespecified accuracy challenger | Moving/joint gains exceed one count control in paired records, with meaningful coverage and worst-block costs |
| One fixed matched-count draw | Mechanism control | Separates this crop comparison from merely retaining fewer correspondences; does not estimate seed variance |
| SIFT | Secondary frontend control | Several modality gaps and crop effects change with frontend; learned and SIFT results cannot be substituted |

There is **no demonstrated universal deployment winner**. Do not implement a
record-ID switch, tuned identity/static backup or camera gate from these results.
A new policy would require its own prospective protocol and validation.

## Evidence that drives the decision

Original→crop, learned RGB-first; failures remain in each fixed denominator.
FlashLight/SmokeRoom use nominal supplied camera-reference poses:

| Record | Moving R≤1° | Moving R≤5° | Joint R≤5°/directed-t≤15° | Full outputs | Longest full5°bad run, sampled anchors |
|---|---|---|---|---|---|
| FlashLight | 176→180 /192 | 185→192 /192 | 452→465 /519 | 527→523 /541 | 4→6 |
| SmokeRoom | 155→196 /236 | 192→221 /236 | 356→410 /483 | 551→519 /838 | 26→20 |

The same-count controls give FlashLight176/185moving and451joint; SmokeRoom
168/204moving and371joint. The crop gains therefore exceed this fixed count draw,
while SmokeRoom whole-common coarse success falls507→503/837. No causal rig-only
or sampling-variability claim follows.

**OutdoorNight is a separate nominal gyro-agreement tier.** RGB-first moving
within1°/5° improves125/180→158/205of242 (count146/193); full outputs fall840→822of971,
and the longest full5°bad run rises9→20anchors. Original fixed thermal already
has158/212moving agreements, and thermal-first164/218. Cropped thermal-first
171/222remains below its count control177/222at the strict bound. Neither a
universal RGB-first nor a universal crop choice is supported. OutdoorNight has
no qualified translation endpoint in this comparison.

The longest primary bad runs are entirely missing outputs outside the frozen
≥2°moving cohorts. They do not establish physical robot failure. At0.5s anchor
spacing, kbad decisions span(k−1)*0.5s between samples; these are not continuous
dropout or VO-recovery durations. Strong moving averages and acceptable low-motion
availability must be evaluated separately.

## Follow-on recording results

[ViViD++](vivid-zero-rotation-control-2026-11-17.md) adds learned thermal
proposals, with no gain over identity in the nominal 5° diagnostic.
[IRS low-dynamics recordings](irs-rtvi-mocap-comparison-2026-11-17.md) favor
thermal-first in the dark and visual-first in the bright record.
[IRS faster dark](irs-rtvi-dark-fast-replication-2026-11-17.md) replicates the
thermal-first direction. Its fixed shorter-interval control improves motion
agreement, with a longer no-proposal block near the start of the recording.
These recording-dependent outcomes preserve the need for all four fixed
comparators and a qualified reference for any stronger accuracy claim.

## What must be validated independently

The next executable study should be a **prospective recording/reference intake**,
not another fit or selector on these three exposed records. Before model execution:

| Requirement | Concrete evidence or decision required |
|---|---|
| Independent data | A new recording not used to choose the crop/policy, with documented light or smoke transition and stop/start motion, retaining clear and low-motion controls. Check existing local/public inventory first; no private-upload dependency is assumed. One new record tests that record, not broad generalization. |
| Qualified reference | Establish camera/body frames, extrinsics, clock/exposure semantics, interpolation/gap support and reference provenance before scoring. A reference sharing the evaluated sensor stream needs an explicit dependence qualification; gyro alone remains angular agreement, not independent full-pose truth. |
| Frozen evaluation | Pin the full recording, models, source/runtime, masks, counts, thresholds, four status policies and missing-data rules before predictions. Keep these three records as exposed development evidence. No successful-window, seed or threshold replacement after outcomes. |
| Comparable endpoints | Retain fixed1°/5° rotation, moving and low-motion/full populations, output counts, paired gains/losses and censored temporal blocks. Joint camera-direction scoring requires qualified translation support. Keep reference tiers separate. |
| Application requirements | Set tolerated error, no-output periods and latency before the independent trial. Those application limits are not supplied here; observed maxima must not become acceptance thresholds after the fact. No candidate currently has a deployment pass. |
| Stateful and compute validation | Deployment claims additionally require a sensor-matched stateful estimator with trajectories, drift, real failure/recovery and measured live request/latency costs. Native RGB stereo, thermal stereo and an RGB–thermal extension require distinct input contracts. Cached matcher projections and pair proposals cannot establish these results. |

If a candidate recording lacks the required reference, retain useful input/support
characterization and state the limit; do not fit axes/clocks or invent translation
truth. Reopen closed OutdoorNight reference listings or Gascola calibration gaps
only when a new source supplies relevant evidence. No new dataset download,
contact, model run, policy or deployment is part of this decision note.

## Evidence receipt and scope

The machine-readable table `results/baseline-recommendation/evidence-table.json`
retains all36learned combinations of three records, three variants and four
policies. Extraction verifies passed parent audits, immutable prediction/choice/
score-summary locks and216cross-checks of source denominators, availability and
angular totals against independently audited temporal summaries. The complete
parent artifacts retain SIFT, failed cases, all cohorts and source limitations.
No new accuracy, timing, geometry or learning outcome is produced by this synthesis.

Nominal reference/exposure/calibration and unknown checkpoint-training membership
remain qualified; no held-out, physical-calibration or new-method claim is added.
Prior SubT-MRS/OpenCV reading is reused; no fresh paper-body review. The preserved
rawE-audit failure/SVD amendment remains part of the parent provenance.
The verified incremental local packet is under
`/root/ore-ms-artifacts/baseline-recommendation-2026-11-17/`; no commit, push or Box upload.
