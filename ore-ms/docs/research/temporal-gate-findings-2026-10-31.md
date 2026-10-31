# Temporal gate and history-rule evidence synthesis

2026-10-31 UTC; branch `ore_ms`, source commit `37c08b0a57fd09fdbf5e534ae15f7473a9d2634c`.

The fixed learned gate has no demonstrated consistent development improvement. The existing history rule avoids one known RGB recovery outlier, but its entire matched night translation improvement over raw fallback comes from that single event. Keep both as research controls; this evidence does not support adopting a new selector or increasing model capacity.

## What was compared

This synthesis uses the frozen [EXP091 comparison](../../experiments/EXP-091-temporal-gate-development.md): 274 decisions across three already-exposed recordings, with 161 complete-feature/both-supported fitting rows. The night early/middle windows belong to one recording. Combined remains the primary model; uncertainty-only, health-only and current-only remain controls. All 16 declared fits converged, and every family failed the original consistent-improvement diagnostic. No refit, new prediction, threshold, recording, reserve or replacement primary is introduced here.

Losses remain nominal-body constant-twist interval projections against recorded SLAM. The separate [EXP089 prefix check](../../experiments/EXP-089-prefix-model-pipeline.md) established replay-prefix behavior for the unchanged raw/history rules. Neither experiment established physical rig/reference timing, a verified checkpoint-training holdout or live operation.

## Match intervals before comparing error

History is available on 262/274 decisions, versus 268 for raw fallback and each gate. The table below uses exactly the same intervals for history, raw and combined within each recording; all six additional history abstentions are excluded from all three columns. Translation proxy RMSE is in metres.

| Recording | Matched n | Raw | History | Combined | History/raw changed choices |
|---|---:|---:|---:|---:|---:|
| Day 0 | 97 | 0.031505 | 0.031505 | 0.055852 | 0 |
| Day 1 | 51 | 0.035326 | 0.035326 | 0.035476 | 0 |
| Night | 114 | 0.102029 | 0.039978 | 0.102023 | 1 |

On the day recordings, history makes exactly the same choices as raw/RGB on every matched interval. Its larger full-population conditional RMSE on those days therefore reflects different availability, not worse matched choices. History drops one initial decision per window: one Day 0 decision, three Day 1 decisions and two night decisions. Their raw translation RMSEs are 0.026514, 0.012586 and 0.013902 m respectively; they are retained as explicit omissions, not assigned zero error.

The single night change is `EXP089/middle/96`: raw and all four gates choose RGB; history chooses thermal. RGB translation proxy loss is 1.003081 m, thermal 0.040678 m. Its signed squared-error gain is 1.004516683 m², exactly 100% of history’s net matched night translation improvement over raw. History versus combined has fifteen changed night choices; this same event supplies 100.0118% of the net gain, so the other changed choices collectively offset a small part of that benefit. No outlier was removed to obtain these statements.

## Common availability is a second descriptive view

The following intersection requires all eight methods to be available, making every column comparable within a recording. This is an availability-selected retrospective cohort, not a new test or a replacement for the original diagnostic. In particular, only 18/118 night decisions remain.

| Method | Day 0 (n=97) | Day 1 (n=51) | Night (n=18) |
|---|---:|---:|---:|
| rgb_only | 0.031505 | 0.035326 | 0.236628 |
| thermal_only | 0.080154 | 0.042954 | 0.013348 |
| raw_advice | 0.031505 | 0.035326 | 0.236628 |
| history | 0.031505 | 0.035326 | 0.013652 |
| combined | 0.055852 | 0.035476 | 0.236614 |
| uncertainty | 0.048640 | 0.035326 | 0.236623 |
| health | 0.048149 | 0.035326 | 0.236628 |
| current | 0.071681 | 0.035326 | 0.236610 |

These intersections contain 96/48/17 gate-eligible rows respectively. The remaining common rows use the original fallback. Thermal remains slightly better than history on the night intersection; history has no demonstrated quality gain over the appropriate fixed expert. This narrow cohort cannot replace full availability accounting. Both translation and rotation, all exact IDs, omission losses and all twenty-one history-versus-other per-recording comparisons are saved. Rotation was not a fit target.

## Evidence limits and decisions

| Question | Supported evidence | Consequence |
|---|---|---|
| Does the learned gate transfer consistently? | All four fixed families fail the declared development diagnostic; all records already informed design. | Preserve the negative result; no new default or larger-model claim. |
| Does history supply a general quality selector? | One changed raw/history choice explains the entire matched night gain; six extra abstentions. | Keep raw/history as explicit controls and report coverage with matched errors. |
| Are the motion outputs physically ready? | Existing projections use nominal frames and a stationary-twist assumption; recording-specific rig, acquisition clock and reference timing remain unverified. | Keep the existing motion-admission boundary; these scores do not authorize physical motion output. |
| Is the MACT checkpoint training split established? | Exact checkpoint bytes are known; recording membership/recipe reply remains unresolved. | A new training run could document its own split, but cannot resolve the rig/reference question. |
| Is an untouched test ready? | The [exposure ledger](recording-exposure-ledger-2026-10-31.md) marks all current Box TD3 recordings exposed; two MS2 reserves are reported but unassigned. | Do not equate recording exclusion here with a prospective or training-heldout test. |
| Are live deadlines validated? | Saved controller prefixes establish replay receipt ordering; worker clocks and sensor epochs are separate. | A real deployment still needs measured controller timing and a declared deadline. |

The [motion-output contract](arbiter-motion-output-2026-10-30.md) remains authoritative for admission: unverified frames/timing do not become verified because a proxy score improves. No default was changed and no reserve was opened.

## Next bounded step

Trace `EXP089/middle/96` and its two preceding native pairs through the saved health events, controller receipts, solver motion and interval projection. Bind the exact first received evidence and distinguish recovery/support history from model projection assumptions. Compare that trace with what the fixed gate actually observed. This uses existing artifacts and can proceed while the owner’s recording/training provenance reply is pending. It should explain the saved replay mechanism without claiming a physical failure cause, adding a heuristic, refitting or consuming a reserve.

Before any later prospective quality claim, independently establish the recording/reference/clock target and freeze a source-backed whole-recording assignment plus exact method artifacts. Before any live motion use, supply the verified frame/timing evidence required by admission.

## Audit and recovery

The synthesis freezes eleven source-file hashes and its script. A separate standard-library Decimal audit reconstructs losses directly from saved actions and expert labels, checks source matched summaries and squared-error gain identities, and verifies all cohorts and omissions. It performs 27,639 recursive value/schema checks and 84 source matched-RMSE checks across 21 comparisons and two metrics. Eight semantic corruptions are rejected; JSON key reordering passes. These are arithmetic checks, not independent scientific observations. No new unit suite was needed for this analysis/documentation step.

Artifacts: `results/temporal-gate-synthesis/run-001/{summarize.py,audit.py,protocol.json,summary.json,validation.json}`. Scripts ran in CPU Docker image `sha256:af124653ef9cd76660bb8a91f5ae609d6488d41fc78269fe03b3f484c5d35fd1` against the stated source commit. Summary audit: `PASS_DECIMAL_SYNTHESIS_AUDIT`.

Recovery: `ore_ms:ore-ms/backups/2026-10-31-temporal-gate-synthesis`, incremental Git history based on `37c08b0a5`, bounded source metadata, analysis/audit scripts, outputs and local lab records. No scientific images, checkpoints, credentials or runtime image binary are included.
