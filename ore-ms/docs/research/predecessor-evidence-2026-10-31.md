# Predecessor evidence across frozen controller prefixes

2026-10-31 UTC; branch `ore_ms`; source commit `1f59252641da1268d8484bb62c1374bfc1067c13`.

Of 439 currently available expert motion candidates, 427 have supported predecessor pair-end health, eleven have no predecessor health because they reference graph frame zero, and one has unsupported predecessor health: the known night RGB recovery. All 427 supported cases exactly match an already observed parent committed pose. The evidence distinguishes root initialization from unsupported recovery; it does not justify treating missing health as a failure or adopting a rejection rule.

This implements the bounded inventory proposed by the [saved recovery trace](recovery-event-trace-2026-10-31.md). It retains all 274 decisions / 548 camera slots in the six existing EXP089/EXP090 windows, grouped into three already-exposed recordings. No labels or proxy-error files were read. No new fit, prediction, recording, reserve, selector or production default is introduced.

## Binding and classification

For each original decision and camera, the inventory finds the unique first motion write ending at the requested native endpoint and checks both exact endpoints. It records the captured parent camera-map pose, graph parent index and native parent timestamp. Graph identities are scoped to the window and camera; repeated frame indices across windows are not shared states.

Write-time parent health must have arrived on the controller journal before the child motion write, with a matching new-frame index and parent timestamp. Decision-time status is separately reconstructed before the original decision. A prior committed parent pose must likewise have been observed before the child write, refer to the same index/time, and match the captured parent pose exactly. Worker clocks are not subtracted from controller receipts.

| Parent-health category | Meaning |
|---|---|
| SUPPORTED_HEALTH | A unique observed parent pair-end has matching frame/time, original need_interp=false and no execution error. |
| UNSUPPORTED_HEALTH | Matching parent health explicitly requires interpolation or reports an execution error. |
| ABSENT_HEALTH | No parent pair-end snapshot was observed by the relevant cutoff. |
| PARTIAL_HEALTH | Parent support/error fields are insufficient to classify. |
| PARENT_TIME_MISMATCH | A frame-index match has a different parent timestamp. |
| AMBIGUOUS_HEALTH | Multiple parent pair-end snapshots claim the same scoped graph index. |

These are descriptions of the recorded pair-end evidence. “Supported” is not optimizer convergence, physical frame verification, pose covariance, or an admitted motion. A graph-root marker is recorded separately; it never converts absent health into supported health.

## Complete inventory

| Recording | Camera slots | Current exact-motion candidates | Supported parent health | Absent parent health / roots | Unsupported parent health | No first write |
|---|---:|---:|---:|---:|---:|---:|
| Day 0 | 198 | 196 | 194 | 2 | 0 | 2 |
| Day 1 | 114 | 108 | 102 | 6 | 0 | 6 |
| Night | 236 | 135 | 131 | 3 | 1 | 101 |
| Total | 548 | 439 | 427 | 11 | 1 | 109 |

Every unique exact first-write candidate is currently available in these saved prefixes and has a valid existing interval-model projection. The remaining 109 slots have no first write by the decision. There are no interval-mismatched or ambiguous candidates here. Partial, timestamp-mismatched and ambiguous predecessor-health counts are also zero. Such absent empirical cases are not silently treated as verified behavior.

For all 439 candidates, write-time and decision-time parent-health categories agree. For the 427 supported-parent candidates, a prior parent write exists and its committed pose matches the child’s captured parent pose exactly. None of these comparisons requires interpolating a later pose or selecting a result after the cutoff.

The twelve candidates without a prior observed parent write divide into eleven frame-zero roots and the single unsupported recovery. The eleven roots are the first pair of their window/camera: two in Day 0, six in the three Day 1 windows, and three across the two night windows. Their timestamps also match their window’s first native row. There is no pair-end support snapshot for those roots in the journal. That is an explicit initialization-evidence case, not proof that the root pose is physically verified or unsupported.

Only `EXP089/middle/96/rgb` has unsupported non-root predecessor health. Its graph parent 96 has last observed interpolation-required health and no earlier parent write; its first-write parent snapshot is identity. This agrees with the saved trace. This inventory used no loss values to select that category or count the other cases.

## Consequences for a separate state contract

The available logs can bind a supported predecessor to exact frame/time, receipt order and a previously observed committed pose for 427 candidates. They also identify the unsupported recovery separately. They cannot supply a missing root pair-end event retroactively.

A future opt-in predecessor-state validator should keep supported-parent evidence, explicit root-initialization evidence and unsupported/absent/partial evidence distinct. If root initialization is accepted, it needs a documented initialization contract backed by evidence; graph index zero alone is not that evidence. Simply requiring an ordinary prior pair-end for every candidate would exclude all eleven root candidates as well as the known recovery. That would be a policy decision, not a conclusion of this accounting.

No new rejection rule was simulated or adopted. Current availability, the history rule and all gate outputs remain exactly as saved. The [existing motion-admission boundary](arbiter-motion-output-2026-10-30.md) still requires its separate frame/clock/deadline evidence; predecessor bookkeeping cannot replace it. Nor do these counts establish improved accuracy, a physical failure cause or transfer to another recording.

The next bounded implementation can express this as a separate opt-in evidence validator with an explicit root case and meaningful fixtures, leaving existing defaults unchanged. Acceptance semantics must be specified before any retrospective quality comparison. Physical rig/reference provenance, checkpoint training membership and measured live timing remain independent unresolved inputs.

## Validation and reproduction

A separate standard-library auditor rescans the original timelines at each decision rather than importing the inventory implementation. It reconstructs availability, exact candidate identities, both receipt cutoffs, parent health/frame/time bindings, prior writes, pose equality, every slot and every aggregate. It passes 89,099 recursive schema/value checks over all 548 slots. Ten semantic mutations are rejected, including late evidence, unsupported/root relabeling, graph-index/time cross-binding, pose changes, hidden prior writes, lost slots/recordings and altered counts. JSON key reordering passes.

Eight directed fixtures exercise cases absent from the real logs: supported/unsupported health, future support excluded at write but visible at a later decision, equal-sequence exclusion, parent timestamp mismatch, partial health and ambiguity. The fixture results pass. The initial mutation harness exposed a missing dictionary-type assertion; that audit assertion was corrected and the inventory remained unchanged. Attempts and final checks are preserved. No unrelated unit suite was repeated for this analysis/documentation step.

Artifacts: `results/predecessor-evidence/run-001/{inventory.py,audit.py,protocol.json,inventory.json,validation.json}`. Source hashes bind all input metadata and the inventory script. CPU Docker image: `sha256:af124653ef9cd76660bb8a91f5ae609d6488d41fc78269fe03b3f484c5d35fd1`; scripts use only the standard library. Audit status: `PASS_INDEPENDENT_PREDECESSOR_INVENTORY`. Reproduction uses the saved source layout at `/root/AirStack/ore-ms`; writes are exclusive, so preserve previous outputs before rerunning.

Recovery: `ore_ms:ore-ms/backups/2026-10-31-predecessor-evidence`, incremental history based on `1f5925264`, all bounded metadata inputs, scripts, inventory, audit, directed fixtures and lab records. No images, checkpoints, credentials or runtime image binary are included.
