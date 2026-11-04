# Research position: uncertainty and bounded RGB–thermal arbitration

2026-11-03, branch ore_ms. Original scope remains RGB–thermal geometric uncertainty.

The original research question asks whether learned RGB and thermal uncertainty
is comparable and can identify which camera supplies more reliable geometry.
Earlier experiments support useful within-camera ranking on inspected data, but
cross-camera transfer and simple calibration have not established a general
selector. See the [fall assessment](ms2-fall-decision-2026-10-28.md) and
[transfer findings](ms2-transfer-findings-2026-10-27.md). Successful engineering
checks below do not turn that unresolved premise into a demonstrated result.

The current branch studies a narrower prospective method: estimator-health and
history-based arbitration with sparse inactive-camera probes, explicit
availability and output-readiness semantics. Its research question is whether
useful native motion can be maintained while avoiding unnecessary expert calls,
including through an availability transition. This is a method feasibility and
resource-accounting branch, not yet a calibrated-uncertainty fusion result.

| Evidence | What it establishes | What remains open |
|---|---|---|
| [EXP119](../../experiments/EXP-119-closed-loop-sparse.md) | Actual own-feedback skipping reduces calls 434 to 334 with 204/205 guarded deliveries on the exposed windows | Active camera stays fixed within each window; no adaptive superiority |
| [EXP120](../../experiments/EXP-120-availability-transitions.md) | Explicit synthetic outages can make switching useful: 85/95 vs 71/95 fixed-camera deliveries | Natural failures, silent-fault detection, independent conditions |
| [EXP121](../../experiments/EXP-121-readiness-continuation.md) | Output-ready probing can restore inactive outputs but lose a handover interval | Fresh health alone does not ensure contiguous native inputs |
| [EXP122](../../experiments/EXP-122-buffered-handover.md) | One real retained input repairs the observed continuity break; live synthetic candidate gives 85/95 with 141 vs 158 full-read calls | Real acquisition deadlines, bounded worker ownership and broader replication |
| [EXP123](../../experiments/EXP-123-retention-capacity.md) | Storage capacity affects whether that exact predecessor is available: 2 MiB misses, 4/8 MiB hit in fixed traces | Native behavior with worker leases and bounded control transport |
| [EXP124](../../experiments/EXP-124-leased-handover.md) | 4 MiB store + 2 MiB shared arena preserves 85/95 deliveries and all four returns, 141 vs 158 calls; 2 MiB diagnostic drops to 84/95 | Total process/GPU memory, acquisition cadence, natural recovery and physical accuracy remain separate |

The immediate contribution being tested is a causal, resource-accounted
scheduling and handover mechanism. A positive bounded replay is evidence that the
mechanism can execute under those stated limits. It is not evidence of improved
pose accuracy, calibrated probability, independent generalization, or universal
advantage over the best fixed camera. Byte-exact controls and independent audits
support implementation validity; they are not independent scientific replications.

After this mechanism boundary, the next research evidence should include a
separately predeclared condition/recording, simple fixed-camera and matched-cost
controls, and justified reference/deadline semantics. More engineering iterations
on the same synthetic day interval cannot establish that broader claim. Existing
exposed inputs suffice for bounded software checks; sustained natural returns with
a useful alternate and independent pose references would strengthen the study.

EXP124 makes the tradeoff explicit: primary restores4/4 returns versus original
sparse1/4, at141 versus133 calls and the same85/95 selected-delivery count. Full
read uses158 calls. This is readiness at measured cost, not evidence of selected
delivery or accuracy superiority over the original sparse baseline. The2MiB
native diagnostic loses the handover frame and drops to84/95. Serial controller
processing is51.592 seconds for a9.900-second source span; live cadence is not
established. Next research tests should address independent-condition utility
with fixed-camera and matched-cost controls rather than treating more checks on
this synthetic interval as generalization evidence.
