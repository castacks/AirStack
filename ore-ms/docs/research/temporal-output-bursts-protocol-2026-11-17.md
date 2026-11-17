# Fixed temporal missing-output and angular-error bursts

17 November 2026 America/New_York. Follows the
[OutdoorNight crop transfer](outdoornight-crop-findings-2026-11-16.md) and
[paired matched-count study](matched-count-findings-2026-11-16.md).
Does the fixed crop shorten the worst sampled unusable-output blocks despite
its aggregate gains and coverage losses? Descriptive exposed-cache study only.

## Fixed population and inputs

All541FlashLight,838SmokeRoom and971OutdoorNight anchors (2350total), original,
count,bottom, both frontends and four status policies (56400selected event rows).
Read immutable prediction/choice/error/summary/cohort locks and passed audits;
no fits, inference, new references or altered choices. Reuse nominal supplied
camera-reference errors for FlashLight/SmokeRoom; OutdoorNight stays a separate
nominal gyro-agreement tier. Own sensor endpoints and timestamps stay attached.
Preflight the full unique source populations, choices/status/error bindings,
reference support, and exact common camera anchor grid before temporal metrics.
Both cameras must share monotonically increasing500000000ns anchor steps;
otherwise stop rather than fit or impute time. No recording/seed/window selection.

## States, adjacency and boundaries

For each event, output presence is exactly the locked selected sensor. No output
is `missing`. For angular bounds1°/5°, use only the original BOTH-camera common
reference/gyro cohort: outside it state `unknown`, even if output is missing.
On common anchors, missing output is `missing`, error>bound is `disagree`, error
≤bound is `good`. No difference in thresholds from original score semantics.
Availability ignores reference support: `missing` or `good` on every anchor.

Analyze two views: full original anchor sequence, and original paired≥2° moving
cohort. For moving view, every nonmoving anchor is `outside` and terminates a run;
never concatenate sparse moving indices. Unknown anchors terminate angular runs.
Missing/disagree both count as unusable for angular runs and may mix within one
maximal block. Availability runs contain missing only. Keep every declared index.

Reuse `paired_failure_strata.runs` for maximal consecutive bad index groups;
new code supplies tier/variant/event adapters, states and gap/censor boundaries.
Each episode saves first/last original index/anchor_ns, length, missing/disagree
counts, preceding/following state or `record_boundary`, and sampled span in ns.
No interpolation or gap bridging. Left/right censored means adjacent state is
record boundary, unknown or outside; a neighboring good means observed onset or
return. Return lag exists ONLY if the immediately next eligible anchor is good,
and equals its anchor_ns minus first bad anchor_ns. Do not jump to later good
across a gap or extrapolate beyond the last anchor. A singleton spans0ns.

The sampled span is (k−1)*0.5s for kconsecutive bad decisions; conditional return
lag is k*0.5s where observed. Neither is continuous exposure coverage, original
frame-rate dropout duration, persistent VO recovery or time-to-relocalization.

## Endpoints and stop

Save full event states, all episodes and per-record/variant/frontend/policy/view/
endpoint summaries: assessed/unknown/outside/good/missing/disagree/bad counts,
episode counts, exact length histogram, longest length and sampled span (null if
no episodes), left/right censored counts and observed returns, conditional max
return lag (null if none). Also original→count,original→crop,count→crop deltas
for bad counts, episode counts and longest length, zero when no episodes for that
delta only. Aggregate marginal counts must reproduce locked parent summaries.

Primary: full learned RGB-first longest≤5°-unusable block and longest no-output
block for EACH record, compared to both controls. Strict1°, moving view, fixed
cameras, thermal-first and SIFT remain complete controls; no cross-record pooling,
IID significance, reference-tier equivalence or post-hoc camera/static rescue.

Independent audit reconstructs event binding from immutable inputs and segments
states sequentially without production run helpers; verifies every episode,
unknown/cohort/record boundary, censor flag, span/lag, histogram and paired delta.
Synthetic clean/singleton/mixed/gap/edge controls plus rehashed event, episode and
summary corruptions must fail, while complete unmodified data passes. Freeze
protocol/source/test/input hashes before computing temporal results.

Prior source/paper reading reused; no new paper-body reading or method novelty.
Stop after complete findings, figure, registry/handoff/index and member-verified
incremental local packet. No model execution, new fit, selector, crop/clock/seed
changes, training, Box upload, Git push or automatic commit follows this assay.
