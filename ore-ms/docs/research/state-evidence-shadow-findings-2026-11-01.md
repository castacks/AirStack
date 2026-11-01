# State-evidence shadow boundary: implementation and coverage

2026-11-01 America/New_York; branch `ore_ms`. Semantics declared at `703f305b8`
before implementation; tested implementation `80fc3a4fb` frozen before replay.

A separate opt-in shadow boundary intersects existing raw availability with
predecessor-state evidence for the same exact received child. On the four saved
day/night control/witness captures, it changes the known night RGB recovery to
thermal fallback. Witnessed initialization avoids the history rule's extra
cold-start abstention. Missing control roots instead incur one initial
abstention per run. No conditional errors or quality improvement were measured.

## API and fixed semantics

`Scripts/Setup/state_evidence_shadow.py::state_evidence_advisory` takes native
input rows/pair, scoped per-camera controller envelopes, original deadline and
explicit root envelopes. The function validates both journal namespaces,
contiguous worker identities, exact integer ordinals/receipts, unique shared
controller ordinals and receipt ordering. Invalid global context raises
ValueError and emits no advice; it does not silently truncate late inputs or
fall back around malformed journal context.

It computes unchanged raw availability internally. For each raw-available
camera it binds the unique exact first-write child and calls the unchanged
predecessor validator on that prefix/deadline. Cached or caller-supplied pass
results are not accepted. Eligibility requires both raw availability and
`PASS_PREDECESSOR_EVIDENCE`. Root envelopes are explicitly supplied by the replay
caller from the original same-camera prefix; the function never discovers or
backfills them. Invalid/missing state evidence excludes that expert, while a raw
unavailable camera is `NOT_CHECKED/raw_unavailable` and cannot be promoted.

Fixed RGB preference when both qualify, sole eligible expert otherwise, neither
qualifies => ABSTAIN. No history warm-up, threshold, fitted quality preference,
pose repair or motion transform. Diagnostics retain both experts, raw action,
eligibility and exact predecessor source identities. Output is always
shadow-only, `physical_motion_admitted=False`, `reference_or_loss_used=False`
and `production_defaults_changed=False`. Scope/clock/documentation labels remain
caller attestations, not authenticated physical provenance.

See the [predeclared design](state-evidence-shadow-design-2026-11-01.md).
Underlying estimator, raw/history, predecessor validator, physical admission,
exact-chain and learned-gate sources remain unchanged. This API is not wired
into either estimator or the live controller.

## Frozen replay and controls

Input population: original EXP092 day19decisions and EXP093 middle-night99decisions,
both flag-off control and flag-on witness runs:236decisions/472camera slots.
Repeated executions of two already-exposed windows are not four independent
scientific conditions. The design used prior knowledge of the failure; no
prospective/held-out evaluation is claimed.

Before replay, protocol/declaration lock614input, source, proof and script hashes,
including complete prior audited predecessor bindings and original controller
journals. Each original deadline is replayed once with unchanged raw and
symmetric two-interval history controls on that same prefix, alongside the new
boundary. Original advice/availability are preserved and checked. No reference
labels, proxy errors, image decoding, neural inference, training or reserves.

| Case | Decisions | Raw available | History available | Shadow available | Shadow actions |
|---|---:|---:|---:|---:|---|
| Day control |19|18|17|17|17RGB /2ABSTAIN|
| Day witness |19|18|17|18|18RGB /1ABSTAIN|
| Night control |99|98|97|97|97THERMAL /2ABSTAIN|
| Night witness |99|98|97|98|98THERMAL /1ABSTAIN|
| Total |236|232|228|230|35RGB /195THERMAL /6ABSTAIN|

The boundary changes exactly four raw decisions: day-control pair0RGB→ABSTAIN,
night-control pair0THERMAL→ABSTAIN, and night pair96RGB→THERMAL in each arm.
The two initial abstentions arise from missing documented initialization,
not measured pose error. Original raw advice is retained separately and never
rewritten.

Relative to history, control actions are identical. In each witness capture,
only pair0differs: the boundary accepts documented initialization while the
history control abstains for its fixed warm-up. Night pair96matches history's
thermal choice. New witness coverage therefore equals raw in these two runs;
this is conditional on actual root documentation, not automatic initialization.

Per-camera evidence totals:262ordinary passes,3documented-root passes,
3missing-root rejections,2unsupported-parent rejections and202raw-unavailable
untested slots. All472slots remain;265camera slots qualify for shadow eligibility.
No missing evidence was fabricated. The two unsupported-parent rejections are
the same known recovery in repeated control/witness executions, not independent
new failures. All physical admission flags remain false.

## Tests and independent audit

231CPU Docker tests pass:32new meaningful boundary cases plus199predecessor,
root/recorder, raw/history/shared, pairing and admission/chain regressions.
Fixtures cover the complete eligibility truth table, valid/missing/late/invalid
root fallback, raw-unavailable nonpromotion, unsupported nonroot with a supplied
root, exact child/parent binding, namespace/clock/deadline/cross-camera journal
conflicts, large integer timestamps, immutable inputs and no function file reads.

The first synthetic test attempt lacked the recorder's
`derived_delta_unavailable_reason` field and correctly failed schema handling
(16failed/215passed). The fixture was completed; no boundary rule was loosened.
Final:231passed in3.14s. Frozen source/test hashes bind this successful run.

The independent auditor imports no new boundary/replay, raw/history code or
predecessor validator. It rescans source deadlines and journals, independently
reconstructs raw and history availability/actions and combines them with prior
independently audited same-child predecessor bindings. Prior proofs, source
hashes, native intervals, receipts, scope and child identities are rechecked.
All output fields and complete case/population summaries match across16,554
recursive type/value/schema checks.

Eighteen semantic corruptions reject promoted rejected/unavailable experts,
bypassed fallback, fabricated root pass, wrong child/scope/deadline, changed
raw/original/history advice, false physical/production/label flags, omitted case/
decision, wrong count, duplicate sample and truthy integer eligibility.
Key reordering passes; all614protected hashes remain unchanged. Final status:
`PASS_INDEPENDENT_STATE_EVIDENCE_SHADOW`. These are bookkeeping and policy-contract
checks, not independent scientific observations or verified physical reliability.

## Interpretation, recovery and next step

The boundary expresses a state-evidence requirement without fitting a quality
model. This replay shows its exact decisions and coverage tradeoff. It does not
show lower errors, general robustness, calibrated confidence or a justified
physical pose. Missing initialization can reduce coverage without implying an
inaccurate pose. Root documentation does not override an unsupported later
parent. Existing production defaults remain unchanged.

Artifacts: `results/state-evidence-shadow/run-001/` contains preserved declaration,
614-file protocol, all236raw/history/shadow comparisons and complete evidence,
case/aggregate summaries and independent mutation validation. Recovery includes
the frozen standalone replay/auditor, test attempts/results, original metadata
and source bindings. CPU event-test image6f8c1a104929was used; no GPU inference.

Next, separately freeze a matched-support conditional comparison of these locked
shadow/raw/history predictions against the existing nominal recorded-SLAM proxy,
retaining all abstentions and translation/rotation losses. Treat it as a known-
development-window diagnostic; do not tune or claim transfer from the known
pair96change. Physical/reference/clock and checkpoint-training provenance remain
separate unresolved inputs. No such scoring was executed in this step.

Recovery: `ore_ms:ore-ms/backups/2026-11-01-state-evidence-shadow`.
