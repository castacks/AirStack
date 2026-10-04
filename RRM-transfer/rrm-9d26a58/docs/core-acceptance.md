# Core acceptance campaign

This campaign qualifies **mock component behavior and the measurement instrument**,
not integrated SIL, learned reasoning, perception, physical stopping, or comparative
robot performance. It does not expand RRM-EM or change live-motion authority.

After the standard CPU bootstrap, export to a new directory:

```bash
# From the RRM directory; use .venv/bin/python on a venv-capable host.
PYTHONPATH=.rrm-deps python3 scripts/core_acceptance.py --output /tmp/rrm-acceptance-run
PYTHONPATH=.rrm-deps python3 scripts/core_acceptance.py --output /tmp/rrm-acceptance-run --verify
```

`--case` selects named cases; `--repetitions 2` repeats the frozen matrix. Repetitions
exercise scheduling stability, not distinct stochastic seeds or confidence intervals.
`--attempt-timeout` controls the outer harness deadline (default 5 seconds); fixture
callback limits are independently declared at 0.1 seconds. These are test bounds,
not calibrated deployment limits or physical stop-response measurements.

Campaigns produced by current source declare `core-call-limits/v3`. Each bounded
event has a unique write identity, and a trace-write fault carries an append receipt.
A record flushed before its callback deadline is observed can remain replayable only
when it is adjacent, identity-bound and valid for that fault phase. Pre-commit,
missing, reordered, wrong-phase, rejecting-gate and duplicate-identity cases remain
incomplete. Earlier v2 bundles retain their original source-pinned reader; current
replay intentionally does not reinterpret them.

In the retained v3 regression, two 34-attempt runs each had 33 expectation matches
and 31 complete attempts. One encountered a pre-dispatch write deadline; the other
successfully bound a late numeric record but lost later interruption evidence. Both
remain unqualified and UNKNOWN. Thus v3 establishes strict reconstruction semantics,
not deadline-free callback/sidecar scheduling, and no 0.1-second fixture bound was
widened to obtain a favorable rate.

## Frozen cases

| Family | Cases |
| --- | --- |
| Nominal | Pick, place |
| Refusal and effects | Unsafe pre-dispatch context; transient and persistent grasp failure |
| Active change | Human entry; displaced target |
| State evidence | Weak confidence; stale relation coverage; observation loss |
| Execution | First-chunk numeric rejection; stalled policy; application exception |
| Proposal liveness | Initial and recovery planning stalls |
| Unconfirmed/incomplete | Missing cancel acknowledgement; permanent trace-write loss |

These are not integrated S01–S10 trials. Scenario-specific assets/coordinates and
limits live in mock fixtures/profile evidence; the core loop has no campaign-ID or
scene-name branches. One mock adapter is not cross-embodiment validation.

The 2026-10-03 sidecar-finalization increment removes per-gate disk synchronization
from safety-label callbacks. Each label is still written and flushed in event order;
the sidecar is synchronized once at attempt close, before `result.json` is published.
Synchronization errors propagate, and both files are closed even on error. A stalled
finalization remains subject to the outer harness deadline; missing worker results
and failed harness exits remain unqualified and UNKNOWN, even with a valid trace.
The trace append receipt establishes an in-process flushed trace record, not a
durable safety-label commit or a power-loss guarantee for the entire bundle.

A retained pre-change 34-attempt baseline matched 33 expectations and qualified 31
attempts. Two subsequent exports with the finalization change each matched 34/34 and
qualified 32/34; their two intentional trace-loss attempts remain UNKNOWN. These
observations qualify the changed finalization behavior under this host's scheduling,
not the elimination of all write deadlines. Per-event thread scheduling, flush stalls,
and final storage synchronization remain possible failure modes. The baseline uses
its retained original reader and is not compared as a distinct architecture arm.

## Distinct outcome measures

- `verified_goal_rate`: replay-grounded `goal_met=true` over **all attempts**.
- `legacy_task_success_rate`: existing core metric, including expected-abort success;
  not synonymous with reaching the goal.
- `expectation_match_rate`: observed behavior matches a pre-authored fixture expectation.
- `replay_qualified_acceptance_rate`: expectation match **and** complete replayable evidence.
- `evidence_complete_rate`: valid replay, complete canonical worker result, trace/config
  binding, qualified event labels and successful harness exit. Missing evidence
  remains in the denominator.

The intentional trace-write-loss case can match its error expectation while remaining
evidence-incomplete, `goal_met=null`, and `stop_status=UNKNOWN` in campaign reporting.
Its retained pre-failure prefix is checked for framing/order and selected joins;
it is not a reconstructed complete stop chain or a qualified acceptance pass.
Similarly, cancelling without acknowledgement remains `SAFE_UNCONFIRMED`, never
promoted to safe because the campaign expected that condition.

## Event-scoped safety adjudication (T)

Campaign schema `rrm-core-acceptance/v2` freezes authored mock classification schedules
before execution and requires a separate `safety-labels.jsonl` for every attempt,
including an empty sidecar when planning fails before any safety decision. Labels
are selected by gate and simulation tick, never copied from verifier verdicts.
Ordered one-to-one joins bind event sequence, run, action/digest, plan version, state
digest and, for active decisions, dispatch/cycle. Missing, extra, reordered or forged
labels disqualify the attempt even if artifact hashes have been recomputed.

`safety_adjudication` separates symbolic `safety1`, active dynamic-symbolic gates,
and numeric `safety2`. UNSAFE is positive; FAIL is a detected unsafe condition.
Each stage reports TP/FP/TN/FN and rate denominators. Undefined rates are null.
Authored UNKNOWN decisions are counted explicitly and excluded from confusion
denominators; displacement does not imply an invented numeric classification.
Only complete qualified attempts contribute classifier counts. Unqualified prefix
decisions, missing/invalid labels, unavailable-label attempts and complete attempts
with no decisions are exposed separately alongside all-attempt label coverage.

Episode-wide labels remain for core replay compatibility, not dynamic confusion
scoring. These event labels are independently specified from verifier decisions,
but generated in the same fixture process: they are not external or blinded
adjudication. Deterministic mock rates establish measurement correctness, not
general robot-safety performance. No physical unsafe-dispatch rate is published.
This is T telemetry/evaluation supporting S safety supervision and M execution
monitoring; no control, reasoner, policy, adapter or RRM-EM authority is changed.

## Attempt lifecycle and integrity

The matrix, expectations, ground-truth schedules, attempt IDs, call limits and numeric/capability profiles
are frozen in `campaign.json` before execution. Each attempt has a fsynced STARTED
ledger record before launch and a FINISHED record referencing its assessment.
Workers run in separate processes. A harness timeout terminates the direct mock
worker and reports containment; it proves neither physical stop nor descendant
process-group cleanup. No fixture launches descendant processes or hardware.

Worker crashes, launch errors, timeouts, missing/corrupt results and invalid replay
remain attempted, unqualified outcomes; the next fixture is still evaluated.
Parent-level interruption can leave an unfinished ledger and no sealed manifest:
offline verification then fails, and the partial directory must be preserved.
Fresh output directories are mandatory; no overwrite or best-of-repeat selection.

Each bundle contains:

- `campaign.json`, `attempts.jsonl`, `summary.json`, `manifest.json`;
- per-attempt `events.jsonl` (possibly partial/missing), `result.json` when available,
  `safety-labels.jsonl`, `harness.json`, `assessment.json`, and `worker.log`.

Offline verification checks exact artifact inventory/hashes, authored matrix,
attempt ledger identity/order, declaration/config binding, worker metrics and
independent replay/scoring. Manifest pins initial/final executable source digests
and runner digest. A different current reader is rejected; retain the historical
source revision to assess an earlier bundle. Git identity/dirty state are provenance,
not proof of which executable bytes ran.

`verify --valid=true` means an intact, reconstructible bundle, **not** all expectations
passed. A faithfully recorded crashed campaign can be integrity-valid and acceptance-
failed. Hashes catch corruption; they are not signatures or protection against an
attacker rewriting all artifacts and provenance. A later integrated campaign needs
qualified adapters, independent outcome labels and the seeded protocol in
[SCRUM-8 evaluation](scrum-8/evaluation.md).

For repeatability across two distinct exports with an identical frozen schedule,
use the [paired campaign report](core-comparison.md). It retains every attempted
pair, unknown outcomes and separate evidence/safety denominators. Canonical JSON
reconstruction rejects boolean/integer substitutions in ledger and count evidence.
