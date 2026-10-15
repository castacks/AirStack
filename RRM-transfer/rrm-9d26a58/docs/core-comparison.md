# Paired core campaign comparison

## Independent intent development probe

`scripts/independent_intent_probe.py` queries the existing private Cosmos worker
with authored semantic C02 facts and a constant neutral image (transport only).
This tests semantic output contracts, not visual perception, physical execution or
general language understanding. The independent restricted resolver receives no
model output; the worker receives no binding, label or resolved reference target.

The corrected opaque-ID corpus has 12 scheduled development cases. One transport
URLError stopped the run after six responses; five remaining cases were NOT_RUN.
No retry or response repair was performed. On responded cases only:

| Authored intent | Scheduled | Responded | Valid-contract intent matches |
|---|---:|---:|---:|
| READY | 4 | 4 | 2 |
| NEEDS_CLARIFICATION | 7 | 2 | 1 |
| UNSUPPORTED | 1 | 0 | Not evaluated |

Overall this is 3/6 matches among responses, **not** 3/12 accuracy. All four READY
responses declared the correct target; two omitted required grounded-entity lists
and were held rather than repaired. Two proposals passed the independent guard.
On duplicate blue markers the model arbitrarily chose one despite a valid local
schema; the independent resolver withheld a binding/plan and requested clarification.
The other responded ambiguity case correctly requested clarification.

Reviewer discovery: the initial 12-response run exposed case-name cues through task
and snapshot IDs. Its 4/12 matches are retained separately as exploratory only, with
the original source snapshot. The corrected run uses opaque IDs and is primary;
neither run is held-out, and their results must not be pooled. Reference unit tests
match all 12 authored labels, which is not evidence of 12 model responses.

Local source hashes and before/after worker capability records stayed unchanged.
The endpoint does not attest checkpoint, semantic prompt or generation settings;
stable capabilities/source hashes alone do not establish those identities. The
underlying transport-error cause was not retained and remains unknown. No runtime
activation, robot dispatch or permission change occurred. See
`.rrm-artifacts/independent-intent-grounding-20261015/` for raw evidence.

For a separately approved new probe, from the RRM root with the configured runtime:

```bash
python scripts/independent_intent_probe.py --worker-url "$RRM_COSMOS_WORKER_URL" --output /tmp/rrm-intent-new
python -m unittest discover -s tests -p test_intent_resolution.py -v
```

Use a fresh output directory and an existing private worker; no public service port
or deployment is required. The probe stops issuing requests after the first transport
or identity error. Broad integration: 624 tests, 623 pass, one independently reproduced
pre-existing LAND-fixture error (missing `armed` state); no full documentation build
was performed for these existing-page updates.

### Approved missing-case continuation

`scripts/continue_intent_probe.py` verifies the saved opaque run's artifact manifest,
inference-source hashes, corpus, requests and recomputed assessments. It selects only
ERROR/NOT_RUN cases, never repeats a responded case, preserves task/context/image/step
and assigns fresh cycle IDs. Three bounded health/capability pairs must match the
prior proposal-only contract. It stops on the first RPC/identity error and retains
safe structured transport-cause metadata. Manifest hashes detect drift; they do not
authenticate an issuer. Neither script grants permission or dispatches actions.

The approved continuation returned all six selected cases with no new errors:

| Missing case | Raw declared status | Independent outcome |
|---|---|---|
| Conflicting color | READY | NEEDS_CLARIFICATION |
| Stale target | READY | NEEDS_CLARIFICATION |
| Absent blue marker | NEEDS_CLARIFICATION | NEEDS_CLARIFICATION |
| Unsupported inspection | READY | UNSUPPORTED |
| Unknown qualifier | READY | NEEDS_CLARIFICATION |
| Unknown competitor | READY | NEEDS_CLARIFICATION |

Only absent-blue matched the authored intent contract: 1/6, including 1/5
clarification cases and 0/1 unsupported cases. Two raw READY outputs were locally
schema-accepted (inspection and unknown competitor); neither acquired a binding or
plan. The other three READY outputs omitted required grounded_entities. All six
were withheld, not repaired into reference plans. Existing six responses were not
resampled; prior ERROR delivery to inference remains unknown.

Descriptive cross-session coverage is now 12/12 unique cases with 4/12 valid-contract
matches (READY 2/4, clarification 2/7, unsupported 0/1), two guarded proposals, two
clear-request contract failures held, seven clarification holds and one unsupported
hold. This joins unchanged prior responses with the approved continuation; it is
not an uninterrupted run, held-out estimate or controlled performance comparison.
Do not include the earlier case-cued exploratory run in these counts.

Exposed capabilities matched before/after both sessions. The reported entrypoint hash
matches historical commit `8037a9954`, not the current worker source. This identifies
that entrypoint only, not imported semantic prompt/parser/generator modules, loaded
checkpoint bytes or settings. Local workflow pins describe a future launch, not
proof of this process. No restart/deployment was performed; prompt/checkpoint/settings
remain unattested. Stronger provenance requires a separately approved source-matched
worker rollout, followed by a newly frozen development evaluation before held-out work.

From the RRM root, with a retained prior opaque output and a fresh output directory:

```bash
python scripts/continue_intent_probe.py --previous /path/to/prior-opaque-probe --output /tmp/rrm-intent-continuation-new --worker-url "$RRM_COSMOS_WORKER_URL"
python -m unittest discover -s tests -p 'test_intent*.py' -v
```

Each continuation requires an explicit approved protocol; the CLI is not a retry
daemon. Source drift or changed capabilities prevent inference. Evidence is retained
in `.rrm-artifacts/intent-probe-continuation-20261015/` including its prior-run inputs.
Six new continuation tests and all 22 existing intent tests pass. Final isolated
suite: 630 tests, 629 pass, one unchanged LAND-fixture missing-`armed` error. One
independent reviewer verified raw hashes, selection/input parity and recomputed scores.
Existing documentation pages were updated; no full MkDocs build is claimed.

## Matched-tape shadow checking campaign

`scripts/shadow_robustness_campaign.py` is a separate, non-dispatching qualification
instrument for sensor-neutral reasoning checks. It audits 24 authored development
cases / 27 proposal-feedback steps under five checking variants. Each variant gets
the exact same semantic observations, raw proposals, backend feedback, requested
task effect and bounded replay schedule. Labels are independently authored fixture
records, excluded from each variant's decision inputs. No model, ROS, physical
backend or live safety authority is invoked.

The two axes are current C02-resolved identity eligibility versus its legacy
raw-positive ablation, and resolved task-effect evidence versus a backend-status
completion ablation. Syntax, entity/verb vocabulary checks and common synthetic
stop, permission and deadline filters remain in every arm. The legacy parser uses
a private function-global namespace; production parser globals are unchanged.
These are checking ablations, **not** a full RRM implementation versus a capable
closed-loop baseline. In particular, the status-only variant is not the sole
baseline for a future architectural claim.

A fifth variant applies the caller-bound task intent consistency guard. The same
explicit binding input is given to every variant, authored from the tape's existing
requested task effect, not from labels or model output. Only the fifth consumes it;
the original four remain unbound controls. This adds a guard, not extra oracle
observations or a learned interpretation algorithm. Report schema is v2; the v1
corpus hash and its original cases/proposals/labels remain unchanged. Earlier reports
need their pinned historical source reader, not the current v2 implementation.

From the RRM directory, using an isolated Python with existing requirements:

```bash
python scripts/shadow_robustness_campaign.py --output /tmp/rrm-shadow-new
python scripts/shadow_robustness_campaign.py --output /tmp/rrm-shadow-new --verify
```

Output must not exist. The bundle holds `corpus.json`, `report.json`, `run.json`
and `manifest.json`. Verification requires the current pinned corpus and executable
source, checks all artifact hashes, and independently reconstructs every arm/step
and count. Modified case selection, identities, budgets, labels, reports or source
are rejected; no chosen intersection is compared. Hashes are corruption detection,
not signatures or evidence that authored labels are physically correct. Runtime
records Python/Pydantic versions and instrument duration, not model/robot latency.

The frozen development census produces:

| Checking variant | Invalid proposal acceptances / 11 | False completion claims / 15 | Unsupported completion claims / 6 |
| --- | ---: | ---: | ---: |
| Context + effect |1|0|0|
| Raw identity ablation + effect |5|0|0|
| Context + status effect ablation |1|7|1|
| Both ablated |5|11|1|
| Bound intent + context + effect |0|0|0|

The denominators are independently labeled invalid proposals, false actual goals
and unknown actual goals, respectively; unknown labels are not relabeled failures.
All five have zero refusals among 15 labeled valid proposal steps. There is one
unknown proposal label, eight UNKNOWN observed effect resolutions, and every step
stays in the report. Context/effect variants claim all six labeled true goals;
status-only effect variants claim five, missing an observed goal despite backend
failure. A completion claim is not actual goal achievement and implies no actuation.

The residual invalid acceptance is an intentional wrong-valid-target challenge:
the contract parser accepts a grounded but task-inappropriate entity. Resolving the
requested task effect, rather than trusting the model's different goal, avoids a
false completion claim but does not repair intent alignment. The fifth bound variant
rejects this proposal; all four unbound controls retain it. Correct binding production
from qualitative language remains open. This is a development regression after
observing the failure, not a held-out performance gain or general intent proof.

The tape continues identically after refusals/claims and supplies the same two-step
recovery examples to every variant. Therefore nonclaim→claim transitions measure
only replay behavior, not causal recovery, a closed-loop counterfactual or skill
performance. The backend identity denotes a frozen feedback tape, not an executed
qualified controller. There are no images, depth maps, real scene trials, learned
proposals, held-out results, statistical uncertainty intervals or architecture
advantage claims. Qualification of a real matched-backend learned campaign remains
the next boundary; do not substitute these deterministic counts for that experiment.

## Same-implementation core campaign instrument

`scripts/core_comparison.py` implements T telemetry/evaluation over two independently
verified [core acceptance campaigns](core-acceptance.md). It matches every frozen
attempt and reports paired outcomes, evidence coverage, stage-specific safety
counts and measured harness latency. Current scope is **repeatability of the same
mock implementation**. The report does not establish an architectural advantage.

## Export and reconstruct a report

After the [CPU bootstrap](../README.md#run-it-now--no-gpu-no-models-no-isaac-sim), use
fresh campaign directories and a new report file outside both directories:

```bash
# From the RRM directory; use .venv/bin/python on a venv-capable host.
PYTHONPATH=.rrm-deps python3 scripts/core_acceptance.py --output /tmp/rrm-reference --repetitions 2
PYTHONPATH=.rrm-deps python3 scripts/core_acceptance.py --output /tmp/rrm-candidate --repetitions 2
PYTHONPATH=.rrm-deps python3 scripts/core_comparison.py \
  --reference /tmp/rrm-reference --candidate /tmp/rrm-candidate \
  --output /tmp/rrm-comparison.json
PYTHONPATH=.rrm-deps python3 scripts/core_comparison.py \
  --reference /tmp/rrm-reference --candidate /tmp/rrm-candidate \
  --output /tmp/rrm-comparison.json --verify
```

Reference/candidate name report roles, not different algorithms. The tool reads
existing bundles, writes a new report exclusively, and reconstructs that entire
report on `--verify`. It never starts campaign workers or an adapter. A report
cannot overwrite an existing file or enter either source bundle.

## Qualification

Both campaigns must pass independent offline verification with the current pinned
reader and have unchanged executable source throughout execution. Their complete
frozen specifications must match, including case order, repetitions, seed, authored
expectations/labels, numeric/capability profiles and time bounds. Different ordered
schedules cannot be compared through a selected intersection.

The reader rejects directory aliases, identical artifact inventories and reused
recorded run IDs within or across bundles. A copied crash-only bundle is rejected
even if its manifest timestamp changes. Attempts that never recorded a run ID stay
in the report with null identity and explicit per-arm identity coverage. Different
artifact bytes and UUIDs are consistency evidence, not proof of independent trials.

Inputs are consumed from manifest-bound bytes and both bundles are reverified after
report construction. Counts, ledgers and reconstructed objects use canonical JSON
comparison, so booleans cannot substitute for integers. Reports bind input manifest,
configuration, executable source and comparison CLI digests. Hashes detect corruption;
they are not signatures. Executable source pins do not establish matching interpreter,
installed dependencies, hardware or scheduler load. Earlier bundles require their
historical source reader; differing implementations are currently ineligible.

## Outcome and measurement definitions

| Report field | Meaning |
| --- | --- |
| `goal_outcome_matrix` | All nine reference/candidate combinations of VERIFIED_GOAL, GOAL_NOT_VERIFIED and UNKNOWN, over every scheduled pair |
| `candidate_goal_gains` / `candidate_goal_losses` | False→true / true→false changes in recorded goal verification among pairs with both reports available |
| `both_goal_reports_available` | Both arms have a qualified Boolean goal report; this does not establish independently adjudicated negative truth |
| `pairs_with_unknown_goal` | Either arm lacks a qualified goal outcome, including crash, timeout and intentional trace loss |
| `jointly_qualified_pairs` | Both arms have complete replay-qualified evidence; distinct from expectation match or goal achievement |
| `all_attempt_rate_differences` | Candidate minus reference verified numerator, divided by the common total attempt count; interpreted alongside unknown/evidence coverage |
| `arms.*.safety_adjudication` | Each arm's qualified gate counts, rates and explicit coverage gaps; gates can have different sample counts |
| `harness_latency_s.all_attempts` | Median, nearest-rank p95 and maximum outer process/harness elapsed seconds, including failed attempts |
| `harness_latency_s.jointly_qualified_pairs` | The same summaries restricted to explicitly counted pairs with complete evidence |

Each latency group reports reference, candidate and paired candidate-minus-reference
distributions. p95 is sorted sample `ceil(0.95*n)`; an empty sample has count zero
and null statistics. Harness duration includes process startup and containment;
it is not worker-only, robot completion or physical stop latency. Differences from
these deterministic repetitions have no uncertainty interval or significance claim.
Timing differences can reflect host scheduling rather than system behavior.

Earlier retained validation encountered real evidence-write deadline failures at
the fixture's 0.1-second bound, under both concurrent and sequential execution.
Some traces lacked a terminal record; others contained a late gate record that did
not satisfy replay's strict fault-phase boundary. Those attempts remain unqualified
and UNKNOWN in comparison. The reporter does not repair the trace, widen deadlines,
or select only passing repetitions. Current source adds v3 commit receipts and strict
identity/phase reconstruction for records already flushed when timeout is observed.
It deliberately leaves missing, pre-commit, reordered and rejecting-gate writes
incomplete. Historical v2 outcomes are not retroactively upgraded. This closes the
narrow late-commit reconstruction defect, not host scheduling sensitivity or durable
storage qualification.

Two retained v3 34-attempt regressions each produced 33 expectation matches and 31
complete attempts. The additional unqualified attempt was different in each run:
one pre-dispatch gate write timed out; one active numeric write was reconstructed but
a subsequent interruption record was not. These all-attempt observations keep the
remaining scheduling/liveness limitation visible rather than treating strict replay
reconstruction as a throughput fix.

The original legacy `task_success` includes expected-abort success. The report retains
it as a separate all-attempt rate, never a substitute for `goal_met`. Safety confusion
uses authored mock labels independent of verifier decisions, emitted in the same
fixture process. It does not provide external adjudication of safety or completion.

`GOAL_NOT_VERIFIED` means a complete replay reports `goal_met=false`. Planning failure
can produce this report with unavailable terminal observation; it does not prove the
goal predicate false through fresh observation. `UNKNOWN` means the campaign lacks
a qualified goal report because attempt evidence is incomplete. Neither state is
counted as a verified goal. Gains/losses describe changes in recorded goal verification,
not independently adjudicated changes in physical task completion.

## Research next step

This instrument establishes pairing, coverage and reconstruction before the
[controlled architecture experiment](scrum-8/retriever-positioning.md#next-comparative-experiment).
Distinct arms need frozen model/observation/adapter/budget parity, readers for each
source revision, independently adjudicated outcomes, and retained predeclared trials.
Permissions and stop authority remain enforced in executable arms. No change to
RRM control, embodiment adapters or the frozen RRM-EM proposal follows from this tool.
