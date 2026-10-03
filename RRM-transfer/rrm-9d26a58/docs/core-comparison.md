# Paired core campaign comparison

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

The retained validation also encountered real evidence-write deadline failures at
the fixture's 0.1-second bound, under both concurrent and sequential execution.
Some traces lacked a terminal record; others contained a late gate record that did
not satisfy replay's strict fault-phase boundary. Those attempts remain unqualified
and UNKNOWN in comparison. The reporter does not repair the trace, widen deadlines,
or select only passing repetitions. These observations leave evidence-write timing
and late-write fault reconstruction as a separate core reliability gap.

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
