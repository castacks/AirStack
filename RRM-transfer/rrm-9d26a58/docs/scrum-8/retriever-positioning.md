# Retriever adjacency and RRM research positioning

Reviewed 2026-10-02. This is a research assessment and proposed experiment, not a
change to Phase 1 requirements, runtime selection, or the frozen RRM-EM scope.

## Assessment

[Retriever](https://www.retriever.systems/) makes temporal composition explicit
through stateful Flows, local clocks and edge synchronization. Its example agent
combines belief, planning, skill monitoring and control. The
[paper, version 2](https://arxiv.org/html/2607.17213v2) conditions replay on recorded
inputs and timing, including stochastic outputs; its evaluation distinguishes
task progress, projected duration and human-assisted runs from autonomous completion.

There is substantial architectural overlap. Belief plus planning plus learned
skills plus feedback is not, by itself, a defensible RRM novelty claim. This does
not establish that RRM has no distinct contribution: novelty needs a broader
literature review and experiments, not an inference from one project website.

RRM's candidate distinction is **independently checked, evidence-bound execution**:
explicit predicted effects, contextual safety, scoped capability/permission/approval
and authorization, observation-grounded completion, bounded failure handling, and
honest separation of cancelled, safe-confirmed and unconfirmed outcomes. These are
design properties and research hypotheses, not demonstrated superiority. Retriever's
timing contracts could complement such a layer; a middleware migration is neither
necessary to test the hypothesis nor authorized by this assessment.

## What current evidence establishes

The core Oracle and fault fixtures measure deterministic contract behavior and
reconstruction. They do not measure learned reasoning, perception, physical
collision safety, cross-embodiment transfer or autonomous manipulation performance.
Read [current goal-to-finish status](end-to-end-status.md) and the
[SIL evaluation specification](evaluation.md) before making those claims.

The callback-deadline pass left planning and replanning unbounded. The current core
increment bounds those proposal callbacks, isolates their inputs, validates returned
graphs and records failed attempts. It addresses a concrete failure mode, not a
new temporal-composition contribution. An independent reviewer checked alignment
and exposed replay-generation and recovery-trigger binding gaps during development.

## Next comparative experiment

Hypothesis: with tasks, models, observations and adapter held constant, independently
checked predictive effects and evidence-bound admission reduce stale/invalid execution
and false success, without excessive false refusal or unacceptable completion cost.

1. Freeze task corpus, seeds, model/prompt/config revisions, adapter/controller,
   observation source, budgets, intervention protocol and outcome adjudication.
2. Compare a closed-loop baseline with the same planner, belief, policy and recovery
   opportunities against the full RRM boundary. Add explicit ablations for predictive
   effect checking and contextual evidence checks. Removing all feedback is not the
   sole baseline; that would confound the claimed benefit.
3. Include nominal safe trials, displaced targets, ambiguity, recoverable/persistent
   failure, stale/contradictory observations and delayed reasoning. Permissions and
   stop authority remain enforced in every executable arm. Evaluate unsafe proposals
   in shadow/offline replay rather than bypassing protective controls to collect data.
4. Use independent ground truth for safety and semantic completion—not the verifier's
   own verdict. Report all attempted, failed, unfinished and evidence-incomplete runs.
5. Report verified goal rate, false-success rate, unsafe proposals/admissions,
   false-refusal rate, recovery, intervention count and median/p95/max latency.
   Keep actual completion separate from partial progress; never substitute projected
   duration for measured completion. Include denominators and uncertainty intervals.

Start by qualifying the measurement instrument offline. Only a separately qualified,
version-pinned simulator campaign can support integrated performance claims.
The [paired core campaign instrument](../core-comparison.md) now reconstructs all
attempts from two verified same-implementation exports, preserving unknown outcomes
and timing/coverage denominators. This qualifies repeatability reporting; distinct
architecture arms, external adjudication and model/environment parity remain open.
Retriever is relevant related work and a potential later runtime comparison;
neither an integration dependency nor a strawman policy-only competitor.
