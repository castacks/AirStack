# Historical prototype roadmap

The original August 2026 planning sketch targeted a March 2027 IROS submission
and ordered Isaac/Panda integration, GR00T policy, learned reasoning, tasks/metrics,
baseline arms, seeded campaigns and paper preparation. That schedule and its
conference-date assumptions are retired, not verified current deadlines.

Use the [revisable working roadmap](../WORK_PHASES.md) for proposed next steps and
[current measured status](scrum-8/end-to-end-status.md) for evidence.
Prototype phase numbers and Panda/GR00T selections do not govern current work.

## Rationale worth retaining

- Seeded reset and reproducible observations precede defensible comparative results.
- Numeric/swept-volume safety needs embodiment-specific measured evidence; placeholder
  limits and semantic support do not establish collision-free execution.
- Authored safety labels must be independent of verifier decisions. Traces alone
  cannot establish whether a verdict was correct.
- An oracle provides a reference within its vocabulary, not universal ground truth.
- Report per-metric results; the [historical target matrix](benchmarks.md#historical-target-matrix)
  and its weights are hypotheses, not approved thresholds. A hard safety violation
  cannot be averaged away by task success.
- Cross-embodiment claims require verified outcomes with an actual adapter; swapping
  a profile or selecting a pre-registered policy does not prove transfer.

The superseded detailed calendar remains recoverable from Git history.
