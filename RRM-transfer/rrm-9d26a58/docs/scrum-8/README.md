# SCRUM-8 — architecture allocation and interface specification

The [working roadmap](../../WORK_PHASES.md) organizes next steps and is revisable.
Requirements/contracts govern this design. Engineering “Phase 1” means the historical
needs/requirements baseline; C01–C09/S01–S10 are contracts/scenarios, not roadmap stages.

Prepared 2026-09-13 UTC against RRM source snapshot `9d26a58eb8516b6754c5d12b041cdc789950e047` and AirStack `ore_proj` / `a6dad8caf54722e5eba3481367e914ce213e6135`.

This is the working SCRUM-8 design and incremental implementation record. It does not establish integrated SIL compliance or close the Jira item. The allocation and contracts were written before new runtime primitives. Earlier prototype architecture, benchmarks and setup prescriptions remain historical; this specification governs the continuation under the engineering Phase 1 requirements baseline.

## Design and evaluation

- [Architecture and allocation](architecture.md)
- [Logical interface contracts](interfaces.md)
- [SIL scenarios, metrics and curriculum mapping](evaluation.md)
- [Scene difficulty and reasoning-evaluation ladder](scene-selection.md)
- [Model and artifact persistence](model-and-artifact-persistence.md)
- [Cosmos Reason2 model-selection gate](model-selection-gate.md)
- [Dynamic embodiment feasibility and admission workflow](feasibility-workflow.md)
- [Embodiment-neutral goal intake and routing](semantic-goal-routing.md)
- [Current goal-to-finish performance status](end-to-end-status.md)
- [Historical hand embodiment decision and qualification gates](hand-embodiment-decision.md)
- [Proposed learned embodiment model (RRM-EM)](../embodiment-learning-architecture.md)
- [Retriever adjacency and research positioning](retriever-positioning.md)
- [Core acceptance campaign and authored event labels](../core-acceptance.md)
- [Paired core campaign comparison](../core-comparison.md)

## Operations and evidence

- [Command console](command-console.md): deterministic GUI-to-public-task path and its limits.
- [Visual evaluation protocol](vision-evaluation-protocol.md): observation pairing, scoring and rejection criteria.
- [Office control qualification](office-control-qualification.md): bounded control investigation and admission gates.
- [Historical Office learned-plan runbook](office-live-demo.md): transfer/import procedure, not current flight authority.
- [Historical remote inspection lessons](remote-state.md): allocation and namespace pitfalls.
- [Historical validation ledger](validation.md): dated software and simulator checkpoints.
- [Historical hand integration gates](integration-plan.md): G0–G7, not the current roadmap.
- [Detailed handoff](../../HANDOFF.md): chronological evidence, failures and continuation records.

Current results belong in the performance-status page; new run narratives belong
in the handoff/evidence records, not copied into every design or runbook page.

## Document lifecycle

- Items in this index are the current SCRUM-8 specification, runbooks and measured
  status unless their own header says otherwise.
- [The RRM-EM architecture](../embodiment-learning-architecture.md) is a proposed
  research extension, not current verified functionality.
- [The original RRM-1 architecture](../architecture.md) and evidence/run-history
  documents retain earlier designs and observations for provenance. They are not
  silently rewritten into current claims; their status banners and the current
  [end-to-end status](end-to-end-status.md) determine how they may be used.

## Source authority

Live reads on 2026-09-13 confirmed [CONOPS v4](https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1048599), [requirements v3](https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/786434), and the [architecture outline v1](https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1015810). SCRUM-6 and SCRUM-7 are Done. [SCRUM-8](https://deboabolade.atlassian.net/browse/SCRUM-8) remains To Do and links to SCRUM-9 for the simulation/SIL/HIL pipeline. No Jira or Confluence writes were performed.

User instructions and the engineering Phase 1 requirements baseline take precedence over the old A10G/Panda/GR00T/Qwen choices. Curriculum examples are reference material, not additional approved requirements.

## Design readiness

All 11 requirements have a primary owner, supporting responsibilities, contract boundary and planned evidence. Logical semantics are specified for nominal operation, uncertainty, refusal, recovery, interruption and reconstruction. Model, transport, scene asset, embodiment-specific limits and deployment parameters remain separate decisions. Runtime implementation status is recorded explicitly; architecture coverage is not verification coverage.
