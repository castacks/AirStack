# SCRUM-8 — architecture allocation and interface specification

Prepared 2026-09-13 UTC against RRM source snapshot `9d26a58eb8516b6754c5d12b041cdc789950e047` and AirStack `ore_proj` / `a6dad8caf54722e5eba3481367e914ce213e6135`.

This is the working SCRUM-8 design and incremental implementation record. It does not establish integrated SIL compliance or close the Jira item. The allocation and contracts were written before new runtime primitives. Earlier prototype architecture, benchmarks and setup prescriptions remain historical; this specification governs the continuation under the Phase 1 baseline.

Read in order:

1. [Architecture and allocation](architecture.md)
2. [Logical interface contracts](interfaces.md)
3. [SIL scenarios, metrics and curriculum mapping](evaluation.md)
4. [Remote inspection and readiness](remote-state.md)
5. [Implementation and validation record](validation.md)
6. [End-to-end SIL integration plan](integration-plan.md)
7. [Scene difficulty and reasoning-evaluation ladder](scene-selection.md)
8. [Model and artifact persistence](model-and-artifact-persistence.md)
9. [Cosmos Reason2 model-selection gate](model-selection-gate.md)
10. [Dynamic embodiment feasibility and admission workflow](feasibility-workflow.md)

## Source authority

Live reads on 2026-09-13 confirmed [CONOPS v4](https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1048599), [requirements v3](https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/786434), and the [architecture outline v1](https://deboabolade.atlassian.net/wiki/spaces/SCRUM/pages/1015810). SCRUM-6 and SCRUM-7 are Done. [SCRUM-8](https://deboabolade.atlassian.net/browse/SCRUM-8) remains To Do and links to SCRUM-9 for the simulation/SIL/HIL pipeline. No Jira or Confluence writes were performed.

User instructions and Phase 1 requirements take precedence over the old A10G/Panda/GR00T/Qwen choices. Curriculum examples are reference material, not additional approved requirements. Search also exposed an RRM2 capstone collection in the Software Development space; it is not silently merged into the SCRUM-7 baseline.

## Design readiness

All 11 requirements have a primary owner, supporting responsibilities, contract boundary and planned evidence. Logical semantics are specified for nominal operation, uncertainty, refusal, recovery, interruption and reconstruction. Model, transport, scene asset, embodiment-specific limits and deployment parameters remain separate decisions. Runtime implementation status is recorded explicitly; architecture coverage is not verification coverage.
