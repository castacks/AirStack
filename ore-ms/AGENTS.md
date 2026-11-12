# Research continuity for ore-ms

The user explicitly wants research decisions informed by the available papers
and wants to avoid repeated diagnostic or tuning loops. Preserve that preference
across sessions.

- Before recommending a new method, consult relevant primary papers and released
  implementations, then compare the proposal with the existing experiment inventory.
  Explain the specific measured failure it addresses and what would be new.
- Separate author claims, our interpretation, implementation feasibility and
  results actually obtained here. Mark abstract-only reading and unchecked code.
- A failed configuration closes that configuration. It does not establish that
  thermal VO, uncertainty learning or shared representations cannot work.
- Missing dense labels can constrain evaluation without ruling out published
  self-supervised or synthetic-supervision methods. State the exact limitation.
- Keep proposals finite: one hypothesis, fixed controls, explicit endpoints and
  a stopping decision. Avoid a succession of renamed scores or parameter searches.
- Read the latest dated progress brief and its linked decision note before
  resuming an older experiment's proposed next step.

Latest executed feasibility: `docs/research/shared-motion-feasibility-findings-2026-11-12.md`.
Independent gradient audit fails one check: no training or tolerance rescue.
Research decision: `docs/research/multisensor-research-reset-2026-11-12.md`.
Current synthesis: `docs/research/progress-and-next-steps-2026-11-12.md`.
The older research-direction literature note is historical; its rotation and
tracking follow-ups have already been executed and closed.

## Living paper references

Maintain `docs/research/references/README.md`, `papers.json` and `papers.bib` whenever a paper informs a proposal, implementation or experiment. Keep stable paper IDs, primary URLs/versions, reading depth/date, code/checkpoint pins, limitations and linked local decisions/results. Preserve historical reading events. Add cited historical sources from `citation-index.json` to the curated registry before relying on them; a citation is not proof of reading. Never describe a mechanism prototype as an author-system reproduction.
