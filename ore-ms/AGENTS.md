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

Latest numerical resolution: `docs/research/shared-motion-stationarity-findings-2026-11-12.md`.
Saved gradient disagreement is resolved by independent high-precision stationarity
checks; original failed audit remains historical. No prototype/tolerance changes.
Latest input decision: `docs/research/shared-motion-input-contract-findings-2026-11-12.md`.
Direct strict-interval binding is closed: nominal geometry passes,0/19 recorded
night intervals match. No repeated clock searches or automatic training. A
supplied-pair nominal benchmark requires a distinct explicit protocol.
Research decision: `docs/research/multisensor-research-reset-2026-11-12.md`.
Current synthesis: `docs/research/progress-and-next-steps-2026-11-12.md`.
The older research-direction literature note is historical; its rotation and
tracking follow-ups have already been executed and closed.

## Living paper references

Maintain `docs/research/references/README.md`, `papers.json` and `papers.bib` whenever a paper informs a proposal, implementation or experiment. Keep stable paper IDs, primary URLs/versions, reading depth/date, code/checkpoint pins, limitations and linked local decisions/results. Preserve historical reading events. Add cited historical sources from `citation-index.json` to the curated registry before relying on them; a citation is not proof of reading. Never describe a mechanism prototype as an author-system reproduction.

Prior-art/inventory correction: `docs/research/research-relevance-and-prior-art-2026-11-12.md`. Basic thermal covariance weighting is established by MACThermal; MSO is direct RGB–thermal uncertainty-weighted factor-graph prior work (full paper/code novelty gap unverified). EXP009 and nominal `thermal_fixed`/`thermal_cov` already isolate weighting on saved inputs with mixed metrics. Do not present those controls as entirely untested or an automatically novel direction. Original contribution target remains cross-modal comparability/predictive reliability, not generic covariance weighting.

Latest prior-art depth: `docs/research/mso-method-overlap-review-2026-11-12.md`.
Full MSO thesis Chapter5 now read: RGB/thermal image-covariance-weighted sliding-window
BA and dynamic factor removal are established overlap. Chapter5 explicitly excludes
depth/landmark/pose uncertainty; statistical cross-modal calibration/ranking not
demonstrated there. Later IROS2026 DFR submission manuscript and exact MSO code remain
unverified. Existing EXP060/061/069/070 already study relative errors, coverage/NLL
and recording exclusion; do not rediscover them or equate improved NLL with fusion
benefit. No new experiment selected; contribution novelty remains unresolved.

Latest contribution draft: `docs/research/contribution-statement-2026-11-13.md`.
Proposed empirical characterization of uncertainty ranking, cross-camera selection,
statistical fit and downstream pose utility. No validated new fusion method or
verified novelty. Cohorts/targets differ; do not infer a causal chain across
experiments. Old unexamined-recording counts are historical: November6 whole-record
execution exposed its two candidates. No new experiment selected.
