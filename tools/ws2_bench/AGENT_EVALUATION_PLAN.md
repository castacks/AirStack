# Evaluation and publication direction

Current priority (2026-10-22): the first12 functional flights and12 additional
PULP development flights completed without mission failures. The matched PULP
pilot now uses two held-out seeds × Random/Search/Astra × eight new pairs,
at most96 reserved flights. The first six pairs measure failure discovery and
same-setting/same-outcome confirmation; the last two test a frozen conditional
outcome prediction through a common reporter. Source, weights, runtime identity,
priors, model/effort and execution order are frozen before outcomes. Exact pilot
protocol is saved in `campaigns/matched_selection_pilot_20261022_r2/protocol.json`.
The first attempt stopped before takeoff because disabled patch geometry was
validated against an unused small surface. Its startup failure and source snapshot
remain in `matched_selection_pilot_20261022`; r2 fixes that initialization only
and checks that seeds, draws, order, priors and budgets are unchanged.
This small pilot cannot establish general superiority or root-cause accuracy.
Future comparisons across targets still require the same effective space,
evidence, prior information and budget. Historical narrower-space results are
development references, not matched controls. Preserve every result even when
the LLM loses. Defense implementation/effect validation follows attack/agent
evaluation. These two deferred tasks must remain in future-work updates.

The October18–19 comparison is recorded separately in
CORRIDOR_AGENT_VALIDATION.md. The older proposal below is research rationale,
not the current run budget. October22 authorizes an initial12-flight functional
batch followed by autonomous development and additional experiments.

Original proposal: 2026-10-14. Sensor validation alone is not evidence that an
agent beats search.

The fixed ten-flight study now found Kim noise32 collisions in two matched
pairs, and one delay0.5 progress failure; MonoNav passed both tested factors.
These cases were selected by the developer, **not discovered by Claude**. Use
them to check the system, not as wins in a future policy comparison. The noise
case is at an obvious parameter endpoint; an endpoint sweep may find it easily.
Include that simple baseline rather than making an agent win look harder than
it was. Give every method the same prior evidence, or reserve these cases as
development data and compare on held-out conditions.

## What could make the agent better?

The useful claim is **more distinct, reproducible and actionable failure evidence
per fixed experiment budget**, not convenient automation or fluent reports. A
general bench and a Claude call alone are a modest systems contribution. Random
and numerical search can already optimize a scalar failure score efficiently.

An agent should decide which experiment reduces uncertainty: explore an untested
scene, repeat a suspected failure, isolate one perturbation, locate a failure
threshold, or challenge a proposed explanation. For example, after failure with
noise+delay, choose matched noise-only and delay-only controls instead of blindly
increasing both. Keep model, speed, allowed bounds and outcome oracle fixed.
Expanded Claude now selects these four experiment purposes with validated source
rounds and comparison constraints. Functional validation is underway; this does
not establish scientific usefulness or superiority over a strong search policy.

Useful diagnostic inputs include collision object/location, measured camera age,
pre-failure depth/command summaries and model-specific state. The updated episode diagnostics now supply recent input age, predicted depth,
actions/commands and motion to selection and reporting. These asynchronous
summaries do not establish causal attribution. Give the same available evidence
to comparison methods and predefine how their policies use it. Distinguish
environment-only clean failures from incremental sensor-attack failures: both
can reveal operational limits, but they answer different questions.

| Desired advantage | Evidence to show | Avoid overclaiming |
|---|---|---|
| Sample efficiency | Distinct reproduced failure families versus cumulative flights; flights to first confirmed family | Include clean flights, repetitions and all real attempts; report wall time and LLM cost separately |
| Breadth | Predefined failure-mode × scene × isolated-factor coverage | A new coordinate or LLM wording does not automatically define a new failure |
| Explanatory usefulness | Fraction of proposed hypotheses supported by held-out intervention tests | Plausible narrative is not a causal explanation |
| Actionable defenses | Clean/attack outcomes before and after one or two selected defenses on held-out cases | Check progress, latency and false stops; never win by stopping every flight |
| Portability | Same testing/report loop through at least a third real model adapter, model-appropriate task criteria | Shared adapter code and two models alone do not establish universal coverage |

## Fair comparison proposal

- Compare uniform Random, a properly tuned search baseline, and Claude using the
  same adapter, condition generator, bounds, evidence, flight budget, pairing,
  success criteria and criteria for counting a reproduced failure. All confirmation
  flights consume budget; who chooses to repeat/isolate is part of the policy. Keep the current heuristic Search as
  a readable baseline, but beating only that weak local mutation policy is not
  a strong paper result. Add one established mixed-space optimizer (e.g. TPE)
  before claiming a broad advantage over search.
- First proposed comparison: 2 models × 3 methods × 3 independent seeds ×
  8 flights = **144 flights**. This is a proposal, not authorization to execute.
  Eight flights mean four clean/attack pairs; confirmations consume those pairs.
  Feasibility/clean qualification is common preparation, separately counted.
  A stronger paper will likely need more seeds and held-out scenes; decide after
  this pilot, not by running until a desired winner appears.
- Report confidence/variation across seeds and targets; randomize method order.
  Freeze all versions/prompts and comparison rules before running. Keep failures
  from infrastructure out of model performance but include them in resource cost.
- Define failure families before inspecting policy labels: collision/unsafe
  contact, timeout/insufficient progress, and planner termination, tagged with
  scene type and isolated perturbation. Merge equivalent symptoms by a blinded
  review; do not let the evaluated LLM grade its own novelty.
- Add a history-disabled Claude ablation to test whether feedback matters, and
  a matched fixed/scripted design. A manual comparison should use multiple people
  with a stated expertise range, the same budget and visible evidence, no cherry-
  picked best run. Mason alone is not a representative expert baseline.
- Report report-quality and defense-validation metrics for every method. Give
  all methods the same final reporting model if the question is about selection;
  separately evaluate whether LLM reporting improves interpretation.

## A presentation that demonstrates the contribution

1. Short paired clean/attack replay in a scene that really requires avoidance,
   with the actual input/depth, trajectory and contact/progress evidence.
2. One sequence of hypothesis → selected isolation/repeat → measured result.
   Show what the evidence ruled out, not only a collision compilation.
3. A cumulative confirmed-failure curve against flight count, plus a coverage
   table. Use all budgeted runs and seed variation, not the best-looking seed.
4. A concise report card: vulnerable condition, replay ID, limits, defense
   hypothesis, and a held-out before/after safety/progress check if completed.

If Claude ties or loses on failure search, say so. A reusable, validated testing
tool or an empirical study can still be useful; do not claim algorithmic novelty
from adding an LLM API.

## Publication candidates

These are fit/effort judgments, not acceptance predictions or an official venue
ranking. Sources checked 2026-10-06.

- **AutoODD:** coauthor Jay Patrikar lists it as **CoRL 2025 [Workshop]**, not a
  main-conference paper: https://www.jaypatrikar.me/ . Its preprint is
  https://arxiv.org/abs/2509.08638 . An OpenReview record exists at
  https://openreview.net/forum?id=6JdRaSPJ0o (browser verification prevented reading
  its decision directly). Do not describe it as a CoRL main-track acceptance.
- **First workshop option:** a future CoRL/ICRA/IROS safe/robust robot learning
  workshop. SAFE-ROL's 2025 official scope includes validation, generalization and
  safe learning: https://sites.google.com/view/corl-2025-safe-rol-workshop . This
  is a past edition illustrating fit, not an open submission deadline. Check the
  chosen edition and archival policy when its CFP is available.
- **Practical full-paper target: ICUAS 2027.** UAV learning-based perception,
  navigation/control and safety are explicit topics. Official page currently
  lists full papers due **5 February 2027**, conference **14–17 June 2027**:
  https://uasconferences.com/2027_icuas/ . A well-validated UAV testing system
  with comparative experiments is a reasonable scope match in our judgment.
- **Testing-tool target: ICST Testing Tools and Data Showcase.** Particularly
  suitable if the contribution is a reusable test tool plus reproducible data.
  The 2027 track exists but its page currently has no detailed CFP:
  https://conf.researchr.org/track/icst-2027/icst-2027-testing-tools-and-data-showcase .
  Prior official scope accepts research prototypes:
  https://conf.researchr.org/track/icst-2026/icst-2026-testing-tools-and-data-showcase .
  ICST main research is a different, demanding target; it explicitly covers
  testing AI systems and cyber-physical systems:
  https://conf.researchr.org/track/icst-2027/icst-2027-research-papers .
- **Later journal target: IEEE RA-L.** Scope includes innovative robotics ideas
  and application case studies, but this needs a stronger contribution and
  evaluation than the current integration. It is not the easiest default:
  https://www.ieee-ras.org/publications/ieee-robotics-and-automation-letters/ .
- **Applied journal alternative: Journal of Intelligent & Robotic Systems.**
  Its robotics/unmanned-systems scope fits a thoroughly validated UAV testing
  system: https://link.springer.com/journal/10846/aims-and-scope . It has also
  published UAV test-generation work, such as
  https://link.springer.com/article/10.1007/s10846-026-02381-1 . This supports topical
  fit, not a promise of acceptance or a claim that novelty is unnecessary.

Recommended decision: aim first for **ICUAS** if a regular UAV paper matters, or
**ICST's tool track** if the reusable testing system is the main contribution.
A relevant workshop is appropriate for earlier feedback. Strengthen the claim
around experimental diagnosis and validated defensive follow-up before aiming
at a stronger robotics/ML main track or journal.

## Authorized corridor comparison — 2026-10-18

User approved three targets × three policies × 12 reserved flight slots = **108**,
plus at most12 separate functional-validation flights. This supersedes the earlier
wait-for-review restriction for this specific budget; the old144-flight proposal
remains unexecuted. Each 12-slot campaign has six clean/attack pairs. Failed clean
controls skip the attack slot without reallocation, so the executed total may be lower.

Use geometry v4 in `corridor_space.py`, one target per campaign. All policies share
bounds, frozen target presets, contact scoring, diagnostics and exact pair geometry.
Random/Search also have rule-based repeat/isolation/bisection; Claude chooses its own
purposes. Search remains a **local heuristic**, not a strong optimizer or BO claim.
A later paper needs a stronger optimizer and independent campaign seeds.

Primary descriptive outcomes: reproduced conditions per reserved and actual flight
budget, first-reproduction cost, and predefined shape × perturbation-set × outcome
families. Secondary diagnostic outcomes: valid actual isolation interventions and
observed scalar-bracket reductions. Neither family counts nor bracket probes prove
internal causes, monotonic response or statistical significance.

All methods get the same final Claude reporting model; only agent mode uses LLM
selection. Review defense proposals for evidence fit, concrete intervention,
trade-offs and a baseline-vs-defense / clean-vs-attack / repeated-held-out validation
plan. Field/citation presence is not a semantic quality score. Defense effectiveness
and human performance are not being tested in this comparison. Functional-validation
runs and historical fixed-matrix runs are excluded from policy wins.

Diagnostic comparison uses **distinct measured factor contrasts**, not how often
an LLM labels a test `isolate`: a failed combined condition must have a valid,
matched single-factor observation with unchanged geometry/light and retained
factor value. Identical repeated contrasts count once. Executed isolation/probe
counts remain separate. Conditional scalar probes require a measured passing
endpoint with every other factor unchanged; an all-off clean control cannot anchor
a multi-factor bracket. Reported intervals are observations, not proven thresholds.

The authorized study finished on October19:105 actual flights and3 skipped slots,
with no infrastructure errors and all artifact checks passing. Claude had fewer
reproduced conditions than Random/Search but more distinct measured factor contrasts.
This is not evidence of overall agent superiority. Full counts, report recovery,
defense review and limits are in [CORRIDOR_AGENT_VALIDATION.md](CORRIDOR_AGENT_VALIDATION.md).
No additional comparison budget is implied. Next study design should address
confirmation-budget allocation, matched held-out cases and a stronger optimizer;
report fluency or proposal counts are not benefit metrics.
