# WS2 agent: completed work and next steps

## Project objective

Build a general agent-driven test bench for learning-based UAV obstacle-avoidance
models: connect one target, select applicable attacks and environment conditions,
execute and evaluate tests, identify reproducible vulnerabilities, then produce
an evidence-based report with possible defensive methods. The shared experiment
loop should remain unchanged when a new model is connected through an adapter.
Attack availability and mission success criteria remain target-specific.

The LLM selects experiments and interprets evidence. The bench validates settings,
executes flights and calculates authoritative outcomes. Proposed defenses are
recommendations with a validation plan; they are not automatically implemented,
retrained or presented as proven improvements.

## Implemented and checked on 2026-10-06

- The same bench supports LLM-free feedback/random/search and optional Claude
  Team selection plus a separate final interpretation. One campaign audits one
  target. The bench owns execution, metric calculation and authoritative outcomes.
- Expanded mode generates fresh continuous obstacle coordinates from a seed,
  density, corridor width and side bias. Bounds, support, intersections and
  source identity are checked; explicit transforms are saved for replay.
- October14: expanded mode permits the original FCRN patch for Kim only under
  the new user authorization. MonoNav patch remains blocked. Earlier sensor-only
  results remain patch-off experiments; never relabel them as patch validation.
- Added offset obstacle, slalom and narrow-gap challenges with saved geometry,
  protected endpoints and checked free-space routes. Real qualification and
  single-factor noise/delay results are in `VALIDATION.md`.
- Shared ModelAdapter defines worker/mission/reset/attack contracts; MonoNav and
  Kim use it. Third-model support has a CPU contract test, not a third real flight.
- English final reports now include evidence-linked unvalidated defense proposals
  with rationale, tradeoffs and validation plans; measured metrics stay authoritative.
- The earlier protected-corridor MonoNav pilot remains 12/12 goal reached. It
  demonstrated no agent advantage and is not a ZoeDepth patch-attack evaluation.

See [EVALUATION.md](EVALUATION.md) for bounds/protocol and
[VALIDATION.md](VALIDATION.md) for the measured evidence and limitations.

## October14 implementation and validation

Expanded Claude selects explore/repeat/isolate/threshold purposes, with validated
source rounds and unchanged comparison variables. Failed clean controls skip
the attack twin and return baseline evidence to the agent in record mode. The
reserved slot is not reassigned. Diagnostics join recent input age, prediction,
commands and GT motion; reports receive this evidence and executed experiment
purposes. Kim-only patch controls now use target-specific bounds.

Validation is in `campaigns/agent_followups_20261014/`: 15 actual flights
(MonoNav8, Kim7), 140 CPU tests and both artifact audits passed. Claude selected
an actual Kim repeat: two passing clean controls and two patch-conditioned
collisions. Patch exposure/causality remains unverified. MonoNav passed both
combined-sensor pairs on protected geometry; slalom/gap clean controls failed.
Isolation/threshold passed CPU tests but were not selected in real flights.
See VALIDATION.md for evidence and remaining limitations.
No larger policy comparison or third-model integration was executed in that batch.

The subsequent four-flight learned/GT diagnosis is documented in
[DEPTH_DIAGNOSTIC.md](DEPTH_DIAGNOSTIC.md) and VALIDATION.md. It did not qualify
clean slalom flight: Mono learned stopped; Mono GT ended in a TSDF runtime error
(inconclusive); Kim learned failed the stationary-time criterion; Kim GT collided.
Those results motivated checks of MonoNav's TSDF input/allocation behavior and
Kim's input/action/controller chain, rather than assuming depth alone was the cause.
Public RGB candidates are recorded in [RGB_MODEL_CANDIDATES.md](RGB_MODEL_CANDIDATES.md).

Subsequently, PULP-DroNet v3 was connected as the third real worker, including
CLI/web selection, capability validation and shared GT scoring. Ten clean
development flights did not qualify it (8collisions,2insufficient progress).
Low-speed tracking and yaw conversion were improved, but glass/column failures
remain. See [PULP_DRONET_VALIDATION.md](PULP_DRONET_VALIDATION.md). The task of
**connecting** a third model is complete. The later bent-corridor work is tracked
in [CORRIDOR_VALIDATION.md](CORRIDOR_VALIDATION.md): MonoNav and PULP Tiny each
completed repeated clean U-course traversals and matched sensor trials. Kim
then completed four clean traversals on a longer two-block U, with a noise
collision and a passing patch trial. The common course now has10/10 clean
completions across the three targets; each also completed L, curved and zigzag
checks with a frozen target profile. Noise caused a MonoNav planner stop and
Kim/PULP contacts. These are small fixed-condition comparisons, not an agent
advantage study. Office/ResBlock failures remain unchanged.
MonoNav's empty-depth handling, TSDF allocation/retention and horizontal path
execution were corrected before those corridor trials. This is not evidence
that the previous Office/slalom failures have all been resolved.

## Corridor agent implementation — October18

The adaptive corridor mode now uses version4 geometry. Claude selects shape,
obstacle count and route-relative positions, sensor conditions and Kim-only patch
surface/placement. Exact settings are validated and saved; the common flight loop
executes one target per campaign. Random/Search use the same action space and
receive rule-based diagnostic follow-ups. See [CORRIDOR_AGENT.md](CORRIDOR_AGENT.md).

Functional validation completed 11 actual MonoNav flights within the separate
12-flight allowance. One failed clean control skipped its attack twin. Claude
removed that obstacle, repeated a combined noise24/delay0.4 failure, isolated
delay0.4 (completed) and noise24 (insufficient progress), then selected noise12
(insufficient progress). The 11-flight artifact audit passed. This verifies an
executed diagnostic loop, not an advantage over another selection policy or a
statistically established noise threshold. Raw evidence is in
`campaigns/corridor_agent_validation_20261018/mononav/`; see
[CORRIDOR_AGENT_VALIDATION.md](CORRIDOR_AGENT_VALIDATION.md) for the evidence review.

The user authorized a separate **108-slot** comparison: MonoNav/Kim/PULP ×
Random/Search/Claude ×12 reserved slots. Failed clean controls skip attack slots;
actual and reserved costs are reported separately. The study uses frozen presets,
sources and scoring, with the same final reporting model for every policy.
This specific approval supersedes the earlier wait-for-review restriction.
Comparison completed October19:105 actual flights,3 skipped attack slots, no
infrastructure errors, all105 artifact checks passed. Reproduced conditions
Random/Search/Claude: MonoNav3/3/1, Kim2/2/0, PULP1/1/0. Claude made more measured
factor contrasts (2 per target versus1/1/0), but this did not show an advantage
in reproduced failures or first-reproduction cost. All9 English reports are
available after a report-validator negation fix and one provider-only retry.
Seven of17 defense proposals need design revision; none has measured efficacy.
The source snapshot preserves the original comparison. Later prompt/report
improvements are documented separately in CORRIDOR_AGENT_VALIDATION.md.

## Remaining research priorities

1. **Broaden agent-selected corridor validation and budget planning.**
   Office expanded mode already generates bounded layouts; the recorded corridor
   suite remains historical fixed-condition evidence. The new v4 action space
   exposes shape, obstacle count/positions, patch placement and sensor settings.
   Endpoint/overlap/path checks are implemented; measured camera exposure remains
   separate from physical mount validation. Continue checking these across targets;
   save exact transforms and reuse them in clean/attack twins. Connect these
   scenes to explore/repeat/isolate/threshold, without giving reactive models GT
   route guidance. Historical v3 trials must not be relabeled agent-driven.
   Reserve enough pairs to confirm discovered failures: the initial Claude
   Kim/PULP campaigns exhausted their budgets without an exact-condition repeat.
2. **Validate clean reliability and attack effects.** Use repeated and independent
   layouts/seeds for all three models; preserve failures and fix target settings
   within comparisons. Confirm Kim patch exposure and FCRN prediction changes,
   and isolate noise/delay effects. Wait for Rui's ZoeDepth attack; no patch
   transfer to MonoNav/PULP. Geometry feasibility alone is not clean qualification.
3. **Validate useful defense proposals.** Give each proposal evidence, a causal
   hypothesis, a concrete intervention and a test plan. Check selected defenses
   on held-out clean/attack cases, including progress, latency and false-stop
   costs. Keep untested proposals distinct from measured improvements.
4. **Measure the agent's advantage under equal budgets.** Compare reproduced
   failure discovery, diversity, factor isolation and report/defense usefulness
   against random, search and manual design. Count confirmation flights and LLM
   costs. The completed 108-slot initial comparison is described
   in [AGENT_EVALUATION_PLAN.md](AGENT_EVALUATION_PLAN.md). Independent campaign
   seeds, a stronger optimizer and a human baseline remain future studies, requiring
   a separately agreed budget. Recorded fixed-condition
   videos illustrate the bench; they do not establish an agent advantage.

## Retained design constraints

The2026-09-22 request was to generate new obstacle positions rather than only
choose from24saved sets. The expanded generator now implements that request.
Original saved layouts remain available for historical replay. Structural walls
stay fixed, geometry changes happen between flights, and exact paired placement,
seeds, asset identity and validation evidence are retained. The LLM proposes
bounded experiments; the validated bench executes and scores them.
