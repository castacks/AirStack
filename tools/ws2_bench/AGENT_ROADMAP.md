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

## Next priorities

1. Continue target-specific attack-effect validation from the current sensor-only
   evidence and the new Kim-only patch runs. Wait for Rui's separately developed
   ZoeDepth patch. Never claim arbitrary patch transfer.
2. Obtain repeatably passing clean controls in meaningful avoidance scenes;
   the October14 slalom/gap cases failed clean in the tested seeds. Preserve failed clean results; a feasible geometric route is
   not a planner success guarantee. Keep speed/mission fixed within comparisons.
3. **Wait for user review before the larger method comparison.**
   [AGENT_EVALUATION_PLAN.md](AGENT_EVALUATION_PLAN.md) proposes distinct reproduced
   failures per budget, hypothesis isolation and validated defensive follow-up,
   fair baselines/manual comparisons, presentation evidence and publication venues.
4. Extend the three-model common-corridor evidence to independent layouts/seeds. Registration or a few successful
   traversals alone does not establish general clean-flight reliability.
5. Test selected defense proposals on held-out clean/attack cases; report safety
   and progress/false-stop costs. Generating a proposal is not proving its effect.
6. Prepare the next meeting's slides from measured results, preserving Ravi's
   original design/order. The October6 meeting used Ravi's original file named
   `0923 presentation.pptx` (actually presented October1). User records video.

## Retained design constraints

The2026-09-22 request was to generate new obstacle positions rather than only
choose from24saved sets. The expanded generator now implements that request.
Original saved layouts remain available for historical replay. Structural walls
stay fixed, geometry changes happen between flights, and exact paired placement,
seeds, asset identity and validation evidence are retained. The LLM proposes
bounded experiments; the validated bench executes and scores them.
