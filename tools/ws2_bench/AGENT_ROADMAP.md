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
- Current expanded mode excludes patches for both models. MonoNav patch requests
  are rejected at runtime and removed from allowed saved-mode actions. Legacy Kim
  FCRN patch controls remain available, but are outside current sensor validation.
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

## Next priorities

1. Continue target-specific attack-effect validation from the current sensor-only
   evidence. Wait for Rui's separately developed ZoeDepth patch; current FCRN
   patch tests are also deferred per user. Never claim arbitrary patch transfer.
2. Qualify slalom/narrow-gap and held-out scenes beyond the initial offset-obstacle
   runtime cases. Preserve failed clean results; a feasible geometric route is
   not a planner success guarantee. Keep speed/mission fixed within comparisons.
3. **Wait for user review before the larger method comparison.**
   [AGENT_EVALUATION_PLAN.md](AGENT_EVALUATION_PLAN.md) proposes distinct reproduced
   failures per budget, hypothesis isolation and validated defensive follow-up,
   fair baselines/manual comparisons, presentation evidence and publication venues.
4. Connect and qualify a third real model through the existing adapter contract.
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
