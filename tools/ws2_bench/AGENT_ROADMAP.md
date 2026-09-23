# Future LLM adversary: generate new obstacle placements

User decision,2026-09-22. This is planned work, not implemented behavior.

The current web bench selects from24saved layouts: eight each for Easy, Medium
and Hard. It does not sample fresh obstacle coordinates for each campaign.
The future LLM adversary must be able to explore **new obstacle positions**,
instead of only selecting one of those saved sets.

- Let the LLM choose a spatial hypothesis or placement constraints from prior
  outcomes (for example, an obstacle near a turn or a narrower passage).
  A seeded placement generator should sample fresh coordinates within that
  request. A fixed layout ID must not be the only environment action.
- Expose additional prop counts and density alongside positions; the current
  Easy1+1, Medium3+3 and Hard5+5plants/columns can remain baseline presets.
  Environment changes and sensor/patch attack parameters stay independently
  configurable so results can be interpreted.
- Patch actions are limited to on/off and physical size. Preserve the supplied
  texture colors and opaque material; do not expose contrast, opacity or patch
  height as an agent action. Size is not a calibrated attack-effect strength.
- Use the bench's validated scene interface. Check floor support, intersections,
  protected start/goal space, allowed objects/regions and route feasibility.
  Preserve structural walls. Geometry checks alone do not prove a planner passes.
- Apply geometry changes between episodes. Use the same realized coordinates,
  initial state and scene conditions for each clean/attack pair; disable only
  the selected attacks in the clean twin.
- Save explicit realized transforms, seed, source asset identity and validation
  results. Replaying a failure must restore those values without asking the LLM
  to regenerate the scene. Keep the existing saved layouts as reproducible baselines.
- The LLM reads metrics and failure evidence, selects subsequent tests within
  the agreed parameter bounds and budget, and may summarize the evidence. The
  bench retains configuration validation, flight execution, evaluation and logging.
  Compare against random/search baselines with the same budget. Avoid attributing
  failure to an attack when its paired clean flight already failed.

First implementation step: add a seed-driven explicit-placement schema and
generator to the existing bench, validate clean/attack replay, then expose that
interface to the LLM. Keep the web UI for observation and operator pause/stop.
