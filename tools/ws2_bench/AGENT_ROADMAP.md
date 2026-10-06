# WS2 agent: completed work and next steps

## Implemented and checked on 2026-10-06

- The same bench supports LLM-free feedback/random/search and optional Claude
  Team selection plus a separate final interpretation. One campaign audits one
  target. The bench owns execution, metric calculation and authoritative outcomes.
- Expanded mode generates fresh continuous obstacle coordinates from a seed,
  density, corridor width and side bias. Bounds, support, intersections and
  source identity are checked; explicit transforms are saved for replay.
- Noise, illumination, delay and original opaque patch on/off, size and schedule
  are exposed within common bounds. Clean/attack twins share geometry and light.
  Patch opacity, contrast and height are not agent controls.
- English requests, reasons, replies and reports are visible in the existing web
  viewer alongside actual simulator and planner inference images.
- Two independent reference clean flights passed. The subsequent MonoNav pilot
  completed4flights per method,12total, with every flight reaching the goal.
  No method found a failing condition; no Claude advantage was demonstrated.

See [EVALUATION.md](EVALUATION.md) for bounds/protocol and
[VALIDATION.md](VALIDATION.md) for the measured evidence and limitations.

## Next priorities — planned, not yet implemented or evaluated

1. **Establish useful attack conditions.** Measure whether the surface patch is
   actually in the target camera view and its projected size/exposure. Verify
   its effect on the deployed depth model before attributing flight changes to
   the texture. Rui's patch targets FCRN; deployed TensorFlow/PyTorch preprocessing
   parity remains unresolved, and transfer to MonoNav's ZoeDepth is unproven.
   Use separate noise-only, delay-only and patch-only trials to identify failure
   boundaries and avoid confounding combined factors. Rendering a patch is not
   evidence of an effective attack. Finalize permitted bounds with CyLab.

2. **Broaden feasible navigation challenges.** The pilot preserves a straight8m
   corridor. Add turns, narrower passages and obstacle placements that require
   avoidance, while retaining protected launch/goal regions and a feasible route.
   Qualify clean controls on the new missions. Report environment-only baseline
   failures separately from added sensor/patch effects. Do not change mission,
   speed, planner or bounds between policies within a comparison.

3. **Run a larger matched-budget evaluation.** Agree on the next flight budget
   before execution; the completed12-flight budget is not permission for more.
   Use multiple independent seeds and balanced method orders, the same action
   space, target and confirmation rules, and saved-case replay. Measure distinct
   reproducible failure mechanisms, flights to a failure, and LLM latency/usage.
   Different configuration IDs do not necessarily represent different failure
   mechanisms. Keep inference errors and failed clean controls separate. All
   three methods are already automated, so reduced human effort needs its own
   measurement. Preserve a no-advantage result if that is what the data show.

4. **Prepare the presentation from measured results.** Show the actual
   condition→flight→metrics→Claude decision loop and the report. State that
   integration and this clean route were validated; improved failure discovery
   remains unestablished. The user records video. Expand the same evaluation to
   Kim only after its own clean mission and target-specific attacks are checked.

## Retained design constraints

The2026-09-22 request was to generate new obstacle positions rather than only
choose from24saved sets. The expanded generator now implements that request.
Original saved layouts remain available for historical replay. Structural walls
stay fixed, geometry changes happen between flights, and exact paired placement,
seeds, asset identity and validation evidence are retained. The LLM proposes
bounded experiments; the validated bench executes and scores them.
