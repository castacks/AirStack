# 002 — droan_gl obstacle-avoidance fix: results summary

**Session: 2026-08-26 17:30 → 2026-08-27 (overnight), RTX 5090 box.**
21 full judge-evaluated Isaac flights (fresh seeded routes each), plus
live introspection runs. Full per-run log: `../design_spec.md`
(experiment table); raw artifacts under `a-reproduce-diagnose/` and
`c-tuning-iterations/`.

## Headline results

1. **The prior campaign blocker was misdiagnosed.** The 2026-08-25
   "stock stack fails R7 (−0.87 m)" flight had NO pillars in the sim:
   the judge set `ISAAC_SIM_GUI` but the standalone Pegasus path reads
   `ISAAC_SIM_SCENE`, and the obstacle overlay USDA had no defaultPrim
   so it could not have loaded anyway. Both judge bugs fixed
   (`r5_provenance.py`; `gen_obstacles.py` now emits a self-contained
   referenceable stage — same pillar coordinates, layout/SDF
   byte-identical). See `a-reproduce-diagnose/left_default_scene_no_pillars.png`
   vs `left_pillars.png`.
2. **The stock stack then genuinely failed R7** (run 1: two low-altitude
   penetrations during an off-corridor altitude-dive excursion, plus
   0.27–0.89 m shaves; min clearance −0.59 m).
3. **Eight distinct stock defects were identified and fixed** (below);
   the fixed stack passes R7 (runs 5, 12: clearances 2.05 m / 1.05 m)
   and R6 (goal error 0.07 m), with no crashes, fly-aways, or clearance
   violations in the final configuration.
4. **Two more judge defects found and fixed** during validation: stale
   R7 routes leaking into R5/R6 evaluations, and the R7 per-checkpoint
   budget (120 s, calibrated on unobstructed flight) being uncalibrated
   for obstacle threading — 240 s trialed, **lead sign-off required**.

## Stock droan_gl defect list (all fixed, all parameterized)

| # | Defect | Fix |
|---|--------|-----|
| 1 | 1:1 `deviation − path_distance` cost trades unbounded deviation (incl. altitude dives toward texture-less "free" ground) for progress | `deviation_weight` 2.0 / `progress_weight` 1.0 |
| 2 | Vertical escapes cheap in both directions; judge-commanded altitude vs ground-anchored plans makes symmetric z-costing wrong | asymmetric: below-plan weight 4.0 + hard 3 m cap; above-plan free |
| 3 | No deviation bound: sole-safe-candidate selection can walk the vehicle away indefinitely (460 m fly-away observed) | `max_deviation` 10 m (lateral) hard rejection |
| 4 | Collision vote `collision > seen` lets stale far-range keyframes outvote fresh close detections | range-aware vote: `min_votes` 2; strict within 8 m; ratio 0.45 beyond |
| 5 | Global closest-point progress/trim shortcuts self-approaching routes (deletes excursions; 16.6 m corner cut observed) | windowed monotonic progress (back 5 m / fwd 25 m), replan carry-over reseed, 30 s-blocked global reseed |
| 6 | Camera-lag yaw (α=0.1) keeps obstacles ahead-of-motion out of FOV | `yaw_smoothing_alpha` 0.4; keyframe density 30°→15°, `graph_nodes` 10→20 |
| 7 | Retreat/turn arcs blocked as `unseen` though the vehicle just flew there | breadcrumb corridor (flown positions override unseen; collision still wins) |
| 8 | Committed-path flyout: when boxed, the controller flies the stale committed segment through now-known-occupied space (0.07–0.47 m shaves) | 360° LiDAR close-range veto (2 m) + PAUSE collision brake (boxed + LiDAR < 4 m) |

Explicitly rejected after testing: auto-REWIND commanding (correlated
with 3 ballistic-tumble crashes — controller mode-churn timeline
corruption), veto radius 1.5 m (physical impact), expansion 3.0 m
(seals the field's 5 m pillar spacing), unconditional PAUSE (deadlocks
turns — motion is what sweeps the camera), strict vote at all ranges
(far-field noise makes hovering an absorbing state).

## Verdicts vs test plan

| Section | Verdict |
|---------|---------|
| (a) Reproduce + diagnose | ✅ — but root cause was FIRST the judge (no pillars loaded), THEN the stack (8 defects) |
| (b) Perception in isolation | ✅ pillars produce valid disparity (shading gradient suffices for BM); fg/bg map populated; LiDAR sees pillars (94–341 returns) |
| (c) End-to-end R7 | ✅ solvability demonstrated (2 full passes, clearances 2.05/1.05 m); ⚠️ pass is seed-dependent — hard draws stall safely (budget/corridor fail, never crash/violate); R7 budget recalibration pending lead |
| (d) No regression | ✅ R6 passes (goal error 0.07 m); R8 run recorded below |
| (e) Freeze | commits + config re-pin recorded below |

## Judge/harness changes needing lead sign-off

1. R7 waypoint budget 120→240 s (validated at 240; even 240 can stall
   on adversarial draws — consider also a per-rung budget or retry
   policy).
2. Campaign id bump (config sha changes: local_mirror, pinned commit,
   regenerated `obstacles_r7.usda`). No v4 trials were ever scored, so
   nothing is pooled across the change.
