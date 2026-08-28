# 009 — yaw-sweep unstick: results summary

- **Written:** 2026-08-28 (overnight session 2026-08-27 22:20 → 2026-08-28 00:20 EDT)
- **Branch/pin:** validation workspace at `study/droan-gl-avoidance-fix`
  `891ca138` + the yaw-sweep patch (`droan_gl_yaw_sweep_v2.patch`, not yet
  committed); judge = agent_study runner with `HARD_CAP_S` 900→1800
  (uncommitted, proposed v6 change)
- **Harness:** `agent_study/runs/ref_validation_r7_b240_001/` — A1-identical
  workspace, reference planner b, shim budget 240 s/checkpoint, clearance
  gate 1.0 m, fresh wall-time seed per run

## Headline

**The yaw-sweep works at what it targeted — the absorbing PAUSE hover is
eliminated and route completion jumped dramatically — but droan_gl still
passes 0/10 sweep-era R7 runs (0/15 including baseline), and the residual
failures are architectural, not parametric.** Meets the pre-registered
"if the heuristic doesn't get R7 solvable, swap planners" criterion for a
planner swap.

## (c) R7 solvability — ❌ (0/5 baseline → 0/5 v1 → 0/5 v2)

Baseline (008 addendum): 3× absorbing PAUSE hover, 1× churn, 1× shave;
typical progress ≲ half the route.

v1 sweep (PAUSE-trigger only; artifacts `c-r7-solvability/v1/`):

| Run | Seed | cps | Outcome | Cause |
|---|---|---|---|---|
| 1 | 884229 | 6/7 | in-order fail, 526 s | far-boxed hover (no PAUSE → no trigger) |
| 2 | 884805 | 7/7 | goal 8.89 m | judge `HARD_CAP_S=900` truncated a still-progressing flight |
| 3 | 885921 | full | clearance 0.045 m | close-quarters graze between sweeps |
| 4 | 886736 | full | clearance 0.557 m | shave (would pass a 0.5 m gate) |
| 5 | 887161 | 9/9 | goal 13.29 m | far-boxed hover, zero sweeps fired |

v2 sweep (far-boxed trigger + `HARD_CAP_S=1800`; artifacts
`c-r7-solvability/v2/`; min_clearance below computed over the full flown
track, so it is reported even for runs whose verdict failed earlier):

| Run | Seed | cps | Goal err | Dur | zmax | min clearance | Outcome |
|---|---|---|---|---|---|---|---|
| 1 | 887684 | 6/8 | 25.6 m | 256 s | 13.8 | 0.71 m | attempts exhausted at early pocket |
| 2 | 888034 | 8/8 | **0.33 m** | 1277 s | 20.7 | **0.08 m** | full route + goal; killed by clearance |
| 3 | 889251 | 8/8 | 8.48 m | 242 s | 16.3 | **0.01 m** | attempts exhausted; near-contact en route |
| 4 | 889579 | 7/7 | 8.80 m | 454 s | 21.0 | 0.76 m | attempts exhausted |
| 5 | 890156 | 7/7 | 13.65 m | 301 s | 21.5 | 0.26 m | attempts exhausted at first pocket |

## (a), (b), (d) verdicts

- **(a) Build/sanity ✅** — clean build; sweep durations as designed.
- **(b) Sweep behavior ✅** — dozens of live triggers across 10 flights;
  the attempt counter resetting between episodes proves sweep → candidate
  → resume worked repeatedly (v1 run 2 ground through a 21-min flight
  with ~8 recovery episodes); both trigger paths observed ("boxed while
  paused", "far-boxed hover (no pause)"); **zero tumbles, dives, or
  fly-aways in 10 flights** — the bounded-by-construction safety argument
  held.
- **(d) No regression — not run** (R6/R8 skipped once (c) failed the
  campaign question; run before any merge of the patch).

## Why the residuals are architectural

1. **Vote-blocked pockets (4 of 5 v2 fails):** the sweep converts
   'unseen' arcs into evidence, but at hard pockets the blockage is
   camera-voted *collision* — spinning adds keyframes but cannot erase
   standing votes, so attempts exhaust. Fixing this means a principled
   belief-decay/eviction policy — core DROAN redesign, not a parameter.
2. **Close-quarters precision (near-contacts at 0.01–0.26 m in 3 of 5 v2
   runs, during pocket-grinding):** near pillar tops and at close range,
   both sensors have holes (stereo minimum range/FOV, LiDAR vertical FOV)
   and the 1 m veto voxel quantizes; the reactive escape flies through
   what the belief never saw. In a real deployment these are collisions
   but for luck. No gate value fixes this (0.01 m fails any gate).
3. What the sweep DID fix (viewpoint starvation) was the last
   *recoverable* defect; 008 had already fixed the eight parametric ones
   and rejected the crash-prone recovery mechanisms.

## Recommendation to the lead

Per the pre-registered criterion (2026-08-27): **swap the stack's local
planner for a modern (map-based, e.g. ESDF/occupancy kinodynamic)
implementation** as the R7-composition prerequisite, rather than
continuing to tune a 2018 reactive design against architectural limits.
Keep this branch's fixes and the yaw-sweep (committed) regardless — they
are genuine platform improvements and the study's defect-mining record.
Judge-side changes to carry into the v6 re-freeze: `HARD_CAP_S` 1800,
R7 budget 240 s, and the practice/eval obstacle-layout split (approved
2026-08-27).
