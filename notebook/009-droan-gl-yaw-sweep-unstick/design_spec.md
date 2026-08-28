# 009 — droan_gl yaw-sweep unstick (active-perception pause recovery)

- **Date started:** 2026-08-27  **Last updated:** 2026-08-28
- **Branch:** `study/droan-gl-avoidance-fix` (develop-based study pin; work
  tested in the agent-study validation workspace at `891ca138`)
- **Commit started from:** `891ca138` (parent-repo record branch:
  `airstack-paper` @ `3d887c10`)
- **Campaign:** Agent study (paper Sec. VI-C) — R7 reference-solvability
  blocker, continuation of [008](../008-droan-gl-r7-avoidance-fix/design_spec.md)
- **Overall status:** `DONE` — implementation complete and validated in
  10 judge flights; test (c) verdict ❌ (0/10) → per the pre-registered
  criterion the recommendation is a planner swap, see
  [results/results_summary.md](results/results_summary.md)

## Problem context

Campaign v5's re-demonstration batch (008 addendum, 2026-08-28: 0/5 R7
passes at a 240 s/checkpoint budget) showed the frozen droan_gl safe
config fails R7 by **absorbing PAUSE hover** (3/5), local-minimum churn
(1/5), and a clearance shave (1/5) — never by budget. The absorbing
hover is structural: when boxed, droan_gl PAUSEs the trajectory
controller; a paused vehicle holds position AND heading, so the
forward-facing stereo camera's viewpoint freezes; the belief map (which
treats 'unseen' as unsafe) never changes; no new safe candidate can
appear; the pause is permanent. The code's own comment names the
dependency: "the forward-only camera cannot resolve 'unseen' without
motion."

The recovery mechanisms that would break the hover (controller REWIND,
breadcrumb-retreat via appended segments) were rejected in 008: a
*spatially extended* segment appended mid-pause is raced by the
controller's virtual-time tracker at unbounded speed (ballistic-tumble
crashes), pending a controller-level safe-retreat primitive.

Lead decision (2026-08-27 session): R7 stays a **composition rung** —
the platform's local layer must reliably thread the (geometrically
sparse: 2% occupied, median 6.5 m gaps) field so the study measures
agent integration, not platform repair. Try a **yaw-sweep unstick**; if
it cannot make R7 reference-solvable, swap droan_gl for a modern local
planner (DROAN is 2018-era).

## Proposed implementation

### 2.1 Yaw-sweep unstick in droan_gl — `DONE`

Active-perception recovery: when PAUSEd and still boxed after a delay,
command the vehicle to fly a **tiny circle** around the hover point
with yaw stepping through a full revolution, so the stereo camera
sweeps the surroundings and converts 'unseen' arcs into real
free/occupied evidence. The existing candidate evaluation then finds a
real exit and the normal resume path un-pauses.

**Safety argument (why this cannot reproduce the 008 tumbles):** the
virtual-time race is only dangerous through spatially extended
segments — racing time along a large retreat arc commands large
position/velocity setpoints. The sweep segment's waypoints all lie on
a `sweep_radius` (~0.5 m) ring around the hover point: even a
worst-case time race commands positions inside that ball, and yaw slew
is bounded by the attitude controller. Bounded by construction.

Mechanics (mirrors the disabled `auto_unstick` block's integration
points in `droan_gl_node.cpp`):

- Trigger: in the `best_traj_index < 0` branch — `auto_yaw_sweep &&
  paused_ && (now - paused_since_) > yaw_sweep_delay_s &&
  sweep_attempts_ < yaw_sweep_max_attempts`.
- Action: build a `TrajectoryXYZVYaw` of waypoints on a circle of
  `yaw_sweep_radius` around the current tracking point, ~15°/waypoint
  over `yaw_sweep_revolutions`, per-waypoint velocity
  `yaw_sweep_velocity`, yaw rotating uniformly through the sweep
  (starting from current yaw); switch mode to ADD_SEGMENT and publish
  (same command path as normal planning).
- Guards: `no_pause_until_ = now + expected sweep duration + margin`
  (else the still-close LiDAR instantly re-PAUSEs and truncates the
  sweep); `sweep_attempts_` increments per sweep and resets when a safe
  trajectory is found; trajectory-library note — duplicate-position
  waypoints are erased and time = distance/velocity, hence the circle
  (a pure yaw-in-place trajectory collapses to one waypoint).
- Params (all in droan_gl config): `auto_yaw_sweep` (study config:
  true), `yaw_sweep_delay_s` (5), `yaw_sweep_radius` (0.5),
  `yaw_sweep_velocity` (0.35 — keep ≥/near controller
  `min_virtual_tracking_velocity`=0.3), `yaw_sweep_revolutions` (1.25),
  `yaw_sweep_max_attempts` (2).

Expected sweep rate: 2π·0.5/0.35 ≈ 9 s/rev ≈ 40°/s → a 15°-density
keyframe every ~0.4 s.

### 2.1b v2: far-boxed trigger + judge cap (added 2026-08-28) — `DONE`

v1 batch findings (5 runs, 0 passes, but the absorbing PAUSE hover was
eliminated — full per-run log in `results/c-r7-solvability/v1/`):

| Run | Seed | Outcome | Cause |
|---|---|---|---|
| 1 | 884229 | FAIL in-order, 6/7 cps, 526 s | far-boxed hover: camera-voted box with LiDAR > `auto_brake_range` never PAUSEs → no sweep trigger |
| 2 | 884805 | FAIL goal 8.89 m, **7/7 cps**, still progressing | judge `HARD_CAP_S=900` truncated the flight (budget 240 × 8 cps allows ~1980 s) |
| 3 | 885921 | FAIL clearance 0.045 m (route + goal complete) | close-quarters graze between sweeps (pillar 41.2,−4.1, t=185) — not sweep-caused |
| 4 | 886736 | FAIL clearance 0.557 m (route + goal complete, 359 s) | shave; would pass a 0.5 m gate |
| 5 | 887161 | FAIL goal 13.29 m, 9/9 in-order | far-boxed hover again; zero sweeps fired |

v2 changes: (i) far-boxed trigger — anchor the tracking point during
consecutive no-candidate ticks; a sustained (< 1 m displacement,
> `yaw_sweep_delay_s`) hover arms the sweep even without PAUSE;
(ii) judge `HARD_CAP_S` 900 → 1800 in `r5_provenance.py` (proposed v6
judge change, applied for validation — stalls still end early via the
stationary check). Remaining known risk: close-quarters clearance
(0.045/0.557 m in 2 of 5 v1 runs) — untouched by v2, informs the
clearance-gate decision.

### 2.2 Validation harness — `DONE` (from 008 addendum)

Reuse `agent_study/runs/ref_validation_r7_b240_001/`: A1-identical
workspace at the pin, reference planner b applied, shim budget 240 s,
5 × `./judge --scoring R7` with fresh seeded routes. Edit droan_gl in
the workspace, build in-container, rerun the batch.

## Test plan

- **(a) Build + unit sanity:** droan_gl builds in the robot container;
  a sweep segment's generated waypoint times are finite and total
  duration ≈ expected (log check on first triggered sweep).
- **(b) Sweep behavior in isolation:** in a triggered sweep (judge R7
  flight), logs show pause → sweep → resume; odometry shows the circle
  (≤ ~1 m displacement) and no altitude dive/tumble; the flight
  continues after the sweep (absorbing hover broken).
- **(c) R7 reference solvability, frozen params + sweep:** ≥5 fresh
  seeded `./judge --scoring R7` runs. Target: strong majority pass
  (≥4/5) with zero crashes/fly-aways. Compare failure modes vs the 008
  addendum baseline (0/5). Clearance gate stays 1.0 m for this test —
  shaves are recorded separately so the gate decision stays informed.
- **(d) No regression:** R6 (empty world, provenance + swap) and R8
  (landing) still pass with the sweep enabled — the sweep must never
  trigger in the empty world (it only fires from a sustained boxed
  PAUSE).
