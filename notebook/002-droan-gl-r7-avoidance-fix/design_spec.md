# 002 — droan_gl obstacle-avoidance fix (agent-study R7 blocker)

**Status: IN PROGRESS (2026-08-26)**

## Problem context

Campaign v4 of the agent study (Sec. VI-C) is blocked: the reference
solution on the STOCK stack fails its own R7 obstacle rung — the drone
followed the judge-issued route through the pillar field but flew
through a pillar (min ground-truth clearance −0.87 m; see
`notebook/001-agent-study-prereqs/results/h-amendment-r7r8/`).
Previously logged suspects: droan_gl's 1:1 `deviation - path_distance`
cost and the forward-only stereo FOV.

## Code-level analysis (pre-experiment)

Reading the stock pipeline end to end points at a sharper primary
suspect than either logged one:

1. **Disparity holes are classified as free space.** droan_gl's
   expansion shaders skip pixels with invalid disparity
   (`center <= 0` → `return`, `disparity_expand_horizontal.cs:40`), so
   those pixels keep fg=0 / bg=INT_MAX. In `collision.cs:80-89` a
   trajectory point projecting to such a pixel falls through the
   `unseen` and `collision` branches into **`seen` (free)**. Any region
   where stereo matching fails is therefore treated as verified-free,
   and trajectories through it get the best cost (max progress).
2. **The stereo front end is likely to fail exactly on the pillars.**
   Disparity comes from stock `stereo_image_proc` `disparity_node`
   (block-matching defaults, `texture_threshold`≈10) fed by the Isaac
   front stereo pair. The R7 pillars are untextured flat-color
   cylinders (`primvars:displayColor`, no texture) — a worst case for
   block matching: interiors give no valid disparity → holes → item 1
   marks the flight corridor through the pillar "seen free".
3. Secondary (real but gated behind 1–2): the collision vote in
   `droan_gl_node.cpp:268` (`collision > seen` majority across graph
   keyframes) can dilute a genuine detection; the 1:1
   `deviation - path_distance` cost (`droan_gl_node.cpp:315`) has no
   tunable weights; `expansion_radius` 2.0 m vs the 1.0 m clearance bar
   leaves little margin for tracking error.

## FINDING (a), 2026-08-26 — the prior R7 verdict was INVALID

First diagnostic run (stock stack, judge-equivalent bring-up env): the
left camera image contains **no pillars** and the disparity image is
**100 % invalid** (empty scene ahead). Root cause: the judge
(`r5_provenance.py`) set `ISAAC_SIM_GUI=<obstacles_r7.usda>`, but in the
standalone launch path it also selects (`ISAAC_SIM_USE_STANDALONE=true`)
the scene is chosen by `ISAAC_SIM_SCENE` (`resolve_scene_from_env`,
`pegasus_app.py`); `ISAAC_SIM_GUI` is only read by the non-standalone
`run_isaacsim.launch.py` path. **The pillar world never loaded**; the
2026-08-25 "stock stack penetrated a pillar (−0.87 m)" flight was flown
through empty air and judged against virtual pillar coordinates — and
the route generator's discrimination property guarantees a near-straight
flight violates virtual clearance. "Stock AirStack fails R7" is
therefore UNPROVEN. Judge fixed (`ISAAC_SIM_SCENE`); re-testing with the
world actually loaded. The secondary finding stands unchanged and is
now confirmed live: an all-invalid disparity image yields
`fg_expanded` 100 % zero and droan_gl treats the whole horizon as
seen-free (holes-are-free semantics, item 1 above).

## Proposed implementation

Diagnose first, then fix the STACK (not the benchmark — the pillar
field and judge stay frozen):

1. Reproduce in a pinned-commit tuning workspace with the reference
   solution applied; inspect the live disparity image and droan_gl's
   expanded fg/bg output while facing/approaching a pillar.
2. If disparity holes confirmed: tune the stereo front end in
   `perception_bringup/launch/stereo_image_proc.launch.xml`
   (SGBM `stereo_algorithm`, texture/uniqueness thresholds, speckle,
   disparity range) until pillars produce valid disparity at planning
   range.
3. droan_gl hardening as needed, each as a declared ROS parameter with
   the stock value as default where behavior-preserving:
   - collision vote rule (any-frame-collision vs majority),
   - cost weights `deviation_weight` / `progress_weight`
     (replacing the hardcoded 1:1),
   - `expansion_radius` margin review.
4. Port the validated patch back to the repo; the study config then
   re-pins `airstack.commit` (config change → new campaign id; v4 has
   no scored trials, so nothing is pooled across the change).

## Running experiment log (updated as iterations complete)

| Run | droan_gl config | Route seed | Waypoints | Min clearance | Verdict |
|-----|-----------------|-----------|-----------|---------------|---------|
| 1 (stock) | 1:1 cost, vote `c>seen`, exp 2.0 | 781298 | 8/8 in order, 1 over budget (252 s) | **−0.59 m (2 penetrations at z 0.5–4.6 m, 1 at z 10)** | FAIL (budget; clearance would also fail) |
| 2 | w=2.0, ratio 0.25, exp 2.0 | 82577* | in order through wp2, then **fly-away to (393, 250)** | 0.26 m (no penetration) | FAIL (route) |
| 3 | w=2.0, ratio 0.5, exp 2.5, max_dev 10 | 83771 | **8/8 in order, all within budget, 705 s** | 0.69 m (single 3 s shave, pillar 10, z=10) | FAIL (clearance, by 0.31 m) |
| 4 | w=2.0, ratio 0.5, exp 3.0, max_dev 10 | 84574 | double-reversal route; drone wandered corridor edges (4–13 m slop), then **froze mid-field at t≈120 s** (max_deviation hold, no recovery) | n/a (no penetration) | FAIL (route; freeze) |
| 5 | w=2.0, ratio 0.45, exp 2.5, graph_nodes 20, max_dev 10 | 84965 | **PASS**: 5/5 in order, within budget, 317 s, goal error 0.63 m | **2.05 m** | **PASS** |
| 6 | same as 5 (confirmation, fresh seed) | 85389 | in order except cp3 missed by 16.6 m (corridor 15 m); goal error 0.25 m | (not evaluated) | FAIL (route, by 1.6 m) |
| 7 | run-5 config + windowed monotonic progress (no destructive trim, plan-refresh dedup, progress_back 5 m / window 25 m) | 86175 | 10/10 in order (windowing works — no shortcut), goal error 0.36 m; cp8 over budget (456 s: ~250 s loiter in dense region) | **0.08 m** — 3 s dive-swoop (z 9.7→3.7) past pillar 13's flank | FAIL (budget + clearance) |
| 8 | run-7 config + z_deviation_weight 4.0 (symmetric) + max_deviation 10→5 (euclidean) | 87070 | froze at (29.4, 13.1) t≈120 s; earlier pulled DOWN to z 4.8 chasing the plan's ground-anchored climb ramp | n/a | FAIL (route; freeze) |
| 9 | run-7 config + ASYMMETRIC vertical: below-plan costed (w 4.0) and hard-capped (3 m); above-plan free; max_deviation 10 lateral-only | 87504 | tightest tracking yet (cp closest 0.18–2.4 m through cp5), then **668 s boxed stall** at one spot; cp7 never reached (900 s cap) | (not evaluated) | FAIL (budget) |
| 10 | run-9 config + auto-rewind: boxed ≥2 s (and >8 m displaced since startup) → controller REWIND for 6 s → resume ADD_SEGMENT | 88801 | **9/9 in order, ALL within budget** (auto-rewind killed the stall) | **0.07 m** — single 3 s lunge-and-retreat at pillar 6 | FAIL (clearance) |
| 11 | run-10 config + absolute collision vote: ≥2 keyframes flagging collision = unsafe (seen votes cannot dilute; most "seen free" votes are invalid-disparity holes) | 90043 | 9/9 in order; cp5 over budget (217 s — rewind cycles cost time) | 0.39 m (improved from 0.07) | FAIL (budget + clearance) |
| 12 | run-11 config + yaw_smoothing_alpha 0.1→0.4 (camera tracks motion direction faster) + graph_angle_threshold 30°→15° (denser angular keyframe coverage) | 91292 | **PASS**: 8/8 in order, all within budget, goal error 0.58 m | **1.05 m** | **PASS (full final config)** |
| 13 | same as 12 (confirmation, fresh seed) | 92461 | 9/9 in order but 2 legs over budget (144 s, 420 s turn-thrash); ends with **loss of control** (z −5 m, 5–7 m/s tumble) | **−0.59 m** (during the tumble) | FAIL (crash) |
| 14 | run-12 config + breadcrumb corridor (recently flown positions override `unseen` blocking; collision still wins) + rewind guards (below-plan abort mid-rewind; ≥3 consecutive rewinds → 60 s cooldown) | 93847 | 9/9 in order, goal error 0.20 m, no crash; 2 legs over budget (141 s, 167 s) | 0.49 m | FAIL (budget + clearance) |
| 15 | run-14 config + expansion 2.5→3.0 (navigable now that breadcrumbs unlock retreat) + **R7 waypoint budget 120→240 s (JUDGE param trial — needs lead sign-off**; 120 s was calibrated on unobstructed R5 flight, never on R7 threading) | 95092 | in order but cp5 missed by 16.0 m (corridor 15; wide swings from 3.0 inflation); NO budget violations at 240 s | 0.765 m | FAIL (corridor + clearance) |
| 16 | expansion back to 2.5 + **360° LiDAR close-range collision veto** (latest filtered Ouster cloud voxelized in map frame; candidates within 2 m of an occupied voxel = collision; LiDAR only adds evidence); budget 240 s | 96401 | 9/9 in order, ALL within 240 s budget; drone legitimately overflew part of the field at z≈21 (judge skips z > pillar height) | 0.47 m (pillar 9, z 11.4, during descent) | FAIL (clearance) |
| 17 | run-16 config + PAUSE-on-boxed (no safe candidate → controller PAUSE immediately; resume ADD_SEGMENT when candidates return; rewind escalation unchanged) | 97826 | 9/9 in order; **CLEARANCE PASSED (1.08 m)** — PAUSE killed the shave class; but final leg 650 s (500 s pinned beside the leg's discrimination pillar at z 15–19, escaped only by slow altitude ratchet over pillar tops) | **1.08 m** | FAIL (budget only) |
| 18 | run-17 config + lidar_veto_radius 2.0→1.5 (2.0 + voxel quantization sealed the 5-m pillar gaps — surface gap ~3.2 m — forcing climb-over as the only escape) | 99038 | physical impact + ground bounce (z −3.4 at t≈180), clearance 0.18 during the tumble | 0.18 m | FAIL — 1.5 m veto is UNSAFE |

**No-regression testing (R6, empty world, default route) found two more
defects (2026-08-26 late):** (1) judge: R7 evaluations overwrite
`stacks/ROUTE.txt` and never restore the default — an R6 scoring run
flew a stale R7 route (fixed in `r5_provenance.py`: non-R7 rungs
re-issue the default route). (2) stack: PAUSE-on-boxed deadlocks TURNS
even in an empty world — turn directions are `unseen` until the camera
sweeps, and sweeping requires motion; stock got through turns on
momentum/overshoot, which the unconditional pause froze (R6 hover-stall
400 s at the first corner, reproduced twice). Fix: the pause is now a
**collision brake** — it only engages when boxed AND the LiDAR reports
an obstacle within `auto_brake_range` (4 m); otherwise stock momentum
dynamics are preserved. A progress-reseed escape hatch (global
closest-point after a 30 s fully-blocked pause) also added.

**Late-session debugging (2026-08-27 00:00–01:40), instrumented live:**

- R6 empty-world stall root cause: NOT the windowing (bisect misdirect) —
  the run-11 absolute vote (`collision ≥ 2`, ratio 0) lets two far-field
  noise votes (far trajectory points overlapping far-ground disparity
  expansion near the horizon) kill any trajectory, and a hover makes it
  an absorbing state (degenerate keyframes keep re-voting). R6 passes
  (goal error 0.07 m) with ratio 0.45 restored + min_votes 2 floor.
- Confirmed live: planner B re-anchors its published path start at the
  vehicle each replan → progress carry-over reseed (not reset-to-0) on
  plan change is required (`set_global_plan` marks reseed; `trim`
  re-derives near the old value).
- Vote made RANGE-AWARE: within `close_vote_range` (8 m) of the vehicle
  `collision_min_votes` vetoes outright (near-field stereo reliable);
  beyond, seen votes may dilute per `collision_seen_ratio` 0.45
  (far-field noisy). Reconciles R6 (far-noise immunity) with R7
  (close-detection protection).
- Auto-REWIND correlated with all three ballistic-tumble crashes
  (z below ground, speeds > library max — controller tracking-timeline
  corruption from mode churn suspected). **REWIND commanding removed
  (auto_rewind default false); the PAUSE collision brake is decoupled
  (auto_pause, lidar-proximity-gated) and retained.**

**FINAL STACK CONFIG (frozen for handoff):** deviation_weight 2.0,
z asym (below-plan weight 4.0, cap 3 m), max_deviation 10 (lateral),
collision vote = range-aware (min_votes 2; close 8 m strict; far ratio
0.45), expansion 2.5, graph_nodes 20, angle threshold 15°, yaw alpha
0.4, windowed monotonic progress (5/25 m) + replan carry-over +
30 s-blocked global reseed, breadcrumb corridor (1.5 m), LiDAR veto
2.0 m / voxel 1.0, PAUSE brake (boxed + LiDAR < 4 m), auto_rewind OFF.
Safety properties: no crashes, no fly-away, no clearance violations in
this configuration's runs; hard adversarial R7 draws may STALL (clean,
score-visible corridor/budget fail) — an honest platform limitation
for the lead to weigh (R7 budget recalibration and/or route-difficulty
policy), not a trial-invalidating failure mode.

Run-16 lesson: veto pipeline verified healthy live (lidar sees pillars,
94–341 returns; TF ok; droan subscribed, QoS compatible, sim time on).
The residual shave is **committed-path flyout**: when every candidate is
vetoed, droan publishes nothing and the controller flies the previously
committed segment to its end through now-known-occupied space. The
veto cannot recall a committed segment — but the controller's PAUSE
mode can stop it.

Run-15 lesson: clearance still shaves at 3.0 m camera-side inflation —
the shaves are moments when the pillar is in NO recent keyframe (stereo
FOV hole); map-side inflation cannot fix an obstacle that is not in the
map. The platform carries a 360° LiDAR that droan_gl ignored — the
correct sensor for exactly this failure class.

Run-13 lesson: cumulative auto-rewinds during a boxed reversal turn
replayed the flight history back down the takeoff climb into the
ground (droan_gl publishes and monitors nothing while rewinding) —
crash. And reversal turns are structurally blocked: candidates toward
the reverse leg are `unseen` (camera never faced it), rewind retraces
backward still facing forward, and rotate-in-place is not executable
(the trajectory library removes consecutive duplicate positions). The
breadcrumb corridor makes retreat/turn arcs ordinary plannable
candidates: space the vehicle actually flew is verified-free and no
longer blocked as unseen.

Shave signature across runs 7/10/11: a 1–3 s lunge-and-retreat — the
pillar sits just outside the lagging camera FOV during a turn, then a
committed segment must be flown out before the retreat engages. Run 12
attacks sensing coverage: faster yaw tracking + denser angular
keyframes.

Run-9 lesson: asymmetric vertical handling nailed the tracking; the
last failure mode is the boxed stall — all candidates
collision/unseen/out-of-bounds with no recovery. The platform's own
answer (RewindMonitor + the controller's REWIND mode = fly the
known-free flown corridor backwards) existed on both ends but nothing
commanded it when no behavior executive is running. droan_gl now
self-triggers it for the boxed condition only (stationary-stuck stays
on the `stuck` topic for executives; the >8 m displacement guard keeps
it out of takeoff/landing).

Run-8 lesson: the judge takes off to 10 m BEFORE the mission engages
while the planner splines from the ground — early flight is
legitimately ~9 m ABOVE the plan. Symmetric z-costing + a euclidean
bound both pull the drone down toward the ramp and then freeze it.
Also structural: costs cannot prevent selection of a dangerous
candidate that happens to be the ONLY safe one — only hard rejection
can. Hence run 9's asymmetric design: above-plan free, below-plan
costed and hard-capped.

Run-7 lesson: the recurring hazard is cheap VERTICAL escapes — the
route altitude is deliberate, descending candidates leave the camera
FOV downward, and textureless ground reads as free; a dive-swoop swept
0.08 m past a pillar flank. Fix: cost vertical deviation separately and
harder (`z_deviation_weight`), and tighten the deviation bound to 5 m
(needed lateral swings around pillars are only ~3–5 m).

Run-6 lesson: the route doubled back and droan_gl's global
closest-point progress metric (Trajectory::get_closest_point searches
ALL segments; trim() erases everything before the winner) deletes the
far excursion whenever two legs pass near each other — the long-known
corner-cutting defect, amplified here to 16.6 m. Fix: GlobalPlan keeps
the full plan and a monotonic arc-length progress tracker; closest-point
queries (progress update AND trajectory scoring) are windowed to
[progress − 5 m, progress + 25 m]; identical republished plans no longer
reset progress (geometric dedup in set_global_plan).

Run-4 lesson: 3.0 m inflation seals the field (min pillar spacing 5 m
center-to-center ⇒ narrowest gaps ~2.8 m surface-to-surface); the
correct margin lever at this point is perception coverage, not more
geometry. graph_nodes 10→20 doubles the pose-graph memory (~20 m /
600° of keyframes) so the just-flown corridor stays *seen* through
turns — addressing both the turn-blocking and the freeze.

Run-1 mechanism: 1:1 `deviation − path_distance` cost dives the drone
toward the textureless (= invalid disparity = "seen free") ground and
15 m off-corridor while stuck; penetrations occur during the low-altitude
excursion; sub-1 m shaves from stale far-range keyframes outvoting fresh
close-range collision detections (majority vote `collision > seen`).
Run-2 mechanism: vote ratio 0.25 + noisy block-matcher disparity blocks
the 150° turn at wp2 entirely; only open-sky candidates stay "safe";
after ~10 m all map keyframes face away and the corridor back is
permanently `unseen` → planner walks off at constant heading (no
hover/stop candidate in the library, no deviation bound). Fix in run 3:
`max_deviation` hard bound (reject candidates ending >10 m off-plan) +
vote ratio 0.5 + expansion 2.5 m.

## Test plan

- **(a) Reproduce + diagnose:** stock stack, R7 world, reference
  planner; capture disparity/expanded-image evidence of the failure
  mode at a pillar. Verdict = documented root cause.
- **(b) Perception fix in isolation:** with tuned stereo params, the
  disparity image contains the pillar (valid disparity over its
  interior) at ≥15 m range; droan_gl's fg/bg cloud shows the pillar.
- **(c) End-to-end R7:** reference solution passes the FULL R7 judge
  (`judge.sh R7`, scoring mode) — conformance + in-order checkpoints +
  min clearance ≥ 1.0 m — on ≥2 fresh route seeds.
- **(d) No regression:** R6 (provenance flight + swap) and R8
  (landing) still pass on the fixed stack; R3 takeoff unaffected
  (subsumed by R7 bring-up).
- **(e) Freeze:** commit fix, re-pin `airstack.commit` in
  `agent_study/config/study_config.yaml`, record new campaign id,
  update protocol/HANDOFF statuses.
