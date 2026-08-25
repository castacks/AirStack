# RAVEN-Multi: CBBA bundles + comms-robustness — implementation plan

## Context / current state
`ConsensusAssigner` is a **shared-solution CBAA**: every robot gossips full state
(positions, confirmed BBs, ray-leads); each solves the *same* 1:1 assignment from
identical inputs, so it is conflict-free by construction — no bid exchange, no
consensus rounds. Cost = xy surface distance + soft ray penalty (25) − finite
retention (`switch_margin` 8) + peer repulsion. Returns `{agent: task_key}`.
Tasks = confirmed BBs + persistent ray-leads. Frontiers are a *separate downstream
fallback* used only when a robot is assigned no task.

Goals: (1) **bundles** (L>1); (2) **robustness under imperfect comms** for the
degradation sim, without breaking the shared-solution guarantee where it must hold.
Not a goal: optimal coordination (impossible under divergence) — graceful degradation.

## 1. Bundles (L)
- `assign()` → `{agent: [keys]}` ordered bundle; `my_task()` = head. `bundle_len`
  ROS param, default 1 ⇒ behaves exactly like today.
- **Ordering — ray terminal.** A bundle grows only while its *last* task is a BB.
  Allowed: `[BB]`, `[ray]`, `[BB,BB]`, `[BB,ray]`, `[BB,…,BB,ray]`. Never `[ray,*]`.
  A ray's range is unknown, so you cannot route a path *through* it. Frontier: solo
  only, never bundled, never a tail.
- **Reallocation — receding-horizon (commit head, replan tail).** The head is the
  only executed (nav) target; it is committed via the full `switch_margin` and never
  abandoned mid-flight. The tail is re-optimised every tick with a *half* margin
  (`0.5·switch_margin`) — light hysteresis, because retargeting the tail costs
  nothing (nothing has moved). So `(BB1,ray1)` → `(BB1,BB2)` once BB2 out-scores
  ray1 (the 25 ray penalty makes a confirmed BB win unless ~25 m farther).
- **Equity (anti-greed).** Pass 1 assigns *every* agent a head before pass 2 grows
  any bundle; pass 2..L grows breadth-first (every bundle gets its 2nd before any
  gets a 3rd). Growth is gated by a **detour cap** `BUNDLE_MAX_DETOUR_M` (20 m ≈
  cluster-mate radius) so a bundle never reserves a far task away from a closer free
  drone. `BUNDLE_SLOT_PENALTY` (12) adds diminishing marginal gain per extra slot —
  **note: inert at L=2** (added equally to all of one agent's tail candidates); it
  only changes outcomes at L≥3.

## 2. Geography prior — the cross-map-lure fix
Problem: a far unclaimed BB gets assigned to the only *free* (far) agent → it flies
across the map → regains comms en route → the BB was already claimed → drops it →
undirected wander.

Fix (shared-consistent): a target belongs to its geographically closest agent.
`owner_dist[T] = min over agents of eff_dist(agent, T)`;
`cost[A,T] += GEO_PRIOR_W · max(0, dist(A,T) − owner_dist[T] − GEO_MARGIN)`.
The closest agent pays 0; a far agent is penalised by how much farther it is (beyond
a `GEO_MARGIN` deadband), so it declines the far BB → it stays unassigned → frontier
fallback (local work), and the BB waits for its closest agent. Uses only shared
inputs ⇒ deterministic ⇒ preserves the shared solution. **No absolute pursuit-radius
cap** (that would orphan a genuinely isolated far target forever); a genuinely-closest
agent still goes.

## 3. Age-weighted peers — robustness under degradation
Each peer carries `w = f(now − last_seen)`: 1 while fresh, decaying to 0 at the TTL
(8 s), excluded past TTL (existing). Used two places:
- **Ownership:** `eff_dist(peer,T) = dist / max(w, ε)` — a stale peer "owns" targets
  less, so a *live* agent inherits a silent/dead peer's BBs (fixes "closer peer is
  dead → target never served").
- **Repulsion:** scaled by `w` — don't avoid a region on account of a stale peer.

This is *intentionally* per-robot and not shared-consistent: it only matters under
degradation (where consistency is already lost) and buys robustness there; under good
comms `w≈1` everywhere ⇒ no-op. This is the main robustness lever for the sim.

## 4. Frontiers — deconflicted fallback, NOT in the shared assigner
Frontiers must **not** enter the shared assigner: they are per-robot (each robot only
knows its own), so if robot_2 can't see robot_1's frontier it computes robot_1 as
"heading to BB X", avoids X, while robot_1 is off exploring ⇒ **X is orphaned**.
So: after the shared BB/ray consensus, if `my_task is None`, pick the best local
frontier, adding **peer-repulsion from shared peer positions** to the choice — this
deconflicts exploration (attacks the measured `far_redundancy` 1.4–1.8). Solo, never
bundled. The geo-prior (§2) is what prevents the lure; the frontier gives productive
local work and post-drop recovery (no wander).

## 5. Debug logging (enabled now)
Log this robot's **full** `assigned_map` (every agent's bundle), each peer's age, and
the task table — i.e. "what I think everyone is doing." Diffing the three robots' logs
quantifies divergence under degradation. Gated by a debug flag, on for now.

## Edge cases reviewed
- **L=1** ⇒ pass 2 skipped ⇒ byte-for-byte today's behaviour (default).
- **Head completed** (visited ⇒ task removed): pass 1 re-picks; the old tail gets half
  retention as a head candidate ⇒ smooth "bundle advances" transition.
- **Ray→BB conversion:** ray task vanishes, BB appears; bundle re-forms next tick.
- **Closer peer is dead:** age-weight inflates its `eff_dist` ⇒ it stops owning ⇒ live
  agent takes the BB. Without this, geo-prior would defer to a corpse forever.
- **Geo-prior deferring to a *busy* closest agent:** bounded — the target waits, the far
  agent stays productive on a frontier, and the closest agent takes it when free.
- **Isolated far target, no closer peer:** genuinely-closest agent goes (correct); we
  accept the long trip rather than orphan it.
- **Under degradation two robots pick the same target:** unavoidable; age-weight +
  retention + geo-prior shrink it; the debug log measures the residual.
- **Slot penalty inert at L=2:** documented; real equity is heads-first + detour cap.
- **No tasks / no peers:** `assign` returns `{}` (existing guard).

## Params (all in metres so they compose in the cost)
| param | value | role |
|---|---|---|
| `RAY_COST_PENALTY` | 25 | BB-vs-ray crossover distance |
| `switch_margin` | 8 | head retention / hysteresis (existing) |
| tail retention | 4 (=0.5·margin) | light tail hysteresis |
| `PEER_REPULSION_W`/`SCALE` | 15 / 15 | spatial spread |
| `BUNDLE_MAX_DETOUR_M` | 20 | cluster-mate radius (≈ target spacing; scene-dependent) |
| `BUNDLE_SLOT_PENALTY` | 12 | diminishing marginal gain (L≥3 only) |
| `GEO_PRIOR_W` | 1.0 | how hard to defer to a closer peer |
| `GEO_MARGIN` | 15 | deadband: a peer must be *clearly* closer to steal ownership |
| peer TTL | 8 s | exclude past this; decay `w` toward it |

## Files
- `bid_manager.py` — `ConsensusAssigner(bundle_len)`: two-pass bundle build, geo-prior,
  age-weighted `eff_dist`/repulsion; `assign`→`{agent:[keys]}`; `my_task`→head; new
  constants; `_repulsion`/`_retention`/`_is_ray` helpers.
- `raven_nav_node.py` — `bundle_len` param; pass per-agent age weights into `assign`;
  `my_task` = head; frontier fallback gains peer-repulsion; debug log of full
  `assigned_map` + peer ages; `_publish_auction_table` owner flatten over bundles
  (`{tk: aid for aid, ks in map.items() for tk in ks}` + `queued` flag for non-head).
- `frontier_behavior.py` — peer-repulsion term in frontier selection (if not already).
- Mission-settable L: `int32 bundle_len -1` in `SemanticSearchTask.action`, map in
  `action_relay._build_semantic_search_goal`, `-p bundle_len:=` in `semantic_search_task`,
  `bundle_len: 2` in the mission goal. Default 1 (legacy) when unset.
- `test_bid_manager.py` — assign returns lists (`{k for ks in v for k in ks}`); bundle
  tests (`[BB,BB]`, `[BB,ray]`, ray-head-solo, L=1 single); geo-prior defer test;
  age-weight "dead peer → reassigned" test; equity heads-first test.

## Open items
- `BUNDLE_MAX_DETOUR_M` and `GEO_MARGIN` are scene-dependent (target spacing) — set per
  scene like the voxel thresholds, or accept one global value for the baseline.
- Validate all params in sim; the degradation sim is the harness for age-weighting.
