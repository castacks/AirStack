# Results Summary: Agent Study Prerequisites (P-1, P-2)

> Spec: [`../design_spec.md`](../design_spec.md) · Date: 2026-08-04 · Commits tested: AirStack `55d9b887` (develop), paper repo `b41cba2`

## (a) Waypoint judge end-to-end (P-1)

**Setup:** `airstack test -m waypoint_flight --sim {isaacsim,msairsim} --num-robots 1 --stress-iterations 1 --gui -v`, local RTX 4090, final frozen config (open 30 m square climbing +10 m; 15 m corridor / 2.5 m goal / 120 s per waypoint).

| Metric | Isaac Sim | ms-airsim (Blocks) | Pass criterion | Pass? |
|--------|-----------|--------------------|----------------|-------|
| Phases passed | 4/4 | 4/4 | 4/4 | ✅ |
| Corner closest approaches | 5.67 / 5.72 m | 5.93 / 5.67 m | ≤ 15 m each, in order | ✅ |
| `final_goal_error_m` | 0.28 | 0.89 | ≤ 2.5 m | ✅ |
| `route_time_sim_s` | ~90 | 88.9 | within budget (360 s) | ✅ |
| Landing `final_altitude_m` | ≈ 0 | −0.149 | < 0.5 m | ✅ |

Judge refusal behavior (the reason the judge asserts on the odometry
track, not the action result): a closed-loop route made `NavigateTask`
return `success: true` **without flying** — the checker failed it
(1/4 waypoints); a goal-beeline flight was failed at 0/3 waypoints
(closest approaches 8.3 / 8.9 m ≫ then-tolerance). Both refusals
occurred on real flights during calibration.

![Isaac Sim mid-route](a-waypoint-judge/waypoint_flight_isaac.jpg)
![ms-airsim Blocks mid-route](a-waypoint-judge/waypoint_flight_msairsim_blocks.jpg)
![Foxglove during the route](a-waypoint-judge/waypoint_flight_foxglove.png)

**Interpretation:** the stack's navigation contract is "reach the goal
precisely, follow the corridor loosely" (stock `droan_gl` cost =
`deviation − path_distance` cuts corners ~4–7 m by design); tolerances
were calibrated to that contract and then frozen. Corner-cut depth
agreed across backends (~5.7 m both sims) — same planner code, different
physics. Raw artifacts: `a-waypoint-judge/{isaac,msairsim}_metrics.json`
and `_summary.txt`. Findings filed on the PR: droan_gl dies on
empty-frame goals (uncaught tf2 exception); PX4↔AirSim bring-up flake
(1/4 runs); one transient false ON_GROUND mid-descent (1/4, did not
reproduce).

## (b) Fresh planners standalone (P-2)

**Setup:** host ROS 2 Jazzy (`/usr/bin/python3`), `ROS_DOMAIN_ID=77`; unit checks on module functions, then each planner run standalone with `route:="10,0,5; 10,10,5"` and fake odometry published at (1, 2, 0.5), yaw 0.

| Check | planner_a (alpha) | planner_b (beta) | Pass? |
|-------|-------------------|------------------|-------|
| Route parse / 90° yaw anchoring | exact | exact | ✅ |
| Dense path spacing ≤ 1 m, endpoints exact | 21 poses, max gap 1.0 m | — | ✅ |
| Spline interpolates every waypoint | — | < 1e-6 m at each | ✅ |
| Spline bows from straight path | — | 0.74 m max | ✅ |
| Live loopback: poses published | 23 | 23 | ✅ |
| Live loopback: final pose = anchored final waypoint | (11, 12, 5.5) exact | (11, 12, 5.5) exact | ✅ |
| Frame propagated from odometry | `map` | `map` | ✅ |

**Interpretation:** both planners satisfy CONTRACT.md standalone with no
AirStack dependencies. One implementation bug was caught by the bow
check: duplicated-endpoint spline padding collapsed centripetal knot
spacing to straight lines; fixed with reflection padding. Raw artifacts:
`b-fresh-planners/path_{a,b}.yaml`, `planner_{a,b}.log`.

## (c) Reference integration baseline

**SKIPPED per lead decision (2026-08-24)** — the study proceeds without
the human baseline; revisit before paper submission. (Was: human
integration of planner A, R4→R5→R6 under the (a) judge, wall-clock
logged.)

## (d) Trial pipeline validation (P-8, mock agents)

**Setup:** `run_trial.py --smoke` (no sim, dry judges, no submodules),
AirStack pinned @ `e5889c06`, three scripted agents; assertions in
`agent_study/runner/pipeline_selftest.py`. Quantitative results:

| Trial | Cap | Judge invocations | Refusals | Agent exit | Pass? |
|-------|-----|-------------------|----------|-----------|-------|
| A2 + noop (×2 runs) | 20 | 0 | 0 | 0 | ✅ |
| A1 + judge-spammer (5 calls) | 2 | **2 executed** | **3 refused (exit 75)** | 0 | ✅ |
| A4 + judge-spammer (5 calls) | 0 (open-loop) | 0 | 5 refused, open-loop message | 0 | ✅ |

Pipeline invariants verified:

- Workspace cloned fresh at the pinned commit (`e5889c06…`) every trial.
- A2 ablation removed exactly 38 files (AGENTS.md, CLAUDE.md, `.agents/`
  tree — manifest archived); README/docs/mkdocs/tests survive; provided
  planners + contract staged; `./judge` shim present.
- Agent-created files captured in `workspace_diff.patch`; transcript and
  stderr archived; `results.json` carries `config_sha256`,
  `prompt_sha256`, `airstack_commit`, `runner_commit`.
- **Repeatability:** two independent A2 smoke trials byte-identical on
  every invariant results field and on the ablation manifest
  (config `a163b9f0…`, prompt `00964c05…`).
- Refusal exit code (75) is distinct from judge failure (1), so agents
  and analysis can tell "budget exhausted" from "test failed".

**Interpretation (qualitative):** the smoke tier caught one real runner
bug before any tokens were spent — mock-agent script paths resolved
against the workspace cwd instead of the caller's, so the agent silently
never ran; fixed by resolving to absolute + existence check. This is
exactly the class of pipeline error that would have invalidated real
trials. Residual known gap: the cap counts official `./judge` calls;
an agent invoking the underlying pytest harness directly is uncounted
but visible in transcript/diff (audit rule documented in
`agent_study/README.md`). Raw artifacts: `d-trial-pipeline/`.

## (e) Real pilot trial (A1, claude-sonnet-5)

**Setup:** `run_trial.py --arm A1 --model claude-sonnet-5 --trial-index 1`,
launched 2026-08-25 06:35 UTC; AirStack pinned @ `e5889c06`, frozen
prompts (`00964c05…` composed), cap 20, wall-clock cap 4 h. Full archive
in `agent_study/runs/A1_claude-sonnet-5_ladder_claude_001/`.

| Metric | Value |
|--------|-------|
| Official score (`highest_rung_passed`) | **R3** (R4 fail gates R5/R6) |
| Judge invocations | 6 / 20 (0 refusals) |
| Agent wall-clock | 45.6 min (voluntary stop; not timed out) |
| Total trial wall-clock (incl. scoring) | 62.5 min |
| Tokens out | 168,257 |
| Cost | $22.95 |
| Workspace diff | 43 KB (planners staged into the stack's launch config) |

**Judge-call sequence (behavioral):** R4-fail (probe) → R1, R2, R3 pass →
R5 pass → R6-fail (confirm) → voluntary stop with a written
justification: remaining budget "would only burn without new
information" because it had root-caused R4's failure to a harness bug.

**Interpretation — the pilot did exactly its job. Three judge defects
found, two of them BY the subject agent:**

1. **R4 check bug (agent-diagnosed, verified, FIXED):** the awk in
   `r4_planner_check.sh` kept the trailing colon from `ros2 node info`
   output, so the topic-echo always got an invalid topic name — any
   correct integration would fail R4. The agent read `ros2node`'s
   source to prove version-stable formatting. Reproduced offline;
   fixed (`sub(/:$/,"",$1)`).
2. **Scoring-pass R4/R6 ran against a downed stack (runner-observed,
   FIXED):** graph-level checks executed 1 s after R3's teardown.
   Scoring now brings the agent's stack up, polls the check (up to
   5 min), and tears down (`scoring_graph_check` in `judge.sh`;
   R6 precheck split into `r6_precheck.sh`).
3. **R5 provenance flaw (agent-exploited, OPEN — needs lead decision):**
   `waypoint_flight` dispatches the route itself via NavigateTask, so
   R5 passes with NO planner in the flight path. The agent noticed,
   passed R5 without wiring its planner into the flight, and argued
   this satisfies R4's letter. It also justified NOT publishing to
   `global_plan` by identifying a plausible real stack race: droan_gl's
   `global_plan_callback` and its NavigateTask handler write the same
   internal object with no task-active guard (candidate AirStack
   issue). Fix requires redefining the R5 judge: judge only commands
   takeoff, flight must follow the PLANNER's published route, odometry
   checked by `waypoint_checker` against that route. This changes the
   frozen judge → new campaign; decision pending.

**Score adjudication:** R3 stands under the frozen judge. Under a
colon-fixed R4 the agent's integration would likely have passed R4
(node up + publishing), but R5-with-provenance would have failed it —
the planner's route never drove a flight. The official result is
recorded as-is; this pilot is pipeline validation, not campaign data.

**Cost projection:** at ~$23 + ~1 h per trial, the protocol's ~60-run
ladder ≈ $1.4k + ~60 GPU-hours — tractable, and validates the
protocol's "cost is GPU-minutes" framing.

## (f) Judge v2 (provenance) validation

**Setup:** lead approved the R5/R6 provenance redesign 2026-08-25
(campaign `2026-08-icra27-vic-v2`): the judge brings the stack up,
requires the expected planner alive (R6: forbids the other), captures
the planner's OWN published route, enforces geometry gates (≥60 m,
≥2 heading changes — anti-trivial-route), commands ONLY takeoff, and
judges the flown odometry against the planner's checkpoints
(`r5_provenance.py`). Negative tests against the pilot workspace
(whose integration passed old-R5 without flying):

| Test | Expected | Result | Pass? |
|------|----------|--------|-------|
| v2 R5 vs pilot workspace | FAIL (workspace's final state runs planner *beta*, not alpha) | FAIL: "planner route_planner_alpha not found" | ✅ |
| v2 R6 vs pilot workspace (full path) | FAIL at flight-following | route captured (94.4 m, 2 turns, 8 checkpoints) → takeoff → hover → FAIL: "flown track did not follow the planner route in order" | ✅ |

**The v2 judge now refuses exactly the behavior old-R5 wrongly
accepted.** Debugging findings along the way (both fixed): the judge's
raw `airstack up` needed the harness's SIM_CONFIG extras —
`PLAY_SIM_ON_START=true` is load-bearing (without it Isaac loads but
never plays, PX4/MAVROS never link; a great agent-legibility datapoint:
that knowledge lives only in the pytest harness); px4-ready timeout
raised to 420 s for fresh-workspace first builds.

**Positive controls (2026-08-25, ~03:00): PASSED — task solvability
under judge v2 proven.** Reference solution
(`agent_study/reference_solution/`: pilot-style launch integration
with planner output wired onto the real `global_plan` + a mission-glue
node that flips the trajectory controller to ADD_SEGMENT once takeoff
settles) applied to a fresh pinned workspace:

| Positive control | Result |
|------------------|--------|
| v2 R5 (planner alpha) | PASS — 7 checkpoints in order, goal error **0.08 m** |
| v2 R6 (swap to beta, alpha gone) | PASS — 8 checkpoints in order, goal error **0.03 m** |

The R6 swap in the reference is a single launch-arg change — direct
evidence for the C1 module-swap claim. Two more environment gaps found
and fixed in the runner along the way: fresh clones lack the untracked
Isaac Nucleus credentials (`omni_pass.env` — now provisioned by
`prepare_workspace`, and notably the pilot AGENT solved this same
preflight itself); first bring-up compiles ros_ws in-container (px4
timeout accommodates). Verdicts: `f-judge-v2/r{5,6}_positive_verdict.json`.

## (g) Calibration batch (2026-08-25, A1/A2 × sonnet, judge v2)

**Setup:** `run_batch.sh`, sequential trials, campaign `vic-v2`,
AirStack @ `e5889c06`. Batch 1 = trials #2; batch 2 = trials #3 after
runner fixes. Raw artifacts: `g-calibration-batch/`.

| Trial | Agent time / cost | Judge calls | Official → adjudicated score | Notes |
|-------|-------------------|-------------|------------------------------|-------|
| A1 #2 | 26 min / $6.05 | 0 | null | censored: bg-task ceiling killed backgrounded bring-up; scoring bring-up also broken (unresolved; superseded) |
| A2 #2 | 17 min / $5.35 | 0 | R2 | censored: same ceiling; R3 "fail" = landing-timeout flake |
| A1 #3 | 47 min / ~$11 | 6/20, all pass | **R3 → R6 (full ladder)** | first full-ladder completion; re-score of final state confirmed R6 (goal error 1.22 m) |
| A2 #3 | 18 min / $6.71 | 0 | null | censored: CLI 100-turn cap hit before first judge call; scoring R1 "fail" = stability-poll flake (7/8 passed) |
| A2 #4 | 30 min / $6.66 | 0 | **R5** (top-down: R6 fail, R5 pass) | VALID (fixed runner): working flying integration, but no judge use, no swap, no report |
| A3 #1 | 80 min / $12.09 | 8/20 | **R6 (full ladder)** — adjudicated by shim re-score (goal 1.25 m) | VALID (prompt v3): assembled PX4+gz+MAVROS system from bare parts; official scoring nulled by a runner env bug (fixed: scoring now runs through the workspace judge shim) |

**Valid trials only (one per arm, sonnet-5, adjudicated):**

| | **A1 #3 (scaffolded)** | **A2 #4 (ablated)** | **A3 #1 (bare parts)** |
|---|---|---|---|
| Adjudicated score | **R6 — full ladder** | R5 | **R6 — full ladder** |
| Agent time | 47 min | 30 min | 80 min |
| Cost | ~$11 | $6.66 | $12.09 |
| Judge calls | 6/20, methodical | 0/20 | 8/20, methodical |
| Planner swap (R6) | completed | not attempted | completed |
| AGENT_REPORT.md | yes | no | yes |
| Final goal error (re-score) | 1.22 m | — (R6 fail) | 1.25 m |

Caveats: n=1 per arm; A1/A2 ran under prompt v2, A3 under v3
(contract-script lines added — routes and judge semantics otherwise
identical); all three predate Amendment 1, so scores are on the
original R1–R6 ladder.

**Harness defects found & fixed by this batch (all pre-campaign):**

1. Headless CLI kills backgrounded tasks at 600 s → both #2 agents
   clipped before any judge call (`CLAUDE_CODE_PRINT_BG_WAIT_CEILING_MS=0`).
2. CLI default 100-turn cap censored A2 #3 mid-work (`--max-turns 1000`).
   Turn count is a measured variable — the cap must never bind.
3. **Bottom-up scoring is unsound for this ladder:** R4/R5 (alpha
   running) and R6 (alpha gone) are mutually exclusive in one final
   state, so any agent completing the swap scored ≤R3. Fixed: top-down
   scoring (first passing rung from R6 downward sets the score; lower
   rungs implied). A1 #3 adjudicated R6 under this rule via preserved-
   workspace re-score.
4. `DISPLAY` stripped from trial env (Foxglove sign-in popups on the
   operator's screen; GCS launches a sign-in-gated desktop app —
   flagged separately for the clone-and-run release gate).

**Qualitative highlights:** A1 #3's integration is the most elegant
solution seen — it read `droan_gl` source, identified that only
NavigateTask flips the controller out of hover, and built a
`mission_trigger` node that sends ONE NavigateTask goal seeded with
the live planner path (vs. the reference solution's mode-service
glue); proper ament package, canonical module-launch convention,
mkdocs nav entry, 6/6 judge calls used with zero waste. Both A2 agents
independently produced well-formed colcon packages without AGENTS.md —
repo structure alone appears to teach file conventions; what the
ablated arm lacked showed up as turn burn (A2 #3 hit the cap before
its first judge call). Flake ledger grew: landing-action 55 s timeout;
liveliness stability poll transient.

**Campaign readiness:** runner is now stable through a full
agent-pass-score cycle, including top-down scoring exercised for real
(A2 #4: R6 fail → R5 pass → implied below).

**Harness defect #4 (A3 trial): scoring env inconsistency.** The
scoring pass called `judge.sh` directly without the shim's `STUDY_*`
exports, so A3 scoring ran the AirStack code path and instantly nulled
all six rungs (`./airstack.sh: No such file`). Fixed: scoring now
invokes the workspace's own `./judge --scoring <rung>` — agent judge
and scoring judge are byte-identical by construction. A3 #1 re-scored
through the shim: R6 PASS, goal error 1.25 m.

**A3 #1 integrity audit (the arm where the agent builds the odometry
source):** no judge-file tampering (git clean); judged odometry is
50 Hz `/mavros/global_position/local` — stock MAVROS fed by PX4's EKF,
not agent-authored data; flight signatures physical (106 s flights,
speeds ≤8.6 m/s, takeoff overshoot to 10.27 m); and the adjudicating
re-score was a fresh judge-controlled flight. Architecture (from its
report): `mission_manager.py` offboard state machine streaming 20 Hz
setpoints, gz clock bridge, MAVROS on SITL mavlink, planner selection
file, and a self-diagnosed ROS-domain-mismatch workaround.

**First valid A1-vs-A2 datapoint (n=1 each, sonnet):**

| | A1 #3 | A2 #4 |
|---|---|---|
| Adjudicated score | **R6 (full ladder)** | **R5** |
| Judge calls | 6/20, methodical | 0/20 |
| AGENT_REPORT.md | yes | no |
| Planner swap (R6) | completed | not attempted/failed |
| Core integration | flies | flies |

Early shape of the legibility gap: the ablated agent's *engineering*
was sound (its integration flies the provenance judge), but the
*workflow* behaviors — self-verification via the official judge,
completing the swap, writing the report — were absent. Notably ZERO
judge usage across all three A2 agents vs. methodical use by A1 #3,
despite identical judge instructions in the arm-neutral prompt footer.
Hypothesis for the campaign: scaffolding buys verification discipline
and task completion more than raw capability. n=1; do not over-read.

## (h) Amendment 1: R7/R8 completeness rungs — implementation + validation

**Built (2026-08-25 evening, lead-approved):** protocol amendment;
prompt v4 (contract scripts + `./land`, world-frame `ROUTE.txt` re-read
per bring-up); seeded pillar-field generator (layout JSON + gz world +
Isaac USDA overlay, same 14 pillar coordinates); fresh-route generator
with a PROVEN discrimination property (a blind straight-line flight
provably violates the 1.0 m clearance bar — verified geometrically);
clearance checker (validated both directions); R7/R8 judge wiring for
all arms; conformance gate (planner's published path must adopt the
judge-issued route).

**Validation iterations (each failure one layer deeper — themselves
integration-scaffolding evidence):** gz package naming → PEP 668 →
PX4 make target auto-launching the sim → wrapper tilde-expansion of
`~/` remaps → `provided/` not mounted into the robot container (only
`stacks/` is; route mirrored there by the judge) → frame ambiguity
(spawn-anchored routes vs world-frame pillars; resolved by making all
routes ABSOLUTE world-frame, planner `relative:=false`).

**Final validation flight — the pre-registered outcome:**

| Check | Result |
|-------|--------|
| Route issuance + stacks mirror | ✅ |
| Conformance (planner adopted issued route) | ✅ |
| Checkpoints followed in order | ✅ |
| **Obstacle clearance** | ❌ **min −0.87 m — flew through a pillar** |

**Verdict: R7 machinery validated and discriminating; STOCK AIRSTACK
FAILS ITS OWN OBSTACLE RUNG.** Campaign v4 is blocked on fixing the
platform's local avoidance (droan_gl's 1:1 deviation-vs-progress cost
and forward-only stereo FOV — both previously logged as findings).
The benchmark caught a real quality gap before any paper claim
depended on it — this is the repo-quality-benchmark concept working
as designed, and strong VI-B material.

## Overall Verdict

| Spec section | Verdict |
|--------------|---------|
| (a) Waypoint judge end-to-end | ✅ both backends, tolerances frozen |
| (b) Fresh planners standalone | ✅ both planners |
| (c) Reference integration baseline | ⏭️ skipped per lead (2026-08-24) |
| (d) Trial pipeline validation | ✅ all assertions; 1 bug found+fixed |
| (e) Real pilot trial | ✅ completed end-to-end; 3 judge defects found (2 fixed, R5 redesign → (f)) |
| (f) Judge v2 provenance validation | ✅ negative AND positive controls pass |
| (g) Calibration batch | ✅ 4 harness defects fixed; A1 R6, A2 R5, A3 R6 (n=1 each) |
| (h) Amendment 1 (R7/R8) | ✅ machinery; ❌ solvability — **stock stack fails R7; campaign v4 blocked on avoidance fix** |

**Known limitations:** (a) validated at num_robots=1 only; multi-robot
judging deferred to T4. A3 environment (P-4) not implemented — A3 trials
error out cleanly. P-6 prompts are DRAFT pending independent wording
review. Smoke trials skip submodule init and real judges; the real pilot
trial (e) is the remaining P-8 acceptance step. msairsim bring-up flake
(~1/4) means failed bring-ups must be rerun, not scored.
