# Design Spec: Agent Study Prerequisites (ICRA'27 Sec. VI-C)

> Notebook entry: `notebook/001-agent-study-prereqs/` · Date started: 2026-08-03 · Branch: `airstack-paper`
>
> **Status: `WIP`** <!-- P-1/P-2 DONE; P-3..P-8 not started -->
>
> **Canonical spec:** `agent_study/agent_study_protocol.md` (in the
> PRIVATE `agent_study/` submodule; moved 2026-08-26 from the paper
> submodule, which is now Overleaf-only) is the authoritative design
> document for this work — arms A1–A4, milestone ladder, add-on tasks,
> run budget, guardrails, and the P-1…P-8 prerequisite specs live there
> and are status-tracked there. This notebook entry wraps it with
> per-prerequisite implementation status and holds the raw validation
> artifacts. Historical `ICRA_2027_AirStack_Paper/agent_study/...`
> paths in older sections refer to the pre-move location.

## 1. Problem Context

The ICRA 2027 AirStack paper's exploratory study (Sec. VI-C, "Coding
Agents as Reproducible Proxy Developers") needs its blocking
prerequisites built before trials can start — this is the paper's
longest-wall-clock item (START FIRST per `release_gate_and_tasks.tex`).
The study measures agent success on a 6-rung bring-up-to-flight ladder
across four arms (scaffolded / scaffolding-ablated / assemble-yourself /
open-loop), judged automatically by the pytest system-test harness.
Design was settled 2026-08-03; every task is a miniaturized version of
what a case-study team actually did, and judge criteria must be
calibrated against stock stack behavior, then frozen.

## 2. Proposed Implementation

One subsection per prerequisite (detailed specs in the canonical doc).

### 2.1 P-1 Waypoint-flight judge — `DONE`

`tests/system/test_waypoint_flight.py` (mark `waypoint_flight`) + the
standalone stdlib-only `tests/waypoint_checker.py`. Merged to AirStack
`develop` @ `55d9b887` (PR castacks/AirStack#378). Frozen parameters:
15 m intermediate corridor / 2.5 m final goal / 120 s per waypoint;
routes must end away from the start and default +10 m above takeoff
altitude. Judge asserts on the odometry track, not the action result.

### 2.2 P-2 Fresh planners + contract — `DONE`

`ICRA_2027_AirStack_Paper/agent_study/planners/` @ paper repo
`b41cba2` (PRIVATE until study completion): `planner_a.py`
(`route_planner_alpha`, straight-segment replan), `planner_b.py`
(`route_planner_beta`, centripetal Catmull-Rom through the same
waypoints), `CONTRACT.md` (non-AirStack topic names `~/odom_in` /
`~/planned_path`; params route/relative/spacing_m/arrival_radius_m/
replan_rate_hz).

### 2.3 P-3 A2 ablation script — `WIP`

`agent_study/runner/ablate_a2.sh`: scripted, re-runnable strip of
agent-facing scaffolding (AGENTS.md, CLAUDE.md, `.agents/`,
`docs/development/ai_agent_guide.md`) from a prepared workspace clone.
Structural validation done; functional acceptance (ablated tree builds
+ passes liveliness) pending.

### 2.4 P-4 A3 environment spec — `DONE`

`agent_study/a3_env/` (image `airstack-study-a3:v1`): ros:jazzy + PX4
v1.15.4 SITL prebuilt + Gazebo Harmonic + ros_gz + MAVROS, host
networking. Acceptance passed 2026-08-25. A3 judging: contract scripts
(prompt v3) + host-mode graph/provenance judges. Build itself surfaced
3 assemble-it-yourself frictions (gz package naming, PEP 668,
PX4 make target that launches the sim) — a preview of what A3 agents
face.

### 2.5 P-5 Seeded-defect bank — `DESIGN/TODO`

### 2.6 P-6 Arm-neutral prompts — `WIP`

`agent_study/prompts/prompts.yaml`: ladder prompt (outcome-only
wording, judge usage, iteration budget), T4/T5 stubs. Drafted;
independent wording review + hash-pinning pending (Lead).

### 2.7 P-7 Minimal launch flag — `DESIGN/TODO` (release-gate P1)

### 2.8 P-8 Trial runner + archiving — `DONE`

Acceptance met 2026-08-25: real pilot trial
(`A1_claude-sonnet-5_ladder_claude_001`) completed end-to-end with a
complete archive (score R3, $22.95, 45.6 min agent time, 6/20 judge
calls, voluntary stop). Pilot found 3 judge defects — R4 topic-name
colon bug and scoring-vs-downed-stack gap FIXED; R5 provenance
redesign pending lead decision (changes frozen judge → new campaign).

`agent_study/runner/run_trial.py` + `judge.sh` + rung checks +
`config/study_config.yaml` + `results_schema.json`. One command = one
trial: pinned workspace clone (per-arm transform), task inputs staged,
generated `./judge` shim with mechanical iteration-cap counting
(state outside the workspace; A4 cap = 0), agent launched headless
(`claude -p` stream-json transcript; mock-agent mode for pipeline
tests), wall-clock kill, final rung-scoring pass, full archive to
`runs/<trial_id>/` with schema'd `results.json`. Designed so the same
pipeline can later run as an AirStack repo-quality benchmark.

### 2.9 Amendment 1: R7/R8 completeness rungs — `WIP`

Machinery DONE (obstacle field generator, world assets both backends,
fresh-route issuance with discrimination guarantee, conformance gate,
clearance judge, R8 landing cycle, prompt v4/campaign v4, retention
tooling). BLOCKED on platform: stock droan_gl fails R7 (pillar
penetration −0.87 m) — AirStack avoidance fix required before
campaign v4 trials. See test plan (h).

### Affected packages

| Package / location | Change |
|--------------------|--------|
| `tests/` (AirStack, public) | `test_waypoint_flight.py`, `waypoint_checker.py`, marks/options/README (merged) |
| `ICRA_2027_AirStack_Paper/agent_study/` (private) | planners + contract; ops assets to come |

### Interfaces

| Topic / Param | Type | Direction | Purpose |
|---------------|------|-----------|---------|
| `~/odom_in` | `nav_msgs/Odometry` | planner sub | robot state; first msg anchors route |
| `~/planned_path` | `nav_msgs/Path` | planner pub | dense route for the local planner to follow |
| `--waypoints` / `--waypoint-tolerance` / `--goal-tolerance` / `--waypoint-timeout` | pytest opts | judge | route + frozen tolerances |

## 3. Test Plan

### (a) Waypoint judge end-to-end (P-1)

- **What is run:** `airstack test -m waypoint_flight --sim {isaacsim,msairsim} --num-robots 1 --stress-iterations 1 --gui -v`
- **What is measured:** 4-phase chain verdicts; per-waypoint closest approach, `final_goal_error_m`, `route_time_sim_s`; judge's refusal behavior on degenerate flights.
- **Pass criteria:** 4/4 phases pass on BOTH sim backends with the frozen tolerances; judge fails no-op and beeline flights.

### (b) Fresh planners standalone (P-2)

- **What is run:** pure-logic unit checks (parse/anchor/densify/spline) + live ROS 2 loopback (fake odometry in → path out) on host Jazzy, per planner.
- **What is measured:** route anchoring (position+yaw), pose density/spacing, endpoint exactness, spline-vs-straight geometry, frame propagation.
- **Pass criteria:** final pose == anchored final waypoint; spacing ≤ `spacing_m`; spline interpolates every waypoint and visibly bows (>0.3 m) between them; frame copied from odometry.

### (c) Reference integration baseline (P-2 acceptance / pre-P-8)

- **What is run:** human (not agent) integrates planner A into stock AirStack; full R4→R5→R6 flight judged by (a)'s harness.
- **What is measured:** P-1 judge verdicts on a provided-planner flight; human wall-clock (study sanity baseline).
- **Pass criteria:** R4/R5/R6 pass; baseline time logged. **SKIPPED for now per lead decision (2026-08-24)** — study proceeds without the human baseline; revisit before paper submission.

### (d) Trial pipeline validation (P-8, mock agents)

- **What is run:** `run_trial.py` in smoke mode with scripted mock agents — (1) noop agent on arm A2, (2) judge-spamming agent on A1 with cap 2, (3) noop on A4 (cap 0) — no sim, dry judges.
- **What is measured:** workspace prepared at the pinned commit; A2 ablation removes exactly the scaffolding list and nothing else; `./judge` shim counts and refuses past the cap (and refuses entirely on A4); transcript + diff + results.json archived; results.json validates against the schema; two identical smoke runs produce identical results.json modulo timestamps (repeatability).
- **Pass criteria:** all assertions above hold; refusal exit code distinct from judge failure.

### (e) Real pilot trial (P-8 acceptance)

- **What is run:** one real closed-loop A1 ladder trial with a pinned model on this machine (GPU sim), full archive.
- **What is measured:** end-to-end viability; tokens/cost/wall-clock capture; rung scoring.
- **Pass criteria:** trial completes (any score), archive complete and replayable. **DONE 2026-08-25** (score R3; 3 judge defects surfaced).

### (f) Judge v2 provenance validation

- **What is run:** the redesigned R5/R6 judge (`r5_provenance.py`) against the pilot workspace — negative controls that old-R5 wrongly passed.
- **What is measured:** discrimination — the judge must reject a publishes-but-doesn't-fly integration at the correct stage (wrong planner → node-not-found; full path → flight-following failure).
- **Pass criteria:** both negative tests fail for the documented reasons. **DONE 2026-08-25.** Positive controls also DONE same night: reference solution passes v2 R5 (0.08 m) and R6 (0.03 m) — **campaign cleared for real trials.**

### (g) Calibration batch (A1/A2 single trials, sonnet)

- **What is run:** `run_batch.sh` — sequential real trials A1 & A2 (×2 batches), full archive, scoring.
- **What is measured:** end-to-end trial viability under judge v2; harness parameters that censor agent behavior; scoring correctness; per-trial cost/time envelope.
- **Pass criteria:** trials complete and every observed anomaly is either fixed in the runner or documented as a rerun-rule flake before campaign scoring begins. **DONE 2026-08-25** — 3 harness defects found+fixed (bg-task ceiling, 100-turn cap, bottom-up scoring vs. mutually-exclusive rung states); first confirmed full-ladder R6 completion (A1#3).

### (h) Amendment 1: R7/R8 completeness rungs (implementation + validation)

- **What is run:** protocol Amendment 1 (lead-approved 2026-08-25) implementation — R7 obstacle-route rung (fixed seeded pillar field emitted as matched gz world + Isaac USDA overlay; judge-issued fresh world-frame routes with a proven discrimination property: a blind straight flight geometrically violates the 1.0 m clearance bar; conformance gate that the planner adopted the issued route; ground-frame clearance check) and R8 landing rung; prompt v4 (`./land` contract script, world-frame `ROUTE.txt` re-read per bring-up); campaign v4; scoring/schema R1–R8. Validation: geometric unit checks (clearance both directions, blind-flight violation proof) + reference-solution solvability flights.
- **What is measured:** end-to-end R7 machinery correctness; whether stock AirStack (the reference) can pass its own obstacle rung.
- **Pass criteria:** machinery validated (route issuance, conformance, clearance) AND solvability demonstrated before campaign trials. **Machinery DONE 2026-08-25/26** (six validation iterations, each one layer deeper: gz packaging, PEP 668, PX4 make target auto-launch, `~/` tilde expansion, `provided/` not container-mounted, spawn-anchored vs world frames). **Solvability OPEN — stock stack FAILS R7** (followed all checkpoints but penetrated a pillar, min clearance −0.87 m): campaign v4 blocked on fixing droan_gl local avoidance (1:1 deviation-vs-progress cost; forward-only stereo FOV). Artifacts: `results/h-amendment-r7r8/`.

