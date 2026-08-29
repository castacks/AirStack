# Research Strategy — AirStack / ICRA 2027

> **How to use this file.** Notebook entries (`notebook/NNN-*/`) are
> single-topic evidence;
> [`paper_positioning.md`](../ICRA_2027_AirStack_Paper/paper_positioning.md)
> is the deep framing record; **this file is the big picture** — the
> current strategic direction, the campaigns that group the notebook
> entries, and the history of how findings updated the direction.
> **Read §Current direction and §Campaigns before starting or advising
> on any notebook entry.**
>
> Two mutation regimes:
>
> - The top three sections (§Current direction, §Strategic choices in
>   force, §Campaigns) are **edited in place** — they describe *now*.
>   Keep each to ~one screen; if one outgrows a screen, push detail
>   down into positioning.md or a notebook entry and link to it.
> - §Direction log is **append-only, newest first** — the history IS
>   the content. Never rewrite or prune it; correct an old entry only
>   by appending a newer one.
>
> Last updated: 2026-08-28

## Current direction

The thesis: AirStack is a **full-stack reference architecture with a
validated sim-to-real workflow** that measurably speeds researchers
from idea to hardware demonstration. Deliverable: an ICRA 2027 systems
paper (deadline **2026-09-15**) built on three claims — C1 architecture,
C2 sim-to-real workflow, C3 enablement — plus the released, CI-tested
artifact. Claims, named concepts, and voice rules:
[positioning.md §Core positioning](../ICRA_2027_AirStack_Paper/paper_positioning.md).

Evidence plan: the **five case studies** answering a fixed six-question
instrument are PRIMARY; Sec. VI (sim fidelity, defect mining, agent
study, threats) is SUPPORTING
([positioning.md §Evidence hierarchy](../ICRA_2027_AirStack_Paper/paper_positioning.md)).

Current phase (as of 2026-08-28 20:00): **campaign v6 TRIALS ARE
RUNNING** (entry [011](011-agent-study-v6-trials/design_spec.md)).
The preliminary trial arc caught two headless-environment defects
(R8-implication scoring bug; `claude -p` session truncation), both
fixed runner-side before any trial was scored; the first scored trial
is a **full-ladder R8 pass** (A1/claude-sonnet-5, 9/20 judge calls,
2 h 22 m, $13.31) — arm separation must now come from A2/A3/A4.
Round-robin continues serially on the RTX 5090 box. v6 foundation:
MIGHTY swap (entry [010](010-mighty-local-planner-module/design_spec.md),
DONE, R7 solvability 5/5), pin `study/mighty-swap` @ `961fb9e1`,
asm_mighty v0.1.0. In parallel: prose-ifying `main.tex`
section by section, and the release gate (7 paper-blocking items; ~63
tasks still `\task{}` in `release_gate_and_tasks.tex` as of
2026-08-27).

Hard calendar gates: **2026-09-15** ICRA submission; case-study
interviews and release-gate items must land before the final claims
audit ([positioning.md §Release gate](../ICRA_2027_AirStack_Paper/paper_positioning.md)).

## Strategic choices in force

1. **Pitch = reference architecture + validated sim-to-real workflow;
   "boilerplate" is banned.** Not "best autonomy stack." Decided
   2026-07-20 (outline commit).
   [positioning.md §Core positioning](../ICRA_2027_AirStack_Paper/paper_positioning.md).
2. **Case studies are PRIMARY; the agent study is exploratory — the
   paper must stand without it.** Decided 2026-08-03 after senior-PhD
   internal review.
   [positioning.md §Evidence hierarchy](../ICRA_2027_AirStack_Paper/paper_positioning.md).
3. **The CUT list stays cut**: no time-boxed developer exercise, no
   historical counterfactual dressed as data (one labeled qualitative
   remark max). Decided 2026-08-03, stated in the paper as a methods
   position.
4. **Simulators are complementary parallel backends — never "legacy"
   in paper text.** Isaac = physics/sensor fidelity; AirSim = large
   photoreal scenes. Decided 2026-07-20.
   [positioning.md §Simulator framing](../ICRA_2027_AirStack_Paper/paper_positioning.md).
5. **Voice: measurements only** — no "seamlessly/easily/rapidly", no
   headlined "N× faster"; everything checkable against artifacts.
   July 2026. [positioning.md](../ICRA_2027_AirStack_Paper/paper_positioning.md).
6. **Agent-study integrity: fix the stack, never the benchmark.** The
   pillar field + judge stay frozen once calibrated; platform defects
   they expose get fixed in AirStack. Study assets (planners, prompts,
   judge, reference solution) stay in the PRIVATE `agent_study/` repo
   until the study concludes. In force 2026-08-25/26.
   [008 design spec](008-droan-gl-r7-avoidance-fix/design_spec.md),
   [agent_study/HANDOFF.md](../agent_study/HANDOFF.md).
7. **The study proceeds without a human reference baseline** (test
   plan (c) skipped) — lead decision 2026-08-24; revisit before
   submission.
   [007 design spec §3(c)](007-agent-study-prereqs/design_spec.md).
8. **Release gate = claim–reality consistency, not feature
   completeness.** Everything beyond the seven gate items → tabled to
   Limitations/Future Work.
   [positioning.md §Release gate](../ICRA_2027_AirStack_Paper/paper_positioning.md).

## Campaigns

All entries live under `notebook/` (tracked on `airstack-paper`):
001–006 are the develop-track platform/docs work (merged in from the
former local-only `notebook_docs/` on 2026-08-27; 005's raw footage
stays local-only per `.gitignore`), 007–008 the agent study. Statuses
come from the entries' own labels.

| Campaign | Goal | Entries | Status |
|---|---|---|---|
| Agent study (paper Sec. VI-C) | Run the four-arm proxy-developer study on the bring-up-to-flight ladder, judged by the pytest harness | [007-agent-study-prereqs](007-agent-study-prereqs/design_spec.md), [008-droan-gl-r7-avoidance-fix](008-droan-gl-r7-avoidance-fix/design_spec.md), [009-droan-gl-yaw-sweep-unstick](009-droan-gl-yaw-sweep-unstick/design_spec.md), [010-mighty-local-planner-module](010-mighty-local-planner-module/design_spec.md), [011-agent-study-v6-trials](011-agent-study-v6-trials/design_spec.md) | 007 `WIP` (P-5, P-7 open, non-blocking); 008 `DONE` (verdict (c) ❌ under frozen config); 009 `DONE` (sweep works, R7 still 0/10); 010 `DONE` — MIGHTY swapped in as `asm_mighty` v0.1.0; R7 reference solvability 5/5 under frozen v6; 011 `WIP` — TRIALS RUNNING, 1 scored (A1/sonnet = R8 full ladder), runner defects fixed pre-scoring (see log) |
| Paper writing & positioning | Sec. I–V prose, case-study interviews, figures | — (lives in the `ICRA_2027_AirStack_Paper/` submodule; no notebook entries) | Related Work + Design Principles prose done 2026-08-03; interviews not recorded anywhere yet |
| Release gate / v1.0 readiness | The seven paper-blocking items (clone-and-run, verified hardware path, …) | — (no notebook entries yet) | ~63 open `\task{}` vs 1 `\done{}` in `release_gate_and_tasks.tex` |
| Modular AirStack (RFC #379/#380) | Monolith → modules, stacks, fleets — built to support the paper's modularity positioning: module swapping (C1) and easy upstreaming of features from forked projects (lead, recorded 2026-08-27) | [002-rfc-modular-airstack](002-rfc-modular-airstack/design_spec.md) | `WIP` per its header (impl merged to develop 2026-08-24 as PRs #388–#396; release mechanics, module CI tags, P3 dispatch smoke open) |
| Launch & CLI developer experience | Kill sim-launch friction — intent flags, readiness signal, scene selection, command groups — purely for developer usage and adoption (lead, recorded 2026-08-27) | [001-pre-rfc-workflow-cleanup](001-pre-rfc-workflow-cleanup/design_spec.md), [003-scene-flag](003-scene-flag/design_spec.md), [004-osmo-command-group](004-osmo-command-group/design_spec.md) | 001 `DONE`; 003 `DONE`; 004 results PASS (no header status) |
| Docs & public presentation | Accurate, Diátaxis-organized docs site and a demo-video landing page — likewise for developer usage and adoption (lead, recorded 2026-08-27) | [005-landing-page-demo-video](005-landing-page-demo-video/design_spec.md), [006-diataxis-docs-audit](006-diataxis-docs-audit/design_spec.md) | 005 deliverables committed 2026-08-24/25; 006 `DONE` — overhaul merged to develop 2026-08-25 as PR #404 (`66e62efd`) |

## Direction log

Append-only, newest first. Append on any strategic update — a claim
strengthened or weakened, a descope, a new campaign, a priority shift,
a standing choice flipped. Routine feature completions that don't move
the strategy get no entry. Format: `### YYYY-MM-DD — what happened`,
answering *what we learned or decided, what it changed, link to the
evidence*.

### 2026-08-28 (20:00) — v6 trials STARTED; two headless-environment defects fixed pre-scoring; first scored trial = full-ladder R8

Trials began (entry [011](011-agent-study-v6-trials/design_spec.md)).
The preliminary sanity trial caught, before anything was scored:
(1) an R8-implication scoring bug — R8 (bare takeoff+land) passed on a
stock bring-up and top-down scoring implied R1–R7; the implication
chain now tops out at R7, R8 scored only after an R7 pass; (2) two
`claude -p` session-truncation vectors (ScheduleWakeup/Monitor promise
re-invocation print mode never delivers; background Bash killed ~5 s
after the final result) — session-persistence tools denied and
background Bash disabled with 20/60-min foreground timeouts. Runner
commits `a56441b`+`41ca11e`; agent-facing judges/prompts/config
untouched (config_sha256 stable), so the campaign pin stands. First
scored trial: **A1/claude-sonnet-5 = R8, the study's first full-ladder
pass** (9/20 judge calls, 2 h 22 m, $13.31, single R8 retry). Strategic
read: with a modern local planner, the extended ladder no longer
separates arms at A1's ceiling — the dose–response hypothesis now
rides on A2/A3/A4 (and on cost/iteration metrics, which remain
informative regardless). Also decided (lead): `agent_study/HANDOFF.md`
retired; notebook + this file are the running record.

### 2026-08-28 (07:00) — MIGHTY swap COMPLETE; R7 reference-solvable 5/5; campaign v6 ready for trials

The swap decided this morning (next entry) was implemented, validated,
and frozen in one session (entry
[010](010-mighty-local-planner-module/design_spec.md), DONE). What
landed: `asm_mighty` v0.1.0 (PRIVATE castacks repo until the study
concludes) — MIGHTY + acl-mapping Jazzy-ported and bridged onto the
untouched NavigateTask / trajectory_controller seams; `full_mighty`
reference stack; campaign v6 frozen (pin `study/mighty-swap` @
`961fb9e1`; R7 budget 240 s; practice/eval layout split with
judge-time-only eval-scene staging). Evidence: practice field 7/7;
**official frozen-config judged batch 5/5** (goal errors 0.01–0.15 m,
min clearances 1.59–1.65 m). What it changed: (a) the v5 blocker is
closed — trials may start; (b) C1 gains a real module-swap
demonstration (the swap is one include + one pin, and the module
workflow survived a planner-scale stress test with 5 usability
findings); (c) a transferable systems lesson for the paper: a
timeline-open-loop planner behind a tracking controller needs
vehicle-anchored replanning + receding-horizon override handoff — ten
distinct integration defects were surfaced BY the judged solvability
protocol itself and fixed (notebook 010 §(e)); (d) diffaero remains
postponed (depth-only hardware track). Watch item: asm_mighty must
flip public at study conclusion (index/catalog registration deferred).

### 2026-08-28 — local planner swap DECIDED: MIGHTY as external module `asm_mighty`; diffaero postponed; campaign v6 bundles everything

The pending swap decision (previous entry) resolved the same day. Two
candidates were researched (repos + papers): **MIGHTY** (mit-acl,
RA-L 2026, arXiv:2511.10822 — A*/safe-corridor/Hermite-spline NLP,
Gurobi-free, ROS 2 Humble, BSD-3, actively maintained, PX4-proven) and
**diffaero** (BIT, arXiv:2509.10247 — differentiable-sim training
framework for depth-only visuomotor policies). Lead decided:
**MIGHTY** — its persistent map + omnidirectional lidar input +
hard-margin corridors address exactly droan_gl's two terminal R7
failure modes (vote-blocked pockets, FOV-hole near-contacts), while
diffaero would re-import the same reactive/memoryless/narrow-FOV
failure class, bypass the trajectory-controller safety seam, and carry
sim-to-sim transfer risk against the deadline. **diffaero is postponed**
(future candidate for the hardware depth-only story — no heavy lidar;
overnight-trainable on one GPU). Delivery decision: `asm_mighty` as an
**external module repo, explicitly to stress-test the RFC #379 module
integration workflow** (feeds Modular AirStack campaign + C1
module-swap evidence). Confirmed: the swap folds into **campaign v6**
together with the queued judge changes and the restructure re-pin —
one recalibration. Evidence:
[010 design spec](010-mighty-local-planner-module/design_spec.md).

### 2026-08-28 — droan_gl hit its R7 ceiling; local-planner swap recommended; R7 confirmed as a composition rung

The agent study's R7 blocker resolved into a strategic finding
(evidence: [008](008-droan-gl-r7-avoidance-fix/design_spec.md) addendum
+ [009](009-droan-gl-yaw-sweep-unstick/design_spec.md)): the frozen
droan_gl config fails R7 0/5 with the budget question moot; a yaw-sweep
unstick (active-perception pause recovery, implemented + validated in
10 judge flights) eliminates the dominant absorbing-hover mode with
zero crashes but still 0/10 — residuals (vote-blocked pockets,
0.01–0.26 m near-contacts from sensor FOV holes) are architectural to
the 2018 reactive design. Per the lead's pre-registered criterion
(2026-08-27: "if the yaw heuristic doesn't work, swap planners"), the
recommendation is a modern map-based local planner as the R7
prerequisite — decision pending. Standing choices recorded the same
session: R7 stays a **composition rung** (the platform's local layer
must work; agents integrate against it — feeds C1), the R7 scene
layout becomes answer-key material with a practice/eval (val/test)
layout split, and the lead's positioning bar is explicit: "the
reference stack that lets you focus on your global planner and not
worry about local planning" requires a reliably-performing local
planner.

### 2026-08-27 — airstack-paper rebased onto latest develop; restructure flagged as an agent-study variable

`airstack-paper` was rebased onto develop @ `d4a04df6`, so the paper
branch now sits on the fully restructured platform: the RFC #379/#380
modular architecture (merged to develop 2026-08-24) plus develop's
newest docs/sim commits — including the Diátaxis overhaul, whose merge
(PR #404, 2026-08-25, `66e62efd`) closes the "merge not recorded" gap
previously flagged in §Campaigns. **Lead's note (2026-08-27): the
restructure will impact the agent study and may impact its numbers.**
The study's pinned workspaces, reference solution, and calibrated
judge budgets predate the modular restructure; re-pinning onto
post-restructure code changes the task surface agents face (stack
entry launch files instead of layer bringups, the module CLI, the
reorganized docs), so results measured across the pin change cannot
be pooled — per the existing campaign-id-bump rule, a re-pin means a
new campaign id and recalibration where budgets were tuned on the old
tree. Evidence: [agent_study/HANDOFF.md](../agent_study/HANDOFF.md),
[007 design spec](007-agent-study-prereqs/design_spec.md),
[008 results](008-droan-gl-r7-avoidance-fix/results/results_summary.md).

### 2026-08-27 — strategy layer created; log seeded retroactively; notebook consolidated

This file created as the layer between single-topic notebook entries
and positioning.md. All entries below were reconstructed today from
paper_positioning.md, the notebook entries' own dates and statuses,
agent_study/HANDOFF.md, and `git log`; gaps are flagged inline rather
than guessed. Same day: the local-only `notebook_docs/` (develop-track
platform/docs entries 001–006) was merged into `notebook/` and
committed, and the agent-study entries renumbered 001→007, 002→008 so
the sequence follows the architectural work. The lead also recorded
the motives behind the platform campaigns (see the 2026-08-19→25
entry and §Campaigns).

### 2026-08-27 — R8 passes; campaign v5 frozen config has caveats

Under the frozen SAFE config the fixed stack passes R6 (goal error
0.07 m) and R8 (landed at rest), and completes R7 routes in-order and
in-budget but can stall short on adversarial route draws — no crashes
or clearance violations. Breadcrumb-retreat unstick gated OFF by
default (reproduces ballistic-tumble crashes; a controller-level
safe-retreat primitive is the real fix). Study re-pinned to
`891ca138`. What it changed: campaign v5 is unblocked *except* two
lead decisions — R7 per-checkpoint budget 120→240 s, and re-demonstrating
reference solvability under the frozen params. Evidence:
[008 results summary](008-droan-gl-r7-avoidance-fix/results/results_summary.md),
[agent_study/HANDOFF.md](../agent_study/HANDOFF.md).

### 2026-08-26 → 2026-08-27 — v4 "stock stack fails R7" verdict RETRACTED; eight real avoidance defects found and fixed

Diagnosis session (notebook 002): the 2026-08-25 campaign blocker was
misdiagnosed — the judge set `ISAAC_SIM_GUI`, which the standalone
Pegasus path ignores (`ISAAC_SIM_SCENE` loads the scene), and the
obstacle USDA had no defaultPrim; **the pillar world never loaded**,
so the −0.87 m "penetration" was a flight through empty air judged
against virtual coordinates. Judge fixed (v5). With pillars actually
loaded the stock stack then *genuinely* failed R7, and eight distinct
droan_gl defects were identified, fixed, and parameterized (deviation
cost, vertical escapes, unbounded deviation, stale collision votes,
progress shortcutting, camera-lag yaw, retreat-as-unseen, committed-
path flyout). What it changed: (a) judge verdicts about scene-dependent
rungs now require visual confirmation the scene loaded; (b) the
benchmark stays frozen and the *stack* gets fixed (choice #6);
(c) the campaign blocker shrank from "platform can't avoid obstacles"
to "lead must sign off the R7 budget." Evidence:
[008 design spec](008-droan-gl-r7-avoidance-fix/design_spec.md),
[008 results](008-droan-gl-r7-avoidance-fix/results/results_summary.md).

### 2026-08-26 — study made portable and private

Agent-study assets (planners, prompts, judge, reference solution —
the answer key) moved from the paper submodule to the PRIVATE
`agent_study/` submodule; they stay private until the study concludes.
`notebook/` made TRACKED on `airstack-paper` (overriding develop's
local-only convention) so the experimental record travels between
machines. Runs retention policy added after a 248 GB log leak.
Evidence: [agent_study/HANDOFF.md](../agent_study/HANDOFF.md), commits
`9592bedc`, `2d5550ee`, `9caeb33c`.

### 2026-08-25 — Amendment 1: R7/R8 completeness rungs; campaign v4 blocked same day

Lead approved Amendment 1: the ladder gains R7 (obstacle-route rung —
seeded pillar field, judge-issued fresh routes with a proven
discrimination property, 1.0 m clearance bar) and R8 (landing cycle).
Machinery built and validated 2026-08-25/26. The campaign was then
immediately blocked when the reference solution on the stock stack
appeared to fail R7 (−0.87 m) — a verdict later retracted (see
2026-08-26→27 entry). What it changed: study scope grew from
"integrate and fly" to "integrate, fly, avoid, land"; the blocker
spawned notebook entry 002. Evidence:
[007 design spec §2.9, §3(h)](007-agent-study-prereqs/design_spec.md).

### 2026-08-25 — judge v2 validated; campaign cleared for real trials

The pilot's judge defects led to a provenance redesign (judge v2):
negative controls confirmed it rejects publishes-but-doesn't-fly
integrations at the correct stage; positive controls confirmed the
reference solution passes R5 (0.08 m) and R6 (0.03 m). Calibration
batch (real A1/A2 trials) found and fixed three harness defects
(bg-task ceiling, 100-turn cap, bottom-up scoring) and produced the
first full-ladder R6 completion. What it changed: the study moved from
"pipeline works" to "trials may be scored." Evidence:
[007 results §(f),(g)](007-agent-study-prereqs/results/results_summary.md).

### 2026-08-24 → 2026-08-25 — pilot trial done; models frozen; human baseline dropped

First real closed-loop A1 ladder trial completed end-to-end (score R3,
$22.95, 45.6 min agent time) — and surfaced three judge defects,
redirecting effort to judge hardening before any scored trials. P-6
prompts frozen; campaign models fixed at Sonnet + Opus (2026-08-24).
Lead decision 2026-08-24: **skip the human reference-integration
baseline** (test plan (c)) — study proceeds without it, revisit before
submission. Evidence:
[007 design spec §2.8, §3(c),(e)](007-agent-study-prereqs/design_spec.md),
commits `20e4cbad`, `34d7bf34`.

### 2026-08-19 → 2026-08-25 — platform sprint: modular AirStack shipped; DX + docs overhauled (develop track)

A design session (2026-08-19/20) mapped sim-launch friction and split
the work into a pre-RFC cleanup campaign (entry 001, DONE 2026-08-20:
intent flags, `airstack ready`, guards, logs) and the RFC #379/#380
modular campaign (entry 002: modules/stacks/fleets, staged as
PRs #388–#396, merged to develop 2026-08-24; legacy `AUTONOMY_ROLE`
removed). Around it: Release 0.19.0 + relicense to BSD-3-Clause-Clear
(2026-08-22), scene flag + CLI command groups + landing-page demo
video (2026-08-24, entries 003/004/005), Diátaxis docs audit
(2026-08-25, entry 006 — 18 accuracy defects fixed on a 9-commit
overhaul branch). Motives (recorded by the lead 2026-08-27): the
modular restructure was done **to support the paper's modularity
positioning** — module swapping (C1) and ease of upstreaming features
from forked projects — while the docs overhaul and CLI cleanup were
done simply **for improved developer usage and adoption**. What it
changed for the research direction: the modular architecture claim C1
describes is now the *shipped* architecture, not a plan, and the
docs/artifact story feeds the release gate's clone-and-run item.
Evidence: [001](001-pre-rfc-workflow-cleanup/design_spec.md),
[002](002-rfc-modular-airstack/design_spec.md),
[006](006-diataxis-docs-audit/design_spec.md).

### 2026-08-03 → 2026-08-04 — evidence hierarchy restructured; agent study designed and prerequisites started

After a senior-PhD internal review, the paper's evidence was
restructured (2026-08-03): the five case studies + released artifact
became PRIMARY, everything in Sec. VI became SUPPORTING, and two
evidence forms were CUT permanently (time-boxed developer exercise,
historical counterfactual as data) — declining to dress anecdotes as
data is now a stated methods position. The agent study protocol was
settled the same day (arms A1–A4, 6-rung ladder, pytest-harness judge,
"agents as reproducible proxy developers"), and its first
prerequisites landed 2026-08-04: P-1 waypoint judge (merged to develop,
PR #378) and P-2 fresh planners + contract. DFM2 reuse analysis
integrated (+34k LOC on 110k base, 97 % additive). AGENTS.md pointed
at positioning.md as mandatory reading. Evidence:
[positioning.md §Evidence hierarchy](../ICRA_2027_AirStack_Paper/paper_positioning.md),
[007 design spec](007-agent-study-prereqs/design_spec.md), commits
`7607a72d`, `abb6046d`, `407ec557`, `eeeb0aa8`, `823dec5e`.

### 2026-07-20 → 2026-07-21 — paper founded: positioning, claims, case-study corrections, release gate

The ICRA 2027 paper submodule was created with the working outline and
the three claims C1/C2/C3. Core positioning settled: full-stack
reference architecture + validated sim-to-real workflow ("boilerplate"
banned); the CI testing loop positioned as intellectual merit
(*continuous flight readiness*, *defect-cost gradient*); AirSim
reframed as a complementary parallel backend, never "legacy" in paper
text. Case-study facts corrected — notably **DFM2 is SIM-ONLY on
AirStack** (its hardware was a ground robot on a different stack) —
and the release-gate checklist + six-question case-study instrument
established (2026-07-21). Evidence:
[positioning.md](../ICRA_2027_AirStack_Paper/paper_positioning.md),
commits `96020591`, `e3e39baa`, `e76a0ea5`, `db14d880`, `c601a270`,
`d4d2ee41`. (Positioning.md dates the discussions only as "July–August
2026"; the dates here are the first commits recording each decision.)
