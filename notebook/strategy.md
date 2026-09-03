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
> Last updated: 2026-09-02

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

Current phase (as of 2026-09-02): **campaign v6 COMPLETE — 40/40
trials scored** (entry [011](011-agent-study-v6-trials/design_spec.md),
§(c-final)). All three arm gaps quantified with a stable failure
taxonomy — under the POST-RELABEL arm names (Amendment 2, 2026-09-02:
A3 = open loop, A4 = bare parts; raw artifacts keep original labels):
A1 80% full-ladder / A2 60% (opus 5-for-5, sonnet 1-for-5 —
legibility×model interaction) / A3 30% (open-loop lottery) / A4 0%
≥R7. §(d) analysis DONE (2026-09-02): survival figure,
tab:agents, cycle counts, feedback taxonomy — all recomputed from raw
results.json and cross-checked against the hand tallies. Next for
Sec. VI-C: prose-ify into the paper submodule + release-artifact
bundle; T4/T5 add-ons still open (P-5 blocks T5). v6 foundation: MIGHTY swap (entry
[010](010-mighty-local-planner-module/design_spec.md), DONE), pin
`study/mighty-swap` @ `961fb9e1`, asm_mighty v0.1.0. Paper state
(2026-09-02): `main.tex` prose-ified end to end (Lessons stays a
numbered list; team-data `\todo{}`s remain), claims audit run — six
gaps fixed/flagged, see the 2026-09-02 direction-log entry; release
gate still open (7 paper-blocking items; ~63 tasks `\task{}` in
`release_gate_and_tasks.tex`).

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
| Agent study (paper Sec. VI-C) | Run the four-arm proxy-developer study on the bring-up-to-flight ladder, judged by the pytest harness | [007-agent-study-prereqs](007-agent-study-prereqs/design_spec.md), [008-droan-gl-r7-avoidance-fix](008-droan-gl-r7-avoidance-fix/design_spec.md), [009-droan-gl-yaw-sweep-unstick](009-droan-gl-yaw-sweep-unstick/design_spec.md), [010-mighty-local-planner-module](010-mighty-local-planner-module/design_spec.md), [011-agent-study-v6-trials](011-agent-study-v6-trials/design_spec.md) | 007 `WIP` (P-5, P-7 open, non-blocking); 008 `DONE` (verdict (c) ❌ under frozen config); 009 `DONE` (sweep works, R7 still 0/10); 010 `DONE` — MIGHTY swapped in as `asm_mighty` v0.1.0; R7 reference solvability 5/5 under frozen v6; 011 `DONE` — 40/40 trials scored + §(d) paper analysis complete 2026-09-02 (figure, tab:agents, cycles, taxonomy) |
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

### 2026-09-02 — Amendment 2: arm labels A3/A4 SWAPPED for presentation (A3 = open loop, A4 = bare parts)

Lead decision: the paper reads better with the arms ordered by what
they remove — A3 = "A1 open loop" (was A4), A4 = "bare parts /
assemble-it-yourself" (was A3). Implemented as a presentation-layer
relabel: `analysis.py` maps raw→display at load and cross-checks
against the unchanged raw-labeled hand tallies (figure/table/output
regenerated, colors and markers follow the semantics so drafts stay
visually comparable); main.tex VI-C, tab:agents, the figure caption,
and positioning.md updated; relabel banners added to notebook 007/011
(their body text and §(c-final) tallies keep original labels);
protocol Amendment 2 recorded in the private agent_study repo. Raw
trial artifacts (`runs/A3_*`/`A4_*`, results.json, transcripts) are
NEVER renamed — the released artifact bundle must ship the mapping
note. Entries below this one use the ORIGINAL labels (A3 = bare
parts, A4 = open loop); per the append-only rule they are not
rewritten.

### 2026-09-02 — behavior-layer story settled; "upstream = publish a module" woven into the paper

Two lead clarifications closed audit findings 4 and 8 (paper
submodule `940509c`). (1) Behavior layer: earlier AirStack shipped a
behavior-tree executor, removed because a simple state machine proved
easier; that state machine lives today in the GCS Foxglove task panel
(robot-commands extension) and is acknowledged architectural debt.
The paper now tells this as shipped (IV-A) with the onboard
re-architecture named as roadmap (Limitations) — resolving the
release-gate behavior item via the describe-what-ships branch, unless
Hardware ships an executor before submission. Swarm CBF's
"behavior-tree executor" sentence remains an interview question
(possibly true version-scoped, pre-removal). (2) Positioning
refinement: under RFC #379/#380/#385, contributing back ≠ merging to
core — optional capabilities ship as pullable module repos
(`asm_dfm2_disturbances` is the exemplar). Woven into III-A (module
repos, pin-by-CLI, public index with declared + CI-verified compat),
intro, abstract, the Q6 framing, and DFM2's Upstreamed paragraph.
New release-gate-adjacent todos: verify the module index is public;
register+pin asm_dfm2_disturbances before submission.

### 2026-09-02 — main.tex prose-ified through VI; claims audit run: six claim–reality gaps fixed or flagged

All remaining outline sections except Lessons (a deliberate numbered
list) were prose-ified (paper submodule `f924b36`), and the lead's
claims-audit task ran against the release branch alongside it
(`analysis/claims_audit_2026-09-02.md` in the paper repo). Fixed in
paper text: stale "per-layer bringup packages" → stack-level
composition (RFC #379 reality); removed `AUTONOMY_ROLE` → split-stack
wording; CI claim scoped (builds automatic per PR, flight marks
per-PR by `/pytest` trigger — a decision option is making one flight
mark automatic to restore the stronger sentence); gossip
relay/radius-degradation model and GCS satellite tiles de-claimed
(not implemented). Flagged for decisions: behavior layer ships no BT
executor (release-gate item confirmed live; Swarm CBF's accounting
paragraph names a "behavior-tree executor" the release line cannot
back — raise in their interview); DFM2 disturbance module synced
locally but not pinned in `airstack.yaml`. Build is clean at 11
pages incl. red todos (~2 pages must come back before submission).
Cross-platform agent evaluation added to Limitations as named future
work requiring a stack-neutral task suite and judge (lead decision
2026-09-02: rejected for this paper — home-field-benchmark confound
plus deadline math; banked as a next-paper seed).

### 2026-09-02 — Correction: A3 R7-chase self-destructions are 2, not 3

The "CAMPAIGN v6 COMPLETE" entry below (and 011's §(c-final) headline,
now corrected in place with a marker) said 3/10 A3 trials destroyed a
working R6 system chasing R7. The per-trial log supports 2 (trials 19
and 27); the third zero-rung final state (trial 35) never cleared R4
in-session — the R4-integration wall, a different mechanism. Caught
while fact-checking the paper prose against the per-trial record; the
paper text uses the corrected phrasing.

### 2026-09-02 — Sec. VI-C analysis produced; recomputation matches

Notebook 011 §(d) is complete
([results_summary.md §(d)](011-agent-study-v6-trials/results/results_summary.md)):
`d-analysis/analysis.py` recomputes everything from the 40
`results.json`/`judge_log.jsonl`/`transcript.jsonl` files and
cross-checks the hand-tallied §(c-final) — matrix, survival, and
tab:agents all reproduce (one flag: A3 mean time is 2 h 39 m, hand
table rounded to 2 h 40 m; A4's distinct prompt hash verified as the
expected judge-cap-0 footer). Deliverables ready for the paper:
rung-survival figure (pooled, Wilson bands; per-model small multiples
for the appendix — the A2 opus/sonnet split panel is the interaction
picture), generated `tab_agents.tex` with A2 per-model sub-rows, and
the two protocol prose sentences (cycles: A1 median 1 judge cycle per
rung everywhere, A3 2.5 at bring-up / 3 at integration, 5/10 A3
trials burned 19 runs on R7 without a pass; taxonomy: judge verdicts,
topic rates/contents, logs, TF errors). VI-C is now a writing task,
not an analysis task; remaining study work is the release-artifact
bundle and the optional T4/T5 add-ons.

### 2026-09-02 — CAMPAIGN v6 COMPLETE: 40/40 trials, all four gaps quantified

The ladder campaign finished (entry
[011](011-agent-study-v6-trials/results/results_summary.md), §(c-final)):
**A1 80% full-ladder (survival 1.0→0.8) · A2 60% (opus 5/5, sonnet
1/5 — the legibility×model interaction) · A3 0% ≥R7 in 10 tries with
3 self-destructions · A4 30% R8, NULL→R8 lottery.** Mean rung 7.5 /
6.9 / 3.9 / 4.7; cost $17–29/trial, ~$820 total, 4.2 days serial on
one GPU, zero unresolved infra confounds (2 excluded+documented).
Sec. VI-C now has its survival figure and tab:agents data; per-model
reporting required for A2. Remaining for the paper: figure + table
generation, transcript-derived cycle counts/feedback taxonomy, and the
release-artifact bundle. T4/T5 add-ons remain open (P-5 still
blocking T5). The exploratory section can now be written as a
highlighted finding: every removed platform ingredient produced a
distinct, named failure mode, absent under the full scaffolded
closed-loop configuration.

### 2026-09-01 — Round 4 complete (4/cell): model×scaffolding interaction emerges

Cumulative (entry [011](011-agent-study-v6-trials/results/results_summary.md)):
**A1 R8×8 · A2/opus R8×4 vs A2/son R8,R5,R6,R5 · A3 R6×5,R3,NULL×2 ·
A4/opus R8,R3,R5,R8 vs A4/son NULL,R6,R8,R3.** New finding: the
**scaffolding×model interaction** — opus holds the ceiling without
scaffolding (A2/opus 4/4 R8), sonnet does not (A2/son 1/4) despite
being 4/4 WITH it (A1/son) — scaffolding matters most for the smaller
model. Failure taxonomy stabilized: swap-not-persisted (A2/son, A4),
R7-chase self-destruction (A3/son ×2), time-capped assembly (A3/opus);
all three absent in A1 (8/8 R8). Round 5 (final, → 5/cell = 40
trials) started.

### 2026-08-31 — Round 3 complete (3/cell): A1's zero-variance ceiling vs everyone else's spread

Cumulative (entry [011](011-agent-study-v6-trials/results/results_summary.md)):
**A1 R8×6 · A2 4/6 R8 (+R5,R6) · A3 0/6 past R6 (incl. one NULL from a
system-breaking R7 attempt) · A4 spans NULL→R8 both models (3/6 ≥R6).**
The picture at n=3/cell: the scaffolded closed-loop arm is
deterministic at the full ladder; ablation costs consistency
(final-state persistence failures); no platform means the obstacle
rung is unreachable and attempts to reach it can destroy a working
system; no closed loop makes outcomes a lottery. Rung-survival curves
will carry this cleanly. Round 4 running; 5/cell (40 trials) remains
the target — round 5 completes it.

### 2026-08-30 — Round 2 complete (2/cell): arm separation holds and sharpens

Cumulative (entry [011](011-agent-study-v6-trials/results/results_summary.md)):
**A1 R8×4 · A2 R8,R8,R8,R5 · A3 R6,R3,R6,R6 · A4 R8,NULL,R3,R6.**
A1 is deterministic at the full ladder; A2 dropped one trial to R5 (the
swap wasn't left persistent — final-state scoring caught what session
memory hid), so the legibility gap now appears in score; A3 is
hard-capped at R6 with 0/4 R7 clearances (the platform gap is
structural); A4 spans NULL→R8 — open-loop is a coin flip where
closed-loop is deterministic, making variance (not mean) the C2
argument. Ops: one operator-error contamination (an orphaned duplicate
agent racing the official agent in one workspace — the two detected
each other via cross-session messaging and the official agent stood
down) was caught, excluded, and structurally prevented (runner
preflight isolation guard, agent_study `05540d7`). Round 3 started;
target remains ≥5/cell.

### 2026-08-29 — Round 1 complete (8/8 cells): the extended ladder separates the arms

Scores (entry [011](011-agent-study-v6-trials/results/results_summary.md)):
**A1 R8,R8 · A2 R8,R8 · A3 R6,R3 · A4 R8,NULL.** What round 1 says:
(1) **Platform gap is real and two-mode** — A3/sonnet capped by
capability (R7 pillar penetration, no sense-and-avoid buildable in
budget), A3/opus capped by time (2 h 47 m of 4 h spent reaching R1;
killed consolidating R6). (2) **Closed-loop value shows as variance,
not ceiling** — A4/opus matched A1 blind (R8), A4/sonnet shipped a
non-booting system (NULL, MAVROS never connects): the loop removes the
catastrophic tail. Same model + repo closed-loop = R8. (3) **A1–A2
legibility gap invisible in score at n=2**; shows only as extra debug
cycles. (4) Amendment 1 vindicated: the original R1–R6 ladder would
have read A1=A2=A3(son). Caveats: n=2/cell (target ≥5), A3/opus usage
metrics lost to the wall-clock kill (CLI result event never emitted —
known limitation, note for the artifact). Round 2 (trial-index 2)
started with the same rotation.

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
