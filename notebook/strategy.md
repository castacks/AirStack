# Research Strategy — AirStack / ICRA 2027

> **How to use this file.** Notebook entries
> (`notebook/`, `notebook_docs/`) are single-topic evidence;
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
> Last updated: 2026-08-27

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

Current phase (as of 2026-08-27): the **agent study (Sec. VI-C)** is
the active sprint — it is the paper's longest-wall-clock item (START
FIRST). Prerequisites P-1…P-4, P-6, P-8 are done; campaign **v5** is
ready pending two small lead decisions (R7 waypoint budget 120→240 s;
reference-solvability re-demonstration under frozen params) — see
[agent_study/HANDOFF.md](../agent_study/HANDOFF.md) and
[002-droan-gl-r7-avoidance-fix](002-droan-gl-r7-avoidance-fix/design_spec.md).
In parallel: prose-ifying `main.tex` section by section, and the
release gate (7 paper-blocking items; ~63 tasks still `\task{}` in
`release_gate_and_tasks.tex` as of today).

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
   [002 design spec](002-droan-gl-r7-avoidance-fix/design_spec.md),
   [agent_study/HANDOFF.md](../agent_study/HANDOFF.md).
7. **The study proceeds without a human reference baseline** (test
   plan (c) skipped) — lead decision 2026-08-24; revisit before
   submission.
   [001 design spec §3(c)](001-agent-study-prereqs/design_spec.md).
8. **Release gate = claim–reality consistency, not feature
   completeness.** Everything beyond the seven gate items → tabled to
   Limitations/Future Work.
   [positioning.md §Release gate](../ICRA_2027_AirStack_Paper/paper_positioning.md).

## Campaigns

Entries `001`/`002` live in `notebook/` (tracked on `airstack-paper`);
`d001`–`d006` live in `notebook_docs/` (**local-only on this machine**,
develop-branch work). Statuses come from the entries' own labels.

| Campaign | Goal | Entries | Status |
|---|---|---|---|
| Agent study (paper Sec. VI-C) | Run the four-arm proxy-developer study on the bring-up-to-flight ladder, judged by the pytest harness | [001-agent-study-prereqs](001-agent-study-prereqs/design_spec.md), [002-droan-gl-r7-avoidance-fix](002-droan-gl-r7-avoidance-fix/design_spec.md) | 001 `WIP` (P-5, P-7 open, non-blocking); 002 `IN PROGRESS`; campaign v5 pending two lead decisions |
| Paper writing & positioning | Sec. I–V prose, case-study interviews, figures | — (lives in the `ICRA_2027_AirStack_Paper/` submodule; no notebook entries) | Related Work + Design Principles prose done 2026-08-03; interviews not recorded anywhere yet |
| Release gate / v1.0 readiness | The seven paper-blocking items (clone-and-run, verified hardware path, …) | — (no notebook entries yet) | ~63 open `\task{}` vs 1 `\done{}` in `release_gate_and_tasks.tex` |
| Modular AirStack (RFC #379/#380) | Monolith → modules, stacks, fleets; the shipped architecture claim C1 describes | d002 ([notebook_docs/002](../notebook_docs/002-rfc-modular-airstack/design_spec.md)) | `WIP` per its header (impl merged to develop 2026-08-24 as PRs #388–#396; release mechanics, module CI tags, P3 dispatch smoke open) |
| Launch & CLI developer experience | Kill sim-launch friction: intent flags, readiness signal, scene selection, command groups | d001 ([pre-rfc-workflow-cleanup](../notebook_docs/001-pre-rfc-workflow-cleanup/design_spec.md)), d003 (scene flag), d004 (osmo/config/images groups) | d001 `DONE`; d003 `DONE`; d004 results PASS (no header status) |
| Docs & public presentation | Accurate, Diátaxis-organized docs site; demo-video landing page | d005 (landing-page video), d006 (Diátaxis audit) | d005 deliverables committed 2026-08-24/25; d006 audit done, 9-commit overhaul branch built — **merge to develop not recorded** |

## Direction log

Append-only, newest first. Append on any strategic update — a claim
strengthened or weakened, a descope, a new campaign, a priority shift,
a standing choice flipped. Routine feature completions that don't move
the strategy get no entry. Format: `### YYYY-MM-DD — what happened`,
answering *what we learned or decided, what it changed, link to the
evidence*.

### 2026-08-27 — strategy layer created; log seeded retroactively

This file created as the layer between single-topic notebook entries
and positioning.md. All entries below were reconstructed today from
paper_positioning.md, the notebook entries' own dates and statuses,
agent_study/HANDOFF.md, and `git log`; gaps are flagged inline rather
than guessed.

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
[002 results summary](002-droan-gl-r7-avoidance-fix/results/results_summary.md),
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
[002 design spec](002-droan-gl-r7-avoidance-fix/design_spec.md),
[002 results](002-droan-gl-r7-avoidance-fix/results/results_summary.md).

### 2026-08-26 — study made portable and private

Agent-study assets (planners, prompts, judge, reference solution —
the answer key) moved from the paper submodule to the PRIVATE
`agent_study/` submodule; they stay private until the study concludes.
`notebook/` made TRACKED on `airstack-paper` (overriding develop's
local-only convention) so the experimental record travels between
machines. Runs retention policy added after a 248 GB log leak.
Evidence: [agent_study/HANDOFF.md](../agent_study/HANDOFF.md), commits
`de6417a3`, `65b800dc`, `5d138f96`.

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
[001 design spec §2.9, §3(h)](001-agent-study-prereqs/design_spec.md).

### 2026-08-25 — judge v2 validated; campaign cleared for real trials

The pilot's judge defects led to a provenance redesign (judge v2):
negative controls confirmed it rejects publishes-but-doesn't-fly
integrations at the correct stage; positive controls confirmed the
reference solution passes R5 (0.08 m) and R6 (0.03 m). Calibration
batch (real A1/A2 trials) found and fixed three harness defects
(bg-task ceiling, 100-turn cap, bottom-up scoring) and produced the
first full-ladder R6 completion. What it changed: the study moved from
"pipeline works" to "trials may be scored." Evidence:
[001 results §(f),(g)](001-agent-study-prereqs/results/results_summary.md).

### 2026-08-24 → 2026-08-25 — pilot trial done; models frozen; human baseline dropped

First real closed-loop A1 ladder trial completed end-to-end (score R3,
$22.95, 45.6 min agent time) — and surfaced three judge defects,
redirecting effort to judge hardening before any scored trials. P-6
prompts frozen; campaign models fixed at Sonnet + Opus (2026-08-24).
Lead decision 2026-08-24: **skip the human reference-integration
baseline** (test plan (c)) — study proceeds without it, revisit before
submission. Evidence:
[001 design spec §2.8, §3(c),(e)](001-agent-study-prereqs/design_spec.md),
commits `69f30648`, `0dfe6a2c`.

### 2026-08-19 → 2026-08-25 — platform sprint: modular AirStack shipped; DX + docs overhauled (develop track)

A design session (2026-08-19/20) mapped sim-launch friction and split
the work into a pre-RFC cleanup campaign (d001, DONE 2026-08-20:
intent flags, `airstack ready`, guards, logs) and the RFC #379/#380
modular campaign (d002: modules/stacks/fleets, staged as PRs #388–#396,
merged to develop 2026-08-24; legacy `AUTONOMY_ROLE` removed). Around
it: Release 0.19.0 + relicense to BSD-3-Clause-Clear (2026-08-22),
scene flag + CLI command groups + landing-page demo video (2026-08-24,
d003/d004/d005), Diátaxis docs audit (2026-08-25, d006 — 18 accuracy
defects fixed on a 9-commit overhaul branch). What it changed for the
research direction: the modular architecture claim C1 describes is now
the *shipped* architecture, not a plan, and the docs/artifact story
feeds the release gate's clone-and-run item. Evidence:
[d001](../notebook_docs/001-pre-rfc-workflow-cleanup/design_spec.md),
[d002](../notebook_docs/002-rfc-modular-airstack/design_spec.md),
[d006](../notebook_docs/006-diataxis-docs-audit/design_spec.md)
(local-only).

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
[001 design spec](001-agent-study-prereqs/design_spec.md), commits
`29d86cec`, `c8f98f7f`, `dd13017a`, `6fdd3329`, `b145660d`.

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
commits `f5e71b6d`, `9c244723`, `05849733`, `796848e2`, `101cb865`,
`5ca9598f`. (Positioning.md dates the discussions only as "July–August
2026"; the dates here are the first commits recording each decision.)
