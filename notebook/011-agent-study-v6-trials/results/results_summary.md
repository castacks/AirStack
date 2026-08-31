# Results Summary — Agent Study Campaign v6 Trials

> Entry: [design_spec.md](../design_spec.md) · Status: **WIP — trials
> in progress** · Campaign `2026-08-icra27-vic-v6`, config
> `3b2402f6…`, prompt `afad954d…`, AirStack `961fb9e1`
> (study/mighty-swap, asm_mighty v0.1.0). Trial artifacts live in the
> PRIVATE `agent_study/runs/` (gitignored there); this file is the
> shareable record.

## (a) v6 pipeline sanity — PASS (2026-08-28)

- Smoke trials (A2 noop ×2, A1 judge-spammer cap=2, A4 spammer) +
  `pipeline_selftest.py`: cap enforcement (2 executed / 3 refused;
  all refused on A4), repeatability (invariant fields byte-identical),
  archive completeness — all under the v6 config hash.
- v6 practice/eval layout split verified: fresh workspaces contain
  `obstacles_practice.usda` (seed 20260828) and NO eval scene (answer
  key stays judge-time only).
- A3 image `airstack-study-a3:v1` rebuilt from cache; images at pinned
  `v0.20.0-alpha.16` present; Nucleus creds in place.

## (b) Preliminary trial arc: two environment defects found + fixed — DONE (2026-08-28)

Both fixed in `agent_study` runner commits **before any trial was
scored**; agent-facing judges/caps/prompts/config untouched
(config_sha256 unchanged). Trial attempts ruled infra per rule 4,
archived as `runs/A1_claude-sonnet-5_ladder_claude_001.env-truncated*/`
with disposition notes.

1. **R8-implication scoring bug** (fix `a56441b`). Top-down scoring
   dates from when R6 topped the ladder; Amendment 1 stacked R8 (bare
   takeoff+land — exercises NO planner) on top, so a stock bring-up
   passed R8 and implied R1–R7. Attempt 1 "scored" a full ladder with
   0 judge invocations and planner alpha still configured. Fix: the
   implication chain tops out at R7; R8 is scored only after an R7
   pass (else `skipped`). Re-scoring attempt 1's final state gave R4.
2. **Headless session truncation** (fixes `a56441b` + `41ca11e`).
   `claude -p` (v2.1.241) exits at turn end: ScheduleWakeup/Monitor
   results promise re-invocation that never comes (attempt 1, 15.5 min,
   $4.90), and background Bash tasks are killed ~5 s after the final
   result (attempt 2, 31.5 min, $15.36 — agent backgrounded its
   bring-up and yielded; `CLAUDE_CODE_PRINT_BG_WAIT_CEILING_MS` covers
   only background subagents). Fix: deny ScheduleWakeup+Monitor and set
   `CLAUDE_CODE_DISABLE_BACKGROUND_TASKS=1`, with foreground Bash
   timeouts raised (default 20 min / max 60 min; R7 scoring ≈13 min
   observed) so judge flights run synchronously in-turn.

Ops notes: box hit 95 % disk mid-session (`docker builder prune` freed
64 GB; prune trial/mock workspaces promptly). Attempt 2 also validated
the fixed scoring chain on a real workspace (R7→R4 fail, R3 pass, R8
skipped — no bogus implication).

## (c) Trial matrix — WIP

One row per SCORED trial (infra reruns noted, never scored). Runner:
`41ca11e` unless noted.

| # | Trial | Score | Judge calls | Agent time | Cost | Notes |
|---|-------|-------|------------|-----------|------|-------|
| 1 | A1 / claude-sonnet-5 / #1 | **R8** (full ladder) | 9 / 20 | 2 h 22 m | $13.31 | Climbed R1→R8 in order; single retry on R8; official scoring passed R7 (fresh eval route) then R8 (landed at rest +0.03 m). First full-ladder completion in the study. 2 infra reruns (see (b)). |
| 2 | A2 / claude-opus-5 / #1 | **R8** (full ladder) | 10 / 20 | 2 h 02 m | $21.21 | Ablated arm; one R5 fail→fix cycle, R6 re-verified once, R7/R8 first try; official scoring R7+R8 pass. Ablation cost iterations/cost, not score. |
| 3 | A3 / claude-sonnet-5 / #1 | **R6** | 17 / 20 | 2 h 05 m | $30.31 | Bare-parts arm: 6 fails to clear R1, 2 more at R3, 2 at R4; R5/R6 then first-try. R7 FAIL — min clearance **−0.427 m** (pillar penetration; no sense-and-avoid chain built). First arm divergence, exactly at the Amendment-1 completeness rung. 313 k tokens out (~2–3× A1/A2). |
| 4 | A4 / claude-opus-5 / #1 | **R8** (full ladder) | 0 / 0 (open loop) | 1 h 46 m | $17.99 | Blind integration passed R7+R8 official scoring first try; agent never even attempted a judge call (0 refusals). On this sample the closed-loop gap (A1–A4) is zero while the platform gap (A1–A3) bites at R7. n=1 caveat. |
| 5 | A1 / claude-opus-5 / #1 | **R8** (full ladder) | 9 / 20 | 2 h 11 m | $32.93 | Zero in-session judge failures; self-validated before first call (opened with R4), one extra R7 verification. Official R7+R8 pass. |
| 6 | A2 / claude-sonnet-5 / #1 | **R8** (full ladder) | 10 / 20 | 2 h 29 m | $10.47 | Two R3 failures (~30 min debug cycle) then clean R4→R8; official R7+R8 pass. |
| 7 | A3 / claude-opus-5 / #1 | **R3** | 13 / 20 | 4 h 00 m (killed) | n/a (kill lost usage event) | Bare parts: ~2 h 47 m assembling before first R1 pass; in-session reached R6, but the 4 h wall-clock kill hit before consolidation. Final-state scoring: R7 fail (no avoidance), R6 fail (beta up but route degenerate — "route too straight", spline never materialized), R5/R4 unsatisfiable post-swap (designed cascade), R3 pass. |
| 8 | A4 / claude-sonnet-5 / #1 | **NULL** (0 rungs) | 0 / 0 (open loop) | 1 h 19 m | $14.72 | Declared done at 78 min; final state does not boot — MAVROS never connects, interface/trajectory sentinel nodes missing; all 7 scored rungs failed on real bring-up attempts (consistent → agent-broken state, not flake). Same model+repo as trial 1 (closed loop, R8). |

| 9 | A1 / claude-sonnet-5 / #2 | **R8** (full ladder) | 15 / 20 | 2 h 38 m | $22.06 | Same score as #1, more friction: four-fail R5 provenance arc + one R8 retry. Official R7+R8 pass. |
| 10 | A2 / claude-opus-5 / #2 | **R8** (full ladder) | 5 / 20 | 2 h 12 m | $14.15 | Most economical trial yet: self-validated R1–R5, spent calls only on R6/R7/R8 (+2 R7 re-verifications). Official R7+R8 pass. |
| 11 | A3 / claude-sonnet-5 / #2 | **R6** | 13 / 20 | 1 h 20 m | $12.49 | Replicates the A3 sonnet ceiling, much faster than #1: 2 R1 fails + 5 R4 fails, then R5/R6 first-try; stopped voluntarily after R6, never attempted R7. One infra rerun (operator session kill, `.interrupted/`). |
| 12 | A4 / claude-opus-5 / #2 | **R3** | 0 / 0 (open loop) | 1 h 05 m | $18.54 | Stack boots and flies, but no planner node at bring-up ("route_planner_beta not found") — blind integration never persisted the planner into launch. A4 cell now R8, NULL, R3: open-loop variance, not ceiling. |
| 13 | A1 / claude-opus-5 / #2 | **R8** (full ladder) | 12 / 20 | 1 h 38 m | $20.31 | Opened at R7 (~38 min in, all lower rungs self-validated), zero judge failures; spent remaining calls on a full R1–R8 verification sweep. Official R7+R8 pass. |
| 14 | A2 / claude-sonnet-5 / #2 | **R5** | 12 / 20 | 3 h 16 m | $22.76 | First AirStack-arm sub-ceiling score. In-session cleared R1–R8, but final state boots with planner ALPHA (R5 pass, 0.08 m goal error) and no beta node — the swap wasn't left persistent, so R6/R7 fail at scoring. Final-state discipline caught what session memory hid. |
| 15 | A3 / claude-opus-5 / #2 | **R6** | 11 / 20 | 2 h 40 m | $32.81 | Big improvement over #1 (R3, time-capped): R1 by 1 h 19 m, R6 by 1 h 36 m, then ~30 min attempting avoidance for R7 (one in-session R7 fail), stopped with a consolidated R6 state. Official R7 fail / R6 pass. |
| 16 | A4 / claude-sonnet-5 / #2 | **R6** | 0 / 0 (open loop) | 1 h 30 m | $10.38 | Clean rerun (first attempt INFRA-CONTAMINATED, `.contaminated/`: an orphaned duplicate agent from a chained-launch operator error raced the official agent — they detected each other via cross-session messaging and the official agent stood down; preflight isolation guard added, runner `05540d7`). Rerun: blind integration flew the beta route (goal error 1.11 m), failed R7 obstacles. |
| 17 | A1 / claude-sonnet-5 / #3 | **R8** (full ladder) | 12 / 20 | 1 h 59 m | $9.83 | Cheapest full ladder yet. One R3 and one R4 retry, R5–R8 first try, plus verification sweep. A1 5/5 at ceiling. |
| 18 | A2 / claude-opus-5 / #3 | **R8** (full ladder) | 12 / 20 | 2 h 33 m | $23.59 | Never attempted R7/R8 in-session (spent calls on R1–R6 + re-verification); final state passed both at scoring. |
| 19 | A3 / claude-sonnet-5 / #3 | **NULL** (0 rungs) | 10 / 20 | 2 h 54 m | $20.23 | Fastest A3 climb yet (R1–R6 in 7 calls by 1 h 27 m), then three genuine in-session R7 avoidance attempts broke the system — final state fails even R1 (/clock not publishing). Chasing R7 without regression safety-rails cost the whole ladder. |
| 20 | A4 / claude-opus-5 / #3 | **R5** | 0 / 0 (open loop) | 1 h 21 m | $19.66 | Blind alpha-route flight clean at scoring (0.14 m goal error); R6/R7 fail — beta swap not working in final state. |
| 21 | A1 / claude-opus-5 / #3 | **R8** (full ladder) | 8 / 20 | 2 h 31 m | $24.82 | Stopped after R6 in-session; R7+R8 passed at scoring. A1 6/6. |
| 22 | A2 / claude-sonnet-5 / #3 | **R6** | 9 / 20 | 2 h 04 m | $11.76 | Clean climb to R6, one R7 attempt failed, stopped with budget left (11 calls, ~1.5 h unused). Official R7 fail / R6 pass. |
| 23 | A3 / claude-opus-5 / #3 | **R6** | 13 / 20 | 2 h 38 m | $35.36 | 3 R1 fails, then steady climb; ~30 min avoidance work but never attempted R7 in-session; consolidated R6 held at scoring. Ops: orphaned host processes from A3 workspaces (mavros launch, respawning planners, a file-watch loop) found and swept post-trial — A3 agents leave host-level processes the runner's teardown misses. |
| 24 | A4 / claude-sonnet-5 / #3 | **R8** (full ladder) | 0 / 0 (open loop) | 1 h 37 m | $16.48 | Blind full ladder — R7+R8 passed at scoring. Sonnet's open-loop spread now spans NULL→R8. |
| 25 | A1 / claude-sonnet-5 / #4 | **R8** (full ladder) | 9 / 20 | 2 h 32 m | $9.49 | Slow self-validating start, then flawless R4→R8 (only one R4 retry). A1 7/7. |
| 26 | A2 / claude-opus-5 / #4 | **R8** (full ladder) | 7 / 20 | 2 h 09 m | $20.07 | Zero judge failures; R7+R8 blind at scoring. A2/opus 4/4 R8. |
| 27 | A3 / claude-sonnet-5 / #4 | — running | — | — | — | Started 2026-08-31 13:59 UTC. |

### Round-3 synthesis (24 trials, 3 per cell, 2026-08-31)

Cumulative: **A1 R8×6 · A2 R8,R8,R8,R5,R8,R6 · A3 R6,R3,R6,R6,NULL,R6 ·
A4 R8,NULL,R3,R6,R5,R8.**

- **A1: 6/6 full ladders** — zero variance across models. Median 9.5
  judge calls, ~2 h 20 m, ~$17.
- **A2: 4/6 R8** — two sub-ceiling scores, both final-state
  persistence failures (swap left on alpha; quit after one R7 fail
  with 11 calls unspent). Scaffolding's measured value so far =
  consistency at the top of the ladder, not reachability.
- **A3: 0/6 past R6** (5×R6-ish, 1 NULL, 1 R3). The obstacle rung is
  a wall: agents either stop at R6 or break their system attempting
  avoidance. Highest costs of any arm (~$26 avg where measured, 2–3×
  tokens).
- **A4: full NULL→R8 span both models** (NULL,R6,R8 sonnet /
  R8,R3,R5 opus): 3/6 R8-or-R6, 3/6 R5-or-below. Open loop is a
  lottery; closed loop (A1) is deterministic. Mean rung ~R5.9 vs A1's
  R8.
- Ops finding: A3 agents leave orphaned host processes (mavros,
  respawning planners, watch loops) that survive trial teardown —
  swept manually; candidate runner improvement if more A3 rounds run.

### Round-2 synthesis (16/16 cells, 2 per cell, 2026-08-30)

Cumulative rung-survival by arm: **A1 R8×4 · A2 R8,R8,R8,R5 ·
A3 R6,R3,R6,R6 · A4 R8,NULL,R3,R6.**

- **A1 is 4/4 at the full ladder** (9–15 judge calls, $13–33). The
  scaffolded closed-loop arm is the stable reference point.
- **A2 dropped a trial to R5** — the agent completed the swap
  in-session but left the workspace booting planner alpha; final-state
  scoring caught it. Legibility gap now visible in score, not just
  debug cycles.
- **A3 is hard-capped at R6**: 0/4 R7 attempts cleared (one pillar
  penetration, one time-out before consolidation, two voluntary stops
  without attempting avoidance). The platform gap is structural, and
  costs run high (13–17 calls, ~2–3× tokens).
- **A4 spans NULL→R8** (NULL, R3, R6, R8): open-loop score is a coin
  flip while closed-loop A1 is deterministic — variance, not mean, is
  the closed-loop story (C2).
- Ops: one operator-error contamination (trial 16 attempt 1) caught
  and excluded; preflight isolation guard now enforces host
  cleanliness per trial.

### Round-1 synthesis (8/8 cells, 2026-08-29)

Rung-survival by arm (pooled over models): **A1 R8,R8 · A2 R8,R8 ·
A3 R6,R3 · A4 R8,NULL.**

- **Platform gap (A1–A3):** both A3 trials capped by the missing
  platform — sonnet by capability (R7 pillar penetration −0.427 m, no
  sense-and-avoid), opus by time (2 h 47 m of the 4 h budget consumed
  before the first R1 pass; killed while consolidating R6). Cost also
  separates: A3 ran 17 and 13 judge calls vs 9–10 for A1/A2, and ~2–3×
  the tokens.
- **Closed-loop gap (A1–A4):** not a ceiling effect but a variance
  effect — open-loop opus matched A1 (R8, blind, cheapest AirStack-arm
  trial), open-loop sonnet shipped a system that doesn't boot (NULL).
  Closed loop removes the catastrophic tail rather than raising the
  best case. Needs more trials to firm up, but this is the C2 story.
- **Legibility gap (A1–A2):** invisible in score at n=2; visible in
  friction (A2 spent extra cycles: R5 fail→fix for opus, 2×R3 fail for
  sonnet; later first judge calls).
- The extended ladder did its job: the original R1–R6 ladder would
  have scored A1=A2=A3(son); R7/R8 + final-state scoring produced the
  separation.

Round-robin order: A1:son → A2:opus → A3:son → A4:opus → A1:opus →
A2:son → repeat with rising trial index.

## (d) Analysis for the paper — deferred

Rung-survival curves and `tab:agents` inputs once cells have ≥5
trials.
