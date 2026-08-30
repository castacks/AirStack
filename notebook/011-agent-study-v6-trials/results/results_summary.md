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
| 14 | A2 / claude-sonnet-5 / #2 | — running | — | — | — | Started 2026-08-30 04:18 UTC. |

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
