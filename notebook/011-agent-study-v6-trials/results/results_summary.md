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
| 5 | A1 / claude-opus-5 / #1 | — running | — | — | — | Started 2026-08-29 06:25 UTC. |

Round-robin order: A1:son → A2:opus → A3:son → A4:opus → A1:opus →
A2:son → repeat with rising trial index.

## (d) Analysis for the paper — deferred

Rung-survival curves and `tab:agents` inputs once cells have ≥5
trials.
