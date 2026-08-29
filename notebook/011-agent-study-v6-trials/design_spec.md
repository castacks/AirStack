# Design Spec: Agent Study Campaign v6 — Trial Execution (ICRA'27 Sec. VI-C)

> Notebook entry: `notebook/011-agent-study-v6-trials/` · Date started: 2026-08-28 · Branch: `airstack-paper`
>
> **Status: `WIP`** <!-- trials in progress; 1 scored -->
>
> **Canonical spec:** `agent_study/agent_study_protocol.md` (PRIVATE
> submodule) — arms, ladder, caps, guardrails. Prerequisites and
> pipeline validation: [007](../007-agent-study-prereqs/design_spec.md).
> R7 solvability under the frozen v6 config:
> [010 §(e)](../010-mighty-local-planner-module/design_spec.md).
> This entry is the execution record of the **v6 trial campaign
> itself** (`2026-08-icra27-vic-v6`): per-trial dispositions, runner
> defects found during execution, and the accumulating score matrix.
> NOTE (lead, 2026-08-28): `agent_study/HANDOFF.md` is retired; this
> entry + `notebook/strategy.md` are the running record.

## 1. Problem Context

Campaign v6 (config `3b2402f6…`, prompt `afad954d…`, AirStack pin
`study/mighty-swap` @ `961fb9e1`, asm_mighty v0.1.0) was declared
READY FOR TRIALS after R7 reference solvability 5/5 (entry 010). The
run matrix is ~4 arms × 2 models × ~7 trials, strictly serial on one
GPU (RTX 5090 box), arms/models interleaved round-robin per the
reproducibility rules. Per rule 4, infra/environment failures are
rerun with the same index and never scored.

## 2. Execution Plan

1. Pipeline smoke (mock agents, no sim/tokens) under the v6 config.
2. One preliminary real trial (A1/claude-sonnet-5/#1), sanity-checked
   end to end before committing to the batch.
3. Round-robin batch: A1:son → A2:opus → A3:son → A4:opus →
   A1:opus → A2:son → … until ≥5 trials/cell or budget/schedule stops.
4. Prune adjudicated workspaces (`runner/prune_run.sh`) — the box runs
   ~86–95 % disk.

## 3. Test Plan (lettered; artifacts under `results/`)

- **(a) v6 pipeline sanity** — smoke trials + `pipeline_selftest.py`
  under the frozen config; practice/eval layout split staged correctly.
- **(b) preliminary real trial + runner-defect record** — the
  A1/sonnet-5/#1 arc, including any runner/environment defects found
  and fixed before the first scored trial.
- **(c) trial matrix** — accumulating per-trial results
  (`results_summary.md` table), round-robin order, dispositions.
- **(d) analysis for the paper** — rung-survival curves + `tab:agents`
  inputs, produced once cells fill (deferred until enough trials).
