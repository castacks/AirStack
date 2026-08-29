# Getting the Most out of Your Coding Agent

This guide is for a human developer directing a coding agent (Claude Code, OpenHands, etc.) on AirStack. The repo does a lot of the prompting for you — this page tells you what to ask for so the built-in machinery actually gets used.

## Why this repo is agent-ready

Every agent session starts with [`AGENTS.md`](https://github.com/castacks/AirStack/blob/develop/AGENTS.md) in context (`CLAUDE.md` symlinks to it): architecture, topic conventions, reference implementations, pitfalls. Beyond that, [`.agents/skills/`](https://github.com/castacks/AirStack/blob/develop/.agents/README.md) holds 20+ step-verified workflow guides that agents discover by task description, and the [feature notebook convention](intermediate/feature_notebook.md) gives every feature a design spec and evidence trail that outlive the session. Agents also get a condensed cheat sheet at [AI Agent Quick Reference](advanced/ai_agent_guide.md).

## 1. Start every feature with a notebook design spec

Before the agent writes any code, ask it to create the notebook entry:

> Follow the use-feature-notebook skill: write `notebook/NNN-my-feature/design_spec.md` from our discussion before implementing anything. Show me the spec first.

`AGENTS.md` already instructs agents to do this at the start of every feature, but an explicit nudge guarantees it — and reviewing the spec before code is written is the cheapest design review you will ever do. A good spec (template in [`use-feature-notebook`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/use-feature-notebook)) contains:

- **Problem context** — the motivation, constraints, and decisions *from your conversation*. This is the one section that preserves context existing nowhere else; check it captures what you actually discussed.
- **Proposed implementation** — affected packages, nodes, topics, data flow, with per-section status labels (`DESIGN/TODO` / `WIP` / `DONE`) the agent updates as it works, and `Date started` / `Last updated` in the header — a lab journal entry without a date is unusable later.
- **Lettered test plan** — sections `(a)`, `(b)`, `(c)`…, each stating what runs (unit test, system-test mark, sim scenario), what is measured, and what counts as pass. The letters name the results folders later.

Why this pays off: agent sessions end, but the spec survives on disk — the next session (or the next agent, or you in three weeks) picks it up instead of re-deriving intent, and the status labels show exactly which parts are designed, in progress, or finished. Note `notebook/` is **gitignored and local-only**: nothing in it is committed or referenced from committed code; its content reaches the world only distilled into the PR body ([full convention](intermediate/feature_notebook.md)).

## 2. Point the agent at skills, not raw prompts

Instead of describing a workflow from scratch, name the skill — *"use the add-ros2-package skill"* beats a paragraph of instructions, because the skill encodes repo-specific steps (templates, install directives, canonical-default launch args) that a generic prompt misses. Agents match skills to tasks automatically via each skill's trigger description, but naming one removes the guesswork. High-leverage ones for common asks:

| Ask | Skill |
|-----|-------|
| New algorithm module | [`add-ros2-package`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/add-ros2-package), then [`integrate-module-into-layer`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/integrate-module-into-layer) to wire it into a stack |
| New launch topology | [`create-stack`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/create-stack) |
| "It doesn't work" | [`debug-module`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/debug-module) |
| End-to-end verification | [`test-in-simulation`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/test-in-simulation) |
| Docs for the new thing | [`update-documentation`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/update-documentation) |

The full catalog with one-line triggers is the table in [`.agents/README.md`](https://github.com/castacks/AirStack/blob/develop/.agents/README.md) (mirrored in `AGENTS.md`). Two other steering moves that cost you one sentence each:

- **Name the reference implementation.** `AGENTS.md` lists one well-structured package per module type (e.g. `droan_local_planner` for local planners, `random_walk` for global planners); *"study the DROAN local planner first, then follow its structure"* anchors the agent to working code instead of invented patterns.
- **Say where the work lands.** Modules get wired in a stack's entry launch file, not in per-layer bringups (the old layer-bringup workflow is legacy) — telling the agent which stack you're targeting up front avoids a wrong-locus integration.

## 3. Make the agent test and record

Don't accept "it builds" as done. Ask for evidence against the spec's lettered plan:

1. Each test run drops raw artifacts (metrics files copied from `tests/results/<timestamp>/`, plots, sim screenshots, log *excerpts* — not full container logs) into `notebook/NNN-slug/results/<letter>-<section>/`, letters matching the spec, run timestamps preserved.
2. After validation, the agent writes `results/results_summary.md` — self-contained, tables and figures embedded directly (not linked), one section per letter, ending with per-section verdicts and known limitations.
3. The PR body is populated from the notebook: problem context from `design_spec.md`, validation tables from `results_summary.md`, key figures uploaded as PR attachments — the only route notebook content takes off your machine, since reviewers can't see `notebook/`.

A useful review habit: read `results_summary.md` before the diff. If a lettered section has no artifacts folder, that part of the plan wasn't run.

## 4. Close the loop: capture what the agent learned

When an agent spends real time digging — a long grep-and-read session, a debugging chain that resolved on a non-obvious cause, a discovery contradicting `AGENTS.md` — tell it to run [`capture-discovered-knowledge`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/capture-discovered-knowledge) before finishing. The skill decides where the knowledge belongs (fix the wrong claim in `AGENTS.md`, extend an existing skill, or the package README for module-scoped quirks) and holds a deliberately high bar against bloat: if the code already says it, don't persist it. This is what keeps the next session from paying the same discovery cost. Keep the scopes straight: per-feature evidence stays in the notebook; only durable, repo-wide mechanisms get promoted to `AGENTS.md` or a skill.

## 5. Practical tips (all verified in this repo)

- **Non-interactive `docker exec`, always.** All development happens inside containers, and agents get stuck on interactive prompts. The pattern is `docker exec airstack-robot-desktop-1 bash -c "<command>"` — never `airstack connect` or `docker exec -it` in an agent session ([`use-airstack-cli`](https://github.com/castacks/AirStack/tree/develop/.agents/skills/use-airstack-cli)).
- **`bws` / `sws` aliases.** Inside robot containers, `bws` is `colcon build` with the repo's flags and `sws` sources `install/setup.bash`:

    ```bash
    docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_pkg && sws && ros2 launch my_pkg my.launch.xml"
    ```

- **Bring the stack up agent-friendly.** `airstack up robot-desktop --no-autolaunch` starts the container without launching the autonomy stack (so the agent controls what runs), and `airstack ready` blocks until containers → sim `/clock` → nodes → PX4 are actually up (`--json` for scripts) — better than the agent inventing sleep loops.
- **Verify with `airstack test` marks.** `airstack test -m unit -v` for fast hermetic checks, `-m "build_docker or build_packages"` for build health, `-m liveliness` / `-m sensors` / `-m takeoff_hover_land` for staged sim verification ([tests/README.md](https://github.com/castacks/AirStack/blob/develop/tests/README.md)). Have the agent copy the resulting `tests/results/<timestamp>/metrics.json` into the notebook.
- **`wiring.md` is generated ground truth.** Each stack's `wiring.md` is snapshotted from the *running* graph and drift-checked in CI — never hand-edited. After the agent changes a stack's topology, it must regenerate it (`airstack test -m wiring --stack <name> ...`) or CI fails ([stacks guide](stacks.md#wiringmd-generation-and-drift-checking)).
- **Logs are `docker logs`-visible.** Container tmux output is mirrored to `docker logs`, so `docker logs airstack-robot-desktop-1` is the agent's window into bringup — no tmux attach needed.

None of this makes an agent infallible — it still writes plausible-looking wiring that doesn't connect and tests that pass vacuously. The notebook spec, the lettered evidence, and `wiring.md` drift-checking exist precisely so you can check the agent's work without re-doing it.

## Related pages

- [Feature Notebook](intermediate/feature_notebook.md) — the full notebook convention this guide builds on
- [AI Agent Quick Reference](advanced/ai_agent_guide.md) — the condensed agent-facing cheat sheet
- [Contributing Guide](intermediate/contributing.md) — the human PR process the notebook feeds into
