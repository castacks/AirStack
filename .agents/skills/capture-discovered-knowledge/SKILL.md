---
name: capture-discovered-knowledge
description: Persist hard-won discoveries to AGENTS.md or a new/existing SKILL.md so future agents don't repeat the discovery cost. Trigger after any long context-discovery task (multi-minute grep / file-reading session, parallel research agents, debugging that took several iterations) or whenever you learn something critical, surprising, undocumented, or that contradicted prior assumptions in AGENTS.md or a skill. Decides between updating AGENTS.md, updating an existing skill, or creating a new skill.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Capture Discovered Knowledge for Future Agents

## When to Use

Fire this skill at the **end** of a task when any of the following is true:

- The task required a long context-discovery phase: many greps across the repo, reading multiple unfamiliar files, or running parallel research agents.
- You learned a mechanism that was **not documented** in [AGENTS.md](../../../AGENTS.md), the relevant skill, or the package README — and a future agent would need it to do similar work.
- A discovery **contradicted** something stated in AGENTS.md or a skill (a stale claim, a renamed file, a removed flag, a wrong path).
- A debugging session resolved on a non-obvious cause (env var that must be unset, a hostname-vs-container-name dispatch, a hidden config precedence rule).
- You produced a new skill, deprecated one, or merged two — the registry in AGENTS.md must reflect that.
- A user-issued correction revealed a project convention that isn't written down.

**Strong trigger:** the task ran more than ~5 minutes and most of that time was reading rather than writing. That cost should be amortized.

## When NOT to Use

Persisting noise is worse than persisting nothing. Skip this skill when:

- The discovery is already captured in code (well-named symbols, an existing config key, a clear comment) — the code itself is the source of truth.
- The fact is only true for the current branch or PR (in-flight refactors, temporary workarounds being removed in the same PR).
- The information is conversation-scoped (current task progress, a plan, a TODO list).
- AGENTS.md or an existing skill *already* covers it. Re-read before adding.
- It's a one-off bug fix with no recurrence risk. The commit message is enough.

If unsure, prefer NOT writing. AGENTS.md is loaded into every agent's context — bloat has a real cost.

## Decision: Where Does This Knowledge Belong?

Use this tree, top-down. Stop at the first match.

1. **Does it correct a wrong claim** in [AGENTS.md](../../../AGENTS.md) or an existing SKILL.md?
   → Edit that file directly. Do not duplicate the correction elsewhere.

2. **Is it a project-wide mechanism** that any agent might encounter (env var flow, container lifecycle gotcha, CI gate, naming convention, where state is computed)?
   → Add a short paragraph or table row to [AGENTS.md](../../../AGENTS.md). Pick the best existing section; only create a new H2/H3 if the topic is genuinely new.

3. **Is it a multi-step workflow** (more than ~5 distinct steps, or with conditional branches) that recurs?
   → Update an existing skill if one is close, otherwise create a new skill under [.agents/skills/](..). Then add a row to the skills table in AGENTS.md.

4. **Is it scoped to a single module / package**?
   → Update that package's `README.md`, not AGENTS.md.

5. **Is it about *how to write code* in a particular subsystem** (style, idiom, helper to prefer)?
   → Update the relevant skill if one exists. If not and it's broadly useful, consider a new skill. If narrow, leave it for the code reviewer.

## Updating AGENTS.md

Keep edits surgical. AGENTS.md is read on every agent invocation, so every line should earn its place.

**Do:**

- Lead with the mechanism in one sentence, then the file/line that's authoritative.
- Link to source files with markdown so the reader can verify (`[robot/docker/.bashrc](robot/docker/.bashrc)`).
- Put corrections inline where the wrong claim used to live — don't leave the wrong claim and append a footnote.
- If you're adding more than ~10 lines, ask whether it really belongs in a skill instead.

**Don't:**

- Restate things the file already says.
- Add aspirational guidance ("we should…", "TODO: document…"). AGENTS.md is descriptive of the current state.
- Add emojis or decorative formatting beyond what the file already uses.
- Re-paragraph long-line content to satisfy MD013; the file's prevailing style ignores it.

## Creating a New Skill

Only create a new skill when the topic is **recurring**, **non-trivial**, and **doesn't fit into an existing skill without distorting it**. The bar is high: 9 → 14 skills is a one-week jump; 14 → 30 is unmanageable.

Steps:

1. Pick a verb-led kebab-case name: `verb-object` (e.g., `add-ros2-package`, `bump-version-and-release`). Avoid noun-only names — they don't read well in trigger descriptions.
2. Read 2 existing skills closest in style to yours. Match their frontmatter shape exactly: `name`, `description`, `license: Apache-2.0`, `metadata.author: AirLab CMU`, `metadata.repository: AirStack`.
3. Write the description as a **one-line trigger** that contains the words an agent would use when describing the task. The trigger model fires on this string, so include the keywords (commands, file names, error phrases, env vars).
4. Section structure: `## When to Use` → numbered or topical body → `## Common Pitfalls` → optional skeleton/cheatsheet at the end.
5. Verify all claims against the actual files. If you can't find a referenced file, the skill is wrong.
6. Add the skill to the table in [AGENTS.md](../../../AGENTS.md) under "Common Workflows (Skills)". One line, one clear when-to-use phrase.
7. Do **not** update unrelated skills with cross-references unless it's load-bearing — the trigger system handles routing.

## Updating an Existing Skill

When the discovery extends rather than replaces an existing skill:

- Edit the relevant section in place. Don't append a "Recent Findings" trailer — fold it in.
- If the change makes the description-line trigger inaccurate, update the description too.
- If a "Common Pitfalls" section exists, that's usually the right home for one-line gotchas.
- Bump nothing — skills aren't versioned.

## Deprecating or Merging Skills

If a discovery makes a skill redundant or wrong as a whole:

- Prefer merging into the surviving skill rather than leaving a stub.
- Delete the old `SKILL.md` and the directory.
- Remove the row from the AGENTS.md skills table.
- If anything else in the repo links to the old skill path, update those links in the same edit.

## Quality Bar

Before saving, confirm:

- [ ] The claim is verified against the current code, not memory or a prior conversation.
- [ ] A future agent reading only this addition (no surrounding context) would understand it.
- [ ] The "why" or "how it ends up that way" is included when the mechanism is non-obvious — not just the end-state fact.
- [ ] The addition is in the smallest scope that fits (package README < skill < AGENTS.md).
- [ ] No duplicate of an existing claim.

## Common Pitfalls

1. **Over-saving** — turning every bug fix into an AGENTS.md note. The git history is also memory; trust it.
2. **Wrong scope** — putting module-specific quirks in AGENTS.md instead of the package README, or vice versa.
3. **Restating code** — if a function name or config key already explains the behavior, the doc adds noise.
4. **Stale references** — linking files by path without verifying they still exist at that path.
5. **Description bloat** — skill descriptions that pile on keywords stop firing reliably. Keep them as a single, concrete trigger sentence.
6. **Forgetting the registry** — creating a new skill but not adding it to the AGENTS.md table makes it invisible to future agents.
7. **Saving in the wrong direction** — recording "I tried X and it failed" instead of "the mechanism is Y." Future agents need the mechanism, not the trial.

## Quick Self-Check at End of Task

Three questions, ~10 seconds:

1. *If a different agent picked up this same task tomorrow, is there anything they'd waste time rediscovering?*
2. *Did anything I just learned contradict AGENTS.md or a skill?*
3. *Did I create, merge, or deprecate a skill?*

If any answer is yes → invoke this skill. If all are no → you're done.
