---
name: use-feature-notebook
description: Maintain a local, gitignored notebook/ directory that records the design spec and test results for every feature an agent implements. Trigger at the START of any feature-implementation task (create notebook/NNN-feature-slug/design_spec.md before writing code), while implementing (keep the spec's per-section status labels DESIGN/TODO / WIP / DONE current), whenever tests for that feature produce output worth keeping (store under results/<section>/), and when opening the feature's PR (populate the PR body from results/results_summary.md).
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Use the Feature Notebook

## Purpose

Every feature implemented by a coding agent gets a **notebook entry**: a numbered folder under `notebook/` at the repo root that holds the design spec written *before* implementation and the test results produced *during* validation. The notebook is the agent's lab journal — it captures the session context that would otherwise be lost when the conversation ends, and it is the source material for the feature's PR description.

`notebook/` is **gitignored and local-only**. It never lands in a commit. Each developer's machine has its own copy. What *does* leave the machine is the distilled content: the PR body is populated from `results/results_summary.md`, and figures/tables from `results/` are attached to the PR.

## Directory Layout

```
notebook/
├── 001-add-new-planner/
│   ├── design_spec.md              # Written BEFORE implementation
│   └── results/
│       ├── results_summary.md      # Written AFTER tests; feeds the PR
│       ├── a-planner-core/         # Raw artifacts for test section (a)
│       │   ├── run1_metrics.json
│       │   └── trajectory_plot.png
│       └── b-planner-hyperparameters/   # Raw artifacts for test section (b)
│           └── sweep_table.csv
├── 002-fix-lidar-filter/
│   └── ...
```

Naming rules:

- **Feature folder:** `NNN-short-kebab-slug`, where `NNN` is zero-padded three digits. Pick the next number by listing `notebook/` and incrementing the highest existing prefix (start at `001` if empty or missing — create `notebook/` yourself, it is not committed).
- **Results subfolders:** one per lettered test section in `design_spec.md`, named `<letter>-<section-slug>` (e.g. section "(a) Planner core" → `results/a-planner-core/`). The letters MUST match the test-plan section letters in the spec so a reader can navigate spec ↔ results directly.

**Date and timestamp everything.** The notebook is a lab journal, and a journal entry without a date is unusable later. Every design doc, experiment, and results file records when it happened:

- `design_spec.md` header: `Date started` and `Last updated` (update the latter whenever you revise the spec), as `YYYY-MM-DD`.
- Each test run stored under `results/<letter>-<slug>/`: record the run timestamp (`YYYY-MM-DD HH:MM` local time) — keep the harness's timestamped directory name when copying from `tests/results/<timestamp>/`, or prefix artifact filenames / note the timestamp in the section of `results_summary.md`.
- `results_summary.md` header: the date written; each per-section **Setup** line: when that run was executed.

## Workflow

### 1. On starting a feature — write `design_spec.md`

Before writing any implementation code, create `notebook/NNN-feature-slug/design_spec.md` from [assets/design_spec_template.md](assets/design_spec_template.md). It must capture:

- **Problem context** — what the developer is trying to solve, in the developer's own framing from the session: motivation, constraints, prior attempts, and any decisions already made in the conversation. This is the section that preserves context which exists nowhere else.
- **Proposed implementation** — the design: affected packages, new/changed nodes and topics, algorithms, data flow. Diagrams (mermaid) welcome. Split into subsections if the implementation has multiple parts.
- **Test plan** — lettered sections `(a)`, `(b)`, `(c)`… each describing one validation axis: what is run (unit test, system test mark, sim scenario), what is measured, and what outcome counts as pass. These letters define the `results/` subfolder names.

If the design changes materially mid-implementation, update the spec — it should describe what was actually built, with a short note on what changed and why.

### 2. While implementing — keep the spec's status labels current

`design_spec.md` carries an implementation status at two levels, using the values **`DESIGN/TODO`**, **`WIP`**, or **`DONE`**:

- **Overall status** in the header block — the least-advanced status of any implementation section (all sections `DONE` → overall `DONE`; anything in progress → `WIP`; nothing started → `DESIGN/TODO`).
- **Per-section status** on each Proposed Implementation subsection heading (e.g. `### 2.1 Cost-map integration — \`WIP\``) — so when the implementation has multiple parts, a reader can see exactly which parts are designed, in progress, or finished.

Update the labels **as you work**, not retroactively: mark a section `WIP` when you start writing its code and `DONE` when it is implemented and building. A spec whose statuses lag reality misleads the next agent that picks up the feature.

### 3. During validation — store raw results

Every test run that validates the feature drops its artifacts into the matching section folder, e.g. `notebook/001-add-new-planner/results/a-planner-core/`:

- Metrics files (`metrics.json`, CSVs), copied from `tests/results/<timestamp>/` when using the system test harness
- Plots and screenshots (cross-track error curves, Foxglove/RViz captures, sim screenshots)
- Relevant log excerpts — excerpts, not full container logs

Keep raw artifacts as-produced; interpretation belongs in the summary. Preserve the run's timestamp with the artifacts (keep the `tests/results/<timestamp>/` directory name, or timestamp-prefix the copied files) so repeated runs of the same section stay distinguishable and ordered.

### 4. After validation — write `results/results_summary.md`

Create `results/results_summary.md` from [assets/results_summary_template.md](assets/results_summary_template.md). One section per test-plan letter, mirroring the spec. The summary must be **self-contained**: embed the quantitative tables and qualitative figures directly in the document (markdown tables; images via relative paths like `![xte](a-planner-core/trajectory_plot.png)`) so a developer can understand the results all at once without opening the raw artifact folders. End with an overall verdict: which spec sections passed, which didn't, known limitations.

### 5. On opening the PR — populate it from the notebook

The PR body for the feature is built from the notebook, since reviewers cannot see `notebook/` itself:

- **Motivation / context** ← `design_spec.md` problem context
- **What changed** ← proposed implementation (as-built)
- **Validation** ← `results_summary.md`: paste the summary tables, upload the key figures as PR attachments, and state the per-section verdicts

## Pitfalls

- ❌ Writing the spec after the code — the spec exists to record intent and session context before they're lost.
- ❌ Stale status labels — a spec still marked `DESIGN/TODO` (or a section marked `WIP`) after the work shipped misleads the next reader; update statuses as you go.
- ❌ Committing `notebook/` or referencing `notebook/...` paths from committed code, docs, or tests — it doesn't exist on other machines or in CI.
- ❌ Results subfolder letters that don't match the spec's test-plan letters.
- ❌ Undated documents or results — a spec without `Date started`/`Last updated`, or test artifacts with no run timestamp, can't be sequenced against other runs or the code they tested.
- ❌ A `results_summary.md` that just links to raw files — embed the tables and figures.
- ❌ Confusing this with [capture-discovered-knowledge](../capture-discovered-knowledge): the notebook records *per-feature* design and evidence locally; durable repo-wide knowledge still goes to AGENTS.md/skills, and module documentation still follows [update-documentation](../update-documentation).
