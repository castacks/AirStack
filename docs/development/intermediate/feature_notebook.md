# Feature Notebook

Every feature implemented with a coding agent gets a **notebook entry**: a numbered folder under `notebook/` at the repository root that records the design spec *before* implementation and the test results *after*. The notebook is a lab journal — it preserves the session context (problem framing, design decisions, validation evidence) that would otherwise be lost when the agent conversation ends, and it is the source material for the feature's pull request description.

!!! warning "Local-only"
    `notebook/` is **gitignored**. It never lands in a commit, and committed code, docs, and tests must never reference `notebook/...` paths — the directory doesn't exist on other machines or in CI. Its content leaves your machine one way: distilled into the feature's PR description.

## Directory Layout

```text
notebook/
├── 001-add-new-planner/
│   ├── design_spec.md              # Written BEFORE implementation
│   └── results/
│       ├── results_summary.md      # Written AFTER tests; feeds the PR
│       ├── a-planner-core/         # Raw artifacts for test section (a)
│       │   ├── run1_metrics.json
│       │   └── trajectory_plot.png
│       └── b-planner-hyperparameters/
│           └── sweep_table.csv
├── 002-fix-lidar-filter/
│   └── ...
```

- **Feature folders** are named `NNN-short-kebab-slug` with a zero-padded three-digit prefix. Pick the next number by incrementing the highest existing prefix (start at `001`).
- **Results subfolders** are named `<letter>-<section-slug>`, one per lettered test-plan section in `design_spec.md` — section "(a) Planner core" maps to `results/a-planner-core/` — so a reader can navigate spec ↔ results directly.

## Workflow

### 1. Before coding — write the design spec

Create `notebook/NNN-feature-slug/design_spec.md` with three parts:

- **Problem context** — what you're trying to solve, in your own framing from the session: motivation, constraints, prior attempts, decisions already made. This is the section that preserves context which exists nowhere else.
- **Proposed implementation** — affected packages, new/changed nodes and topics, algorithms, data flow. Split into subsections if the implementation has multiple parts.
- **Test plan** — lettered sections `(a)`, `(b)`, `(c)`…, each describing what is run, what is measured, and what counts as pass.

### 2. While implementing — keep status labels current

The spec carries an implementation status at two levels, using **`DESIGN/TODO`**, **`WIP`**, or **`DONE`**:

- An **overall status** in the header — the least-advanced status of any implementation section.
- A **per-section status** on each implementation subsection heading (e.g. `### 2.1 Cost-map integration — WIP`), so a reader sees exactly which parts are designed, in progress, or finished.

Update labels as you work, not retroactively — a spec whose statuses lag reality misleads the next person (or agent) who picks up the feature.

### 3. During validation — store raw results

Each test run drops its artifacts into the matching lettered section folder: metrics files (e.g. `metrics.json` copied from `tests/results/<timestamp>/`), plots, sim screenshots, and relevant log *excerpts*. Keep raw artifacts as-produced; interpretation belongs in the summary.

### 4. After validation — write the results summary

Write `results/results_summary.md` with one section per test-plan letter. It must be **self-contained**: embed the quantitative tables and qualitative figures directly in the document (markdown tables; images via relative paths) so a developer can understand all the results at once without opening the raw artifact folders. End with an overall verdict — which spec sections passed, which didn't, and known limitations.

### 5. Opening the PR — populate it from the notebook

Reviewers can't see `notebook/`, so the PR body carries the distilled content:

| PR section | Source |
|------------|--------|
| Motivation / context | `design_spec.md` problem context |
| What changed | Proposed implementation (as-built) |
| Validation | `results_summary.md` — paste the tables, upload key figures as PR attachments, state per-section verdicts |

## Templates and Agent Skill

Fill-in templates for both documents, and the full agent-facing workflow (including pitfalls), live in the [`use-feature-notebook` skill](https://github.com/castacks/AirStack/tree/develop/.agents/skills/use-feature-notebook) under `.agents/skills/`. Coding agents are instructed via `AGENTS.md` to follow this workflow at the start of every feature implementation.
