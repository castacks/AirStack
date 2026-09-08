# Defect-Mining Prompt for Case-Study Teams

**Feeds:** Sec. VI-B "Where Bugs Are Found: Defect-Discovery Shift" of the
ICRA 2027 AirStack paper (`\label{sec:eval:defects}`).

**How to use (for team leads):**

1. Clone the repo(s) your project actually developed in (your AirStack fork
   and/or your project repo). Make sure full history is present
   (`git fetch --unshallow` if needed) and, if the repo is on GitHub, that
   `gh` is authenticated (needed for PR/CI mining; skip otherwise).
2. Fill in the `{PLACEHOLDERS}` in the prompt below.
3. Paste the prompt into a coding agent (e.g., Claude Code) running inside
   the clone.
4. Send back the two output files it produces: `defects.csv` and
   `mining_report.md`. Do not hand-edit `defects.csv` — if a row is wrong,
   tell us and we regenerate; the paper's claim is that this history is
   auditable.

The paper reports **counts and classes only** — never "engineer-days saved"
or any extrapolation — so the mining must be conservative: a smaller, fully
provenanced dataset beats a larger, fuzzy one.

---

## PROMPT (copy everything below this line)

You are mining the git/PR/CI history of a robotics research project for
**integration defects** — concrete malfunctions in a working system — and
recording **where each one was discovered**. The output feeds a systems
paper's claim that continuous simulation-based testing shifts defect
discovery earlier (pre-merge CI or desk simulation) instead of the field.
Your output must be fully auditable: every row cites a real commit, PR, CI
run, or issue that a reviewer can open and check. It is acceptable — and
expected — to return fewer rows with solid provenance rather than more rows
with guesses.

### Inputs

- Project name: `{PROJECT_NAME}` (one of: RAVEN, DFM2, Hummingbird,
  Shimizu, SwarmCBF, AirStack-core)
- Repo(s) to mine: `{REPO_URLS_OR_PATHS}`
- Mining window: `{START_DATE}` (e.g., the AirStack fork point or the date
  system tests went live in this repo) to `{END_DATE}`; branches:
  `{BRANCHES, default: default branch + merged PRs into it}`
- AirStack fork point, if this repo is a fork: `{FORK_SHA_OR_TAG, or n/a}`

### What counts as a defect

One row per **distinct defect**, not per commit (a defect fixed across
three commits is one row). Include:

- Bugs fixed in response to a failing CI run on a PR (fix commit pushed to
  the PR after a red check).
- Bugs identified in PR review threads and fixed before merge.
- Bug-fix commits on mined branches (`fix`, `bug`, `broken`, `repair`,
  `revert`, `hotfix`, and non-keyword commits whose diff is clearly a
  malfunction repair).
- Defects documented in issues, field-test notes, or flight-log postmortems
  in the repo, if they name a concrete malfunction.

Exclude: style/lint/formatting, documentation, feature work, refactors with
no misbehavior being fixed, dependency version bumps that weren't fixing a
breakage, and anything you cannot tie to a verifiable artifact.

### Procedure

1. **Enumerate, then report denominators.** Count everything you scanned:
   total commits in the window, merged PRs, closed issues, CI runs (if
   GitHub Actions history is reachable via `gh run list` / `gh api`).
   Record the exact commands you used. These denominators go in the report
   verbatim.
2. **Collect candidate defects** from the sources above (`git log --grep`,
   `gh pr list --state merged` + review threads, `gh api` for check runs,
   issue search). Read the actual diff or thread for each candidate — do
   not classify from the commit subject line alone.
3. **Deduplicate** into distinct defects.
4. **Classify each defect** on the two axes below.
5. **Write the two output files** in the current directory.

### Axis 1 — defect class (fixed taxonomy; do not invent classes)

| class | covers |
|---|---|
| `remap_topic` | topic name/remap/namespace mismatches, missing `ROBOT_NAME` namespacing, QoS mismatch preventing connection |
| `tf_frame` | wrong/missing frame ids, TF tree breaks, extrinsics applied in the wrong frame |
| `parameter` | wrong/missing params or config files, launch-arg wiring, `allow_substs` / substitution failures, units |
| `timing_clock` | sim time vs wall time, `/clock` readiness, startup ordering, races, timeouts |
| `build` | dependency/CMake/`package.xml`/colcon/Docker-image build breakage |
| `logic` | algorithmic/control/math errors inside a module (not integration wiring) |
| `other` | anything else — describe it; recurring `other`s may become a new class |

Give `class_confidence` = `high` only when the diff or log states it
outright; `medium` when inferred from the diff; `low` when inferred from
the message alone.

### Axis 2 — discovery venue (the paper's key variable)

| venue | required evidence |
|---|---|
| `ci_premerge` | a failing automated check on the PR/commit preceding the fix — link the run or the PR's check history |
| `sim_interactive` | thread/commit/notes explicitly say it was found running simulation at a desk (not by CI) |
| `bench_hardware` | found on hardware pre-deployment (bench, tethered, lab) |
| `field` | found during a field test / deployment / demo |
| `unknown` | no evidence of venue — use this freely; do not guess |

Venue requires positive evidence. A fix commit with no CI history and no
thread is `unknown`, not `sim_interactive`. Set `venue_confidence` the same
way as class confidence.

### Experiment-ruining proposal (advisory only)

For each defect, propose `yes`/`no`/`uncertain` for: *had this not been
caught where it was, would it plausibly have surfaced only in the field or
mid-way through an expensive interactive sim/hardware session and ruined
that session?* One short factual rationale. Two paper authors will make the
final call independently; your proposal is a first pass, so prefer
`uncertain` over confident speculation.

### Output file 1: `defects.csv`

Exact header (one row per defect; no extra columns; UTF-8; quote fields
containing commas):

```
project,repo,defect_id,date,evidence_type,evidence_url,commit_sha,defect_class,class_confidence,discovery_venue,venue_confidence,description,evidence_quote,experiment_ruining_proposed,ruining_rationale,fix_files,fix_loc
```

- `defect_id`: `{PROJECT_NAME}-001`, `-002`, … in date order.
- `date`: ISO date of discovery evidence (fallback: fix-commit author date).
- `evidence_type`: `ci_run` | `pr_thread` | `commit_msg` | `issue` | `notes_file`.
- `evidence_url`: full URL (or repo-relative path + SHA for local-only repos).
- `commit_sha`: full SHA of the (first) fix commit.
- `description`: ≤25 words, concrete ("depth topic remap missing robot
  namespace, local planner received no obstacles"), no adjectives.
- `evidence_quote`: verbatim short quote from the commit message, CI log, or
  thread that justifies the classification.
- `fix_files` / `fix_loc`: files touched and lines changed by the fix
  commit(s), from `git show --stat`.

### Output file 2: `mining_report.md`

Sections, in order: **(1) Method** — exact commands, repos, SHAs, window,
mining date, agent/model used. **(2) Denominators** — commits, PRs, issues,
CI runs scanned; whether CI history was reachable. **(3) Summary table** —
counts as defect class × discovery venue. **(4) Example candidates** — the
2–3 most vivid `ci_premerge` catches, each told in two factual sentences
(what broke, how the automated test caught it). **(5) Limitations** — what
you could not see (e.g., CI logs expired, squash merges hiding fix
history, work done outside PRs).

### Hard rules

- Every row must be independently verifiable from its `evidence_url` /
  `commit_sha`. No verifiable artifact → no row.
- Never fabricate, estimate, or extrapolate. No time-saved estimates
  anywhere. If you are unsure, mark `unknown`/`uncertain` and move on.
- Do not modify the repository. Read-only mining; write only the two output
  files.
- If a data source is unavailable (no `gh` auth, private CI, shallow
  clone), state that in Limitations and continue with what is available
  rather than approximating what is missing.
