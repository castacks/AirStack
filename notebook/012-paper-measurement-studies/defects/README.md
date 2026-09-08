# Defect-discovery mining (Sec. VI-B)

Lives in the AirStack repo (`notebook/012-paper-measurement-studies/defects/`, branch
`airstack-paper`), not in the Overleaf-synced paper repo, so executable bits survive.

**Question.** Where are integration defects found — pre-merge CI, interactive
simulation at a desk, the bench, or the field — and of what kind?

**Headline (2026-09-08, `results/summary.md`).** 204 distinct defects across
the six histories (108 in the five case studies, 96 on the AirStack trunk since
system tests went live). 61% are integration-wiring classes. Where the venue is
evidenced, case-study defects were found in interactive Isaac Sim sessions 40
times, on bench hardware 10 times, in the field 0 times, and in pre-merge CI 0
times — no case-study branch ever went through CI (system tests postdate all
five). On the trunk, 18 defects were caught pre-merge (5 stack, 13 harness), by
build/unit jobs; none by a flight-test job on a PR. Venue attribution follows
documentation practice (Hummingbird supplies 36 of the 40 simulation rows).

**Two stages.**

1. **Enumerate (deterministic)** — `mine_defects.py` lists every commit in a
   window with its message, files and line counts, flags fix-keyword hits, and,
   for GitHub repos, pulls every merged PR with its commits, the workflow runs
   and conclusions on each commit, review/conversation comments, and flags
   RED→FIX sequences (a failing automated check followed by a later push to the
   same PR). It writes `denominators.json` (counts + exact commands),
   `commits.tsv`, `candidates.md`, `pr_timeline.{md,json}`, `issues.json`.
   `run_defect_mining.sh` runs it for every project with the pinned windows.
2. **Classify (judgement, auditable)** — a person or coding agent follows
   `defect_mining_prompt.md`: read each candidate's diff/thread, keep only
   concrete malfunction repairs, dedupe by root cause, classify on the fixed
   two-axis taxonomy (defect class × discovery venue), and write
   `results/<project>/defects.csv` + `mining_report.md`. Every row cites a
   commit/run/thread URL from Stage 1. Venue needs positive evidence
   (`unknown` otherwise); the report records denominators and limitations.
3. **Aggregate** — `summarize_defects.py results/*/defects.csv hummingbird/defects.csv`
   validates the schema and writes `results/summary.md` (class × venue per
   project and pooled) and `results/table_defects.tex`; `--verify PROJECT=path`
   checks every cited SHA resolves in a local clone.

**Reproduce.**

```bash
cd notebook/012-paper-measurement-studies/defects
./run_defect_mining.sh          # Stage 1 for all projects -> results/<project>/...
# Stage 2: follow defect_mining_prompt.md per project (results already committed here)
python3 summarize_defects.py results/*/defects.csv hummingbird/defects.csv --out results
```

**Windows mined** (`run_defect_mining.sh`):

| project | repo · refs | window | GitHub history |
|---|---|---|---|
| AirStack-core | castacks/AirStack `develop` | 2026-04-28 (system tests live, `2624ffd7`) → 2026-09-08 | merged PRs into develop/main, all workflow runs, issues |
| RAVEN | castacks/AirStack `raven`; seungchan-kim/RayFronts `raven`; castacks/RAVEN | fork points → tips | none (no PRs) |
| DFM2 | castacks/airstack-dfm2 (private); castacks/DontFoolMeTwice | `19bf91d8` → `3af8f8e0`; whole history | 1 PR, 5 runs |
| Hummingbird | JohnYanxinLiu/Hummingbird-AirStack (private; team-run, files in `hummingbird/`) | `1c41f8c0` → 2026-09-01 | 0 runs in window |
| Shimizu | castacks/AirStack `junbin/planning_demo` | `39e5e698` → `a50c19e9` | none |
| Swarm CBF | castacks/AirStack `yikuan/SVG_ground_control` | `e4b499d1` → `564d43e4` | none |

Classification for RAVEN, DFM2, Shimizu, Swarm CBF and AirStack-core was done
by Claude (Claude Code) following the prompt, from the Stage-1 files, on
2026-09-08; Hummingbird was mined by its team (Claude Opus 5, 2026-09-01). The
`experiment_ruining_proposed` column is the miner's advisory proposal, not the
authors' adjudication.
