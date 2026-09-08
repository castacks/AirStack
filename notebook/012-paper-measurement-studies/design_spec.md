# 012 — Paper measurement studies: LOC reuse (Table I) and defect mining (Sec. VI-B)

**Status:** `DONE` (2026-09-08). Scripts and results moved here from the
paper submodule's `analysis/` on 2026-09-08 because that repo is
Overleaf-synced and Overleaf drops executable bits and silently skips
files; the paper keeps only prose and a pointer (`analysis/README.md`).

## Problem context

The ICRA 2027 paper's case-study accounting (Table I) and its
"where bugs are found" section (VI-B) must be reproducible from released
scripts, not hand-run once. Five case studies live in five different
places (AirStack branches, a composite RAVEN repo with submodules, a
private DFM2 fork, a private Hummingbird mirror), so one manifest of
pinned SHAs and one method for all of them is the deliverable.

## Implementation

- [`reuse/`](reuse/README.md) — `loc_reuse.py` + `projects.yaml` + `run_reuse_study.sh`;
  outputs in `reuse/results/` (`summary.md`, `table_rows.tex`, per-project md/json).
- [`defects/`](defects/README.md) — `mine_defects.py` (Stage 1, deterministic),
  `defect_mining_prompt.md` (Stage 2 protocol, copy of the paper's),
  `summarize_defects.py`; per-project `results/<p>/defects.csv` + `mining_report.md`;
  Hummingbird's team-run files in `defects/hummingbird/`.

## Test plan

- (a) Reuse: rerun `run_reuse_study.sh` on a clean clone reproduces `results/summary.md` — verified 2026-09-08 from this location.
- (b) Defects: `summarize_defects.py --verify` resolves every cited SHA in the AirStack-hosted projects — 127/127 on 2026-09-08.

## Results

See `reuse/results/summary.md` and `defects/results/summary.md`; the
paper-facing numbers and the VI-B reframing are logged in
[`../strategy.md`](../strategy.md) (2026-09-08 entry).
