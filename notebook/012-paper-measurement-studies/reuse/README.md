# Lines-of-code reuse study (Sec. V, Table I)

Lives in the AirStack repo (`notebook/012-paper-measurement-studies/reuse/`, branch
`airstack-paper`), not in the Overleaf-synced paper repo, so executable bits survive.

**Question.** For each case study, how much code did the team write, how much
pre-existing AirStack did it reuse unchanged, and how much of the base did it
have to modify in place?

**Method (one rule for all projects).** `git diff --numstat` between a pinned
BASE (the AirStack fork point = `git merge-base <team-branch> castacks/AirStack`)
and a pinned HEAD (the team's final code), with:

- *source* = code (`.py .cpp .h .hpp .c .js .sh …`) + config (`.yaml .xml .launch
  .cmake CMakeLists.txt Dockerfile .env .urdf .msg/.srv/.action …`); docs, data
  (`.json .rviz .ui .obj .usd`) and binaries reported separately;
- *vendored* = third-party or copied code the team brought in, listed per
  project in `projects.yaml` with a rationale, excluded from *authored*;
- *base excluded* = third-party code bundled inside AirStack itself (`glad`,
  `stb_image`, `xdot`, generated Foxglove `dist/` bundles, OpenVDB `Find*.cmake`,
  the vendored `natnet_ros2` driver), removed from every denominator;
- submodule pointers skipped; renames disabled so results match plain `git diff`.

Ratios: `authored / base`, `unmodified base / final` where
`final = base − authored_deleted + authored + vendored`, and base files / source
lines modified in place.

**Reproduce.**

```bash
cd notebook/012-paper-measurement-studies/reuse
./run_reuse_study.sh            # clones/fetches pinned refs into ./repos, writes ./results
```

`results/summary.md` is the cross-project table, `results/table_rows.tex` the
rows pasted into Table I, `results/<project>.md|json` the per-file detail
(every modified base file with its size, new files by directory, lines by
extension). `castacks/airstack-dfm2` is private: without access the DFM2 row
is skipped and the other four still run. The Hummingbird fork is a private
mirror the paper authors cannot see; its row transcribes the team's own report
(`hummingbird/CODE_REUSE_ANALYSIS_HUMMINGBIRD.md`, same git-diff method, its own
source definition) and the script recomputes only the public fork-point base.

`ICRA_2027_AirStack_Paper/analysis/dfm2_reuse_analysis.md` (2026-08-04) is an earlier, hand-run version of the
DFM2 measurement with a looser file filter; it agrees with `results/dfm2.md` on
the shape (~21k authored lines excluding vendored RayFronts, 97–99% in new
files, <0.3% of base touched) and is kept as a cross-check.

**Pinned inputs** (from `projects.yaml`):

| project | repo · branch | base (fork point) | head |
|---|---|---|---|
| RAVEN | castacks/AirStack `raven`; seungchan-kim/RayFronts `raven` (base = RayFronts trunk); seungchan-kim/LVLM; castacks/RAVEN | `2d4f4be3` (2026-02-20); `cded3eee` | `278acbff`; `8d838d79`; `7e5c652f`; `91ef2e7a` |
| DFM2 | castacks/airstack-dfm2 `main` (private) | `19bf91d8` (2026-02-11) | `3af8f8e0` |
| Hummingbird | JohnYanxinLiu/Hummingbird-AirStack (private mirror; author-reported) | `1c41f8c0` (2026-08-17) | `106dc726` |
| Shimizu | castacks/AirStack `junbin/planning_demo` | `39e5e698` (2025-11-07) | `a50c19e9` |
| Swarm CBF | castacks/AirStack `yikuan/SVG_ground_control` | `e4b499d1` (2026-05-20) | `564d43e4` |
