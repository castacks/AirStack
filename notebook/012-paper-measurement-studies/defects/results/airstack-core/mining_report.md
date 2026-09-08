# Defect Mining Report — AirStack-core

Feeds Sec. VI-B "Where Bugs Are Found: Defect-Discovery Shift" of the ICRA 2027
AirStack paper. Companion dataset: `defects.csv` (96 rows). This is the paper's own
repository, mined with the same two-stage protocol as the case-study teams.

---

## 1. Method

**Project:** AirStack-core. **Repo:** `castacks/AirStack` (GitHub), local clone at
`/home/andrew/Development/AirStack` (full history; branch `airstack-paper` with uncommitted
changes, used read-only).

**Window:** 2026-04-28 to 2026-09-08. Base `2624ffd7b43e8e6e3222ffe39f92e96205116c40`,
head `origin/develop` = `3cddcb24d1f190fe936aae91f208935cea5d8e83`. Branches: merged PRs
into `develop` and `main`, plus direct pushes to `main` in the window (the CI bring-up
commits of 2026-04-28..30 and 2026-05-20 landed without PRs).

**Mining date:** 2026-09-08. **Agent/model:** Claude (Claude Fable 5.1) via Claude Code.

**Stage 1 (enumeration)** was produced by `mine_defects.py`; its exact
commands, counts and the run/PR/issue inventories are in
`results/airstack-core/denominators.json`, `pr_timeline.{md,json}`, `candidates.md`,
`commits.tsv` and `issues.json`. Stage 2 (this report) did not re-enumerate.

**Stage 2 commands actually run** (read-only; no checkout/reset/stash/pull):

```bash
# every failing run in the 58 RED->FIX sequences (62 unique run IDs) + 21 further failures
gh run view <run_id> --repo castacks/AirStack                     # job list, conclusions
gh run view <run_id> --repo castacks/AirStack --log-failed        # failed-step logs (HTTP 410 when expired)
gh run view <run_id> --repo castacks/AirStack --log               # full logs for 30657350644 32073548050
                                                                  #   32528385863 32529755897 31219923715
gh run list --repo castacks/AirStack --workflow system-tests.yml --limit 200 \
   --json databaseId,event,conclusion,headBranch,headSha,createdAt,displayTitle
# PR-branch commits absent from the local clone after squash merges
gh api repos/castacks/AirStack/commits/<sha> --jq '{sha,date:.commit.author.date,msg:.commit.message,files:[.files[]|{f:.filename,a:.additions,d:.deletions,patch}]}'
#   (e41e30dcf3 50be34dd86 586849af62 cec4be2362 a9ab6774a7 38be4ba09b ab2daec373 053bce0cf2 8668e3f806 6a2d393ca4 2068677702 4e2b011268)
gh issue view <n> --repo castacks/AirStack --json title,state,createdAt,closedAt,body,comments   # 342 349 357 360 362 364
# diffs of every candidate fix commit (about 90 commits), and existence checks for PR-branch SHAs
git -C /home/andrew/Development/AirStack show --stat --format='%H%n%an %aI%n%B' <sha>
git -C /home/andrew/Development/AirStack show --format= <sha> -- . ':(exclude)*.md' ':(exclude).env'
git -C /home/andrew/Development/AirStack cat-file -e <sha>^{commit}
git -C /home/andrew/Development/AirStack show <squash_sha>:<path>      # verify review-flagged bugs were fixed in the merged tree
git -C /home/andrew/Development/AirStack log --all -i --grep=cyclone   # issue #364 follow-up check (none)
# dataset generation (scratchpad script): git rev-parse --verify <sha>^{commit}; git show --numstat --format= <sha>
python3 summarize_defects.py results/airstack-core/defects.csv --out <scratch>/val-core --verify AirStack-core=/home/andrew/Development/AirStack
```

**Classification rules applied.** One row per distinct root cause (a fix spread over
several commits is one row; one commit fixing independent things is several rows —
`fix_loc` then says "commit shared by multiple listed defects", do not sum it).
`ci_premerge` only where the failing run was opened and the following diff addresses that
failure; `venue_confidence` is `high` only when the log text was readable and matches the
fix, `medium`/`low` when the run had expired or the check was post-merge. Review-found bugs
have no venue in the taxonomy: they are `unknown` with `venue_confidence: high`,
`evidence_type: pr_thread`, and the description starts "found in PR review". Every
description is prefixed `[stack]` (autonomy/simulation code or configuration, including
the `airstack` CLI and Docker images the stack runs in) or `[harness]` (pytest harness,
workflow YAML, orchestrator, docs-deploy tooling). Pure VERSION-gate and branch-target
failures are counted in Sec. 3, not rowed. Documentation-only fixes were excluded per the
protocol, including the docs-site hotfix series of 2026-08-29 (PRs #414–#419, #421).

**Local-only rows.** Nine fixes exist only as PR-branch commits that the squash merge
dropped from the local clone (PRs #354, #359, #377, #378 and the Copilot autofixes in
#351). For those, `commit_sha` is the squash-merge commit (which resolves locally and
contains the fix) and `evidence_url` points at the PR-branch commit or review thread on
GitHub; `fix_files`/`fix_loc` name the PR-branch commit explicitly.

---

## 2. Denominators

From `denominators.json` (verbatim) unless marked "Stage 2":

| Quantity | Count | Notes |
|---|---:|---|
| Commits in window (non-merge, `origin/develop` not `2624ffd7`) | **171** | 144 Andrew Jong, 10 John Liu, 9 github-actions[bot], 4 pvkumara, 3 Krrish Jain, 1 Sebastian Scherer |
| Merge commits in window | 24 | |
| Keyword-hit candidate commits | 77 | all 77 read; `candidates.md` |
| Merged PRs | **52** | 38 into `develop`, 14 into `main`; all 52 walked in `pr_timeline.md` |
| PR review comments | **138** | full bodies read for #350, #351, #359, #365, #376 |
| RED→FIX sequences (failing check on a PR commit, later push to the same PR) | **58** | 62 unique failing run IDs; dispositions in Sec. 3 |
| Issues in window (created or closed) | **17** | 6 read in full (#342, #349, #357, #360, #362, #364) |
| GitHub Actions runs in window | **926** | CI history **reachable** (`gh` authenticated) |
| — System Tests, `pull_request` | 81 | 22 success / 24 failure / 35 cancelled |
| — System Tests, `issue_comment` (`/pytest`) | 41 | 17 failure / 16 skipped / 8 cancelled |
| — System Tests, `workflow_dispatch` | 41 | 6 success / 25 failure / 10 cancelled |
| — Unit Tests, `pull_request` | 35 | 20 success / 15 failure |
| — Check VERSION Increment | 249 | 230 success / 19 failure |
| — Enforce Branch Targets | 167 | 162 success / 5 failure |
| Run logs still retrievable (Stage 2) | **45 of 62** RED→FIX runs | 17 expired (HTTP 410; every run created on or before 2026-05-29). 21 further runs opened, all readable |
| **Defects recorded** | **96** | `defects.csv` |

---

## 3. Summary tables

### 3.1 Defect class × discovery venue (96 rows)

| class | `ci_premerge` | `sim_interactive` | `bench_hardware` | `field` | `unknown` | total |
|---|---:|---:|---:|---:|---:|---:|
| `remap_topic` | 0 | 0 | 0 | 0 | 6 | **6** |
| `tf_frame` | 0 | 1 | 0 | 0 | 0 | **1** |
| `parameter` | 3 | 1 | 0 | 0 | 16 | **20** |
| `timing_clock` | 1 | 3 | 0 | 0 | 4 | **8** |
| `build` | 8 | 1 | 3 | 0 | 10 | **22** |
| `logic` | 6 | 2 | 0 | 0 | 22 | **30** |
| `other` | 0 | 0 | 0 | 0 | 9 | **9** |
| **total** | **18** | **8** | **3** | **0** | **67** | **96** |

Integration-wiring classes (remap/TF/parameter/timing/build): 57 of 96. Class confidence:
high 49, medium 45, low 2. Venue confidence: high 27, medium 5, low 64 (`unknown` rows
are recorded `low`). Experiment-ruining proposal: yes 1 (row 032), uncertain 29, no 66.

### 3.2 `[stack]` vs `[harness]`

| | `[stack]` | `[harness]` | total |
|---|---:|---:|---:|
| `ci_premerge` | 5 | 13 | 18 |
| `sim_interactive` | 3 | 5 | 8 |
| `bench_hardware` | 3 | 0 | 3 |
| `unknown` | 54 | 13 | 67 |
| **total** | **65** | **31** | **96** |

By class, `[stack]`: build 18, logic 16, parameter 15, other 7, remap_topic 6,
timing_clock 3. `[harness]`: logic 14, parameter 5, timing_clock 5, build 4, other 2,
tf_frame 1 (row 046: the waypoint test's frame-less Path crashed `droan_gl`; the crash is in
the stack, the fix landed in the test client).

### 3.3 Disposition of the 58 RED→FIX sequences

| Disposition | Sequences | Notes |
|---|---:|---|
| Version-increment gate only (`Check VERSION Increment`) | 15 | 13 unique; `a871838ee5` and `06981aa5f7` are listed under both #396 and #410. One (#352) was `VERSION 'droan_gl_test' does not match the required format`. Process catches, not rowed |
| Branch-target gate only (`Enforce Branch Targets`) | 3 | #359 `37119e4981`; #419 `9ef135a795`, `d341ea10a9` (hotfix PRs opened against the wrong base). Not rowed |
| Gate failure **plus** a defect | 3 | #403 `2222efdd35` → row 094; #410 `7d2782100b` → rows 089/090; #421 `795b502dfd` → row 096 |
| Infrastructure flakiness / no code fix | 5 | "self-hosted runner lost communication": #348 `54befc772e`, #378 `586849af62`, #381 `d1364bef53`; ms-airsim process crashed during CI bring-up: #403 `7d2b4dae9d`, `be9ed0d6a2` (the follow-up commits reclassify such failures as infrastructure; no simulator fix) |
| Log expired, following pushes unrelated | 2 | #348 `e0432879a9`, `d3096ab454` (merge commits) |
| Failure readable but no fix in the sequence | 2 | `55d9b887d9` docker-build on `develop` push (apt exit 100), listed under #398 and #410 |
| **Confirmed defect** (failing run opened, following diff fixes it) | **28** | maps to 11 distinct rows, see below |

The 31 defect-bearing sequences (28 + 3 mixed) resolve to **11 distinct rows**: 004 (submodule
checkout), 030 (unit tests need ROS), 044 (overlayfs DinD), 049 (`.env` inline-comment
parsing), 050 (isaac-sim dpkg; caught three times: `31210711111`, `31220240599`,
`31237536714`), 062 (host collection sweep), 088 (gh-pages push race; 6 sequences),
089 + 090 (Metrics-Report `yaml` crash and unit-test environment; **14 sequences** across
PRs #388, #395, #396, #398, #400, #403, #410, #411), 094 (`--config-only` skipped the
AUTONOMY_ROLE check), 096 (`!ENV` in the contract loader). Seven of the 58 sequences are
exact duplicates because PR #410 (release to `main`) re-lists `develop` commits.

Seven further `ci_premerge` rows come from failing runs **outside** the Stage-1 RED→FIX
set — `issue_comment` (`/pytest`) and `workflow_dispatch` runs on PR branches, and one
issue: rows 001 (issue #349, AirSim on the headless CI instance), 003 and 005 (the
2026-04-30 `/pytest` workflow bring-up on `main`), 051, 053, 054, 055 (the `/pytest` runs
on the OSMO branches, 2026-08-10..12).

### 3.4 Review-found defects

**13 rows** were identified in PR review threads and fixed before merge: 009–015 (PR #351),
017, 018, 020, 021, 022 (PR #350), 031 (PR #359). All 13 were raised by the GitHub Copilot
code-review bot; the human review comments in the window (#359, #365) produced
documentation changes and one revert of an inert parameter change (`sphere_radius`), not
defect rows. Two further Copilot findings were checked and found **not** addressed in the
merged tree (`scene_prep.py` OmniGraph `SET_VALUES` tuple form, PR #350/#354;
`BodyBinding.from_dict` rejecting an empty `target_prim`, PR #376) and are not rowed.

### 3.5 Which automated job caught each `ci_premerge` row

| Catching job | Rows | n |
|---|---|---:|
| System Tests → `build_docker` (image builds) | 044 (also `build_packages`), 050 | 2 |
| System Tests → `build_packages` (`colcon test` inside the built robot container) | 051, 053, 054 | 3 |
| System Tests → checkout / submodule step | 004 | 1 |
| System Tests → pytest session start / collection (before any test ran) | 055, 062 | 2 |
| System Tests → Metrics Report job | 089 | 1 |
| System Tests → workflow trigger steps (`/pytest` plumbing) | 003, 005 | 2 |
| System Tests → simulator bring-up (liveliness-class), `workflow_dispatch` on `main` during CI bring-up | 001 | 1 |
| Unit Tests workflow (`ubuntu-latest`, contract tests) | 030, 090, 094, 096 | 4 |
| docker-build workflow (Auto Build on Docker Image Tag Change) | 049 | 1 |
| Docs-deploy workflows (post-merge push) | 088 | 1 |
| **liveliness / sensors / takeoff_hover_land / autonomy flight tests on a PR** | — | **0** |

No PR-triggered simulated-flight test produced a RED→FIX defect in this window. PR-open
System Tests were deliberately build-scoped from 2026-08-14 (`f937b50fb1`, `4a78ea28cf`:
"Automatic PR validation is deliberately build-scoped"), 35 of 81 PR-triggered System Tests
runs were cancelled (superseded pushes), and of the 24 PR-triggered failures 15 were the
single Metrics-Report crash (row 089) with the `Run Tests` job green, 3 were runner loss,
3 were the collection sweep (row 062 lineage: `29848590175`, `31770222796`, `32073548050`),
1 was the submodule checkout (row 004) and 2 are expired. Flight-class runs happened via
`/pytest` and `workflow_dispatch` (41 each) and locally; the defects they found are the
`sim_interactive` rows 032, 046, 061, 072, 076–078 (venue confidence low where the
evidence is a local harness run rather than an explicit statement).

---

## 4. Example `ci_premerge` catches

The request preferred full-stack simulated-flight catches of remap/TF/param/timing
defects; the record contains none at the PR gate (Sec. 3.5), so the three most concrete
CI catches are given, all reproducible from the linked runs.

**Row 044 — overlayfs data-root killed every image build (`f56810dca5`).** The first
`/pytest` run on the new OSMO ephemeral runner (run `30659044300`, 2026-07-31) failed all
four `build_docker` and all four `build_packages` tests with one BuildKit error,
`mount source: "overlay" ... err: invalid argument`, because the inner dockerd kept its
data-root on the pod's overlayfs rootfs where Linux refuses to stack a second overlay; each
failure looked like an unrelated `apt-get`/`WORKDIR` error. The fix makes
`runner-entrypoint.sh` choose a storage backend by performing a real overlay mount before
dockerd starts and documents the signature (runs `30657350644`, `30659044300`).

**Row 054 — pytest 8 shadowed apt pytest 7.4 inside the robot image (`865eb0de95`,
earlier partial fix `0b131d62fe`).** `build_packages` runs `colcon test` inside the freshly
built robot container; in run `31640024989` (2026-08-12) every in-container pytest aborted
with `Argument(s) {'path'} are declared in the hookimpl but can not be found in the
hookspec`, because pip had pulled pytest>=8 over Jazzy's apt pytest 7.4 while the apt
`launch_testing` plugin still declared the removed `path=` hook. The image now pins
`pytest==7.4.*` and the harness sets `PYTEST_DISABLE_PLUGIN_AUTOLOAD` so cache images with
unpinned pytest still run the `lidar_point_cloud_filter` tests.

**Row 094 — a merge silently re-enabled a removed launch variable (`c4d2ca21c1`).**
Merging `develop` into PR #403 placed the removed-`AUTONOMY_ROLE` preflight check below the
new `--config-only` early return, so a stale `AUTONOMY_ROLE` in an env file would again
pass preflight without error; the Unit Tests job (run `33234614177`, 2026-08-29) failed
`test_launch_intent_contract.py::test_autonomy_role_set_is_fatal` with `assert 0 != 0`.
The fix moved the check ahead of the early return so the configuration contract also holds
on the hermetic path.

Two stack-side catches at other layers are worth naming: row 001 (issue #349), where the
ms-airsim image built on a non-desktop Ubuntu base crashed AirSim at launch on the headless
CI GPU instance during the CI bring-up week, and row 004 (run `25195327326`), where PR #350
pinned the `vdb_mapping_ros2` submodule to a commit that had never been pushed and the
System Tests checkout failed with `remote error: upload-pack: not our ref`.

---

## 5. Limitations

- **Expired logs.** 17 of the 62 failing runs in the RED→FIX set returned HTTP 410 (every
  run created on or before 2026-05-29). April–May sequences were classified from the run
  conclusion plus the following diff; rows 003, 005 and 030 therefore carry
  `venue_confidence: low`/`medium`, and two #348 sequences could not be attributed at all.
- **Squash merges.** All PRs were squash-merged; PR-branch fix commits survive only where
  the branch still exists on `origin`. Nine rows use the squash commit as `commit_sha`
  (Sec. 1); their `fix_files`/`fix_loc` name the PR-branch commit or are `n/a` where a
  Copilot autofix could not be isolated.
- **PR system tests were mostly not flight tests.** 35 of 81 PR-triggered System Tests
  runs were cancelled and PR-open runs were build-scoped from 2026-08-14; 15 of the 24
  PR-triggered failures were one harness crash (row 089) that turned every PR red from
  ~2026-08-20 to 2026-08-29 while the test job itself passed. Simulated flights ran through
  `/pytest`, `workflow_dispatch` (41 runs each, 25 + 17 failures, only partly examined) and
  on developers' machines. The `ci_premerge` column is therefore dominated by build,
  image and harness defects, and the `sim_interactive` rows were found by running the
  same pytest harness locally against Isaac Sim — a "desk-run test suite" venue the
  taxonomy does not have (Hummingbird's report notes the same gap); those six rows are
  `sim_interactive`/`low`.
- **Post-merge CI counted as `ci_premerge`.** Row 088 (gh-pages push race) was caught by
  push-triggered docs deploys on `develop`/`main`, not on a PR; it is `ci_premerge` with
  `venue_confidence: medium` and flagged in its description. Row 001's failing runs were
  `workflow_dispatch` on `main` during the CI bring-up week (issue evidence).
- **Venue is mostly unknown.** 67 of 96 rows have no positive evidence of where the defect
  was found; the mid-August RFC and audit commits describe root causes in detail but not
  the discovery setting. No `field` row exists: the trunk repo saw no field deployment in
  the window (hardware work was Jetson bench bring-up, rows 023, 037, 038).
- **Work outside PRs.** 31 commits went directly to `main` (2026-04-28..30 CI bring-up,
  2026-05-20 release plumbing); their only CI evidence is `workflow_dispatch`/`issue_comment`
  runs whose logs have expired. The nine RFC PRs (#388–#396) landed as one batch on
  2026-08-24 after the branch-local validation described in their messages; that validation
  is not in CI history.
- **Duplicated sequences.** PR #410 (release to `main`) re-lists `develop` commits, so 7 of
  the 58 sequences are duplicates of #388/#395/#396/#398 sequences.
- **Open issues not rowed.** #357 (Isaac Sim crashes with guest Nucleus credentials), #360
  (Foxglove WebGL context loss), #362 (slow Isaac scene load) and #364 (Fast DDS
  `/usr/local` library shadowing observed as sentinel nodes missing in liveliness runs; only
  a Cyclone DDS workaround, no commit) name concrete malfunctions but have no fix commit and
  are excluded. Issue #342 (Pegasus CPU load, RTF 0.38 → 0.75 via PR #348) is a performance
  change, not a malfunction, and is excluded.
- **Shared fix commits inflate `fix_loc`.** 16 rows share a commit with another row
  (`56814b3424`, `87a22f4b54`, `8242ae85cb` ×3; `2d71bf29a7`, `aa1e9acfb0`, `efb7c86860`,
  `0b7b33d85b`, `a8ce342d99`, `b07cf268cf`, `efad619056` ×2); their `fix_loc` is marked
  "commit shared" or restricted to the listed files. Do not sum `fix_loc`.
- **Documentation practice bias.** August commits (co-authored with coding agents) state
  root cause and symptom explicitly; April–May commits ("Fix bug", "bug fixes") mostly do
  not. Row density by month reflects that as much as defect incidence.
- **Excluded by design.** Documentation and docs-site fixes (18 verified doc defects in
  `6d0bf18d0e`, the 2026-08-29 hotfix series), style/lint, features, dependency bumps
  without a breakage, and the `sphere_radius` revert (`8209379201`, an inert value).
