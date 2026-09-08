# Defect Mining Report — DFM2 (Don't Fool Me Twice)

Feeds Sec. VI-B "Where Bugs Are Found: Defect-Discovery Shift" of the ICRA 2027
AirStack paper. Companion dataset: `defects.csv` (12 rows: 8 `castacks/airstack-dfm2`,
4 `castacks/DontFoolMeTwice`).

---

## 1. Method

**Project:** DFM2 — online disturbance adaptation: a `resilience` world-model ROS package
plus an Isaac Sim disturbance library (fan / spherical / conical force fields, strobe
lights, lens flare, ice floor). On AirStack the work is **simulation-only** (Isaac Sim at a
desk); the project's hardware runs were on a ground robot on a different stack and are
outside both repos.

**Repos mined (two), both pre-enumerated by Stage 1:**

| repo | visibility | scope | local clone (read-only) |
|---|---|---|---|
| `castacks/airstack-dfm2` | **private** fork of `castacks/AirStack` | fork point `19bf91d82380bc5a5cdd44bce06b265a23dd7524` → tip `3af8f8e0797207ac9d0c8232012ec1362101d04a` (`origin/main`), plus side branches (below) | `scratchpad/repos/airstack-dfm2` |
| `castacks/DontFoolMeTwice` | public | whole history, 34 commits, tip `4ec60f84a2416b17ff45e0aef4c613355c49cc62` | `scratchpad/repos/DontFoolMeTwice` |

Both clones are full (`git rev-parse --is-shallow-repository` → `false`); `gh` is
authenticated for both repos.

**Mining date:** 2026-09-08. **Agent/model:** Claude (Fable 5.1) via Claude Code.

**Stage 1** (`mine_defects.py`, run twice — once per repo) produced
`results/dfm2/airstack-dfm2/{denominators.json,commits.tsv,candidates.md,pr_timeline.md,pr_timeline.json,issues.json}`
and `results/dfm2/dontfoolmetwice/{denominators.json,commits.tsv,candidates.md}`. Its
exact commands are recorded verbatim in the two `denominators.json` files (`commands`
array). Key Stage-1 parameters: `airstack-dfm2` topological window
`3af8f8e0 --not 19bf91d8` (GitHub history bounded `--since=2026-02-01`; an earlier run of
Stage 1 also applied `--since` to commits and listed only 26 of the 38 non-merge commits —
the enumerator was fixed and re-run, so `denominators.json` now lists all 38); `DontFoolMeTwice` whole history to `4ec60f84`; keyword regex
`\b(fix|fixes|fixed|fixing|bug|bugs|broken|breaks?|repair|revert|hotfix|crash|regression|wrong|incorrect|fail(?:s|ed|ing|ure)?|error|issue|typo|workaround|patch)\b`.

**Stage 2 (this report) read the diff of every commit.** Commands used (all read-only):

```bash
# fork topology and side branches (airstack-dfm2)
git branch -r
git merge-base --is-ancestor 19bf91d8 <ref>                  # per remote ref
git rev-list --count 19bf91d8..origin/main                    # 43 (38 non-merge, 5 merges)
git log --format=... origin/<branch> --not origin/main        # navin/dfm2-integration, random-scene-gen, wind_sim
git grep -n -E '^(<<<<<<<|=======|>>>>>>>)' origin/<branch>   # conflict-marker check

# every commit read
git show --stat=120 --format='%H%n%aI %an%n%s%n%b' <sha>      # all 43 fork-delta commits
git show --format=... <sha> -- . ':!*.usd' ':!*.usdc' ...     # full textual diff, binaries excluded
git show --format=... <sha> -- . ':!*.pyc' ':!RayFronts/**' > scratchpad/dfm2-diffs/<sha>.diff   # all 34 DontFoolMeTwice commits
git diff 0f41de8b 0a246da0 ; git diff 038d54cf 56fde91a       # PR #1 "doesn't work yet" -> "Working" transitions
git show --numstat --format= <sha>                            # fix_files / fix_loc
git cat-file -e <sha>^{commit}                                # SHA verification, in the right clone

# GitHub side
gh pr list   --repo castacks/airstack-dfm2 --state all --json ...      # 1 PR
gh issue list --repo castacks/airstack-dfm2 --state all                 # 0
gh run list  --repo castacks/airstack-dfm2 --limit 200 --json ...       # 13 runs all-time, 1 failure
gh run view 21875848911 --repo castacks/airstack-dfm2 --json headSha,jobs   # the failed run
gh run view 21875848911 --repo castacks/airstack-dfm2 --log-failed      # HTTP 410: logs expired
gh pr list / gh issue list / gh run list --repo castacks/DontFoolMeTwice   # all empty
gh repo view castacks/{airstack-dfm2,DontFoolMeTwice} --json isPrivate,defaultBranchRef
```

**Refs included for `airstack-dfm2`.** Stage 1 covered `origin/main` only. Stage 2 extended
coverage to:

- the **whole fork delta** `19bf91d8..origin/main` — 43 commits (38 non-merge, 5 merges).
  All 38 were read, including the 12 authored 2026-01-27 → 2026-02-11 that the first
  (date-filtered) Stage-1 run had omitted: `f8877780, 4c9e56bf, f37a9faf, 09fc1f26, f5de5214, 6c0fb1ea,
  624598fd, 1f3383e0, 784b5c08, 143be32f, 1f4669e9, 81e767a5`.
- `origin/navin/dfm2-integration` — 13 commits not on `main` (8 by Navin, 5 re-authored
  copies of `main` commits by Andrew Jong; the 5 duplicates were dropped).
- `origin/random-scene-gen` — 11 commits not on `main` (10 non-merge, by krrishj18).
- `origin/wind_sim` — 10 commits not on `main` (9 non-merge): the pre-squash history of
  PR #1; read for the "doesn't work yet → Working" transition, no new rows.
- `origin/IceFloor` and `origin/StrobeLight` point at the same SHA as `origin/main`
  (`3af8f8e0`); nothing extra. `origin/gh-pages` (3 docs commits) and the upstream
  mirrors `up/main`, `up/develop` were excluded.

Note on the fork point: `19bf91d8` (upstream *"Fully automated standalone mode…"*,
2026-02-11 19:26 -05:00) enters `main` via the upstream merge `10480b38`; the fork's own
pre-2026-02-11 commits are therefore *not* descendants of it but *are* in
`19bf91d8..origin/main`. The delta set is used as-is; it contains no upstream commits.

**Deduplication.** One row per root cause: the arrows-vs-cone fixes in `a2708e5f` are two
rows (different mistakes); the three lamp-path commits on `random-scene-gen`
(`17d58737` → `9b14c171`) are one row; the two commits that bracket the semantic-voxel
"drift" (`acb08de6` states it, `5bbd2c56` fixes it) are one row.

**Venue rule applied strictly.** Every row is `unknown`. No commit message, PR thread,
issue, or note in either repo states *where* a malfunction was observed. By project
description all AirStack-side work was desk Isaac Sim, but that is project-level
knowledge, not per-defect evidence, so it was not used to upgrade any row (see
Limitations).

---

## 2. Denominators

### `castacks/airstack-dfm2` (private)

| Quantity | Count | Notes |
|---|---:|---|
| Non-merge commits, fork delta `19bf91d8..origin/main` | **38** | `denominators.json`: `commits_in_window: 38`; all read in Stage 2 |
| Merge commits in fork delta | 5 | 4 upstream merges + `378ecda5` (fork-internal merge) |
| Side-branch commits not on `main` read | 34 | `navin/dfm2-integration` 13 (5 duplicates dropped) · `random-scene-gen` 11 · `wind_sim` 10 |
| Keyword candidates (Stage 1) | 4 | `fa089a78, 341e9e17, d9ccd660, a2708e5f` — all four became rows |
| Merged PRs | **1** | #1 "Wind sim" (`wind_sim` → `main`, 2026-02-11), 10 commits, **0** review comments, **0** issue comments |
| Issues (any state) | **0** | |
| GitHub Actions runs, Stage-1 window (since 2026-02-01) | 9 | 8 `success`, **1 `failure`** (the 2026-02-10 docker-build run below) |
| GitHub Actions runs, all-time | **13** | 2026-01-27 → 2026-02-19; **1 failure** (run `21875848911`, 2026-02-10, `docker-build` job) — see DFM2-005 |
| Red→fix sequences on a PR | **0** | PR #1 commits have no check runs at all |
| **Defects recorded** | **8** | 5 on `main`, 2 on `random-scene-gen`, 1 CI run |

CI history **is reachable but structurally thin**: the only workflow is
`Auto Build on Docker Image Tag Change` (push to `main`/`develop` touching `.env`; the
`docker-build` job runs only when `DOCKER_IMAGE_TAG` changes). It executed a real build
exactly once (2026-02-10) and failed; every other run skipped the build job. Nothing runs
on pull requests, so `ci_premerge` is structurally unavailable. Logs of the failed run
have expired (HTTP 410).

### `castacks/DontFoolMeTwice` (public)

| Quantity | Count | Notes |
|---|---:|---|
| Commits, whole history | **34** | `denominators.json`; 0 merges |
| Commits whose diff was read | 34 | filtered to non-`.pyc`, non-`RayFronts/` (vendored) |
| Keyword candidates (Stage 1) | 5 | `acb08de6, 5bbd2c56, 310fd867, cd352806, 9a580c08` — 3 became rows (`5bbd2c56` absorbing `acb08de6`, `9a580c08`); `310fd867` and `cd352806` are feature drops despite "fixes"/"patch" |
| Merged PRs / PRs any state | **0** | `gh pr list --state all` empty |
| Issues (any state) | **0** | |
| GitHub Actions runs | **0** | `gh run list` empty; no workflows in repo |
| **Defects recorded** | **4** | |

---

## 3. Summary table — defect class × discovery venue

| class | `ci_premerge` | `sim_interactive` | `bench_hardware` | `field` | `unknown` | total |
|---|---:|---:|---:|---:|---:|---:|
| `tf_frame` | 0 | 0 | 0 | 0 | 2 | **2** |
| `parameter` | 0 | 0 | 0 | 0 | 2 | **2** |
| `timing_clock` | 0 | 0 | 0 | 0 | 2 | **2** |
| `build` | 0 | 0 | 0 | 0 | 2 | **2** |
| `logic` | 0 | 0 | 0 | 0 | 2 | **2** |
| `other` | 0 | 0 | 0 | 0 | 2 | **2** |
| **total** | **0** | **0** | **0** | **0** | **12** | **12** |

Per repo: `castacks/airstack-dfm2` 8 (build 1, tf_frame 2, logic 1, other 2,
parameter 2); `castacks/DontFoolMeTwice` 4 (timing_clock 2, logic 1, build 1).

Confidence: `class_confidence` high 1 / medium 8 / low 3; `venue_confidence` low 12
(all `unknown`). Experiment-ruining proposal: **yes 0, uncertain 4, no 8**.
Integration-wiring classes (remap/TF/parameter/timing/build): 8 of 12.

**Read of the table.** The venue column is uninformative for this project — not because
defects were found in the field, but because nobody wrote down where they were found.
Twelve rows over 72 distinct commits is a low yield; the two repos are research-code
drops with terse subjects ("mature GP fit", "is this dfm2?", "Whatever changes navin
made"), so most of what was classified rests on reading the diff (8 of 12 at `medium`).
Five of the eight fork rows are visual/asset defects in the Isaac Sim disturbance library
(fan model orientation, blade parenting, arrow and cone placement) fixed within one
working day (2026-02-12); the two `random-scene-gen` rows are config/asset-path breakage;
the DontFoolMeTwice rows are two cross-node data-handoff timing bugs, one double-counting
logic bug, and one CMake version error.

---

## 4. Example candidates

The prompt asks for the most vivid `ci_premerge` catches. **There are none: neither repo
runs any check on pull requests, and the one PR has zero check runs.** The closest thing
is a post-merge push-CI failure that the taxonomy cannot place, plus the two most
concrete diff-verified rows.

**`DFM2-005` — the one automated build ever attempted, and it failed.**
The fork's only workflow builds and pushes the compose images when `DOCKER_IMAGE_TAG`
changes in `.env`. That happened once, on `784b5c08` ("Change version to 0.0.1",
2026-02-10), thirteen minutes after `624598fd` had added the DFM2/RayFronts dependency
stack to `Dockerfile.robot`; the `docker-build` job failed, the logs have since expired
(HTTP 410), the tag never changed again so the job never re-ran, and the Dockerfile at the
tip still carries the line `apt install ros-humble-vision-msgs && apt install
ros-humble-ackermann-msgs` without `-y` (a known non-interactive-build breaker — but the
log is gone, so the row does not claim this was the cause). Recorded as `build`/low,
venue `unknown` because the taxonomy has no post-merge-CI cell.

**`DFM2-001` — semantic voxels "drifting" because RGB, depth and pose were never
synchronised.** `acb08de6` ("Voxel Mapping works, need to fix drift") captured *the
latest* pose and depth whenever an RGB frame arrived and projected the VLM hotspot mask
with them; `5bbd2c56` ("Drift fix, will clean and optimise") replaces that with a
timestamp-matched sync buffer (80–120 ms tolerance, interpolation, sync-quality stats) and
filters non-finite/zero depths. `timing_clock`, medium — the diff is unambiguous about the
mechanism, the message only names the symptom.

**`DFM2-011` — an `.env` with git conflict markers in it.** `a8985439` ("added modular
warehouse", `random-scene-gen`) committed `<<<<<<< Updated upstream … ======= …
>>>>>>> Stashed changes` around `ISAAC_SIM_SCRIPT_NAME`; `5f9db7f6` ("fixed .env") removed
the four lines twelve minutes later. `parameter`, medium; `experiment_ruining_proposed`
no — bring-up fails at env parse before any session starts.

---

## 5. Limitations

- **Private repo auditability.** Eight of twelve `evidence_url`s point into
  `castacks/airstack-dfm2`, which is private; a reviewer needs org access to open them.
  The failed CI run's logs are already unrecoverable (`--log-failed` → HTTP 410), so that
  row is auditable only as a job-level `failure` conclusion, not a cause.
- **No PR threads, no issues, no pre-merge CI, in either repo.** `ci_premerge` and
  `pr_thread` evidence types are structurally absent. PR #1's ten commits have no check
  runs. DontFoolMeTwice has no workflows at all and was developed by direct push.
- **The taxonomy lacks a post-merge-CI venue.** `DFM2-005` was surfaced by a push-triggered
  workflow on `main`, not on a PR and not by a human. Following the Hummingbird precedent
  for taxonomy gaps it is recorded `unknown`; the paper authors may prefer a
  `ci_postmerge` cell.
- **All venues are `unknown` by evidence, though the venue is known by project context.**
  Every AirStack-side defect here can only manifest inside a running Isaac Sim scene, and
  the project lead states the work was desk simulation. No commit says so, so per protocol
  no row was upgraded to `sim_interactive`. If the authors accept project-level venue
  attribution, all eight `airstack-dfm2` rows would move to `sim_interactive`; the four
  DontFoolMeTwice rows (pre-AirStack, ground-robot era) would not.
- **Terse messages → diff-inferred classes.** Eight rows are `medium` because the message
  names at most a symptom ("Fix arrows", "Drift fix"). Two rows (`DFM2-002`, `DFM2-003`)
  quote code comments from the diff rather than a message, marked "(diff …)" in
  `evidence_quote`. Three rows (`DFM2-005`, `-008`, `-010`) are `low`: the fix is a binary
  USD asset or an expired log, so only the subject line is available.
- **Deliberately excluded borderline items** (listed so the authors can reverse the call):
  - PR #1's "Add fan force field (doesn't work yet)" → "Working force field" → "working
    force fields" iterations (`0f41de8b`→`0a246da0`, `038d54cf`→`56fde91a` on `wind_sim`):
    the first approach applied `PhysxForceAPI` to an empty Xform (no rigid body) and the
    drone later needed `dynamic_control.apply_body_force` on its articulation body. Treated
    as feature-under-development rather than a malfunction in a working system.
  - `f8877780` comments out a blocking `result.wait()` on the global-planner toggle
    service inside a timer callback and unconditionally reports success — plausibly a
    deadlock workaround, but the message says nothing and it could equally be "no such
    service in this setup".
  - `bc2710ab` (`navin/dfm2-integration`) rewires the resilience configs from ground-robot
    topics to AirStack topics (`/robot_1/sensors/front_stereo/left/image_rect`,
    `/robot_1/odometry_conversion/odometry`, `sensor_frame: base_link_ZED_X`) — the
    `remap_topic` class the paper cares about, but it is initial integration, not a fix.
    The same commit committed `<<<<<<< HEAD … >>>>>>> 2b0b40a` conflict markers into
    `example_px4_pegasus_with_fan_force_field.py`; they are **still present at the branch
    tip** (a `SyntaxError` if that launch script is selected), never fixed, never
    documented, so no row.
  - `26237586` re-enables the `/vlm_answer` subscription that `bc2710ab` had stubbed with a
    hard-coded `"box fan"` answer for the 2026-02-12 demo — reverting a demo hack.
  - DontFoolMeTwice `6d188040`/`84f3743a` (external-path timeout handling), `414480ea`
    ("FLU convention" — establishes a frame convention in analysis scripts, no prior wrong
    frame identified), `310fd867` ("mapping fixes" — adds inactivity export and a
    25-point narration window; no malfunction identifiable), `cd352806` ("patch" = new
    capability), and the 23.8k-line `1ec9c004` "merge all changes" (performance
    "bottleneck" fixes and defensive guards only).
- **Shared fix commits inflate `fix_loc`.** `DFM2-006` and `DFM2-007` share `a2708e5f`;
  `fix_loc` is the whole commit's count on both rows, marked as such. Do not sum
  `fix_loc` across rows. `DFM2-003`'s commit is mostly a costmap rework around the
  dedup fix.
- **Two rows live only on a side branch.** `DFM2-011` and `DFM2-012` exist only on
  `origin/random-scene-gen` (never merged to `main`); their SHAs resolve in the clone and on
  GitHub but not from `origin/main`.
- **Stage-1 vs Stage-2 coverage.** Stage 1 (after the enumerator fix) lists all 38
  non-merge fork-delta commits on `main`; Stage 2 additionally read 34 side-branch commits
  not on `main`, from which two rows (`DFM2-011`, `DFM2-012`, both on `origin/random-scene-gen`) come.
- **Validation.** `python3 summarize_defects.py results/dfm2/defects.csv --out <scratch>`
  exits 0. All 12 `commit_sha`s resolve via `git cat-file -e <sha>^{commit}` in the
  correct clone (8 in `airstack-dfm2`, 4 in `DontFoolMeTwice`). Note that
  `summarize_defects.py --verify DFM2=<one path>` reports 4 unresolvable rows because it
  accepts a single clone per project; the DontFoolMeTwice SHAs were verified separately.
