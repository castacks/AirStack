# Defect Mining Report — Shimizu

Feeds Sec. VI-B "Where Bugs Are Found: Defect-Discovery Shift" of the ICRA 2027
AirStack paper. Companion dataset: `defects.csv` (16 rows).

---

## 1. Method

**Project:** Shimizu (construction-site exploration / inspection planning demo,
AirLab CMU; Gazebo and Isaac Sim, later a ModalAI VOXL vehicle with mocap and a
Livox lidar).
**Repo mined:** `castacks/AirStack`, branch `junbin/planning_demo`
(tip `a50c19e9edd092f2606432be3e046a090a10da10`). Local read-only clone at
`/home/andrew/Development/AirStack` (another branch checked out; the branch was
read only through `git show` / `git diff` / `git log` / `git grep`; nothing was
checked out, reset, stashed, pulled or modified).
**AirStack fork point:** `39e5e698cf3404abe70c208b99e7667db5a28ab6`.
**Mining window:** every commit reachable from the branch tip and not from the
fork point (`a50c19e9 --not 39e5e698`), i.e. 2025-08-12 → 2026-06-08 by author
date; mining date 2026-09-08.
**Agent/model:** Claude via Claude Code (model `claude-fable-5-1`).

**Stage 1 (enumeration)** was run beforehand with
`mine_defects.py`; its outputs are in this directory and were
taken as given, not re-derived: `denominators.json` (exact commands, counts,
git version), `commits.tsv` (all 41 non-merge commits), `candidates.md` (every
commit with full message and file list; 7 keyword hits).

**Stage 2 (this report) — commands used.** All `git` commands were run as
`git -C /home/andrew/Development/AirStack ...`:

```bash
# PR / CI denominator (the one GitHub query permitted for this branch)
gh pr list --repo castacks/AirStack --state all --head junbin/planning_demo      # -> empty, exit 0

# every one of the 41 non-merge commits: message + stat + full diff (binaries/rviz/usd/pcd excluded)
git show --stat=200 --format='COMMIT %H%n%aI %an%n%s%n%b' <sha>
git show --format='COMMIT %H%n%aI %an%n%s%n%b' <sha> -- . ':(exclude)*.rviz' ':(exclude)*.usd' \
    ':(exclude)*.dae' ':(exclude)*.stl' ':(exclude)*.png' ':(exclude)*.jpg' ':(exclude)*.pcd' ':(exclude)*.csv'
# for the 11 large commits (>600 diff lines) the modified-file hunks were read in full and
# new-file-only content was treated as feature work:
git show --diff-filter=M --format=... <sha> -- . ':(exclude)...'
git show --diff-filter=A --name-only --format= <sha> | wc -l      # A/M/D/R counts per large commit

# merge topology behind the 2025-09-10 fix cluster
git log --merges --format='%H %aI %an | %s | parents: %p' a50c19e9 --not 39e5e698
git log --format='%h %aI %an | %s' 39e5e698..origin/junbin/planning_demo -- robot/docker/docker-compose.yaml
git show <c>:robot/docker/docker-compose.yaml      # c in 7d754556 e0fff314 0c467a58 ae3f4550 773b42f1 a7a53ae7 e5790633

# root-cause verification
git ls-tree <c> robot/ros_ws/src/autonomy/4_global/a_world_models/        # vdb_mapping gitlinks at ed4fdc4e dde8d6b4 e0fff314 tip
git diff ed4fdc4e dde8d6b4 -- .gitmodules robot/ros_ws/src/autonomy/4_global/a_world_models/
git grep -n -i vdb_mapping 0ea9a854^ -- <exploration src/include/package.xml>   # -> no hits
git grep -n -i vdb_mapping 31f969e6^ -- <inspection_planning>                   # -> README only
git show ed4fdc4e:<exploration>/CMakeModules/FindOpenVDB.cmake | grep -n Blosc  # line 638: find_package(Blosc REQUIRED)
git grep -n sub_target_path_topic 37b3e0e4 -- <exploration>                     # declared in node, absent from config/launch
git grep -n safe_robot_r_ dd521a90^ -- <integrated_planner>                     # hard-coded 0.5 in Astar::initialize vs node param
git show <fix-sha> -- <file> | grep -n 'SensorDataQoS\|create_wall_timer\|rclcpp::create_timer'
git show --numstat --format= <fix-sha>                                          # fix_files / fix_loc
```

**Classification rules applied.** One row per distinct root cause. Feature
work, refactors, documentation, visualization-only changes, parameter tuning
(gains, speeds, voxel sizes, exploration bounds), hacks that disable a check
rather than repair it (`is_seen = true; is_free = true;` in
`disparity_graph_cost_map.cpp`, `advance_trackingpoint = true; // hacky: open
loop for now`), and submodule pointer bumps were excluded. Because commit
messages on this branch are terse ("another fix", "mod for shimizu sim demo",
"livox config"), the diff was the primary evidence for every row: `class_confidence`
is `high` only where the message or an in-diff comment names the malfunction,
otherwise `medium`. No commit, note or thread states where any defect was
observed, so every row is `discovery_venue = unknown` (`venue_confidence = low`).

**Deduplication.** `0ea9a854` and `593be4ac` are byte-identical diffs of the same
fix committed on two branches five minutes apart (one row, `Shimizu-003`), and
that row also absorbs `31f969e6`, which removes the same unresolvable
`vdb_mapping` dependency from a second package. `Shimizu-010` (odometry QoS)
spans `3431e263` and `dd521a90` (same change in two controller variants).
Conversely `393142b8` yields two rows, `dd521a90` three, `a50c19e9` two, each a
separate root cause; their `fix_loc` fields say so and must not be summed.

---

## 2. Denominators

| Quantity | Count | Notes |
|---|---:|---|
| Non-merge commits in the window | **41** | `git log --no-merges a50c19e9 --not 39e5e698` (denominators.json) |
| — messages read in full | 41 | every one |
| — diffs read | 41 | every one; for the 11 commits over 600 diff lines, all modified-file hunks plus stat of added files |
| — keyword-regex hits (Stage 1) | 7 | `fix|bug|broken|...` regex in denominators.json; 6 of 7 produced rows, 1 (`e579063` also counted) — see below |
| Merge commits in the window | **14** | `git rev-list --merges --count`; excluded from mining (see Limitations) |
| Authors | 3 | Junbin Yuan 35, caomuqing 4, YuanJunbin 2 |
| Pull requests for the branch, any state | **0** | `gh pr list --repo castacks/AirStack --state all --head junbin/planning_demo` returned nothing |
| PR review threads | 0 | no PRs |
| Issues | n/a | not mined: nothing on the branch references an issue and no `#N` appears in any message |
| CI runs | **n/a** | no PRs exist for the branch, so no PR check runs; workflow runs were not queried (outside the agreed read-only scope) |
| Notes / postmortem files | 0 | none on the branch |
| **Defects recorded** | **16** | `defects.csv` |

Of the 7 keyword hits, 6 became rows (`ed4fdc4e`, `2ed7fecd`, `e579063`,
`0ea9a854`/`593be4ac` as one, `a7a53ae7`, `f1f65593`); the remaining rows came
from non-keyword commits whose diffs are unambiguous repairs.

**CI history is structurally absent, not merely unreachable.** The branch was
developed by direct pushes with periodic merges from `main`; no PR was ever
opened for it, so `ci_premerge` and `pr_thread` evidence cannot exist here.

---

## 3. Summary table — defect class × discovery venue

| class | `ci_premerge` | `sim_interactive` | `bench_hardware` | `field` | `unknown` | total |
|---|---:|---:|---:|---:|---:|---:|
| `remap_topic` | 0 | 0 | 0 | 0 | 2 | **2** |
| `tf_frame` | 0 | 0 | 0 | 0 | 1 | **1** |
| `parameter` | 0 | 0 | 0 | 0 | 2 | **2** |
| `timing_clock` | 0 | 0 | 0 | 0 | 2 | **2** |
| `build` | 0 | 0 | 0 | 0 | 2 | **2** |
| `logic` | 0 | 0 | 0 | 0 | 4 | **4** |
| `other` | 0 | 0 | 0 | 0 | 3 | **3** |
| **total** | **0** | **0** | **0** | **0** | **16** | **16** |

Confidence distribution: `class_confidence` high 3 / medium 13 / low 0;
`venue_confidence` low 16 (venue is `unknown` on every row).

Experiment-ruining proposal (advisory): **yes 2** (`Shimizu-007`, `Shimizu-014`),
**uncertain 9**, **no 5** (the five build/compose defects, which fail before
anything runs).

**Read of the table.** Integration-wiring classes (`remap_topic` + `tf_frame` +
`parameter` + `timing_clock` + `build`) account for 9 of 16; the three `other`
rows are all docker-compose file breakages produced on a single evening of
branch merging (2025-09-10, `Shimizu-002/004/005`), which the fixed taxonomy has
no cell for — if other teams report the same, a "deploy_config" class would
absorb them. Only 4 rows are `logic` errors inside a module. Nothing can be said
about venue: the branch carries no CI, no PRs and no notes, and the messages do
not say where anything was observed.

---

## 4. Example candidates

The prompt asks for the most vivid `ci_premerge` catches. **There are none: this
branch never had a pull request or a check run.** The two most vivid rows of the
only venue that exists (`unknown`) are given instead; both are defects that a
pre-merge build-and-launch check would have exposed had one existed.

**`Shimizu-004` — a three-way merge deleted the tmux session that autolaunch
depends on.** Upstream `main` had moved `tmux new -d -s robot_bringup` inside the
`if [ $autolaunch == 'true' ]` block while the branch had edited the adjacent
launch line; when the two were merged (`0c467a58`, 21:08) the conflict was
resolved by keeping the branch's line, which left *neither* copy of `tmux new`,
so `tmux send-keys -t robot_bringup` targeted a session that did not exist and
autolaunch silently did nothing. Four minutes later a second merge
(`ae3f4550`) pulled in a teammate's local compose edits (`network_mode: host`,
ports and network commented out) plus literal conflict markers; the file was
repaired in three commits over 35 minutes (`a7a53ae7`, `e579063`, `2ed7fecd`),
the last of which fixed an unterminated quote introduced by the second.

**`Shimizu-014` — the lidar odometry bridge was a copy of the mocap bridge,
extrinsics included.** `odom_tf_lidar_node` (added 20:02 on 2026-04-09 in
`2a361011`) subscribed to `/lidar_odom` and multiplied SuperOdom's pose by the
mocap rigid-body→IMU calibration matrix before publishing it as
`/fmu/in/vehicle_visual_odometry` for PX4 EKF2 and as the map→body TF for
mapping. Two hours later `f1f65593` ("frame fix for superodom lidar") replaced
the chain with the tilted-lidar rotations (x-left→x-front, 29.5° tilt,
13 mm/11 mm/10 mm offsets), switched the input to `/laser_odometry`, added a
node that rewrites the registered-scan `frame_id` to `superodom_map`, and a
static `map→superodom_map` TF. The launch file's odometry remap for the tracker
was not updated until 2026-06-08 (`Shimizu-015`).

---

## 5. Limitations

- **Venue is unknown for every row.** Commit messages average four words and
  none says where a problem was seen; there are no PRs, review threads, CI runs,
  issues or notes. The branch targets Gazebo, Isaac Sim, and a ModalAI vehicle
  with mocap and Livox lidar, so a given fix could have come from any venue.
  Rows were not upgraded to `sim_interactive` on inference (e.g. `Shimizu-011`,
  wall-clock timers, can only manifest under sim time, and "mod for shimizu sim
  demo" is the commit subject — still recorded `unknown`).
- **Build-time discoveries have no taxonomy cell.** `Shimizu-001/002/003/004/005`
  necessarily surfaced when the developer built or brought up the container at a
  desk; that is neither `sim_interactive` nor `bench_hardware`, so they are
  `unknown`. If other teams see the same, a `build_desk` venue would be
  informative.
- **Merge commits were not mined.** 14 merges are excluded; their conflict
  resolutions can themselves introduce or repair breakage (as the 2025-09-10
  cluster shows; a later merge `6822bb79` also touched docker-compose.yaml).
- **The `vdb_edt_ros2` submodule is opaque.** Five commits bump the pointer of
  `YuanJunbin/vdb_edt_ros2` (`da248b1c`, `3431e263`, `dd521a90`, `06e83c96`,
  `2799a9c1`); any mapping/EDT defects fixed there are invisible to this mining.
- **Large mixed commits share one SHA across several rows.** `393142b8` (2 rows),
  `dd521a90` (3), `a50c19e9` (2) each fix several unrelated things plus tuning and
  feature work; `fix_files` names the specific files but `fix_loc` is per-file or
  whole-commit as marked. Do not sum `fix_loc`.
- **Borderline changes excluded, listed for audit.** NaN/min-range point filtering
  added to `lidarCallback` (`72d5334f`, "gazebo configuration finished" — a
  robustness addition with no stated malfunction); `check_local_path_free`
  toggling `q &&` (`06e83c96`, an optimistic/pessimistic unknown-space policy that
  the comments describe as intentional); submodule URL `git@`→`https://`
  (`06e83c96`, no stated failure); mutexes added across three controller
  callbacks (`2799a9c1`, no evidence of a race having occurred); `/qvio/odom`→
  `/ov/odom` (`3431e263`, indistinguishable from a source switch). The PX4
  publisher QoS change in `a50c19e9` ("must match for reliable delivery") is
  folded into `Shimizu-016` rather than its own row, because a RELIABLE publisher
  already matches a BEST_EFFORT subscriber under DDS rules, so its independent
  effect is doubtful; the timestamp change in the same hunk is the credible fix.
- **`class_confidence` is `medium` for 13 of 16 rows.** With no message text to
  lean on, the class is inferred from the diff; a reviewer may reasonably move a
  compose-file row from `other` to `build` or `parameter`, or `Shimizu-008`
  (camera-axis convention) from `logic` to `tf_frame`.
- **Two commits are byte-identical duplicates** (`0ea9a854`, `593be4ac`) from
  the branch pair `junbin/planning_dev` / `junbin/planning_demo`; they are one
  row. Both SHAs resolve in the clone.
- **Read-only access; a live checkout of another branch.** All evidence was read
  from git objects; nothing was built or run to confirm any malfunction.
