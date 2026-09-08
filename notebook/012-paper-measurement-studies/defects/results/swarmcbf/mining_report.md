# Defect Mining Report — SwarmCBF

Feeds Sec. VI-B "Where Bugs Are Found: Defect-Discovery Shift" of the ICRA 2027
AirStack paper. Companion dataset: `defects.csv` (15 rows).

---

## 1. Method

**Project:** SwarmCBF — multi-drone CBF safety-filter ground controller
(`svg_ground_control`) with OptiTrack mocap, Isaac Sim + PX4 SITL rehearsal, and
real ModalAI VOXL2 Starling drones.
**Repo mined:** `castacks/AirStack`, branch `yikuan/SVG_ground_control`
(tip `564d43e4542c1e2dd4cd24cebeb21b3fb3843ad2`, 2026-08-11). Local clone at
`/home/andrew/Development/AirStack` (read-only use; another branch checked out,
no checkout/reset/stash performed). `gh` authenticated as `andrewjong`.
**AirStack fork point:** `e4b499d120ef5157232c6ef6b1109488a94c9641` (Release 0.18.0).
**Mining window:** fork point → 2026-09-08 (all 21 commits are dated
2026-06-10 … 2026-08-11). **Mining date:** 2026-09-08.
**Agent/model:** Claude (Claude Code), Stage 1 by `mine_defects.py`,
Stage 2 (this report) by the agent reading every diff.

**Stage 1 (enumeration, already done, not repeated):** `mine_defects.py`
produced `results/swarmcbf/denominators.json`, `commits.tsv`, `candidates.md`.
Its recorded commands (verbatim from `denominators.json`):

```bash
git -C /home/andrew/Development/AirStack log --no-merges \
  --format=%H%x1f%aI%x1f%an%x1f%s%x1f%b%x1e \
  564d43e4542c1e2dd4cd24cebeb21b3fb3843ad2 --not e4b499d120ef5157232c6ef6b1109488a94c9641
git -C /home/andrew/Development/AirStack rev-list --merges --count \
  564d43e4542c1e2dd4cd24cebeb21b3fb3843ad2 --not e4b499d120ef5157232c6ef6b1109488a94c9641
git -C /home/andrew/Development/AirStack show --numstat --format= <sha>   # x21
```
Keyword regex: `\b(fix|fixes|fixed|fixing|bug|bugs|broken|breaks?|repair|revert|hotfix|crash|regression|wrong|incorrect|fail(?:s|ed|ing|ure)?|error|issue|typo|workaround|patch)\b`
→ 6 keyword hits; all 21 commits were carried into Stage 2 regardless.

**Stage 2 (classification) commands:**

```bash
# every commit, in order, message + stat
for s in $(git -C /home/andrew/Development/AirStack rev-list --reverse \
    564d43e4542c1e2dd4cd24cebeb21b3fb3843ad2 --not e4b499d120ef5157232c6ef6b1109488a94c9641); do
  git -C /home/andrew/Development/AirStack show --stat --format='%H%n%aI%n%an%n%s%n%n%b' $s; done
# full diffs (read for all 21; vendored natnet_ros2 and reference/*.cpp hunks skimmed)
git -C /home/andrew/Development/AirStack show <sha>
git -C /home/andrew/Development/AirStack show <sha> -- <path>       # large commits, per file
git -C /home/andrew/Development/AirStack show --numstat --format= <sha>   # fix_files / fix_loc
# notes-file evidence: the experiment log at the tip and at each recording commit
git -C /home/andrew/Development/AirStack show 564d43e4:robot/ros_ws/src/svg_ground_control/experiment.md
git -C /home/andrew/Development/AirStack show <sha> -- robot/ros_ws/src/svg_ground_control/experiment.md
# provenance checks
git -C /home/andrew/Development/AirStack show e4b499d1:robot/ros_ws/src/interface/robot_interface/src/robot_interface_node.cpp
git -C /home/andrew/Development/AirStack show 59679402:robot/ros_ws/src/svg_ground_control/config/swarm_real.yaml
# PR / CI
gh auth status
gh pr list --repo castacks/AirStack --state all --head yikuan/SVG_ground_control            # -> (empty)
gh pr list --repo castacks/AirStack --state all --head yikuan/SVG_ground_control --json number,title   # -> []
```

**Deduplication.** Candidates were collapsed by root cause. Three distinct
provisioning-script defects share one commit (`564d43e4` → `SwarmCBF-011/-012/-013`),
so their `fix_loc` is the whole-file line count, marked as shared. Two defects were
documented in the notes file first and fixed in code later (`-006`, `-007`: recorded
in `736b0564`, fixed in `a46f04b5`); `date` is the recording date and `commit_sha` is
the fix commit. The 2026-06-21 troubleshooting row "real drone won't arm (fuse
failure), no /fmu/out/vehicle_odometry" was **not** given its own row: its most
likely root cause on that date is `SwarmCBF-010` (px4_interface never loaded, fixed
two days later), and the notes do not state a separate root cause.

**Venue mapping.** All real-drone work on this branch happened in the lab
mocap room (OptiTrack volume, ModalAI drones on a LAN). Per the protocol's
definitions this is hardware pre-deployment, so it is recorded as
`bench_hardware`; no row is `field`. Sim rows are Isaac Sim + PX4 SITL run
interactively at a desk (`sim_interactive`).

---

## 2. Denominators

| Quantity | Count | Notes |
|---|---:|---|
| Commits in the mining window | **21** | `rev-list 564d43e4 --not e4b499d1`; 0 merge commits; single author (`yikuan`) |
| — commit messages read in full | 21 | every one |
| — diffs read | 21 | every one; the 8,621-line vendored `natnet_ros2` import (`c1f69fe4`) and the two vendored `reference/*.cpp` files were skimmed, not read line by line |
| Keyword-hit candidates (Stage 1) | 6 | 5 of the 6 yielded rows; `4479ea7c` (rosbag path) was excluded, see Limitations |
| Non-keyword commits yielding rows | 5 | `06a1f616`, `11a1237f`, `736b0564`, `94425c25`, `a46f04b5` |
| Merged PRs from this branch | **0** | `gh pr list --state all --head yikuan/SVG_ground_control` → empty |
| PRs any state / review threads | 0 / 0 | none exist for this branch |
| Issues | 0 | none reference this branch or package |
| CI runs | **n/a** | no PR was opened, so no `system-tests.yml` / `unit-tests.yml` run was ever triggered for this branch; nothing to fetch |
| Notes files read | 3 | `robot/ros_ws/src/svg_ground_control/experiment.md` (1,089 lines at tip, plus its state at 11a1237f, 59679402, 736b0564, f6441c5f, 94425c25, a46f04b5), `svg_ground_control/README.md`, the `voxl_setup_real_drone.sh` header comments |
| **Defects recorded** | **15** | `defects.csv` |

**CI history is structurally absent, not expired.** The branch was developed by
direct push with no pull request. AirStack's pre-merge simulation tests
(`system-tests.yml`) run on PR events, so they never executed against this code.
Every defect below was found by a human running the system.

---

## 3. Summary table — defect class × discovery venue

| class | `ci_premerge` | `sim_interactive` | `bench_hardware` | `field` | `unknown` | total |
|---|---:|---:|---:|---:|---:|---:|
| `parameter` | 0 | 1 | 3 | 0 | 2 | **6** |
| `other` | 0 | 0 | 3 | 0 | 0 | **3** |
| `logic` | 0 | 1 | 0 | 0 | 1 | **2** |
| `tf_frame` | 0 | 1 | 0 | 0 | 0 | **1** |
| `remap_topic` | 0 | 0 | 0 | 0 | 1 | **1** |
| `timing_clock` | 0 | 0 | 1 | 0 | 0 | **1** |
| `build` | 0 | 0 | 1 | 0 | 0 | **1** |
| **total** | **0** | **3** | **8** | **0** | **4** | **15** |

Confidence distribution: `class_confidence` high 11 / medium 4 / low 0;
`venue_confidence` high 1 / medium 10 / low 4 (all four `low` are the `unknown`-venue rows).

Experiment-ruining proposal (advisory): **yes 2**, **uncertain 4**, **no 9**.

**Read of the table.** Integration-wiring classes
(`parameter` + `tf_frame` + `remap_topic` + `timing_clock` + `build`) account for
10 of 15 (67%); only 2 are `logic`. The 3 `other` rows are all shell/systemd
defects in the VOXL2 provisioning script and Docker networking — infrastructure
between the ROS graph and the vehicle, which the taxonomy has no cell for.

The venue split tracks the project's two phases. The sim phase (2026-06-10 …
06-15) produced the two most consequential defects (`-004`, `-005`), both caught
by desk simulation and both of the kind that would have been dangerous on
hardware. The hardware phase (06-20 … 08-11) produced 8 `bench_hardware` rows,
all discovered on the real drone at the bench before flight; 6 of those 8
(`-006`, `-007`, `-010`, `-011`, `-012`, `-013`) concern the ground-PC ↔ vehicle
transport (Docker networking, uXRCE-DDS agent/client, plugin loading) — a path
that Isaac/SITL cannot exercise, so desk simulation could not have caught them.
`SwarmCBF-010` is an upstream AirStack defect (`robot_interface_node.cpp` read the
`interface` parameter and then ignored it at the fork point) that only this
project's DDS-hardware path exposed.

---

## 4. Example candidates

The protocol asks for the most vivid `ci_premerge` catches. **There are none: this
branch was never opened as a PR, so AirStack's pre-merge simulation tests never ran
against it.** The three most vivid catches one venue over — two by desk simulation,
one on the bench — are:

**`SwarmCBF-005` — three drones flying perfect shapes in three different worlds.**
Each PX4 SITL instance places its EKF origin at its own spawn point, and the
commander summed the three local odometries as if they shared a frame: drone_1
"held its post" 2 m from it, the intruder ran its whole shuttle 2 m to the right
of the gap, and the CBF reacted to phantom geometry while missing a real 0.49 m
near-miss. Caught by rosbag analysis of an Isaac run (`squeeze_191528`), not by any
test; the fix adds a per-drone `drone_position_offsets` parameter and the
functional tests now publish odometry in per-drone local frames.

**`SwarmCBF-004` — the safety filter pushed the intruder away from the gap.**
Only teleop drones were exempt from the CBF, so the autonomous squeeze intruder was
filtered like everyone else; the pair-constraint gradient points away from the
holders on approach, and with PX4's velocity lag the intruder stalled or turned back
instead of forcing the holders to yield. The ideal-integrator functional test had
passed; the failure appeared only in the Isaac run, and the fix added a first-order
lag model to the test so it reproduces the sim failure mode.

**`SwarmCBF-010` — the interface plugin parameter that was read and ignored.**
`robot_interface_node.cpp` fetched the `interface` parameter and then called
`createSharedInstance("mavros_interface::MAVROSInterface")` regardless, so on a real
uXRCE-DDS drone `px4_interface` never loaded and nothing subscribed to
`fmu/out/vehicle_odometry`. Found on the bench during real-drone goal-tracking
bring-up; the defect is present in upstream AirStack at the fork point, and the
sim path never exposes it because MAVROS is the correct plugin there.

---

## 5. Limitations

- **No PRs, no CI, no issues.** The branch was developed by direct push (21 commits,
  0 merges, 1 author). `ci_premerge` and `pr_thread` evidence types are structurally
  unavailable; the zero in that column reflects an absent practice, not an absent record.
- **Single author, self-reported.** Every row derives from one developer's commit
  messages and experiment log. The messages are unusually detailed (root cause,
  measurements, bag names) — 11 of 15 rows carry `class_confidence: high` — but the row
  count reflects documentation practice as much as defect incidence.
- **Venue evidence for hardware rows is mostly `medium`.** The 8 `bench_hardware`
  rows are symptoms that can only occur on the real vehicle (VOXL2 boot scripts, QGC
  messages, `/fmu/*` topics, LAN reachability), but the notes rarely say "we saw this
  on <date>". No row was upgraded to `field`; all real-drone work was in the lab mocap
  volume.
- **Four rows are `unknown` venue** (`-001`, `-002`, `-008`, `-014`): the commit or
  note states the root cause but not where it was observed. `-014` (CBF fixed rows) is
  also the row most open to being read as an improvement rather than a repair; it is
  included because the diff comment explicitly describes the prior behavior as
  producing "evasion that is never executed".
- **Notes-file rows depend on the troubleshooting table being a record, not a
  forecast.** Six rows (`-002`, `-003`, `-006`, `-007`, `-009`, `-015`) are anchored in
  `experiment.md` troubleshooting rows. Only rows with a concrete symptom string, a
  stated root cause inside the project's stack, and (for `-006`/`-007`) a later code fix
  were kept. Rows judged anticipatory or external were excluded: mirrored mocap frame
  (`px4_vio_frame`, no evidence it occurred), best-effort `ros2 topic echo` gotcha
  (diagnostic tool, "not a real outage"), Isaac RTX segfault on driver 595.x
  (documented as "NOT an AirStack bug", fix is a driver downgrade), stale static IP on
  the VOXL, per-drone infrastructure not started in multi-drone runs (operator
  omission), NUM_ROBOTS scaling containers (knob semantics), hybrid "real drone never
  moves" (operator omission), and the onboard-VIO competing-source note (procedural,
  no symptom narrative).
- **`4479ea7c` (rosbag recorded to a non-mounted path, invisible on the host) was
  excluded** as a defect in the experiment procedure rather than in the robot system;
  it is a keyword hit that a looser reading would count.
- **Shared fix commits inflate `fix_loc`.** `564d43e4` carries three provisioning
  rows and one CBF row plus feature work (formation profiles); `11a1237f` and
  `a46f04b5` are feature/verification commits containing the fix. `fix_loc` is per file
  where the file is specific to the defect and marked "whole commit" otherwise; do not
  sum across rows.
- **Diff coverage.** All 21 diffs were read; the 8,621-line vendored `natnet_ros2`
  import and the two vendored `reference/*.cpp` files were skimmed for launch/package
  changes only. No defect was mined from vendored third-party code.
- **Read-only clone with another branch checked out.** All inspection used
  `git show`/`git log` against `origin/yikuan/SVG_ground_control`; the working tree was
  not touched. Every `commit_sha` resolves in the clone (`summarize_defects.py --verify`).
