# Defect Mining Report — RAVEN

Feeds Sec. VI-B "Where Bugs Are Found: Defect-Discovery Shift" of the ICRA 2027
AirStack paper. Companion dataset: `defects.csv` (19 rows).

RAVEN (open-set semantic aerial navigation: RayFronts mapping + a behavior
tree on top of AirStack; validated in Isaac Sim environments, then flown on a
real drone with a Jetson Orin) spans three repositories, all mined here.

---

## 1. Method

**Project:** RAVEN (AirLab CMU). **Mining date:** 2026-09-08.
**Agent/model:** Claude (Claude Code, model `claude-fable-5-1`) for Stage 2
(classification); Stage 1 (enumeration) was the repository's own
`mine_defects.py`, run once per repo.

**Repositories, windows, clones (all read-only):**

| Repo | Branch / window | Commits | Local clone used for `git show` |
|---|---|---:|---|
| `castacks/AirStack` | `raven`, after fork point `2d4f4be313fdb2221b23cf728fd906ae30c6d62a` → tip `278acbffaf748cd6e0102b3a25cfea544e031c83` | 25 | `/home/andrew/Development/AirStack` (`origin/raven`; user's live repo, never checked out or modified) |
| `seungchan-kim/RayFronts` | `raven`, after base `cded3eee90ad82fbff44c58e7dff31523c7fe529` → tip `8d838d79e444a035b71865034e4c33020b981fc8` | 60 | `…/scratchpad/repos/RayFronts` |
| `castacks/RAVEN` | `main`, whole history → tip `91ef2e7aacb9d2cbb6bbaf72bdcac7754638a778` | 71 | `…/scratchpad/repos/RAVEN` |

**Stage 1 (enumeration).** `mine_defects.py` was run three times, producing
`results/raven/{airstack,rayfronts,raven-top}/{denominators.json,commits.tsv,candidates.md}`.
The exact git commands it executed are recorded verbatim in each
`denominators.json` under `commands`; the enumeration command for each repo
is of the form

```bash
git -C <clone> log --no-merges --format=%H%x1f%aI%x1f%an%x1f%s%x1f%b%x1e <tip> --not <base>
git -C <clone> rev-list --merges --count <tip> --not <base>
git -C <clone> show --numstat --format= <sha>          # for every commit
```

with the keyword regex
`\b(fix|fixes|fixed|fixing|bug|bugs|broken|breaks?|repair|revert|hotfix|crash|regression|wrong|incorrect|fail(?:s|ed|ing|ure)?|error|issue|typo|workaround|patch)\b`
used only to *mark* candidates; `candidates.md` lists every commit (`--all-messages`).

**Stage 2 (classification) — commands used:**

```bash
# full diffs of every commit, per repo (rviz/USD/PNG excluded from AirStack; pure file
# additions and annotation *.json edits in RayFronts read via numstat + targeted `git show`)
git -C /home/andrew/Development/AirStack log --reverse -p --stat --format='=====COMMIT %H%n%aI %an%n%s%n%b' \
    2d4f4be313fdb2221b23cf728fd906ae30c6d62a..origin/raven -- . ':(exclude)*.usd' ':(exclude)*.png' ':(exclude)*.rviz'
git -C repos/RayFronts log --reverse -p --stat --diff-filter=MRDC --format=... cded3eee..8d838d79 -- . ':(exclude)*.json' ':(exclude)*.png'
git -C repos/RayFronts log --reverse --diff-filter=A --name-only cded3eee..8d838d79     # added files, listed
git -C repos/RayFronts show --stat <sha>; git show --format= <sha> -- '*.json'          # annotation-edit commits
git -C repos/RAVEN log --reverse -p --stat --format=... 91ef2e7a -- . ':(exclude)*.usd' ':(exclude)*.png'

# cross-container wiring checks (which side publishes what)
git -C /home/andrew/Development/AirStack grep -n "depth_ground_truth\|left/depth" origin/raven -- simulation robot
git -C /home/andrew/Development/AirStack grep -n "front_stereo\|topicName" origin/raven -- '*spawn_zed_camera.py'
git -C /home/andrew/Development/AirStack grep -n "omni_pass\|ISAAC_SIM_SCRIPT_NAME" origin/raven -- simulation/isaac-sim/docker
git -C repos/RayFronts grep -n "input_text\|input_prompt" 7aaad28^ -- '*.py'
git -C repos/RayFronts grep -n "sklearn" cd5f9ab^ -- '*.py'; git show cd5f9ab^:docker/desktop.Dockerfile
git -C repos/RayFronts show 8d838d79:rayfronts/configs/dataset/ros2isaacsim.yaml

# PR / issue / CI denominators
gh pr list --repo castacks/RAVEN --state all --limit 200
gh issue list --repo castacks/RAVEN --state all --limit 200; gh issue view 1|2 --repo castacks/RAVEN
gh run list --repo castacks/RAVEN --limit 50
gh pr list --repo seungchan-kim/RayFronts --state all; gh issue list --repo seungchan-kim/RayFronts --state all
gh run list --repo seungchan-kim/RayFronts --limit 20
gh pr list --repo castacks/AirStack --state all --head raven; gh pr list --repo castacks/AirStack --state all --base raven
gh run list --repo castacks/AirStack --branch raven --limit 20
gh api repos/seungchan-kim/LVLM/compare/4f3be587...7e5c652f; gh api repos/seungchan-kim/LVLM/contents/Dockerfile?ref=<sha>

# SHA verification (every commit_sha and every evidence_url SHA, in the clone of its own repo)
git -C <clone> cat-file -e <sha>^{commit}
# schema validation
python3 summarize_defects.py results/raven/defects.csv --out <scratch>/val-raven     # exit 0
```

**What was read.** All 156 commit messages and all 156 diffs. For RayFronts,
the hunk-level diff was read for every modification/rename/deletion; pure file
additions (new behaviors, annotation scripts, the initial 2 125-line import) were
read at the file-list level, and the seven annotation-JSON edit commits were
inspected with `--stat` plus the first ~40 changed lines. For AirStack, the six
`robot.rviz` edits were read as stats only.

**Deduplication.** One row per root cause. Where the same defect appears on two
sides of the RAVEN/AirStack/RayFronts boundary it is one row: the fix commit is
the `commit_sha` and the other repo's commit (usually a submodule-pointer bump
whose message names the symptom) supplies `evidence_url`/`evidence_quote`
(`RAVEN-013`, `RAVEN-017`). Where one commit fixes two independent defects it
yields two rows (`RAVEN-009`/`-010` share `1f48484c`).

**SHA verification result.** All 19 `commit_sha` values and all 19
`evidence_url` SHAs resolve with `git cat-file -e <sha>^{commit}` in the clone of
the repo they cite (RayFronts 13 + 2 evidence, AirStack 4, RAVEN 2 + 2
evidence). Zero unresolved.

---

## 2. Denominators

| Quantity | AirStack `raven` | RayFronts `raven` | RAVEN `main` | Total |
|---|---:|---:|---:|---:|
| Commits in window | **25** | **60** | **71** | **156** |
| — merge commits | 0 | 0 | 0 | 0 |
| — distinct authors | 2 (Seungchan Kim 21, krrishj18 4) | 1 | 1 (three git identities) | 2 people |
| — keyword-matched candidates (Stage 1) | 2 | 14 | 14 | 30 |
| — commit messages read in full | 25 | 60 | 71 | 156 |
| — diffs inspected | 25 | 60 | 71 | 156 |
| Pull requests (any state) | **0** on `raven` (`--head raven` and `--base raven` both empty) | **0** | **0** | 0 |
| PR review threads | 0 | 0 | 0 | 0 |
| Issues (any state) | n/a (branch of the main repo; not mined) | disabled on the fork | **2** open, 0 comments | 2 |
| GitHub Actions runs | **0** on branch `raven` (`gh run list --branch raven` empty) | **0** (no runs) | **0** (no workflows) | 0 |
| **Defects recorded** | **4** | **13** | **2** | **19** |

Notes on the denominators:

- **CI history was reachable and is genuinely empty for all three repos.**
  `castacks/RAVEN` and the RayFronts fork have no workflow runs at all; the
  AirStack `raven` branch never received a PR, so the upstream `system-tests`
  workflow (PR-triggered) never ran against it. `ci_premerge` and `pr_thread`
  evidence types are therefore structurally unavailable. Every commit is a
  direct push to the integration branch.
- **The two RAVEN issues are not defect reports.** #1 (2025-10-27) asks for the
  code-release date; #2 (2026-04-09, "cannot log in
  https://airlab-nucleus.andrew.cmu.edu/omni/web3/") has an empty body and
  concerns access to an external Nucleus server, not the code. Neither yields a row.
- `castacks/RAVEN` also tracks a fourth repo as a submodule (`seungchan-kim/LVLM`,
  public, 5 pointer bumps). It was consulted only to check one candidate (see
  Limitations) and was not mined for its own defects — out of the agreed scope.

---

## 3. Summary table — defect class × discovery venue

Generated by `summarize_defects.py` (exit 0):

| class | `ci_premerge` | `sim_interactive` | `bench_hardware` | `field` | `unknown` | total |
|---|---:|---:|---:|---:|---:|---:|
| `remap_topic` | 0 | 1 | 0 | 0 | 1 | **2** |
| `tf_frame` | 0 | 0 | 0 | 0 | 3 | **3** |
| `parameter` | 0 | 0 | 0 | 0 | 5 | **5** |
| `build` | 0 | 0 | 0 | 0 | 2 | **2** |
| `logic` | 0 | 0 | 0 | 0 | 7 | **7** |
| **total** | **0** | **1** | **0** | **0** | **18** | **19** |

Rows per repo: `seungchan-kim/RayFronts` 13, `castacks/AirStack` 4,
`castacks/RAVEN` 2. Integration-wiring classes
(`remap_topic` + `tf_frame` + `parameter` + `build`) = **12 of 19 (63%)**;
`logic` = 7 (all inside RayFronts' behavior code).

Confidence: `class_confidence` high 11 / medium 8 / low 0;
`venue_confidence` medium 1 / low 18.
Experiment-ruining proposal (advisory): **yes 2**, **uncertain 10**, **no 7**.

**Read of the table.** The venue column is almost entirely `unknown` (18/19)
because this project's commit messages are terse ("fix typo", "fixed scale",
"input_text -> prompt") and there are no PRs, CI runs, issues or notes files
to supply venue. This is a limitation of the record, not evidence of where
bugs were found. The one `sim_interactive` row (`RAVEN-012`) is the only fix
whose own message names the simulator ("modified depth topic for isaac-sim
config"), and its venue confidence is only `medium`. The class column is the
informative part: 12 of 19 defects are wiring, not algorithms, and 4 of those
12 sit exactly on the container boundary between RayFronts and AirStack
(`-012` depth topic, `-014` camera extrinsic, `-013` LiDAR range feeding the
planner, `-017` spawn-pose env-var precedence), with a fifth on the boundary
between the operator tool and the mapper (`-015`).

Timeline: 6 `logic` rows are from the Aug–Sep 2025 algorithm-development
phase in RayFronts; the other 13 rows cluster in Mar–May 2026, when the three
repos were assembled into the public release and run end-to-end in Isaac Sim.

---

## 4. Example candidates

The protocol asks for the most vivid `ci_premerge` catches. **There are none to
report: no repository in this project has a single pre-merge CI run.** The
three rows below are the most vivid cross-container integration defects
instead; none carries a recorded discovery venue beyond what is stated.

**`RAVEN-012` — the mapper subscribed to a depth topic the simulator never published.**
RayFronts' Isaac Sim dataset config (`rayfronts/configs/dataset/ros2isaacsim.yaml`,
written in Aug 2025 against an earlier AirStack) subscribed depth on
`/robot_1/sensors/front_stereo/left/depth`, while the Pegasus ZED subgraph in the
AirStack `raven` branch publishes `.../left/depth_ground_truth`
(`spawn_zed_camera.py` line 278); RGB, odometry and `camera_info` names still
matched, so intrinsics loaded and the node sat silently with no synchronized
frames. Fixed 2026-04-27 by a one-line topic change (`039f18ce`); the same-day
RAVEN README commit (`4fbf5d1c`) added the "Checking if RayFronts mapper is
loading intrinsics" troubleshooting section telling users to `ros2 topic echo`
the sensor topics while the sim is playing.

**`RAVEN-015` — the operator's prompt tool published to a topic nobody subscribed.**
`input_prompt.py`, the documented way to give RAVEN a target ("set new input"),
published `std_msgs/String` on `/input_text`; the mapping server had moved its
subscription to `/input_prompt` on 2025-08-22 (`7c571427`), and `mission_checker`
and the VLFM baseline also listened on `/input_prompt`. Prompts typed into the
tool therefore never reached the behavior tree, leaving the system in
frontier-only exploration; the one-line rename landed 2026-04-30 (`7aaad284`),
one day after the README first documented the tool.

**`RAVEN-013` — a LiDAR minimum range that regressed across the repo hand-off.**
RAVEN's own March 2026 scene scripts (`castacks/RAVEN` `169991d7`) spawned the
simulated Ouster with `lidar_min_range = 4.0, # Minimum detection range (m) to
avoid propeller hits`. When the scenes were re-authored inside AirStack on
2026-04-03 (`3f5f2a4e`) they took the AirStack example's `0.75`, and on
2026-04-27 all four were raised to `3.5` with the RAVEN pointer commit
explaining why: "lidar min range increase to avoid hallucinated obstacles"
(`8bf4e59b`) — self-returns from the drone were entering the local planner's
obstacle model.

Two further boundary rows worth a reviewer's glance: `RAVEN-014` (a hard-coded
10° camera-pitch extrinsic for the retired Spirit drone still being applied to
Pegasus Iris poses whose camera has zero tilt; the fix's own comment states the
history) and `RAVEN-017` (drone spawn quaternion from `launch_raven.sh` not
reaching the Isaac scene because AirStack's `.env` lacked `DRONE_*` defaults and
was sourced after the per-environment pose block).

---

## 5. Limitations

- **Venue is almost never recoverable.** 18 of 19 rows are `unknown`. Commit
  messages are 2–8 words, there are no PRs, review threads, CI runs, issues or
  notes files, and the RAVEN README's troubleshooting sections describe checks
  rather than incidents. Several fixes touch simulation-only files (Isaac Sim
  scene scripts, `ros2isaacsim.yaml`) and can only have manifested in
  simulation, but per the protocol they are recorded `unknown` rather than
  inferred. Readers should treat the venue column as "not recorded", not as
  evidence against desk-simulation discovery.
- **The hardware phase is not in these repositories.** RAVEN was flown on a real
  drone (Jetson Orin), but none of the 156 commits touches hardware bring-up;
  the RayFronts commit `f6f030b2` explicitly retires the "Spirit drone" setup
  for the Pegasus-based code release. `bench_hardware = 0` and `field = 0` are
  scoping artifacts of what was committed, not evidence that no hardware
  defects occurred.
- **Pre-window work is squashed.** RayFronts commit `288694df` ("copied
  working branch of airstation01") imports 2 125 lines of prior development in
  one commit. Its diff against the RayFronts base contains what are clearly
  earlier integration fixes (pose message type `PoseStamped` → `Odometry`,
  RGBA vs BGRA channel order for Isaac images, the pitch extrinsic later removed
  in `RAVEN-014`), but they cannot be dated or attributed and are not rows.
- **No PR/CI/issue evidence types.** `ci_premerge`, `pr_thread` and `issue`
  are structurally unavailable (Sec. 2). One row (`RAVEN-010`) uses
  `notes_file` for a defect that is documented with a workaround in the README
  but has no code fix.
- **Excluded candidates, for auditability.** Read and deliberately not
  recorded: (a) seven RayFronts annotation-JSON edits with "fix" in the message
  (`413d2c0f`, `c1b5065b`, `f0204284`, `b5e999a0`, `4f2d2eca`, `18b1649e`,
  `6a405e47`) — evaluation ground-truth data corrections (label renames, a
  start-pose translation sign, a yaw sign), not system code; (b) corrections
  embedded in feature commits whose messages do not mention them
  (`2bf21a2b` voxel dispatch calling the frontier behavior while voxel `execute`
  was still a stub; `fb1b9ffe` negative box sizes; `2933ed9f` guard against an
  empty ray-group list; `8858896` mode-text overwrite); (c) `aede16bf` "fix on
  launch_raven inputs", which replaced a one-hour-old design (regex-parsing the
  AirStack launch script) with env-var reads in lock-step with the AirStack
  change one minute later — co-evolution, not a repair of a running system;
  (d) `58c465c2` (torch cu121 → cu130, `timm`/`transformers` pins "for
  5070,80,90 series") — almost certainly a dependency breakage on RTX 50-series
  GPUs, but no symptom is recorded; (e) `5476a73c`/`launch_raven.sh` adding an
  explicit `python3 internvl3.py` command to the LVLM container — the LVLM
  Dockerfile has `CMD ["/bin/bash"]` at both pointers and the LVLM change is a
  new 203-line method, so this is feature wiring; (f) README-only fixes
  (`ac9bbd11` wrong script name in the run instructions, `29b98f06`
  `.gitmodules` branch rename, oral-presentation and typo edits);
  (g) `278acbff`/`7ec90cdd` (ConstructionSite spawn point moved and colliders
  disabled, no stated reason), `5667c6b3` (download script rewritten from
  `gdown` to `curl`, no stated breakage), and all RViz/trajectory-thickness and
  parameter-tuning commits.
- **Shared and mixed fix commits inflate `fix_loc`.** `1f48484c` yields two
  rows (`-009`, `-010`); `3c909f85`, `f6f030b2`, `7aaad284`, `f7ca3698`,
  `d1f73821` mix the fix with tuning or features. `fix_loc` names the specific
  file's lines and the whole-commit total where they differ; do not sum
  `fix_loc` across rows.
- **Classification is from diffs and messages, not from the author.** 8 rows
  are `class_confidence: medium` because the message does not state the
  malfunction and the class is inferred from the hunk (e.g. `RAVEN-002`
  indentation, `RAVEN-011` missing `scikit-learn`). The author was not consulted.
- **Read-only compliance.** No repository was modified, checked out, fetched or
  reset; the AirStack clone is the user's live working repo with a different
  branch checked out and was accessed only through `git -C … show/log/grep`.
  The only files written are `results/raven/defects.csv` and this report.
