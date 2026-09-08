# Defect Mining Report — Hummingbird

Feeds Sec. VI-B "Where Bugs Are Found: Defect-Discovery Shift" of the ICRA 2027
AirStack paper. Companion dataset: `defects.csv` (46 rows).

---

## 1. Method

**Project:** Hummingbird (wire-perching aerial manipulation, AirLab CMU)
**Repo mined:** `JohnYanxinLiu/Hummingbird-AirStack` — a **private mirror** of
`castacks/AirStack`. Local clone at `/home/johnl/Development/Hummingbird-AirStack`,
full history (`git rev-parse --is-shallow-repository` → `false`), `gh` authenticated
as `JohnYanxinLiu`.

**AirStack fork point:** `1c41f8c029a6b579fa3910e8c69c0dcc02e78c22`
(*"OptiTrack (3/3): Isaac wrapper, mocap EV fusion in sim, and a Circle-trajectory
e2e (#376)"*, 2026-08-17) — **exactly the tip of `upstream/develop`**. Verified:

```bash
git rev-list --left-right --count upstream/develop...HEAD   # -> 0  64
```

Zero commits behind means the entire delta is Hummingbird work with no upstream
drift mixed in.

**Mining window:** fork point (2026-08-17) → 2026-09-01 (mining date).
**Mining date:** 2026-09-01. **Agent/model:** Claude Opus 5 (1M context), via Claude Code.

**Branch set.** The window is defined **topologically**, not by author date: the
mirror's history was rebuilt, so teammate commits carry pre-fork author dates while
sitting on post-fork branches. Branches were selected by descent from the fork point:

```bash
git for-each-ref --format='%(refname)' refs/heads refs/remotes/origin \
  | grep -v -E 'backup/|origin/HEAD|origin/gh-pages' > mirror_refs.txt
for r in $(cat mirror_refs.txt); do
  git merge-base --is-ancestor $BASE $r && echo $r          # -> the 6 post-fork refs
done
git rev-list $POST_REFS --not $BASE | wc -l                 # -> 65 commits
```

Post-fork refs: `hummingbird-develop`, `infra/core_simulation_infrastructure`,
`infra/wind-field`, `infra/wire-realism`, `rough/workshop_demo`, and the two
`origin/infra/*` mirrors of those. Teammate branches are included per the project
lead's instruction; within this window that contributes 2 commits by `andysippel`
(rebased 2026-07-28 sim2real work) and 63 by `John`.

**Other commands used:**

```bash
# candidate collection
git log --grep -iE 'fix|bug|broken|repair|revert|hotfix' $BASE..<ref>
git show -s --format='%H%n%an %aI%n%B' <sha>          # full message read for all 65
git show --numstat --format='' <sha>                  # fix_files / fix_loc
git branch -r --contains <sha>                        # push/auditability check

# PR / CI / issue enumeration
gh pr list   --repo JohnYanxinLiu/Hummingbird-AirStack --state all  --limit 100
gh pr view <n> --repo ... --json comments,reviews
gh run list  --repo JohnYanxinLiu/Hummingbird-AirStack --limit 100
gh issue list --repo JohnYanxinLiu/Hummingbird-AirStack --state all

# notes-file evidence
git log --format='%h %ad %s' -- .agents/hummingbird-knowledge/notes/*.md
```

**Deduplication.** Rebased duplicates across branches were collapsed by
`(author-date, subject)`: 292 mirror-only commit objects → 252 distinct commits
repo-wide. Candidates were then deduplicated into distinct defects by root cause —
several commits fix one defect (e.g. the 10 m takeoff default, `034339f8` +
`dc5fe9b8`, is one row), and several rows share one commit (e.g. `6981e12f` fixes
four distinct defects, each its own row).

---

## 2. Denominators

Everything scanned, in the window unless stated:

| Quantity | Count | Notes |
|---|---:|---|
| Commits in the mining window | **65** | union of the 6 post-fork refs, `git rev-list $POST --not $BASE` |
| — commit messages read in full | 65 | every one |
| — diffs/stats inspected | 34 | every commit that yielded a candidate |
| Merged PRs into the window's branches | **0** | the mirror has no merged PRs at all |
| PRs on the mirror, any state | 4 | #1–#4, all created 2026-04-30 → 2026-07-07, all pre-window |
| PR review threads / review comments | **0** | `reviews: []` and `comments: []` on all four |
| Issues (any state) | **0** | `gh issue list --state all` returns empty |
| GitHub Actions runs on the mirror, all time | 17 | CI history **reachable** (`gh` authenticated) |
| — runs inside the mining window | **0** | newest run of any kind: 2026-07-07 |
| Knowledge-note files mined | 6 | 4 notes + 2 plans, 688 lines, under `.agents/hummingbird-knowledge/` |
| **Defects recorded** | **46** | `defects.csv` |

Repo-wide context (outside the window, not mined for rows): 2,460 commits total;
292 mirror-only commit objects across all 38 mirror refs, 252 distinct after
deduplication, of which **190 are pre-fork-point** Hummingbird work on branches that
forked from earlier upstream points (`optitrack_demo` 2026-04, `andy/sim2real-overnight`
2026-06/07, `feat/hummingbird-*` 2026-07).

**CI history was reachable and is genuinely empty for this window.** This is a finding,
not a gap: the Hummingbird mirror runs no pre-merge automation. Work lands by direct
push to integration branches, and every defect below was found by a human running the
system.

---

## 3. Summary table — defect class × discovery venue

| class | `ci_premerge` | `sim_interactive` | `bench_hardware` | `field` | `unknown` | total |
|---|---:|---:|---:|---:|---:|---:|
| `parameter` | 0 | 9 | 1 | 0 | 4 | **14** |
| `logic` | 0 | 8 | 0 | 0 | 1 | **9** |
| `timing_clock` | 0 | 8 | 0 | 0 | 0 | **8** |
| `other` | 0 | 5 | 1 | 0 | 2 | **8** |
| `tf_frame` | 0 | 5 | 0 | 0 | 0 | **5** |
| `remap_topic` | 0 | 1 | 0 | 0 | 0 | **1** |
| `build` | 0 | 0 | 0 | 0 | 1 | **1** |
| **total** | **0** | **36** | **2** | **0** | **8** | **46** |

Confidence distribution: `class_confidence` high 43 / medium 3 / low 0;
`venue_confidence` high 25 / medium 10 / low 11.

Experiment-ruining proposal (advisory, for the two paper authors to adjudicate):
**yes 23**, **uncertain 17**, **no 6**.

**Read of the table.** 36 of 46 defects (78%) were found by a human running the Isaac
simulation at a desk. Zero were caught by pre-merge CI, because none ran. Zero are
attributed to a field test — the April 2026 flight-demo era is outside this window
(see Limitations). The two `bench_hardware` rows are a weighed landing gear
(`Hummingbird-046`) and the real-drone Jetson brownout on disarm (`Hummingbird-045`).

The class distribution is the paper-relevant part: **`parameter` + `timing_clock` +
`tf_frame` + `remap_topic` = 28 of 46 (61%) are integration-wiring defects**, not
algorithm errors. Only 9 are `logic` — mistakes inside a module's own reasoning.

---

## 4. Example candidates

The prompt asks for the most vivid `ci_premerge` catches. **There are none to report:
this project has zero pre-merge CI runs in the window.** Substituted below are the
three most vivid `sim_interactive` catches — each is a defect that desk simulation
caught before it reached the mocap room, which is the same claim the section supports,
one venue over.

**`Hummingbird-007` — the sim silently flew the wrong aircraft.**
Forwarding a scene knob into the container as `- HUMMINGBIRD_DRONE_USD=${...:-}`
defines the variable as the empty string when unset, and `os.environ.get(key, DEFAULT)`
returns `""` rather than the default once the key exists — so the scene loaded the stock
Pegasus iris instead of the 45 MB Hummingbird USD. The only symptom was one warning line
with an empty path buried in Isaac's startup spam; downstream, there was no claw, no
gripper, and therefore no Teensy TCP server, leaving the robot container retrying a
refused connection. Caught by noticing the missing claw in a running scene, not by any test.

**`Hummingbird-024` — a non-reentrant lock that drops OFFBOARD mid-flight.**
`_abandon()` took `_state_lock` and then called `_pose()`, which takes it again;
`threading.Lock` is not reentrant, so the action thread deadlocked while holding the
task lock, `_tick()` blocked, the `cmd_pose` stream stopped, and PX4 would drop OFFBOARD
about 0.5 s later — while airborne. It had already fired once, as an unexplained
"land: goal rejected" after a navigate timeout in an earlier flight test, and was only
identified when that earlier symptom was traced during verification of an unrelated change.

**`Hummingbird-038` — a success report for a 22 m flyaway.**
On the first live latch, the AUTO.LAND descent slid the latched claw along the wire into
the pole, and the conductor's elastic recoil slung the drone to 22 m. The hang classifier
used a `z >= z_latch - 0.75` floor, so it classified the flyaway as `HANGING` and the task
reported success; the ratchet stays `LATCHED` even when it closes on air, so claw state
could not veto it. The fix requires a position band in both z and y.

---

## 5. Limitations

- **No CI, no PRs, no issues in this window.** The mirror's 4 PRs are all pre-window and
  carry zero reviews and zero comments; its newest workflow run of any kind is 2026-07-07.
  So `ci_premerge` and `pr_thread` evidence types are structurally unavailable here, and
  the `ci_premerge` cell being zero reflects an absent practice rather than an absent record.
  Upstream `castacks/AirStack` — where this author landed 19 PRs with real check runs
  between 2025-11 and 2026-08 — was **excluded by the agreed scope** and is where any
  `ci_premerge` rows for this author would be found.
- **The window excludes the field-test era.** 190 distinct mirror-only commits predate the
  fork point, including the 2026-04-13 demo (`optitrack_demo`) and the 2026-07-30 mocap
  flight campaign. That is where `field`-venue defects would live; `field = 0` in this table
  is a scoping artifact, not evidence that no field defects occurred. Those commits also
  carry much terser messages, so mining them would produce markedly lower-confidence rows.
- **Classification evidence is commit messages plus file-level stats, not hunk-level diffs.**
  This project's messages are unusually detailed — most state the root cause, the
  measurement, and the log line — which is why 43 of 46 rows carry `class_confidence: high`.
  But full textual diffs were not read for every row, so a classification that contradicts
  the message would not have been caught.
- **A venue category is missing for desk-run tests.** Three rows (`-005`, `-016`, `-033`)
  are defects found by running `pytest`/`colcon test` at a desk — not simulation, not CI.
  The taxonomy has no cell for this, so they are recorded `unknown` (or `sim_interactive`
  at `low` confidence where the test drives the sim). If other teams report the same,
  this may warrant a `test_desk` venue.
- **`sim_interactive` at `low` confidence appears where a run is implied but not stated.**
  11 rows carry `venue_confidence: low`. Venue was never upgraded on inference alone; where
  a commit describes a fix with no statement of how it was found, the row is `unknown`.
- **Two rows are not yet pushed.** `Hummingbird-046` (`8d8698f8`) and the wire-friction
  work (`106dc726`) exist only on the local `rough/workshop_demo`; their `evidence_url` is
  written as `local-clone:rough/workshop_demo@<sha>` and they are auditable only in this
  clone until pushed.
- **Shared fix commits inflate `fix_loc`.** Where one commit fixes several distinct defects
  (`6981e12f` → 4 rows; `18280ea5`, `0ea27dc8`, `dbc2e93c`, `d6eff60f`, `69611a6c` → 2 each),
  `fix_files` names the specific file but `fix_loc` is the whole commit's line count, marked
  `(whole commit; fixes multiple listed defects)`. Do not sum `fix_loc` across rows.
- **Six rows come from knowledge notes, and two of those defects are unfixed.**
  `Hummingbird-042` (snapshot hook steps physics while playing) is recorded as open as of
  2026-08-27, and `Hummingbird-045` (ESC-driven Jetson brownout) is deferred as
  ESC-programming territory. Their `fix_loc` is `n/a`; `commit_sha` points at the commit
  that records the defect, not one that fixes it.
- **Author self-reporting.** Most commit messages were co-authored with an AI assistant and
  are unusually forthcoming about root cause and blame ("Two bugs, both mine"). This makes
  the dataset richer than a typical repo's, and it means the row count reflects
  documentation practice as much as defect incidence. A project that documented less would
  mine thinner from the same amount of breakage.
