---
name: svg-release-branches
description: How the SVG ground-control branches are coordinated — develop on yikuan/SVG_ground_control_dev, release to yikuan/SVG_ground_control as ONE squash commit that names the dev commit it came from. Never merge between the two.
license: Apache-2.0
metadata:
  author: AirLab CMU (SVG)
  repository: AirStack
---

# Skill: SVG release branches (dev → release by squash)

## The two branches

| branch | role | history |
|---|---|---|
| `yikuan/SVG_ground_control_dev` | daily work: every commit, merges from collaborators' branches (e.g. the foxglove lane merged at `c352c15`), bag-driven fixes | full, with lanes |
| `yikuan/SVG_ground_control` | release: one commit per verified, collective update | straight line, one commit per release, **no** link into dev in the graph |

Each release commit's message ends with `Squash of yikuan/SVG_ground_control_dev at <hash> (<subject>)`. That line is the only link back to dev: GitHub renders the hash as a link and `git show <release commit>` names the dev commit it corresponds to. Git draws no arrow for it, by design.

## Why squash and not merge

A merge commit has two parents, and everything reachable from the dev parent becomes part of the release branch's history: the graph shows a lane joining dev, and all of dev's commits appear in `git log` on the release branch. There is no merge that records the link but hides the commits — the drawn link *is* the history. A squash records the content only, so the release line stays one commit per release. The trade-off is the missing arrow, which the hash in the message replaces.

## Cutting a release

Only after the batch on dev is verified in flight (bags in `robot/ros_ws/bags/`, tests green: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider test/` in the robot container, package rebuilt).

```bash
git checkout yikuan/SVG_ground_control
git merge --squash yikuan/SVG_ground_control_dev
git -c user.name=yikuan -c user.email=fangyikuan@163.com commit -m "Release YYYY-MM-DD: <one-line summary>

<what changed, grouped: flight control / CBF / fence / teleop / foxglove / configs+docs / tests>

Squash of yikuan/SVG_ground_control_dev at $(git rev-parse --short yikuan/SVG_ground_control_dev) ($(git log -1 --format=%s yikuan/SVG_ground_control_dev))."
git tag -a vYYYY.MM.DD -m "SVG release YYYY-MM-DD"     # optional
git push origin yikuan/SVG_ground_control --tags
git checkout yikuan/SVG_ground_control_dev
```

Also document the release in `robot/ros_ws/src/svg_ground_control/README.md` (the "Update YYYY-MM-DD" section lists what each release contains, with the bags that motivated each change) — do that on dev *before* squashing so the README lands in the release commit.

`git merge --squash` keeps working release after release even though the branches share no recent ancestor: Git finds the base at the previous release point (identical trees), so each squash contains only what changed on dev since. Conflicts in a squash mean someone edited the release branch directly — resolve once, then keep to the rules below.

## Rules

- **Never merge dev into release or release into dev.** Either direction drags the hidden history along and the release line grows lanes.
- **Never commit directly on the release branch.** A hotfix goes on dev, then a (small) squash release.
- **Verify before pushing a release:** `git diff yikuan/SVG_ground_control yikuan/SVG_ground_control_dev` must be empty right after the squash commit (`wc -l` = 0), and the release commit must have exactly one parent (`git show -s --format=%p`).
- **Commits use the user's git identity, no agent co-author line** (`git -c user.name=yikuan -c user.email=fangyikuan@163.com …`).
- **Force pushes** are only ever needed to *repair* the release branch (e.g. after a mistaken merge). Use `--force-with-lease=yikuan/SVG_ground_control:<expected remote hash>` so nothing unexpected is overwritten, and re-check `git ls-remote origin` afterwards: a push can succeed even when the tool reports it blocked.
- `px4_logs/` (ULogs, tens of MB) and scratch analysis scripts stay untracked.

## Repairing a release branch that got merged history

If the release branch ever ends up pointing at dev (or at a merge of it), rebuild the single release commit without touching the working tree:

```bash
# from the dev branch; <prev_release> = the last correct release commit (or its parent)
new=$(printf '%s' "<release message>" | git -c user.name=yikuan -c user.email=fangyikuan@163.com \
      commit-tree yikuan/SVG_ground_control_dev^{tree} -p <prev_release>)
git branch -f yikuan/SVG_ground_control "$new"
git push --force-with-lease=yikuan/SVG_ground_control:<current remote hash> origin yikuan/SVG_ground_control
```

`commit-tree` + `branch -f` never checks anything out, so the working tree and the dev branch are untouched. This is how the 2026-09-27 release (`cf719f0`, squash of dev `cb7f270`) was made.
