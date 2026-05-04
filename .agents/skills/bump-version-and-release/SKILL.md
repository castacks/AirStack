---
name: bump-version-and-release
description: Bump the AirStack VERSION in .env (semver) before merging a PR that changes Docker image content, and update CHANGELOG. Required to pass the check-version-increment gate and to trigger the docker-build release workflow.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Bump VERSION and Cut a Release

## When to Use

Bump the `VERSION=` line in `/.env` whenever a pull request **materially changes the contents of any Docker image** that AirStack publishes. Skipping the bump will fail the `check-version-increment` workflow on every PR — this is the single most common preventable CI failure for new contributors.

Bump VERSION when the PR touches:

- Any `Dockerfile` under `robot/`, `simulation/isaac-sim/`, `simulation/ms-airsim/`, `gcs/`, `common/`, or `tests/docker/`
- `docker-compose.yaml` or any included sub-compose file (when the change affects what is built or installed into images)
- Code that is **baked into** an image (i.e., copied during build, not bind-mounted at runtime). For `DOCKER_IMAGE_BUILD_MODE="prebuilt"` this includes `robot/ros_ws/src/**`. For `DOCKER_IMAGE_BUILD_MODE="dev"` (the current default in `.env`) the workspace is bind-mounted, so source-only changes there do not strictly require a rebuild — but bumping is still safer if you are unsure.
- Apt packages, pip requirements, ROS package manifests installed during the build
- Entry-point scripts, tmux configs, or `.bashrc` snippets copied into images
- Submodule pointer updates that affect image contents

If in doubt, bump. The CI gate enforces a strict increment vs. the base branch — you cannot land a PR with the same VERSION as the base.

## When NOT to Bump

The version-increment check still runs on every PR, but the **only** thing it requires is that VERSION be valid semver and strictly greater than the base. For documentation-only PRs you have two options:

- **Preferred:** still bump the patch (or pre-release counter) by one. It costs nothing, keeps the gate happy, and the docker-build only fires on push to `main`/`develop` *and* a VERSION change, so an extra alpha bump on a docs PR is cheap.
- **If you really want to avoid a rebuild:** the docker-build workflow only triggers when `.env` is in the changed paths AND `VERSION=` differs from `HEAD~1`. So a docs-only PR that does not touch `.env` will not rebuild — but the PR will still fail the increment check unless you bump. There is no clean way to "opt out" of the check; the simplest path is to bump the pre-release counter.

Do **not** bump for:

- Comment-only changes to `.env`
- CI workflow tweaks that do not affect what goes in the images
- Test-only edits under `tests/` (unless you also touched `tests/docker/`)

If you change *only* docs but the gate still fails because base advanced past you, rebase and bump again.

## How VERSION Drives the Build Pipeline

Three workflows in `.github/workflows/` interact with `VERSION`:

### 1. `check-version-increment.yml` — the PR gate

- **Trigger:** every `pull_request`.
- **Logic:** runs a Python script that reads `VERSION=` from the PR's `.env`, reads the same line from `origin/<base_ref>:.env`, and validates:
  - PR version matches the regex `^(\d+)\.(\d+)\.(\d+)(?:-(alpha|beta|rc)\.(\d+))?$`
  - PR version is **strictly greater than** base version, with pre-release ordering `alpha < beta < rc < (no suffix / release)`
- **Accepted formats** (from the workflow's own error message):
  ```
  MAJOR.MINOR.PATCH            (e.g. 1.2.3)
  MAJOR.MINOR.PATCH-alpha.N    (e.g. 1.3.0-alpha.1)
  MAJOR.MINOR.PATCH-beta.N     (e.g. 1.3.0-beta.2)
  MAJOR.MINOR.PATCH-rc.N       (e.g. 1.3.0-rc.3)
  ```
- **Rejected:** `1.2`, `1.2.3-rc1` (no dot before N), `1.2.3-dev`, `1.2.3+meta`, `v1.2.3`, anything with build metadata.
- **Comparison tuple:** `(major, minor, patch, pre_rank, pre_num)` where `pre_rank = alpha:0, beta:1, rc:2, release:3`. So `1.3.0-alpha.5 < 1.3.0-beta.1 < 1.3.0-rc.1 < 1.3.0`. A release version always sorts above any pre-release of the same `MAJOR.MINOR.PATCH`.

### 2. `docker-build.yml` — the publish trigger

- **Trigger:** push to `main` or `develop` whose changed paths include `.env`, **and** the `VERSION=` line in `.env` differs from the previous commit. Also runs on manual `workflow_dispatch`.
- **Behavior on tag change:**
  1. Runs on a self-hosted ephemeral GPU runner (`[self-hosted, airstack-ephemeral]`).
  2. `docker compose build` for profiles `desktop,isaac-sim,ms-airsim`.
  3. `docker compose push` to `${PROJECT_DOCKER_REGISTRY}` (set in `.env` — currently `airlab-docker.andrew.cmu.edu/airstack`).
  4. Keyless `cosign sign` of every pushed image digest via GitHub OIDC.
  5. `cosign verify` against the workflow's certificate identity.
- **Skip behavior:** if the merge commit on `main`/`develop` does not actually change `VERSION=`, the build job is skipped (the check-changes job sets `tag-changed=false`).

### 3. `deploy_docs_from_release.yaml` — versioned docs

- **Trigger:** GitHub `release` event with `types: [published]`.
- **Behavior:** runs `mike deploy --push --update-aliases <release.tag_name> latest`, publishing the docs site under the release tag and pointing the `latest` alias at it.
- Companion workflows publish unversioned docs from `main` (default alias `main`) and `develop` (alias `develop`).

So the full release path is: bump `VERSION` → PR → merge to `main`/`develop` (rebuild + push + sign) → cut a GitHub Release matching that VERSION (versioned docs go live).

## Choosing the Bump Type

Use this decision tree on the **current** version (currently `0.18.0-alpha.7`):

```
Is this a breaking API/topic/interface change?
├── yes → bump MAJOR, reset MINOR=0, PATCH=0           (e.g. 0.18.0-alpha.7 → 1.0.0-alpha.1 if pre-1.0)
└── no
    ├── New feature / new module / new Docker image content?
    │   └── yes → bump MINOR, reset PATCH=0            (e.g. 0.18.0 → 0.19.0)
    └── Bug fix / small tweak / dependency bump?
        └── yes → bump PATCH                            (e.g. 0.18.0 → 0.18.1)

Are you mid-cycle on a pre-release line (suffix present)?
├── Same line, more iteration       → increment N      (0.18.0-alpha.7 → 0.18.0-alpha.8)
├── Promoting alpha → beta          → reset N to 1     (0.18.0-alpha.7 → 0.18.0-beta.1)
├── Promoting beta → rc             → reset N to 1     (0.18.0-beta.4  → 0.18.0-rc.1)
└── Promoting rc → release          → drop suffix      (0.18.0-rc.3    → 0.18.0)
```

Notes:

- AirStack is pre-1.0; many "breaking" changes still bump MINOR rather than MAJOR. Use judgment, and prefer pre-release suffixes (`-alpha.N`) for the active development line so feature PRs do not have to fight over MINOR numbers.
- The current pattern in git history is per-PR alpha bumps on the development line and a final un-suffixed bump at release time (e.g. `0.16.1-rc → 0.16.1`, `0.17.0-rc1 → 0.17.0` — note the older `-rc1` form predates the current validator and would be rejected today; use `-rc.1`).

## Bumping Steps

### 1. Read the current VERSION

```bash
airstack version
# → AirStack Version: 0.18.0-alpha.7
```

(Equivalent: `grep '^VERSION=' .env`.)

### 2. Edit `.env`

Open `/.env` and change exactly the `VERSION=` line. Keep the surrounding comments and quoting intact:

```diff
- VERSION="0.18.0-alpha.7"
+ VERSION="0.18.0-alpha.8"
```

The validator strips surrounding `"` or `'`, so either quoting style works, but match the existing style (double quotes).

### 3. Update `CHANGELOG.md`

Add an entry under `## [Unreleased]` describing your change (see "CHANGELOG Conventions" below). For a true release (no pre-release suffix), promote `[Unreleased]` to a new dated version section.

### 4. Verify locally

```bash
airstack version                                    # prints the new value
grep '^VERSION=' .env                               # sanity-check the literal line
git diff .env CHANGELOG.md                          # review the diff
```

Optional regex preflight (mirrors the CI check):

```bash
python3 -c 'import re,sys; v=open(".env").read(); m=re.search(r"^VERSION\s*=\s*\"?([^\"#\s]+)", v, re.M); print(m.group(1)); assert re.fullmatch(r"^(\d+)\.(\d+)\.(\d+)(?:-(alpha|beta|rc)\.(\d+))?$", m.group(1)), "INVALID"'
```

### 5. Commit

Use a clear, conventional message:

```
Bump version to 0.18.0-alpha.8
```

Recent commits in this repo use exactly this phrasing (`Bump version to 0.17.0`, `Bump version to 0.16.1`).

## CHANGELOG Conventions

`CHANGELOG.md` follows [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/) and Semantic Versioning. The literal layout in the repo is:

```markdown
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- <bullet>

### Changed

- <bullet>

### Fixed

- <bullet>

## [1.0.0] - 2024-12-19

First official public release.

### Added

- <bullets>

### Fixed

- <bullets>

### Changed

- <bullets>

### Removed

- <bullets>
```

Rules:

- Use the H2 sections **Added**, **Changed**, **Fixed**, **Removed**, **Deprecated**, **Security** as needed (Keep a Changelog standard set).
- For pre-release bumps (`-alpha.N`, `-beta.N`, `-rc.N`), keep your bullets under `## [Unreleased]`. Do not create a section per alpha.
- For a release bump (no suffix), rename `[Unreleased]` to `## [<VERSION>] - <YYYY-MM-DD>` and add a fresh empty `## [Unreleased]` above it.
- Use ISO date format `YYYY-MM-DD`.
- Write user-facing prose, not commit log dumps. Mention new modules, breaking changes, and notable behavior shifts.

## Common Pitfalls

- **Forgetting the bump.** The `check-version-increment` job fails with `::error::VERSION must be strictly greater than the base branch version.` Bump and force-push the branch.
- **Invalid semver.** Forms like `1.2`, `1.2.3-rc1`, `1.2.3-dev`, `1.2.3+sha.abc`, `v1.2.3`, or empty strings fail with `::error::VERSION '<x>' does not match the required format.` The only allowed pre-release tags are exactly `alpha`, `beta`, `rc`, each followed by a literal dot and an integer (e.g. `-rc.1`, never `-rc1`).
- **Going backwards.** `0.18.0 → 0.18.0-rc.1` looks like progress but is a regression: release > rc. Always move forward in the comparison tuple.
- **Two PRs racing for the same number.** Whichever merges last wins; the loser's `check-version-increment` will start failing the moment the base advances past it. Rebase on the updated base branch and bump again.
- **Bumping but forgetting the CHANGELOG.** No CI gate enforces this, but reviewers will (and the release docs workflow lists what shipped per version, so missing entries become invisible history).
- **Bumping for pure docs PRs.** Wastes a registry tag. Prefer to keep docs-only changes off `.env` if possible — but if the gate is failing, an alpha bump is the path of least resistance.
- **Editing `VERSION=` quoting.** The extractor regex `^VERSION\s*=\s*["\']?([^"\'#\s]+)` handles double quotes, single quotes, or no quotes, and stops at `#`/whitespace. Don't add inline comments after the value (e.g. `VERSION="0.18.1" # bumped`) — the trailing `# bumped` will be stripped from the value but obscures intent; put comments on their own line above.
- **Touching only sub-compose `.env` files.** The check looks at the **repo-root** `.env` only. `robot/docker/.env` and friends are container env files, not the version source of truth.
- **Force-pushing after merge to fix CHANGELOG.** Don't. Land a follow-up PR with the CHANGELOG correction (and, by the rules above, another tiny VERSION bump).

## Release Checklist

For a normal feature/fix PR:

1. [ ] Confirm the PR changes Docker image content or otherwise warrants a bump (see "When to Use").
2. [ ] Pick the bump type (see "Choosing the Bump Type").
3. [ ] Edit `/.env` — change only the `VERSION=` line.
4. [ ] Update `CHANGELOG.md` under `## [Unreleased]`.
5. [ ] `airstack version` and `git diff .env CHANGELOG.md` to verify.
6. [ ] Commit (`Bump version to <new>` is the established style).
7. [ ] Push and open the PR. Confirm `Check VERSION Increment` passes green.
8. [ ] After review, merge into `develop` (or `main` per branch policy).
9. [ ] Watch `Auto Build on Docker Image Tag Change` on the merged commit. It runs on the ephemeral GPU runner; expect ~10–60 min depending on profiles. It must succeed before downstream consumers can `docker compose pull` the new tag.

For a true release (dropping the pre-release suffix):

1. [ ] Land final fixes on `develop` with `-rc.N` bumps.
2. [ ] Open a PR that bumps `VERSION="X.Y.Z-rc.N"` → `VERSION="X.Y.Z"`.
3. [ ] In the same PR, promote `## [Unreleased]` to `## [X.Y.Z] - YYYY-MM-DD` and add a fresh empty `## [Unreleased]`.
4. [ ] Merge to `main`.
5. [ ] Wait for `docker-build.yml` to push and sign all images.
6. [ ] Create a GitHub Release with tag `X.Y.Z` (matching `VERSION` exactly). Publishing the release fires `deploy_docs_from_release.yaml`, which runs `mike deploy --push --update-aliases X.Y.Z latest` and updates the versioned docs site.
7. [ ] Verify the docs site shows the new version under the version selector and that `latest` resolves to it.

## References

- [`/.env`](../../../.env) — source of truth for `VERSION=`
- [`/CHANGELOG.md`](../../../CHANGELOG.md) — release history
- [`/.github/workflows/check-version-increment.yml`](../../../.github/workflows/check-version-increment.yml) — the PR gate (semver regex lives here)
- [`/.github/workflows/docker-build.yml`](../../../.github/workflows/docker-build.yml) — build/push/sign on tag change
- [`/.github/workflows/deploy_docs_from_release.yaml`](../../../.github/workflows/deploy_docs_from_release.yaml) — versioned docs on release
- [`/.github/workflows/deploy_docs_from_main.yaml`](../../../.github/workflows/deploy_docs_from_main.yaml) and [`deploy_docs_from_develop.yaml`](../../../.github/workflows/deploy_docs_from_develop.yaml) — branch-tracking docs aliases
- [`/airstack.sh`](../../../airstack.sh) — defines `airstack version` and `get_VERSION` (used everywhere image tags are built)
- [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/)
- [Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html)

## Related Skills

- [`run-system-tests`](../run-system-tests) — what fires on every PR alongside the version check
- [`update-documentation`](../update-documentation) — for docs-only PRs that may still need a VERSION bump to clear the gate
