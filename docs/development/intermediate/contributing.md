# Contributing

This page describes how to merge content back into main.

## Dependencies
Make sure to add your ROS2 package dependencies to your `package.xml` file.
These get installed when the docker image is built.

If you need to add a dependency that's not in the docker image, please add a section to the `Dockerfile` in the `docker/` directory.

## Documentation

Please make sure to document your work.
Docs are under `AirStack/docs/`. The navigation tree is under `AirStack/mkdocs.yml`.

This documentation is built with Material MKDocs.
Visit [mkdocs.org](https://www.mkdocs.org) and [mkdocs-material](https://squidfunk.github.io/mkdocs-material/) to learn how to use it.

### Commands

```
pip install mkdocs-material
mkdocs serve
```
Launches docs on https://localhost:8000.

- `mkdocs -h` - Print help message and exit.

### Project layout

    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.

## Branching Strategy

This project follows a [Gitflow](https://nvie.com/posts/a-successful-git-branching-model/)-inspired branching model with two long-lived branches:

- **`main`** — always reflects production-ready code. Only receives merges from `develop` (releases) and `hotfix/*` branches (urgent fixes).
- **`develop`** — the integration branch where all new features and non-urgent fixes are merged.

### Feature branches

For new features and non-urgent bug fixes, branch off `develop`:

```bash
git checkout develop
git checkout -b feature/my-feature
```

Open your pull request targeting `develop`. PRs targeting `main` from non-hotfix branches will be automatically rejected (see [Branch Enforcement](#branch-enforcement) below).

### Hotfix branches

For urgent fixes that must go directly to production, branch off `main`:

```bash
git checkout main
git checkout -b hotfix/my-fix
```

Open your pull request targeting `main`. After it merges, the fix is automatically synced back to `develop` (see [Automatic Sync](#automatic-sync-main--develop) below).

## Branch Enforcement

A GitHub Actions workflow (`.github/workflows/enforce-branch-targets.yml`) runs on every pull request and enforces the following rules:

| Source branch | Allowed target | Blocked target |
|---|---|---|
| `feature/*`, `fix/*`, or any non-hotfix branch | `develop` | `main` |
| `hotfix/*` | `main` | `develop` |
| `develop` | `main` | — |
| `main` | `develop` | — |

If your PR targets the wrong base branch, the check will fail with a message explaining the violation. To fix it, close the PR and reopen it against the correct base branch.

## Automatic Sync: main → develop

To keep the git histories of `main` and `develop` related, a GitHub Actions workflow (`.github/workflows/sync-develop-from-main.yaml`) merges `main` back into `develop` and pushes directly after every push to `main`. The workflow bypasses `develop`'s ruleset using a `SYNC_PAT` secret owned by a Repository admin. This ensures that release merge commits and hotfixes are always present in `develop`'s history, preventing divergence and conflicts in future releases.

### VERSION handling on develop

`develop` always carries a pre-release VERSION (e.g. `0.19.0-alpha.3`) so that it stays strictly greater than `main` and satisfies the `Verify VERSION is valid and incremented` check (`.github/workflows/check-version-increment.yml`), which requires every PR to bump `.env`'s `VERSION` above its base branch. The sync workflow bumps `develop`'s VERSION as part of the merge using two rules:

| Condition | Action | Example |
|---|---|---|
| `main`'s `x.y.z` ≥ `develop`'s base `x.y.z` (a release just landed on main) | Roll `develop` to the next minor's `alpha.0` | main `0.19.0`, develop `0.19.0-alpha.7` → develop `0.20.0-alpha.0` |
| `main`'s `x.y.z` < `develop`'s base (a hotfix landed on main) | Preserve `develop`'s pre-release channel and bump the counter | main `0.19.1`, develop `0.20.0-alpha.0` → develop `0.20.0-alpha.1` |

The workflow auto-resolves conflicts on the `VERSION=` line of `.env` (keeps `develop`'s side, then applies the bump). Any other merge conflict aborts the sync and must be resolved manually:

```bash
git checkout -B sync/main-to-develop origin/develop
git merge origin/main
# resolve conflicts
# manually bump VERSION in .env per the rules above
git commit
git push --force-with-lease origin sync/main-to-develop
# then open / update the PR targeting develop
```

## Merge

Submit a pull request to the appropriate base branch per the [Branching Strategy](#branching-strategy) above.

All tests must pass before merging.

Regression tests are run so that we don't break anything.
