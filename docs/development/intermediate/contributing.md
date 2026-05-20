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

To keep the git histories of `main` and `develop` related, a GitHub Actions workflow (`.github/workflows/sync-develop.yml`) automatically merges `main` back into `develop` after every push to `main`. This ensures that release merge commits and hotfixes are always present in `develop`'s history, preventing divergence and conflicts in future releases.

If the automatic sync fails due to a merge conflict, it will need to be resolved manually:

```bash
git checkout develop
git merge origin/main
# resolve conflicts
git push origin develop
```

## Merge

Submit a pull request to the appropriate base branch per the [Branching Strategy](#branching-strategy) above.

All tests must pass before merging.

Regression tests are run so that we don't break anything.
