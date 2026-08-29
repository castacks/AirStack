# Git Hooks (deprecated)

!!! warning "These hooks are obsolete — do not install them."

The docker-versioning pre-commit hook in this directory wrote the current git
commit hash into the `.env` `VERSION` variable. That scheme has been replaced:
`VERSION` must now be **valid semver, strictly greater than the base branch**,
enforced by the `check-version-increment.yml` CI gate on every pull request.
A hook-written commit hash fails that gate.

The supported flow is:

1. Bump `VERSION` in `.env` to the next semver value.
2. Record the change in the versioned Release Notes
   (`docs/release_notes/index.md`).

See the `bump-version-and-release` skill (`.agents/skills/bump-version-and-release`)
for the full workflow.

> Note: `airstack config git-hooks` (and `airstack config all`) still installs
> the old hook from this directory; until that CLI path is removed, avoid
> running it. The hook script is kept only so existing installs can be
> identified and removed (`rm .git/hooks/pre-commit`).
