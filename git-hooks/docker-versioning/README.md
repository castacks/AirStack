# Docker Versioning Hook (deprecated)

Do not install this hook. It writes the git commit hash into the `.env`
`VERSION` variable, which conflicts with the current versioning scheme:
`VERSION` must be valid semver and strictly greater than the base branch,
enforced by the `check-version-increment.yml` CI gate.

See [`git-hooks/README.md`](../README.md) for the supported release flow and
how to remove an existing install of this hook.
