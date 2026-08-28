# 004 — Consolidate flat CLI commands into command groups (osmo, config, images)

## Problem context

Three areas of the CLI registered many flat top-level commands, each taking a
line in `airstack help`: `osmo:*` (7 commands), `config` + `config:*` (4), and
`image-build`/`image-push`/`image-pull`/`images`/`image-delete`/`rmi` (6).
Every other multi-verb area (`module`, `stack`, `fleet`) already uses a single
command with a dispatcher plus a detailed `airstack help <group>` page.

## Proposed implementation

- **Groups.** `airstack osmo <sub>` (setup|up|logs|ide|webrtc|foxglove|down),
  `airstack config [sub]` (all default |isaac-sim|nucleus|git-hooks — bare
  `config` keeps running everything), `airstack images [sub]` (list default
  |build|push|pull|delete|rm — bare `images` keeps listing; `rm` absorbs
  `rmi`). Each gets a case in `print_command_help` for `airstack help <group>`.
- **`images` dispatcher quirk.** The subcommand is the first NON-FLAG arg:
  CI legitimately calls `./airstack.sh --progress=quiet images pull ...`, and
  the top-level dispatcher hands flags through in order.
- **Back-compat.** New `COMMAND_HIDDEN` assoc array in `airstack.sh`: hidden
  commands still dispatch but are skipped by `print_usage`/`list_commands`.
  All old spellings (`osmo:up`, `config:nucleus`, `image-build`, `rmi`, ...)
  are registered as hidden aliases that print a one-line deprecation warning
  and forward.
- **Call sites updated** to the new spellings: CI workflows
  (`system-tests.yml`, `module-system-tests.yml`),
  `tests/system/test_build_docker.py`, `tests/conftest.py` hint text,
  AGENTS.md, `.airstack/README.md`, docs (getting_started, ms-airsim,
  docker-build-profiles, ci_cd, both airstack-cli references, the
  extending-the-CLI guide now teaches the dispatcher + COMMAND_HIDDEN
  pattern), and `.agents/skills/` (use-airstack-cli, run-system-tests,
  docker-build-profiles, write-isaac-sim-scene). The pytest flag
  `--no-image-build` keeps its name.

## Test plan

- **a-cli-paths** — `bash -n` all scripts; `airstack help` / `commands` show
  one row per group and no legacy rows; `airstack help <group>` prints the
  subcommand reference; unknown subcommands error with group help; every
  legacy alias warns then forwards; bare `images` lists; bare `config`
  dispatches to all; `--progress=quiet images pull` parses flags-first;
  workflow YAMLs parse; `airstack lint` (contract suite) passes.
