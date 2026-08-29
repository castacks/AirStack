# AirStack CLI (`airstack.sh`)

`airstack.sh` is a **host-side bash CLI** (bash ≥ 4; it re-execs a newer bash
on macOS). It wraps the native `docker compose` on the host — there is no
containerized CLI — and loads drop-in command modules from
`.airstack/modules/*.sh` at startup: each module defines `cmd_*` functions and
a `register_<name>_commands` function that fills the `COMMANDS` /
`COMMAND_HELP` maps (plus `COMMAND_HIDDEN` for deprecated alias spellings
that should dispatch but stay out of the help listings). `_lib.sh` holds
shared helpers and is sourced first.

`airstack setup` installs a shell function that finds the nearest checkout's
`airstack.sh`, so `airstack <command>` works from any subdirectory.

## Command map (by source file)

| Source | Commands |
| ------ | -------- |
| `airstack.sh` (built-ins) | `install`, `setup`, `up`, `down`, `clean`, `connect`, `status`, `logs`, `version`, `images` (`list`, `build`, `push`, `pull`, `delete`, `rm`), `help` (+ `commands` to list everything) |
| `modules/config.sh` | `config` (`all`, `isaac-sim`, `nucleus`, `git-hooks`) |
| `modules/dev.sh` | `test` (containerized pytest runner), `docs`, `lint` |
| `modules/doctor.sh` | `doctor` (observe-and-report checks; `--live`, `--snapshot`) |
| `modules/fleet.sh` | `fleet` (`list`, `generate` — per-robot compose + split-stack DDS-router configs) |
| `modules/module.sh` | `module` (`add`, `remove`, `list`, `sync`, `create`, `lock`, `doctor`) |
| `modules/osmo.sh` | `osmo` (`setup`, `up`, `logs`, `ide`, `webrtc`, `foxglove`, `down`) |
| `modules/ready.sh` | `ready` (wait for flight-readiness; `--json`) |
| `modules/stack.sh` | `stack` (`list`, `new`, `diff`) |
| `modules/sync.sh` | `sync` (checkout sync from `airstack.yaml`; `--no-hooks`) |

`airstack help <command>` prints per-command usage; `airstack commands` lists
everything grouped by prefix. Launch flags on `up` (`--sim`, `--robots`,
`--stack`, `--fleet`, `--headless`, `--play/--no-play`, `--no-autolaunch`,
`--wait`, `--dry-run`) derive and export env vars — see `airstack help up`.

## Adding a command

Drop a file in `.airstack/modules/` — it is auto-loaded (files starting with
`_` are shared libraries, skipped by the loader):

```bash
# .airstack/modules/mymodule.sh
function cmd_mymodule_hello { log_info "hello from $PROJECT_ROOT"; }
function register_mymodule_commands {
    COMMANDS["hello"]="cmd_mymodule_hello"
    COMMAND_HELP["hello"]="Say hello"
}
```

Available helpers: `log_info` / `log_warn` / `log_error`, `check_docker`,
`run_docker_compose` (base compose file + `.env` folded in),
`resolve_launch_var` (env > `--env-file` > `.env` precedence), and the
`_lib.sh` shared helpers (`_require_python_yaml`, `_env_value`,
`_robot_containers`, `_container_identity`).

## Also in this directory

- `generated/` — machine-local generated artifacts (module/fleet compose
  overlays, DDS-router configs, layer plans); gitignored, regenerated on demand.
- `runs/` — effective launch config per `airstack up` (newest 50 kept).

## Further reading

- Modules: [docs/development/modules.md](../docs/development/modules.md)
- Stacks: [docs/development/stacks.md](../docs/development/stacks.md)
- Fleets: [docs/development/fleets.md](../docs/development/fleets.md)
- CLI internals & extension guide:
  [docs/development/advanced/airstack-cli/](../docs/development/advanced/airstack-cli/)
