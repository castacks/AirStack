---
name: use-airstack-cli
description: Operate AirStack via the airstack CLI — up/down with launch-intent flags (--sim, --robots, --stack, --fleet, --headless, --no-autolaunch, --dry-run, --wait), module management (add/list/sync/remove/create/lock/doctor), stack and fleet commands, doctor and ready checks — and run commands inside containers using the non-interactive docker exec pattern. Use whenever you need to start/stop services, build the workspace, source the workspace, run ros2 commands, or inspect logs in any AirStack container.
license: BSD-3-Clause-Clear
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Use the AirStack CLI and Container Exec Patterns

## When to Use

Use this skill any time you need to:

- Start, stop, or inspect AirStack services (robot, isaac-sim, ms-airsim, gcs, docs)
- Launch a specific **stack** (`--stack`) or **fleet** (`--fleet`)
- Manage **modules** (`airstack module add|list|sync|remove|create|lock|doctor`)
- Build or source the ROS 2 workspace inside a container
- Run `ros2` commands (node list, topic echo/hz, param get, launch, etc.)
- Tail or grep container logs
- Iterate on code without the autolaunch sequence taking over the container
- Run the system test suite or build the docs site

This skill is the foundation for almost every other AirStack workflow — `debug-module`,
`test-in-simulation`, `add-ros2-package`, `create-module`, `create-stack`, and
`integrate-module-into-layer` all rely on the patterns described here.

## Why `airstack`, Not Raw `docker compose`

Always prefer `airstack <subcommand>` over `docker compose ...` directly:

- Resolves **launch-intent flags** (`--sim`, `--robots`, `--stack`, `--fleet`, ...)
  into exported env vars before compose sees anything, prints the **effective launch
  config**, and saves it under `.airstack/runs/<timestamp>/effective_config.env`.
- Runs **preflight validation** (exactly one sim profile, URDF↔sim match,
  NUM_ROBOTS↔Isaac-script consistency, missing images, removed `AUTONOMY_ROLE`)
  before starting anything. `AIRSTACK_SKIP_PREFLIGHT=1` downgrades errors to warnings.
- Automatically includes the **generated overlays**: module mounts
  (`.airstack/generated/docker-compose.modules.yaml`, opt out with
  `AIRSTACK_NO_MODULE_COMPOSE=1`) and heterogeneous-fleet services
  (`.airstack/generated/docker-compose.fleet.yaml`) — for both `up` and `down`.
- Loads `.env` via `--env-file` and applies the include set from the top-level
  `docker-compose.yaml`; resolves compose **profiles** from `COMPOSE_PROFILES`.
- Gives partial container-name matching for `connect` and `logs`.

Drop to raw `docker` only for: `docker exec <container> bash -c "<cmd>"` (the CLI does
not wrap exec), `docker logs <container>` for raw streams, and `docker ps` to discover
container names.

## Container Lifecycle

### One-time host setup

```bash
# Install Docker Engine + NVIDIA Container Toolkit (skip if already installed)
airstack install

# Configure AirStack: add `airstack` to your shell profile, git submodules,
# Isaac/Nucleus/git-hooks config, and (if modules.repos exists) a module sync
airstack setup
```

### Starting services: `airstack up` and its flags

`airstack up` takes AirStack **intent flags** (consumed before compose sees the args)
plus optional compose service names and passthrough flags (`--build`, `--recreate`):

| Flag | What it does |
|------|--------------|
| `--sim isaac\|airsim` | Selects the simulator: swaps the compose profile (`isaac-sim` / `ms-airsim`) and the matching `URDF_FILE` |
| `--robots N` | Exports `NUM_ROBOTS=N`; on Isaac also keeps `ISAAC_SIM_SCRIPT_NAME` consistent (auto-selects the multi-drone script for N>1) |
| `--stack NAME[:ENTRY]` | Launches `stacks/NAME/launch/ENTRY.launch.xml` (default entry: `stack`). Stacks are the **only** launch dispatch; no `--stack` = `full_default`. `NAME:onboard` / `NAME:offboard` select split-stack entries |
| `--fleet NAME` | Launches `config/fleets/NAME.yaml`: validates it, exports `FLEET_CONFIG_FILE`, derives `NUM_ROBOTS`, selects the Isaac fleet spawner (`fleet_spawn.py`), and for heterogeneous fleets includes the generated per-robot services. Mutually exclusive with `--robots` |
| `--headless` | Sets `ISAAC_SIM_HEADLESS=true`, `MS_AIRSIM_HEADLESS=true`, `QT_QPA_PLATFORM=offscreen` |
| `--play` / `--no-play` | Whether the sim auto-presses Play on start (`PLAY_SIM_ON_START`) |
| `--no-autolaunch` | Containers start **idle** (no tmuxinator launch sequence) — the development mode |
| `--wait` | Block until the stack is flight-ready (runs `airstack ready` after up) |
| `--dry-run` | Print + validate the resolved launch config, start nothing (shadows compose's own `up --dry-run`) |

```bash
airstack up                                    # default profile from .env
airstack up --sim isaac --robots 2             # Isaac, two robots, consistent sim script
airstack up --sim airsim --headless --play --wait
airstack up --stack lite_default --sim isaac   # launch a specific stack
airstack up --stack lite_offload_global:offboard   # split-stack ground half
airstack up --fleet sim_three_mixed --sim isaac    # fleet launch (RFC #380)
airstack up --dry-run --sim isaac              # validate config, start nothing
airstack up robot-desktop                      # one service only
```

**Env vars still work.** Flags only export env vars (shell env has highest compose
precedence), so `AUTOLAUNCH=false airstack up` or `NUM_ROBOTS=2 airstack up` behave
exactly like `--no-autolaunch` / `--robots 2`. Prefer the flags — they validate input
and keep derived settings (profiles, URDF, Isaac script) consistent.

**`AUTONOMY_ROLE` was removed.** A set `AUTONOMY_ROLE` (env, `--env-file`, or `.env`)
is a preflight **error**. Migration: `full` → `full_default` (the no-stack default),
`onboard` → `lite_default`, onboard/offboard split → `lite_offload_global:onboard` /
`:offboard`.

### CRITICAL: `--no-autolaunch` for development

By default `AUTOLAUNCH="true"` in `.env`, so a freshly started robot or sim container
immediately runs its tmuxinator launch sequence. **For development and debugging you
almost always want it disabled** so the container starts idle and you can iterate on
launch files, rebuild packages, and start/stop nodes by hand:

```bash
# Start the robot container without autolaunching the autonomy stack
airstack up --no-autolaunch robot-desktop

# Combine with other flags
airstack up --no-autolaunch --robots 2 --sim isaac
```

(The legacy form `AUTOLAUNCH=false airstack up robot-desktop` still works.)

### Waiting for readiness

```bash
airstack ready          # containers → sim /clock → autonomy nodes → PX4; blocks until green
airstack ready --json   # machine-readable, for scripts
```

Or pass `--wait` to `airstack up` to run it automatically.

### Inspecting and stopping

```bash
airstack status          # all containers with ROBOT_NAME + ROS_DOMAIN_ID columns
airstack logs robot-desktop      # tail logs (partial name matching)
airstack down            # stop everything (includes generated module/fleet services)
airstack down robot-desktop      # stop one service
airstack clean           # remove ALL ROS 2 build artifacts (build/, install/, log/, .egg-info, __pycache__)
```

Note: `airstack clean` deletes host-side colcon build artifacts (forcing a full
rebuild on next up) — it does **not** touch containers, volumes, or networks.

### Container naming convention

Compose generates names of the form `<project>-<service>-<index>`. With the default
`PROJECT_NAME="airstack"`: `airstack-robot-desktop-1`, `airstack-isaac-sim-1`,
`airstack-ms-airsim-1`, `airstack-gcs-1`, `airstack-docs-1`. With `--robots 2` you
also get `airstack-robot-desktop-2`. Always confirm with `airstack status` or
`docker ps --format '{{.Names}}'` rather than guessing.

## Modules, Stacks, Fleets, Doctor (RFC #379/#380)

### Modules

Modules are thin external repos declared in `./modules.repos` (pinned to tags/SHAs —
branch refs are refused), synced into the gitignored `./modules/` dir and overlaid
into the checkout. Guide: `docs/development/modules.md`.

```bash
airstack module add <git-url> --version <tag-or-sha>   # pin + sync (branches refused)
airstack module add ../asm_optitrack                   # local path (recorded under x-local-modules)
airstack module add <url> --version v0.1.0 --no-hooks  # skip host_setup hooks
airstack module list                                   # NAME / TYPE / VERSION/PIN / TARGETS / VALID
airstack module sync [--no-hooks]                      # (re)clone, validate, overlay, layer plan, hooks
airstack module remove <name>                          # drop entry, checkout, overlay artifacts
airstack module create --in-tree <name>                # scaffold robot/ros_ws/src/modules/<name>/ (fork research)
airstack module lock [--build] [--check-conflicts]     # recompute layer_plan.json + modules.lock; --build runs the docker layer chain
airstack module doctor                                 # validate manifests + overlay integrity
airstack module doctor --drift                         # classify fork changes: module-contained vs extraction debt (never blocks)
```

After sync, module mounts are included automatically by `airstack up`
(`.airstack/generated/docker-compose.modules.yaml`; opt out with
`AIRSTACK_NO_MODULE_COMPOSE=1`). Isaac module launch scripts are addressable as
`ISAAC_SIM_SCRIPT_NAME=modules/<module>/<script>.py`.

### Stacks

Stack folders under `stacks/` are self-contained topologies (pinned `modules.repos`,
`launch/` entry points, generated `wiring.md`). Guide: `docs/development/stacks.md`.

```bash
airstack stack list                  # name, entry points (+bridge marker), wiring.md?, airstack_compat
airstack stack new <source> <dest>   # copy a reference stack (does NOT copy wiring.md — regenerate it)
airstack stack diff <a> <b> [--json] # compare generated wiring graphs (nodes/edges/topics/QoS), not XML noise
airstack up --stack <name>[:<entry>] # run one
```

Reference stacks today: `full_default` (the no-stack default), `full_droan_cpu`,
`full_macvo`, `lite_default`, `lite_offload_global` (split: `:onboard`/`:offboard`).

### Fleets

Fleet files under `config/fleets/` declare who exists, which vehicle, which stack,
and which ground hosts run split-stack offboard halves. Guide:
`docs/development/fleets.md`.

```bash
airstack fleet list              # table: robots, vehicles, stacks, shape (homogeneous/heterogeneous/+split)
airstack fleet generate <fleet>  # write .airstack/generated/docker-compose.fleet.yaml (heterogeneous only;
                                 # homogeneous fleets need no generation — deploy.replicas handles them)
airstack up --fleet <name> [--sim isaac|airsim]
```

### Sync from `airstack.yaml`

```bash
airstack sync    # upsert airstack.yaml modules into modules.repos, run module sync,
                 # fetch external stack repos into stacks/.external/<alias>/,
                 # validate the declared fleet, record effective_sources.yaml
```

### Doctor

Observe-and-report health checks. Default mode exits non-zero only on the two hard
gates (module dep conflicts, control-setpoint topics in a `bridge.yaml`).

```bash
airstack doctor                       # compose-time checks: manifests, overlay, dep conflicts, stack anatomy, bridge safety
airstack doctor --live                # diff the RUNNING graph vs the stack's committed wiring.md (exit 1 on drift)
airstack doctor --live --strict       # make safety-floor warnings fatal
airstack doctor --snapshot            # WRITE the observed graph to stacks/<name>/wiring.md (hardware bring-up path)
airstack doctor --live --stack NAME   # name the stack explicitly (default: inferred from AIRSTACK_STACK_DIR)
```

## Running Commands Inside Containers

### The mandatory pattern for agents

```bash
docker exec <container> bash -c "<command>"
```

**Never use `docker exec -it`.** Interactive mode opens a TTY, which hangs the agent
waiting for input, leaves you in the host shell after the command exits, and can't be
captured cleanly by tool-result parsing. The non-interactive `bash -c "..."` form runs,
exits, and returns stdout/stderr.

### Examples

```bash
# Quick health check
docker exec airstack-robot-desktop-1 bash -c "ros2 node list"

# Multiple commands chained — sws first, then a ros2 call
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 topic list | grep odom"

# Echo a topic exactly once (will exit cleanly)
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot_1/odometry --once"

# Measure a topic's rate for a few seconds, then stop
docker exec airstack-robot-desktop-1 bash -c "timeout 5 ros2 topic hz /robot_1/odometry"
```

The `timeout N <cmd>` wrapper is invaluable for any `ros2` command that would otherwise
run indefinitely (`topic echo`, `topic hz`, `bag record`, `launch`).

### `airstack connect` is for humans only

`airstack connect <name>` opens an interactive shell into a container. Agents must not
call it — it opens a TTY and hangs the tool call. Use `docker exec ... bash -c "..."`.

## Building and Sourcing

Inside every robot/desktop container, two aliases are pre-installed:

| Alias | Expands to                                 | Purpose                          |
|-------|--------------------------------------------|----------------------------------|
| `bws` | `colcon build` with the AirStack flag set  | Build the ROS 2 workspace        |
| `sws` | `source install/setup.bash`                | Source the workspace overlay     |

### Build the whole workspace

```bash
docker exec airstack-robot-desktop-1 bash -c "bws"
```

### Build a single package (the common case during iteration)

```bash
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package"
```

### Build with debug symbols (for GDB / valgrind)

```bash
docker exec airstack-robot-desktop-1 bash -c \
  "bws --packages-select my_package --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
```

### Source the workspace before running ros2

`bws` builds, but the new install tree is **not** automatically on the path of a fresh
`docker exec`. Always chain `sws &&` before any `ros2 run` or `ros2 launch`:

```bash
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch my_package my_launch.xml"
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 run my_package my_node"
```

### Iteration loop after editing C++ code

```bash
# Edit on host (code is bind-mounted into the container), then rebuild + relaunch:
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package"
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 run my_package my_node"
```

For Python-only changes, run `bws` once after creating the package (to install the
entry point); later edits are picked up live without rebuilding.

## Log Inspection and Debugging

### CLI logs (recommended)

```bash
# Tail logs (partial name matching)
airstack logs robot-desktop
airstack logs isaac-sim
```

### Raw docker logs (for grep, redirection, last-N lines)

```bash
# Last 100 lines
docker logs --tail 100 airstack-robot-desktop-1

# Stream and grep for errors
docker logs -f airstack-robot-desktop-1 2>&1 | grep -iE "error|fail|crash"

# Save full log for offline analysis
docker logs airstack-robot-desktop-1 > /tmp/robot.log 2>&1
```

### Per-node logs inside the container

ROS 2 writes per-node logs under `~/.ros/log/`:

```bash
docker exec airstack-robot-desktop-1 bash -c "ls -la ~/.ros/log/latest/"
docker exec airstack-robot-desktop-1 bash -c "tail -100 ~/.ros/log/latest/<node>.log"
```

### Live container resource usage

```bash
docker stats --no-stream airstack-robot-desktop-1
```

## Other Useful Subcommands

### Documentation

```bash
# Build and serve the MkDocs site
airstack docs
```

### Tests

The system test suite (pytest, runs against the full Docker stack) is invoked through:

```bash
airstack test -m unit -v
airstack test -m "build_docker or build_packages" -v
airstack test -m liveliness --sim msairsim --num-robots 1 -v
airstack test -m sensors --sim isaacsim --num-robots 1 -v   # topic Hz + LiDAR (after liveliness if both selected)
airstack test -m takeoff_hover_land --sim msairsim --takeoff-velocities 0.5,1,2 -v
airstack test -m wiring --stack full_default -v             # observed-wiring drift check / regeneration
```

For full details on **pytest system tests** (fixtures, all 11 marks, `--stack` /
`--fleet` options, metrics, `/pytest` CI), see the
[`run-system-tests`](../run-system-tests/SKILL.md) skill and
[`tests/README.md`](../../../tests/README.md). For **authoring and running
scenarios inside Isaac Sim or AirSim** (missions, RViz checks, scene tweaks), see
the [`test-in-simulation`](../test-in-simulation/SKILL.md) skill.

### Lint and format

```bash
airstack lint
airstack format
```

### Image management

```bash
airstack images          # List AirStack images
airstack images build     # Build images locally
airstack images push      # Push to registry
airstack images pull      # Pull from registry
airstack images delete    # Remove all matching images
```

### Configuration helpers

```bash
airstack config              # Run all config steps
airstack config isaac-sim   # Configure Isaac Sim cache/settings
airstack config nucleus     # Configure Omniverse Nucleus credentials
airstack config git-hooks   # Install git pre-commit hooks
```

### Per-command help

```bash
airstack help <command>      # detailed usage for up, module, stack, fleet, doctor, sync, test, ...
airstack commands            # list every registered command
```

## Common Pitfalls

1. **Forgetting `--no-autolaunch` during development**
   - Symptom: container starts the full autonomy stack, ports are taken, you cannot
     iterate on launch files.
   - Fix: bring it down with `airstack down`, then `airstack up --no-autolaunch <svc>`.

2. **Using `docker exec -it` from an agent**
   - Symptom: the tool call hangs until timeout.
   - Fix: always use `docker exec <container> bash -c "<command>"`. Add
     `timeout N` in front of any long-running ROS 2 command.

3. **Running `ros2` on the host**
   - Symptom: `command not found` or a stale system ROS install responds.
   - Fix: every `ros2 ...` invocation must be wrapped in
     `docker exec airstack-robot-desktop-1 bash -c "..."`. Host has no AirStack overlay.

4. **Forgetting to `sws` after `bws`**
   - Symptom: `Package 'my_package' not found` or `ros2 launch` reports the launch file
     missing even though the build succeeded.
   - Fix: chain them: `bash -c "sws && ros2 launch my_package my_launch.xml"`. Each
     `docker exec` is a fresh shell — you must source every time.

5. **Forgetting to `bws` after editing C++**
   - Symptom: behavior is unchanged after code edits; old binary still runs.
   - Fix: `bws --packages-select my_package` before relaunching the node.

6. **Guessing container names**
   - Symptom: `Error: No such container: airstack-robot-1` (the actual name is
     `airstack-robot-desktop-1`).
   - Fix: list them with `airstack status` or
     `docker ps --format '{{.Names}}'` first.

7. **Setting `AUTONOMY_ROLE`**
   - Symptom: preflight error — `AUTONOMY_ROLE was removed`.
   - Fix: select a stack instead: `airstack up --stack <name>[:<entry>]`
     (no `--stack` = `full_default`).

8. **Using `docker compose` directly instead of `airstack up`**
   - Symptom: missing profiles, missing env vars, missing module/fleet overlays,
     no preflight.
   - Fix: always go through `airstack up` / `airstack down`, which include the
     generated compose files and validate the resolved config.

9. **Adding a module pinned to a branch**
   - Symptom: `'main' looks like a branch, not a pin.`
   - Fix: pin a tag or a commit SHA: `airstack module add <url> --version v0.1.0`.

10. **Module code missing inside the container after `module add`**
    - Symptom: package not found although `airstack module list` shows it.
    - Fix: `airstack up` must include the generated overlay (it does by default —
      check you didn't set `AIRSTACK_NO_MODULE_COMPOSE=1`), and the workspace must
      be rebuilt (`bws`) after the overlay changes.

## Quick Reference Cheatsheet

```bash
# ---- Lifecycle ----
airstack install                                 # Install Docker + nvidia-container-toolkit (one time)
airstack setup                                   # Shell function, submodules, config, module sync (one time)
airstack up                                      # Start default profile from .env (stack: full_default)
airstack up --sim isaac|airsim                   # Pick the simulator (profile + URDF + Isaac script derived)
airstack up --sim isaac --robots 2               # Multi-robot (keeps NUM_ROBOTS and the sim script consistent)
airstack up --stack lite_default --sim isaac     # Launch a specific stack
airstack up --stack lite_offload_global:offboard # Split-stack entry point
airstack up --fleet sim_three_mixed --sim isaac  # Fleet launch (fleet file defines the robots)
airstack up --headless --play --wait             # Headless, auto-play sim, block until flight-ready
airstack up --dry-run --sim airsim               # Print + validate resolved config; start nothing
airstack ready                                   # Wait until flight-ready (--json for scripts)
airstack up robot-desktop                        # Start one service
airstack up --no-autolaunch robot-desktop        # Start idle (for development) — IMPORTANT
airstack status                                  # List running containers (+ROBOT_NAME, ROS_DOMAIN_ID)
airstack down                                    # Stop and remove containers (incl. generated services)
airstack clean                                   # Remove ROS 2 build artifacts (forces full rebuild)
airstack logs robot-desktop                      # Tail logs (partial name OK)

# ---- Modules / stacks / fleets ----
airstack module add <url> --version v0.1.0       # Pin + sync an external module (branches refused)
airstack module list|sync|remove <name>          # Table / re-sync + overlay / drop
airstack module lock --build                     # Build the module Docker layer chain
airstack module doctor [--drift]                 # Manifests + overlay / fork-drift report
airstack stack list|new <src> <dst>|diff <a> <b> # Stacks: table / copy / wiring diff
airstack fleet list|generate <name>              # Fleets: table / per-robot compose (heterogeneous)
airstack sync                                    # Sync checkout from airstack.yaml
airstack doctor [--live|--snapshot] [--stack N]  # Health checks / live wiring drift / snapshot

# ---- Exec inside container (NEVER use -it) ----
docker exec airstack-robot-desktop-1 bash -c "ros2 node list"
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 topic list"
docker exec airstack-robot-desktop-1 bash -c "timeout 5 ros2 topic hz /robot_1/odometry"
docker exec airstack-robot-desktop-1 bash -c "ros2 topic echo /robot_1/odometry --once"

# ---- Build & source ----
docker exec airstack-robot-desktop-1 bash -c "bws"
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package"
docker exec airstack-robot-desktop-1 bash -c "bws --packages-select my_package --cmake-args '-DCMAKE_BUILD_TYPE=Debug'"
docker exec airstack-robot-desktop-1 bash -c "sws && ros2 launch my_package my_launch.xml"

# ---- Logs ----
docker logs --tail 100 airstack-robot-desktop-1
docker logs -f airstack-robot-desktop-1 2>&1 | grep -iE "error|fail"

# ---- Other ----
airstack docs                                    # Build + serve MkDocs
airstack test -m liveliness -v                   # Stack infra tests
airstack test -m sensors -v                      # Sensor topic + LiDAR tests (Isaac batching — see tests/README)
airstack lint                                    # Lint
airstack format                                  # Format
```

## References

- [`AGENTS.md`](../../../AGENTS.md) — sections "AirStack CLI Tool" and
  "Docker Development Workflow"
- [`.airstack/README.md`](../../../.airstack/README.md) — full CLI documentation,
  including command-module extension and troubleshooting
- [`docs/development/modules.md`](../../../docs/development/modules.md) — module
  system guide (dep tiers, overlay, lock)
- [`docs/development/stacks.md`](../../../docs/development/stacks.md) — stack folders,
  entry points, wiring.md
- [`docs/development/fleets.md`](../../../docs/development/fleets.md) — fleet files
  and split placement
- [`.env`](../../../.env) — every variable that can be overridden on the
  `airstack up` command line
- [`docker-compose.yaml`](../../../docker-compose.yaml) — top-level compose file
  showing which sub-compose files are included
- **Related Skills:**
  - [create-module](../create-module) / [create-stack](../create-stack) — the things
    `module`/`stack` commands manage
  - [configure-multi-robot](../configure-multi-robot) — fleets and legacy NUM_ROBOTS
  - [debug-module](../debug-module) — uses these exec patterns for diagnostics
  - [test-in-simulation](../test-in-simulation) — uses `airstack up` and exec patterns
  - [add-ros2-package](../add-ros2-package) — uses `bws --packages-select` for builds
  - [integrate-module-into-layer](../integrate-module-into-layer) — uses
    `--no-autolaunch` to verify launch file changes
