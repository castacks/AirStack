---
name: use-airstack-cli
description: Operate AirStack via the airstack CLI and run commands inside containers using the non-interactive docker exec pattern. Use whenever you need to start/stop services, build the workspace, source the workspace, run ros2 commands, or inspect logs in any AirStack container.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Use the AirStack CLI and Container Exec Patterns

## When to Use

Use this skill any time you need to:

- Start, stop, or inspect AirStack services (robot, isaac-sim, ms-airsim, gcs, docs)
- Build or source the ROS 2 workspace inside a container
- Run `ros2` commands (node list, topic echo/hz, param get, launch, etc.)
- Tail or grep container logs
- Iterate on code without the autolaunch sequence taking over the container
- Run the system test suite or build the docs site

This skill is the foundation for almost every other AirStack workflow — `debug-module`,
`test-in-simulation`, `add-ros2-package`, and `integrate-module-into-layer` all rely on
the patterns described here.

## Why `airstack`, Not Raw `docker compose`

Always prefer `airstack <subcommand>` over `docker compose ...` directly:

- Runs a **containerized** docker-compose pinned to a known version (consistent across
  hosts and CI runners).
- Loads `.env`, propagates host env overrides, and applies the right include set from
  the top-level `docker-compose.yaml` (isaac-sim / ms-airsim / robot / gcs / docs).
- Resolves Compose **profiles** (`desktop`, `isaac-sim`, `ms-airsim`, etc.) from
  `COMPOSE_PROFILES` in `.env` automatically.
- Gives partial container-name matching for `connect` and `logs`.

Drop to raw `docker` only for: `docker exec <container> bash -c "<cmd>"` (CLI does not
wrap exec), `docker logs <container>` for raw streams, and `docker ps` to discover
container names.

## Container Lifecycle

### One-time host setup

```bash
# Install Docker Engine + NVIDIA Container Toolkit (skip if already installed)
airstack install

# Configure AirStack: add `airstack` to PATH, set up shell completion, etc.
airstack setup
```

`airstack install` is only needed once per host (and only if Docker / nvidia-container-toolkit
are missing). `airstack setup` is needed once per shell user; rerun if you switch shells
(bash <-> zsh) or if `~/.airstack.conf` is missing.

### Starting services

The most common entrypoints:

```bash
# Start the default profile from .env (typically: desktop + isaac-sim)
airstack up

# Start a specific service (matches docker-compose service name)
airstack up robot-desktop
airstack up isaac-sim
airstack up ms-airsim
airstack up gcs

# Start multiple services
airstack up isaac-sim robot-desktop
```

### CRITICAL: `AUTOLAUNCH=false` for development

By default, `AUTOLAUNCH="true"` in `.env`, which means a freshly started robot or sim
container immediately runs its tmuxinator launch sequence. **For development and
debugging, you almost always want this disabled** so the container starts idle and you
can iterate on launch files, rebuild packages, and start/stop nodes by hand:

```bash
# Start the robot container without autolaunching the autonomy stack
AUTOLAUNCH=false airstack up robot-desktop

# Combine with other overrides
AUTOLAUNCH=false NUM_ROBOTS=2 airstack up robot-desktop isaac-sim
```

Any variable defined in `.env` can be overridden this way (the wrapper exports each
`.env` key into the compose container). Common ones for agents:

| Variable                     | What it controls                                          |
|------------------------------|-----------------------------------------------------------|
| `AUTOLAUNCH`                 | Whether the container auto-runs the launch sequence       |
| `NUM_ROBOTS`                 | How many robot containers spawn                           |
| `ROBOT_NAME`                 | Namespace prefix for ROS topics                           |
| `VERSION`                    | Docker image tag to use                                   |
| `COMPOSE_PROFILES`           | Which compose profiles are active                         |
| `ISAAC_SIM_USE_STANDALONE`   | Run Isaac Sim as a standalone Python script               |
| `ISAAC_SIM_SCRIPT_NAME`      | Which Isaac Sim launch script to run                      |

### Inspecting and stopping

```bash
# Show all running containers (with airstack container names)
airstack status

# Tail logs for a single container (partial name matching works)
airstack logs robot-desktop
airstack logs isaac-sim

# Stop services but keep containers around
airstack stop
airstack stop robot-desktop

# Stop and remove all containers (clean slate)
airstack down
```

### Container naming convention

Compose generates names of the form `<project>-<service>-<index>`. With the default
`PROJECT_NAME="airstack"`: `airstack-robot-desktop-1`, `airstack-isaac-sim-1`,
`airstack-ms-airsim-1`, `airstack-gcs-1`, `airstack-docs-1`. With `NUM_ROBOTS=2` you
also get `airstack-robot-desktop-2`. Always confirm with `airstack status` or
`docker ps --format '{{.Names}}'` rather than guessing.

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
airstack test -m "build_docker or build_packages" -v
airstack test -m liveliness --sim msairsim --num-robots 1 -v
airstack test -m takeoff_hover_land --sim msairsim --takeoff-velocities 0.5,1,2 -v
```

For full details on the test fixtures, marks, and metrics reporting, see the
`test-in-simulation` skill and `tests/README.md`.

### Lint and format

```bash
airstack lint
airstack format
```

### Image management

```bash
airstack images          # List AirStack images
airstack image build     # Build images locally
airstack image push      # Push to registry
airstack image pull      # Pull from registry
airstack image delete    # Remove a tagged image
airstack rmi             # Remove all AirStack images
```

### Configuration helpers

```bash
airstack config              # Run all config steps
airstack config:isaac-sim    # Configure Isaac Sim cache/settings
airstack config:nucleus      # Configure Omniverse Nucleus credentials
airstack config:git-hooks    # Install git pre-commit hooks
```

## Common Pitfalls

1. **Forgetting `AUTOLAUNCH=false` during development**
   - Symptom: container starts the full autonomy stack, ports are taken, you cannot
     iterate on launch files.
   - Fix: bring it down with `airstack down`, then `AUTOLAUNCH=false airstack up <svc>`.

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

7. **Setting overrides inside the wrong shell context**
   - Symptom: `NUM_ROBOTS=2 airstack up` is run in a sub-shell that exited before
     the up call — the value is lost.
   - Fix: prefix the variable on the same line as `airstack up`, or `export` it first.

8. **Using `docker compose` directly instead of `airstack up`**
   - Symptom: missing profiles, missing env vars, mismatched compose-plugin version.
   - Fix: always go through `airstack up`, which mounts the right project directory
     and pinned compose binary.

## Quick Reference Cheatsheet

```bash
# ---- Lifecycle ----
airstack install                                 # Install Docker + nvidia-container-toolkit (one time)
airstack setup                                   # Add airstack to PATH (one time per shell)
airstack up                                      # Start default profile from .env
airstack up robot-desktop                        # Start one service
AUTOLAUNCH=false airstack up robot-desktop       # Start idle (for development) — IMPORTANT
NUM_ROBOTS=2 AUTOLAUNCH=false airstack up        # Multi-robot, idle
airstack status                                  # List running containers
airstack stop                                    # Stop services
airstack down                                    # Stop and remove containers
airstack logs robot-desktop                      # Tail logs (partial name OK)

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
airstack test -m liveliness -v                   # Run system tests
airstack lint                                    # Lint
airstack format                                  # Format
```

## References

- [`AGENTS.md`](../../../AGENTS.md) — sections "AirStack CLI Tool" and
  "Docker Development Workflow"
- [`.airstack/README.md`](../../../.airstack/README.md) — full CLI documentation,
  including module extension and troubleshooting
- [`.env`](../../../.env) — every variable that can be overridden on the
  `airstack up` command line
- [`docker-compose.yaml`](../../../docker-compose.yaml) — top-level compose file
  showing which sub-compose files are included
- **Related Skills:**
  - [debug-module](../debug-module) — uses these exec patterns for diagnostics
  - [test-in-simulation](../test-in-simulation) — uses `airstack up` and exec patterns
  - [add-ros2-package](../add-ros2-package) — uses `bws --packages-select` for builds
  - [integrate-module-into-layer](../integrate-module-into-layer) — uses
    `AUTOLAUNCH=false` to verify launch file changes
