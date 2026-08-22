# AirStack CLI Command Reference

This is your reference guide for all `airstack` CLI commands. If you haven't already, see [Key Concepts](../key_concepts.md) for the philosophy behind the CLI.

!!! tip "Quick Help"

    - `airstack commands` - List all available commands
    - `airstack help <command>` - Get help for a specific command

## Setup

Before using the CLI, run setup to add it to your PATH:

```bash
./airstack.sh setup
```

This adds the `airstack` command to your shell profile so you can use it from any directory.

## Basic Usage

```bash
airstack <command> [options]
airstack commands              # List all commands
airstack help <command>        # Get help for a command
```

## Command Reference

All commands, grouped by area. Run `airstack help <command>` for full options.

### Setup & Installation

| Command | Description |
|---------|-------------|
| `install` | Install dependencies (Docker Engine, NVIDIA Container Toolkit) |
| `setup` | Configure AirStack settings and add to shell profile |
| `config` | Run all configuration tasks |
| `config:isaac-sim` | Configure Isaac Sim settings |
| `config:nucleus` | Configure AirLab Nucleus login |
| `config:git-hooks` | Set up Git hooks |

### Container Lifecycle

| Command | Description |
|---------|-------------|
| `up` | Start services (`--sim`, `--robots`, `--stack`, `--fleet`, `--headless`, `--play`/`--no-play`, `--no-autolaunch`, `--wait`, `--dry-run` — see flags table below) |
| `down` | Stop services |
| `status` | Show status of all containers |
| `connect` | Connect to a running container (supports partial name matching) |
| `logs` | View logs for a container (supports partial name matching) |
| `clean` | Remove all ROS 2 build artifacts (build/, install/, log/) |

### Docker Images

| Command | Description |
|---------|-------------|
| `image-build` | Build or rebuild Docker Compose service images |
| `image-push` | Push Docker Compose service images to a registry |
| `image-pull` | Pull Docker Compose service images from a registry |
| `images` | List Docker images filtered by PROJECT_NAME from .env |
| `image-delete` | Delete all Docker images matching PROJECT_NAME (prompts unless -y) |
| `rmi` | Remove Docker images by search term |

### Modules, Stacks, and Fleets

| Command | Description |
|---------|-------------|
| `module` | Manage AirStack modules: add\|remove\|list\|sync\|create\|lock\|doctor (see `airstack help module`) |
| `stack` | Manage stack folders: list\|new\|diff (see `airstack help stack`) |
| `fleet` | Manage fleet files: list\|generate (see `airstack help fleet`) |
| `sync` | Sync the checkout from airstack.yaml: modules, external stack repos, fleet validation |

See the [Modular AirStack Walkthrough](../../../getting_started/modular_airstack.md) and the guides for [Modules](../../modules.md), [Stacks](../../stacks.md), and [Fleets](../../fleets.md).

### Diagnostics

| Command | Description |
|---------|-------------|
| `doctor` | Observe-and-report health checks: compose-time gates, `--live` wiring drift, `--snapshot` |
| `ready` | Wait until the running stack is flight-ready (containers → sim clock → nodes → PX4); `--json` for scripts |
| `version` | Display the current AirStack version |
| `help` | Show help information |
| `commands` | List all available commands |

### Development Tasks

| Command | Description |
|---------|-------------|
| `test` | Run pytest in the containerized test runner (all args forward to pytest; see `airstack help test`) |
| `docs` | Build documentation (options: serve) |
| `lint` | Static checks: tests/meta contract suite, `bash -n` over the CLI, `py_compile` over `tools/` |

### Remote Development (OSMO)

| Command | Description |
|---------|-------------|
| `osmo:setup` | One-time per-user OSMO credential setup (airlab-docker-registry, airlab-docker-login, airlab-nucleus) |
| `osmo:up` | Submit osmo/workflows/airstack-dev.yaml with your SSH pubkey injected (`--pool POOL`, `--key PATH`, `--branch BRANCH`) |
| `osmo:logs` | Follow the workspace task logs |
| `osmo:ide` | Port-forward sshd (2200:22) and open VS Code/Cursor on Host airstack-osmo |
| `osmo:webrtc` | Port-forward Isaac Sim WebRTC ranges (TCP foreground + UDP background) |
| `osmo:foxglove` | Install AirStack Foxglove extensions locally, then port-forward GCS Foxglove websocket (8766:8766) |
| `osmo:down` | Cancel the active workflow (push to git before running this) |

## `airstack up` Flags

`airstack up [service...] [flags]` consumes these launch-intent flags before passing everything else through to Docker Compose:

| Flag | Description |
|------|-------------|
| `--sim isaac\|airsim\|simple` | Pick the simulator: swaps in the matching compose profile (`isaac-sim` / `ms-airsim`) and selects the matching URDF; `simple` launches the lightweight kinematic simple-sim (no PX4/MAVROS) with the `simple-robot` service in place of `robot-desktop` |
| `--robots N` | Number of robots (positive integer): exports `NUM_ROBOTS=N` and, on Isaac Sim, auto-selects the single-/multi-drone launch script to match. Mutually exclusive with `--fleet` (the fleet file defines the robot count) |
| `--headless` | Run the simulator without a GUI (sets `ISAAC_SIM_HEADLESS`, `MS_AIRSIM_HEADLESS`, offscreen Qt) |
| `--play` / `--no-play` | Start (or don't start) simulation playback automatically (`PLAY_SIM_ON_START`) |
| `--no-autolaunch` | Start containers without auto-launching the autonomy stack (`AUTOLAUNCH=false`) — the usual mode for development |
| `--wait` | After starting containers, block until the stack is flight-ready (runs `airstack ready`) |
| `--dry-run` | Validate the resolved configuration and preflight checks, then exit without starting services |
| `--stack NAME[:ENTRY]` | Launch a stack folder: `stacks/NAME/launch/ENTRY.launch.xml` (default entry `stack`). Stacks are the only launch dispatch; no `--stack` launches the trunk reference stack `full_default`. See [Stacks](../../stacks.md) |
| `--fleet NAME` | Launch a fleet (`config/fleets/NAME.yaml`): exports `FLEET_CONFIG_FILE`, derives `NUM_ROBOTS`, selects the Isaac fleet spawner, and (for heterogeneous fleets) includes the generated per-robot services. See [Fleets](../../fleets.md) |

Also supported: `--build` (build images before starting), `--recreate` (recreate containers), and `--env-file FILE` (layer an extra env file on top of the root `.env`).

## Command Details

### Installation

The `install` command sets up the necessary dependencies for AirStack development:

```bash
airstack install [options]
```

Options:

- `--force`: Force reinstallation of components
- `--no-docker`: Skip Docker installation

### Setup

The `setup` command configures your environment for AirStack development:

```bash
airstack setup [options]
```

Options:

- `--no-shell`: Skip adding to shell profile
- `--no-config`: Skip configuration tasks

### Container Management

The AirStack CLI provides several commands for managing Docker containers:

```bash
# Start services
airstack up [service...]
airstack up robot-desktop  # start only the robot service
airstack up isaac-sim  # start only the Isaac Sim service
airstack up gcs  # start only the Ground Control Station service
airstack up docs # start only the documentation service

# Stop services
airstack down [service...]

# Show container status
airstack status

# Connect to a container shell, supports partial name matching
# (real names look like airstack-robot-desktop-1, airstack-gcs-1, ...)
airstack connect robot-desktop-1

# View container logs
airstack logs robot-desktop-1
```

### Configuration Commands

```bash
# Run all configuration tasks
airstack config

# Configure Isaac Sim
airstack config:isaac-sim

# Configure Nucleus
airstack config:nucleus

# Configure Git hooks
airstack config:git-hooks
```
