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

## Core Commands

The AirStack CLI includes several built-in commands:

| Command | Description |
|---------|-------------|
| `install` | Install dependencies (Docker Engine, Docker Compose, etc.) |
| `setup` | Configure AirStack settings and add to shell profile |
| `up` | Start services using Docker Compose |
| `down` | Stop services |
| `connect` | Connect to a running container (supports partial name matching) |
| `status` | Show status of all containers |
| `logs` | View logs for a container (supports partial name matching) |
| `help` | Show help information |

### Installation

The `install` command sets up the necessary dependencies for AirStack development:

```bash
airstack install [options]
```

Options:

- `--force`: Force reinstallation of components
- `--no-docker`: Skip Docker installation
- `--with-wintak`: Install WinTAK VirtualBox environment

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
airstack connect <container_name>

# View container logs
airstack logs <container_name>
```

## Module-Specific Commands

In addition to the core commands, the AirStack CLI includes several module-specific commands:

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

### WinTAK Commands

```bash
# Install WinTAK
airstack wintak:install

# Start WinTAK VM
airstack wintak:start

# Stop WinTAK VM
airstack wintak:stop
```
