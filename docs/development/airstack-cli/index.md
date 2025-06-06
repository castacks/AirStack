# AirStack CLI Tool

The AirStack CLI tool (`airstack.sh`) provides a unified interface for common development tasks in the AirStack project, including setup, installation, and container management. It simplifies the development workflow and ensures consistency across different environments.

## Overview

The AirStack CLI tool is designed to be:

- **Modular**: Commands are organized into modules that can be easily extended
- **Consistent**: Provides a unified interface for all AirStack-related tasks
- **Helpful**: Includes detailed help text and error messages
- **Extensible**: New commands can be added without modifying the core script

At its core, `airstack.sh` is simply a light wrapper around [docker compose](https://docs.docker.com/compose/), providing additional functionality and convenience for AirStack development.

## Basic Usage

```bash
./airstack.sh <command> [options]
```

To see all available commands:

```bash
./airstack.sh commands
```

To get help for a specific command:

```bash
./airstack.sh help <command>
```

## Adding to Shell Profile

For convenience, you can add the AirStack CLI to your shell profile to make it available from any directory:

```bash
./airstack.sh setup
# now you can use `airstack` instead of `./airstack.sh`
airstack commands
```

This will add the necessary aliases to your shell profile (`~/.bashrc`, `~/.zshrc`, etc.) so you can use the `airstack` command from any directory.

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
airstack up robot  # start only the robot service
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
