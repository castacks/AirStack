# AirStack CLI Tool

The AirStack CLI tool (`airstack.sh`) provides a unified interface for common development tasks in the AirStack project, including setup, installation, and container management. It simplifies the development workflow and ensures consistency across different environments.

## Overview

The AirStack CLI tool is designed to be:

- **Modular**: Commands are organized into modules that can be easily extended
- **Consistent**: Provides a unified interface for all AirStack-related tasks
- **Helpful**: Includes detailed help text and error messages
- **Extensible**: New commands can be added without modifying the core script

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
./airstack.sh install [options]
```

Options:
- `--force`: Force reinstallation of components
- `--no-docker`: Skip Docker installation
- `--with-wintak`: Install WinTAK VirtualBox environment

### Setup

The `setup` command configures your environment for AirStack development:

```bash
./airstack.sh setup [options]
```

Options:
- `--no-shell`: Skip adding to shell profile
- `--no-config`: Skip configuration tasks

### Container Management

The AirStack CLI provides several commands for managing Docker containers:

```bash
# Start services
./airstack.sh up [service...]

# Stop services
./airstack.sh down [service...]

# Show container status
./airstack.sh status

# Connect to a container shell
./airstack.sh connect <container_name>

# View container logs
./airstack.sh logs <container_name>
```

## Module-Specific Commands

In addition to the core commands, the AirStack CLI includes several module-specific commands:

### Configuration Commands

```bash
# Run all configuration tasks
./airstack.sh config

# Configure Isaac Sim
./airstack.sh config:isaac-sim

# Configure Nucleus
./airstack.sh config:nucleus

# Configure Git hooks
./airstack.sh config:git-hooks
```

### WinTAK Commands

```bash
# Install WinTAK
./airstack.sh wintak:install

# Start WinTAK VM
./airstack.sh wintak:start

# Stop WinTAK VM
./airstack.sh wintak:stop
```

## Adding to Shell Profile

For convenience, you can add the AirStack CLI to your shell profile to make it available from any directory:

```bash
./airstack.sh setup
```

This will add the necessary aliases to your shell profile (`~/.bashrc`, `~/.zshrc`, etc.) so you can use the `airstack` command from any directory.