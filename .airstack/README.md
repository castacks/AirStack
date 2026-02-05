# AirStack CLI Tool

The `airstack` command-line tool provides a unified interface for common development tasks in the AirStack project, including setup, installation, and container management.

## Architecture Overview

The AirStack CLI uses a **containerized docker-compose** approach to ensure all developers use the same version (v5.0.0+) regardless of their host machine's Docker installation. This solves version compatibility issues across different developer environments.

### How It Works

The solution uses **Docker socket mounting** (not Docker-in-Docker):
- The `airstack-cli` container includes a pinned version of docker-compose
- It mounts the host's Docker socket (`/var/run/docker.sock`)
- Commands run in the container but control the host's Docker daemon
- Containers spawned by docker-compose run as siblings on the host, not nested

This ensures consistency across all development environments while maintaining simplicity and compatibility.

## Installation

The `airstack` tool is included in the AirStack repository. To set it up:

1. Clone the AirStack repository:
   ```bash
   git clone https://github.com/castacks/AirStack.git
   cd AirStack
   ```

2. Run the setup command:
   ```bash
   ./airstack setup
   ```

This will add the `airstack` command to your PATH by modifying your shell profile (`.bashrc` or `.zshrc`).

**On first use**, the CLI container will automatically build itself. This may take a few minutes but only happens once.

## Basic Usage

```bash
airstack <command> [options]
```

To see all available commands:
```bash
airstack commands
```

To get help for a specific command:
```bash
airstack help <command>
```

## Core Commands

- `install`: Install dependencies (Docker Engine, NVIDIA Container Toolkit)
- `setup`: Configure AirStack settings and add to shell profile
- `up [service]`: Start services using Docker Compose
- `stop [service]`: Stop services
- `connect [container]`: Connect to a running container (supports partial name matching)
- `status`: Show status of all containers
- `logs [container]`: View logs for a container (supports partial name matching)

### Container Name Matching

The `connect` and `logs` commands support partial name matching for container names. This means you can:

1. Use just a portion of the container name (e.g., `airstack connect web` will match `airstack_web_1`)
2. If multiple containers match, you'll be shown a list and prompted to select one
3. The matching is case-insensitive and supports fuzzy matching

Example:
```bash
$ airstack connect web
[WARN] Multiple containers match 'web'. Please be more specific or select from the list below:
NUM     CONTAINER NAME  IMAGE   STATUS
1       airstack_web_1  nginx:latest    Up 2 hours
2       airstack_webapi_1       node:14 Up 2 hours

Options:
  1. Enter a number to select a container
  2. Type 'q' to quit
  3. Press Ctrl+C to cancel and try again with a more specific name

Your selection: 1
[INFO] Connecting to container: airstack_web_1
[INFO] Tip: Next time, you can directly use 'airstack connect airstack_web_1' for this container
```

## Development Commands

- `test`: Run tests
- `docs`: Build documen
- `rebuild-cli`: Rebuild the CLI container (useful after updating docker-compose version)

## Developer Requirements

**Minimal requirements:**
- Docker installed (any recent version with socket at `/var/run/docker.sock`)
- The Docker daemon must be running

**NOT required:**
- Specific docker-compose version on host
- docker-compose CLI on host (container provides it)

## Customization

### Changing Docker Compose Version

To update the docker-compose version used by the CLI, edit `Dockerfile.airstack-cli`:

```dockerfile
ARG COMPOSE_VERSION=5.1.0  # Change this version
```

Then rebuild the CLI container:
```bash
airstack rebuild-cli
```

### Adding Tools to the CLI

You can add additional packages to the CLI environment by modifying `Dockerfile.airstack-cli`:

```dockerfile
RUN apk add --no-cache \
    bash \
    curl \
    git \
    your-tool-here
```

After making changes, run `airstack rebuild-cli` to apply them.tation
- `lint`: Lint code
- `format`: Format code

## Extending the Tool

The `airstack` tool is designed to be easily extensible. You can add new commands by creating module files in the `.airstack/modules/` directory.

###Migration from Direct Docker Compose

If you have existing containers running from a previous version
## Customization

### Changing Docker Compose Version

Edit `Dockerfile.airstack-cli`:

```dockerfile
ARG COMPOSE_VERSION=5.1.0  # Change this version
```

Then rebuild:
```bash
./airstack.sh rebuild-cli
```

### Adding Tools to the CLI

Add packages to the Dockerfile:

```dockerfile
RUN apk add --no-cache \
    bash \
    curl \
    git \
    your-tool-here
```

## Developer Requirements

**Minimal requirements:**
- Docker installed (any recent version with socket at `/var/run/docker.sock`)
- The Docker daemon must be running

**NOT required:**
- Specific docker-compose version on host
- docker-compose CLI on host (container provides it)

## Migration from Direct Docker Compose

If you have existing containers:

1. **No action needed** - containers will continue running
2. New `up` commands use containerized compose
3. Existing `docker compose` commands on host still work
4. The `.env` file works identically

## Troubleshooting

### CLI container won't build
```bash
# Ensure Dockerfile.airstack-cli exists
ls -la Dockerfile.airstack-cli

# Try building manually
docker build -f Dockerfile.airstack-cli -t airstack-cli:latest .
```

### Docker socket permission denied
```bash
# Check Docker group membership
groups | grep docker

# If not in docker group:
sudo usermod -aG docker $USER
# Then log out and back in
```

### Environment variables not propagating
- The `.env` file must be in the project root
- The entire project directory is mounted into the container
- `USER_ID` and `GROUP_ID` are automatically set from host

## Advanced: How the Wrapper Works

The `run_docker_compose()` function:

```bash
docker run --rm -i \
    -v /var/run/docker.sock:/var/run/docker.sock \  # Host Docker control
    -v "$PROJECT_ROOT:$PROJECT_ROOT" \               # Mount project
    -w "$PROJECT_ROOT" \                              # Set working dir
    -e USER_ID="$(id -u)" \                          # Pass user ID
    -e GROUP_ID="$(id -g)" \                         # Pass group ID
    --network host \                                  # Host networking
    airstack-cli:latest \
    docker compose "$@"                               # Run compose command
```

This ensures:
- Compose commands execute with host's Docker daemon
- Project files are accessible at same paths
- User/group IDs match for file permissions
- Network configuration matches host

## Benefits

1. **Consistency**: All developers use identical docker-compose version
2. **Isolation**: No host environment interference
3. **Simplicity**: No version checking or installation logic needed
4. **Maintainability**: Single Dockerfile defines tooling
5. **Compatibility**: Works on any Docker-capable host
6. **Flexibility**: Easy to update compose version or add tools

## License

This tool is part of the AirStack project and is subject to the same license terms.Configuration

The `airstack` tool stores its configuration in `~/.airstack.conf`. This file is created when you run `airstack setup`.

## Advanced: Technical Detail# Implementation Files

The containerized CLI implementation consists of:

1. **Dockerfile.airstack-cli**
   - Defines the containerized CLI environment
   - Pins docker-compose to v5.0.0
   - Includes bash, curl, git for full functionality

2. **airstack.sh** (modified)
   - Added `ensure_cli_container()` - builds CLI image if needed
   - Added `run_docker_compose()` - wrapper for containerized compose
   - Modified `cmd_up()` and `cmd_down()` to use the wrapper
   - Added `cmd_rebuild_cli()` - rebuild CLI container with new version
   - Removed docker-compose version check (now guaranteed by container)

3. **.env** (unchanged)
   - Still used for docker-compose variable interpolation
   - Automatically mounted into the CLI container with project directory

### Benefits of This Approach