# AirStack CLI Tool

The `airstack` command-line tool provides a unified interface for common development tasks in the AirStack project, including setup, installation, and container management. It uses a containerized docker-compose approach to ensure all developers use the same version (v5.0.0+) regardless of their host machine's Docker installation.

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

### Requirements

**Minimal requirements:**
- Docker installed (any recent version with socket at `/var/run/docker.sock`)
- The Docker daemon must be running

**NOT required:**
- Specific docker-compose version on host
- docker-compose CLI on host (container provides it)

**Note:** The `airstack install` command will install Docker Engine and NVIDIA Container Toolkit, but will **not** install docker-compose on the host. Docker Compose runs entirely in the containerized CLI environment.

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

### First Time Setup

When you run any airstack command that requires docker-compose for the first time, the CLI container will auto-build:
```bash
./airstack.sh up
```

The CLI container includes a pinned version of docker-compose and all necessary tools.

## Core Commands

- `install`: Install dependencies (Docker Engine, NVIDIA Container Toolkit, etc.)
- `setup`: Configure AirStack settings and add to shell profile
- `up [service]`: Start services using containerized Docker Compose
- `stop [service]`: Stop services
- `down`: Stop and remove containers
- `connect [container]`: Connect to a running container (supports partial name matching)
- `status`: Show status of all containers
- `logs [container]`: View logs for a container (supports partial name matching)
- `rebuild-cli`: Rebuild the CLI container with latest docker-compose version

### Container Name Matching

The `connect` and `logs` commands support partial name matching for container names. This means you can:

1. Use just a portion of the container name (e.g., `airstack connect robot` will match `airstack-robot-1`)
2. If multiple containers match, you'll be shown a list and prompted to select one
3. The matching is case-insensitive and supports fuzzy matching

Example:
```bash
$ airstack connect robot
[WARN] Multiple containers match 'robot'. Please be more specific or select from the list below:
NUM     CONTAINER NAME    STATUS
1       airstack-robot-1  Up 2 hours
2       airstack-robot-2  Up 2 hours

Options:
  1. Enter a number to select a container
  2. Type 'q' to quit
  3. Press Ctrl+C to cancel and try again with a more specific name

Your selection: 1
[INFO] Connecting to container: airstack-robot-1
[INFO] Tip: Next time, you can directly use 'airstack connect airstack-robot-1' for this container
```

### Environment Variable Overrides

You can override any `.env` variable on the command line:

```bash
# Override single variable
ISAAC_SIM_USE_STANDALONE=true ./airstack.sh up isaac-sim robot

# Override multiple variables
NUM_ROBOTS=3 AUTOLAUNCH=false ./airstack.sh up

# Works with any .env variable
DOCKER_IMAGE_TAG=dev-latest ./airstack.sh up
```

The wrapper automatically:
1. Reads all variable names from `.env`
2. Passes them from your current shell environment into the container
3. Command-line overrides naturally take precedence

## Development Commands

- `test`: Run tests
- `docs`: Build documentation
- ```
    log_info "Running my command..."
       # Your command implementation here
   }

   # Register commands from this module
   function register_mymodule_commands {
       COMMANDS["mycommand"]="cmd_mymodule_mycommand"
       COMMAND_HELP["mycommand"]="Description of my command"
   }
   ```

3. Make the module executable:
   ```bash
   chmod +x .airstack/modules/mymodule.sh
   ```

Your new command will be automatically loaded and available as `airstack mycommand`.

### Available Helper Functions

When creating modules, you can use these helper functions:

- `log_info "message"`: Print an info message
- `log_warn "message"`: Print a warning message
- `log_error "message"`: Print an error message
- `check_docker`: Check if Docker is installed and running

## Configuration

The `airstack` tool stores its configuration in `~/.airstack.conf`. This file is created when you run `airstack setup`.

## lint`: Lint code
- `format`: Format code

## Extending the Tool

The `airstack` tool is designed to be easily extensible. You can add new commands by creating module files in the `.airstack/modules/` directory.

### Creating a New Module

1. Create a new `.sh` file in the `.airstack/modules/` directory:
   ```bash
   touch .airstack/modules/mymodule.sh
   ```

2. Add your command functions and register them:
   ```bash
   #!/usr/bin/env bash

   # Function to implement your command
   function cmd_mymodule_mycommand {
   Containerized Docker Compose Implementation

The AirStack CLI uses a containerized approach for docker-compose to ensure consistency across all development environments.

### How It Works

The solution uses **Docker socket mounting** (not Docker-in-Docker):
- The `airstack-cli` container includes a pinned version of docker-compose (v5.0.0+)
- It mounts the host's Docker socket (`/var/run/docker.sock`)
- Commands run in the container but control the host's Docker daemon
- Containers spawned by docker-compose run as siblings on the host, not nested

### Key Files

1. **Dockerfile.airstack-cli**
   - Defines the containerized CLI environment
   - Pins docker-compose to a specific version
   - Includes bash, curl, git for full functionality

2. **airstack.sh**
   - `ensure_cli_container()` - builds CLI image if needed
   - `run_docker_compose()` - wrapper for containerized compose
   - `cmd_rebuild_cli()` - rebuild CLI container with new version

3. **.env**
   - Used for docker-compose variable interpolation
   - Automatically mounted into the CLI container with project directory

### Customization

#### Changing Docker Compose Version

Edit `Dockerfile.airstack-cli`:

```dockerfile
ARG COMPOSE_VERSION=5.1.0  # Change this version
```

Then rebuild:
```bash
./airstack.sh rebuild-cli
```

#### Adding Tools to the CLI

Add packages to the Dockerfile:

```dockerfile
RUN apk add --no-cache \
    bash \
    curl \
    git \
    your-tool-here
```

### Migration from Direct Docker Compose

If you have existing containers:

1. **No action needed** - containers will continue running
2. New `up` commands use containerized compose
3. Existing `docker compose` commands on host still work
4. The `.env` file works identically

### Advanced: How the Wrapper Works

The `run_docker_compose()` function:

1. **Reads `.env` file** to extract all variable names
2. **Passes them from current environment** using `-e VARNAME` flags
3. **Mounts necessary resources** and executes docker-compose

```bash
docker run --rm -i \
    -v /var/run/docker.sock:/var/run/docker.sock \  # Host Docker control
    -v "$PROJECT_ROOT:$PROJECT_ROOT" \               # Mount project
    -v /tmp/.X11-unix:/tmp/.X11-unix \               # X11 socket for GUI
    -w "$PROJECT_ROOT" \                              # Set working dir
    -e USER_ID="$(id -u)" \                          # Pass user ID
    -e GROUP_ID="$(id -g)" \                         # Pass group ID
    -e DISPLAY="$DISPLAY" \                          # X11 display for GUI
    -e PROJECT_NAME \                                 # All .env variables...
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

**Note:** The `airstack install` command will install Docker Engine and NVIDIA Container Toolkit, but will **not** install docker-compose on the host. Docker Compose runs entirely in the containerized CLI environment.

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

### GUI windows not displaying
```bash
# Check DISPLAY is set on host
echo $DISPLAY
# Should show something like ":0" or ":1"

# Ensure xhost allows connections
xhost +
# Should show: "access control disabled, clients can connect from any host"

# Check your docker-compose.yaml has X11 config
# It should include:
environment:
  - DISPLAY=${DISPLAY}
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix

# Verify DISPLAY is passed through
./airstack.sh up
# The containerized compose will have DISPLAY set
```

### Environment variable overrides not working
```bash
# Verify the variable is defined in .env
grep ISAAC_SIM_USE_STANDALONE .env
# Should show: ISAAC_SIM_USE_STANDALONE="false"

# Check if the override is being set
ISAAC_SIM_USE_STANDALONE=true env | grep ISAAC_SIM_USE_STANDALONE
# Should show: ISAAC_SIM_USE_STANDALONE=true

# The variable must be defined in .env to be passed through
# If you need a new variable, add it to .env first (even with a default value)

# Debug: Check what's passed to the container
# Add this temporarily to run_docker_compose:
# echo "Passing variables: ${env_args[@]}"
```

## Benefits

The containerized approach provides:

1. **Consistency**: All developers use identical docker-compose version
2. **Isolation**: No host environment interference
3. **Simplicity**: No version checking or installation logic needed
4. **Maintainability**: Single Dockerfile defines tooling
5. **Compatibility**: Works on any Docker-capable host
6. **Flexibility**: Easy to update compose version or add tools

## License

This tool is part of the AirStack project and is subject to the same license terms.