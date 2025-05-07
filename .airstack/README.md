# AirStack CLI Tool

The `airstack` command-line tool provides a unified interface for common development tasks in the AirStack project, including setup, installation, and container management.

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

- `install`: Install dependencies (Docker Engine, Docker Compose, etc.)
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
- `docs`: Build documentation
- `lint`: Lint code
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

## License

This tool is part of the AirStack project and is subject to the same license terms.