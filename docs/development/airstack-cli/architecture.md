# Advanced: AirStack CLI Architecture

This document describes the architecture of the AirStack CLI tool, explaining how it's organized and how the different components work together.

## Overview

The AirStack CLI tool is built as a modular Bash application with the following components:

- **Main Script (`airstack.sh`)**: The entry point that handles command dispatching and provides core utilities
- **Command Modules**: Separate scripts that implement specific functionality
- **Command Registry**: A system for registering and discovering commands
- **Help System**: A system for providing help text for commands

## Directory Structure

```
AirStack/
├── airstack.sh              # Main script
├── .airstack/               # AirStack configuration directory
│   └── modules/             # Command modules directory
│       ├── config.sh        # Configuration module
│       ├── wintak.sh        # WinTAK module
│       └── ...              # Other modules
```

## Main Script

The main script (`airstack.sh`) serves as the entry point for the AirStack CLI tool. It:

1. Sets up the environment
2. Loads command modules
3. Registers built-in commands
4. Dispatches commands to the appropriate handler

### Command Dispatching

The main script uses a command dispatcher pattern:

```bash
# Main command dispatcher
if [ $# -eq 0 ]; then
    print_usage
    exit 0
fi

command="$1"
shift

# Check if command exists
if [[ -n "${COMMANDS[$command]}" ]]; then
    # Execute the command function
    ${COMMANDS[$command]} "$@"
elif [ "$command" == "commands" ]; then
    # Special case for listing all commands
    list_commands
else
    log_error "Unknown command: $command"
    print_usage
    exit 1
fi
```

This pattern allows for dynamic command registration and execution.

## Command Registry

Commands are registered in associative arrays:

```bash
# Arrays to store available commands and their help text
declare -A COMMANDS
declare -A COMMAND_HELP
```

Each module registers its commands by adding entries to these arrays:

```bash
function register_mymodule_commands {
    COMMANDS["mymodule:action"]="cmd_mymodule_action"
    COMMAND_HELP["mymodule:action"]="Description of your command"
}
```

## Module Loading

Modules are loaded dynamically from the `.airstack/modules/` directory:

```bash
function load_command_modules {
    # Skip if modules directory doesn't exist
    if [ ! -d "$MODULES_DIR" ]; then
        return
    fi
    
    # Load all .sh files in the modules directory
    for module in "$MODULES_DIR"/*.sh; do
        if [ -f "$module" ]; then
            # Source the module file
            source "$module"
            
            # Extract module name from filename
            module_name=$(basename "$module" .sh)
            
            # Register the module's commands if it has a register_commands function
            if declare -f "register_${module_name}_commands" > /dev/null; then
                "register_${module_name}_commands"
            fi
        fi
    done
}
```

This allows for easy extension of the AirStack CLI without modifying the core script.

## Command Implementation

Commands are implemented as Bash functions with a consistent naming pattern:

```bash
function cmd_<module>_<command> {
    # Command implementation
}
```

For example:

```bash
function cmd_wintak_install {
    log_info "Installing WINTAK..."
    # Implementation details...
    return 0
}
```

## Help System

The help system provides information about available commands:

```bash
function print_command_help {
    local command="$1"
    
    if [[ -n "${COMMAND_HELP[$command]}" ]]; then
        echo -e "airstack $command - ${COMMAND_HELP[$command]}"
        echo ""
        
        # Command-specific usage and options
        case "$command" in
            install)
                echo "Usage: airstack install [options]"
                echo ""
                echo "Options:"
                echo "  --force       Force reinstallation of components"
                echo "  --no-docker   Skip Docker installation"
                echo "  --with-wintak Install WinTAK VirtualBox environment"
                ;;
            # Other commands...
        esac
    else
        log_error "Unknown command: $command"
        print_usage
    fi
}
```

## Logging System

The AirStack CLI includes a simple logging system for consistent output formatting:

```bash
function log_info {
    echo -e "${GREEN}[INFO]${NC} $1"
}

function log_warn {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

function log_error {
    echo -e "${RED}[ERROR]${NC} $1"
}

function log_debug {
    echo -e "${BLUE}[DEBUG]${NC} $1"
}
```

## Command Execution Flow

When a user runs a command, the following sequence occurs:

1. The main script parses the command and arguments
2. It looks up the command in the `COMMANDS` array
3. If found, it executes the corresponding function
4. If not found, it displays an error message and usage information

## Design Principles

The AirStack CLI tool follows several design principles:

1. **Modularity**: Functionality is separated into modules
2. **Extensibility**: New commands can be added without modifying the core script
3. **Consistency**: Commands follow a consistent naming and implementation pattern
4. **Discoverability**: Help text and command listing make it easy to discover functionality
5. **Robustness**: Error handling and logging provide clear feedback

## Future Enhancements

Potential future enhancements for the AirStack CLI architecture include:

1. **Command Aliases**: Allow commands to have multiple names
2. **Command Grouping**: Group related commands for better organization
3. **Tab Completion**: Add support for tab completion in shells
4. **Plugin System**: Allow for more dynamic loading of plugins
5. **Configuration System**: Add support for user-specific configuration