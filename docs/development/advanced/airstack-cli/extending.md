# Advanced: Extending the AirStack CLI

The AirStack CLI tool is designed to be easily extensible through modules. This guide explains how to create new modules and add commands to the AirStack CLI.

## Module System Overview

The AirStack CLI uses a modular architecture where:

- Each module is a separate Bash script in the `.airstack/modules/` directory
- Modules can define their own commands and functions
- Modules are automatically loaded when the AirStack CLI starts
- Each module registers its commands with the main script

## Creating a New Module

To create a new module:

1. Create a new Bash script in the `.airstack/modules/` directory
2. Define your command functions
3. Create a registration function to register your commands
4. Make the script executable

### Module Template

Here's a template for creating a new module:

```bash
#!/usr/bin/env bash

# mymodule.sh - Description of your module
# This module provides commands for...

# Function to implement your command
function cmd_mymodule_action {
    log_info "Performing action..."
    
    # Your command implementation here
    
    log_info "Action completed!"
    return 0
}

# Dispatcher: routes `airstack mymodule <subcommand>` to the right function.
function cmd_mymodule_dispatch {
    local sub="${1:-help}"
    if [ $# -gt 0 ]; then shift; fi
    case "$sub" in
        action) cmd_mymodule_action "$@" ;;
        help|-h|--help) print_command_help mymodule ;;
        *)
            log_error "Unknown mymodule subcommand: '$sub'"
            print_command_help mymodule
            return 1
            ;;
    esac
}

# Register commands from this module
function register_mymodule_commands {
    COMMANDS["mymodule"]="cmd_mymodule_dispatch"
    
    # Add command help
    COMMAND_HELP["mymodule"]="Description of your command group: action (see 'airstack help mymodule')"
}
```

Save this file as `.airstack/modules/mymodule.sh` and make it executable:

```bash
chmod +x .airstack/modules/mymodule.sh
```

## Module Naming Conventions

To ensure consistency and avoid conflicts:

- Module filenames should use lowercase and end with `.sh`
- Command functions should be prefixed with `cmd_`
- Registration functions should be named `register_<modulename>_commands`
- A module registers ONE top-level command (its group name); subcommands are
  routed by a `cmd_<modulename>_dispatch` function, invoked as
  `airstack <module> <subcommand>` (see the `fleet`, `osmo`, `config`, and
  `images` groups for reference)
- To rename or deprecate a command spelling without breaking scripts, register
  the old name as a forwarding alias and set `COMMAND_HIDDEN["<old-name>"]=1`
  so it keeps working but no longer appears in `airstack help` /
  `airstack commands`

## Available Utilities

When creating modules, you can use several utility functions provided by the main script:

### Logging Functions

```bash
log_info "Informational message"    # Green text
log_warn "Warning message"          # Yellow text
log_error "Error message"           # Red text
log_debug "Debug message"           # Blue text
```

### Environment Variables

```bash
$PROJECT_ROOT    # Root directory of the AirStack project
$SCRIPT_DIR      # Directory containing the main script
$MODULES_DIR     # Directory containing the modules
```

## Example: Creating a Custom Module

Let's create a simple module that provides a command to check system resources:

```bash
#!/usr/bin/env bash

# sysinfo.sh - System information commands for AirStack
# This module provides commands for checking system resources

# Function to check system resources
function cmd_sysinfo_resources {
    log_info "Checking system resources..."
    
    echo "CPU Usage:"
    top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1"%"}'
    
    echo "Memory Usage:"
    free -m | awk 'NR==2{printf "%.2f%%\n", $3*100/$2}'
    
    echo "Disk Usage:"
    df -h | grep -E '^/dev/' | awk '{print $1 " " $5}'
    
    log_info "Resource check completed!"
    return 0
}

# Dispatcher for the `sysinfo` command group.
function cmd_sysinfo_dispatch {
    local sub="${1:-help}"
    if [ $# -gt 0 ]; then shift; fi
    case "$sub" in
        resources) cmd_sysinfo_resources "$@" ;;
        help|-h|--help) print_command_help sysinfo ;;
        *)
            log_error "Unknown sysinfo subcommand: '$sub'"
            print_command_help sysinfo
            return 1
            ;;
    esac
}

# Register commands from this module
function register_sysinfo_commands {
    COMMANDS["sysinfo"]="cmd_sysinfo_dispatch"
    
    # Add command help
    COMMAND_HELP["sysinfo"]="System information: resources (see 'airstack help sysinfo')"
}
```

Save this file as `.airstack/modules/sysinfo.sh` and make it executable:

```bash
chmod +x .airstack/modules/sysinfo.sh
```

Now you can use your new command:

```bash
./airstack.sh sysinfo resources
```

## Best Practices for Module Development

When developing modules for the AirStack CLI, follow these best practices:

1. **Keep modules focused**: Each module should have a specific purpose
2. **Use descriptive names**: Command names should clearly indicate their function
3. **Provide helpful error messages**: Use the logging functions to provide clear feedback
4. **Include help text**: Always register help text for your commands
5. **Handle errors gracefully**: Check for errors and return appropriate exit codes
6. **Use consistent formatting**: Follow the style of the existing modules
7. **Document your module**: Include comments explaining what your module does

## Real-World Example: Config Module

The config module (`.airstack/modules/config.sh`) provides commands for configuring the AirStack environment:

```bash
#!/usr/bin/env bash

# config.sh - Configuration-related commands for AirStack
# This module provides commands for configuring the AirStack environment

# Function to configure Isaac Sim settings
function cmd_config_isaac_sim {
    log_info "Configuring Isaac Sim..."

    # Implementation details...

    return 0
}

# Function to configure AirLab Omniverse Nucleus Server login
function cmd_config_nucleus {
    log_info "Configuring Nucleus..."

    # Implementation details...

    return 0
}

# Function to set up Git hooks
function cmd_config_git_hooks {
    log_info "Configuring Git hooks..."

    # Implementation details...

    return 0
}

# Function to run all configuration tasks
function cmd_config_all {
    cmd_config_isaac_sim
    cmd_config_nucleus
    cmd_config_git_hooks
}

# Dispatcher for the `config` command group. Bare `airstack config` runs
# all configuration tasks.
function cmd_config_dispatch {
    local sub="${1:-all}"
    if [ $# -gt 0 ]; then shift; fi
    case "$sub" in
        all)       cmd_config_all "$@" ;;
        isaac-sim) cmd_config_isaac_sim "$@" ;;
        nucleus)   cmd_config_nucleus "$@" ;;
        git-hooks) cmd_config_git_hooks "$@" ;;
        help|-h|--help) print_command_help config ;;
        *)
            log_error "Unknown config subcommand: '$sub'"
            print_command_help config
            return 1
            ;;
    esac
}

# Register commands from this module
function register_config_commands {
    COMMANDS["config"]="cmd_config_dispatch"
    COMMAND_HELP["config"]="Configure AirStack: all (default)|isaac-sim|nucleus|git-hooks"
}
```

This module provides one command group for configuring the environment:
`airstack config` (all tasks), `airstack config isaac-sim`,
`airstack config nucleus`, and `airstack config git-hooks`.
