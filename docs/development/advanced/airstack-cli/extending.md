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

# Register commands from this module
function register_mymodule_commands {
    COMMANDS["mymodule:action"]="cmd_mymodule_action"
    
    # Add command help
    COMMAND_HELP["mymodule:action"]="Description of your command"
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
- Command names should use the format `<module>:<command>`

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

# Register commands from this module
function register_sysinfo_commands {
    COMMANDS["sysinfo:resources"]="cmd_sysinfo_resources"
    
    # Add command help
    COMMAND_HELP["sysinfo:resources"]="Check system resources (CPU, memory, disk)"
}
```

Save this file as `.airstack/modules/sysinfo.sh` and make it executable:

```bash
chmod +x .airstack/modules/sysinfo.sh
```

Now you can use your new command:

```bash
./airstack.sh sysinfo:resources
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

## Real-World Example: WinTAK Module

The WinTAK module provides commands for installing and managing the WinTAK virtual machine:

```bash
#!/usr/bin/env bash

# wintak.sh - WINTAK-related commands for AirStack
# This module provides commands for installing and managing WINTAK

# Function to install WINTAK
function cmd_wintak_install {
    log_info "Installing WINTAK..."
    
    # Implementation details...
    
    return 0
}

# Function to start WINTAK
function cmd_wintak_start {
    log_info "Starting WinTAK..."
    
    # Implementation details...
    
    return 0
}

# Function to stop WINTAK
function cmd_wintak_stop {
    log_info "Stopping WinTAK..."
    
    # Implementation details...
    
    return 0
}

# Register commands from this module
function register_wintak_commands {
    COMMANDS["wintak:install"]="cmd_wintak_install"
    COMMANDS["wintak:start"]="cmd_wintak_start"
    COMMANDS["wintak:stop"]="cmd_wintak_stop"
    
    # Add command help
    COMMAND_HELP["wintak:install"]="Install WinTAK VirtualBox environment"
    COMMAND_HELP["wintak:start"]="Start WinTAK virtual machine"
    COMMAND_HELP["wintak:stop"]="Stop WinTAK virtual machine"
}
```

This module provides three commands for managing WinTAK: `wintak:install`, `wintak:start`, and `wintak:stop`.