#!/usr/bin/env bash

# airstack - A convenience tool for AirStack development
# 
# This script provides a unified interface for common development tasks
# in the AirStack project, including setup, installation, and container management.

set -e

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

# Directory for command modules
MODULES_DIR="$PROJECT_ROOT/.airstack/modules"

# Create modules directory if it doesn't exist
mkdir -p "$MODULES_DIR"

# Color codes for output formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print usage information
function print_usage {
    echo -e "${BLUE}AirStack Development Tool${NC}"
    echo ""
    echo "Usage: airstack <command> [options]"
    echo ""
    echo "Available commands:"
    
    # Sort commands alphabetically
    local sorted_commands=($(echo "${!COMMANDS[@]}" | tr ' ' '\n' | sort))
    
    # Calculate the longest command name for padding
    local max_len=0
    for cmd in "${sorted_commands[@]}"; do
        if [ ${#cmd} -gt $max_len ]; then
            max_len=${#cmd}
        fi
    done
    
    # Add 4 spaces of padding
    max_len=$((max_len + 4))
    
    # Print each command with its help text
    for cmd in "${sorted_commands[@]}"; do
        # Skip hidden commands (those starting with _)
        if [[ "$cmd" == _* ]]; then
            continue
        fi
        
        # Get help text or use a default
        local help_text="${COMMAND_HELP[$cmd]:-No description available}"
        
        # Calculate padding
        local padding=$((max_len - ${#cmd}))
        local pad=$(printf '%*s' "$padding" '')
        
        echo "  $cmd$pad$help_text"
    done
    
    echo ""
    echo "For more information on a command, run: airstack help <command>"
    echo "To see all available commands, run: airstack commands"
}

# Print command-specific help
function print_command_help {
    local command="$1"
    
    # Check if command exists
    if [[ -z "${COMMANDS[$command]}" ]]; then
        log_error "Unknown command: $command"
        print_usage
        return 1
    fi
    
    # Get help text
    local help_text="${COMMAND_HELP[$command]:-No description available}"
    
    # Print command-specific help
    echo -e "${BLUE}airstack $command${NC} - $help_text"
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
        setup)
            echo "Usage: airstack setup [options]"
            echo ""
            echo "Options:"
            echo "  --no-shell    Don't modify shell configuration"
            echo "  --no-config   Skip configuration tasks (Isaac Sim, Nucleus, Git hooks)"
            ;;
        up)
            echo "Usage: airstack up [service_name] [options]"
            echo ""
            echo "Options:"
            echo "  --build       Build images before starting containers"
            echo "  --recreate    Recreate containers even if their configuration and image haven't changed"
            ;;
        connect)
            echo "Usage: airstack connect [container_name] [options]"
            echo ""
            echo "Options:"
            echo "  --command=CMD  Run specific command instead of shell (default: bash)"
            ;;
        down)
            echo "Usage: airstack down [service_name]"
            echo ""
            echo "If no service name is provided, all services will be shutdown."
            ;;
        status)
            echo "Usage: airstack status"
            echo ""
            echo "Shows the status of all running containers."
            ;;
        logs)
            echo "Usage: airstack logs [container_name] [options]"
            echo ""
            echo "View logs for the specified container."
            echo ""
            echo "Options:"
            echo "  --no-follow    Don't follow log output"
            echo "  --tail=N       Show only the last N lines (default: all)"
            ;;
        test)
            echo "Usage: airstack test [options]"
            echo ""
            echo "Options:"
            echo "  --path=PATH    Path to test directory"
            echo "  --filter=PATTERN  Filter tests by pattern"
            ;;
        docs)
            echo "Usage: airstack docs [serve]"
            echo ""
            echo "Build documentation. If 'serve' is specified, also start a local server."
            ;;
        *)
            # For commands without specific help, just show the general help
            echo "Usage: airstack $command [options]"
            echo ""
            echo "For more information, check the documentation or source code."
            ;;
    esac
}

# Log messages with different severity levels
function log_info {
    echo -e "${GREEN}[INFO]${NC} $1"
}

function log_warn {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

function log_error {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

# Check if Docker is installed and running
function check_docker {
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed. Run 'airstack install' first."
        exit 1
    fi

    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running."
        exit 1
    fi
}

# Find container by partial name using regex
function find_container {
    local search_term="$1"
    local containers
    
    # Get list of running containers
    containers=$(docker ps --format "{{.Names}}\t{{.Image}}\t{{.Status}}")
    
    # Find matches using grep
    matches=$(echo "$containers" | grep -i "$search_term" || true)
    match_count=$(echo "$matches" | grep -v "^$" | wc -l)
    
    if [ "$match_count" -eq 0 ]; then
        # Try a more flexible search if exact match fails
        log_warn "No exact matches for '$search_term', trying fuzzy search..."
        matches=$(echo "$containers" | grep -i ".*$search_term.*" || true)
        match_count=$(echo "$matches" | grep -v "^$" | wc -l)
        
        if [ "$match_count" -eq 0 ]; then
            log_error "No running containers match '$search_term'"
            
            # Show available containers as a suggestion
            available=$(docker ps --format "{{.Names}}")
            if [ -n "$available" ]; then
                log_info "Available containers:"
                echo "$available"
            fi
            
            return 1
        fi
    fi
    
    if [ "$match_count" -eq 1 ]; then
        # Extract just the container name (first column)
        container_name=$(echo "$matches" | awk '{print $1}')
        echo "$container_name"
        return 0
    else
        log_warn "Multiple containers match '$search_term'. Please be more specific or select from the list below:"
        # Format the output as a table with numbers
        echo -e "${BLUE}NUM\tCONTAINER NAME\tIMAGE\tSTATUS${NC}"
        echo "$matches" | awk '{print NR "\t" $0}'
        echo ""
        echo "Options:"
        echo "  1. Enter a number to select a container"
        echo "  2. Type 'q' to quit"
        echo "  3. Press Ctrl+C to cancel and try again with a more specific name"
        echo ""
        read -p "Your selection: " selection
        
        if [ "$selection" = "q" ]; then
            log_info "Operation cancelled"
            return 1
        elif [[ "$selection" =~ ^[0-9]+$ ]] && [ "$selection" -gt 0 ] && [ "$selection" -le "$match_count" ]; then
            # Extract just the container name from the selected line
            container_name=$(echo "$matches" | sed -n "${selection}p" | awk '{print $1}')
            echo "$container_name"
            
            # Provide a tip for future use
            log_info "Tip: Next time, you can directly use 'airstack connect $container_name' for this container"
            return 0
        else
            log_error "Invalid selection. Please enter a number between 1 and $match_count, or 'q' to quit."
            
            # Give the user another chance to select
            echo ""
            read -p "Try again (or 'q' to quit): " selection
            
            if [ "$selection" = "q" ]; then
                log_info "Operation cancelled"
                return 1
            elif [[ "$selection" =~ ^[0-9]+$ ]] && [ "$selection" -gt 0 ] && [ "$selection" -le "$match_count" ]; then
                container_name=$(echo "$matches" | sed -n "${selection}p" | awk '{print $1}')
                echo "$container_name"
                return 0
            else
                log_error "Invalid selection again. Please try the command again with a more specific container name."
                return 1
            fi
        fi
    fi
}

# Command implementations
function cmd_install {
    log_info "Installing dependencies..."
    
    # Check for --force flag
    local force=false
    # Check for --no-docker flag
    local install_docker=true
    # Check for --with-wintak flag
    local install_wintak=false
    
    for arg in "$@"; do
        if [ "$arg" == "--force" ]; then
            force=true
        elif [ "$arg" == "--no-docker" ]; then
            install_docker=false
        elif [ "$arg" == "--with-wintak" ]; then
            install_wintak=true
        fi
    done
    
    # Install Docker if needed
    if [ "$install_docker" = true ]; then
        if command -v docker &> /dev/null && [ "$force" = false ]; then
            log_info "Docker is already installed"
        else
            log_info "Installing Docker..."
            
            # Detect OS
            if [ -f /etc/os-release ]; then
                . /etc/os-release
                OS=$ID
            else
                OS=$(uname -s)
            fi
            
            case "$OS" in
                ubuntu|debian)
                    log_info "Detected Debian/Ubuntu system"
                    
                    # Update package index
                    log_info "Updating package index..."
                    sudo apt-get update
                    
                    # Install prerequisites
                    log_info "Installing prerequisites..."
                    sudo apt-get install -y apt-transport-https ca-certificates curl gnupg lsb-release
                    
                    # Add Docker's official GPG key
                    log_info "Adding Docker's GPG key..."
                    curl -fsSL https://download.docker.com/linux/$OS/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
                    
                    # Set up the stable repository
                    log_info "Setting up Docker repository..."
                    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/$OS $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
                    
                    # Update package index again
                    sudo apt-get update
                    
                    # Install Docker Engine
                    log_info "Installing Docker Engine..."
                    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
                    ;;
                    
                fedora|centos|rhel)
                    log_info "Detected Fedora/CentOS/RHEL system"
                    
                    # Install prerequisites
                    log_info "Installing prerequisites..."
                    sudo dnf -y install dnf-plugins-core
                    
                    # Add Docker repository
                    log_info "Setting up Docker repository..."
                    sudo dnf config-manager --add-repo https://download.docker.com/linux/$OS/docker-ce.repo
                    
                    # Install Docker Engine
                    log_info "Installing Docker Engine..."
                    sudo dnf -y install docker-ce docker-ce-cli containerd.io docker-compose-plugin
                    ;;
                    
                darwin)
                    log_info "Detected macOS system"
                    log_info "Please install Docker Desktop for Mac from https://www.docker.com/products/docker-desktop"
                    log_info "After installation, run 'airstack setup' to complete the configuration"
                    ;;
                    
                *)
                    log_error "Unsupported operating system: $OS"
                    log_info "Please install Docker manually from https://docs.docker.com/engine/install/"
                    ;;
            esac
            
            # Start Docker service if not running
            if systemctl status docker &>/dev/null; then
                log_info "Starting Docker service..."
                sudo systemctl enable --now docker
                
                # Add current user to docker group
                log_info "Adding user to docker group..."
                sudo usermod -aG docker "$USER"
                log_info "You may need to log out and back in for group changes to take effect"
            fi
            
            log_info "Docker installation complete"
        fi
        
        # Install Docker Compose if needed
        if ! command -v docker compose &> /dev/null && [ "$force" = false ]; then
            log_info "Installing Docker Compose..."
            
            # Docker Compose is now included with Docker Engine as a plugin
            # For older Docker versions, we'll install the standalone version
            if docker --help | grep -q "compose"; then
                log_info "Docker Compose plugin is already installed"
            else
                log_info "Installing Docker Compose standalone version..."
                COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
                sudo curl -L "https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
                sudo chmod +x /usr/local/bin/docker-compose
                log_info "Docker Compose installation complete"
            fi
        fi
    fi
    
    # Install WINTAK if requested
    if [ "$install_wintak" = true ]; then
        # Check if the wintak module is available
        if declare -f "cmd_wintak_install" > /dev/null; then
            log_info "Installing WINTAK..."
            cmd_wintak_install
        else
            log_error "WINTAK module not loaded. Cannot install WINTAK."
            log_info "Please make sure the wintak.sh module is in the .airstack/modules directory."
        fi
    fi
    
    log_info "Installation complete!"
}

function cmd_setup {
    log_info "Setting up AirStack environment..."
    
    # Check for --no-shell flag
    local modify_shell=true
    local skip_config=false
    
    for arg in "$@"; do
        if [ "$arg" == "--no-shell" ]; then
            modify_shell=false
        elif [ "$arg" == "--no-config" ]; then
            skip_config=true
        fi
    done
    
    # Add to shell profile if requested
    if [ "$modify_shell" = true ]; then
        local shell_profile=""
        
        if [ -f "$HOME/.zshrc" ]; then
            shell_profile="$HOME/.zshrc"
        elif [ -f "$HOME/.bashrc" ]; then
            shell_profile="$HOME/.bashrc"
        else
            log_warn "Could not determine shell profile. Please add the following line manually:"
            echo "alias airstack=\"$PROJECT_ROOT/airstack.sh\""
            return
        fi
        
        if grep -q "# AirStack alias" "$shell_profile"; then
            log_info "'airstack' alias already registered"
        else
            log_info "Adding 'airstack' alias in $shell_profile"
            echo "" >> "$shell_profile"
            echo "# AirStack alias" >> "$shell_profile"
            echo "alias airstack=\"$PROJECT_ROOT/airstack.sh\"" >> "$shell_profile"
            echo "Added to $shell_profile. Please restart your shell or run 'source $shell_profile'. Then you'll be able to use the 'airstack' command from any directory."
        fi
    fi
    
    # Run configuration tasks if not skipped
    if [ "$skip_config" = false ]; then
        # Check if the config module is available
        if declare -f "cmd_config_all" > /dev/null; then
            log_info "Running configuration tasks..."
            cmd_config_all
        else
            log_warn "Configuration module not loaded. Skipping configuration tasks."
        fi
    fi
    
    log_info "Setup complete!"
}

function cmd_up {
    check_docker
    
    # Build docker-compose command
    local cmd="docker compose -f $PROJECT_ROOT/docker-compose.yaml up $@ -d"
    
    eval "$cmd"
    log_info "Services brought up successfully"
}

function cmd_down {
    check_docker
    
    local services=("$@")
    
    # Build docker-compose command
    local cmd="docker compose -f $PROJECT_ROOT/docker-compose.yaml --profile '*' down"
    
    # Add services if specified
    if [ ${#services[@]} -gt 0 ]; then
        cmd="docker compose -f $PROJECT_ROOT/docker-compose.yaml down ${services[*]}"
    fi
    
    log_info "Shutting down services: ${services[*]:-all}"
    eval "$cmd"
    log_info "Services shutdown successfully"
}

function cmd_connect {
    check_docker
    
    if [ $# -eq 0 ]; then
        log_error "Container name required"
        print_command_help "connect"
        
        # Show available containers as a helpful suggestion
        local available=$(docker ps --format "{{.Names}}")
        if [ -n "$available" ]; then
            log_info "Available containers:"
            echo "$available"
        fi
        
        exit 1
    fi
    
    local container_pattern="$1"
    shift
    
    # Default command is bash, but can be overridden
    local command="bash"
    for arg in "$@"; do
        if [[ "$arg" == --command=* ]]; then
            command="${arg#--command=}"
        fi
    done
    
    # Find container by pattern
    local container
    container=$(find_container "$container_pattern")
    
    if [ $? -eq 0 ]; then
        log_info "Connecting to container: $container"
        
        # Check if the command exists in the container
        if ! docker exec "$container" which "$command" &> /dev/null; then
            log_warn "Command '$command' not found in container. Falling back to /bin/sh"
            command="sh"
        fi
        
        # Connect to the container
        docker exec -it "$container" "$command"
        
        # Check exit status
        local exit_status=$?
        if [ $exit_status -ne 0 ]; then
            log_warn "Command exited with status $exit_status"
        fi
    else
        log_error "Failed to connect to container. Please try again with a more specific name."
    fi
}

function cmd_status {
    check_docker
    
    log_info "AirStack container status:"
    docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
}

function cmd_logs {
    check_docker
    
    if [ $# -eq 0 ]; then
        log_error "Container name required"
        
        # Show available containers as a helpful suggestion
        local available=$(docker ps --format "{{.Names}}")
        if [ -n "$available" ]; then
            log_info "Available containers:"
            echo "$available"
        fi
        
        exit 1
    fi
    
    local container_pattern="$1"
    shift
    
    # Parse options
    local follow=true
    local tail="all"
    
    for arg in "$@"; do
        if [ "$arg" == "--no-follow" ]; then
            follow=false
        elif [[ "$arg" == --tail=* ]]; then
            tail="${arg#--tail=}"
        fi
    done
    
    # Find container by pattern
    local container
    container=$(find_container "$container_pattern")
    
    if [ $? -eq 0 ]; then
        log_info "Showing logs for container: $container"
        
        # Build the logs command
        local cmd="docker logs"
        
        if [ "$follow" = true ]; then
            cmd="$cmd -f"
        fi
        
        if [ "$tail" != "all" ]; then
            cmd="$cmd --tail $tail"
        fi
        
        # Execute the command
        eval "$cmd $container"
    else
        log_error "Failed to find container. Please try again with a more specific name."
    fi
}

# Function to load external command modules
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

# Arrays to store available commands and their help text
declare -A COMMANDS
declare -A COMMAND_HELP

# Register built-in commands
function register_builtin_commands {
    COMMANDS["install"]="cmd_install"
    COMMANDS["setup"]="cmd_setup"
    COMMANDS["up"]="cmd_up"
    COMMANDS["down"]="cmd_down"
    COMMANDS["connect"]="cmd_connect"
    COMMANDS["status"]="cmd_status"
    COMMANDS["logs"]="cmd_logs"
    COMMANDS["help"]="cmd_help"
    
    # Register help text for built-in commands
    COMMAND_HELP["install"]="Install dependencies (Docker Engine, Docker Compose, etc.)"
    COMMAND_HELP["setup"]="Configure AirStack settings and add to shell profile"
    COMMAND_HELP["up"]="Start services using Docker Compose"
    COMMAND_HELP["down"]="down services"
    COMMAND_HELP["connect"]="Connect to a running container (supports partial name matching)"
    COMMAND_HELP["status"]="Show status of all containers"
    COMMAND_HELP["logs"]="View logs for a container (supports partial name matching)"
    COMMAND_HELP["help"]="Show help information"
}

# Help command implementation
function cmd_help {
    if [ $# -eq 0 ]; then
        print_usage
    else
        print_command_help "$1"
    fi
}

# Function to list all available commands
function list_commands {
    echo -e "${BLUE}All Available AirStack Commands:${NC}"
    echo ""
    
    # Sort commands alphabetically
    local sorted_commands=($(echo "${!COMMANDS[@]}" | tr ' ' '\n' | sort))
    
    # Calculate the longest command name for padding
    local max_len=0
    for cmd in "${sorted_commands[@]}"; do
        if [ ${#cmd} -gt $max_len ]; then
            max_len=${#cmd}
        fi
    done
    
    # Add 4 spaces of padding
    max_len=$((max_len + 4))
    
    # Group commands by module (based on prefix)
    declare -A modules
    
    for cmd in "${sorted_commands[@]}"; do
        # Skip hidden commands (those starting with _)
        if [[ "$cmd" == _* ]]; then
            continue
        fi
        
        # Determine module based on command name prefix
        local module="core"
        if [[ "$cmd" == *_* ]]; then
            module="${cmd%%_*}"
        fi
        
        # Add command to its module group
        modules["$module"]+="$cmd "
    done
    
    # Print commands by module
    for module in $(echo "${!modules[@]}" | tr ' ' '\n' | sort); do
        echo -e "${YELLOW}$module:${NC}"
        
        for cmd in ${modules["$module"]}; do
            # Get help text or use a default
            local help_text="${COMMAND_HELP[$cmd]:-No description available}"
            
            # Calculate padding
            local padding=$((max_len - ${#cmd}))
            local pad=$(printf '%*s' "$padding" '')
            
            echo "  $cmd$pad$help_text"
        done
        
        echo ""
    done
}

# Register built-in commands
register_builtin_commands

# Load external command modules
load_command_modules

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

exit 0