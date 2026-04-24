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
BOLDCYAN='\033[38;5;14;1m'
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print usage information
function print_usage {
    local version
    version="$(get_VERSION)"
    echo -e "${BOLDCYAN}AirStack Development Tool v${version} ($SCRIPT_DIR)${NC}"
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
    echo ""
    echo "Note: The airstack command will automatically use the airstack.sh script from the"
    echo "current directory or nearest parent directory containing an AirStack repository."
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
            echo ""
            echo "This command adds an 'airstack' function to your shell profile that will"
            echo "automatically find and use the airstack.sh script from the current directory"
            echo "or nearest parent directory containing an AirStack repository."
            ;;
        up)
            echo "Usage: airstack up [service_name] [options]"
            echo ""
            echo "Options:"
            echo "  --build       Build images before starting containers"
            echo "  --recreate    Recreate containers even if their configuration and image haven't changed"
            echo "  --robot_num N Set ROBOT_NAME/ROBOT_INDEX for robot HITL containers (numeric only)"
            ;;
        images)
            echo "Usage: airstack images"
            echo ""
            echo "List Docker images whose repository/tag contains the PROJECT_NAME value from .env."
            echo "Shows all images if PROJECT_NAME is not set."
            ;;
        image-build)
            echo "Usage: airstack image-build [service_name...] [options]"
            echo ""
            echo "Build or rebuild Docker Compose services. Passes ENV variables from .env"
            echo "and any prepended environment variables (e.g. VERSION=x airstack build robot)."
            echo ""
            echo "Options:"
            echo "  --no-cache         Do not use cache when building the image"
            echo "  --pull             Always attempt to pull a newer version of the image"
            echo "  --push             Push service images"
            echo "  --progress=VALUE   Set type of progress output (auto, tty, plain, json, quiet)"
            echo "  --env-file         Specify an alternate environment file"
            echo ""
            echo "Any additional flags are passed directly to 'docker compose build'."
            ;;
        connect)
            echo "Usage: airstack connect [container_name] [options]"
            echo ""
            echo "By default, attaches to an existing tmux session inside the container."
            echo "Exiting tmux fully disconnects from the container."
            echo ""
            echo "Options:"
            echo "  --command=CMD  Run a specific command instead of attaching to tmux (e.g., --command=bash)"
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
        version)
            echo "Usage: airstack version"
            echo ""
            echo "Display the current AirStack version from VERSION in .env file."
            ;;
        rmi)
            echo "Usage: airstack rmi [flags] <search_term>"
            echo ""
            echo "Remove Docker images whose tag matches the given search term."
            echo ""
            echo "Options:"
            echo "  -f    Force removal of the image"
            echo ""
            echo "Examples:"
            echo "  airstack rmi myimage"
            echo "  airstack rmi -f myimage"
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

# Get VERSION from .env file
function get_VERSION {
    local env_file="$PROJECT_ROOT/.env"
    
    if [ ! -f "$env_file" ]; then
        log_warn ".env file not found, using 'latest' tag"
        echo "latest"
        return
    fi
    
    # Extract VERSION from .env file
    local version=$(grep -E "^VERSION=" "$env_file" | cut -d'=' -f2 | tr -d '"' | tr -d "'")
    
    if [ -z "$version" ]; then
        log_warn "VERSION not found in .env, using 'latest' tag"
        echo "latest"
        return
    fi
    
    echo "$version"
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

# Wrapper function to run docker compose natively on the host
function run_docker_compose {
    local env_args=()
    local env_file="$PROJECT_ROOT/.env"

    if [ -f "$env_file" ]; then
        env_args+=("--env-file" "$env_file")
    fi

    docker compose "${env_args[@]}" "$@"
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
        log_warn "No exact matches for '$search_term', trying fuzzy search..." >&2
        matches=$(echo "$containers" | grep -i ".*$search_term.*" || true)
        match_count=$(echo "$matches" | grep -v "^$" | wc -l)
        
        if [ "$match_count" -eq 0 ]; then
            log_error "No running containers match '$search_term'"
            
            # Show available containers as a suggestion
            available=$(docker ps --format "{{.Names}}")
            if [ -n "$available" ]; then
                log_info "Available containers:" >&2
                echo "$available" >&2
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
        log_warn "Multiple containers match '$search_term'. Please be more specific or select from the list below:" >&2
        # Format the output as a table with numbers (redirect to stderr so it's not captured)
        echo -e "${BLUE}NUM\tCONTAINER NAME\tIMAGE\tSTATUS${NC}" >&2
        echo "$matches" | awk '{print NR "\t" $0}' >&2
        echo "" >&2
        echo "Options:" >&2
        echo "  1. Enter a number to select a container" >&2
        echo "  2. Type 'q' to quit" >&2
        echo "  3. Press Ctrl+C to cancel and try again with a more specific name" >&2
        echo "" >&2
        read -p "Your selection: " selection <&2
        
        if [ "$selection" = "q" ]; then
            log_info "Operation cancelled" >&2
            return 1
        elif [[ "$selection" =~ ^[0-9]+$ ]] && [ "$selection" -gt 0 ] && [ "$selection" -le "$match_count" ]; then
            # Extract just the container name from the selected line
            container_name=$(echo "$matches" | sed -n "${selection}p" | awk '{print $1}')
            echo "$container_name"
            
            # Provide a tip for future use
            log_info "Tip: Next time, you can directly use 'airstack connect $container_name' for this container" >&2
            return 0
        else
            log_error "Invalid selection. Please enter a number between 1 and $match_count, or 'q' to quit."
            
            # Give the user another chance to select
            echo "" >&2
            read -p "Try again (or 'q' to quit): " selection <&2
            
            if [ "$selection" = "q" ]; then
                log_info "Operation cancelled" >&2
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
        
        # Install NVIDIA Container Toolkit for GPU support
        if [ "$OS" = "ubuntu" ] || [ "$OS" = "debian" ]; then
            log_info "Installing NVIDIA Container Toolkit..."
            
            # Install prerequisites
            log_info "Installing prerequisites for NVIDIA Container Toolkit..."
            sudo apt-get update && sudo apt-get install -y --no-install-recommends \
                curl \
                gnupg2
            
            # Configure the production repository
            log_info "Configuring NVIDIA Container Toolkit repository..."
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
                && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
                sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
            
            # Update the packages list from the repository
            log_info "Updating package list..."
            sudo apt-get update
            
            # Install the NVIDIA Container Toolkit packages
            log_info "Installing NVIDIA Container Toolkit packages..."
            export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.1-1
            sudo apt-get install -y \
                nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
                nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
                libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
                libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
            
            # Configure Docker to use NVIDIA runtime
            log_info "Configuring Docker to use NVIDIA runtime..."
            sudo nvidia-ctk runtime configure --runtime=docker
            
            # Restart Docker service to apply changes
            if systemctl is-active --quiet docker; then
                log_info "Restarting Docker service to apply NVIDIA runtime configuration..."
                sudo systemctl restart docker
            fi
            
            log_info "NVIDIA Container Toolkit installation complete"
        else
            log_warn "NVIDIA Container Toolkit installation is only supported on Ubuntu/Debian systems"
            log_info "For other systems, please install manually: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html"
        fi
        
        # Check Docker Compose version
        local compose_min_version="5.0.0"
        local compose_targt_version="5.0.2"
        log_info "Checking Docker Compose version..."
        local compose_current
        if compose_current=$(docker compose version --short 2>/dev/null); then
            if ! printf '%s\n%s\n' "$compose_min_version" "$compose_current" | sort -V -C; then
                log_warn "Docker Compose version $compose_current is older than the minimum required version $compose_min_version"
                read -p "Would you like to upgrade Docker Compose to the latest version (v$compose_targt_version)? [y/N] " upgrade_choice
                if [[ "$upgrade_choice" =~ ^[Yy]$ ]]; then
                    log_info "Upgrading Docker Compose..."
                    local _dc_config=${DOCKER_CONFIG:-$HOME/.docker}
                    local arch
                    arch=$(uname -m)
                    mkdir -p "$_dc_config/cli-plugins"
                    curl -SL "https://github.com/docker/compose/releases/download/v$compose_targt_version/docker-compose-linux-$arch" \
                        -o "$_dc_config/cli-plugins/docker-compose"
                    chmod +x "$_dc_config/cli-plugins/docker-compose"
                    log_info "Docker Compose upgraded successfully"
                    log_info "New version: $(docker compose version --short)"
                else
                    log_warn "Skipping Docker Compose upgrade. Some features may not work correctly."
                fi
            else
                log_info "Docker Compose version $compose_current meets the minimum requirement ($compose_min_version)"
            fi
        else
            log_warn "Could not determine Docker Compose version. Please ensure Docker Compose v$compose_min_version or newer is installed."
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
            log_warn "Could not determine shell profile. Please add the airstack function to your shell profile manually."
            echo "You can find the function definition in the setup section of $PROJECT_ROOT/airstack.sh"
            return
        fi
        
        if grep -q "# AirStack function" "$shell_profile"; then
            log_info "'airstack' function already registered"
        else
            log_info "Adding 'airstack' function in $shell_profile"
            echo "" >> "$shell_profile"
            echo "# AirStack function" >> "$shell_profile"
            echo 'function airstack() {' >> "$shell_profile"
            echo '    # Start from the current directory and look for airstack.sh' >> "$shell_profile"
            echo '    local current_dir="$(pwd)"' >> "$shell_profile"
            echo '    local found_scripts=()' >> "$shell_profile"
            echo '    local script_path=""' >> "$shell_profile"
            echo '' >> "$shell_profile"
            echo '    # First check if there is an airstack.sh in the current directory or any parent directory' >> "$shell_profile"
            echo '    while [[ "$current_dir" != "" ]]; do' >> "$shell_profile"
            echo '        if [[ -f "$current_dir/airstack.sh" ]]; then' >> "$shell_profile"
            echo '            found_scripts+=("$current_dir/airstack.sh")' >> "$shell_profile"
            echo '            # We found one, but keep looking to check for ambiguities' >> "$shell_profile"
            echo '        fi' >> "$shell_profile"
            echo '        # Stop if we reach the root directory' >> "$shell_profile"
            echo '        if [[ "$current_dir" == "/" ]]; then' >> "$shell_profile"
            echo '            break' >> "$shell_profile"
            echo '        fi' >> "$shell_profile"
            echo '        # Move up one directory' >> "$shell_profile"
            echo '        current_dir="$(dirname "$current_dir")"' >> "$shell_profile"
            echo '    done' >> "$shell_profile"
            echo '' >> "$shell_profile"
            echo '    # Check how many scripts we found' >> "$shell_profile"
            echo '    if [[ ${#found_scripts[@]} -eq 0 ]]; then' >> "$shell_profile"
            echo '        echo -e "\033[0;31m[ERROR]\033[0m No airstack.sh script found in the current directory or any parent directory."' >> "$shell_profile"
            echo '        return 1' >> "$shell_profile"
            echo '    elif [[ ${#found_scripts[@]} -gt 1 ]]; then' >> "$shell_profile"
            echo '        echo -e "\033[0;31m[ERROR]\033[0m Multiple airstack.sh scripts found in the directory hierarchy:"' >> "$shell_profile"
            echo '        for script in "${found_scripts[@]}"; do' >> "$shell_profile"
            echo '            echo "  - $script"' >> "$shell_profile"
            echo '        done' >> "$shell_profile"
            echo '        echo "Please cd to the specific AirStack repository you want to use."' >> "$shell_profile"
            echo '        return 1' >> "$shell_profile"
            echo '    else' >> "$shell_profile"
            echo '        # We found exactly one script, use it' >> "$shell_profile"
            echo '        script_path="${found_scripts[@]:0:1}"' >> "$shell_profile"
            echo '        "$script_path" "$@"' >> "$shell_profile"
            echo '    fi' >> "$shell_profile"
            echo '}' >> "$shell_profile"
            echo "Added to $shell_profile. Please restart your shell or run 'source $shell_profile'. Then you'll be able to use the 'airstack' command from any sub-directory."
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

    log_info "Making sure git submodules are initialized and updated..."
    git submodule update --init --recursive || log_warn "Failed to update git submodules. Some packages may not launch, please check your git credentials."
    
    log_info "Setup complete!"
}

# Classify arguments for a docker compose invocation.
# Global docker compose flags (must appear before the subcommand) are separated
# from subcommand-specific arguments.
#
# Usage: classify_compose_args <global_array_name> <subcmd_array_name> "$@"
#
# Example:
#   local global_args=() subcmd_args=()
#   classify_compose_args global_args subcmd_args "$@"
function classify_compose_args {
    local -n _global=$1
    local -n _subcmd=$2
    shift 2

    # Global flags that consume the next argument as their value
    local -A _takes_value=(
        [--ansi]=1 [--env-file]=1 [-f]=1 [--file]=1
        [--parallel]=1 [--profile]=1 [--progress]=1
        [--project-directory]=1 [--workdir]=1
        [-p]=1 [--project-name]=1 [-H]=1 [--host]=1
    )
    # Global flags that are boolean (standalone, no value)
    local -A _is_bool=(
        [--compatibility]=1 [--dry-run]=1 [--verbose]=1 [--version]=1
    )

    local args=("$@")
    local i=0
    while [ $i -lt ${#args[@]} ]; do
        local arg="${args[$i]}"
        if [[ -n "${_takes_value[$arg]+x}" ]]; then
            # Flag + value pair -> global
            _global+=("$arg")
            i=$((i+1))
            if [ $i -lt ${#args[@]} ]; then
                _global+=("${args[$i]}")
            else
                echo "[ERROR] Missing value for compose global flag: $arg" >&2
                return 1
            fi
        elif [[ -n "${_is_bool[$arg]+x}" ]]; then
            # Boolean flag -> global
            _global+=("$arg")
        elif [[ "$arg" == -* && "$arg" == *=* ]]; then
            # --flag=value form: check the flag name portion
            local flag_name="${arg%%=*}"
            if [[ -n "${_takes_value[$flag_name]+x}" || -n "${_is_bool[$flag_name]+x}" ]]; then
                _global+=("$arg")
            else
                _subcmd+=("$arg")
            fi
        else
            _subcmd+=("$arg")
        fi
        i=$((i+1))
    done
}

function cmd_up {
    check_docker

    local robot_num=""
    local filtered_args=()
    local args=("$@")
    local i=0
    while [ $i -lt ${#args[@]} ]; do
        local arg="${args[$i]}"
        if [ "$arg" = "--robot_num" ]; then
            i=$((i+1))
            if [ $i -ge ${#args[@]} ]; then
                log_error "--robot_num requires a value"
                exit 1
            fi
            robot_num="${args[$i]}"
            if ! [[ "$robot_num" =~ ^[0-9]+$ ]]; then
                log_error "--robot_num must be a number (received: $robot_num)"
                exit 1
            fi
        elif [[ "$arg" == --robot_num=* ]]; then
            robot_num="${arg#--robot_num=}"
            if ! [[ "$robot_num" =~ ^[0-9]+$ ]]; then
                log_error "--robot_num must be a number (received: $robot_num)"
                exit 1
            fi
        else
            filtered_args+=("$arg")
        fi
        i=$((i+1))
    done

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "${filtered_args[@]}"

    local service_count=0
    local target_is_hitl=false
    for s in "${subcmd_args[@]}"; do
        if [[ "$s" != -* ]]; then
            service_count=$((service_count+1))
            if [ "$s" = "robot-desktop-hitl" ]; then
                target_is_hitl=true
            fi
        fi
    done

    if [ -n "$robot_num" ]; then
        if [ "$service_count" -ne 1 ] || [ "$target_is_hitl" = false ]; then
            log_error "--robot_num can only be used with exactly one service: robot-desktop-hitl"
            log_error "Example: airstack up robot-desktop-hitl --robot_num 2"
            exit 1
        fi
    elif [ "$service_count" -eq 1 ] && [ "$target_is_hitl" = true ]; then
        robot_num="1"
        log_info "No --robot_num provided; defaulting to robot_num=1 for robot-desktop-hitl"
    fi

    # Ensure only one simulator profile is active
    local p="${COMPOSE_PROFILES:-$(sed -n 's/^COMPOSE_PROFILES=//p' "$PROJECT_ROOT/.env" 2>/dev/null | tr -d '"')}"
    for arg in "${global_args[@]}"; do p+=",${arg}"; done
    local n=0; for s in isaac-sim ms-airsim simple; do [[ ",$p," == *",$s,"* ]] && n=$((n+1)); done
    (( n > 1 )) && log_error "Only one simulator profile can be active at a time (isaac-sim, ms-airsim, simple)." && exit 1

    # Warn if URDF_FILE doesn't match the active simulator
    local urdf=$(sed -n 's/^URDF_FILE=//p' "$PROJECT_ROOT/.env" 2>/dev/null | tr -d '"')
    if [[ -n "$urdf" ]]; then
        [[ ",$p," == *",ms-airsim,"* && "$urdf" != *.ms-airsim.* ]] && log_warn "URDF_FILE ($urdf) does not match ms-airsim profile. Expected *.ms-airsim.* URDF."
        [[ ",$p," == *",isaac-sim,"* && "$urdf" != *.pegasus.* && "$urdf" != *.isaacsim.* ]] && log_warn "URDF_FILE ($urdf) does not match isaac-sim profile. Expected *.pegasus.* or *.isaacsim.* URDF."
    fi

    # Add xhost + to allow GUI applications
    xhost + &> /dev/null || true

    log_info "Starting services..."
    if [ -n "$robot_num" ]; then
        local robot_name="robot_${robot_num}"
        log_info "Using robot identity from --robot_num: ROBOT_NAME=${robot_name}, ROBOT_INDEX=${robot_num}"
        ROBOT_NAME="$robot_name" ROBOT_INDEX="$robot_num" run_docker_compose -f "$PROJECT_ROOT/docker-compose.yaml" "${global_args[@]}" up "${subcmd_args[@]}" -d
    else
        run_docker_compose -f "$PROJECT_ROOT/docker-compose.yaml" "${global_args[@]}" up "${subcmd_args[@]}" -d
    fi
    log_info "Services brought up successfully"
}

function cmd_image_build {
    check_docker

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "$@"

    log_info "Building services..."
    run_docker_compose -f "$PROJECT_ROOT/docker-compose.yaml" "${global_args[@]}" build "${subcmd_args[@]}"
    log_info "Build completed successfully"
}

function cmd_image_push {
    check_docker

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "$@"

    log_info "Pushing service images..."
    run_docker_compose -f "$PROJECT_ROOT/docker-compose.yaml" "${global_args[@]}" push "${subcmd_args[@]}"
    log_info "Push completed successfully"
}

function cmd_image_pull {
    check_docker

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "$@"

    log_info "Pulling service images..."
    run_docker_compose -f "$PROJECT_ROOT/docker-compose.yaml" "${global_args[@]}" pull "${subcmd_args[@]}"
    log_info "Pull completed successfully"
}

function cmd_images {
    check_docker

    local env_file="$PROJECT_ROOT/.env"
    local project_name=""

    if [ -f "$env_file" ]; then
        project_name=$(grep -E "^PROJECT_NAME=" "$env_file" | cut -d'=' -f2 | tr -d '"' | tr -d "'")
    fi

    if [ -z "$project_name" ]; then
        log_warn "PROJECT_NAME not found in .env, showing all images"
        docker images
    else
        log_info "Showing images matching project: $project_name"
        docker images | head -1
        docker images | grep -i "$project_name" || true
    fi
}

function cmd_down {
    check_docker
    
    local services=("$@")
    
    # Build compose arguments
    local compose_args=("-f" "$PROJECT_ROOT/docker-compose.yaml")
    
    # Add services if specified
    if [ ${#services[@]} -gt 0 ]; then
        compose_args+=("down" "${services[@]}")
    else
        compose_args+=("--profile" "*" "down")
    fi
    
    log_info "Shutting down services: ${services[*]:-all}"
    run_docker_compose "${compose_args[@]}"
    log_info "Services shutdown successfully"
}

function cmd_clean {
    local dirs_to_clean=(
        "$PROJECT_ROOT/robot/ros_ws/build"
        "$PROJECT_ROOT/robot/ros_ws/install"
        "$PROJECT_ROOT/robot/ros_ws/log"
        "$PROJECT_ROOT/gcs/ros_ws/build"
        "$PROJECT_ROOT/gcs/ros_ws/install"
        "$PROJECT_ROOT/gcs/ros_ws/log"
        "$PROJECT_ROOT/simulation/ms-airsim/ros_ws/build"
        "$PROJECT_ROOT/simulation/ms-airsim/ros_ws/install"
        "$PROJECT_ROOT/simulation/ms-airsim/ros_ws/log"
    )

    log_info "Cleaning all ROS 2 build artifacts..."
    for dir in "${dirs_to_clean[@]}"; do
        if [ -d "$dir" ]; then
            rm -rf "$dir"
            echo "  Removed $dir"
        fi
    done

    # Clean .egg-info from source directories (generated by colcon build --symlink-install)
    local egg_count=$(find "$PROJECT_ROOT" -type d -name "*.egg-info" -not -path "*/.git/*" -not -path "*/build/*" -not -path "*/install/*" 2>/dev/null | wc -l)
    find "$PROJECT_ROOT" -type d -name "*.egg-info" -not -path "*/.git/*" -not -path "*/build/*" -not -path "*/install/*" -exec rm -rf {} + 2>/dev/null || true
    if [ "$egg_count" -gt 0 ]; then
        echo "  Removed $egg_count .egg-info directories from source trees"
    fi

    # Clean Python bytecode caches
    local pycache_count=$(find "$PROJECT_ROOT" -type d -name "__pycache__" -not -path "*/.git/*" 2>/dev/null | wc -l)
    find "$PROJECT_ROOT" -type d -name "__pycache__" -not -path "*/.git/*" -exec rm -rf {} + 2>/dev/null || true
    if [ "$pycache_count" -gt 0 ]; then
        echo "  Removed $pycache_count __pycache__ directories"
    fi

    log_info "Clean complete. Next 'airstack up' will trigger a full rebuild."
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
    
    # By default, attach to a tmux session. If --command is specified, run that command directly.
    local command=""
    local command_specified=false
    for arg in "$@"; do
        if [[ "$arg" == --command=* ]]; then
            command="${arg#--command=}"
            command_specified=true
        fi
    done
    
    # Find container by pattern
    local container
    container=$(find_container "$container_pattern")
    
    if [ $? -eq 0 ]; then
        log_info "Connecting to container: $container"
        
        if [ "$command_specified" = true ]; then
            # Run the specified command directly, no tmux involvement
            if ! docker exec "$container" which "$command" &> /dev/null; then
                log_warn "Command '$command' not found in container. Falling back to /bin/sh"
                command="sh"
            fi
            docker exec -it "$container" "$command"
        else
            # Default: attach to an existing tmux session. Exiting tmux fully disconnects from the container.
            if docker exec "$container" which tmux &> /dev/null; then
                docker exec -it "$container" tmux a
            else
                log_warn "tmux not found in container. Falling back to bash."
                local fallback="bash"
                if ! docker exec "$container" which bash &> /dev/null; then
                    fallback="sh"
                fi
                docker exec -it "$container" "$fallback"
            fi
        fi
        
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

    # Get list of running containers: name, status, ports
    local containers
    containers=$(docker ps --format "{{.Names}}\t{{.Status}}\t{{.Ports}}")

    if [ -z "$containers" ]; then
        echo "No running containers."
        return 0
    fi

    # Collect rows: container_name, robot_name, robot_index, status, ports
    local -a rows=()
    while IFS=$'\t' read -r name status ports; do
        local robot_name robot_index vars
        # Single exec: unique prefix lets us grep out .bashrc echo noise
        vars=$(docker exec "$name" bash --login -c \
            'printf "AIRSTACK_VARS:%s:%s\n" "$ROBOT_NAME" "$ROBOT_INDEX"' 2>/dev/null \
            | grep "^AIRSTACK_VARS:" | tail -1)
        if [ -n "$vars" ]; then
            robot_name="${vars#AIRSTACK_VARS:}"  # strip leading prefix
            robot_index="${robot_name##*:}"      # everything after last colon
            robot_name="${robot_name%%:*}"        # everything before first colon
        fi
        [ -z "$robot_name" ]  && robot_name="N/A"
        [ -z "$robot_index" ] && robot_index="N/A"
        rows+=("${name}|${robot_name}|${robot_index}|${status}|${ports}")
    done <<< "$containers"

    # Sort rows alphabetically by container name
    local sorted
    sorted=$(printf '%s\n' "${rows[@]}" | sort -t'|' -k1,1)
    mapfile -t rows <<< "$sorted"

    # Determine column widths (minimum = header length)
    local w_container=14 w_robot=10 w_index=12 w_status=6 w_ports=5
    for row in "${rows[@]}"; do
        IFS='|' read -r c_name c_robot c_index c_status c_ports <<< "$row"
        [ ${#c_name}   -gt $w_container ] && w_container=${#c_name}
        [ ${#c_robot}  -gt $w_robot ]     && w_robot=${#c_robot}
        [ ${#c_index}  -gt $w_index ]     && w_index=${#c_index}
        [ ${#c_status} -gt $w_status ]    && w_status=${#c_status}
    done
    # Add padding
    w_container=$((w_container + 2))
    w_robot=$((w_robot + 2))
    w_index=$((w_index + 2))
    w_status=$((w_status + 2))

    # Print header
    printf "${BOLDCYAN}%-${w_container}s %-${w_robot}s %-${w_index}s %-${w_status}s %s${NC}\n" \
        "CONTAINER NAME" "ROBOT_NAME" "ROBOT_INDEX" "STATUS" "PORTS"
    # Separator
    printf "%-${w_container}s %-${w_robot}s %-${w_index}s %-${w_status}s %s\n" \
        "$(printf '%*s' "$w_container" '' | tr ' ' '-')" \
        "$(printf '%*s' "$w_robot"     '' | tr ' ' '-')" \
        "$(printf '%*s' "$w_index"     '' | tr ' ' '-')" \
        "$(printf '%*s' "$w_status"    '' | tr ' ' '-')" \
        "-----"

    # Print rows
    for row in "${rows[@]}"; do
        IFS='|' read -r c_name c_robot c_index c_status c_ports <<< "$row"
        printf "%-${w_container}s %-${w_robot}s %-${w_index}s %-${w_status}s %s\n" \
            "$c_name" "$c_robot" "$c_index" "$c_status" "$c_ports"
    done
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

function cmd_version {
    local env_file="$PROJECT_ROOT/.env"
    
    if [ ! -f "$env_file" ]; then
        log_error ".env file not found at $env_file"
        return 1
    fi
    
    # Extract VERSION from .env file
    local version=$(grep -E "^VERSION=" "$env_file" | cut -d'=' -f2 | tr -d '"' | tr -d "'")
    
    if [ -z "$version" ]; then
        log_error "VERSION not found in .env file"
        return 1
    fi
    
    echo -e "${BOLDCYAN}AirStack Version:${NC} $version"
}

function cmd_rmi {
    check_docker
    
    if [ $# -eq 0 ]; then
        log_error "Search term required"
        print_command_help "rmi"
        exit 1
    fi
    
    local search_term=""
    local rmi_flags=()
    
    for arg in "$@"; do
        if [[ "$arg" == -* ]]; then
            rmi_flags+=("$arg")
        else
            search_term="$arg"
        fi
    done
    
    if [ -z "$search_term" ]; then
        log_error "Search term required"
        print_command_help "rmi"
        exit 1
    fi
    
    log_info "Removing images matching: $search_term"
    docker images -a | grep "$search_term" | awk '{print $2}' | xargs docker rmi "${rmi_flags[@]}"
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
    COMMANDS["image-build"]="cmd_image_build"
    COMMANDS["image-push"]="cmd_image_push"
    COMMANDS["image-pull"]="cmd_image_pull"
    COMMANDS["images"]="cmd_images"
    COMMANDS["up"]="cmd_up"
    COMMANDS["down"]="cmd_down"
    COMMANDS["clean"]="cmd_clean"
    COMMANDS["connect"]="cmd_connect"
    COMMANDS["status"]="cmd_status"
    COMMANDS["logs"]="cmd_logs"
    COMMANDS["version"]="cmd_version"
    COMMANDS["rmi"]="cmd_rmi"
    COMMANDS["help"]="cmd_help"
    
    # Register help text for built-in commands
    COMMAND_HELP["install"]="Install dependencies (Docker Engine, NVIDIA Container Toolkit)"
    COMMAND_HELP["setup"]="Configure AirStack settings and add to shell profile"
    COMMAND_HELP["image-build"]="Build or rebuild Docker Compose service images"
    COMMAND_HELP["image-push"]="Push Docker Compose service images to a registry"
    COMMAND_HELP["image-pull"]="Pull Docker Compose service images from a registry"
    COMMAND_HELP["images"]="List Docker images filtered by PROJECT_NAME from .env"
    COMMAND_HELP["up"]="Start services using Docker Compose"
    COMMAND_HELP["down"]="down services"
    COMMAND_HELP["clean"]="Remove all ROS 2 build artifacts (build/, install/, log/)"
    COMMAND_HELP["connect"]="Connect to a running container (supports partial name matching)"
    COMMAND_HELP["status"]="Show status of all containers"
    COMMAND_HELP["logs"]="View logs for a container (supports partial name matching)"
    COMMAND_HELP["version"]="Display the current AirStack version"
    COMMAND_HELP["rmi"]="Remove Docker images by search term"
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

# Simple command detection - just look for the first argument that matches a command
command=""

# First check if the first argument is a command
if [[ -n "${COMMANDS[$1]}" || "$1" == "commands" || "$1" == "help" ]]; then
    command="$1"
else
    # Otherwise, look through all arguments for a command
    for arg in "$@"; do
        if [[ -n "${COMMANDS[$arg]}" || "$arg" == "commands" || "$arg" == "help" ]]; then
            command="$arg"
            break
        fi
    done
    
    # If still no command found, show usage
    if [ -z "$command" ]; then
        print_usage
        exit 1
    fi
fi

# Check if command exists
if [[ -n "${COMMANDS[$command]}" ]]; then
    # Execute the command function with filtered arguments
    # We need to remove the command name from the arguments
    filtered_args=()
    for arg in "$@"; do
        if [[ "$arg" != "$command" ]]; then
            filtered_args+=("$arg")
        fi
    done
    
    # Execute the command with the filtered arguments
    ${COMMANDS[$command]} "${filtered_args[@]}"
elif [ "$command" == "commands" ]; then
    # Special case for listing all commands
    list_commands
else
    log_error "Unknown command: $command"
    print_usage
    exit 1
fi

exit 0