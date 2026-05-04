#!/usr/bin/env bash

# dev.sh - Development-related commands for AirStack
# This module provides commands for development tasks

# Run tests via the dockerized pytest runner. All args forward to pytest.
function cmd_dev_test {
    check_docker
    local compose_file="$PROJECT_ROOT/tests/docker/docker-compose.yaml"
    local env_file="$PROJECT_ROOT/.env"
    export AIRSTACK_PATH="$PROJECT_ROOT"
    # Grant X access so sim containers spawned by tests in GUI mode
    # (`pytest --gui`) can reach the host's X server. No-op otherwise.
    xhost + || log_warn "xhost failed (is DISPLAY set? xhost installed?)"
    docker compose --env-file "$env_file" -f "$compose_file" build --quiet
    docker compose --env-file "$env_file" -f "$compose_file" run --rm test pytest "$@"
}

# Function to build documentation
function cmd_dev_docs {
    log_info "Building documentation..."
    
    # Check if mkdocs is installed
    if ! command -v mkdocs &> /dev/null; then
        log_warn "mkdocs not found. Installing..."
        pip install mkdocs mkdocs-material
    fi
    
    # Build documentation
    cd "$PROJECT_ROOT"
    mkdocs build
    
    # Serve documentation if requested
    if [[ "$1" == "serve" ]]; then
        log_info "Serving documentation at http://localhost:8000"
        mkdocs serve
    fi
}

# Function to lint code
function cmd_dev_lint {
    log_info "Linting code..."
    
    # Add your linting commands here
    # For example:
    # flake8 "$PROJECT_ROOT"
    echo "Linting command would run here"
}

# Function to format code
function cmd_dev_format {
    log_info "Formatting code..."
    
    # Add your formatting commands here
    # For example:
    # black "$PROJECT_ROOT"
    echo "Formatting command would run here"
}

# Register commands from this module
function register_dev_commands {
    COMMANDS["test"]="cmd_dev_test"
    COMMANDS["docs"]="cmd_dev_docs"
    COMMANDS["lint"]="cmd_dev_lint"
    COMMANDS["format"]="cmd_dev_format"
    
    # Add command help
    COMMAND_HELP["test"]="Run tests (options: --path=PATH, --filter=PATTERN)"
    COMMAND_HELP["docs"]="Build documentation (options: serve)"
    COMMAND_HELP["lint"]="Lint code"
    COMMAND_HELP["format"]="Format code"
}