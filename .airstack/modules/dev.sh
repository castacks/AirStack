#!/usr/bin/env bash

# dev.sh - Development-related commands for AirStack
# This module provides commands for development tasks

# Function to run tests
function cmd_dev_test {
    log_info "Running tests..."
    
    local test_path="$PROJECT_ROOT/tests"
    local test_filter=""
    
    # Parse arguments
    for arg in "$@"; do
        if [[ "$arg" == --path=* ]]; then
            test_path="${arg#--path=}"
        elif [[ "$arg" == --filter=* ]]; then
            test_filter="${arg#--filter=}"
        fi
    done
    
    if [ -n "$test_filter" ]; then
        log_info "Running tests matching '$test_filter' in $test_path"
        # Add your test command here, e.g.:
        # pytest "$test_path" -k "$test_filter"
        echo "Test command would run here with filter: $test_filter"
    else
        log_info "Running all tests in $test_path"
        # Add your test command here, e.g.:
        # pytest "$test_path"
        echo "Test command would run here"
    fi
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