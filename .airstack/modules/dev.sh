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

# Fast host-side static checks: the tests/meta contract+lint suite, bash
# syntax over the CLI itself, and py_compile over tools/. No Docker stack.
function cmd_dev_lint {
    local failed=0

    log_info "1/3 Contract + lint suite (tests/meta, pytest -m unit-compatible)..."
    if ! (cd "$PROJECT_ROOT" && env -u PYTHONPATH AIRSTACK_ROOT="$PROJECT_ROOT" \
            python3 -m pytest tests/meta -q -p no:launch_testing -p no:launch_ros); then
        log_error "tests/meta failed."
        failed=1
    fi

    log_info "2/3 bash -n over airstack.sh and .airstack/modules/*.sh..."
    local sh_file
    for sh_file in "$PROJECT_ROOT/airstack.sh" "$PROJECT_ROOT/.airstack/modules"/*.sh; do
        if ! bash -n "$sh_file"; then
            log_error "bash syntax error: $sh_file"
            failed=1
        fi
    done

    log_info "3/3 python3 -m py_compile over tools/**/*.py..."
    local py_files
    py_files=$(find "$PROJECT_ROOT/tools" -name '*.py' -not -path '*/__pycache__/*')
    if [ -n "$py_files" ] && ! echo "$py_files" | xargs python3 -m py_compile; then
        log_error "py_compile failed (see above)."
        failed=1
    fi

    if [ "$failed" -ne 0 ]; then
        log_error "Lint failed."
        return 1
    fi
    log_info "Lint passed."
}

# NOTE: `airstack format` was unregistered — it was a stub that only echoed.
# No formatter is configured for this repo yet; re-register a cmd_dev_format
# here (COMMANDS/COMMAND_HELP) once one is chosen.

# Register commands from this module
function register_dev_commands {
    COMMANDS["test"]="cmd_dev_test"
    COMMANDS["docs"]="cmd_dev_docs"
    COMMANDS["lint"]="cmd_dev_lint"

    # Add command help
    COMMAND_HELP["test"]="Run pytest in the containerized test runner (all args forward to pytest; see 'airstack help test')"
    COMMAND_HELP["docs"]="Build documentation (options: serve)"
    COMMAND_HELP["lint"]="Static checks: tests/meta contract suite, bash -n over the CLI, py_compile over tools/"
}