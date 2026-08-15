#!/usr/bin/env bash

# safe.sh - S.A.F.E. benchmark evaluation of the AirStack stack
# Thin CLI wrapper around eval/run.py so safety evaluation is a first-class
# AirStack command. All arguments forward to the S.A.F.E. runner, which
# owns the stack bring-up (`airstack up`) itself.

function cmd_safe {
    local python_bin="${SAFE_PYTHON:-python3}"

    if ! "$python_bin" -c "import safe_core" &> /dev/null; then
        log_error "safe_core not importable by '$python_bin'."
        log_info "Install the S.A.F.E. benchmark first:  pip install -e /path/to/benchmark"
        log_info "(or set SAFE_PYTHON to the interpreter that has it)"
        exit 1
    fi

    if [ $# -eq 0 ]; then
        log_info "S.A.F.E. evaluation of the AirStack stack. Examples:"
        log_info "  airstack safe --agent droan --domains empty --simple -y     # smoke test"
        log_info "  airstack safe --agents droan aggressive --domains warehouse -N 5 -P 5 -T 10"
        log_info "  airstack safe --help                                        # full flag reference"
        exit 0
    fi

    exec "$python_bin" "$PROJECT_ROOT/eval/run.py" "$@"
}

# Register commands from this module
function register_safe_commands {
    COMMANDS["safe"]="cmd_safe"
    COMMAND_HELP["safe"]="Run the S.A.F.E. safety benchmark against the stack (see eval/README.md)"
}
