#!/usr/bin/env bash

# doctor.sh — `airstack doctor` (RFC #379 §4, Phase P5-E3).
#
# Observe-and-report health checks. Default mode runs the compose-time battery
# (module manifests, overlay integrity, dep-conflict HARD GATE, stack anatomy,
# bridge.yaml placement HARD GATE); --live diffs the running graph against the
# selected stack's committed wiring.md; --snapshot writes the stack's
# wiring.md from a real bring-up with unverified-in-CI provenance.
#
# All logic lives in tools/doctor/ (python3 + PyYAML); this file is only the
# CLI registration. Note `airstack module doctor` remains the module-scoped
# subset — `airstack doctor` is the whole-checkout battery.

DOCTOR_TOOL="${PROJECT_ROOT}/tools/doctor/__init__.py"

function cmd_doctor {
    if [ "${1:-}" = "help" ] || [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
        print_command_help doctor
        return 0
    fi
    if ! command -v python3 >/dev/null 2>&1; then
        log_error "python3 is required for 'airstack doctor'."
        return 1
    fi
    if ! python3 -c 'import yaml' 2>/dev/null; then
        log_error "PyYAML is required (pip3 install --user pyyaml)."
        return 1
    fi
    python3 "$DOCTOR_TOOL" --project-root "$PROJECT_ROOT" "$@"
}

# Register commands from this module.
function register_doctor_commands {
    COMMANDS["doctor"]="cmd_doctor"
    COMMAND_HELP["doctor"]="Observe-and-report health checks: compose-time gates, --live wiring drift, --snapshot (RFC #379 §4)"
}
