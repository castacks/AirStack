#!/usr/bin/env bash

# fleet.sh — `airstack fleet` command group (RFC #380 §2, Phase P6).
#
# Manages fleet files (config/fleets/*.yaml): who exists, which body (vehicle),
# which brain (stack), and which hosts run each split's offboard half.
#
# Subcommands: list | generate | help
# Resolution logic lives in tools/fleet/resolve_fleet.py; compose generation in
# tools/fleet/generate_fleet_compose.py. Guide: docs/development/fleets.md

FLEETS_DIR="${PROJECT_ROOT}/config/fleets"
FLEET_RESOLVER="${PROJECT_ROOT}/tools/fleet/resolve_fleet.py"
FLEET_COMPOSE_GENERATOR="${PROJECT_ROOT}/tools/fleet/generate_fleet_compose.py"
FLEET_GENERATED_COMPOSE="${PROJECT_ROOT}/.airstack/generated/docker-compose.fleet.yaml"

function _fleet_check_python {
    if ! command -v python3 >/dev/null 2>&1; then
        log_error "python3 is required for 'airstack fleet' commands."
        return 1
    fi
    if ! python3 -c 'import yaml' 2>/dev/null; then
        log_error "PyYAML is required (pip3 install --user pyyaml)."
        return 1
    fi
}

# Resolve a fleet argument (name or path) to a host-side file path.
function _fleet_file_of {
    local ref="$1"
    if [[ "$ref" == *.yaml || "$ref" == */* ]]; then
        [[ "$ref" != /* ]] && ref="$PROJECT_ROOT/$ref"
        echo "$ref"
    else
        echo "$FLEETS_DIR/$ref.yaml"
    fi
}

# ── subcommands ──────────────────────────────────────────────────────────────

function cmd_fleet_list {
    _fleet_check_python || return 1
    if [ ! -d "$FLEETS_DIR" ]; then
        log_error "No fleets directory at ${FLEETS_DIR}."
        return 1
    fi
    FLEETS_DIR="$FLEETS_DIR" PROJECT_ROOT_ENV="$PROJECT_ROOT" \
        FLEET_RESOLVER="$FLEET_RESOLVER" python3 - <<'PY'
import importlib.util, os, sys

spec = importlib.util.spec_from_file_location("airstack_resolve_fleet", os.environ["FLEET_RESOLVER"])
rf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rf)

fleets_dir = os.environ["FLEETS_DIR"]
root = os.environ["PROJECT_ROOT_ENV"]

rows = []
for fname in sorted(os.listdir(fleets_dir)):
    if not fname.endswith(".yaml"):
        continue
    name = fname[: -len(".yaml")]
    path = os.path.join(fleets_dir, fname)
    try:
        fleet = rf.load_fleet(path)
        errors = rf.validate_fleet(fleet, root)
        if errors:
            rows.append((name, "?", "?", "?", f"INVALID: {errors[0][:60]}"))
            continue
        resolved = rf.resolve_fleet(fleet, root)
        robots = resolved["robots"]
        stacks = sorted({r["stack_ref"] for r in robots})
        vehicles = sorted({r["vehicle"] for r in robots})
        split = any(r["hosts"] for r in robots)
        homogeneous = rf.fleet_is_homogeneous(fleet, root)
        shape = "homogeneous" if homogeneous else "heterogeneous"
        if split:
            shape += "+split"
        rows.append((name, str(len(robots)), ",".join(vehicles), ",".join(stacks), shape))
    except rf.FleetError as exc:
        rows.append((name, "?", "?", "?", f"INVALID: {str(exc)[:60]}"))

if not rows:
    print("No fleet files under config/fleets/. See docs/development/fleets.md.")
    raise SystemExit(0)

headers = ("FLEET", "ROBOTS", "VEHICLES", "STACKS", "SHAPE")
widths = [max(len(str(r[i])) for r in rows + [headers]) for i in range(5)]
fmt = "  ".join("{:<%d}" % w for w in widths)
print(fmt.format(*headers))
for row in rows:
    print(fmt.format(*row))
PY
}

function cmd_fleet_generate {
    _fleet_check_python || return 1
    local ref="${1:-}"
    if [ -z "$ref" ] || [ $# -gt 1 ]; then
        log_error "Usage: airstack fleet generate <fleet>"
        log_error "  e.g. airstack fleet generate sim_three_mixed"
        return 1
    fi
    local fleet_file
    fleet_file="$(_fleet_file_of "$ref")"
    if [ ! -f "$fleet_file" ]; then
        log_error "Fleet not found: ${fleet_file}"
        cmd_fleet_list
        return 1
    fi
    python3 "$FLEET_COMPOSE_GENERATOR" "$fleet_file" --project-root "$PROJECT_ROOT"
}

# Dispatcher for the `fleet` command group.
function cmd_fleet_dispatch {
    local sub="${1:-help}"
    if [ $# -gt 0 ]; then shift; fi
    case "$sub" in
        list)     cmd_fleet_list "$@" ;;
        generate) cmd_fleet_generate "$@" ;;
        help|-h|--help) print_command_help fleet ;;
        *)
            log_error "Unknown fleet subcommand: '$sub'"
            print_command_help fleet
            return 1
            ;;
    esac
}

# Register commands from this module.
function register_fleet_commands {
    COMMANDS["fleet"]="cmd_fleet_dispatch"
    COMMAND_HELP["fleet"]="Manage fleet files: list|generate (RFC #380 §2; see 'airstack help fleet')"
}
