#!/usr/bin/env bash

# airstack - A convenience tool for AirStack development
# 
# This script provides a unified interface for common development tasks
# in the AirStack project, including setup, installation, and container management.

# Re-exec under bash 4+ if necessary. macOS ships bash 3.2 which can't handle
# `declare -A` (associative arrays) used throughout this script. Searches for
# a newer bash via $AIRSTACK_BASH, then common Homebrew install paths, then
# any `bash` on PATH that reports version >= 4. Sets AIRSTACK_REEXEC_BASH=1
# to guard against infinite re-exec loops.
if [ -z "${AIRSTACK_REEXEC_BASH:-}" ] && [ "${BASH_VERSINFO[0]:-0}" -lt 4 ]; then
    _airstack_candidates=(
        "${AIRSTACK_BASH:-}"
        /opt/homebrew/bin/bash      # Apple Silicon Homebrew
        /usr/local/bin/bash         # Intel Homebrew
        /opt/local/bin/bash         # MacPorts
    )
    if command -v bash5 >/dev/null 2>&1; then
        _airstack_candidates+=("$(command -v bash5)")
    fi
    # Add any `bash` on PATH whose version is >= 4 (other than the one we just
    # got here from, which is < 4 by the if-check above).
    for _alt in $(command -v -a bash 2>/dev/null); do
        _airstack_candidates+=("$_alt")
    done

    for _airstack_alt_bash in "${_airstack_candidates[@]}"; do
        [ -z "$_airstack_alt_bash" ] && continue
        [ -x "$_airstack_alt_bash" ] || continue
        # Probe BASH_VERSINFO[0] without sourcing the script.
        if "$_airstack_alt_bash" -c '[ "${BASH_VERSINFO[0]:-0}" -ge 4 ]' 2>/dev/null; then
            export AIRSTACK_REEXEC_BASH=1
            exec "$_airstack_alt_bash" "$0" "$@"
        fi
    done

    cat >&2 <<'EOF'
[ERROR] airstack.sh requires bash 4 or newer (your bash is 3.x).
        macOS ships bash 3.2 by default; install a modern bash with:
            brew install bash
        Or set AIRSTACK_BASH=/path/to/bash >= 4 before invoking this script.
EOF
    exit 1
fi
unset AIRSTACK_REEXEC_BASH

set -e

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

# Directory for command modules
MODULES_DIR="$PROJECT_ROOT/.airstack/modules"

# Create modules directory if it doesn't exist
mkdir -p "$MODULES_DIR"

# Shared helpers (_require_python_yaml, _env_value, _robot_containers,
# _container_identity) — sourced explicitly BEFORE load_command_modules so
# both built-in commands and every command module can rely on them.
if [ -f "$MODULES_DIR/_lib.sh" ]; then
    source "$MODULES_DIR/_lib.sh"
fi

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
            echo "Launch-intent options (consumed by airstack; they derive + export env vars):"
            echo "  --sim isaac|airsim|simple"
            echo "                      Select the simulator: swaps the compose profile and"
            echo "                      the matching URDF_FILE. 'simple' launches the"
            echo "                      lightweight kinematic simple-sim (no PX4/MAVROS):"
            echo "                      the simple-robot service replaces robot-desktop"
            echo "                      (SIM_TYPE=simple), the 'desktop' profile is dropped"
            echo "                      (no GCS), and URDF_FILE is left unchanged."
            echo "  --robots N          Robot count (NUM_ROBOTS); on Isaac also selects the"
            echo "                      matching one-/multi-drone launch script."
            echo "  --stack NAME        Launch a stack folder (stacks/NAME/launch/stack.launch.xml)."
            echo "                      Stacks are the only launch dispatch; no --stack (and no stack"
            echo "                      env) launches the trunk reference stack full_default."
            echo "                      NAME:ENTRY selects an alternate entry file"
            echo "                      (launch/ENTRY.launch.xml). See docs/development/stacks.md."
            echo "  --fleet NAME        Launch a fleet (config/fleets/NAME.yaml): exports FLEET_CONFIG_FILE,"
            echo "                      derives NUM_ROBOTS, selects the Isaac fleet spawner, and (for"
            echo "                      heterogeneous fleets) generates the per-robot services plus"
            echo "                      split-stack DDS-router configs. Mutually exclusive with"
            echo "                      --robots. See docs/development/fleets.md."
            echo "  --headless          Run the sim without a GUI (ISAAC_SIM_HEADLESS/MS_AIRSIM_HEADLESS)."
            echo "  --play / --no-play  Start the sim playing, or paused (PLAY_SIM_ON_START)."
            echo "  --no-autolaunch     Start containers idle (AUTOLAUNCH=false; launch manually)."
            echo "  --wait              After starting, block until flight-ready (airstack ready)."
            echo "  --dry-run           Validate + print the derived launch config; start nothing."
            echo ""
            echo "Anything else (e.g. --build, service names) is passed through to"
            echo "'docker compose up'."
            ;;
        fleet)
            echo "Usage: airstack fleet <subcommand>"
            echo ""
            echo "Manage fleet files (RFC #380 §2): config/fleets/*.yaml declare who"
            echo "exists, which vehicle (body), which stack (brain), and which ground"
            echo "hosts run each split stack's offboard half."
            echo "Guide: docs/development/fleets.md"
            echo ""
            echo "Subcommands:"
            echo "  list             Table of fleets: robots, vehicles, stacks, shape"
            echo "                   (homogeneous / heterogeneous / +split)."
            echo "  generate <fleet> Write .airstack/generated/docker-compose.fleet.yaml:"
            echo "                   one self-contained service per robot plus one per"
            echo "                   (ground host x offboard tenant) — AND the DDS-router"
            echo "                   configs for every split stack the fleet places"
            echo "                   (.airstack/generated/dds_router.<stack>.yaml)."
            echo "                   Homogeneous fleets need no generation"
            echo "                   (deploy.replicas handles them) — the command says so"
            echo "                   and writes nothing."
            echo ""
            echo "Run a fleet:  airstack up --fleet <name> [--sim isaac|airsim]"
            ;;
        sync)
            echo "Usage: airstack sync [--no-hooks]"
            echo ""
            echo "Options:"
            echo "  --no-hooks    Skip module host_setup hooks during the module sync step"
            echo ""
            echo "Sync the checkout from airstack.yaml (RFC #380 §3): upsert its"
            echo "modules: additions into modules.repos (naming every deviation), run"
            echo "the module sync, fetch declared external stack repos into gitignored"
            echo "stacks/.external/<alias>/ (fleets address them as <alias>/<stack>),"
            echo "validate the declared fleet, warn if release: has drifted off the"
            echo ".env VERSION line, and record the resolved sources in"
            echo ".airstack/generated/effective_sources.yaml."
            echo ""
            echo "NOT done (future work): rewriting .env (it stays hand-edited) and"
            echo "release-set pin resolution against a registry (RFC #379 §7)."
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
            echo "Remove Docker images whose repository:tag contains the search term"
            echo "(fixed-string match; lists the matches and asks before removing)."
            echo ""
            echo "Options:"
            echo "  -f, --force   Force removal (docker rmi -f)"
            echo "  -y, --yes     Skip the confirmation prompt"
            echo ""
            echo "Examples:"
            echo "  airstack rmi myimage"
            echo "  airstack rmi -f -y myimage"
            ;;
        ready)
            echo "Usage: airstack ready [--json]"
            echo ""
            echo "Wait until the running stack is flight-ready, gate by gate:"
            echo "  1. robot containers running"
            echo "  2. sim publishing /clock"
            echo "  3. sentinel autonomy nodes up (per robot)"
            echo "  4. PX4 ready: MAVROS connected, then EKF odometry streaming"
            echo ""
            echo "Options:"
            echo "  --json    Print a JSON verdict on stdout (progress goes to stderr):"
            echo "            {\"ready\": bool, \"elapsed_s\": N, \"gates\": {...}}"
            echo ""
            echo "Budgets are env-overridable: READY_CONTAINERS_TIMEOUT, READY_CLOCK_TIMEOUT,"
            echo "READY_NODES_TIMEOUT, READY_PX4_TIMEOUT, READY_POLL_INTERVAL."
            ;;
        test)
            echo "Usage: airstack test [pytest options]"
            echo ""
            echo "Build the containerized test runner (tests/docker/) and run pytest"
            echo "inside it. All arguments are forwarded directly to pytest."
            echo "Results are written to tests/results/<timestamp>/."
            echo ""
            echo "Test marks (-m) — see tests/pytest.ini:"
            echo "  unit            Fast hermetic tests (no Docker stack)"
            echo "  build_docker    Docker image build tests (no GPU needed)"
            echo "  build_packages  colcon workspace build tests (no GPU needed)"
            echo "  integration     Robot container + a host-side component (no sim/GPU)"
            echo "  liveliness      Container and process health (Docker, tmux, sentinel nodes)"
            echo "  wiring          Observed wiring snapshot, drift-checked vs the committed golden"
            echo "  sensors         Sim and robot sensor topic rates, LiDAR validation, sim RTF"
            echo "  takeoff_hover_land  Takeoff / hover / land flight chain"
            echo "  autonomy        Fixed-pattern trajectory path-tracker benchmark"
            echo "  waypoint_flight Ordered-waypoint navigation judged on the odometry track"
            echo "  simple_sim      Simple-sim smoke test (containers, /clock, sentinel nodes;"
            echo "                  run with --sim simplesim --num-robots 1)"
            echo "  optitrack       OptiTrack NatNet end-to-end (tests live in the asm_optitrack module)"
            echo ""
            echo "AirStack-specific options (defaults from tests/conftest.py):"
            echo "  --sim=TARGETS              Comma-separated sim targets: isaacsim, msairsim,"
            echo "                             simplesim (default: isaacsim)"
            echo "  --num-robots=COUNTS        Comma-separated robot counts (default: 1,3)"
            echo "  --stack=NAME               Stack folder under stacks/ to launch (default:"
            echo "                             the full_default dispatch)"
            echo "  --fleet=NAME               Fleet preset under config/fleets/ (derives the"
            echo "                             robot count, overriding --num-robots)"
            echo "  --stress-iterations=N      Up/down cycles per config (default: 1)"
            echo "  --stable-duration=SECS     Seconds test_stable polls for (default: 120)"
            echo "  --stable-interval=SECS     Seconds between polls (default: 10)"
            echo "  --takeoff-velocities=LIST  Comma-separated takeoff/land speeds in m/s"
            echo "                             (default: 0.5)"
            echo "  --trajectory-types=LIST    Fixed trajectory types for the autonomy mark"
            echo "                             (default: Circle,Figure8,Racetrack,Line)"
            echo "  --waypoints='x,y,z; ...'   Ordered waypoint route for waypoint_flight"
            echo "                             (default: open 30 m square, 3 corners, +10 m)"
            echo "  --waypoint-tolerance=M     Pass distance to intermediate waypoints (default: 15)"
            echo "  --goal-tolerance=M         Pass distance to the final waypoint (default: 2.5)"
            echo "  --waypoint-timeout=SECS    Per-waypoint time budget (default: 120)"
            echo "  --gui                      Show simulator GUI (default: headless)"
            echo "  --no-image-build           CI flag for system-tests.yml (ignored by pytest)"
            echo ""
            echo "Examples:"
            echo "  # Build tests only — fast, no GPU needed"
            echo "  airstack test -m 'build_docker or build_packages' -v"
            echo ""
            echo "  # Liveliness run — ms-airsim, 1 robot, 60 s stability window"
            echo "  airstack test -m liveliness --sim msairsim --num-robots 1 \\"
            echo "    --stress-iterations 1 --stable-duration 60 -v"
            echo ""
            echo "  # Takeoff/hover/land run — at 0.5, 1, and 2 m/s"
            echo "  airstack test -m takeoff_hover_land --sim msairsim --num-robots 1 \\"
            echo "    --stress-iterations 1 --takeoff-velocities 0.5,1,2 -v"
            ;;
        docs)
            echo "Usage: airstack docs [serve]"
            echo ""
            echo "Build documentation. If 'serve' is specified, also start a local server."
            ;;
        module)
            echo "Usage: airstack module <subcommand> [options]"
            echo ""
            echo "Manage AirStack modules (RFC #379): thin external repos declared in"
            echo "./modules.repos (vcs2l format, PINNED to tags/SHAs — never branches),"
            echo "synced into the gitignored ./modules/ dir and overlaid into the checkout"
            echo "by tools/module_overlay.py. Guide: docs/development/modules.md"
            echo ""
            echo "Subcommands:"
            echo "  add <git-url|local-path> [--version <tag-or-sha>] [--no-hooks]"
            echo "                Record a module in modules.repos and sync. Git URLs MUST be"
            echo "                pinned with --version (branch names like main/develop are"
            echo "                refused). Local paths are recorded under x-local-modules."
            echo "  sync [--no-hooks]"
            echo "                Clone/link everything in modules.repos (vcs import --recursive),"
            echo "                validate each module.yaml, place overlay symlinks, regenerate"
            echo "                .airstack/generated/docker-compose.modules.yaml, run host_setup"
            echo "                hooks (skipped with --no-hooks)."
            echo "  list          Table of modules: name, type, version/pin, targets, valid."
            echo "  remove <name> Drop the modules.repos entry, checkout, and overlay artifacts."
            echo "  create --in-tree <name>"
            echo "                Scaffold robot/ros_ws/src/modules/<name>/ (stub module.yaml,"
            echo "                ament_python package, launch file with canonical-default topic"
            echo "                args and no remaps, test/) for fork research (RFC #379 §11)."
            echo "  lock [--check-conflicts] [--build]"
            echo "                Recompute the Docker layer plan + modules.lock (RFC #379 §6:"
            echo "                .airstack/generated/layer_plan.json); extra flags pass through"
            echo "                to tools/compose_module_layers.py."
            echo "  doctor [--drift]"
            echo "                No args: validate all manifests + overlay integrity (exit 1"
            echo "                only when the overlay is broken). --drift: classify changes"
            echo "                vs merge-base with origin/develop into module-contained vs"
            echo "                extraction debt — informs, never blocks (always exit 0)."
            echo ""
            echo "After sync, just start the stack: 'airstack up' includes the generated"
            echo "module mounts automatically (opt out with AIRSTACK_NO_MODULE_COMPOSE=1)."
            ;;
        stack)
            echo "Usage: airstack stack <subcommand> [options]"
            echo ""
            echo "Manage stack folders (RFC #379 §3): self-contained topologies under"
            echo "stacks/ — pinned modules.repos, launch/ entry points (one per host"
            echo "role for split stacks, plus bridge.yaml), docker-compose stub, and a"
            echo "generated wiring.md. Guide: docs/development/stacks.md"
            echo ""
            echo "Subcommands:"
            echo "  list          Table of stacks: name, entry points (+bridge marker for"
            echo "                split stacks), wiring.md present?, airstack_compat."
            echo "  new <source> <dest>"
            echo "                Copy a reference stack to stacks/<dest>. Refuses to"
            echo "                overwrite; deliberately does NOT copy wiring.md (it is"
            echo "                the source's observed graph) — regenerate it with"
            echo "                'airstack test -m wiring --stack <dest>'."
            echo "  diff <a> <b> [--json]"
            echo "                Compare two stacks' generated wiring.md graphs"
            echo "                (tools/stack_diff.py over the wiring-graph trailers):"
            echo "                nodes/edges/topics added or removed, QoS and type"
            echo "                mismatches — topology differences, not XML noise."
            echo ""
            echo "Run a stack:      airstack up --stack <name>[:<entry>]"
            echo "Check a checkout: airstack doctor"
            ;;
        doctor)
            echo "Usage: airstack doctor [--live | --snapshot] [--stack NAME] [--strict]"
            echo ""
            echo "Observe-and-report health checks (RFC #379 §4). Doctor never edits"
            echo "anything and exits non-zero in default mode only on the two enumerated"
            echo "hard gates."
            echo ""
            echo "Default (compose-time) checks, in order:"
            echo "  1. module manifests valid            (tools/validate_module.py)"
            echo "  2. overlay integrity                 (module_overlay.py --check)"
            echo "  3. HARD GATE #1: module dep conflicts (compose_module_layers.py"
            echo "     --check-conflicts — a conflict would compose a broken image)"
            echo "  4. stack folder anatomy (incl. split stack => bridge.yaml required)"
            echo "  5. HARD GATE #2: control-setpoint / trajectory-group names in any"
            echo "     bridge.yaml (gen_dds_router.py --check — command authority stays"
            echo "     onboard, RFC #380 §2)"
            echo ""
            echo "Modes:"
            echo "  --live        Capture the RUNNING graph (docker exec per robot) and"
            echo "                diff it against the stack's committed wiring.md; exit 1"
            echo "                on drift. Also scans for publishers of control-setpoint"
            echo "                / trajectory-group topics outside the blessed controller"
            echo "                chain (reported loudly; fatal only with --strict)."
            echo "  --snapshot    Same capture, WRITTEN to stacks/<name>/wiring.md with a"
            echo "                provenance line ('observed on <host>, <date>, <sha> —"
            echo "                unverified-in-CI') — the hardware bring-up path."
            echo "  --stack NAME  Stack to diff/snapshot (default: inferred from"
            echo "                AIRSTACK_STACK_DIR as exported by 'airstack up ...stack')."
            echo "  --strict      Make --live safety-floor warnings fatal."
            echo ""
            echo "Module-scoped checks only: airstack module doctor [--drift]"
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

# Get VERSION from .env file (parsing shared with cmd_version via _env_value)
function get_VERSION {
    local version
    version=$(_env_value VERSION "$PROJECT_ROOT/.env")

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

# Wrapper function to run docker compose natively on the host.
# The base compose file ($PROJECT_ROOT/docker-compose.yaml) is folded in here —
# callers pass only ADDITIONAL -f overlay files (module/fleet overlays), never
# the base one. (The tests runner uses its own separate compose file and calls
# `docker compose` directly — see .airstack/modules/dev.sh.)
function run_docker_compose {
    local env_args=()
    local env_file="$PROJECT_ROOT/.env"

    if [ -f "$env_file" ]; then
        env_args+=("--env-file" "$env_file")
    fi

    docker compose "${env_args[@]}" -f "$PROJECT_ROOT/docker-compose.yaml" "$@"
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
            log_info "Operation canceled" >&2
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
                log_info "Operation canceled" >&2
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

    for arg in "$@"; do
        if [ "$arg" == "--force" ]; then
            force=true
        elif [ "$arg" == "--no-docker" ]; then
            install_docker=false
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

    # Sync modules (RFC #379): module hooks (hooks.host_setup) cover host-side
    # setup steps like proprietary SDK downloads, so setup just re-runs sync.
    if [ -f "$PROJECT_ROOT/modules.repos" ]; then
        if declare -f "cmd_module_dispatch" > /dev/null; then
            log_info "Syncing modules from modules.repos..."
            cmd_module_dispatch sync || log_warn "Module sync failed — run 'airstack module sync' manually."
        else
            log_warn "Module command module not loaded — skipping module sync. Run 'airstack module sync' manually."
        fi
    fi

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
                log_error "Missing value for compose global flag: $arg"
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

# robot-l4t uses Dockerfile.robot with BASE_IMAGE=robot-l4t-stack-base. Compose v2+ may schedule
# service builds in parallel, so BUILD FROM that tag can race before stack-base finishes. Ensure
# the intermediary image exists first (still requires --profile l4t like the robot-l4t build).
function ensure_robot_l4t_stack_base() {
    local -n _ga="$1"
    local -n _sc="$2"
    local wants_l4t=false
    local has_stack=false
    for arg in "${_sc[@]}"; do
        if [[ "$arg" == robot-l4t ]]; then
            wants_l4t=true
        fi
        if [[ "$arg" == robot-l4t-stack-base ]]; then
            has_stack=true
        fi
    done
    if [[ "$wants_l4t" != true ]] || [[ "$has_stack" == true ]]; then
        return 0
    fi
    log_info "Building robot-l4t-stack-base before robot-l4t..."
    local build_opts=()
    for arg in "${_sc[@]}"; do
        if [[ "$arg" == -* ]]; then
            build_opts+=("$arg")
        fi
    done
    run_docker_compose "${_ga[@]}" build "${build_opts[@]}" robot-l4t-stack-base
}

# `docker compose push` only publishes each service's `image:`. The floating
# cache tags are declared in `build.tags`, so they need an explicit push. Read
# them back out of the resolved config instead of reconstructing the names here,
# so this stays correct as services are added.
function push_cache_tags() {
    local -n _ga="$1"
    local -n _sc="$2"

    if ! command -v jq >/dev/null 2>&1; then
        log_warn "jq not found; skipping cache-tag push (floating cache will go stale)"
        return 0
    fi

    # Service names only — drop any flags that were passed through to the subcommand.
    local services=()
    for arg in "${_sc[@]}"; do
        [[ "$arg" == -* ]] || services+=("$arg")
    done

    local tags
    tags=$(run_docker_compose "${_ga[@]}" config --format json 2>/dev/null \
        | jq -r --arg svcs "${services[*]}" --arg pfx ":${CACHE_TAG:-cache}_" '
            .services | to_entries[]
            | select($svcs == "" or (($svcs | split(" ")) | index(.key)))
            | (.value.build.tags // [])[]
            | select(contains($pfx))
          ' 2>/dev/null | sort -u)

    if [[ -z "$tags" ]]; then
        log_warn "No cache tags resolved from compose config; nothing to publish"
        return 0
    fi

    while IFS= read -r tag; do
        [[ -n "$tag" ]] || continue
        log_info "Pushing cache tag $tag"
        docker push "$tag" || log_warn "Failed to push cache tag $tag"
    done <<< "$tags"
}

# ---------------------------------------------------------------------------
# Launch intent: airstack-specific flags on `airstack up`
# (--sim isaac|airsim|simple, --robots N, --headless, --play/--no-play,
#  --no-autolaunch, --wait, --dry-run)
#
# Flags derive and EXPORT env vars; docker compose interpolation gives shell
# env highest precedence, so exports win over .env / --env-file without
# editing any file. Flags set leaf values only — they select among declared
# configurations and never define new structure (RFC #380 §4 discipline).
# ---------------------------------------------------------------------------

# (env-file value parsing lives in .airstack/modules/_lib.sh: _env_value)

# Resolve NAME the way docker compose interpolation will: OS environment wins,
# then user --env-file files (later wins), then .env. Extra args are scanned
# for --env-file tokens.
function resolve_launch_var {
    local name="$1"; shift
    local val
    val=$(_env_value "$name" "$PROJECT_ROOT/.env")
    local args=("$@") i=0 file
    while [ $i -lt ${#args[@]} ]; do
        file=""
        if [[ "${args[$i]}" == "--env-file" ]]; then
            i=$((i+1)); file="${args[$i]:-}"
        elif [[ "${args[$i]}" == --env-file=* ]]; then
            file="${args[$i]#--env-file=}"
        fi
        if [[ -n "$file" ]]; then
            [[ "$file" != /* ]] && file="$PROJECT_ROOT/$file"
            local v; v=$(_env_value "$name" "$file")
            [[ -n "$v" ]] && val="$v"
        fi
        i=$((i+1))
    done
    if [[ -n "${!name:-}" ]]; then val="${!name}"; fi
    echo "$val"
}

# Consume intent flags from the arg list; remaining args land in the nameref
# array. Sets AIRSTACK_INTENT_* globals for apply_launch_intent.
function parse_launch_intent {
    local -n _rest_out=$1; shift
    AIRSTACK_INTENT_SIM=""
    AIRSTACK_INTENT_ROBOTS=""
    AIRSTACK_INTENT_HEADLESS=""
    AIRSTACK_INTENT_PLAY=""
    AIRSTACK_INTENT_AUTOLAUNCH=""
    AIRSTACK_INTENT_STACK=""
    AIRSTACK_INTENT_FLEET=""
    AIRSTACK_FLEET_COMPOSE_FILE=""
    AIRSTACK_DRY_RUN=""
    AIRSTACK_UP_WAIT=""

    local args=("$@") i=0 a
    while [ $i -lt ${#args[@]} ]; do
        a="${args[$i]}"
        case "$a" in
            --sim)          i=$((i+1)); AIRSTACK_INTENT_SIM="${args[$i]:-}";;
            --sim=*)        AIRSTACK_INTENT_SIM="${a#--sim=}";;
            --robots)       i=$((i+1)); AIRSTACK_INTENT_ROBOTS="${args[$i]:-}";;
            --robots=*)     AIRSTACK_INTENT_ROBOTS="${a#--robots=}";;
            --headless)     AIRSTACK_INTENT_HEADLESS="true";;
            --play)         AIRSTACK_INTENT_PLAY="true";;
            --no-play)      AIRSTACK_INTENT_PLAY="false";;
            --no-autolaunch) AIRSTACK_INTENT_AUTOLAUNCH="false";;
            --stack)        i=$((i+1)); AIRSTACK_INTENT_STACK="${args[$i]:-}";;
            --stack=*)      AIRSTACK_INTENT_STACK="${a#--stack=}";;
            --fleet)        i=$((i+1)); AIRSTACK_INTENT_FLEET="${args[$i]:-}";;
            --fleet=*)      AIRSTACK_INTENT_FLEET="${a#--fleet=}";;
            --wait)         AIRSTACK_UP_WAIT="1";;
            # NOTE: shadows compose's own `up --dry-run`; ours validates the
            # derived launch config and exits without starting services.
            --dry-run)      AIRSTACK_DRY_RUN="1";;
            *)              _rest_out+=("$a");;
        esac
        i=$((i+1))
    done

    if [[ -n "$AIRSTACK_INTENT_ROBOTS" && ! "$AIRSTACK_INTENT_ROBOTS" =~ ^[1-9][0-9]*$ ]]; then
        log_error "--robots must be a positive integer (got '$AIRSTACK_INTENT_ROBOTS')"
        return 1
    fi
    case "$AIRSTACK_INTENT_SIM" in
        ""|isaac|isaacsim|airsim|msairsim|ms-airsim|simple|simplesim|simple-sim) ;;
        *) log_error "Unknown --sim '$AIRSTACK_INTENT_SIM' (expected: isaac | airsim | simple)"; return 1;;
    esac
    if [[ -n "$AIRSTACK_INTENT_FLEET" && -n "$AIRSTACK_INTENT_ROBOTS" ]]; then
        log_error "--fleet and --robots are mutually exclusive: the fleet file defines the robot count (RFC #380 §2)."
        return 1
    fi
    return 0
}

# Validate a --stack selection against the host stacks/ tree and export the
# CONTAINER paths the launch dispatch reads (stacks/ is bind-mounted at
# /root/AirStack/stacks by robot-base-docker-compose.yaml). Accepts the
# split-entry form `<name>:<entry>` (entry names launch/<entry>.launch.xml;
# reserved for split stacks — RFC #380 §2). Default entry: stack.
function apply_stack_intent {
    local stack_name="$AIRSTACK_INTENT_STACK" stack_entry="stack"
    if [[ "$stack_name" == *:* ]]; then
        stack_entry="${stack_name#*:}"
        stack_name="${stack_name%%:*}"
    fi
    if [[ -z "$stack_name" || -z "$stack_entry" ]]; then
        log_error "--stack requires a stack name (got '$AIRSTACK_INTENT_STACK'; expected <name> or <name>:<entry>)"
        return 1
    fi
    local stack_host_dir="$PROJECT_ROOT/stacks/$stack_name"
    if [[ ! -d "$stack_host_dir" ]]; then
        local available
        available=$(ls -1 "$PROJECT_ROOT/stacks" 2>/dev/null | grep -v '^\.' | tr '\n' ' ')
        log_error "Unknown stack '$stack_name' — $stack_host_dir does not exist. Available stacks: ${available:-<none>}"
        return 1
    fi
    if [[ ! -f "$stack_host_dir/launch/$stack_entry.launch.xml" ]]; then
        log_error "Stack '$stack_name' has no entry point launch/$stack_entry.launch.xml (expected $stack_host_dir/launch/$stack_entry.launch.xml)"
        return 1
    fi
    export AIRSTACK_STACK_DIR="/root/AirStack/stacks/$stack_name"
    export AIRSTACK_STACK_ENTRY="$stack_entry"
    return 0
}

# Fleet dispatch (RFC #380 §2): validate the fleet file, export
# FLEET_CONFIG_FILE (the CONTAINER path — config/ is bind-mounted at
# /root/AirStack/config) plus the derived NUM_ROBOTS, switch an
# untouched-default Isaac script to the generic fleet spawner, and — for
# HETEROGENEOUS fleets — regenerate the per-robot compose services and swap
# the desktop profile for the generated services' `fleet` profile (so
# deploy.replicas doesn't double-spawn identical containers next to them).
#
# Everything is leaf-value precedence: explicit OS-env NUM_ROBOTS /
# ISAAC_SIM_SCRIPT_NAME still win, with an override banner.
#
# Arg 1: fleet selector — a name under config/fleets/, a host path, or the
# container path form (as the test harness passes via FLEET_CONFIG_FILE).
# Remaining args: passed through to resolve_launch_var (--env-file scanning).
function apply_fleet_intent {
    local fleet_ref="$1"; shift
    fleet_ref="${fleet_ref#/root/AirStack/}"   # container path → checkout-relative
    local fleet_host
    if [[ "$fleet_ref" == *.yaml || "$fleet_ref" == */* ]]; then
        fleet_host="$fleet_ref"
        [[ "$fleet_host" != /* ]] && fleet_host="$PROJECT_ROOT/$fleet_ref"
    else
        fleet_host="$PROJECT_ROOT/config/fleets/$fleet_ref.yaml"
    fi
    if [[ ! -f "$fleet_host" ]]; then
        local available
        available=$(ls -1 "$PROJECT_ROOT/config/fleets" 2>/dev/null | sed 's/\.yaml$//' | tr '\n' ' ')
        log_error "Unknown fleet '$fleet_ref' — $fleet_host does not exist. Available fleets: ${available:-<none>}"
        return 1
    fi
    local fleet_name
    fleet_name="$(basename "$fleet_host" .yaml)"

    if ! python3 "$PROJECT_ROOT/tools/fleet/resolve_fleet.py" "$fleet_host" \
            --project-root "$PROJECT_ROOT" --validate >/dev/null; then
        log_error "Fleet '$fleet_name' failed validation (named errors above)."
        return 1
    fi

    local fleet_rel="${fleet_host#$PROJECT_ROOT/}"
    export FLEET_CONFIG_FILE="/root/AirStack/$fleet_rel"

    # NUM_ROBOTS is implicit in the fleet (RFC #380 §3 migration table).
    local fleet_robots
    fleet_robots=$(FLEET_HOST="$fleet_host" python3 -c '
import os, yaml
with open(os.environ["FLEET_HOST"], encoding="utf-8") as f:
    print(len((yaml.safe_load(f) or {}).get("robots") or {}))')
    if [[ -n "${NUM_ROBOTS:-}" && "$NUM_ROBOTS" != "$fleet_robots" ]]; then
        log_warn "OVERRIDE: explicit NUM_ROBOTS=$NUM_ROBOTS wins over fleet '$fleet_name' ($fleet_robots robot(s)) — containers and sim spawns will disagree with the fleet file."
    else
        export NUM_ROBOTS="$fleet_robots"
    fi

    # Isaac: the fleet spawner reads spawns/vehicles/scene from the fleet file,
    # replacing the hardcoded one-/multi-drone example scripts. Explicit OS-env
    # ISAAC_SIM_SCRIPT_NAME still wins; the stock defaults get switched.
    local profiles script
    profiles=$(resolve_launch_var COMPOSE_PROFILES "$@")
    if [[ ",$profiles," == *",isaac-sim,"* ]]; then
        if [[ -n "${ISAAC_SIM_SCRIPT_NAME:-}" ]]; then
            [[ "$ISAAC_SIM_SCRIPT_NAME" != "fleet_spawn.py" ]] && \
                log_warn "OVERRIDE: explicit ISAAC_SIM_SCRIPT_NAME='$ISAAC_SIM_SCRIPT_NAME' wins over the fleet spawner — make sure it spawns fleet '$fleet_name' (reads FLEET_CONFIG_FILE)."
        else
            script=$(resolve_launch_var ISAAC_SIM_SCRIPT_NAME "$@")
            case "$script" in
                example_one_px4_pegasus_launch_script.py|example_multi_px4_pegasus_launch_script.py|"")
                    log_info "--fleet $fleet_name → ISAAC_SIM_SCRIPT_NAME=fleet_spawn.py (was ${script:-<unset>})"
                    export ISAAC_SIM_SCRIPT_NAME="fleet_spawn.py";;
                fleet_spawn.py) ;;
                *)
                    log_warn "Custom ISAAC_SIM_SCRIPT_NAME='$script' with a fleet: make sure it spawns fleet '$fleet_name' (reads FLEET_CONFIG_FILE).";;
            esac
        fi
    fi

    # Heterogeneous fleet → deploy.replicas cannot stamp it: regenerate the
    # per-robot services AND the split stacks' bridge-derived DDS-router
    # configs (one pipeline: tools/fleet/generate_fleet_compose.py does both
    # and prints what it generated), then include the compose file (cmd_up
    # adds the -f); homogeneous fleets keep replicas untouched. On --dry-run
    # nothing is written — the generator prints what it WOULD generate.
    AIRSTACK_FLEET_COMPOSE_FILE=""
    local shape
    shape=$(python3 "$PROJECT_ROOT/tools/fleet/generate_fleet_compose.py" \
        "$fleet_host" --project-root "$PROJECT_ROOT" --check-homogeneous) || return 1
    if [[ "$shape" == "heterogeneous" ]]; then
        local gen_flags=()
        [[ "${AIRSTACK_DRY_RUN:-}" == "1" ]] && gen_flags+=(--dry-run)
        python3 "$PROJECT_ROOT/tools/fleet/generate_fleet_compose.py" \
            "$fleet_host" --project-root "$PROJECT_ROOT" "${gen_flags[@]}" || return 1
        if [[ "${AIRSTACK_DRY_RUN:-}" != "1" ]]; then
            AIRSTACK_FLEET_COMPOSE_FILE="$PROJECT_ROOT/.airstack/generated/docker-compose.fleet.yaml"
        fi
        local cur kept=() p
        cur=$(resolve_launch_var COMPOSE_PROFILES "$@")
        IFS=',' read -ra _fparr <<< "$cur"
        for p in "${_fparr[@]}"; do
            case "$p" in desktop|fleet|"") ;; *) kept+=("$p");; esac
        done
        kept+=("fleet")
        export COMPOSE_PROFILES=$(IFS=','; echo "${kept[*]}")
        if [[ "${AIRSTACK_DRY_RUN:-}" == "1" ]]; then
            log_info "--fleet $fleet_name is heterogeneous → would generate per-robot services (profile 'fleet' replaces 'desktop')"
        else
            log_info "--fleet $fleet_name is heterogeneous → generated per-robot services (profile 'fleet' replaces 'desktop')"
        fi
    fi
    return 0
}

# Derive + export env vars from the parsed intent. Args: remaining CLI args
# (scanned for --env-file when resolving current values).
function apply_launch_intent {
    if [[ -n "$AIRSTACK_INTENT_STACK" ]]; then
        apply_stack_intent || return 1
    fi

    # Stacks are the ONLY launch dispatch (the legacy AUTONOMY_ROLE dispatch
    # was removed): no stack selected anywhere (--stack / env / --env-file /
    # .env) = the trunk reference stack full_default. Exported here so the
    # effective config always names the stack that will actually launch.
    # (Fleet runs re-resolve per container: robot/docker/.bashrc overrides
    # these from the fleet file when FLEET_CONFIG_FILE is set.)
    if [[ -z "$(resolve_launch_var AIRSTACK_STACK_DIR "$@")" ]]; then
        export AIRSTACK_STACK_DIR="/root/AirStack/stacks/full_default"
    fi
    if [[ -z "$(resolve_launch_var AIRSTACK_STACK_ENTRY "$@")" ]]; then
        export AIRSTACK_STACK_ENTRY="stack"
    fi

    if [[ -n "$AIRSTACK_INTENT_SIM" ]]; then
        local sim_profile urdf=""
        case "$AIRSTACK_INTENT_SIM" in
            isaac|isaacsim)
                sim_profile="isaac-sim"
                urdf="robot_descriptions/iris/urdf/iris_with_sensors.pegasus.robot.urdf";;
            airsim|msairsim|ms-airsim)
                sim_profile="ms-airsim"
                urdf="robot_descriptions/iris/urdf/iris_stereo.ms-airsim.urdf";;
            simple|simplesim|simple-sim)
                # simple-sim (lightweight kinematic sim, no PX4/MAVROS): the
                # simple-robot service (SIM_TYPE=simple) IS the robot container,
                # so the 'desktop' profile must go too — keeping it would start
                # robot-desktop alongside simple-robot: two robot_1 graphs on
                # domain 1. No URDF swap: the sim publishes its own camera TFs
                # and doesn't consume URDF_FILE.
                sim_profile="simple";;
        esac
        # Swap only the simulator profile; preserve the others (desktop, l4t, ...)
        # — except for simple, which also drops 'desktop' (see above).
        local profiles kept=() p
        profiles=$(resolve_launch_var COMPOSE_PROFILES "$@")
        IFS=',' read -ra _parr <<< "$profiles"
        for p in "${_parr[@]}"; do
            case "$p" in
                isaac-sim|ms-airsim|simple|"") ;;
                desktop) [[ "$sim_profile" == "simple" ]] || kept+=("$p");;
                *) kept+=("$p");;
            esac
        done
        kept+=("$sim_profile")
        export COMPOSE_PROFILES=$(IFS=','; echo "${kept[*]}")
        [[ -n "$urdf" ]] && export URDF_FILE="$urdf"
        if [[ "$sim_profile" == "simple" ]]; then
            log_info "--sim simple → simple-robot replaces robot-desktop (profile 'desktop' dropped; no GCS, no MAVROS/PX4)"
        fi
    fi

    # Fleet dispatch (RFC #380 §2): triggered by --fleet, or by an env /
    # --env-file / .env FLEET_CONFIG_FILE (the opt-in path the test harness
    # uses). No fleet anywhere = byte-identical legacy behavior.
    local _fleet_sel="$AIRSTACK_INTENT_FLEET"
    if [[ -z "$_fleet_sel" ]]; then
        _fleet_sel=$(resolve_launch_var FLEET_CONFIG_FILE "$@")
    fi
    if [[ -n "$_fleet_sel" ]]; then
        apply_fleet_intent "$_fleet_sel" "$@" || return 1
    fi

    [[ -n "$AIRSTACK_INTENT_ROBOTS" ]]     && export NUM_ROBOTS="$AIRSTACK_INTENT_ROBOTS"
    [[ -n "$AIRSTACK_INTENT_PLAY" ]]       && export PLAY_SIM_ON_START="$AIRSTACK_INTENT_PLAY"
    [[ -n "$AIRSTACK_INTENT_AUTOLAUNCH" ]] && export AUTOLAUNCH="$AIRSTACK_INTENT_AUTOLAUNCH"
    if [[ "$AIRSTACK_INTENT_HEADLESS" == "true" ]]; then
        export ISAAC_SIM_HEADLESS="true"
        export MS_AIRSIM_HEADLESS="true"
        export QT_QPA_PLATFORM="offscreen"
    fi

    # --robots on Isaac: keep NUM_ROBOTS and the launch script consistent.
    # The default single-drone script hardcodes exactly one drone; NUM_ROBOTS>1
    # with it means N robot containers and 1 drone in sim (silent today).
    local profiles_final
    profiles_final=$(resolve_launch_var COMPOSE_PROFILES "$@")
    if [[ -n "$AIRSTACK_INTENT_ROBOTS" && ",$profiles_final," == *",isaac-sim,"* ]]; then
        local script want=""
        script=$(resolve_launch_var ISAAC_SIM_SCRIPT_NAME "$@")
        case "$script" in
            example_one_px4_pegasus_launch_script.py|example_multi_px4_pegasus_launch_script.py)
                (( AIRSTACK_INTENT_ROBOTS > 1 )) \
                    && want="example_multi_px4_pegasus_launch_script.py" \
                    || want="example_one_px4_pegasus_launch_script.py";;
            *)
                if (( AIRSTACK_INTENT_ROBOTS > 1 )); then
                    log_warn "Custom ISAAC_SIM_SCRIPT_NAME='$script' with --robots $AIRSTACK_INTENT_ROBOTS: make sure it spawns $AIRSTACK_INTENT_ROBOTS drones (reads NUM_ROBOTS)."
                fi;;
        esac
        if [[ -n "$want" && "$want" != "$script" ]]; then
            log_info "--robots $AIRSTACK_INTENT_ROBOTS → ISAAC_SIM_SCRIPT_NAME=$want (was $script)"
            export ISAAC_SIM_SCRIPT_NAME="$want"
        fi
    fi
    return 0
}

# Print the resolved launch configuration and dump it (gitignored) so every
# run leaves a record of what it actually launched.
function print_launch_config {
    local keys=(COMPOSE_PROFILES NUM_ROBOTS URDF_FILE AUTOLAUNCH PLAY_SIM_ON_START
                ISAAC_SIM_SCRIPT_NAME ISAAC_SIM_USE_STANDALONE ISAAC_SIM_HEADLESS
                MS_AIRSIM_HEADLESS AIRSTACK_STACK_DIR AIRSTACK_STACK_ENTRY
                VERSION DOCKER_IMAGE_BUILD_MODE)
    # FLEET_CONFIG_FILE only appears when a fleet is selected — the no-fleet
    # effective config stays byte-identical to the pre-fleet contract.
    local _fleet_cfg
    _fleet_cfg=$(resolve_launch_var FLEET_CONFIG_FILE "$@")
    [[ -n "$_fleet_cfg" ]] && keys+=(FLEET_CONFIG_FILE)
    local k v lines=()
    for k in "${keys[@]}"; do
        v=$(resolve_launch_var "$k" "$@")
        lines+=("$k=$v")
    done

    local profiles
    profiles=$(resolve_launch_var COMPOSE_PROFILES "$@")
    log_info "Launch config: profiles=$profiles robots=$(resolve_launch_var NUM_ROBOTS "$@") autolaunch=$(resolve_launch_var AUTOLAUNCH "$@") play_on_start=$(resolve_launch_var PLAY_SIM_ON_START "$@")"
    if [[ ",$profiles," == *",isaac-sim,"* ]]; then
        log_info "  isaac: script=$(resolve_launch_var ISAAC_SIM_SCRIPT_NAME "$@") headless=$(resolve_launch_var ISAAC_SIM_HEADLESS "$@")"
    fi
    log_info "  urdf=$(resolve_launch_var URDF_FILE "$@")"
    local _stack_dir
    _stack_dir=$(resolve_launch_var AIRSTACK_STACK_DIR "$@")
    if [[ -n "$_stack_dir" ]]; then
        log_info "  stack: dir=$_stack_dir entry=$(resolve_launch_var AIRSTACK_STACK_ENTRY "$@")"
    fi
    if [[ -n "$_fleet_cfg" ]]; then
        local _fleet_host="${_fleet_cfg#/root/AirStack/}"
        [[ "$_fleet_host" != /* ]] && _fleet_host="$PROJECT_ROOT/$_fleet_host"
        log_info "  fleet: $_fleet_cfg — resolved robots:"
        if [[ -f "$_fleet_host" ]]; then
            python3 "$PROJECT_ROOT/tools/fleet/resolve_fleet.py" "$_fleet_host" \
                --project-root "$PROJECT_ROOT" --table 2>/dev/null | sed 's/^/    /'
        else
            log_warn "  fleet file not found on the host side: $_fleet_host"
        fi
    fi

    echo "--- effective launch config ---"
    printf '%s\n' "${lines[@]}"
    echo "--- end effective launch config ---"

    # Dump for reproducibility (precursor of RFC #380's effective_config.yaml).
    # Best-effort: a read-only checkout (e.g. the tests container) skips it.
    # PID suffix keeps back-to-back runs within the same second in distinct dirs.
    local run_dir="$PROJECT_ROOT/.airstack/runs/$(date +%Y-%m-%d_%H-%M-%S)_$$"
    if mkdir -p "$run_dir" 2>/dev/null && printf '%s\n' "${lines[@]}" > "$run_dir/effective_config.env" 2>/dev/null; then
        log_info "  effective config saved to ${run_dir#$PROJECT_ROOT/}/effective_config.env"

        # Prune: keep only the newest 50 run records (dir names are
        # timestamp_pid — no spaces, xargs-safe). This also clears any
        # historical backlog opportunistically.
        local stale
        stale=$(ls -1dt "$PROJECT_ROOT/.airstack/runs"/*/ 2>/dev/null | tail -n +51)
        if [[ -n "$stale" ]]; then
            log_info "  pruning $(echo "$stale" | wc -l) old run record(s) from .airstack/runs/ (keeping newest 50)"
            echo "$stale" | xargs -r rm -rf
        fi
    fi
    return 0
}

# Fail-fast validation of the RESOLVED configuration (env > --env-file > .env),
# fixing the historical bypass where guards sed'd .env only and missed
# --env-file overrides. AIRSTACK_SKIP_PREFLIGHT=1 downgrades errors to warnings.
# Takes ONE nameref: the compose global args (--env-file/--profile scanning).
function preflight_up {
    local -n _pf_global=$1
    local errors=0

    function _pf_error {
        if [[ "${AIRSTACK_SKIP_PREFLIGHT:-}" == "1" ]]; then
            log_warn "(preflight skipped) $1"
        else
            log_error "$1"
            errors=1
        fi
    }

    # Resolved profiles + any --profile args passed on the CLI
    local profiles p i=0
    profiles=$(resolve_launch_var COMPOSE_PROFILES "${_pf_global[@]}")
    while [ $i -lt ${#_pf_global[@]} ]; do
        if [[ "${_pf_global[$i]}" == "--profile" ]]; then
            i=$((i+1)); profiles+=",${_pf_global[$i]:-}"
        elif [[ "${_pf_global[$i]}" == --profile=* ]]; then
            profiles+=",${_pf_global[$i]#--profile=}"
        fi
        i=$((i+1))
    done

    # 1. Exactly one simulator profile
    local n=0 s
    for s in isaac-sim ms-airsim simple; do [[ ",$profiles," == *",$s,"* ]] && n=$((n+1)); done
    if (( n > 1 )); then
        _pf_error "Only one simulator profile can be active at a time (isaac-sim, ms-airsim, simple). Resolved: $profiles"
    elif (( n == 0 )); then
        log_warn "No simulator profile active — robot containers will wait for a sim that never starts. Use 'airstack up --sim isaac|airsim|simple' (or add a sim profile)."
    fi

    # 2. URDF must match the simulator
    local urdf
    urdf=$(resolve_launch_var URDF_FILE "${_pf_global[@]}")
    if [[ -n "$urdf" ]]; then
        [[ ",$profiles," == *",ms-airsim,"* && "$urdf" != *.ms-airsim.* ]] && log_warn "URDF_FILE ($urdf) does not match ms-airsim profile. Expected *.ms-airsim.* URDF (or use --sim airsim)."
        [[ ",$profiles," == *",isaac-sim,"* && "$urdf" != *.pegasus.* && "$urdf" != *.isaacsim.* ]] && log_warn "URDF_FILE ($urdf) does not match isaac-sim profile. Expected *.pegasus.* or *.isaacsim.* URDF (or use --sim isaac)."
    fi

    if [[ ",$profiles," == *",isaac-sim,"* ]]; then
        # 3. NUM_ROBOTS vs single-drone launch script (the silent 3-containers-1-drone footgun)
        local num script
        num=$(resolve_launch_var NUM_ROBOTS "${_pf_global[@]}")
        script=$(resolve_launch_var ISAAC_SIM_SCRIPT_NAME "${_pf_global[@]}")
        if [[ "$num" =~ ^[0-9]+$ ]] && (( num > 1 )) && [[ "$script" == example_one_* ]]; then
            _pf_error "NUM_ROBOTS=$num but ISAAC_SIM_SCRIPT_NAME='$script' spawns exactly 1 drone: robots 2..$num would have no PX4 to talk to. Fix: 'airstack up --sim isaac --robots $num' (auto-selects the multi script) or set ISAAC_SIM_SCRIPT_NAME=example_multi_px4_pegasus_launch_script.py"
        fi

        # 4. Files the isaac-sim service hard-requires
        if [ ! -f "$PROJECT_ROOT/simulation/isaac-sim/docker/omni_pass.env" ]; then
            _pf_error "simulation/isaac-sim/docker/omni_pass.env is missing (Nucleus credentials). Run 'airstack setup' to create it."
        fi
        if [ ! -e "$PROJECT_ROOT/simulation/isaac-sim/extensions/PegasusSimulator/extensions/pegasus.simulator" ]; then
            _pf_error "PegasusSimulator submodule is empty — the Isaac launch script will fail to import pegasus. Run: git submodule update --init --recursive"
        fi
    fi

    # 5. Missing images: compose 'up' silently starts a very long build
    local imgs img missing=()
    imgs=$(run_docker_compose "${_pf_global[@]}" config --images 2>/dev/null | sort -u)
    for img in $imgs; do
        docker image inspect "$img" >/dev/null 2>&1 || missing+=("$img")
    done
    if (( ${#missing[@]} > 0 )); then
        log_warn "Images not present locally — compose will BUILD them from scratch (can take a long time):"
        printf '  - %s\n' "${missing[@]}" >&2
        log_warn "To use prebuilt images instead: airstack image-pull   (set AIRSTACK_NO_IMAGE_BUILD=1 to forbid implicit builds)"
    fi

    # 6. ROBOT_NAME resolution needs Docker >= 29 (see robot/docker/.bashrc)
    local docker_major
    docker_major=$(docker version --format '{{.Server.Version}}' 2>/dev/null | cut -d. -f1)
    if [[ "$docker_major" =~ ^[0-9]+$ ]] && (( docker_major < 29 )); then
        log_warn "Docker $docker_major < 29: container-name DNS resolution fails, robots will resolve as 'unknown_robot' on domain 0 (MAVROS will not connect). Upgrade Docker or set ROBOT_NAME_SOURCE=hostname."
    fi

    # 7. Deprecation shim (remove in 0.21.0): LAUNCH_NATNET no longer does
    # anything — OptiTrack was extracted to the asm_optitrack module.
    local _pf_natnet
    _pf_natnet=$(resolve_launch_var LAUNCH_NATNET "${_pf_global[@]}")
    if [[ -n "$_pf_natnet" ]]; then
        log_warn "LAUNCH_NATNET is gone — OptiTrack moved to the asm_optitrack module (airstack module add https://github.com/castacks/asm_optitrack --version <tag>); see docs/development/modules.md"
    fi

    # 8. Deprecation shim (remove in 0.21.0): AUTONOMY_ROLE was REMOVED
    # (stacks — RFC #379 — are the only launch dispatch). A set value counts
    # only via env / --env-file / .env; nothing in the compose files defaults
    # it anymore.
    local _pf_role
    _pf_role=$(resolve_launch_var AUTONOMY_ROLE "${_pf_global[@]}")
    if [[ -n "$_pf_role" ]]; then
        _pf_error "AUTONOMY_ROLE was removed — select a stack: airstack up --stack <name> (see docs/development/stacks.md). Migration: full → full_default (the no-stack default), onboard → lite_default, onboard/offboard split → lite_offload_global:onboard / :offboard."
    fi

    unset -f _pf_error
    return $errors
}

function cmd_up {
    check_docker

    # Airstack launch-intent flags (consumed before compose sees the args)
    local rest_args=()
    parse_launch_intent rest_args "$@" || exit 1
    apply_launch_intent "${rest_args[@]}" || exit 1

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "${rest_args[@]}"

    # Module overlay: when `airstack module sync` has generated a compose
    # override (volume mounts for synced modules), include it automatically so
    # module packages/extensions reach the containers. Opt out with
    # AIRSTACK_NO_MODULE_COMPOSE=1. Absent file = no modules = no change.
    local module_compose="$PROJECT_ROOT/.airstack/generated/docker-compose.modules.yaml"
    if [[ -f "$module_compose" && "${AIRSTACK_NO_MODULE_COMPOSE:-}" != "1" ]]; then
        log_info "Module overlay active → including ${module_compose#$PROJECT_ROOT/}"
        global_args+=(-f "$module_compose")
    fi

    # Fleet overlay (RFC #380 §2): apply_fleet_intent regenerated per-robot
    # services for a heterogeneous fleet — include them (the fleet profile is
    # already active in COMPOSE_PROFILES; homogeneous fleets never set this).
    if [[ -n "${AIRSTACK_FLEET_COMPOSE_FILE:-}" ]]; then
        log_info "Fleet overlay active → including ${AIRSTACK_FLEET_COMPOSE_FILE#$PROJECT_ROOT/}"
        global_args+=(-f "$AIRSTACK_FLEET_COMPOSE_FILE")
    fi

    print_launch_config "${global_args[@]}"
    if ! preflight_up global_args; then
        log_error "Preflight failed — not starting services. (AIRSTACK_SKIP_PREFLIGHT=1 to override.)"
        exit 1
    fi

    if [[ "$AIRSTACK_DRY_RUN" == "1" ]]; then
        log_info "--dry-run: configuration validated; not starting services."
        return 0
    fi

    # Add xhost + to allow GUI applications
    xhost + &> /dev/null || true

    # Registry-cache mode (CI / opt-in): pull existing images first so `up`
    # uses the registry copy as-is and skips the implicit rebuild path. No-op
    # when AIRSTACK_REGISTRY_CACHE is unset.
    if [[ "${AIRSTACK_REGISTRY_CACHE:-}" == "1" ]]; then
        log_info "AIRSTACK_REGISTRY_CACHE=1 → pulling images before up..."
        run_docker_compose "${global_args[@]}" pull --ignore-pull-failures "${subcmd_args[@]}" || \
            log_warn "Pre-up pull encountered failures; continuing with whatever is local"
    fi

    log_info "Starting services..."
    local up_opts=()
    if [[ "${AIRSTACK_NO_IMAGE_BUILD:-}" == "1" ]]; then
        log_info "AIRSTACK_NO_IMAGE_BUILD=1 → compose up --no-build"
        up_opts+=(--no-build)
    fi
    run_docker_compose "${global_args[@]}" up "${up_opts[@]}" "${subcmd_args[@]}" -d
    log_info "Containers started. (Workspaces may still be building and the sim loading — run 'airstack ready' to wait for flight-readiness.)"

    if [[ "$AIRSTACK_UP_WAIT" == "1" ]]; then
        cmd_ready
    fi
}

function cmd_image_build {
    check_docker

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "$@"
    # Jetson only: building robot-l4t needs robot-l4t-stack-base first
    for arg in "${subcmd_args[@]}"; do
        if [[ "$arg" == robot-l4t ]]; then
            ensure_robot_l4t_stack_base global_args subcmd_args
            break
        fi
    done

    # Registry-cache mode (CI / opt-in): pre-pull existing images to seed the
    # local cache, build with BUILDKIT_INLINE_CACHE=1 so the resulting image
    # carries layer-cache metadata. The cache_from declarations in each
    # component compose file make BuildKit actually reuse the pulled layers.
    # No-op when the env var is unset.
    #
    # Reading and publishing the cache are separate switches. A PR bumps VERSION
    # (check-version-increment enforces it), so the versioned cache_from entry is
    # guaranteed to miss and the floating CACHE_TAG entry is what actually hits.
    # PR runs must not write that floating tag: an unmerged branch would poison
    # the shared cache and publish an unreleased VERSION. Only trusted branches
    # set AIRSTACK_REGISTRY_CACHE_PUSH=1.
    if [[ "${AIRSTACK_REGISTRY_CACHE:-}" == "1" ]]; then
        log_info "AIRSTACK_REGISTRY_CACHE=1 → pulling for cache seed..."
        run_docker_compose "${global_args[@]}" pull --ignore-pull-failures "${subcmd_args[@]}" || \
            log_warn "Pre-build pull encountered failures; continuing without cache seed"

        log_info "Building services with BUILDKIT_INLINE_CACHE=1..."
        run_docker_compose "${global_args[@]}" build --build-arg BUILDKIT_INLINE_CACHE=1 "${subcmd_args[@]}"

        if [[ "${AIRSTACK_REGISTRY_CACHE_PUSH:-}" == "1" ]]; then
            log_info "Pushing built images for next-run cache..."
            run_docker_compose "${global_args[@]}" push --ignore-push-failures "${subcmd_args[@]}" || \
                log_warn "Post-build push encountered failures; future runs may not benefit from cache"
            push_cache_tags global_args subcmd_args
        else
            log_info "AIRSTACK_REGISTRY_CACHE_PUSH is not 1 → cache is read-only for this run"
        fi
    else
        log_info "Building services..."
        run_docker_compose "${global_args[@]}" build "${subcmd_args[@]}"
    fi
    log_info "Build completed successfully"
}

function cmd_image_push {
    check_docker

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "$@"

    log_info "Pushing service images..."
    run_docker_compose "${global_args[@]}" push "${subcmd_args[@]}"
    log_info "Push completed successfully"
}

function cmd_image_pull {
    check_docker

    local global_args=()
    local subcmd_args=()
    classify_compose_args global_args subcmd_args "$@"

    log_info "Pulling service images..."
    run_docker_compose "${global_args[@]}" pull "${subcmd_args[@]}"
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

function cmd_image_delete {
    check_docker

    local env_file="$PROJECT_ROOT/.env"
    local project_name=""
    if [ -f "$env_file" ]; then
        project_name=$(grep -E "^PROJECT_NAME=" "$env_file" | cut -d'=' -f2 | tr -d '"' | tr -d "'")
    fi
    if [ -z "$project_name" ]; then
        log_error "PROJECT_NAME not found in .env; refusing to delete."
        return 1
    fi

    # Match images whose repository contains "/PROJECT_NAME" or equals "PROJECT_NAME".
    # Using a regex anchored to a path segment avoids false positives on similar names.
    local refs
    refs=$(docker images --format '{{.Repository}}:{{.Tag}}' \
        | grep -E "(^|/)${project_name}(:|$)" || true)

    if [ -z "$refs" ]; then
        log_info "No images found matching project: $project_name"
        return 0
    fi

    log_info "The following images will be deleted:"
    echo "$refs" | sed 's/^/  /'

    # Confirm unless --yes / -y is passed.
    local auto_yes=false
    for arg in "$@"; do
        [[ "$arg" == "-y" || "$arg" == "--yes" ]] && auto_yes=true
    done
    if ! $auto_yes; then
        read -r -p "Delete these images? [y/N] " reply
        [[ "$reply" =~ ^[Yy]$ ]] || { log_info "Aborted."; return 0; }
    fi

    echo "$refs" | xargs -r docker rmi -f
    log_info "Done."
}

function cmd_down {
    check_docker
    
    local services=("$@")

    # Build compose arguments (the base compose file is folded into
    # run_docker_compose; only overlay -f files are added here)
    local compose_args=()

    # Generated overlays (module mounts, fleet services) must be visible to
    # `down` too — a fleet-generated service that `down` can't see becomes an
    # invisible orphan spinning at full CPU (ddsrouters busy-loop without a
    # sim /clock). --remove-orphans below is the belt-and-braces backstop.
    local _gen
    for _gen in "$PROJECT_ROOT/.airstack/generated/docker-compose.modules.yaml" \
                "$PROJECT_ROOT/.airstack/generated/docker-compose.fleet.yaml"; do
        [ -f "$_gen" ] && compose_args+=("-f" "$_gen")
    done

    # Add services if specified
    if [ ${#services[@]} -gt 0 ]; then
        compose_args+=("down" "${services[@]}")
    else
        compose_args+=("--profile" "*" "down" "--remove-orphans")
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
        "$PROJECT_ROOT/simulation/simple-sim/ros_ws/build"
        "$PROJECT_ROOT/simulation/simple-sim/ros_ws/install"
        "$PROJECT_ROOT/simulation/simple-sim/ros_ws/log"
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
    
    # Find container by pattern. NOTE: `var=$(fn)` + `[ $? -eq 0 ]` is a trap
    # under `set -e` — a failing assignment exits the script before the check
    # runs — so test the assignment directly.
    local container
    if ! container=$(find_container "$container_pattern"); then
        log_error "Failed to connect to container. Please try again with a more specific name."
        return 1
    fi

    log_info "Connecting to container: $container"

    local exit_status=0
    if [ "$command_specified" = true ]; then
        # Run the specified command directly, no tmux involvement
        if ! docker exec "$container" which "$command" &> /dev/null; then
            log_warn "Command '$command' not found in container. Falling back to /bin/sh"
            command="sh"
        fi
        docker exec -it "$container" "$command" || exit_status=$?
    else
        # Default: attach to an existing tmux session. Exiting tmux fully disconnects from the container.
        if docker exec "$container" which tmux &> /dev/null; then
            docker exec -it "$container" tmux a || exit_status=$?
        else
            log_warn "tmux not found in container. Falling back to bash."
            local fallback="bash"
            if ! docker exec "$container" which bash &> /dev/null; then
                fallback="sh"
            fi
            docker exec -it "$container" "$fallback" || exit_status=$?
        fi
    fi

    if [ $exit_status -ne 0 ]; then
        log_warn "Command exited with status $exit_status"
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

    # Collect rows: container_name, robot_name, ros_domain_id, status, ports
    local -a rows=()
    while IFS=$'\t' read -r name status ports; do
        local robot_name="" ros_domain_id="" identity
        # Single exec per container (shared helper in _lib.sh)
        if identity=$(_container_identity "$name"); then
            IFS=$'\t' read -r robot_name ros_domain_id <<< "$identity"
        fi
        [ -z "$robot_name" ]    && robot_name="N/A"
        [ -z "$ros_domain_id" ] && ros_domain_id="N/A"
        rows+=("${name}|${robot_name}|${ros_domain_id}|${status}|${ports}")
    done <<< "$containers"

    # Sort rows alphabetically by container name
    local sorted
    sorted=$(printf '%s\n' "${rows[@]}" | sort -t'|' -k1,1)
    mapfile -t rows <<< "$sorted"

    # Determine column widths (minimum = header length)
    local w_container=14 w_robot=10 w_domain=13 w_status=6 w_ports=5
    for row in "${rows[@]}"; do
        IFS='|' read -r c_name c_robot c_domain c_status c_ports <<< "$row"
        [ ${#c_name}   -gt $w_container ] && w_container=${#c_name}
        [ ${#c_robot}  -gt $w_robot ]     && w_robot=${#c_robot}
        [ ${#c_domain} -gt $w_domain ]    && w_domain=${#c_domain}
        [ ${#c_status} -gt $w_status ]    && w_status=${#c_status}
    done
    # Add padding
    w_container=$((w_container + 2))
    w_robot=$((w_robot + 2))
    w_domain=$((w_domain + 2))
    w_status=$((w_status + 2))

    # Print header
    printf "${BOLDCYAN}%-${w_container}s %-${w_robot}s %-${w_domain}s %-${w_status}s %s${NC}\n" \
        "CONTAINER NAME" "ROBOT_NAME" "ROS_DOMAIN_ID" "STATUS" "PORTS"
    # Separator
    printf "%-${w_container}s %-${w_robot}s %-${w_domain}s %-${w_status}s %s\n" \
        "$(printf '%*s' "$w_container" '' | tr ' ' '-')" \
        "$(printf '%*s' "$w_robot"     '' | tr ' ' '-')" \
        "$(printf '%*s' "$w_domain"    '' | tr ' ' '-')" \
        "$(printf '%*s' "$w_status"    '' | tr ' ' '-')" \
        "-----"

    # Print rows
    for row in "${rows[@]}"; do
        IFS='|' read -r c_name c_robot c_domain c_status c_ports <<< "$row"
        printf "%-${w_container}s %-${w_robot}s %-${w_domain}s %-${w_status}s %s\n" \
            "$c_name" "$c_robot" "$c_domain" "$c_status" "$c_ports"
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
    
    # Find container by pattern (see cmd_connect for the set -e note).
    local container
    if ! container=$(find_container "$container_pattern"); then
        log_error "Failed to find container. Please try again with a more specific name."
        return 1
    fi

    log_info "Showing logs for container: $container"

    # Build the logs command
    local logs_args=()
    if [ "$follow" = true ]; then
        logs_args+=(-f)
    fi
    if [ "$tail" != "all" ]; then
        logs_args+=(--tail "$tail")
    fi

    docker logs "${logs_args[@]}" "$container"
}

function cmd_version {
    local version
    version=$(_env_value VERSION "$PROJECT_ROOT/.env")

    if [ -z "$version" ]; then
        log_error "VERSION not found in .env file ($PROJECT_ROOT/.env)"
        return 1
    fi

    echo -e "${BOLDCYAN}AirStack Version:${NC} $version"
}

# Remove images by search term: same engine as cmd_image_delete (full
# repo:tag refs from `docker images --format`, never positional-column
# parsing) but matching a user-given term instead of PROJECT_NAME.
function cmd_rmi {
    check_docker

    local search_term="" force=false auto_yes=false
    for arg in "$@"; do
        case "$arg" in
            -f|--force) force=true ;;
            -y|--yes)   auto_yes=true ;;
            -*) log_error "Unknown option for 'rmi': $arg"; print_command_help "rmi"; return 1 ;;
            *)
                if [ -n "$search_term" ]; then
                    log_error "'rmi' takes exactly one search term (got '$search_term' and '$arg')"
                    return 1
                fi
                search_term="$arg" ;;
        esac
    done

    if [ -z "$search_term" ]; then
        log_error "Search term required"
        print_command_help "rmi"
        return 1
    fi

    # Fixed-string match against the full repo:tag ref; dangling <none>
    # images have no removable ref, so skip them.
    local refs
    refs=$(docker images --format '{{.Repository}}:{{.Tag}}' \
        | grep -v '<none>' | grep -F -- "$search_term" || true)

    if [ -z "$refs" ]; then
        log_info "No images match '$search_term'."
        return 0
    fi

    log_info "The following images will be removed:"
    echo "$refs" | sed 's/^/  /'

    if ! $auto_yes; then
        read -r -p "Remove these images? [y/N] " reply
        [[ "$reply" =~ ^[Yy]$ ]] || { log_info "Aborted."; return 0; }
    fi

    local rmi_flags=()
    $force && rmi_flags+=(-f)
    echo "$refs" | xargs -r docker rmi "${rmi_flags[@]}"
    log_info "Done."
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
            # Extract module name from filename
            module_name=$(basename "$module" .sh)

            # _*.sh files are shared libraries (already sourced explicitly
            # by airstack.sh), not command modules.
            if [[ "$module_name" == _* ]]; then
                continue
            fi

            # Source the module file
            source "$module"

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
    COMMANDS["image-delete"]="cmd_image_delete"
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
    COMMAND_HELP["image-delete"]="Delete all Docker images matching PROJECT_NAME (prompts unless -y)"
    COMMAND_HELP["up"]="Start services [--sim isaac|airsim|simple] [--robots N] [--stack NAME] [--fleet NAME] [--headless] [--play|--no-play] [--no-autolaunch] [--wait] [--dry-run]"
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