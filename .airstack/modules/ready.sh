#!/bin/bash
# Readiness gates for a running AirStack stack.
#
# `airstack up` reports success the moment `docker compose up -d` returns —
# before workspaces build, the sim loads, or PX4 boots. `airstack ready`
# answers the question users otherwise guess at: "can I press Takeoff yet?"
#
# Gates and budgets mirror the system-test suite (the source of truth for
# real-world timings — tests/system/test_liveliness.py and
# tests/system/test_takeoff_hover_land.py):
#   1. containers Running                     (120 s)
#   2. sim publishing /clock                  (600 s — Isaac scene loads are slow)
#   3. sentinel ROS 2 nodes per robot         (300 s — includes the colcon build in dev mode)
#   4. PX4 ready per robot: MAVROS connected  (300 s)
#      then local_position/odom streaming (EKF converged = armable; connected
#      alone fires ~25 s too early and takeoff returns "failed to arm")
#
# --json contract: stdout carries ONLY the JSON verdict; all progress lines
# (banner, ✓/✗ polls, log_info/log_warn) go to stderr, so
# `airstack ready --json 2>/dev/null | python3 -m json.tool` always parses.

# Defaults match the system-test budgets; overridable from the environment
# (e.g. READY_CLOCK_TIMEOUT=60 airstack ready).
: "${READY_CONTAINERS_TIMEOUT:=120}"
: "${READY_CLOCK_TIMEOUT:=600}"
: "${READY_NODES_TIMEOUT:=300}"
: "${READY_PX4_TIMEOUT:=300}"
: "${READY_POLL_INTERVAL:=5}"

# Sentinel nodes expected per robot, built from the RESOLVED robot name
# (matches tests/system/test_liveliness.py). For default robot_N fleets the
# name is robot_<domain>, so behavior is identical to the old domain-derived
# paths — but fleets with custom robot names now gate on the right graph.
READY_SENTINEL_TEMPLATES=(
    "/%s/interface/mavros/mavros"
    "/%s/robot_state_publisher"
    "/%s/trajectory_controller/trajectory_control_node"
)

function _ready_now { date +%s; }

function _ready_elapsed {
    echo "$(( $(_ready_now) - $1 ))"
}

# Run a ros2 command inside a robot container on a given domain, sourcing the
# workspace if it is built yet (mavros msgs need it).
function _ready_ros2_exec {
    local container="$1" domain="$2" cmd="$3" timeout_s="${4:-10}"
    docker exec "$container" bash -c "
        source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
        [ -f /root/AirStack/robot/ros_ws/install/setup.bash ] && source /root/AirStack/robot/ros_ws/install/setup.bash >/dev/null 2>&1
        export ROS_DOMAIN_ID=$domain
        timeout $timeout_s $cmd" 2>/dev/null
}

# Poll a predicate function until it returns 0 or the timeout expires.
# Usage: _ready_poll <timeout_s> <label> <fn> [args...]
function _ready_poll {
    local timeout_s="$1" label="$2" fn="$3"; shift 3
    local start; start=$(_ready_now)
    while true; do
        if "$fn" "$@"; then
            echo -e "  ${GREEN}✓${NC} $label ($(_ready_elapsed "$start")s)"
            return 0
        fi
        if (( $(_ready_elapsed "$start") >= timeout_s )); then
            echo -e "  ${RED}✗${NC} $label — timed out after ${timeout_s}s"
            return 1
        fi
        sleep "$READY_POLL_INTERVAL"
    done
}

# Gate helpers share one argument order: <domain> <name> <container>
# (identity → placement), except gates that need no identity.

function _gate_containers_ok {
    local n
    n=$(_robot_containers | wc -l)
    (( n >= 1 ))
}

function _gate_clock_ok {
    local container="$1"
    _ready_ros2_exec "$container" 1 "ros2 topic echo --once /clock" 10 | grep -q "nanosec"
}

function _gate_nodes_ok {
    local domain="$1" name="$2" container="$3" nodes tmpl missing=0
    nodes=$(_ready_ros2_exec "$container" "$domain" "ros2 node list" 10)
    for tmpl in "${READY_SENTINEL_TEMPLATES[@]}"; do
        local want; want=$(printf "$tmpl" "$name")
        if ! grep -qx "$want" <<< "$nodes"; then missing=1; fi
    done
    (( missing == 0 ))
}

function _gate_px4_connected_ok {
    local domain="$1" name="$2" container="$3"
    _ready_ros2_exec "$container" "$domain" \
        "ros2 topic echo --once --csv --field connected /${name}/interface/mavros/state" 6 \
        | grep -qx "True"
}

function _gate_px4_odom_ok {
    local domain="$1" name="$2" container="$3"
    _ready_ros2_exec "$container" "$domain" \
        "ros2 topic echo --once /${name}/interface/mavros/local_position/odom" 6 \
        | grep -q "pose:"
}

# Run every gate, filling the caller's `results` map and READY_OVERALL.
# All output from this function is progress reporting — cmd_ready redirects
# it wholesale to stderr in --json mode.
function _ready_run_gates {
    READY_OVERALL=0

    # Heads-up when the stack is idle by design.
    local autolaunch
    autolaunch=$(resolve_launch_var AUTOLAUNCH)
    if [[ "$autolaunch" == "false" ]]; then
        log_warn "AUTOLAUNCH=false — containers are idle by design; readiness will not be reached until you launch the stack manually."
    fi

    echo "Waiting for the AirStack stack to become flight-ready..."

    # ---- Gate 1: containers ----
    if _ready_poll "$READY_CONTAINERS_TIMEOUT" "robot containers running" _gate_containers_ok; then
        results[containers]=ok
    else
        results[containers]=failed
        log_error "No robot containers running. Did you run 'airstack up'? Check: airstack status"
        READY_OVERALL=1
    fi

    local containers=() c
    while IFS= read -r c; do [ -n "$c" ] && containers+=("$c"); done < <(_robot_containers)

    if (( READY_OVERALL == 0 )); then
        local first="${containers[0]}"

        # ---- Gate 2: sim /clock ----
        if _ready_poll "$READY_CLOCK_TIMEOUT" "sim publishing /clock" _gate_clock_ok "$first"; then
            results[sim_clock]=ok
        else
            results[sim_clock]=failed
            log_error "Sim never published /clock. Inspect the sim: airstack connect isaac-sim (or ms-airsim), or airstack logs <sim>"
            log_error "If PLAY_SIM_ON_START=false, press Play in the Isaac Sim window."
            READY_OVERALL=1
        fi
    fi

    # ---- Gates 3+4 per robot ----
    if (( READY_OVERALL == 0 )); then
        for c in "${containers[@]}"; do
            # One exec resolves both name and domain (_lib.sh helper).
            local identity domain="" name=""
            if identity=$(_container_identity "$c"); then
                IFS=$'\t' read -r name domain <<< "$identity"
            fi
            if [ -z "$domain" ] || [ -z "$name" ] || [ "$name" == "unknown_robot" ]; then
                log_error "$c: ROBOT_NAME/ROS_DOMAIN_ID did not resolve (got '${name:-?}'). See robot/docker/robot_name_map/."
                results["nodes_${c}"]=failed
                READY_OVERALL=1
                continue
            fi

            if _ready_poll "$READY_NODES_TIMEOUT" "$name (domain $domain): autonomy nodes up" _gate_nodes_ok "$domain" "$name" "$c"; then
                results["nodes_${name}"]=ok
            else
                results["nodes_${name}"]=failed
                log_error "$name: sentinel nodes missing. The workspace may still be building, or the launch crashed — inspect: airstack connect $c   (tmux window 'bringup'), or airstack logs $c"
                READY_OVERALL=1
                continue
            fi

            if _ready_poll "$READY_PX4_TIMEOUT" "$name: MAVROS connected to PX4" _gate_px4_connected_ok "$domain" "$name" "$c"; then
                results["px4_connected_${name}"]=ok
            else
                results["px4_connected_${name}"]=failed
                log_error "$name: MAVROS never connected to PX4 (port = 14540 + domain). Is the sim running with $((${#containers[@]})) drone(s)? For Isaac multi-robot, ISAAC_SIM_SCRIPT_NAME must be a multi script (use 'airstack up --robots N')."
                READY_OVERALL=1
                continue
            fi

            if _ready_poll "$READY_PX4_TIMEOUT" "$name: PX4 EKF ready (armable)" _gate_px4_odom_ok "$domain" "$name" "$c"; then
                results["px4_ready_${name}"]=ok
            else
                results["px4_ready_${name}"]=failed
                log_error "$name: PX4 connected but local_position/odom never streamed (EKF has no local origin)."
                READY_OVERALL=1
            fi
        done
    fi
    return 0   # verdict travels via READY_OVERALL (set -e safety)
}

function cmd_ready {
    check_docker

    local json=false
    for arg in "$@"; do
        case "$arg" in
            --json) json=true ;;
        esac
    done

    local start; start=$(_ready_now)
    # gate results for --json: name=ok|failed (filled by _ready_run_gates)
    local -A results=()
    READY_OVERALL=0

    # In --json mode ALL progress goes to stderr; stdout stays reserved for
    # the JSON document (see the contract note at the top of this file).
    if $json; then
        _ready_run_gates 1>&2
    else
        _ready_run_gates
    fi
    local overall=$READY_OVERALL

    local total; total=$(_ready_elapsed "$start")

    if $json; then
        local first_entry=true key
        echo -n '{"ready": '
        (( overall == 0 )) && echo -n 'true' || echo -n 'false'
        echo -n ", \"elapsed_s\": $total, \"gates\": {"
        for key in "${!results[@]}"; do
            $first_entry || echo -n ", "
            first_entry=false
            echo -n "\"$key\": \"${results[$key]}\""
        done
        echo '}}'
    else
        if (( overall == 0 )); then
            log_info "Stack is flight-ready (${total}s). Takeoff is available (GCS Robot Tasks panel, or: ros2 action send_goal /robot_1/tasks/takeoff ...)."
        else
            log_error "Stack did not become ready (${total}s). See messages above."
        fi
    fi
    return $overall
}

function register_ready_commands {
    COMMANDS["ready"]="cmd_ready"
    COMMAND_HELP["ready"]="Wait until the running stack is flight-ready (containers → sim clock → nodes → PX4); --json for scripts"
}
