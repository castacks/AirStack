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

# Defaults match the system-test budgets; overridable from the environment
# (e.g. READY_CLOCK_TIMEOUT=60 airstack ready).
: "${READY_CONTAINERS_TIMEOUT:=120}"
: "${READY_CLOCK_TIMEOUT:=600}"
: "${READY_NODES_TIMEOUT:=300}"
: "${READY_PX4_TIMEOUT:=300}"
: "${READY_POLL_INTERVAL:=5}"

# Sentinel nodes expected per robot domain (matches tests/system/test_liveliness.py).
READY_SENTINEL_TEMPLATES=(
    "/robot_%d/interface/mavros/mavros"
    "/robot_%d/robot_state_publisher"
    "/robot_%d/trajectory_controller/trajectory_control_node"
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

# List running robot containers, one per line: compose replicas
# (airstack-robot-desktop-N) AND fleet-generated per-robot services
# (airstack-robot_N-1). Ground hosts (gcs-robot_N tenants) are NOT robots —
# they never run MAVROS/PX4 — so exclude them.
function _ready_robot_containers {
    docker ps --format '{{.Names}}' | grep -E -- '-robot[-_]' | grep -v 'gcs-' | sort
}

# domain for robot container (via the same .bashrc resolution airstack status uses)
function _ready_domain_of {
    local container="$1" vars
    vars=$(docker exec "$container" bash --login -c \
        'printf "AIRSTACK_VARS:%s:%s\n" "$ROBOT_NAME" "$ROS_DOMAIN_ID"' 2>/dev/null \
        | grep "^AIRSTACK_VARS:" | tail -1)
    [ -z "$vars" ] && return 1
    echo "${vars##*:}"
}

function _ready_robot_name_of {
    local container="$1" vars
    vars=$(docker exec "$container" bash --login -c \
        'printf "AIRSTACK_VARS:%s:%s\n" "$ROBOT_NAME" "$ROS_DOMAIN_ID"' 2>/dev/null \
        | grep "^AIRSTACK_VARS:" | tail -1)
    [ -z "$vars" ] && return 1
    vars="${vars#AIRSTACK_VARS:}"
    echo "${vars%%:*}"
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

function _gate_containers_ok {
    local n
    n=$(_ready_robot_containers | wc -l)
    (( n >= 1 ))
}

function _gate_clock_ok {
    local container="$1"
    _ready_ros2_exec "$container" 1 "ros2 topic echo --once /clock" 10 | grep -q "nanosec"
}

function _gate_nodes_ok {
    local container="$2" domain="$1" nodes tmpl missing=0
    nodes=$(_ready_ros2_exec "$container" "$domain" "ros2 node list" 10)
    for tmpl in "${READY_SENTINEL_TEMPLATES[@]}"; do
        local want; want=$(printf "$tmpl" "$domain")
        if ! grep -qx "$want" <<< "$nodes"; then missing=1; fi
    done
    (( missing == 0 ))
}

function _gate_px4_connected_ok {
    local domain="$1" container="$2"
    _ready_ros2_exec "$container" "$domain" \
        "ros2 topic echo --once --csv --field connected /robot_${domain}/interface/mavros/state" 6 \
        | grep -qx "True"
}

function _gate_px4_odom_ok {
    local domain="$1" container="$2"
    _ready_ros2_exec "$container" "$domain" \
        "ros2 topic echo --once /robot_${domain}/interface/mavros/local_position/odom" 6 \
        | grep -q "pose:"
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
    local overall=0
    # gate results for --json: name=ok|failed
    local -A results=()

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
        overall=1
    fi

    local containers=() c
    while IFS= read -r c; do [ -n "$c" ] && containers+=("$c"); done < <(_ready_robot_containers)

    if (( overall == 0 )); then
        local first="${containers[0]}"

        # ---- Gate 2: sim /clock ----
        if _ready_poll "$READY_CLOCK_TIMEOUT" "sim publishing /clock" _gate_clock_ok "$first"; then
            results[sim_clock]=ok
        else
            results[sim_clock]=failed
            log_error "Sim never published /clock. Inspect the sim: airstack connect isaac-sim (or ms-airsim), or airstack logs <sim>"
            log_error "If PLAY_SIM_ON_START=false, press Play in the Isaac Sim window."
            overall=1
        fi
    fi

    # ---- Gates 3+4 per robot ----
    if (( overall == 0 )); then
        for c in "${containers[@]}"; do
            local domain name
            domain=$(_ready_domain_of "$c" || true)
            name=$(_ready_robot_name_of "$c" || true)
            if [ -z "$domain" ] || [ "$name" == "unknown_robot" ]; then
                log_error "$c: ROBOT_NAME/ROS_DOMAIN_ID did not resolve (got '${name:-?}'). See robot/docker/robot_name_map/."
                results["nodes_${c}"]=failed
                overall=1
                continue
            fi

            if _ready_poll "$READY_NODES_TIMEOUT" "$name (domain $domain): autonomy nodes up" _gate_nodes_ok "$domain" "$c"; then
                results["nodes_${name}"]=ok
            else
                results["nodes_${name}"]=failed
                log_error "$name: sentinel nodes missing. The workspace may still be building, or the launch crashed — inspect: airstack connect $c   (tmux window 'bringup'), or airstack logs $c"
                overall=1
                continue
            fi

            if _ready_poll "$READY_PX4_TIMEOUT" "$name: MAVROS connected to PX4" _gate_px4_connected_ok "$domain" "$c"; then
                results["px4_connected_${name}"]=ok
            else
                results["px4_connected_${name}"]=failed
                log_error "$name: MAVROS never connected to PX4 (port = 14540 + domain). Is the sim running with $((${#containers[@]})) drone(s)? For Isaac multi-robot, ISAAC_SIM_SCRIPT_NAME must be a multi script (use 'airstack up --robots N')."
                overall=1
                continue
            fi

            if _ready_poll "$READY_PX4_TIMEOUT" "$name: PX4 EKF ready (armable)" _gate_px4_odom_ok "$domain" "$c"; then
                results["px4_ready_${name}"]=ok
            else
                results["px4_ready_${name}"]=failed
                log_error "$name: PX4 connected but local_position/odom never streamed (EKF has no local origin)."
                overall=1
            fi
        done
    fi

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
