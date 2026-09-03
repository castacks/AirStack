#!/usr/bin/env bash
# Bring up the BV/drone_3 policy-support stack in one in-container tmux session.
#
# The host script starts Docker services and attaches the terminal. ROS commands
# and interactive shells live inside the AirStack robot container.

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_NAME="${SESSION_NAME:-drone_soccer_lab}"
CONTAINER="${CONTAINER:-airstack-robot-desktop-1}"
DDS_CONTAINER="${DDS_CONTAINER:-micro_ros_agent_jazzy}"
DDS_IMAGE="${DDS_IMAGE:-microros/micro-ros-agent:jazzy}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8892}"
DRONE_NAME="drone_3"
TARGET_SYSTEMS="3"
BALL_NAME="VolleyBall"
MOTIVE_SERVER_IP="${MOTIVE_SERVER_IP:-192.168.50.5}"
MOTIVE_CLIENT_IP="${MOTIVE_CLIENT_IP:-192.168.50.6}"
POLICY_REQUIREMENTS="/root/AirStack/robot/ros_ws/src/svg_ground_control/requirements-policy.txt"
ATTACH=1
STOP=0

usage() {
    cat <<'EOF'
Usage: start_drone_soccer_lab_tmux.sh [--no-attach] [--session NAME]
       start_drone_soccer_lab_tmux.sh --stop [--session NAME]

Creates a tmux session inside the AirStack robot container:
  runtime   NatNet, real_interfaces, and mocap_bridge panes
  policy    policy launch/start/stop/goal commands, typed but NOT run
  shell     sourced ROS shell for additional commands

The Micro XRCE-DDS agent still runs as a sibling host-networked container;
everything you interact with runs inside the robot container.
Missing policy dependencies are installed from requirements-policy.txt before
the tmux session is created or attached.

Environment overrides:
  SESSION_NAME, CONTAINER, DDS_CONTAINER, DDS_IMAGE, ROS_DOMAIN_ID,
  DDS_AGENT_PORT, MOTIVE_SERVER_IP, MOTIVE_CLIENT_IP

Defaults are the verified BV setup: drone_3, system ID 3, ROS domain 1,
DDS UDP port 8892, VolleyBall, Motive 192.168.50.5, AirStation 192.168.50.6.

Detach with Ctrl-b d. Reattach from the host with:
  docker exec -it airstack-robot-desktop-1 \
    tmux attach-session -t drone_soccer_lab

Stop the tmux session, DDS agent, and robot container with:
  ./scripts/start_drone_soccer_lab_tmux.sh --stop

No arm, offboard, takeoff, land, trajectory_commander, swarm_commander, or
policy_commander command is executed by this script.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --no-attach)
            ATTACH=0
            shift
            ;;
        --session)
            SESSION_NAME="${2:?--session requires a value}"
            shift 2
            ;;
        --stop)
            STOP=1
            ATTACH=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if ! command -v docker >/dev/null 2>&1; then
    echo "Required command not found: docker" >&2
    exit 1
fi

container_running() {
    docker ps --format '{{.Names}}' | grep -Fxq "$1"
}

container_tmux() {
    docker exec "$CONTAINER" tmux "$@"
}

if [ "$STOP" -eq 1 ]; then
    if container_running "$CONTAINER" && \
       container_tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        container_tmux kill-session -t "$SESSION_NAME"
        echo "Stopped in-container tmux session: $SESSION_NAME"
    fi
    for container in "$DDS_CONTAINER" "$CONTAINER"; do
        if container_running "$container"; then
            docker stop "$container" >/dev/null
            echo "Stopped container: $container"
        fi
    done
    exit 0
fi

if ! container_running "$CONTAINER"; then
    echo "Starting idle AirStack robot container: $CONTAINER"
    (
        cd "$REPO_ROOT"
        AUTOLAUNCH=false ./airstack.sh up robot-desktop
    )
fi

if ! container_running "$CONTAINER"; then
    echo "Robot container did not start: $CONTAINER" >&2
    exit 1
fi

if ! docker exec "$CONTAINER" bash -lc \
    'command -v tmux' >/dev/null 2>&1; then
    echo "tmux is not installed in robot container: $CONTAINER" >&2
    exit 1
fi

# colcon/setuptools does not install install_requires into the container. The
# v0.18.0 robot image also predates these policy dependencies, so enforce them
# every time the launcher starts; the fast path is only an import check.
if docker exec "$CONTAINER" python3 -c \
    'import stable_baselines3, gymnasium' >/dev/null 2>&1; then
    echo "Policy dependencies are available."
else
    echo "Policy dependencies are missing; installing requirements-policy.txt..."
    if ! docker exec "$CONTAINER" test -f "$POLICY_REQUIREMENTS"; then
        echo "Policy requirements file not found: $POLICY_REQUIREMENTS" >&2
        exit 1
    fi
    docker exec "$CONTAINER" python3 -m pip install \
        --break-system-packages \
        --no-cache-dir \
        --requirement "$POLICY_REQUIREMENTS"
    if ! docker exec "$CONTAINER" python3 -c \
        'import stable_baselines3, gymnasium'; then
        echo "Policy dependency verification failed after installation." >&2
        exit 1
    fi
    echo "Policy dependencies installed and verified."
fi

if container_tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "In-container tmux session '$SESSION_NAME' already exists."
    if [ "$ATTACH" -eq 1 ]; then
        exec docker exec -it "$CONTAINER" \
            tmux attach-session -t "$SESSION_NAME"
    fi
    exit 0
fi

# Do not create duplicate ROS nodes if an older manual bringup is still alive.
EXISTING_NODES="$(docker exec "$CONTAINER" bash -lc \
    "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; sws >/dev/null; timeout 5 ros2 node list" \
    2>/dev/null || true)"
CONFLICTING_NODES="$(printf '%s\n' "$EXISTING_NODES" | grep -E \
    '(^|/)natnet_ros$|(^|/)mocap_bridge$|/fmu/px4_interface$' || true)"
if [ -n "$CONFLICTING_NODES" ]; then
    echo "Refusing to start duplicate lab nodes. Already visible:" >&2
    while IFS= read -r node; do
        printf '  %s\n' "$node" >&2
    done <<< "$CONFLICTING_NODES"
    echo "Stop the previous manual bringup, then run this script again." >&2
    exit 1
fi

if container_running "$DDS_CONTAINER"; then
    DDS_DESCRIPTION="reusing $DDS_CONTAINER"
elif docker ps -a --format '{{.Names}}' | grep -Fxq "$DDS_CONTAINER"; then
    echo "A stopped container named '$DDS_CONTAINER' already exists." >&2
    echo "Remove or rename it before running this launcher." >&2
    exit 1
else
    docker run -d --rm --name "$DDS_CONTAINER" \
        --network host \
        -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
        "$DDS_IMAGE" \
        udp4 --port "$DDS_AGENT_PORT" -v4 >/dev/null
    DDS_DESCRIPTION="started $DDS_CONTAINER on UDP $DDS_AGENT_PORT"
fi

ros_command() {
    local inner_command="$1"
    printf 'bash -lc %q' \
        "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; sws >/dev/null; exec $inner_command"
}

NATNET_INNER="ros2 launch natnet_ros2 natnet_ros2.launch.py serverIP:=$MOTIVE_SERVER_IP clientIP:=$MOTIVE_CLIENT_IP activate:=true pub_rigid_body:=true"
INTERFACES_INNER="ros2 launch svg_ground_control real_interfaces.launch.py drones:=$DRONE_NAME target_systems:=$TARGET_SYSTEMS"
MOCAP_INNER="ros2 run svg_ground_control mocap_bridge --ros-args --params-file \$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/trajectory_commander_drone3_fmu_hover.yaml -p extra_body_names:=[$BALL_NAME]"
POLICY_LAUNCH_COMMAND="ros2 launch svg_ground_control policy_commander.launch.py config:=\$(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/config/drone_soccer/policy_commander_drone3.yaml"
POLICY_START_COMMAND="ros2 service call /policy_commander/start std_srvs/srv/Trigger"
POLICY_STOP_COMMAND="ros2 service call /policy_commander/stop std_srvs/srv/Trigger"
POLICY_GOAL_COMMAND="ros2 topic echo /policy_commander/goal"

NATNET_COMMAND="$(ros_command "$NATNET_INNER")"
INTERFACES_COMMAND="$(ros_command "$INTERFACES_INNER")"
MOCAP_COMMAND="$(ros_command "$MOCAP_INNER")"
SOURCED_SHELL_COMMAND="$(ros_command 'bash')"

container_tmux new-session -d -s "$SESSION_NAME" \
    -n runtime "$NATNET_COMMAND"
container_tmux set-option -t "$SESSION_NAME" remain-on-exit on >/dev/null
NATNET_PANE="$(container_tmux display-message -p \
    -t "$SESSION_NAME:runtime" '#{pane_id}')"
INTERFACES_PANE="$(container_tmux split-window -v -t "$NATNET_PANE" \
    -P -F '#{pane_id}' "$INTERFACES_COMMAND")"
MOCAP_PANE="$(container_tmux split-window -h -t "$INTERFACES_PANE" \
    -P -F '#{pane_id}' "$MOCAP_COMMAND")"
container_tmux select-pane -t "$NATNET_PANE" -T "NatNet"
container_tmux select-pane -t "$INTERFACES_PANE" -T "PX4 interfaces"
container_tmux select-pane -t "$MOCAP_PANE" -T "mocap_bridge"
container_tmux select-layout -t "$SESSION_NAME:runtime" \
    main-horizontal >/dev/null

# Policy commands are intentionally prepopulated without an Enter key.
container_tmux new-window -d -t "$SESSION_NAME" -n policy \
    "$SOURCED_SHELL_COMMAND"
POLICY_LAUNCH_PANE="$(container_tmux display-message -p \
    -t "$SESSION_NAME:policy" '#{pane_id}')"
POLICY_START_PANE="$(container_tmux split-window -h \
    -t "$POLICY_LAUNCH_PANE" -P -F '#{pane_id}' "$SOURCED_SHELL_COMMAND")"
POLICY_STOP_PANE="$(container_tmux split-window -v \
    -t "$POLICY_START_PANE" -P -F '#{pane_id}' "$SOURCED_SHELL_COMMAND")"
POLICY_GOAL_PANE="$(container_tmux split-window -v \
    -t "$POLICY_LAUNCH_PANE" -P -F '#{pane_id}' "$SOURCED_SHELL_COMMAND")"
container_tmux select-pane -t "$POLICY_LAUNCH_PANE" \
    -T "POLICY LAUNCH - press Enter"
container_tmux select-pane -t "$POLICY_START_PANE" \
    -T "POLICY START - press Enter"
container_tmux select-pane -t "$POLICY_STOP_PANE" \
    -T "POLICY STOP - press Enter"
container_tmux select-pane -t "$POLICY_GOAL_PANE" \
    -T "GOAL TOPIC - press Enter"
container_tmux send-keys -t "$POLICY_LAUNCH_PANE" \
    -l "$POLICY_LAUNCH_COMMAND"
container_tmux send-keys -t "$POLICY_START_PANE" \
    -l "$POLICY_START_COMMAND"
container_tmux send-keys -t "$POLICY_STOP_PANE" \
    -l "$POLICY_STOP_COMMAND"
container_tmux send-keys -t "$POLICY_GOAL_PANE" \
    -l "$POLICY_GOAL_COMMAND"
container_tmux select-layout -t "$SESSION_NAME:policy" tiled >/dev/null

container_tmux new-window -d -t "$SESSION_NAME" -n shell \
    "$SOURCED_SHELL_COMMAND"
SHELL_PANE="$(container_tmux display-message -p \
    -t "$SESSION_NAME:shell" '#{pane_id}')"
container_tmux select-pane -t "$SHELL_PANE" -T "Sourced ROS shell"

for window in runtime policy shell; do
    container_tmux set-window-option -t "$SESSION_NAME:$window" \
        pane-border-status top >/dev/null
    container_tmux set-window-option -t "$SESSION_NAME:$window" \
        pane-border-format ' #{pane_title} ' >/dev/null
done

container_tmux select-window -t "$SESSION_NAME:runtime"
container_tmux select-pane -t "$NATNET_PANE"

echo "Created tmux session '$SESSION_NAME' inside $CONTAINER"
echo "  DDS     : $DDS_DESCRIPTION"
echo "  runtime : NatNet + real_interfaces + mocap_bridge started"
echo "  policy  : commands populated only; none executed"
echo "  shell   : sourced and ready for additional ROS commands"
echo
echo "Review all flight-readiness gates before executing policy or flight commands."

if [ "$ATTACH" -eq 1 ]; then
    exec docker exec -it "$CONTAINER" \
        tmux attach-session -t "$SESSION_NAME"
fi
