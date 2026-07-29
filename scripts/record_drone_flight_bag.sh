#!/usr/bin/env bash
# Record a drone flight bag from the AirStack robot container.
#
# Default use for the current Starling setup:
#   ./scripts/record_drone_flight_bag.sh
#
# Stop after landing/disarming with Ctrl-C. Bags are written inside the robot
# container under /bags, which Docker Compose bind-mounts to robot/bags.

set -euo pipefail

CONTAINER="airstack-robot-desktop-1"
DRONE="drone_4"
BALL="VolleyBall"
INCLUDE_BALL=1 # 0 = do not record ball topic, 1 = record ball topic
OUTPUT_DIR="/bags"
CHECK_ONLY=0
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"

usage() {
    cat <<'EOF'
Usage:
  record_drone_flight_bag.sh [options]

Options:
  --container NAME    Docker container to run rosbag in.
                      Default: airstack-robot-desktop-1
  --drone NAME        Drone namespace / rigid body name.
                      Default: drone_4
  --ball NAME         Ball rigid body topic name without slashes.
                      Default: VolleyBall
  --no-ball           Do not record a ball topic.
  --output-dir PATH   Container output directory.
                      Default: /bags
  --check-only        Only check topic visibility; do not start recording.
  -h, --help          Show this help.

Examples:
  ./scripts/record_drone_flight_bag.sh
  ./scripts/record_drone_flight_bag.sh --ball VolleyBall
  ./scripts/record_drone_flight_bag.sh --drone drone_2 --ball SoccerBall
  ./scripts/record_drone_flight_bag.sh --no-ball

Stop recording with Ctrl-C after landing/disarming.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --container)
            CONTAINER="${2:?--container needs a value}"
            shift 2
            ;;
        --drone)
            DRONE="${2:?--drone needs a value}"
            shift 2
            ;;
        --ball)
            BALL="${2:?--ball needs a value}"
            INCLUDE_BALL=1
            shift 2
            ;;
        --no-ball)
            INCLUDE_BALL=0
            shift
            ;;
        --output-dir)
            OUTPUT_DIR="${2:?--output-dir needs a value}"
            shift 2
            ;;
        --check-only)
            CHECK_ONLY=1
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

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' is not running." >&2
    exit 1
fi

sanitize() {
    printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

STAMP="$(date +%Y%m%d_%H%M%S)"
NAME_PART="$(sanitize "${DRONE}")"
if [ "$INCLUDE_BALL" -eq 1 ]; then
    NAME_PART="${NAME_PART}_$(sanitize "${BALL}")"
fi

BAG_DIR="${OUTPUT_DIR}/${NAME_PART}_${STAMP}"
LATEST_FILE="${OUTPUT_DIR}/latest_${NAME_PART}_bag.txt"
TMUX_SESSION="bag_record_${NAME_PART}_${STAMP}"

TOPICS=(
    "/${DRONE}/pose"
    "/${DRONE}/odometry_conversion/odometry"
    "/${DRONE}/fmu/pose_command"
    "/policy_commander/obs"
    "/policy_commander/action"
    "/policy_commander/waypoint"
    "/${DRONE}/fmu/in/vehicle_visual_odometry"
    "/${DRONE}/fmu/out/vehicle_local_position"
    "/${DRONE}/fmu/out/vehicle_odometry"
    "/${DRONE}/fmu/out/vehicle_attitude"
    "/${DRONE}/fmu/out/vehicle_status"
    "/${DRONE}/fmu/out/battery_status"
    "/${DRONE}/fmu/out/failsafe_flags"
    "/${DRONE}/fmu/in/trajectory_setpoint"
    "/${DRONE}/fmu/in/offboard_control_mode"
)

if [ "$INCLUDE_BALL" -eq 1 ]; then
    TOPICS+=("/${BALL}/pose" "/${BALL}/mocap_odometry")
fi

printf -v TOPIC_ARGS '%q ' "${TOPICS[@]}"

echo "Recording flight bag"
echo "  container : $CONTAINER"
echo "  drone     : $DRONE"
if [ "$INCLUDE_BALL" -eq 1 ]; then
    echo "  ball      : $BALL -> /${BALL}/pose"
else
    echo "  ball      : disabled"
fi
echo "  output    : $BAG_DIR"
echo "  storage   : mcap"
echo "  ROS domain: $ROS_DOMAIN_ID"
if [ "$OUTPUT_DIR" = "/bags" ]; then
    HOST_BAG="/home/yutongw/Desktop/AirStack/robot/bags/$(basename "$BAG_DIR")"
    echo "  host path : $HOST_BAG"
    echo "  mount     : /bags is Docker Compose bind-mounted from robot/bags"
    echo "              No docker cp is needed when using the default output dir."
fi
echo
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Checking visible topics in container..."

docker exec "$CONTAINER" bash -lc "
set -euo pipefail
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
set +u
sws >/dev/null
set -u
VISIBLE_TOPICS=\"\$(ros2 topic list)\"
MISSING_TOPICS=0
for topic in $TOPIC_ARGS; do
    if grep -Fxq \"\$topic\" <<< \"\$VISIBLE_TOPICS\"; then
        PUB_COUNT=\"\$(ros2 topic info \"\$topic\" 2>/dev/null | awk '/Publisher count:/ {print \$3}')\"
        if [ \"\${PUB_COUNT:-0}\" -gt 0 ]; then
            echo \"  OK      \$topic (publishers: \$PUB_COUNT)\"
        else
            echo \"  NO PUB  \$topic\"
            MISSING_TOPICS=1
        fi
    else
        echo \"  MISSING \$topic\"
        MISSING_TOPICS=1
    fi
done
if [ \"\$MISSING_TOPICS\" -ne 0 ]; then
    echo
    echo 'Refusing to record because one or more required topics are unavailable.' >&2
    exit 1
fi
"

echo
if [ "$CHECK_ONLY" -eq 1 ]; then
    echo "--check-only set; not starting ros2 bag record."
    exit 0
fi

docker exec "$CONTAINER" bash -lc "
set -euo pipefail
mkdir -p '$OUTPUT_DIR'
echo '$BAG_DIR' > '$LATEST_FILE'
"

printf -v QUOTED_DOMAIN '%q' "$ROS_DOMAIN_ID"
printf -v QUOTED_BAG_DIR '%q' "$BAG_DIR"
RECORD_COMMAND="\
set -euo pipefail; \
export ROS_DOMAIN_ID=$QUOTED_DOMAIN; \
set +u; \
sws >/dev/null; \
set -u; \
exec ros2 bag record $TOPIC_ARGS -s mcap -o $QUOTED_BAG_DIR"

docker exec "$CONTAINER" tmux new-session -d -s "$TMUX_SESSION" \
    bash -lc "$RECORD_COMMAND"

echo "Starting ros2 bag record in container tmux session: $TMUX_SESSION"
echo "Press Ctrl-C after landing/disarming."
echo

sleep 1
if ! docker exec "$CONTAINER" tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "Recorder session exited during startup." >&2
    exit 1
fi
docker exec "$CONTAINER" tmux capture-pane -pt "$TMUX_SESSION" -S -40 || true

STOP_REQUESTED=0
STOP_FAILED=0

stop_recording() {
    if ! docker exec "$CONTAINER" tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
        return 0
    fi

    echo
    echo "Stopping recorder and finalizing metadata..."
    docker exec "$CONTAINER" tmux send-keys -t "$TMUX_SESSION" C-c

    for _ in $(seq 1 40); do
        if ! docker exec "$CONTAINER" tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
            return 0
        fi
        sleep 0.25 || true
    done

    echo "Recorder did not stop within 10 seconds; session is still running." >&2
    return 1
}

handle_stop_signal() {
    STOP_REQUESTED=1
    if ! stop_recording; then
        STOP_FAILED=1
    fi
}

trap handle_stop_signal INT TERM

while docker exec "$CONTAINER" tmux has-session -t "$TMUX_SESSION" 2>/dev/null; do
    sleep 1 || true
    if [ "$STOP_REQUESTED" -eq 1 ]; then
        break
    fi
done

trap - INT TERM

echo
if [ "$STOP_FAILED" -ne 0 ] || \
   docker exec "$CONTAINER" tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "Recording has not stopped cleanly." >&2
    exit 1
fi

if ! docker exec "$CONTAINER" test -f "$BAG_DIR/metadata.yaml"; then
    echo "Recorder stopped, but metadata.yaml was not created in $BAG_DIR." >&2
    exit 1
fi

echo "Recording stopped and metadata finalized."
echo "Latest bag marker in container: $LATEST_FILE"
echo "Bag path in container: $BAG_DIR"
if [ "$OUTPUT_DIR" = "/bags" ]; then
    HOST_BAG="/home/yutongw/Desktop/AirStack/robot/bags/$(basename "$BAG_DIR")"
    echo "Host path via Docker Compose /bags bind mount: $HOST_BAG"
fi

exit 0
