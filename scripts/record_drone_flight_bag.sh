#!/usr/bin/env bash
# Record a drone flight bag from the AirStack robot container.
#
# Default use for the current Starling setup:
#   ./scripts/record_drone_flight_bag.sh
#
# Stop after landing/disarming with Ctrl-C. Bags are written inside the robot
# container under /bags, which Docker Compose bind-mounts to robot/bags.

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="airstack-robot-desktop-1"
DRONES=("drone_2" "drone_3")
DRONES_OVERRIDDEN=0
BALL="VolleyBall"
INCLUDE_BALL=1 # 0 = do not record ball topic, 1 = record ball topic
OUTPUT_DIR="/bags"
CHECK_ONLY=0
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
EXCLUDED_TOPICS=()

usage() {
    cat <<'EOF'
Usage:
  record_drone_flight_bag.sh [options]

Options:
  --container NAME    Docker container to run rosbag in.
                      Default: airstack-robot-desktop-1
  --drone NAME        Drone namespace / rigid body name. May be repeated.
                      Default: drone_2 and drone_3
  --ball NAME         Ball rigid body topic name without slashes.
                      Default: VolleyBall
  --no-ball           Do not record a ball topic.
  --output-dir PATH   Container output directory.
                      Default: /bags
  --exclude-topic TOPIC
                      Do not check or record TOPIC. May be repeated.
  --check-only        Only check topic visibility; do not start recording.
  -h, --help          Show this help.

Examples:
  ./scripts/record_drone_flight_bag.sh
  ./scripts/record_drone_flight_bag.sh --ball VolleyBall
  ./scripts/record_drone_flight_bag.sh --drone drone_2 --ball VolleyBall
  ./scripts/record_drone_flight_bag.sh --drone drone_2 --drone drone_3
  ./scripts/record_drone_flight_bag.sh --no-ball
  ./scripts/record_drone_flight_bag.sh --ball SoccerBall \
    --exclude-topic /drone_2/policy_commander/obs \
    --exclude-topic /drone_2/policy_commander/action \
    --exclude-topic /drone_2/policy_commander/waypoint

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
            if [ "$DRONES_OVERRIDDEN" -eq 0 ]; then
                DRONES=()
                DRONES_OVERRIDDEN=1
            fi
            DRONES+=("${2:?--drone needs a value}")
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
        --exclude-topic)
            EXCLUDED_TOPICS+=("${2:?--exclude-topic needs a value}")
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
NAME_PART=""
for drone in "${DRONES[@]}"; do
    if [ -n "$NAME_PART" ]; then
        NAME_PART="${NAME_PART}_"
    fi
    NAME_PART="${NAME_PART}$(sanitize "$drone")"
done
if [ "$INCLUDE_BALL" -eq 1 ]; then
    NAME_PART="${NAME_PART}_$(sanitize "${BALL}")"
fi

BAG_DIR="${OUTPUT_DIR}/${NAME_PART}_${STAMP}"
LATEST_FILE="${OUTPUT_DIR}/latest_${NAME_PART}_bag.txt"
TMUX_SESSION="bag_record_${NAME_PART}_${STAMP}"

TOPICS=()
for drone in "${DRONES[@]}"; do
    TOPICS+=(
        "/${drone}/pose"
        "/${drone}/odometry_conversion/odometry"
        "/${drone}/policy_commander/obs"
        "/${drone}/policy_commander/action"
        "/${drone}/policy_commander/waypoint"
        "/${drone}/fmu/in/vehicle_visual_odometry"
        "/${drone}/fmu/in/trajectory_setpoint"
        "/${drone}/fmu/in/offboard_control_mode"
        "/${drone}/fmu/out/vehicle_local_position"
        "/${drone}/fmu/out/vehicle_odometry"
        "/${drone}/fmu/out/vehicle_attitude"
        "/${drone}/fmu/out/vehicle_status"
        "/${drone}/fmu/out/battery_status"
        # Temporarily unavailable on drone_2; restore when PX4 publishes them.
        # "/${drone}/fmu/out/hover_thrust_estimate"
        # "/${drone}/fmu/out/actuator_motors"
        "/${drone}/fmu/out/failsafe_flags"
    )
done

if [ "$INCLUDE_BALL" -eq 1 ]; then
    TOPICS+=("/${BALL}/pose" "/${BALL}/mocap_odometry")
fi

if [ "${#EXCLUDED_TOPICS[@]}" -gt 0 ]; then
    FILTERED_TOPICS=()
    for topic in "${TOPICS[@]}"; do
        excluded=0
        for excluded_topic in "${EXCLUDED_TOPICS[@]}"; do
            if [ "$topic" = "$excluded_topic" ]; then
                excluded=1
                break
            fi
        done
        if [ "$excluded" -eq 0 ]; then
            FILTERED_TOPICS+=("$topic")
        fi
    done
    TOPICS=("${FILTERED_TOPICS[@]}")
fi

printf -v TOPIC_ARGS '%q ' "${TOPICS[@]}"

echo "Recording flight bag"
echo "  container : $CONTAINER"
echo "  drones    : ${DRONES[*]}"
if [ "$INCLUDE_BALL" -eq 1 ]; then
    echo "  ball      : $BALL -> /${BALL}/pose"
else
    echo "  ball      : disabled"
fi
echo "  output    : $BAG_DIR"
echo "  storage   : mcap"
echo "  ROS domain: $ROS_DOMAIN_ID"
if [ "$OUTPUT_DIR" = "/bags" ]; then
    HOST_BAG="$REPO_ROOT/robot/bags/$(basename "$BAG_DIR")"
    echo "  host path : $HOST_BAG"
    echo "  mount     : /bags is Docker Compose bind-mounted from robot/bags"
    echo "              No docker cp is needed when using the default output dir."
fi
echo
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
if [ "${#EXCLUDED_TOPICS[@]}" -gt 0 ]; then
    echo
    echo "Excluded topics:"
    printf '  %s\n' "${EXCLUDED_TOPICS[@]}"
fi
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
