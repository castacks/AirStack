#!/usr/bin/env bash
# =============================================================================
#  mtl_sortie.sh — fly ONE robot's MTL search sortie, run INSIDE its robot
#  container (the stacks/ folder is mounted there). Called in parallel for every
#  robot by scripts/mtl_start_mission.sh on the host:
#
#    docker exec -e ROS_DOMAIN_ID=N airstack-robot-desktop-N bash -ic \
#      "bash /root/AirStack/stacks/mtl_search/scripts/mtl_sortie.sh --robot robot_N --run-id RUN"
#
#  Sequence (each step fails loudly instead of hanging):
#    1. preflight: this robot's mtl_search_planner node and /<robot>/search_mission
#       action must exist - otherwise it does NOT take off;
#    2. start the rosbag (MCAP) of everything worth replaying in Foxglove:
#       runs/<run_id>/<robot>/bag/  (see TOPICS below);
#    3. wait for a healthy state estimate, take off (requires success: true);
#    4. send the SearchMission goal with an ACCEPTANCE WATCHDOG: if the goal is
#       not accepted within --accept-timeout s, dump diagnostics and retry;
#    5. stream feedback until the result, stop the bag, exit 0 on success.
# =============================================================================
set -uo pipefail

ROBOT="${ROBOT_NAME:-robot_1}"
RUN_ID="$(date -u +%Y%m%d-%H%M%S)"
ALT=30
VEL=2
TAKEOFF=1
START=true
RECORD=1
IMAGES=1
RUNS_ROOT="${MTL_RUNS_ROOT:-/root/AirStack/runs}"
ACCEPT_TIMEOUT=20
RETRIES=3
SCENARIO="${MTL_SCENARIO:-/root/AirStack/stacks/mtl_search/config/scenario.json}"
ALT_OFFSET=1

usage() { sed -n '2,22p' "$0"; cat <<'EOF'
options: --robot NAME --run-id ID --alt M --vel M/S --no-takeoff --dry-run
         --no-record --no-images --runs-root DIR --accept-timeout S --retries N
         --scenario FILE --no-alt-offset
  --alt is the team cruise altitude; this robot's altitude_offset_m from the
  scenario (mission.yaml team.altitude_separation_m) is added unless --no-alt-offset.
EOF
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot) ROBOT="$2"; shift 2 ;;
    --run-id) RUN_ID="$2"; shift 2 ;;
    --alt) ALT="$2"; shift 2 ;;
    --vel) VEL="$2"; shift 2 ;;
    --no-takeoff) TAKEOFF=0; shift ;;
    --dry-run) START=false; TAKEOFF=0; shift ;;
    --no-record) RECORD=0; shift ;;
    --no-images) IMAGES=0; shift ;;
    --runs-root) RUNS_ROOT="$2"; shift 2 ;;
    --accept-timeout) ACCEPT_TIMEOUT="$2"; shift 2 ;;
    --retries) RETRIES="$2"; shift 2 ;;
    --scenario) SCENARIO="$2"; shift 2 ;;
    --no-alt-offset) ALT_OFFSET=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# Vertical deconfliction layer: take off straight to this robot's cruise layer
# (the planner's track is shifted by the same offset, so INGRESS needs no climb).
if [[ "${ALT_OFFSET}" == 1 && -f "${SCENARIO}" ]]; then
  DZ="$(python3 - "${SCENARIO}" "${ROBOT}" <<'PY' 2>/dev/null
import json, sys
sc = json.load(open(sys.argv[1]))
for a in sc.get("team", {}).get("agents", []):
    if a.get("name") == sys.argv[2]:
        print(float(a.get("altitude_offset_m", 0.0) or 0.0)); break
else:
    print(0.0)
PY
)"
  if [[ -n "${DZ}" ]]; then
    ALT="$(python3 -c "print(round(${ALT} + ${DZ}, 3))")"
  fi
fi

R="/${ROBOT}"
OUT="${RUNS_ROOT}/${RUN_ID}/${ROBOT}"
TMP="$(mktemp -d)"
mkdir -p "${OUT}"
log() { echo "[mtl_sortie ${ROBOT} $(date +%H:%M:%S)] $*"; }

if ! command -v ros2 >/dev/null 2>&1; then
  for f in /root/AirStack/robot/ros_ws/install/setup.bash /opt/ros/*/setup.bash; do
    # shellcheck disable=SC1090
    [[ -f "$f" ]] && source "$f" && break
  done
fi

diagnostics() {
  log "---- diagnostics (${ROBOT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}) ----"
  timeout 10 ros2 node list 2>/dev/null | grep -E "${R}/(mtl_|trajectory_control|control/pid|takeoff)" || log "  (no MTL / control nodes visible!)"
  timeout 10 ros2 action info "${R}/search_mission" 2>&1 | sed 's/^/  /'
  timeout 8 ros2 topic echo --once "${R}/search/plan" --field plan_id 2>/dev/null | sed 's/^/  last plan_id: /' \
    || log "  no search/plan latched (planner never planned)"
  timeout 8 ros2 topic echo --once "${R}/search/follower_status" --field state_name 2>/dev/null | sed 's/^/  follower: /' \
    || log "  no search/follower_status (follower not running?)"
  log "---- end diagnostics ----"
}

# ---------------------------------------------------------------- 1. preflight
preflight_ok=0
for _ in $(seq 1 15); do
  if timeout 10 ros2 action list 2>/dev/null | grep -qx "${R}/search_mission" \
     && timeout 10 ros2 node list 2>/dev/null | grep -qx "${R}/mtl_search_planner"; then
    preflight_ok=1; break
  fi
  sleep 2
done
if [[ "${preflight_ok}" != 1 ]]; then
  log "ERROR: ${R}/mtl_search_planner or its ${R}/search_mission action is not running - NOT taking off."
  log "       Check the planner's output in this container's launch tmux (airstack connect robot-desktop-${ROBOT##*_})."
  diagnostics
  exit 3
fi
log "preflight ok: ${R}/mtl_search_planner serves ${R}/search_mission"

# ---------------------------------------------------------------- 2. rosbag
BAG_PID=""
start_bag() {
  [[ "${RECORD}" == 1 ]] || return 0
  local topics=(
    /tf /tf_static
    "${R}/robot_description"
    "${R}/odometry_conversion/odometry"
    "${R}/interface/mavros/global_position/global"
    "${R}/interface/mavros/local_position/pose"
    "${R}/interface/mavros/local_position/velocity_local"
    "${R}/interface/mavros/state"
    "${R}/interface/mavros/battery"
    "${R}/interface/cmd_roll_pitch_yawrate_thrust"
    "${R}/gimbal/camera_info" "${R}/gimbal/state" "${R}/gimbal/cmd_pitch_yaw"
    "${R}/search/plan" "${R}/search/planned_trajectory" "${R}/search/planned_boresight"
    "${R}/search/planned_path" "${R}/search/markers" "${R}/search/follower_status"
    "${R}/search/carrot" "${R}/search/aim_point" "${R}/search/abort"
    "${R}/search/detection_markers" "${R}/search/footprint" "${R}/search/metrics"
    "${R}/search_mission/_action/feedback" "${R}/search_mission/_action/status"
    "${R}/tasks/takeoff/_action/feedback" "${R}/tasks/takeoff/_action/status"
    "${R}/trajectory_controller/tracking_point" "${R}/trajectory_controller/tracking_point_nominal"
    "${R}/trajectory_controller/look_ahead" "${R}/trajectory_controller/trajectory_vis"
    "${R}/behavior/drone_safety_monitor/state_estimate_timed_out"
  )
  [[ "${IMAGES}" == 1 ]] && topics+=("${R}/gimbal/rgb")
  local cfg
  cfg="$(cd "$(dirname "$0")/.." && pwd)/config/rosbag_mcap_storage.yaml"
  rm -rf "${OUT}/bag"
  # set -m: without job control bash starts background jobs with SIGINT ignored,
  # and the recorder must get SIGINT to close the MCAP cleanly.
  set -m
  ros2 bag record -s mcap --storage-config-file "${cfg}" --include-hidden-topics \
      --max-cache-size 1073741824 -o "${OUT}/bag" --topics "${topics[@]}" \
      >"${OUT}/bag_record.log" 2>&1 &
  BAG_PID=$!
  set +m
  sleep 3
  if kill -0 "${BAG_PID}" 2>/dev/null; then
    log "recording ${#topics[@]} topics -> ${OUT}/bag (log: bag_record.log)"
  else
    log "WARNING: ros2 bag record exited early - see ${OUT}/bag_record.log"; BAG_PID=""
  fi
}
stop_bag() {
  [[ -n "${BAG_PID}" ]] || return 0
  sleep 3   # let the last status / markers land
  kill -INT "${BAG_PID}" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "${BAG_PID}" 2>/dev/null || break; sleep 1; done
  kill -0 "${BAG_PID}" 2>/dev/null && kill -TERM "${BAG_PID}" 2>/dev/null
  log "bag closed: $(du -sh "${OUT}/bag" 2>/dev/null | cut -f1)"
  BAG_PID=""
}
cleanup() { stop_bag; rm -rf "${TMP}"; }
trap cleanup EXIT
trap 'log "interrupted"; exit 130' INT TERM

start_bag

# ---------------------------------------------------------------- 3. takeoff
if [[ "${TAKEOFF}" == 1 ]]; then
  for _ in $(seq 1 9); do
    timeout 8 ros2 topic echo --once "${R}/behavior/drone_safety_monitor/state_estimate_timed_out" 2>/dev/null \
      | grep -q 'data: false' && break
    sleep 2
  done
  log "takeoff to ${ALT} m at ${VEL} m/s"
  tk="$(PYTHONUNBUFFERED=1 ros2 action send_goal "${R}/tasks/takeoff" task_msgs/action/TakeoffTask \
        "{target_altitude_m: ${ALT}, velocity_m_s: ${VEL}}" 2>&1)"
  echo "${tk}"
  if ! echo "${tk}" | grep -qi 'success: true'; then
    log "ERROR: takeoff failed"; exit 4
  fi
fi

# ---------------------------------------------------------------- 4. mission goal
GOAL="{start_mission: ${START}, run_id: \"${RUN_ID}\"}"
attempt=1
while :; do
  log "sending ${R}/search_mission goal (attempt ${attempt}/${RETRIES}): ${GOAL}"
  out="${TMP}/goal_${attempt}.log"
  # PYTHONUNBUFFERED: the CLI's "Goal accepted" must reach the file immediately,
  # or the watchdog below would mistake stdio buffering for a lost goal.
  PYTHONUNBUFFERED=1 ros2 action send_goal --feedback "${R}/search_mission" mtl_msgs/action/SearchMission \
      "${GOAL}" >"${out}" 2>&1 &
  gpid=$!
  accepted=0
  for _ in $(seq 1 $((ACCEPT_TIMEOUT * 2))); do
    if grep -q "Goal accepted" "${out}"; then accepted=1; break; fi
    if grep -q "Goal was rejected" "${out}" || ! kill -0 "${gpid}" 2>/dev/null; then break; fi
    sleep 0.5
  done
  if [[ "${accepted}" == 1 ]]; then
    log "goal accepted"
    tail -n +1 --pid="${gpid}" -f "${out}"
    wait "${gpid}"
    break
  fi
  kill -INT "${gpid}" 2>/dev/null; sleep 1; kill -KILL "${gpid}" 2>/dev/null
  cat "${out}"
  if (( attempt > 1 )) && grep -q "Goal was rejected" "${out}"; then
    # The planner rejects only while a sortie is active: an earlier attempt WAS
    # accepted (its response got lost). Follow that sortie instead of re-sending.
    log "planner reports a sortie already active - following it via search/follower_status"
    state=""
    for _ in $(seq 1 600); do
      state="$(timeout 8 ros2 topic echo --once "${R}/search/follower_status" --field state_name 2>/dev/null | head -n1)"
      [[ "${state}" == COMPLETE || "${state}" == ABORTED ]] && break
      sleep 2
    done
    log "sortie ended: ${state:-unknown}"
    [[ "${state}" == COMPLETE ]] && exit 0
    exit 6
  fi
  log "goal NOT accepted within ${ACCEPT_TIMEOUT} s"
  diagnostics
  if (( attempt >= RETRIES )); then
    log "ERROR: giving up after ${RETRIES} attempts; the drone is left hovering at the takeoff point."
    exit 5
  fi
  attempt=$((attempt + 1))
  sleep 3
done

# ---------------------------------------------------------------- 5. result
if grep -qi 'success: true' "${out}"; then
  log "sortie succeeded"
  exit 0
fi
log "sortie did not succeed (see the result above)"
exit 6
