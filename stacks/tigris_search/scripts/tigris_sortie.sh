#!/usr/bin/env bash
# =============================================================================
#  tigris_sortie.sh — fly ONE robot's TIGRIS search sortie, run INSIDE its robot
#  container (the stacks/ folder is mounted there). The same sequence as
#  stacks/mtl_search/scripts/mtl_sortie.sh, with the planner node, the scenario
#  path and the bag topic list adapted; called by scripts/tigris_start_mission.sh:
#
#    docker exec -e ROS_DOMAIN_ID=1 airstack-robot-desktop-1 bash -ic \
#      "bash /root/AirStack/stacks/tigris_search/scripts/tigris_sortie.sh --robot robot_1 --run-id RUN"
#
#  Sequence (each step fails loudly instead of hanging):
#    1. preflight: this robot's tigris_search_planner node, its /<robot>/search_mission
#       action and the /<robot>/tasks/takeoff action must exist - otherwise it
#       does NOT take off;
#    2. start the rosbag (MCAP) of everything worth replaying in Foxglove:
#       runs/<run_id>/<robot>/bag/  (see TOPICS below), and wait until the
#       recorder has finished subscribing (its discovery burst is over);
#    3+4. stacks/mtl_search/scripts/mtl_sortie_client.py (shared, planner-agnostic;
#       run with --tag tigris_sortie) - ONE ROS node does takeoff and the SearchMission
#       goal: waits for a healthy state estimate, confirms each goal's acceptance
#       from the goal response OR the server's status topic, and resends when a
#       goal request is lost (the old `ros2 action send_goal` CLI calls hung
#       forever when that happened: "drone never takes off / never searches");
#    5. stop the bag; exit with the client's code.
#  SIGINT/SIGTERM: the active goal is cancelled (drone holds position), the bag
#  is closed cleanly.
# =============================================================================
set -uo pipefail

ROBOT="${ROBOT_NAME:-robot_1}"
RUN_ID="$(date -u +%Y%m%d-%H%M%S)"
ALT=30
VEL=2
TAKEOFF=1
DRY_RUN=0
RECORD=1
IMAGES=1
RUNS_ROOT="${MTL_RUNS_ROOT:-/root/AirStack/runs}"
ACCEPT_TIMEOUT=10
RETRIES=4
SCENARIO="${TIGRIS_SCENARIO:-/root/AirStack/stacks/tigris_search/config/scenario.json}"
PLANNER_NODE="tigris_search_planner"
ALT_OFFSET=1
HERE="$(cd "$(dirname "$0")" && pwd)"
# The sortie client is shared with the mtl_search stack (same actions, same messages).
CLIENT="${TIGRIS_SORTIE_CLIENT:-${HERE}/../../mtl_search/scripts/mtl_sortie_client.py}"

usage() { sed -n '2,27p' "$0"; cat <<'EOF'
options: --robot NAME --run-id ID --alt M --vel M/S --no-takeoff --dry-run
         --no-record --no-images --runs-root DIR --accept-timeout S --retries N
         --scenario FILE --no-alt-offset
  --alt is the team cruise altitude; this robot's altitude_offset_m from the
  scenario (mission.yaml team.altitude_separation_m) is added unless --no-alt-offset.
  --accept-timeout / --retries: per goal attempt, passed to the sortie client.
EOF
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot) ROBOT="$2"; shift 2 ;;
    --run-id) RUN_ID="$2"; shift 2 ;;
    --alt) ALT="$2"; shift 2 ;;
    --vel) VEL="$2"; shift 2 ;;
    --no-takeoff) TAKEOFF=0; shift ;;
    --dry-run) DRY_RUN=1; TAKEOFF=0; shift ;;
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
mkdir -p "${OUT}"
log() { echo "[tigris_sortie ${ROBOT} $(date +%H:%M:%S)] $*"; }

if ! command -v ros2 >/dev/null 2>&1; then
  for f in /root/AirStack/robot/ros_ws/install/setup.bash /opt/ros/*/setup.bash; do
    # shellcheck disable=SC1090
    [[ -f "$f" ]] && source "$f" && break
  done
fi

diagnostics() {
  log "---- diagnostics (${ROBOT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}) ----"
  timeout 10 ros2 node list 2>/dev/null | grep -E "${R}/(tigris_|mtl_|trajectory_control|control/pid|takeoff)" || log "  (no TIGRIS / MTL / control nodes visible!)"
  timeout 10 ros2 action info "${R}/search_mission" 2>&1 | sed 's/^/  /'
  timeout 10 ros2 action info "${R}/tasks/takeoff" 2>&1 | sed 's/^/  /'
  timeout 8 ros2 topic echo --once "${R}/search/plan" --field plan_id 2>/dev/null | sed 's/^/  last plan_id: /' \
    || log "  no search/plan latched (planner never planned)"
  timeout 8 ros2 topic echo --once "${R}/search/follower_status" --field state_name 2>/dev/null | sed 's/^/  follower: /' \
    || log "  no search/follower_status (follower not running?)"
  log "---- end diagnostics ----"
}

# ---------------------------------------------------------------- 1. preflight
need_actions=("${R}/search_mission")
[[ "${TAKEOFF}" == 1 ]] && need_actions+=("${R}/tasks/takeoff")
preflight_ok=0
missing=""
for _ in $(seq 1 15); do
  actions="$(timeout 10 ros2 action list 2>/dev/null)"
  missing=""
  for a in "${need_actions[@]}"; do
    grep -qx -- "${a}" <<<"${actions}" || missing+=" ${a}"
  done
  if [[ -z "${missing}" ]] && timeout 10 ros2 node list 2>/dev/null | grep -qx "${R}/${PLANNER_NODE}"; then
    preflight_ok=1; break
  fi
  sleep 2
done
if [[ "${preflight_ok}" != 1 ]]; then
  log "ERROR: preflight failed (missing:${missing:- ${R}/${PLANNER_NODE} node}) - NOT taking off."
  log "       Check that node's output in this container's launch tmux (airstack connect robot-desktop-${ROBOT##*_})."
  diagnostics
  exit 3
fi
log "preflight ok: ${R}/${PLANNER_NODE} serves ${R}/search_mission$([[ "${TAKEOFF}" == 1 ]] && echo ", ${R}/tasks/takeoff is up")"

# ---------------------------------------------------------------- 2. rosbag
BAG_PID=""
CLIENT_PID=""
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
    "${R}/search/planned_path" "${R}/search/markers" "${R}/search/follower_status" "${R}/search/tigris_status"
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
  cfg="$(cd "${HERE}/.." && pwd)/config/rosbag_mcap_storage.yaml"
  rm -rf "${OUT}/bag"
  # set -m: without job control bash starts background jobs with SIGINT ignored,
  # and the recorder must get SIGINT to close the MCAP cleanly.
  set -m
  ros2 bag record -s mcap --storage-config-file "${cfg}" --include-hidden-topics \
      --max-cache-size 1073741824 -o "${OUT}/bag" --topics "${topics[@]}" \
      >"${OUT}/bag_record.log" 2>&1 &
  BAG_PID=$!
  set +m
  sleep 1
  if ! kill -0 "${BAG_PID}" 2>/dev/null; then
    log "WARNING: ros2 bag record exited early - see ${OUT}/bag_record.log"; BAG_PID=""; return 0
  fi
  # Wait until the recorder stops adding subscriptions (count stable for 2 s,
  # at most 15 s): its discovery burst should not overlap the goal requests.
  local n=0 prev=-1 stable=0
  for _ in $(seq 1 30); do
    n="$(grep -c "Subscribed to topic" "${OUT}/bag_record.log" 2>/dev/null || true)"
    n="${n:-0}"
    if [[ "${n}" == "${prev}" && "${n}" -gt 0 ]]; then
      stable=$((stable + 1)); (( stable >= 4 )) && break
    else
      stable=0
    fi
    prev="${n}"
    sleep 0.5
  done
  log "recording ${#topics[@]} topics (${n} subscribed) -> ${OUT}/bag (log: bag_record.log)"
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
on_signal() {
  log "interrupted"
  if [[ -n "${CLIENT_PID}" ]] && kill -0 "${CLIENT_PID}" 2>/dev/null; then
    kill -INT "${CLIENT_PID}" 2>/dev/null        # cancels the active goal
    for _ in $(seq 1 10); do kill -0 "${CLIENT_PID}" 2>/dev/null || break; sleep 0.5; done
    kill -KILL "${CLIENT_PID}" 2>/dev/null
  fi
  exit 130
}
trap stop_bag EXIT
trap on_signal INT TERM

start_bag

# ---------------------------------------------------------------- 3+4. takeoff + mission
client_args=(--robot "${ROBOT}" --run-id "${RUN_ID}" --alt "${ALT}" --vel "${VEL}"
             --accept-timeout "${ACCEPT_TIMEOUT}" --retries "${RETRIES}" --tag tigris_sortie)
[[ "${TAKEOFF}" == 1 ]] || client_args+=(--no-takeoff)
[[ "${DRY_RUN}" == 1 ]] && client_args+=(--dry-run)
# Background + wait (not foreground) so the INT/TERM trap runs immediately and
# can forward the signal; the client installs its own SIGINT/SIGTERM handlers.
if [[ ! -f "${CLIENT}" ]]; then
  log "ERROR: sortie client not found: ${CLIENT}"; exit 2
fi
python3 -u "${CLIENT}" "${client_args[@]}" &
CLIENT_PID=$!
wait "${CLIENT_PID}"
rc=$?
CLIENT_PID=""

# ---------------------------------------------------------------- 5. result
case "${rc}" in
  0) log "sortie succeeded" ;;
  3) log "ERROR: an action server vanished before the goal could be sent"; diagnostics ;;
  4) log "ERROR: takeoff failed" ;;
  5) log "ERROR: a goal was never accepted; the drone is left where it is (on the ground or hovering)"; diagnostics ;;
  130) log "interrupted" ;;
  *) log "sortie did not succeed (exit ${rc}; see the result above)" ;;
esac
exit "${rc}"
