#!/usr/bin/env bash
# =============================================================================
#  tigris_start_mission.sh — fly one TIGRIS sortie on a running AirStack sim.
#  (A copy of scripts/mtl_start_mission.sh for the tigris_search stack.)
#
#  Prerequisite (see docs/tutorials/tigris_baseline.md):
#      airstack up --sim isaac --fleet tigris_search_fleet --stack tigris_search --play --wait
#  (the fleet selects search_mission_scene.py + this stack's scenario bundle itself)
#
#  For every robot (in parallel) it runs stacks/tigris_search/scripts/tigris_sortie.sh
#  inside that robot's container, with ONE shared run_id: preflight (the
#  robot's planner must be up, or it stays on the ground), rosbag (MCAP) of
#  odometry / TF / gimbal camera / plan / follower / metrics into
#  runs/<run_id>/<robot>/bag/, then the shared sortie client
#  (stacks/mtl_search/scripts/mtl_sortie_client.py, one ROS node per robot) for
#  takeoff + the SearchMission goal, each confirmed from the server's status
#  topic and re-sent if the request was lost. Robots start --stagger s apart so
#  their discovery bursts don't coincide. Ctrl-C stops the sorties INSIDE the
#  containers too (active goals are cancelled, bags closed).
#  Afterwards: the team report + both TIGRIS reward curves
#  (scripts/analyze_tigris_run.py) and the team Foxglove file
#  (scripts/mtl_foxglove.py -> runs/<run_id>/foxglove/).
#
#  usage: bash scripts/tigris_start_mission.sh [-n 1] [-r RUN_ID] [-a 30] [-v 2]
#                                      [--no-takeoff] [--dry-run] [--no-analyze]
#                                      [--no-record] [--no-images] [--no-foxglove]
#                                      [--stagger 2] [--container-prefix airstack-robot-desktop-]
#   -n N          robots robot_1..robot_N  (default: 1 = tigris_search_fleet)
#   -r RUN_ID     run folder name          (default: UTC timestamp)
#   -a ALT        team cruise/takeoff altitude [m] (default: 30 = mission.yaml flight.takeoff_altitude_m);
#                 robot_N adds its layer offset (N-1) * team.altitude_separation_m
#   -v VEL        takeoff velocity [m/s]   (default: 2)
#   --dry-run     plan + publish only (SearchMission start_mission: false), no flight
#   --no-record   no rosbag;  --no-images: bag without gimbal/rgb (~14 MB/s/robot raw)
#   --no-foxglove skip the team Foxglove export
#   --stagger S   seconds between robot starts (default: 2)
# =============================================================================
set -euo pipefail

N=1
RUN_ID="$(date -u +%Y%m%d-%H%M%S)"
ALT=300
VEL=6
TAKEOFF=1
START=true
ANALYZE=1
RECORD=1
IMAGES=1
FOXGLOVE=1
PREFIX="airstack-robot-desktop-"
STAGGER=2

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n) N="$2"; shift 2 ;;
    -r) RUN_ID="$2"; shift 2 ;;
    -a) ALT="$2"; shift 2 ;;
    -v) VEL="$2"; shift 2 ;;
    --no-takeoff) TAKEOFF=0; shift ;;
    --dry-run) START=false; TAKEOFF=0; shift ;;
    --no-analyze) ANALYZE=0; shift ;;
    --no-record) RECORD=0; shift ;;
    --no-images) IMAGES=0; shift ;;
    --no-foxglove) FOXGLOVE=0; shift ;;
    --container-prefix) PREFIX="$2"; shift 2 ;;
    --stagger) STAGGER="$2"; shift 2 ;;
    -h|--help) sed -n '2,37p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MISSION_YAML="${REPO}/stacks/tigris_search/config/mission.yaml"
if [[ -z "${ALT}" ]]; then
  ALT="$(python3 -c "import yaml; m=yaml.safe_load(open('${MISSION_YAML}')).get('mission',{}); print(m.get('flight',{}).get('takeoff_altitude_m', m.get('aircraft',{}).get('altitude_m', 30.0)))" 2>/dev/null || echo 30)"
fi
if [[ -z "${VEL}" ]]; then
  VEL="$(python3 -c "import yaml; m=yaml.safe_load(open('${MISSION_YAML}')).get('mission',{}); print(m.get('flight',{}).get('takeoff_velocity_m_s', 2.0))" 2>/dev/null || echo 2)"
fi
LOG_DIR="${REPO}/runs/${RUN_ID}/_launcher_logs"
mkdir -p "${LOG_DIR}"
echo "[tigris_start_mission] run ${RUN_ID}: ${N} robot(s), takeoff=${TAKEOFF} alt=${ALT} m, start_mission=${START}"

fly_one() {
  local i="$1" name="robot_$1" c="${PREFIX}$1"
  # All the per-robot logic (preflight, rosbag, takeoff, goal-acceptance watchdog)
  # lives in the stack folder, which is mounted into every robot container.
  local args="--robot ${name} --run-id ${RUN_ID} --alt ${ALT} --vel ${VEL}"
  [[ "${TAKEOFF}" == 1 ]] || args+=" --no-takeoff"
  [[ "${START}" == true ]] || args+=" --dry-run"
  [[ "${RECORD}" == 1 ]] || args+=" --no-record"
  [[ "${IMAGES}" == 1 ]] || args+=" --no-images"
  # bash -i: robot/docker/.bashrc resolves ROBOT_NAME / ROS_DOMAIN_ID and defines sws.
  local cmd="sws >/dev/null 2>&1; bash /root/AirStack/stacks/tigris_search/scripts/tigris_sortie.sh ${args}"
  if docker exec -e ROS_DOMAIN_ID="${i}" "${c}" bash -ic "${cmd}" 2>&1 | tee "${LOG_DIR}/${name}.log" | sed -u "s/^/  [${name}] /"; then
    echo "[tigris_start_mission] ${name}: sortie finished (log ${LOG_DIR}/${name}.log)"
    return 0
  fi
  echo "[tigris_start_mission] ${name}: FAILED - last lines of ${LOG_DIR}/${name}.log:" >&2
  tail -n 25 "${LOG_DIR}/${name}.log" | sed "s/^/    [${name}] /" >&2
  return 1
}

# Ctrl-C: `docker exec` does not forward signals, so without this the sortie
# scripts (and their goals / rosbags) keep running inside the containers.
INTERRUPTED=0
stop_sorties() {
  trap '' INT TERM
  INTERRUPTED=1
  echo "[tigris_start_mission] interrupted - stopping the sorties inside the containers (goals are cancelled, bags closed)" >&2
  local kpids=()
  for i in $(seq 1 "${N}"); do
    docker exec "${PREFIX}${i}" pkill -INT -f "tigris_sortie.sh --robot robot_${i} " >/dev/null 2>&1 &
    kpids+=("$!")
  done
  wait "${kpids[@]}" 2>/dev/null || true   # pkill exits 1 for a sortie that already ended
}
trap stop_sorties INT TERM

pids=()
for i in $(seq 1 "${N}"); do
  (( i > 1 )) && sleep "${STAGGER}"
  [[ "${INTERRUPTED}" == 1 ]] && break
  fly_one "${i}" &
  pids+=("$!")
done
fail=0
for p in "${pids[@]}"; do
  # `wait` returns early when the trap fires; wait again for the real exit.
  while :; do
    wait "${p}" && rc=0 || rc=$?
    kill -0 "${p}" 2>/dev/null || break
  done
  (( rc == 0 )) || fail=1
done
if [[ "${INTERRUPTED}" == 1 ]]; then
  echo "[tigris_start_mission] stopped; partial run in ${REPO}/runs/${RUN_ID}" >&2
  exit 130
fi

if [[ "${ANALYZE}" == 1 && "${START}" == true ]]; then
  # Wait briefly for tigris_metrics_logger to write telemetry.csv
  wait_s=0
  max_wait=30
  sleep 10
  while [[ ${wait_s} -lt ${max_wait} ]]; do
    found_all=1
    for i in $(seq 1 "${N}"); do
      if [[ ! -f "${REPO}/runs/${RUN_ID}/robot_${i}/telemetry.csv" ]]; then
        found_all=0
        break
      fi
    done
    # if [[ ! -f "${REPO}/runs/${RUN_ID}/ground_truth.json" || ! -f "${REPO}/runs/${RUN_ID}/belief.png" ]]; then
    #   found_all=0
    # fi
    [[ ${found_all} == 1 ]] && break
    sleep 1
    (( wait_s++ ))
  done
  python3 "${REPO}/scripts/analyze_tigris_run.py" --run-dir "${REPO}/runs/${RUN_ID}" || fail=1
  ln -sfn "${RUN_ID}" "${REPO}/runs/latest" 2>/dev/null || true
  if [[ "${FOXGLOVE}" == 1 ]]; then
    # Team Foxglove file (needs: pip install -r scripts/requirements-mtl-viz.txt)
    python3 "${REPO}/scripts/mtl_foxglove.py" --run-dir "${REPO}/runs/${RUN_ID}" \
      || echo "[tigris_start_mission] Foxglove export failed (the per-robot bags are intact)" >&2
  fi
fi
exit "${fail}"
