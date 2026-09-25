#!/usr/bin/env bash
# =============================================================================
#  mtl_start_mission.sh — fly one MTL team sortie on a running AirStack sim.
#
#  Prerequisite (see docs/tutorials/mtl_target_localization.md):
#    ISAAC_SIM_SCRIPT_NAME=search_mission_scene.py \
#      airstack up --sim isaac --fleet mtl_search_fleet --stack mtl_search --play --wait
#
#  For every robot (in parallel) it runs stacks/mtl_search/scripts/mtl_sortie.sh
#  inside that robot's container, with ONE shared run_id: preflight (the
#  robot's planner must be up, or it stays on the ground), rosbag (MCAP) of
#  odometry / TF / gimbal camera / plan / follower / metrics into
#  runs/<run_id>/<robot>/bag/, takeoff, the SearchMission goal with an
#  acceptance watchdog + retries. Afterwards: the team report
#  (scripts/analyze_mtl_run.py) and the team Foxglove file
#  (scripts/mtl_foxglove.py -> runs/<run_id>/foxglove/).
#
#  usage: bash scripts/mtl_start_mission.sh [-n 3] [-r RUN_ID] [-a 30] [-v 2]
#                                      [--no-takeoff] [--dry-run] [--no-analyze]
#                                      [--no-record] [--no-images] [--no-foxglove]
#                                      [--container-prefix airstack-robot-desktop-]
#   -n N          robots robot_1..robot_N  (default: 3 = mtl_search_fleet)
#   -r RUN_ID     run folder name          (default: UTC timestamp)
#   -a ALT        team cruise/takeoff altitude [m] (default: 30 = mission.yaml flight.takeoff_altitude_m);
#                 robot_N adds its layer offset (N-1) * team.altitude_separation_m
#   -v VEL        takeoff velocity [m/s]   (default: 2)
#   --dry-run     plan + publish only (SearchMission start_mission: false), no flight
#   --no-record   no rosbag;  --no-images: bag without gimbal/rgb (~14 MB/s/robot raw)
#   --no-foxglove skip the team Foxglove export
# =============================================================================
set -euo pipefail

N=3
RUN_ID="$(date -u +%Y%m%d-%H%M%S)"
ALT=30
VEL=2
TAKEOFF=1
START=true
ANALYZE=1
RECORD=1
IMAGES=1
FOXGLOVE=1
PREFIX="airstack-robot-desktop-"

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
    -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${REPO}/runs/${RUN_ID}/_launcher_logs"
mkdir -p "${LOG_DIR}"
echo "[mtl_start_mission] run ${RUN_ID}: ${N} robot(s), takeoff=${TAKEOFF} alt=${ALT} m, start_mission=${START}"

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
  local cmd="sws >/dev/null 2>&1; bash /root/AirStack/stacks/mtl_search/scripts/mtl_sortie.sh ${args}"
  if docker exec -e ROS_DOMAIN_ID="${i}" "${c}" bash -ic "${cmd}" >"${LOG_DIR}/${name}.log" 2>&1; then
    echo "[mtl_start_mission] ${name}: sortie finished (log ${LOG_DIR}/${name}.log)"
    return 0
  fi
  echo "[mtl_start_mission] ${name}: FAILED - last lines of ${LOG_DIR}/${name}.log:" >&2
  tail -n 25 "${LOG_DIR}/${name}.log" | sed "s/^/    [${name}] /" >&2
  return 1
}

pids=()
for i in $(seq 1 "${N}"); do
  fly_one "${i}" &
  pids+=("$!")
done
fail=0
for p in "${pids[@]}"; do wait "${p}" || fail=1; done

if [[ "${ANALYZE}" == 1 && "${START}" == true ]]; then
  python3 "${REPO}/scripts/analyze_mtl_run.py" --run-dir "${REPO}/runs/${RUN_ID}" || fail=1
  ln -sfn "${RUN_ID}" "${REPO}/runs/latest" 2>/dev/null || true
  if [[ "${FOXGLOVE}" == 1 ]]; then
    # Team Foxglove file (needs: pip install -r scripts/requirements-mtl-viz.txt)
    python3 "${REPO}/scripts/mtl_foxglove.py" --run-dir "${REPO}/runs/${RUN_ID}" \
      || echo "[mtl_start_mission] Foxglove export failed (the per-robot bags are intact)" >&2
  fi
fi
exit "${fail}"
