#!/usr/bin/env bash
# raven_monitor.sh — tail everything worth watching during a RAYFRONTS_MODE=
# shared raven run: the shared server's own log (offboard-compute), each
# robot's raven_nav log (semantic_search_task's `_spawn` tee — verified in
# node.py: `_spawn(cmd, log_name='raven')` writes
# `/tmp/raven_{robot_name}.log`, `_sanitize('robot_N') == 'robot_N'`
# unchanged, so this is literally `/tmp/raven_robot_N.log`), and a handful of
# status/detection topics, all merged into one file under ~/raven_previews/.
#
# Build plan: _plans/raven_single_rayfronts_shared_plan.md, §5 WP-C item 4.
#
# Usage:
#   scripts/raven_monitor.sh [NUM_ROBOTS]      # default 2
#
# Stop with Ctrl-C — cleans up every background `docker exec` it started.
# Never touches (starts/stops/kills) any compose SERVICE or container itself,
# only reads from already-running ones via `docker exec`.
set -u

NUM_ROBOTS="${1:-2}"
OUT_DIR="${HOME}/raven_previews"
mkdir -p "$OUT_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
OUT_FILE="$OUT_DIR/monitor_${TS}.log"

echo "[raven_monitor] writing merged log to $OUT_FILE"
echo "[raven_monitor] watching offboard-compute + $NUM_ROBOTS robot container(s) — Ctrl-C to stop"

declare -a PIDS=()

cleanup() {
  echo
  echo "[raven_monitor] stopping ($(( ${#PIDS[@]} )) tail(s)) — no container is touched, only local tail processes"
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" >/dev/null 2>&1 || true
  done
  wait 2>/dev/null || true
  echo "[raven_monitor] log kept at $OUT_FILE"
}
trap cleanup INT TERM EXIT

tee_line() {   # tee_line <label> — prefixes every line of stdin with <label>
  local label="$1"
  sed -u "s/^/[$label] /"
}

# ── container-name resolution ────────────────────────────────────────────
# `docker exec` needs the docker CONTAINER name (airstack-robot-desktop-N),
# not the ROS name (robot_N) — mission_runner.py's own fallback pattern
# (`airstack-robot-desktop-{n}` if `docker ps` doesn't have it) is mirrored
# here, since that is this repo's actual naming scheme (robot/docker/
# docker-compose.yaml's robot-desktop service, deploy.replicas).
robot_container() {   # robot_container <n>
  local n="$1" found
  found="$(docker ps --format '{{.Names}}' 2>/dev/null \
    | grep -E "robot.*-desktop.*-${n}\$|robot.*-${n}\$" | head -1)"
  if [ -n "$found" ]; then
    echo "$found"
  else
    echo "airstack-robot-desktop-${n}"
  fi
}

if ! command -v docker >/dev/null 2>&1; then
  echo "[raven_monitor] docker CLI not found — nothing to watch" >&2
  exit 1
fi

# ── offboard-compute: the shared server's own logs ───────────────────────
if docker inspect offboard-compute >/dev/null 2>&1; then
  docker exec offboard-compute bash -c \
    'tail -n +1 -F /tmp/offboard/rayfronts_mapping.log /tmp/offboard/rayfronts_encoder.log 2>&1' \
    2>&1 | tee_line offboard-compute >> "$OUT_FILE" &
  PIDS+=("$!")
else
  echo "[raven_monitor] offboard-compute container not found/running — skipping its log" \
    | tee_line WARN | tee -a "$OUT_FILE"
fi

# ── per-robot: raven's own log tee + status/detection topics ─────────────
for n in $(seq 1 "$NUM_ROBOTS"); do
  robot="robot_${n}"
  container="$(robot_container "$n")"
  if ! docker inspect "$container" >/dev/null 2>&1; then
    echo "[raven_monitor] $container not found/running — skipping $robot" \
      | tee_line WARN | tee -a "$OUT_FILE"
    continue
  fi

  # raven_nav's unfiltered stdout tee (semantic_search_task/node.py's
  # `_spawn(..., log_name='raven')`); only exists once a SemanticSearchTask
  # goal has actually started raven on this robot.
  docker exec "$container" bash -c \
    "tail -n +1 -F /tmp/raven_${robot}.log 2>&1" \
    2>&1 | tee_line "$robot:raven" >> "$OUT_FILE" &
  PIDS+=("$!")

  # Status/detection topics — all std_msgs/String on this robot's OWN
  # ROS_DOMAIN_ID (already set in the container's env), so a bare
  # `source /opt/ros/jazzy/setup.bash` is enough: none of these are custom
  # message types (interface_contract.md / the build plan §2.1/§2.2).
  for topic_pair in \
      "rayfronts_status:/${robot}/rayfronts/status" \
      "discoveries:/${robot}/raven_nav/discoveries" \
      "confirmed_targets:/${robot}/raven_nav/confirmed_targets" \
      "navigation_mode:/${robot}/navigation_mode" \
      "voxel_table:/${robot}/debug/voxel_table" \
      "lvlm_request:/${robot}/raven_nav/lvlm_request" \
      ; do
    name="${topic_pair%%:*}"
    topic="${topic_pair#*:}"
    docker exec "$container" bash -c \
      "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; exec ros2 topic echo '$topic' 2>&1" \
      2>&1 | tee_line "$robot:$name" >> "$OUT_FILE" &
    PIDS+=("$!")
  done
done

echo "[raven_monitor] $(( ${#PIDS[@]} )) tail(s) running — Ctrl-C to stop"
wait
