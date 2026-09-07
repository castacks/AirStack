#!/usr/bin/env bash
# Durable handoff for the assigned two-GPU shared-RAVEN pod.
#
# Start this while the current Fire/Suburban recovery is active. It waits for
# that exact runner, then starts one fresh <=12 h runner per three-scene batch.
# `set -e` intentionally halts the queue on any exhausted/failed batch so a
# repeated fault is inspected instead of blindly burning the remaining cells.
set -euo pipefail

launcher_pid=${1:-59}
current_runner_pid=${2:?current mission-runner PID required}
cd /root/AirStack

[[ $(ps -p "$launcher_pid" -o args=) == *mission_launcher.sh* ]]
[[ $(ps -p "$launcher_pid" -o stat=) == *T* ]]

while kill -0 "$current_runner_pid" 2>/dev/null; do
  sleep 30
done

missions=(
  osmo/missions/raven_hurricane_suburban_remaining_2gpu1.yaml
  osmo/missions/raven_tornado_suburban_remaining_2gpu1.yaml
  osmo/missions/raven_earthquake_suburban_remaining_2gpu1.yaml
  osmo/missions/raven_fire_urban_remaining_2gpu1.yaml
  osmo/missions/raven_hurricane_urban_remaining_2gpu1.yaml
  osmo/missions/raven_tornado_urban_remaining_2gpu1.yaml
  osmo/missions/raven_earthquake_urban_remaining_2gpu1.yaml
)

for mission in "${missions[@]}"; do
  [[ $(ps -p "$launcher_pid" -o stat=) == *T* ]]
  echo "[$(date -u +%FT%TZ)] starting $mission"
  bash osmo/workspace/run_held_mission.sh "$launcher_pid" "$mission"
  echo "[$(date -u +%FT%TZ)] passed and uploaded $mission"
done
