#!/usr/bin/env bash
# Durable handoff for the assigned two-GPU shared-RAVEN pod.
#
# Start this while the current Fire/Suburban recovery is active. It waits for
# that exact runner, then starts one fresh <=12 h runner per scene.
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
  osmo/missions/raven_firesuburbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_firesuburbanl3v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_hurricanesuburbanl1v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_hurricanesuburbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_hurricanesuburbanl3v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_tornadosuburbanl1v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_tornadosuburbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_tornadosuburbanl3v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_earthquakesuburbanl1v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_earthquakesuburbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_earthquakesuburbanl3v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_fireurbanl1v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_fireurbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_fireurbanl3v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_hurricaneurbanl1v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_hurricaneurbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_hurricaneurbanl3v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_tornadourbanl1v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_tornadourbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_tornadourbanl3v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_earthquakeurbanl1v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_earthquakeurbanl2v1_raven_remaining_2gpu1.yaml
  osmo/missions/raven_earthquakeurbanl3v1_raven_remaining_2gpu1.yaml
)

for mission in "${missions[@]}"; do
  [[ $(ps -p "$launcher_pid" -o stat=) == *T* ]]
  echo "[$(date -u +%FT%TZ)] starting $mission"
  bash osmo/workspace/run_held_mission.sh "$launcher_pid" "$mission"
  echo "[$(date -u +%FT%TZ)] passed and uploaded $mission"
done
