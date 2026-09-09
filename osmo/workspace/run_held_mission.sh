#!/usr/bin/env bash
# Launch a manual mission while the verified auto-launcher remains stopped.
set -euo pipefail
launcher_pid=${1:?launcher PID required}
mission=${2:?mission path required}
cd /root/AirStack
[[ $(ps -p "$launcher_pid" -o args=) == *mission_launcher.sh* ]] || exit 1
[[ $(ps -p "$launcher_pid" -o stat=) == *T* ]] || exit 1
if pgrep -f '^python3 .*mission_runner.py' >/dev/null; then
  echo 'Refusing duplicate mission runner' >&2
  exit 1
fi
# Carry storage credentials without printing or persisting them.
while IFS= read -r -d '' pair; do
  case "$pair" in AIRLAB_STORAGE_*) export "$pair" ;; esac
done < "/proc/$launcher_pid/environ"
: "${AIRLAB_STORAGE_USER:?missing storage user}"
: "${AIRLAB_STORAGE_PASS:?missing storage password}"
export OSMO_MISSION_NO_UPLOAD=false
export OSMO_UPLOAD_PER_ITERATION=true
export OSMO_UPLOAD_FAILED_ITERATIONS=false
export OSMO_PRUNE_UPLOADED_BAGS=always
exec timeout --foreground --signal=INT --kill-after=5m 42900s \
  python3 -u osmo/workspace/mission_runner.py "$mission" \
  --airstack-root /root/AirStack
