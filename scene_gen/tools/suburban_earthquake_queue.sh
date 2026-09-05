#!/usr/bin/env bash
# Run Suburban Earthquake levels independently.  A failed level is retried and
# recorded, but can never prevent a later level from being attempted.
set -uo pipefail

REPO="${REPO:-/root/AirStack}"
LEVELS="${LEVELS:-${*:-1 2 3}}"
ATTEMPTS="${SUBURBAN_QUAKE_LEVEL_ATTEMPTS:-3}"
LOG_DIR="${LOG_DIR:-/root/docker/isaac-sim/logs/suburban_quake_dataset}"

mkdir -p "$LOG_DIR"
overall=0
for level in $LEVELS; do
  rc=1
  attempt=1
  while [ "$attempt" -le "$ATTEMPTS" ]; do
    printf '\n== INDEPENDENT LEVEL %s ATTEMPT %s/%s START %s ==\n' \
      "$level" "$attempt" "$ATTEMPTS" "$(date -u +%FT%TZ)"
    (
      cd "$REPO" || exit 1
      LEVELS="$level" LOG_DIR="$LOG_DIR" \
        bash scene_gen/tools/suburban_earthquake_cell.sh
    )
    rc=$?
    printf '%s\n' "$rc" > "$LOG_DIR/level${level}.attempt${attempt}.exit"
    printf '== INDEPENDENT LEVEL %s ATTEMPT %s EXIT %s %s ==\n' \
      "$level" "$attempt" "$rc" "$(date -u +%FT%TZ)"
    [ "$rc" = 0 ] && break
    attempt=$((attempt + 1))
  done
  printf '%s\n' "$rc" > "$LOG_DIR/level${level}.exit"
  [ "$rc" = 0 ] || overall=1
done

printf '%s\n' "$overall" > "$LOG_DIR/queue.exit"
exit "$overall"
