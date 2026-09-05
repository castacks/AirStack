#!/usr/bin/env bash
# Recover freeze/upload-only Suburban Earthquake levels while another level
# finishes its cache bake. This controller never rebuilds a valid CPU scene.
set -uo pipefail

REPO="${REPO:-/root/AirStack}"
CONTAINER="${CONTAINER:-isaac-sim}"
LOG_ROOT="${LOG_ROOT:-/root/docker/isaac-sim/logs/suburban_quake_dataset}"
WORK_ROOT="${WORK_ROOT:-/isaac-sim/.cache/suburban_earthquake/work}"
PREWARM_ROOT="${PREWARM_ROOT:-/isaac-sim/.cache/suburban_earthquake/prewarm}"
ATTEMPTS="${SUBURBAN_QUAKE_LEVEL_ATTEMPTS:-3}"
POLL_S="${RECOVERY_POLL_S:-15}"

wait_for_kit_slot() {
  while docker exec "$CONTAINER" pgrep -f \
      '[s]uburb_earthquake_bake_launch_script.py' >/dev/null 2>&1 || \
      docker exec "$CONTAINER" pgrep -f \
      '[s]uburb_earthquake_freeze_launch_script.py' >/dev/null 2>&1; do
    sleep "$POLL_S"
  done
}

run_level() {
  local level="$1"
  local log_dir="${LOG_ROOT}/recovery_l${level}"
  printf 'L%s freeze/upload recovery start %s\n' \
    "$level" "$(date -u +%FT%TZ)"
  SUBURBAN_QUAKE_LEVEL_ATTEMPTS="$ATTEMPTS" LEVELS="$level" \
    LOG_DIR="$log_dir" DATASET_UPLOAD_WORKERS="${DATASET_UPLOAD_WORKERS:-8}" \
    bash "$REPO/scene_gen/tools/suburban_earthquake_queue.sh"
  local rc=$?
  printf '%s\n' "$rc" > "${LOG_ROOT}/recovery_l${level}_controller.exit"
  printf 'L%s recovery exit %s at %s\n' \
    "$level" "$rc" "$(date -u +%FT%TZ)"
  return "$rc"
}

promote_l1_prewarm() {
  local exit_file="${LOG_ROOT}/l1_prewarm.exit"
  local prewarm="${PREWARM_ROOT}/level_1/scene"
  local target="${WORK_ROOT}/level_1/scene"
  local stamp
  [ -f "$exit_file" ] || return 1
  [ "$(tr -d '[:space:]' < "$exit_file")" = 0 ] || return 1
  docker exec "$CONTAINER" test -d "$prewarm" || return 1
  stamp="$(date +%Y%m%d_%H%M%S)"
  docker exec "$CONTAINER" bash -lc \
    "if [ -d '$target' ]; then mv '$target' '${target}.prewarm_replaced_${stamp}'; fi; mv '$prewarm' '$target'"
}

mkdir -p "$LOG_ROOT"
cd "$REPO" || exit 97
printf 'export recovery controller start %s\n' "$(date -u +%FT%TZ)"

# The live L3 process owns Kit until its last cache is settled. Its following
# CPU assembly gives these two already-built scenes a safe freeze window.
wait_for_kit_slot
sleep 20
run_level 2
l2_rc=$?

promote_l1_prewarm
promote_rc=$?
printf '%s\n' "$promote_rc" > "${LOG_ROOT}/l1_promotion.exit"
wait_for_kit_slot
run_level 1
l1_rc=$?

printf 'export recovery controller done l2=%s l1=%s %s\n' \
  "$l2_rc" "$l1_rc" "$(date -u +%FT%TZ)"
exit $((l2_rc != 0 || promote_rc != 0 || l1_rc != 0))
