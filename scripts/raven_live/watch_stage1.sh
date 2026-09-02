#!/usr/bin/env bash
# watch_stage1.sh — read the Stage-1 build+freeze out of the two places that
# actually have it, and give one verdict.
#
#   scripts/raven_live/watch_stage1.sh            # one shot: verdict + evidence
#   scripts/raven_live/watch_stage1.sh --follow   # poll until it finishes
#
# exit 0  the export completed  (the FREEZE_EXIT banner is present and no
#         EXPORT FAILED / Traceback / PEOPLE PASS FAILED before it)
# exit 1  a failure line is present
# exit 2  still running / not enough evidence yet (one-shot mode only)
#
# WHERE IT LOOKS, and why both:
#   * the PIPED PANE LOG on the host (~/docker/isaac-sim/logs/raven_stage1.log)
#     — everything the pane printed from the moment stage1_freeze.sh opened
#     the pipe. Readable without docker.
#   * the KIT LOG inside the container — the COMPLETE record. `print()` from
#     the launcher is mirrored there as `[py stdout]` once the app is up, with
#     no 2000-line history cap, and it is the only place that explains a
#     SILENT pane (`Waiting for RtPso ... N seconds so far` = shader compile,
#     measured at 133 s cold).
# `docker logs isaac-sim` is EMPTY for this container. Do not bother.
set -uo pipefail

CONTAINER="${CONTAINER:-isaac-sim}"
PANE_LOG_HOST="${PANE_LOG_HOST:-${HOME}/docker/isaac-sim/logs/raven_stage1.log}"
KIT_LOG_DIR="${KIT_LOG_DIR:-/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1}"
POLL_S="${POLL_S:-20}"

FOLLOW=0
[ "${1:-}" = "--follow" ] && FOLLOW=1

STRIP_ANSI="s/\x1b\[[0-9;]*[a-zA-Z]//g"

gather() {
  # Both sources, ANSI-stripped, into one stream on stdout.
  if [ -r "$PANE_LOG_HOST" ]; then
    sed -e "$STRIP_ANSI" "$PANE_LOG_HOST" 2>/dev/null
  fi
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
    docker exec "$CONTAINER" bash -c '
      D="$1"; F="$D/$(ls -t "$D" 2>/dev/null | head -1)"
      [ -f "$F" ] && grep -a "py stdout\|py stderr\|\[Error\]" "$F" || true
    ' _ "$KIT_LOG_DIR" 2>/dev/null | sed -e "$STRIP_ANSI"
  fi
}

# Evidence lines, in the order they should appear.
GOOD_PRESET='\[compile_disaster\] (compiled high-level spec|loaded low-level config)'
GOOD_TRACK='\[tornado\] track .* m wide toward'
GOOD_PEOPLE='\[tornado\] people: [0-9]+ authored'
GOOD_ASSEMBLED='TORNADO TRACK ASSEMBLED'
GOOD_FLAT='\[freeze\] flattened in'
GOOD_FROZEN='^.*FROZEN (OK|\*\*\* NOT SELF-CONTAINED)'
GOOD_EXIT='\[tornado\] FREEZE_EXIT set'
BAD='EXPORT FAILED|PEOPLE PASS FAILED|GT_hints FAILED|GT_people copy FAILED|PortabilityError|Traceback \(most recent call last\)|ARCH_DIR .* does not exist|NO SCOUR BANDS|are UNDAMAGED because their'

report() {
  local text="$1"
  echo "──────────────────────────────────────────────────────────────────────"
  echo " sources: pane=${PANE_LOG_HOST}$( [ -r "$PANE_LOG_HOST" ] && echo " ($(wc -l < "$PANE_LOG_HOST") lines)" || echo " (ABSENT)")"
  echo "──────────────────────────────────────────────────────────────────────"

  local label pat line pair bad
  for pair in \
      "preset      |$GOOD_PRESET" \
      "track       |$GOOD_TRACK" \
      "people      |$GOOD_PEOPLE" \
      "assembled   |$GOOD_ASSEMBLED" \
      "flatten     |$GOOD_FLAT" \
      "verify      |$GOOD_FROZEN" \
      "freeze exit |$GOOD_EXIT" ; do
    label="${pair%%|*}"; pat="${pair#*|}"
    line="$(printf '%s\n' "$text" | grep -aE "$pat" | tail -1)"
    if [ -n "$line" ]; then
      printf '  ✓ %s %s\n' "$label" "$(printf '%s' "$line" | sed 's/^.*py stdout[]:]*//' | cut -c1-150)"
    else
      printf '  · %s (not yet)\n' "$label"
    fi
  done

  echo
  bad="$(printf '%s\n' "$text" | grep -aE "$BAD" | sort -u)"
  if [ -n "$bad" ]; then
    echo "  !! FAILURE LINES:"
    printf '%s\n' "$bad" | sed 's/^/     /' | head -30
    echo
    echo "  Full traceback context (last 40 lines around the first hit):"
    printf '%s\n' "$text" | grep -aE -A 25 "$BAD" | head -40 | sed 's/^/     /'
    return 1
  fi
  return 0
}

verdict() {
  local text="$1"
  # Order matters: a failure BEFORE the exit banner still fails.
  if printf '%s\n' "$text" | grep -aqE "$BAD"; then return 1; fi
  if printf '%s\n' "$text" | grep -aqE "$GOOD_EXIT"; then return 0; fi
  return 2
}

if [ "$FOLLOW" -eq 1 ]; then
  echo "[watch] polling every ${POLL_S}s — Ctrl-C to stop"
  while :; do
    text="$(gather)"
    report "$text" || true
    verdict "$text"; rc=$?
    if [ "$rc" -eq 0 ]; then
      echo
      echo "[watch] ✅ EXPORT COMPLETE — the FREEZE_EXIT banner is present and no"
      echo "[watch]    failure line came before it. Next:"
      echo "[watch]      scripts/raven_live/validate_freeze.sh"
      exit 0
    fi
    if [ "$rc" -eq 1 ]; then
      echo
      echo "[watch] ❌ FAILED — see the lines above."
      echo "[watch]    If it is a portability gate: read"
      echo "[watch]      _test_freeze/raven_suburb_tornado_250/freeze_report.json"
      echo "[watch]    (the launcher writes exc.info there so the diagnosis survives)."
      exit 1
    fi
    sleep "$POLL_S"
  done
fi

text="$(gather)"
report "$text" || true
verdict "$text"; rc=$?
case "$rc" in
  0) echo; echo "[watch] ✅ EXPORT COMPLETE" ;;
  1) echo; echo "[watch] ❌ FAILED" ;;
  2) echo; echo "[watch] ⏳ still running (or nothing logged yet)."
     echo "[watch]    A silent pane right after 'app ready' is the RtPso shader"
     echo "[watch]    compile — 133 s cold, 14 s warm. Check it is alive:"
     echo "[watch]      docker exec ${CONTAINER} bash -c 'pgrep -af launch_script | head -2; nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader'" ;;
esac
exit "$rc"
