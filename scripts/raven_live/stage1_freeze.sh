#!/usr/bin/env bash
# stage1_freeze.sh — build + freeze the `suburb_tornado_250` cell in the
# ALREADY-RUNNING isaac-sim container, via the tmux pane. No `airstack
# up/down`, no container recreate, no `.env` edit.
#
#   scripts/raven_live/stage1_freeze.sh              # DRY RUN: print every
#                                                    # command, touch nothing
#   scripts/raven_live/stage1_freeze.sh --go         # actually do it
#
# WHY THE PANE AND NOT `.env` + `airstack up`. Verified against
# `simulation/isaac-sim/docker/docker-compose.yaml`: the isaac-sim service
# `environment:` block declares NONE of TOR_SEED, TOR_PEOPLE, TOR_PLANKS,
# TOR_TRACK_PER100, TOR_GROUND, TOR_MIN_TIPPED, PEOPLE_JSON, PEOPLE_SNAPS,
# SCOUR_RELIEF, FREEZE_OUT, FREEZE_NAME, FREEZE_EXPORT or FREEZE_EXIT.
# Compose forwards nothing it does not name, so the Stage-1 `.env` block in
# `_plans/raven_test_scene_runbook.md` §2 would be SILENTLY DROPPED at the
# container boundary: the launcher would build with TOR_SEED=11 (its own
# default, not the preset's 10), write the people JSON next to the
# archetypes, and never freeze anything — while printing a perfectly happy
# banner. Setting them on the command line the pane runs is the only path
# that works today. (Adding them to the compose `environment:` block would
# be the other fix; that is a code change, not a run-night change.)
#
# THE EMPTY-STRING TRAP (run-isaac-sim-launcher/SKILL.md §4). The three knobs
# the compose file DOES declare — SCENE_CONFIG, ARCH_DIR, SNAP_DIR — exist in
# the container as EMPTY STRINGS whenever `.env` leaves them blank, so
# `os.environ.get("ARCH_DIR", <default>)` returns "" and never reaches its
# default. For ARCH_DIR that does not raise: the archetype library silently
# resolves to nothing and every house is built LIVE and INTACT under a
# success banner. Hence: every knob below is passed explicitly, defaults
# included.
#
# Read the result with scripts/raven_live/watch_stage1.sh.
set -euo pipefail

CONTAINER="${CONTAINER:-isaac-sim}"
SESSION="${SESSION:-isaac}"

# ── knobs (override by exporting before the call) ──────────────────────────
SCENE_CONFIG="${SCENE_CONFIG:-suburb_tornado_250}"
ARCH_DIR="${ARCH_DIR:-/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado}"
# MUST equal the preset's layout `seed: 10`. suburb_tornado_100.yaml's own
# comment: "the two must agree or the plan PNG is not the scene you get".
# The launcher's own default is 11.
TOR_SEED="${TOR_SEED:-10}"
TOR_PLANKS="${TOR_PLANKS:-140}"          # launcher default
TOR_TRACK_PER100="${TOR_TRACK_PER100:-4.5}"   # launcher default
TOR_GROUND="${TOR_GROUND:-1}"            # mud overlay + relief on
SCOUR_RELIEF="${SCOUR_RELIEF:-1}"        # the 3D relief on top of the overlay
TOR_PEOPLE="${TOR_PEOPLE:-1}"            # the casualty pass — THE POINT
TOR_MIN_TIPPED="${TOR_MIN_TIPPED:-0}"    # 0 = measured scene, no forced rolls
FREEZE_OUT="${FREEZE_OUT:-/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250}"
FREEZE_NAME="${FREEZE_NAME:-RavenSuburbTornado250}"
PEOPLE_JSON="${PEOPLE_JSON:-${FREEZE_OUT}/humans_${TOR_SEED}.json}"
FREEZE_EXPORT="${FREEZE_EXPORT:-1}"
FREEZE_EXIT="${FREEZE_EXIT:-1}"
# Review PNGs. MUST live under /isaac-sim/.nvidia-omniverse/logs (the compose
# mount of ~/docker/isaac-sim/logs) or the host cannot read them. Set
# SNAP_DIR= (empty) and PEOPLE_SNAPS=0 to skip the ~2-4 min of captures.
SNAP_DIR="${SNAP_DIR:-/isaac-sim/.nvidia-omniverse/logs/raven_t250}"
PEOPLE_SNAPS="${PEOPLE_SNAPS:-4}"
# EMPTY ON PURPOSE. `disaster.freeze.make_portable`'s docstring (run 8): AEC
# pool discovery enumerates its asset pool by LOCAL DIRECTORY LISTING, which
# cannot enumerate an omniverse:// root, so an AIRSTACK_ASSET_ROOT build
# produces a DIFFERENT layout than the bakes were solved against. Build local;
# make_portable does the mirroring before the flatten.
AIRSTACK_ASSET_ROOT="${AIRSTACK_ASSET_ROOT:-}"

PANE_LOG="${PANE_LOG:-/isaac-sim/.nvidia-omniverse/logs/raven_stage1.log}"
HOST_PANE_LOG="${HOME}/docker/isaac-sim/logs/$(basename "$PANE_LOG")"
LAUNCHER=/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/suburb_tornado_launch_script.py

GO=0
[ "${1:-}" = "--go" ] && GO=1

die() { echo "stage1_freeze: $*" >&2; exit 1; }
say() { echo "[stage1] $*"; }

# The one line the pane will run. `$ISAAC_SIM_PYTHONPATH` MUST reach the pane
# UNEXPANDED (it is defined by the container's .bashrc, not by this shell) —
# hence the backslash. Everything before python.sh is per-run env; everything
# after the script path goes to Kit as a carb setting.
#
# The two fractionalCutoutOpacity flags are already in the launcher's own
# SimulationApp(extra_args=...) and are re-asserted from carb after the
# Pegasus stage load; repeated here because a Kit setting given on the
# command line is the one thing that survives every code path, and the
# symptom of losing it is "I don't see the ground at all".
LINE="clear; \
SCENE_CONFIG=${SCENE_CONFIG} \
ARCH_DIR=${ARCH_DIR} \
TOR_SEED=${TOR_SEED} \
TOR_PLANKS=${TOR_PLANKS} \
TOR_TRACK_PER100=${TOR_TRACK_PER100} \
TOR_GROUND=${TOR_GROUND} \
SCOUR_RELIEF=${SCOUR_RELIEF} \
TOR_PEOPLE=${TOR_PEOPLE} \
TOR_MIN_TIPPED=${TOR_MIN_TIPPED} \
PEOPLE_JSON=${PEOPLE_JSON} \
PEOPLE_SNAPS=${PEOPLE_SNAPS} \
SNAP_DIR=${SNAP_DIR} \
FREEZE_OUT=${FREEZE_OUT} \
FREEZE_NAME=${FREEZE_NAME} \
FREEZE_EXPORT=${FREEZE_EXPORT} \
FREEZE_EXIT=${FREEZE_EXIT} \
AIRSTACK_ASSET_ROOT=${AIRSTACK_ASSET_ROOT} \
PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" /isaac-sim/python.sh ${LAUNCHER} \
--ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts \
--/rtx/raytracing/fractionalCutoutOpacity=true \
--/rtx/pathtracing/fractionalCutoutOpacity=true"

cat <<EOF
[stage1] container   : ${CONTAINER} (tmux session '${SESSION}')
[stage1] pane log    : ${PANE_LOG}
[stage1]   host side : ${HOST_PANE_LOG}
[stage1] freeze out  : ${FREEZE_OUT}
[stage1]   host side : $(cd "$(dirname "$0")/../.." && pwd)/${FREEZE_OUT#/isaac-sim/AirStack/}
[stage1] people json : ${PEOPLE_JSON}
[stage1] launch line :
${LINE}

EOF

if [ "$GO" -ne 1 ]; then
  cat <<'EOF'
[stage1] DRY RUN — nothing was sent. This script Ctrl-Cs whatever the pane is
[stage1] currently running, so it will not do that without --go. If somebody is
[stage1] looking at the current scene, ask first.
[stage1] Re-run as:  scripts/raven_live/stage1_freeze.sh --go
EOF
  exit 0
fi

# ── 0. the container must already be up ────────────────────────────────────
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" \
  || die "container '$CONTAINER' is not running. This script deliberately does
        NOT bring it up (a recreate loses runtime pip installs and takes
        longer than a pane relaunch). Start it first:
            ISAAC_SIM_SCRIPT_NAME=suburb_tornado_launch_script.py ./airstack.sh up isaac-sim"

docker exec "$CONTAINER" tmux has-session -t "$SESSION" 2>/dev/null \
  || die "no tmux session '$SESSION' in $CONTAINER"

# ── 1. widen the pane, then START THE PIPE BEFORE ANYTHING ELSE ────────────
# pipe-pane captures only from the moment it is opened and survives the C-c,
# so it has to come first. `-o` makes a second call a no-op.
say "resizing pane and opening pipe-pane -> ${PANE_LOG}"
docker exec "$CONTAINER" tmux resize-window -t "$SESSION" -x 200 -y 50 || true
docker exec "$CONTAINER" tmux pipe-pane -t "$SESSION" -o "cat >> ${PANE_LOG}"

# ── 2. stop whatever is running and wait for the prompt ────────────────────
say "sending C-c to the pane"
docker exec "$CONTAINER" tmux send-keys -t "$SESSION" C-c
say "waiting up to 180 s for the shell prompt to come back"
ok=0
for i in $(seq 1 90); do
  last="$(docker exec "$CONTAINER" tmux capture-pane -p -J -t "$SESSION" \
          | grep -v '^[[:space:]]*$' | tail -1 || true)"
  # trim trailing whitespace — the prompt renders as '...:~# ' with a space
  trimmed="${last%"${last##*[![:space:]]}"}"
  case "$trimmed" in
    *'#'|*'$') ok=1; break ;;
  esac
  sleep 2
done
[ "$ok" -eq 1 ] || die "the pane never returned to a prompt (last line: '${last:-<empty>}').
        Kit may still be shutting down — check
          docker exec ${CONTAINER} tmux capture-pane -p -J -t ${SESSION} | tail -20
        and re-run when the prompt is back."
say "prompt is back: ${last}"

# ── 2b. KILL ORPHANED PX4 ──────────────────────────────────────────────────
# Ctrl-C on Kit does NOT take Pegasus' px4 children with it. An orphan keeps
# its instance directory and its MAVLink ports, the next run's px4 goes
# <defunct>, MAVROS connects to the ORPHAN and prints a healthy FCU version,
# and the only visible symptom is TakeoffTask rejected with "state estimate
# timed out" — which reads like a planner bug. Not optional.
say "killing any orphaned PX4 (the C-c does not take them with it)"
# the [p] bracket keeps pgrep/pkill from matching THIS bash -c's own cmdline
# (a plain -f pattern kills the probe shell itself -> docker exec returns 137
# -> set -e aborts this script before the launch line is ever sent).
docker exec "$CONTAINER" bash -c \
  'pgrep -af "px4_sitl_default/bin/[p]x4" || true; pkill -9 -f "px4_sitl_default/bin/[p]x4" || true; sleep 2' || true

# ── 3. drop the old scrollback ─────────────────────────────────────────────
# So a later capture-pane cannot hand back the PREVIOUS run's banner as this
# run's. (`clear` alone leaves tmux history intact — hence both.)
say "clearing tmux history"
docker exec "$CONTAINER" tmux clear-history -t "$SESSION"

# ── 4. one line, every knob explicit ───────────────────────────────────────
say "sending the launch line"
docker exec "$CONTAINER" tmux send-keys -t "$SESSION" "$LINE" ENTER

cat <<EOF

[stage1] SENT. Expect ~2-4 min of silence (RtPso shader compile) before the
[stage1] first print, then the assembly, then the freeze.

  watch it:      scripts/raven_live/watch_stage1.sh --follow
  raw host log:  tail -f ${HOST_PANE_LOG}
  pane:          docker exec ${CONTAINER} tmux capture-pane -p -J -t ${SESSION} -S -3000 | tail -40

[stage1] Done when the pane prints:
[stage1]   [tornado] FREEZE_EXIT set — closing after the export
EOF
