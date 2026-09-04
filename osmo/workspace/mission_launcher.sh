#!/usr/bin/env bash
# mission_launcher.sh — no-rebuild mission entrypoint for AirStack-on-OSMO.
#
# The airstack-mission.yaml workflow runs the *baked* image entrypoint with
# OSMO_AIRSTACK_UP=false (pod setup only: sshd, inner dockerd, branch clone,
# Nucleus creds, registry login), then hands off to THIS script — which lives
# in the clone, so the mission engine always comes from your branch and no
# workspace-image rebuild is needed to iterate. (PyYAML, which the baked image
# may predate, is installed at runtime below.)
#
# Responsibilities:
#   1. Wait for the setup the baked entrypoint performs in the background
#      (inner dockerd ready, clone present, registry login done).
#   2. Ensure PyYAML is importable.
#   3. Run mission_runner.py against $OSMO_MISSION_FILE.
#   4. Keep-alive: stay foreground (sleep) so the pod survives for
#      `airstack osmo:fetch`, or exit so OSMO uploads /osmo/output + frees GPU.

set -uo pipefail

AIRSTACK_ROOT="${AIRSTACK_ROOT:-/root/AirStack}"

log()  { echo "[mission-launcher] $*"; }
fail() { echo "[mission-launcher] ERROR: $*" >&2; }

# Kill the backgrounded `tee` (BAKED_PID, from the workflow bootstrap) so it
# releases this task's OSMO stdout pipe — else osmo_exec never sees EOF on our
# exit and the pod never frees the GPU. Call right before an intentional exit.
release_osmo_pipe() { [ -n "${BAKED_PID:-}" ] && kill "$BAKED_PID" 2>/dev/null; true; }

# Fast DDS LARGE_DATA uses shared memory between same-host/same-version
# participants; the segments live in /dev/shm, which every stack container
# bind-mounts from the pod. The pod default is 64M — it fills immediately
# and later participants then fail SHM registration. Regrow in place.
mount -o remount,size=8G /dev/shm 2>/dev/null
log "/dev/shm: $(df -h /dev/shm | awk 'NR==2{print $2" total, "$5" used"}')"

# wait_for <description> <timeout_s> <command...> — poll until the command
# succeeds or the timeout elapses. Returns non-zero on timeout.
wait_for() {
  local desc="$1" timeout="$2"; shift 2
  local i=0
  until "$@" >/dev/null 2>&1; do
    i=$((i + 1))
    if [ "$i" -ge "$timeout" ]; then
      fail "timed out after ${timeout}s waiting for: ${desc}"
      return 1
    fi
    [ $((i % 15)) -eq 0 ] && log "still waiting for ${desc} (${i}s)"
    sleep 1
  done
  log "ready: ${desc}"
}

# ── 1. setup readiness (driven by the backgrounded baked entrypoint) ───────
# If dockerd or the clone never appear, the pod is broken — stay alive (when
# keep-alive) so it can be inspected over SSH rather than vanishing.
if ! wait_for "inner dockerd" 180 docker info; then
  [ "${OSMO_MISSION_KEEP_ALIVE:-true}" = "true" ] && exec sleep infinity
  exit 1
fi
if ! wait_for "branch clone" 600 test -f "$AIRSTACK_ROOT/osmo/workspace/mission_runner.py"; then
  [ "${OSMO_MISSION_KEEP_ALIVE:-true}" = "true" ] && exec sleep infinity
  exit 1
fi

# The deployed :latest image's baked entrypoint ignores OSMO_AIRSTACK_UP and
# runs its OWN `airstack up`. Running the mission's bring-up concurrently
# collides (duplicate network, container-name conflicts, "network not found"
# mid-teardown) and fails the first iteration. So wait for the baked
# entrypoint to FINISH — the bootstrap tees its output to $BAKED_LOG, and its
# terminal "sleeping forever" line is printed exactly once, after its
# `airstack up` completes (success or fail). This also subsumes the clone +
# registry-login steps it performs, and warms the inner image cache the
# mission reuses. On a rebuilt image where the hook works, the entrypoint
# skips `up` and prints that line almost immediately, so this is a fast no-op.
#
# We deliberately do NOT detect completion via `pgrep -f "sleep infinity"`:
# the AirStack containers themselves run `sleep infinity`, so that matches the
# instant the baked bring-up starts a container — firing the handoff in the
# middle of its `airstack up`, which is exactly the race we're avoiding.
BAKED_LOG="${BAKED_ENTRYPOINT_LOG:-/tmp/baked-entrypoint.log}"
log "waiting for the baked entrypoint to finish its own bring-up (avoids a concurrent 'airstack up')"
if ! wait_for "baked entrypoint complete" 2400 grep -q "sleeping forever" "$BAKED_LOG"; then
  log "WARN: baked entrypoint didn't signal completion in 40m — proceeding; first iteration may race its bring-up"
fi
sleep 5  # let baked's compose settle before the mission's first ensure_down/up

# ── 2. PyYAML (mission_runner imports yaml) ────────────────────────────────
if ! python3 -c "import yaml" >/dev/null 2>&1; then
  log "installing PyYAML (image predates python3-yaml)"
  pip3 install --break-system-packages --quiet pyyaml \
    || { apt-get update -qq && apt-get install -y -qq python3-yaml; } \
    || fail "could not install PyYAML — mission_runner will fail to import yaml"
fi

# ── 3. run the mission ─────────────────────────────────────────────────────
MISSION="${OSMO_MISSION_FILE:-}"
if [ -z "$MISSION" ]; then
  fail "OSMO_MISSION_FILE not set — nothing to run"
else
  MISSION_PATH="$AIRSTACK_ROOT/$MISSION"
  [ -f "$MISSION_PATH" ] || MISSION_PATH="$MISSION"   # allow an absolute path
  if [ -f "$MISSION_PATH" ]; then
    log "running mission: $MISSION_PATH"
    if [ -n "${OSMO_MISSION_MAX_WALL_SECONDS:-}" ]; then
      log "mission wall-time cap: ${OSMO_MISSION_MAX_WALL_SECONDS}s"
      timeout --foreground --signal=INT --kill-after=15m \
        "${OSMO_MISSION_MAX_WALL_SECONDS}s" \
        python3 "$AIRSTACK_ROOT/osmo/workspace/mission_runner.py" "$MISSION_PATH" \
          --airstack-root "$AIRSTACK_ROOT"
    else
      python3 "$AIRSTACK_ROOT/osmo/workspace/mission_runner.py" "$MISSION_PATH" \
        --airstack-root "$AIRSTACK_ROOT"
    fi
    log "mission_runner exited $?"
  else
    fail "mission file not found: $MISSION (looked under the clone and as an absolute path)"
  fi
fi

# ── 3b. NAS upload + teardown (optional) ───────────────────────────────────
# With the airlab-storage credential injected and a destination set (--nas-dest,
# else the spec's nas_dest:), rsync results to the NAS. Success → pod exits (GPU
# freed); failure → stay alive so results survive for osmo:fetch.
NAS_YAML_DEST=""
if [ -f "${MISSION_PATH:-}" ]; then
  NAS_YAML_DEST="$(python3 -c 'import yaml,sys; d=yaml.safe_load(open(sys.argv[1])) or {}; print(d.get("nas_dest") or "")' "$MISSION_PATH" 2>/dev/null)"
fi
NAS_DEST="${OSMO_MISSION_UPLOAD_DEST:-$NAS_YAML_DEST}"

if [ "${OSMO_MISSION_NO_UPLOAD:-false}" != "true" ] \
   && [ -n "${AIRLAB_STORAGE_USER:-}" ] && [ -n "${AIRLAB_STORAGE_PASS:-}" ] \
   && [ -n "$NAS_DEST" ]; then
  if ! command -v sshpass >/dev/null 2>&1; then
    log "installing sshpass (image predates it)"
    { apt-get update -qq && apt-get install -y -qq sshpass; } \
      || fail "could not install sshpass — skipping NAS upload"
  fi
  if command -v sshpass >/dev/null 2>&1; then
    nas_host="${AIRLAB_STORAGE_HOST:-airlab-storage.andrew.cmu.edu}"
    results_dir="${OSMO_RESULTS_ROOT:-$AIRSTACK_ROOT/osmo/results}"
    ssh_cmd="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
    log "NAS upload: ${results_dir}/ → ${nas_host}:${NAS_DEST%/}/"
    export SSHPASS="$AIRLAB_STORAGE_PASS"
    # NAS share (Synology rsync 3.1.x) rejects --mkpath and can't chmod/chown the
    # SMB-backed volume, so skip those attrs; nas_dest base must already exist.
    sshpass -e rsync -rltz --partial --timeout=1800 \
      --no-perms --no-owner --no-group --omit-dir-times -e "$ssh_cmd" \
      "${results_dir}/" "${AIRLAB_STORAGE_USER}@${nas_host}:${NAS_DEST%/}/"
    nas_rc=$?
    unset SSHPASS
    if [ "$nas_rc" -eq 0 ]; then
      log "NAS upload OK → ${nas_host}:${NAS_DEST} — tearing pod down (GPU freed)"
      release_osmo_pipe
      exit 0
    fi
    fail "NAS upload FAILED (rc=$nas_rc) — keeping pod alive for 'airstack osmo:fetch' / SSH debug"
    exec sleep infinity
  fi
elif [ "${OSMO_MISSION_NO_UPLOAD:-false}" != "true" ] \
     && [ -n "${AIRLAB_STORAGE_USER:-}" ] && [ -z "$NAS_DEST" ]; then
  log "airlab-storage set but no nas_dest / --nas-dest — skipping upload; pod stays alive."
fi

# ── 4. lifetime ────────────────────────────────────────────────────────────
if [ "${OSMO_MISSION_KEEP_ALIVE:-true}" = "true" ]; then
  log "OSMO_MISSION_KEEP_ALIVE=true — pod stays alive; fetch with 'airstack osmo:fetch'"
  exec sleep infinity
fi
log "OSMO_MISSION_KEEP_ALIVE=false — exiting so OSMO uploads /osmo/output and frees the GPU"
release_osmo_pipe
exit 0
