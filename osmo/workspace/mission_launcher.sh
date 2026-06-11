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
AIRLAB_REGISTRY="${AIRLAB_REGISTRY:-airlab-docker.andrew.cmu.edu}"

log()  { echo "[mission-launcher] $*"; }
fail() { echo "[mission-launcher] ERROR: $*" >&2; }

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

# Registry login is performed by the baked entrypoint (step 5) only when
# AIRLAB_REGISTRY_USER is set. Wait for it so `airstack up`'s image pulls
# don't race the login; warn (don't abort) if it never lands.
if [ -n "${AIRLAB_REGISTRY_USER:-}" ]; then
  wait_for "registry login (${AIRLAB_REGISTRY})" 180 \
    grep -q "$AIRLAB_REGISTRY" /root/.docker/config.json \
    || log "WARN: registry login not detected — image pulls may fail"
fi

# The deployed :latest image's baked entrypoint runs its OWN `airstack up`
# (that image predates the OSMO_AIRSTACK_UP=false hook, so it ignores the
# request to skip it). If the mission started its own bring-up concurrently,
# the two compose runs collide — duplicate network, container-name conflicts,
# "network not found" mid-teardown — and the first iteration fails. So wait
# for the baked entrypoint to FINISH its bring-up (which also warms the inner
# image cache the mission then reuses) before handing off. Detect completion
# by its terminal `sleep infinity`. On a rebuilt image where the hook works,
# the entrypoint skips `up` and reaches that sleep almost immediately, so this
# is a fast no-op — correct either way.
log "waiting for the baked entrypoint to finish its own bring-up (avoids a concurrent 'airstack up')"
sleep 10   # let the baked entrypoint actually reach its `airstack up`
if wait_for "baked entrypoint idle" 2400 pgrep -f "sleep infinity"; then
  sleep 5  # let compose fully release the network before the mission's first down/up
else
  log "WARN: baked entrypoint still busy after 40m — proceeding; first iteration may race its bring-up"
fi

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
    python3 "$AIRSTACK_ROOT/osmo/workspace/mission_runner.py" "$MISSION_PATH" \
      --airstack-root "$AIRSTACK_ROOT"
    log "mission_runner exited $?"
  else
    fail "mission file not found: $MISSION (looked under the clone and as an absolute path)"
  fi
fi

# ── 4. lifetime ────────────────────────────────────────────────────────────
if [ "${OSMO_MISSION_KEEP_ALIVE:-true}" = "true" ]; then
  log "OSMO_MISSION_KEEP_ALIVE=true — pod stays alive; fetch with 'airstack osmo:fetch'"
  exec sleep infinity
fi
log "OSMO_MISSION_KEEP_ALIVE=false — exiting so OSMO uploads /osmo/output and frees the GPU"
exit 0
