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

# UDP socket buffer limits: fastdds.xml requests 16MB buffers for the large
# image streams; the kernel caps requests at rmem_max/wmem_max (~212KB
# default), and overflow drops whole frames crossing the inner docker bridge.
# Done here (not the baked entrypoint) so it ships with the branch. sysctl
# can fail with exit 0 ("permission denied, ignoring"), so verify by readback.
sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216 \
  net.core.rmem_default=4194304 net.core.wmem_default=4194304 >/dev/null 2>&1
if [ "$(sysctl -n net.core.rmem_max 2>/dev/null)" = "16777216" ]; then
  log "UDP socket buffer limits raised: net.core.rmem_max=16777216"
else
  log "WARN: UDP buffer limits NOT raised (rmem_max=$(sysctl -n net.core.rmem_max 2>/dev/null)) — image topics may drop frames"
fi

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
