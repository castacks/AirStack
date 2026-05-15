#!/usr/bin/env bash
# entrypoint.sh — airstack-osmo-workspace pod startup.
#
# Order of operations:
#   1. Install SSH_PUB_KEY into authorized_keys, generate sshd host keys,
#      start sshd. Done first so the student can SSH in even if a later step
#      fails (huge debugging accelerator).
#   2. Start the inner Docker daemon (DinD) with the NVIDIA runtime so Isaac
#      Sim sees the GPU forwarded into the pod.
#   3. Clone AirStack into /root/AirStack (skipped if already cloned by a
#      previous pod incarnation).
#   4. Materialize simulation/isaac-sim/docker/omni_pass.env from the
#      `airlab-nucleus` OSMO GENERIC credential.
#   5. docker login airlab-docker.andrew.cmu.edu using the
#      `airlab-docker-login` OSMO GENERIC credential.
#   6. cd /root/AirStack && ./airstack.sh up
#   7. sleep infinity so port-forwards keep working.
#
# All steps are idempotent across pod restarts: re-running this script
# inside the same pod is safe.

set -uo pipefail

log() { echo "[entrypoint] $*"; }
fail() { echo "[entrypoint] ERROR: $*" >&2; exit 1; }

# ─── 0. Stale-state cleanup ────────────────────────────────────────────────
#
# Cursor / VS Code Remote-SSH guards its server install with a file lock
# at /tmp/cursor-remote-lock.* (and a sibling .target file naming the PIDs
# that hold it). If a previous connect attempt crashed mid-install
# (e.g. the port-forward died while the install was in flight, as
# happened on airstack-dev-13 / 2026-05-14), the lock file outlives the
# dead PIDs and every subsequent IDE retry bails out *silently* at the
# lock check — leaving an empty bin/<sha>/ dir and the user staring at
# a "Connecting to remote host (attempt 1)..." spinner forever.
#
# A fresh pod has nothing to preserve here, so clearing these on startup
# is always safe.
rm -f /tmp/cursor-remote-lock.* /tmp/vscode-remote-lock.* 2>/dev/null || true

# ─── 1. SSHD ───────────────────────────────────────────────────────────────

log "configuring sshd"

mkdir -p /root/.ssh && chmod 700 /root/.ssh

if [ -z "${SSH_PUB_KEY:-}" ]; then
  fail "SSH_PUB_KEY not set. Re-submit with --set-env \"SSH_PUB_KEY=\$(cat ~/.ssh/id_ed25519.pub)\""
fi

# Always overwrite — last submit wins. Single-user dev pod.
echo "${SSH_PUB_KEY}" > /root/.ssh/authorized_keys
chmod 600 /root/.ssh/authorized_keys

# Generate fresh host keys if missing (first boot of this pod).
ssh-keygen -A

mkdir -p /var/run/sshd
/usr/sbin/sshd
log "sshd listening on :22"

# ─── 2. Inner dockerd (DinD) ───────────────────────────────────────────────

log "starting inner dockerd (DinD with NVIDIA runtime)"

# nvidia-container-toolkit ships a CLI that registers the nvidia runtime in
# the dockerd config and (optionally) sets it as the default. We want it as
# the default so `airstack up` doesn't have to specify --runtime.
nvidia-ctk runtime configure --runtime=docker --set-as-default || \
  log "WARN: nvidia-ctk runtime configure failed — Isaac Sim probably won't see the GPU"

# Pre-flight diagnostics so failures surface in OSMO logs (the pod is gone
# by the time anyone reads /var/log/dockerd.log otherwise).
log "diagnostics: kernel=$(uname -r) cgroups=$(stat -fc %T /sys/fs/cgroup 2>/dev/null) rootfs=$(stat -fc %T / 2>/dev/null)"
log "diagnostics: /var/lib/docker fs=$(stat -fc %T /var/lib/docker 2>/dev/null || echo absent)"

# Inner dockerd setup. We try storage drivers in order: overlay2 (fastest,
# works on most modern hosts) → fuse-overlayfs (rootless-friendly, may not be
# present) → vfs (always works, slowest). Falling back avoids the
# overlay-on-overlay failure that bites DinD on some kernel/storage
# combinations.
#
# Concurrency: dockerd's defaults are --max-concurrent-downloads=3 and
# --max-concurrent-uploads=5. With 2 GB+ AirStack image blobs on a 10 GbE
# pool, a single TLS pull stream tops out around 300-500 MiB/s (CPU-bound
# on the registry-side TLS encryption), so 3 parallel streams cap the
# whole bring-up around the 300 MiB/s mark seen empirically against the
# airlab-backup-10g registry — even though Ceph + 10 GbE can do far more.
# Bumping to 10 streams overlaps blob downloads enough to saturate the
# pipe without overwhelming the registry. Override with DOCKERD_MAX_*
# env vars at submit time if a particular pool needs different tuning.
DOCKERD_MAX_DOWNLOADS="${DOCKERD_MAX_DOWNLOADS:-10}"
DOCKERD_MAX_UPLOADS="${DOCKERD_MAX_UPLOADS:-10}"

_start_dockerd() {
  local driver="$1"
  : > /var/log/dockerd.log
  # We rely on /var/lib/docker being a bind-mount of /mnt/airstack-data
  # (200Gi Cinder volume) so during-pull disk peaks aren't constrained by
  # node ephemeral-storage.
  nohup dockerd \
    --host=unix:///var/run/docker.sock \
    --storage-driver="$driver" \
    --max-concurrent-downloads="$DOCKERD_MAX_DOWNLOADS" \
    --max-concurrent-uploads="$DOCKERD_MAX_UPLOADS" \
    > /var/log/dockerd.log 2>&1 &
  DOCKERD_PID=$!
  log "dockerd started (pid=$DOCKERD_PID, storage-driver=$driver); waiting for socket"
  for i in $(seq 1 30); do
    if docker info >/dev/null 2>&1; then
      log "dockerd ready (storage-driver=$driver)"
      return 0
    fi
    if ! kill -0 "$DOCKERD_PID" 2>/dev/null; then
      log "dockerd exited; tailing /var/log/dockerd.log:"
      tail -40 /var/log/dockerd.log | sed 's/^/[dockerd] /'
      return 1
    fi
    sleep 1
  done
  log "dockerd unresponsive after 30s; tailing /var/log/dockerd.log:"
  tail -40 /var/log/dockerd.log | sed 's/^/[dockerd] /'
  kill "$DOCKERD_PID" 2>/dev/null || true
  return 1
}

DOCKERD_OK=false
for drv in overlay2 fuse-overlayfs vfs; do
  if _start_dockerd "$drv"; then
    DOCKERD_OK=true
    break
  fi
  log "WARN: dockerd failed with storage-driver=$drv; trying next"
done
if [ "$DOCKERD_OK" != "true" ]; then
  fail "dockerd refused to start with any of overlay2 / fuse-overlayfs / vfs"
fi

# ─── 3. Clone AirStack ─────────────────────────────────────────────────────

AIRSTACK_REPO_URL="${AIRSTACK_REPO_URL:-https://github.com/castacks/AirStack.git}"
AIRSTACK_BRANCH="${AIRSTACK_BRANCH:-main}"
AIRSTACK_ROOT=/root/AirStack

if [ ! -d "$AIRSTACK_ROOT/.git" ]; then
  log "cloning $AIRSTACK_REPO_URL ($AIRSTACK_BRANCH) -> $AIRSTACK_ROOT"
  git clone --recursive --branch "$AIRSTACK_BRANCH" "$AIRSTACK_REPO_URL" "$AIRSTACK_ROOT" \
    || fail "git clone failed"
else
  log "$AIRSTACK_ROOT already cloned (skipping)"
fi

# Make sure the airstack CLI is on PATH for interactive shells.
ln -sf "$AIRSTACK_ROOT/airstack.sh" /usr/local/bin/airstack
ln -sf "$AIRSTACK_ROOT/airstack.sh" /usr/local/bin/airstack.sh

# ─── 4. omni_pass.env from airlab-nucleus credential ───────────────────────

OMNI_PASS_FILE="$AIRSTACK_ROOT/simulation/isaac-sim/docker/omni_pass.env"

if [ -z "${OMNI_USER:-}" ] || [ -z "${OMNI_PASS:-}" ]; then
  log "WARN: airlab-nucleus OSMO credential not set."
  log "WARN: Run on your laptop:"
  log "WARN:   osmo credential set airlab-nucleus --type GENERIC \\"
  log "WARN:     --payload omni_user=<andrew_id> omni_pass=<nucleus_api_token> \\"
  log "WARN:     omni_server=omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1"
  log "WARN: Falling back to guest/guest (read-only Nucleus) — Isaac Sim assets may fail to load."
fi

# Default to read-only Nucleus access so a missing credential degrades
# instead of crashing the pod.
: "${OMNI_USER:=guest}"
: "${OMNI_PASS:=guest}"
: "${OMNI_SERVER:=omniverse://airlab-nucleus.andrew.cmu.edu/NVIDIA/Assets/Isaac/5.1}"

# If OMNI_PASS looks like a Nucleus API JWT (header starts with `eyJ`),
# switch to API-token auth: omniclient expects the literal sentinel
# username `$omni-api-token` paired with the JWT as the password.
# Setting OMNI_USER to the actual Andrew ID would route the JWT through
# the password-verification path instead and Nucleus would silently
# DENY (visible only in the auth-service log as
# `InternalCredentials.auth … 'username': '<andrew>' … status: DENIED`).
#
# docker-compose v2 interpolates env_file values, so the literal `$`
# must be doubled to `$$` to survive Compose's parser. The container
# ultimately sees `OMNI_USER=$omni-api-token`.
case "$OMNI_PASS" in
  eyJ*.*.*)
    log "OMNI_PASS looks like a JWT — using API-token auth (OMNI_USER=\$omni-api-token)"
    OMNI_USER_LINE='OMNI_USER=$$omni-api-token'
    ;;
  *)
    OMNI_USER_LINE="OMNI_USER=${OMNI_USER}"
    ;;
esac

log "writing $OMNI_PASS_FILE (${OMNI_USER_LINE}, omni_server=${OMNI_SERVER})"
cat > "$OMNI_PASS_FILE" <<EOF
${OMNI_USER_LINE}
OMNI_PASS=${OMNI_PASS}
OMNI_SERVER=${OMNI_SERVER}
ACCEPT_EULA=Y
OMNI_ENV_PRIVACY_CONSENT=Y
EOF
chmod 600 "$OMNI_PASS_FILE"

# ─── 5. docker login (airlab registry, inner dockerd) ──────────────────────

AIRLAB_REGISTRY="${AIRLAB_REGISTRY:-airlab-docker.andrew.cmu.edu}"
if [ -n "${AIRLAB_REGISTRY_USER:-}" ] && [ -n "${AIRLAB_REGISTRY_PASS:-}" ]; then
  log "docker login $AIRLAB_REGISTRY (inner dockerd)"
  echo "${AIRLAB_REGISTRY_PASS}" \
    | docker login "$AIRLAB_REGISTRY" \
        -u "${AIRLAB_REGISTRY_USER}" --password-stdin \
    || log "WARN: docker login $AIRLAB_REGISTRY failed — AirStack image-pull will probably fail"
else
  log "WARN: airlab-docker-login OSMO credential not set — inner image-pulls will fail."
  log "WARN: Run on your laptop:"
  log "WARN:   osmo credential set airlab-docker-login --type GENERIC \\"
  log "WARN:     --payload username=<andrew_id> password=<andrew_password>"
fi

# ─── 6. airstack up ────────────────────────────────────────────────────────

# Honor optional overrides passed in via OSMO env. Defaults match a "single
# robot, Isaac Sim with WebRTC livestream" dev session.
export AUTOLAUNCH="${AUTOLAUNCH:-true}"
export NUM_ROBOTS="${NUM_ROBOTS:-1}"
export ISAAC_SIM_LIVESTREAM="${ISAAC_SIM_LIVESTREAM:-true}"

# COMPOSE_PROFILES selection: the default `desktop,isaac-sim` from .env runs
# the standard isaac-sim service. If the student wants livestream, they (or
# we) swap to the isaac-sim-livestream profile, which is the OSMO-friendly
# variant defined in simulation/isaac-sim/docker/docker-compose.yaml.
if [ "$ISAAC_SIM_LIVESTREAM" = "true" ]; then
  export COMPOSE_PROFILES="${COMPOSE_PROFILES:-desktop,isaac-sim-livestream}"
else
  export COMPOSE_PROFILES="${COMPOSE_PROFILES:-desktop,isaac-sim}"
fi

log "airstack up (COMPOSE_PROFILES=$COMPOSE_PROFILES, NUM_ROBOTS=$NUM_ROBOTS, livestream=$ISAAC_SIM_LIVESTREAM)"
cd "$AIRSTACK_ROOT"
./airstack.sh up || log "WARN: airstack up exited non-zero — pod stays alive for debugging via SSH"

# ─── 7. Sleep ──────────────────────────────────────────────────────────────

log "entrypoint complete; sleeping forever so port-forwards keep working"
log "pod-side log paths:"
log "  - dockerd:    /var/log/dockerd.log"
log "  - airstack:   docker logs airstack-isaac-sim-1 / airstack-robot-desktop-1 / airstack-gcs-1"
exec sleep infinity
