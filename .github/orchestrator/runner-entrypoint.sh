#!/usr/bin/env bash
# Entry point for the AirStack CI ephemeral-runner container (an OSMO task).
#
# Starts an inner Docker daemon — the AirStack test harness runs `airstack up`
# (docker compose) inside this container — waits for it, then runs exactly ONE
# ephemeral GitHub Actions job via the single-use JIT config. When run.sh exits,
# the OSMO task completes and the pod is destroyed: one job per pod, same as the
# old OpenStack VM.
#
# Requires a privileged pod (dockerd) scheduled on a GPU platform; the NVIDIA
# container toolkit (baked into the image) lets the inner dockerd pass the node
# GPU through to the compose containers.
set -euo pipefail

: "${ENCODED_JIT_CONFIG:?ENCODED_JIT_CONFIG must be set by the workflow}"

log() { echo "[runner-entrypoint] $*"; }

# ---------------------------------------------------------------------------
# Docker storage backend
#
# The pod's root filesystem is overlayfs, and Linux refuses to use a directory
# on overlayfs as an overlay `upperdir` (EINVAL). A dockerd whose data-root sits
# on the pod rootfs still *pulls* images fine — containerd unpacks layers with
# plain writes — but every build step that needs a real mount dies with:
#
#   failed to solve: ... mount source: "overlay",
#   target: "/var/lib/docker/buildkit/containerd-overlayfs/cachemounts/...",
#   err: invalid argument
#
# which is what took out all of build_docker / build_packages. So put the
# data-root somewhere overlay actually works, and verify it by performing a real
# overlay mount rather than trusting the filesystem type.
# ---------------------------------------------------------------------------

DOCKER_DATA_ROOT=/var/lib/docker
LOOP_IMG=/docker-data.img
# Headroom left for everything that is not the Docker data-root (the runner's
# _work checkout, logs, the loop image's own metadata).
LOOP_HEADROOM_MB=20480
MIN_BACKING_MB=51200   # Below ~50 GiB the sim images can't fit regardless.

# True if an overlay mount whose upperdir lives under $1 can actually be made.
overlay_upperdir_works() {
  local base=$1 probe rc=1
  probe=$(mktemp -d "$base/.overlay-probe.XXXXXX" 2>/dev/null) || return 1
  mkdir -p "$probe"/{lower,upper,work,merged}
  if mount -t overlay overlay \
      -o "lowerdir=$probe/lower,upperdir=$probe/upper,workdir=$probe/work" \
      "$probe/merged" 2>/dev/null; then
    umount "$probe/merged" && rc=0
  fi
  rm -rf "$probe"
  return $rc
}

free_mb() { df -Pm "$1" | awk 'NR==2 {print $4}'; }

# Preferred: a loopback ext4 image mounted at the data-root. Self-contained
# (no dependency on how the pool exposes storage), gives real overlay2, and dies
# with the pod. The image file is sparse, so it only consumes what Docker writes.
setup_loopback() {
  local size_mb=${DOCKER_LOOP_SIZE_MB:-}
  if [[ -z "$size_mb" ]]; then
    size_mb=$(( $(free_mb /) - LOOP_HEADROOM_MB ))
  fi
  if (( size_mb < MIN_BACKING_MB )); then
    log "loopback: only ${size_mb}MB usable, need ${MIN_BACKING_MB}MB — skipping"
    return 1
  fi

  # /dev/loop-control only exists once the loop module is loaded on the node.
  [[ -e /dev/loop-control ]] || modprobe loop 2>/dev/null || true
  if [[ ! -e /dev/loop-control ]]; then
    log "loopback: no /dev/loop-control — skipping"
    return 1
  fi

  log "loopback: creating ${size_mb}MB ext4 image at $LOOP_IMG"
  truncate -s "${size_mb}M" "$LOOP_IMG" || return 1
  # No journal and lazy inode-table init: this filesystem never outlives the
  # pod, so durability buys nothing and mkfs stays fast.
  mkfs.ext4 -q -F -m 0 -O ^has_journal -E lazy_itable_init=1 "$LOOP_IMG" || return 1

  mkdir -p "$DOCKER_DATA_ROOT"
  mount -o loop "$LOOP_IMG" "$DOCKER_DATA_ROOT" || return 1

  if ! overlay_upperdir_works "$DOCKER_DATA_ROOT"; then
    log "loopback: mounted but overlay still rejected — unwinding"
    umount "$DOCKER_DATA_ROOT" || true
    rm -f "$LOOP_IMG"
    return 1
  fi
  return 0
}

# Fallback: a real filesystem already mounted into the pod. Kubernetes emptyDir,
# hostPath and PVC volumes are backed by the node disk rather than the overlay
# rootfs, so overlay works there.
setup_real_fs() {
  local best="" best_free=0 mnt fstype opts avail
  while read -r _ mnt fstype opts _; do
    case "$fstype" in ext2|ext3|ext4|xfs|btrfs) ;; *) continue ;; esac
    [[ -d "$mnt" && -w "$mnt" ]] || continue
    [[ ",$opts," == *",ro,"* ]] && continue
    # OSMO's ctrl sidecar owns these: /osmo/data/output is uploaded as job
    # artifacts and the socket dir is its IPC channel.
    case "$mnt" in /osmo/data/output*|/osmo/data/socket*) continue ;; esac
    avail=$(free_mb "$mnt")
    if (( avail > best_free )); then best_free=$avail; best=$mnt; fi
  done < /proc/mounts

  if [[ -z "$best" ]] || (( best_free < MIN_BACKING_MB )); then
    log "real-fs: no mounted filesystem with >=${MIN_BACKING_MB}MB free — skipping"
    return 1
  fi

  DOCKER_DATA_ROOT="$best/airstack-docker-data"
  mkdir -p "$DOCKER_DATA_ROOT"
  if ! overlay_upperdir_works "$DOCKER_DATA_ROOT"; then
    log "real-fs: overlay rejected under $best — skipping"
    DOCKER_DATA_ROOT=/var/lib/docker
    return 1
  fi
  log "real-fs: using $DOCKER_DATA_ROOT (${best_free}MB free)"
  return 0
}

mkdir -p /etc/docker
if setup_loopback; then
  log "storage: overlay2 on a loopback ext4 image"
  printf '{"data-root": "%s"}\n' "$DOCKER_DATA_ROOT" > /etc/docker/daemon.json
elif setup_real_fs; then
  log "storage: overlay2 on $DOCKER_DATA_ROOT"
  printf '{"data-root": "%s"}\n' "$DOCKER_DATA_ROOT" > /etc/docker/daemon.json
elif [[ -e /dev/fuse ]] && command -v fuse-overlayfs >/dev/null 2>&1; then
  # fuse-overlayfs stacks on overlayfs where the kernel driver won't. Slower
  # than overlay2 but nowhere near as bad as vfs. `storage-driver` only applies
  # to the classic image store, so the containerd snapshotter has to go.
  log "storage: fuse-overlayfs (no overlay-capable filesystem found)"
  cat > /etc/docker/daemon.json <<'JSON'
{"storage-driver": "fuse-overlayfs", "features": {"containerd-snapshotter": false}}
JSON
else
  # Always works, but copies the whole filesystem per layer. The sim images are
  # large enough that this will likely exhaust the pod's storage request.
  log "WARN: storage: falling back to vfs — builds will be slow and may run out of disk"
  cat > /etc/docker/daemon.json <<'JSON'
{"storage-driver": "vfs", "features": {"containerd-snapshotter": false}}
JSON
fi

# Start dockerd in the background (needs privileged).
dockerd >/var/log/dockerd.log 2>&1 &

# Wait for the daemon to accept connections (~60s budget).
for _ in $(seq 1 60); do
  if docker info >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if ! docker info >/dev/null 2>&1; then
  echo "ERROR: dockerd did not become ready" >&2
  cat /var/log/dockerd.log >&2 || true
  exit 1
fi

log "storage driver: $(docker info --format '{{.Driver}}' 2>/dev/null || echo unknown)" \
    "data-root: $(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo unknown)"

# Non-fatal GPU sanity check — surfaces GPU/privileged/toolkit misconfig early.
nvidia-smi || echo "WARN: nvidia-smi unavailable (check GPU + privileged + toolkit)"

cd /home/runner/actions-runner
# The JIT config makes this runner single-use + ephemeral; run.sh returns after
# one job, which completes the task and lets OSMO reap the pod.
exec ./run.sh --jitconfig "${ENCODED_JIT_CONFIG}"
