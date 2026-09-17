#!/usr/bin/env bash
# Persistent PSC cache bootstrap for the RRM shadow-reasoner candidate.
# Run with: RRM_PERSIST_ROOT=/ocean/projects/<group>/<user>/physical-ai \
#   bash scripts/psc_cosmos_reason2_8b.sh preflight|download

set -euo pipefail

readonly MODEL_ID="nvidia/Cosmos-Reason2-8B"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  RRM_PERSIST_ROOT=/ocean/projects/<group>/<user>/physical-ai \
    bash scripts/psc_cosmos_reason2_8b.sh preflight|download

preflight  Validate the persistent cache root and Hugging Face authentication.
download   Cache nvidia/Cosmos-Reason2-8B once in PSC Ocean and write a
           non-secret revision/checksum manifest.

The script never accepts or prints tokens.  Run download only after accepting the
model's Hugging Face terms and only from a PSC context allowed to transfer data.
EOF
}

require_root() {
  : "${RRM_PERSIST_ROOT:?Set RRM_PERSIST_ROOT to the PSC Ocean project root.}"
  [[ "$RRM_PERSIST_ROOT" = /* ]] || die "RRM_PERSIST_ROOT must be an absolute path"
  [[ "$RRM_PERSIST_ROOT" != /tmp && "$RRM_PERSIST_ROOT" != /tmp/* ]] || \
    die "RRM_PERSIST_ROOT must not be in /tmp"
  [[ -d "$RRM_PERSIST_ROOT" ]] || die "RRM_PERSIST_ROOT does not exist: $RRM_PERSIST_ROOT"
}

cache_root() {
  printf '%s/checkpoints/cosmos/hf\n' "$RRM_PERSIST_ROOT"
}

manifest_root() {
  printf '%s/checkpoints/cosmos/manifests\n' "$RRM_PERSIST_ROOT"
}

preflight() {
  require_root
  command -v hf >/dev/null 2>&1 || die "Hugging Face CLI 'hf' is not installed"
  printf 'model_id=%s\ncache_root=%s\n' "$MODEL_ID" "$(cache_root)"
  df -h "$RRM_PERSIST_ROOT"
  hf auth whoami >/dev/null || die "Run 'hf auth login' with a read-only token after accepting model terms"
  printf 'Hugging Face authentication and persistent cache root are ready. No files downloaded.\n'
}

download() {
  preflight
  local cache manifests revision snapshot checksum_file metadata_file
  cache="$(cache_root)"
  manifests="$(manifest_root)"
  mkdir -p "$cache" "$manifests"

  printf 'Downloading %s to persistent cache %s\n' "$MODEL_ID" "$cache"
  hf download "$MODEL_ID" --cache-dir "$cache"

  revision="$(cat "$cache/models--nvidia--Cosmos-Reason2-8B/refs/main")"
  snapshot="$cache/models--nvidia--Cosmos-Reason2-8B/snapshots/$revision"
  [[ -d "$snapshot" ]] || die "Expected snapshot directory is missing: $snapshot"
  checksum_file="$manifests/cosmos-reason2-8b-$revision.sha256"
  metadata_file="$manifests/cosmos-reason2-8b-$revision.json"
  (
    cd "$snapshot"
    find -L . -type f -print0 | sort -z | xargs -0 sha256sum
  ) >"$checksum_file"
  python3 - "$metadata_file" "$revision" "$cache" "$checksum_file" <<'PY'
import json
import sys
from datetime import datetime, timezone

path, revision, cache, checksums = sys.argv[1:]
payload = {
    "model_id": "nvidia/Cosmos-Reason2-8B",
    "revision": revision,
    "cache_root": cache,
    "checksums": checksums,
    "license": "NVIDIA Open Model License Agreement",
    "rrm_role": "shadow-only C04/C05 candidate producer; never control or dispatch",
    "retrieved_at_utc": datetime.now(timezone.utc).isoformat(),
}
with open(path, "w", encoding="utf-8") as stream:
    json.dump(payload, stream, indent=2, sort_keys=True)
    stream.write("\n")
PY
  printf 'Cached revision=%s\nmanifest=%s\nchecksums=%s\n' \
    "$revision" "$metadata_file" "$checksum_file"
}

case "${1:-}" in
  preflight) preflight ;;
  download) download ;;
  -h|--help|help) usage ;;
  *) usage >&2; exit 2 ;;
esac
