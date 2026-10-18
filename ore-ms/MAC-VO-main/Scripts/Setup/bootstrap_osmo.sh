#!/usr/bin/env bash
# Prepare a disposable OSMO workflow for Box-backed data staging.
#
# This script deliberately never contains a Box credential. Supply an OSMO
# workflow secret named RCLONE_CONFIG_B64 only after completing the Box OAuth
# flow yourself. Its value is the base64-encoded rclone.conf. Set
# RCLONE_REMOTE to the configured remote name (default: ore_ms).

set -euo pipefail

readonly RCLONE_CONFIG_DIR="${RCLONE_CONFIG_DIR:-${HOME}/.config/rclone}"
readonly RCLONE_CONFIG_FILE="${RCLONE_CONFIG:-${RCLONE_CONFIG_DIR}/rclone.conf}"
readonly RCLONE_REMOTE="${RCLONE_REMOTE:-ore_ms}"
readonly RCLONE_CONFIG_B64="${RCLONE_CONFIG_B64:-${RCLONE_BOX_CONFIG_B64:-}}"

if ! command -v rclone >/dev/null 2>&1; then
    echo "rclone is not installed. Use the project Docker image, which installs it, or install rclone in this workflow image." >&2
    exit 1
fi

if [[ -z "${RCLONE_CONFIG_B64}" ]]; then
    cat >&2 <<'EOF'
Box is not configured in this workflow.

Set the OSMO secret RCLONE_CONFIG_B64 to the base64-encoded rclone.conf
created by your one-time Box OAuth setup. Set RCLONE_REMOTE to the rclone
remote name (currently ore_ms). Do not put that token in this repo, Box, a
notebook, or a shell-history file.
EOF
    exit 2
fi

umask 077
mkdir -p "$(dirname "${RCLONE_CONFIG_FILE}")"
printf '%s' "${RCLONE_CONFIG_B64}" | base64 --decode > "${RCLONE_CONFIG_FILE}"
chmod 600 "${RCLONE_CONFIG_FILE}"

if ! rclone listremotes | grep -qx "${RCLONE_REMOTE}:"; then
    echo "The supplied rclone configuration does not define '${RCLONE_REMOTE}:'." >&2
    exit 3
fi

cat <<EOF
rclone is ready. The Box token is available only at:
  ${RCLONE_CONFIG_FILE}

Example pull (use copy, never sync, for experiment inputs):
  rclone copyto ${RCLONE_REMOTE}:ore-ms/macslam.zip ../data/archives/macslam.zip --progress
EOF
