#!/usr/bin/env bash
# One-time orchestrator-VM setup (OSMO backend). Run as root on the
# airstack-ci-cd-orchestrator instance after cloning the repo.
#
# Pre-reqs (do these *before* running this script):
#   1. /tmp/github-pat exists with the GitHub PAT contents.
#   2. /tmp/osmo-token exists with the OSMO service-account access token
#      (from `osmo token set` — see .github/orchestrator/README.md). Optional
#      at setup time; you can stage it later before starting the service.
#   3. This repo cloned somewhere readable (this script copies code out of
#      its containing directory).
#
# The orchestrator host is lightweight and needs NO GPU — it only polls GitHub
# and submits OSMO workflows. 1 vCPU / 2GB RAM / 20GB disk is plenty.

set -euo pipefail

INSTALL_DIR=/opt/airstack-orchestrator
CONFIG_DIR=/etc/airstack-orchestrator
STATE_DIR=/var/lib/airstack-orchestrator
USER_NAME=orchestrator

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: setup.sh must run as root" >&2
    exit 1
fi

echo "==> Creating orchestrator user"
if ! id "$USER_NAME" >/dev/null 2>&1; then
    useradd --system --create-home --shell /usr/sbin/nologin "$USER_NAME"
fi

echo "==> Installing system packages"
apt-get update
apt-get install -y python3 python3-venv python3-pip curl ca-certificates

echo "==> Installing the OSMO CLI"
if command -v osmo >/dev/null 2>&1; then
    echo "    osmo already installed ($(command -v osmo)); skipping"
else
    # Latest client. Pin to a release from https://github.com/NVIDIA/OSMO/releases
    # if you need a specific version.
    curl -fsSL https://raw.githubusercontent.com/NVIDIA/OSMO/refs/heads/main/install.sh | bash
    command -v osmo >/dev/null 2>&1 \
        || echo "WARNING: osmo not on PATH after install — check the installer output" >&2
fi

echo "==> Creating directories"
install -d -o "$USER_NAME" -g "$USER_NAME" -m 0750 "$INSTALL_DIR"
install -d -o root -g "$USER_NAME" -m 0750 "$CONFIG_DIR"
install -d -o "$USER_NAME" -g "$USER_NAME" -m 0750 "$STATE_DIR"
# XDG dirs for the osmo CLI login session (see the systemd unit's HOME/XDG env).
install -d -o "$USER_NAME" -g "$USER_NAME" -m 0700 "$STATE_DIR/.config"
install -d -o "$USER_NAME" -g "$USER_NAME" -m 0700 "$STATE_DIR/.cache"
install -d -o "$USER_NAME" -g "$USER_NAME" -m 0700 "$STATE_DIR/.state"

echo "==> Copying orchestrator files to $INSTALL_DIR"
install -o "$USER_NAME" -g "$USER_NAME" -m 0755 \
    "$REPO_DIR/orchestrator.py" "$INSTALL_DIR/orchestrator.py"
install -o "$USER_NAME" -g "$USER_NAME" -m 0644 \
    "$REPO_DIR/runner-workflow.yaml.j2" "$INSTALL_DIR/runner-workflow.yaml.j2"

echo "==> Building Python venv"
sudo -u "$USER_NAME" python3 -m venv "$INSTALL_DIR/venv"
sudo -u "$USER_NAME" "$INSTALL_DIR/venv/bin/pip" install --upgrade pip
sudo -u "$USER_NAME" "$INSTALL_DIR/venv/bin/pip" install -r "$REPO_DIR/requirements.txt"

echo "==> Staging config (if not present)"
if [[ ! -f "$CONFIG_DIR/config.yaml" ]]; then
    install -o root -g "$USER_NAME" -m 0640 \
        "$REPO_DIR/config.example.yaml" "$CONFIG_DIR/config.yaml"
    echo "    config.yaml installed from example — edit before starting service"
fi

echo "==> Installing GitHub PAT (from /tmp/github-pat)"
if [[ ! -f /tmp/github-pat ]]; then
    echo "ERROR: /tmp/github-pat not found. scp it over before running setup." >&2
    exit 1
fi
install -o root -g "$USER_NAME" -m 0640 /tmp/github-pat "$CONFIG_DIR/github-pat"
shred -u /tmp/github-pat

echo "==> Installing OSMO service-account token (from /tmp/osmo-token)"
if [[ -f /tmp/osmo-token ]]; then
    install -o root -g "$USER_NAME" -m 0640 /tmp/osmo-token "$CONFIG_DIR/osmo-token"
    shred -u /tmp/osmo-token
else
    echo "WARNING: /tmp/osmo-token not found." >&2
    echo "         Stage the OSMO service-account token before starting the service:" >&2
    echo "           sudo install -o root -g $USER_NAME -m 0640 /tmp/osmo-token $CONFIG_DIR/osmo-token" >&2
fi

echo "==> Installing systemd unit"
install -o root -g root -m 0644 \
    "$REPO_DIR/airstack-orchestrator.service" \
    /etc/systemd/system/airstack-orchestrator.service
systemctl daemon-reload

echo
echo "Setup complete. Next steps:"
echo "  1. Edit $CONFIG_DIR/config.yaml — set osmo_url, pool, platform,"
echo "     runner_image, and resources."
echo "  2. Ensure $CONFIG_DIR/osmo-token holds the OSMO service-account token."
echo "  3. systemctl enable --now airstack-orchestrator.service"
echo "  4. journalctl -u airstack-orchestrator.service -f"
