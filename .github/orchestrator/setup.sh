#!/usr/bin/env bash
# One-time orchestrator-VM setup. Run as root on the airstack-ci-cd-orchestrator
# OpenStack instance after cloning the repo.
#
# Pre-reqs (do these *before* running this script):
#   1. ~/.config/openstack/clouds.yaml staged for the orchestrator user
#      (application credential — see .github/orchestrator/README.md).
#   2. /tmp/github-pat exists with the GitHub PAT contents.
#   3. This repo cloned somewhere readable (this script copies code out of
#      its containing directory).

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
apt-get install -y python3 python3-venv python3-pip

echo "==> Creating directories"
install -d -o "$USER_NAME" -g "$USER_NAME" -m 0750 "$INSTALL_DIR"
install -d -o root -g "$USER_NAME" -m 0750 "$CONFIG_DIR"
install -d -o "$USER_NAME" -g "$USER_NAME" -m 0750 "$STATE_DIR"

echo "==> Copying orchestrator files to $INSTALL_DIR"
install -o "$USER_NAME" -g "$USER_NAME" -m 0755 \
    "$REPO_DIR/orchestrator.py" "$INSTALL_DIR/orchestrator.py"
install -o "$USER_NAME" -g "$USER_NAME" -m 0644 \
    "$REPO_DIR/cloud-init.yaml.j2" "$INSTALL_DIR/cloud-init.yaml.j2"

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

echo "==> Verifying clouds.yaml"
CLOUDS_YAML="/home/$USER_NAME/.config/openstack/clouds.yaml"
if [[ ! -f "$CLOUDS_YAML" ]]; then
    echo "WARNING: $CLOUDS_YAML missing." >&2
    echo "         Create it (application credential) before starting the service." >&2
fi

echo "==> Installing systemd unit"
install -o root -g root -m 0644 \
    "$REPO_DIR/airstack-orchestrator.service" \
    /etc/systemd/system/airstack-orchestrator.service
systemctl daemon-reload

echo
echo "Setup complete. Next steps:"
echo "  1. Edit $CONFIG_DIR/config.yaml — fill flavor/network/keypair/security_group."
echo "  2. Verify $CLOUDS_YAML exists with the application credential."
echo "  3. systemctl enable --now airstack-orchestrator.service"
echo "  4. journalctl -u airstack-orchestrator.service -f"
