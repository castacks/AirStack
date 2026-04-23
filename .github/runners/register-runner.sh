#!/usr/bin/env bash
# AirStack ephemeral GitHub Actions runner loop.
#
# Registers a fresh runner, executes exactly one job, then loops to re-register.
# The --ephemeral flag tells the GitHub API to remove the runner after one job,
# preventing stale registrations and cross-job state pollution.
#
# Setup (one-time, on the OpenStack VM):
#   1. Create a non-root runner user:
#        sudo useradd -m -s /bin/bash runner
#        sudo usermod -aG docker runner
#
#   2. Download and unpack the GitHub Actions runner into RUNNER_DIR:
#        sudo mkdir -p /opt/actions-runner
#        cd /opt/actions-runner
#        # Get the latest runner URL from:
#        # https://github.com/actions/runner/releases
#        curl -Lo actions-runner.tar.gz <URL>
#        tar xzf actions-runner.tar.gz
#        sudo chown -R runner:runner /opt/actions-runner
#
#   3. Store a GitHub PAT (repo scope for private repos, public_repo for public):
#        echo "ghp_YOUR_TOKEN_HERE" | sudo tee /etc/github-runner-pat
#        sudo chmod 600 /etc/github-runner-pat
#        sudo chown runner:runner /etc/github-runner-pat
#
#   4. Copy this script into the runner directory and make it executable:
#        sudo cp register-runner.sh /opt/actions-runner/register-runner.sh
#        sudo chown runner:runner /opt/actions-runner/register-runner.sh
#        sudo chmod +x /opt/actions-runner/register-runner.sh
#
#   5. Install the systemd unit (see airstack-runner.service) and enable it:
#        sudo cp airstack-runner.service /etc/systemd/system/
#        sudo systemctl daemon-reload
#        sudo systemctl enable --now airstack-runner.service
#
# Configuration: set these in /etc/github-runner-env (loaded by the systemd unit)
# or export them before running this script manually.

set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/YOUR_ORG/AirStack}"
# Derived from REPO_URL for the registration token API call, e.g. "YOUR_ORG/AirStack"
REPO_PATH="${REPO_PATH:-$(echo "$REPO_URL" | sed 's|https://github.com/||')}"
RUNNER_DIR="${RUNNER_DIR:-/opt/actions-runner}"
PAT_FILE="${PAT_FILE:-/etc/github-runner-pat}"
RUNNER_LABELS="${RUNNER_LABELS:-self-hosted,airstack,gpu}"
RUNNER_GROUP="${RUNNER_GROUP:-Default}"

if [ ! -f "$PAT_FILE" ]; then
  echo "ERROR: PAT file not found at $PAT_FILE" >&2
  exit 1
fi

echo "Starting ephemeral runner loop for $REPO_URL"

while true; do
  echo "[$(date -u +%FT%TZ)] Requesting registration token..."

  TOKEN=$(curl -sf -X POST \
    -H "Authorization: token $(cat "$PAT_FILE")" \
    -H "Accept: application/vnd.github+json" \
    "https://api.github.com/repos/${REPO_PATH}/actions/runners/registration-token" \
    | jq -r .token)

  if [ -z "$TOKEN" ] || [ "$TOKEN" = "null" ]; then
    echo "ERROR: Failed to obtain registration token. Check PAT and repo path." >&2
    sleep 30
    continue
  fi

  echo "[$(date -u +%FT%TZ)] Configuring runner (ephemeral)..."

  # --ephemeral: runner de-registers itself after completing one job.
  # --replace: allows re-registration with the same name after a restart.
  # Runner name encodes hostname + PID so parallel instances are unique.
  "$RUNNER_DIR/config.sh" \
    --url "$REPO_URL" \
    --token "$TOKEN" \
    --name "openstack-$(hostname -s)-$$" \
    --labels "$RUNNER_LABELS" \
    --runnergroup "$RUNNER_GROUP" \
    --ephemeral \
    --unattended \
    --replace

  echo "[$(date -u +%FT%TZ)] Runner configured. Waiting for a job..."

  # run.sh blocks until the job completes, then returns (ephemeral runner exits cleanly).
  "$RUNNER_DIR/run.sh" || true

  echo "[$(date -u +%FT%TZ)] Job finished. Re-registering..."

  # Brief pause to avoid hammering the API if config.sh / run.sh fail immediately.
  sleep 2
done
