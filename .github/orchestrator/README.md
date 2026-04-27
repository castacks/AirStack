# AirStack CI Orchestrator

Long-running service that watches GitHub for queued workflow jobs and spawns truly ephemeral OpenStack instances to execute each one. The orchestrator VM is the only host that holds the GitHub PAT and the OpenStack credential; the workers are destroyed after a single job.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Orchestrator VM  (airstack-ci-cd-orchestrator)             │
│                                                             │
│  airstack-orchestrator.service → orchestrator.py            │
│    spawn loop  (every 15s):                                 │
│      • GET  /repos/<repo>/actions/runs?status=queued        │
│      • POST /repos/<repo>/actions/runners/generate-jitconfig│
│      • openstack server create  (image, flavor, user_data)  │
│      • record (job_id → server_id) in state.json            │
│    reap loop   (every 30s):                                 │
│      • job completed     → openstack server delete          │
│      • job age > N min   → force delete (straggler)         │
│      • owned but not in state → orphan reap                 │
│                                                             │
│  /etc/airstack-orchestrator/                                │
│      config.yaml                                            │
│      github-pat                                             │
│  /home/orchestrator/.config/openstack/clouds.yaml           │
│  /var/lib/airstack-orchestrator/state.json                  │
└─────────┬─────────────────────────────────┬─────────────────┘
          │ Nova / Neutron API              │ GitHub REST API
          ▼                                 ▼
┌──────────────────────────────────┐  ┌──────────────────────┐
│  Ephemeral worker (per job)      │  │  GitHub Actions      │
│  Image: Ubuntu-24.04-GPU-Headless│  │  workflow_job queue  │
│  cloud-init:                     │  └──────────────────────┘
│    install docker + nv toolkit   │
│    download GH runner            │
│    run.sh --jitconfig <token>    │
│    shutdown -h +1                │
└──────────────────────────────────┘
```

Key properties:

- **Truly ephemeral**: every job runs on a clean VM. No Docker layer cache pollution, no leftover networks, no carry-over from prior runs.
- **PAT isolation**: the GitHub PAT lives only on the orchestrator. Workers receive a single-use [JIT runner config](https://docs.github.com/en/rest/actions/self-hosted-runners?apiVersion=2022-11-28#create-configuration-for-a-just-in-time-runner-for-a-repository) — a base64 token bound to one runner registration, valid only for a short window.
- **Application-credential auth**: the orchestrator authenticates to OpenStack with an application credential (revocable, scoped, no password), not the user's `openrc.sh`.
- **Crash-safe reaping**: every server we spawn is tagged with `airstack-role=ephemeral-runner`. The reap loop force-deletes any owned server not present in `state.json`, so a crashed orchestrator can't leak instances.

## One-time setup

### 1. Create OpenStack application credential

On your local workstation (not the orchestrator VM):

```bash
source ~/.airlabcloud/openrc.sh
openstack application credential create airstack-orchestrator \
  --description "AirStack CI orchestrator — spawns ephemeral test runners"
```

The output prints `id` and `secret`. Build a `clouds.yaml`:

```yaml
clouds:
  airstack:
    auth_type: v3applicationcredential
    auth:
      auth_url: https://airlab-cloud.andrew.cmu.edu:5000/v3/
      application_credential_id: <id from above>
      application_credential_secret: <secret from above>
    region_name: Airlab
    interface: public
    identity_api_version: 3
```

### 2. Stage credentials on the orchestrator VM

```bash
# clouds.yaml: install for the orchestrator user (created in step 3)
scp clouds.yaml ubuntu@<orchestrator-ip>:/tmp/clouds.yaml

# GitHub PAT: needs `Actions: read/write` and `Administration: read/write`
# (fine-grained) or classic `repo` scope.
scp ~/.airlabcloud/airstack-github-pat.txt \
    ubuntu@<orchestrator-ip>:/tmp/github-pat
```

### 3. Run setup.sh

On the orchestrator VM:

```bash
git clone https://github.com/castacks/AirStack.git /tmp/airstack
sudo bash /tmp/airstack/.github/orchestrator/setup.sh
```

`setup.sh` creates the `orchestrator` system user, builds the Python venv, copies `orchestrator.py` and `cloud-init.yaml.j2` into `/opt/airstack-orchestrator/`, scaffolds `/etc/airstack-orchestrator/`, installs the systemd unit, and consumes `/tmp/github-pat`.

You still need to put the `clouds.yaml` in place under the orchestrator user's home:

```bash
sudo install -d -o orchestrator -g orchestrator -m 0700 \
    /home/orchestrator/.config/openstack
sudo install -o orchestrator -g orchestrator -m 0600 \
    /tmp/clouds.yaml /home/orchestrator/.config/openstack/clouds.yaml
sudo shred -u /tmp/clouds.yaml
```

### 4. Fill in `/etc/airstack-orchestrator/config.yaml`

Edit the placeholders the example ships with:

| Field | What goes here | How to find it |
|------|---------------|----------------|
| `flavor_name` | OpenStack flavor with GPU + enough disk | `openstack flavor list` |
| `network_name` | Network the workers attach to | `openstack network list` |
| `keypair_name` | SSH keypair for break-glass access | `openstack keypair list` |
| `security_group` | Outbound 443 must be allowed | `openstack security group list` |
| `availability_zone` | Optional AZ for the spawned instance; leave empty to let Nova pick | `openstack availability zone list` |
| `repo` | `owner/name` of the repo to poll | from GitHub URL |
| `runner_version` | Version tag from [actions/runner releases](https://github.com/actions/runner/releases) | check before each major upgrade |

### 5. Start the service

```bash
sudo systemctl enable --now airstack-orchestrator.service
journalctl -u airstack-orchestrator.service -f
```

You should see `orchestrator started: repo=... labels=... max_concurrent=N` and then periodic poll activity.

## End-to-end verification

```bash
# Trigger a fast build-only run.
gh workflow run integration-tests.yml -f marks=build_docker

# Within ~30s, a server should appear:
openstack server list --metadata airstack-role=ephemeral-runner

# Watch GitHub → Actions → Runners — the ephemeral runner should appear,
# pick up the job, then disappear.

# Within ~30s of job completion, the server should be gone:
openstack server list --metadata airstack-role=ephemeral-runner
```

## Operational notes

- **State file**: `/var/lib/airstack-orchestrator/state.json` is the in-flight job tracker. Wiping it triggers an orphan sweep on the next reap iteration — owned servers will be force-deleted. Don't wipe it while jobs are mid-flight unless that's what you want.
- **Stuck instance**: any server older than `max_job_minutes` (default 90) is force-deleted regardless of GitHub job status. Bump this if liveliness/autonomy runs grow longer than ~75 minutes.
- **PAT rotation**: `sudo install -o root -g orchestrator -m 0640 /tmp/new-pat /etc/airstack-orchestrator/github-pat && sudo systemctl restart airstack-orchestrator.service`.
- **Pause spawning** (e.g. for maintenance): `sudo systemctl stop airstack-orchestrator.service`. Already-spawned workers will still complete their jobs and self-shutdown; on restart, the reap loop deletes them.
- **Logs**: `journalctl -u airstack-orchestrator.service -f`. Cloud-init logs from individual workers are visible only via `openstack console log show <server>` while the worker is running.
