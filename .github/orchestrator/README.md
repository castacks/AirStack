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
| `boot_volume_size_gb` | Set >0 if your flavor has `disk=0` (common for GPU flavors) — boots from a Cinder volume of this size sourced from `image_id`; leave 0 for direct image-boot | `openstack flavor show <flavor>` (check disk field) |
| `floating_ips` | Pre-allocated FIP pool, rotated through sequentially — each spawn picks the first free one. `max_concurrent` is capped at `len(pool)`. Leave empty to skip FIP attachment | `openstack floating ip list` |
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
gh workflow run system-tests.yml -f marks=build_docker

# Within ~30s, a server should appear:
openstack server list --metadata airstack-role=ephemeral-runner
# or if your OpenStack setup doesn't support metadata queries:
openstack server list --name '^ephemeral-'

# Watch GitHub → Actions → Runners — the ephemeral runner should appear,
# pick up the job, then disappear.

# Within ~30s of job completion, the server should be gone:
openstack server list --metadata airstack-role=ephemeral-runner
openstack server list --name '^ephemeral-'
```

## Operational notes

- **State file**: `/var/lib/airstack-orchestrator/state.json` is the in-flight job tracker. Wiping it triggers an orphan sweep on the next reap iteration — owned servers will be force-deleted. Don't wipe it while jobs are mid-flight unless that's what you want.
- **Stuck instance**: any server older than `max_job_minutes` (default 90) is force-deleted regardless of GitHub job status. Bump this if liveliness/autonomy runs grow longer than ~75 minutes.
- **PAT rotation**: `sudo install -o root -g orchestrator -m 0640 /tmp/new-pat /etc/airstack-orchestrator/github-pat && sudo systemctl restart airstack-orchestrator.service`.
- **Pause spawning** (e.g. for maintenance): `sudo systemctl stop airstack-orchestrator.service`. Already-spawned workers will still complete their jobs and self-shutdown; on restart, the reap loop deletes them.
- **Logs**: `journalctl -u airstack-orchestrator.service -f`. Cloud-init logs from individual workers are visible only via `openstack console log show <server>` while the worker is running.

## Debugging a failed job

When a GitHub workflow run fails or stalls, the failure can be in any of four places: the orchestrator (didn't spawn), cloud-init (didn't bootstrap), the GH Actions runner (didn't register or crashed), or the workflow steps themselves. Each has a different inspection path.

### 1. Find which worker ran the job

`state.json` is the authoritative job ↔ server ↔ floating-IP map:

```bash
sudo jq -r '.jobs | to_entries[] | "\(.key)\t\(.value.server_id)\t\(.value.floating_ip)\t\(.value.runner_name)"' \
  /var/lib/airstack-orchestrator/state.json
```

Pick the row for your failing `job_id` (visible in the GitHub Actions URL). Save the values:

```bash
JOB_ID=73286176852          # from the GitHub UI
SERVER=$(sudo jq -r ".jobs[\"$JOB_ID\"].server_id"   /var/lib/airstack-orchestrator/state.json)
FIP=$(   sudo jq -r ".jobs[\"$JOB_ID\"].floating_ip" /var/lib/airstack-orchestrator/state.json)
```

If the job isn't in `state.json`, the orchestrator never spawned for it — see step 2 below.

### 2. Did the orchestrator spawn at all?

```bash
sudo journalctl -u airstack-orchestrator.service --since "30 min ago" --no-pager
```

What you want to see for a healthy spawn:

```text
spawned server <uuid> for job <job_id> (<job name>)
attached floating IP <addr> to server <uuid> (job <job_id>)
```

Common things that block a spawn (and how to spot them):

| Log line / symptom | What it means | Fix |
|---|---|---|
| `find_queued_jobs failed: 401 ...` | PAT expired / wrong scope | Rotate the PAT (see Operational notes) |
| `spawn failed for job ...: Block Device Mapping is Invalid` | Flavor has `disk=0` and `boot_volume_size_gb` is 0 | Set `boot_volume_size_gb > 0` |
| `no free floating IP in pool` | All FIPs in `floating_ips` are already in use | Wait for an in-flight job to complete, or expand the pool |
| `floating_ips configured but not found` | Pool addresses don't exist in the project | Double-check `openstack floating ip list` |
| Job is queued in GitHub but no `spawned` log | Runner labels in the workflow's `runs-on` don't match `runner_labels` in config | Make them match |

### 3. SSH into a running worker

If the worker is `ACTIVE`, the floating IP is attached and you can connect directly. The keypair was injected during spawn — use the matching private key:

```bash
ssh -i <keypair>.pem ubuntu@"$FIP"
```

If your workstation can't reach the FIP subnet, jump through the orchestrator (which is on the same network):

```bash
ssh -J ubuntu@<orchestrator-ip> -i <keypair>.pem ubuntu@"$FIP"
```

### 4. SSH into a SHUTOFF worker

Workers shut themselves down after `run.sh` exits (whether the job succeeded, failed, or the runner crashed). The orchestrator only deletes a server once GitHub reports the job `completed`, so a SHUTOFF worker is preserved while you debug.

```bash
# Optional but safer — keep the orchestrator from reaping mid-session.
sudo systemctl stop airstack-orchestrator.service

openstack server start "$SERVER"
# Wait ~30s, then SSH using the FIP from state.json.
ssh -i <keypair>.pem ubuntu@"$FIP"
```

When done, delete the worker manually and resume the orchestrator:

```bash
openstack server delete "$SERVER"
sudo jq "del(.jobs[\"$JOB_ID\"])" /var/lib/airstack-orchestrator/state.json \
  | sudo tee /var/lib/airstack-orchestrator/state.json.new >/dev/null
sudo mv /var/lib/airstack-orchestrator/state.json.new /var/lib/airstack-orchestrator/state.json
sudo systemctl start airstack-orchestrator.service
```

### 5. What to read once you're on the worker

```bash
# Combined boot + cloud-init output. Most useful single file: shows every
# line our airstack-runner-bootstrap.sh printed, including run.sh's exit.
sudo less /var/log/cloud-init-output.log
sudo tail -300 /var/log/cloud-init-output.log

# Cloud-init's structured log — quick way to surface errors.
sudo grep -E 'WARN|ERROR|FAIL' /var/log/cloud-init.log

# GitHub Actions runner diagnostics. The Worker_*.log corresponds to the
# actual job execution; Runner_*.log covers registration and dispatch.
ls -lt /home/ubuntu/actions-runner/_diag/
sudo tail -300 /home/ubuntu/actions-runner/_diag/Runner_*.log
sudo tail -300 /home/ubuntu/actions-runner/_diag/Worker_*.log

# Sanity-check Docker came up cleanly — a frequent failure point.
sudo systemctl status docker
docker info 2>&1 | head
```

### 6. Console log fallback

Some flavors on this cloud don't expose the serial console (`openstack console log show` returns *Guest does not have a console available*). For those, the SSH path above is the only option. Where it does work, the console log persists across SHUTOFF and is faster than restarting the VM:

```bash
openstack console log show "$SERVER" | tail -200
```

### 7. Common failure patterns at the worker

| Symptom in `cloud-init-output.log` (near end) | Cause | Fix |
|---|---|---|
| `Could not connect to api.github.com` / DNS errors | Security group blocking egress, or no NAT for the network | Allow outbound 443; if behind NAT, ensure FIP networking covers egress |
| `Bad credentials` / `Invalid configuration ... runnerEvent` | JIT config TTL elapsed before `run.sh` started — bootstrap took too long | Pre-bake Docker + nvidia-container-toolkit into the image to shrink bootstrap |
| `nvidia-ctk: command not found` or NVIDIA driver mismatch | Image's driver doesn't match the toolkit version | Use a different image, or pin a compatible toolkit version |
| `apt-get update` fails | Image's apt sources are unreachable from this network | Check network/security-group; or pre-bake packages into the image |
| Runner registered, then `pytest` failed | A normal test failure | Read the GitHub Actions log — that's the canonical view of the workflow output |
| `No space left on device` | `boot_volume_size_gb` too small for Docker images + sim assets | Bump `boot_volume_size_gb` |
