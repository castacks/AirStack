# AirStack CI Orchestrator (OSMO backend)

This describes how a small always-on orchestrator service runs GitHub Actions jobs on truly ephemeral GPU workers scheduled by [NVIDIA OSMO](https://nvidia.github.io/OSMO/). The orchestrator is a Python service that continuously polls GitHub for queued workflow jobs, submits a fresh **OSMO workflow** for each one (a single-use JIT runner in a privileged, GPU-enabled container), and reaps it when the job completes. Each CI job runs on a clean pod with no state shared between runs and no long-lived credentials on the worker.

On the GitHub side, `system-tests.yml` uses `runs-on: [self-hosted, airstack-ephemeral]`, a single-use JIT runner config, and a same-repo fork guard. The worker semantics are one-job-per-worker, destroy-after: when the runner's `run.sh` exits after one job, the OSMO task completes and the pod is torn down.

The orchestrator host is the only machine that holds the GitHub PAT and the OSMO service-account token; workers are destroyed after a single job.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Orchestrator host  (airstack-ci-cd-orchestrator, no GPU)   │
│                                                             │
│  airstack-orchestrator.service → orchestrator.py            │
│    spawn loop  (every 15s):                                 │
│      • GET  /repos/<repo>/actions/runs?status=queued        │
│      • POST /repos/<repo>/actions/runners/generate-jitconfig│
│      • osmo workflow submit runner-workflow.yaml --pool ... │
│      • record (job_id → workflow_id) in state.json          │
│    reap loop   (every 30s):                                 │
│      • job completed     → osmo workflow cancel (if live)   │
│      • job age > N min   → osmo workflow cancel (straggler) │
│      • our-named but not in state → orphan cancel           │
│                                                             │
│  /etc/airstack-orchestrator/                                │
│      config.yaml                                            │
│      github-pat                                             │
│      osmo-token           (OSMO service-account token)      │
│  /var/lib/airstack-orchestrator/state.json                  │
│  /var/lib/airstack-orchestrator/.config/osmo  (CLI session) │
└─────────┬─────────────────────────────────┬─────────────────┘
          │ osmo CLI (submit/query/cancel)   │ GitHub REST API
          ▼                                 ▼
┌──────────────────────────────────┐  ┌──────────────────────┐
│  OSMO CI GPU pool (privileged)   │  │  GitHub Actions      │
│  Ephemeral runner pod (per job): │  │  workflow_job queue  │
│    Image: airstack-ci-runner     │  └──────────────────────┘
│    start dockerd (DinD)          │
│    run.sh --jitconfig <token>    │
│    exit → task done → pod reaped │
└──────────────────────────────────┘
```

Key properties:

- **Truly ephemeral**: every job runs on a clean pod. No Docker layer cache pollution, no leftover containers, no carry-over from prior runs.
- **PAT isolation**: the GitHub PAT lives only on the orchestrator. Workers receive a single-use [JIT runner config](https://docs.github.com/en/rest/actions/self-hosted-runners?apiVersion=2022-11-28#create-configuration-for-a-just-in-time-runner-for-a-repository) — a base64 token bound to one runner registration, valid only for a short window.
- **Service-account auth**: the orchestrator authenticates to OSMO with a shared, non-personal [service-account token](https://nvidia.github.io/OSMO/main/deployment_guide/appendix/authentication/service_accounts.html). CI runs never route through an individual's account, so PRs don't consume anyone's personal GPU quota and nothing breaks when a person leaves.
- **Crash-safe reaping**: every workflow is named `gha-runner-<job_id>-<ts>`. The reap loop cancels any active workflow with that prefix not present in `state.json`, so a crashed orchestrator can't leak workflows.

## Prerequisites

- **Orchestrator host** — a small always-on VM (no GPU). 1 vCPU, 2GB RAM, 20GB disk, outbound internet to `api.github.com` and your OSMO URL. This is the only long-lived piece; the GPU compute is ephemeral pods on OSMO.
- **An OSMO service account + dedicated CI pool.** Ask your OSMO admin to:
  1. Create a service account (e.g. `svc-airstack-ci`) and a long-lived access token — `osmo user create` + `osmo token set`. On IdP-backed deployments (e.g. auth tied to the CMU Andrew directory) this is a non-personal identity, so it survives people graduating/leaving. If policy forbids OSMO-native service accounts, use a *functional/departmental* identity, never a personal one.
  2. Grant that account a role whose policy allows `workflow:Create/Cancel/Query` **scoped to a dedicated CI GPU pool** (e.g. `pool/airstack-ci`) that has its own allocation, so CI doesn't contend with researchers' interactive jobs.
  3. **Enable "Privileged Mode Allowed"** on that pool's platform. The AirStack tests run `airstack up` (docker compose) inside the worker, which requires an inner Docker daemon → a privileged container. Without this, submissions are rejected.
  4. Confirm the API gateway (Envoy) accepts OSMO access tokens for the API (not only interactive IdP logins).
- **A prebaked runner image** pushed to a registry the pool can pull (see below).

## One-time setup

### 1. Build & push the runner image

The worker image bakes in Docker CE + compose, the NVIDIA container toolkit, and the GitHub Actions runner, so pod start is fast and the JIT token can't expire mid-bootstrap (no install work at boot).

```bash
cd .github/orchestrator
./build-and-push.sh
# or manually:
# docker build -f runner.Dockerfile \
#   --build-arg RUNNER_VERSION=2.334.0 \
#   -t airlab-docker.andrew.cmu.edu/airstack/airstack-ci-runner:2.334.0 .
# docker push airlab-docker.andrew.cmu.edu/airstack/airstack-ci-runner:2.334.0
```

No local Docker? Submit the one-shot OSMO builder (needs your Harbor creds in OSMO):

```bash
osmo workflow submit .github/orchestrator/build-runner-on-osmo.yaml \
  --pool airstack --priority HIGH
```

Set `runner_image: airlab-docker.andrew.cmu.edu/airstack/airstack-ci-runner:2.334.0` in `config.yaml` (step 4). Keep `RUNNER_VERSION` in sync with an [actions/runner release](https://github.com/actions/runner/releases).

**Pool note (AirLab):** use the Keycloak-autosynced `airstack` pool (`privileged_allowed: true`). A hand-created `airstack-ci` pool is wiped by `synchronize_osmo_team_pools.py`. Ephemerality is per-job OSMO workflows, not a separate pool.

### 2. Stage credentials on the orchestrator host

```bash
# GitHub PAT: needs `Actions: read/write` and `Administration: read/write`
# (fine-grained) or classic `repo` scope.
scp ~/airstack-github-pat.txt ubuntu@<orchestrator-ip>:/tmp/github-pat

# OSMO service-account token (from `osmo token set`, provided by your admin).
scp ~/svc-airstack-ci-token.txt ubuntu@<orchestrator-ip>:/tmp/osmo-token
```

### 3. Run setup.sh

On the orchestrator host:

```bash
git clone https://github.com/castacks/AirStack.git /tmp/airstack
sudo bash /tmp/airstack/.github/orchestrator/setup.sh
```

`setup.sh` creates the `orchestrator` system user, installs the `osmo` CLI, builds the Python venv, copies `orchestrator.py` and `runner-workflow.yaml.j2` into `/opt/airstack-orchestrator/`, scaffolds `/etc/airstack-orchestrator/`, installs the systemd unit, and consumes `/tmp/github-pat` and `/tmp/osmo-token`.

### 4. Fill in `/etc/airstack-orchestrator/config.yaml`

| Field | What goes here | How to find it |
|------|---------------|----------------|
| `osmo_url` | Your OSMO web service URL | from your OSMO admin |
| `pool` | Dedicated CI GPU pool | `osmo pool list` / the OSMO UI |
| `platform` | Optional hardware type within the pool; empty = pool default | `osmo pool list` |
| `runner_image` | Image from step 1 | the registry you pushed to |
| `cpu` / `gpu` / `memory` / `storage` | Resource request for the worker | size for full stack + sim |
| `privileged` | Must be `true` (docker compose inside the pod) | — |
| `priority` | `HIGH` \| `NORMAL` \| `LOW` | — |
| `repos` | list of `owner/name` repos to poll (legacy `repo:` accepted) | from GitHub URLs |
| `runner_version` | Runner version baked into `runner_image` | matches step 1 |
| `max_concurrent` | Max simultaneous in-flight workflows | — |
| `max_job_minutes` | Straggler cancel ceiling | exceed the longest job |

### 5. Start the service

```bash
sudo systemctl enable --now airstack-orchestrator.service
journalctl -u airstack-orchestrator.service -f
```

You should see `orchestrator started (OSMO backend): repos=[...] pool=... max_concurrent=N`, an `osmo login succeeded` line, and then periodic poll activity.

## End-to-end verification

```bash
# Trigger a fast build-only run.
gh workflow run system-tests.yml -f marks=build_docker

# Within ~30s, a workflow should appear in the CI pool:
osmo workflow list --name gha-runner- --pool airstack-ci

# Watch GitHub → Actions → Runners — the ephemeral runner should appear,
# pick up the job, then disappear.

# Within ~30s of job completion, the workflow should be terminal / gone from
# the active list:
osmo workflow list --name gha-runner- --pool airstack-ci --status RUNNING PENDING WAITING
```

## Module repos

Module repos (`asm_*`) that call trunk's reusable
[`module-system-tests.yml`](https://github.com/castacks/AirStack/blob/main/.github/workflows/module-system-tests.yml) with the
default `runs-on: [self-hosted, airstack-ephemeral]` queue jobs **in their own
repo**. One orchestrator instance polls them all — list every repo under
`repos:` in `/etc/airstack-orchestrator/config.yaml`:

```yaml
repos:
  - "castacks/AirStack"
  - "castacks/asm_dfm2_disturbances"
  - "castacks/asm_optitrack"
  - "castacks/asm_macvo"
```

(The legacy singular `repo:` key is still accepted.) Two requirements when
adding a repo:

1. **Extend the PAT.** The fine-grained GitHub PAT must cover each listed
   repo with `Actions: read/write` + `Administration: read/write` (JIT runner
   registration is per-repo).
2. **Restart the service** (`systemctl restart airstack-orchestrator`) and
   confirm the startup line lists every repo:
   `orchestrator started (OSMO backend): repos=[...]`.

The shared `max_concurrent` cap and the orphan sweep span all polled repos —
no per-repo instances, prefixes, or state files needed.

First-party only: the reusable workflow refuses callers outside the castacks
org, mirroring the fork-PR block. Org-level polling across registered repos
(one instance, many repos) is planned as a future replacement for this
per-repo setup.

## Operational notes

- **State file**: `/var/lib/airstack-orchestrator/state.json` is the in-flight job tracker (`job_id → workflow_id`). Wiping it triggers an orphan sweep on the next reap iteration — active `gha-runner-*` workflows will be cancelled. Don't wipe it while jobs are mid-flight unless that's what you want.
- **Straggler**: any workflow whose job has run longer than `max_job_minutes` (default 48h) is force-cancelled regardless of GitHub job status.
- **OSMO token rotation** (tokens expire — default 31 days): mint a new one and restart.
  ```bash
  # (admin) osmo token set svc-airstack-ci-token-2 --user svc-airstack-ci \
  #           --roles <ci-pool-role> osmo-user --expires-at 2027-12-31
  sudo install -o root -g orchestrator -m 0640 /tmp/osmo-token /etc/airstack-orchestrator/osmo-token
  sudo systemctl restart airstack-orchestrator.service   # re-runs `osmo login`
  ```
- **PAT rotation**: `sudo install -o root -g orchestrator -m 0640 /tmp/new-pat /etc/airstack-orchestrator/github-pat && sudo systemctl restart airstack-orchestrator.service`.
- **Pause spawning** (e.g. for maintenance): `sudo systemctl stop airstack-orchestrator.service`. Already-submitted workers still complete their jobs; on restart, the reap loop cleans up.
- **Logs**: `journalctl -u airstack-orchestrator.service -f`. Per-worker logs come from `osmo workflow logs <id>`.

## Debugging a failed job

When a GitHub workflow run fails or stalls, the failure can be in one of four places: the orchestrator (didn't submit), the OSMO task (didn't schedule/pull), the GH Actions runner (didn't register or crashed), or the workflow steps themselves. Each has a different inspection path.

### 1. Find which workflow ran the job

`state.json` is the authoritative job ↔ workflow map:

```bash
sudo jq -r '.jobs | to_entries[] | "\(.key)\t\(.value.workflow_id)\t\(.value.workflow_name)"' \
  /var/lib/airstack-orchestrator/state.json
```

Pick the row for your failing `job_id` (visible in the GitHub Actions URL):

```bash
JOB_ID=73286176852          # from the GitHub UI
WF=$(sudo jq -r ".jobs[\"$JOB_ID\"].workflow_id" /var/lib/airstack-orchestrator/state.json)
```

If the job isn't in `state.json`, the orchestrator never submitted for it — see step 2.

### 2. Did the orchestrator submit at all?

```bash
sudo journalctl -u airstack-orchestrator.service --since "30 min ago" --no-pager
```

Healthy submit looks like:

```text
submitted workflow <id> for job <job_id> (<job name>)
```

Common things that block a submit (and how to spot them):

| Log line / symptom | What it means | Fix |
|---|---|---|
| `find_queued_jobs failed: 401 ...` | GitHub PAT expired / wrong scope | Rotate the PAT |
| `osmo login failed ...` / `auth error` | OSMO token expired/invalid, or Envoy rejects access tokens | Rotate the OSMO token; confirm gateway accepts access tokens |
| `osmo workflow submit failed ... privileged` | Pool platform doesn't allow privileged | Ask admin to enable "Privileged Mode Allowed" on the CI pool |
| `osmo workflow submit failed ... pool` / permission | Service-account role lacks `workflow:Create` on the pool | Fix the role's pool-scoped policy |
| Job queued in GitHub but no `submitted` log | `runs-on` labels don't match `runner_labels` | Make them match |

### 3. Inspect the workflow / worker

```bash
# Status and scheduling detail.
osmo workflow query "$WF" --verbose

# Scheduling / lifecycle events (image pull, start, evict, ...).
osmo workflow events "$WF" --task runner

# Combined stdout of the runner task — shows dockerd start, run.sh, and the job.
osmo workflow logs "$WF" --task runner
osmo workflow logs "$WF" --task runner --error   # error stream
osmo workflow logs "$WF" --task runner -n 300    # last 300 lines
```

### 4. Break-glass shell into a running worker

If the workflow is still `RUNNING`, exec into the pod:

```bash
osmo workflow exec "$WF" runner            # /bin/bash in the runner task
```

Once inside:

```bash
# GitHub Actions runner diagnostics.
ls -lt /home/runner/actions-runner/_diag/
tail -300 /home/runner/actions-runner/_diag/Runner_*.log
tail -300 /home/runner/actions-runner/_diag/Worker_*.log

# Inner Docker daemon (a frequent failure point for `airstack up`).
cat /var/log/dockerd.log
docker info 2>&1 | head
nvidia-smi
```

### 5. Common failure patterns at the worker

| Symptom in `osmo workflow logs` | Cause | Fix |
|---|---|---|
| `dockerd did not become ready` | Pod not privileged / DinD blocked | Enable privileged on the pool platform |
| `nvidia-smi unavailable` / no GPU | GPU not requested/passed, or toolkit missing | Check `gpu:` request, platform GPUs, privileged |
| `Could not connect to api.github.com` | Egress blocked from the pool | Allow outbound 443 from the CI pool |
| `Bad credentials` / `Invalid ... runnerEvent` | JIT config TTL elapsed before `run.sh` started | Prebake the image (already done) so start is fast |
| `Cannot connect to the Docker daemon` during tests | inner dockerd crashed | Read `/var/log/dockerd.log` via `osmo workflow exec` |
| Runner registered, then `pytest` failed | A normal test failure | Read the GitHub Actions log — the canonical view |
| `No space left on device` | `storage` too small for images + sim assets | Bump `storage` in `config.yaml` |
| `failed to solve: ... mount source: "overlay" ... err: invalid argument` | Docker data-root landed on the pod's overlay rootfs | See "Nested DinD and overlayfs" below |

### Nested DinD and overlayfs

The single most likely way to break every Docker build at once. The pod's root
filesystem is overlayfs, and **Linux refuses to use a directory on overlayfs as
an overlay `upperdir`** (it returns `EINVAL`). A dockerd whose data-root sits on
the pod rootfs looks healthy — `docker info` works, image pulls succeed, because
containerd unpacks layers with plain writes — and then every build step that
needs a real mount fails:

```text
failed to solve: process "/bin/bash -c apt-get ... " did not complete successfully:
mount source: "overlay",
target: "/var/lib/docker/buildkit/containerd-overlayfs/cachemounts/buildkit1459786452",
fstype: overlay, ... err: invalid argument
```

That signature took out all four `build_docker` tests and all four
`build_packages` tests in one run, with each failure looking like an unrelated
`apt-get`/`WORKDIR` problem.

[`runner-entrypoint.sh`](https://github.com/castacks/AirStack/blob/main/.github/orchestrator/runner-entrypoint.sh) handles this before starting
dockerd. It picks a storage backend by **performing a real overlay mount** to
test each option rather than trusting the filesystem type, and falls back in
this order:

| Order | Backend | Notes |
|---|---|---|
| 1 | Loopback ext4 image mounted at `/var/lib/docker` | Preferred. Real `overlay2`, self-contained, dies with the pod. Sparse, so it only consumes what Docker writes. Sized to free space on `/` minus 20 GiB, or `DOCKER_LOOP_SIZE_MB`. |
| 2 | A real filesystem already mounted in the pod | Kubernetes `emptyDir`/`hostPath`/PVC volumes live on the node disk, not the overlay rootfs. `/osmo/data/output` and `/osmo/data/socket` are skipped — the OSMO ctrl sidecar owns them. |
| 3 | `fuse-overlayfs` driver | Stacks where the kernel driver won't. Needs `/dev/fuse`. |
| 4 | `vfs` driver | Always works, copies the whole filesystem per layer. Too slow and too large for the sim images — **reaching this is a red flag**, not a working state. |

The chosen backend is logged at startup, so confirm it in the job log before
debugging anything else:

```bash
osmo workflow logs "$WF" --task runner | grep -E 'runner-entrypoint|storage driver'
```

Backends 3 and 4 also set `features.containerd-snapshotter: false`, because
`storage-driver` is only honoured by the classic image store.

The one-shot [`build-runner-on-osmo.yaml`](https://github.com/castacks/AirStack/blob/main/.github/orchestrator/build-runner-on-osmo.yaml) builder
sidesteps the same problem differently — `vfs` plus `DOCKER_BUILDKIT=0` — which
is fine there because it builds one small image.
