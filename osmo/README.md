# AirStack on OSMO

This directory holds the bits that let students develop on AirStack remotely
through [NVIDIA OSMO](https://github.com/NVIDIA/OSMO):

```
osmo/
├── README.md                     # This file (admin / operator reference)
├── workflows/
│   └── airstack-dev.yaml         # The OSMO workflow students submit
└── workspace/
    ├── Dockerfile                # The airstack-osmo-workspace image
    ├── sshd_config               # Pubkey-only sshd config baked into the image
    └── entrypoint.sh             # Pod startup: sshd, dockerd, clone, airstack up
```

The student-facing walkthrough lives in
[`docs/tutorials/airstack_on_osmo.md`](../docs/tutorials/airstack_on_osmo.md)
— including the per-user **Step 0** for registering OSMO credentials. This
README is the **lab admin / operator** reference: pool requirements,
workspace image build & push, validation stages, plus a credential summary
for context.

> **Scope:** developer workflow only. CI/CD on OSMO is **not** part of this
> integration — the existing `system-tests.yml` + OpenStack orchestrator path
> is unchanged.

## Architecture in one minute

A student submits one OSMO task that runs a Docker-in-Docker (DinD) pod with
sshd. Inside that pod, `airstack.sh up` brings up the regular
three-container AirStack stack (Isaac Sim, robot-desktop, GCS) on the inner
Docker daemon. The student attaches VS Code or Cursor over Remote-SSH and
streams Isaac Sim (WebRTC) and the GCS Foxglove bridge (websocket) back to
their laptop via `osmo workflow port-forward`.

```
Student laptop                       OSMO workspace pod (GPU)
─────────────────                    ─────────────────────────────────────
VS Code / Cursor   ── ssh ──►  port-forward 2200:22 ──► sshd
Isaac Sim WebRTC   ── webrtc ►  port-forward 47995… ──► inner isaac-sim ctnr
app.foxglove.dev   ── ws ────►  port-forward 8766 ────► inner gcs ctnr (8765)
                                                       ▲
                                                       │  inner dockerd
                                                       │  (NVIDIA runtime)
                                                       │
                                            airstack.sh up brings these 3 up
```

## Pool requirements

The OSMO pool the workflow runs on must satisfy:

| Requirement | Why |
|---|---|
| GPU pool with NVIDIA driver + `nvidia-container-toolkit` on each node | Isaac Sim needs the GPU. The toolkit must be on the node so the inner `dockerd` (configured with `--add-runtime nvidia=...`, `default-runtime: nvidia`) can hand the device to the inner Isaac Sim container. |
| No NetworkPolicy blocking pod-namespace ports `47995–48012/tcp+udp`, `49000–49007/tcp+udp`, `49100/tcp`, `8766/tcp`, `22/tcp` | These are the ports `osmo workflow port-forward` reaches inside the pod NS for Isaac Sim WebRTC, GCS Foxglove websocket, and sshd. |
| Resource limits ≥ `cpu: 16`, `memory: 64Gi`, `storage: 200Gi`, `gpu: 1` | Isaac Sim + AirStack images + `colcon build` working tree. Adjust upward if running multiple robots or heavy bag recording. |

`hostNetwork: true` is **not** required. `osmo workflow port-forward` reaches
the pod's network namespace, which is where the inner `dockerd` publishes
ports via standard NAT (or `network_mode: host` on individual inner
containers, both of which terminate at the pod NS, not the cluster node).

## OSMO credentials (per user, one time)

OSMO credentials live in **each user's** OSMO profile, not in a lab-wide
store. Every student registers their own three credentials with `osmo
credential set` once on their laptop. The full walkthrough — including the
exact `osmo credential set ...` commands and how to obtain a Nucleus API
token — lives in
[`docs/tutorials/airstack_on_osmo.md` Step 0](../docs/tutorials/airstack_on_osmo.md#step-0--register-your-osmo-credentials-one-time).

The three credentials, summarized for quick reference:

| Name | Type | Used for | Referenced in workflow YAML? |
|---|---|---|---|
| `airlab-docker-registry` | `REGISTRY` | OSMO's automatic pull of the workspace image (`airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:...`) | No — OSMO auto-attaches it to any image whose hostname matches the credential's `registry=` field. |
| `airlab-docker-login` | `GENERIC` | `entrypoint.sh` calls `docker login airlab-docker.andrew.cmu.edu` on the **inner** dockerd before `airstack up`, so the inner Compose stack can pull AirStack images | Yes — exposed as env vars `AIRLAB_REGISTRY_USER`/`AIRLAB_REGISTRY_PASS`. |
| `airlab-nucleus` | `GENERIC` | `entrypoint.sh` materializes `simulation/isaac-sim/docker/omni_pass.env` from it so Compose can env-file it into the Isaac Sim container | Yes — exposed as env vars `OMNI_USER`/`OMNI_PASS`/`OMNI_SERVER`. |

The convenience helper `airstack osmo:setup` in
[`.airstack/modules/osmo.sh`](../.airstack/modules/osmo.sh) prompts for the
underlying values (Andrew ID, AirLab password, Nucleus API token) and runs
all three `osmo credential set` commands.

> **Why a `REGISTRY` and a `GENERIC` credential for the same registry?**
> OSMO `REGISTRY` credentials drive Kubernetes `imagePullSecrets` —
> auto-attached but not exposed to the container as env vars. The
> **inner** dockerd (DinD) that `entrypoint.sh` starts is a separate
> Docker daemon and needs its own `docker login`. Hence the two-credential
> split.

## Build & push the workspace image

The workspace image is built once and pushed to the AirLab registry; students
never build it themselves.

```bash
cd osmo/workspace
docker build -t airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest .
docker push   airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest
```

Tag a versioned release alongside `latest` if you change anything in
`Dockerfile`, `sshd_config`, or `entrypoint.sh`:

```bash
docker tag  airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest \
            airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:v0.1.0
docker push airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:v0.1.0
```

Then update the `image:` field in
[`workflows/airstack-dev.yaml`](workflows/airstack-dev.yaml) to match.

The image bakes:

- Ubuntu 24.04 base with `docker-ce`, `docker-compose-plugin`, `nvidia-container-toolkit`
- `git`, `python3`, `curl`
- `openssh-server` with **password auth permanently disabled** (pubkey only) via the baked `sshd_config`
- The AirStack `airstack.sh` CLI script on `PATH`

The image does **not** bake the AirStack source tree. `entrypoint.sh` clones
it on first start (and skips re-cloning across pod restarts).

## Validation stages

Run these in order against a fresh submission. Each unlocks the next; if (a)
fails don't bother trying (b).

### (a) sshd reachable, key auth works

```bash
osmo workflow submit osmo/workflows/airstack-dev.yaml \
  --pool <gpu-pool> \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)"
# → record <wf-id>

osmo workflow port-forward <wf-id> workspace --port 2200:22 --connect-timeout 86400 &
ssh -p 2200 -o StrictHostKeyChecking=accept-new root@localhost 'echo ok && whoami'
# → "ok\nroot"
```

If SSH fails: check `osmo workflow logs <wf-id> workspace` for the
`SSH_PUB_KEY not set` error or for `sshd` failing to start.

### (b) VS Code / Cursor Remote-SSH attaches and opens `/root/AirStack`

Add to `~/.ssh/config`:

```
Host airstack-osmo
  HostName localhost
  Port 2200
  User root
  StrictHostKeyChecking accept-new
```

Then in VS Code: Command Palette → **Remote-SSH: Connect to Host…** →
`airstack-osmo` → open folder `/root/AirStack`. The IDE will install its
remote server in the pod on first connect (~50 MB download, slow on a fresh
pod; cached afterwards).

### (c) `airstack up` brings the three containers Up

In the IDE's integrated terminal (or `osmo workflow exec`):

```bash
docker ps
# → expect: airstack-isaac-sim-1, airstack-robot-desktop-1, airstack-gcs-1
```

If any container is missing or restarting, the most common causes (in order):

1. The user's `airlab-docker-login` GENERIC credential is wrong / unset →
   inner `docker pull` from `airlab-docker.andrew.cmu.edu` failed.
   Re-run `airstack osmo:setup` (or the explicit `osmo credential set
   airlab-docker-login ...` command in the tutorial Step 0).
2. `nvidia-container-toolkit` is not configured on the node → inner Isaac Sim
   can't see the GPU. Check `docker info | grep -i runtime` inside the
   workspace pod; you should see `nvidia` in the runtime list.
3. The pod ran out of `storage:` quota during the image pull. Bump it.

### (d) Isaac Sim WebRTC client renders

Two port-forwards (TCP + UDP):

```bash
osmo workflow port-forward <wf-id> workspace \
  --port 47995-48012,49000-49007,49100 --connect-timeout 86400 &
osmo workflow port-forward <wf-id> workspace \
  --port 47995-48012,49000-49007 --udp --connect-timeout 86400 &
```

Open the Omniverse Streaming Client (or a browser WebRTC client) at
`http://localhost`.

If the stream is blank: check that the Pegasus standalone script was launched
with `--/app/livestream/enabled=true`. The
[`isaac-sim-livestream`](../simulation/isaac-sim/docker/docker-compose.yaml)
Compose profile is what wires that argument; verify the workflow YAML has
`ISAAC_SIM_LIVESTREAM=true` in `environment:`.

### (e) Foxglove websocket loads the AirStack layout

```bash
osmo workflow port-forward <wf-id> workspace --port 8766:8766 --connect-timeout 86400 &
```

Open [https://app.foxglove.dev](https://app.foxglove.dev) → **Open
connection** → `ws://localhost:8766` → **Layouts** → **Import from file** →
[`gcs/foxglove_extensions/airstack_default.json`](../gcs/foxglove_extensions/airstack_default.json).

The wider Foxglove layout / panel-import flow is documented in
[`docs/gcs/foxglove.md`](../docs/gcs/foxglove.md); the only OSMO-specific
piece is the `port-forward` line in front of it.

## Nucleus connectivity from OSMO

`airlab-nucleus.andrew.cmu.edu` runs the standard Omniverse Enterprise
Nucleus stack with TLS termination at its Ingress Router (NGINX) on **port
443**. Per [NVIDIA's TLS doc](https://docs.omniverse.nvidia.com/nucleus/latest/enterprise/installation/tls.html),
clients only need outbound TCP **443** — the Ingress Router path-based-
routes requests (`/omni/api`, `/omni/auth`, `/omni/lft`, `/omni/conn`,
`/omni/web3/...`) to the internal service ports (3009, 3100, 3030, 3019,
3400). Omniclient detects SSL/TLS and prefers it, so the OSMO pod (whose
egress allows 80/443/22) reaches Nucleus over the same single 443 the
Web3 navigator uses. **The native protocol ports 3009–3180 do NOT need to
be open from OSMO** as long as TLS is configured on the Nucleus side.

If you see Isaac Sim's "Login Required" popup at startup:

1. **Check the auth-service log on the Nucleus host** (`ssh
   ubuntu@<nucleus-host>; sudo docker logs --tail 200
   base_stack-nucleus-auth-1`). Look for `InternalCredentials.auth:
   {... 'username': '<your_andrew_id>'} → status: 'DENIED'` lines. That
   means the API token in your `airlab-nucleus` OSMO credential is
   revoked, expired, or has whitespace/quoting damage.
2. **Regenerate the token** at
   <https://airlab-nucleus.andrew.cmu.edu/omni/web3/> → right-click the
   cloud icon → **API Tokens** → create a new one.
3. **Update the OSMO credential** with `airstack osmo:setup` (or the
   raw `osmo credential set airlab-nucleus ...` command from the
   tutorial Step 0) and **resubmit the workflow** so the new token
   lands in `omni_pass.env` on pod boot. To live-patch a running pod
   instead, edit `simulation/isaac-sim/docker/omni_pass.env` inside
   the workspace and `docker compose --profile isaac-sim-livestream
   restart isaac-sim-livestream`.

## Out of scope (followups)

- **OSMO-native split** — three separate OSMO tasks for `isaac-sim` /
  `robot-desktop` / `gcs` instead of one DinD pod. Larger refactor of
  Compose, DDS networking, and `tests/conftest.py`. The `osmo/workflows/`
  layout leaves room for additional workflow files when this is done.
- **Persistent workspace** — mount `/root/AirStack` to a PVC so uncommitted
  edits survive `osmo workflow cancel`. Pool-policy dependent.
- **CI/CD on OSMO** — the existing `.github/workflows/system-tests.yml` +
  OpenStack ephemeral runner path is unchanged. Migrating CI to OSMO is a
  separate effort.
