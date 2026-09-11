# (c) Pod image + OSMO pilot — running log (2026-09-11, agent_study `4122415`)

## Pod image `airlab-docker.andrew.cmu.edu/airstack/agent-study-pod:v1`

| Check | Result |
|---|---|
| Build (FROM airstack-osmo-workspace:latest; claude 2.1.265; vcstool 0.3.0 via pip; study uid 1000 in docker group) | OK, 314 MB pushed 09:27Z; rebuilt for sshd/auth-probe/ownership fixes (last push 10:20Z) |
| `archive.ubuntu.com` unreachable from wildfire-inferno (0 B/s) | build stalled 10 min; switched to `mirror.pit.teraswitch.com` (91 MB/s) via `APT_MIRROR` build arg |
| Local privileged run (mock agent, `--smoke`) | dockerd falls back to fuse-overlayfs locally (overlay rootfs), runs the trial, moves workspace aside, writes `.TRIAL_DONE rc=0` |
| Local sshd + rsync-over-ssh (`SSH_PUB_KEY` → root authorized_keys) | `SSH_OK`, `RSYNC_OK` |
| On OSMO (provision pods) | data-root `/osmo/run/docker` is ext4 → **overlay2**; inner-docker GPU visible (`nvidia-smi -L` in a container, with NVML driver-file warnings); harbor login OK |

## OSMO findings (all cost real wall-clock, recorded so the next campaign skips them)

1. **`osmo workflow rsync` is disabled on the lab deployment** — every
   upload/download (one-shot, `--daemon`, `--rsync` at submit, groups+lead
   layout) fails with HTTP 403 `Rsync is not enabled for this workflow!`.
   The public docs say rsync needs no YAML field. Transport switched to
   sshd-in-pod + `osmo workflow port-forward <wf> trial --port <local>:22`
   + `rsync -e ssh` (same mechanism as `airstack osmo ide`).
2. **`osmo workflow exec` needs a TTY** (`[Errno 25] Inappropriate ioctl`)
   — wrap with `script -qfc "osmo workflow exec …" /dev/null`.
3. **Scheduling contention is per-node, not per-pool quota.** With three
   `krrishj` AirStack workflows (one at 24 CPU / 2 GPU / 900 Gi), two
   `micahn` training jobs and our two provisioning pods, a 16 CPU / 64 Gi /
   300 Gi request was unschedulable ("4 node(s) didn't have enough CPU
   cores; 2 GPUs; 1 memory; 1 ephemeral-storage"), 8/32/250 likewise, and
   4/24/200 likewise; **4 CPU / 16 Gi / 80 Gi scheduled in ~4 min**. The
   `osmo pool list` GPU column (10/12 free) does not reflect this.
   Trial pods will use 8 CPU / 32 Gi / 250 Gi when the provisioning pods
   have released their nodes, else the orchestrator's `--cpu/--memory/
   --storage` overrides.
4. `osmo workflow submit --set` templating (`{{ field }}` + `default-values:`)
   works for name and resource fields; `--set-env` values are not echoed by
   `--dry-run`.
5. Bake progress is invisible in `workflow logs` while `docker buildx bake |
   tail` buffers; peek with the TTY-wrapped exec (`docker images`, `uptime`).

## Provisioning

| Job | Status | Notes |
|---|---|---|
| `agent-study-provision-as2-2` (8 CPU / 32 Gi / 300 Gi) | COMPLETED in 30 min | Harmonic variant built from upstream `docker/humble_gzharmonic/dockerfile` with base + project_gazebo pinned; pushed `agent-study-as2:humble-gzharmonic-82d85abcd904` (1.8 GB) |
| `agent-study-provision-1` (UAS `make images`, 16 CPU / 64 Gi) | RUNNING (51 min at 10:20Z) | 13 targets incl. `cuda_pytorch`, forked gz-sim build |

## Mock pilot (E2, noop agent, `--smoke`) on OSMO

- `as-e2-none-001-{1..4}`: 1–3 cancelled while resizing/unschedulable; #4
  RUNNING but bundle upload failed (rsync disabled → finding 1).
- Rerun with the SSH transport: see orchestrator log `orch_pilot_mock2.log`
  (result appended below when available).

### Mock pilot result (SSH transport) — 2026-09-11 06:22–06:24 EDT, agent_study `4122415`

`as-e2-none-001-5` (4 CPU / 16 Gi / 80 Gi): submitted 06:22:23 → RUNNING
06:22 → port-forward + bundle upload + marker 06:23:36 → pod ran the smoke
trial (clones at pins, manifest 1238 files, results.json) and wrote
`.TRIAL_DONE rc=0` 06:23:43 → orchestrator downloaded
`runs/E2_none_ladder_mock-noop_agent_001/` (236 KB, 17 files incl.
`pod_env.json`: RTX PRO 5000 Blackwell, driver 580.126.20, 48 CPU / 216 GB
node, overlay2) and cancelled the workflow 06:24:24. **Section (c) PASS.**
Fix folded in afterwards: `runner_commit` was `unknown` in the git-less
bundle → runner now reads `RUNNER_COMMIT` (+`-dirty`); orchestrator writes
`osmo_workflow.txt` into the downloaded trial dir.
