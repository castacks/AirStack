# Access: OSMO pod `airstack-dev-183` (live handover, 2026-09-01)

Handed over by the hurricane session (disaster-dataset-1e). The pod is
RUNNING and fully provisioned; do not re-provision blindly — read this first.

## The numbers

| what | value |
|---|---|
| workflow id | **airstack-dev-183** |
| reached RUNNING | 2026-08-31 23:57:57 EDT |
| exec_timeout | 20 h → dies **~2026-09-01 19:58 EDT** on its own |
| local ssh tunnel | **localhost:2204** (already open, host pid 4150807) |
| ssh | `ssh -p 2204 root@localhost` (StrictHostKeyChecking=no; key auth as for every pod) |
| sim container | `isaac-sim-livestream` |
| GPUs | 4x NVIDIA RTX PRO 5000 Blackwell, 48 GB each |
| pod hostname | ecf416004b29 (inner container) |

If the tunnel is dead, reopen it exactly like the provision script does:

    nohup osmo workflow port-forward airstack-dev-183 workspace \
        --port 2204:22 --connect-timeout 86400 > /tmp/osmo_ide_dev183.log 2>&1 &

**Port etiquette:** 2200, 2201, 2202 belong to OTHER sessions' forwards on
this machine. Do not bind or kill them. 2204 is this pod's.

## Using the repo tooling against it

Every `scene_gen/tools/osmo_*.sh` helper honours `OSMO_SSH_LOCAL_PORT`:

    export OSMO_SSH_LOCAL_PORT=2204
    ./scene_gen/tools/osmo_isaac.sh exec  '<cmd in the sim container>'
    ./scene_gen/tools/osmo_isaac.sh ssh   '<cmd on the pod host>'
    ./scene_gen/tools/osmo_sync.sh <files...> | --dir <dir>     # push (to /root/AirStack)
    ./scene_gen/tools/osmo_pull.sh <name> <dest>                # pull a snapshot dir

Paths: the clone lives at `/root/AirStack` on the pod host, bind-mounted to
`/isaac-sim/AirStack` in the container. Logs/snapshots:
`/root/docker/isaac-sim/logs` ↔ `/isaac-sim/.nvidia-omniverse/logs`.
`docker logs` is EMPTY for the sim container — read your own redirect file
or the tmux pane (`osmo_isaac.sh pane`).

## What is on it right now

- Provisioned with `SYNC_ASSETS=1` (megascans, aec vegetation, objaverse
  419M, archetypes_hurricane 34 tree files + archetypes_tornado 71, all
  hurricane code) PLUS every hurricane fix through FINAL10 (2026-09-01).
  `render_preflight.sh` last reported `PREFLIGHT OK: 128 files` against the
  local working tree at handover time — if the local tree moves on, re-sync
  before trusting it.
- Leftover run dirs under the logs mount: `FINAL6..FINAL10_*`, `VAL2/3_*`,
  `TREE_PROBE`, `STAGE_DUMP`, `hurricane_ws/` (an isolated workspace from a
  borrowed-pod era; safe to delete). Nothing is running; the launcher exits
  after SCENE_DONE unless `HUR_KEEP_OPEN=1`.
- `AIRSTACK_ASSET_ROOT` is set to Nucleus in the container env; the
  hurricane renders passed `AIRSTACK_ASSET_ROOT=` (empty) per launch so
  `airstack://` resolves against the synced clone. Decide per launch.

## Render pattern that works here (headless, self-exiting, ~2-3 min/500 m)

    export OSMO_SSH_LOCAL_PORT=2204
    ./scene_gen/tools/osmo_isaac.sh ssh "docker exec -d isaac-sim-livestream bash -lc '\
      PYTHONUNBUFFERED=1 AIRSTACK_ASSET_ROOT= SCENE_CONFIG=<preset> HUR_SEED=11 \
      HUR_HEADLESS=1 HUR_DOME_INT=1400 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/<NAME> \
      /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/<launcher>.py \
      --ext-folder /isaac-sim/.local/share/ov/data/documents/Kit/shared/exts \
      > /isaac-sim/.nvidia-omniverse/logs/<NAME>.log 2>&1'"
    # poll: grep -c SCENE_DONE <log>; pull: tar over ssh from the logs mount

Before ANY render that must reflect local changes, run BOTH gates:
`OSMO_SSH_LOCAL_PORT=2204 bash scene_gen/tools/render_preflight.sh`
(content checksums local↔pod; add any new runtime-imported module to its
LIST) and `python3 scene_gen/tools/hurricane_layout_png.py` (2D placement +
predicted-material gate) — the hurricane sessions' painful lessons are baked
into those two scripts and `SESSION_2026-08-31_round2.md`.

## Ownership / lifetime

The hurricane session is DONE with this pod and runs local-only now; the pod
is intentionally left RUNNING for the next agent. Cancel it when finished
(`osmo workflow cancel airstack-dev-183`) or let the 20 h timeout reap it.
If you are not the only user, announce yourself to the session registry
(ListAgents / cross-session message) before heavy GPU use — the machine's
sessions coordinate card use by message.
